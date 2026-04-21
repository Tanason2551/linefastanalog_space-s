#include <light_CD74HC4067.h>

CD74HC4067 mux(10,11,12,13);
const int signal_pin = A0;

int a[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
// ค่า Preset สามารถปรับแก้ตัวเลขได้ตามต้องการ
int blackRef[] = {849,868,848,857,863,843,819,775,847,855,861,836,856,790,844,799};
int whiteRef[] = {85,84,84,84,85,84,82,81,84,85,85,83,83,82,85,84};
int avgRef[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int digitalVal[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

// เพิ่มตัวแปรสำหรับเลือกสีเส้น (Global Variable)
bool isWhiteLine = false;

int readS(int sensorPin){
  mux.channel(sensorPin);
  return analogRead(signal_pin);
}

void readAll(){
  for(int i=0;i<16;i++){
    a[i]=readS(i);
    delay(1);    
  }
}

// =====================================================
// 1. โหมดดึงค่า Preset ที่ตั้งไว้ใน Array มาใช้งาน
// =====================================================
void setSensorBW(){
  for(int i=0;i<16;i++)
    avgRef[i]=(blackRef[i]+whiteRef[i])/2;
}

// =====================================================
// 2. โหมด Auto-Calibrate (สแกนปัดซ้ายขวา)
// =====================================================
void calibrateSensorAuto(int pauseTime, int samples) {
  int minV[16];
  int maxV[16];
  
  for (int i = 0; i < 16; i++) {
    minV[i] = 1023;
    maxV[i] = 0;
  }
  
  for (int s = 0; s <= samples; s++) {
    readAll();
    for (int i = 0; i < 16; i++) {
      if (a[i] < minV[i]) minV[i] = a[i];
      if (a[i] > maxV[i]) maxV[i] = a[i];
    }
    delay(pauseTime);
  }
  
  for (int i = 0; i < 16; i++) {
    whiteRef[i] = minV[i] + 20; // เผื่อ Noise นิดหน่อย
    blackRef[i] = maxV[i] - 20;
    avgRef[i] = (blackRef[i] + whiteRef[i]) / 2;
  }
}

// ฟังก์ชันดั้งเดิม แปลง Analog เป็น Digital (1/0)
void A2D(){
  for(int i=0;i<16;i++){
    a[i]=readS(i);
    // สลับค่า Digital ตามสีของเส้น
    if (!isWhiteLine) {
      digitalVal[i]=(a[i]>avgRef[i]?1:0); // สำหรับเส้นดำ
    } else {
      digitalVal[i]=(a[i]>avgRef[i]?0:1); // สำหรับเส้นขาว
    }
  }
}

void showAnalog(){
  A2D();
  for(int i=0;i<15;i++){
    Serial.print(String(a[i]));
    Serial.print(",");
  }
  Serial.println( String(a[15]));
}

void showDigital(){
  A2D();
  for(int i=0;i<15;i++){
    Serial.print( String(digitalVal[i]));
    Serial.print(",");
  }
  Serial.println(String(digitalVal[15]));
}

// =====================================================
// NEW: calErr หาค่า Error โดยใช้ตำแหน่งของเซนเซอร์โดยตรง
// =====================================================
float calErr(float previousErr){
  A2D(); // ควบคู่กับการหา Digital value เพื่อใช้เช็คทางแยกแบบเดิม
  
  float avg = 0;
  int sum = 0;
  bool online = false;
  
  int trackThreshold = 250; 
  int noiseThreshold = 50;  

  for(int i=0; i<16; i++){
    long val;
    // ปรับน้ำหนักและ Invert สเกลค่าแสงตามสีของเส้น
    if (!isWhiteLine) {
      val = map(a[i], whiteRef[i], blackRef[i], 0, 1000); // เส้นดำ
    } else {
      val = map(a[i], blackRef[i], whiteRef[i], 0, 1000); // เส้นขาว
    }
    val = constrain(val, 0, 1000); 
    
    if (val > trackThreshold) online = true;
    
    // คำนวณแบบถ่วงน้ำหนักตาม "ตำแหน่งเซนเซอร์ (i)"
    if (val > noiseThreshold) {
      avg += val * i; // i คือ 0-15
      sum += val;
    }
  }

  if (!online) {
    if (previousErr < 0) return -7.5; // หลุดขวา เลี้ยวซ้ายสุด
    else return 7.5;                  // หลุดซ้าย เลี้ยวขวาสุด
  }

  // หาตำแหน่งปัจจุบันของเส้น (0.0 ถีง 15.0)
  float position = avg / sum;
  
  // แปลงให้ Center อยู่ที่ 0 (7.5 คือกึ่งกลางระหว่างเซนเซอร์ตัวที่ 7 และ 8)
  float error = position - 7.5; 
  
  return error;
}

// =====================================================
// calErrif ของเดิม สำหรับตอนเลี้ยวตามปกติ (Digital 100%)
// =====================================================
int calErrif(int previousErr){
  A2D();

    for(int i=2;i<6;i++){
      if(digitalVal[i]==1)
      {
        return (7-i)*(-1);
      }
    }
    
    for(int i=13;i>9;i--){
      if(digitalVal[i]==1){
        return i-8;
      }
    }
    if(digitalVal[7]==1 || digitalVal[8]==1)
      return 0;  

    else if (digitalVal[0]== 1|| digitalVal[1]==1){
      return -9;
    }
    else if (digitalVal[15]== 1|| digitalVal[14]==1){
      return 9;
    }
  return previousErr;
}