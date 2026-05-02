#include <light_CD74HC4067.h>

// ==========================================
// กำหนดจำนวนเซนเซอร์ที่ใช้งานจริงตรงนี้ (เช่น 8, 10, 16)
#define NUM_SENSORS 16
// กำหนด Channel ของ Multiplexer ที่ต้องการใช้งานให้สอดคล้องกับจำนวนด้านบน
// ตัวอย่าง: ถ้ามีแผง 16 ตัว แต่จะใช้ 8 ตัวตรงกลาง คือ Channel 4, 5, 6, 7, 8, 9, 10, 11
const int activeChannels[NUM_SENSORS] = {0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
// ==========================================

CD74HC4067 mux(10,11,12,13);
const int signal_pin = A0;

// เผื่อ Array ไว้ 16 ตัวเพื่อป้องกัน Error แต่ระบบจะอ่านแค่ตามจำนวน NUM_SENSORS
int a[16] = {0};
int blackRef[16] = {858,834,842,848,849,854,854,859,825,817,854,842,832,825,825,857};
int whiteRef[16] = {81,75,74,75,75,74,74,75,74,73,75,75,74,74,77,81};
int avgRef[16] = {0};
int digitalVal[16] = {0};

bool isWhiteLine = true;

int readS(int sensorIdx){
  // ให้อ่านจาก Channel ของ Multiplexer ตามที่เราระบุไว้ใน Array ด้านบน
  mux.channel(activeChannels[sensorIdx]); 
  delayMicroseconds(20); 
  return analogRead(signal_pin);
}

void readAll(){
  for(int i=0; i<NUM_SENSORS; i++){
    a[i]=readS(i);
  }
}

void setSensorBW(){
  for(int i=0; i<NUM_SENSORS; i++)
    avgRef[i] = (blackRef[i]+whiteRef[i])/2;
}

void calibrateSensorAuto(int pauseTime, int samples) {
  int minV[16];
  int maxV[16];
  
  for (int i = 0; i<NUM_SENSORS; i++) {
    minV[i] = 1023;
    maxV[i] = 0;
  }
  
  for (int s = 0; s <= samples; s++) {
    readAll();
    for (int i = 0; i<NUM_SENSORS; i++) {
      if (a[i] < minV[i]) minV[i] = a[i];
      if (a[i] > maxV[i]) maxV[i] = a[i];
    }
    delay(pauseTime);
  }
  
  for (int i = 0; i<NUM_SENSORS; i++) {
    whiteRef[i] = minV[i] + 20; 
    blackRef[i] = maxV[i] - 20;
    // ป้องกันค่าสวิงกรณีที่สีดำและขาวค่าต่างกันน้อยมากๆ
    if (blackRef[i] < whiteRef[i] + 10) blackRef[i] = whiteRef[i] + 10;
    
    avgRef[i] = (blackRef[i] + whiteRef[i]) / 2;
  }
}

void A2D(){
  for(int i=0; i<NUM_SENSORS; i++){
    a[i] = readS(i);
    if (!isWhiteLine) {
      digitalVal[i] = (a[i] > avgRef[i] ? 1 : 0); 
    } else {
      digitalVal[i] = (a[i] > avgRef[i] ? 0 : 1); 
    }
  }
}

void showAnalog(){
  A2D();
  for(int i=0; i<NUM_SENSORS-1; i++){
    Serial.print(String(a[i]) + ",");
  }
  Serial.println(String(a[NUM_SENSORS-1]));
}

void showDigital(){
  A2D();
  for(int i=0; i<NUM_SENSORS-1; i++){
    Serial.print(String(digitalVal[i]) + ",");
  }
  Serial.println(String(digitalVal[NUM_SENSORS-1]));
}

float calErr(float previousErr){
  A2D(); 
  
  long maxVal = 0;
  long vals[NUM_SENSORS];

  // 1. ดึงค่าและหาค่าสูงสุด (Max Value) ในลูปปัจจุบัน
  for(int i=0; i<NUM_SENSORS; i++){
    long val;
    if (!isWhiteLine) val = map(a[i], whiteRef[i], blackRef[i], 0, 1000);
    else val = map(a[i], blackRef[i], whiteRef[i], 0, 1000); 
    
    val = constrain(val, 0, 1000); 
    vals[i] = val;

    if (val > maxVal) {
      maxVal = val;
    }
  }

  float centerPos = (NUM_SENSORS - 1.0) / 2.0;

  // 2. เช็คว่าหลุดเส้นหรือไม่ โดยดูจากค่าสูงสุด (เผื่อเส้นบางมาก ตั้งไว้ที่ 40)
  if (maxVal < 40) {
    float extremeError = centerPos;
    if (previousErr < 0) return -extremeError; 
    else return extremeError;                  
  }

  // 3. ใช้ "Dynamic Threshold" กรอง Noise พื้นขาวทิ้ง 100%
  // ตัดเฉพาะค่าที่เกิน 50% ของค่าที่สว่าง/เข้มที่สุดในขณะนั้น
  long dynamicThreshold = maxVal / 2;
  if (dynamicThreshold < 20) dynamicThreshold = 20;

  float avg = 0;
  int sum = 0;

  for(int i=0; i<NUM_SENSORS; i++){
    if (vals[i] > dynamicThreshold) {
      long cleanVal = vals[i] - dynamicThreshold; // หักลบฐานออกให้เริ่มที่ 0
      avg += cleanVal * i; 
      sum += cleanVal;
    }
  }

  if (sum == 0) return previousErr;

  float position = avg / sum;
  float error = position - centerPos; 
  
  return error;
}

// แก้ไขฟังก์ชัน showError ให้จำค่า Previous Error ได้อย่างถูกต้อง
float debugPrevErr = 0;
void showError() {
  debugPrevErr = calErr(debugPrevErr);
  Serial.print("Current Error: ");
  Serial.println(debugPrevErr);
}


int calErrif(int previousErr){
  A2D();
  int midL = (NUM_SENSORS / 2) - 1;
  int midR = (NUM_SENSORS / 2);
  int maxIdx = NUM_SENSORS - 1;

  if(digitalVal[midL]==1 || digitalVal[midR]==1) return 0;
  else if (digitalVal[0]==1 || digitalVal[1]==1) return -9;
  else if (digitalVal[maxIdx]==1 || digitalVal[maxIdx-1]==1) return 9;

  return previousErr;
}