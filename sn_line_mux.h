#include <light_CD74HC4067.h>

// ==========================================
// กำหนดจำนวนเซนเซอร์ที่ใช้งานจริงตรงนี้ (เช่น 8, 10, 16)
#define NUM_SENSORS 8
// กำหนด Channel ของ Multiplexer ที่ต้องการใช้งานให้สอดคล้องกับจำนวนด้านบน
// ตัวอย่าง: ถ้ามีแผง 16 ตัว แต่จะใช้ 8 ตัวตรงกลาง คือ Channel 4, 5, 6, 7, 8, 9, 10, 11
const int activeChannels[NUM_SENSORS] = { 4, 5, 6, 7, 8, 9, 10, 11};
// ==========================================

CD74HC4067 mux(10,11,12,13);
const int signal_pin = A0;

// เผื่อ Array ไว้ 16 ตัวเพื่อป้องกัน Error แต่ระบบจะอ่านแค่ตามจำนวน NUM_SENSORS
int a[16] = {0};
int blackRef[16] = {849,868,848,857,863,843,819,775,847,855,861,836,856,790,844,799};
int whiteRef[16] = {85,84,84,84,85,84,82,81,84,85,85,83,83,82,85,84};
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
    avgRef[i] = (blackRef[i] + whiteRef[i]) / 2;
  }
}

void A2D(){
  for(int i=0; i<NUM_SENSORS; i++){
    a[i] = readS(i);
    if (!isWhiteLine) {
      digitalVal[i] = (a[i] > avgRef[i] ? 0 : 1); 
    } else {
      digitalVal[i] = (a[i] > avgRef[i] ? 1 : 0); 
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
  
  float avg = 0;
  int sum = 0;
  bool online = false;
  
  int trackThreshold = 250; 
  int noiseThreshold = 50;  

  for(int i=0; i<NUM_SENSORS; i++){
    long val;
    if (!isWhiteLine) val = map(a[i], whiteRef[i], blackRef[i], 0, 1000);
    else val = map(a[i], blackRef[i], whiteRef[i], 0, 1000); 
    
    val = constrain(val, 0, 1000); 
    if (val > trackThreshold) online = true;
    
    if (val > noiseThreshold) {
      avg += val * i; 
      sum += val;
    }
  }

  // คำนวณจุดกึ่งกลางอัตโนมัติ (เช่น 8 ตัว กลางคือ 3.5 | 16 ตัว กลางคือ 7.5)
  float centerPos = (NUM_SENSORS - 1.0) / 2.0;

  if (!online) {
    if (previousErr < 0) return -centerPos; 
    else return centerPos;                  
  }

  float position = avg / sum;
  float error = position - centerPos; 
  
  return error;
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