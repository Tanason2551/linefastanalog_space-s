#include "PID_line_mux.h"
float err = 0; // เปลี่ยนเป็น float รองรับ Analog Error

void setup(){
  RobotSetup();
  Serial.begin(115200);
  
  isWhiteLine = true;

  while (true) {
    if (digitalRead(SWA_PIN) == PRESS) {
      Serial.println("Auto-Calibrating... Please sweep sensor across line");
      calibrateSensorAuto(10, 300);
      Serial.println("Auto-Calibration Done!");
      delay(500);
      break;
    }
    else if (digitalRead(SWB_PIN) == PRESS) {
      setSensorBW();
      Serial.println("Preset-Calibration Done!");
      delay(500);
      break;
    }
  }
  while (digitalRead(SWA_PIN) != PRESS && digitalRead(SWB_PIN) != PRESS) {
  }
  delay(500);


  // หลังจากบรรทัดนี้ จะใช้โค้ด Motion ของคุณได้เหมือนเดิมทั้งหมด
  // lineFollow_PID_time_Smooth(700,40, 70, 40, 7, 0, 30,70);
  lineFollow_PID_time_speed(20000,25,7,0,60,3,100);
  // lineFollow_PID_time_Smooth(400,40, 80, 50, 7, 0, 30,70);
  // lineFollow_PID_time_speed(1500,25,6,0,20,1.5,70);
  // stop_s(25, 7, 0, 30);
  // turn_R_swing_smooth(25);
  // lineFollow_PID_time_Smooth(1000,40, 60, 50, 7, 0, 30,70);
  // lineFollow_PID_time_speed(10000,35,7,0,20,1.5,70);
 
  MotorStop();
}

void loop(){
  // err=calErr(err);
  // if (digitalRead(SWB_PIN) == PRESS) {
  //   Motor(0,100);
  // }
  // else if(digitalRead(SWA_PIN) == PRESS) {
  //   Motor(100,0);
  // }
  // else {
  //   MotorStop();
  // }
  // showAnalog();
  // showDigital();
  // Serial.println(calErr(err));
  // lineFollow_PID(20,2,0,0);
  // lineFollow_PID(40,6,0,10);
  // delay(500);
}