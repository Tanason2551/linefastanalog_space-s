#include "PID_line_mux.h"
float err = 0; // เปลี่ยนเป็น float รองรับ Analog Error

void setup(){
  RobotSetup();
  Serial.begin(115200);
  
  isWhiteLine = false;

  // while (true) {
  //   if (digitalRead(SWA_PIN) == PRESS) {
  //     Serial.println("Auto-Calibrating... Please sweep sensor across line");
  //     calibrateSensorAuto(10, 300);
  //     Serial.println("Auto-Calibration Done!");
  //     delay(500);
  //     break;
  //   }
  //   else if (digitalRead(SWB_PIN) == PRESS) {
      setSensorBW();
      // Serial.println("Preset-Calibration Done!");
      // delay(500);
      // break;
  //   }
  // }
  while (digitalRead(SWA_PIN) != PRESS && digitalRead(SWB_PIN) != PRESS) {
  }
  delay(500);



  // lineFollow_PID_time_Smooth(900,40, 100, 110, 5, 0, 100,70);
  // stop_s(30, 4, 0, 70);
  // turn_L_swing_smooth(40);
  // stop_s(30, 5, 0, 70);
  // turn_L_swing_smooth(40);
  // lineFollow_PID_time_speed(4600,40,5.5,0,80,4,70);
  // lineFollow_PID_time_Smooth(350,40, 70, 110, 5, 0, 100,70);
  // lineFollow_PID_time_speed(3400,40,5,0,70,4.5,70);
  // stop_s(30, 4, 0, 70);
  // turn_L_swing_smooth(40);
  // lineFollow_PID_time_speed(2000,40,5,0,70,4.5,70);
  // lineFollow_PID_time_speed(1800,45,5,0,70,4.5,70);
  // lineFollow_PID_time_Smooth(800,40, 100, 110, 5, 0, 100,70);





  lineFollow_PID_time_Smooth(900,40, 100, 110, 5, 0, 100,70);
  lineFollow_PID_time_speed(500,40,5,0,80,4,70);
  lineFollow_PID_time_Smooth(400,40, 100, 110, 5, 0, 100,70);
  lineFollow_PID_time_speed(7100,40,5,0,70,4.5,70);
  lineFollow_PID_time_Smooth(350,40, 70, 110, 5, 0, 100,70);
  lineFollow_PID_time_speed(4800,40,5.5,0,80,4,70);
  stop_s(35, 4, 0, 70);
  turn_R_swing_smooth(40);
  stop_s(35, 4, 0, 70);
  turn_R_swing_smooth(40);
  lineFollow_PID_time_speed(500,40,5,0,70,4.5,70);
 lineFollow_PID_time_Smooth(800,40, 100, 110, 5, 0, 100,70);
 
  MotorStop();
}

void loop(){

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
  // showError();
}