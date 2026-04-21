#include "HardwareSerial.h"
#include "sn_line_mux.h"
#include "mt_line_mux.h"
float  errors = 0, output = 0, integral = 0, derivative = 0, previous_error = 0;

void run_F_time(unsigned long timeoutTime,int RUN_speed){ //เดินหน้า
  unsigned long lastTime = millis();
  unsigned long currentTime = lastTime;
  while (currentTime - lastTime < timeoutTime){
    currentTime = millis();
      Motor(RUN_speed,RUN_speed);
    }
    MotorStop();
    return;
}

void run_B_time(unsigned long timeoutTime,int RUN_speed){ //เดินถอยหลัง
  unsigned long lastTime = millis();
  unsigned long currentTime = lastTime;
  while (currentTime - lastTime < timeoutTime){
    currentTime = millis();
      Motor(-RUN_speed,-RUN_speed);
    }
    MotorStop();
    return;
}

void lineFollow_PID(int RUN_PID_speed , float RUN_PID_KP, float RUN_PID_KI, float RUN_PID_KD ) {

  errors = calErr(previous_error);
  integral = integral + errors ;
  derivative = (errors - previous_error) ;
  output = RUN_PID_KP * errors  + RUN_PID_KI * integral + RUN_PID_KD * derivative;
  int motorL = constrain(RUN_PID_speed + output, -70, 100);
  int motorR = constrain(RUN_PID_speed - output, -70, 100);
  Motor(motorL,motorR);
  previous_error = errors;
  // Serial.print(motorL);
  // Serial.print(":");
  // Serial.println(motorR);
}

void lineFollow_PID_time(unsigned long timeoutTime,int RUN_PID_speed , float RUN_PID_KP, float RUN_PID_KI, float RUN_PID_KD) {

  unsigned long lastTime = millis();
  unsigned long currentTime = lastTime;


  while (currentTime - lastTime < timeoutTime){
    currentTime = millis();
    int speed_PID = RUN_PID_speed;
    errors = calErr(previous_error);
    if (errors == 0) integral = 0;
    integral = integral + errors ; 
    derivative = (errors - previous_error) ;
    output = (RUN_PID_KP * errors)   + (RUN_PID_KD * derivative);
    int motorL = constrain(RUN_PID_speed + output, -70, 100);
    int motorR = constrain(RUN_PID_speed - output, -70, 100);
    Motor(motorL,motorR);
    previous_error = errors;
  }
    // motor(1, 0); 
    // motor(2, 0);
    return;
}

void PID_time_turn (unsigned long timeoutTime,int RUN_PID_speed , float RUN_PID_KP, float RUN_PID_KI, float RUN_PID_KD){
  unsigned long lastTime = millis();
  unsigned long currentTime = lastTime;
  while (currentTime - lastTime < timeoutTime){
    currentTime = millis();
    int speed_PID = RUN_PID_speed;
    errors = calErrif(previous_error);
    integral = integral + errors ;
    derivative = (errors - previous_error) ;
    output = RUN_PID_KP * errors  + RUN_PID_KI * integral + RUN_PID_KD * derivative;
    int motorL = constrain(RUN_PID_speed + output, -80, 100);
    int motorR = constrain(RUN_PID_speed - output, -80, 100);
    Motor(motorL,motorR);
    previous_error = errors;
    // Serial.println(errors);
  }
}

void lineFollow_PID_time_Smooth(unsigned long timeoutTime, int min_speed, int max_speed, float accel_slope, float RUN_PID_KP, float RUN_PID_KI, float RUN_PID_KD, int Maxturn) {
  
  unsigned long startTime = millis();
  unsigned long currentTime = startTime;
  unsigned long elapsedTime = 0;

  float previous_error = 0;
  float integral = 0;
  float derivative = 0;
  float output = 0;
  float errors = 0;

  float slope_per_ms = accel_slope / 1000.0;

  unsigned long accelTime = (max_speed - min_speed) / slope_per_ms;
  float actual_max_speed = max_speed;

  if (accelTime * 2 > timeoutTime) {
    accelTime = timeoutTime / 2;
    actual_max_speed = min_speed + (slope_per_ms * accelTime); 
  }

  unsigned long decelStartTime = timeoutTime - accelTime;

  int current_base_speed = min_speed;

  while (elapsedTime < timeoutTime) {
    currentTime = millis();
    elapsedTime = currentTime - startTime;

    if (elapsedTime < accelTime) {
      current_base_speed = min_speed + (slope_per_ms * elapsedTime);
      
    } else if (elapsedTime > decelStartTime) {
      unsigned long decelElapsedTime = elapsedTime - decelStartTime;
      current_base_speed = actual_max_speed - (slope_per_ms * decelElapsedTime);
      
    } else {
      current_base_speed = actual_max_speed;
    }


    errors = calErr(previous_error);
    
    if (errors == 0) integral = 0;
    integral = integral + errors; 
    derivative = (errors - previous_error);
    output = (RUN_PID_KP * errors) + (RUN_PID_KI * integral) + (RUN_PID_KD * derivative);
    int motorL = constrain(current_base_speed + output, -Maxturn, 100);
    int motorR = constrain(current_base_speed - output, -Maxturn, 100);
    
    Motor(motorL, motorR);
    
    previous_error = errors;
  }

  // เมื่อหมดเวลา (หลุดลูป while) สามารถสั่งให้หุ่นหยุดนิ่ง หรือวิ่งความเร็วต่ำสุดต่อไปก็ได้
  // Motor(0, 0); // หากต้องการให้หยุดสนิทเมื่อหมดเวลา ให้ลบ // ออก
  return;
}

void lineFollow_PID_time_speed(unsigned long timeoutTime, int RUN_PID_speed, float RUN_PID_KP, float RUN_PID_KI, float RUN_PID_KD, float RUN_PID_SlowingFactor, int Maxturn) {

  unsigned long lastTime = millis();
  unsigned long currentTime = lastTime;
  int deadband_threshold = 1; 
  while (currentTime - lastTime < timeoutTime) {
    currentTime = millis();
    
    errors = calErr(previous_error);
    if (abs(errors) < deadband_threshold) {
      errors = 0;      
      integral = 0;     
    }

    if (errors == 0) integral = 0;
    integral = integral + errors; 
    derivative = (errors - previous_error);
    output = (RUN_PID_KP * errors) + (RUN_PID_KD * derivative);

    float speed_reduction = abs(errors) * RUN_PID_SlowingFactor;
    int current_speed = RUN_PID_speed - speed_reduction;
    
    int motorL = constrain(current_speed + output, -Maxturn, 100);
    int motorR = constrain(current_speed - output, -Maxturn, 100);

    Motor(motorL, motorR);
    previous_error = errors;
  }

}




void stop_s (int RUN_PID_speed , float RUN_PID_KP, float RUN_PID_KI, float RUN_PID_KD){ //หยุดเมื่อเซนเซอร์ข้างใดข้างหนึ่งเจอเส้น
  while (true){
    lineFollow_PID(RUN_PID_speed,RUN_PID_KP,RUN_PID_KI,RUN_PID_KD);
    int b=0,w=0;
    for(int i=0;i<15;i++){
      if (digitalVal[i] == 1) {
        b++;
      } else {
        w++;
      }
    }
    if(b>w){
      break;
    }
    }
    return;
}

void stop_s4 (int RUN_PID_speed , float RUN_PID_KP, float RUN_PID_KI, float RUN_PID_KD){ //หยุดเมื่อเซนเซอร์ข้างใดข้างหนึ่งเจอเส้น
  while (true){
    lineFollow_PID(RUN_PID_speed,RUN_PID_KP,RUN_PID_KI,RUN_PID_KD);
    int b=0,w=0;
    for(int i=0;i<15;i++){
      if (digitalVal[i] == 1) {
        b++;
      } else {
        w++;
      }
    }
    if(b>4){
      break;
    }
    }
    return;
}

void turn_R (unsigned long timeoutTime,int speed){
  unsigned long lastTime = millis();
  unsigned long currentTime = lastTime;
    while (currentTime - lastTime < timeoutTime){
    currentTime = millis();
    Motor(speed,-speed);
  }
    return;
}
void turn_L (unsigned long timeoutTime,int speed){
  unsigned long lastTime = millis();
  unsigned long currentTime = lastTime;
    while (currentTime - lastTime < timeoutTime){
    currentTime = millis();
    Motor(speed,-speed);
  }
    return;
}

void turn_R_1steps (int turn_speed){
    Motor(turn_speed, -turn_speed); 


  while (true) {
    A2D();
    if (digitalVal[7] == 1) {
      break; 
    }
  }
}
void turn_L_1steps (int turn_speed) {
  Motor(-turn_speed, turn_speed); 

  while (true) {
    A2D();
    if (digitalVal[7] == 1) {
      break;
    }
  }
}

void turn_R_3steps (int turn_speed){
    Motor(turn_speed, -turn_speed); 

  while (true) {
    A2D();
    if (digitalVal[0] == 1) {
      break;
    }
  }
  while (true) {
    A2D();
    if (digitalVal[0] == 0) {
      break; 
    }
  }
  while (true) {
    A2D();
    if (digitalVal[7] == 1) {
      break;
    }
  }
}


void turn_L_3steps (int turn_speed) {
  Motor(-turn_speed, turn_speed); 

  while (true) {
    A2D();
    if (digitalVal[15] == 1) {
      break;
    }
  }
  while (true) {
    A2D();
    if (digitalVal[15] == 0) {
      break;
    }
  }
  while (true) {
    A2D();
    if (digitalVal[7] == 1) {
      break;
    }
  }
}


void turn_R_optimized (int fast_speed, int slow_speed) {
  Motor(fast_speed, -fast_speed); 
  while (true) {
    A2D(); 
    if (digitalVal[0] == 1) {
      break; 
    }
  }
  while (true) {
    A2D(); 
    if (digitalVal[0] == 0) {
      break; 
    }
  }

  while (true) {
    A2D(); 
    if (digitalVal[15] == 1 || digitalVal[14] == 1) {
      break;
    }
  }

  float current_speed = fast_speed;
  float decrement_step = 10;
  while (true) {
    A2D(); 
    if (digitalVal[7] == 1) {
      break; 
    }
    if (current_speed > slow_speed) {
      current_speed -= decrement_step; 
    }
    Motor((int)current_speed, -(int)current_speed);
  }

  Motor(-slow_speed, slow_speed); 
  delay(60);
}


void turn_L_optimized (int fast_speed, int slow_speed) {
  Motor(-fast_speed, fast_speed); 

  while (true) {
    A2D(); 
    if (digitalVal[15] == 1) {
      break; 
    }
  }

  while (true) {
    A2D(); 
    if (digitalVal[15] == 0) {
      break; 
    }
  }

  while (true) {
    A2D(); 
    if (digitalVal[0] == 1 || digitalVal[1] == 1) {
      break;
    }
  }

  float current_speed = fast_speed;  
  float decrement_step = 10;
  while (true) {
    A2D(); 
    if (digitalVal[7] == 1) {
      break; 
    }

    if (current_speed > slow_speed) {
      current_speed -= decrement_step; 
    }

    Motor(-(int)current_speed, (int)current_speed);
  }

  Motor(slow_speed, -slow_speed); 
  delay(60);            
  
}



// =========================================================================
// 2. ฟังก์ชันเลี้ยวแบบ Swing Turn (ล้อในหยุด ล้อนอกดัน) - ใช้การเช็คเซนเซอร์แทน Delay
// =========================================================================
void turn_R_swing_smooth(int turn_speed) {
  // สั่งล้อขวา(ล้อใน)หยุด, ล้อซ้าย(ล้อนอก)ดัน หุ่นจะวาดโค้งขวา
  Motor(turn_speed, 0); 
  
  // ลูปที่ 1: รอให้หุ่นเชิดหัวออกจากเส้นทางแยกเดิม (เซนเซอร์กลางอ่านเจอสีขาว)
  while (true) {
    A2D();
    if (digitalVal[7] == 0 && digitalVal[8] == 0) {
      break; 
    }
  }

  // ลูปที่ 2: รอจนกว่าเซนเซอร์ตรงกลางจะสวิงไปเตะเส้นเป้าหมาย
  while (true) {
    A2D();
    if (digitalVal[7] == 1 || digitalVal[8] == 1) {
      break; 
    }
  }
}

void turn_L_swing_smooth(int turn_speed) {
  // สั่งล้อขวา(ล้อนอก)ดัน, ล้อซ้าย(ล้อใน)หยุด
  Motor(0, turn_speed); 
  
  // ลูปที่ 1: รอให้หุ่นเชิดหัวออกจากเส้นทางแยกเดิม
  while (true) {
    A2D();
    if (digitalVal[7] == 0 && digitalVal[8] == 0) {
      break; 
    }
  }

  // ลูปที่ 2: รอจนกว่าเซนเซอร์ตรงกลางจะสวิงไปเตะเส้นเป้าหมาย
  while (true) {
    A2D();
    if (digitalVal[7] == 1 || digitalVal[8] == 1) {
      break; 
    }
  }
}

// =========================================================================
// 3. ฟังก์ชันเลี้ยวแบบ Arc Turn (ตีโค้ง ล้อไม่หยุดเลย) - ใช้การเช็คเซนเซอร์แทน Delay
// =========================================================================
void turn_R_arc_smooth(int fast_speed, int slow_speed) {
  // ล้อซ้ายวิ่งเร็ว ล้อขวาวิ่งช้า (แต่ยังเดินหน้าทั้งคู่)
  Motor(fast_speed, slow_speed); 
  
  // ลูปที่ 1: รอให้หุ่นเชิดหัวออกจากเส้นทางแยกเดิม
  while (true) {
    A2D();
    if (digitalVal[7] == 0 && digitalVal[8] == 0) {
      break; 
    }
  }

  // ลูปที่ 2: รอจนกว่าเซนเซอร์ตรงกลางจะสวิงไปเตะเส้นเป้าหมาย
  while (true) {
    A2D();
    if (digitalVal[7] == 1 || digitalVal[8] == 1) {
      break; 
    }
  }
}

void turn_L_arc_smooth(int fast_speed, int slow_speed) {
  // ล้อขวาวิ่งเร็ว ล้อซ้ายวิ่งช้า
  Motor(slow_speed, fast_speed); 
  
  // ลูปที่ 1: รอให้หุ่นเชิดหัวออกจากเส้นทางแยกเดิม
  while (true) {
    A2D();
    if (digitalVal[7] == 0 && digitalVal[8] == 0) {
      break; 
    }
  }

  // ลูปที่ 2: รอจนกว่าเซนเซอร์ตรงกลางจะสวิงไปเตะเส้นเป้าหมาย
  while (true) {
    A2D();
    if (digitalVal[7] == 1 || digitalVal[8] == 1) {
      break; 
    }
  }
}


