#include "lb_line_mux.h"

void Motor1(int Pow) {
  bool dir = (Pow >= 0 ? true : false);
  Pow = abs(Pow);
  Pow = (Pow * 255) / 100;
  digitalWrite(PH1, dir);
  analogWrite(EN1, Pow);
}

void Motor2(int Pow) {
  bool dir = (Pow >= 0 ? true : false);
  Pow = abs(Pow);
  Pow = (Pow * 255) / 100;
  digitalWrite(PH2, dir);
  analogWrite(EN2, Pow);
}

void Motor(int leftspeed, int rightspeed) {
  Motor1(leftspeed);
  Motor2(rightspeed);
}

void MotorStop() {
  Motor(0,0);
  delay(5);
}