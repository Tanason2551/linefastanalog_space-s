#define SWA_PIN 7
#define SWB_PIN 4
#define PRESS 0
#define PH1 3
#define EN1 5 

#define PH2 2 
#define EN2 6 

void RobotSetup() {
  Serial.begin(115200);
  pinMode(SWA_PIN, INPUT);
  pinMode(SWB_PIN, INPUT);
  pinMode(PH1, OUTPUT);
  pinMode(EN1, OUTPUT);  
  pinMode(PH2, OUTPUT);
  pinMode(EN2, OUTPUT);
}

void SW_A() {
  while (1) {
    if (digitalRead(SWA_PIN) == PRESS) {
      break;
    }
  }
  delay(200);
}

void SW_B() {
  while (1) {
    if (digitalRead(SWB_PIN) == PRESS) {
      break;
    }
  }
  delay(200);
}

int SW_A_PRESS() {
  if (digitalRead(SWA_PIN) == PRESS) return PRESS;
  else {
    return 1;
  }
}

int SW_B_PRESS() {
  if (digitalRead(SWB_PIN) == PRESS) return PRESS;
  else {
    return 1;
  }
}