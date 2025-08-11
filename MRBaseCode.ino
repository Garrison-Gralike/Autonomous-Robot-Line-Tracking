#include "functions.h"
#define BAUDRATE 9600

void setup() {
  encoder_init();
  motorInit();
  setMotorSpeed(0, 100);  // left motor
  setMotorSpeed(1, 100);  // right motor
  Serial.begin(BAUDRATE);
  Serial1.begin(BAUDRATE);
  timerInit();
  wheelDesVel[0]=0.;
  wheelDesVel[1]=0.;
}
void loop() {
  command_server();
}
void control(void)
{
  getCurrentStatus();
  lowLevelControl();
  }
  ISR(TIMER1_COMPA_vect) /* timer compare interrupt service routine*/
  {
  control();
}