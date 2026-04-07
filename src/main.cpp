#include <Arduino.h>
#include "rc_input.h"

RCInput rc;
elapsedMillis printTimer;

void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  rc.begin(Serial7);

  Serial.println("CRSF channel dump test");
}

void loop() {
  rc.update();
  RCCommand cmd = rc.getCommand();

  if (printTimer >= 250) {
    printTimer = 0;
    digitalWrite(13, !digitalRead(13));

    Serial.print("valid="); Serial.print(cmd.valid);
    Serial.print(" armed="); Serial.print(cmd.armed);
    Serial.print("steer="); Serial.print(cmd.steer, 3);
    Serial.print(" throttle="); Serial.println(cmd.throttle, 3);
  }
}