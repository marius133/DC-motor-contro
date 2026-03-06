#include <Arduino.h>
#include <Encoder.h>
#include <MotorDriver.h>

// -------------------- Encoder pins --------------------
constexpr int A_L = 6, B_L = 7;
constexpr int A_R = 4, B_R = 5;

Encoder encLeft(A_L, B_L);
Encoder encRight(A_R, B_R);

// -------------------- Motor driver pins --------------------
// EN2, EN1, PWM1, PWM2, IS2, IS1

constexpr uint8_t L_EN2  = 20;
constexpr uint8_t L_EN1  = 21;
constexpr uint8_t L_PWM1 = 22;
constexpr uint8_t L_PWM2 = 23;

constexpr uint8_t R_EN2  = 26;
constexpr uint8_t R_EN1  = 27;
constexpr uint8_t R_PWM1 = 24;
constexpr uint8_t R_PWM2 = 25;

MotorDriver::DriverPins pinsLeft  { L_EN2, L_EN1, L_PWM1, L_PWM2, 0, 0 };
MotorDriver::DriverPins pinsRight { R_EN2, R_EN1, R_PWM1, R_PWM2, 0, 0 };

// CONFIG: ferdig før motor-objektene konstrueres
MotorDriver::DriverConfig cfg = []{
  MotorDriver::DriverConfig c;
  c.pwm_freq_hz = 20000;
  c.pwm_resolution_bits = 12;
  c.deadzone = 0.02f;
  c.invert = false;
  return c;
}();

MotorDriver motorLeft(pinsLeft, cfg);
MotorDriver motorRight(pinsRight, cfg);

elapsedMillis heartbeatTimer;
elapsedMillis runTimer;

void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);

  pinMode(A_L, INPUT_PULLUP);
  pinMode(B_L, INPUT_PULLUP);
  pinMode(A_R, INPUT_PULLUP);
  pinMode(B_R, INPUT_PULLUP);

  encLeft.write(0);
  encRight.write(0);

  motorLeft.begin();
  motorRight.begin();
  motorLeft.enable(true);
  motorRight.enable(true);

  Serial.println("Test: forward only, u=0.30 for 5s, then stop 2s, repeat.");
  heartbeatTimer = 0;
  runTimer = 0;
}

void loop() {
  // Kjør framover 5s, stopp 2s, repeat
  const float u = 0.15f;   // øk litt for å sikre at den faktisk starter

  if (runTimer < 5000) {
    motorLeft.setCommand(u);
    motorRight.setCommand(u);
  } else if (runTimer < 7000) {
    motorLeft.setCommand(0.0f);
    motorRight.setCommand(0.0f);
  } else {
    runTimer = 0;
  }

  // Heartbeat print alltid
  if (heartbeatTimer >= 250) {
    heartbeatTimer = 0;
    digitalWrite(13, !digitalRead(13));

    Serial.print("ms="); Serial.print(millis());
    Serial.print(" EncL="); Serial.print(encLeft.read());
    Serial.print(" EncR="); Serial.print(encRight.read());
    Serial.print(" uL="); Serial.print(motorLeft.lastCommand(), 3);
    Serial.print(" uR="); Serial.println(motorRight.lastCommand(), 3);
  }
}