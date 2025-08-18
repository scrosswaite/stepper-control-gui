#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <ezButton.h>
#include <Stepper.h>

// --- Motion Parameters ---
const int STEPS_PER_REV = 200;
const int STEPS_PER_MM  = STEPS_PER_REV / 12;   // 12mm lead screw
const int JOG_STEPS     = STEPS_PER_MM / 2;     // 0.5mm jog
const long DUNK_STEPS   = 50L * STEPS_PER_MM;   // 50mm dunk

// --- Pin Assignments ---
const int IN1_PIN = 8;
const int IN2_PIN = 9;
const int IN3_PIN = 10;
const int IN4_PIN = 11;

const int LIMIT1_PIN = 2;
const int LIMIT2_PIN = 3;

const int LED_MOVING_PIN = A3;

const int BTN_FWD_PIN   = 4;
const int BTN_REV_PIN   = 5;
const int BTN_HOME_PIN  = 6;
const int BTN_DUNK_PIN  = 7;
const int BTN_CALIB_PIN = 12;

// --- Global State ---
long currentPosition = 0;
bool isMoving = false;
unsigned long lastTiltTime = 0;
volatile bool cancelRequested = false;
bool holdTorque = false;   // NEW: default to coils off at rest

// Read any pending serial and set cancel flag while moving
inline void pollCancelFromSerial() {
  if (Serial.available()) {
    // very short timeout so this doesn't block stepping
    unsigned long prev = Serial.getTimeout();
    Serial.setTimeout(1);
    String cmd = Serial.readStringUntil('\n');
    Serial.setTimeout(prev);

    cmd.trim();
    if (cmd.length()) {
      if (cmd == "STOP" || cmd == "CANCEL") {
        cancelRequested = true;
        Serial.println("STOPPING");   // GUI can unlock on this
      } else if (cmd == "ESTOP") {
        cancelRequested = true;
        Serial.println("ESTOP");      // GUI can unlock on this
      }
    }
  }
}

// --- Objects ---
MPU6050 mpu;
Stepper myStepper(STEPS_PER_REV, IN1_PIN, IN2_PIN, IN3_PIN, IN4_PIN);

ezButton buttonFwd(BTN_FWD_PIN);
ezButton buttonRev(BTN_REV_PIN);
ezButton buttonHome(BTN_HOME_PIN);
ezButton buttonDunk(BTN_DUNK_PIN);
ezButton buttonCalib(BTN_CALIB_PIN);

// --- Helpers ---
bool limitReached() {
  return (digitalRead(LIMIT1_PIN) == LOW || digitalRead(LIMIT2_PIN) == LOW);
}

// NEW: fully de-energise coils
inline void coilsOff() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
}

// Interruptible step loop with limit & cancel checks
void moveMotor(long steps) {
  if (isMoving) return;

  // Pre-check
  if ((steps > 0 && digitalRead(LIMIT1_PIN) == LOW) ||
      (steps < 0 && digitalRead(LIMIT2_PIN) == LOW)) {
    Serial.println("LIMIT");
    return;
  }

  isMoving = true;
  digitalWrite(LED_MOVING_PIN, HIGH);

  long remaining = labs(steps);
  int dir = (steps >= 0) ? 1 : -1;
  long moved = 0;

  while (remaining > 0) {

    pollCancelFromSerial();

    if (cancelRequested) {
      cancelRequested = false;
      Serial.println("STOPPED");
      break;
    }
    if ((dir > 0 && digitalRead(LIMIT1_PIN) == LOW) ||
        (dir < 0 && digitalRead(LIMIT2_PIN) == LOW)) {
      Serial.println("LIMIT");
      break;
    }

    myStepper.step(dir);
    currentPosition += dir;
    moved += dir;
    remaining--;
  }

  digitalWrite(LED_MOVING_PIN, LOW);
  isMoving = false;

  // cut idle current unless holding torque requested
  if (!holdTorque) {
    coilsOff();
  }

  // Always report position after a motion attempt
  Serial.print("POS ");
  Serial.println(currentPosition);

  if (remaining == 0) {
    float degMoved = (float)moved / STEPS_PER_REV * 360.0f;
    Serial.print("MOVED ");
    Serial.println(degMoved, 2);
  }
}

void computePitchRoll(float &pitch, float &roll) {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  float fAx = ax / 16384.0f;
  float fAy = ay / 16384.0f;
  float fAz = az / 16384.0f;
  pitch = atan2(fAy, sqrt(fAx * fAx + fAz * fAz)) * 180.0f / PI;
  roll  = atan2(-fAx, fAz) * 180.0f / PI;
}

void handleSerialCommands() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("MOVE ")) {
      float deg = cmd.substring(5).toFloat();
      long steps = lround((deg / 360.0f) * STEPS_PER_REV);
      moveMotor(steps);
    }
    else if (cmd == "HOME") {
      moveMotor(-currentPosition);
      currentPosition = 0;
      Serial.println("HOMED");
      Serial.print("POS ");
      Serial.println(currentPosition);
    }
    else if (cmd == "DUNK") {
      moveMotor(-DUNK_STEPS);
    }
    else if (cmd == "GET_POS") {
      Serial.print("POS ");
      Serial.println(currentPosition);
    }
    else if (cmd == "STOP" || cmd == "CANCEL") {
      cancelRequested = true;
      Serial.println("STOPPING");
      if (!isMoving && !holdTorque) coilsOff();  
    }
    else if (cmd == "ESTOP") {
      cancelRequested = true;
      Serial.println("ESTOP");
      if (!isMoving && !holdTorque) coilsOff();   
    }
    else if (cmd == "HOLD ON") {                 
      holdTorque = true;
      Serial.println("HOLD ON");
    }
    else if (cmd == "HOLD OFF") {                  
      holdTorque = false;
      coilsOff();
      Serial.println("HOLD OFF");
    } 
    else if (cmd == "SET_ZERO") {
      currentPosition = 0;
      Serial.println("HOMED");
      Serial.print("POS ");
      Serial.println(currentPosition);
      if (!isMoving && !holdTorque) {
        coilsOff();
      }
    }
  }
}

void handleButtons() {
  buttonFwd.loop(); buttonRev.loop(); buttonHome.loop(); buttonDunk.loop(); buttonCalib.loop();

  if (buttonFwd.isPressed())  { moveMotor(JOG_STEPS); }
  if (buttonRev.isPressed())  { moveMotor(-JOG_STEPS); }
  if (buttonHome.isPressed()) {
    moveMotor(-currentPosition);
    currentPosition = 0;
    Serial.println("HOMED");
    Serial.print("POS ");
    Serial.println(currentPosition);
  }
  if (buttonDunk.isPressed()) { moveMotor(-DUNK_STEPS); }
  if (buttonCalib.isPressed()){ Serial.println("CALIBRATE"); }
}

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(50);
  Serial.println("Stepper ready");

  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
  }

  myStepper.setSpeed(100); // RPM

  pinMode(LIMIT1_PIN, INPUT_PULLUP);
  pinMode(LIMIT2_PIN, INPUT_PULLUP);
  pinMode(LED_MOVING_PIN, OUTPUT);

  pinMode(BTN_FWD_PIN,   INPUT_PULLUP);
  pinMode(BTN_REV_PIN,   INPUT_PULLUP);
  pinMode(BTN_HOME_PIN,  INPUT_PULLUP);
  pinMode(BTN_DUNK_PIN,  INPUT_PULLUP);
  pinMode(BTN_CALIB_PIN, INPUT_PULLUP);

  buttonFwd.setDebounceTime(50);
  buttonRev.setDebounceTime(50);
  buttonHome.setDebounceTime(50);
  buttonDunk.setDebounceTime(50);
  buttonCalib.setDebounceTime(50);

  // NEW: start de-energised
  coilsOff();
}

void loop() {
  handleButtons();
  handleSerialCommands();

  unsigned long now = millis();
  if (now - lastTiltTime >= 100) {
    lastTiltTime = now;
    float pitch, roll;
    computePitchRoll(pitch, roll);

    Serial.print("TILT ");
    Serial.print(pitch, 2);
    Serial.print(" ");
    Serial.println(roll, 2);
  }
}
