#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <ezButton.h>
#include <AccelStepper.h>
#include <EEPROM.h>

// --- Global Offset Variables ---
float pitch_offset = 0.0;
float roll_offset  = 0.0;

// --- Motion Parameters ---
const int  STEPS_PER_REV = 200;
const int  STEPS_PER_MM  = STEPS_PER_REV / 2;   // 2 mm lead screw
const int  JOG_STEPS     = STEPS_PER_MM / 2;    // 0.5 mm jog
const long DUNK_STEPS    = 50L * STEPS_PER_MM;  // 50 mm dunk

// --- Pin Assignments for TB6600 Drivers ---
// Motor 1 (Original)
const int STEP_PIN_1 = 9;
const int DIR_PIN_1  = 8;
const int ENA_PIN_1  = 10;

// Motor 2
const int STEP_PIN_2 = 11;
const int DIR_PIN_2  = 12;
const int ENA_PIN_2  = 13;

// Motor 3 (New)
const int STEP_PIN_3 = 4;
const int DIR_PIN_3  = 5;
const int ENA_PIN_3  = 6;

// --- Other Pin Assignments ---
const int LIMIT1_PIN     = 2;
const int LIMIT2_PIN     = 3;
const int LED_MOVING_PIN = A3;

// --- Button Pins (Control Motor 1 by default) ---
const int BTN_FWD_PIN   = A0;
const int BTN_REV_PIN   = A1;
const int BTN_HOME_PIN  = A2;
const int BTN_DUNK_PIN  = 7;
const int BTN_CALIB_PIN = A4;

// ===================================
// Auto-Levelling Configuration
// ===================================
const float KP = 1;                   // Proportional Gain
const float LEVEL_DEAD_ZONE = 0.1;    // Degrees

// Geometry (mm): motor positions relative to plate center
const float M1_Y  = 220.0;
const float M2_X  = 190.0;
const float M23_Y = -110.0;

bool isLeveling = false;
bool levelingCompletedMessageSent = false;
unsigned long lastLevelingTime = 0;
const unsigned long LEVELING_INTERVAL_MS = 50;

// Forward declaration
void computePitchRoll(float &pitch, float &roll); // (kept for compatibility, unused below)

// =============================
// Encapsulated Components
// =============================

class StepperController {
private:
  AccelStepper stepper;
  const int enaPin;
  const int ledPin;
  const int limit1Pin;
  const int limit2Pin;

  bool invertDir;

  bool holdTorque = false;
  bool wasRunning = false;
  String serialInputBuffer;

  void enableMotor()  { if (enaPin > 0) digitalWrite(enaPin, LOW); }
  void disableMotor() { if (enaPin > 0) digitalWrite(enaPin, HIGH); }

public:
  // Constructor (added invertDir parameter)
  StepperController(int stepPin, int dirPin, int enaPin_,
                    int ledPin_, int limit1Pin_, int limit2Pin_,
                    bool invertDir_ = false)
    : stepper(AccelStepper::DRIVER, stepPin, dirPin),
      enaPin(enaPin_), ledPin(ledPin_),
      limit1Pin(limit1Pin_), limit2Pin(limit2Pin_),
      invertDir(invertDir_) {}

  void begin() {
    if (enaPin > 0) pinMode(enaPin, OUTPUT);
    if (ledPin > 0) pinMode(ledPin, OUTPUT);
    if (limit1Pin > 0) pinMode(limit1Pin, INPUT_PULLUP);
    if (limit2Pin > 0) pinMode(limit2Pin, INPUT_PULLUP);

    if (enaPin > 0) digitalWrite(enaPin, HIGH); // disabled initially
    stepper.setMaxSpeed(4000);
    stepper.setAcceleration(500);
    if (enaPin > 0) {
      stepper.setEnablePin(enaPin);
      stepper.setPinsInverted(invertDir, false, true); // DIR invert, STEP normal, ENA active-low
    }

    if (ledPin > 0) digitalWrite(ledPin, LOW);
  }

  // Runtime direction inversion
  void invertDirection(bool invert) {
    invertDir = invert;
    stepper.setPinsInverted(invertDir, false, true);
  }

  // High-level actions
  void moveRelative(long steps) { stepper.move(steps); }
  void moveToZero()             { stepper.moveTo(0); }
  void jogForward()             { stepper.move(JOG_STEPS); }
  void jogBackward()            { stepper.move(-JOG_STEPS); }
  void dunk()                   { stepper.move(-DUNK_STEPS); }
  void stop()                   { stepper.stop(); }
  long position() const         { return stepper.currentPosition(); }
  void setZero()                { stepper.setCurrentPosition(0); }
  bool isRunning() const        { return stepper.isRunning(); }

  // Speed control
  void setSpeed(float maxSpeed, float acceleration) {
    stepper.setMaxSpeed(maxSpeed);
    stepper.setAcceleration(acceleration);
  }

  // Serial command handling
  static void processCommand(String cmd);

  void handleSerial() {
    while (Serial.available() > 0) {
      char c = (char)Serial.read();
      if (c == '\n' || c == '\r') {
        processCommand(serialInputBuffer);
        serialInputBuffer = "";
      } else {
        serialInputBuffer += c;
      }
    }
  }

  // UPDATED: include motor index for tagged prints
  void update(int motorIndex) {
    bool running = stepper.isRunning();

    if (!wasRunning && running) {
      if (ledPin > 0) digitalWrite(ledPin, HIGH);
      enableMotor();
    }

    if (running && limit1Pin > 0 && limit2Pin > 0) {
      bool movingForward = stepper.distanceToGo() > 0;
      if ((movingForward && digitalRead(limit1Pin) == LOW) ||
          (!movingForward && digitalRead(limit2Pin) == LOW)) {
        stepper.stop();
        Serial.print("LIMIT "); Serial.println(motorIndex); // tagged limit (optional but helpful)
      }
    }

    if (wasRunning && !running) {
      if (ledPin > 0) digitalWrite(ledPin, LOW);
      Serial.print("MOVED "); Serial.println(motorIndex);   // tagged moved
      Serial.print("POS ");                                  // tagged position
      Serial.print(motorIndex);
      Serial.print(" ");
      Serial.println(stepper.currentPosition());
      if (!holdTorque) {
        disableMotor();
      }
    }

    wasRunning = running;
    stepper.run();
  }

  // Allow static handler to access private enable/disable via instances
  friend void startLeveling();
  friend void stopLeveling();
};

// =============================
// Single application objects
// =============================

// Motor 1: normal dir
// Motor 2: inverted dir
// Motor 3: inverted dir
StepperController motors[] = {
  StepperController(STEP_PIN_1, DIR_PIN_1, ENA_PIN_1, LED_MOVING_PIN, LIMIT1_PIN, LIMIT2_PIN, false),
  StepperController(STEP_PIN_2, DIR_PIN_2, ENA_PIN_2, -1, -1, -1, true),
  StepperController(STEP_PIN_3, DIR_PIN_3, ENA_PIN_3, -1, -1, -1, true)
};
const int NUM_MOTORS = sizeof(motors) / sizeof(motors[0]);

class TiltSensor {
private:
  MPU6050 mpu;
  unsigned long lastRead = 0;
  const unsigned long periodMs = 1000;
public:
  void begin() {
    Wire.begin();
    mpu.initialize();
    if (!mpu.testConnection()) {
      Serial.println("MPU6050 connection failed!");
    }
  }

  void getRawPitchRoll(float &raw_pitch, float &raw_roll) {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    float fAx = ax / 16384.0f;
    float fAy = ay / 16384.0f;
    float fAz = az / 16384.0f;
    raw_pitch = atan2(fAy, sqrt(fAx * fAx + fAz * fAz)) * 180.0f / PI;
    raw_roll  = atan2(-fAx, fAz) * 180.0f / PI;
  }

  void getPitchRoll(float &pitch, float &roll) {
    float raw_pitch, raw_roll;
    getRawPitchRoll(raw_pitch, raw_roll);
    pitch = raw_pitch - pitch_offset;
    roll  = raw_roll  - roll_offset;
  }

  void update() {
    unsigned long now = millis();
    if (now - lastRead >= periodMs) {
      lastRead = now;
      float pitch, roll;
      getPitchRoll(pitch, roll);
      Serial.print("TILT ");
      Serial.print(pitch, 2);
      Serial.print(" ");
      Serial.println(roll, 2);
    }
  }
};

class ButtonPanel {
private:
  ezButton btnFwd;
  ezButton btnRev;
  ezButton btnHome;
  ezButton btnDunk;
  ezButton btnCalib;

public:
  ButtonPanel(int fwd, int rev, int home, int dunk, int calib)
    : btnFwd(fwd), btnRev(rev), btnHome(home), btnDunk(dunk), btnCalib(calib) {}

  void begin() {
    btnFwd.setDebounceTime(50);
    btnRev.setDebounceTime(50);
    btnHome.setDebounceTime(50);
    btnDunk.setDebounceTime(50);
    btnCalib.setDebounceTime(50);
  }

  void update(StepperController &ctrl) {
    btnFwd.loop();
    btnRev.loop();
    btnHome.loop();
    btnDunk.loop();
    btnCalib.loop();

    if (btnFwd.isPressed())   ctrl.jogForward();
    if (btnRev.isPressed())   ctrl.jogBackward();
    if (btnHome.isPressed())  ctrl.moveToZero();
    if (btnDunk.isPressed())  ctrl.dunk();
    if (btnCalib.isPressed()) Serial.println("CALIBRATE");
  }
};

TiltSensor tilt;
ButtonPanel buttons(BTN_FWD_PIN, BTN_REV_PIN, BTN_HOME_PIN, BTN_DUNK_PIN, BTN_CALIB_PIN);

// ===================================
// Auto-Levelling Functions
// ===================================
void startLeveling() {
  isLeveling = true;
  levelingCompletedMessageSent = false;
  Serial.println("LEVELING_ON");
}

void stopLeveling() {
  isLeveling = false;
  // Stop any corrective movements
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].stop();
  }
  Serial.println("LEVELING_OFF");
}

void updateLeveling() {
  if (!isLeveling) return;

  unsigned long now = millis();
  if (now - lastLevelingTime < LEVELING_INTERVAL_MS) return;
  lastLevelingTime = now;

  // If any motor is currently running, wait
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (motors[i].isRunning()) return;
  }

  float currentPitch, currentRoll;
  tilt.getPitchRoll(currentPitch, currentRoll);

  // Inside dead-zone?
  if (abs(currentPitch) < LEVEL_DEAD_ZONE && abs(currentRoll) < LEVEL_DEAD_ZONE) {
    if (!levelingCompletedMessageSent) {
      Serial.println("Levelling has been completed");
      levelingCompletedMessageSent = true;
    }
    return; // remain in leveling mode but do nothing
  } else {
    levelingCompletedMessageSent = false;
  }

  // Drive error toward zero (radians)
  float pitch_error_rad = -currentPitch * (PI / 180.0f);
  float roll_error_rad  = -currentRoll  * (PI / 180.0f);

  // Kinematics: required height deltas (mm)
  float dh1 = M1_Y  * pitch_error_rad;
  float dh2 = (M2_X * roll_error_rad) + (M23_Y * pitch_error_rad);
  float dh3 = (-M2_X * roll_error_rad) + (M23_Y * pitch_error_rad);

  // Convert to steps with proportional gain
  long steps1 = lround(dh1 * STEPS_PER_MM * KP);
  long steps2 = lround(dh2 * STEPS_PER_MM * KP);
  long steps3 = lround(dh3 * STEPS_PER_MM * KP);

  // Minimum 1-step nudge if non-zero dh rounded to 0
  auto clampMin1 = [](long steps, float dh) -> long {
    if (steps == 0 && dh != 0.0f) return (dh > 0.0f) ? 1L : -1L;
    return steps;
  };
  steps1 = clampMin1(steps1, dh1);
  steps2 = clampMin1(steps2, dh2);
  steps3 = clampMin1(steps3, dh3);

  // Command motors
  motors[0].moveRelative(steps1);
  motors[1].moveRelative(steps2);
  motors[2].moveRelative(steps3);
}

// =============================
// Command parser (static)
// =============================
void StepperController::processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  // Accelerometer calibration (sets current orientation to zero)
  if (cmd == "CALIBRATE_ACCEL") {
    Serial.println("Calibrating accelerometer... do not move the plate.");
    float raw_p, raw_r;
    tilt.getRawPitchRoll(raw_p, raw_r);
    pitch_offset = raw_p;
    roll_offset  = raw_r;

    EEPROM.put(0, pitch_offset);
    EEPROM.put(sizeof(float), roll_offset);

    Serial.println("ACCEL_CALIBRATED");
    return;
  }

  // Leveling commands first
  if (cmd == "LEVEL_ON") {
    startLeveling();
    return;
  } else if (cmd == "LEVEL_OFF") {
    stopLeveling();
    return;
  }

  // Any non-leveling command stops leveling
  if (isLeveling) {
    stopLeveling();
  }

  // Speed configuration for all motors
  if (cmd.startsWith("CONFIG_SPEED ")) {
    int first_space  = cmd.indexOf(' ');
    int second_space = cmd.indexOf(' ', first_space + 1);

    if (first_space > 0 && second_space > 0) {
      float max_speed = cmd.substring(first_space + 1, second_space).toFloat();
      float accel     = cmd.substring(second_space + 1).toFloat();

      for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].setSpeed(max_speed, accel);
      }
      Serial.println("SPEED_CONFIG_OK");
    }
  }

  // Specific motor control: "MOVE_M <motor_index> <value> <units>"
  else if (cmd.startsWith("MOVE_M ")) {
    int first_space  = cmd.indexOf(' ');
    int second_space = cmd.indexOf(' ', first_space + 1);
    int third_space  = cmd.indexOf(' ', second_space + 1);

    if (first_space > 0 && second_space > 0 && third_space > 0) {
      int motor_index = cmd.substring(first_space + 1, second_space).toInt();
      float value     = cmd.substring(second_space + 1, third_space).toFloat();
      String units    = cmd.substring(third_space + 1);

      long steps;
      if (units.equalsIgnoreCase("mm")) {
        steps = lround(value * STEPS_PER_MM);
      } else { // assume steps
        steps = lround(value);
      }

      if (motor_index >= 0 && motor_index < NUM_MOTORS) {
        motors[motor_index].moveRelative(steps);
      } else if (motor_index == NUM_MOTORS || motor_index == 3) { // "All"
        for (int i = 0; i < NUM_MOTORS; i++) {
          motors[i].moveRelative(steps);
        }
      }
    }
  }

  // Original single-axis commands (motor 0)
  else if (cmd.startsWith("MOVE ")) {
    float deg  = cmd.substring(5).toFloat();
    long steps = lround((deg / 360.0f) * STEPS_PER_REV);
    motors[0].moveRelative(steps);
  } else if (cmd == "HOME") {
    motors[0].moveToZero();
  } else if (cmd == "DUNK") {
    motors[0].dunk();
  } else if (cmd == "GET_POS") {
    Serial.print("POS "); Serial.print(0); Serial.print(" ");
    Serial.println(motors[0].position());
  } else if (cmd == "STOP" || cmd == "CANCEL" || cmd == "ESTOP") {
    for (int i = 0; i < NUM_MOTORS; i++) {
      motors[i].stop();
    }
    Serial.println(cmd == "ESTOP" ? "ESTOP" : "STOPPING");
  } else if (cmd == "HOLD ON") {
    for (int i = 0; i < NUM_MOTORS; i++) {
      motors[i].holdTorque = true;
      motors[i].enableMotor();
    }
    Serial.println("HOLD ON");
  } else if (cmd == "HOLD OFF") {
    for (int i = 0; i < NUM_MOTORS; i++) {
      motors[i].holdTorque = false;
      if (!motors[i].isRunning()) {
        motors[i].disableMotor();
      }
    }
    Serial.println("HOLD OFF");
  } else if (cmd == "SET_ZERO") {
    motors[0].setZero();
    Serial.println("HOMED"); // UI expects this token
    Serial.print("POS "); Serial.print(0); Serial.print(" ");
    Serial.println(motors[0].position());
  }
}

// =============================
// Arduino entry points
// =============================
void setup() {
  Serial.begin(9600);

  // Load accelerometer offsets from EEPROM (default to 0 if uninitialized)
  EEPROM.get(0, pitch_offset);
  EEPROM.get(sizeof(float), roll_offset);
  if (isnan(pitch_offset)) pitch_offset = 0.0;
  if (isnan(roll_offset))  roll_offset  = 0.0;

  Serial.println("Stepper ready (TB600) - Multi-Motor with Leveling");

  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].begin();
  }
  tilt.begin();
  buttons.begin();
}

void loop() {
  // Buttons control the primary motor (motor 0)
  buttons.update(motors[0]);

  // Handle incoming serial commands
  motors[0].handleSerial();

  // Update all motors (tagging outputs by index)
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].update(i);
  }

  // Push TILT telemetry for the GUI
  tilt.update();

  // If active, run leveling controller
  updateLeveling();
}
