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
const int  MICROSTEPS    = 16;             // <<< set to your TB6600 DIP (1,2,4,8,16,32…)
const int  STEPS_PER_REV = 200 * MICROSTEPS;
const float LEAD_MM      = 2.0f;           // 2 mm lead screw
const int  STEPS_PER_MM  = (int)(STEPS_PER_REV / LEAD_MM);

const int  JOG_STEPS     = STEPS_PER_MM / 2;    
const long DUNK_STEPS    = (long)(50.0f * STEPS_PER_MM);  

// --- Pin Assignments for TB6600 Drivers ---
const int STEP_PIN_1 = 9;
const int DIR_PIN_1  = 8;
const int ENA_PIN_1  = 10;

const int STEP_PIN_2 = 11;
const int DIR_PIN_2  = 12;
const int ENA_PIN_2  = 13;

const int STEP_PIN_3 = 4;
const int DIR_PIN_3  = 5;
const int ENA_PIN_3  = 6;

// --- Other Pin Assignments ---
const int LIMIT1_PIN = 2;
const int LIMIT2_PIN = 3;
const int LED_MOVING_PIN = A3;

// --- Buttons disabled for now ---
#define DISABLE_BUTTONS 1

// ===================================
// Auto-Levelling Configuration
// ===================================
float KP = 0.8f;                         
const float LEVEL_DEAD_ZONE = 0.1f;      
const unsigned long LEVELING_INTERVAL_MS = 50;

bool isLeveling = false;
bool levelingCompletedMessageSent = false;
unsigned long lastLevelingTime = 0;

// Plate geometry (mm)
const float X_by_motor[3] = { +110.0f, -270.0f,   0.0f };
const float Y_by_motor[3] = { +150.0f, +150.0f, -230.0f };

const long MAX_STEP_CORR = 240;          

// =============================
// StepperController class
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

    if (enaPin > 0) digitalWrite(enaPin, HIGH); 
    stepper.setMaxSpeed(4000);
    stepper.setAcceleration(500);
    if (enaPin > 0) {
      stepper.setEnablePin(enaPin);
      stepper.setPinsInverted(invertDir, false, true);
    }
    if (ledPin > 0) digitalWrite(ledPin, LOW);
  }

  void invertDirection(bool invert) {
    invertDir = invert;
    stepper.setPinsInverted(invertDir, false, true);
  }

  void moveRelative(long steps) { stepper.move(steps); }
  void moveToZero()             { stepper.moveTo(0); }
  void jogForward()             { stepper.move(JOG_STEPS); }
  void jogBackward()            { stepper.move(-JOG_STEPS); }
  void dunk()                   { stepper.move(-DUNK_STEPS); }
  void stop()                   { stepper.stop(); }
  long position() const         { return stepper.currentPosition(); }
  void setZero()                { stepper.setCurrentPosition(0); }
  bool isRunning() const        { return stepper.isRunning(); }

  void setSpeed(float maxSpeed, float acceleration) {
    stepper.setMaxSpeed(maxSpeed);
    stepper.setAcceleration(acceleration);
  }

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

  void update() {
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
        Serial.println("LIMIT");
      }
    }

    if (wasRunning && !running) {
      if (ledPin > 0) digitalWrite(ledPin, LOW);
      if (!isLeveling) {
        Serial.println("MOVED");
        Serial.print("POS ");
        Serial.println(stepper.currentPosition());
      }
      if (!holdTorque) disableMotor();
    }

    wasRunning = running;
    stepper.run();
  }

  friend void StepperController::processCommand(String cmd);
};

// =============================
// Objects
// =============================
StepperController motors[] = {
  StepperController(STEP_PIN_1, DIR_PIN_1, ENA_PIN_1, LED_MOVING_PIN, LIMIT1_PIN, LIMIT2_PIN, false),
  StepperController(STEP_PIN_2, DIR_PIN_2, ENA_PIN_2, -1, -1, -1, true),
  StepperController(STEP_PIN_3, DIR_PIN_3, ENA_PIN_3, -1, -1, -1, true)
};
const int NUM_MOTORS = sizeof(motors) / sizeof(motors[0]);

// --- Helper: check motor status ---
static inline bool anyMotorRunning() {
  for (int i = 0; i < NUM_MOTORS; ++i) {
    if (motors[i].isRunning()) return true;
  }
  return false;
}

// =============================
// TiltSensor class
// =============================
class TiltSensor {
private:
  MPU6050 mpu;
  bool  filtInit  = false;
  float pf = 0.0f, rf = 0.0f;
  float tauSec    = 0.4f;
  unsigned long lastTsMs = 0;

public:
  void begin() {
    Wire.begin();
    Wire.setClock(400000);  
    #if defined(WIRE_HAS_TIMEOUT) || defined(ARDUINO_ARCH_AVR)
      Wire.setWireTimeout(5000, true);
    #elif defined(ARDUINO_ARCH_ESP32)
      Wire.setTimeout(5);
    #endif

    mpu.initialize();
    if (!mpu.testConnection()) {
      Serial.println("MPU6050 connection failed!");
    }
    mpu.setDLPFMode(MPU6050_DLPF_BW_10);
  }

  void setFilterTau(float tau_s) {
    if (tau_s < 0.01f) tau_s = 0.01f;
    if (tau_s > 5.0f)  tau_s = 5.0f;
    tauSec = tau_s;
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
    float rp, rr;
    getRawPitchRoll(rp, rr);
    rp -= pitch_offset;
    rr -= roll_offset;

    unsigned long nowMs = millis();
    float dt = (lastTsMs == 0) ? 0.01f : (nowMs - lastTsMs) / 1000.0f;
    lastTsMs = nowMs;

    float alpha = dt / (tauSec + dt);
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;

    if (!filtInit) { pf = rp; rf = rr; filtInit = true; }
    else           { pf += alpha * (rp - pf); rf += alpha * (rr - rf); }

    pitch = pf; roll = rf;
  }
};

TiltSensor tilt;

// --- Shared tilt state ---
float latestPitch = 0.0f;
float latestRoll  = 0.0f;

// --- Unified tilt updater ---
void updateTilt() {
  static unsigned long lastPrint = 0;

  // Always compute filtered values
  tilt.getPitchRoll(latestPitch, latestRoll);

  // Print once per second
  unsigned long now = millis();
  if (now - lastPrint >= 1000) {
    lastPrint = now;
    Serial.print("TILT ");
    Serial.print(latestPitch, 2);
    Serial.print(" ");
    Serial.println(latestRoll, 2);
  }
}

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

  for (int i = 0; i < NUM_MOTORS; i++) {
    if (motors[i].isRunning()) return;
  }

  float currentPitch = latestPitch;
  float currentRoll  = latestRoll;

  if (abs(currentPitch) < LEVEL_DEAD_ZONE && abs(currentRoll) < LEVEL_DEAD_ZONE) {
    if (!levelingCompletedMessageSent) {
      Serial.println("Levelling has been completed");
      levelingCompletedMessageSent = true;
    }
    return;
  } else {
    levelingCompletedMessageSent = false;
  }

  float pitch_error_rad = -currentPitch * (PI / 180.0f);
  float roll_error_rad  = -currentRoll  * (PI / 180.0f);

  const float a = roll_error_rad;
  const float b = pitch_error_rad;

  static float step_accum[3] = {0,0,0};

  for (int i = 0; i < NUM_MOTORS; ++i) {
    float dh_mm   = a * X_by_motor[i] + b * Y_by_motor[i];   
    float target  = dh_mm * STEPS_PER_MM * KP;               
    step_accum[i] += target;

    long steps = lround(step_accum[i]);                      
    step_accum[i] -= steps;                                  

    steps = constrain(steps, -MAX_STEP_CORR, MAX_STEP_CORR); 
    if (steps != 0) {
      motors[i].moveRelative(steps);
    }
  }
}

// ===================================
// Serial Command Processing
// ===================================
// (unchanged)
void StepperController::processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

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

  if (cmd == "LEVEL_ON")  { startLeveling(); return; }
  if (cmd == "LEVEL_OFF") { stopLeveling();  return; }

  if (isLeveling) {
    stopLeveling();
  }

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

  else if (cmd.startsWith("FILTER_TAU ")) {
    float tau = cmd.substring(11).toFloat();
    tilt.setFilterTau(tau);
    Serial.print("FILTER_TAU_OK ");
    Serial.println(tau, 3);
  }

  else if (cmd.startsWith("MOVE_M ")) {
    int first_space  = cmd.indexOf(' ');
    int second_space = cmd.indexOf(' ', first_space + 1);
    int third_space  = cmd.indexOf(' ', second_space + 1);
    if (first_space > 0 && second_space > 0 && third_space > 0) {
      int motor_index = cmd.substring(first_space + 1, second_space).toInt();
      float value     = cmd.substring(second_space + 1, third_space).toFloat();
      String units    = cmd.substring(third_space + 1);
      long steps;
      if (units.equalsIgnoreCase("mm")) steps = lround(value * STEPS_PER_MM);
      else                              steps = lround(value);
      if (motor_index >= 0 && motor_index < NUM_MOTORS) {
        motors[motor_index].moveRelative(steps);
      } else if (motor_index == NUM_MOTORS || motor_index == 3) {
        for (int i = 0; i < NUM_MOTORS; i++) motors[i].moveRelative(steps);
      }
    }
  }

  else if (cmd.startsWith("MOVE ")) {
    float deg  = cmd.substring(5).toFloat();
    long steps = lround((deg / 360.0f) * STEPS_PER_REV);
    motors[0].moveRelative(steps);
  }

  else if (cmd == "HOME") {
    motors[0].moveToZero();
  } else if (cmd == "DUNK") {
    motors[0].dunk();
  } else if (cmd == "GET_POS") {
    Serial.print("POS ");
    Serial.println(motors[0].position());
  } else if (cmd == "STOP" || cmd == "CANCEL" || cmd == "ESTOP") {
    for (int i = 0; i < NUM_MOTORS; i++) { motors[i].stop(); }
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
      if (!motors[i].isRunning()) { motors[i].disableMotor(); }
    }
    Serial.println("HOLD OFF");
  } else if (cmd == "SET_ZERO") {
    motors[0].setZero();
    Serial.println("HOMED");
    Serial.print("POS ");
    Serial.println(motors[0].position());
  }
}

// =============================
// Arduino entry points
// =============================
void setup() {
  Serial.begin(115200);

  EEPROM.get(0, pitch_offset);
  EEPROM.get(sizeof(float), roll_offset);
  if (isnan(pitch_offset)) pitch_offset = 0.0;
  if (isnan(roll_offset))  roll_offset  = 0.0;

  Serial.println("Stepper ready (TB600) - Multi-Motor with Leveling");
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].begin();
  }
  tilt.begin();
}

void loop() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].update();
  }

  motors[0].handleSerial();

  updateTilt();

  updateLeveling();
}
