#include <Arduino.h>
#include <ezButton.h>
#include <AccelStepper.h>
#include <EEPROM.h>

// --- MPU-6050 DMP ---
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// ==============================
// Motion / Geometry Parameters
// ==============================
const int STEPS_PER_REV = 200;
// CHANGED: make STEPS_PER_MM runtime-configurable (was const)
volatile long STEPS_PER_MM = STEPS_PER_REV / 2; // assumes 2 mm lead screw by default
const int JOG_STEPS = STEPS_PER_MM / 2;         // 0.5 mm jog
const long DUNK_STEPS = 50L * STEPS_PER_MM;     // 50 mm dunk

// --- TB6600 Pins ---
const int STEP_PIN_1 = 9;
const int DIR_PIN_1  = 8;
const int ENA_PIN_1  = 10;

const int STEP_PIN_2 = 11;
const int DIR_PIN_2  = 12;
const int ENA_PIN_2  = 13;

const int STEP_PIN_3 = 4;
const int DIR_PIN_3  = 5;
const int ENA_PIN_3  = 6;

// --- Other Pins ---
const int LIMIT1_PIN = 2;
const int LIMIT2_PIN = 3;
const int LED_MOVING_PIN = A3;
const int BTN_FWD_PIN = A0;
const int BTN_REV_PIN = A1;
const int BTN_HOME_PIN = A2;
const int BTN_DUNK_PIN = 7;
const int BTN_CALIB_PIN = A4;

// ===================================
// Auto-Levelling Configuration
// ===================================
const float KP = 4.0f; // proportional gain

// Hysteresis (NEW)
const float DEAD_IN_DEG  = 0.10f; // stop when both |pitch| & |roll| < this
const float DEAD_OUT_DEG = 0.25f; // re-engage only if either exceeds this

// Smoothing (NEW)
const float ALPHA = 0.25f; // IIR low-pass; 0..1; higher = less smoothing

// Rate limit / anti-accumulation (NEW)
const unsigned long LEVELING_INTERVAL_MS = 50; // control tick
const long MAX_STEPS_PER_TICK = 200;           // clamp per motor per tick
const long MAX_PENDING_STEPS  = 800;           // skip adding if too much queued

// Triangle geometry (platform)
const float M1_Y  = 220.0f;
const float M2_X  = 190.0f;
const float M23_Y = -110.0f;

bool isLeveling = false;
bool levelingCompletedMessageSent = false;
unsigned long lastLevelingTime = 0;

// --- Timer for TILT streaming to GUI ---
unsigned long lastTiltSendTime = 0;
const unsigned long TILT_SEND_INTERVAL_MS = 1000;

// ===================================
// StepperController
// ===================================
class StepperController {
private:
  AccelStepper stepper;
  const int enaPin;
  const int ledPin;
  const int limit1Pin;
  const int limit2Pin;
  bool wasRunning = false;

  void enableMotor() {
    if (enaPin > 0) digitalWrite(enaPin, LOW);
  }
  void disableMotor() {
    if (enaPin > 0) digitalWrite(enaPin, HIGH);
  }

public:
  bool holdTorque = false;

  StepperController(int stepPin, int dirPin, int enaPin_, int ledPin_, int limit1Pin_, int limit2Pin_)
  : stepper(AccelStepper::DRIVER, stepPin, dirPin),
    enaPin(enaPin_), ledPin(ledPin_), limit1Pin(limit1Pin_), limit2Pin(limit2Pin_) {}

  void begin() {
    if (enaPin  > 0) pinMode(enaPin, OUTPUT);
    if (ledPin  > 0) pinMode(ledPin, OUTPUT);
    if (limit1Pin > 0) pinMode(limit1Pin, INPUT_PULLUP);
    if (limit2Pin > 0) pinMode(limit2Pin, INPUT_PULLUP);
    if (enaPin  > 0) digitalWrite(enaPin, HIGH);

    stepper.setMaxSpeed(4000);
    stepper.setAcceleration(500);
    if (enaPin > 0) {
      stepper.setEnablePin(enaPin);
      stepper.setPinsInverted(false, false, true);
    }
    if (ledPin > 0) digitalWrite(ledPin, LOW);
  }

  void moveRelative(long steps) { stepper.move(steps); }
  void moveToZero() { stepper.moveTo(0); }
  void jogForward() { stepper.move(JOG_STEPS); }
  void jogBackward(){ stepper.move(-JOG_STEPS); }
  void dunk()       { stepper.move(-DUNK_STEPS); }
  void stop()       { stepper.stop(); }
  long position() const { return stepper.currentPosition(); }
  void setZero()    { stepper.setCurrentPosition(0); }
  bool isRunning() const { return stepper.isRunning(); }
  void setSpeed(float maxSpeed, float acceleration) {
    stepper.setMaxSpeed(maxSpeed);
    stepper.setAcceleration(acceleration);
  }
  long pending() const { return stepper.distanceToGo(); } // NEW

  static void processCommand(String cmd);

  // Serial handler per original design
  String serialInputBuffer;
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
      Serial.println("MOVED");
      Serial.print("POS ");
      Serial.println(stepper.currentPosition());
      if (!holdTorque) {
        disableMotor();
      }
    }

    wasRunning = running;
    stepper.run();
  }
};

// ===================================
// TiltSensor
// ===================================
class TiltSensor {
private:
  MPU6050 mpu;
  bool dmpReady = false;
  uint8_t devStatus = 0;
  uint16_t packetSize = 0;
  uint8_t fifoBuffer[64];
  Quaternion q;
  VectorFloat gravity;
  float ypr[3];

public:
  void begin() {
    Wire.begin();
    Wire.setClock(400000);
    mpu.initialize();

    if (!mpu.testConnection()) {
      Serial.println("MPU6050 connection failed!");
      return;
    }

    devStatus = mpu.dmpInitialize();
    // Offsets (as in your code)
    mpu.setXAccelOffset(-2977);
    mpu.setYAccelOffset(5198);
    mpu.setZAccelOffset(792);
    mpu.setXGyroOffset(45);
    mpu.setYGyroOffset(-25);
    mpu.setZGyroOffset(-11);

    if (devStatus == 0) {
      mpu.setDMPEnabled(true);
      dmpReady = true;
      packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
    }
  }

  // NEW: return bool so caller can skip tick if no fresh data
  bool readPitchRoll(float &pitch, float &roll) {
    if (!dmpReady) return false;
    if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) return false;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // FIX #1 from your original: swapped axes to match orientation
    roll  = ypr[1] * 180.0f / M_PI;  // was pitch
    pitch = ypr[2] * 180.0f / M_PI;  // was roll
    return true;
  }

  // Backward-compat wrapper (unused now, but harmless)
  void getPitchRoll(float &pitch, float &roll) {
    readPitchRoll(pitch, roll);
  }
};

// ===================================
// Buttons
// ===================================
class ButtonPanel {
private:
  ezButton btnFwd, btnRev, btnHome, btnDunk, btnCalib;
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

    if (btnFwd.isPressed()) ctrl.jogForward();
    if (btnRev.isPressed()) ctrl.jogBackward();
    if (btnHome.isPressed()) ctrl.moveToZero();
    if (btnDunk.isPressed()) ctrl.dunk();
    if (btnCalib.isPressed()) Serial.println("CALIBRATE_ACCEL");
  }
};

// ===================================
// Instances
// ===================================
StepperController motors[] = {
  StepperController(STEP_PIN_1, DIR_PIN_1, ENA_PIN_1, LED_MOVING_PIN, LIMIT1_PIN, LIMIT2_PIN),
  StepperController(STEP_PIN_2, DIR_PIN_2, ENA_PIN_2, -1, -1, -1),
  StepperController(STEP_PIN_3, DIR_PIN_3, ENA_PIN_3, -1, -1, -1)
};
const int NUM_MOTORS = sizeof(motors) / sizeof(motors[0]);

TiltSensor tilt;
ButtonPanel buttons(BTN_FWD_PIN, BTN_REV_PIN, BTN_HOME_PIN, BTN_DUNK_PIN, BTN_CALIB_PIN);

// ===================================
// Levelling helpers
// ===================================
void startLeveling() {
  isLeveling = true;
  levelingCompletedMessageSent = false;
  Serial.println("LEVELING_ON");
}

void stopLeveling() {
  isLeveling = false;
  for (int i = 0; i < NUM_MOTORS; i++) motors[i].stop();
  Serial.println("LEVELING_OFF");
}

// Filter state (NEW)
static float filtPitch = 0.0f, filtRoll = 0.0f;
static bool  filtInit = false;
// Hysteresis latch (NEW)
static bool inDead = false;

void updateLeveling() {
  if (!isLeveling) return;

  unsigned long now = millis();
  if (now - lastLevelingTime < LEVELING_INTERVAL_MS) return;
  lastLevelingTime = now;

  // 1) Get fresh sensor data; if none, skip this tick
  float rawPitch = 0.0f, rawRoll = 0.0f;
  if (!tilt.readPitchRoll(rawPitch, rawRoll)) {
    return;
  }

  // 2) Low-pass filter for smoother control
  if (!filtInit) {
    filtPitch = rawPitch;
    filtRoll  = rawRoll;
    filtInit  = true;
  } else {
    filtPitch = ALPHA * rawPitch + (1.0f - ALPHA) * filtPitch;
    filtRoll  = ALPHA * rawRoll  + (1.0f - ALPHA) * filtRoll;
  }

  float ap = fabs(filtPitch);
  float ar = fabs(filtRoll);

  // 3) Hysteresis dead-zone
  if (!inDead && ap < DEAD_IN_DEG && ar < DEAD_IN_DEG) {
    inDead = true;
    if (!levelingCompletedMessageSent) {
      Serial.println("Levelling has been completed");
      levelingCompletedMessageSent = true;
    }
    stopLeveling(); // send LEVELING_OFF so GUI unlocks consistently
    return;
  }
  if (inDead && (ap > DEAD_OUT_DEG || ar > DEAD_OUT_DEG)) {
    inDead = false; // leave dead-zone -> resume control
  }
  if (inDead) {
    return; // remain idle inside dead-zone
  }

  // 4) Compute height deltas from filtered errors
  const float pitch_error_rad = -filtPitch * (PI / 180.0f);
  const float roll_error_rad  = -filtRoll  * (PI / 180.0f);

  float dh1 = M1_Y * pitch_error_rad;
  float dh2 = (M2_X * roll_error_rad) + (M23_Y * pitch_error_rad);
  float dh3 = (M2_X * roll_error_rad) - (M23_Y * pitch_error_rad);

  long s1 = lround(dh1 * STEPS_PER_MM * KP);
  long s2 = lround(dh2 * STEPS_PER_MM * KP);
  long s3 = lround(dh3 * STEPS_PER_MM * KP);

  // 5) Clamp per-tick increments (anti-accumulation)
  s1 = constrain(s1, -MAX_STEPS_PER_TICK, +MAX_STEPS_PER_TICK);
  s2 = constrain(s2, -MAX_STEPS_PER_TICK, +MAX_STEPS_PER_TICK);
  s3 = constrain(s3, -MAX_STEPS_PER_TICK, +MAX_STEPS_PER_TICK);

  // 6) Skip adding if too much is already queued (optional guard)
  if (labs(motors[0].pending()) < MAX_PENDING_STEPS) motors[0].moveRelative(s1);
  if (labs(motors[1].pending()) < MAX_PENDING_STEPS) motors[1].moveRelative(s2);
  if (labs(motors[2].pending()) < MAX_PENDING_STEPS) motors[2].moveRelative(s3);
}

// ===================================
// Command processing
// ===================================
void StepperController::processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "CALIBRATE_ACCEL") {
    Serial.println("DMP uses pre-set offsets. Manual calibration command ignored.");
    return;
  }

  if (cmd == "LEVEL_ON") { startLeveling(); return; }
  if (cmd == "LEVEL_OFF"){ stopLeveling();  return; }

  if (isLeveling) stopLeveling();

  if (cmd.startsWith("CONFIG_SPEED ")) {
    int first_space  = cmd.indexOf(' ');
    int second_space = cmd.indexOf(' ', first_space + 1);
    if (first_space > 0 && second_space > 0) {
      float max_speed = cmd.substring(first_space + 1, second_space).toFloat();
      float accel     = cmd.substring(second_space + 1).toFloat();
      for (int i = 0; i < NUM_MOTORS; i++) motors[i].setSpeed(max_speed, accel);
      Serial.println("SPEED_CONFIG_OK");
    }
  }
  else if (cmd.startsWith("CONFIG_STEPS_PER_MM ")) { // NEW optional config
    float spm = cmd.substring(String("CONFIG_STEPS_PER_MM ").length()).toFloat();
    if (spm > 0.0f && spm < 100000.0f) {
      STEPS_PER_MM = lround(spm);
      Serial.println("STEPS_PER_MM_OK");
    } else {
      Serial.println("STEPS_PER_MM_INVALID");
    }
  }
  else if (cmd.startsWith("MOVE_M ")) {
    int first_space  = cmd.indexOf(' ');
    int second_space = cmd.indexOf(' ', first_space + 1);
    int third_space  = cmd.indexOf(' ', second_space + 1);
    if (first_space > 0 && second_space > 0 && third_space > 0) {
      int motor_index = cmd.substring(first_space + 1, second_space).toInt();
      float value     = cmd.substring(second_space + 1, third_space).toFloat();
      String units    = cmd.substring(third_space + 1);

      long steps = units.equalsIgnoreCase("mm") ? lround(value * STEPS_PER_MM) : lround(value);
      if (motor_index >= 0 && motor_index < NUM_MOTORS) {
        motors[motor_index].moveRelative(steps);
      } else if (motor_index == NUM_MOTORS || motor_index == 3) {
        for (int i = 0; i < NUM_MOTORS; i++) motors[i].moveRelative(steps);
      }
    }
  }
  else if (cmd.startsWith("MOVE ")) {
    float deg = cmd.substring(5).toFloat();
    long steps = lround((deg / 360.0f) * STEPS_PER_REV);
    motors[0].moveRelative(steps);
  }
  else if (cmd == "HOME") {
    motors[0].moveToZero();
  }
  else if (cmd == "DUNK") {
    motors[0].dunk();
  }
  else if (cmd == "GET_POS") {
    Serial.print("POS ");
    Serial.println(motors[0].position());
  }
  else if (cmd == "STOP" || cmd == "CANCEL" || cmd == "ESTOP") {
    for (int i = 0; i < NUM_MOTORS; i++) motors[i].stop();
    Serial.println(cmd == "ESTOP" ? "ESTOP" : "STOPPING");
  }
  else if (cmd == "HOLD ON") {
    for (int i = 0; i < NUM_MOTORS; i++) {
      motors[i].holdTorque = true;
      motors[i].update(); // ensure enable state
    }
    Serial.println("HOLD ON");
  }
  else if (cmd == "HOLD OFF") {
    for (int i = 0; i < NUM_MOTORS; i++) {
      motors[i].holdTorque = false;
      if (!motors[i].isRunning()) ; // disabled in update() end
    }
    Serial.println("HOLD OFF");
  }
  else if (cmd == "SET_ZERO") {
    motors[0].setZero();
    Serial.println("HOMED");
    Serial.print("POS ");
    Serial.println(motors[0].position());
  }
}

// ===================================
// Arduino setup/loop
// ===================================
void setup() {
  Serial.begin(115200);
  Serial.println("Stepper ready - Multi-Motor with DMP Leveling (v3)");

  for (int i = 0; i < NUM_MOTORS; i++) motors[i].begin();
  tilt.begin();
  buttons.begin();
}

void loop() {
  motors[0].handleSerial();   // single serial ingress point (as before)
  buttons.update(motors[0]);  // panel actions

  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].update();
  }

  // Periodic TILT streaming to GUI
  unsigned long now = millis();
  if (now - lastTiltSendTime >= TILT_SEND_INTERVAL_MS) {
    lastTiltSendTime = now;
    float p, r;
    if (tilt.readPitchRoll(p, r)) {
      Serial.print("TILT ");
      Serial.print(p, 2);
      Serial.print(" ");
      Serial.println(r, 2);
    }
  }

  updateLeveling();
}
