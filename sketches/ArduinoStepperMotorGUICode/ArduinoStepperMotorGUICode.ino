#include <Arduino.h>
#include <ezButton.h>
#include <AccelStepper.h>
#include <EEPROM.h>

// --- New Libraries for MPU-6050 DMP ---
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// --- Motion Parameters ---
const int  STEPS_PER_REV = 200;
const int  STEPS_PER_MM  = STEPS_PER_REV / 2; // 2mm lead screw
const int  JOG_STEPS     = STEPS_PER_MM / 2; // 0.5mm jog
const long DUNK_STEPS    = 50L * STEPS_PER_MM; // 50mm dunk

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
const int BTN_FWD_PIN   = A0;
const int BTN_REV_PIN   = A1;
const int BTN_HOME_PIN  = A2;
const int BTN_DUNK_PIN  = 7;
const int BTN_CALIB_PIN = A4;

// ===================================
// Auto-Levelling Configuration
// ===================================
const float KP = 1;
const float LEVEL_DEAD_ZONE = 0.1;
const float M1_Y = 220.0;
const float M2_X = 190.0;
const float M23_Y = -110.0;
bool isLeveling = false;
bool levelingCompletedMessageSent = false;
unsigned long lastLevelingTime = 0;
const unsigned long LEVELING_INTERVAL_MS = 50;

// --- Timer variables for sending tilt data to GUI ---
unsigned long lastTiltSendTime = 0;
const unsigned long TILT_SEND_INTERVAL_MS = 1000; // Send data every 1 second

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
  bool holdTorque = false;
  bool wasRunning = false;
  String serialInputBuffer;
  void enableMotor()  { if (enaPin > 0) digitalWrite(enaPin, LOW); }
  void disableMotor() { if (enaPin > 0) digitalWrite(enaPin, HIGH); }

public:
  StepperController(int stepPin, int dirPin, int enaPin_, int ledPin_, int limit1Pin_, int limit2Pin_)
    : stepper(AccelStepper::DRIVER, stepPin, dirPin), enaPin(enaPin_), ledPin(ledPin_), limit1Pin(limit1Pin_), limit2Pin(limit2Pin_) {}

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
        stepper.setPinsInverted(false, false, true);
    }
    if (ledPin > 0) digitalWrite(ledPin, LOW);
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
      if ((movingForward && digitalRead(limit1Pin) == LOW) || (!movingForward && digitalRead(limit2Pin) == LOW)) {
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

class TiltSensor {
private:
  MPU6050 mpu;
  bool dmpReady = false;
  uint8_t devStatus;
  uint16_t packetSize;
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

  void getPitchRoll(float &pitch, float &roll) {
      if (!dmpReady) return;
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          pitch = ypr[1] * 180.0f / M_PI;
          roll  = ypr[2] * 180.0f / M_PI;
      }
  }
};

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
    btnFwd.loop(); btnRev.loop(); btnHome.loop(); btnDunk.loop(); btnCalib.loop();
    if (btnFwd.isPressed())   ctrl.jogForward();
    if (btnRev.isPressed())   ctrl.jogBackward();
    if (btnHome.isPressed())  ctrl.moveToZero();
    if (btnDunk.isPressed())  ctrl.dunk();
    if (btnCalib.isPressed()) Serial.println("CALIBRATE_ACCEL");
  }
};

StepperController motors[] = {
  StepperController(STEP_PIN_1, DIR_PIN_1, ENA_PIN_1, LED_MOVING_PIN, LIMIT1_PIN, LIMIT2_PIN),
  StepperController(STEP_PIN_2, DIR_PIN_2, ENA_PIN_2, -1, -1, -1),
  StepperController(STEP_PIN_3, DIR_PIN_3, ENA_PIN_3, -1, -1, -1)
};
const int NUM_MOTORS = sizeof(motors) / sizeof(motors[0]);

TiltSensor tilt;
ButtonPanel buttons(BTN_FWD_PIN, BTN_REV_PIN, BTN_HOME_PIN, BTN_DUNK_PIN, BTN_CALIB_PIN);

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

void updateLeveling() {
  if (!isLeveling) return;
  unsigned long now = millis();
  if (now - lastLevelingTime < LEVELING_INTERVAL_MS) return;
  lastLevelingTime = now;
  for (int i = 0; i < NUM_MOTORS; i++) if (motors[i].isRunning()) return;

  float currentPitch, currentRoll;
  tilt.getPitchRoll(currentPitch, currentRoll);

  if (abs(currentPitch) < LEVEL_DEAD_ZONE && abs(currentRoll) < LEVEL_DEAD_ZONE) {
    if (!levelingCompletedMessageSent) {
      Serial.println("Levelling has been completed");
      levelingCompletedMessageSent = true;
    }
    return;
  } else {
    levelingCompletedMessageSent = false;
  }

  float pitch_error_rad = -currentPitch * (PI / 180.0);
  float roll_error_rad = -currentRoll * (PI / 180.0);
  float dh1 = M1_Y * pitch_error_rad;
  float dh2 = (M2_X * roll_error_rad) + (M23_Y * pitch_error_rad);
  float dh3 = (-M2_X * roll_error_rad) + (M23_Y * pitch_error_rad);
  motors[0].moveRelative(lround(dh1 * STEPS_PER_MM * KP));
  motors[1].moveRelative(lround(dh2 * STEPS_PER_MM * KP));
  motors[2].moveRelative(lround(dh3 * STEPS_PER_MM * KP));
}

void StepperController::processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  if (cmd == "CALIBRATE_ACCEL") {
    Serial.println("DMP uses pre-set offsets. Manual calibration command ignored.");
    return;
  }
  if (cmd == "LEVEL_ON") { startLeveling(); return; }
  if (cmd == "LEVEL_OFF") { stopLeveling(); return; }
  if (isLeveling) stopLeveling();

  if (cmd.startsWith("CONFIG_SPEED ")) {
    int first_space = cmd.indexOf(' ');
    int second_space = cmd.indexOf(' ', first_space + 1);
    if (first_space > 0 && second_space > 0) {
      float max_speed = cmd.substring(first_space + 1, second_space).toFloat();
      float accel = cmd.substring(second_space + 1).toFloat();
      for (int i = 0; i < NUM_MOTORS; i++) motors[i].setSpeed(max_speed, accel);
      Serial.println("SPEED_CONFIG_OK");
    }
  } else if (cmd.startsWith("MOVE_M ")) {
    int first_space = cmd.indexOf(' ');
    int second_space = cmd.indexOf(' ', first_space + 1);
    int third_space = cmd.indexOf(' ', second_space + 1);
    if (first_space > 0 && second_space > 0 && third_space > 0) {
      int motor_index = cmd.substring(first_space + 1, second_space).toInt();
      float value = cmd.substring(second_space + 1, third_space).toFloat();
      String units = cmd.substring(third_space + 1);
      long steps = units.equalsIgnoreCase("mm") ? lround(value * STEPS_PER_MM) : lround(value);
      if (motor_index >= 0 && motor_index < NUM_MOTORS) motors[motor_index].moveRelative(steps);
      else if (motor_index == NUM_MOTORS || motor_index == 3) for (int i = 0; i < NUM_MOTORS; i++) motors[i].moveRelative(steps);
    }
  } else if (cmd.startsWith("MOVE ")) { motors[0].moveRelative(lround((cmd.substring(5).toFloat() / 360.0f) * STEPS_PER_REV));
  } else if (cmd == "HOME") { motors[0].moveToZero();
  } else if (cmd == "DUNK") { motors[0].dunk();
  } else if (cmd == "GET_POS") { Serial.print("POS "); Serial.println(motors[0].position());
  } else if (cmd == "STOP" || cmd == "CANCEL" || cmd == "ESTOP") { for(int i=0; i<NUM_MOTORS; i++) motors[i].stop(); Serial.println(cmd == "ESTOP" ? "ESTOP" : "STOPPING");
  } else if (cmd == "HOLD ON") { for(int i=0; i<NUM_MOTORS; i++) { motors[i].holdTorque = true; motors[i].enableMotor(); } Serial.println("HOLD ON");
  } else if (cmd == "HOLD OFF") { for(int i=0; i<NUM_MOTORS; i++) { motors[i].holdTorque = false; if (!motors[i].isRunning()) motors[i].disableMotor(); } Serial.println("HOLD OFF");
  } else if (cmd == "SET_ZERO") { motors[0].setZero(); Serial.println("HOMED"); Serial.print("POS "); Serial.println(motors[0].position());
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Stepper ready - Multi-Motor with DMP Leveling (v2)");
  for (int i = 0; i < NUM_MOTORS; i++) motors[i].begin();
  tilt.begin();
  buttons.begin();
}

void loop() {
  // === CORRECTED LOOP ORDER ===

  // 1. First, always check for and process incoming commands from the GUI
  motors[0].handleSerial();

  // 2. Update the button states
  buttons.update(motors[0]);

  // 3. Run the engine for all motors
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].update();
  }

  // 4. Send TILT data to GUI periodically (so it doesn't interfere with command listening)
  unsigned long now = millis();
  if (now - lastTiltSendTime >= TILT_SEND_INTERVAL_MS) {
    lastTiltSendTime = now;
    float currentPitch, currentRoll;
    tilt.getPitchRoll(currentPitch, currentRoll);
    Serial.print("TILT ");
    Serial.print(currentPitch, 2);
    Serial.print(" ");
    Serial.println(currentRoll, 2);
  }

  // 5. Run the auto-leveling logic
  updateLeveling();
}