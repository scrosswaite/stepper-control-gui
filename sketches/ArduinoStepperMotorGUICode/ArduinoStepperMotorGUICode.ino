#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <ezButton.h>
#include <AccelStepper.h>

// --- Motion Parameters ---
const int  STEPS_PER_REV = 200;
const int  STEPS_PER_MM  = STEPS_PER_REV / 2;   // 2mm lead screw
const int  JOG_STEPS     = STEPS_PER_MM / 2;    // 0.5mm jog
const long DUNK_STEPS    = 50L * STEPS_PER_MM;  // 50mm dunk

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
const int LIMIT1_PIN = 2;
const int LIMIT2_PIN = 3;
const int LED_MOVING_PIN = A3;

// --- Button Pins (Control Motor 1 by default) ---
const int BTN_FWD_PIN   = A0;
const int BTN_REV_PIN   = A1;
const int BTN_HOME_PIN  = A2;
const int BTN_DUNK_PIN  = 7;
const int BTN_CALIB_PIN = A4;

// ===================================
// NEW: Auto-Levelling Configuration
// ===================================
// --- Tuning Parameters (ADJUST THESE) ---
const float KP = 1; // Proportional Gain: Start low (e.g., 0.1) and increase slowly.
const float LEVEL_DEAD_ZONE = 0.25; // Degrees: How close to 0 is "good enough".

// --- Platform Geometry (in mm) ---
const float M1_Y = 220.0;
const float M2_X = 190.0;
const float M23_Y = -110.0;

// --- State Variables ---
bool isLeveling = false;
unsigned long lastLevelingTime = 0;
const unsigned long LEVELING_INTERVAL_MS = 50; // Run leveling logic every 50ms

// Forward declaration for use in StepperController class
void computePitchRoll(float &pitch, float &roll);

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

  void enableMotor()  { if (enaPin > 0) digitalWrite(enaPin, LOW); } // TB6600: LOW = enable
  void disableMotor() { if (enaPin > 0) digitalWrite(enaPin, HIGH); }

public:
  StepperController(int stepPin, int dirPin, int enaPin_,
                    int ledPin_, int limit1Pin_, int limit2Pin_)
    : stepper(AccelStepper::DRIVER, stepPin, dirPin),
      enaPin(enaPin_), ledPin(ledPin_),
      limit1Pin(limit1Pin_), limit2Pin(limit2Pin_) {}

  void begin() {
    if (enaPin > 0) pinMode(enaPin, OUTPUT);
    if (ledPin > 0) pinMode(ledPin, OUTPUT);
    if (limit1Pin > 0) pinMode(limit1Pin, INPUT_PULLUP);
    if (limit2Pin > 0) pinMode(limit2Pin, INPUT_PULLUP);

    // Stepper config
    if (enaPin > 0) digitalWrite(enaPin, HIGH); // disabled initially
    stepper.setMaxSpeed(4000);
    stepper.setAcceleration(500);
    if (enaPin > 0) {
        stepper.setEnablePin(enaPin);
        stepper.setPinsInverted(false, false, true); // invert enable (LOW=on)
    }


    if (ledPin > 0) digitalWrite(ledPin, LOW);
  }

  // ----- High-level actions -----
  void moveRelative(long steps) { stepper.move(steps); }
  void moveToZero()             { stepper.moveTo(0); }
  void jogForward()             { stepper.move(JOG_STEPS); }
  void jogBackward()            { stepper.move(-JOG_STEPS); }
  void dunk()                   { stepper.move(-DUNK_STEPS); }
  void stop()                   { stepper.stop(); }
  long position() const         { return stepper.currentPosition(); }
  void setZero()                { stepper.setCurrentPosition(0); }
  bool isRunning() const        { return stepper.isRunning(); }

  // ----- NEW: Speed control -----
  void setSpeed(float maxSpeed, float acceleration) {
    stepper.setMaxSpeed(maxSpeed);
    stepper.setAcceleration(acceleration);
  }

  // ----- Serial command handling (with internal buffer) -----
  // Forward declaration of the global motors array
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

  // ----- Periodic update (call every loop) -----
  void update() {
    bool running = stepper.isRunning();

    // Transition: started
    if (!wasRunning && running) {
      if (ledPin > 0) digitalWrite(ledPin, HIGH);
      enableMotor();
    }

    // While moving: handle limits
    if (running && limit1Pin > 0 && limit2Pin > 0) {
      bool movingForward = stepper.distanceToGo() > 0;
      if ((movingForward && digitalRead(limit1Pin) == LOW) ||
          (!movingForward && digitalRead(limit2Pin) == LOW)) {
        stepper.stop();
        Serial.println("LIMIT");
      }
    }

    // Transition: stopped
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
    stepper.run(); // motor engine
  }
};

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

  // Make this a public member to be accessible by the leveling function
  void getPitchRoll(float &pitch, float &roll) {
      int16_t ax, ay, az;
      mpu.getAcceleration(&ax, &ay, &az);
      float fAx = ax / 16384.0f;
      float fAy = ay / 16384.0f;
      float fAz = az / 16384.0f;
      pitch = atan2(fAy, sqrt(fAx * fAx + fAz * fAz)) * 180.0f / PI;
      roll  = atan2(-fAx, fAz) * 180.0f / PI;
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

  // Update now takes a reference to the primary motor
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

// =============================
// Single application objects
// =============================

StepperController motors[] = {
  StepperController(STEP_PIN_1, DIR_PIN_1, ENA_PIN_1, LED_MOVING_PIN, LIMIT1_PIN, LIMIT2_PIN),
  StepperController(STEP_PIN_2, DIR_PIN_2, ENA_PIN_2, -1, -1, -1), // -1 for unused LED/limit pins
  StepperController(STEP_PIN_3, DIR_PIN_3, ENA_PIN_3, -1, -1, -1)  // -1 for unused LED/limit pins
};
const int NUM_MOTORS = sizeof(motors) / sizeof(motors[0]);

TiltSensor tilt;
ButtonPanel buttons(BTN_FWD_PIN, BTN_REV_PIN, BTN_HOME_PIN, BTN_DUNK_PIN, BTN_CALIB_PIN);

// ===================================
// NEW: Auto-Levelling Functions
// ===================================
void startLeveling() {
  isLeveling = true;
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

  // Don't interfere with other moves and don't run too fast
  unsigned long now = millis();
  if (now - lastLevelingTime < LEVELING_INTERVAL_MS) return;
  lastLevelingTime = now;

  // Check if any motor is busy with a large command
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (motors[i].isRunning()) return;
  }

  float currentPitch, currentRoll;
  tilt.getPitchRoll(currentPitch, currentRoll);

  // If we are within the dead zone, do nothing.
  if (abs(currentPitch) < LEVEL_DEAD_ZONE && abs(currentRoll) < LEVEL_DEAD_ZONE) {
    return;
  }

  // The "error" is the current angle, as we want to drive it to zero.
  // We need to move in the OPPOSITE direction of the error.
  float pitch_error_rad = -currentPitch * (PI / 180.0);
  float roll_error_rad = -currentRoll * (PI / 180.0);

  // Kinematic Equations: Calculate required height change (dh) in mm for each motor
  float dh1 = M1_Y * pitch_error_rad;
  float dh2 = (M2_X * roll_error_rad) + (M23_Y * pitch_error_rad);
  float dh3 = (-M2_X * roll_error_rad) + (M23_Y * pitch_error_rad);

  // Convert mm to steps and apply Proportional Gain
  long steps1 = lround(dh1 * STEPS_PER_MM * KP);
  long steps2 = lround(dh2 * STEPS_PER_MM * KP);
  long steps3 = lround(dh3 * STEPS_PER_MM * KP);

  // Command the motors to move by the calculated small amount
  motors[0].moveRelative(steps1);
  motors[1].moveRelative(steps2);
  motors[2].moveRelative(steps3);
}


// Process command is now a static member of StepperController to access the global motors array
void StepperController::processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

    // --- NEW: Handle Leveling Commands First ---
  if (cmd == "LEVEL_ON") {
    startLeveling();
    return; // Exit after handling command
  } else if (cmd == "LEVEL_OFF") {
    stopLeveling();
    return; // Exit after handling command
  }

  // If a non-leveling command is received, stop leveling
  if (isLeveling) {
    stopLeveling();
  }

  // --- NEW: Speed Configuration Command ---
  if (cmd.startsWith("CONFIG_SPEED ")) {
    int first_space = cmd.indexOf(' ');
    int second_space = cmd.indexOf(' ', first_space + 1);

    if (first_space > 0 && second_space > 0) {
      float max_speed = cmd.substring(first_space + 1, second_space).toFloat();
      float accel = cmd.substring(second_space + 1).toFloat();
      
      // Apply to all motors
      for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].setSpeed(max_speed, accel);
      }
      Serial.println("SPEED_CONFIG_OK");
    }
  }

  // NEW: Command for specific motor control
  else if (cmd.startsWith("MOVE_M ")) {
    // Expected format: "MOVE_M <motor_index> <value> <units>"
    // e.g., "MOVE_M 1 -100 steps"
    int first_space = cmd.indexOf(' ');
    int second_space = cmd.indexOf(' ', first_space + 1);
    int third_space = cmd.indexOf(' ', second_space + 1);

    if (first_space > 0 && second_space > 0 && third_space > 0) {
      int motor_index = cmd.substring(first_space + 1, second_space).toInt();
      float value = cmd.substring(second_space + 1, third_space).toFloat();
      String units = cmd.substring(third_space + 1);
      
      long steps;
      if (units.equalsIgnoreCase("mm")) {
          steps = lround(value * STEPS_PER_MM);
      } else { // assume steps
          steps = lround(value);
      }

      if (motor_index >= 0 && motor_index < NUM_MOTORS) {
          motors[motor_index].moveRelative(steps);
      } else if (motor_index == NUM_MOTORS || motor_index == 3) { // 3 is sent from UI for "All"
          for (int i = 0; i < NUM_MOTORS; i++) {
              motors[i].moveRelative(steps);
          }
      }
    }
  } else if (cmd.startsWith("MOVE ")) { // ORIGINAL command, controls motor 0
    float deg = cmd.substring(5).toFloat();
    long steps = lround((deg / 360.0f) * STEPS_PER_REV);
    motors[0].moveRelative(steps);

  } else if (cmd == "HOME") { // Controls motor 0
    motors[0].moveToZero();
  } else if (cmd == "DUNK") { // Controls motor 0
    motors[0].dunk();
  } else if (cmd == "GET_POS") { // Gets position of motor 0
    Serial.print("POS ");
    Serial.println(motors[0].position());
  } else if (cmd == "STOP" || cmd == "CANCEL" || cmd == "ESTOP") { // Stops ALL motors
    for(int i = 0; i < NUM_MOTORS; i++) {
        motors[i].stop();
    }
    Serial.println(cmd == "ESTOP" ? "ESTOP" : "STOPPING");
  } else if (cmd == "HOLD ON") { // Engages ALL motors
     for(int i = 0; i < NUM_MOTORS; i++) {
        motors[i].holdTorque = true;
        motors[i].enableMotor();
    }
    Serial.println("HOLD ON");

  } else if (cmd == "HOLD OFF") { // Disengages ALL motors
    for(int i = 0; i < NUM_MOTORS; i++) {
        motors[i].holdTorque = false;
        if (!motors[i].isRunning()) {
            motors[i].disableMotor();
        }
    }
    Serial.println("HOLD OFF");
  } else if (cmd == "SET_ZERO") { // Sets zero for motor 0
    motors[0].setZero();
    Serial.println("HOMED"); // Maintain original response for UI
    Serial.print("POS ");
    Serial.println(motors[0].position());
  }
}

// =============================
// Arduino entry points
// =============================
void setup() {
  Serial.begin(9600);
  Serial.println("Stepper ready (TB6600) - Multi-Motor with Leveling");
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

  // Update all motors
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].update();
  }

  // Update sensor readings for GUI
  tilt.update();

  // NEW: Continuously run the leveling logic if active
  updateLeveling();
}