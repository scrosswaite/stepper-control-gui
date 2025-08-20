#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <ezButton.h>
#include <AccelStepper.h>

// --- Motion Parameters ---
const int STEPS_PER_REV = 200;
const int STEPS_PER_MM  = STEPS_PER_REV / 12;   // 12mm lead screw
const int JOG_STEPS     = STEPS_PER_MM / 2;     // 0.5mm jog
const long DUNK_STEPS   = 50L * STEPS_PER_MM;   // 50mm dunk

// --- Pin Assignments for TB6600 ---
const int STEP_PIN = 9;
const int DIR_PIN  = 8;
const int ENA_PIN  = 10;

// --- Other Pin Assignments ---
const int LIMIT1_PIN = 2;
const int LIMIT2_PIN = 3;
const int LED_MOVING_PIN = A3;

const int BTN_FWD_PIN   = 4;
const int BTN_REV_PIN   = 5;
const int BTN_HOME_PIN  = 6;
const int BTN_DUNK_PIN  = 7;
const int BTN_CALIB_PIN = 12;

// --- Global State ---
unsigned long lastTiltTime = 0;
bool holdTorque = false;
String serialInputBuffer = ""; // Buffer for incoming serial commands

// --- Objects ---
MPU6050 mpu;
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

ezButton buttonFwd(BTN_FWD_PIN);
ezButton buttonRev(BTN_REV_PIN);
ezButton buttonHome(BTN_HOME_PIN);
ezButton buttonDunk(BTN_DUNK_PIN);
ezButton buttonCalib(BTN_CALIB_PIN);


// --- Helper Functions ---
void enableMotor() {
  digitalWrite(ENA_PIN, LOW);
}
void disableMotor() {
  digitalWrite(ENA_PIN, HIGH);
}

// --- Command Processing ---
void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd.startsWith("MOVE ")) {
    float deg = cmd.substring(5).toFloat();
    long steps = lround((deg / 360.0f) * STEPS_PER_REV);
    stepper.move(steps);
  } else if (cmd == "HOME") {
    stepper.moveTo(0);
  } else if (cmd == "DUNK") {
    stepper.move(-DUNK_STEPS);
  } else if (cmd == "GET_POS") {
    Serial.print("POS ");
    Serial.println(stepper.currentPosition());
  } else if (cmd == "STOP" || cmd == "CANCEL" || cmd == "ESTOP") {
    stepper.stop();
    Serial.println(cmd == "ESTOP" ? "ESTOP" : "STOPPING");
  } else if (cmd == "HOLD ON") {
    holdTorque = true;
    enableMotor();
    Serial.println("HOLD ON");
  } else if (cmd == "HOLD OFF") {
    holdTorque = false;
    if (!stepper.isRunning()) {
       disableMotor();
    }
    Serial.println("HOLD OFF");
  } else if (cmd == "SET_ZERO") {
    stepper.setCurrentPosition(0);
    Serial.println("HOMED");
    Serial.print("POS ");
    Serial.println(stepper.currentPosition());
  }
}

// --- NON-BLOCKING Command Handlers ---
void handleSerialCommands() {
  while (Serial.available() > 0) {
    char receivedChar = (char)Serial.read();
    if (receivedChar == '\n' || receivedChar == '\r') {
      processCommand(serialInputBuffer);
      serialInputBuffer = ""; // Clear buffer for next command
    } else {
      serialInputBuffer += receivedChar;
    }
  }
}

void handleButtons() {
  buttonFwd.loop();
  buttonRev.loop();
  buttonHome.loop();
  buttonDunk.loop();
  buttonCalib.loop();

  if (buttonFwd.isPressed())  { stepper.move(JOG_STEPS); }
  if (buttonRev.isPressed())  { stepper.move(-JOG_STEPS); }
  if (buttonHome.isPressed()) { stepper.moveTo(0); }
  if (buttonDunk.isPressed()) { stepper.move(-DUNK_STEPS); }
  if (buttonCalib.isPressed()){ Serial.println("CALIBRATE"); }
}

// --- MPU6050 Functions ---
void computePitchRoll(float &pitch, float &roll) {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  float fAx = ax / 16384.0f;
  float fAy = ay / 16384.0f;
  float fAz = az / 16384.0f;
  pitch = atan2(fAy, sqrt(fAx * fAx + fAz * fAz)) * 180.0f / PI;
  roll  = atan2(-fAx, fAz) * 180.0f / PI;
}

// --- Main Setup ---
void setup() {
  Serial.begin(9600);
  Serial.println("Stepper ready (TB6600)");

  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
  }

  pinMode(ENA_PIN, OUTPUT);
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);
  stepper.setEnablePin(ENA_PIN);
  stepper.setPinsInverted(false, false, true);

  disableMotor();

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
}


// --- Main Loop ---
void loop() {
  // Always check for user input first
  handleButtons();
  handleSerialCommands();

  // State management based on whether the motor is running
  static bool wasRunning = false;
  bool isRunning = stepper.isRunning();

  // Actions to do ONCE when a move STARTS
  if (!wasRunning && isRunning) {
    digitalWrite(LED_MOVING_PIN, HIGH);
    enableMotor();
  }

  // Actions to do continuously WHILE moving
  if (isRunning) {
    bool movingForward = stepper.distanceToGo() > 0;
    if ((movingForward && digitalRead(LIMIT1_PIN) == LOW) || (!movingForward && digitalRead(LIMIT2_PIN) == LOW)) {
      stepper.stop();
      Serial.println("LIMIT");
    }
  }

  // Actions to do ONCE when a move STOPS
  if (wasRunning && !isRunning) {
    digitalWrite(LED_MOVING_PIN, LOW);
    Serial.println("MOVED"); // <<< FIX IS HERE
    Serial.print("POS ");
    Serial.println(stepper.currentPosition());
    if (!holdTorque) {
      disableMotor();
    }
  }
  wasRunning = isRunning; // Update state for next cycle

  // The motor's "engine" - MUST be called every loop
  stepper.run();

  // MPU6050 sensor reading
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