#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <ezButton.h>
#include <AccelStepper.h>

// --- Motion Parameters (constants are fine as globals) ---
const int  STEPS_PER_REV = 200;
const int  STEPS_PER_MM  = STEPS_PER_REV / 2;   // 12mm lead screw
const int  JOG_STEPS     = STEPS_PER_MM / 2;     // 0.5mm jog
const long DUNK_STEPS    = 50L * STEPS_PER_MM;   // 50mm dunk

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

  void enableMotor()  { digitalWrite(enaPin, LOW); }   // TB6600: LOW = enable
  void disableMotor() { digitalWrite(enaPin, HIGH); }

public:
  StepperController(int stepPin, int dirPin, int enaPin_,
                    int ledPin_, int limit1Pin_, int limit2Pin_)
    : stepper(AccelStepper::DRIVER, stepPin, dirPin),
      enaPin(enaPin_), ledPin(ledPin_),
      limit1Pin(limit1Pin_), limit2Pin(limit2Pin_) {}

  void begin() {
    pinMode(enaPin, OUTPUT);
    pinMode(ledPin, OUTPUT);
    pinMode(limit1Pin, INPUT_PULLUP);
    pinMode(limit2Pin, INPUT_PULLUP);

    // Stepper config
    digitalWrite(enaPin, HIGH); // disabled initially
    stepper.setMaxSpeed(4000);
    stepper.setAcceleration(500);
    stepper.setEnablePin(enaPin);
    stepper.setPinsInverted(false, false, true); // invert enable (LOW=on)

    digitalWrite(ledPin, LOW);
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

  // ----- Serial command handling (with internal buffer) -----
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

  // ----- Periodic update (call every loop) -----
  void update() {
    bool running = stepper.isRunning();

    // Transition: started
    if (!wasRunning && running) {
      digitalWrite(ledPin, HIGH);
      enableMotor();
    }

    // While moving: handle limits
    if (running) {
      bool movingForward = stepper.distanceToGo() > 0;
      if ((movingForward && digitalRead(limit1Pin) == LOW) ||
          (!movingForward && digitalRead(limit2Pin) == LOW)) {
        stepper.stop();
        Serial.println("LIMIT");
      }
    }

    // Transition: stopped
    if (wasRunning && !running) {
      digitalWrite(ledPin, LOW);
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
  const unsigned long periodMs = 100;

  void computePitchRoll(float &pitch, float &roll) {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    float fAx = ax / 16384.0f;
    float fAy = ay / 16384.0f;
    float fAz = az / 16384.0f;
    pitch = atan2(fAy, sqrt(fAx * fAx + fAz * fAz)) * 180.0f / PI;
    roll  = atan2(-fAx, fAz) * 180.0f / PI;
  }

public:
  void begin() {
    Wire.begin();
    mpu.initialize();
    if (!mpu.testConnection()) {
      Serial.println("MPU6050 connection failed!");
    }
  }

  void update() {
    unsigned long now = millis();
    if (now - lastRead >= periodMs) {
      lastRead = now;
      float pitch, roll;
      computePitchRoll(pitch, roll);
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
    pinMode(BTN_FWD_PIN,   INPUT_PULLUP);
    pinMode(BTN_REV_PIN,   INPUT_PULLUP);
    pinMode(BTN_HOME_PIN,  INPUT_PULLUP);
    pinMode(BTN_DUNK_PIN,  INPUT_PULLUP);
    pinMode(BTN_CALIB_PIN, INPUT_PULLUP);

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

// =============================
// Single application objects
// =============================
StepperController controller(STEP_PIN, DIR_PIN, ENA_PIN,
                             LED_MOVING_PIN, LIMIT1_PIN, LIMIT2_PIN);
TiltSensor tilt;
ButtonPanel buttons(BTN_FWD_PIN, BTN_REV_PIN, BTN_HOME_PIN, BTN_DUNK_PIN, BTN_CALIB_PIN);

// =============================
// Arduino entry points
// =============================
void setup() {
  Serial.begin(9600);
  Serial.println("Stepper ready (TB6600)");

  controller.begin();
  tilt.begin();
  buttons.begin();
}

void loop() {
  buttons.update(controller);
  controller.handleSerial();
  controller.update();
  tilt.update();
}
