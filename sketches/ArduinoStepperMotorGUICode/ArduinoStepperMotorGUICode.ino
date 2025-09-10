#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <ezButton.h>
#include <AccelStepper.h>
#include <EEPROM.h>

// ==============================
// ----- User Configuration -----
// ==============================
const float LEAD_MM        = 2.0f;   // screw lead (mm/rev)
const int   MICROSTEPS     = 16;     // set to match TB6600 DIP switches
const int   STEPS_PER_REV  = 200;    // full steps per rev
const float STEPS_PER_MM   = (STEPS_PER_REV * MICROSTEPS) / LEAD_MM;

const float JOG_MM         = 0.5f;
const float DUNK_MM        = 50.0f;

const long  JOG_STEPS      = lround(JOG_MM  * STEPS_PER_MM);
const long  DUNK_STEPS     = lround(DUNK_MM * STEPS_PER_MM);

// -----------------------------
// Geometry (mm) relative to plate origin
// Motor 1 at (0, M1_Y), Motor 2 at (+M2_X, M23_Y), Motor 3 at (-M2_X, M23_Y)
const float M1_Y  = 220.0f;
const float M2_X  = 190.0f;
const float M23_Y = -110.0f;

// Leveling loop timing & control
const unsigned long LEVELING_INTERVAL_MS = 25;     // control period (~40 Hz)
const float LEVEL_DEAD_ZONE_DEG = 0.10f;           // stop band
const float K_V_MM_PER_RAD = 80.0f;                // velocity gain (mm/s per rad)

// Motor speed/accel limits (in steps/s)
const float MAX_SPEED_STEPS = 3000.0f;             // must be <= setMaxSpeed()
const float ACCEL_STEPS     = 2000.0f;             // ramp for AccelStepper

// ==============================
// -------- Pin Assignments -----
// ==============================
// TB6600 #1
const int STEP_PIN_1 = 9;
const int DIR_PIN_1  = 8;
const int ENA_PIN_1  = 10;
// TB6600 #2
const int STEP_PIN_2 = 11;
const int DIR_PIN_2  = 12;
const int ENA_PIN_2  = 13;
// TB6600 #3
const int STEP_PIN_3 = 4;
const int DIR_PIN_3  = 5;
const int ENA_PIN_3  = 6;

// Limit switches (if used; active LOW)
const int LIMIT1_PIN = 2;
const int LIMIT2_PIN = 3;

// UI
const int LED_MOVING_PIN = A3;

// Buttons (jog/home/dunk + calibration passthrough)
const int BTN_FWD_PIN   = A0;
const int BTN_REV_PIN   = A1;
const int BTN_HOME_PIN  = A2;
const int BTN_DUNK_PIN  = 7;
const int BTN_CALIB_PIN = A4;

// ==============================
// --------- Globals ------------
// ==============================
float pitch_offset = 0.0f;
float roll_offset  = 0.0f;

bool   isLeveling = false;
bool   levelMsgSent = false;
unsigned long lastLevelingTime = 0;

// Forward decl
void startLeveling();
void stopLeveling();
void updateLeveling();

// ==============================
// ---------- IMU ---------------
// ==============================
class TiltSensor {
  MPU6050 mpu;
  uint32_t last_ts_ms = 0;
  // complementary filter state
  float p_deg = 0.0f, r_deg = 0.0f;

public:
  void begin() {
    Wire.begin();
    Wire.setClock(400000);                // Fast I2C
    mpu.initialize();
    // ranges + low-pass filter
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setFullScaleGyroRange (MPU6050_GYRO_FS_250);
    mpu.setDLPFMode(4);                  // ~21 Hz; try 3..4

    if (!mpu.testConnection()) {
      Serial.println("MPU6050 connection failed!");
    }
    last_ts_ms = millis();
  }

  // raw accel-only pitch/roll (deg) — used for calibration
  void getRawPitchRoll(float &raw_pitch, float &raw_roll) {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax,&ay,&az);
    float fAx = ax / 16384.0f;
    float fAy = ay / 16384.0f;
    float fAz = az / 16384.0f;
    raw_pitch = atan2f(fAy, sqrtf(fAx*fAx + fAz*fAz)) * 180.0f/PI;
    raw_roll  = atan2f(-fAx, fAz) * 180.0f/PI;
  }

  // filtered pitch/roll (deg), gravity-referenced
  void getPitchRoll(float &pitch, float &roll) {
    uint32_t now = millis();
    float dt = (now - last_ts_ms) * 0.001f;
    if (dt <= 0) dt = 0.001f;
    last_ts_ms = now;

    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);

    // accel angles (deg)
    float aPitch = atan2f((float)ay, sqrtf((float)ax*ax + (float)az*az)) * 180.0f/PI;
    float aRoll  = atan2f(-(float)ax, (float)az) * 180.0f/PI;

    // gyro deg/s at 250 dps full-scale (131 LSB/deg/s)
    float gRoll  =  gx / 131.0f;
    float gPitch =  gy / 131.0f;

    // integrate gyro
    p_deg += gPitch * dt;
    r_deg += gRoll  * dt;

    // complementary fuse
    const float ALPHA = 0.98f;
    p_deg = ALPHA * p_deg + (1.0f - ALPHA) * aPitch;
    r_deg = ALPHA * r_deg + (1.0f - ALPHA) * aRoll;

    pitch = p_deg - pitch_offset;
    roll  = r_deg - roll_offset;
  }

  // telemetry (only when idle)
  void pushTelemetry() {
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint < 1000) return;
    lastPrint = millis();
    float p, r;
    getPitchRoll(p, r);
    Serial.print("TILT ");
    Serial.print(p, 2);
    Serial.print(" ");
    Serial.println(r, 2);
  }
};

TiltSensor tilt;

// ==============================
// ------- Stepper wrapper ------
// ==============================
class StepperController {
public:
  AccelStepper stepper;
  const int enaPin, ledPin, limit1Pin, limit2Pin;
  bool invertDir;
  bool holdTorque = false;

  // leveling velocity command (steps/s) and accumulator
  float v_cmd = 0.0f;
  float stepAccum = 0.0f;

private:
  bool wasRunning = false;
  String serialBuf;

  void enableMotor()  { if (enaPin > 0) digitalWrite(enaPin, LOW);  }
  void disableMotor() { if (enaPin > 0) digitalWrite(enaPin, HIGH); }

public:
  StepperController(int stepPin, int dirPin, int enaPin_,
                    int ledPin_, int limit1Pin_, int limit2Pin_,
                    bool invertDir_=false)
  : stepper(AccelStepper::DRIVER, stepPin, dirPin),
    enaPin(enaPin_), ledPin(ledPin_),
    limit1Pin(limit1Pin_), limit2Pin(limit2Pin_),
    invertDir(invertDir_) {}

  void begin() {
    if (enaPin   > 0) pinMode(enaPin,   OUTPUT);
    if (ledPin   > 0) pinMode(ledPin,   OUTPUT);
    if (limit1Pin> 0) pinMode(limit1Pin,INPUT_PULLUP);
    if (limit2Pin> 0) pinMode(limit2Pin,INPUT_PULLUP);
    if (enaPin > 0) digitalWrite(enaPin, HIGH); // disabled initially

    stepper.setMaxSpeed(MAX_SPEED_STEPS);
    stepper.setAcceleration(ACCEL_STEPS);
    if (enaPin > 0) {
      stepper.setEnablePin(enaPin);
      stepper.setPinsInverted(invertDir, false, true); // DIR invert, STEP normal, EN active-low
    }
    if (ledPin > 0) digitalWrite(ledPin, LOW);
  }

  void invertDirection(bool inv) {
    invertDir = inv;
    stepper.setPinsInverted(invertDir, false, true);
  }

  // high-level actions
  void moveRelative(long steps) { stepper.move(steps); }
  void moveToZero()             { stepper.moveTo(0); }
  void jogForward()             { stepper.move(JOG_STEPS); }
  void jogBackward()            { stepper.move(-JOG_STEPS); }
  void dunk()                   { stepper.move(-DUNK_STEPS); }
  void stop()                   { stepper.stop(); }
  long position() const         { return stepper.currentPosition(); }
  void setZero()                { stepper.setCurrentPosition(0); }
  bool isRunning() const        { return stepper.isRunning(); }

  void setSpeedLimits(float maxSpeed, float accel) {
    stepper.setMaxSpeed(maxSpeed);
    stepper.setAcceleration(accel);
  }

  void setLevelingVelocity(float v_steps_per_s) { v_cmd = v_steps_per_s; }

  void handleSerial(); // declared below

  void update(int idx, float dt, bool levelingActive) {
    bool running = stepper.isRunning();

    if (!wasRunning && running) {
      if (ledPin > 0) digitalWrite(ledPin, HIGH);
      enableMotor();
    }

    // limit switches
    if (running && limit1Pin>0 && limit2Pin>0) {
      bool movingForward = stepper.distanceToGo() > 0;
      if ((movingForward && digitalRead(limit1Pin)==LOW) ||
          (!movingForward && digitalRead(limit2Pin)==LOW)) {
        stepper.stop();
        Serial.print("LIMIT "); Serial.println(idx);
      }
    }

    // Leveling: convert velocity to tiny position increments (uses accel limiting)
    if (levelingActive) {
      stepAccum += v_cmd * dt;                 // steps this tick
      long inc = (long)stepAccum;
      stepAccum -= inc;
      if (inc != 0L) stepper.move(inc);
      stepper.run();
    } else {
      stepper.run();
    }

    // on stop
    if (wasRunning && !stepper.isRunning()) {
      if (ledPin > 0) digitalWrite(ledPin, LOW);
      Serial.print("MOVED "); Serial.println(idx);
      Serial.print("POS ");   Serial.print(idx); Serial.print(" ");
      Serial.println(stepper.currentPosition());
      if (!holdTorque) disableMotor();
    }

    wasRunning = stepper.isRunning();
  }

  friend void startLeveling();
  friend void stopLeveling();
  friend void updateLeveling();
};

StepperController motors[] = {
  // #1 normal, #2/#3 inverted (adjust if needed)
  StepperController(STEP_PIN_1, DIR_PIN_1, ENA_PIN_1, LED_MOVING_PIN, LIMIT1_PIN, LIMIT2_PIN, false),
  StepperController(STEP_PIN_2, DIR_PIN_2, ENA_PIN_2, -1, -1, -1, true),
  StepperController(STEP_PIN_3, DIR_PIN_3, ENA_PIN_3, -1, -1, -1, true)
};
const int NUM_MOTORS = sizeof(motors)/sizeof(motors[0]);

// ==============================
// -------- Buttons -------------
// ==============================
class ButtonPanel {
  ezButton btnFwd, btnRev, btnHome, btnDunk, btnCalib;
public:
  ButtonPanel(int fwd,int rev,int home,int dunk,int calib)
  : btnFwd(fwd),btnRev(rev),btnHome(home),btnDunk(dunk),btnCalib(calib) {}

  void begin() {
    btnFwd.setDebounceTime(50);
    btnRev.setDebounceTime(50);
    btnHome.setDebounceTime(50);
    btnDunk.setDebounceTime(50);
    btnCalib.setDebounceTime(50);
  }

  void update(StepperController &ctrl){
    btnFwd.loop(); btnRev.loop(); btnHome.loop(); btnDunk.loop(); btnCalib.loop();
    if (btnFwd.isPressed())  ctrl.jogForward();
    if (btnRev.isPressed())  ctrl.jogBackward();
    if (btnHome.isPressed()) ctrl.moveToZero();
    if (btnDunk.isPressed()) ctrl.dunk();
    if (btnCalib.isPressed()) Serial.println("CALIBRATE");
  }
} buttons(BTN_FWD_PIN, BTN_REV_PIN, BTN_HOME_PIN, BTN_DUNK_PIN, BTN_CALIB_PIN);

// ==============================
// ------- Leveling logic -------
// ==============================
void startLeveling() {
  isLeveling = true;
  levelMsgSent = false;
  for (int i=0;i<NUM_MOTORS;i++) {
    if (motors[i].enaPin>0) digitalWrite(motors[i].enaPin, LOW);
    motors[i].v_cmd = 0.0f;
    motors[i].stepAccum = 0.0f;
  }
  lastLevelingTime = millis();
  Serial.println("LEVELING_ON");
}

void stopLeveling() {
  isLeveling = false;
  for (int i=0;i<NUM_MOTORS;i++) {
    motors[i].v_cmd = 0.0f;
    motors[i].stepAccum = 0.0f;
    motors[i].stop();
  }
  Serial.println("LEVELING_OFF");
}

void updateLeveling() {
  if (!isLeveling) return;

  unsigned long now = millis();
  if (now - lastLevelingTime < LEVELING_INTERVAL_MS) return;
  float dt = (now - lastLevelingTime) * 0.001f;
  lastLevelingTime = now;

  // read tilt
  float pitch_deg, roll_deg;
  tilt.getPitchRoll(pitch_deg, roll_deg);

  // dead-zone check
  if (fabs(pitch_deg) < LEVEL_DEAD_ZONE_DEG && fabs(roll_deg) < LEVEL_DEAD_ZONE_DEG) {
    if (!levelMsgSent) {
      Serial.println("Levelling has been completed");
      levelMsgSent = true;
    }
    for (int i=0;i<NUM_MOTORS;i++) motors[i].setLevelingVelocity(0.0f);
    return;
  } else {
    levelMsgSent = false;
  }

  // errors (rad), signs chosen to drive toward zero
  const float pitch_e = -pitch_deg * (PI/180.0f);
  const float roll_e  = -roll_deg  * (PI/180.0f);

  // small-angle linear kinematics: dh = y*pitch + x*roll
  const float dh1 = M1_Y  * pitch_e;
  const float dh2 = M23_Y * pitch_e +  M2_X * roll_e;
  const float dh3 = M23_Y * pitch_e + (-M2_X)* roll_e;

  // convert to step velocities
  auto clamp = [](float v, float lim){ return v>lim? lim : (v<-lim? -lim : v); };
  float v1 = clamp(dh1 * K_V_MM_PER_RAD * STEPS_PER_MM, MAX_SPEED_STEPS);
  float v2 = clamp(dh2 * K_V_MM_PER_RAD * STEPS_PER_MM, MAX_SPEED_STEPS);
  float v3 = clamp(dh3 * K_V_MM_PER_RAD * STEPS_PER_MM, MAX_SPEED_STEPS);

  motors[0].setLevelingVelocity(v1);
  motors[1].setLevelingVelocity(v2);
  motors[2].setLevelingVelocity(v3);

  // Each motor will convert v->position increment in its update()
}

// ==============================
// ------- Command parser -------
// ==============================
void StepperController::handleSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c=='\n' || c=='\r') {
      String cmd = serialBuf; serialBuf = "";
      cmd.trim();
      if (!cmd.length()) continue;

      if (cmd == "CALIBRATE_ACCEL") {
        Serial.println("Calibrating accelerometer... do not move the plate.");
        float raw_p, raw_r;
        tilt.getRawPitchRoll(raw_p, raw_r);
        pitch_offset = raw_p; roll_offset = raw_r;
        EEPROM.put(0, pitch_offset);
        EEPROM.put(sizeof(float), roll_offset);
        Serial.println("ACCEL_CALIBRATED");
        continue;
      }

      if (cmd == "LEVEL_ON")  { startLeveling(); continue; }
      if (cmd == "LEVEL_OFF") { stopLeveling();  continue; }

      if (isLeveling && (cmd=="STOP" || cmd=="CANCEL" || cmd=="ESTOP")) {
        stopLeveling();
      }

      if (cmd.startsWith("CONFIG_SPEED ")) {
        int s1 = cmd.indexOf(' '), s2 = cmd.indexOf(' ', s1+1);
        if (s1>0 && s2>0) {
          float maxSp = cmd.substring(s1+1, s2).toFloat();
          float acc   = cmd.substring(s2+1).toFloat();
          for (int i=0;i<NUM_MOTORS;i++) motors[i].setSpeedLimits(maxSp, acc);
          Serial.println("SPEED_CONFIG_OK");
        }
        continue;
      }

      if (cmd.startsWith("MOVE_M ")) {
        int s1=cmd.indexOf(' '), s2=cmd.indexOf(' ', s1+1), s3=cmd.indexOf(' ', s2+1);
        if (s1>0 && s2>0 && s3>0) {
          int motor_index = cmd.substring(s1+1, s2).toInt();
          float value     = cmd.substring(s2+1, s3).toFloat();
          String units    = cmd.substring(s3+1);
          long steps = units.equalsIgnoreCase("mm") ? lround(value * STEPS_PER_MM)
                                                    : lround(value);
          if (motor_index >= 0 && motor_index < NUM_MOTORS) {
            motors[motor_index].moveRelative(steps);
          } else if (motor_index == NUM_MOTORS || motor_index == 3) {
            for (int i=0;i<NUM_MOTORS;i++) motors[i].moveRelative(steps);
          }
        }
        continue;
      }

      if (cmd.startsWith("MOVE ")) {
        float deg = cmd.substring(5).toFloat();
        long steps = lround((deg / 360.0f) * (STEPS_PER_REV * MICROSTEPS));
        motors[0].moveRelative(steps);
        continue;
      }

      if (cmd == "HOME") {
        motors[0].moveToZero(); continue;
      }
      if (cmd == "DUNK") {
        motors[0].dunk(); continue;
      }
      if (cmd == "GET_POS") {
        Serial.print("POS "); Serial.print(0); Serial.print(" ");
        Serial.println(motors[0].position());
        continue;
      }
      if (cmd == "STOP" || cmd=="CANCEL" || cmd=="ESTOP") {
        for (int i=0;i<NUM_MOTORS;i++) motors[i].stop();
        Serial.println(cmd=="ESTOP" ? "ESTOP" : "STOPPING");
        continue;
      }
      if (cmd == "HOLD ON") {
        for (int i=0;i<NUM_MOTORS;i++){ motors[i].holdTorque=true; if (motors[i].enaPin>0) digitalWrite(motors[i].enaPin, LOW); }
        Serial.println("HOLD ON"); continue;
      }
      if (cmd == "HOLD OFF") {
        for (int i=0;i<NUM_MOTORS;i++){ motors[i].holdTorque=false; if (!motors[i].isRunning() && motors[i].enaPin>0) digitalWrite(motors[i].enaPin, HIGH); }
        Serial.println("HOLD OFF"); continue;
      }
      if (cmd == "SET_ZERO") {
        motors[0].setZero();
        Serial.println("HOMED");
        Serial.print("POS "); Serial.print(0); Serial.print(" ");
        Serial.println(motors[0].position());
        continue;
      }
    } else {
      serialBuf += c;
    }
  }
}

// ==============================
// ---------- Arduino -----------
// ==============================
void setup() {
  Serial.begin(9600);

  EEPROM.get(0, pitch_offset);
  EEPROM.get(sizeof(float), roll_offset);
  if (isnan(pitch_offset)) pitch_offset = 0.0f;
  if (isnan(roll_offset))  roll_offset  = 0.0f;

  pinMode(LED_MOVING_PIN, OUTPUT);
  digitalWrite(LED_MOVING_PIN, LOW);

  for (int i=0;i<NUM_MOTORS;i++) motors[i].begin();
  tilt.begin();

  Serial.println("Stepper ready (TB6600) - 3 Actuators with Auto-Leveling");
}

void loop() {
  // Buttons on motor 0 (manual jog/home/dunk)
  buttons.update(motors[0]);

  // Serial command handling (read once; applies to all)
  motors[0].handleSerial();

  // Run motors
  const bool levelingActive = isLeveling;
  static unsigned long lastTick = millis();
  unsigned long now = millis();
  float dt = (now - lastTick) * 0.001f;
  if (dt <= 0) dt = 0.001f;
  lastTick = now;

  for (int i=0;i<NUM_MOTORS;i++) {
    motors[i].update(i, dt, levelingActive);
  }

  // Telemetry only when all are idle
  bool anyRunning = false;
  for (int i=0;i<NUM_MOTORS;i++) if (motors[i].isRunning()) { anyRunning = true; break; }
  if (!anyRunning) tilt.pushTelemetry();

  // Leveling controller
  updateLeveling();
}
