#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <ezButton.h>
#include <AccelStepper.h>
#include <EEPROM.h>
#include <limits.h>

// ==============================
// ----- User Configuration -----
// ==============================
const float LEAD_MM        = 2.0f;  // screw lead (mm/rev)
const int   MICROSTEPS     = 8;     // set to match TB6600 DIP switches
const int   STEPS_PER_REV  = 200;   // full steps per rev
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
const unsigned long LEVELING_INTERVAL_MS = 20;       // ~50 Hz
const float LEVEL_DEAD_IN_DEG  = 0.30f;              // widened enter band
const float LEVEL_DEAD_OUT_DEG = 0.50f;              // widened leave band
const float K_V_MM_PER_RAD     = 15.0f;              // gentler velocity gain

// Motor speed/accel limits (in steps/s)
const float MAX_SPEED_STEPS = 3000.0f;               // must be <= setMaxSpeed()
const float ACCEL_STEPS     = 5000.0f;               // higher accel for smoother ramps

// Velocity command shaping
const float DV_MAX = 3000.0f;   // steps/s^2 max slew rate for v_cmd
const float V_MIN  = 300.0f;    // steps/s minimum useful speed (deadband)

// Defaults (keep your current constants too)
const float LEAD_MM_DEFAULT     = LEAD_MM;
const int   STEPS_PER_REV_DEF   = STEPS_PER_REV;
const float MAX_SPEED_STEPS_DEF = MAX_SPEED_STEPS;
const float ACCEL_STEPS_DEF     = ACCEL_STEPS;

// Runtime mechanics/speeds
static float g_lead_mm         = LEAD_MM_DEFAULT;
static int   g_steps_per_rev   = STEPS_PER_REV_DEF;
static float g_steps_per_mm    = (STEPS_PER_REV_DEF * MICROSTEPS) / LEAD_MM_DEFAULT;
static float g_max_speed_steps = MAX_SPEED_STEPS_DEF;
static float g_accel_steps     = ACCEL_STEPS_DEF;

// helpers (keep for completeness)
static long runtimeJogSteps()  { return lround(JOG_MM  * g_steps_per_mm); }
static long runtimeDunkSteps() { return lround(DUNK_MM * g_steps_per_mm); }

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
const int EEPROM_ADDR_GBIAS_X = sizeof(float) * 2;
const int EEPROM_ADDR_GBIAS_Y = sizeof(float) * 3;

bool   isLeveling = false;
bool   levelMsgSent = false;
unsigned long lastLevelingTime = 0;

// Dead-zone hysteresis & dwell
const unsigned long DEAD_DWELL_MS = 300;  // ms before "completed" message
bool deadIn = false;
unsigned long deadEnterMs = 0;

// PI terms (I disabled for now via gain = 0)
const float K_I_MM_PER_RADs = 0.0f; // integral OFF for stability-first tuning
const float I_DECAY_PER_S    = 0.2f; // leak (kept; unused when Ki=0)
const float I_MAX_RADs       = 0.1f; // clamp integral magnitude

float int_pitch = 0.0f, int_roll = 0.0f;

bool imu_ok = true;

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

  float gb_roll_dps = 0.0f;  // bias for gx (roll)
  float gb_pitch_dps = 0.0f; // bias for gy (pitch)

  // --- Kalman filter tuning variables ---
  float Q_angle = 0.001f;
  float Q_bias = 0.003f;
  float R_measure = 0.03f;

  // --- Pitch State Variables ---
  float angle_pitch = 0.0f;
  float bias_pitch = 0.0f;
  float P_pitch[2][2] = {{0, 0}, {0, 0}};

  // --- Roll State Variables ---
  float angle_roll = 0.0f;
  float bias_roll = 0.0f;
  float P_roll[2][2] = {{0, 0}, {0, 0}};

public:
  void begin() {
    Wire.begin();
    Wire.setClock(400000); // Fast I2C
    mpu.initialize();
    // ranges + low-pass filter
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setFullScaleGyroRange (MPU6050_GYRO_FS_250);
    mpu.setDLPFMode(4); // ~21 Hz

    if (!mpu.testConnection()) {
      Serial.println("MPU6050 connection failed!");
      imu_ok = false;
    } else {
      imu_ok = true;
    }

    EEPROM.get(EEPROM_ADDR_GBIAS_X, gb_roll_dps);
    EEPROM.get(EEPROM_ADDR_GBIAS_Y, gb_pitch_dps);
    if (isnan(gb_roll_dps))  gb_roll_dps  = 0.0f;
    if (isnan(gb_pitch_dps)) gb_pitch_dps = 0.0f;

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

  // filtered pitch/roll (deg), gravity-referenced using Kalman Filter
  void getPitchRoll(float &pitch, float &roll) {
    uint32_t now = millis();
    float dt = (now - last_ts_ms) * 0.001f;
    if (dt <= 0) dt = 0.001f;
    last_ts_ms = now;

    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);

    // Calculate angles from accelerometer
    float accel_angle_pitch = atan2f((float)ay, sqrtf((float)ax*ax + (float)az*az)) * 180.0f/PI;
    float accel_angle_roll  = atan2f(-(float)ax, (float)az) * 180.0f/PI;

    // Gyro rates in deg/s
    float gyro_rate_pitch = gy / 131.0f;
    float gyro_rate_roll  = gx / 131.0f;

    // ======== PITCH AXIS KALMAN FILTER ========
    {
      // Predict
      float rate = gyro_rate_pitch - bias_pitch;
      angle_pitch += dt * rate;
      // Covariance
      P_pitch[0][0] += dt * (dt * P_pitch[1][1] - P_pitch[0][1] - P_pitch[1][0] + Q_angle);
      P_pitch[0][1] -= dt * P_pitch[1][1];
      P_pitch[1][0] -= dt * P_pitch[1][1];
      P_pitch[1][1] += Q_bias * dt;
      // Update
      float S = P_pitch[0][0] + R_measure;
      float K[2];
      K[0] = P_pitch[0][0] / S;
      K[1] = P_pitch[1][0] / S;
      float y = accel_angle_pitch - angle_pitch;
      angle_pitch += K[0] * y;
      bias_pitch  += K[1] * y;
      float P00_temp = P_pitch[0][0];
      float P01_temp = P_pitch[0][1];
      P_pitch[0][0] -= K[0] * P00_temp;
      P_pitch[0][1] -= K[0] * P01_temp;
      P_pitch[1][0] -= K[1] * P00_temp;
      P_pitch[1][1] -= K[1] * P01_temp;
    }

    // ======== ROLL AXIS KALMAN FILTER ========
    {
      // Predict
      float rate = gyro_rate_roll - bias_roll;
      angle_roll += dt * rate;
      // Covariance
      P_roll[0][0] += dt * (dt * P_roll[1][1] - P_roll[0][1] - P_roll[1][0] + Q_angle);
      P_roll[0][1] -= dt * P_roll[1][1];
      P_roll[1][0] -= dt * P_roll[1][1];
      P_roll[1][1] += Q_bias * dt;
      // Update
      float S = P_roll[0][0] + R_measure;
      float K[2];
      K[0] = P_roll[0][0] / S;
      K[1] = P_roll[1][0] / S;
      float y = accel_angle_roll - angle_roll;
      angle_roll += K[0] * y;
      bias_roll  += K[1] * y;
      float P00_temp = P_roll[0][0];
      float P01_temp = P_roll[0][1];
      P_roll[0][0] -= K[0] * P00_temp;
      P_roll[0][1] -= K[0] * P01_temp;
      P_roll[1][0] -= K[1] * P00_temp;
      P_roll[1][1] -= K[1] * P01_temp;
    }

    // Return the filtered values
    pitch = angle_pitch - pitch_offset;
    roll  = angle_roll - roll_offset;
  }
  
  void calibrateGyro() {
    const int num_samples = 1000;
    float gx_sum = 0;
    float gy_sum = 0;

    Serial.println("Calibrating gyroscope... keep the sensor still.");

    for (int i = 0; i < num_samples; i++) {
      int16_t ax, ay, az, gx, gy, gz;
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      gx_sum += gx;
      gy_sum += gy;
      delay(2);
    }

    gb_roll_dps = (gx_sum / num_samples) / 131.0f;
    gb_pitch_dps = (gy_sum / num_samples) / 131.0f;

    EEPROM.put(EEPROM_ADDR_GBIAS_X, gb_roll_dps);
    EEPROM.put(EEPROM_ADDR_GBIAS_Y, gb_pitch_dps);

    Serial.println("Gyro calibration complete.");
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

  // leveling velocity command (steps/s)
  float v_cmd = 0.0f;
  float stepAccum = 0.0f; // fractional steps accumulator for the walking target

  // soft travel limits
  long softMin = LONG_MIN;  // default disabled
  long softMax = LONG_MAX;

public:
  bool wasRunning = false;
  String serialBuf;

  void enableMotor()  { if (enaPin > 0) digitalWrite(enaPin, LOW); }
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
    stepper.setMinPulseWidth(5); // TB6600 robustness

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

  void setSoftLimits(long minSteps, long maxSteps) {
    softMin = minSteps;
    softMax = maxSteps;
  }

  void handleSerial(); // declared below

  void update(int idx, float dt, bool levelingActive) {
    // A "running" proxy that works in both modes
    bool running_now = levelingActive ? (fabs(v_cmd) > 1.0f || stepper.isRunning())
                                      : stepper.isRunning();

    // On start: LED + enable
    if (!wasRunning && running_now) {
      if (ledPin > 0) digitalWrite(ledPin, HIGH);
      enableMotor();
    }

    // ----- Limit switches -----
    if (limit1Pin>0 && limit2Pin>0) {
      if (!levelingActive && stepper.isRunning()) {
        bool movingForward = stepper.distanceToGo() > 0;
        if ((movingForward && digitalRead(limit1Pin)==LOW) ||
            (!movingForward && digitalRead(limit2Pin)==LOW)) {
          stepper.stop();
          Serial.print("LIMIT "); Serial.println(idx);
        }
      }
    }

    // --- Soft limits check (position mode only here) ---
    long pos = stepper.currentPosition();
    if (!levelingActive && stepper.isRunning()) {
      if (pos >= softMax && stepper.distanceToGo() > 0) {
        stepper.stop(); Serial.print("SOFT_LIMIT_MAX "); Serial.println(idx);
      }
      if (pos <= softMin && stepper.distanceToGo() < 0) {
        stepper.stop(); Serial.print("SOFT_LIMIT_MIN "); Serial.println(idx);
      }
    }

    // ----- Motion service -----
    if (levelingActive) {
      // Walking target: advance by v_cmd*dt (with fractional accumulation)
      stepAccum += v_cmd * dt;         // steps this tick (may be fractional)
      long inc = (long)stepAccum;      // whole steps to command
      stepAccum -= inc;                // keep remainder

      // Hard stop at soft limits in speed/levelling mode
      if ((pos >= softMax && inc > 0) || (pos <= softMin && inc < 0)) {
        inc = 0;
        v_cmd = 0.0f;
        Serial.print("SOFT_LIMIT_"); Serial.println(pos >= softMax ? "MAX" : "MIN");
      }

      // Limit-switch safety in levelling (use direction of increment)
      if (limit1Pin>0 && limit2Pin>0 && inc != 0) {
        bool movingForward = inc > 0;
        if ((movingForward && digitalRead(limit1Pin)==LOW) ||
            (!movingForward && digitalRead(limit2Pin)==LOW)) {
          inc = 0;
          v_cmd = 0.0f;
          Serial.print("LIMIT "); Serial.println(idx);
        }
      }

      if (inc != 0) {
        // Advance a persistent target; moveTo (not move) preserves velocity ramping
        stepper.moveTo(stepper.targetPosition() + inc);
      }
      stepper.run(); // respects acceleration
    } else {
      stepper.run(); // position moves (with acceleration)
    }

    // On stop
    bool running_after = levelingActive ? (fabs(v_cmd) > 1.0f || stepper.isRunning())
                                        : stepper.isRunning();
    if (wasRunning && !running_after) {
      if (ledPin > 0) digitalWrite(ledPin, LOW);
      Serial.print("MOVED "); Serial.println(idx);
      Serial.print("POS ");   Serial.print(idx); Serial.print(" ");
      Serial.println(stepper.currentPosition());
      if (!holdTorque) disableMotor();
    }

    wasRunning = running_after;
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
  if (!imu_ok) { Serial.println("LEVELLING_IMU_NOT_READY"); return; }
  isLeveling = true;
  levelMsgSent = false;
  for (int i=0;i<NUM_MOTORS;i++) {
    motors[i].enableMotor();
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
    if (!motors[i].holdTorque) motors[i].disableMotor();
  }
  Serial.println("LEVELING_OFF");
}

void updateLeveling() {
  if (!isLeveling) return;

  unsigned long now = millis();
  if (now - lastLevelingTime < LEVELING_INTERVAL_MS) return;
  float dt = (now - lastLevelingTime) * 0.001f;
  if (dt <= 0) dt = 0.001f;
  lastLevelingTime = now;

  // read tilt
  float pitch_deg, roll_deg;
  tilt.getPitchRoll(pitch_deg, roll_deg);

  // ---- extra smoothing on KF output (small LPF to tame chatter) ----
  static bool pr_init = false;
  static float p_f = 0.0f, r_f = 0.0f;
  const float beta = 0.05f; // 0..1; lower = more smoothing
  if (!pr_init) { p_f = pitch_deg; r_f = roll_deg; pr_init = true; }
  p_f = beta * pitch_deg + (1.0f - beta) * p_f;
  r_f = beta * roll_deg  + (1.0f - beta) * r_f;

  // Use smoothed values
  pitch_deg = p_f;
  roll_deg  = r_f;

  // --- Dead-zone with hysteresis and dwell ---
  bool inside  = (fabs(pitch_deg) < LEVEL_DEAD_IN_DEG) && (fabs(roll_deg) < LEVEL_DEAD_IN_DEG);
  bool outside = (fabs(pitch_deg) > LEVEL_DEAD_OUT_DEG) || (fabs(roll_deg) > LEVEL_DEAD_OUT_DEG);

  if (!deadIn && inside) {
    deadIn = true;
    deadEnterMs = now;
  } else if (deadIn && outside) {
    deadIn = false;
    levelMsgSent = false;
  }


  if (deadIn) {
    // dwell before messaging; stop motion
    if (!levelMsgSent && (now - deadEnterMs) > DEAD_DWELL_MS) {
      Serial.println("Levelling has been completed");
      levelMsgSent = true;
    }
    for (int i=0;i<NUM_MOTORS;i++) motors[i].setLevelingVelocity(0.0f);
    return;
  }

  // errors (rad), signs chosen to drive toward zero
  const float pitch_e = -pitch_deg * (PI/180.0f);
  const float roll_e  = -roll_deg  * (PI/180.0f);

  // Integral with leak & clamp (harmless when Ki=0)
  auto clampf = [](float v, float lim){ return v>lim? lim : (v<-lim? -lim : v); };
  int_pitch = clampf(int_pitch * (1.0f - I_DECAY_PER_S*dt) + pitch_e*dt, I_MAX_RADs);
  int_roll  = clampf(int_roll  * (1.0f - I_DECAY_PER_S*dt) + roll_e *dt, I_MAX_RADs);

  // small-angle linear kinematics: dh = y*pitch + x*roll (plus integral term scaled)
  const float ki_scale = (K_I_MM_PER_RADs == 0.0f) ? 0.0f : (K_I_MM_PER_RADs / K_V_MM_PER_RAD);
  const float dh1 = (M1_Y  * pitch_e)                  + (M1_Y  * int_pitch + 0.0f * int_roll) * ki_scale;
  const float dh2 = (M23_Y * pitch_e +  M2_X * roll_e) + (M23_Y * int_pitch +  M2_X * int_roll) * ki_scale;
  const float dh3 = (M23_Y * pitch_e + (-M2_X)*roll_e) + (M23_Y * int_pitch + (-M2_X)*int_roll) * ki_scale;

  // convert to step velocities (raw)
  float v1_raw = dh1 * K_V_MM_PER_RAD * STEPS_PER_MM;
  float v2_raw = dh2 * K_V_MM_PER_RAD * STEPS_PER_MM;
  float v3_raw = dh3 * K_V_MM_PER_RAD * STEPS_PER_MM;

  // small blend (optional), then rate-limit and deadband
  const float alpha = 0.10f; // 0..1; higher = more responsive
  float v1_des = alpha*v1_raw + (1.0f-alpha)*motors[0].v_cmd;
  float v2_des = alpha*v2_raw + (1.0f-alpha)*motors[1].v_cmd;
  float v3_des = alpha*v3_raw + (1.0f-alpha)*motors[2].v_cmd;

  auto rate_limit = [&](float v_des, float v_prev) {
    float dv = v_des - v_prev;
    float dv_max_tick = DV_MAX * dt;
    if (dv >  dv_max_tick) dv =  dv_max_tick;
    if (dv < -dv_max_tick) dv = -dv_max_tick;
    float v_out = v_prev + dv;
    // deadband around zero to avoid buzz
    if (fabs(v_out) < V_MIN) v_out = 0.0f;
    // clamp to max speed
    float vmax = MAX_SPEED_STEPS;
    if (v_out >  vmax) v_out =  vmax;
    if (v_out < -vmax) v_out = -vmax;
    return v_out;
  };

  float v1 = rate_limit(v1_des, motors[0].v_cmd);
  float v2 = rate_limit(v2_des, motors[1].v_cmd);
  float v3 = rate_limit(v3_des, motors[2].v_cmd);

  static uint32_t lastDbg = 0;
if (millis() - lastDbg > 250) {
  lastDbg = millis();
  Serial.print(F("DBG p/r(deg): "));
  Serial.print(pitch_deg, 2); Serial.print(F(" / ")); Serial.print(roll_deg, 2);
  Serial.print(F(" | v_raw: "));
  Serial.print(v1_raw, 0); Serial.print(' '); Serial.print(v2_raw, 0); Serial.print(' '); Serial.print(v3_raw, 0);
  Serial.print(F(" | v_cmd: "));
  Serial.print(v1, 0); Serial.print(' '); Serial.print(v2, 0); Serial.print(' '); Serial.print(v3, 0);
  Serial.println();
}

  motors[0].setLevelingVelocity(v1);
  motors[1].setLevelingVelocity(v2);
  motors[2].setLevelingVelocity(v3);
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
      
      if (cmd == "CALIBRATE_GYRO") {
        tilt.calibrateGyro();
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

      if (cmd == "GET_CONFIG") {
        Serial.print("CONFIG ");
        Serial.print("LEAD_MM "); Serial.print(g_lead_mm); Serial.print(" ");
        Serial.print("STEPS_PER_REV "); Serial.print(g_steps_per_rev); Serial.print(" ");
        Serial.print("STEPS_PER_MM "); Serial.print(g_steps_per_mm); Serial.print(" ");
        Serial.print("MAX_SPEED "); Serial.print(g_max_speed_steps); Serial.print(" ");
        Serial.print("ACCEL "); Serial.println(g_accel_steps);
        continue;
      }

      if (cmd.startsWith("SET_LIMITS ")) {
        int s1 = cmd.indexOf(' '), s2 = cmd.indexOf(' ', s1+1), s3 = cmd.indexOf(' ', s2+1);
        if (s1>0 && s2>0 && s3>0) {
          int motor_index = cmd.substring(s1+1, s2).toInt();
          long minS = cmd.substring(s2+1, s3).toInt();
          long maxS = cmd.substring(s3+1).toInt();
          if (motor_index >= 0 && motor_index < NUM_MOTORS) {
            motors[motor_index].setSoftLimits(minS, maxS);
            Serial.println("LIMITS_OK");
          }
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
        Serial.print("POS ");
        Serial.print(0); Serial.print(" ");
        Serial.println(motors[0].position());
        continue;
      }
      if (cmd == "STOP" || cmd=="CANCEL" || cmd=="ESTOP") {
        for (int i=0;i<NUM_MOTORS;i++) motors[i].stop();
        Serial.println(cmd=="ESTOP" ? "ESTOP" : "STOPPING");
        continue;
      }
      if (cmd == "HOLD ON") {
        for (int i=0;i<NUM_MOTORS;i++){ motors[i].holdTorque=true; motors[i].enableMotor(); }
        Serial.println("HOLD ON"); continue;
      }
      if (cmd == "HOLD OFF") {
        for (int i=0;i<NUM_MOTORS;i++){ motors[i].holdTorque=false; if (!motors[i].isRunning()) motors[i].disableMotor(); }
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

  for (int i=0;i<NUM_MOTORS;i++) {
    motors[i].begin();
    // Example soft limits: -75 mm to +75 mm
    motors[i].setSoftLimits(lround(-75.0f * g_steps_per_mm), lround(75.0f * g_steps_per_mm));
  }
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
  for (int i=0;i<NUM_MOTORS;i++) {
    if (levelingActive ? (fabs(motors[i].v_cmd) > 1.0f) : motors[i].isRunning()) { anyRunning = true; break; }
  }
  if (!anyRunning) tilt.pushTelemetry();

  // Leveling controller
  updateLeveling();
}
