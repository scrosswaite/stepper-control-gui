// full_test_active_low_300mm_dunk.ino
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>

// —–––– Pin assignments –––––—
const int ENA_PIN  = 6;    // active‑LOW enable
const int DIR_PIN  = 5;    // direction
const int PUL_PIN  = 4;    // step pulse

// Manual switches (to GND when pressed)
const int SW_FWD          = 9;   // jog +0.5 mm
const int SW_REV          = 8;   // jog –0.5 mm
const int SW_HOME         = 12;  // home to zero
const int SW_MOVE_ONE_REV = 11;  // dunk → -50 mm
const int SW_CALIB        = 10;  // calibrate
const int LIMIT1_PIN = 2; // Limit Switch 2
const int LIMIT2_PIN = 3; // Limit Switch 3

// LEDs
const int LED_PWR_PIN    = A1;
const int LED_READY_PIN  = A2;
const int LED_MOVING_PIN = A3;
const int LED_ERROR_PIN  = A4;  // unused

// Motion parameters
const int    STEPS_PER_REV = 200;     // full‑steps per revolution
const unsigned long PULSE_US = 200;   // 200 µs HIGH, 200 µs LOW

// Derived for jog & dunk, based on 2 mm lead screw:
const int    STEPS_PER_MM = STEPS_PER_REV / 2;      // 100 steps per mm
const int    JOG_STEPS    = STEPS_PER_MM / 2;       // 0.5 mm → 50 steps
const long   DUNK_STEPS   = 50L * STEPS_PER_MM;     // 50 mm → 5 000 steps

// Debounce timing
const unsigned long debounceDelay = 20;  // ms

// Blink timing
const unsigned long blinkInterval = 100; // ms

// State
long  currentPosition = 0;
bool  isMoving        = false;

// Debounce trackers
bool    fwdStable=HIGH, fwdLast=HIGH, lastFwd=HIGH;
bool    revStable=HIGH, revLast=HIGH, lastRev=HIGH;
bool    homeStable=HIGH, homeLast=HIGH, lastHome=HIGH;
bool    dunkStable=HIGH, dunkLast=HIGH, lastDunk=HIGH;
bool    calibStable=HIGH, calibLast=HIGH, lastCalib=HIGH;
unsigned long fwdDeb=0, revDeb=0, homeDeb=0, dunkDeb=0, calibDeb=0;

// Blink trackers
unsigned long lastBlink = 0;
bool         blinkState = LOW;

// MPU6050 Integration
MPU6050 mpu;
unsigned long lastTiltTime = 0;

// —–––– Helpers –––––—
void updateBlink(){
  unsigned long now = millis();
  if(now - lastBlink >= blinkInterval){
    lastBlink = now;
    blinkState = !blinkState;
    digitalWrite(LED_MOVING_PIN, blinkState);
  }
}

// returns true if either limit switch is currently pressed
bool limitReached() {
  return digitalRead(LIMIT1_PIN) == LOW
    || digitalRead(LIMIT2_PIN) == LOW;
}

void doSteps(long steps, int dir){
  digitalWrite(DIR_PIN, dir > 0 ? HIGH : LOW);
  isMoving = true;

  for(long i = 0; i < abs(steps); ++i){
    // immediate check before each pulse:
    if(limitReached()){
      Serial.println("LIMIT TRIPPED – STOPPING");
      digitalWrite(ENA_PIN, HIGH); // disable driver (active‑LOW)
      break;
    }

    digitalWrite(PUL_PIN, HIGH);
    delayMicroseconds(PULSE_US);
    digitalWrite(PUL_PIN, LOW);
    delayMicroseconds(PULSE_US);

    currentPosition += (dir > 0 ? +1 : -1);
    updateBlink();
  }

  // make sure MOVING LED is off
  digitalWrite(LED_MOVING_PIN, LOW);
  isMoving = false;
}

void computePitchRoll(float &pitch, float &roll) {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  // convert to g
  float fAx = ax / 16384.0;
  float fAy = ay / 16384.0;
  float fAz = az / 16384.0;
  // tilt formulas:
  pitch = atan2(fAy, sqrt(fAx*fAx + fAz*fAz)) * 180.0 / PI;
  roll = atan2(-fAx, fAz) * 180.0 / PI;
}

// —–––– Setup –––––—
void setup(){
  Serial.begin(9600);
  Serial.println("Stepper ready (0.5 mm jog, 50 mm dunk)");

  //MPU6050
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {

    Serial.println("MPU6050 connection failed!");
  }

  // driver pins
  pinMode(ENA_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PUL_PIN, OUTPUT);
  digitalWrite(ENA_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(PUL_PIN, LOW);

  // inputs
  pinMode(SW_FWD,          INPUT_PULLUP);
  pinMode(SW_REV,          INPUT_PULLUP);
  pinMode(SW_HOME,         INPUT_PULLUP);
  pinMode(SW_MOVE_ONE_REV, INPUT_PULLUP);
  pinMode(SW_CALIB,        INPUT_PULLUP);
  pinMode(LIMIT1_PIN, INPUT_PULLUP);
  pinMode(LIMIT2_PIN, INPUT_PULLUP);

  // LEDs
  pinMode(LED_PWR_PIN,    OUTPUT);
  pinMode(LED_READY_PIN,  OUTPUT);
  pinMode(LED_MOVING_PIN, OUTPUT);
  pinMode(LED_ERROR_PIN,  OUTPUT);
  digitalWrite(LED_PWR_PIN, HIGH);
}

// —–––– Main loop –––––—
void loop(){
  unsigned long now = millis();
  bool r;

  // — Jog +0.5 mm —
  r = digitalRead(SW_FWD);
  if(r != fwdLast) fwdDeb = now;
  if(now - fwdDeb > debounceDelay && r != fwdStable){
    fwdStable = r;
    if(fwdStable==LOW && lastFwd==HIGH){
      doSteps(JOG_STEPS, +1);
      Serial.println("JOG +0.5 mm");
    }
    lastFwd = fwdStable;
  }
  fwdLast = r;

  // — Jog -0.5 mm —
  r = digitalRead(SW_REV);
  if(r != revLast) revDeb = now;
  if(now - revDeb > debounceDelay && r != revStable){
    revStable = r;
    if(revStable==LOW && lastRev==HIGH){
      doSteps(JOG_STEPS, -1);
      Serial.println("JOG -0.5 mm");
    }
    lastRev = revStable;
  }
  revLast = r;

  // — Home to zero —
  r = digitalRead(SW_HOME);
  if(r != homeLast) homeDeb = now;
  if(now - homeDeb > debounceDelay && r != homeStable){
    homeStable = r;
    if(homeStable==LOW && lastHome==HIGH){
      Serial.println("HOMED (button)");
      doSteps(abs(currentPosition), currentPosition>0 ? -1 : +1);
      currentPosition = 0;
    }
    lastHome = homeStable;
  }
  homeLast = r;

  // — Dunk = -50 mm —
  r = digitalRead(SW_MOVE_ONE_REV);
  if(r != dunkLast) dunkDeb = now;
  if(now - dunkDeb > debounceDelay && r != dunkStable){
    dunkStable = r;
    if(dunkStable==LOW && lastDunk==HIGH){
      doSteps(DUNK_STEPS, -1);
      Serial.println("DUNK -50 mm");
    }
    lastDunk = dunkStable;
  }
  dunkLast = r;

  // — Calibrate (print only) —
  r = digitalRead(SW_CALIB);
  if(r != calibLast) calibDeb = now;
  if(now - calibDeb > debounceDelay && r != calibStable){
    calibStable = r;
    if(calibStable==LOW && lastCalib==HIGH){
      Serial.println("CALIBRATE");
    }
    lastCalib = calibStable;
  }
  calibLast = r;

  // — Ready LED —
  bool anyActive = 
       digitalRead(SW_FWD)==LOW ||
       digitalRead(SW_REV)==LOW ||
       digitalRead(SW_HOME)==LOW ||
       digitalRead(SW_MOVE_ONE_REV)==LOW ||
       digitalRead(SW_CALIB)==LOW ||
       isMoving;
  digitalWrite(LED_READY_PIN, anyActive);

  if (now - lastTiltTime >= 100) {
    lastTiltTime = now;
    float pitch, roll;
    computePitchRoll(pitch, roll);
    Serial.print("TILT");
    Serial.print(pitch, 2);
    Serial.print(" ");
    Serial.println(roll, 2);
  }

  // — Serial MOVE / HOME commands —
  if(Serial.available()){
    String cmd = Serial.readStringUntil('\n'); cmd.trim();
    if(cmd.startsWith("MOVE ")){
      float deg = cmd.substring(5).toFloat();
      long  steps = lround((deg/360.0)*STEPS_PER_REV);
      Serial.print("Serial MOVE "); Serial.println(deg);
      doSteps(steps, (deg>0?+1:-1));
      Serial.print("MOVED "); Serial.print(deg); Serial.println(" deg");
    }
    else if(cmd=="HOME"){
      Serial.println("Serial HOME");
      doSteps(abs(currentPosition),(currentPosition>0?-1:+1));
      currentPosition = 0;
      Serial.println("HOMED");
    }
    else if(cmd=="DUNK") {
      // Mimic physical button
      doSteps(DUNK_STEPS, -1);
      Serial.println("DUNK -50mm");
    }
    else if(cmd="LEVEL") {
      Serial.println("LEVELING");
      // potential levelling routine here
    }
  }
}
