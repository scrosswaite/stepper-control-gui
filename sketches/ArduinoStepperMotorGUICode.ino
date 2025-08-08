#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <ezButton.h> // <-- NEW: Include the button library

// --- Pin Assignments ---
const int ENA_PIN = 6;
const int DIR_PIN = 5;
const int PUL_PIN = 4;
const int LIMIT1_PIN = 2;
const int LIMIT2_PIN = 3;
const int LED_MOVING_PIN = A3;

// --- Motion Parameters ---
const int STEPS_PER_REV = 200;
const unsigned long PULSE_US = 200;
const int STEPS_PER_MM = STEPS_PER_REV / 2; // Assuming 2mm lead screw
const int JOG_STEPS = STEPS_PER_MM / 2; // 0.5mm jog
const long DUNK_STEPS = 50L * STEPS_PER_MM; // 50mm dunk

// --- Global State ---
long currentPosition = 0;
bool isMoving = false;
unsigned long lastTiltTime = 0;

// --- Object Initialization ---
MPU6050 mpu;

// NEW: Create ezButton objects for each switch
ezButton buttonFwd(9);
ezButton buttonRev(8);
ezButton buttonHome(12);
ezButton buttonDunk(11);
ezButton buttonCalib(10);

// --- Helper Functions ---

bool limitReached() {
    return (digitalRead(LIMIT1_PIN) == LOW || digitalRead(LIMIT2_PIN) == LOW);
}

void doSteps(long steps, int dir) {
    if (isMoving) return; // Prevent new moves while already moving

    digitalWrite(DIR_PIN, dir > 0 ? HIGH : LOW);
    isMoving = true;
    digitalWrite(LED_MOVING_PIN, HIGH);

    for (long i = 0; i < abs(steps); ++i) {
        if (limitReached()) {
            Serial.println("LIMIT"); // Send limit signal to GUI
            break;
        }
        digitalWrite(PUL_PIN, HIGH);
        delayMicroseconds(PULSE_US);
        digitalWrite(PUL_PIN, LOW);
        delayMicroseconds(PULSE_US);
        currentPosition += (dir > 0 ? +1 : -1);
    }

    isMoving = false;
    digitalWrite(LED_MOVING_PIN, LOW);
    // Send a confirmation message back to the GUI
    if (dir != 0) { // Don't send for homing
        Serial.print("MOVED ");
        Serial.println((float)steps / STEPS_PER_REV * 360.0, 2);
    }
}

void computePitchRoll(float &pitch, float &roll) {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    float fAx = ax / 16384.0;
    float fAy = ay / 16384.0;
    float fAz = az / 16384.0;
    pitch = atan2(fAy, sqrt(fAx * fAx + fAz * fAz)) * 180.0 / PI;
    roll = atan2(-fAx, fAz) * 180.0 / PI;
}

void handleSerialCommands() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();

        if (cmd.startsWith("MOVE ")) {
            float deg = cmd.substring(5).toFloat();
            long steps = lround((deg / 360.0) * STEPS_PER_REV);
            doSteps(steps, (deg > 0 ? +1 : -1));
        } else if (cmd == "HOME") {
            doSteps(abs(currentPosition), (currentPosition > 0 ? -1 : +1));
            currentPosition = 0;
            Serial.println("HOMED");
        } else if (cmd == "DUNK") {
            doSteps(DUNK_STEPS, -1);
        } else if (cmd == "LEVEL") {
            Serial.println("LEVELING"); // Placeholder for future logic
        }
    }
}

void handleButtons() {
    buttonFwd.loop();
    buttonRev.loop();
    buttonHome.loop();
    buttonDunk.loop();
    buttonCalib.loop();

    if (buttonFwd.isPressed()) {
        doSteps(JOG_STEPS, +1);
    }
    if (buttonRev.isPressed()) {
        doSteps(JOG_STEPS, -1);
    }
    if (buttonHome.isPressed()) {
        doSteps(abs(currentPosition), (currentPosition > 0 ? -1 : +1));
        currentPosition = 0;
        Serial.println("HOMED");
    }
    if (buttonDunk.isPressed()) {
        doSteps(DUNK_STEPS, -1);
    }
    if (buttonCalib.isPressed()) {
        Serial.println("CALIBRATE"); // Signal to GUI to start calibration
    }
}

// --- Main Setup and Loop ---

void setup() {
    Serial.begin(9600);
    Serial.println("Stepper ready");

    Wire.begin();
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
    }

    pinMode(ENA_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(PUL_PIN, OUTPUT);
    digitalWrite(ENA_PIN, LOW); // Enable driver (active low)

    pinMode(LIMIT1_PIN, INPUT_PULLUP);
    pinMode(LIMIT2_PIN, INPUT_PULLUP);
    pinMode(LED_MOVING_PIN, OUTPUT);
    
    // Set debounce time for all buttons
    buttonFwd.setDebounceTime(50);
    buttonRev.setDebounceTime(50);
    buttonHome.setDebounceTime(50);
    buttonDunk.setDebounceTime(50);
    buttonCalib.setDebounceTime(50);
}

void loop() {
    handleButtons();
    handleSerialCommands();

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