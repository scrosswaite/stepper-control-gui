#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>

MPU6050 mpu;

bool dmpReady = false;  
uint8_t devStatus;      
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 

Quaternion q;           
VectorFloat gravity;    
float ypr[3];           

void setup() {
  Serial.begin(115200);
  Wire.begin();

  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println("MPU6050 ready");
  } else {
    Serial.print("DMP init failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }
}

void loop() {
  if (!dmpReady) return;

  fifoCount = mpu.getFIFOCount();

  if (fifoCount < packetSize) return;

  if (fifoCount >= 1024) {
    mpu.resetFIFO();
    return;
  }

  while (fifoCount >= packetSize) {
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
  }

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  float yaw   = ypr[0] * 180/M_PI;
  float pitch = ypr[1] * 180/M_PI;
  float roll  = ypr[2] * 180/M_PI;

  // send as CSV for Python
  Serial.print(roll); Serial.print(",");
  Serial.print(pitch); Serial.print(",");
  Serial.println(yaw);
}
