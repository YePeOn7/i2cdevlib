#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps612.h"
#include "Wire.h"
MPU6050 mpu;

uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO

float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int16_t gyro[3];
int16_t acc[3];

char calibrationState = 0;

void MPU6050_setup()
{
  mpu.initialize();

  // verify connection
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  mpu.dmpSetEnableGyroCalibration(0);
  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.print("Packet size: ");
    Serial.println(packetSize);
  }
  else
  {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // Serial.println("Calibrating...");
  // mpu.dmpWaitForCalibration(0.015, 20000);
}

void checkRAWOutput()
{
    mpu.readGyroRaw(gyro);
    mpu.readAccRaw(acc);
    Serial.print(gyro[0]);
    Serial.print("\t");
    Serial.print(gyro[1]);
    Serial.print("\t");
    Serial.print(gyro[2]);
    Serial.print("\t");
    Serial.print(acc[0]);
    Serial.print("\t");
    Serial.print(acc[1]);
    Serial.print("\t");
    Serial.print(acc[2]);
    Serial.println(" ");
}

void checkYawInfo()
{
  if(mpu.dmpGetYawPitchRoll(ypr, acc, gyro))
  { 
    mpu.dmpGyroContinuosCalibration(ypr);
    int yawOffset = mpu.getZGyroOffset();

    Serial.print("GyroZ: ");
    Serial.print(gyro[2]);
    Serial.print("\tyaw: "); 
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\toffset: ");
    Serial.print(yawOffset);
    Serial.println();
    delay(100);
  }
}

void changeOffset(int value)
{
  int currentOffset = mpu.getZGyroOffset();
  
  currentOffset+=value;
  mpu.setZGyroOffset(currentOffset);

}

/****************** SETUP *****************/
void setup()
{
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);
  
  MPU6050_setup();
  pinMode(12, INPUT_PULLUP);
}

/****************** MAIN LOOP **************/
void loop()
{
  checkYawInfo();

  // if(calibrationState && !digitalRead(12))
  // {
  //   changeOffset(5);
  //   Serial.println("Changeing offset by 5");
  //   calibrationState = 0;
  // }
  // else if(!calibrationState && digitalRead(12))
  // {
  //   changeOffset(-5);
  //   Serial.println("Changing ofset by -5");
  //   calibrationState = 1;
  // }
}
