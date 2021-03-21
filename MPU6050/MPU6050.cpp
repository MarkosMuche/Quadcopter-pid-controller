#include <Wire.h>
#include "Arduino.h"
#include "MPU6050.h"
using namespace std;
#define X 0
#define Y 1
#define Z 2
MPU6050 ::MPU6050() {
}
void MPU6050::setupSensor() {
  Wire.begin();                         // Start I2C communication
  TWBR = 12;                            // Set the I2C clock speed to 400kHz.
// Configure power management
  Wire.beginTransmission(this->MPU_ADDRESS); // Start communication with MPU
  Wire.write(0x6B);                    // Request the PWR_MGMT_1 register
  Wire.write(0x00);                    // Apply the desired configuration to the register
  Wire.endTransmission();              // End the transmission

  // Configure the gyro's sensitivity
  Wire.beginTransmission(this->MPU_ADDRESS); // Start communication with MPU
  Wire.write(0x1B);                    // Request the GYRO_CONFIG register
  Wire.write(0x08);                    // Apply the desired configuration to the register : ±500°/s
  Wire.endTransmission();              // End the transmission

  // Configure the acceleromter's sensitivity
  Wire.beginTransmission(this->MPU_ADDRESS); // Start communication with MPU
  Wire.write(0x1C);                    // Request the ACCEL_CONFIG register
  Wire.write(0x10);                    // Apply the desired configuration to the register : ±8g
  Wire.endTransmission();              // End the transmission

  // Configure low pass filter
  Wire.beginTransmission(this->MPU_ADDRESS); // Start communication with MPU
  Wire.write(0x1A);                    // Request the CONFIG register
  Wire.write(0x03);                    // Set Digital Low Pass Filter about ~43Hz
  Wire.endTransmission();              // End the transmission

}
float* MPU6050::readSensor(float sensorReturn[6]) {
  float  gyroAngle[3];
  Wire.beginTransmission(this->MPU_ADDRESS); // Start communicating with the MPU-6050
  Wire.write(0x3B);                    // Send the requested starting register
  Wire.endTransmission();              // End the transmission
  Wire.requestFrom(this->MPU_ADDRESS, 14);   // Request 14 bytes from the MPU-6050

  // Wait until all the bytes are received
  while (Wire.available() < 14);
  float accRaw[3];
  float gyroRaw[3];
  float gyroRate[3];
  float accAngle[2];
  accRaw[X]  = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the accRaw[X] variable
  accRaw[Y]  = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the accRaw[Y] variable
  accRaw[Z]  = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the accRaw[Z] variable
  float temperature = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the temperature variable
  gyroRaw[X] = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the gyroRaw[X] variable
  gyroRaw[Y] = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the gyroRaw[Y] variable
  gyroRaw[Z] = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the gyroRaw[Z] variable
  // accel angle conversion
  // Calculate total 3D acceleration vector : √(X² + Y² + Z²)
  float  accMagnitude = sqrt(pow(accRaw[X], 2) + pow(accRaw[Y], 2) + pow(accRaw[Z], 2));
  // To prevent asin to produce a NaN, make sure the input value is within [-1;+1]
  if (abs(accRaw[X]) < accMagnitude) {
    accAngle[X] = asin((float)accRaw[Y] / accMagnitude) * (180 / PI); // asin gives angle in radian. Convert to degree multiplying by 180/pi
  }

  if (abs(accRaw[Y]) < accMagnitude) {
    accAngle[Y] = asin((float)accRaw[X] / accMagnitude) * (180 / PI);
  }
  // offset subtraction
  gyroRaw[X] -= gyroOffset[X];
  gyroRaw[Y] -= gyroOffset[Y];
  gyroRaw[Z] -= gyroOffset[Z];

  // Rate calculation
  gyroRate[X] = gyroRaw[X] / GyroConst;
  gyroRate[Y] = gyroRaw[Y] / GyroConst;
  gyroRate[Z] = gyroRaw[Z] / GyroConst;

  // Angle calculation using integration
  gyroAngle[X] += (gyroRaw[X] / (FREQ * GyroConst));
  gyroAngle[Y] += (-gyroRaw[Y] / (FREQ * GyroConst)); // Change sign to match the accelerometer's one
  gyroAngle[Z] += (-gyroRaw[Z] / (FREQ * GyroConst));

  // Transfer roll to pitch if IMU has yawed
  gyroAngle[Y] += gyroAngle[X] * sin(gyroRaw[Z] * (PI / (FREQ * GyroConst * 180)));
  gyroAngle[X] -= gyroAngle[Y] * sin(gyroRaw[Z] * (PI / (FREQ * GyroConst * 180)));
  //float& rategyro = gyroRate;
  sensorReturn[0] = gyroRate[0];
  sensorReturn[1] = gyroRate[1];
  sensorReturn[2] = gyroRate[2];
  sensorReturn[3] = accAngle[0];
  sensorReturn[4] = accAngle[1];
  sensorReturn[5] = gyroAngle[Z];

  return sensorReturn;
}

void MPU6050::resetGyroAngles() {
  gyroAngle[X] = accAngle[X];
  gyroAngle[Y] = accAngle[Y];
}
