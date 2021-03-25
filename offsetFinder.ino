/* This arduino sketch is used to find the Gyroscope reading offsets. 
You can just upload the code into arduino and read from the MPU6050. 
finally the gyro offset values will be displayed. You can set those values to 
MPU6050 objects. That will subtract the offset values to get the right values.
*/

#include <Wire.h>
#define X 0
#define Y 1
#define Z 2
float sum[3]={0,0,0};
void setup() {
  Serial.begin(115200);
  Serial.print(sum[X]);Serial.print("\t");
 
}
void loop() {
   for(int i=0;i<10000;i++)
  {
  float  gyroAngle[3];
  Wire.beginTransmission(0x68); // Start communicating with the MPU-6050
  Wire.write(0x3B);                    // Send the requested starting register
  Wire.endTransmission();              // End the transmission
  Wire.requestFrom(0x68, 14);   // Request 14 bytes from the MPU-6050
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
  //Serial.print(gyroRaw[Z]);Serial.print("\t");
sum[X]+=gyroRaw[X];
sum[Y]+=gyroRaw[Y];
sum[Z]+=gyroRaw[Z];
  }
  sum[X]/=10000;
  sum[Y]/=10000;
  sum[Z]/=10000;
  Serial.print(sum[X]);Serial.print("\t");
  Serial.print(sum[Y]);Serial.print("\t");
  Serial.print(sum[Z]);Serial.println("\t");
}
