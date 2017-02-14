#include <Kalman.h>  // Source: https://github.com/TKJElectronics/KalmanFilter
#include <Wire.h>

/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.
 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").
 Contact information
 -------------------
 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */


//#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double acclX, acclY, acclZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
byte MPUAddress = 0x68;

// TODO: Make calibration routine

//Replace original i2cRead and i2cWrite with these subroutines.  JGH
//Write a single byte to the MPU6050 over I2C.
void writeByte(byte device, byte reg, byte data)  {
  Wire.beginTransmission(device);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

////Write multiple bytes to the MPU6050 over I2C.  UNTESTED!
//void writeBytes(byte device, byte reg, byte *data, byte length)  {
//  Wire.beginTransmission(device);
//  Wire.write(reg);
//  Wire.write(data, length);
//  Wire.endTransmission();
//}

//Read a single byte from the MPU6050 over I2C.
byte readByte(int device, byte reg)  {
  byte result;
  Wire.beginTransmission((byte)device);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(device, 1);
  while (!Wire.available()) {}
  result = Wire.read();
  return result;
}

//Read multiple bytes from the MPU6050 over I2C.
void readBytes(byte device, byte reg, byte count)  {
  Wire.beginTransmission(device);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(device, count);
  while (Wire.available()<count) {}
  //bytes are in the buffer, so use Wire.read() to get them.
  for (int i=0; i<count; i++) i2cData[i] = Wire.read(); //Fill the i2cData buffer.
}
//=======================================
void setup() {
  Serial.begin(115200);
  while (Serial.available()) Serial.read();  //clear the buffer.
  Wire.begin();
  //TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

//  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
//  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
//  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
//  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
//  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
//  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
//
//  while (i2cRead(0x75, i2cData, 1));
//  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
//    Serial.print(F("Error reading sensor"));
//    while (1);
//  }
  delay(10);

  writeByte(MPUAddress, 0x19, 7);  //Sample rate divider.
  writeByte(MPUAddress, 0x1A, 6);  //DLPF.
  writeByte(MPUAddress, 0x1B, 0);  //Gyro range.
  writeByte(MPUAddress, 0x1C, 0);  //Accel range.
  writeByte(MPUAddress, 0x6B, 1);  //PWR_MGMT_1 => 0, and PLL with X axis gyro reference.
  writeByte(MPUAddress, 0x23, 0);  //Disable FIFO.
  writeByte(MPUAddress, 0x3B, 1);  //Interrupt Enable.

  Serial.print("I am address: ");  Serial.println(readByte(MPUAddress, 0x75));
  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while ((readByte(MPUAddress, 58) & 1) == 0) {} //Wait for data ready.
  readBytes(MPUAddress, 0x3B, 6);  //read accel data only.
  acclX = (i2cData[0] << 8) | i2cData[1];
  acclY = (i2cData[2] << 8) | i2cData[3];
  acclZ = (i2cData[4] << 8) | i2cData[5];

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(acclY, acclZ) * RAD_TO_DEG;
  double pitch = atan(-acclX / sqrt(acclY * acclY + acclZ * acclZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(acclY / sqrt(acclX * acclX + acclZ * acclZ)) * RAD_TO_DEG;
  double pitch = atan2(-acclX, acclZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}

void loop() {
  /* Update all the values */
  while ((readByte(MPUAddress, 58) & 1) == 0) {} //Wait for data ready.
  readBytes(MPU6050, 0x3B; 14);
  acclX = ((i2cData[0] << 8) | i2cData[1]);
  acclY = ((i2cData[2] << 8) | i2cData[3]);
  acclZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(acclY, acclZ) * RAD_TO_DEG;
  double pitch = atan(-acclX / sqrt(acclY * acclY + acclZ * acclZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(acclY / sqrt(acclX * acclX + acclZ * acclZ)) * RAD_TO_DEG;
  double pitch = atan2(-acclX, acclZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print(acclX,3); Serial.print("\t");
  Serial.print(acclY,3); Serial.print("\t");
  Serial.print(acclZ,3); Serial.print("\t");
  Serial.print(gyroX,3); Serial.print("\t");
  Serial.print(gyroY,3); Serial.print("\t");
  Serial.print(gyroZ,3); Serial.print("\t");
  Serial.print("\t");
#endif

  Serial.print(roll,3); Serial.print("\t");
  Serial.print(gyroXangle,3); Serial.print("\t");
  Serial.print(compAngleX,3); Serial.print("\t");
  Serial.print(kalAngleX,3); Serial.print("\t");

  Serial.print("\t");

  Serial.print(pitch,3); Serial.print("\t");
  Serial.print(gyroYangle,3); Serial.print("\t");
  Serial.print(compAngleY,3); Serial.print("\t");
  Serial.print(kalAngleY,3); Serial.print("\t");

#if 0 // Set to 1 to print the temperature
  Serial.print("\t");
  double temperature = (double)tempRaw / 340.0 + 36.53;
  Serial.print(temperature); Serial.print("\t");
#endif

  Serial.print("\r\n");
  delay(2);
}

