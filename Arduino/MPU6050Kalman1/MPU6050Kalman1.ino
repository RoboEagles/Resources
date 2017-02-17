/* This version moves the Kalman filter class directly into the source code, 
 * because we couldn't get the library to work correctly.  This one works
 * as expected.  JGH  2/15/2017
*/

//#include <Kalman.h>  // Source: https://github.com/TKJElectronics/KalmanFilter
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
//================================================================================
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

//#ifndef _Kalman_h
//#define _Kalman_h

//#include "Arduino.h"

class Kalman {
public:
    Kalman() {
        /* We will set the variables like so, these can also be tuned by the user */
        Q_angle = 0.001;
        Q_bias = 0.003;
        R_measure = 0.03;

        angle = 0; // Reset the angle
        bias = 0; // Reset bias

        P[0][0] = 0; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
        P[0][1] = 0;
        P[1][0] = 0;
        P[1][1] = 0;
    };
    // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
    double getAngle(double newAngle, double newRate, double dt) {
        // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
        // Modified by Kristian Lauszus
        // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

        // Discrete Kalman filter time update equations - Time Update ("Predict")
        // Update xhat - Project the state ahead
        /* Step 1 */
        rate = newRate - bias;
        angle += dt * rate;

        // Update estimation error covariance - Project the error covariance ahead
        /* Step 2 */
        P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;

        // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
        // Calculate Kalman gain - Compute the Kalman gain
        /* Step 4 */
        S = P[0][0] + R_measure;
        /* Step 5 */
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;

        // Calculate angle and bias - Update estimate with measurement zk (newAngle)
        /* Step 3 */
        y = newAngle - angle;
        /* Step 6 */
        angle += K[0] * y;
        bias += K[1] * y;

        // Calculate estimation error covariance - Update the error covariance
        /* Step 7 */
        P[0][0] -= K[0] * P[0][0];
        P[0][1] -= K[0] * P[0][1];
        P[1][0] -= K[1] * P[0][0];
        P[1][1] -= K[1] * P[0][1];

        return angle;
    };
    void setAngle(double newAngle) { angle = newAngle; }; // Used to set angle, this should be set as the starting angle
    double getRate() { return rate; }; // Return the unbiased rate

    /* These are used to tune the Kalman filter */
    void setQangle(double newQ_angle) { Q_angle = newQ_angle; };
    void setQbias(double newQ_bias) { Q_bias = newQ_bias; };
    void setRmeasure(double newR_measure) { R_measure = newR_measure; };

    double getQangle() { return Q_angle; };
    double getQbias() { return Q_bias; };
    double getRmeasure() { return R_measure; };

private:
    /* Kalman filter variables */
    double Q_angle; // Process noise variance for the accelerometer
    double Q_bias; // Process noise variance for the gyro bias
    double R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    double angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    double bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    double rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    double P[2][2]; // Error covariance matrix - This is a 2x2 matrix
    double K[2]; // Kalman gain - This is a 2x1 vector
    double y; // Angle difference
    double S; // Estimate error
};

//#endif

//================================================================================
//#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double acclX, acclY, acclZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double cmpAngX, cmpAngY; // Calculated angle using a complementary filter
double kalAngX, kalAngY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
byte MPUAddress = 0x68;
int lines = 0;

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
ithub
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
  Serial.println();
  
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

  Serial.print("I am address: ");  Serial.print(readByte(MPUAddress, 0x75),HEX); Serial.println(F(" Hex"));
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
  cmpAngX = roll;
  cmpAngY = pitch;

#if 1 // Set to 1 to activate
  Serial.print(F("dt     ")); Serial.print(F("\t"));
  Serial.print(F("acclX  ")); Serial.print(F("\t"));
  Serial.print(F("acclY  ")); Serial.print(F("\t"));
  Serial.print(F("acclZ  ")); Serial.print(F("\t"));
  Serial.print(F("gyroX  ")); Serial.print(F("\t"));
  Serial.print(F("gyroY  ")); Serial.print(F("\t"));
  Serial.print(F("gyroZ  ")); Serial.print(F("\t"));
  Serial.print(F("\t"));
#endif

  Serial.print(F("roll   ")); Serial.print(F("\t"));
  Serial.print(F("Xangle ")); Serial.print(F("\t"));
  Serial.print(F("cmpAngX")); Serial.print(F("\t"));
  Serial.print(F("kalAngX")); Serial.print(F("\t"));

  Serial.print(F("\t"));

  Serial.print(F("pitch ")); Serial.print(F("\t"));
  Serial.print(F("Yangle")); Serial.print(F("\t"));
  Serial.print(F("cmpAngY")); Serial.print(F("\t"));
  Serial.print(F("kalAngY")); Serial.print(F("\t"));
  Serial.println();

  timer = micros();
}

void loop() {
  /* Update all the values */
  while ((readByte(MPUAddress, 58) & 1) == 0) {} //Wait for data ready.
  readBytes(MPUAddress, 0x3B, 14);
//  for (int j=0; j<14; j++) {
//    Serial.print(i2cData[j],HEX);  Serial.print(F(" "));  if ((j % 2) == 1) Serial.print(F("   "));
//  }
//  Serial.println();
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
  if ((roll < -90 && kalAngX > 90) || (roll > 90 && kalAngX < -90)) {
    kalmanX.setAngle(roll);
    cmpAngX = roll;
    kalAngX = roll;
    gyroXangle = roll;
  } else
    kalAngX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngY > 90) || (pitch > 90 && kalAngY < -90)) {
    kalmanY.setAngle(pitch);
    cmpAngY = pitch;
    kalAngY = pitch;
    gyroYangle = pitch;
  } else
    kalAngY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  cmpAngX = 0.93 * (cmpAngX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  cmpAngY = 0.93 * (cmpAngY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngY;

  /* Print Data */
  if (lines < 250) {
#if 1 // Set to 1 to activate
  Serial.print(dt,3); Serial.print(F("\t"));
  Serial.print(acclX/16384.0,3); Serial.print(F("\t"));
  Serial.print(acclY/16384.0,3); Serial.print(F("\t"));
  Serial.print(acclZ/16384.0,3); Serial.print(F("\t"));
  Serial.print(gyroX/131.0,3); Serial.print(F("\t"));
  Serial.print(gyroY/131.0,3); Serial.print(F("\t"));
  Serial.print(gyroZ/131.0,3); Serial.print(F("\t"));
  Serial.print(F("\t"));
#endif

  Serial.print(roll,4); Serial.print(F("\t"));
  Serial.print(gyroXangle,4); Serial.print(F("\t"));
  Serial.print(cmpAngX,4); Serial.print(F("\t"));
  Serial.print(kalAngX,4); Serial.print(F("\t"));
  Serial.print(F("\t"));

  Serial.print(pitch,4); Serial.print(F("\t"));
  Serial.print(gyroYangle,4); Serial.print(F("\t"));
  Serial.print(cmpAngY,4); Serial.print(F("\t"));
  Serial.print(kalAngY,4); Serial.print(F("\t"));

#if 0 // Set to 1 to print the temperature
  Serial.print(F("\t"));
  double temperature = (double)tempRaw / 340.0 + 36.53;
  Serial.print(temperature); Serial.print(F("\t"));
#endif

  Serial.print(F("\r\n"));
  Serial.flush();
  lines += 1;
  delay(2);
  }
}

