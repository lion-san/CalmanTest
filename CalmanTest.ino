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
  
#include <Wire.h> 
#include <Kalman.h>
#include <SparkFunLSM9DS1.h>
  
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf 
#define led 13
#define button 7
#define LSM9DS1_M  0x1E                 // SPIアドレス設定 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B                // SPIアドレス設定 if SDO_AG is LOW



LSM9DS1 imu;
Kalman kalmanX; // Create the Kalman instances 
Kalman kalmanY; 
    
/* IMU Data */
double accX, accY, accZ; 
double gyroX, gyroY, gyroZ; 

  
double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro only 
double compAngleX, compAngleY; // Calculated angle using a complementary filter 
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter 
  
uint32_t timer; 
uint8_t i2cData[14]; // Buffer for I2C data 
  
float roll, pitch; 
  
void setup() { 
  Serial.begin(115200); 
  Wire.begin(); 
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz 
/*  
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz 
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling 
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s 
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g 
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once 
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode 
  
  while (i2cRead(0x75, i2cData, 1)); 
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register 
    Serial.print(F("Error reading sensor")); 
    while (1); 
  } 
*/

  //=== LSM9DS1 Initialize =====================================
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress  = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;

  if (!imu.begin())              //センサ接続エラー時の表示
  {
    Serial.println(F("Failed to communicate with LSM9DS1."));
    while (1)
      ;
  }
  //=======================================================


  
  delay(100); // Wait for sensor to stabilize 
  
  /* Set kalman and gyro starting angle */
  imu.readGyro();
  imu.readAccel();
  imu.readMag();

  
  accX = imu.ax; 
  accY = imu.ay; 
  accZ = imu.az; 


  
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26 
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2 
  // It is then converted from radians to degrees 
#ifdef RESTRICT_PITCH // Eq. 25 and 26 
  roll  = atan2(accY, accZ) * RAD_TO_DEG; 
  pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG; 
#else // Eq. 28 and 29 
  roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG; 
  pitch = atan2(-accX, accZ) * RAD_TO_DEG; 
#endif
  
  kalmanX.setAngle(roll); // Set starting angle 
  kalmanY.setAngle(pitch); 
  gyroXangle = roll; 
  gyroYangle = pitch; 
  compAngleX = roll; 
  compAngleY = pitch; 
  
  pinMode(led, OUTPUT); 
  digitalWrite(led, HIGH); 
  pinMode(button, INPUT_PULLUP); 
  
  timer = micros(); 
} 
  
void loop() { 
  /* Update all the values */

  imu.readGyro();
  imu.readAccel();
  imu.readMag();

  
  accX = imu.ax; 
  accY = imu.ay; 
  accZ = imu.az; 

  
  gyroX = imu.gx; 
  gyroY = imu.gy; 
  gyroZ = imu.gz; 
  
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time 
  timer = micros(); 
  
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26 
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2 
  // It is then converted from radians to degrees 
#ifdef RESTRICT_PITCH // Eq. 25 and 26 
  roll  = atan2(accY, accZ) * RAD_TO_DEG;//+++++++++++++++++++++++ 
  pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG; 
#else // Eq. 28 and 29 
  roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG; 
  pitch = atan2(-accX, accZ) * RAD_TO_DEG; 
#endif
  
  double gyroXrate = gyroX / 131.0; // Convert to deg/s 
  double gyroYrate = gyroY / 131.0; // Convert to deg/s 
  double gyroZrate = gyroZ / 131.0; // Convert to deg/s 
  
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
  gyroZangle += gyroZrate * dt; 
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate 
  //gyroYangle += kalmanY.getRate() * dt; 
  
  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter 
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch; 
  
  // Reset the gyro angle when it has drifted too much 
  if (gyroXangle < -180 || gyroXangle > 180) 
    gyroXangle = kalAngleX; 
  if (gyroYangle < -180 || gyroYangle > 180) 
    gyroYangle = kalAngleY; 
  
  if(digitalRead(button) == LOW){ 
    gyroZangle = 0; 
  } 
  
    Serial.print("MadgwickAHRS: ");
    Serial.print("0.0");
    Serial.print(" ");
    Serial.print(kalAngleY);
    Serial.print(" ");
    Serial.println(kalAngleX);
  
  }
