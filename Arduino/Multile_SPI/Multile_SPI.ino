/*
Basic_SPI.ino
Roman Käslin
rkaeslin@student.ethz.ch
2017-05-14

adapted from
Brian R Taylor
brian.taylor@bolderflight.com
2016-10-10 

Copyright (c) 2016 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ICM20608G.h"
#include "AS5048A.h"
#include "AHRS.h"
#include "CommunicationUtils.h"

const int PinA = 9;
const int PinB = 10;
const int PinC = 15;

// an ICM20608G object with the ICM-20608-G sensor on Teensy Chip Select pin 10 & 9
ICM20608G IMUFoot(10);
ICM20608G IMUShank(9);
AS5048A ENCODER(15);
AHRS AHRSFoot(-0.0084, 0.0056, -0.0052);
AHRS AHRSShank(-0.0180, -0.0073, 0.0015);

const int number = 1;
float ax1[number], ay1[number], az1[number], gx1[number], gy1[number], gz1[number], t1[number];
float ax2[number], ay2[number], az2[number], gx2[number], gy2[number], gz2[number], t2[number];
float q01[number], q11[number], q21[number], q31[number];
float q02[number], q12[number], q22[number], q32[number];
float angle[number];
int beginStatus1;
int beginStatus2;
int beginStatus3;
int i = 0;
unsigned long stime;

Eigen::MatrixXf A(7,7);
Eigen::MatrixXf Q(3,3);
Eigen::MatrixXf H(3,7);
Eigen::MatrixXf R(3,3);
Eigen::MatrixXf K(7,3);
Eigen::MatrixXf P(7,7);
unsigned long lastUpdate, now1;
float q0, q1, q2, q3; // quaternion of sensor frame relative to auxiliary frame
float bx, by, bz;

void setup() {
  // serial to display data
  Serial.begin(250000);

  pinMode (PinA, OUTPUT);
  digitalWrite (PinA, HIGH);
  pinMode (PinB, OUTPUT);
  digitalWrite (PinB, HIGH);
  pinMode (PinC, OUTPUT);
  digitalWrite (PinC, HIGH);

  // start communication with IMU and 
  // set the accelerometer and gyro ranges.
  // ACCELEROMETER 2G 4G 8G 16G
  // GYRO 250DPS 500DPS 1000DPS 2000DPS
  beginStatus1 = IMUFoot.begin(ACCEL_RANGE_4G,GYRO_RANGE_250DPS);
  beginStatus2 = IMUShank.begin(ACCEL_RANGE_4G,GYRO_RANGE_250DPS);
  beginStatus3 = ENCODER.begin();

  IMUFoot.setFilt(GYRO_DLPF_BANDWIDTH_250HZ, ACCEL_BYPASS_DLPF_BANDWIDTH_1046HZ, 0);
  IMUShank.setFilt(GYRO_DLPF_BANDWIDTH_250HZ, ACCEL_BYPASS_DLPF_BANDWIDTH_1046HZ, 0);
  
  //define zero position of encoder
  //int state = ENCODER.setZero();
  uint16_t zeroPos = 3980;
  int state = ENCODER.setZeroPos(&zeroPos);
  Serial.print("Encoder set to zero; successfully (1) or not (0): ");
  Serial.println(state);

  float gyroNoise = 0.008*sqrt(8000)/180.0*M_PI;
  float gyroBias = 5.0/180.0*M_PI;
  float accelNoise = 0.00025*9.81*sqrt(4000);
  q0 = 1; q1 = 0; q2 = 0; q3 = 0;
  bx=-0.0084; by=0.0056; bz=-0.0052;
    P << 1.0/3.0, 0, 0, 0, 0, 0, 0,
         0, 1.0/3.0, 0, 0, 0, 0, 0,
         0, 0, 1.0/3.0, 0, 0, 0, 0,
         0, 0, 0, 1.0/3.0, 0, 0, 0,
         0, 0, 0, 0, gyroBias, 0, 0,
         0, 0, 0, 0, 0, gyroBias, 0, 
         0, 0, 0, 0, 0, 0, gyroBias;
    Q << gyroNoise, 0, 0,
         0, gyroNoise, 0,
         0, 0, gyroNoise;
    R << accelNoise, 0, 0,
         0, accelNoise, 0,
         0, 0, accelNoise;
  lastUpdate = micros();;
  
  stime = micros();
}

void loop() {
  if(beginStatus1 < 0) {
    delay(1000);
    Serial.println("IMUSole initialization unsuccessful");
    Serial.println("Check IMUSole wiring or try cycling power");
    delay(1000);
  }
  else{
    // get the temperature data (C)
    IMUFoot.getTemp(&t1[i]);                    // workaround to read out temperature first since first readout after AMS AS5048A does not work
    
     /* getMotion6 */
    // get both the accel (m/s/s) and gyro (rad/s) data
    IMUFoot.getMotion6(&ax1[i], &ay1[i], &az1[i], &gx1[i], &gy1[i], &gz1[i]); 

     /* getQuaternion */
    float q[4];
    //AHRSFoot.getQDCM(q, &ax1[i], &ay1[i], &az1[i], &gx1[i], &gy1[i], &gz1[i]);
    AHRSFoot.getQEKF(q, &ax1[i], &ay1[i], &az1[i], &gx1[i], &gy1[i], &gz1[i], &angle[i]);
    q01[i] = q[0];
    q11[i] = q[1];
    q21[i] = q[2];
    q31[i] = q[3];
  
  }
  if(beginStatus2 < 0) {
    delay(1000);
    Serial.println("IMUShank initialization unsuccessful");
    Serial.println("Check IMUShank wiring or try cycling power");
    delay(1000);
  }
  else{
    /* getMotion6 */
    // get both the accel (m/s/s) and gyro (rad/s) data
    IMUShank.getMotion6(&ax2[i], &ay2[i], &az2[i], &gx2[i], &gy2[i], &gz2[i]);

    /* getQuaternion */
    float q[4];
    float heading = 0;
    //AHRSShank.getQDCM(q, &ax2[i], &ay2[i], &az2[i], &gx2[i], &gy2[i], &gz2[i]);
    AHRSShank.getQEKF(q, &ax2[i], &ay2[i], &az2[i], &gx2[i], &gy2[i], &gz2[i], &heading);
    q02[i] = q[0];
    q12[i] = q[1];
    q22[i] = q[2];
    q32[i] = q[3];
}
  if(beginStatus3 < 0) {
    delay(1000);
    Serial.println("IENCODER initialization unsuccessful");
    Serial.println("Check ENCODER wiring or try cycling power");
    delay(1000);
  }
  else{
    int state = ENCODER.getAngle(&angle[i]);
    //Serial.println(state);
    //printData3();
  }

//  // Prediction
//  now1 = micros();
//  float sampleTime = (now1 - lastUpdate) / 1000000.0;
////  Serial.println(sampleTime,6);
//  lastUpdate = now1;
//  float t = sampleTime / 2.0;
//  A << 1, t*bx, t*by, t*bz, t*q1, t*q2, t*q3,
//       -t*bx, 1, -t*bz, t*by, -t*q0, t*q3, -t*q2,
//       -t*by, t*bz, 1, -t*bx, -t*q3, -t*q0, t*q1,
//       -t*bz, -t*by, t*bx, 1, t*q2, -t*q1, -t*q0,
//       0, 0, 0, 0, 1, 0, 0,
//       0, 0, 0 ,0, 0, 1, 0,
//       0, 0, 0 ,0, 0, 0, 1;
//       
//  Eigen::MatrixXf Quat(4,4);
//  Quat << q0, -q1, -q2, -q3, 
//          q1, q0, -q3, q2, 
//          q2, q3, q0, -q1, 
//          q3, -q2, q1, q0;
//
//  Eigen::VectorXf u(4);
//  Eigen::VectorXf omega(4);
//  omega << 0, gx1[i], gy1[i], gz1[i];
//  u = 0.5 * Quat * omega * sampleTime;
//
//  Eigen::VectorXf b(4);
//  Eigen::VectorXf bias(4);
//  bias << 0, bx, by, bz;
//  b = 0.5 * Quat * bias * sampleTime;
//
//  q0 = q0 - b(0) + u(0);
//  q1 = q1 - b(1) + u(1);
//  q2 = q2 - b(2) + u(2);
//  q3 = q3 - b(3) + u(3);
//  bx = bx,
//  by = by,
//  bz = bz;
//
//  // Normalise quaternion
//  float recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
//  q0 *= recipNorm;
//  q1 *= recipNorm;
//  q2 *= recipNorm;
//  q3 *= recipNorm;
////
////  Serial.println("x_prio: ");
////  Serial.print(q0);
////  Serial.print("\t");
////  Serial.print(q1);
////  Serial.print("\t");
////  Serial.print(q2);
////  Serial.print("\t");
////  Serial.print(q3);
////  Serial.print("\t");
////  Serial.print(bx);
////  Serial.print("\t");
////  Serial.print(by);
////  Serial.print("\t");
////  Serial.println(bz);
//  
//  Eigen::MatrixXf Quat73(7,3);
//  Quat73 << Quat.block<4,3>(0,1),
//            0.1, 0, 0,
//            0, 0.1, 0,
//            0, 0, 0.1;
//
//  P = A * P * A.transpose() + 0.25 * Quat73 * Q * Quat73.transpose();
//
////   Serial.println("P_prio: ");
////   for (int j=0; j<7; j++) {
////    for (int k=0; k<7; k++) {
////     Serial.print(P(j,k));
////     Serial.print("\t");
////    }
////    Serial.println("");
////   }
//
//   // Measurement Update
//   H << -2*q2,  2*q3, -2*q0,  2*q1, 0, 0, 0,
//         2*q1,  2*q0,  2*q3,  2*q2, 0, 0, 0,
//         2*q0, -2*q1, -2*q2,  2*q3, 0, 0, 0;
////   Serial.println("H: ");
////   for (int j=0; j<3; j++) {
////    for (int k=0; k<7; k++) {
////     Serial.print(H(j,k));
////     Serial.print("\t");
////    }
////    Serial.println("");
////   }
//
//   K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
//
////   Serial.println("K: ");
////   for (int j=0; j<7; j++) {
////    for (int k=0; k<3; k++) {
////     Serial.print(K(j,k),6);
////     Serial.print("\t");
////    }
////    Serial.println("");
////   }
//   
//   Eigen::VectorXf z(3);
//   float a = sqrt(ax1[i] * ax1[i] + ay1[i] * ay1[i] + az1[i] * az1[i]);
//   z << ax1[i] / a, ay1[i] / a, az1[i] / a;//, tan(psi);
////  Serial.print("a: ");
////  Serial.print(ax1[i]);
////  Serial.print("\t");
////  Serial.print(ay1[i]);
////  Serial.print("\t");
////  Serial.print(az1[i]);
////  Serial.print("\t");
////  Serial.println(a);
//    
////  Serial.println("z: ");
////  Serial.print(z(0),6);
////  Serial.print("\t");
////  Serial.print(z(1),6);
////  Serial.print("\t");
////  Serial.println(z(2),6);
//  
//   Eigen::VectorXf h(3);
//   h << 2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), q0*q0 - q1*q1 - q2*q2 + q3*q3;
////  Serial.println("h: ");
////  Serial.print(h(0),6);
////  Serial.print("\t");
////  Serial.print(h(1),6);
////  Serial.print("\t");
////  Serial.println(h(2),6);
//
//   Eigen::VectorXf x_prio(7);
//   x_prio << q0, q1, q2, q3, bx, by, bz;
//   Eigen::VectorXf x_post(7);
//   x_post = x_prio + K * (z-h);
//
//   Eigen::VectorXf update(7);
//   update = K * (z-h);
//
////  Serial.println("update: ");
////  Serial.print(update(0),6);
////  Serial.print("\t");
////  Serial.print(update(1),6);
////  Serial.print("\t");
////  Serial.print(update(2),6);
////  Serial.print("\t");
////  Serial.print(update(3),6);
////  Serial.print("\t");
////  Serial.print(update(4),6);
////  Serial.print("\t");
////  Serial.print(update(5),6);
////  Serial.print("\t");
////  Serial.println(update(6),6);
//
//   Eigen::MatrixXf I = Eigen::MatrixXf::Identity(7,7);
//   P = (I - K * H) * P;
//
////   Serial.println("P_post: ");
////   for (int j=0; j<7; j++) {
////    for (int k=0; k<7; k++) {
////     Serial.print(P(j,k));
////     Serial.print("\t");
////    }
////    Serial.println("");
////   }
//
//   // Store in Variables
//   q0 = x_post(0);
//   q1 = x_post(1);
//   q2 = x_post(2);
//   q3 = x_post(3);
//   bx = x_post(4);
//   by = x_post(5);
//   bz = x_post(6);
//
//  // Normalise quaternion
//  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
//  q0 *= recipNorm;
//  q1 *= recipNorm;
//  q2 *= recipNorm;
//  q3 *= recipNorm;
//  
////  Serial.println("x_post: ");
////  Serial.print(q0);
////  Serial.print("\t");
////  Serial.print(q1);
////  Serial.print("\t");
////  Serial.print(q2);
////  Serial.print("\t");
////  Serial.print(q3);
////  Serial.print("\t");
////  Serial.print(bx);
////  Serial.print("\t");
////  Serial.print(by);
////  Serial.print("\t");
////  Serial.println(bz);
//
//    q01[i] = q0;
//    q11[i] = q1;
//    q21[i] = q2;
//    q31[i] = q3;

  float q1[4] = {q01[i], q11[i],q21[i],q31[i]};
  float q2[4] = {q02[i], q12[i],q22[i],q32[i]};
  float angles1[3];
  quatToEul(q1, angles1);
  float angles2[3];
  quatToEul(q2, angles2);
  if (angle[i] > 180) { angle[i] -= 360; }
  angle[i] = angles1[0] + angle[i]*3.14159265359f/180.0f;
  if (angle[i] > 3.14159265359) { angle[i] -= 2*3.14159265359; }
  if (angle[i] < 3.14159265359) { angle[i] += 2*3.14159265359; }
  angles2[0] = angles2[0] - angle[i];
  angles2[1] = 0;
  angles2[2] = 0;
  float qcorr2[4];
  eulToQuat(angles2, qcorr2);
  float invqcorr2[4];
  invertQuat(qcorr2, invqcorr2);
  quatMult(invqcorr2, q2, q2);

  float invq2[4];
  invertQuat(q2, invq2);
  float qrel[4];
  quatMult(q1, invq2, qrel);
  
//  float angles[3];
//  quatToEul(qrel, angles);
//  angles[1] = 0;
//  angles[2] = 0;
//  float qcorr[4];
//  eulToQuat(angles, qcorr);
//  float invqcorr[4];
//  invertQuat(qcorr, invqcorr);
//  quatMult(qcorr, qrel, qrel);
  

//  float qyaw[4];
//  eulToQuat(angles, qyaw);
//  quatMult(qyaw, qrel, qrel);

  serialPrintFloatArr(q1, 4);
  Serial.println(""); //line break
  delay(100);

  i++;
  if (i == number){
    stime = micros() - stime;
    float sumgx2 = 0.0;
    float sumgy2 = 0.0;
    float sumgz2 = 0.0;
    unsigned long USBtime = micros();
    for (int j=0; j<number; j++) {
      int commas = 6;
//      Serial.print(ax1[j],commas);
//      Serial.print("\t");
//      Serial.print(ay1[j],commas);
//      Serial.print("\t");
//      Serial.print(az1[j],commas);
//      Serial.print("\t");
//      Serial.print(gx1[j],commas);
//      Serial.print("\t");
//      Serial.print(gy1[j],commas);
//      Serial.print("\t");
//      Serial.print(gz1[j],commas);
//      Serial.print("\t");

//      Serial.print(ax2[j],commas);
//      Serial.print("\t");
//      Serial.print(ay2[j],commas);
//      Serial.print("\t");
//      Serial.print(az2[j],commas);
//      Serial.print("\t");
//      Serial.print(gx2[j],commas);
//      Serial.print("\t");
//      Serial.print(gy2[j],commas);
//      Serial.print("\t");
//      Serial.print(gz2[j],commas);
//      Serial.println("\t");
//
//      Serial.println(angle[j],commas);
    }
    USBtime = micros() - USBtime;
      
//    float rate = float(number)/stime*1000000;
//    Serial.print("rate [Hz]: ");
//    Serial.println(rate);
//    float USBrate = float(number)/USBtime*1000000;
//    Serial.print("USB rate [Hz]: ");
//    Serial.println(USBrate);

//    double sum = 0;
//    double sum2 = 0;
//    sum += values[j];

    //delay(5000);
    stime = micros();
    i = 0;
  }
}
