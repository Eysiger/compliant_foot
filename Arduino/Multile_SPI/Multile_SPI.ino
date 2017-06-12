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
ICM20608G IMUFoot(PinB);
ICM20608G IMUShank(PinA);
AS5048A ENCODER(PinC);
//AHRS AHRSFoot(-0.0084, 0.0056, -0.0052);
//AHRS AHRSShank(-0.0180, -0.0073, 0.0015);
AHRS AHRS(-0.0084, 0.0056, -0.0052, -0.0180, -0.0073, 0.0015);

const int number = 1000;
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

Eigen::MatrixXf A(14,14);
Eigen::MatrixXf Q(6,6);
Eigen::MatrixXf H(7,14);
Eigen::MatrixXf R(7,7);
Eigen::MatrixXf K(14,7);
Eigen::MatrixXf P(14,14);
unsigned long lastUpdate, now1;
Eigen::VectorXf x(14);
Eigen::MatrixXf Quat2(4,4);
Eigen::MatrixXf Quat1(4,4);
Eigen::VectorXf u1(4);
Eigen::VectorXf omega1(4);
Eigen::VectorXf bias1(4);
Eigen::VectorXf u2(4);
Eigen::VectorXf omega2(4);
Eigen::VectorXf bias2(4);
Eigen::MatrixXf L(14,6);
Eigen::MatrixXf LQL(14,14);
Eigen::MatrixXf APA(14,14);
Eigen::VectorXf z(7);
Eigen::VectorXf h(7);
Eigen::MatrixXf I = Eigen::MatrixXf::Identity(14,14);
const float G = 9.807f;

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
  uint16_t zeroPos = 12172;
  int state = ENCODER.setZeroPos(&zeroPos);
  Serial.print("Encoder set to zero; successfully (1) or not (0): ");
  Serial.println(state);

  float gyroNoise = 0.008*sqrt(8000)/180.0*M_PI;
  float gyroBias = 5.0/180.0*M_PI;
  float accelNoise = 0.00025*9.81*sqrt(4000);
  float encoderNoise = 0.06/180*M_PI;
    x << 1, 0, 0, 0, -0.0084, 0.0056, -0.0052, 1, 0, 0, 0, -0.0180, -0.0073, 0.0015;
    P << 1.0/3.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 1.0/3.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 1.0/3.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 1.0/3.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, gyroBias, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, gyroBias, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, gyroBias, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 1.0/3.0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 1.0/3.0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0/3.0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0/3.0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, gyroBias, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, gyroBias, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, gyroBias;
    Q << gyroNoise, 0, 0, 0, 0, 0,
         0, gyroNoise, 0, 0, 0, 0,
         0, 0, gyroNoise, 0, 0, 0,
         0, 0, 0, gyroNoise, 0, 0,
         0, 0, 0, 0, gyroNoise, 0,
         0, 0, 0, 0, 0, gyroNoise;
    R << accelNoise, 0, 0, 0, 0, 0, 0,
         0, accelNoise, 0, 0, 0, 0, 0,
         0, 0, accelNoise, 0, 0, 0, 0,
         0, 0, 0, accelNoise, 0, 0, 0,
         0, 0, 0, 0, accelNoise, 0, 0,
         0, 0, 0, 0, 0, accelNoise, 0,
         0, 0, 0, 0, 0, 0, encoderNoise;
  lastUpdate = micros();
  
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
//    float q[4];
//    //AHRSFoot.getQDCM(q, &ax1[i], &ay1[i], &az1[i], &gx1[i], &gy1[i], &gz1[i]);
//    AHRSFoot.getQEKF(q, &ax1[i], &ay1[i], &az1[i], &gx1[i], &gy1[i], &gz1[i], &angle[i]);
//    q01[i] = q[0];
//    q11[i] = q[1];
//    q21[i] = q[2];
//    q31[i] = q[3];
  
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
//    float q[4];
//    float heading = 0;
//    //AHRSShank.getQDCM(q, &ax2[i], &ay2[i], &az2[i], &gx2[i], &gy2[i], &gz2[i]);
//    AHRSShank.getQEKF(q, &ax2[i], &ay2[i], &az2[i], &gx2[i], &gy2[i], &gz2[i], &heading);
//    q02[i] = q[0];
//    q12[i] = q[1];
//    q22[i] = q[2];
//    q32[i] = q[3];
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

  // Prediction
  Serial.print("start: ");
  Serial.println(micros() - stime);
  
  now1 = micros();
  float sampleTime = (now1 - lastUpdate) / 1000000.0;
//  Serial.println(sampleTime,6);
  lastUpdate = now1;
 
  Quat1 << x(0), -x(1), -x(2), -x(3), 
          x(1),  x(0), -x(3),  x(2), 
          x(2),  x(3),  x(0), -x(1), 
          x(3), -x(2),  x(1),  x(0);
          
  Quat2 << x(7), -x(8),  -x(9), -x(10), 
           x(8),  x(7), -x(10),   x(9), 
           x(9), x(10),   x(7),  -x(8), 
          x(10), -x(9),   x(8),   x(7);
          
  Serial.print("after Quat: ");
  Serial.println(micros() - stime);

  omega1 << 0, gx1[i], gy1[i], gz1[i];
  bias1 << 0, x(4), x(5), x(6);
  u1 = 0.5 * Quat1 * (omega1 - bias1) * sampleTime;

  omega2 << 0, gx2[i], gy2[i], gz2[i];
  bias2 << 0, x(11), x(12), x(13);
  u2 = 0.5 * Quat2 * (omega2 - bias2) * sampleTime;

  Serial.print("after u: ");
  Serial.println(micros() - stime);

  float t = sampleTime / 2.0;
  A <<       1,  t*x(4),  t*x(5),  t*x(6),  t*x(1),  t*x(2),  t*x(3),        0,        0,        0,        0,        0,        0,        0,
       -t*x(4),       1, -t*x(6),  t*x(5), -t*x(0),  t*x(3), -t*x(2),        0,        0,        0,        0,        0,        0,        0,
       -t*x(5),  t*x(6),       1, -t*x(4), -t*x(3), -t*x(0),  t*x(1),        0,        0,        0,        0,        0,        0,        0,
       -t*x(6), -t*x(5),  t*x(4),       1,  t*x(2), -t*x(1), -t*x(0),        0,        0,        0,        0,        0,        0,        0,
             0,       0,       0,       0,       1,       0,       0,        0,        0,        0,        0,        0,        0,        0,
             0,       0,       0,       0,       0,       1,       0,        0,        0,        0,        0,        0,        0,        0,
             0,       0,       0,       0,       0,       0,       1,        0,        0,        0,        0,        0,        0,        0,
             0,       0,       0,       0,       0,       0,       0,        1,  t*x(11),  t*x(12),  t*x(13),   t*x(8),   t*x(9),  t*x(10),
             0,       0,       0,       0,       0,       0,       0, -t*x(11),        1, -t*x(13),  t*x(12),  -t*x(7),  t*x(10),  -t*x(9),
             0,       0,       0,       0,       0,       0,       0, -t*x(12),  t*x(13),        1, -t*x(11), -t*x(10),  -t*x(7),   t*x(8),
             0,       0,       0,       0,       0,       0,       0, -t*x(13), -t*x(12),  t*x(11),        1,   t*x(9),  -t*x(8),  -t*x(7),
             0,       0,       0,       0,       0,       0,       0,        0,        0,        0,        0,        1,        0,        0,
             0,       0,       0,       0,       0,       0,       0,        0,        0,        0,        0,        0,        1,        0,
             0,       0,       0,       0,       0,       0,       0,        0,        0,        0,        0,        0,        0,        1;

  Serial.print("after A: ");
  Serial.println(micros() - stime);
  
  LQL <<         Quat1.block<4,3>(0,1)*Q.block<3,3>(0,0)*(Quat1.block<4,3>(0,1).transpose()), Eigen::MatrixXf::Zero(4,10),
                           Eigen::MatrixXf::Zero(3,14),
            Eigen::MatrixXf::Zero(4,7),      Quat2.block<4,3>(0,1)*Q.block<3,3>(3,3)*(Quat2.block<4,3>(0,1).transpose()), Eigen::MatrixXf::Zero(4,3),
                          Eigen::MatrixXf::Zero(3,14);

  Serial.print("after L: ");
  Serial.println(micros() - stime);
  
//   Serial.println("L: ");
//   for (int j=0; j<14; j++) {
//    for (int k=0; k<6; k++) {
//     Serial.print(L(j,k));
//     Serial.print("\t");
//    }
//    Serial.println("");
//   }

  APA << A.block<7,7>(0,0) * P.block<7,7>(0,0) * (A.block<7,7>(0,0).transpose()), Eigen::MatrixXf::Zero(7,7),
         Eigen::MatrixXf::Zero(7,7),                                              A.block<7,7>(7,7) * P.block<7,7>(7,7) * (A.block<7,7>(7,7).transpose());
  
  P = APA + 0.25 * LQL; //* Q * L.transpose();

  Serial.print("after P_prio: ");
  Serial.println(micros() - stime);

//   Serial.println("P_prio: ");
//   for (int j=0; j<14; j++) {
//    for (int k=0; k<14; k++) {
//     Serial.print(P(j,k));
//     Serial.print("\t");
//    }
//    Serial.println("");
//   }

 x(0) = x(0) + u1(0);
  x(1) = x(1) + u1(1);
  x(2) = x(2) + u1(2);
  x(3) = x(3) + u1(3);
  //x(4) = x(4);
  //x(5) = x(5);
  //x(6) = x(6);

  x(7)  =  x(7) + u2(0);
  x(8)  =  x(8) + u2(1);
  x(9)  =  x(9) + u2(2);
  x(10) = x(10) + u2(3);
  //x(11) = x(11);
  //x(12) = x(12);
  //x(13) = x(13);

//  Serial.println("x_prio: ");
//  for (int j=0; j<14; j++) {
//    Serial.print(x(j));
//    Serial.print("\t");
//  }
//  Serial.println("");

  Serial.print("after x_prio: ");
  Serial.println(micros() - stime);

  // Normalise quaternion1
  float recipNorm = invSqrt(x(0) * x(0) + x(1) * x(1) + x(2) * x(2) + x(3) * x(3));
  x(0) *= recipNorm;
  x(1) *= recipNorm;
  x(2) *= recipNorm;
  x(3) *= recipNorm;

  // Normalise quaternion2
  recipNorm = invSqrt(x(7) * x(7) + x(8) * x(8) + x(9) * x(9) + x(10) * x(10));
  x(7)  *= recipNorm;
  x(8)  *= recipNorm;
  x(9)  *= recipNorm;
  x(10) *= recipNorm;

  Serial.print("after normalization: ");
  Serial.println(micros() - stime);

   // Measurement Update
    float q0r = x(0)*x(7) + x(1)*x(8) + x(2)*x(9) + x(3)*x(10);
    float q1r = -x(0)*x(8) + x(1)*x(7) + x(2)*x(10) - x(3)*x(9);
    float q2r = -x(0)*x(9) - x(1)*x(10) + x(2)*x(7) + x(3)*x(8);
    float q3r = -x(0)*x(10) + x(1)*x(9) - x(2)*x(8) + x(3)*x(7);
    
    float pr  =  q0r*q0r + q1r*q1r - q2r*q2r - q3r*q3r;
    float p1  =  x(0)*x(0) + x(1)*x(1) - x(2)*x(2) - x(3)*x(3);
    float p2  =  x(7)*x(7) + x(8)*x(8) - x(9)*x(9) - x(10)*x(10);
    float p2_0_12_3 = x(7)*x(7) - x(8)*x(8) + x(9)*x(9) - x(10)*x(10);

    float fr_0312  =    q0r*q3r +    q1r*q2r; 
    float f1_0312  =  x(0)*x(3) +  x(1)*x(2);
    float f1_02_13 =  x(0)*x(2) -  x(1)*x(3);
    float f2_0312  = x(7)*x(10) +  x(8)*x(9);
    float f2_03_12 = x(7)*x(10) -  x(8)*x(9);
    float f2_02_13 =  x(7)*x(9) - x(8)*x(10);
    float f2_0123  =  x(7)*x(8) + x(9)*x(10); 

    float d01 = 4*( 2*f2_02_13*x(2) + 2*f2_0312*x(3) + p2*x(0))*fr_0312 + 2*( 2*f2_0123*x(2) + 2*f2_03_12*x(0) - p2_0_12_3*x(3))*pr;
    float d11 = 4*(-2*f2_02_13*x(3) + 2*f2_0312*x(2) + p2*x(1))*fr_0312 + 2*(-2*f2_0123*x(3) + 2*f2_03_12*x(1) - p2_0_12_3*x(2))*pr;
    float d21 = 4*( 2*f2_02_13*x(0) + 2*f2_0312*x(1) - p2*x(2))*fr_0312 + 2*( 2*f2_0123*x(0) - 2*f2_03_12*x(2) - p2_0_12_3*x(1))*pr;
    float d31 = 4*(-2*f2_02_13*x(1) + 2*f2_0312*x(0) - p2*x(3))*fr_0312 + 2*(-2*f2_0123*x(1) - 2*f2_03_12*x(3) - p2_0_12_3*x(0))*pr;
    
    float d02 = 4*(  2*f1_02_13*x(9) + 2*f1_0312*x(10) +  p1*x(7))*fr_0312 + 2*( 2*f1_02_13*x(8) -  2*f1_0312*x(7) + p1*x(10))*pr;
    float d12 = 4*(-2*f1_02_13*x(10) +  2*f1_0312*x(9) +  p1*x(8))*fr_0312 + 2*( 2*f1_02_13*x(7) +  2*f1_0312*x(8) - p1*x(9))*pr;
    float d22 = 4*(  2*f1_02_13*x(7) +  2*f1_0312*x(8) -  p1*x(9))*fr_0312 + 2*(2*f1_02_13*x(10) -  2*f1_0312*x(9) - p1*x(8))*pr;
    float d32 = 4*( -2*f1_02_13*x(8) +  2*f1_0312*x(7) - p1*x(10))*fr_0312 + 2*( 2*f1_02_13*x(9) + 2*f1_0312*x(10) + p1*x(7))*pr;
    
    float no = 4*fr_0312*fr_0312 + pr*pr;
    
//   float d01 =  2*( x(0)*x(0)*x(3) + 2*x(0)*x(1)*x(2) - x(1)*x(1)*x(3) + x(2)*x(2)*x(3) + x(3)*x(3)*x(3));
//   float d11 =  2*(-x(0)*x(0)*x(2) + 2*x(0)*x(1)*x(3) + x(1)*x(1)*x(2) + x(2)*x(2)*x(2) + x(2)*x(3)*x(3));
//   float d21 = -2*( x(0)*x(0)*x(1) + 2*x(0)*x(2)*x(3) + x(1)*x(1)*x(1) + x(2)*x(2)*x(1) - x(1)*x(3)*x(3));
//   float d31 = -2*( x(0)*x(0)*x(0) + 2*x(1)*x(2)*x(3) + x(1)*x(1)*x(0) - x(2)*x(2)*x(0) + x(0)*x(3)*x(3));
//   float no1 =  4*(x(0)*x(3) + x(1)*x(2))*(x(0)*x(3) + x(1)*x(2)) + (x(0)*x(0) + x(1)*x(1) - x(2)*x(2) - x(3)*x(3)) * (x(0)*x(0) + x(1)*x(1) - x(2)*x(2) - x(3)*x(3));
//
//   float d02 = -2*( x(7)*x(7)*x(10) +  2*x(7)*x(8)*x(9) - x(8)*x(8)*x(10) + x(9)*x(9)*x(10) + x(10)*x(10)*x(10));
//   float d12 =  -2*(-x(7)*x(7)*x(9) + 2*x(7)*x(8)*x(10) +  x(8)*x(8)*x(9) +  x(9)*x(9)*x(9) +  x(9)*x(10)*x(10));
//   float d22 =   2*( x(7)*x(7)*x(8) + 2*x(7)*x(9)*x(10) +  x(8)*x(8)*x(8) +  x(9)*x(9)*x(8) -  x(8)*x(10)*x(10));
//   float d32 =   2*( x(7)*x(7)*x(7) + 2*x(8)*x(9)*x(10) +  x(8)*x(8)*x(7) -  x(9)*x(9)*x(7) +  x(7)*x(10)*x(10));
//   float no2 = 4*(x(7)*x(10) + x(8)*x(9))*(x(7)*x(10) + x(8)*x(9)) + (x(7)*x(7) + x(8)*x(8) - x(9)*x(9) - x(10)*x(10)) * (x(7)*x(7) + x(8)*x(8) - x(9)*x(9) - x(10)*x(10));
   
   H << -2*x(2),  2*x(3), -2*x(0),  2*x(1), 0, 0, 0,       0,       0,       0,       0, 0, 0, 0,
         2*x(1),  2*x(0),  2*x(3),  2*x(2), 0, 0, 0,       0,       0,       0,       0, 0, 0, 0,
         2*x(0), -2*x(1), -2*x(2),  2*x(3), 0, 0, 0,       0,       0,       0,       0, 0, 0, 0,
              0,       0,       0,       0, 0, 0, 0, -2*x(9), 2*x(10), -2*x(7),  2*x(8), 0, 0, 0,
              0,       0,       0,       0, 0, 0, 0,  2*x(8),  2*x(7), 2*x(10),  2*x(9), 0, 0, 0,
              0,       0,       0,       0, 0, 0, 0,  2*x(7), -2*x(8), -2*x(9), 2*x(10), 0, 0, 0,
              //0,       0,       0,       0, 0, 0, 0,       0,       0,       0,       0, 0, 0, 0;
         d01/no,  d11/no,  d21/no,  d31/no, 0, 0, 0,  d02/no,  d12/no,  d22/no,  d32/no, 0, 0, 0;      
        //d01/no1, d11/no1, d21/no1, d31/no1, 0, 0, 0, d02/no2, d12/no2, d22/no2, d32/no2, 0, 0, 0;

  Serial.print("after H: ");
  Serial.println(micros() - stime);
  
//   Serial.println("H: ");
//   for (int j=0; j<7; j++) {
//    for (int k=0; k<14; k++) {
//     Serial.print(H(j,k));
//     Serial.print("\t");
//    }
//    Serial.println("");
//   }
   
   K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

  Serial.print("after K: ");
  Serial.println(micros() - stime);

//   Serial.println("mult: ");
//   for (int j=0; j<7; j++) {
//    for (int k=0; k<7; k++) {
//     Serial.print(mult(j,k),6);
//     Serial.print("\t");
//    }
//    Serial.println("");
//   }

//   Serial.println("K: ");
//   for (int j=0; j<14; j++) {
//    for (int k=0; k<7; k++) {
//     Serial.print(K(j,k),6);
//     Serial.print("\t");
//    }
//    Serial.println("");
//   }
   
   float a1 = sqrt(ax1[i] * ax1[i] + ay1[i] * ay1[i] + az1[i] * az1[i]);
   float a2 = sqrt(ax2[i] * ax2[i] + ay2[i] * ay2[i] + az2[i] * az2[i]);
   if (angle[i] > 180) { angle[i] -= 360; }
   angle[i] = angle[i]/180.0f*3.14159265359f;
   if ( (0.1*G > fabs(a1-G)) || (0.1*G > fabs(a2-G)) ) {
     z << ax1[i] / a1, ay1[i] / a1, az1[i] / a1, ax2[i] / a2, ay2[i] / a2, az2[i] / a2, angle[i];
   }
   else {
     z << 2*(x(1)*x(3) - x(0)*x(2)), 2*(x(2)*x(3) + x(0)*x(1)), x(0)*x(0) - x(1)*x(1) - x(2)*x(2) + x(3)*x(3),
          2*(x(8)*x(10) - x(7)*x(9)), 2*(x(9)*x(10) + x(7)*x(8)), x(7)*x(7) - x(8)*x(8) - x(9)*x(9) + x(10)*x(10),
          angle[i];
   }
   
  Serial.print("after z: ");
  Serial.println(micros() - stime);
  
//  Serial.print("a1: ");
//  Serial.print(ax1[i]);
//  Serial.print("\t");
//  Serial.print(ay1[i]);
//  Serial.print("\t");
//  Serial.print(az1[i]);
//  Serial.print("\t");
//  Serial.println(a1);
//
//  Serial.print("a2: ");
//  Serial.print(ax2[i]);
//  Serial.print("\t");
//  Serial.print(ay2[i]);
//  Serial.print("\t");
//  Serial.print(az2[i]);
//  Serial.print("\t");
//  Serial.println(a2);

//  Serial.println("qrel: ");
//    Serial.print(q0r);
//    Serial.print("\t");
//        Serial.print(q1r);
//    Serial.print("\t");
//        Serial.print(q2r);
//    Serial.print("\t");
//        Serial.print(q3r);
//    Serial.print("\t");
//  Serial.println("");
//
//  Serial.println("z: ");
//  for (int j=0; j<7; j++) {
//    Serial.print(z(j),6);
//    Serial.print("\t");
//  }
//  Serial.println("");
  
   h << 2*(x(1)*x(3) - x(0)*x(2)), 2*(x(2)*x(3) + x(0)*x(1)), x(0)*x(0) - x(1)*x(1) - x(2)*x(2) + x(3)*x(3),
        2*(x(8)*x(10) - x(7)*x(9)), 2*(x(9)*x(10) + x(7)*x(8)), x(7)*x(7) - x(8)*x(8) - x(9)*x(9) + x(10)*x(10),
        -atan2(2 * (q0r*q3r + q1r*q2r), (q0r*q0r + q1r*q1r - q2r*q2r - q3r*q3r));
        //-atan2(2*(x(3)*x(0) + x(1)*x(2)), x(0)*x(0)+x(1)*x(1)-x(2)*x(2)-x(3)*x(3)) + atan2(2*(x(10)*x(7) + x(8)*x(9)), x(7)*x(7)+x(8)*x(8)-x(9)*x(9)-x(10)*x(10));
   
   Serial.print("after h: ");
   Serial.println(micros() - stime);
   
//  Serial.println("h: ");
//  for (int j=0; j<7; j++) {
//    Serial.print(h(j),6);
//    Serial.print("\t");
//  }
//  Serial.println("");

   x = x + K * (z-h);

  Serial.print("after x_post: ");
  Serial.println(micros() - stime);

//   Eigen::VectorXf update(7);
//   update = K * (z-h);
//
//  Serial.println("update: ");
//  for (int j=0; j<14; j++) {
//    Serial.print(update(j),6);
//    Serial.print("\t");
//  }
//  Serial.println(""); 

   P = (I - K * H) * P;
   
  Serial.print("after P_post: ");
  Serial.println(micros() - stime);

//   Serial.println("P_post: ");
//   for (int j=0; j<14; j++) {
//    for (int k=0; k<14; k++) {
//     Serial.print(P(j,k));
//     Serial.print("\t");
//    }
//    Serial.println("");
//   }

  // Normalise quaternion1
  recipNorm = invSqrt(x(0) * x(0) + x(1) * x(1) + x(2) * x(2) + x(3) * x(3));
  x(0) *= recipNorm;
  x(1) *= recipNorm;
  x(2) *= recipNorm;
  x(3) *= recipNorm;

  // Normalise quaternion2
  recipNorm = invSqrt(x(7) * x(7) + x(8) * x(8) + x(9) * x(9) + x(10) * x(10));
  x(7)  *= recipNorm;
  x(8)  *= recipNorm;
  x(9)  *= recipNorm;
  x(10) *= recipNorm;

  Serial.print("after normalization: ");
  Serial.println(micros() - stime);
  
//  Serial.println("x_post: ");
//  for (int j=0; j<14; j++) {
//    Serial.print(x(j));
//    Serial.print("\t");
//  }
//  Serial.println(""); 

    q01[i] = x(0);
    q11[i] = x(1);
    q21[i] = x(2);
    q31[i] = x(3);

    q02[i] = x(7);
    q12[i] = x(8);
    q22[i] = x(9);
    q32[i] = x(10);
  float q1[4] = {q01[i], q11[i],q21[i],q31[i]};
  float q2[4] = {q02[i], q12[i],q22[i],q32[i]};

//  if (angle[i] > 180) { angle[i] -= 360; }
//  angle[i] = angle[i]/180.0f*3.14159265359f;
//  float q1[4];
//  float q2[4];
//  AHRS.getQEKF(q1, q2, &ax1[i], &ay1[i], &az1[i], &gx1[i], &gy1[i], &gz1[i], &ax2[i], &ay2[i], &az2[i], &gx2[i], &gy2[i], &gz2[i], &angle[i]);

  float invq2[4];
  invertQuat(q2, invq2);
  float qrel[4];
  quatMult(q1, invq2, qrel);

  Serial.print("after qrel: ");
  Serial.println(micros() - stime);

//    Serial.println("qrel: ");
//  for (int j=0; j<4; j++) {
//    Serial.print(qrel[j]);
//    Serial.print("\t");
//  }
//  Serial.println(""); 


//  serialPrintFloatArr(qrel, 4);
//  Serial.println(""); //line break
//  delay(100);

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
      
    float rate = float(number)/stime*1000000;
    Serial.print("rate [Hz]: ");
    Serial.println(rate);
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
