/*
AHRS.cpp
Roman Käslin
rkaeslin@student.ethz.ch
2017-05-14

adapted from

*/

#include "AHRS.h"

AHRS::AHRS(float bx1, float by1, float bz1, float bx2, float by2, float bz2) {
    x << 1, 0, 0, 0, bx1, by1, bz1, 1, 0, 0, 0, bx2, by2, bz2;
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
    I = Eigen::MatrixXf::Identity(14,14);
    integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;
    lastUpdate = micros();
}

void AHRS::EKFupdate(float* ax1, float* ay1, float* az1, float* gx1, float* gy1, float* gz1, float* ax2, float* ay2, float* az2, float* gx2, float* gy2, float* gz2, float* psi) {
    // Prediction

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

    Eigen::MatrixXf Quat1(4,4);
    Quat1 << x(0), -x(1), -x(2), -x(3), 
             x(1),  x(0), -x(3),  x(2), 
             x(2),  x(3),  x(0), -x(1), 
             x(3), -x(2),  x(1),  x(0);

    Eigen::MatrixXf Quat2(4,4);
    Quat2 <<  x(7), -x(8),  -x(9), -x(10), 
              x(8),  x(7), -x(10),   x(9), 
              x(9), x(10),   x(7),  -x(8), 
             x(10), -x(9),   x(8),   x(7);

    Eigen::VectorXf u1(4);
    Eigen::VectorXf omega1(4);
    omega1 << 0, *gx1, *gy1, *gz1;
    u1 = 0.5 * Quat1 * omega1 * sampleTime;

    Eigen::VectorXf u2(4);
    Eigen::VectorXf omega2(4);
    omega2 << 0, *gx2, *gy2, *gz2;
    u2 = 0.5 * Quat2 * omega2 * sampleTime;

    Eigen::VectorXf b1(4);
    Eigen::VectorXf bias1(4);
    bias1 << 0, x(4), x(5), x(6);
    b1 = 0.5 * Quat1 * bias1 * sampleTime;

    Eigen::VectorXf b2(4);
    Eigen::VectorXf bias2(4);
    bias2 << 0, x(11), x(12), x(13);
    b2 = 0.5 * Quat2 * bias2 * sampleTime;

    L <<         Quat1.block<4,3>(0,1), Eigen::MatrixXf::Zero(4,3),
                         0.1,   0,   0,                0,   0,   0, 
                           0, 0.1,   0,                0,   0,   0,
                           0,   0, 0.1,                0,   0,   0,
            Eigen::MatrixXf::Zero(4,3),      Quat2.block<4,3>(0,1),
                           0,   0,   0,              0.1,   0,   0,
                           0,   0,   0,                0, 0.1,   0,
                           0,   0,   0,                0,   0, 0.1;

    x(0) = x(0) - b1(0) + u1(0);
    x(1) = x(1) - b1(1) + u1(1);
    x(2) = x(2) - b1(2) + u1(2);
    x(3) = x(3) - b1(3) + u1(3);
    //x(4) = x(4);
    //x(5) = x(5);
    //x(6) = x(6);
    x(7)  =  x(7) - b2(0) + u2(0);
    x(8)  =  x(8) - b2(1) + u2(1);
    x(9)  =  x(9) - b2(2) + u2(2);
    x(10) = x(10) - b2(3) + u2(3);
    //x(11) = x(11);
    //x(12) = x(12);
    //x(13) = x(13);

    P = A * P * A.transpose() + 0.25 * L * Q * L.transpose();

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

    // Measurement Update

    float d01 = -2*( x(0)*x(0)*x(3) + 2*x(0)*x(1)*x(2) - x(1)*x(1)*x(3) + x(2)*x(2)*x(3) + x(3)*x(3)*x(3));
    float d11 = -2*(-x(0)*x(0)*x(2) + 2*x(0)*x(1)*x(3) + x(1)*x(1)*x(2) + x(2)*x(2)*x(2) + x(2)*x(3)*x(3));
    float d21 =  2*( x(0)*x(0)*x(1) + 2*x(0)*x(2)*x(3) + x(1)*x(1)*x(1) + x(2)*x(2)*x(1) - x(1)*x(3)*x(3));
    float d31 =  2*( x(0)*x(0)*x(0) + 2*x(1)*x(2)*x(3) + x(1)*x(1)*x(0) - x(2)*x(2)*x(0) + x(0)*x(3)*x(3));
    float no1 = (x(0)*x(0) + x(1)*x(1) - x(2)*x(2) - x(3)*x(3)) * (x(0)*x(0) + x(1)*x(1) - x(2)*x(2) - x(3)*x(3));

    float d02 = 2*( x(7)*x(7)*x(10) +  2*x(7)*x(8)*x(9) - x(8)*x(8)*x(10) + x(9)*x(9)*x(10) + x(10)*x(10)*x(10));
    float d12 =  2*(-x(7)*x(7)*x(9) + 2*x(7)*x(8)*x(10) +  x(8)*x(8)*x(9) +  x(9)*x(9)*x(9) +  x(9)*x(10)*x(10));
    float d22 = -2*( x(7)*x(7)*x(8) + 2*x(7)*x(9)*x(10) +  x(8)*x(8)*x(8) +  x(9)*x(9)*x(8) -  x(8)*x(10)*x(10));
    float d32 = -2*( x(7)*x(7)*x(7) + 2*x(8)*x(9)*x(10) +  x(8)*x(8)*x(7) -  x(9)*x(9)*x(7) +  x(7)*x(10)*x(10));
    float no2 = (x(7)*x(7) + x(8)*x(8) - x(9)*x(9) - x(10)*x(10)) * (x(7)*x(7) + x(8)*x(8) - x(9)*x(9) - x(10)*x(10));

    H << -2*x(2),  2*x(3), -2*x(0),  2*x(1), 0, 0, 0,       0,       0,       0,       0, 0, 0, 0,
          2*x(1),  2*x(0),  2*x(3),  2*x(2), 0, 0, 0,       0,       0,       0,       0, 0, 0, 0,
          2*x(0), -2*x(1), -2*x(2),  2*x(3), 0, 0, 0,       0,       0,       0,       0, 0, 0, 0,
               0,       0,       0,       0, 0, 0, 0, -2*x(9), 2*x(10), -2*x(7),  2*x(8), 0, 0, 0,
               0,       0,       0,       0, 0, 0, 0,  2*x(8),  2*x(7), 2*x(10),  2*x(9), 0, 0, 0,
               0,       0,       0,       0, 0, 0, 0,  2*x(7), -2*x(8), -2*x(9), 2*x(10), 0, 0, 0,
         d01/no1, d11/no1, d21/no1, d31/no1, 0, 0, 0, d02/no2, d12/no2, d22/no2, d32/no2, 0, 0, 0;

    K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

    float a1 = sqrt(*ax1 * *ax1 + *ay1 * *ay1 + *az1 * *az1);
    float a2 = sqrt(*ax2 * *ax2 + *ay2 * *ay2 + *az2 * *az2);
    z << *ax1 / a1, *ay1 / a1, *az1 / a1, *ax2 / a2, *ay2 / a2, *az2 / a2, tan(*psi);
    h <<  2*(x(1)*x(3) - x(0)*x(2)), 2*(x(2)*x(3) + x(0)*x(1)), x(0)*x(0) - x(1)*x(1) - x(2)*x(2) + x(3)*x(3),
         2*(x(8)*x(10) - x(7)*x(9)), 2*(x(9)*x(10) + x(7)*x(8)), x(7)*x(7) - x(8)*x(8) - x(9)*x(9) + x(10)*x(10),
          2*(x(3)*x(0) + x(1)*x(2)) / (x(0)*x(0) + x(1)*x(1) - x(2)*x(2) - x(3)*x(3)) - 2*(x(10)*x(7) + x(8)*x(9)) / (x(7)*x(7) + x(8)*x(8) - x(9)*x(9) - x(10)*x(10));

    x = x + K * (z-h);

    P = (I - K * H) * P;

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
}

void AHRS::getQEKF(float* q1, float* q2, float* ax1, float* ay1, float* az1, float* gx1, float* gy1, float* gz1, float* ax2, float* ay2, float* az2, float* gx2, float* gy2, float* gz2, float* psi) {  
  now = micros();
  sampleTime = (now - lastUpdate) / 1000000.0;
  lastUpdate = now;

  EKFupdate(ax1, ay1, az1, gx1, gy1, gz1, ax2, ay2, az2, gx2, gy2, gz2, psi);
  q1[0] = x(0);
  q1[1] = x(1);
  q1[2] = x(2);
  q1[3] = x(3);

  q2[0] = x(7);
  q2[1] = x(8);
  q2[2] = x(9);
  q2[3] = x(10);
}

void AHRS::DCMupdate(float* ax, float* ay, float* az, float* gx, float* gy, float* gz) {
  float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
  float qa, qb, qc;

  // Auxiliary variables to avoid repeated arithmetic
  q0q0 = q0 * q0;
  q0q1 = q0 * q1;
  q0q2 = q0 * q2;
  q0q3 = q0 * q3;
  q1q1 = q1 * q1;
  q1q2 = q1 * q2;
  q1q3 = q1 * q3;
  q2q2 = q2 * q2;
  q2q3 = q2 * q3;
  q3q3 = q3 * q3;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if((*ax != 0.0f) && (*ay != 0.0f) && (*az != 0.0f)) {
    float halfvx, halfvy, halfvz;
    
    // Normalise accelerometer measurement
    recipNorm = invSqrt(*ax * *ax + *ay * *ay + *az * *az);
    *ax *= recipNorm;
    *ay *= recipNorm;
    *az *= recipNorm;
    
    // Estimated direction of gravity
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;
  
    // Error is sum of cross product between estimated direction and measured direction of field vectors
    halfex += (*ay * halfvz - *az * halfvy);
    halfey += (*az * halfvx - *ax * halfvz);
    halfez += (*ax * halfvy - *ay * halfvx);
  }

  // Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
  if(halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f) {
      integralFBx += twoKi * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / sampleFreq);
      integralFBz += twoKi * halfez * (1.0f / sampleFreq);
      *gx += integralFBx;  // apply integral feedback
      *gy += integralFBy;
      *gz += integralFBz;
    }
    else {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    *gx += twoKp * halfex;
    *gy += twoKp * halfey;
    *gz += twoKp * halfez;
  }
  
  // Integrate rate of change of quaternion
  *gx *= (0.5f * (1.0f / sampleFreq));   // pre-multiply common factors
  *gy *= (0.5f * (1.0f / sampleFreq));
  *gz *= (0.5f * (1.0f / sampleFreq));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * *gx - qc * *gy - q3 * *gz);
  q1 += (qa * *gx + qc * *gz - q3 * *gy);
  q2 += (qa * *gy - qb * *gz + q3 * *gx);
  q3 += (qa * *gz + qb * *gy - qc * *gx);
  
  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void AHRS::getQDCM(float* q, float* ax, float* ay, float* az, float* gx, float* gy, float* gz) {  
  now = micros();
  sampleFreq = 1.0 / ((now - lastUpdate) / 1000000.0);
  lastUpdate = now;

  DCMupdate(ax, ay, az, gx, gy, gz);
  q[0] = q0;
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;
}

void quatToEul(float* q, float* angles) {
  angles[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1); // psi
  angles[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]); // theta
  angles[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1); // phi
  // angles[0] = atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1); // psi   //wikipedia definition
  // angles[1] = asin(-2 * q[1] * q[3] + 2 * q[0] * q[2]); // theta
  // angles[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1); // phi
}

void eulToQuat(float* angles, float* q){
  float c2 = cos(angles[2]/2);
  float s2 = sin(angles[2]/2);
  float c1 = cos(angles[1]/2);
  float s1 = sin(angles[1]/2);
  float c0 = cos(angles[0]/2);
  float s0 = sin(angles[0]/2);

  q[0] = c2*c1*c0 + s2*s1*s0;
  q[1] = - s2*c1*c0 + c2*s1*s0;
  q[2] = - c2*s1*c0 - s2*c1*s0;
  q[3] = - c2*c1*s0 + s2*s1*c0;
  // q[0] = c2*c1*c0 + s2*s1*s0;    //wikipedia definition
  // q[1] = s2*c1*c0 - c2*s1*s0;
  // q[2] = c2*s1*c0 + s2*c1*s0;
  // q[3] = c2*c1*s0 - s2*s1*c0;
}

void getAngles(float* q, float* angles) {
  float a[3]; //Euler
  quatToEul(q, a);

  angles[0] = a[0];
  angles[1] = a[1];
  angles[2] = a[2];
  
  if(angles[0] < 0)angles[0] += 2*M_PI;
  if(angles[1] < 0)angles[1] += 2*M_PI;
  if(angles[2] < 0)angles[2] += 2*M_PI;
}

void getYawPitchRoll(float* q, float* ypr) {
  float gxhat, gyhat, gzhat; // estimated gravity direction
  
  gxhat = 2 * (q[1]*q[3] - q[0]*q[2]);
  gyhat = 2 * (q[0]*q[1] + q[2]*q[3]);
  gzhat = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
  
  ypr[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1);
  ypr[1] = atan(gxhat / sqrt(gyhat*gyhat + gzhat*gzhat));
  ypr[2] = atan(gyhat / sqrt(gxhat*gxhat + gzhat*gzhat));
}

float invSqrt(float number) {
  volatile long i;
  volatile float x, y;
  volatile const float f = 1.5F;

  x = number * 0.5F;
  y = number;
  i = * ( long * ) &y;
  i = 0x5f375a86 - ( i >> 1 );
  y = * ( float * ) &i;
  y = y * ( f - ( x * y * y ) );
  return y;
} 

void quatMult(float q[4], float p[4], float r[4]) {
  // Input: two quaternions to be multiplied
  // Output: output of the multiplication
  float temp[4];
  // JPL
  temp[0] = -q[1]*p[1] - q[2]*p[2] - q[3]*p[3] + q[0]*p[0];
  temp[1] = q[0]*p[1] + q[3]*p[2] - q[2]*p[3] + q[1]*p[0];
  temp[2] = -q[3]*p[1] + q[0]*p[2] + q[1]*p[3] + q[2]*p[0];
  temp[3] = q[2]*p[1] - q[1]*p[2] + q[0]*p[3] + q[3]*p[0];
  r[0] = temp[0];
  r[1] = temp[1];
  r[2] = temp[2];
  r[3] = temp[3];
}  

void invertQuat(float q[4], float r[4]) {
  r[0] = q[0];
  r[1] = -q[1];
  r[2] = -q[2];
  r[3] = -q[3];
}

void quatToRotMat(float q[4], Eigen::MatrixXf& Rot){
  Rot(0,0) = 2*q[0]*q[0] - 1 + 2*q[1]*q[1];
  Rot(0,1) = 2*q[0]*q[3] + 2*q[1]*q[2];
  Rot(0,2) = -2*q[0]*q[2] + 2*q[1]*q[3];
  Rot(1,0) = -2*q[0]*q[3] + 2*q[2]*q[1];
  Rot(1,1) = 2*q[0]*q[0] - 1  + 2*q[2]*q[2];
  Rot(1,2) = 2*q[0]*q[1] + 2*q[2]*q[3];
  Rot(2,0) = 2*q[0]*q[2] + 2*q[3]*q[1];
  Rot(2,1) = -2*q[0]*q[1] + 2*q[3]*q[2];
  Rot(2,2) = 2*q[0]*q[0] - 1  + 2*q[3]*q[3];
}

void rotMatToQuat(Eigen::MatrixXf& Rot, float q[4]){
  q[0] = sqrt(1+Rot(0,0)+Rot(1,1)+Rot(2,2))/2;
  q[1] = (Rot(1,2)-Rot(2,1))/(4*q[0]);
  q[2] = (Rot(2,0)-Rot(0,2))/(4*q[0]);
  q[3] = (Rot(0,1)-Rot(1,0))/(4*q[0]);
}
