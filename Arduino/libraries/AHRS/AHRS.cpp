/*
AHRS.cpp
Roman Käslin
rkaeslin@student.ethz.ch
2017-05-14

adapted from

*/

#include "Arduino.h"
#include "AHRS.h"

AHRS::AHRS(){
    q0 = 1; q1 = 0; q2 = 0; q3 = 0;
    integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;
    lastUpdate = 0;
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

void AHRS::getEuler(float* q, float* angles) {
  angles[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1) * 180/M_PI; // psi
  angles[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]) * 180/M_PI; // theta
  angles[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1) * 180/M_PI; // phi
}

void AHRS::getAngles(float* q, float* angles) {
  float a[3]; //Euler
  getEuler(q, a);

  angles[0] = a[0];
  angles[1] = a[1];
  angles[2] = a[2];
  
  if(angles[0] < 0)angles[0] += 360;
  if(angles[1] < 0)angles[1] += 360;
  if(angles[2] < 0)angles[2] += 360;
}

void AHRS::getYawPitchRoll(float* q, float* ypr) {
  float gxhat, gyhat, gzhat; // estimated gravity direction
  
  gxhat = 2 * (q[1]*q[3] - q[0]*q[2]);
  gyhat = 2 * (q[0]*q[1] + q[2]*q[3]);
  gzhat = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
  
  ypr[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1) * 180/M_PI;
  ypr[1] = atan(gxhat / sqrt(gyhat*gyhat + gzhat*gzhat))  * 180/M_PI;
  ypr[2] = atan(gyhat / sqrt(gxhat*gxhat + gzhat*gzhat))  * 180/M_PI;
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
