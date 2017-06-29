/*
Quaternion.cpp
Roman Käslin
rkaeslin@student.ethz.ch
2017-06-24
*/

#include "Quaternion.h"

void quatMult(float* q, float* p, float* r) {
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

void invertQuat(float* q, float* r) {
  r[0] = q[0];
  r[1] = -q[1];
  r[2] = -q[2];
  r[3] = -q[3];
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

void quatToRotMat(float* q, Eigen::MatrixXf& Rot){
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

void rotMatToQuat(Eigen::MatrixXf& Rot, float* q){
  q[0] = sqrt(1+Rot(0,0)+Rot(1,1)+Rot(2,2))/2;
  q[1] = (Rot(1,2)-Rot(2,1))/(4*q[0]);
  q[2] = (Rot(2,0)-Rot(0,2))/(4*q[0]);
  q[3] = (Rot(0,1)-Rot(1,0))/(4*q[0]);
}