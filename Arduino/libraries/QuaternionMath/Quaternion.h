/*
Quaternion.h
Roman Käslin
rkaeslin@student.ethz.ch
2017-06-24

*/

#ifndef QUATERNION_h
#define QUATERNION_h

#include "Arduino.h"
#include <Eigen.h>
#include <Eigen/LU>

void quatMult(float q[4], float p[4], float r[4]);
void invertQuat(float q[4], float r[4]);

void quatToEul(float* q, float* angles);
void eulToQuat(float* angles, float* q);
void getYawPitchRoll(float* q, float* ypr);
void getAngles(float* q, float* angles);

void quatToRotMat(float q[4], Eigen::MatrixXf& Rot);
void rotMatToQuat(Eigen::MatrixXf& Rot, float q[4]);

#endif