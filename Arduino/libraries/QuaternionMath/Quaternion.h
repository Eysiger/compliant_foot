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

void quatMult(float* q, float* p, float* r);
void invertQuat(float* q, float* r);

void quatToEul(float* q, float* angles);
void eulToQuat(float* angles, float* q);
void getYawPitchRoll(float* q, float* ypr);
void getAngles(float* q, float* angles);

void quatToRotMat(float* q, Eigen::MatrixXf& Rot);
void rotMatToQuat(Eigen::MatrixXf& Rot, float* q);

#endif