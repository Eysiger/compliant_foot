/*
AHRS.h
Roman Käslin
rkaeslin@student.ethz.ch
2017-05-14

adapted from

*/

#ifndef AHRS_h
#define AHRS_h

#include "Arduino.h"
#include <Eigen.h>
#include <Eigen/LU>

class AHRS{
    public:
        AHRS(float bbx, float bby, float bbz);
        void getQEKF(float* q, float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* psi);
        void getQDCM(float* q, float* ax, float* ay, float* az, float* gx, float* gy, float* gz);

    private:
        void EKFupdate(float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* psi);
        void DCMupdate(float* ax, float* ay, float* az, float* gx, float* gy, float* gz);
        volatile float twoKp = (2.0f * 0.5f); // 2 * proportional gain;      // 2 * proportional gain (Kp)
        volatile float twoKi = (2.0f * 0.1f); // 2 * integral gain;      // 2 * integral gain (Ki)
        float q0, q1, q2, q3; // quaternion of sensor frame relative to auxiliary frame
        float bx, by, bz;
        float gyroNoise = 0.008*sqrt(8000)/180*M_PI;
        float gyroBias = 5/180*M_PI;
        float accelNoise = 0.00025*9.81*sqrt(4000);
        float encoderNoise = 0.06/180*M_PI;
        Eigen::Matrix<float, 7, 1> x;
        Eigen::Matrix<float, 7, 7> P;
        Eigen::Matrix<float, 7, 7> A;
        Eigen::Matrix<float, 3, 3> Q;
        Eigen::Matrix<float, 3, 7> H;
        Eigen::Matrix<float, 3, 3> R;
        Eigen::Matrix<float, 7, 3> K;
        volatile float integralFBx,  integralFBy, integralFBz;
        unsigned long lastUpdate, now; // sample period expressed in milliseconds
        float sampleFreq; // half the sample period expressed in seconds
        float sampleTime;

        const float G = 9.807f;
};

void quatMult(float q[4], float p[4], float r[4]);
void invertQuat(float q[4], float r[4]);

void quatToEul(float* q, float* angles);
void eulToQuat(float* angles, float* q);
void getYawPitchRoll(float* q, float* ypr);
void getAngles(float* q, float* angles);
float invSqrt(float number);

void quatMult(float q[4], float p[4], float r[4]);
void invertQuat(float q[4], float r[4]);

void quatToRotMat(float q[4], Eigen::MatrixXf& Rot);
void rotMatToQuat(Eigen::MatrixXf& Rot, float q[4]);

#endif
