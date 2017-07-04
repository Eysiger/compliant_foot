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
#include "Quaternion.h"
#include <Eigen.h>
#include <Eigen/LU>

class AHRS{
    public:
        AHRS(float bx1, float by1, float bz1, float bx2, float by2, float bz2);
        void getQEKF(float* q1, float* q2, float* ax1, float* ay1, float* az1, float* gx1, float* gy1, float* gz1, float* ax2, float* ay2, float* az2, float* gx2, float* gy2, float* gz2, float* psi);
        void getComp(float* q1, float* q2, float* ax1, float* ay1, float* az1, float* gx1, float* gy1, float* gz1, float* ax2, float* ay2, float* az2, float* gx2, float* gy2, float* gz2, float* psi);

    private:
        void EKFupdate(float* ax1, float* ay1, float* az1, float* gx1, float* gy1, float* gz1, float* ax2, float* ay2, float* az2, float* gx2, float* gy2, float* gz2, float* psi);
        void CompUpdate(float* ax1, float* ay1, float* az1, float* gx1, float* gy1, float* gz1, float* ax2, float* ay2, float* az2, float* gx2, float* gy2, float* gz2, float* psi);
        float gyroNoise = 0.008*sqrt(8000)/180*M_PI;
        float gyroBias = 5/180*M_PI;
        float accelNoise = 0.00025*9.81*sqrt(4000);
        float encoderNoise = 0.06/180*M_PI;
        Eigen::Matrix<float, 14, 1> x;
        Eigen::Matrix<float, 14, 14> P;
        Eigen::Matrix<float, 14, 14> A;
        Eigen::Matrix<float, 4, 1> u1;
        Eigen::Matrix<float, 4, 1> omega1;
        Eigen::Matrix<float, 4, 1> bias1;
        Eigen::Matrix<float, 4, 1> u2;
        Eigen::Matrix<float, 4, 1> omega2;
        Eigen::Matrix<float, 4, 1> bias2;
        Eigen::Matrix<float, 6, 6> Q;
        Eigen::Matrix<float, 14, 6> L;
        Eigen::Matrix<float, 14, 14> APA;
        Eigen::Matrix<float, 14, 14> LQL;
        Eigen::Matrix<float, 7, 14> H;
        Eigen::Matrix<float, 7, 7> R;
        Eigen::Matrix<float, 14, 7> K;
        Eigen::Matrix<float, 7, 1> h;
        Eigen::Matrix<float, 7, 1> z;
        Eigen::Matrix<float, 14, 14> I;

        Eigen::Matrix<float, 8, 1> x2;
        Eigen::Matrix<float, 8, 8> P2;
        Eigen::Matrix<float, 8, 8> A2;
        Eigen::Matrix<float, 8, 6> L2;
        Eigen::Matrix<float, 9, 8> H2;
        Eigen::Matrix<float, 8, 9> K2;
        Eigen::Matrix<float, 8, 8> I2;
        Eigen::Matrix<float, 9, 1> h2;
        Eigen::Matrix<float, 9, 1> z2;

        float sampleTime;
        
        unsigned long lastUpdate, now; // sample period expressed in milliseconds

        const float G = 9.807f;
};

float invSqrt(float number);

#endif
