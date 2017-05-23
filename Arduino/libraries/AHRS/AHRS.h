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

class AHRS{
    public:
        AHRS();
        void getQDCM(float* q, float* ax, float* ay, float* az, float* gx, float* gy, float* gz);
        void getEuler(float* q, float* angles);
        void getYawPitchRoll(float* q, float* ypr);
        void getAngles(float* q, float* angles);
    private:
        void DCMupdate(float* gx, float* gy, float* gz, float* ax, float* ay, float* az);
        volatile float twoKp = (2.0f * 0.5f); // 2 * proportional gain;      // 2 * proportional gain (Kp)
        volatile float twoKi = (2.0f * 0.1f); // 2 * integral gain;      // 2 * integral gain (Ki)
        volatile float q0, q1, q2, q3; // quaternion of sensor frame relative to auxiliary frame
        volatile float integralFBx,  integralFBy, integralFBz;
        unsigned long lastUpdate, now; // sample period expressed in milliseconds
        float sampleFreq; // half the sample period expressed in seconds
};

float invSqrt(float number);

#endif
