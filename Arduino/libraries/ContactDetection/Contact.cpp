/*
Contact.cpp
Roman Käslin
rkaeslin@student.ethz.ch
2017-06-19
*/

#include "Contact.h"

Contact::Contact() : contact_(false), detectForceThreshold_(-20), accThreshold_(4*9.8), accForceThreshold_(-15), removeForceThreshold_(-10) {
	
}

void Contact::update(float* q1, float* q2, float* ax1, float* ay1, float* az1, float* forces, bool* contact) {
    float invq2[4];
    invertQuat(q2, invq2);

    quatMult(invq2, forces, forces);
    quatMult(forces, q2, forces);

    float accZ = 0;

    for (int i = 0; i < sizeof(ax1); i++) {
        float temp[4] = { 0, ax1[i], ay1[i], az1[i] };

        quatMult(invq2, temp, temp);
        quatMult(temp, q2, temp);
        if ( fabs(temp[3]) > accZ) {
            accZ = fabs(temp[3]);
        }
    }
    if (contact_ == false) {
        if (forces[3] < detectForceThreshold_) {
            contact_ = true;
        }
        else if ( (forces[3] < accForceThreshold_) && (accZ > accThreshold_) ) {
            contact_ = true;
        }
    }
    else {
        if (forces[3] > removeForceThreshold_) {
            contact_ = false;
        }
    }
    *contact = contact_;
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