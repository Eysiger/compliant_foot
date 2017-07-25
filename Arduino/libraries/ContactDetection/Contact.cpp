/*
Contact.cpp
Roman Käslin
rkaeslin@student.ethz.ch
2017-06-19
*/

#include "Contact.h"

Contact::Contact() : contact_(false), normContact_(false), detectContactThreshold_(-20), accThreshold_(4*G), accForceThreshold_(-15), removeContactThreshold_(-10) {
	
}

void Contact::updateZ(float* q2, float* ax1, float* ay1, float* az1, float* worldForces, bool* contact) {
    float invq2[4];
    invertQuat(q2, invq2);

    // search for biggest acceleration along world z-axis within the last measurements
    float accZ = 0;
    for (int i = 0; i < sizeof(ax1); i++) {
        float acc[4] = { 0, ax1[i], ay1[i], az1[i] };

        // rotate the measured acceleration to world coordinate frame
        quatMult(acc, invq2, acc);
        quatMult(q2, acc, acc);

        if ( fabs(acc[3]-G) > accZ) {
            accZ = fabs(acc[3]-G);
        }
    }

    // pressure is negative force for BOTA sensor!
    // if the contact state was set false, check along world z-axis for detect contact threshold and combined acceleration & contact threshold
    if (contact_ == false) {
        if (worldForces[2] < detectContactThreshold_) {
            contact_ = true;
        }
        else if ( (worldForces[2] < accForceThreshold_) && (accZ > accThreshold_) ) {
            contact_ = true;
        }
    }
    // if the contact state was already set true, check along world z-axis for remove contact threshold
    else {
        if (worldForces[2] > removeContactThreshold_) {
            contact_ = false;
        }
    }

    *contact = contact_;
}

void Contact::updateNorm(float* forces, bool* contact) {
    // calculate the norm of the forces
    float norm = sqrt(forces[0]*forces[0] + forces[1]*forces[1] + forces[2]*forces[2]);
    // if the contact state was set false, check norm for detect contact threshold
    if (normContact_ == false) {
        if (-norm < detectContactThreshold_) {
            normContact_ = true;
        }
    }
    // if the contact state was already set true, check norm for remove contact threshold
    else {
        if (-norm > removeContactThreshold_) {
            normContact_ = false;
        }
    }

    *contact = normContact_;
}