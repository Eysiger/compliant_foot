/*
Contact.cpp
Roman Käslin
rkaeslin@student.ethz.ch
2017-06-19
*/

#include "Contact.h"

Contact::Contact() : contact_(false), detectContactThreshold_(-20), accThreshold_(4*9.8), accForceThreshold_(-15), removeContactThreshold_(-10) {
	
}

void Contact::update(float* q2, float* ax1, float* ay1, float* az1, float* worldForces, bool* contact) {
    float invq2[4];
    invertQuat(q2, invq2);
    
    float accZ = 0;

    for (int i = 0; i < sizeof(ax1); i++) {
        float acc[4] = { 0, ax1[i], ay1[i], az1[i] };

        quatMult(invq2, acc, acc);
        quatMult(acc, q2, acc);
        if ( fabs(acc[3]) > accZ) {
            accZ = fabs(acc[3]);
        }
    }
    if (contact_ == false) {
        if (worldForces[3] < detectContactThreshold_) {
            contact_ = true;
        }
        else if ( (worldForces[3] < accForceThreshold_) && (accZ > accThreshold_) ) {
            contact_ = true;
        }
    }
    else {
        if (worldForces[3] > removeContactThreshold_) {
            contact_ = false;
        }
    }

    *contact = contact_;
}