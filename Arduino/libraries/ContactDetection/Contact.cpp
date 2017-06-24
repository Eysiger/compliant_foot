/*
Contact.cpp
Roman Käslin
rkaeslin@student.ethz.ch
2017-06-19
*/

#include "Contact.h"

Contact::Contact() : contact_(false), detectContactThreshold_(-20), accThreshold_(4*9.8), accForceThreshold_(-15), removeContactThreshold_(-10) {
	
}

void Contact::update(float* q2, float* ax1, float* ay1, float* az1, float* forces, float* torques, bool* contact) {
    float invq2[4];
    invertQuat(q2, invq2);

    float tempForces[4] = {0, forces[0], forces[1], forces[2]};
    quatMult(invq2, tempForces, tempForces);
    quatMult(tempForces, q2, tempForces);

    float tempTorques[4] = {0, torques[0], torques[1], torques[2]};
    quatMult(invq2, tempTorques, tempTorques);
    quatMult(tempTorques, q2, tempTorques);

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
        if (tempForces[3] < detectContactThreshold_) {
            contact_ = true;
        }
        else if ( (tempForces[3] < accForceThreshold_) && (accZ > accThreshold_) ) {
            contact_ = true;
        }
    }
    else {
        if (tempForces[3] > removeContactThreshold_) {
            contact_ = false;
        }
    }

    *contact = contact_;
    forces[0] = tempForces[1];
    forces[1] = tempForces[2];
    forces[2] = tempForces[3];
    torques[0] = tempTorques[1];
    torques[1] = tempTorques[2];
    torques[2] = tempTorques[3];
}