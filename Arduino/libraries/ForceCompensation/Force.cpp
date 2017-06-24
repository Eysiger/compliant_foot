/*
Force.cpp
Roman Käslin
rkaeslin@student.ethz.ch
2017-06-24
*/

#include "Force.h"

Force::Force() : massBeforePivot_(0.040), massAfterPivot_(0.121), distanceSensorPivot_(4.5), distancePivotCenterOfMassBefore_(-8.8), distancePivotCenterOfMassAfter_(7.9) {
	
}

void Force::compensateAcceleration(float* qrel, float ax1, float ay1, float az1, float* forces, float* torques) {

    float acc[4] = {0, ax1, ay1, az1};
    float tempForces[4] = {0, forces[0], forces[1], forces[2]};
    float tempTorques[4] = {0, torques[0], torques[1], torques[2]};



    
    forces[0] = tempForces[1];
    forces[1] = tempForces[2];
    forces[2] = tempForces[3];
    torques[0] = tempTorques[1];
    torques[1] = tempTorques[2];
    torques[2] = tempTorques[3];
}

void Force::compensateShell(float* qrel, float* forces, float* torques) {

}