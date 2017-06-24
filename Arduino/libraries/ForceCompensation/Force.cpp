/*
Force.cpp
Roman Käslin
rkaeslin@student.ethz.ch
2017-06-24
*/

#include "Force.h"

Force::Force() : massBeforePivot_(0.040), massAfterPivot_(0.121), distanceSensorPivot_(-36.5), distancePivotCenterOfMassBefore_(8.8), distancePivotCenterOfMassAfter_(-7.9) {
	
}

void Force::compensateAcceleration(float* qrel, float ax1, float ay1, float az1, float* forces, float* torques) {
    float invqrel[4];
    invertQuat(qrel, invqrel);

    float acc[4] = {0, ax1, ay1, az1};
    quatMult(invqrel, acc, acc);
    quatMult(acc, qrel, acc);

    float distanceAfterPivot[4] = {0, 0, 0, distancePivotCenterOfMassAfter_};
    quatMult(invqrel, distanceAfterPivot, distanceAfterPivot);
    quatMult(distanceAfterPivot, qrel, distanceAfterPivot);

    float massAfterSensor = (massBeforePivot_ + massAfterPivot_);
    float massTimesLeverBeforeZ = massBeforePivot_ * (distanceSensorPivot_ + distancePivotCenterOfMassBefore_)/1000.0;
    float massTimesLeverAfterX = massAfterPivot_ * distanceAfterPivot[1]/1000.0;
    float massTimesLeverAfterY = massAfterPivot_ * distanceAfterPivot[2]/1000.0;
    float massTimesLeverAfterZ = massAfterPivot_ * (distanceSensorPivot_ + distanceAfterPivot[3])/1000.0;
    
    forces[0] -= massAfterSensor * acc[1];
    forces[1] -= massAfterSensor * acc[2];
    forces[2] -= massAfterSensor * acc[3];
    torques[0] -= -massTimesLeverBeforeZ * acc[2] + massTimesLeverAfterY * acc[3] - massTimesLeverAfterZ * acc[2];
    torques[1] -=  massTimesLeverBeforeZ * acc[1] + massTimesLeverAfterZ * acc[1] - massTimesLeverAfterX * acc[3];
    torques[2] -=                                   massTimesLeverAfterX * acc[2] - massTimesLeverAfterY * acc[1];
}

void Force::compensateShell(float* qrel, float* forces, float* torques) {

}