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

void Force::rotateToWorldCoordiantes(float* q2, float* forces, float* torques, float* worldForces, float* worldTorques) {
  float psiComp = -atan2(2*q2[0]*q2[3] + 2*q2[1]*q2[2], 2*q2[0]*q2[0] + 2*q2[1]*q2[1] - 1);
  float qrotz[4] = {cos(0.5*psiComp),0,0,sin(0.5*psiComp)};

  float q2Comp[4];
  float invq2[4];
  quatMult(q2, qrotz, q2Comp);
  invertQuat(q2Comp, invq2);

  float tempForces[4] = {0, forces[0], forces[1], forces[2]};
  quatMult(invq2, tempForces, tempForces);
  quatMult(tempForces, q2Comp, tempForces);

  float tempTorques[4] = {0, torques[0], torques[1], torques[2]};
  quatMult(invq2, tempTorques, tempTorques);
  quatMult(tempTorques, q2Comp, tempTorques);

  worldForces[0] = tempForces[1];
  worldForces[1] = tempForces[2];
  worldForces[2] = tempForces[3];
  worldTorques[0] = tempTorques[1];
  worldTorques[1] = tempTorques[2];
  worldTorques[2] = tempTorques[3];
}

