/*
Force.cpp
Roman Käslin
rkaeslin@student.ethz.ch
2017-06-24
*/

#include "Force.h"

Force::Force() : massBeforePivot_(0.040), massAfterPivot_(0.121), distanceSensorPivot_(-36.5), distancePivotCenterOfMassBefore_(8.8), distancePivotCenterOfMassAfter_(-7.9) {
	
}

void Force::compensateAcceleration(float* qrel, float* ax1, float* ay1, float* az1, float* forces, float* torques, float* compForces, float* compTorques) {
  // calculate the rotation introduced around yaw of frame 2 and remove it
  float psi = atan2(2*qrel[0]*qrel[3] + 2*qrel[1]*qrel[2], 2*qrel[0]*qrel[0] + 2*qrel[1]*qrel[1] - 1);
  float qrotz[4] = {cos(0.5 * -psi), 0, 0, sin(0.5 * -psi)};

  float qrelComp[4];
  float invqrelComp[4];
  quatMult(qrotz, qrel, qrelComp);
  invertQuat(qrelComp, invqrelComp);

  // rotate measured acceleration of the foot sole IMU to the force sensor orientation
  float acc[4] = {0, *ax1, *ay1, *az1};
  quatMult(acc, invqrelComp, acc);
  quatMult(qrelComp, acc, acc);

  // calculate vector pointing from the pivot point to the COG of the mass after the pivot joint
  float distanceAfterPivot[4] = {0, 0, 0, distancePivotCenterOfMassAfter_};
  quatMult(distanceAfterPivot, invqrelComp, distanceAfterPivot);
  quatMult(qrelComp, distanceAfterPivot, distanceAfterPivot);

  // add masses and multiply them by the levers
  float massAfterSensor = (massBeforePivot_ + massAfterPivot_);
  float massTimesLeverBeforeZ = massBeforePivot_ * (distanceSensorPivot_ + distancePivotCenterOfMassBefore_)/1000.0;
  float massTimesLeverAfterX = massAfterPivot_ * distanceAfterPivot[1]/1000.0;
  float massTimesLeverAfterY = massAfterPivot_ * distanceAfterPivot[2]/1000.0;
  float massTimesLeverAfterZ = massAfterPivot_ * (distanceSensorPivot_ + distanceAfterPivot[3])/1000.0;
  
  // compute the resulting forces and torques (weight * lever x acceleration) and subtract them
  compForces[0] = forces[0] - massAfterSensor * acc[1];
  compForces[1] = forces[1] - massAfterSensor * acc[2];
  compForces[2] = forces[2] - massAfterSensor * acc[3];
  compTorques[0] = torques[0] - (-massTimesLeverBeforeZ * acc[2] + massTimesLeverAfterY * acc[3] - massTimesLeverAfterZ * acc[2]);
  compTorques[1] = torques[1] - ( massTimesLeverBeforeZ * acc[1] + massTimesLeverAfterZ * acc[1] - massTimesLeverAfterX * acc[3]);
  compTorques[2] = torques[2] - (                                  massTimesLeverAfterX * acc[2] - massTimesLeverAfterY * acc[1]);
}

void Force::compensateShell(float* qrel, float* forces, float* torques, float* compForces, float* compTorques) {
  // not yet implemented
}

void Force::rotateToWorldCoordiantes(float* q2, float* angle, float* forces, float* torques, float* worldForces, float* worldTorques) {
  // calculate the rotation introduced around yaw of world frame and remove it
  float psiComp = atan2(2*q2[0]*q2[3] + 2*q2[1]*q2[2], 2*q2[0]*q2[0] + 2*q2[1]*q2[1] - 1);
  float qrotz[4] = {cos(0.5 * -psiComp), 0, 0, sin(0.5 * -psiComp)};

  float q2Comp[4];
  quatMult(qrotz, q2, q2Comp);

  // rotate the shank coordinate frame by the angle measured by the encoder to get force sensor frame
  //*angle
  float qrotenc[4] = {cos(0.5 * *angle), 0, 0, sin(0.5 * *angle)};

  float invq2[4];
  quatMult(q2Comp, qrotenc, q2Comp);
  invertQuat(q2Comp, invq2);

  // rotate the forces and torques to world coordinate frame
  float tempForces[4] = {0, forces[0], forces[1], forces[2]};
  quatMult(tempForces, invq2, tempForces);
  quatMult(q2Comp, tempForces, tempForces);

  float tempTorques[4] = {0, torques[0], torques[1], torques[2]};
  quatMult(tempTorques, invq2, tempTorques);
  quatMult(q2Comp, tempTorques, tempTorques);

  // stroe the temporary values as output values
  worldForces[0] = tempForces[1];
  worldForces[1] = tempForces[2];
  worldForces[2] = tempForces[3];
  worldTorques[0] = tempTorques[1];
  worldTorques[1] = tempTorques[2];
  worldTorques[2] = tempTorques[3];
}

