/*
Force.h
Roman Käslin
rkaeslin@student.ethz.ch
2017-06-24

*/

#ifndef FORCE_h
#define FORCE_h

#include "Arduino.h"
#include "Quaternion.h"
#include <Eigen.h>
#include <Eigen/LU>

class Force{
    public:
        Force();

        // uses the accelerometer measurements to compensate for inertial forces the force sensor measures
        void compensateAcceleration(float* qrel, float* ax1, float* ay1, float* az1, float* forces, float* torques, float* compForces, float* compTorques);

        // compensates the force excerted by the shell depending on the relative foot sole position (not implemented yet)
        void compensateShell(float* qrel, float* forces, float* torques, float* compForces, float* compTorques);

        // rotates the measured forces to the world coordinate frame
        void rotateToZCoordiantes(float* q2, float* angle, float* forces, float* torques, float* worldForces, float* worldTorques);
        
    private:
        float massBeforePivot_;
    	float massAfterPivot_;

        float distanceSensorPivot_;
        float distancePivotCenterOfMassBefore_;
        float distancePivotCenterOfMassAfter_;
};

#endif