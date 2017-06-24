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

        void compensateAcceleration(float* qrel, float ax1, float ay1, float az1, float* forces, float* torques);

        void compensateShell(float* qrel, float* forces, float* torques);
        
    private:
        float massBeforePivot_;
    	float massAfterPivot_;

        float distanceSensorPivot_;
        float distancePivotCenterOfMassBefore_;
        float distancePivotCenterOfMassAfter_;
};

#endif