/*
Contact.h
Roman Käslin
rkaeslin@student.ethz.ch
2017-06-19

*/

#ifndef CONTACT_h
#define CONTACT_h

#include "Arduino.h"

class Contact{
    public:
        Contact();

        void update(float* q1, float* q2, float* ax1, float* ay1, float* az1, float* forces, float* torques, bool* contact);

        void setDetectForceThreshold(float detectForceThreshold) {
        	detectForceThreshold_ = detectForceThreshold;
        }
        void setAccAndForceThreshold(float accThreshold, float accForceThreshold) {
        	accForceThreshold_ = accForceThreshold;
        	accThreshold_ = accThreshold;
        }
        void setRemoveForceThreshold(float removeForceThreshold) {
        	removeForceThreshold_ = removeForceThreshold;
        }
        
    private:
    	bool contact_;

    	float detectForceThreshold_;
    	float accThreshold_;
    	float accForceThreshold_;
       	float removeForceThreshold_;
};

void quatMult(float q[4], float p[4], float r[4]);
void invertQuat(float q[4], float r[4]);

#endif