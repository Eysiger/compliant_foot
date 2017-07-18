/*
Contact.h
Roman Käslin
rkaeslin@student.ethz.ch
2017-06-19

*/

#ifndef CONTACT_h
#define CONTACT_h

#include "Arduino.h"
#include "Quaternion.h"

class Contact{
    public:
        Contact();

        void update(float* q2, float* ax1, float* ay1, float* az1, float* worldForces, bool* contact);

        void updateNorm(float* forces, bool* contact);


        void setDetectContactThreshold(float detectContactThreshold) {
        	detectContactThreshold_ = detectContactThreshold;
        }
        void setAccAndForceThreshold(float accThreshold, float accForceThreshold) {
        	accForceThreshold_ = accForceThreshold;
        	accThreshold_ = accThreshold;
        }
        void setRemoveContactThreshold(float removeContactThreshold) {
        	removeContactThreshold_ = removeContactThreshold;
        }
        
    private:
    	bool contact_;
        bool normContact_;

    	float detectContactThreshold_;
    	float accThreshold_;
    	float accForceThreshold_;
       	float removeContactThreshold_;

       	const float G = 9.807f;
};

#endif