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

        // updates the zContact estimate usiing forces and acceleration along world z-axis
        void updateZ(float* q2, float* ax1, float* ay1, float* az1, float* worldForces, bool* contact);

        // updates the normContact estimate using the norm of the forces
        void updateNorm(float* forces, bool* contact);

        // allows setting the force threshold for detecting a contact
        void setDetectContactThreshold(float detectContactThreshold) {
        	detectContactThreshold_ = detectContactThreshold;
        }
        // allows setting the combined acceleration and force threshold for detecting a contact
        void setAccAndForceThreshold(float accThreshold, float accForceThreshold) {
        	accForceThreshold_ = accForceThreshold;
        	accThreshold_ = accThreshold;
        }
        // allows setting the force threshold for setting the contact state to false
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