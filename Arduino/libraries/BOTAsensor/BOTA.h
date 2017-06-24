/*
BOTA.h
Roman Käslin
rkaeslin@student.ethz.ch
2017-06-19

*/

#ifndef BOTA_h
#define BOTA_h

#include "Arduino.h"
#include <Eigen.h>
#include <Eigen/LU>

#define UART Serial1

class BOTA{
    public:
        BOTA(uint8_t TxPin, uint8_t RxPin);

        void begin();

        bool getForces(float* Fx, float* Fy, float* Fz, float* Tx, float* Ty, float* Tz);

        void setOffset(float Fx, float Fy, float Fz, float Tx, float Ty, float Tz);
        void getOffset(float* Fx, float* Fy, float* Fz, float* Tx, float* Ty, float* Tz);

        void setZero();
    private:
    	Eigen::Matrix<float, 6, 6> A;
		Eigen::Matrix<float, 6, 1> ef;
		Eigen::Matrix<float, 6, 1> F;
		Eigen::Matrix<float, 6, 1> offset;

		uint8_t TxPin_;
		uint8_t RxPin_;

		int variable_;
		bool started_;
		int sign_;
		Eigen::Matrix<int,6,1> data_;
};

#endif