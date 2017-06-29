/*
BOTA.cpp
Roman Käslin
rkaeslin@student.ethz.ch
2017-06-19
*/

#include "BOTA.h"

BOTA::BOTA(uint8_t TxPin, uint8_t RxPin): variable_(0), started_(false), sign_(1) {
	TxPin_ = TxPin;
	RxPin_ = RxPin;
	data_ << 0, 0, 0, 0, 0, 0;
}

void BOTA::begin(){
	UART.setTX(TxPin_);
  	UART.setRX(RxPin_);

  	UART.begin(230400);
  	A <<  -0.001473513720803,   0.024934656756038,  -0.020341823640977,  -0.020552003033123,   0.011483767341789,   0.014051224287734,
           0.022724842326727,  -0.010607849529059,  -0.014724750724608,  -0.001461044317035,   0.018206029868300,  -0.017519364420186,
           0.020362123488925,   0.022827582060519,   0.022940548564196,  -0.001960668445840,  -0.002258007026808,   0.002192924621157,
           0.000046607185284,   0.000016986520174,   0.000000163248150,  -0.000001311378072,   0.000360472536316,  -0.000315323099915,
           0.000059678987001,  -0.000016759770216,   0.000096927174460,   0.000374515402127,  -0.000175665618951,  -0.000231607347764,
          -0.000045420096828,   0.000052311293160,  -0.000025849051812,  -0.000267459924206,  -0.000204332913294,  -0.000128001988874;
    // A << 1, 0, 0, 0, 0, 0,
    // 	 0, 1, 0, 0, 0, 0,
    // 	 0, 0, 1, 0, 0, 0,
    // 	 0, 0, 0, 1, 0, 0,
    // 	 0, 0, 0, 0, 1, 0,
    // 	 0, 0, 0, 0, 0, 1;
    offset <<  0, 0, 0, 0, 0, 0;
}

bool BOTA::getForces(float* Fx, float* Fy, float* Fz, float* Tx, float* Ty, float* Tz) {
	if (UART.available() > 0) {
	    int reading;
	    if (!started_) {
	      	reading = UART.read();
	      	if ( reading == 10 ) {
	        	started_ = true;
	      	}
	      	else { return  0;}
	    }
      	while ( (UART.available() > 0) && started_) {
        	reading = UART.read();
        	switch (reading) {
          		case 45:
            		sign_ = -1;
            		break;
          		case 32:
            		data_[variable_] *= sign_;
            		sign_ = 1;
            		variable_++;
            		break;
          		case 13:
                    ef[0] = data_[0];
            		ef[1] = data_[1];
            		ef[2] = data_[2];
            		ef[3] = data_[3];
            		ef[4] = data_[4];
            		ef[5] = data_[5];
            		F = A*ef;

            		*Fx = F[0] - offset[0];
            		*Fy = F[1] - offset[1];
            		*Fz = F[2] - offset[2];
            		*Tx = F[3] - offset[3];
            		*Ty = F[4] - offset[4];
            		*Tz = F[5] - offset[5];

            		started_ = false;
            		variable_ = 0;
            		data_[0] = 0; data_[1] = 0; data_[2] = 0; data_[3] = 0; data_[4] = 0; data_[5] = 0;
            		return 1;
          		default:
            		data_[variable_] *= 10;
            		data_[variable_] += reading - 48;
        	}
      	}
	    return 0;
	}
	else { 
		return 0; 
	}
}

void BOTA::setOffset(float Fx, float Fy, float Fz, float Tx, float Ty, float Tz) {
	offset[0] += Fx;
	offset[1] += Fy;
	offset[2] += Fz;
	offset[3] += Tx;
	offset[4] += Ty;
	offset[5] += Tz;
}

void BOTA::getOffset(float* Fx, float* Fy, float* Fz, float* Tx, float* Ty, float* Tz) {
	*Fx = offset[0];
	*Fy = offset[1];
	*Fz = offset[2];
	*Tx = offset[3];
	*Ty = offset[4];
	*Tz = offset[5];
}

void BOTA::setZero() {
	int number = 1000;
	float sumfx=0;
    float sumfy=0;
    float sumfz=0;
    float sumtx=0;
    float sumty=0;
    float sumtz=0;
	for (int i = 0; i < number; i++) {
		float fx, fy, fz, tx, ty, tz;
		getForces(&fx, &fy, &fz, &tx, &ty, &tz);
		sumfx += fx;
	    sumfy += fy;
	    sumfz += fz;
	    sumtx += tx;
	    sumty += ty;
	    sumtz += tz;
	    delay(6);		//adapt to speed of BOTA sensor
	}

	offset[0] += sumfx/((float)number);
	offset[1] += sumfy/((float)number);
	offset[2] += sumfz/((float)number);
	offset[3] += sumtx/((float)number);
	offset[4] += sumty/((float)number);
	offset[5] += sumtz/((float)number);
}