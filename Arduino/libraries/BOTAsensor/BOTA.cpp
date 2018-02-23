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
  // open UART connection to BOTA sensor
	UART.setTX(TxPin_);
	UART.setRX(RxPin_);

	UART.begin(230400);

  // calibration matrix
	// A <<  -0.001473513720803*1.10,   0.024934656756038*1.15,  -0.020341823640977*1.15,  -0.020552003033123*1.15,   0.011483767341789*1.15,  0.014051224287734*1.15,
 	//       0.022724842326727*1.26,  -0.010607849529059*1.26,  -0.014724750724608*1.26,  -0.001461044317035*1.26,   0.018206029868300*1.26,  -0.017519364420186*1.26,
 	//       0.020362123488925*1.04,   0.022827582060519*1.02,   0.022940548564196*1.02,  -0.001960668445840*1.02,  -0.002258007026808*1.02,   0.002192924621157*1.02,
 	//       0.000046607185284,   0.000016986520174,   0.000000163248150,  -0.000001311378072,   0.000360472536316,  -0.000315323099915,
 	//       0.000059678987001,  -0.000016759770216,   0.000096927174460,   0.000374515402127,  -0.000175665618951,  -0.000231607347764,
 	//      -0.000045420096828,   0.000052311293160,  -0.000025849051812,  -0.000267459924206,  -0.000204332913294,  -0.000128001988874;
	// A <<    0.016473712048946,  -0.018434943107967,   0.031281811378483,   0.000032893581935,  -0.000045927098331,   0.000010725431344,
	//	 	  -0.022626986842835,   0.004565202033777,   0.031422573611919,  -0.000004472595228,  -0.000110103726654,  -0.000023413098322,
	// 		  -0.020538490974638,  -0.041284874434838,   0.031408465631777,   0.000074608467239,  -0.000085562424479,  -0.000016709093433,
	//	 	   0.001491159981988,  -0.026005500389798,   0.001982932241946,   0.000365158238077,   0.000024215308372,  -0.000191441588167,
	//	 	   0.019016987981133,   0.013257240338992,  -0.001319076943630,  -0.000165475383068,   0.000348541943653,  -0.000193627307394,
	//	 	  -0.018166941193630,   0.011742689160121,  -0.000414893196491,  -0.000169496258839,  -0.000332052703287,  -0.000207958212985;
	A <<   0.018434943107967,  -0.004565202033777,   0.041284874434838,   0.026005500389798,  -0.013257240338992,  -0.011742689160121,
		  -0.016473712048946,   0.022626986842835,   0.020538490974638,  -0.001491159981988,  -0.019016987981133,   0.018166941193630,
		  -0.031281811378483,  -0.031422573611919,  -0.031408465631777,  -0.001982932241946,   0.001319076943630,   0.000414893196491,
		   0.000045927098331,   0.000110103726654,   0.000085562424479,  -0.000024215308372,  -0.000348541943653,   0.000332052703287,
		  -0.000032893581935,   0.000004472595228,  -0.000074608467239,  -0.000365158238077,   0.000165475383068,   0.000169496258839,
		  -0.000010725431344,   0.000023413098322,   0.000016709093433,   0.000191441588167,   0.000193627307394,   0.000207958212985;
    // A <<  1, 0, 0, 0, 0, 0,
    //	 	 0, 1, 0, 0, 0, 0,
    //	 	 0, 0, 1, 0, 0, 0,
    //	 	 0, 0, 0, 1, 0, 0,
    //	 	 0, 0, 0, 0, 1, 0,
    //	 	 0, 0, 0, 0, 0, 1;

  // set all offsets to zero
  offset[0] = 0;
  offset[1] = 0;
  offset[2] = 0;
  offset[3] = 0;
  offset[4] = 0;
  offset[5] = 0;
}

bool BOTA::getForces(float* Force, float* Torque) {
  //check if data is available
	if (UART.available() > 0) {
	    int reading;
      // if the read-in hasn't started check if header (new line ASCII 10) is found otherwise return
	    if (!started_) {
	      	reading = UART.read();
	      	if ( reading == 10 ) {
	        	started_ = true;
	      	}
	      	else { return  0;}
	    }
      // while data is available and the read-in is started, read in the sensor values
    	while ( (UART.available() > 0) && started_) {
      	reading = UART.read();
      	switch (reading) {
        		case 45:  // ASCII minus
          		sign_ = -1;
          		break;
        		case 32:  // ASCII space (next variable)
          		data_[variable_] *= sign_;
          		sign_ = 1;
          		variable_++;
          		break;
        		case 13:  // ASCII carriage return
              // apply calibration matrix
                ef[0] = data_[0];
          		ef[1] = data_[1];
          		ef[2] = data_[2];
          		ef[3] = data_[3];
          		ef[4] = data_[4];
          		ef[5] = data_[5];
          		F = A*ef;

              // apply offsets and store as output
          		Force[0] = F[0] - offset[0];
          		Force[1] = F[1] - offset[1];
          		Force[2] = F[2] - offset[2];
          		Torque[0] = F[3] - offset[3];
          		Torque[1] = F[4] - offset[4];
          		Torque[2] = F[5] - offset[5];

              // reset for next data read-in
          		started_ = false;
          		variable_ = 0;
          		data_[0] = 0; data_[1] = 0; data_[2] = 0; data_[3] = 0; data_[4] = 0; data_[5] = 0;
          		return 1;
        		default:  // ASCII numbers
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
	offset[0] = Fx;
	offset[1] = Fy;
	offset[2] = Fz;
	offset[3] = Tx;
	offset[4] = Ty;
	offset[5] = Tz;
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
  // average over provided number of measurements
	int number = 1000;
	float sumfx=0;
  float sumfy=0;
  float sumfz=0;
  float sumtx=0;
  float sumty=0;
  float sumtz=0;

  // read-in the forces and torques and sum them up individually
	for (int i = 0; i < number; i++) {
		float f[3], t[3];
		getForces(f, t);
		sumfx += f[0];
    sumfy += f[1];
    sumfz += f[2];
    sumtx += t[0];
    sumty += t[1];
    sumtz += t[2];
    delay(6);		//adapt to speed of BOTA sensor
	}

  // calculate offset as the mean of the measured data
	offset[0] += sumfx/((float)number);
	offset[1] += sumfy/((float)number);
	offset[2] += sumfz/((float)number);
	offset[3] += sumtx/((float)number);
	offset[4] += sumty/((float)number);
	offset[5] += sumtz/((float)number);
}