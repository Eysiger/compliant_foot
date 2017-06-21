/*
ICM20608G.cpp
Roman Käslin
rkaeslin@student.ethz.ch
2017-05-14

adapted from
MPU9250.cpp
Brian R Taylor
brian.taylor@bolderflight.com
2017-01-04

Copyright (c) 2016 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

// Teensy 3.0 || Teensy 3.1/3.2 || Teensy 3.5 || Teensy 3.6 || Teensy LC 
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || \
	defined(__MK66FX1M0__) || defined(__MKL26Z64__)

#include "Arduino.h"
#include "ICM20608G.h"
#include "i2c_t3.h"  // I2C library
#include "SPI.h" // SPI Library

/* ICM20608G object, input the I2C address and I2C bus */
ICM20608G::ICM20608G(uint8_t address, uint8_t bus){
    _address = address; // I2C address
    _bus = bus; // I2C bus
    _userDefI2C = false; // automatic I2C setup
    _useSPI = false; // set to use I2C instead of SPI
}

/* ICM20608G object, input the I2C address, I2C bus, and I2C pins */
ICM20608G::ICM20608G(uint8_t address, uint8_t bus, i2c_pins pins){
    _address = address; // I2C address
    _bus = bus; // I2C bus
    _pins = pins; // I2C pins
    _pullups = I2C_PULLUP_EXT; // I2C pullups
    _userDefI2C = true; // user defined I2C
    _useSPI = false; // set to use I2C instead of SPI
}

/* ICM20608G object, input the I2C address, I2C bus, I2C pins, and I2C pullups */
ICM20608G::ICM20608G(uint8_t address, uint8_t bus, i2c_pins pins, i2c_pullup pullups){
    _address = address; // I2C address
    _bus = bus; // I2C bus
    _pins = pins; // I2C pins
    _pullups = pullups; // I2C pullups
    _userDefI2C = true; // user defined I2C
    _useSPI = false; // set to use I2C instead of SPI
}

/* ICM20608G object, input the SPI CS Pin */
ICM20608G::ICM20608G(uint8_t csPin){
    _csPin = csPin; // SPI CS Pin
    _mosiPin = MOSI_PIN_11;	// SPI MOSI Pin, set to default
    _useSPI = true; // set to use SPI instead of I2C
    _useSPIHS = false; // defaul to low speed SPI transactions until data reads start to occur
}

/* ICM20608G object, input the SPI CS Pin and MOSI Pin */
ICM20608G::ICM20608G(uint8_t csPin, spi_mosi_pin pin){
    _csPin = csPin; // SPI CS Pin
    _mosiPin = pin;	// SPI MOSI Pin
    _useSPI = true; // set to use SPI instead of I2C
    _useSPIHS = false; // defaul to low speed SPI transactions until data reads start to occur
}

/* starts I2C communication and sets up the ICM20608G */
int ICM20608G::begin(icm20608G_accel_range accelRange, icm20608G_gyro_range gyroRange){
    if( _useSPI ){ // using SPI for communication

        // setting CS pin to output
        pinMode(_csPin,OUTPUT);

        // setting CS pin high
        digitalWriteFast(_csPin,HIGH);

        // Teensy 3.0 || Teensy 3.1/3.2
		#if defined(__MK20DX128__) || defined(__MK20DX256__)

	        // configure and begin the SPI
	        switch( _mosiPin ){

				case MOSI_PIN_7:	// SPI bus 0 alternate 1
				    SPI.setMOSI(7);
	        		SPI.setMISO(8);
	        		SPI.setSCK(14);
	        		SPI.begin();
	        		break;
				case MOSI_PIN_11:	// SPI bus 0 default
					SPI.setMOSI(11);
	        		SPI.setMISO(12);
	        		SPI.setSCK(13);
	        		SPI.begin();
	        		break;
	        }

        #endif

        // Teensy 3.5 || Teensy 3.6 
		#if defined(__MK64FX512__) || defined(__MK66FX1M0__)

	        // configure and begin the SPI
	        switch( _mosiPin ){

	        	case MOSI_PIN_0:	// SPI bus 1 default
	        		SPI1.setMOSI(0);
	        		SPI1.setMISO(1);
	        		SPI1.setSCK(32);
	        		SPI1.begin();
	        		break;
				case MOSI_PIN_7:	// SPI bus 0 alternate 1
				    SPI.setMOSI(7);
	        		SPI.setMISO(8);
	        		SPI.setSCK(14);
	        		SPI.begin();
	        		break;
				case MOSI_PIN_11:	// SPI bus 0 default
					SPI.setMOSI(11);
	        		SPI.setMISO(12);
	        		SPI.setSCK(13);
	        		SPI.begin();
	        		break;
				case MOSI_PIN_21:	// SPI bus 1 alternate
		        	SPI1.setMOSI(21);
	        		SPI1.setMISO(5);
	        		SPI1.setSCK(20);
	        		SPI1.begin();
	        		break;
				case MOSI_PIN_28:	// SPI bus 0 alternate 2
	        		SPI.setMOSI(28);
	        		SPI.setMISO(39);
	        		SPI.setSCK(27);
	        		SPI.begin();
	        		break;
				case MOSI_PIN_44:	// SPI bus 2 default
	        		SPI2.setMOSI(44);
	        		SPI2.setMISO(45);
	        		SPI2.setSCK(46);
	        		SPI2.begin();
	        		break;
				case MOSI_PIN_52:	// SPI bus 2 alternate
	        		SPI2.setMOSI(52);
	        		SPI2.setMISO(51);
	        		SPI2.setSCK(53);
	        		SPI2.begin();
	        		break;
	        }

        #endif

        // Teensy LC 
		#if defined(__MKL26Z64__)

			// configure and begin the SPI
	        switch( _mosiPin ){

	        	case MOSI_PIN_0:	// SPI bus 1 default
	        		SPI1.setMOSI(0);
	        		SPI1.setMISO(1);
	        		SPI1.setSCK(20);
	        		SPI1.begin();
	        		break;
				case MOSI_PIN_7:	// SPI bus 0 alternate 1
				    SPI.setMOSI(7);
	        		SPI.setMISO(8);
	        		SPI.setSCK(14);
	        		SPI.begin();
	        		break;
				case MOSI_PIN_11:	// SPI bus 0 default
					SPI.setMOSI(11);
	        		SPI.setMISO(12);
	        		SPI.setSCK(13);
	        		SPI.begin();
	        		break;
				case MOSI_PIN_21:	// SPI bus 1 alternate
		        	SPI1.setMOSI(21);
	        		SPI1.setMISO(5);
	        		SPI1.setSCK(20);
	        		SPI1.begin();
	        		break;
	        }

		#endif
    }
    else{ // using I2C for communication

        if( !_userDefI2C ) { // setup the I2C pins and pullups based on bus number if not defined by user
            /* setting the I2C pins, pullups, and protecting against _bus out of range */
            _pullups = I2C_PULLUP_EXT; // default to external pullups

            #if defined(__MK20DX128__) // Teensy 3.0
                _pins = I2C_PINS_18_19;
                _bus = 0;
            #endif

            #if defined(__MK20DX256__) // Teensy 3.1/3.2
                if(_bus == 1) {
                    _pins = I2C_PINS_29_30;
                }
                else{
                    _pins = I2C_PINS_18_19;
                    _bus = 0;
                }

            #endif

            #if defined(__MK64FX512__) // Teensy 3.5
                if(_bus == 2) {
                    _pins = I2C_PINS_3_4;
                }
                else if(_bus == 1) {
                    _pins = I2C_PINS_37_38;
                }
                else{
                    _pins = I2C_PINS_18_19;
                    _bus = 0;
                }

            #endif

            #if defined(__MK66FX1M0__) // Teensy 3.6
                if(_bus == 3) {
                    _pins = I2C_PINS_56_57;
                }
                else if(_bus == 2) {
                    _pins = I2C_PINS_3_4;
                }
                else if(_bus == 1) {
                    _pins = I2C_PINS_37_38;
                }
                else{
                    _pins = I2C_PINS_18_19;
                    _bus = 0;
                }

            #endif

            #if defined(__MKL26Z64__) // Teensy LC
                if(_bus == 1) {
                    _pins = I2C_PINS_22_23;
                }
                else{
                    _pins = I2C_PINS_18_19;
                    _bus = 0;
                }

            #endif
        }

        // starting the I2C bus
        i2c_t3(_bus).begin(I2C_MASTER, 0, _pins, _pullups, _i2cRate);
    }

    // select clock source to gyro
    // if( !writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) ){
    //     return -1;
    // }

    // reset the ICM20608G
    writeRegister(PWR_MGMNT_1,PWR_RESET);

    // wait for ICM20608G to come back up
    delay(1000);


    // select clock source to gyro and disable sleep mode
    if( !writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) ){
        return -1;
    }

    if( !writeRegister(USER_CTRL,I2C_DIS) ){
        return -1;
    }

    
    // check the WHO AM I byte, expected value is 0xAF (decimal 175)
    if( whoAmI() != 175 ){
        return -1;
    }

    // enable accelerometer and gyro
    if( !writeRegister(PWR_MGMNT_2,SEN_ENABLE) ){
        return -1;
    }

    /* setup the accel and gyro ranges */
    switch(accelRange) {

        case ACCEL_RANGE_2G:
            // setting the accel range to 2G
            if( !writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_2G) ){
                return -1;
            }
            _accelScale = G * 2.0f/32767.5f; // setting the accel scale to 2G
            break;

        case ACCEL_RANGE_4G:
            // setting the accel range to 4G
            if( !writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_4G) ){
                return -1;
            }
            _accelScale = G * 4.0f/32767.5f; // setting the accel scale to 4G
            break;

        case ACCEL_RANGE_8G:
            // setting the accel range to 8G
            if( !writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_8G) ){
                return -1;
            }
            _accelScale = G * 8.0f/32767.5f; // setting the accel scale to 8G
            break;

        case ACCEL_RANGE_16G:
            // setting the accel range to 16G
            if( !writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_16G) ){
                return -1;
            }
            _accelScale = G * 16.0f/32767.5f; // setting the accel scale to 16G
            break;
    }

    switch(gyroRange) {
        case GYRO_RANGE_250DPS:
            // setting the gyro range to 250DPS
            if( !writeRegister(GYRO_CONFIG,GYRO_FS_SEL_250DPS) ){
                return -1;
            }
            _gyroScale = 250.0f/32767.5f * _d2r; // setting the gyro scale to 250DPS
            break;

        case GYRO_RANGE_500DPS:
            // setting the gyro range to 500DPS
            if( !writeRegister(GYRO_CONFIG,GYRO_FS_SEL_500DPS) ){
                return -1;
            }
            _gyroScale = 500.0f/32767.5f * _d2r; // setting the gyro scale to 500DPS
            break;

        case GYRO_RANGE_1000DPS:
            // setting the gyro range to 1000DPS
            if( !writeRegister(GYRO_CONFIG,GYRO_FS_SEL_1000DPS) ){
                return -1;
            }
            _gyroScale = 1000.0f/32767.5f * _d2r; // setting the gyro scale to 1000DPS
            break;

        case GYRO_RANGE_2000DPS:
            // setting the gyro range to 2000DPS
            if( !writeRegister(GYRO_CONFIG,GYRO_FS_SEL_2000DPS) ){
                return -1;
            }
            _gyroScale = 2000.0f/32767.5f * _d2r; // setting the gyro scale to 2000DPS
            break;
    }

    // select clock source to gyro
    // if( !writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) ){
    //     return -1;
    // }

    // successful init, return 0
    return 0;
}

/* sets the DLPF and interrupt settings */
int ICM20608G::setFilt(icm20608G_gyro_dlpf_bandwidth gyro_bandwidth, icm20608G_accel_dlpf_bandwidth accel_bandwidth, uint8_t SRD){
    uint8_t GYRO_CONFIG_TEMP;
    readRegisters(GYRO_CONFIG, 1, &GYRO_CONFIG_TEMP);

    switch(gyro_bandwidth) {
        case GYRO_BYPASS_DLPF_BANDWIDTH_8173HZ:
            if( !writeRegister(GYRO_CONFIG, (GYRO_CONFIG_TEMP & 0xFC) | GYRO_BYPASS_DLPF_8173) ){ // setting gyro bandwidth to 184Hz
                return -1;
            }
            break;

        case GYRO_BYPASS_DLPF_BANDWIDTH_3281HZ:
            if( !writeRegister(GYRO_CONFIG, (GYRO_CONFIG_TEMP & 0xFC) | GYRO_BYPASS_DLPF_3281) ){ // setting gyro bandwidth to 184Hz
                return -1;
            }
            break;

        case GYRO_DLPF_BANDWIDTH_3281HZ:
            if( !writeRegister(GYRO_CONFIG, (GYRO_CONFIG_TEMP & 0xFC) | GYRO_USE_DLPF) ){ // setting gyro bandwidth to 184Hz
                return -1;
            }
            if( !writeRegister(CONFIG,GYRO_DLPF_3281) ){ // setting gyro bandwidth to 184Hz
                return -1;
            }
            break;

        case GYRO_DLPF_BANDWIDTH_250HZ:
            if( !writeRegister(GYRO_CONFIG, (GYRO_CONFIG_TEMP & 0xFC) | GYRO_USE_DLPF) ){ // setting gyro bandwidth to 184Hz
                return -1;
            }
            if( !writeRegister(CONFIG,GYRO_DLPF_250) ){ // setting gyro bandwidth to 184Hz
                return -1;
            }
            break;

        case GYRO_DLPF_BANDWIDTH_176HZ:
            if( !writeRegister(GYRO_CONFIG, (GYRO_CONFIG_TEMP & 0xFC) | GYRO_USE_DLPF) ){ // setting gyro bandwidth to 184Hz
                return -1;
            }
            if( !writeRegister(CONFIG,GYRO_DLPF_176) ){ // setting gyro bandwidth to 184Hz
                return -1;
            }
            break;

        case GYRO_DLPF_BANDWIDTH_92HZ:
            if( !writeRegister(GYRO_CONFIG, (GYRO_CONFIG_TEMP & 0xFC) | GYRO_USE_DLPF) ){ // setting gyro bandwidth to 184Hz
                return -1;
            }
            if( !writeRegister(CONFIG,GYRO_DLPF_92) ){ // setting gyro bandwidth to 92Hz
                return -1;
            }
            break; 

        case GYRO_DLPF_BANDWIDTH_41HZ:
            if( !writeRegister(GYRO_CONFIG, (GYRO_CONFIG_TEMP & 0xFC) | GYRO_USE_DLPF) ){ // setting gyro bandwidth to 184Hz
                return -1;
            }
            if( !writeRegister(CONFIG,GYRO_DLPF_41) ){ // setting gyro bandwidth to 41Hz
                return -1;
            } 
            break;

        case GYRO_DLPF_BANDWIDTH_20HZ:
            if( !writeRegister(GYRO_CONFIG, (GYRO_CONFIG_TEMP & 0xFC) | GYRO_USE_DLPF) ){ // setting gyro bandwidth to 184Hz
                return -1;
            }
            if( !writeRegister(CONFIG,GYRO_DLPF_20) ){ // setting gyro bandwidth to 20Hz
                return -1;
            }
            break;

        case GYRO_DLPF_BANDWIDTH_10HZ:
            if( !writeRegister(GYRO_CONFIG, (GYRO_CONFIG_TEMP & 0xFC) | GYRO_USE_DLPF) ){ // setting gyro bandwidth to 184Hz
                return -1;
            }
            if( !writeRegister(CONFIG,GYRO_DLPF_10) ){ // setting gyro bandwidth to 10Hz
                return -1;
            }
            break;

        case GYRO_DLPF_BANDWIDTH_5HZ:
            if( !writeRegister(GYRO_CONFIG, (GYRO_CONFIG_TEMP & 0xFC) | GYRO_USE_DLPF) ){ // setting gyro bandwidth to 184Hz
                return -1;
            }
            if( !writeRegister(CONFIG,GYRO_DLPF_5) ){ // setting gyro bandwidth to 5Hz
                return -1;
            }
            break; 
    }

    switch(accel_bandwidth) {
        case ACCEL_BYPASS_DLPF_BANDWIDTH_1046HZ:
            if( !writeRegister(ACCEL_CONFIG2,ACCEL_BYPASS_DLPF_1046) ){ // setting accel bandwidth to 184Hz
                return -1;
            } 
            break; 

        case ACCEL_DLPF_BANDWIDTH_420HZ:
            if( !writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_420) ){ // setting accel bandwidth to 184Hz
                return -1;
            } 
            break; 

        case ACCEL_DLPF_BANDWIDTH_218HZ:
            if( !writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_218) ){ // setting accel bandwidth to 184Hz
                return -1;
            } 
            break; 

        case ACCEL_DLPF_BANDWIDTH_99HZ:
            if( !writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_99) ){ // setting accel bandwidth to 92Hz
                return -1;
            } 
            break; 

        case ACCEL_DLPF_BANDWIDTH_45HZ:
            if( !writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_45) ){ // setting accel bandwidth to 41Hz
                return -1;
            } 
            break; 

        case ACCEL_DLPF_BANDWIDTH_21HZ:
            if( !writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_21) ){ // setting accel bandwidth to 20Hz
                return -1;
            } 
            break;

        case ACCEL_DLPF_BANDWIDTH_10HZ:
            if( !writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_10) ){ // setting accel bandwidth to 10Hz
                return -1;
            } 
            break;

        case ACCEL_DLPF_BANDWIDTH_5HZ:
            if( !writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_5) ){ // setting accel bandwidth to 5Hz
                return -1;
            } 
            break; 
    }

    /* setting the sample rate divider */
    if( !writeRegister(SMPDIV,SRD) ){ // setting the sample rate divider
        return -1;
    } 

    /* setting the interrupt */
    if( !writeRegister(INT_PIN_CFG,INT_PULSE_50US) ){ // setup interrupt, 50 us pulse
        return -1;
    }  
    if( !writeRegister(INT_ENABLE,INT_RAW_RDY_EN) ){ // set to data ready
        return -1;
    }  

    // successful filter setup, return 0
    return 0; 
}

/* enables and disables the interrupt */
int ICM20608G::enableInt(bool enable){

	if(enable){
		/* setting the interrupt */
	    if( !writeRegister(INT_PIN_CFG,INT_PULSE_50US) ){ // setup interrupt, 50 us pulse
	        return -1;
	    }  
	    if( !writeRegister(INT_ENABLE,INT_RAW_RDY_EN) ){ // set to data ready
	        return -1;
	    }  
	}
	else{
	    if( !writeRegister(INT_ENABLE,INT_DISABLE) ){ // disable interrupt
	        return -1;
	    }  
	}

    // successful interrupt setup, return 0
    return 0; 
}

/* get accelerometer data given pointers to store the three values, return data as counts */
void ICM20608G::getAccelCounts(int16_t* ax, int16_t* ay, int16_t* az){
    uint8_t buff[6];
    int16_t axx, ayy, azz;
    _useSPIHS = true; // use the high speed SPI for data readout

    readRegisters(ACCEL_OUT, sizeof(buff), &buff[0]); // grab the data from the ICM20608G

    axx = (((int16_t)buff[0]) << 8) | buff[1];  // combine into 16 bit values
    ayy = (((int16_t)buff[2]) << 8) | buff[3];
    azz = (((int16_t)buff[4]) << 8) | buff[5];

    *ax = tX[0]*axx + tX[1]*ayy + tX[2]*azz; // transform axes
    *ay = tY[0]*axx + tY[1]*ayy + tY[2]*azz;
    *az = tZ[0]*axx + tZ[1]*ayy + tZ[2]*azz;
}

/* get accelerometer data given pointers to store the three values */
void ICM20608G::getAccel(float* ax, float* ay, float* az){
    int16_t accel[3];

    getAccelCounts(&accel[0], &accel[1], &accel[2]);

    *ax = ((float) accel[0]) * _accelScale; // typecast and scale to values
    *ay = ((float) accel[1]) * _accelScale;
    *az = ((float) accel[2]) * _accelScale;
}

/* get gyro data given pointers to store the three values, return data as counts */
void ICM20608G::getGyroCounts(int16_t* gx, int16_t* gy, int16_t* gz){
    uint8_t buff[6];
    int16_t gxx, gyy, gzz;
    _useSPIHS = true; // use the high speed SPI for data readout

    readRegisters(GYRO_OUT, sizeof(buff), &buff[0]); // grab the data from the ICM20608G

    gxx = (((int16_t)buff[0]) << 8) | buff[1];  // combine into 16 bit values
    gyy = (((int16_t)buff[2]) << 8) | buff[3];
    gzz = (((int16_t)buff[4]) << 8) | buff[5];

    *gx = tX[0]*gxx + tX[1]*gyy + tX[2]*gzz; // transform axes
    *gy = tY[0]*gxx + tY[1]*gyy + tY[2]*gzz;
    *gz = tZ[0]*gxx + tZ[1]*gyy + tZ[2]*gzz;
}

/* get gyro data given pointers to store the three values */
void ICM20608G::getGyro(float* gx, float* gy, float* gz){
    int16_t gyro[3];

    getGyroCounts(&gyro[0], &gyro[1], &gyro[2]);

    *gx = ((float) gyro[0]) * _gyroScale; // typecast and scale to values
    *gy = ((float) gyro[1]) * _gyroScale;
    *gz = ((float) gyro[2]) * _gyroScale;
}

/* get temperature data given pointer to store the value, return data as counts */
void ICM20608G::getTempCounts(int16_t* t){
    uint8_t buff[2];
    _useSPIHS = true; // use the high speed SPI for data readout

    readRegisters(TEMP_OUT, sizeof(buff), &buff[0]); // grab the data from the ICM20608G

    *t = (((int16_t)buff[0]) << 8) | buff[1];  // combine into 16 bit value and return
}

/* get temperature data given pointer to store the values */
void ICM20608G::getTemp(float* t){
    int16_t tempCount;

    getTempCounts(&tempCount);

    *t = (( ((float) tempCount) - _tempOffset )/_tempScale) + _tempOffset;
}

/* get accelerometer and gyro data given pointers to store values, return data as counts */
void ICM20608G::getMotion6Counts(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz){
    uint8_t buff[14];
    int16_t axx, ayy, azz, gxx, gyy, gzz;
    _useSPIHS = true; // use the high speed SPI for data readout

    readRegisters(ACCEL_OUT, sizeof(buff), &buff[0]); // grab the data from the ICM20608G

    axx = (((int16_t)buff[0]) << 8) | buff[1];  // combine into 16 bit values
    ayy = (((int16_t)buff[2]) << 8) | buff[3];
    azz = (((int16_t)buff[4]) << 8) | buff[5];

    gxx = (((int16_t)buff[8]) << 8) | buff[9];
    gyy = (((int16_t)buff[10]) << 8) | buff[11];
    gzz = (((int16_t)buff[12]) << 8) | buff[13];

    *ax = tX[0]*axx + tX[1]*ayy + tX[2]*azz; // transform axes
    *ay = tY[0]*axx + tY[1]*ayy + tY[2]*azz;
    *az = tZ[0]*axx + tZ[1]*ayy + tZ[2]*azz;

    *gx = tX[0]*gxx + tX[1]*gyy + tX[2]*gzz;
    *gy = tY[0]*gxx + tY[1]*gyy + tY[2]*gzz;
    *gz = tZ[0]*gxx + tZ[1]*gyy + tZ[2]*gzz;
}

/* get accelerometer and gyro data given pointers to store values */
void ICM20608G::getMotion6(float* ax, float* ay, float* az, float* gx, float* gy, float* gz){
    int16_t accel[3];
    int16_t gyro[3];

    getMotion6Counts(&accel[0], &accel[1], &accel[2], &gyro[0], &gyro[1], &gyro[2]);

    *ax = ((float) accel[0]) * _accelScale; // typecast and scale to values
    *ay = ((float) accel[1]) * _accelScale;
    *az = ((float) accel[2]) * _accelScale;

    *gx = ((float) gyro[0]) * _gyroScale;
    *gy = ((float) gyro[1]) * _gyroScale;
    *gz = ((float) gyro[2]) * _gyroScale;
}

/* get accelerometer, gyro and temperature data given pointers to store values, return data as counts */
void ICM20608G::getMotion7Counts(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* t){
    uint8_t buff[14];
    int16_t axx, ayy, azz, gxx, gyy, gzz;
    _useSPIHS = true; // use the high speed SPI for data readout

    readRegisters(ACCEL_OUT, sizeof(buff), &buff[0]); // grab the data from the ICM20608G

    axx = (((int16_t)buff[0]) << 8) | buff[1];  // combine into 16 bit values
    ayy = (((int16_t)buff[2]) << 8) | buff[3];
    azz = (((int16_t)buff[4]) << 8) | buff[5];

    *t = (((int16_t)buff[6]) << 8) | buff[7];

    gxx = (((int16_t)buff[8]) << 8) | buff[9];
    gyy = (((int16_t)buff[10]) << 8) | buff[11];
    gzz = (((int16_t)buff[12]) << 8) | buff[13];

    *ax = tX[0]*axx + tX[1]*ayy + tX[2]*azz; // transform axes
    *ay = tY[0]*axx + tY[1]*ayy + tY[2]*azz;
    *az = tZ[0]*axx + tZ[1]*ayy + tZ[2]*azz;

    *gx = tX[0]*gxx + tX[1]*gyy + tX[2]*gzz;
    *gy = tY[0]*gxx + tY[1]*gyy + tY[2]*gzz;
    *gz = tZ[0]*gxx + tZ[1]*gyy + tZ[2]*gzz;
}

/* get accelerometer, gyro, and temperature data given pointers to store values */
void ICM20608G::getMotion7(float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* t){
    int16_t accel[3];
    int16_t gyro[3];
    int16_t tempCount;

    getMotion7Counts(&accel[0], &accel[1], &accel[2], &gyro[0], &gyro[1], &gyro[2], &tempCount);

    *ax = ((float) accel[0]) * _accelScale; // typecast and scale to values
    *ay = ((float) accel[1]) * _accelScale;
    *az = ((float) accel[2]) * _accelScale;

    *gx = ((float) gyro[0]) * _gyroScale;
    *gy = ((float) gyro[1]) * _gyroScale;
    *gz = ((float) gyro[2]) * _gyroScale;

    *t = (( ((float) tempCount) - _tempOffset )/_tempScale) + _tempOffset; 
}

/* writes a byte to ICM20608G register given a register address and data */
bool ICM20608G::writeRegister(uint8_t subAddress, uint8_t data){
    uint8_t buff[1];

    /* write data to device */
    if( _useSPI ){

    	// Teensy 3.0 || Teensy 3.1/3.2
		#if defined(__MK20DX128__) || defined(__MK20DX256__)

	    	if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
		        SPI.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
		        digitalWriteFast(_csPin,LOW); // select the ICM20608G chip
		        SPI.transfer(subAddress); // write the register address
		        SPI.transfer(data); // write the data
		        digitalWriteFast(_csPin,HIGH); // deselect the ICM20608G chip
		        SPI.endTransaction(); // end the transaction
	    	}

    	#endif

        // Teensy 3.5 || Teensy 3.6 
		#if defined(__MK64FX512__) || defined(__MK66FX1M0__)

	    	if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)||(_mosiPin == MOSI_PIN_28)){
		        SPI.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
		        digitalWriteFast(_csPin,LOW); // select the ICM20608G chip
		        SPI.transfer(subAddress); // write the register address
		        SPI.transfer(data); // write the data
		        digitalWriteFast(_csPin,HIGH); // deselect the ICM20608G chip
		        SPI.endTransaction(); // end the transaction
	    	}
	    	else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
		        SPI1.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
		        digitalWriteFast(_csPin,LOW); // select the ICM20608G chip
		        SPI1.transfer(subAddress); // write the register address
		        SPI1.transfer(data); // write the data
		        digitalWriteFast(_csPin,HIGH); // deselect the ICM20608G chip
		        SPI1.endTransaction(); // end the transaction
	    	}
	    	else if((_mosiPin == MOSI_PIN_44)||(_mosiPin == MOSI_PIN_52)){
		    	SPI2.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
		        digitalWriteFast(_csPin,LOW); // select the ICM20608G chip
		        SPI2.transfer(subAddress); // write the register address
		        SPI2.transfer(data); // write the data
		        digitalWriteFast(_csPin,HIGH); // deselect the ICM20608G chip
		        SPI2.endTransaction(); // end the transaction	
	    	}

    	#endif

        // Teensy LC 
		#if defined(__MKL26Z64__)

	    	if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
		        SPI.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
		        digitalWriteFast(_csPin,LOW); // select the ICM20608G chip
		        SPI.transfer(subAddress); // write the register address
		        SPI.transfer(data); // write the data
		        digitalWriteFast(_csPin,HIGH); // deselect the ICM20608G chip
		        SPI.endTransaction(); // end the transaction
	    	}
	    	else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
		        SPI1.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
		        digitalWriteFast(_csPin,LOW); // select the ICM20608G chip
		        SPI1.transfer(subAddress); // write the register address
		        SPI1.transfer(data); // write the data
		        digitalWriteFast(_csPin,HIGH); // deselect the ICM20608G chip
		        SPI1.endTransaction(); // end the transaction
	    	}

    	#endif
    }
    else{
      	i2c_t3(_bus).beginTransmission(_address); // open the device
      	i2c_t3(_bus).write(subAddress); // write the register address
      	i2c_t3(_bus).write(data); // write the data
      	i2c_t3(_bus).endTransmission();
    }
    delay(10); // need to slow down how fast I write to ICM20608G

  	/* read back the register */
  	readRegisters(subAddress,sizeof(buff),&buff[0]);

  	/* check the read back register against the written register */
  	if(buff[0] == data) {
  		return true;
  	}
  	else{
  		return false;
  	}
}

/* reads registers from ICM20608G given a starting register address, number of bytes, and a pointer to store data */
void ICM20608G::readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest){
    
    if( _useSPI ){

    	// Teensy 3.0 || Teensy 3.1/3.2
		#if defined(__MK20DX128__) || defined(__MK20DX256__)

	    	if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
		        // begin the transaction
		        if(_useSPIHS){
		            SPI.beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
		        }
		        else{
		            SPI.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
		        }
		        digitalWriteFast(_csPin,LOW); // select the ICM20608G chip

		        SPI.transfer(subAddress | SPI_READ); // specify the starting register address

		        for(uint8_t i = 0; i < count; i++){
		            dest[i] = SPI.transfer(0x00); // read the data
		        }

		        digitalWriteFast(_csPin,HIGH); // deselect the ICM20608G chip
		        SPI.endTransaction(); // end the transaction
	    	}

    	#endif

        // Teensy 3.5 || Teensy 3.6 
		#if defined(__MK64FX512__) || defined(__MK66FX1M0__)

	    	if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)||(_mosiPin == MOSI_PIN_28)){
		        // begin the transaction
		        if(_useSPIHS){
		            SPI.beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
		        }
		        else{
		            SPI.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
		        }
		        digitalWriteFast(_csPin,LOW); // select the ICM20608G chip

		        SPI.transfer(subAddress | SPI_READ); // specify the starting register address

		        for(uint8_t i = 0; i < count; i++){
		            dest[i] = SPI.transfer(0x00); // read the data
		        }

		        digitalWriteFast(_csPin,HIGH); // deselect the ICM20608G chip
		        SPI.endTransaction(); // end the transaction
	    	}
	    	else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
		        // begin the transaction
		        if(_useSPIHS){
		            SPI1.beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
		        }
		        else{
		            SPI1.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
		        }
		        digitalWriteFast(_csPin,LOW); // select the ICM20608G chip

		        SPI1.transfer(subAddress | SPI_READ); // specify the starting register address

		        for(uint8_t i = 0; i < count; i++){
		            dest[i] = SPI1.transfer(0x00); // read the data
		        }

		        digitalWriteFast(_csPin,HIGH); // deselect the ICM20608G chip
		        SPI1.endTransaction(); // end the transaction
	    	}
	    	else if((_mosiPin == MOSI_PIN_44)||(_mosiPin == MOSI_PIN_52)){
		        // begin the transaction
		        if(_useSPIHS){
		            SPI2.beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
		        }
		        else{
		            SPI2.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
		        }
		        digitalWriteFast(_csPin,LOW); // select the ICM20608G chip

		        SPI2.transfer(subAddress | SPI_READ); // specify the starting register address

		        for(uint8_t i = 0; i < count; i++){
		            dest[i] = SPI.transfer(0x00); // read the data
		        }

		        digitalWriteFast(_csPin,HIGH); // deselect the ICM20608G chip
		        SPI2.endTransaction(); // end the transaction
	    	}

    	#endif

        // Teensy LC 
		#if defined(__MKL26Z64__)

	    	if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
		        // begin the transaction
		        if(_useSPIHS){
		            SPI.beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
		        }
		        else{
		            SPI.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
		        }
		        digitalWriteFast(_csPin,LOW); // select the ICM20608G chip

		        SPI.transfer(subAddress | SPI_READ); // specify the starting register address

		        for(uint8_t i = 0; i < count; i++){
		            dest[i] = SPI.transfer(0x00); // read the data
		        }

		        digitalWriteFast(_csPin,HIGH); // deselect the ICM20608G chip
		        SPI.endTransaction(); // end the transaction
	    	}
	    	else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
		        // begin the transaction
		        if(_useSPIHS){
		            SPI1.beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
		        }
		        else{
		            SPI1.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
		        }
		        digitalWriteFast(_csPin,LOW); // select the ICM20608G chip

		        SPI1.transfer(subAddress | SPI_READ); // specify the starting register address

		        for(uint8_t i = 0; i < count; i++){
		            dest[i] = SPI1.transfer(0x00); // read the data
		        }

		        digitalWriteFast(_csPin,HIGH); // deselect the ICM20608G chip
		        SPI1.endTransaction(); // end the transaction
	    	}

    	#endif
    }
    else{
        i2c_t3(_bus).beginTransmission(_address); // open the device
        i2c_t3(_bus).write(subAddress); // specify the starting register address
        i2c_t3(_bus).endTransmission(false);

        i2c_t3(_bus).requestFrom(_address, count); // specify the number of bytes to receive

        uint8_t i = 0; // read the data into the buffer
        while( i2c_t3(_bus).available() ){
            dest[i++] = i2c_t3(_bus).readByte();
        }
    }
}

/* gets the ICM20608G WHO_AM_I register value, expected to be 0x71 */
uint8_t ICM20608G::whoAmI(){
    uint8_t buff[1];

    // read the WHO AM I register
    readRegisters(WHO_AM_I,sizeof(buff),&buff[0]);

    // return the register value
    return buff[0];
}

#endif
