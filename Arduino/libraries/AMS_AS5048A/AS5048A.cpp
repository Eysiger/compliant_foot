/*
AS5048A.cpp
Roman Käslin
rkaeslin@student.ethz.ch
2017-05-16
*/

// Teensy 3.0 || Teensy 3.1/3.2 || Teensy 3.5 || Teensy 3.6 || Teensy LC 
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || \
	defined(__MK66FX1M0__) || defined(__MKL26Z64__)

#include "Arduino.h"
#include "AS5048A.h"
#include "SPI.h" // SPI Library

/* AS5048A object, input the SPI CS Pin */
AS5048A::AS5048A(uint8_t csPin){
    _csPin = csPin; // SPI CS Pin
    _mosiPin = MOSI_PIN_11;	// SPI MOSI Pin, set to default
    _useSPIHS = false; // defaul to low speed SPI transactions until data reads start to occur
}

/* AS5048A object, input the SPI CS Pin and MOSI Pin */
AS5048A::AS5048A(uint8_t csPin, spi_mosi_pin pin){
    _csPin = csPin; // SPI CS Pin
    _mosiPin = pin;	// SPI MOSI Pin
    _useSPIHS = false; // defaul to low speed SPI transactions until data reads start to occur
}

/* starts I2C communication and sets up the AS5048A */
bool AS5048A::begin(){
    uint8_t buff[3];
    uint8_t data[7];

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

    // reset the AS5048A
    // writeRegister(,);

    // wait for AS5048A to come back up
    //delay(1000);

    
    // check the field strength (0 high magnetic field, 255 low magnetic field)
    if( fieldStrength() < 255 ){
        return false;
    }

    // successful init, return true
    return true;
}

/* get encoder data given pointers to store the value, return data as counts */
bool AS5048A::getAngleCounts(uint16_t* angle){
    uint16_t buff[1];
    _useSPIHS = true; // use the high speed SPI for data readout

    readRegisters(READ_ANGLE, sizeof(buff), &buff[0]); // grab the data from the AS5048A

    *angle = buff[0] & 0x3FFF;  // store to output

    if (!check(buff[0])){  // check parity and error flag
        return false;
    }
    return true; // return if successfull
}

/* get encoder data given pointers to store the value */
bool AS5048A::getAngle(float* angle){
    uint16_t count;

    bool check = getAngleCounts(&count); // get angle inclusive check value

    *angle = ((float) count) * _angleScale; // typecast and scale to values

    return check; // return if successfull (0)
}

/* set actual encoder position as actual zero position */
bool AS5048A::setZero(){
    uint16_t angle;

    getAngleCounts(&angle);
    uint16_t high = (angle & 0x3FC0) >> 6;
    uint16_t low = angle & 0x001F;

    writeRegister(WRITE_ZERO_POS1, high);
    writeRegister(WRITE_ZERO_POS2, low);

    uint16_t buff[2];
    readRegisters(READ_ZERO_POS1, 1, &buff[0]);
    readRegisters(READ_ZERO_POS2, 1, &buff[1]);

    if (!check(buff[0])){  // check parity and error flag
        return false;
    }
    if (!check(buff[1])){  // check parity and error flag
        return false;
    }
    
    if(( (buff[0] & 0x00FF) << 6 ) | (buff[1] & 0x001F) == angle) { // check set value
        return true;
    }
    else{
        return false;
    }
}

/* reads out the strength of the magnetic field */
int AS5048A::fieldStrength(){
    uint16_t buff[1];

    readRegisters(READ_DIAGNOSTICS, sizeof(buff), &buff[0]); // grab the data from the AS5048A
    
    if (!check(buff[0])){  // check parity and error flag
        return 255;
    }

    return buff[0] & 0x00FF; // return last 8 bit of diagnostics that represent field strength 
                             // (0 high magnetic field, 255 low magnetic field)
}

/* check parity and error flag */
bool AS5048A::check(uint16_t data){
    if (data & 0x4000) {  // check for error flag
        clearError();     // reset error flag
        return false;
    }

    uint16_t count = 0;
    for (int i=0; i<16; i++){       //counts number of set bits
        count += (data & 0x0001);
        data >> 1;
    }

    if (count % 2){                 // checks for bit unparity
        return false;
    }

    return true;
}

/* clears all errors */
void AS5048A::clearError(){
    uint16_t buff[1];
    readRegisters(CLEAR_ERROR, 0, &buff[0]); // send clear error message
}

/* writes a byte to AS5048A register given a register address and data */
void AS5048A::writeRegister(uint16_t subAddress, uint16_t data){
    uint8_t buff[1];

    /* write data to device */

	// Teensy 3.0 || Teensy 3.1/3.2
	#if defined(__MK20DX128__) || defined(__MK20DX256__)

    	if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
	        SPI.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE1)); // begin the transaction
	        digitalWriteFast(_csPin,LOW); // select the AS5048A chip
	        SPI.transfer16(subAddress); // write the register address
            digitalWriteFast(_csPin,HIGH);
            delayMicroseconds(1);
            digitalWriteFast(_csPin,LOW);
	        SPI.transfer16(data); // write the data
	        digitalWriteFast(_csPin,HIGH); // deselect the AS5048A chip
	        SPI.endTransaction(); // end the transaction
    	}

	#endif

    // Teensy 3.5 || Teensy 3.6 
	#if defined(__MK64FX512__) || defined(__MK66FX1M0__)

    	if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)||(_mosiPin == MOSI_PIN_28)){
	        SPI.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE1)); // begin the transaction
	        digitalWriteFast(_csPin,LOW); // select the AS5048A chip
	        SPI.transfer16(subAddress); // write the register address
            digitalWriteFast(_csPin,HIGH);
            delayMicroseconds(1);
            digitalWriteFast(_csPin,LOW);
	        SPI.transfer16(data); // write the data
	        digitalWriteFast(_csPin,HIGH); // deselect the AS5048A chip
	        SPI.endTransaction(); // end the transaction
    	}
    	else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
	        SPI1.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE1)); // begin the transaction
	        digitalWriteFast(_csPin,LOW); // select the AS5048A chip
	        SPI1.transfer16(subAddress); // write the register address
            digitalWriteFast(_csPin,HIGH);
            delayMicroseconds(1);
            digitalWriteFast(_csPin,LOW);
	        SPI1.transfer16(data); // write the data
	        digitalWriteFast(_csPin,HIGH); // deselect the AS5048A chip
	        SPI1.endTransaction(); // end the transaction
    	}
    	else if((_mosiPin == MOSI_PIN_44)||(_mosiPin == MOSI_PIN_52)){
	    	SPI2.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE1)); // begin the transaction
	        digitalWriteFast(_csPin,LOW); // select the AS5048A chip
	        SPI2.transfer16(subAddress); // write the register address
            digitalWriteFast(_csPin,HIGH);
            delayMicroseconds(1);
            digitalWriteFast(_csPin,LOW);
	        SPI2.transfer16(data); // write the data
	        digitalWriteFast(_csPin,HIGH); // deselect the AS5048A chip
	        SPI2.endTransaction(); // end the transaction	
    	}

	#endif

    // Teensy LC 
	#if defined(__MKL26Z64__)

    	if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
	        SPI.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE1)); // begin the transaction
	        digitalWriteFast(_csPin,LOW); // select the AS5048A chip
	        SPI.transfer16(subAddress); // write the register address
            digitalWriteFast(_csPin,HIGH);
            delayMicroseconds(1);
            digitalWriteFast(_csPin,LOW);
	        SPI.transfer16(data); // write the data
	        digitalWriteFast(_csPin,HIGH); // deselect the AS5048A chip
	        SPI.endTransaction(); // end the transaction
    	}
    	else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
	        SPI1.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE1)); // begin the transaction
	        digitalWriteFast(_csPin,LOW); // select the AS5048A chip
	        SPI1.transfer16(subAddress); // write the register address
            digitalWriteFast(_csPin,HIGH);
            delayMicroseconds(1);
            digitalWriteFast(_csPin,LOW);
	        SPI1.transfer16(data); // write the data
	        digitalWriteFast(_csPin,HIGH); // deselect the AS5048A chip
	        SPI1.endTransaction(); // end the transaction
    	}

	#endif

    delay(10); // need to slow down how fast I write to AS5048A
}

/* reads registers from AS5048A given a starting register address, number of bytes, and a pointer to store data */
void AS5048A::readRegisters(uint16_t subAddress, uint8_t count, uint16_t* dest){

	// Teensy 3.0 || Teensy 3.1/3.2
	#if defined(__MK20DX128__) || defined(__MK20DX256__)

    	if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
	        // begin the transaction
	        if(_useSPIHS){
	            SPI.beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE1));
	        }
	        else{
	            SPI.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE1));
	        }
	        digitalWriteFast(_csPin,LOW); // select the AS5048A chip

	        SPI.transfer16(subAddress | SPI_READ); // specify the starting register address

	        for(uint8_t i = 0; i < count; i++){
                digitalWriteFast(_csPin,HIGH);
                delayMicroseconds(1);
                digitalWriteFast(_csPin,LOW);
	            dest[i] = SPI.transfer16(0x00); // read the data
	        }

	        digitalWriteFast(_csPin,HIGH); // deselect the AS5048A chip
	        SPI.endTransaction(); // end the transaction
    	}

	#endif

    // Teensy 3.5 || Teensy 3.6 
	#if defined(__MK64FX512__) || defined(__MK66FX1M0__)

    	if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)||(_mosiPin == MOSI_PIN_28)){
	        // begin the transaction
	        if(_useSPIHS){
	            SPI.beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE1));
	        }
	        else{
	            SPI.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE1));
	        }
	        digitalWriteFast(_csPin,LOW); // select the AS5048A chip

	        SPI.transfer16(subAddress | SPI_READ); // specify the starting register address

	        for(uint8_t i = 0; i < count; i++){
                digitalWriteFast(_csPin,HIGH);
                delayMicroseconds(1);
                digitalWriteFast(_csPin,LOW);
	            dest[i] = SPI.transfer16(0x00); // read the data
	        }

	        digitalWriteFast(_csPin,HIGH); // deselect the AS5048A chip
	        SPI.endTransaction(); // end the transaction
    	}
    	else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
	        // begin the transaction
	        if(_useSPIHS){
	            SPI1.beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE1));
	        }
	        else{
	            SPI1.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE1));
	        }
	        digitalWriteFast(_csPin,LOW); // select the AS5048A chip

	        SPI1.transfer16(subAddress | SPI_READ); // specify the starting register address

	        for(uint8_t i = 0; i < count; i++){
                digitalWriteFast(_csPin,HIGH);
                delayMicroseconds(1);
                digitalWriteFast(_csPin,LOW);
	            dest[i] = SPI1.transfer16(0x00); // read the data
	        }

	        digitalWriteFast(_csPin,HIGH); // deselect the AS5048A chip
	        SPI1.endTransaction(); // end the transaction
    	}
    	else if((_mosiPin == MOSI_PIN_44)||(_mosiPin == MOSI_PIN_52)){
	        // begin the transaction
	        if(_useSPIHS){
	            SPI2.beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE1));
	        }
	        else{
	            SPI2.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE1));
	        }
	        digitalWriteFast(_csPin,LOW); // select the AS5048A chip

	        SPI2.transfer16(subAddress | SPI_READ); // specify the starting register address

	        for(uint8_t i = 0; i < count; i++){
                digitalWriteFast(_csPin,HIGH);
                delayMicroseconds(1);
                digitalWriteFast(_csPin,LOW);
	            dest[i] = SPI.transfer16(0x00); // read the data
	        }

	        digitalWriteFast(_csPin,HIGH); // deselect the AS5048A chip
	        SPI2.endTransaction(); // end the transaction
    	}

	#endif

    // Teensy LC 
	#if defined(__MKL26Z64__)

    	if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
	        // begin the transaction
	        if(_useSPIHS){
	            SPI.beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE1));
	        }
	        else{
	            SPI.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE1));
	        }
	        digitalWriteFast(_csPin,LOW); // select the AS5048A chip

	        SPI.transfer16(subAddress | SPI_READ); // specify the starting register address

	        for(uint8_t i = 0; i < count; i++){
                digitalWriteFast(_csPin,HIGH);
                delayMicroseconds(1);
                digitalWriteFast(_csPin,LOW);
	            dest[i] = SPI.transfer16(0x00); // read the data
	        }

	        digitalWriteFast(_csPin,HIGH); // deselect the AS5048A chip
	        SPI.endTransaction(); // end the transaction
    	}
    	else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
	        // begin the transaction
	        if(_useSPIHS){
	            SPI1.beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE1));
	        }
	        else{
	            SPI1.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE1));
	        }
	        digitalWriteFast(_csPin,LOW); // select the AS5048A chip

	        SPI1.transfer16(subAddress | SPI_READ); // specify the starting register address

	        for(uint8_t i = 0; i < count; i++){
                digitalWriteFast(_csPin,HIGH);
                delayMicroseconds(1);
                digitalWriteFast(_csPin,LOW);
	            dest[i] = SPI1.transfer16(0x00); // read the data
	        }

	        digitalWriteFast(_csPin,HIGH); // deselect the AS5048A chip
	        SPI1.endTransaction(); // end the transaction
    	}

	#endif
}

#endif
