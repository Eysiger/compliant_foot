#include "ICM20608G.h"
#include "AS5048A.h"
#include "AHRS.h"
#include "CommunicationUtils.h" // remove later

#define UART Serial1

const int PinIMU1 = 10;
const int PinIMU2 = 9;
const int PinEnc = 15;
const int PinTx = 26;
const int PinRx = 27;

// an ICM20608G object with the ICM-20608-G sensor on Teensy pin provided
ICM20608G IMUFoot(PinIMU1);
ICM20608G IMUShank(PinIMU2);

// an AS5048A object with the magnetic Encoder AS5048A sensor on Teensy pin provided
AS5048A ENCODER(PinEnc);

// an AHRS object providing an EKF sensor fusion of two IMUs with an encoder in between (initial values of gyro offsets)
AHRS AHRS(-0.0231, 0.0092, 0.0048, 0.0112, 0.0206, -0.0082);

// a timer object for sensor readout at 4kHz
IntervalTimer fourkHzTimer;

const int number = 10;
float ax1[number], ay1[number], az1[number], gx1[number], gy1[number], gz1[number], t1[number];
float ax2[number], ay2[number], az2[number], gx2[number], gy2[number], gz2[number], t2[number];
float angle[number];
float tax1, tay1, taz1, tgx1, tgy1, tgz1, tax2, tay2, taz2, tgx2, tgy2, tgz2, tangle;
int beginStatus1, beginStatus2, beginStatus3;
int i = 0;

float qrel[4];
bool contact;
uint16_t forces[3];
uint16_t torques[3];

float ex[number], ey[number], ez[number], emx[number], emy[number], emz[number];
int variable = 0;
bool started = false;
int sign = 1;
float data[6] = {0,0,0,0,0,0};
int l = 0;

float median(float array[]) {
    // Allocate an array of the same size and sort it.
    int size = sizeof(array);
    float* sorted = new float[size];
    for (int j = 0; j < size; ++j) {
        sorted[j] = array[j];
    }
    for (int j = size - 1; j > 0; --j) {
        for (int k = 0; k < j; ++k) {
            if (sorted[k] > sorted[k+1]) {
                float temp = sorted[k];
                sorted[k] = sorted[k+1];
                sorted[k+1] = temp;
            }
        }
    }

    // Middle or average of middle values in the sorted array.
    float median = 0.0;
    if ((size % 2) == 0) {
        median = (sorted[size/2] + sorted[(size/2) - 1])/2.0;
    } else {
        median = sorted[size/2];
    }
    delete [] sorted;
    return median;
}

float mean(float array[]) {
    int size = sizeof(array);
    float sum = 0.0;
    for (int j = 0; j < size; ++j) {
        sum += array[j];
    }
    return sum/size;
}

uint8_t checksum(uint8_t array[], int first, int last) {
  int count = 0; 
  for (int j=first; j<=last; j++) {
    for (int k=0; k<8; k++) {
      count += (array[j] >> k) & 0x01;
    }
  }
  return (uint8_t) count;
}

void sensorReadout() {
   if(beginStatus1 < 0) {
    delay(1000);
    Serial.println("IMUSole initialization unsuccessful");
    Serial.println("Check IMUSole wiring or try cycling power");
    delay(1000);
  }
  else{
    IMUFoot.getTemp(&t1[i]);                    // workaround to read out temperature first since first readout after AMS AS5048A does not work
    
    // get both the accel (m/s/s) and gyro (rad/s) data
    IMUFoot.getMotion6(&ax1[i], &ay1[i], &az1[i], &gx1[i], &gy1[i], &gz1[i]);
  }
  if(beginStatus2 < 0) {
    delay(1000);
    Serial.println("IMUShank initialization unsuccessful");
    Serial.println("Check IMUShank wiring or try cycling power");
    delay(1000);
  }
  else{
    // get both the accel (m/s/s) and gyro (rad/s) data
    IMUShank.getMotion6(&ax2[i], &ay2[i], &az2[i], &gx2[i], &gy2[i], &gz2[i]);
  }
  if(beginStatus3 < 0) {
    delay(1000);
    Serial.println("ENCODER initialization unsuccessful");
    Serial.println("Check ENCODER wiring or try cycling power");
    delay(1000);
  }
  else{
    if( !ENCODER.getAngle(&angle[i]) ) {
    Serial.println("Data from angular encoder couldn't be gathered.");
    }
//    // used to determine zero position
//    uint16_t counts;
//    ENCODER.getAngleCounts(&counts);
//    Serial.println(counts);

    if (angle[i] > 180) { angle[i] -= 360; }
    angle[i] = angle[i]/180.0f*3.14159265359f;
  }
  // output raw values of sensors
//  Serial.print(ax1[i],6);
//  Serial.print("\t");
//  Serial.print(ay1[i],6);
//  Serial.print("\t");
//  Serial.print(az1[i],6);
//  Serial.print("\t");
//  Serial.print(gx1[i],6);
//  Serial.print("\t");
//  Serial.print(gy1[i],6);
//  Serial.print("\t");
//  Serial.print(gz1[i],6);
//  Serial.print("\t");
//  Serial.print(ax2[i],6);
//  Serial.print("\t");
//  Serial.print(ay2[i],6);
//  Serial.print("\t");
//  Serial.print(az2[i],6);
//  Serial.print("\t");
//  Serial.print(gx2[i],6);
//  Serial.print("\t");
//  Serial.print(gy2[i],6);
//  Serial.print("\t");
//  Serial.print(gz2[i],6);
//  Serial.print("\t");
//  Serial.println(angle[i],6);
  //if(az1[i] >= 4*9.8) { Serial.println("touch down"); }

  i++;

  if(i == number) {
    //WRITE to USB
    int length = 28;
    byte buffer[length];
    buffer[0] = 0xFF;
    buffer[1] = 0xFF;
    
    if (contact) { buffer[2] = 0x03; }
    else { buffer[2] = 0x00; }
    
    buffer[3] = (uint8_t)(forces[0] >> 8);
    buffer[4] = (uint8_t)forces[0];
    buffer[5] = (uint8_t)(forces[1] >> 8);
    buffer[6] = (uint8_t)forces[1];
    buffer[7] = (uint8_t)(forces[2] >> 8);
    buffer[8] = (uint8_t)forces[2];
    buffer[9] = (uint8_t)(torques[0] >> 8);
    buffer[10] = (uint8_t)torques[0];
    buffer[11] = (uint8_t)(torques[1] >> 8);
    buffer[12] = (uint8_t)torques[1];
    buffer[13] = (uint8_t)(torques[2] >> 8);
    buffer[14] = (uint8_t)torques[2];
    
    uint16_t q0 = (qrel[0]+1)*32767.5;
    buffer[15] = (uint8_t)(q0 >> 8);
    buffer[16] = (uint8_t)q0;
    uint16_t q1 = (qrel[1]+1)*32767.5;
    buffer[17] = (uint8_t)(q1 >> 8);
    buffer[18] = (uint8_t)q1;
    uint16_t q2 = (qrel[2]+1)*32767.5;
    buffer[19] = (uint8_t)(q2 >> 8);
    buffer[20] = (uint8_t)q2;
    uint16_t q3 = (qrel[3]+1)*32767.5;
    buffer[21] = (uint8_t)(q3 >> 8);
    buffer[22] = (uint8_t)q3;
    
    uint32_t now = micros();
    buffer[23] = (uint8_t)(now >> 24);
    buffer[24] = (uint8_t)(now >> 16);
    buffer[25] = (uint8_t)(now >> 8);
    buffer[26] = (uint8_t)now;
    
    buffer[27] = checksum(buffer, 2, length-2);
    
//    Serial.write(buffer,length);
//    Serial.send_now();

    i = 0;
  }
}

void setup() {
  Serial.begin(230400); // 230400 for read in on ROS

  pinMode (PinIMU1, OUTPUT);
  digitalWrite (PinIMU1, HIGH);
  pinMode (PinIMU2, OUTPUT);
  digitalWrite (PinIMU2, HIGH);
  pinMode (PinEnc, OUTPUT);
  digitalWrite (PinEnc, HIGH);

  UART.setTX(PinTx);
  UART.setRX(PinRx);

  beginStatus1 = IMUFoot.begin(ACCEL_RANGE_16G,GYRO_RANGE_250DPS);
  beginStatus2 = IMUShank.begin(ACCEL_RANGE_16G,GYRO_RANGE_250DPS);
  beginStatus3 = ENCODER.begin();

  UART.begin(230400);

  IMUFoot.setFilt(GYRO_DLPF_BANDWIDTH_250HZ, ACCEL_BYPASS_DLPF_BANDWIDTH_1046HZ, 0);
  IMUShank.setFilt(GYRO_DLPF_BANDWIDTH_250HZ, ACCEL_BYPASS_DLPF_BANDWIDTH_1046HZ, 0);
  
  uint16_t zeroPos = 10152;//15567;
  if( !ENCODER.setZeroPos(&zeroPos) ) {
    Serial.println("Encoder could not be set to zero.");
  }
  
  fourkHzTimer.begin(sensorReadout, 250);
}

void loop() {
  float q1[4];
  float q2[4];
  noInterrupts();
  tax1 = median(ax1);
  tay1 = median(ay1);
  taz1 = median(az1);
  tgx1 = mean(gx1);
  tgy1 = mean(gy1);
  tgz1 = mean(gz1);
  tax2 = median(ax2);
  tay2 = median(ay2);
  taz2 = median(az2);
  tgx2 = mean(gx2);
  tgy2 = mean(gy2);
  tgz2 = mean(gz2);
  tangle = median(angle);
  interrupts();
  
  AHRS.getQEKF(q1, q2, &tax1, &tay1, &taz1, &tgx1, &tgy1, &tgz1, &tax2, &tay2, &taz2, &tgx2, &tgy2, &tgz2, &tangle);

  float invq2[4];
  invertQuat(q2, invq2);
  
  noInterrupts();
  quatMult(q1, invq2, qrel);
  
//  Serial.print(qrel[0]);
//  Serial.print("\t");
//  Serial.print(qrel[1]);
//  Serial.print("\t");
//  Serial.print(qrel[2]);
//  Serial.print("\t");
//  Serial.println(qrel[3]);

  // output necessary for visualization with processing
  serialPrintFloatArr(qrel, 4);
  Serial.println(""); //line break
  delay(100);
  
  //serialEvent();

//  forces[0] = ex[(l-1+10)%10]+32767;
//  forces[1] = ey[(l-1+10)%10]+32767;
//  forces[2] = ez[(l-1+10)%10]+32767;
//  torques[0] = emx[(l-1+10)%10]+32767;
//  torques[1] = emy[(l-1+10)%10]+32767;
//  torques[2] = emz[(l-1+10)%10]+32767;

//  Serial.print(ex[(l-1+10)%10]);
//  Serial.print("\t");
//  Serial.print(ey[(l-1+10)%10]);
//  Serial.print("\t");
//  Serial.print(ez[(l-1+10)%10]);
//  Serial.print("\t");
//  Serial.print(emx[(l-1+10)%10]);
//  Serial.print("\t");
//  Serial.print(emy[(l-1+10)%10]);
//  Serial.print("\t");
//  Serial.println(emz[(l-1+10)%10]);

//  Serial.print(mean(ex));
//  Serial.print("\t");
//  Serial.print(mean(ey));
//  Serial.print("\t");
//  Serial.print(mean(ez));
//  Serial.print("\t");
//  Serial.print(mean(emx));
//  Serial.print("\t");
//  Serial.print(mean(emy));
//  Serial.print("\t");
//  Serial.println(mean(emz));
  
  contact = true;
  interrupts();
}

void serialEvent() {
  while (UART.available() > 0) {
    int reading;
    if (!started) {
      reading = UART.read();
      if ( reading == 10 ) {
        started = true;
      }
    }
    else {
      if (UART.available() > 0) {
        reading = UART.read();
        switch (reading) {
          case 45:
            sign = -1;
            break;
          case 32:
            data[variable] *= sign;
            //Serial.print(data[variable]);
            //Serial.print("\t");
            sign = 1;
            variable++;
            break;
          case 13:
            //Serial.println("");
            ex[l] = data[0];
            ey[l] = data[1];
            ez[l] = data[2];
            emx[l] = data[3];
            emy[l] = data[4];
            emz[l] = data[5];
            started = false;
//            Serial.print(ex[l]);
//            Serial.print("\t");
//            Serial.print(ey[l]);
//            Serial.print("\t");
//            Serial.print(ez[l]);
//            Serial.print("\t");
//            Serial.print(emx[l]);
//            Serial.print("\t");
//            Serial.print(emy[l]);
//            Serial.print("\t");
//            Serial.println(emz[l]);
            variable = 0;
            data[0] = 0; data[1] = 0; data[2] = 0; data[3] = 0; data[4] = 0; data[5] = 0;
            l++;
            break;
          default:
            data[variable] *= 10;
            data[variable] += reading - 48;
        }
      }
    }
    if (l==number) {
      l = 0;
    }
  }
}


