#include <TeensyThreads.h>
#include "ICM20608G.h"
#include "AS5048A.h"
#include "AHRS.h"
#include "CommunicationUtils.h"

const int PinA = 9;
const int PinB = 10;
const int PinC = 15;

// an ICM20608G object with the ICM-20608-G sensor on Teensy Chip Select pin 10 & 9
ICM20608G IMUFoot(PinB);
ICM20608G IMUShank(PinA);
AS5048A ENCODER(PinC);
AHRS AHRS(-0.0084, 0.0056, -0.0052, -0.0180, -0.0073, 0.0015);

const int number = 10;
float ax1[number], ay1[number], az1[number], gx1[number], gy1[number], gz1[number], t1[number];
float ax2[number], ay2[number], az2[number], gx2[number], gy2[number], gz2[number], t2[number];
float angle[number];
float tax1, tay1, taz1, tgx1, tgy1, tgz1, tax2, tay2, taz2, tgx2, tgy2, tgz2, tangle;
int beginStatus1;
int beginStatus2;
int beginStatus3;
int i = 0;
int count = 10;
int y = 0;
int z = 0;
bool eval = false;
unsigned long stime;
unsigned long ctime;

float median(float daArray[]) {
    // Allocate an array of the same size and sort it.
    int iSize = sizeof(daArray);
    float* dpSorted = new float[iSize];
    for (int j = 0; j < iSize; ++j) {
        dpSorted[j] = daArray[j];
    }
    for (int j = iSize - 1; j > 0; --j) {
        for (int k = 0; k < j; ++k) {
            if (dpSorted[k] > dpSorted[k+1]) {
                float dTemp = dpSorted[k];
                dpSorted[k] = dpSorted[k+1];
                dpSorted[k+1] = dTemp;
            }
        }
    }

    // Middle or average of middle values in the sorted array.
    float dMedian = 0.0;
    if ((iSize % 2) == 0) {
        dMedian = (dpSorted[iSize/2] + dpSorted[(iSize/2) - 1])/2.0;
    } else {
        dMedian = dpSorted[iSize/2];
    }
    delete [] dpSorted;
    return dMedian;
}

float mean(float daArray[]) {
    int iSize = sizeof(daArray);
    float sum = 0.0;
    for (int j = 0; j < iSize; ++j) {
        sum += daArray[j];
    }
    return sum/iSize;
}

void sensorReadout() {
  while(1) {
    if(beginStatus1 < 0) {
      threads.delay(1000);
      Serial.println("IMUSole initialization unsuccessful");
      Serial.println("Check IMUSole wiring or try cycling power");
      threads.delay(1000);
    }
    else{
      // get the temperature data (C)
      IMUFoot.getTemp(&t1[i]);                    // workaround to read out temperature first since first readout after AMS AS5048A does not work
      
       /* getMotion6 */
      // get both the accel (m/s/s) and gyro (rad/s) data
      IMUFoot.getMotion6(&ax1[i], &ay1[i], &az1[i], &gx1[i], &gy1[i], &gz1[i]);
    }
    if(beginStatus2 < 0) {
      threads.delay(1000);
      Serial.println("IMUShank initialization unsuccessful");
      Serial.println("Check IMUShank wiring or try cycling power");
      threads.delay(1000);
    }
    else{
      /* getMotion6 */
      // get both the accel (m/s/s) and gyro (rad/s) data
      IMUShank.getMotion6(&ax2[i], &ay2[i], &az2[i], &gx2[i], &gy2[i], &gz2[i]);
    }
    if(beginStatus3 < 0) {
      threads.delay(1000);
      Serial.println("ENCODER initialization unsuccessful");
      Serial.println("Check ENCODER wiring or try cycling power");
      threads.delay(1000);
    }
    else{
      int state = ENCODER.getAngle(&angle[i]);
      //Serial.println(state);
      //printData3();
    }
    //Serial.printf("finished %d \n",i);
    //delayMicroseconds(150);
  
    i++;
    if(i == number) {
      // READOUT FORCE SENSOR
      
      eval = true;
      i = 0;
    }
    y++;
    if (y == count) {
      stime = micros() - stime;
      float rate = float(count)/stime*1000000;
//      Serial.print("sensor rate [Hz]: ");
//      Serial.println(rate);
      stime = micros();
      
      y = 0;
    }    
  }
}

void setup() {
  // put your setup code here, to run once:
  // serial to display data
  Serial.begin(250000);

  pinMode (PinA, OUTPUT);
  digitalWrite (PinA, HIGH);
  pinMode (PinB, OUTPUT);
  digitalWrite (PinB, HIGH);
  pinMode (PinC, OUTPUT);
  digitalWrite (PinC, HIGH);

  beginStatus1 = IMUFoot.begin(ACCEL_RANGE_4G,GYRO_RANGE_250DPS);
  beginStatus2 = IMUShank.begin(ACCEL_RANGE_4G,GYRO_RANGE_250DPS);
  beginStatus3 = ENCODER.begin();

  IMUFoot.setFilt(GYRO_DLPF_BANDWIDTH_250HZ, ACCEL_BYPASS_DLPF_BANDWIDTH_1046HZ, 0);
  IMUShank.setFilt(GYRO_DLPF_BANDWIDTH_250HZ, ACCEL_BYPASS_DLPF_BANDWIDTH_1046HZ, 0);
  
  uint16_t zeroPos = 12172;
  int state = ENCODER.setZeroPos(&zeroPos);
  Serial.print("Encoder set to zero; successfully (1) or not (0): ");
  Serial.println(state);
  
  threads.setSliceMicros(250);
  stime = micros();
  threads.addThread(sensorReadout);
  ctime = micros();
}

void loop() {
  if(eval) {
    eval = false;
  }
  else {
    threads.delay(1);
  }
    
  float q1[4];
  float q2[4];
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
  float tempangle = median(angle);
  if (tempangle > 180) { tempangle -= 360; }
  tangle = tempangle/180.0f*3.14159265359f;
  
  AHRS.getQEKF(q1, q2, &tax1, &tay1, &taz1, &tgx1, &tgy1, &tgz1, &tax2, &tay2, &taz2, &tgx2, &tgy2, &tgz2, &tangle);

  float invq2[4];
  invertQuat(q2, invq2);
  float qrel[4];
  quatMult(q1, invq2, qrel);

  serialPrintFloatArr(qrel, 4);
  Serial.println(""); //line break
  delay(100);
  z++;
  if (z == count/10) {
    ctime = micros() - ctime;
    float rate = float(count)/10*1000000.0f/ctime;
//    Serial.print("calc rate [Hz]: ");
//    Serial.println(rate);
    ctime = micros();
    z = 0;
  }
//    Serial.print(tax1,commas);
//    Serial.print("\t");
//    Serial.print(tay1,commas);
//    Serial.print("\t");
//    Serial.print(taz1,commas);
//    Serial.print("\t");
//    Serial.print(tgx1,commas);
//    Serial.print("\t");
//    Serial.print(tgy1,commas);
//    Serial.print("\t");
//    Serial.print(tgz1,commas);
//    Serial.print("\t");
//
//    Serial.print(tax2,commas);
//    Serial.print("\t");
//    Serial.print(tay2,commas);
//    Serial.print("\t");
//    Serial.print(taz2,commas);
//    Serial.print("\t");
//    Serial.print(tgx2,commas);
//    Serial.print("\t");
//    Serial.print(tgy2,commas);
//    Serial.print("\t");
//    Serial.print(tgz2,commas);
//    Serial.print("\t");
//
//    Serial.println(tangle,commas);
}
