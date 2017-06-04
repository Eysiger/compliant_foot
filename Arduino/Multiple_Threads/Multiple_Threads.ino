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
int beginStatus1;
int beginStatus2;
int beginStatus3;
int i = 0;
bool eval = false;
unsigned long stime;
unsigned long ctime;

float median(float daArray[]) {
    // Allocate an array of the same size and sort it.
    int iSize = sizeof(daArray);
    float* dpSorted = new float[iSize];
    for (int i = 0; i < iSize; ++i) {
        dpSorted[i] = daArray[i];
    }
    for (int i = iSize - 1; i > 0; --i) {
        for (int j = 0; j < i; ++j) {
            if (dpSorted[j] > dpSorted[j+1]) {
                float dTemp = dpSorted[j];
                dpSorted[j] = dpSorted[j+1];
                dpSorted[j+1] = dTemp;
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
      Serial.println("IENCODER initialization unsuccessful");
      Serial.println("Check ENCODER wiring or try cycling power");
      threads.delay(1000);
    }
    else{
      int state = ENCODER.getAngle(&angle[i]);
      //Serial.println(state);
      //printData3();
    }
    //Serial.printf("finished %d \n",i);
    //threads.delayMicroseconds(150);
    i++;
    if(i == number) {
      stime = micros() - stime;
      float rate = float(number)/stime*1000000;
//      Serial.print("sensor rate [Hz]: ");
//      Serial.println(rate);
      eval = true;
      i = 0;
      stime = micros();
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
  
  threads.setSliceMillis(1);
  stime = micros();
  threads.addThread(sensorReadout);
  ctime = micros();
}

void loop() {
  if(eval) {
    eval = false;
  }
  else {
    threads.delay(2);
  }
    
  float q1[4];
  float q2[4];
  float tax1 = median(ax1);
  float tay1 = median(ay1);
  float taz1 = median(az1);
  float tgx1 = median(gx1);
  float tgy1 = median(gy1);
  float tgz1 = median(gz1);
  float tax2 = median(ax2);
  float tay2 = median(ay2);
  float taz2 = median(az2);
  float tgx2 = median(gx2);
  float tgy2 = median(gy2);
  float tgz2 = median(gz2);
  float tangle = median(angle);
  if (tangle > 180) { tangle -= 360; }
  tangle = tangle/180.0f*3.14159265359f;
  
  AHRS.getQEKF(q1, q2, &tax1, &tay1, &taz1, &tgx1, &tgy1, &tgz1, &tax2, &tay2, &taz2, &tgx2, &tgy2, &tgz2, &tangle);

  float invq2[4];
  invertQuat(q2, invq2);
  float qrel[4];
  quatMult(q1, invq2, qrel);

  serialPrintFloatArr(qrel, 4);
  Serial.println(""); //line break
  delay(100);

//  ctime = micros() - ctime;
//  float rate = 1000000.0f/ctime;
//  Serial.print("calc rate [Hz]: ");
//  Serial.println(rate);
//  ctime = micros();

//  int commas = 6;
//  for (int j=0; j<number; j++) {
//      Serial.print(ax1[j],commas);
//      Serial.print("\t");
//      Serial.print(ay1[j],commas);
//      Serial.print("\t");
//      Serial.print(az1[j],commas);
//      Serial.print("\t");
//      Serial.print(gx1[j],commas);
//      Serial.print("\t");
//      Serial.print(gy1[j],commas);
//      Serial.print("\t");
//      Serial.print(gz1[j],commas);
//      Serial.print("\t");
//
//      Serial.print(ax2[j],commas);
//      Serial.print("\t");
//      Serial.print(ay2[j],commas);
//      Serial.print("\t");
//      Serial.print(az2[j],commas);
//      Serial.print("\t");
//      Serial.print(gx2[j],commas);
//      Serial.print("\t");
//      Serial.print(gy2[j],commas);
//      Serial.print("\t");
//      Serial.print(gz2[j],commas);
//      Serial.print("\t");
//
//      Serial.println(angle[j],commas);
//  }
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
