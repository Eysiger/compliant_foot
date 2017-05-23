/*
Basic_SPI.ino
Roman Käslin
rkaeslin@student.ethz.ch
2017-05-14

adapted from
Brian R Taylor
brian.taylor@bolderflight.com
2016-10-10 

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

#include "ICM20608G.h"
#include "AS5048A.h"
#include "AHRS.h"
#include "CommunicationUtils.h"

const int PinA = 9;
const int PinB = 10;
const int PinC = 15;

// an ICM20608G object with the ICM-20608-G sensor on Teensy Chip Select pin 10 & 9
ICM20608G IMUSole(10);
ICM20608G IMUShank(9);
AS5048A ENCODER(15);
AHRS AHRSSole;
AHRS AHRSShank;

const int number = 1;
float ax1[number], ay1[number], az1[number], gx1[number], gy1[number], gz1[number], t1[number];
float ax2[number], ay2[number], az2[number], gx2[number], gy2[number], gz2[number], t2[number];
float q01[number], q11[number], q21[number], q31[number];
float q02[number], q12[number], q22[number], q32[number];
float angle[number];
int beginStatus1;
int beginStatus2;
int beginStatus3;
int i = 0;
unsigned long stime;

void setup() {
  // serial to display data
  Serial.begin(250000);

  pinMode (PinA, OUTPUT);
  digitalWrite (PinA, HIGH);
  pinMode (PinB, OUTPUT);
  digitalWrite (PinB, HIGH);
  pinMode (PinC, OUTPUT);
  digitalWrite (PinC, HIGH);

  // start communication with IMU and 
  // set the accelerometer and gyro ranges.
  // ACCELEROMETER 2G 4G 8G 16G
  // GYRO 250DPS 500DPS 1000DPS 2000DPS
  beginStatus1 = IMUSole.begin(ACCEL_RANGE_4G,GYRO_RANGE_250DPS);
  beginStatus2 = IMUShank.begin(ACCEL_RANGE_4G,GYRO_RANGE_250DPS);
  beginStatus3 = ENCODER.begin();

  IMUSole.setFilt(GYRO_DLPF_BANDWIDTH_250HZ, ACCEL_BYPASS_DLPF_BANDWIDTH_1046HZ, 0);
  IMUShank.setFilt(GYRO_DLPF_BANDWIDTH_250HZ, ACCEL_BYPASS_DLPF_BANDWIDTH_1046HZ, 0);
  
  //define zero position of encoder
  int state = ENCODER.setZero();
  Serial.print("Encoder set to zero; successfully (1) or not (0): ");
  Serial.println(state);

  stime = micros();
}

void loop() {
  if(beginStatus1 < 0) {
    delay(1000);
    Serial.println("IMUSole initialization unsuccessful");
    Serial.println("Check IMUSole wiring or try cycling power");
    delay(1000);
  }
  else{
    // get the temperature data (C)
    IMUSole.getTemp(&t1[i]);                    // workaround to read out temperature first since first readout after AMS AS5048A does not work
     
     /* getMotion6 */
    // get both the accel (m/s/s) and gyro (rad/s) data
    IMUSole.getMotion6(&ax1[i], &ay1[i], &az1[i], &gx1[i], &gy1[i], &gz1[i]); 

     /* getQuaternion */
    float q[4];
    AHRSSole.getQDCM(q, &ax1[i], &ay1[i], &az1[i], &gx1[i], &gy1[i], &gz1[i]);
    q01[i] = q[0];
    q11[i] = q[1];
    q21[i] = q[2];
    q31[i] = q[3];

//    Serial.print(q[0]);
//    Serial.print("\t");
//    Serial.print(q[1]);
//    Serial.print("\t");
//    Serial.print(q[2]);
//    Serial.print("\t");
//    Serial.print(q[3]);

    // for processing visualization
//    serialPrintFloatArr(q, 4);
//    Serial.println(""); //line break
//    delay(60);

//     /* getEuler */
//    float angles[3];
//    IMUSole.getEuler(angles);
//    
//    Serial.print(angles[0]);
//    Serial.print("\t");
//    Serial.print(angles[1]);
//    Serial.print("\t");
//    Serial.println(angles[2]);
      
    // print the data
    //printData1();
  }
  if(beginStatus2 < 0) {
    delay(1000);
    Serial.println("IMUShank initialization unsuccessful");
    Serial.println("Check IMUShank wiring or try cycling power");
    delay(1000);
  }
  else{
    /* getMotion6 */
    // get both the accel (m/s/s) and gyro (rad/s) data
    IMUShank.getMotion6(&ax2[i], &ay2[i], &az2[i], &gx2[i], &gy2[i], &gz2[i]);

     /* getQuaternion */
    float q[4];
    AHRSShank.getQDCM(q, &ax2[i], &ay2[i], &az2[i], &gx2[i], &gy2[i], &gz2[i]);
    q02[i] = q[0];
    q12[i] = q[1];
    q22[i] = q[2];
    q32[i] = q[3];

//    Serial.print(q[0]);
//    Serial.print("\t");
//    Serial.print(q[1]);
//    Serial.print("\t");
//    Serial.print(q[2]);
//    Serial.print("\t");
//    Serial.print(q[3]);

    // for processing visualization
//    serialPrintFloatArr(q, 4);
//    Serial.println(""); //line break
//    delay(60);
  
    // print the data
    //printData2();

    /* getMotion7 */
//    // get the accel (m/s/s), gyro (rad/s), and temperature (C) data
//    IMUShank.getMotion7(&ax2, &ay2, &az2, &gx2, &gy2, &gz2, &t2);
  }
  if(beginStatus3 < 0) {
    delay(1000);
    Serial.println("IENCODER initialization unsuccessful");
    Serial.println("Check ENCODER wiring or try cycling power");
    delay(1000);
  }
  else{
    int state = ENCODER.getAngle(&angle[i]);
    //Serial.println(state);
    //printData3();
  }
  

  float q1[4] = {q01[i], q11[i],q21[i],q31[i]};
  float q2[4] = {q02[i], q12[i],q22[i],q32[i]};
  float invq2[4];
  invertQuat(q2,invq2);
  float qrel[4];
  quatMult(invq2,q1,qrel);

  serialPrintFloatArr(qrel, 4);
  Serial.println(""); //line break
  delay(100);

  float angles[3];
  AHRSShank.getEuler(qrel, angles);

  Serial.print(angles[0]);   //yaw
  Serial.print("\t");
  Serial.print(angles[1]);   //roll
  Serial.print("\t");
  Serial.println(angles[2]); //pitch

  i++;
  if (i == number){
    stime = micros() - stime;
    
    unsigned long USBtime = micros();
    for (int j=0; j<number; j++) {
//      int commas = 2;
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
    }
    USBtime = micros() - USBtime;
    
//    float rate = float(number)/stime*1000000;
//    Serial.print("rate [Hz]: ");
//    Serial.println(rate);
//    float USBrate = float(number)/USBtime*1000000;
//    Serial.print("USB rate [Hz]: ");
//    Serial.println(USBrate);

//    double sum = 0;
//    double sum2 = 0;
//    sum += values[j];

    //delay(5000);
    stime = micros();
    i = 0;
  }
}

void printData1(){

  // print the data
  Serial.print(ax1[i],6);
  Serial.print("\t");
  Serial.print(ay1[i],6);
  Serial.print("\t");
  Serial.print(az1[i],6);
  Serial.print("\t");

  Serial.print(gx1[i],6);
  Serial.print("\t");
  Serial.print(gy1[i],6);
  Serial.print("\t");
  Serial.print(gz1[i],6);
  Serial.print("\t");

  Serial.print(t1[i],6);
  Serial.print("\t");
}

void printData2(){
  
  // print the data
  Serial.print(ax2[i],6);
  Serial.print("\t");
  Serial.print(ay2[i],6);
  Serial.print("\t");
  Serial.print(az2[i],6);
  Serial.print("\t");

  Serial.print(gx2[i],6);
  Serial.print("\t");
  Serial.print(gy2[i],6);
  Serial.print("\t");
  Serial.print(gz2[i],6);
  Serial.print("\t");

  Serial.print(t2[i],6);
  Serial.print("\t");
}

void printData3(){
  Serial.println(angle[i],2);
}

void quatMult(float q[4], float p[4], float r[4]) {
  // Input: two quaternions to be multiplied
  // Output: output of the multiplication
  
  // JPL
  r[0] = -q[1]*p[1] - q[2]*p[2] - q[3]*p[3] + q[0]*p[0];
  r[1] = q[0]*p[1] + q[3]*p[2] - q[2]*p[3] + q[1]*p[0];
  r[2] = -q[3]*p[1] + q[0]*p[2] + q[1]*p[3] + q[2]*p[0];
  r[3] = q[2]*p[1] - q[1]*p[2] + q[0]*p[3] + q[3]*p[0];
}  

void invertQuat(float q[4], float r[4]) {
  r[0] = q[0];
  r[1] = -q[1];
  r[2] = -q[2];
  r[3] = -q[3];
}

