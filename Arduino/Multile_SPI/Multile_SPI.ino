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

const int PinA = 9;
const int PinB = 10;
const int PinC = 15;

// an ICM20608G object with the ICM-20608-G sensor on Teensy Chip Select pin 10 & 9
ICM20608G IMUSole(10);
ICM20608G IMUShank(9);
AS5048A ENCODER(15);

const int number = 1000;
float ax1[number], ay1[number], az1[number], gx1[number], gy1[number], gz1[number], t1[number];
float ax2[number], ay2[number], az2[number], gx2[number], gy2[number], gz2[number], t2[number];
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

  //define zero position of encoder
//  int state = ENCODER.setZero();
//  Serial.print("Encoder set to zero; successfully (1) or not (0): ");
//  Serial.println(state);

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
    /* get the individual data sources */
    /* This approach is only recommended if you only would like the specified data source (i.e. only
     * want accel data) since multiple data sources would have a time skew between them. */
    // get the accelerometer data (m/s/s)
//    IMUSole.getAccel(&ax1[i], &ay1[i], &az1[i]);
//  
//    // get the gyro data (rad/s)
//    IMUSole.getGyro(&gx1[i], &gy1[i], &gz1[i]);
  
    /* get multiple data sources */
    /* In this approach we get data from multiple data sources (i.e. both gyro and accel). This is 
     *  the recommended approach since there is no time skew between sources - they are all synced. */
     
     /* getMotion6 */
    // get both the accel (m/s/s) and gyro (rad/s) data
    IMUSole.getMotion6(&ax1[i], &ay1[i], &az1[i], &gx1[i], &gy1[i], &gz1[i]); // workaround since first readout after AMS AS5048A does not work
    IMUSole.getMotion6(&ax1[i], &ay1[i], &az1[i], &gx1[i], &gy1[i], &gz1[i]);

    // get the temperature data (C)
    IMUSole.getTemp(&t1[i]);
  
    // print the data
    //printData1();

    /* getMotion7 */
//    // get the accel (m/s/s), gyro (rad/s), and temperature (C) data
//    IMUSole.getMotion7(&ax1, &ay1, &az1, &gx1, &gy1, &gz1, &t1);
  }
  if(beginStatus2 < 0) {
    delay(1000);
    Serial.println("IMUShank initialization unsuccessful");
    Serial.println("Check IMUShank wiring or try cycling power");
    delay(1000);
  }
  else{
    /* get the individual data sources */
    /* This approach is only recommended if you only would like the specified data source (i.e. only
     *  want accel data) since multiple data sources would have a time skew between them. */
    // get the accelerometer data (m/s/s)
//    IMUShank.getAccel(&ax2, &ay2, &az2);
//  
//    // get the gyro data (rad/s)
//    IMUShank.getGyro(&gx2, &gy2, &gz2);
  
    /* get multiple data sources */
    /* In this approach we get data from multiple data sources (i.e. both gyro and accel). This is 
     *  the recommended approach since there is no time skew between sources - they are all synced. */
  
     /* getMotion6 */
    // get both the accel (m/s/s) and gyro (rad/s) data
    IMUShank.getMotion6(&ax2[i], &ay2[i], &az2[i], &gx2[i], &gy2[i], &gz2[i]);
  
    // get the temperature data (C)
    IMUShank.getTemp(&t2[i]);
  
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
  i++;
  
  if (i == number){
    stime = micros() - stime;
    float rate = float(number)/stime*1000000;
    Serial.print("rate [Hz]: ");
    Serial.println(rate);

    unsigned long USBtime = micros();
    for (int j=0; j<number; j++) {
      Serial.print(ax1[j]);
      Serial.print(ay1[j]);
      Serial.print(az1[j]);
      Serial.print(gx1[j]);
      Serial.print(gy1[j]);
      Serial.print(gz1[j]);

      Serial.print(ax2[j]);
      Serial.print(ay2[j]);
      Serial.print(az2[j]);
      Serial.print(gx2[j]);
      Serial.print(gy2[j]);
      Serial.print(gz2[j]);

      Serial.print(angle[j]);
    }
    USBtime = micros() - USBtime;
    Serial.println(" ");
    float USBrate = float(number)/USBtime*1000000;
    Serial.print("USB rate [Hz]: ");
    Serial.println(USBrate);

//    double sum = 0;
//    double sum2 = 0;
//    sum += values[j];

    delay(5000);
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

