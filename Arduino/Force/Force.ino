//#include "ICM20608G.h"
//#include "AS5048A.h"
//#include "AHRS.h"
#include <Eigen.h>

#define UART Serial1

const int PinTx = 26;
const int PinRx = 27;
uint32_t stime;

const int number = 10;
int ex[number], ey[number], ez[number], emx[number], emy[number], emz[number];

//const int PinIMU1 = 10;
//ICM20608G IMUFoot(PinIMU1);
//const int PinIMU2 = 9;
//ICM20608G IMUShank(PinIMU2);
//const int PinEnc = 15;
//AS5048A ENCODER(PinEnc);

//IntervalTimer fourkHzTimer;

int variable = 0;
bool started = false;
int sign = 1;
int data[6] = {0,0,0,0,0,0};
int l = 0;

Eigen::Matrix<float, 6, 6> A;
Eigen::Matrix<float, 6, 1> ef;
Eigen::Matrix<float, 6, 1> F;

//void sensorReadout() {
//  
//}

void setup() {
  // put your setup code here, to run once:

//  int beginStatus1 = IMUFoot.begin(ACCEL_RANGE_4G,GYRO_RANGE_250DPS);
//  IMUFoot.setFilt(GYRO_DLPF_BANDWIDTH_250HZ, ACCEL_BYPASS_DLPF_BANDWIDTH_1046HZ, 0);
//  int beginStatus2 = IMUShank.begin(ACCEL_RANGE_4G,GYRO_RANGE_250DPS);
//  IMUShank.setFilt(GYRO_DLPF_BANDWIDTH_250HZ, ACCEL_BYPASS_DLPF_BANDWIDTH_1046HZ, 0);
//  int beginStatus3 = ENCODER.begin();
//
//  uint16_t zeroPos = 15567;
//  if( !ENCODER.setZeroPos(&zeroPos) ) {
//    Serial.println("Encoder could not be set to zero.");
//  }
  
  
  UART.setTX(PinTx);
  UART.setRX(PinRx);
  
  Serial.begin(250000);
  Serial.println("Hi");
  
  UART.begin(230400);
  
  stime = micros();

//  fourkHzTimer.begin(sensorReadout, 250);

  A <<  -0.001473513720803,   0.024934656756038,  -0.020341823640977,  -0.020552003033123,   0.011483767341789,   0.014051224287734,
         0.022724842326727,  -0.010607849529059,  -0.014724750724608,  -0.001461044317035,   0.018206029868300,  -0.017519364420186,
         0.020362123488925,   0.022827582060519,   0.022940548564196,  -0.001960668445840,  -0.002258007026808,   0.002192924621157,
         0.000046607185284,   0.000016986520174,   0.000000163248150,  -0.000001311378072,   0.000360472536316,  -0.000315323099915,
         0.000059678987001,  -0.000016759770216,   0.000096927174460,   0.000374515402127,  -0.000175665618951,  -0.000231607347764,
        -0.000045420096828,   0.000052311293160,  -0.000025849051812,  -0.000267459924206,  -0.000204332913294,  -0.000128001988874;
}

void loop() {
  // put your main code here, to run repeatedly:
  serialEvent();
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
            variable = 0;
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
            ef << ex[(l-1+10)%10], ey[(l-1+10)%10], ez[(l-1+10)%10], emx[(l-1+10)%10], emy[(l-1+10)%10], emz[(l-1+10)%10];
            
            F = A*ef;
            Serial.print(F[0]);
            Serial.print("\t");
            Serial.print(F[1]);
            Serial.print("\t");
            Serial.print(F[2]);
            Serial.print("\t");
            Serial.print(F[3]);
            Serial.print("\t");
            Serial.print(F[4]);
            Serial.print("\t");
            Serial.println(F[5]);
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
//      Serial.print(mean(ex));
//      Serial.print("\t");
//      Serial.print(mean(ey));
//      Serial.print("\t");
//      Serial.print(mean(ez));
//      Serial.print("\t");
//      Serial.print(mean(emx));
//      Serial.print("\t");
//      Serial.print(mean(emy));
//      Serial.print("\t");
//      Serial.println(mean(emz));
      l = 0;
    }
  }
}
