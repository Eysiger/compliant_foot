#include "ICM20608G.h"
#include "AS5048A.h"
#include "AHRS.h"
#include "BOTA.h"
#include "CommunicationUtils.h" // remove later

#define UART Serial1

const int PinIMU1 = 10;
const int PinIMU2 = 9;
const int PinEnc = 15;
const int PinTx = 26;
const int PinRx = 27;

// an ICM20608G object with the ICM-20608-G sensor on Teensy pin (SPI chip select) provided
ICM20608G IMUFoot(PinIMU1);
ICM20608G IMUShank(PinIMU2);

// an AS5048A object with the magnetic Encoder AS5048A sensor on Teensy pin (SPI chip select) provided
AS5048A ENCODER(PinEnc);

// a BOTA object with the BOTAsystems Force Sensor on Teensy UART pins Tx and Rx provided
BOTA BOTA(PinTx, PinRx);

// an AHRS object providing an EKF sensor fusion of two IMUs with an encoder in between (initial values of gyro offsets)
AHRS AHRS(-0.0231, 0.0092, 0.0048, 0.0112, 0.0206, -0.0082);

// a timer object for sensor readout at 4kHz
IntervalTimer fourkHzTimer;

const int number = 10;
float ax1[number], ay1[number], az1[number], gx1[number], gy1[number], gz1[number], t1[number];
float ax2[number], ay2[number], az2[number], gx2[number], gy2[number], gz2[number], t2[number];
float angle[number];
float Fx[number], Fy[number], Fz[number], Tx[number], Ty[number], Tz[number];
float tax1, tay1, taz1, tgx1, tgy1, tgz1, tax2, tay2, taz2, tgx2, tgy2, tgz2, tangle;
int beginStatus1, beginStatus2, beginStatus3;
int i = 0;
int l = 0;

float qrel[4];
bool contact;
float forces[4];
float torques[4];

unsigned long stime;

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

  if (i == 1) {
    // Read-out force sensor
    //stime = micros();
    if (BOTA.getForces(&Fx[l], &Fy[l], &Fz[l], &Tx[l], &Ty[l], &Tz[l]) ) {
//      Serial.print(l);
//      Serial.print("\t");
//      Serial.print(Fx[l],6);
//      Serial.print("\t");
//      Serial.print(Fy[l],6);
//      Serial.print("\t");
//      Serial.print(Fz[l],6);
//      Serial.print("\t");
//      Serial.print(Tx[l],6);
//      Serial.print("\t");
//      Serial.print(Ty[l],6);
//      Serial.print("\t");
//      Serial.println(Tz[l],6);
      l++;
      if (l == number) { l = 0; }
    }
    //stime = micros() - stime;
    //Serial.println(stime);
  }

  if(i == number) {
    //WRITE to USB
    int length = 28;
    byte buffer[length];
    buffer[0] = 0xFF;
    buffer[1] = 0xFF;
    
    if (contact) { buffer[2] = 0x03; }
    else { buffer[2] = 0x00; }

    float offset = 32767;

    float NewtonToIntXY = 50.0;
    float NewtonToIntZ = 25.0;
    float NewtonMeterToIntXY = 3000.0;
    float NewtonMeterToIntZ = 2500.0;

    uint16_t fx = Fx[(l-1+10)%10]*NewtonToIntXY + offset;
    buffer[3] = (uint8_t)(fx >> 8);
    buffer[4] = (uint8_t)fx;
    uint16_t fy = Fy[(l-1+10)%10]*NewtonToIntXY + offset;
    buffer[5] = (uint8_t)(fy >> 8);
    buffer[6] = (uint8_t)fy;
    uint16_t fz = Fz[(l-1+10)%10]*NewtonToIntZ + offset;
    buffer[7] = (uint8_t)(fz >> 8);
    buffer[8] = (uint8_t)fz;
    uint16_t tx = Tx[(l-1+10)%10]*NewtonMeterToIntXY + offset;
    buffer[9] = (uint8_t)(tx >> 8);
    buffer[10] = (uint8_t)tx;
    uint16_t ty = Ty[(l-1+10)%10]*NewtonMeterToIntXY + offset;
    buffer[11] = (uint8_t)(ty >> 8);
    buffer[12] = (uint8_t)ty;
    uint16_t tz = Tz[(l-1+10)%10]*NewtonMeterToIntZ + offset;
    buffer[13] = (uint8_t)(tz >> 8);
    buffer[14] = (uint8_t)tz;

    float quatToInt = 32767.5;
    
    uint16_t q0 = (qrel[0]+1) * quatToInt;
    buffer[15] = (uint8_t)(q0 >> 8);
    buffer[16] = (uint8_t)q0;
    uint16_t q1 = (qrel[1]+1) * quatToInt;
    buffer[17] = (uint8_t)(q1 >> 8);
    buffer[18] = (uint8_t)q1;
    uint16_t q2 = (qrel[2]+1) * quatToInt;
    buffer[19] = (uint8_t)(q2 >> 8);
    buffer[20] = (uint8_t)q2;
    uint16_t q3 = (qrel[3]+1) * quatToInt;
    buffer[21] = (uint8_t)(q3 >> 8);
    buffer[22] = (uint8_t)q3;
    
    uint32_t now = micros();
    buffer[23] = (uint8_t)(now >> 24);
    buffer[24] = (uint8_t)(now >> 16);
    buffer[25] = (uint8_t)(now >> 8);
    buffer[26] = (uint8_t)now;
    buffer[27] = checksum(buffer, 2, length-2);
    
    Serial.write(buffer,length);
    Serial.send_now();

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

  beginStatus1 = IMUFoot.begin(ACCEL_RANGE_16G,GYRO_RANGE_250DPS);
  beginStatus2 = IMUShank.begin(ACCEL_RANGE_16G,GYRO_RANGE_250DPS);
  beginStatus3 = ENCODER.begin();
  BOTA.begin();

  IMUFoot.setFilt(GYRO_DLPF_BANDWIDTH_250HZ, ACCEL_BYPASS_DLPF_BANDWIDTH_1046HZ, 0);
  IMUShank.setFilt(GYRO_DLPF_BANDWIDTH_250HZ, ACCEL_BYPASS_DLPF_BANDWIDTH_1046HZ, 0);
  
  uint16_t zeroPos = 10152;//
  if( !ENCODER.setZeroPos(zeroPos) ) {
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
  forces[0] = 0;
  forces[1] = Fx[(l-1+10)%10];
  forces[2] = Fy[(l-1+10)%10];
  forces[3] = Fz[(l-1+10)%10];
  torques[0] = 0;
  torques[1] = Tx[(l-1+10)%10];
  torques[2] = Ty[(l-1+10)%10];
  torques[3] = Tz[(l-1+10)%10];
  interrupts();
  
  AHRS.getQEKF(q1, q2, &tax1, &tay1, &taz1, &tgx1, &tgy1, &tgz1, &tax2, &tay2, &taz2, &tgx2, &tgy2, &tgz2, &tangle);

  float invq2[4];
  invertQuat(q2, invq2);

  quatMult(invq2, forces, forces);
  quatMult(forces, q2, forces);

  quatMult(invq2, torques, torques);
  quatMult(torques, q2, torques);

//  Serial.print(forces[1]);
//  Serial.print("\t");
//  Serial.print(forces[2]);
//  Serial.print("\t");
//  Serial.print(forces[3]);
//  Serial.print("\t");
//  Serial.print(torques[1]);
//  Serial.print("\t");
//  Serial.print(torques[2]);
//  Serial.print("\t");
//  Serial.println(torques[3]);
  
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
//  serialPrintFloatArr(qrel, 4);
//  Serial.println(""); //line break
//  delay(100);
  
  contact = true;
  interrupts();
}
