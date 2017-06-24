#include "ICM20608G.h"
#include "AS5048A.h"
#include "BOTA.h"
#include "AHRS.h"
#include "Contact.h"
#include "Force.h"
#include "Functions.h"

#include "CommunicationUtils.h" // remove later

const int PinIMU1 = 10;
const int PinIMU2 = 9;
const int PinEnc = 15;
const int PinTx = 26;
const int PinRx = 27;

// an ICM20608G object with the ICM-20608-G sensor on Teensy pin (SPI chip select) provided
ICM20608G IMUFootsole(PinIMU1);
ICM20608G IMUShank(PinIMU2);

// an AS5048A object with the magnetic Encoder AS5048A sensor on Teensy pin (SPI chip select) provided
AS5048A ENCODER(PinEnc);

// a BOTA object with the BOTAsystems Force Sensor on Teensy UART pins Tx and Rx provided
BOTA BOTA(PinTx, PinRx);

// an AHRS object providing an EKF sensor fusion of two IMUs with an encoder in between (initial values of gyro offsets)
AHRS AHRS(-0.0231, 0.0092, 0.0048, 0.0112, 0.0206, -0.0082);

// a contactState object to estimate the contact state from acceleration, orientation and forces
Contact ContactState;

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
float worldForces[3];
float worldTorques[3];
float quat1[4];

unsigned long stime;

void sensorReadout() {
   if(beginStatus1 < 0) {
    delay(1000);
    Serial.println("IMUSole initialization unsuccessful");
    Serial.println("Check IMUSole wiring or try cycling power");
    delay(1000);
  }
  else{
    IMUFootsole.getTemp(&t1[i]);                    // workaround to read out temperature first since first readout after AMS AS5048A does not work
    
    // get both the accel (m/s/s) and gyro (rad/s) data
    IMUFootsole.getMotion6(&ax1[i], &ay1[i], &az1[i], &gx1[i], &gy1[i], &gz1[i]);
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
    // get the encoder angle (deg)
    if( !ENCODER.getAngle(&angle[i]) ) {
    Serial.println("Data from angular encoder couldn't be gathered.");
    }
//   // used to determine zero position
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
    int length = 61;
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

    uint16_t fx = worldForces[0]*NewtonToIntXY + offset;
    buffer[3] = (uint8_t)(fx >> 8);
    buffer[4] = (uint8_t)fx;
    uint16_t fy = worldForces[1]*NewtonToIntXY + offset;
    buffer[5] = (uint8_t)(fy >> 8);
    buffer[6] = (uint8_t)fy;
    uint16_t fz = worldForces[2]*NewtonToIntZ + offset;
    buffer[7] = (uint8_t)(fz >> 8);
    buffer[8] = (uint8_t)fz;
    uint16_t tx = worldTorques[0]*NewtonMeterToIntXY + offset;
    buffer[9] = (uint8_t)(tx >> 8);
    buffer[10] = (uint8_t)tx;
    uint16_t ty = worldTorques[1]*NewtonMeterToIntXY + offset;
    buffer[11] = (uint8_t)(ty >> 8);
    buffer[12] = (uint8_t)ty;
    uint16_t tz = worldTorques[2]*NewtonMeterToIntZ + offset;
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

    q0 = (quat1[0]+1) * quatToInt;
    buffer[23] = (uint8_t)(q0 >> 8);
    buffer[24] = (uint8_t)q0;
    q1 = (quat1[1]+1) * quatToInt;
    buffer[25] = (uint8_t)(q1 >> 8);
    buffer[26] = (uint8_t)q1;
    q2 = (quat1[2]+1) * quatToInt;
    buffer[27] = (uint8_t)(q2 >> 8);
    buffer[28] = (uint8_t)q2;
    q3 = (quat1[3]+1) * quatToInt;
    buffer[29] = (uint8_t)(q3 >> 8);
    buffer[30] = (uint8_t)q3;

    float AccToInt = 200.0;
    float GyroToInt = 7500.0;

    uint16_t accx = tax1*AccToInt + offset;
    buffer[31] = (uint8_t)(accx >> 8);
    buffer[32] = (uint8_t)accx;
    uint16_t accy = tay1*AccToInt + offset;
    buffer[33] = (uint8_t)(accy >> 8);
    buffer[34] = (uint8_t)accy;
    uint16_t accz = taz1*AccToInt + offset;
    buffer[35] = (uint8_t)(accz >> 8);
    buffer[36] = (uint8_t)accz;
    uint16_t gyrox = tgx1*GyroToInt + offset;
    buffer[37] = (uint8_t)(gyrox >> 8);
    buffer[38] = (uint8_t)gyrox;
    uint16_t gyroy = tgy1*GyroToInt + offset;
    buffer[39] = (uint8_t)(gyroy >> 8);
    buffer[40] = (uint8_t)gyroy;
    uint16_t gyroz = tgz1*GyroToInt + offset;
    buffer[41] = (uint8_t)(gyroz >> 8);
    buffer[42] = (uint8_t)gyroz;

    accx = tax2*AccToInt + offset;
    buffer[43] = (uint8_t)(accx >> 8);
    buffer[44] = (uint8_t)accx;
    accy = tay2*AccToInt + offset;
    buffer[45] = (uint8_t)(accy >> 8);
    buffer[46] = (uint8_t)accy;
    accz = taz2*AccToInt + offset;
    buffer[47] = (uint8_t)(accz >> 8);
    buffer[48] = (uint8_t)accz;
    gyrox = tgx2*GyroToInt + offset;
    buffer[49] = (uint8_t)(gyrox >> 8);
    buffer[50] = (uint8_t)gyrox;
    gyroy = tgy2*GyroToInt + offset;
    buffer[51] = (uint8_t)(gyroy >> 8);
    buffer[52] = (uint8_t)gyroy;
    gyroz = tgz2*GyroToInt + offset;
    buffer[53] = (uint8_t)(gyroz >> 8);
    buffer[54] = (uint8_t)gyroz;
    
    uint32_t now = micros();
    buffer[55] = (uint8_t)(now >> 24);
    buffer[56] = (uint8_t)(now >> 16);
    buffer[57] = (uint8_t)(now >> 8);
    buffer[58] = (uint8_t)now;
    uint16_t check = checksum(buffer, 2, length-3);;
    buffer[59] = (uint8_t)(check >> 8);
    buffer[60] = (uint8_t)check;
    
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

  beginStatus1 = IMUFootsole.begin(ACCEL_RANGE_16G,GYRO_RANGE_250DPS);
  beginStatus2 = IMUShank.begin(ACCEL_RANGE_16G,GYRO_RANGE_250DPS);
  beginStatus3 = ENCODER.begin();
  BOTA.begin();

  IMUFootsole.setFilt(GYRO_DLPF_BANDWIDTH_250HZ, ACCEL_BYPASS_DLPF_BANDWIDTH_1046HZ, 0);
  IMUShank.setFilt(GYRO_DLPF_BANDWIDTH_250HZ, ACCEL_BYPASS_DLPF_BANDWIDTH_1046HZ, 0);
  
  uint16_t zeroPos = 10152;//
  if( !ENCODER.setZeroPos(zeroPos) ) {
    Serial.println("Encoder could not be set to zero.");
  }

  BOTA.setOffset(0, 0, 0, 0, 0, 0);
  
  ContactState.setDetectContactThreshold(-20);
  ContactState.setAccAndForceThreshold(4*9.8, -15);
  ContactState.setRemoveContactThreshold(-10);

  fourkHzTimer.begin(sensorReadout, 250);
}

void loop() {
  // assure that variables cannot be written and read at the same time
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
  worldForces[0] = Fx[(l-1+10)%10];
  worldForces[1] = Fy[(l-1+10)%10];
  worldForces[2] = Fz[(l-1+10)%10];
  worldTorques[0] = Tx[(l-1+10)%10];
  worldTorques[1] = Ty[(l-1+10)%10];
  worldTorques[2] = Tz[(l-1+10)%10];
  interrupts();
  
  // update poses of shank and footsole with measured data from both IMUs and the angular encoder, returns pose of footsole and shank
  float q1[4];
  float q2[4];
  AHRS.getQEKF(q1, q2, &tax1, &tay1, &taz1, &tgx1, &tgy1, &tgz1, &tax2, &tay2, &taz2, &tgx2, &tgy2, &tgz2, &tangle);

  // compute relative quaternion between q1 and q2
  float invq2[4];
  invertQuat(q2, invq2);

  // assure that variables cannot be written and published at the same time
  noInterrupts();
  quatMult(q1, invq2, qrel);
  quat1[0] = q1[0];
  quat1[1] = q1[1];
  quat1[2] = q1[2];
  quat1[3] = q1[3];
  
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

//  Serial.print(worldForces[0]);
//  Serial.print("\t");
//  Serial.print(worldForces[1]);
//  Serial.print("\t");
//  Serial.print(worldForces[2]);
//  Serial.print("\t");
//  Serial.print(worldTorques[0]);
//  Serial.print("\t");
//  Serial.print(worldTorques[1]);
//  Serial.print("\t");
//  Serial.println(worldTorques[2]);

  // provides the contact state with the provided thresholds and rotates forces and torques in world coordinate frame
  ContactState.update(q2, ax1, ay1, az1, worldForces, worldTorques, &contact);
  
//  Serial.print(worldForces[0]);
//  Serial.print("\t");
//  Serial.print(worldForces[1]);
//  Serial.print("\t");
//  Serial.print(worldForces[2]);
//  Serial.print("\t");
//  Serial.print(worldTorques[0]);
//  Serial.print("\t");
//  Serial.print(worldTorques[1]);
//  Serial.print("\t");
//  Serial.println(worldTorques[2]);
//  Serial.println("");
//  Serial.print("contactState: ");
//  Serial.println(contact);
  
  interrupts();
}
