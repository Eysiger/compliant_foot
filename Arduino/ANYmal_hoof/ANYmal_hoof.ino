#include "ICM20608G.h"
#include "AS5048A.h"
#include "BOTA.h"
#include "AHRS.h"
#include "Contact.h"
#include "Force.h"
#include "Functions.h"

#include "CommunicationUtils.h" // only for visualization in processing

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
AHRS AHRS(-0.0215, -0.0021, 0.0035, 0.0145, 0.0216, -0.0082);

// a Force object that provides functions to compensate for internal forces resulting from shell or acceleration
Force Force;

// a contactState object to estimate the contact state from acceleration, orientation and forces
Contact ContactState;

// a timer object for sensor readout at 4kHz
IntervalTimer fourkHzTimer;

int beginStatusIMU1, beginStatusIMU2, beginStatusEnc;

// variables used for sensor read-in
const int number = 10;
float ax1[number], ay1[number], az1[number], gx1[number], gy1[number], gz1[number], t1[number];
float ax2[number], ay2[number], az2[number], gx2[number], gy2[number], gz2[number], t2[number];
float angle[number];
float forces[3], torques[3];
int i = 0;

// variable used for computation and output
float fax1, fay1, faz1, fgx1, fgy1, fgz1, fax2, fay2, faz2, fgx2, fgy2, fgz2, fangle, ft1, ft2;
bool zContact, normContact;
float compForces[3], compTorques[3];
float worldForces[3], worldTorques[3];
float quat1[4], quat2[4], qrel[4];

int publishSlowDown = 400;
int p = 0;

void sensorReadout() {
   if(beginStatusIMU1 < 0) {
    delay(1000);
    Serial.println("IMUSole initialization unsuccessful");
    Serial.println("Check IMUSole wiring or try cycling power");
    delay(1000);
  }
  else{
    IMUFootsole.getTemp(&t1[i]);                    // workaround to read out temperature first since first readout after AMS AS5048A does not work
    
    // get both the accel (m/s^s) and gyro (rad/s) data
    IMUFootsole.getMotion6(&ax1[i], &ay1[i], &az1[i], &gx1[i], &gy1[i], &gz1[i]);
  }
  
  if(beginStatusIMU2 < 0) {
    delay(1000);
    Serial.println("IMUShank initialization unsuccessful");
    Serial.println("Check IMUShank wiring or try cycling power");
    delay(1000);
  }
  else{
    // get both the accel (m/s^s) and gyro (rad/s) data
    IMUShank.getMotion6(&ax2[i], &ay2[i], &az2[i], &gx2[i], &gy2[i], &gz2[i]);
    IMUShank.getTemp(&t2[i]);
  }
  // used to determine gyro bias
//  Serial.print(gx1[i],6);
//  Serial.print("\t");
//  Serial.print(gy1[i],6);
//  Serial.print("\t");
//  Serial.print(gz1[i],6);
//  Serial.print("\t");
//  Serial.print(gx2[i],6);
//  Serial.print("\t");
//  Serial.print(gy2[i],6);
//  Serial.print("\t");
//  Serial.println(gz2[i],6);

  if(beginStatusEnc < 0) {
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
   // used to determine zero position
//    uint16_t counts;
//    ENCODER.getAngleCounts(&counts);
//    Serial.println(counts);

    if (angle[i] > 180) { angle[i] -= 360; }
    angle[i] = -angle[i]/180.0f*3.14159265359f;
  }

  i++;

  if (i == 1) {
    // Read-out force sensor
    if  (!BOTA.getForces(forces, torques) ) {
      //Serial.println("Data from force sensor couldn't be gathered.");  // commented since not running at 400 Hz
    }
    // used to determine force and torque offset
//    Serial.print(forces[0],6);
//    Serial.print("\t");
//    Serial.print(forces[1],6);
//    Serial.print("\t");
//    Serial.print(forces[2],6);
//    Serial.print("\t");
//    Serial.print(torques[0],6);
//    Serial.print("\t");
//    Serial.print(torques[1],6);
//    Serial.print("\t");
//    Serial.println(torques[2],6);
  }

  if(i == number) {
    //WRITE to USB
    int length = 63;
    byte buffer[length];
    createBuffer(&normContact, &zContact, compForces, compTorques,  
                 quat1, &fax1, &fay1, &faz1, &fgx1, &fgy1, &fgz1, 
                 quat2, &fax2, &fay2, &faz2, &fgx2, &fgy2, &fgz2, &ft2,
                 buffer);
    Serial.write(buffer,length);
    Serial.send_now();

    i = 0;
  }
  p++;
  if (p == publishSlowDown) {
    // output necessary for visualization with processing
//    serialPrintFloatArr(qrel, 4);
//    Serial.println(""); //line break
    p=0;
  }
}

void setup() {
  // initialize a serial connection to communicate with ROS
  Serial.begin(230400);

  // set all sensor select pins as output and on high, such that the sensorsaren't active
  pinMode (PinIMU1, OUTPUT);
  digitalWrite (PinIMU1, HIGH);
  pinMode (PinIMU2, OUTPUT);
  digitalWrite (PinIMU2, HIGH);
  pinMode (PinEnc, OUTPUT);
  digitalWrite (PinEnc, HIGH);

  // initialize the sensors and set its settings
  beginStatusIMU1 = IMUFootsole.begin(ACCEL_RANGE_16G,GYRO_RANGE_2000DPS);
  beginStatusIMU2 = IMUShank.begin(ACCEL_RANGE_16G,GYRO_RANGE_2000DPS);
  beginStatusEnc = ENCODER.begin();
  BOTA.begin();

  IMUFootsole.setFilt(GYRO_DLPF_BANDWIDTH_250HZ, ACCEL_BYPASS_DLPF_BANDWIDTH_1046HZ, 0);
  IMUShank.setFilt(GYRO_DLPF_BANDWIDTH_250HZ, ACCEL_BYPASS_DLPF_BANDWIDTH_1046HZ, 0);

  // define the zero position of the encoder
  uint16_t zeroPos = 10233;//
  if( !ENCODER.setZeroPos(zeroPos) ) {
    Serial.println("Encoder could not be set to zero.");
  }

  // define the force and torque offset
//  BOTA.setZero();
//  float x, y, z, tx, ty, tz;
//  BOTA.getOffset(&x, &y, &z, &tx, &ty, &tz);
  BOTA.setOffset(-0.9,-0.65,-2.65,-0.009,0.000,0.004);

  // define the constact detection thresholds
  ContactState.setDetectContactThreshold(-20);
  ContactState.setAccAndForceThreshold(4*9.8, -15);
  ContactState.setRemoveContactThreshold(-10);

  // start a timer that calls the sensorReadout function with 4 kHz
  fourkHzTimer.begin(sensorReadout, 250);
}

void loop() {
  // assure that variables cannot be written and read at the same time
  noInterrupts();
  // calculate mean and median of fast sensors
  fax1 = median(ax1);
  fay1 = median(ay1);
  faz1 = median(az1);
  fgx1 = mean(gx1);
  fgy1 = mean(gy1);
  fgz1 = mean(gz1);
  fax2 = median(ax2);
  fay2 = median(ay2);
  faz2 = median(az2);
  fgx2 = mean(gx2);
  fgy2 = mean(gy2);
  fgz2 = mean(gz2);
  fangle = median(angle);
  ft1 = mean(t1);
  ft2 = mean(t2);
  interrupts();
  
  // update poses of shank and footsole with measured data from both IMUs and the angular encoder, returns pose of footsole and shank
  float q1[4];
  float q2[4];
  AHRS.getComp(q1, q2, &fax1, &fay1, &faz1, &fgx1, &fgy1, &fgz1, &fax2, &fay2, &faz2, &fgx2, &fgy2, &fgz2, &fangle);
  // AHRS.getQEKF(q1, q2, &fax1, &fay1, &faz1, &fgx1, &fgy1, &fgz1, &fax2, &fay2, &faz2, &fgx2, &fgy2, &fgz2, &fangle);
  
  // assure that variables cannot be written and published at the same time
  noInterrupts();
  
  // compute relative Quaternion in between q2 and q1 expressed in coordinate frame 2
  getRelativeQuaternion(q2, q1, qrel);

  // store poses of shank and footsole in variables used for publishing
  quat1[0] = q1[0];
  quat1[1] = q1[1];
  quat1[2] = q1[2];
  quat1[3] = q1[3];

  quat2[0] = q2[0];
  quat2[1] = q2[1];
  quat2[2] = q2[2];
  quat2[3] = q2[3];
  
  // uses the measured acceleration and orientation to compensate forces resulting from accelerations
  Force.compensateAcceleration(qrel, &fax1, &fay1, &faz1, forces, torques, compForces, compTorques);

  // uses the measured orientation to compensate forces resulting from the outer shell (not implemented)
  // Force.compensateShell(qrel, compForces, compTorques, compForces, compTorques);

  // rotates forces and torques in world coordinate frame
  Force.rotateToZCoordiantes(q2, &fangle, compForces, compTorques, worldForces, worldTorques);
  
  // provides the contact state with the provided thresholds and forces along the gravity vector or the norm of the forces 
  ContactState.updateZ(q2, ax1, ay1, az1, worldForces, &zContact);
  ContactState.updateNorm(compForces, &normContact);
  
  interrupts();
  
  // assures that the loop does not run at a higher spped than 400 Hz
  delay(2);
}
