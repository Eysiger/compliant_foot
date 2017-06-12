#include "ICM20608G.h"
#include "AS5048A.h"
#include "AHRS.h"
#include "CommunicationUtils.h"

const int PinA = 9;
const int PinB = 10;
const int PinC = 15;
enum feet {
  front_right,
  front_left,
  rear_right,
  rear_left
};
const feet foot = front_right;

// an ICM20608G object with the ICM-20608-G sensor on Teensy Chip Select pin 10 & 9
ICM20608G IMUFoot(PinB);
ICM20608G IMUShank(PinA);
AS5048A ENCODER(PinC);
AHRS AHRS(-0.0084, 0.0056, -0.0052, -0.0180, -0.0073, 0.0015);
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
    if (angle[i] > 180) { angle[i] -= 360; }
    angle[i] = angle[i]/180.0f*3.14159265359f;
  }
  // output raw values of sensors
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
  Serial.println(angle[i],6);
//if(az1[i] >= 4*9.8) { Serial.println("touch down"); }

  i++;

  if (i == 1){
     // READOUT FORCE SENSOR
     
  }
    
  if(i == number) {
//    //WRITE to USB
//    int length = 28;
//    byte buffer[length];
//    buffer[0] = 0xFF;
//    buffer[1] = 0xFF;
//    
//    switch( foot ) {
//      case front_right:
//        buffer[2] = 0xC0;
//        break;
//      case front_left:
//        buffer[2] = 0x80;
//        break;
//      case rear_right:
//        buffer[2] = 0x40;
//        break;
//      case rear_left:
//        buffer[2] = 0x00;
//        break;
//    }
//    if (contact) {buffer[2] = buffer[2] | 0x03;}
//    
//    buffer[3] = (uint8_t)(forces[0] >> 8);
//    buffer[4] = (uint8_t)forces[0];
//    buffer[5] = (uint8_t)(forces[1] >> 8);
//    buffer[6] = (uint8_t)forces[1];
//    buffer[7] = (uint8_t)(forces[2] >> 8);
//    buffer[8] = (uint8_t)forces[2];
//    buffer[9] = (uint8_t)(torques[0] >> 8);
//    buffer[10] = (uint8_t)torques[0];
//    buffer[11] = (uint8_t)(torques[1] >> 8);
//    buffer[12] = (uint8_t)torques[1];
//    buffer[13] = (uint8_t)(torques[2] >> 8);
//    buffer[14] = (uint8_t)torques[2];
//    
//    uint16_t q0 = (qrel[0]+1)*32767.5;
//    buffer[15] = (uint8_t)(q0 >> 8);
//    buffer[16] = (uint8_t)q0;
//    uint16_t q1 = (qrel[1]+1)*32767.5;
//    buffer[17] = (uint8_t)(q1 >> 8);
//    buffer[18] = (uint8_t)q1;
//    uint16_t q2 = (qrel[2]+1)*32767.5;
//    buffer[19] = (uint8_t)(q2 >> 8);
//    buffer[20] = (uint8_t)q2;
//    uint16_t q3 = (qrel[3]+1)*32767.5;
//    buffer[21] = (uint8_t)(q3 >> 8);
//    buffer[22] = (uint8_t)q3;
//    
//    uint32_t now = micros();
//    buffer[23] = (uint8_t)(now >> 24);
//    buffer[24] = (uint8_t)(now >> 16);
//    buffer[25] = (uint8_t)(now >> 8);
//    buffer[26] = (uint8_t)now;
//    
//    buffer[27] = checksum(buffer, 2, length-2);
//    
//    Serial.write(buffer,length);
//    Serial.send_now();

    i = 0;
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(230400); // 230400 for read in on ROS

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
  if( !ENCODER.setZeroPos(&zeroPos) ) {
    Serial.println("Encoder could not be set to zero.");
  }
  
  fourkHzTimer.begin(sensorReadout, 250);
}

void loop() {
  // put your main code here, to run repeatedly:
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
  
  forces[0]=0x4646;
  forces[1]=0x4646;
  forces[2]=0x4646;
  torques[0]=0x5454;
  torques[1]=0x5454;
  torques[2]=0x5454;
  
  contact = true;
  interrupts();
  
//  // output necessary for visualization with processing
//  serialPrintFloatArr(qrel, 4);
//  Serial.println(""); //line break
//  delay(100);
}
