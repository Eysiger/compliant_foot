#include <SPI.h>

// define pins used for Slave Select:
const int slaveAPin = 15; // Angular Encoder 15 for Teensy, 53 for DUE, 13 for YUN
const int slaveBPin = 10; // IMU Sole 10 for Teensy, 52 for DUE, 12 for YUN
const int slaveCPin = 9; // IMU Shank 9 for Teensy


// set up the speed, data order and data mode
SPISettings settingsA(10000000, MSBFIRST, SPI_MODE1);
SPISettings settingsB(8000000, MSBFIRST, SPI_MODE2); // up to 8MHz possible, clock duty cycle between 45 and 55%
SPISettings settingsC(1000000, MSBFIRST, SPI_MODE2); // up to 8MHz possible, clock duty cycle between 45 and 55%

word read_angle = 0b1111111111111111;

word set_PWR_MGMT_1 = 0b0110101100000001; // set sleep mode off, use best available clock for gyros
word set_I2C_IF_DIS = 0b0110101001010100; // set SPI mode, enable and reset FIFO
word set_ACCEL_CONFIG = 0b0001110000010000; // set accelerometer (4:3) to 2g (00), 4g (01), 8g (10), 16g (11)
word set_ACCEL_CONFIG2 = 0b0001110100001000; // set digital low pass filter off

word read_accel_x_h = 0b1011101100000000;
word read_accel_x_l = 0b1011110000000000;
byte read_accel_y_h = 0b10111101;
byte read_accel_y_l = 0b10111110;
byte read_accel_z_h = 0b10111111;
byte read_accel_z_l = 0b11000000;

byte read_gyro_x_h = 0b11000011;
byte read_gyro_x_l = 0b11000100;
byte read_gyro_y_h = 0b11000101;
byte read_gyro_y_l = 0b11000110;
byte read_gyro_z_h = 0b11000111;
byte read_gyro_z_l = 0b11001000;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);

  // set the Slave Select Pins as outputs:
  pinMode (slaveAPin, OUTPUT);
  pinMode (slaveBPin, OUTPUT);
  pinMode (slaveCPin, OUTPUT);

  digitalWrite (slaveBPin, HIGH);
  digitalWrite (slaveCPin, HIGH);
  // initialize SPI:
  SPI.begin();

  delay(1000);

//  SPI.beginTransaction(settingsC);
//  digitalWrite (slaveCPin, LOW);
//
//  SPI.transfer16(set_PWR_MGMT_1); // set sleep mode off, use best available clock for gyros
//  SPI.transfer16(set_I2C_IF_DIS); // set SPI mode, enable and reset FIFO
//  SPI.transfer16(set_ACCEL_CONFIG); // set accelerometer sensitivity
//  SPI.transfer16(set_ACCEL_CONFIG2); // set digital low pass filter off
//
//  digitalWrite (slaveCPin, HIGH);
//  SPI.endTransaction();
}
const int number = 1000;
int values[number];
unsigned long times[number];
int i = 0;

void loop() {
  // inizialize transaction with AMS sensor
  SPI.beginTransaction(settingsA);
  digitalWrite (slaveAPin, LOW);
  
  word answer16 = SPI.transfer16(read_angle);
  if (answer16 & 0b0100000000000000) { 
    Serial.println("ErrorFlag");
    SPI.transfer16(0b0100000000000001); // clear ErrorFlag
    }
  answer16 = answer16 & 0b0011111111111111;
  // Serial.println(answer16);
  values[i] = answer16;
  times[i] = micros();
  i++;

  digitalWrite (slaveAPin, HIGH);
  SPI.endTransaction();

  if (i == number) {
    double sum = 0;
    double sum2 = 0;
    for (int j=0; j<number; j++) {
      // Serial.print(values[j]);
      sum += values[j];
      sum2 += values[j]*values[j];
      // Serial.print("\t");
      //Serial.println(times[j]);
    }
    unsigned long USBtime = micros();
    for (int j=0; j<number; j++) {
      Serial.print(values[j]);
      Serial.print(values[j]);
    }
    USBtime = micros() - USBtime;
    
    double mean = sum/number;
    Serial.println(" ");
    Serial.print("mean position [°]: ");
    Serial.print(mean/16384*360);
    Serial.print(" (");
    Serial.print(mean);
    Serial.println(")");
    double var = sum2/number-mean*mean;
    Serial.print("position variance [°]: ");
    Serial.print(var/16384*360);
    Serial.print(" (");
    Serial.print(var);
    Serial.println(")");
    float rate = float(number)/(times[number-1]-times[0])*1000000;
    Serial.print("rate [Hz]: ");
    Serial.println(rate);
    float USBrate = float(number)/USBtime*1000000;
    Serial.print("USB rate [Hz]: ");
    Serial.println(USBrate);
    delay(5000);
    i = 0;
  }

//  // inizialize transaction with InvenSense IMU
//  SPI.beginTransaction(settingsC);
//  digitalWrite (slaveCPin, LOW);
//
//  word accel_x_l = SPI.transfer16(read_accel_x_l);
//  Serial.print("16: ");
//  Serial.println(int(accel_x_l));

//  SPI.transfer(read_accel_x_h);
//  byte accel_x_h = SPI.transfer(0x00);
//  SPI.transfer(read_accel_x_l);
//  byte accel_x_l = SPI.transfer(0x00);
//  byte accel_y_h = SPI.transfer(read_accel_y_h);
//  byte accel_y_l = SPI.transfer(read_accel_y_l);
//  byte accel_z_h = SPI.transfer(read_accel_z_h);
//  byte accel_z_l = SPI.transfer(read_accel_z_l);
//
//  byte gyro_x_h = SPI.transfer(read_gyro_x_h);
//  byte gyro_x_l = SPI.transfer(read_gyro_x_l);
//  byte gyro_y_h = SPI.transfer(read_gyro_y_h);
//  byte gyro_y_l = SPI.transfer(read_gyro_y_l);
//  byte gyro_z_h = SPI.transfer(read_gyro_z_h);
//  byte gyro_z_l = SPI.transfer(read_gyro_z_l);

  // transfer accelerometer data
//  SPI.transfer(read_accel_x_h);
//  byte accel_x_h = SPI.transfer(read_accel_x_l);
//  byte accel_x_l = SPI.transfer(read_accel_y_h);
//  byte accel_y_h = SPI.transfer(read_accel_y_l);
//  byte accel_y_l = SPI.transfer(read_accel_z_h);
//  byte accel_z_h = SPI.transfer(read_accel_z_l);
//  byte accel_z_l = SPI.transfer(0x00);
//
//  // transfer gyroscope data
//  SPI.transfer(read_gyro_x_h);
//  byte gyro_x_h = SPI.transfer(read_gyro_x_l);
//  byte gyro_x_l = SPI.transfer(read_gyro_y_h);
//  byte gyro_y_h = SPI.transfer(read_gyro_y_l);
//  byte gyro_y_l = SPI.transfer(read_gyro_z_h);
//  byte gyro_z_h = SPI.transfer(read_gyro_z_l);
//  byte gyro_z_l = SPI.transfer(0x00);

  // combine values to 16bit
//  int accel_x = (accel_x_h << 8) | accel_x_l;
//  int accel_y = (accel_y_h << 8) | accel_y_l;
//  int accel_z = (accel_z_h << 8) | accel_z_l;
//  int gyro_x = (gyro_x_h << 8) | gyro_x_l;
//  int gyro_y = (gyro_y_h << 8) | gyro_y_l;
//  int gyro_z = (gyro_z_h << 8) | gyro_z_l;
  
//  Serial.print("acc.: ");
//  Serial.print(accel_x);
//  Serial.print(", ");
//  Serial.print(accel_y);
//  Serial.print(", ");
//  Serial.println(accel_z);
//
//  Serial.print("gyro: ");
//  Serial.print(gyro_x);
//  Serial.print(", ");
//  Serial.print(gyro_y);
//  Serial.print(", ");
//  Serial.println(gyro_z);
  
//  digitalWrite (slaveCPin, HIGH);
//  SPI.endTransaction();
}

