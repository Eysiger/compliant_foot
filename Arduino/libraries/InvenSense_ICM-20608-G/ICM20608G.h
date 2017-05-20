/*
ICM20608G.h
Roman Käslin
rkaeslin@student.ethz.ch
2017-05-14

adapted from
MPU9250.cpp
Brian R Taylor
brian.taylor@bolderflight.com
2017-01-04

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

#ifndef ICM20608G_h
#define ICM20608G_h

#include "Arduino.h"
#include "i2c_t3.h"  // I2C library

#ifndef SPI_MOSI_PIN
#define SPI_MOSI_PIN
    // Teensy 3.0 || Teensy 3.1/3.2
    #if defined(__MK20DX128__) || defined(__MK20DX256__)
    enum spi_mosi_pin
    {
      MOSI_PIN_7,
      MOSI_PIN_11
    };
    #endif
    // Teensy 3.5 || Teensy 3.6
    #if defined(__MK64FX512__) || defined(__MK66FX1M0__)
    enum spi_mosi_pin
    {
      MOSI_PIN_0,
      MOSI_PIN_7,
      MOSI_PIN_11,
      MOSI_PIN_21,
      MOSI_PIN_28,
      MOSI_PIN_44,
      MOSI_PIN_52
    };
    #endif
    // Teensy LC 
    #if defined(__MKL26Z64__)
    enum spi_mosi_pin
    {
      MOSI_PIN_0,
      MOSI_PIN_7,
      MOSI_PIN_11,
      MOSI_PIN_21
    };
    #endif
#endif

enum icm20608G_gyro_range
{
    GYRO_RANGE_250DPS,
    GYRO_RANGE_500DPS,
    GYRO_RANGE_1000DPS,
    GYRO_RANGE_2000DPS
};

enum icm20608G_accel_range
{
    ACCEL_RANGE_2G,
    ACCEL_RANGE_4G,
    ACCEL_RANGE_8G,
    ACCEL_RANGE_16G    
};

enum icm20608G_dlpf_bandwidth
{
    DLPF_BANDWIDTH_176HZ,
    DLPF_BANDWIDTH_92HZ,
    DLPF_BANDWIDTH_41HZ,
    DLPF_BANDWIDTH_20HZ,
    DLPF_BANDWIDTH_10HZ,
    DLPF_BANDWIDTH_5HZ
};

class ICM20608G{
    public:
        ICM20608G(uint8_t address, uint8_t bus);
        ICM20608G(uint8_t address, uint8_t bus, i2c_pins pins);
        ICM20608G(uint8_t address, uint8_t bus, i2c_pins pins, i2c_pullup pullups);
        ICM20608G(uint8_t csPin);
        ICM20608G(uint8_t csPin, spi_mosi_pin pin);
        int begin(icm20608G_accel_range accelRange, icm20608G_gyro_range gyroRange);
        int setFilt(icm20608G_dlpf_bandwidth bandwidth, uint8_t SRD);
        int enableInt(bool enable);
        void getAccel(float* ax, float* ay, float* az);
        void getGyro(float* gx, float* gy, float* gz);
        void getTemp(float *t);
        void getMotion6(float* ax, float* ay, float* az, float* gx, float* gy, float* gz);
        void getMotion7(float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* t);

        void getAccelCounts(int16_t* ax, int16_t* ay, int16_t* az);
        void getGyroCounts(int16_t* gx, int16_t* gy, int16_t* gz);
        void getTempCounts(int16_t* t);
        void getMotion6Counts(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
        void getMotion7Counts(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* t);

    private:
        uint8_t _address;
        uint8_t _bus;
        i2c_pins _pins;
        i2c_pullup _pullups;
        bool _userDefI2C;
        uint8_t _csPin;
        spi_mosi_pin _mosiPin;
        bool _useSPI;
        bool _useSPIHS;
        float _accelScale;
        float _gyroScale;
        const float _tempScale = 326.8f;
        const float _tempOffset = 25.0f;

        // SPI constants
        const uint8_t SPI_READ = 0x80;
        const uint32_t SPI_LS_CLOCK = 1000000; // 1 MHz, max. 1 MHz
        const uint32_t SPI_HS_CLOCK = 8000000; // 8 MHz, max. 8 MHz

        // i2c bus frequency
        const uint32_t _i2cRate = 400000;

        // constants
        const float G = 9.807f;
        const float _d2r = 3.14159265359f/180.0f;

        // ICM20608G registers
        const uint8_t ACCEL_OUT = 0x3B;
        const uint8_t GYRO_OUT = 0x43;
        const uint8_t TEMP_OUT = 0x41;

        const uint8_t ACCEL_CONFIG = 0x1C;
        const uint8_t ACCEL_FS_SEL_2G = 0x00;
        const uint8_t ACCEL_FS_SEL_4G = 0x08;
        const uint8_t ACCEL_FS_SEL_8G = 0x10;
        const uint8_t ACCEL_FS_SEL_16G = 0x18;

        const uint8_t GYRO_CONFIG = 0x1B;
        const uint8_t GYRO_FS_SEL_250DPS = 0x00;
        const uint8_t GYRO_FS_SEL_500DPS = 0x08;
        const uint8_t GYRO_FS_SEL_1000DPS = 0x10;
        const uint8_t GYRO_FS_SEL_2000DPS = 0x18;

        const uint8_t ACCEL_CONFIG2 = 0x1D;
        const uint8_t ACCEL_DLPF_176 = 0x01;
        const uint8_t ACCEL_DLPF_92 = 0x02;
        const uint8_t ACCEL_DLPF_41 = 0x03;
        const uint8_t ACCEL_DLPF_20 = 0x04;
        const uint8_t ACCEL_DLPF_10 = 0x05;
        const uint8_t ACCEL_DLPF_5 = 0x06;

        const uint8_t CONFIG = 0x1A;
        const uint8_t GYRO_DLPF_176 = 0x01;
        const uint8_t GYRO_DLPF_92 = 0x02;
        const uint8_t GYRO_DLPF_41 = 0x03;
        const uint8_t GYRO_DLPF_20 = 0x04;
        const uint8_t GYRO_DLPF_10 = 0x05;
        const uint8_t GYRO_DLPF_5 = 0x06;

        const uint8_t SMPDIV = 0x19;

        const uint8_t INT_PIN_CFG = 0x37;
        const uint8_t INT_ENABLE = 0x38;
        const uint8_t INT_DISABLE = 0x00;
        const uint8_t INT_PULSE_50US = 0x00;
        const uint8_t INT_RAW_RDY_EN = 0x01;

        const uint8_t PWR_MGMNT_1 = 0x6B;
        const uint8_t PWR_RESET = 0x80;
        const uint8_t CLOCK_SEL_PLL = 0x01;

        const uint8_t PWR_MGMNT_2 = 0x6C;
        const uint8_t SEN_ENABLE = 0x00;

        const uint8_t USER_CTRL = 0x6A;
        const uint8_t I2C_DIS = 0x10;

        const uint8_t WHO_AM_I = 0x75;

        // transformation matrix
        // transform the accel and gyro axes to match the magnetometer axes 
        const int16_t tX[3] = {1,  0,  0}; 
        const int16_t tY[3] = {0,  1,  0};
        const int16_t tZ[3] = {0,  0,  1};

        bool writeRegister(uint8_t subAddress, uint8_t data);
        void readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
        uint8_t whoAmI();
};

#endif
