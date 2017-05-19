/*
AS5048A.h
Roman Käslin
rkaeslin@student.ethz.ch
2017-05-16
*/

#ifndef AS5048A_h
#define AS5048A_h

#include "Arduino.h"

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

class AS5048A{
    public:
        AS5048A(uint8_t csPin);
        AS5048A(uint8_t csPin, spi_mosi_pin pin);
        bool begin();

        bool getAngle(float* angle);

        bool getAngleCounts(uint16_t* angle);
        
        bool setZero();
        int fieldStrength();
        
    private:
        // uint8_t _address;
        // uint8_t _bus;
        uint8_t _csPin;
        spi_mosi_pin _mosiPin;
        bool _useSPIHS;
        const float _angleScale = 360.0f/16384.0f;

        // SPI constants
        const uint16_t SPI_READ = 0x4000;
        const uint32_t SPI_LS_CLOCK = 100000; // 1 MHz
        const uint32_t SPI_HS_CLOCK = 1000000; // 10 MHz

        // constants
        const float _d2r = 3.14159265359f/180.0f;

        // AS5048A registers
        const uint16_t CLEAR_ERROR = 0x4001;
        const uint16_t READ_ANGLE = 0xFFFF;
        const uint16_t READ_MAGNITUDE = 0x7FFE;
        const uint16_t READ_DIAGNOSTICS = 0x7FFD;

        const uint16_t READ_ZERO_POS1 = 0x4016;
        const uint16_t WRITE_ZERO_POS1 = 0x8016;
        const uint16_t READ_ZERO_POS2 = 0xC017;
        const uint16_t WRITE_ZERO_POS2 = 0x0017;
        const uint16_t NOP = 0x0000;

        bool check(uint16_t data);
        void clearError();
        void writeRegister(uint16_t subAddress, uint16_t data);
        void readRegisters(uint16_t subAddress, uint8_t count, uint16_t* dest);
};

#endif
