/*
Functions.cpp
Roman Käslin
rkaeslin@student.ethz.ch
2017-06-24

*/

#include "Functions.h"

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
    return sum/((float)size);
}

// calculating the checksum of the provided array 
// starting at the element defined with first and including all elements up to last, including last as well
uint16_t checksum(uint8_t array[], int first, int last) {
  int count = 0; 
  for (int j=first; j<=last; j++) {
    for (int k=0; k<8; k++) {
      count += (array[j] >> k) & 0x01;
    }
  }
  return (uint16_t) count;
}

// creates a buffer of bytes from the provided data
void createBuffer(bool* normContact, bool* zContact, float forces[], float torques[], float quat1[], float quat2[], float* t, 
    float* fax1, float* fay1, float* faz1, float* fgx1, float* fgy1, float* fgz1, 
    float* fax2, float* fay2, float* faz2, float* fgx2, float* fgy2, float* fgz2,
    byte* buffer) {

    buffer[0] = 0xFF;
    buffer[1] = 0xFF;

    if (*normContact) { buffer[2] = 0x30; }
    else { buffer[2] = 0x00; }
    if (*zContact) { buffer[2] = buffer[2] | 0x03; }
    else { buffer[2] = buffer[2] | 0x00; }

    float offset = 32767;

    float NewtonToIntXY = 50.0;
    float NewtonToIntZ = 25.0;
    float NewtonMeterToIntXY = 3000.0;
    float NewtonMeterToIntZ = 2500.0;

    uint16_t fx = forces[0]*NewtonToIntXY + offset;
    buffer[3] = (uint8_t)(fx >> 8);
    buffer[4] = (uint8_t)fx;
    uint16_t fy = forces[1]*NewtonToIntXY + offset;
    buffer[5] = (uint8_t)(fy >> 8);
    buffer[6] = (uint8_t)fy;
    uint16_t fz = forces[2]*NewtonToIntZ + offset;
    buffer[7] = (uint8_t)(fz >> 8);
    buffer[8] = (uint8_t)fz;
    uint16_t tx = torques[0]*NewtonMeterToIntXY + offset;
    buffer[9] = (uint8_t)(tx >> 8);
    buffer[10] = (uint8_t)tx;
    uint16_t ty = torques[1]*NewtonMeterToIntXY + offset;
    buffer[11] = (uint8_t)(ty >> 8);
    buffer[12] = (uint8_t)ty;
    uint16_t tz = torques[2]*NewtonMeterToIntZ + offset;
    buffer[13] = (uint8_t)(tz >> 8);
    buffer[14] = (uint8_t)tz;

    float quatToInt = 32767.5;
    
    uint16_t q0 = (quat1[0]+1) * quatToInt;
    buffer[15] = (uint8_t)(q0 >> 8);
    buffer[16] = (uint8_t)q0;
    uint16_t q1 = (quat1[1]+1) * quatToInt;
    buffer[17] = (uint8_t)(q1 >> 8);
    buffer[18] = (uint8_t)q1;
    uint16_t q2 = (quat1[2]+1) * quatToInt;
    buffer[19] = (uint8_t)(q2 >> 8);
    buffer[20] = (uint8_t)q2;
    uint16_t q3 = (quat1[3]+1) * quatToInt;
    buffer[21] = (uint8_t)(q3 >> 8);
    buffer[22] = (uint8_t)q3;

    q0 = (quat2[0]+1) * quatToInt;
    buffer[23] = (uint8_t)(q0 >> 8);
    buffer[24] = (uint8_t)q0;
    q1 = (quat2[1]+1) * quatToInt;
    buffer[25] = (uint8_t)(q1 >> 8);
    buffer[26] = (uint8_t)q1;
    q2 = (quat2[2]+1) * quatToInt;
    buffer[27] = (uint8_t)(q2 >> 8);
    buffer[28] = (uint8_t)q2;
    q3 = (quat2[3]+1) * quatToInt;
    buffer[29] = (uint8_t)(q3 >> 8);
    buffer[30] = (uint8_t)q3;

    float tempToInt = 325.0;
    
    uint16_t temp = *t*tempToInt + offset;
    buffer[31] = (uint8_t)(temp >> 8);
    buffer[32] = (uint8_t)temp;

    float AccToInt = 200.0;
    float GyroToInt = 7500.0;

    uint16_t accx = *fax1*AccToInt + offset;
    buffer[33] = (uint8_t)(accx >> 8);
    buffer[34] = (uint8_t)accx;
    uint16_t accy = *fay1*AccToInt + offset;
    buffer[35] = (uint8_t)(accy >> 8);
    buffer[36] = (uint8_t)accy;
    uint16_t accz = *faz1*AccToInt + offset;
    buffer[37] = (uint8_t)(accz >> 8);
    buffer[38] = (uint8_t)accz;
    uint16_t gyrox = *fgx1*GyroToInt + offset;
    buffer[39] = (uint8_t)(gyrox >> 8);
    buffer[40] = (uint8_t)gyrox;
    uint16_t gyroy = *fgy1*GyroToInt + offset;
    buffer[41] = (uint8_t)(gyroy >> 8);
    buffer[42] = (uint8_t)gyroy;
    uint16_t gyroz = *fgz1*GyroToInt + offset;
    buffer[43] = (uint8_t)(gyroz >> 8);
    buffer[44] = (uint8_t)gyroz;

    accx = *fax2*AccToInt + offset;
    buffer[45] = (uint8_t)(accx >> 8);
    buffer[46] = (uint8_t)accx;
    accy = *fay2*AccToInt + offset;
    buffer[47] = (uint8_t)(accy >> 8);
    buffer[48] = (uint8_t)accy;
    accz = *faz2*AccToInt + offset;
    buffer[49] = (uint8_t)(accz >> 8);
    buffer[50] = (uint8_t)accz;
    gyrox = *fgx2*GyroToInt + offset;
    buffer[51] = (uint8_t)(gyrox >> 8);
    buffer[52] = (uint8_t)gyrox;
    gyroy = *fgy2*GyroToInt + offset;
    buffer[53] = (uint8_t)(gyroy >> 8);
    buffer[54] = (uint8_t)gyroy;
    gyroz = *fgz2*GyroToInt + offset;
    buffer[55] = (uint8_t)(gyroz >> 8);
    buffer[56] = (uint8_t)gyroz;
    
    uint32_t now = micros();
    buffer[57] = (uint8_t)(now >> 24);
    buffer[58] = (uint8_t)(now >> 16);
    buffer[59] = (uint8_t)(now >> 8);
    buffer[60] = (uint8_t)now;
    uint16_t check = checksum(buffer, 2, 60);
    buffer[61] = (uint8_t)(check >> 8);
    buffer[62] = (uint8_t)check;
}
