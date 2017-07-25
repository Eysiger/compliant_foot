/*
Functions.h
Roman Käslin
rkaeslin@student.ethz.ch
2017-06-24

*/

#ifndef FUNCTIONS_h
#define FUNCTIONS_h

#include "Arduino.h"

float median(float array[]);
float mean(float array[]);

// calculating the checksum of the provided array 
// starting at the element defined with first and including all elements up to last, including last as well
uint16_t checksum(uint8_t array[], int first, int last);

// creates a buffer of bytes from the provided data
void createBuffer(bool* normContact, bool* zContact, float forces[], float torques[],
    float quat1[], float* fax1, float* fay1, float* faz1, float* fgx1, float* fgy1, float* fgz1, 
    float quat2[], float* fax2, float* fay2, float* faz2, float* fgx2, float* fgy2, float* fgz2, float* t,
    byte* buffer);

#endif
