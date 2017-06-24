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
uint16_t checksum(uint8_t array[], int first, int last);

#endif
