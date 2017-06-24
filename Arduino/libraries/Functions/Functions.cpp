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

uint16_t checksum(uint8_t array[], int first, int last) {
  int count = 0; 
  for (int j=first; j<=last; j++) {
    for (int k=0; k<8; k++) {
      count += (array[j] >> k) & 0x01;
    }
  }
  return (uint16_t) count;
}
