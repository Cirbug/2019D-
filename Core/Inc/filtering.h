#ifndef __FILTERING_H
#define __FILTERING_H

#include "main.h"

int firstOrderFilter(int newValue, int oldValue, float a);
int middleValueFilter(int N);
int averageFilter(int N);
int moveAverageFilter(void);
int LAverageFilter(ADC_HandleTypeDef *hadc);
int KalmanFilter(ADC_HandleTypeDef *hadc, int inData);
int LimitingFilter(int newValue, uint8_t adcId);

#endif
