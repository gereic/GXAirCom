#ifndef __WS85_H__
#define __WS85_H__

#include <Arduino.h>

void ws85_init(int rxPin);
void ws85_run();
bool ws85_getData(float *dir, float *speed, float *gust, float *temp, float *rain1h, float *batVolt, float *capVolt);

#endif