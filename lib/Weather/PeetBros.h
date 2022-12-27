/*
 * PeetBros.h
 *
 *  Created on: 27.12.2022
 *      Author: gereic
 */

#ifndef SRC_PEETBROS_H_
#define SRC_PEETBROS_H_

#include <Arduino.h>

void peetBros_init(int8_t windSpeedPin,int8_t windDirPin);
uint8_t peetBrosgetNewData(float *Dir, float *Speed);

#endif
