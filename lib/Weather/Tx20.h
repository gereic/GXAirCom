/*
 * tx20.h
 *
 *  Created on: 20.06.2018
 *      Author: gereic
 */

#ifndef SRC_TX20_H_
#define SRC_TX20_H_

#include <Arduino.h>

void tx20_init(int8_t dataPin);
uint8_t tx20getNewData(uint8_t *Dir, uint16_t *Speed); //Dir 0..15 [1/22.5°] speed [1/10m/s]

#endif /* SRC_TX20_H_ */
