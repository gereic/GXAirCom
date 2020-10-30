/*
 * random.h
 *
 *  Created on: May 21, 2017
 *      Author: sid
 */

#ifndef LIB_RANDOM_H_
#define LIB_RANDOM_H_
#include <stdint.h>

void randomSeed(uint32_t dwSeed);
int random_u(int howbig);
int random(int howsmall, int howbig);

#endif /* LIB_RANDOM_H_ */
