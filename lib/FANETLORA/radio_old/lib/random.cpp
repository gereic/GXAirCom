/*
 * random.c
 *
 *  Created on: May 21, 2017
 *      Author: sid
 */

#include <stdlib.h>
#include <stdint.h>

#include "random.h"

void randomSeed(uint32_t dwSeed)
{
	if (dwSeed != 0)
	{
		srand(dwSeed);
	}
}

int random_u(int howbig)
{
	if (howbig == 0)
	{
		return 0;
	}

	return rand() % howbig;
}

int random(int howsmall, int howbig)
{
	if (howsmall >= howbig)
		return howsmall;

	int diff = howbig - howsmall;

	return random_u(diff) + howsmall;
}
