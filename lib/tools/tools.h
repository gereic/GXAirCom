/*!
 * @file gxtools.h
 *
 *
 */

#ifndef __GXTOOLS_H__
#define __GXTOOLS_H__

#include <inttypes.h>

bool timeOver(uint32_t tAct,uint32_t timestamp,uint32_t tTime);
uint32_t gettimeElapsed(uint32_t tAct,uint32_t timestamp);

#endif