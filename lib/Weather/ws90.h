/*
 * ws90.h
 *
 *  Created on: 27.12.2022
 *      Author: gereic
 */

#ifndef SRC_WS90_H_
#define SRC_WS90_H_

#include <Arduino.h>
#include <RadioLib.h>

  typedef struct {      
    bool bValid = false;
    float rssi;
    int id;
    int light_raw;
    float light_lux;
    int battery_mv;
    int battery_lvl;
    int flags;
    bool bTemp;
    int temp_raw;
    float temp_c;
    bool bHum;
    int humidity;
    bool bWind;
    float wind_avg;
    int wind_dir;
    float wind_max;
    int uv_index;
    int rain_raw;
    int supercap_V;
    int firmware;
  } ws90Data;


void ws90_init(float frequency);
void ws90Run();

#endif
