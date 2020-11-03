

/*!
 * @file WeatherUnderground.h
 *
 *
 */

#ifndef __WEATHER_UNDERGROUND_H__
#define __WEATHER_UNDERGROUND_H__

#include <HTTPClient.h>
#include <string.h>
#include <tools.h>
#include <TimeLib.h>

//windspeed [km/h]
//windgust [km/h]
//temp [°F]
//baro 

class WeatherUnderground {
public:
    WeatherUnderground(); //constructor
    bool sendData(String ID,String KEY,float winddir,float windspeed,float windgust, float humidity, float temp, float baro,float rain1h, float raindaily);
    void setSettings(bool windsensor,bool rainsensor);

protected:
private:
    HTTPClient http;
    bool _windsensor;
    bool _rainsensor;
};
#endif