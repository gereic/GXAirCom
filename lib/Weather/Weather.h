

/*!
 * @file Weather.h
 *
 *
 */

#ifndef __Weather_H__
#define __Weather_H__


#include <Arduino.h>
#include <string.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TimeLib.h>
#include <ctime>
#include "TX20.h"

#define DEG2RAD M_PI / 180.0
#define RAD2DEG 180.0 / M_PI


#define WEATHER_REFRESH 2000

#define WEATHER_UPLOAD 300000uL //weather upload intervall 5min

#define Bucket_Size 0.5           // rain bucket size 0.5mm

class Weather {
public:
    typedef struct {
        bool bTemp; //temp exists
        float temp = 0.0; //temp [°C]
        bool bHumidity; //humidity exists
        float Humidity = 0.0; // [%rH]
        bool bPressure; // pressure exists
        float Pressure = 0.0; // [hPa]
        bool bWindDir; //winddir exists
        float WindDir = 0.0; //[Deg]
        bool bWindSpeed; //windspeed exists
        float WindSpeed = 0.0; //[km/h]
        float WindGust = 0.0; //[km/h]
        bool bRain; //rain-sensor exists
        float rain1h; // rain this hour [l/h]
        float rain1d; // rain this day [l/h]
    } weatherData;

    Weather(); //constructor
    void setTempOffset(float tempOffset);
    void setWindDirOffset(int16_t winddirOffset);
    bool begin(TwoWire *pi2c, float height,int8_t oneWirePin, int8_t windDirPin, int8_t windSpeedPin,int8_t rainPin,uint8_t aneoType);
    void run(void);
    bool getValues(weatherData *weather);
    void resetWindGust(void);

protected:
private:
    TwoWire *pI2c;
    bool initBME280(void);
    void runBME280(uint32_t tAct);
    float calcPressure(float p, float t, float h);    
    void copyValues(void);
    void checkAneometer(void);
    void checkRainSensor(void);
    float calcWindspeed(void);
    uint8_t sensorAdr;
    Adafruit_BME280 bme;
    //uint16_t avgFactor; //factor for avg-factor
    float _tempOffset = 0;
    int16_t _winddirOffset = 0;
    float dTemp = 0;
    float dHumidity = 0;
    float dPressure = 0;
    float _height = 0;
    uint8_t _windDirPin;
    bool bFirst;
    weatherData _weather;
    bool bNewWeather;
    SemaphoreHandle_t xMutex;
    OneWire oneWire;
    DallasTemperature sensors;
    bool hasTempSensor;
    DeviceAddress tempSensorAdr;

    int VaneValue;// raw analog value from wind vane
    int Direction;// translated 0 - 360 direction    
    uint32_t _actPulseCount = 0;
    float winddir;
    float sinWinddir;
    float cosWinddir;
    int16_t winddirAvg;
    float windspeed = 0;
    float windgust = 0;
    uint32_t rainTipCount1h = 0;
    uint32_t rainTipCount1d = 0;
    uint8_t actHour = 0;
    uint8_t actDay = 0;
    uint8_t aneometerType = 0;
};
#endif