

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

#define WEATHER_REFRESH 2000

#define WEATHER_UPLOAD 300000uL //weather upload intervall 5min

class Weather {
public:
    typedef struct {
        bool bTemp;
        float temp; //temp [°C]
        bool bHumidity;
        float Humidity;
        bool bPressure;
        float Pressure;
    } weatherData;

    Weather(); //constructor
    void setTempOffset(float tempOffset);
    bool begin(TwoWire *pi2c, float height,uint8_t oneWirePin);
    void run(void);
    void getValues(weatherData *weather);

protected:
private:
    TwoWire *pI2c;
    bool initBME280(void);
    void runBME280(uint32_t tAct);
    float calcPressure(float p, float t, float h);
    float calcExpAvgf(float oldValue, float newValue, float Factor);
    void copyValues(void);
    uint8_t sensorAdr;
    Adafruit_BME280 bme;
    uint16_t avgFactor; //factor for avg-factor
    float _tempOffset = 0;
    float dTemp = 0;
    float dTempOld = 0;
    float dHumidity = 0;
    float dHumidityOld = 0;
    float dPressure = 0;
    float dPressureOld = 0;
    float _height = 0;
    bool bFirst;
    weatherData _weather;
    SemaphoreHandle_t xMutex;
    OneWire oneWire;
    DallasTemperature sensors;
    bool hasTempSensor;
    DeviceAddress tempSensorAdr;
};
#endif