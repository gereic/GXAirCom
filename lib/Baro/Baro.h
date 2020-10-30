

/*!
 * @file Baro.h
 *
 *
 */

#ifndef __BARO_H__
#define __BARO_H__


#include <Arduino.h>
#include <string.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <HMC5883L.h>
#include <MS5611.h>
#include <math.h>
#include <kalmanvert.h>
#include <Adafruit_BME280.h>

//#define BARO_DEBUG
#define BARO_DEBUG_IP "192.168.0.110"
#define BARO_DEBUG_PORT 5010

#define POSITION_MEASURE_STANDARD_DEVIATION 0.3
#ifdef HAVE_ACCELEROMETER 
#define ACCELERATION_MEASURE_STANDARD_DEVIATION 0.3
#else
#define ACCELERATION_MEASURE_STANDARD_DEVIATION 0.6
#endif //HAVE_ACCELEROMETER 

#define SENSORTYPE_NONE 0
#define SENSORTYPE_MS5611 1
#define SENSORTYPE_BME280 2

class Baro {
    struct udpData{
    float temp;
    float pressure;
    float pressureFiltered;
    float altitude;
    float altitudeFiltered;
    float heading;    
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    int16_t mx;
    int16_t my;
    int16_t mz;
    uint32_t loopTime;
    float climb;
    uint8_t newData;
    };    
public:
    Baro(); //constructor
    bool begin(TwoWire *pi2c);
    void run(void);
    void getValues(float *pressure,float *alt,float *climb,float *temp);
    float getHeading(void);
    float getAlt(void);
    bool isNewVAlues();

protected:
private:
    TwoWire *pI2c;
    void calcClimbing(void);
    void copyValues(void);
    bool initMS5611(void);
    bool initBME280(void);
    void runMS5611(uint32_t tAct);
    void runBME280(uint32_t tAct);
    uint8_t sensorType;
    uint8_t sensorAdr;
    bool bNewValues;
    Adafruit_BME280 bme;
    MS5611 ms5611;
    MPU6050 accelgyro;
    HMC5883L mag;
    udpData logData;
    WiFiUDP udp;
    float climbValues[50];
    uint8_t climbIndex;
    float fPressure;
    float fClimbRate;
    float fAltitude;
    float fTemp;
    kalmanvert Kalmanvert;
    uint8_t countReadings;
    SemaphoreHandle_t xMutex;
};



#endif