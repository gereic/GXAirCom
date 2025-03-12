

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
#include <HMC5883L.h>
#include <MS5611.h>
#include <math.h>
#include <kalmanvert.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP3XX.h>
#include <Preferences.h>
#include "helper_3dmath.h"
#include "InterpolationLib.h"

//#define newBaro

//#define BARO_DEBUG
#define BARO_DEBUG_IP "192.168.0.178"
#define BARO_DEBUG_PORT 5010

#define POSITION_MEASURE_STANDARD_DEVIATION 0.1
#ifdef HAVE_ACCELEROMETER 
#define ACCELERATION_MEASURE_STANDARD_DEVIATION 0.3
#else
#define ACCELERATION_MEASURE_STANDARD_DEVIATION 0.6
#endif //HAVE_ACCELEROMETER 

#define SENSORTYPE_NONE 0
#define SENSORTYPE_MS5611 1
#define SENSORTYPE_BME280 2
#define SENSORTYPE_BMP3XX 3

class Baro {
    struct udpData{
    float temp;
    float pressure;
    float pressureFiltered;
    float altitude;
    float altitudeFiltered;
    float heading;    
    uint32_t loopTime;
    float baroPos;
    float pos;
    float velo;
    float acc;
    int16_t mx;
    int16_t my;
    int16_t mz;
    uint8_t newData;
    uint8_t baroCount;
    float pressmeasure;    
    float vAcc;
    float vOffset;
    uint8_t mpuCount;
    int16_t accel[3];
    int16_t gyro[3];
    int16_t aaWorld[3];
    int16_t aaReal[3];
    float gravity[3];
    };    
public:
    Baro(); //constructor
    uint8_t begin(TwoWire *pi2c,SemaphoreHandle_t *_xMutex);
    void setKalmanSettings(float sigmaP,float sigmaA);
    void useMPU(bool bUseMPU);
    void run(void);
    void end(void);
    void getValues(float *pressure,float *alt,float *climb,float *temp);
    void getMPUValues(int16_t accel[3],int16_t gyro[3],float *acc_Z);
    float getHeading(void);
    float getAlt(void);
    bool isNewVAlues();
    bool calibGyro(void);
    bool calibAcc(void);
    bool calibration(void);
    bool calibrate(bool bInit,uint8_t* calibstate);

protected:
private:
    TwoWire *pI2c;
    void calcClimbing(void);
    void copyValues(void);
    bool initMS5611(void);
    bool initBME280(void);
    bool initBMP3XX(void);
    void runMS5611(uint32_t tAct);
    void runBME280(uint32_t tAct);
    void runBMP3XX(uint32_t tAct);
    bool mpuDrdy(void);
    float getGravityCompensatedAccel(float temp);
    void scaleAccel(VectorInt16 *accel,float temp);
    void meansensors(void);
    float getMpuTemp(void);
    uint32_t get2of3(uint32_t *press, uint32_t newPress);
    uint32_t arPress[3]; 
    uint8_t sensorType;
    uint8_t sensorAdr;
    bool bNewValues;
    Adafruit_BME280 bme;
    Adafruit_BMP3XX bmp3xx;
    MS5611 ms5611;
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
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
    float ax_scale,ay_scale,az_scale;
    bool bUseAcc = false;
    int pinDRDYInt = 2;
    uint8_t fifoBuffer[64]; // FIFO storage buffer
    int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz = 0;    
    Interpolation interpolate;
    double tValues[2] = { 29.4, 15.5 };
    double zValues[2] = {   0,  -90 };
    double _sigmaP;
    double _sigmaA;
    int axMin,axMax,ayMin,ayMax,azMin,azMax;
    SemaphoreHandle_t *xMutexI2C;
};



#endif