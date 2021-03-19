/*!
 * @file Ogn.h
 *
 *
 */

#ifndef __OGN_H__
#define __OGN_H__

#include <Arduino.h>
#include <string.h>
#include <WiFi.h>
#include "time.h"
#include "tools.h"
#include <TimeLib.h>

#define OGNSTATUSINTERVALL 300000ul

class Ogn {
public:
  enum aircraft_t : uint8_t
  {
    otherAircraft = 0,
    paraglider = 1,
    hangglider = 2,
    balloon = 3,
    glider = 4,
    poweredAircraft = 5,
    helicopter = 6,
    uav = 7,
    unknown = 15,
  };
  typedef struct {
    String devId; //devId
    float lat; //latitude
    float lon; //longitude
    bool bTemp;
    float temp; //temp [°C]
    float wHeading; //wind heading [°]
    bool bWind;
    float wSpeed; //km/h
    float wGust; //km/h
    bool bHumidity;
    float Humidity;
    bool bBaro;
    float Baro;
    bool bRain;
    float rain1h;
    float rain24h;
    float snr; //signal to noise ratio
  } weatherData;

  Ogn(); //constructor
  bool begin(String user,String version);
  void end(void);
  void run(bool bNetworkOk); //has to be called cyclic
  void setAirMode(bool _AirMode); //sets the mode (for sending heading and speed only if Air-Module)
  void setGPS(float lat,float lon,float alt,float speed,float heading);
  void sendTrackingData(float lat,float lon,float alt,float speed,float heading,float climb,String devId,aircraft_t aircraftType,uint8_t adressType,bool Onlinetracking,float snr);
  void sendGroundTrackingData(float lat,float lon,String devId,uint8_t state,uint8_t adressType,float snr);
  void sendNameData(String devId,String name,float snr);
  void sendWeatherData(weatherData *wData);
  void setClient(Client *_client);
  void setMutex(SemaphoreHandle_t *_xMutex);
  void setBattVoltage(float battVoltage);
  void setStatusData(float pressure, float temp,float hum, float battVoltage);

private:
    void checkClientConnected(uint32_t tAct);
    void connect2Server(uint32_t tAct);
    void sendLoginMsg(void);
    String calcPass(String user);
    void readClient();
    void checkLine(String line);
    void sendStatus(uint32_t tAct);
    void sendReceiverStatus(String sTime);    
    void sendReceiverBeacon(String sTime);
    String getActTimeString();
    uint8_t getSenderDetails(bool onlinetracking,aircraft_t aircraftType,uint8_t addressType);
    uint8_t getAddressType(String devId);
    String getOrigin(uint8_t addressType);
    bool connected = false;
    Client *client;
    SemaphoreHandle_t *xMutex;    
    String _user;
    String _version;
    String _servername;
    float _lat = NAN;
    float _lon = NAN;
    float _alt = NAN;
    float _speed = NAN;
    float _heading = NAN;
    float _BattVoltage = NAN;
    float _Pressure = NAN;
    float _Temp = NAN;
    float _Hum = NAN;
    uint32_t tStatus;
    uint32_t tRecBaecon;
    uint8_t initOk;
    uint8_t GPSOK;
    bool AirMode = false;
};

#endif