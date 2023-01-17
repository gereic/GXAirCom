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

//#define OGNTEST

#define OGNSTATUSINTERVALL 300000ul

class Ogn {
public:
  enum aircraft_t : uint8_t
  {
    UNKNOWN = 0,
    GLIDER_MOTOR_GLIDER = 1,
    TOW_PLANE = 2,
    HELICOPTER_ROTORCRAFT = 3,
    SKYDIVER = 4,
    DROP_PLANE_SKYDIVER = 5,
    HANG_GLIDER = 6,
    PARA_GLIDER = 7,
    AIRCRAFT_RECIPROCATING_ENGINE = 8,
    AIRCRAFT_JET_TURBO_ENGINE = 9,
    UFO = 10,
    BALLOON = 11,
    AIRSHIP = 12,
    UAV = 13,
    GROUND_SUPPORT = 14,
    STATIC_OBJECT = 15
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
  void sendTrackingData(time_t timestamp,float lat,float lon,float alt,float speed,float heading,float climb,String devId,aircraft_t aircraftType,uint8_t adressType,bool Onlinetracking,float snr);
  void sendGroundTrackingData(time_t timestamp,float lat,float lon,float alt,String devId,uint8_t state,uint8_t adressType,float snr);
  void sendNameData(String devId,String name,float snr);
  void sendWeatherData(weatherData *wData);
  void setClient(Client *_client);
  void setMutex(SemaphoreHandle_t *_xMutex);
  void setBattVoltage(float battVoltage);
  void setStatusData(float pressure, float temp,float hum, float battVoltage,uint8_t battPercent);

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
    String getActTimeString(time_t timestamp);
    uint8_t getSenderDetails(bool onlinetracking,aircraft_t aircraftType,uint8_t addressType);
    String getOrigin(uint8_t addressType);
    uint8_t getFANETAircraftType(aircraft_t aircraftType);
    #ifdef OGNTEST
      void sendTestMsg(uint32_t tAct);
    #endif
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
    uint8_t _BattPercent = 0;
    float _Pressure = NAN;
    float _Temp = NAN;
    float _Hum = NAN;
    uint32_t tStatus;
    uint32_t tRecBaecon;
    uint8_t initOk;
    uint8_t GPSOK;
    bool AirMode = false;
    const char *AprsIcon[16] = // Icons for various FLARM acftType's
    { "/z",  //  0 = ?
      "/'",  //  1 = (moto-)glider    (most frequent)
      "/'",  //  2 = tow plane        (often)
      "/X",  //  3 = helicopter       (often)
      "/g" , //  4 = parachute        (rare but seen - often mixed with drop plane)
      "\\^", //  5 = drop plane       (seen)
      "/g" , //  6 = hang-glider      (rare but seen)
      "/g" , //  7 = para-glider      (rare but seen)
      "\\^", //  8 = powered aircraft (often)
      "/^",  //  9 = jet aircraft     (rare but seen)
      "/z",  //  A = UFO              (people set for fun)
      "/O",  //  B = balloon          (seen once)
      "/O",  //  C = airship          (seen once)
      "/'",  //  D = UAV              (drones, can become very common)
      "/z",  //  E = ground support   (ground vehicles at airfields)
      "\\n"  //  F = static object    (ground relay ?)
    } ;    
};

#endif