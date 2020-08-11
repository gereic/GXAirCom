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
  };
  Ogn(); //constructor
  bool begin(String user,String version);
  void end(void);
  void run(void); //has to be called cyclic
  void setGPS(float lat,float lon,float alt,float speed,float heading);
  void sendTrackingData(float lat,float lon,float alt,float speed,float heading,float climb,String devId,aircraft_t aircraftType);

private:
    void connect2Server(void);
    void sendLoginMsg(void);
    String calcPass(String user);
    void readClient();
    void checkLine(String line);
    void sendStatus(uint32_t tAct);
    void sendReceiverStatus(String sTime);    
    void sendReceiverBeacon(String sTime);
    String getActTimeString();
    uint8_t getSenderDetails(aircraft_t aircraftType,String devId);
    bool connected = false;
    WiFiClient client;
    String _user;
    String _version;
    String _servername;
    float _lat = NAN;
    float _lon = NAN;
    float _alt = NAN;
    float _speed = NAN;
    float _heading = NAN;
    uint32_t tStatus;
    uint32_t tRecBaecon;
    uint8_t initOk;
    uint8_t GPSOK;
};

#endif