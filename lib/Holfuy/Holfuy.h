

/*!
 * @file Holfuy.h
 *
 *
 */

#ifndef __HOLFUY_H__
#define __HOLFUY_H__

//#include <HTTPClient.h>
#include <string.h>
#include <tools.h>
#include <TimeLib.h>
#include <ArduinoJson.h>
#include <ArduinoHttpClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WeatherUnderground.h>

//#define SSLCONNECTION

//windspeed [km/h]
//windgust [km/h]
//temp [°F]
//baro 

class Holfuy {
public:
    weatherUndergroundData *wData;
    Holfuy(); //constructor
    ~Holfuy(); //destructor
    bool getData(String ID,String KEY,weatherUndergroundData *data); //get Data from Holfuy with Station-ID and API-Key
    void setClient(Client *_client);
    void setMutex(SemaphoreHandle_t *_xMutex);

protected:
private:
    //HTTPClient http;
    //HttpClient http;
    bool checkKeyExists();
    String httpRequest(const char *serverName, const char *path);
    bool _windsensor;
    bool _rainsensor;
    Client *client;
    #ifdef SSLCONNECTION
    WiFiClientSecure *MyClient;
    #else
    WiFiClient *MyClient;
    #endif
    SemaphoreHandle_t *xMutex;    
};
#endif
