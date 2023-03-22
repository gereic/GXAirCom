

/*!
 * @file Windy.h
 *
 *
 */

#ifndef __WINDY_H__
#define __WINDY_H__

//#include <HTTPClient.h>
#include <string.h>
#include <tools.h>
#include <TimeLib.h>
#include <ArduinoJson.h>
#include <ArduinoHttpClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

//#define SSLCONNECTION

//windspeed [km/h]
//windgust [km/h]
//temp [°F]
//baro 

class Windy {
public:
  typedef struct {
    bool bWind;
    float lat;
    float lon;
    float height;
    float winddir;
    float windspeed;
    float windgust;
    bool bHum = false;
    float humidity;
    bool bTemp = false;
    float temp;
    bool bPress = false;
    float pressure;
    bool bRain = false;
    float rain1h;
    float raindaily;
  } wData;

    Windy(); //constructor
    ~Windy(); //destructor
    bool sendData(String ID,String APIKEY,wData *data); //send Data to Windy with Station-ID and Station-Key
    void setClient(Client *_client);
    void setMutex(SemaphoreHandle_t *_xMutex);

protected:
private:
    //HTTPClient http;
    //HttpClient http;
    bool checkKeyExists();
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