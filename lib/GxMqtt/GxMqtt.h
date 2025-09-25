

/*!
 * @file Weather.h
 *
 *
 */

#ifndef __GXMQTT_H__
#define __GXMQTT_H__


#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Update.h>
#include <ArduinoJson.h>
#include "main.h"

#define GXMQTTMAXSTRING 255

extern bool WebUpdateRunning;
extern String compile_date;

class GxMqtt {
public:
    GxMqtt(); //constructor
    bool begin(SemaphoreHandle_t *_xMutex = NULL,Client *_client = NULL);
    void run(bool bNetworkOk);
    void end(void);
    int16_t getLastCmd(char *c,uint16_t len);
    void sendState(const char *c);
    void sendTopic(const char *topic,const char *payload, boolean retained);

protected:
private:
    void callback(char* topic, byte* payload, unsigned int length);
    void connect();
    void subscribe();
    void sendOnlineTopic(uint8_t state);
    void sendInfo();
    void sendGPS();
    void sendUpdateCmd();
    PubSubClient *pPubSubClient = NULL;
    Client *pClient;
    SemaphoreHandle_t *xMutex;
    char willTopic[50];
    char myTopic[50];
    char stateTopic[50];
    char updStateTopic[50];
    char cmdTopic[50];
    char updTopic[50];
    char wdServiceRxTopic[50];
    char infoTopic[50];
    uint8_t updCmd[5];
    char updstate[100];
    char lastCmd[GXMQTTMAXSTRING];
    uint32_t msgCnt;
    bool restartNow = false;
    uint8_t connState = 0;
    uint32_t fileLen = 0;
    uint32_t rFileLen = 0;
};
#endif