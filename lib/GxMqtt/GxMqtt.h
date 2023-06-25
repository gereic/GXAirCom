

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
#include "main.h"

#define GXMQTTMAXSTRING 255

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
    void sendOnlineTopic(uint8_t state);
    PubSubClient *pPubSubClient = NULL;
    Client *pClient;
    SemaphoreHandle_t *xMutex;
    char myTopic[100];
    char stateTopic[100];
    char lastCmd[GXMQTTMAXSTRING];
};
#endif