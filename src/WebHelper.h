#include <WiFi.h>
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include "main.h"
#include "FanetLora.h"
#include "fileOps.h"
#include <string>
#include <math.h>
#include <Update.h>

//extern WebServer server;
extern struct SettingsData setting;
extern struct statusData status;
extern FanetLora fanet;

extern FanetLora::trackingData testTrackingData;  
extern FanetLora::weatherData testWeatherData;
extern String testString;
extern uint32_t fanetReceiver;
extern uint8_t sendTestData;
extern const char compile_date[];

extern TaskHandle_t xHandleStandard;
extern bool WebUpdateRunning;

void Web_setup(void);
void Web_stop(void);
void Web_loop(void);
