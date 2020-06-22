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

//extern WebServer server;
extern struct SettingsData setting;
extern struct statusData status;
extern FanetLora fanet;

extern trackingData testTrackingData;  
extern weatherData testWeatherData;
extern String testString;
extern uint8_t sendTestData;

void Web_setup(void);
void Web_loop(void);
