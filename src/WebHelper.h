#include <WiFi.h>
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include "main.h"
#include "FanetLora.h"
#include "fileOps.h"
#include <string>

//extern WebServer server;
extern struct SettingsData setting;
extern FanetLora fanet;

void Web_setup(void);
void Web_loop(void);
