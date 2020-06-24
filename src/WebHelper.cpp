#include <WebHelper.h>

// Globals
AsyncWebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(1337);
char msg_buf[500];
int led_state = 0;
#define MAXCLIENTS 10
uint8_t clientPages[MAXCLIENTS];

/***********************************************************
 * Functions
 */

// Callback: receiving any WebSocket message
void onWebSocketEvent(uint8_t client_num,
                      WStype_t type,
                      uint8_t * payload,
                      size_t length) {

  StaticJsonDocument<500> doc;                      //Memory pool
  JsonObject root = doc.to<JsonObject>();
  DeserializationError error;
  uint8_t value = 0;
  // Figure out the type of WebSocket event
  switch(type) {

    // Client has disconnected
    case WStype_DISCONNECTED:
      if (client_num < MAXCLIENTS) clientPages[client_num] = 0;
      Serial.printf("[%u] Disconnected!\n", client_num);
      break;

    // New client has connected
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(client_num);
        Serial.printf("[%u] Connection from ", client_num);
        Serial.println(ip.toString());        
      }
      break;

    // Handle text messages from client
    case WStype_TEXT:

      // Print out raw message
      
      Serial.printf("[%u] Received text: %s\n", client_num, payload);      
      error = deserializeJson(doc, payload);
      if (error) {   //Check for errors in parsing
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.c_str());
        return;
    
      }
      if (root.containsKey("page")){
        value = doc["page"];                    //Get value of sensor measurement
        if (client_num < MAXCLIENTS) clientPages[client_num] = value;
        Serial.print("page=");
        Serial.println(value);
        doc.clear();
        if (clientPages[client_num] == 1){
          doc["headline"] = APPNAME "-" VERSION;
          doc["myDevId"] = setting.myDevId;
          doc["band"] = setting.band;
          doc["ssid"] = setting.ssid;
          doc["password"] = setting.password;
          doc["PilotName"] = setting.PilotName;
          doc["output"] = setting.outputMode;
          doc["oGPS"] = setting.outputGPS;
          doc["oFlarm"] = setting.outputFLARM;
          doc["oFanet"] = setting.outputFANET;
          doc["oLK8EX1"] = setting.outputLK8EX1;
          doc["testmode"] = setting.testMode;
          doc["wifioff"] = (uint8_t)setting.bSwitchWifiOff3Min;
          doc["UDPServerIP"] = setting.UDPServerIP;
          doc["UDPSendPort"] = setting.UDPSendPort;
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);
        }else if (clientPages[client_num] == 10){
          doc["band"] = setting.band;
          doc["ssid"] = setting.ssid;
          doc["password"] = setting.password;
          doc["PilotName"] = setting.PilotName;
          doc["output"] = setting.outputMode;
          doc["oGPS"] = setting.outputGPS;
          doc["oFlarm"] = setting.outputFLARM;
          doc["oFanet"] = setting.outputFANET;
          doc["oLK8EX1"] = setting.outputLK8EX1;
          doc["wifioff"] = (uint8_t)setting.bSwitchWifiOff3Min;
          doc["UDPServerIP"] = setting.UDPServerIP;
          doc["UDPSendPort"] = setting.UDPSendPort;
          doc["testmode"] = setting.testMode;
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);
        }else if (clientPages[client_num] == 101){ //msg-type 1 test
          doc["lat"] = String(testTrackingData.lat,6);
          doc["lon"] = String(testTrackingData.lon,6);
          doc["alt"] = String(testTrackingData.altitude,1);
          doc["speed"] = String(testTrackingData.speed,2);
          doc["climb"] = String(testTrackingData.climb,1);
          doc["heading"] = String(testTrackingData.heading,1);
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);
        }else if (clientPages[client_num] == 102){ //msg-type 2 test
          doc["name"] = testString;
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);
        }else if (clientPages[client_num] == 103){ //msg-type 3 test
          doc["msg"] = testString;
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);
        }else if (clientPages[client_num] == 104){ //msg-type 4 test
          doc["lat"] = String(testWeatherData.lat,6);
          doc["lon"] = String(testWeatherData.lon,6);
          doc["temp"] = String(testWeatherData.temp,1);
          doc["wHeading"] = String(testWeatherData.wHeading,2);
          doc["wSpeed"] = String(testWeatherData.wSpeed,1);
          doc["wGust"] = String(testWeatherData.wGust,1);
          doc["hum"] = String(testWeatherData.Humidity,1);
          doc["pressure"] = String(testWeatherData.Baro,1);
          doc["charge"] = testWeatherData.Charge;
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);

        }
      }else if (root.containsKey("save")){
        //save settings-page
        value = doc["save"];
        if (value == 1){
          //general settings-page
          setting.band = doc["band"].as<uint8_t>();
          setting.ssid = doc["ssid"].as<String>();
          setting.password = doc["password"].as<String>();
          setting.PilotName = doc["PilotName"].as<String>();
          setting.outputMode = doc["output"].as<uint8_t>();
          setting.outputGPS = doc["oGPS"].as<uint8_t>();
          setting.outputFLARM = doc["oFlarm"].as<uint8_t>();
          setting.outputFANET = doc["oFanet"].as<uint8_t>();
          setting.outputLK8EX1 = doc["oLK8EX1"].as<uint8_t>();
          setting.bSwitchWifiOff3Min = (bool)doc["wifioff"].as<uint8_t>();
          setting.UDPServerIP = doc["UDPServerIP"].as<String>();
          setting.UDPSendPort = doc["UDPSendPort"].as<uint16_t>();
          setting.testMode = doc["testmode"].as<uint8_t>();
          Serial.println("write config-to file --> rebooting");
          Serial.flush();
          delay(500);
          write_configFile();

        }
      }else if (root.containsKey("sendfanet")){
        //test-page
        value = doc["sendfanet"];
        if (value == 1){
          //send fanet msgtype 1
          testTrackingData.lat = doc["lat"].as<float>();
          testTrackingData.lon = doc["lon"].as<float>();
          testTrackingData.aircraftType = (eFanetAircraftType)doc["type"].as<uint8_t>();
          testTrackingData.altitude = doc["alt"].as<float>();
          testTrackingData.speed = doc["speed"].as<float>();
          testTrackingData.climb = doc["climb"].as<float>();
          testTrackingData.heading = doc["heading"].as<float>();
          sendTestData = value; //          
        }else if (value == 2){
          //name
          testString = doc["name"].as<String>();
          sendTestData = value;          
        }else if (value == 3){
          //msg
          testString = doc["msg"].as<String>();
          sendTestData = value;          
        }else if (value == 4){
          //send fanet msgtype 4
          testWeatherData.lat = doc["lat"].as<float>();
          testWeatherData.lon = doc["lon"].as<float>();
          testWeatherData.temp = doc["temp"].as<float>();
          testWeatherData.wHeading = doc["wHeading"].as<float>();
          testWeatherData.wSpeed = doc["wSpeed"].as<float>();
          testWeatherData.wGust = doc["wGust"].as<float>();
          testWeatherData.Humidity = doc["hum"].as<float>();
          testWeatherData.Baro = doc["pressure"].as<float>();
          testWeatherData.Charge = doc["charge"].as<uint8_t>();
          sendTestData = value; //          
        }

      }      
      break;

    // For everything else: do nothing
    case WStype_BIN:
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
    default:
      break;
  }
}

String processor(const String& var){
  String sRet = "";
  if(var == "SOCKETIP")
    Serial.println(status.myIP);
    return status.myIP;
  return "";
}

// Callback: send 404 if requested file does not exist
void onPageNotFound(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                  "] HTTP GET request of " + request->url());
  request->send(404, "text/plain", "Not found");
}

void Web_setup(void){
  for (int i = 0;i < MAXCLIENTS;i++) clientPages[i] = 0;
  // On HTTP request for root, provide index.html file
  server.on("/settings.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, request->url(), "text/html",false,processor);
  });
  server.on("/setgeneral.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, request->url(), "text/html",false,processor);
  });
  server.on("/settest.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, request->url(), "text/html",false,processor);
  });
  server.on("/index.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, request->url(), "text/html",false,processor);
  });
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html",false,processor);
  });
  server.on("/msgtype1.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, request->url(), "text/html",false,processor);
  });
  server.on("/msgtype2.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, request->url(), "text/html",false,processor);
  });
  server.on("/msgtype3.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, request->url(), "text/html",false,processor);
  });
  server.on("/msgtype4.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, request->url(), "text/html",false,processor);
  });
  server.on("/msgtype5.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, request->url(), "text/html",false,processor);
  });
  server.on("/msgtype7.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, request->url(), "text/html",false,processor);
  });

  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, request->url(), "text/css");
  });

  // On HTTP request for style sheet, provide style.css
  //server.on("/style.css", HTTP_GET, onCSSRequest);

  // Handle requests for pages that do not exist
  server.onNotFound(onPageNotFound);

  // Start web server
  server.begin();

  // Start WebSocket server and assign callback
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
}

void Web_loop(void){
  static uint32_t tLife = millis();
  static uint16_t counter = 0;
  uint32_t tAct = millis();
  // Look for and handle WebSocket data
  webSocket.loop();
  if ((tAct - tLife) >= 500){
    tLife = tAct;
    StaticJsonDocument<300> doc;                      //Memory pool
    doc.clear();
    doc["counter"] = counter;
    doc["vBatt"] = String(status.vBatt,2);
    doc["gpsFix"] = status.GPS_Fix;
    doc["gpsNumSat"] = status.GPS_NumSat;
    doc["gpslat"] = String(status.GPS_Lat,6);
    doc["gpslon"] = String(status.GPS_Lon,6);
    doc["gpsAlt"] = String(status.GPS_alt,1);
    doc["gpsSpeed"] = String(status.GPS_speed,2);
    doc["climbrate"] = String(status.ClimbRate,1);
    doc["fanetTx"] = status.fanetTx;
    doc["fanetRx"] = status.fanetRx;
    serializeJson(doc, msg_buf);
    for (int i = 0;i <MAXCLIENTS;i++){
      if (clientPages[i] == 1){
        //Serial.printf("Sending to [%u]: %s\n", i, msg_buf);
        webSocket.sendTXT(i, msg_buf);
      }
    }
    counter++;
  }
}