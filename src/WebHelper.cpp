#include <WebHelper.h>

// Globals
AsyncWebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(1337);
int led_state = 0;
char msg_buf[500];
#define MAXCLIENTS 10
uint8_t clientPages[MAXCLIENTS];

String DevelopMenue = "<table style=\"width:100&#37;\"><tr><td style=\"width:100&#37;\"><button onClick=\"location.href='/developmenue.html'\">developer menue</button></td></tr></table><p></p><p></p>";

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
      log_i("[%u] Disconnected!", client_num);
      break;

    // New client has connected
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(client_num);
        log_i("[%u] Connection from ", client_num);
        //log_i("%s",ip.toString());        
      }
      break;

    // Handle text messages from client
    case WStype_TEXT:

      // Print out raw message
      
      log_i("[%u] Received text: %s", client_num, payload);      
      error = deserializeJson(doc, payload);
      if (error) {   //Check for errors in parsing
        log_i("deserializeJson() failed: %s",error.c_str());
        return;
    
      }
      if (root.containsKey("page")){
        value = doc["page"];                    //Get value of sensor measurement
        if (client_num < MAXCLIENTS) clientPages[client_num] = value;
        log_i("page=%d",value);
        doc.clear();
        if (clientPages[client_num] == 1){
          doc["board"] = setting.boardType;
          doc["myDevId"] = setting.myDevId;
          doc["band"] = setting.band;
          doc["ssid"] = setting.ssid;
          doc["password"] = setting.password;
          doc["type"] = (uint8_t)setting.AircraftType;
          doc["PilotName"] = setting.PilotName;
          doc["output"] = setting.outputMode;
          doc["oGPS"] = setting.outputGPS;
          doc["oFlarm"] = setting.outputFLARM;
          doc["oFanet"] = setting.outputFANET;
          doc["oLK8EX1"] = setting.outputLK8EX1;
          doc["awlive"] = setting.awLiveTracking;
          doc["wifioff"] = (uint8_t)setting.bSwitchWifiOff3Min;
          doc["UDPServerIP"] = setting.UDPServerIP;
          doc["UDPSendPort"] = setting.UDPSendPort;
          doc["compiledate"] = String(compile_date);
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);
        }else if (clientPages[client_num] == 2){ //sendmessage

        }else if (clientPages[client_num] == 11){ //settings general
          doc["board"] = setting.boardType;
          doc["band"] = setting.band;
          doc["power"] = setting.LoraPower;
          doc["type"] = (uint8_t)setting.AircraftType;
          doc["PilotName"] = setting.PilotName;
          doc["testmode"] = setting.testMode;
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);
        }else if (clientPages[client_num] == 12){ //settings output
          doc["output"] = setting.outputMode;
          doc["oGPS"] = setting.outputGPS;
          doc["oFlarm"] = setting.outputFLARM;
          doc["oFanet"] = setting.outputFANET;
          doc["oLK8EX1"] = setting.outputLK8EX1;
          doc["awlive"] = setting.awLiveTracking;
          doc["UDPServerIP"] = setting.UDPServerIP;
          doc["UDPSendPort"] = setting.UDPSendPort;
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);
        }else if (clientPages[client_num] == 13){ //settings wifi
          doc["appw"] = setting.appw;
          doc["ssid"] = setting.ssid;
          doc["password"] = setting.password;
          doc["wifioff"] = (uint8_t)setting.bSwitchWifiOff3Min;
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);
        }else if (clientPages[client_num] == 14){ //settings ground station
          doc["gsawid"] = setting.GSAWID;
          doc["gslat"] = setting.GSLAT;
          doc["gslon"] = setting.GSLON;
          doc["gsalt"] = setting.GSAlt;
          doc["gsmode"] = setting.GSMode;
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
          if (root.containsKey("appw")) setting.appw = doc["appw"].as<String>();          
          if (root.containsKey("board")) setting.boardType = doc["board"].as<uint8_t>();          
          if (root.containsKey("power")) setting.LoraPower = constrain(doc["power"].as<uint8_t>(),0,20);          
          if (root.containsKey("band")) setting.band = doc["band"].as<uint8_t>();          
          if (root.containsKey("ssid")) setting.ssid = doc["ssid"].as<String>();
          if (root.containsKey("password")) setting.password = doc["password"].as<String>();
          if (root.containsKey("type")) setting.AircraftType = (FanetLora::aircraft_t)doc["type"].as<uint8_t>();
          if (root.containsKey("PilotName")) setting.PilotName = doc["PilotName"].as<String>();
          if (root.containsKey("output")) setting.outputMode = doc["output"].as<uint8_t>();
          if (root.containsKey("oGPS")) setting.outputGPS = doc["oGPS"].as<uint8_t>();
          if (root.containsKey("oFlarm")) setting.outputFLARM = doc["oFlarm"].as<uint8_t>();
          if (root.containsKey("oFanet")) setting.outputFANET = doc["oFanet"].as<uint8_t>();
          if (root.containsKey("oLK8EX1")) setting.outputLK8EX1 = doc["oLK8EX1"].as<uint8_t>();
          if (root.containsKey("awlive")) setting.awLiveTracking = doc["awlive"].as<uint8_t>();
          if (root.containsKey("wifioff")) setting.bSwitchWifiOff3Min = (bool)doc["wifioff"].as<uint8_t>();
          if (root.containsKey("UDPServerIP")) setting.UDPServerIP = doc["UDPServerIP"].as<String>();
          if (root.containsKey("UDPSendPort")) setting.UDPSendPort = doc["UDPSendPort"].as<uint16_t>();
          if (root.containsKey("testmode")) setting.testMode = doc["testmode"].as<uint8_t>();
          if (root.containsKey("gsawid")) setting.GSAWID = doc["gsawid"].as<String>();
          if (root.containsKey("gslat")) setting.GSLAT = doc["gslat"].as<float>();
          if (root.containsKey("gslon")) setting.GSLON = doc["gslon"].as<float>();
          if (root.containsKey("gsalt")) setting.GSAlt = doc["gsalt"].as<float>();
          if (root.containsKey("gsmode")) setting.GSMode = doc["gsmode"].as<uint8_t>();
          log_i("write config-to file --> rebooting");
          delay(500);
          write_configFile();

        }
      }else if (root.containsKey("rLoopTime")){
        status.tMaxLoop = 0; //reset looptime
      }else if (root.containsKey("sendfanet")){
        //test-page
        value = doc["sendfanet"];
        if (value == 1){
          //send fanet msgtype 1
          testTrackingData.lat = doc["lat"].as<float>();
          testTrackingData.lon = doc["lon"].as<float>();
          testTrackingData.aircraftType = (FanetLora::aircraft_t)doc["type"].as<uint8_t>();
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
          fanetReceiver = strtol( doc["receiver"].as<String>().c_str(), NULL, 16);
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
  if (xHandleBle){
    //delect ble-task to free some memory
    vTaskDelete(xHandleBle); //delete standard-task
    delay(100);
    xHandleBle = NULL;
  }
  
  String sRet = "";
  //log_i("%s",var.c_str());
  if(var == "SOCKETIP"){
    return status.myIP;
  }else if (var == "APPNAME"){
    return APPNAME;
  }else if (var == "PILOT"){
    return setting.PilotName;
  }else if (var == "VERSION"){
    return VERSION;
  }else if (var == "BUILD"){
    return String(compile_date);
  }else if (var == "DEVELOPER"){
    if (setting.testMode){
      return DevelopMenue;
    }else{
      return "";
    }
  }else if (var == "NEIGHBOURS"){
    sRet = "";
    for (int i = 0; i < MAXNEIGHBOURS; i++){
      if (fanet.neighbours[i].devId){
        sRet += "<option value=\"" + fanet.getDevId(fanet.neighbours[i].devId) + "\">" + fanet.neighbours[i].name + " " + fanet.getDevId(fanet.neighbours[i].devId) + "</option>\r\n";
      }
    }
    return sRet;
  }
    
  return "";
}

// Callback: send 404 if requested file does not exist
void onPageNotFound(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  log_e("[%s] HTTP GET request of %s",remote_ip.toString().c_str(),request->url().c_str());
  request->send(404, "text/plain", "Not found");
}

static int restartNow = false;

static void handle_update_progress_cb(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  uint32_t free_space = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
  if (!index){
    //Serial.print("Total bytes:    "); Serial.println(SPIFFS.totalBytes());
    //Serial.print("Used bytes:     "); Serial.println(SPIFFS.usedBytes());
    //Serial.println(filename);
    //Serial.println("Update");
    //log_i("stopping standard-task");
    //vTaskDelete(xHandleStandard); //delete standard-task
    WebUpdateRunning = true;
    delay(500); //wait 1 second until tasks are stopped
    //Update.runAsync(true);
    if (filename == "spiffs.bin"){
      if (!Update.begin(0x30000,U_SPIFFS)) {
        Update.printError(Serial);
      }
    }else{
      if (!Update.begin(free_space,U_FLASH)) {
        Update.printError(Serial);
      }
    }
  }

  if (Update.write(data, len) != len) {
    Update.printError(Serial);
  }

  if (final) {
    if (!Update.end(true)){
      Update.printError(Serial);
    } else {
      restartNow = true;//Set flag so main loop can issue restart call
      Serial.println("Update complete");      
    }
  }
}

void Web_setup(void){
  for (int i = 0;i < MAXCLIENTS;i++) clientPages[i] = 0;
  // On HTTP request for root, provide index.html file
  server.on("/fwupdate", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, request->url() + ".html", "text/html",false,processor);
  });
  // handler for the /update form POST (once file upload finishes)
  server.on("/fwupdate", HTTP_POST, [](AsyncWebServerRequest *request){
      request->send(200);
    }, handle_update_progress_cb);
  server.on("/settings.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, request->url(), "text/html",false,processor);
  });
  server.on("/setgeneral.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, request->url(), "text/html",false,processor);
  });
  server.on("/setgs.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, request->url(), "text/html",false,processor);
  });
  server.on("/setoutput.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, request->url(), "text/html",false,processor);
  });
  server.on("/setwifi.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, request->url(), "text/html",false,processor);
  });
  server.on("/developmenue.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, request->url(), "text/html",false,processor);
  });
  server.on("/index.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, request->url(), "text/html",false,processor);
  });
  server.on("/sendmessage.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, request->url(), "text/html",false,processor);
  });
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html",false,processor);
  });
  server.on("/info.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, request->url(), "text/html",false,processor);
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
  static uint32_t tRestart = millis();
  uint32_t tAct = millis();
  // Look for and handle WebSocket data
  webSocket.loop();
  if ((tAct - tLife) >= 500){
    tLife = tAct;
    StaticJsonDocument<300> doc;                      //Memory pool
    doc.clear();
    doc["counter"] = counter;
    doc["vBatt"] = String((float)status.vBatt/1000.,2);
    doc["gpsFix"] = status.GPS_Fix;
    doc["gpsNumSat"] = status.GPS_NumSat;
    doc["gpslat"] = String(status.GPS_Lat,6);
    doc["gpslon"] = String(status.GPS_Lon,6);
    doc["gpsAlt"] = String(status.GPS_alt,1);
    doc["gpsSpeed"] = String(status.GPS_speed,2);
    doc["climbrate"] = String(status.ClimbRate,1);
    doc["fanetTx"] = status.fanetTx;
    doc["fanetRx"] = status.fanetRx;
    doc["tLoop"] = status.tLoop;
    doc["tMaxLoop"] = status.tMaxLoop;
    doc["freeHeap"] = xPortGetFreeHeapSize();
    doc["fHeapMin"] = xPortGetMinimumEverFreeHeapSize();
    serializeJson(doc, msg_buf);
    for (int i = 0;i <MAXCLIENTS;i++){
      if (clientPages[i] == 1){
        log_d("Sending to [%u]: %s", i, msg_buf);
        webSocket.sendTXT(i, msg_buf);
      }
    }
    counter++;
  }
  if (restartNow){
    if ((tAct - tRestart) >= 1000){
      ESP.restart();
    }    
  }else{
    tRestart = tAct;
  }
}