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
  StaticJsonDocument<650> doc;                      //Memory pool
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
      
      log_i("[%u] Received text with size=%d paylod: %s", client_num, length, payload);      
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
        if (clientPages[client_num] == 1){ //info
          doc["myDevId"] = setting.myDevId;
          doc["compiledate"] = String(compile_date);
          doc["bHasVario"] = (uint8_t)status.vario.bHasVario;       
          doc["bHasMPU"] = (uint8_t)status.vario.bHasMPU;   
          doc["VisWeather"] = (uint8_t)(status.vario.bHasBME | status.bWUBroadCast);
          doc["board"] = setting.boardType;
          doc["disp"] = setting.displayType;
          doc["band"] = setting.band;
          doc["power"] = setting.LoraPower;
          doc["mode"] = setting.Mode;
          doc["type"] = (uint8_t)setting.AircraftType;
          doc["bHasGSM"] = (uint8_t)status.bHasGSM;
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);

          doc.clear();
          doc["wifiRssi"] = String(status.wifiRssi);
          doc["wifiStat"] = String(status.wifiStat);
        #ifdef GSM_MODULE
          doc["GSMRssi"] = String(status.GSMSignalQuality);
          doc["GSMStat"] = String(status.modemstatus);
        #endif
          doc["climbrate"] = String(status.ClimbRate,1);
          doc["vTemp"] = String(status.varioTemp,1);
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);

          doc.clear();
          doc["vBatt"] = String((float)status.vBatt/1000.,2);
          #ifdef AIRMODULE
          doc["gpsFix"] = status.GPS_Fix;
          doc["gpsNumSat"] = status.GPS_NumSat;
          doc["gpsSpeed"] = String(status.GPS_speed,2);
          #endif
          doc["gpslat"] = String(status.GPS_Lat,6);
          doc["gpslon"] = String(status.GPS_Lon,6);
          doc["gpsAlt"] = String(status.GPS_alt,1);
          doc["fanetTx"] = status.fanetTx;
          doc["fanetRx"] = status.fanetRx;
          doc["tLoop"] = status.tLoop;
          doc["tMaxLoop"] = status.tMaxLoop;
          doc["freeHeap"] = xPortGetFreeHeapSize();
          doc["fHeapMin"] = xPortGetMinimumEverFreeHeapSize();
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);

          #ifdef GSMODULE
          doc.clear();
          doc["wsTemp"] = String(status.weather.temp,1);
          doc["wsHum"] = String(status.weather.Humidity,1);
          doc["wsPress"] = String(status.weather.Pressure,2);
          doc["wsWDir"] = String(status.weather.WindDir,1);
          doc["wsWSpeed"] = String(status.weather.WindSpeed,1);
          doc["wsWGust"] = String(status.weather.WindGust,1);
          doc["wsR1h"] = String(status.weather.rain1h,1);
          doc["wsR1d"] = String(status.weather.rain1d,1);
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);
          #endif


        }else if (clientPages[client_num] == 2){ //sendmessage

        }else if (clientPages[client_num] == 10){ //full settings
          doc.clear();
          doc["board"] = setting.boardType;
          doc["disp"] = setting.displayType;
          doc["band"] = setting.band;
          doc["power"] = setting.LoraPower;
          doc["mode"] = setting.Mode;
          doc["type"] = (uint8_t)setting.AircraftType;
          doc["PilotName"] = setting.PilotName;
          doc["ognlive"] = setting.OGNLiveTracking;
          doc["traccar_live"] = setting.traccarLiveTracking;
          doc["traccarsrv"]= setting.TraccarSrv;
          doc["fntMode"] = setting.fanetMode;
          doc["legacytx"] = setting.LegacyTxEnable;
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);

          doc.clear();
          doc["output"] = setting.outputMode;
          doc["oSERIAL"] = setting.bOutputSerial;
          doc["oGPS"] = setting.outputGPS;
          doc["oFlarm"] = setting.outputFLARM;
          doc["oFanet"] = setting.outputFANET;
          doc["oLK8EX1"] = setting.outputLK8EX1;
          doc["awlive"] = setting.awLiveTracking;
          doc["UDPServerIP"] = setting.UDPServerIP;
          doc["UDPSendPort"] = setting.UDPSendPort;
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);

          doc.clear();
          doc["appw"] = setting.wifi.appw;
          doc["wificonnect"] = (uint8_t)setting.wifi.connect;
          doc["ssid"] = setting.wifi.ssid;
          doc["password"] = setting.wifi.password;
          doc["wifioff"] = setting.wifi.tWifiStop;
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);

          doc.clear();
          doc["gslat"] = setting.gs.lat;
          doc["gslon"] = setting.gs.lon;
          doc["gsalt"] = setting.gs.alt;
          doc["gsScr"] = setting.gs.SreenOption;
          doc["gsPs"] = setting.gs.PowerSave;
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);


          doc.clear();
          doc["axOffset"] = setting.vario.accel[0];
          doc["ayOffset"] = setting.vario.accel[1];
          doc["azOffset"] = setting.vario.accel[2];
          doc["gxOffset"] = setting.vario.gyro[0];
          doc["gyOffset"] = setting.vario.gyro[1];
          doc["gzOffset"] = setting.vario.gyro[2];
          doc["t[0]"] = setting.vario.tValues[0];
          doc["t[1]"] = setting.vario.tValues[1];
          doc["z[0]"] = setting.vario.zValues[0];
          doc["z[1]"] = setting.vario.zValues[1];
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);



          doc.clear();
          doc["sFWD"] = setting.wd.sendFanet;
          doc["wdTempOffset"] = setting.wd.tempOffset;
          doc["bHasVario"] = (uint8_t)status.vario.bHasVario;
          doc["bHasMPU"] = (uint8_t)status.vario.bHasMPU;
          doc["vSinkTh"] = serialized(String(setting.vario.sinkingThreshold,2));
          doc["vClimbTh"] = serialized(String(setting.vario.climbingThreshold,2));
          doc["vNClimbSens"] = serialized(String(setting.vario.nearClimbingSensitivity,2));
          doc["vVol"] = setting.vario.volume;
          doc["vBeepFly"] = setting.vario.BeepOnlyWhenFlying;
          doc["useMPU"] = (uint8_t)setting.vario.useMPU;
          doc["vTOffs"] = serialized(String(setting.vario.tempOffset,2));
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);

          doc.clear();
          doc["bHasBME"] = (uint8_t)status.vario.bHasBME;
          doc["WUUlEnable"] = setting.WUUpload.enable;
          doc["WUUlID"] = setting.WUUpload.ID;
          doc["WUUlKEY"] = setting.WUUpload.KEY;
          doc["WIUlEnable"] = setting.WindyUpload.enable;
          doc["WIUlID"] = setting.WindyUpload.ID;
          doc["WIUlKEY"] = setting.WindyUpload.KEY;
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);

          doc.clear();
          doc["bHasGSM"] = (uint8_t)status.bHasGSM;
          doc["GSMAPN"] = setting.gsm.apn;
          doc["GSMUSER"] = setting.gsm.user;
          doc["GSMPWD"] = setting.gsm.pwd;
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);
          

        }else if (clientPages[client_num] == 11){ //settings general
          doc["board"] = setting.boardType;
          doc["band"] = setting.band;
          doc["power"] = setting.LoraPower;
          doc["type"] = (uint8_t)setting.AircraftType;
          doc["PilotName"] = setting.PilotName;
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
          doc["appw"] = setting.wifi.appw;
          doc["ssid"] = setting.wifi.ssid;
          doc["password"] = setting.wifi.password;
          doc["wifioff"] = setting.wifi.tWifiStop;
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);
        }else if (clientPages[client_num] == 14){ //settings ground station
          doc["gslat"] = setting.gs.lat;
          doc["gslon"] = setting.gs.lon;
          doc["gsalt"] = setting.gs.alt;
          doc["mode"] = setting.Mode;
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
      }else if (root.containsKey("configGPS")){
        setting.bConfigGPS = true; //setup GPS
      }else if (root.containsKey("calibGyro")){
        setting.vario.bCalibGyro = true; //calibrate Gyro
      }else if (root.containsKey("calibAcc")){
        setting.vario.bCalibAcc = true; //calibrate accelerometer
      }else if (root.containsKey("save")){
        //save settings-page
        value = doc["save"];
        //general settings-page          
        SettingsData newSetting = setting;
        if (root.containsKey("appw")) newSetting.wifi.appw = doc["appw"].as<String>();          
        if (root.containsKey("wificonnect")) newSetting.wifi.connect = doc["wificonnect"].as<uint8_t>();
        if (root.containsKey("ssid")) newSetting.wifi.ssid = doc["ssid"].as<String>();
        if (root.containsKey("password")) newSetting.wifi.password = doc["password"].as<String>();
        if (root.containsKey("board")) newSetting.boardType = doc["board"].as<uint8_t>();          
        if (root.containsKey("disp")) newSetting.displayType = doc["disp"].as<uint8_t>();          
        if (root.containsKey("power")) newSetting.LoraPower = constrain(doc["power"].as<uint8_t>(),0,20);          
        if (root.containsKey("band")) newSetting.band = doc["band"].as<uint8_t>();          
        if (root.containsKey("type")) newSetting.AircraftType = (FanetLora::aircraft_t)doc["type"].as<uint8_t>();
        if (root.containsKey("PilotName")) newSetting.PilotName = doc["PilotName"].as<String>();
        if (root.containsKey("output")) newSetting.outputMode = doc["output"].as<uint8_t>();
        if (root.containsKey("oSERIAL")) newSetting.bOutputSerial = doc["oSERIAL"].as<uint8_t>();
        if (root.containsKey("oGPS")) newSetting.outputGPS = doc["oGPS"].as<uint8_t>();
        if (root.containsKey("oFlarm")) newSetting.outputFLARM = doc["oFlarm"].as<uint8_t>();
        if (root.containsKey("oFanet")) newSetting.outputFANET = doc["oFanet"].as<uint8_t>();
        if (root.containsKey("oLK8EX1")) newSetting.outputLK8EX1 = doc["oLK8EX1"].as<uint8_t>();
        if (root.containsKey("awlive")) newSetting.awLiveTracking = doc["awlive"].as<uint8_t>();
        if (root.containsKey("wifioff")) newSetting.wifi.tWifiStop = doc["wifioff"].as<uint32_t>();
        if (root.containsKey("UDPServerIP")) newSetting.UDPServerIP = doc["UDPServerIP"].as<String>();
        if (root.containsKey("UDPSendPort")) newSetting.UDPSendPort = doc["UDPSendPort"].as<uint16_t>();
        //gs settings
        if (root.containsKey("gslat")) newSetting.gs.lat = doc["gslat"].as<float>();
        if (root.containsKey("gslon")) newSetting.gs.lon = doc["gslon"].as<float>();
        if (root.containsKey("gsalt")) newSetting.gs.alt = doc["gsalt"].as<float>();
        if (root.containsKey("gsScr")) newSetting.gs.SreenOption = doc["gsScr"].as<uint8_t>();
        if (root.containsKey("gsPs")) newSetting.gs.PowerSave = doc["gsPs"].as<uint8_t>();
        
        if (root.containsKey("mode")) newSetting.Mode = doc["mode"].as<uint8_t>();
        
        if (root.containsKey("ognlive")) newSetting.OGNLiveTracking = doc["ognlive"].as<uint8_t>();
        if (root.containsKey("traccar_live")) newSetting.traccarLiveTracking = doc["traccar_live"].as<uint8_t>();
        if (root.containsKey("traccarsrv")) newSetting.TraccarSrv = doc["traccarsrv"].as<String>();
        if (root.containsKey("legacytx")) newSetting.LegacyTxEnable = doc["legacytx"].as<uint8_t>();
        if (root.containsKey("fntMode")) newSetting.fanetMode = doc["fntMode"].as<uint8_t>();        
        //weatherdata
        if (root.containsKey("sFWD")) newSetting.wd.sendFanet = doc["sFWD"].as<uint8_t>();
        if (root.containsKey("wdTempOffset")) newSetting.wd.tempOffset = doc["wdTempOffset"].as<float>();
        //vario
        if (root.containsKey("vSinkTh")) newSetting.vario.sinkingThreshold = doc["vSinkTh"].as<float>();
        if (root.containsKey("vClimbTh")) newSetting.vario.climbingThreshold = doc["vClimbTh"].as<float>();
        if (root.containsKey("vNClimbSens")) newSetting.vario.nearClimbingSensitivity = doc["vNClimbSens"].as<float>();
        if (root.containsKey("vVol")) newSetting.vario.volume = doc["vVol"].as<uint8_t>();
        if (root.containsKey("vBeepFly")) newSetting.vario.BeepOnlyWhenFlying = doc["vBeepFly"].as<uint8_t>();
        if (root.containsKey("useMPU")) newSetting.vario.useMPU = doc["useMPU"].as<uint8_t>();        
        if (root.containsKey("vTOffs")) newSetting.vario.tempOffset = doc["vTOffs"].as<float>();        
        if (root.containsKey("axOffset")) newSetting.vario.accel[0] = doc["axOffset"].as<int16_t>();
        if (root.containsKey("ayOffset")) newSetting.vario.accel[1] = doc["ayOffset"].as<int16_t>();
        if (root.containsKey("azOffset")) newSetting.vario.accel[2] = doc["azOffset"].as<int16_t>();
        if (root.containsKey("gxOffset")) newSetting.vario.gyro[0] = doc["gxOffset"].as<int16_t>();
        if (root.containsKey("gyOffset")) newSetting.vario.gyro[1] = doc["gyOffset"].as<int16_t>();
        if (root.containsKey("gzOffset")) newSetting.vario.gyro[2] = doc["gzOffset"].as<int16_t>();
        if (root.containsKey("t[0]")) newSetting.vario.tValues[0] = doc["t[0]"].as<float>();
        if (root.containsKey("t[1]")) newSetting.vario.tValues[1] = doc["t[1]"].as<float>();
        if (root.containsKey("z[0]")) newSetting.vario.zValues[0] = doc["z[0]"].as<float>();
        if (root.containsKey("z[1]")) newSetting.vario.zValues[1] = doc["z[1]"].as<float>();

        //weather-underground upload
        if (root.containsKey("WUUlEnable")) newSetting.WUUpload.enable = doc["WUUlEnable"].as<bool>();
        if (root.containsKey("WUUlID")) newSetting.WUUpload.ID = doc["WUUlID"].as<String>();
        if (root.containsKey("WUUlKEY")) newSetting.WUUpload.KEY = doc["WUUlKEY"].as<String>();
        //Windy upload
        if (root.containsKey("WIUlEnable")) newSetting.WindyUpload.enable = doc["WIUlEnable"].as<bool>();
        if (root.containsKey("WIUlID")) newSetting.WindyUpload.ID = doc["WIUlID"].as<String>();
        if (root.containsKey("WIUlKEY")) newSetting.WindyUpload.KEY = doc["WIUlKEY"].as<String>();
        //GSM
        if (root.containsKey("GSMAPN")) newSetting.gsm.apn = doc["GSMAPN"].as<String>();
        if (root.containsKey("GSMUSER")) newSetting.gsm.user = doc["GSMUSER"].as<String>();
        if (root.containsKey("GSMPWD")) newSetting.gsm.pwd = doc["GSMPWD"].as<String>();

        setting = newSetting;
        log_i("write config-to file");
        write_configFile(&newSetting);
        if (value == 2){
          log_i("reboot");
          ESP.restart();
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
    if (setting.Mode == MODE_DEVELOPER){
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
  }else if (var == "NEIGHBOURSLIST"){
    sRet = "";
    for (int i = 0; i < MAXNEIGHBOURS; i++){
      if (fanet.neighbours[i].devId){
        sRet += "<tr><th><a href=\"https://www.google.com/maps/search/?api=1&query=" + String(fanet.neighbours[i].lat,6) + "," + String(fanet.neighbours[i].lon,6)+ "\"  target=\"_blank\">" + fanet.neighbours[i].name + " [" + fanet.getDevId(fanet.neighbours[i].devId) + "]</a></th>" + 
        "<td>lat: " + String(fanet.neighbours[i].lat,6) + "</td>" + 
        "<td>lon: " + String(fanet.neighbours[i].lon,6) + "</td>" + 
        "<td>alt: " + String(fanet.neighbours[i].altitude,0) + "m</td>" +
        "<td>speed: " + String(fanet.neighbours[i].speed,0) + "km/h</td>" +
        "<td>climb: " + String(fanet.neighbours[i].climb,0) + "m/s</td>" +
        "<td>heading: " + String(fanet.neighbours[i].heading,0) + "°</td>" +
        "<td>rssi: " + String(fanet.neighbours[i].rssi) + "dB</td>" +
        "<td>last seen: " + String((millis() - fanet.neighbours[i].tLastMsg) / 1000) + "seconds</td>" +
        "</th>" +
        "\r\n";
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
    if (filename.startsWith("spiffs")){
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
  server.on("/fullsettings.html", HTTP_GET, [](AsyncWebServerRequest *request){
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
  server.on("/neighbours.html", HTTP_GET, [](AsyncWebServerRequest *request){
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

  server.on("/communicator.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, request->url(), "text/html",false,processor);
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

void Web_stop(void){
  webSocket.close();
  server.end();
}

void Web_loop(void){
  static uint32_t tLife = millis();
  static uint32_t tCounter = millis();
  static uint16_t counter = 0;
  static uint32_t tRestart = millis();
  uint32_t tAct = millis();
  bool bSend = false;
  statusData mStatus;
  StaticJsonDocument<300> doc; //Memory pool
  // Look for and handle WebSocket data
  webSocket.loop();



  if ((tAct - tLife) >= 100){
    tLife = tAct;
    //vario
    if (status.vario.bHasVario){
      doc.clear();
      bSend = false;
      if (mStatus.ClimbRate != status.ClimbRate){
        bSend = true;
        mStatus.ClimbRate = status.ClimbRate;
        doc["climbrate"] = String(status.ClimbRate,1);
      }    
      if (mStatus.varioTemp != status.varioTemp){
        bSend = true;
        mStatus.varioTemp = status.varioTemp;
        doc["vTemp"] = String(status.varioTemp,1);
      }    
      if (status.vario.bHasMPU){
        char buff[10];
        for (int i = 0; i < 3; i++){
          if (mStatus.vario.accel[i] != status.vario.accel[i]){
            bSend = true;
            mStatus.vario.accel[i] = status.vario.accel[i];
            sprintf (buff,"accel_%d",i);
            doc[buff] = status.vario.accel[i];
          }    
          if (mStatus.vario.gyro[i] != status.vario.gyro[i]){
            bSend = true;
            mStatus.vario.gyro[i] = status.vario.gyro[i];
            sprintf (buff,"gyro_%d",i);
            doc[buff] = status.vario.gyro[i];
          }    
        }
        if (mStatus.vario.acc_Z != status.vario.acc_Z){
          bSend = true;
          mStatus.vario.acc_Z = status.vario.acc_Z;
          doc["acc_z"] = String(status.vario.acc_Z,2);
        }    
      }
      if (bSend){
        serializeJson(doc, msg_buf);
        for (int i = 0;i <MAXCLIENTS;i++){
          if (clientPages[i] == 1){
            log_d("Sending to [%u]: %s", i, msg_buf);
            webSocket.sendTXT(i, msg_buf);
          }
        }
      }
    }


    doc.clear();
    bSend = false;
    if ((tAct - tCounter) >= 1000){
      tCounter = tAct;
      counter++;
      doc["counter"] = counter;
      bSend = true;
    }
    if (mStatus.vBatt != status.vBatt){
      bSend = true;
      mStatus.vBatt = status.vBatt;
      doc["vBatt"] = String((float)status.vBatt/1000.,2);
    }    
    #ifdef AIRMODULE
    if (mStatus.GPS_Fix != status.GPS_Fix){
      bSend = true;
      mStatus.GPS_Fix = status.GPS_Fix;
      doc["gpsFix"] = status.GPS_Fix;
    }    
    if (mStatus.GPS_NumSat != status.GPS_NumSat){
      bSend = true;
      mStatus.GPS_NumSat = status.GPS_NumSat;
      doc["gpsNumSat"] = status.GPS_NumSat;
    }    
    if (mStatus.GPS_speed != status.GPS_speed){
      bSend = true;
      mStatus.GPS_speed = status.GPS_speed;
      doc["gpsSpeed"] = String(status.GPS_speed,2);
    }    
    #endif
    if (mStatus.GPS_Lat != status.GPS_Lat){
      bSend = true;
      mStatus.GPS_Lat = status.GPS_Lat;
      doc["gpslat"] = String(status.GPS_Lat,6);
    }    
    if (mStatus.GPS_Lon != status.GPS_Lon){
      bSend = true;
      mStatus.GPS_Lon = status.GPS_Lon;
      doc["gpslon"] = String(status.GPS_Lon,6);
    }    
    if (mStatus.GPS_alt != status.GPS_alt){
      bSend = true;
      mStatus.GPS_alt = status.GPS_alt;
      doc["gpsAlt"] = String(status.GPS_alt,1);
    }    
    if (mStatus.fanetTx != status.fanetTx){
      bSend = true;
      mStatus.fanetTx = status.fanetTx;
      doc["fanetTx"] = status.fanetTx;
    }    
    if (mStatus.fanetRx != status.fanetRx){
      bSend = true;
      mStatus.fanetRx = status.fanetRx;
      doc["fanetRx"] = status.fanetRx;
    }    
    if (mStatus.tLoop != status.tLoop){
      bSend = true;
      mStatus.tLoop = status.tLoop;
      doc["tLoop"] = status.tLoop;
    }    
    if (mStatus.tMaxLoop != status.tMaxLoop){
      bSend = true;
      mStatus.tMaxLoop = status.tMaxLoop;
      doc["tMaxLoop"] = status.tMaxLoop;
    }    
    //doc["freeHeap"] = xPortGetFreeHeapSize();
    //doc["fHeapMin"] = xPortGetMinimumEverFreeHeapSize();
    if (bSend){
      serializeJson(doc, msg_buf);
      for (int i = 0;i <MAXCLIENTS;i++){
        if (clientPages[i] == 1){
          log_d("Sending to [%u]: %s", i, msg_buf);
          webSocket.sendTXT(i, msg_buf);
        }
      }
    }

    doc.clear();
    bSend = false;
    if (mStatus.wifiRssi != status.wifiRssi){
      bSend = true;
      mStatus.wifiRssi = status.wifiRssi;
      doc["wifiRssi"] = String(status.wifiRssi);
    }    
    if (mStatus.wifiStat != status.wifiStat){
      bSend = true;
      mStatus.wifiStat = status.wifiStat;
      doc["wifiStat"] = String(status.wifiStat);
    }    
    #ifdef GSM_MODULE
    if (mStatus.GSMSignalQuality != status.GSMSignalQuality){
      bSend = true;
      mStatus.GSMSignalQuality = status.GSMSignalQuality;
      doc["GSMRssi"] = String(status.GSMSignalQuality);
    }    
    if (mStatus.modemstatus != status.modemstatus){
      bSend = true;
      mStatus.modemstatus = status.modemstatus;
      doc["GSMStat"] = String(status.modemstatus);
    }    
    #endif
    if (bSend){
      serializeJson(doc, msg_buf);
      for (int i = 0;i <MAXCLIENTS;i++){
        if (clientPages[i] == 1){
          log_d("Sending to [%u]: %s", i, msg_buf);
          webSocket.sendTXT(i, msg_buf);
        }
      }
    }


    #ifdef GSMODULE
    //weahter-data
    doc.clear();
    bSend = false;
    if (mStatus.weather.temp != status.weather.temp){
      bSend = true;
      mStatus.weather.temp = status.weather.temp;
      doc["wsTemp"] = String(status.weather.temp,1);
    }    
    if (mStatus.weather.Humidity != status.weather.Humidity){
      bSend = true;
      mStatus.weather.Humidity = status.weather.Humidity;
      doc["wsHum"] = String(status.weather.Humidity,1);
    }    
    if (mStatus.weather.Pressure != status.weather.Pressure){
      bSend = true;
      mStatus.weather.Pressure = status.weather.Pressure;
      doc["wsPress"] = String(status.weather.Pressure,2);
    }    
    if (mStatus.weather.WindDir != status.weather.WindDir){
      bSend = true;
      mStatus.weather.WindDir = status.weather.WindDir;
      doc["wsWDir"] = String(status.weather.WindDir,1);
    }    
    if (mStatus.weather.WindSpeed != status.weather.WindSpeed){
      bSend = true;
      mStatus.weather.WindSpeed = status.weather.WindSpeed;
      doc["wsWSpeed"] = String(status.weather.WindSpeed,1);
    }    
    if (mStatus.weather.WindGust != status.weather.WindGust){
      bSend = true;
      mStatus.weather.WindGust = status.weather.WindGust;
      doc["wsWGust"] = String(status.weather.WindGust,1);
    }    
    if (mStatus.weather.rain1h != status.weather.rain1h){
      bSend = true;
      mStatus.weather.rain1h = status.weather.rain1h;
      doc["wsR1h"] = String(status.weather.rain1h,1);
    }    
    if (mStatus.weather.rain1d != status.weather.rain1d){
      bSend = true;
      mStatus.weather.rain1d = status.weather.rain1d;
      doc["wsR1d"] = String(status.weather.rain1d,1);
    }    
    if (bSend){
      serializeJson(doc, msg_buf);
      for (int i = 0;i <MAXCLIENTS;i++){
        if (clientPages[i] == 1){
          log_d("Sending to [%u]: %s", i, msg_buf);
          webSocket.sendTXT(i, msg_buf);
        }
      }
    }
    #endif
  }
  if (restartNow){
    if ((tAct - tRestart) >= 1000){
      ESP.restart();
    }    
  }else{
    tRestart = tAct;
  }
}