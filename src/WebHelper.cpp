#include <WebHelper.h>

// Globals
AsyncWebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(1337);
int led_state = 0;
char msg_buf[500];
#define MAXCLIENTS 10
uint8_t clientPages[MAXCLIENTS];

#define HTML 0
#define CSS 1
#define JS 2

struct websites {
    const char *name;
    const uint8_t *pData;
    size_t len;
    uint8_t type;
};

Logger logger;
/***********************************************************
 * Functions
 */

void sendPage(uint8_t pageNr,uint8_t clientNr);
void sendPageHeader(uint8_t client_num);
void sendReceivers(uint8_t client_num);

void sendReceivers(uint8_t client_num){
  StaticJsonDocument<400> doc;
  doc.clear();
  uint8_t count = fanet.getNeighboursCount();
  for (int i = 0; i < MAXWEATHERDATAS; i++){
    if (fanet.weatherDatas[i].devId){
      count++;
    }
  }
  doc["NBCount"] = count + 1;
  serializeJson(doc, msg_buf);
  webSocket.sendTXT(client_num, msg_buf);
  uint8_t iIndex = 0;
  for (int i = 0; i < MAXNEIGHBOURS; i++){
    if (fanet.neighbours[i].devId){              
      doc.clear();
      doc["INDEX"] = iIndex;
      doc["ID"] = fanet.getDevId(fanet.neighbours[i].devId);
      doc["NAME"] = (fanet.neighbours[i].name.length() > 0) ? fanet.neighbours[i].name : "";
      serializeJson(doc, msg_buf);
      webSocket.sendTXT(client_num, msg_buf);
      iIndex++;
    }
  }
  for (int i = 0; i < MAXWEATHERDATAS; i++){
    if (fanet.weatherDatas[i].devId){              
      doc.clear();
      doc["INDEX"] = iIndex;
      doc["ID"] = fanet.getDevId(fanet.weatherDatas[i].devId);
      doc["NAME"] = (fanet.weatherDatas[i].name.length() > 0) ? fanet.weatherDatas[i].name : "";
      serializeJson(doc, msg_buf);
      webSocket.sendTXT(client_num, msg_buf);
      iIndex++;
    }
  }
  doc.clear();
  doc["INDEX"] = iIndex;
  doc["ID"] = "000000";
  doc["NAME"] = "Send to All";
  serializeJson(doc, msg_buf);
  webSocket.sendTXT(client_num, msg_buf);
  iIndex++;

}



void sendPageHeader(uint8_t client_num){
  StaticJsonDocument<400> doc;
  doc.clear();
  doc["myDevId"] = setting.myDevId;
  doc["appname"] = String(APPNAME "-" VERSION);
  doc["buildDate"] = "build:" + compile_date + " sdk:" + String(ESP.getSdkVersion()) + + " img:" + ENV;
  //doc["buildDate"] = "build:" + String(IsoDate) + " sdk:" + String(ESP.getSdkVersion()) + + " img:" + ENV;
  if (setting.Mode == eMode::GROUND_STATION){
    doc["pilot"] = "station: " + setting.PilotName + " [" + setting.myDevId + "]";
  }else{
    doc["pilot"] = "pilot: " + setting.PilotName + " [" + setting.myDevId + "]";
  }
  doc["myIP"] = status.wifiSTA.ip;
  serializeJson(doc, msg_buf);
  webSocket.sendTXT(client_num, msg_buf);
}

String getEthState(eConnectionState state){
  switch (state){
    case IDLE:
      return "idle";
    case STARTED:
      return "started";
    case CONNECTING:
      return "connecting";
    case CONNECTED:
      return "connected";
    case FULL_CONNECTED:
      return "full connected";
    case DISCONNECTED:
      return "disconnected";
  }
  return "";
}

#ifdef GSM_MODULE
String getNetWorkStat(int16_t state){
  switch (state){
    case 0:
      return "no service";
    case 1:
      return "GSM";
    case 3:
      return "EGPRS";
    case 7:
      return "LTE M1";
    case 9:
      return "LTE NB";
  }
  return "";
}
#endif

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
        log_i("[%u] Connection from %s", client_num,ip.toString().c_str());
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
        sendPageHeader(client_num);
        doc.clear();
        if (clientPages[client_num] == 1){ //index.html
          #ifdef LOGGER
          doc["igcMenue"] = 1;
          #endif
          doc["developer"] = (setting.Mode == eMode::DEVELOPER) ? 1 : 0 ;
          doc["setView"] = setting.settingsView;
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);

        }else if (clientPages[client_num] == 2){ //info
          doc["myDevId"] = setting.myDevId;
          doc["compiledate"] = compile_date;
          doc["bHasVario"] = (uint8_t)status.vario.bHasVario;       
          doc["bHasMPU"] = (uint8_t)status.vario.bHasMPU;   
          doc["VisWeather"] = (uint8_t)((setting.wd.mode.bits.enable | status.bWUBroadCast) & (setting.Mode == eMode::GROUND_STATION));
          doc["board"] = setting.boardType;
          doc["Frequ"] = setting.CPUFrequency;
          doc["disp"] = setting.displayType;
          doc["mode"] = setting.Mode;
          doc["type"] = (uint8_t)setting.AircraftType;
          doc["bHasGSM"] = (uint8_t)status.gsm.bHasGSM;
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);

          doc.clear();
          doc["wifiAPState"] = getEthState(status.wifiAP.state);
          doc["wifiAPIp"] = status.wifiAP.ip;
          doc["wifiSTARssi"] = String(status.wifiSTA.Rssi);
          doc["wifiSTAIp"] = status.wifiSTA.ip;
          doc["wifiSTAState"] = getEthState(status.wifiSTA.state);
        #ifdef GSM_MODULE
          doc["GSMRssi"] = status.gsm.SignalQuality;
          doc["GSMStat"] = getEthState(status.modemstatus);
          doc["GSMNWSTAT"] = getNetWorkStat(status.gsm.networkstat);
          doc["GSMCOPS"] = status.gsm.sOperator;
        #endif
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);

          if (status.rtc.module != RTC_NONE){
            doc.clear();
            doc["RTC"] = uint8_t(status.rtc.module);
            doc["rtcTemp"] = String(status.rtc.temp,1);
            //doc["rtcVolt"] = String(status.rtc.voltage,2);
            serializeJson(doc, msg_buf);
            webSocket.sendTXT(client_num, msg_buf);            
          }
          doc.clear();
          doc["climbrate"] = String(status.vario.ClimbRate,1);
          doc["vTemp"] = String(status.vario.temp,1);
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);

          doc.clear();
          doc["vBatt"] = String((float)status.battery.voltage/1000.,2);
          doc["Battperc"] = status.battery.percent;
          #ifdef AIRMODULE
          if (setting.Mode == eMode::AIR_MODULE){
            doc["gpsFix"] = status.gps.Fix;
            doc["gpsNumSat"] = status.gps.NumSat;
            doc["gpsSpeed"] = String(status.gps.speed,2);
            doc["gpsCourse"] = String(status.gps.course,2);
          }
          #endif
          doc["gpslat"] = String(status.gps.Lat,6);
          doc["gpslon"] = String(status.gps.Lon,6);
          doc["gpsAlt"] = String(status.gps.alt,1);
          doc["gpsGAlt"] = String(status.gps.geoidAlt,1);
          doc["fanetFrequ"] = fanet.getLoraFrequency();
          doc["fanetTx"] = status.fanetTx;
          doc["fanetRx"] = status.fanetRx;
          doc["legFrequ"] = fanet.getFlarmFrequency();
          doc["legTx"] = status.legTx;
          doc["legRx"] = status.legRx;
          doc["tLoop"] = status.tLoop;
          doc["tMaxLoop"] = status.tMaxLoop;
          doc["freeHeap"] = xPortGetFreeHeapSize();
          doc["fHeapMin"] = xPortGetMinimumEverFreeHeapSize();
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);

          #ifdef GSMODULE
          if (setting.Mode == eMode::GROUND_STATION){
            doc.clear();
            doc["wdAnemo"] = setting.wd.anemometer.AnemometerType;
            doc["WsMode"] = setting.wd.mode.mode;
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
          }
          #endif
          doc.clear();
          doc["fuelSensor"] = (uint8_t)setting.bHasFuelSensor;
          doc["fuelValue"] = String(status.fuelSensor,3);
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);



        }else if (clientPages[client_num] == 3){ //sendmessage
          doc.clear();
          doc["setView"] = setting.settingsView;
          doc["FNTMSGIN"] = status.lastFanetMsg;
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);
          sendReceivers(client_num); //send clients

        }else if (clientPages[client_num] == 5){ //FW-Update
          doc["updateState"] = status.updateState;
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);

        }else if (clientPages[client_num] == 10){ //full settings
          doc["setView"] = setting.settingsView;                  
          doc["board"] = setting.boardType;
          doc["Frequ"] = setting.CPUFrequency;
          doc["FrqCor"] = setting.FrqCor;
          doc["disp"] = setting.displayType;
          doc["dispRot"] = setting.displayRotation;
          doc["expwsw"] = setting.bHasExtPowerSw;
          doc["mode"] = setting.Mode;
          doc["type"] = (uint8_t)setting.AircraftType;
          doc["PilotName"] = setting.PilotName;
          doc["ognlive"] = setting.OGNLiveTracking.mode;
          doc["traccar_live"] = setting.traccarLiveTracking;
          doc["traccarsrv"]= setting.TraccarSrv;
          doc["fntMode"] = setting.fanetMode;
          doc["fntPin"] = setting.fanetpin;
          doc["RFM"] = setting.RFMode;
          doc["AUTOUPDATE"] = setting.bAutoupdate;
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);

          doc.clear();
          doc["output"] = setting.outputMode;
          doc["oSERIAL"] = setting.bOutputSerial;
          doc["oGPS"] = setting.outputGPS;
          doc["oFlarm"] = setting.outputFLARM;
          doc["oFanet"] = setting.outputFANET;
          doc["oVario"] = setting.outputModeVario;
          doc["awlive"] = setting.awLiveTracking;
          doc["UDPServerIP"] = setting.UDPServerIP;
          doc["UDPSendPort"] = setting.UDPSendPort;
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);

          doc.clear();
          doc["appw"] = setting.wifi.appw;
          doc["wificonnect"] = (uint8_t)setting.wifi.connect;
          doc["WIFI_MODE"] = setting.wifi.uMode.mode;
          doc["ssid"] = setting.wifi.ssid;
          doc["password"] = setting.wifi.password;
          doc["wifioff"] = setting.wifi.tWifiStop;
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);

          doc.clear();
          doc["getGpsPos"] = command.getGpsPos;
          doc["gslat"] = setting.gs.lat;
          doc["gslon"] = setting.gs.lon;
          doc["gsalt"] = setting.gs.alt;
          doc["gsGeoAlt"] = setting.gs.geoidAlt;
          doc["gsScr"] = setting.gs.SreenOption;
          doc["gsPs"] = setting.gs.PowerSave;
          doc["GsSrO"] = setting.gs.sunriseOffset;
          doc["GsSsO"] = setting.gs.sunsetOffset;
          doc["wdAnemo"] = setting.wd.anemometer.AnemometerType;
          doc["wdFrequ"] = setting.wd.frequency;
          doc["wdAnemoAdsGain"] = setting.wd.anemometer.AnemometerAdsGain;
          doc["wdAnemoAdsWSpeedMinVoltage"] = setting.wd.anemometer.AnemometerAdsWSpeedMinVoltage;
          doc["wdAnemoAdsWSpeedMaxVoltage"] = setting.wd.anemometer.AnemometerAdsWSpeedMaxVoltage;
          doc["wdAnemoAdsWDirMinVoltage"] = setting.wd.anemometer.AnemometerAdsWDirMinVoltage;
          doc["wdAnemoAdsWDirMaxVoltage"] = setting.wd.anemometer.AnemometerAdsWDirMaxVoltage;
          doc["wdAnemoAdsWSpeedMinSpeed"] = setting.wd.anemometer.AnemometerAdsWSpeedMinSpeed;
          doc["wdAnemoAdsWSpeedMaxSpeed"] = setting.wd.anemometer.AnemometerAdsWSpeedMaxSpeed;
          doc["wdAnemoAdsWDirMinDir"] = setting.wd.anemometer.AnemometerAdsWDirMinDir;
          doc["wdAnemoAdsWDirMaxDir"] = setting.wd.anemometer.AnemometerAdsWDirMaxDir;
          doc["wdAnemoAdsVDivR1"] = setting.wd.anemometer.AnemometerAdsVDivR1;
          doc["wdAnemoAdsVDivR2"] = setting.wd.anemometer.AnemometerAdsVDivR2;
          doc["MinBatPerc"] = setting.minBattPercent;
          doc["restartBattPerc"] = setting.restartBattPercent;
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);

          doc.clear();
          for(int i = 0; i <MAXFNTUPLOADSTATIONS;i++){ //Fanet-Upload to WU and Windy
            doc["F2WuF"][i] = setting.FntWuUpload[i].FanetId;
            doc["F2WuI"][i] = setting.FntWuUpload[i].ID;
            doc["F2WuK"][i] = setting.FntWuUpload[i].KEY;
            doc["F2WiF"][i] = setting.FntWiUpload[i].FanetId;
            doc["F2WiI"][i] = setting.FntWiUpload[i].ID;
            doc["F2WiK"][i] = setting.FntWiUpload[i].KEY;
          }
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);

          doc.clear();
          doc["configGPS"] = command.ConfigGPS;
          doc["calibGyro"] = command.CalibGyro;
          doc["calibAcc"] = command.CalibAcc;
          doc["axOffset"] = setting.vario.accel[0];
          doc["ayOffset"] = setting.vario.accel[1];
          doc["azOffset"] = setting.vario.accel[2];
          doc["gxOffset"] = setting.vario.gyro[0];
          doc["gyOffset"] = setting.vario.gyro[1];
          doc["gzOffset"] = setting.vario.gyro[2];
          doc["t[0]"] = setting.vario.tValues[0];
          //doc["t[1]"] = setting.vario.tValues[1];
          doc["z[0]"] = setting.vario.zValues[0];
          //doc["z[1]"] = setting.vario.zValues[1];
          doc["sigmaA"] =  setting.vario.sigmaA;
          doc["sigmaP"] =  setting.vario.sigmaP;
          doc["stateCalibAcc"] = status.calibAccStat;
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);



          doc.clear();
          doc["WsMode"] = setting.wd.mode.mode;
          doc["sFWD"] = setting.wd.sendFanet;
          doc["wdTempOffset"] = setting.wd.tempOffset;
          doc["wdWDirOffset"] = setting.wd.windDirOffset;
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
          doc["WUUlEnable"] = setting.WUUpload.enable;
          doc["WUUlID"] = setting.WUUpload.ID;
          doc["WUUlKEY"] = setting.WUUpload.KEY;
          doc["WIUlEnable"] = setting.WindyUpload.enable;
          doc["WIUlID"] = setting.WindyUpload.ID;
          doc["WIUlKEY"] = setting.WindyUpload.KEY;
          doc["WFFNT"] = setting.wd.avgFactorFanet;
          doc["WIFNT"] = setting.wd.FanetUploadInterval / 1000;
          doc["WFWU"] = setting.wd.avgFactorWU;
          doc["WIWU"] = setting.wd.WUUploadIntervall / 1000;
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);

          doc.clear();
          doc["bHasGSM"] = (uint8_t)status.gsm.bHasGSM;
          doc["GSMAPN"] = setting.gsm.apn;
          doc["GSMUSER"] = setting.gsm.user;
          doc["GSMPWD"] = setting.gsm.pwd;
          doc["GSMMODE"] = setting.gsm.NetworkMode;
          doc["GSMPREF"] = setting.gsm.PreferredMode; 
          doc["NBIOT"] = setting.gsm.NB_IOT_Band;
          doc["CATM"] = setting.gsm.CAT_M_Band;     
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);

          doc.clear();
          doc["fuelSensor"] = (uint8_t)setting.bHasFuelSensor;
          doc["vBatt"] = String((float)status.battery.voltage/1000.,2);
          doc["BATOFFS"] = setting.BattVoltOffs;
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);
          
          doc.clear();
          doc["MqttMode"] = (uint8_t)setting.mqtt.mode.mode ;
          doc["MqttServer"] = setting.mqtt.server ;
          doc["MqttPort"] = setting.mqtt.port;
          doc["MqttPw"] = setting.mqtt.pw;
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);
        }else if (clientPages[client_num] == 101){ //msg-type 1 test
          doc["lat"] = String(fanetTrackingData.lat,6);
          doc["lon"] = String(fanetTrackingData.lon,6);
          doc["alt"] = String(fanetTrackingData.altitude,1);
          doc["speed"] = String(fanetTrackingData.speed,2);
          doc["climb"] = String(fanetTrackingData.climb,1);
          doc["heading"] = String(fanetTrackingData.heading,1);
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);
        }else if (clientPages[client_num] == 102){ //msg-type 2 test
          doc["name"] = fanetString;
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);
        }else if (clientPages[client_num] == 103){ //msg-type 3 test
          doc["msg"] = fanetString;
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);
        }else if (clientPages[client_num] == 104){ //msg-type 4 test
          doc["lat"] = String(fanetWeatherData.lat,6);
          doc["lon"] = String(fanetWeatherData.lon,6);
          doc["temp"] = String(fanetWeatherData.temp,1);
          doc["wHeading"] = String(fanetWeatherData.wHeading,2);
          doc["wSpeed"] = String(fanetWeatherData.wSpeed,1);
          doc["wGust"] = String(fanetWeatherData.wGust,1);
          doc["hum"] = String(fanetWeatherData.Humidity,1);
          doc["pressure"] = String(fanetWeatherData.Baro,1);
          doc["charge"] = fanetWeatherData.Charge;
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);

        }        
      }else if (root.containsKey("getGpsPos")){
        command.getGpsPos = doc["getGpsPos"].as<uint8_t>(); //get GPS Position
      }else if (root.containsKey("configGPS")){
        command.ConfigGPS = doc["configGPS"].as<uint8_t>(); //setup GPS
      }else if (root.containsKey("calibGyro")){
        command.CalibGyro =  doc["calibGyro"].as<uint8_t>(); //calibrate Gyro
      }else if (root.containsKey("calibAcc")){
        command.CalibAcc = doc["calibAcc"].as<uint8_t>(); //calibrate accelerometer
      }else if (root.containsKey("updateState")){
        status.updateState = doc["updateState"].as<uint8_t>();
      }else if (root.containsKey("BATOFFS")){
        setting.BattVoltOffs = doc["BATOFFS"].as<float>();
        write_battOffset();
      }else if (root.containsKey("save")){
        //save settings-page
        value = doc["save"];
        //general settings-page          
        SettingsData newSetting = setting;
        if (root.containsKey("setView")) newSetting.settingsView = doc["setView"].as<uint8_t>();                  
        if (root.containsKey("appw")) newSetting.wifi.appw = doc["appw"].as<String>();          
        if (root.containsKey("wificonnect")) newSetting.wifi.connect = eWifiMode(doc["wificonnect"].as<uint8_t>());
        if (root.containsKey("WIFI_MODE")) newSetting.wifi.uMode.mode = doc["WIFI_MODE"].as<uint8_t>();
        if (root.containsKey("ssid")) newSetting.wifi.ssid = doc["ssid"].as<String>();
        if (root.containsKey("password")) newSetting.wifi.password = doc["password"].as<String>();
        if (root.containsKey("board")) newSetting.boardType = eBoard(doc["board"].as<uint8_t>());          
        if (root.containsKey("Frequ")) newSetting.CPUFrequency = eBoard(doc["Frequ"].as<uint8_t>());    
        if (root.containsKey("FrqCor")) newSetting.FrqCor = eBoard(doc["FrqCor"].as<int32_t>());    
        if (root.containsKey("disp")) newSetting.displayType = eDisplay(doc["disp"].as<uint8_t>());          
        if (root.containsKey("dispRot")) newSetting.displayRotation = doc["dispRot"].as<uint8_t>();          
        if (root.containsKey("expwsw")) newSetting.bHasExtPowerSw = doc["expwsw"].as<uint8_t>();
        if (root.containsKey("type")) newSetting.AircraftType = (FanetLora::aircraft_t)doc["type"].as<uint8_t>();
        if (root.containsKey("PilotName")) newSetting.PilotName = doc["PilotName"].as<String>();
        if (root.containsKey("output")) newSetting.outputMode = eOutput(doc["output"].as<uint8_t>());
        if (root.containsKey("oSERIAL")) newSetting.bOutputSerial = doc["oSERIAL"].as<uint8_t>();
        if (root.containsKey("oGPS")) newSetting.outputGPS = doc["oGPS"].as<uint8_t>();
        if (root.containsKey("oFlarm")) newSetting.outputFLARM = doc["oFlarm"].as<uint8_t>();
        if (root.containsKey("oFanet")) newSetting.outputFANET = doc["oFanet"].as<uint8_t>();
        if (root.containsKey("oVario")) newSetting.outputModeVario = eOutputVario(doc["oVario"].as<uint8_t>());
        if (root.containsKey("awlive")) newSetting.awLiveTracking = doc["awlive"].as<uint8_t>();
        if (root.containsKey("wifioff")) newSetting.wifi.tWifiStop = doc["wifioff"].as<uint32_t>();
        if (root.containsKey("UDPServerIP")) newSetting.UDPServerIP = doc["UDPServerIP"].as<String>();
        if (root.containsKey("UDPSendPort")) newSetting.UDPSendPort = doc["UDPSendPort"].as<uint16_t>();
        if (root.containsKey("AUTOUPDATE")) newSetting.bAutoupdate = doc["AUTOUPDATE"].as<uint8_t>();
        //gs settings
        if (root.containsKey("gslat")) newSetting.gs.lat = doc["gslat"].as<float>();
        if (root.containsKey("gslon")) newSetting.gs.lon = doc["gslon"].as<float>();
        if (root.containsKey("gsalt")) newSetting.gs.alt = doc["gsalt"].as<float>();
        if (root.containsKey("gsGeoAlt")) newSetting.gs.geoidAlt = doc["gsGeoAlt"].as<float>();
        if (root.containsKey("gsScr")) newSetting.gs.SreenOption = eScreenOption(doc["gsScr"].as<uint8_t>());
        if (root.containsKey("gsPs")) newSetting.gs.PowerSave = eGsPower(doc["gsPs"].as<uint8_t>());
        if (root.containsKey("GsSrO")) newSetting.gs.sunriseOffset = eGsPower(doc["GsSrO"].as<int32_t>());
        if (root.containsKey("GsSsO")) newSetting.gs.sunsetOffset = eGsPower(doc["GsSsO"].as<int32_t>());
        //aneometer settings
        if (root.containsKey("wdFrequ")) newSetting.wd.frequency = doc["wdFrequ"].as<float>();
        if (root.containsKey("wdAnemo")) newSetting.wd.anemometer.AnemometerType = eAnemometer(doc["wdAnemo"].as<uint8_t>());
        if (root.containsKey("wdAnemoAdsGain")) newSetting.wd.anemometer.AnemometerAdsGain = eAnemometer(doc["wdAnemoAdsGain"].as<uint8_t>());
        if (root.containsKey("wdAnemoAdsWSpeedMinVoltage")) newSetting.wd.anemometer.AnemometerAdsWSpeedMinVoltage = doc["wdAnemoAdsWSpeedMinVoltage"].as<float>();
        if (root.containsKey("wdAnemoAdsWSpeedMaxVoltage")) newSetting.wd.anemometer.AnemometerAdsWSpeedMaxVoltage = doc["wdAnemoAdsWSpeedMaxVoltage"].as<float>();
        if (root.containsKey("wdAnemoAdsWDirMinVoltage")) newSetting.wd.anemometer.AnemometerAdsWDirMinVoltage = doc["wdAnemoAdsWDirMinVoltage"].as<float>();
        if (root.containsKey("wdAnemoAdsWDirMaxVoltage")) newSetting.wd.anemometer.AnemometerAdsWDirMaxVoltage = doc["wdAnemoAdsWDirMaxVoltage"].as<float>();
        if (root.containsKey("wdAnemoAdsWSpeedMinSpeed")) newSetting.wd.anemometer.AnemometerAdsWSpeedMinSpeed = doc["wdAnemoAdsWSpeedMinSpeed"].as<float>();
        if (root.containsKey("wdAnemoAdsWSpeedMaxSpeed")) newSetting.wd.anemometer.AnemometerAdsWSpeedMaxSpeed = doc["wdAnemoAdsWSpeedMaxSpeed"].as<float>();
        if (root.containsKey("wdAnemoAdsWDirMinDir")) newSetting.wd.anemometer.AnemometerAdsWDirMinDir = doc["wdAnemoAdsWDirMinDir"].as<float>();
        if (root.containsKey("wdAnemoAdsWDirMaxDir")) newSetting.wd.anemometer.AnemometerAdsWDirMaxDir = doc["wdAnemoAdsWDirMaxDir"].as<float>();
        if (root.containsKey("wdAnemoAdsVDivR1")) newSetting.wd.anemometer.AnemometerAdsVDivR1 = doc["wdAnemoAdsVDivR1"].as<float>();
        if (root.containsKey("wdAnemoAdsVDivR2")) newSetting.wd.anemometer.AnemometerAdsVDivR2 = doc["wdAnemoAdsVDivR2"].as<float>();

        if (root.containsKey("MinBatPerc")) newSetting.minBattPercent = doc["MinBatPerc"].as<uint8_t>();
        if (root.containsKey("restartBattPerc")) newSetting.restartBattPercent = doc["restartBattPerc"].as<uint8_t>();
        for(int i = 0; i < MAXFNTUPLOADSTATIONS;i++){ //Fanet-Upload to WU and Windy
          if (doc["F2WuF"][i]) newSetting.FntWuUpload[i].FanetId = doc["F2WuF"][i].as<uint32_t>();
          if (doc["F2WuI"][i]) newSetting.FntWuUpload[i].ID = doc["F2WuI"][i].as<String>();
          if (doc["F2WuK"][i]) newSetting.FntWuUpload[i].KEY = doc["F2WuK"][i].as<String>();
          if (doc["F2WiF"][i]) newSetting.FntWiUpload[i].FanetId = doc["F2WiF"][i].as<uint32_t>();
          if (doc["F2WiI"][i]) newSetting.FntWiUpload[i].ID = doc["F2WiI"][i].as<String>();
          if (doc["F2WiK"][i]) newSetting.FntWiUpload[i].KEY = doc["F2WiK"][i].as<String>();
        }
        
        if (root.containsKey("mode")) newSetting.Mode = eMode(doc["mode"].as<uint8_t>());
        
        if (root.containsKey("ognlive")) newSetting.OGNLiveTracking.mode = doc["ognlive"].as<uint8_t>();
        if (root.containsKey("traccar_live")) newSetting.traccarLiveTracking = doc["traccar_live"].as<uint8_t>();
        if (root.containsKey("traccarsrv")) newSetting.TraccarSrv = doc["traccarsrv"].as<String>();
        if (root.containsKey("RFM")) newSetting.RFMode = doc["RFM"].as<uint8_t>();
        if (root.containsKey("fntMode")) newSetting.fanetMode = eFnMode(doc["fntMode"].as<uint8_t>());
        if (root.containsKey("fntPin")) newSetting.fanetpin = doc["fntPin"].as<uint16_t>();
        //weatherdata
        if (root.containsKey("sFWD")) newSetting.wd.sendFanet = doc["sFWD"].as<uint8_t>();
        if (root.containsKey("wdTempOffset")) newSetting.wd.tempOffset = doc["wdTempOffset"].as<float>();
        if (root.containsKey("wdWDirOffset")) newSetting.wd.windDirOffset = doc["wdWDirOffset"].as<int16_t>();
        if (root.containsKey("WFFNT")) newSetting.wd.avgFactorFanet = doc["WFFNT"].as<float>();
        if (newSetting.wd.avgFactorFanet <= 0) newSetting.wd.avgFactorFanet = 64; //prevent division 0
        if (root.containsKey("WIFNT")) newSetting.wd.FanetUploadInterval = doc["WIFNT"].as<uint32_t>() * 1000;        
        if (newSetting.wd.WUUploadIntervall <= 10000) newSetting.wd.WUUploadIntervall = 10000;
        if (root.containsKey("WFWU")) newSetting.wd.avgFactorWU = doc["WFWU"].as<float>();
        if (newSetting.wd.avgFactorWU <= 0) newSetting.wd.avgFactorWU = 128; //prevent division 0
        if (root.containsKey("WIWU")) newSetting.wd.WUUploadIntervall = doc["WIWU"].as<uint32_t>() * 1000;        
        if (newSetting.wd.WUUploadIntervall <= 10000) newSetting.wd.WUUploadIntervall = 10000;
        if (root.containsKey("WsMode")) newSetting.wd.mode.mode = doc["WsMode"].as<uint8_t>();        
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
        //if (root.containsKey("t[1]")) newSetting.vario.tValues[1] = doc["t[1]"].as<float>();
        if (root.containsKey("z[0]")) newSetting.vario.zValues[0] = doc["z[0]"].as<float>();
        //if (root.containsKey("z[1]")) newSetting.vario.zValues[1] = doc["z[1]"].as<float>();
        if (root.containsKey("sigmaA")) newSetting.vario.sigmaA = doc["sigmaA"].as<float>();
        if (root.containsKey("sigmaP")) newSetting.vario.sigmaP = doc["sigmaP"].as<float>();

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
        if (root.containsKey("GSMMODE")) newSetting.gsm.NetworkMode = eGsmNetworkMode(doc["GSMMODE"].as<uint8_t>());
        if (root.containsKey("GSMPREF")) newSetting.gsm.PreferredMode = eGsmPreferedMode(doc["GSMPREF"].as<uint8_t>());
        if (root.containsKey("NBIOT")) newSetting.gsm.NB_IOT_Band = doc["NBIOT"].as<String>();
        if (root.containsKey("CATM")) newSetting.gsm.CAT_M_Band = doc["CATM"].as<String>();
        //fuel-sensor
        if (root.containsKey("fuelSensor")) newSetting.bHasFuelSensor = doc["fuelSensor"].as<uint8_t>();

        if (root.containsKey("MqttMode")) newSetting.mqtt.mode.mode = doc["MqttMode"].as<uint8_t>();
        if (root.containsKey("MqttServer")) newSetting.mqtt.server = doc["MqttServer"].as<String>();
        if (root.containsKey("MqttPort")) newSetting.mqtt.port = doc["MqttPort"].as<uint16_t>();
        if (root.containsKey("MqttPw")) newSetting.mqtt.pw = doc["MqttPw"].as<String>();

        setting = newSetting;
        //log_i("write config-to file");
        //write_configFile(&newSetting);
        if (value == 2){
          log_i("write config-to file");
          write_configFile(&newSetting);
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
          fanetTrackingData.lat = doc["lat"].as<float>();
          fanetTrackingData.lon = doc["lon"].as<float>();
          fanetTrackingData.aircraftType = (FanetLora::aircraft_t)doc["type"].as<uint8_t>();
          fanetTrackingData.altitude = doc["alt"].as<float>();
          fanetTrackingData.speed = doc["speed"].as<float>();
          fanetTrackingData.climb = doc["climb"].as<float>();
          fanetTrackingData.heading = doc["heading"].as<float>();
          sendFanetData = value; //          
        }else if (value == 2){
          //name
          fanetString = doc["name"].as<String>();
          sendFanetData = value;          
        }else if (value == 3){
          //msg
          fanetString = doc["msg"].as<String>();
          fanetReceiver = strtol( doc["receiver"].as<String>().c_str(), NULL, 16);
          sendFanetData = value;          
        }else if (value == 4){
          //send fanet msgtype 4
          fanetWeatherData.lat = doc["lat"].as<float>();
          fanetWeatherData.lon = doc["lon"].as<float>();
          fanetWeatherData.temp = doc["temp"].as<float>();
          fanetWeatherData.wHeading = doc["wHeading"].as<float>();
          fanetWeatherData.wSpeed = doc["wSpeed"].as<float>();
          fanetWeatherData.wGust = doc["wGust"].as<float>();
          fanetWeatherData.Humidity = doc["hum"].as<float>();
          fanetWeatherData.Baro = doc["pressure"].as<float>();
          fanetWeatherData.Charge = doc["charge"].as<uint8_t>();
          sendFanetData = value; //          
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
      log_i("[%u] type=%d", client_num,type);
      break;
  }
}

void SD_file_delete(AsyncWebServerRequest *request){
  int paramsNr = request->params();
  for(int i=0;i<paramsNr;i++){

    AsyncWebParameter* p = request->getParam(i);
 
     Serial.print("Param name: ");
     Serial.println(p->name());
 
     Serial.print("Param value: ");
     Serial.println(p->value());

    char igcf[40];
    p->value().toCharArray(igcf,40);
    logger.deleteFile(SD_MMC, igcf);
 
  }
}

void SD_file_download(AsyncWebServerRequest *request){

  int paramsNr = request->params();
  for(int i=0;i<paramsNr;i++){

    AsyncWebParameter* p = request->getParam(i);
 
     Serial.print("Param name: ");
     Serial.println(p->name());
 
     Serial.print("Param value: ");
     Serial.println(p->value());
 
    File download = SD_MMC.open(p->value());
    if (download) {
      request->send(SD_MMC, p->value(), "text/text", true);
    } 
  }
}

String processor(const String& var){
  String sRet = "";
  //log_i("%s",var.c_str());
  if (var == "IGCFILELIST"){
    // TODO list all igc files and create link to download
    logger.listFiles(SD_MMC,"/");
    sRet = "";
    char* d = strtok(logger.igclist, ";");
    sRet += "<table><thead><tr><th>Download</th><th>Delete</th></tr></thead><tbody>";
    while (d != NULL ) {
        //Serial.println (d);
        if((!String(d).startsWith("/._") )){//&& !String(d).startsWith("/test"))){
          sRet += "<tr><td><button class='button bsil'>";
          sRet += "<a href='/download?igc="+String(d)+"' download>";
          sRet += "<span style='color:black'>"+String(d)+"</span></a>";
          sRet += "</button></td>";
          sRet += "<td><button title='Delete' class='button bred'>";
          sRet += "<a href='/deleteigc?igc="+String(d)+"' target='_blank'>";
          sRet += " X </a></button></td>";
          sRet += "</tr>";
        }
        d = strtok(NULL, ";");
    }
    sRet += "</tbody></table>";
    return sRet;
  }else if (var == "NEIGHBOURS"){
    sRet = "";
    //sRet += "<option value=\"08AF88\">mytest 08AF88</option>\r\n";
    for (int i = 0; i < MAXNEIGHBOURS; i++){
      if (fanet.neighbours[i].devId){
        sRet += "<option value=\"" + fanet.getDevId(fanet.neighbours[i].devId) + "\">";
        if (fanet.neighbours[i].name.length() > 0){
          sRet += fanet.neighbours[i].name;
        }
        sRet +=  " [" + fanet.getDevId(fanet.neighbours[i].devId) + "]</option>\r\n";
        //sRet += "<option value=\"" + fanet.getDevId(fanet.neighbours[i].devId) + "\">" + fanet.neighbours[i].name + " " + fanet.getDevId(fanet.neighbours[i].devId) + "</option>\r\n";
      }
    }
    for (int i = 0; i < MAXWEATHERDATAS; i++){
      if (fanet.weatherDatas[i].devId){
        sRet += "<option value=\"" + fanet.getDevId(fanet.weatherDatas[i].devId) + "\">";
        if (fanet.weatherDatas[i].name.length() > 0){
          sRet += fanet.weatherDatas[i].name;
        }
        sRet +=  " [" + fanet.getDevId(fanet.weatherDatas[i].devId) + "]</option>\r\n";
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
  //uint32_t free_space = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
  size_t uploaded = index + len;
  size_t uploadSize = request->contentLength();
  int progress = ((uploaded * 100) / uploadSize)/5;
  static int last_perc = 0;  

  if (!index){
    log_i("webupdate starting");
    WebUpdateRunning = true;
    delay(500); //wait 1 second until tasks are stopped
          
    //Update.runAsync(true);
    if (filename.startsWith("spiffs")){
      if (!Update.begin(0x30000,U_SPIFFS)) {
        Update.printError(Serial);
      }
    }else{
      //if (!Update.begin(free_space,U_FLASH)) {
      if (!Update.begin(UPDATE_SIZE_UNKNOWN,U_FLASH)) {
        Update.printError(Serial);
      }
    }
  }
  if (!Update.hasError()){

    if(last_perc != progress){
      last_perc = progress;
      log_i("update progress=%d%",progress*5);
    }
    if (Update.write(data, len) != len){
        Update.printError(Serial);
    }
  }

  if (final) {
    if (!Update.end(true)){
      Update.printError(Serial);
    } else {
      restartNow = true;//Set flag so main loop can issue restart call
      log_i("Update complete");      
    }
  }
}

#ifdef useSpiffsWebsite
void loadFromSPIFFS(AsyncWebServerRequest *request,String path,String dataType = "text/html") {
  AsyncWebServerResponse *response = request->beginResponse(SPIFFS, path, dataType, false);
  response->addHeader("Content-Encoding", "gzip");
  request->send(response);
}
#else
void loadFromFlash(AsyncWebServerRequest *request,const uint8_t * content, size_t len,uint8_t type= HTML) {
  String dataType;
  if (type == HTML){
    dataType = "text/html";
  }else if (type == CSS){
    dataType = "text/css";
  }else if (type == JS){
    dataType = "text/javascript";
  }
  AsyncWebServerResponse *response = request->beginResponse_P(200, dataType,content, len);
  response->addHeader("Content-Encoding", "gzip");
  request->send(response);   
}
#endif


void Web_setup(void){
  websites sites[] = {
    {"/",index_html_gz,index_html_gz_len,HTML},
    {"/index.html",index_html_gz,index_html_gz_len,HTML},
    {"/favicon.ico",favicon_ico_gz,favicon_ico_gz_len,HTML},
    {"/weather.html",weather_html_gz,weather_html_gz_len,HTML},
    {"/neighbours.html",neighbours_html_gz,neighbours_html_gz_len,HTML},
    {"/info.html",info_html_gz,info_html_gz_len,HTML},
    {"/communicator.html",communicator_html_gz,communicator_html_gz_len,HTML},
    {"/sendmessage.html",sendmessage_html_gz,sendmessage_html_gz_len,HTML},
    {"/igclogs.html",igclogs_html_gz,igclogs_html_gz_len,HTML},
    {"/fullsettings.html",fullsettings_html_gz,fullsettings_html_gz_len,HTML},
    {"/fwupdate",fwupdate_html_gz,fwupdate_html_gz_len,HTML},
    {"/style.css",style_css_gz,style_css_gz_len,CSS},
    {"/scripts.js",scripts_js_gz,scripts_js_gz_len,JS}
  };
  for (int i = 0;i < MAXCLIENTS;i++) clientPages[i] = 0;
  // On HTTP request for root, provide index.html file
  for (const auto &site : sites) {
    server.on(site.name, HTTP_GET, [site](AsyncWebServerRequest *request){
      #ifdef useSpiffsWebsite      
        loadFromSPIFFS(request,request->url() + ".html.gz");
      #else
        loadFromFlash(request,site.pData ,site.len,site.type);
      #endif
    });
  }
  // handler for the /update form POST (once file upload finishes)
  server.on("/fwupdate", HTTP_POST, [](AsyncWebServerRequest *request){
      request->send(200);
    }, handle_update_progress_cb);
  /*
  server.on("/download", HTTP_GET, [](AsyncWebServerRequest *request){
    SD_file_download(request);
  });
  server.on("/deleteigc", HTTP_GET, [](AsyncWebServerRequest *request){
    SD_file_delete(request);
    request->redirect("/igclogs.html");    
  });

  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    #ifdef useSpiffsWebsite
      loadFromSPIFFS(request,request->url() + ".gz","text/css");
    #else
      loadFromFlash(request,style_css_gz,style_css_gz_len,"text/css");
    #endif
  });
  server.on("/scripts.js", HTTP_GET, [](AsyncWebServerRequest *request){
    #ifdef useSpiffsWebsite
      loadFromSPIFFS(request,request->url() + ".gz","text/javascript");
    #else
      loadFromFlash(request,scripts_js_gz,scripts_js_gz_len,"text/javascript");
    #endif
  });
  */  
  // Handle requests for pages that do not exist
  server.onNotFound(onPageNotFound);

  log_i("start webserver");
  // Start web server
  server.begin();

  // Start WebSocket server and assign callback
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
}

void Web_stop(void){
  log_i("stop webserver");
  webSocket.close();
  server.end();
}

void sendPage(uint8_t pageNr,uint8_t clientNr){
  static statusData mStatus;
  static commandData mCommand;
  StaticJsonDocument<500> doc; //Memory pool  
  bool bSend = false;
  uint32_t tAct = millis();
  static uint32_t tCounter = millis();
  static uint16_t counter = 0;
  static uint32_t tCount20 = millis();
  static uint32_t tCount30 = millis();
  static float flarmFrequ = 0;
  switch (pageNr) {
    case 2:
      //page info.html
      //vario
      if (status.vario.bHasVario){
        doc.clear();
        bSend = false;
        if (mStatus.vario.ClimbRate != status.vario.ClimbRate){
          bSend = true;
          mStatus.vario.ClimbRate = status.vario.ClimbRate;
          doc["climbrate"] = String(status.vario.ClimbRate,1);
        }    
        if (mStatus.vario.temp != status.vario.temp){
          bSend = true;
          mStatus.vario.temp = status.vario.temp;
          doc["vTemp"] = String(status.vario.temp,1);
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
          webSocket.sendTXT(clientNr, msg_buf);
        }
      }

      doc.clear();
      bSend = false;
      if ((tAct - tCounter) >= 1000){
        tCounter = tAct;
        counter++;
        time_t now;
        char strftime_buf[64];
        struct tm timeinfo;
        time(&now);
        gmtime_r(&now, &timeinfo);
        strftime(strftime_buf, sizeof(strftime_buf), "%F %T", &timeinfo);   
        doc["time"] = strftime_buf;
        //log_i("actual time %s",strftime_buf);
        /*
        if (setting.Mode == eMode::AIR_MODULE){
          if ((status.gps.Time) && (status.gps.Date)){
              doc["GPSTime"] = String(status.gps.Date) + "-" + String(status.gps.Time);
          }
        }
        if (status.gps.Time){
          char gpstime[40];
          snprintf (gpstime,sizeof(gpstime)-1,"GPS: 20%.2s-%.2s-%.2s - %.2s:%.2s:%.2s",&status.gps.Date[4],&status.gps.Date[2],&status.gps.Date[0],&status.gps.Time[0],&status.gps.Time[2],&status.gps.Time[4]);
          char hh[4]; 
          char mm[4];
          char ss[4];
          String(status.gps.Time).substring(0,2).toCharArray(hh,sizeof(hh),0);
          String(status.gps.Time).substring(2,4).toCharArray(mm,sizeof(mm),0);
          String(status.gps.Time).substring(4,6).toCharArray(ss,sizeof(ss),0);
          strcpy(gpstime,status.gps.Date);
          strcat(gpstime," - ");
          strcat(gpstime,hh);
          strcat(gpstime,":");
          strcat(gpstime,mm);
          strcat(gpstime,":");
          strcat(gpstime,ss);
          doc["time"] = gpstime;
        }else{
          doc["time"] = strftime_buf;
        }
        */

        doc["freeHeap"] = xPortGetFreeHeapSize();
        doc["fHeapMin"] = xPortGetMinimumEverFreeHeapSize();
        doc["counter"] = counter;
        bSend = true;
      }
      if (mStatus.battery.voltage != status.battery.voltage){
        bSend = true;
        mStatus.battery.voltage = status.battery.voltage;
        doc["vBatt"] = String((float)status.battery.voltage/1000.,2);
      }    
      if (mStatus.battery.percent != status.battery.percent){
        bSend = true;
        mStatus.battery.percent = status.battery.percent;
        doc["Battperc"] = status.battery.percent;
      }          
      #ifdef AIRMODULE
      if (setting.Mode == eMode::AIR_MODULE){
        if ((status.gps.Time) && (status.gps.Date)){
          if ((strcmp(&status.gps.Date[0],&mStatus.gps.Date[0])) || (strcmp(&status.gps.Time[0],&mStatus.gps.Time[0]))){
            memcpy(&mStatus.gps.Time[0],&status.gps.Time[0],sizeof(status.gps.Time));
            memcpy(&mStatus.gps.Date[0],&status.gps.Date[0],sizeof(status.gps.Time));
            doc["GPSTime"] = String(status.gps.Date) + "-" + String(status.gps.Time);
          }          
        }
 
        if (mStatus.gps.Fix != status.gps.Fix){
          bSend = true;
          mStatus.gps.Fix = status.gps.Fix;
          doc["gpsFix"] = status.gps.Fix;
        }    
        if (mStatus.gps.NumSat != status.gps.NumSat){
          bSend = true;
          mStatus.gps.NumSat = status.gps.NumSat;
          doc["gpsNumSat"] = status.gps.NumSat;
        }    
        if (mStatus.gps.speed != status.gps.speed){
          bSend = true;
          mStatus.gps.speed = status.gps.speed;
          doc["gpsSpeed"] = String(status.gps.speed,2);
        }    
        if (mStatus.gps.course != status.gps.course){
          bSend = true;
          mStatus.gps.course = status.gps.course;
          doc["gpsCourse"] = String(status.gps.course,2);
        }    
      }
      #endif
      if (mStatus.gps.Lat != status.gps.Lat){
        bSend = true;
        mStatus.gps.Lat = status.gps.Lat;
        doc["gpslat"] = String(status.gps.Lat,6);
      }    
      if (mStatus.gps.Lon != status.gps.Lon){
        bSend = true;
        mStatus.gps.Lon = status.gps.Lon;
        doc["gpslon"] = String(status.gps.Lon,6);
      }    
      if (mStatus.gps.alt != status.gps.alt){
        bSend = true;
        mStatus.gps.alt = status.gps.alt;
        doc["gpsAlt"] = String(status.gps.alt,1);
      }    
      if (mStatus.gps.geoidAlt != status.gps.geoidAlt){
        bSend = true;
        mStatus.gps.geoidAlt = status.gps.geoidAlt;
        doc["gpsGAlt"] = String(status.gps.geoidAlt,1);
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
      
      
      if (flarmFrequ != fanet.getFlarmFrequency()){
        bSend = true;
        flarmFrequ = fanet.getFlarmFrequency();
        doc["legFrequ"] = flarmFrequ;
      }
      if (mStatus.legTx != status.legTx){
        bSend = true;
        mStatus.legTx = status.legTx;
        doc["legTx"] = status.legTx;
      }    
      if (mStatus.legRx != status.legRx){
        bSend = true;
        mStatus.legRx = status.legRx;
        doc["legRx"] = status.legRx;
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
      if (bSend){
        serializeJson(doc, msg_buf);
        webSocket.sendTXT(clientNr, msg_buf);
      }

      doc.clear();
      bSend = false;
      if (mStatus.wifiSTA.Rssi != status.wifiSTA.Rssi){
        bSend = true;
        mStatus.wifiSTA.Rssi = status.wifiSTA.Rssi;
        doc["wifiSTARssi"] = String(status.wifiSTA.Rssi);
      }    
      if (mStatus.wifiSTA.state != status.wifiSTA.state){
        bSend = true;
        mStatus.wifiSTA.state = status.wifiSTA.state;
        doc["wifiSTAState"] = getEthState(status.wifiSTA.state);
      }    
      if (mStatus.wifiSTA.ip != status.wifiSTA.ip){
        bSend = true;
        mStatus.wifiSTA.ip = status.wifiSTA.ip;
        doc["wifiSTAIp"] = status.wifiSTA.ip;
      }
      #ifdef GSM_MODULE
      if (mStatus.gsm.SignalQuality != status.gsm.SignalQuality){
        bSend = true;
        mStatus.gsm.SignalQuality = status.gsm.SignalQuality;
        doc["GSMRssi"] = status.gsm.SignalQuality;
      }    
      if (mStatus.gsm.networkstat != status.gsm.networkstat){
        bSend = true;
        mStatus.gsm.networkstat = status.gsm.networkstat;
        doc["GSMNWSTAT"] = getNetWorkStat(status.gsm.networkstat);
      }
      if (!mStatus.gsm.sOperator.equals(status.gsm.sOperator)){
        bSend = true;
        mStatus.gsm.sOperator = status.gsm.sOperator;
        doc["GSMCOPS"] = status.gsm.sOperator; 
      }      
      if (mStatus.modemstatus != status.modemstatus){
        bSend = true;
        mStatus.modemstatus = status.modemstatus;
        doc["GSMStat"] = getEthState(status.modemstatus);
      }    
      #endif
      if (bSend){
        serializeJson(doc, msg_buf);
        webSocket.sendTXT(clientNr, msg_buf);
      }

      if (setting.bHasFuelSensor){
        doc.clear();
        bSend = false;
        if (mStatus.fuelSensor != status.fuelSensor){
          bSend = true;
          mStatus.fuelSensor = status.fuelSensor;
          doc["fuelValue"] = String(status.fuelSensor,3);
        }        
        if (bSend){
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(clientNr, msg_buf);
        }
      }


      #ifdef GSMODULE
      if ((setting.Mode == eMode::GROUND_STATION) && (setting.wd.mode.bits.enable | status.bWUBroadCast)){
        //weather-data
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
          webSocket.sendTXT(clientNr, msg_buf);
        }
      }
      #endif
      break;
    case 3:
      //page send-messages
      doc.clear();
      bSend = false;
      if (mStatus.FanetMsgCount != status.FanetMsgCount){
        bSend = true;
        mStatus.FanetMsgCount = status.FanetMsgCount;
        doc["FNTMSGIN"] = status.lastFanetMsg;
      }    
      if (bSend){
        serializeJson(doc, msg_buf);
        webSocket.sendTXT(clientNr, msg_buf);
      }
      break;
    case 5:
      //page update
      doc.clear();
      bSend = false;
      if (mStatus.updateState != status.updateState){
        bSend = true;
        mStatus.updateState = status.updateState;
        doc["updateState"] = status.updateState;
        if (status.updateState == 10){
          doc["nVers"] = status.sNewVersion;
        }
      }    
      if (bSend){
        serializeJson(doc, msg_buf);
        webSocket.sendTXT(clientNr, msg_buf);
      }
      break;
    case 10:
      //site fullsettings
      doc.clear();
      bSend = false;
      if (mStatus.battery.voltage != status.battery.voltage){
        bSend = true;
        mStatus.battery.voltage = status.battery.voltage;
        doc["vBatt"] = String((float)status.battery.voltage/1000.,2);
      }    
      if (mCommand.ConfigGPS != command.ConfigGPS){
        bSend = true;
        mCommand.ConfigGPS = command.ConfigGPS;
        doc["configGPS"] = mCommand.ConfigGPS;
      }    
      if (mCommand.getGpsPos != command.getGpsPos){
        bSend = true;
        mCommand.getGpsPos = command.getGpsPos;
        doc["getGpsPos"] = mCommand.getGpsPos;
      }      
      if (mCommand.CalibGyro != command.CalibGyro){
        bSend = true;
        mCommand.CalibGyro = command.CalibGyro;
        doc["calibGyro"] = mCommand.CalibGyro;
      }    
      if (mCommand.CalibAcc != command.CalibAcc){
        bSend = true;
        mCommand.CalibAcc = command.CalibAcc;
        doc["calibAcc"] = mCommand.CalibAcc;
      }    
      if (mStatus.calibAccStat != status.calibAccStat){
        bSend = true;
        mStatus.calibAccStat = status.calibAccStat;
        doc["stateCalibAcc"] = mStatus.calibAccStat;
      }    
      if (bSend){
        serializeJson(doc, msg_buf);
        webSocket.sendTXT(clientNr, msg_buf);
      }
      break;
    case 20:
      if ((tAct - tCount20) >= 1000){ //send every second neighbours !!
        tCount20 = tAct;
        doc.clear();
        uint8_t count = fanet.getNeighboursCount();
        doc["NBCount"] = count;
        serializeJson(doc, msg_buf);
        webSocket.sendTXT(clientNr, msg_buf);
        if (count == 0){
          break; //no Neighbours
        }
        uint8_t iIndex = 0;                  
        for (int i = 0; i < MAXNEIGHBOURS; i++){
          if (fanet.neighbours[i].devId){              
            doc.clear();
            doc["INDEX"] = iIndex;
            doc["ID"] = fanet.getDevId(fanet.neighbours[i].devId);
            doc["LAT"] = String(fanet.neighbours[i].lat,6);
            doc["LON"] = String(fanet.neighbours[i].lon,6);
            doc["NAME"] = (fanet.neighbours[i].name.length() > 0) ? fanet.neighbours[i].name : "";
            doc["TYPE"] = fanet.getAircraftType(fanet.neighbours[i].aircraftType);
            doc["STATE"] = fanet.getType(fanet.neighbours[i].type);
            if ((status.gps.Lat != 0) && (status.gps.Lon != 0 )){
              doc["DIST"] =  String(distance(status.gps.Lat,status.gps.Lon,fanet.neighbours[i].lat,fanet.neighbours[i].lon, 'K'),3);
            }else{
              doc["DIST"] = "";
            }
            doc["ALT"] = String(fanet.neighbours[i].altitude,0);
            doc["SPEED"] = String(fanet.neighbours[i].speed,1);
            doc["CLIMB"] = String(fanet.neighbours[i].climb,1);
            doc["HEAD"] = String(fanet.neighbours[i].heading,0);
            doc["RSSI"] = String(fanet.neighbours[i].rssi);
            if (fanet.neighbours[i].addressType & 0x80){
              doc["BY"] = "1";
            }else{
              doc["BY"] = "2";
            }
            doc["SEEN"] = String((millis() - fanet.neighbours[i].tLastMsg) / 1000);
            serializeJson(doc, msg_buf);
            webSocket.sendTXT(clientNr, msg_buf);
            iIndex++;
          }
        }
      }
      break;
    case 30: //weather-stations
      if ((tAct - tCount30) >= 1000){ //send every second weatherstations !!
        tCount30 = tAct;
        doc.clear();
        uint8_t count = 0;
        for (int i = 0; i < MAXWEATHERDATAS; i++){
          if (fanet.weatherDatas[i].devId){
            count++;
          }
        }
        doc["NBCount"] = count;
        serializeJson(doc, msg_buf);
        webSocket.sendTXT(clientNr, msg_buf);
        if (count == 0){
          break; //no weatherstations
        }
        uint8_t iIndex = 0;                  
        for (int i = 0; i < MAXWEATHERDATAS; i++){
          if (fanet.weatherDatas[i].devId){
            doc.clear();
            doc["INDEX"] = iIndex;
            doc["ID"] = fanet.getDevId(fanet.weatherDatas[i].devId);
            doc["LAT"] = String(fanet.weatherDatas[i].lat,6);
            doc["LON"] = String(fanet.weatherDatas[i].lon,6);
            doc["NAME"] = (fanet.weatherDatas[i].name.length() > 0) ? fanet.weatherDatas[i].name : "";
            if ((status.gps.Lat != 0) && (status.gps.Lon != 0 )){
              doc["DIST"] =  String(distance(status.gps.Lat,status.gps.Lon,fanet.weatherDatas[i].lat,fanet.weatherDatas[i].lon, 'K'),1);
            }else{
              doc["DIST"] = "";
            }
            if (fanet.weatherDatas[i].bTemp == 1 ) {
              doc["T"] = String(fanet.weatherDatas[i].temp,0);
            }else{
              doc["T"] = "";
            }
            if (fanet.weatherDatas[i].bWind == 1 ) {
              doc["WD"] = String(fanet.weatherDatas[i].wHeading,0);
              doc["WS"] = String(fanet.weatherDatas[i].wSpeed,0);
              doc["WG"] = String(fanet.weatherDatas[i].wGust,0);
            }else{
              doc["WD"] = "";
              doc["WS"] = "";
              doc["WG"] = "";
            }
            if (fanet.weatherDatas[i].bHumidity == 1 ) {
              doc["H"] = String(fanet.weatherDatas[i].Humidity,0);
            }else{
              doc["H"] = "";
            }
            if (fanet.weatherDatas[i].bBaro == 1 ) {
              doc["P"] = String(fanet.weatherDatas[i].Baro,0);
            }else{
              doc["P"] = "";
            }
            if (fanet.weatherDatas[i].bStateOfCharge == 1 ) {
              doc["B"] =  String(fanet.weatherDatas[i].Charge,1);
            }else{
              doc["B"] = "";
            }
            doc["RSSI"] =  String(fanet.weatherDatas[i].rssi);
            doc["SEEN"] = String((millis() - fanet.weatherDatas[i].tLastMsg) / 1000);
            serializeJson(doc, msg_buf);
            webSocket.sendTXT(clientNr, msg_buf);
            iIndex++;
          }
        }
     }
      break;
  }
  
}

void Web_loop(void){  
  static uint32_t tLife = millis();
  static uint32_t tRestart = millis();
  uint32_t tAct = millis();
  // Look for and handle WebSocket data
  webSocket.loop();

  if ((tAct - tLife) >= 100){
    tLife = tAct;
    //site update
    for (int i = 0;i <MAXCLIENTS;i++){
      if (clientPages[i] > 0){
        sendPage(clientPages[i],i);
      }
    }
  }
  if (restartNow){
    if ((tAct - tRestart) >= 1000){
      ESP.restart();
    }    
  }else{
    tRestart = tAct;
  }
}