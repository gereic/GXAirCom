#include "fileOps.h"

Preferences preferences;  

void load_configFile(SettingsData* pSetting){
  log_i("LOAD CONFIG FILE");
  preferences.begin("settings", false);                         //Ordner settings anlegen/verwenden
  pSetting->wifi.appw = preferences.getString("APPW","12345678");
  pSetting->boardType = preferences.getUChar("BOARDTYPE",BOARD_T_BEAM); //
  pSetting->band = preferences.getUChar("BAND",BAND868); //
  pSetting->LoraPower = preferences.getUChar("LORA_POWER",10);//
  pSetting->awLiveTracking = preferences.getUChar("AWLIVE",0); //
  pSetting->bOutputSerial = preferences.getUChar("OSerial",0); //
  pSetting->outputLK8EX1 = preferences.getUChar("OLK8EX1",1); //
  pSetting->outputFLARM = preferences.getUChar("OFLARM",1); //
  pSetting->outputGPS = preferences.getUChar("OGPS",1); //
  pSetting->outputFANET = preferences.getUChar("OFANET",1); //
  pSetting->PilotName = preferences.getString("PILOTNAME","");
  pSetting->wifi.connect = preferences.getUChar("WIFI_CONNECT",0); //
  pSetting->wifi.ssid = preferences.getString("WIFI_SSID","");
  pSetting->wifi.password = preferences.getString("WIFI_PW","");
  pSetting->wifi.tWifiStop = preferences.getUInt("Time_WIFI_Stop",0);
  pSetting->AircraftType = (FanetLora::aircraft_t)preferences.getUChar("AIRCRAFTTYPE",1);
  pSetting->UDPServerIP = preferences.getString("UDP_SERVER","192.168.4.2"); //UDP-IP-Adress to match connected device
  pSetting->UDPSendPort = preferences.getUInt("UDP_PORT",10110); //Port of udp-server
  pSetting->outputMode = preferences.getUChar("OutputMode",OUTPUT_SERIAL); //output-mode
  pSetting->Mode = preferences.getUChar("Mode",0);
  pSetting->fanetMode = preferences.getUChar("fntMode",0);  
  pSetting->fanetpin = preferences.getUInt("fntPin",1234);
  
  //gs settings
  pSetting->gs.lat = preferences.getFloat("GSLAT",0.0);
  pSetting->gs.lon = preferences.getFloat("GSLON",0.0);
  pSetting->gs.alt = preferences.getFloat("GSALT",0.0);
  pSetting->gs.SreenOption = preferences.getUChar("GSSCR",0);
  pSetting->gs.PowerSave = preferences.getUChar("GSPS",0);

  //live-tracking
  pSetting->OGNLiveTracking = preferences.getUChar("OGN_LIVE",0);
  pSetting->screenNumber = preferences.getUChar("SCREEN",0);
  pSetting->displayType = preferences.getUChar("Display",0);
  pSetting->LegacyTxEnable = preferences.getUChar("LEGACY_TX",0);
  pSetting->traccarLiveTracking = preferences.getUChar("TRACCAR_LIVE",0);
  pSetting->TraccarSrv = preferences.getString("TRACCAR_SRV","");
  
  //weathersettings
  pSetting->wd.sendFanet = preferences.getUChar("FanetWeather",0);
  pSetting->wd.tempOffset = preferences.getFloat("wdTempOffset",0.0);
  pSetting->wd.windDirOffset = preferences.getInt("wdWDirOffset",0);


  //vario
  pSetting->vario.sinkingThreshold = preferences.getFloat("vSinkTh",-2.5);
  pSetting->vario.climbingThreshold = preferences.getFloat("vClimbTh",0.2);
  pSetting->vario.nearClimbingSensitivity = preferences.getFloat("vNClimbSens",0.2);
  pSetting->vario.volume = preferences.getUChar("VarioVolume",127); //full duty-cycle
  pSetting->vario.BeepOnlyWhenFlying = preferences.getUChar("VBeepFlying",1);
  pSetting->vario.useMPU = preferences.getUChar("useMPU",0);
  pSetting->vario.tempOffset = preferences.getFloat("vTOffs",0.0);

  //wu-upload
  pSetting->WUUpload.enable = preferences.getUChar("WUUlEnable",0);
  pSetting->WUUpload.ID = preferences.getString("WUUlID","");
  pSetting->WUUpload.KEY = preferences.getString("WUUlKEY","");

  //windy-upload
  pSetting->WindyUpload.enable = preferences.getUChar("WIUlEnable",0);
  pSetting->WindyUpload.ID = preferences.getString("WIUlID","");
  pSetting->WindyUpload.KEY = preferences.getString("WIUlKEY","");

  //gsm
  pSetting->gsm.apn = preferences.getString("GSMAPN","");
  pSetting->gsm.user = preferences.getString("GSMUSER","");
  pSetting->gsm.pwd = preferences.getString("GSMKEY","");

  preferences.end(); 

  preferences.begin("fastvario", false);
  pSetting->vario.accel[0] = preferences.getInt("axOffset", 0);
  pSetting->vario.accel[1] = preferences.getInt("ayOffset", 0);
  pSetting->vario.accel[2] = preferences.getInt("azOffset", 0);
  pSetting->vario.gyro[0] = preferences.getInt("gxOffset", 0);
  pSetting->vario.gyro[1] = preferences.getInt("gyOffset", 0);
  pSetting->vario.gyro[2] = preferences.getInt("gzOffset", 0);
  pSetting->vario.tValues[0] = preferences.getFloat("t[0]",20.0);
  pSetting->vario.tValues[1] = preferences.getFloat("t[1]",0.0);
  pSetting->vario.zValues[0] = preferences.getFloat("z[0]",0.0);
  pSetting->vario.zValues[1] = preferences.getFloat("z[1]",0.0);
  preferences.end();

}

void write_configFile(SettingsData* pSetting){
  log_i("WRITE CONFIG FILE");
  preferences.begin("settings", false);                         //Ordner settings anlegen/verwenden
  preferences.putString("APPW",pSetting->wifi.appw);
  preferences.putUChar("BOARDTYPE",pSetting->boardType); //
  preferences.putUChar("BAND",pSetting->band); //
  preferences.putUChar("LORA_POWER",pSetting->LoraPower);//
  preferences.putUChar("AWLIVE",pSetting->awLiveTracking); //
  preferences.putUChar("OSerial",pSetting->bOutputSerial); //
  preferences.putUChar("OLK8EX1",pSetting->outputLK8EX1); //
  preferences.putUChar("OFLARM",pSetting->outputFLARM); //
  preferences.putUChar("OGPS",pSetting->outputGPS); //
  preferences.putUChar("OFANET",pSetting->outputFANET); //
  preferences.putString("PILOTNAME",pSetting->PilotName);
  preferences.putUChar("WIFI_CONNECT",pSetting->wifi.connect); //
  preferences.putString("WIFI_SSID",pSetting->wifi.ssid);
  preferences.putString("WIFI_PW",pSetting->wifi.password);
  preferences.putUChar("AIRCRAFTTYPE",uint8_t(pSetting->AircraftType));
  preferences.putUInt("Time_WIFI_Stop",pSetting->wifi.tWifiStop);    
  preferences.putString("UDP_SERVER",pSetting->UDPServerIP); //UDP-IP-Adress for sending Pakets
  preferences.putUInt("UDP_PORT",pSetting->UDPSendPort); //Port of udp-server
  preferences.putUChar("OutputMode",pSetting->outputMode);
  preferences.putUChar("Mode",pSetting->Mode);
  preferences.putUChar("fntMode",pSetting->fanetMode);
  preferences.putUInt("fntPin",pSetting->fanetpin);

  //GS Settings
  preferences.putFloat("GSLAT",pSetting->gs.lat);
  preferences.putFloat("GSLON",pSetting->gs.lon);
  preferences.putFloat("GSALT",pSetting->gs.alt);
  preferences.putUChar("GSSCR",pSetting->gs.SreenOption);
  preferences.putUChar("GSPS",pSetting->gs.PowerSave);

  //live-tracking
  preferences.putUChar("OGN_LIVE",pSetting->OGNLiveTracking);
  preferences.putUChar("Display",pSetting->displayType);
  preferences.putUChar("LEGACY_TX",pSetting->LegacyTxEnable);
  preferences.putUChar("TRACCAR_LIVE",pSetting->traccarLiveTracking);
  preferences.putString("TRACCAR_SRV",pSetting->TraccarSrv);
  

  //vario
  preferences.putFloat("vSinkTh",pSetting->vario.sinkingThreshold);
  preferences.putFloat("vClimbTh",pSetting->vario.climbingThreshold);
  preferences.putFloat("vNClimbSens",pSetting->vario.nearClimbingSensitivity);
  preferences.putUChar("VarioVolume",pSetting->vario.volume);
  preferences.putUChar("VBeepFlying",pSetting->vario.BeepOnlyWhenFlying);
  preferences.putUChar("useMPU",pSetting->vario.useMPU);
  preferences.putFloat("vTOffs",pSetting->vario.tempOffset);

  //weathersettings
  preferences.putUChar("FanetWeather",pSetting->wd.sendFanet);
  preferences.putFloat("wdTempOffset",pSetting->wd.tempOffset);
  preferences.putInt("wdWDirOffset",pSetting->wd.windDirOffset);
  
  //wu-upload
  preferences.putUChar("WUUlEnable",pSetting->WUUpload.enable);
  preferences.putString("WUUlID",pSetting->WUUpload.ID);
  preferences.putString("WUUlKEY",pSetting->WUUpload.KEY);

  //windy-upload
  preferences.putUChar("WIUlEnable",pSetting->WindyUpload.enable);
  preferences.putString("WIUlID",pSetting->WindyUpload.ID);
  preferences.putString("WIUlKEY",pSetting->WindyUpload.KEY);

  //gsm
  preferences.putString("GSMAPN",pSetting->gsm.apn);
  preferences.putString("GSMUSER",pSetting->gsm.user);
  preferences.putString("GSMKEY",pSetting->gsm.pwd);
  preferences.end();

  preferences.begin("fastvario", false);
  if ((pSetting->vario.accel[0] != 0) || (pSetting->vario.accel[1] != 0) || (pSetting->vario.accel[2] != 0)){
    preferences.putInt("axOffset", pSetting->vario.accel[0]);
    preferences.putInt("ayOffset", pSetting->vario.accel[1]);
    preferences.putInt("azOffset", pSetting->vario.accel[2]);
  }
  if ((pSetting->vario.gyro[0] != 0) || (pSetting->vario.gyro[1] != 0) || (pSetting->vario.gyro[2] != 0)){
    preferences.putInt("gxOffset", pSetting->vario.gyro[0]);
    preferences.putInt("gyOffset", pSetting->vario.gyro[1]);
    preferences.putInt("gzOffset", pSetting->vario.gyro[2]);
  }
  preferences.putFloat("t[0]",pSetting->vario.tValues[0]);
  preferences.putFloat("t[1]",pSetting->vario.tValues[1]);
  preferences.putFloat("z[0]",pSetting->vario.zValues[0]);
  preferences.putFloat("z[1]",pSetting->vario.zValues[1]);
  preferences.end();

}

void write_screenNumber(void){
  //log_i("WRITE CONFIG FILE");
  preferences.begin("settings", false);
  preferences.putUChar("SCREEN",setting.screenNumber);
  preferences.end();
}

void write_Volume(void){
  //log_i("WRITE vario volume");
  preferences.begin("settings", false);
  preferences.putUChar("VarioVolume",setting.vario.volume);
  preferences.end();
}