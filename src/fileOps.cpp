#include "fileOps.h"

Preferences preferences;  

void load_configFile(SettingsData* pSetting){
  log_i("LOAD CONFIG FILE");
  preferences.begin("settings", false);                         //Ordner settings anlegen/verwenden
  pSetting->settingsView = preferences.getUChar("setView",0); //
  pSetting->wifi.appw = preferences.getString("APPW","12345678");
  pSetting->boardType = eBoard(preferences.getUChar("BOARDTYPE",eBoard::UNKNOWN)); //
  pSetting->CPUFrequency = preferences.getUChar("CPUFREQU",240); //
  pSetting->FrqCor = preferences.getLong("FrqCor",0); //
  pSetting->bHasExtPowerSw = preferences.getUChar("EXTPWSW",0); //external power-switch
  pSetting->RFMode = preferences.getUChar("RFM",11); //default FntRx + FntTx + LegTx
  pSetting->awLiveTracking = preferences.getUChar("AWLIVE",0); //
  pSetting->bOutputSerial = preferences.getUChar("OSerial",0); //
  pSetting->outputModeVario = eOutputVario(preferences.getUChar("OVario",eOutputVario::OVARIO_LK8EX1)); //
  pSetting->outputFLARM = preferences.getUChar("OFLARM",1); //
  pSetting->outputGPS = preferences.getUChar("OGPS",1); //
  pSetting->outputFANET = preferences.getUChar("OFANET",1); //
  pSetting->PilotName = preferences.getString("PILOTNAME","");
  pSetting->myDevId = preferences.getString("myDevId",""); 
  pSetting->myDevIdType = preferences.getULong("myDevIdType",2); // Default FLARM
  pSetting->gps.customGPSConfig = preferences.getBool("customGPSConfig",false);
  pSetting->gps.Baud = preferences.getULong("GPSBAud",9600);
  pSetting->wifi.uMode.mode = preferences.getUChar("WIFI_MODE",0); //
  pSetting->wifi.connect = eWifiMode(preferences.getUChar("WIFI_CONNECT",eWifiMode::CONNECT_NONE)); //
  pSetting->wifi.ssid = preferences.getString("WIFI_SSID","");
  pSetting->wifi.password = preferences.getString("WIFI_PW","");
  pSetting->wifi.tWifiStop = preferences.getUInt("Time_WIFI_Stop",180); //stop wifi after 3min.
  pSetting->AircraftType = preferences.getUChar("AIRCRAFTTYPE",1);
  pSetting->UDPServerIP = preferences.getString("UDP_SERVER","192.168.4.2"); //UDP-IP-Adress to match connected device
  pSetting->UDPSendPort = preferences.getUInt("UDP_PORT",10110); //Port of udp-server
  pSetting->outputMode = eOutput(preferences.getUChar("OutputMode",eOutput::oBLE)); //output-mode default ble
  pSetting->Mode = eMode(preferences.getUChar("Mode",eMode::AIR_MODULE));
  pSetting->fanetMode = eFnMode(preferences.getUChar("fntMode",eFnMode::FN_GROUNT_AIR_TRACKING));  
  pSetting->fanetpin = preferences.getUInt("fntPin",0000);
  pSetting->bAutoupdate = preferences.getUChar("AUTOUPDATE",1); //auto-update
  
  //gs settings
  pSetting->gs.lat = preferences.getFloat("GSLAT",0.0);
  pSetting->gs.lon = preferences.getFloat("GSLON",0.0);
  pSetting->gs.alt = preferences.getFloat("GSALT",0.0);
  pSetting->gs.geoidAlt = preferences.getFloat("GSGEOALT",0.0);

  pSetting->FntWuUpload[0].FanetId = preferences.getULong("F2WuF0",0);
  pSetting->FntWuUpload[1].FanetId = preferences.getULong("F2WuF1",0);
  pSetting->FntWuUpload[2].FanetId = preferences.getULong("F2WuF2",0);
  pSetting->FntWuUpload[3].FanetId = preferences.getULong("F2WuF3",0);
  pSetting->FntWuUpload[4].FanetId = preferences.getULong("F2WuF4",0);
  pSetting->FntWuUpload[0].ID = preferences.getString("F2WuI0","");
  pSetting->FntWuUpload[1].ID = preferences.getString("F2WuI1","");
  pSetting->FntWuUpload[2].ID = preferences.getString("F2WuI2","");
  pSetting->FntWuUpload[3].ID = preferences.getString("F2WuI3","");
  pSetting->FntWuUpload[4].ID = preferences.getString("F2WuI4","");
  pSetting->FntWuUpload[0].KEY = preferences.getString("F2WuK0","");
  pSetting->FntWuUpload[1].KEY = preferences.getString("F2WuK1","");
  pSetting->FntWuUpload[2].KEY = preferences.getString("F2WuK2","");
  pSetting->FntWuUpload[3].KEY = preferences.getString("F2WuK3","");
  pSetting->FntWuUpload[4].KEY = preferences.getString("F2WuK4","");
  pSetting->FntWiUpload[0].FanetId = preferences.getULong("F2WiF0",0);
  pSetting->FntWiUpload[1].FanetId = preferences.getULong("F2WiF1",0);
  pSetting->FntWiUpload[2].FanetId = preferences.getULong("F2WiF2",0);
  pSetting->FntWiUpload[3].FanetId = preferences.getULong("F2WiF3",0);
  pSetting->FntWiUpload[4].FanetId = preferences.getULong("F2WiF4",0);
  pSetting->FntWiUpload[0].ID = preferences.getString("F2WiI0","");
  pSetting->FntWiUpload[1].ID = preferences.getString("F2WiI1","");
  pSetting->FntWiUpload[2].ID = preferences.getString("F2WiI2","");
  pSetting->FntWiUpload[3].ID = preferences.getString("F2WiI3","");
  pSetting->FntWiUpload[4].ID = preferences.getString("F2WiI4","");
  pSetting->FntWiUpload[0].KEY = preferences.getString("F2WiK0","");
  pSetting->FntWiUpload[1].KEY = preferences.getString("F2WiK1","");
  pSetting->FntWiUpload[2].KEY = preferences.getString("F2WiK2","");
  pSetting->FntWiUpload[3].KEY = preferences.getString("F2WiK3","");
  pSetting->FntWiUpload[4].KEY = preferences.getString("F2WiK4","");
  pSetting->gs.SreenOption = eScreenOption(preferences.getUChar("GSSCR",eScreenOption::ALWAYS_ON));
  pSetting->gs.PowerSave = eGsPower(preferences.getUChar("GSPS",eGsPower::GS_POWER_ALWAYS_ON));
  pSetting->gs.sunriseOffset = eGsPower(preferences.getInt("GsSrO",0));
  pSetting->gs.sunsetOffset = eGsPower(preferences.getInt("GsSsO",0));
  pSetting->wd.anemometer.AnemometerType = eAnemometer(preferences.getUChar("GSANEO",eAnemometer::DAVIS));
  pSetting->wd.anemometer.AnemometerAdsGain = preferences.getUChar("WDANEOADSGAIN",2);;
  pSetting->wd.anemometer.AnemometerAdsWSpeedMinVoltage = preferences.getFloat("WDANEOADSWSMINV",0.0);
  pSetting->wd.anemometer.AnemometerAdsWSpeedMaxVoltage = preferences.getFloat("WDANEOADSWSMAXV",10.0);
  pSetting->wd.anemometer.AnemometerAdsWDirMinVoltage = preferences.getFloat("WDANEOADSWDMINV",0.0);
  pSetting->wd.anemometer.AnemometerAdsWDirMaxVoltage = preferences.getFloat("WDANEOADSWDMAXV",10.0);
  pSetting->wd.anemometer.AnemometerAdsWSpeedMinSpeed = preferences.getFloat("WDANEOADSWSMINS",0.0);
  pSetting->wd.anemometer.AnemometerAdsWSpeedMaxSpeed = preferences.getFloat("WDANEOADSWSMAXS",120.0);
  pSetting->wd.anemometer.AnemometerAdsWDirMinDir = preferences.getFloat("WDANEOADSWDMIND",0.0);
  pSetting->wd.anemometer.AnemometerAdsWDirMaxDir = preferences.getFloat("WDANEOADSWDMAXD",360.0);
  pSetting->wd.anemometer.AnemometerAdsVDivR1 = preferences.getFloat("WDANEOADSVDIVR1",8060000.0);
  pSetting->wd.anemometer.AnemometerAdsVDivR2 = preferences.getFloat("WDANEOADSVDIVR2",402000.0);
  pSetting->BattVoltOffs = preferences.getFloat("BATOFFS",0.0);
  pSetting->minBattPercent = preferences.getUChar("BattMinPerc",20);
  pSetting->restartBattPercent = preferences.getUChar("restartBattPerc",20);

  //live-tracking
  pSetting->OGNLiveTracking.mode = preferences.getUChar("OGN_LIVE",0);
  pSetting->screenNumber = preferences.getUChar("SCREEN",0);
  pSetting->displayType = eDisplay(preferences.getUChar("Display",eDisplay::NO_DISPLAY));
  pSetting->displayRotation = preferences.getUChar("DispRot",0);
  pSetting->traccarLiveTracking = preferences.getUChar("TRACCAR_LIVE",0);
  pSetting->TraccarSrv = preferences.getString("TRACCAR_SRV","");
  
  //weathersettings
  pSetting->wd.mode.mode = preferences.getUChar("WsMode",0);
  pSetting->wd.sendFanet = preferences.getUChar("FanetWeather",1);
  pSetting->wd.tempOffset = preferences.getFloat("wdTempOffset",0.0);
  pSetting->wd.windDirOffset = preferences.getInt("wdWDirOffset",0);
  pSetting->wd.avgFactorFanet = preferences.getFloat("avgFanet",16);
  pSetting->wd.FanetUploadInterval = preferences.getULong("FanetWDInt",40000);
  pSetting->wd.avgFactorWU = preferences.getFloat("avgWU",128);
  pSetting->wd.WUUploadIntervall = preferences.getULong("WUIntervall",300000);


  //vario
  pSetting->vario.sinkingThreshold = preferences.getFloat("vSinkTh",-2.5);
  pSetting->vario.climbingThreshold = preferences.getFloat("vClimbTh",0.2);
  pSetting->vario.nearClimbingSensitivity = preferences.getFloat("vNClimbSens",0.2);
  pSetting->vario.volume = preferences.getUChar("VarioVolume",127); //full duty-cycle
  pSetting->vario.BeepOnlyWhenFlying = preferences.getUChar("VBeepFlying",1);
  pSetting->vario.useMPU = preferences.getUChar("useMPU",0);
  pSetting->vario.tempOffset = preferences.getFloat("vTOffs",0.0);
  pSetting->vario.sigmaP = preferences.getFloat("vSigmaP",0.1);
  pSetting->vario.sigmaA = preferences.getFloat("vSigmaA",0.6);

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
  pSetting->gsm.NetworkMode = eGsmNetworkMode(preferences.getUChar("GSMMODE",eGsmNetworkMode::GSM_NW_AUTO));
  pSetting->gsm.PreferredMode = eGsmPreferedMode(preferences.getUChar("GSMPREF",eGsmPreferedMode::GSM_PREF_NOT_SET));
  pSetting->gsm.NB_IOT_Band = preferences.getString("NBIOT","");
  pSetting->gsm.CAT_M_Band = preferences.getString("CATM","");
  //fuel-sensor
  pSetting->bHasFuelSensor = preferences.getUChar("fuelSensor",0);

  //mqtt
  pSetting->mqtt.mode.mode = preferences.getUChar("MqttEn",0);
  pSetting->mqtt.server = preferences.getString("MqttServer","");
  pSetting->mqtt.port = preferences.getInt("MqttPort",1883);
  pSetting->mqtt.pw = preferences.getString("MqttPw","");

  preferences.end(); 

  //vario
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
  preferences.putUChar("setView",pSetting->settingsView); //
  preferences.putString("APPW",pSetting->wifi.appw);
  preferences.putUChar("BOARDTYPE",pSetting->boardType); //
  preferences.putUChar("CPUFREQU",pSetting->CPUFrequency); //
  preferences.putLong("FrqCor",pSetting->FrqCor);
  preferences.putUChar("EXTPWSW",pSetting->bHasExtPowerSw); //
  preferences.putUChar("RFM",pSetting->RFMode);
  preferences.putUChar("AWLIVE",pSetting->awLiveTracking); //
  preferences.putUChar("OSerial",pSetting->bOutputSerial); //
  preferences.putUChar("OVario",pSetting->outputModeVario); //
  preferences.putUChar("OFLARM",pSetting->outputFLARM); //
  preferences.putUChar("OGPS",pSetting->outputGPS); //
  preferences.putUChar("OFANET",pSetting->outputFANET); //
  preferences.putString("PILOTNAME",pSetting->PilotName);
  preferences.putString("myDevId",pSetting->myDevId);
  preferences.putULong("myDevIdType",pSetting->myDevIdType);
  preferences.putBool("customGPSConfig",pSetting->gps.customGPSConfig);
  preferences.putUChar("WIFI_MODE",pSetting->wifi.uMode.mode); //
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
  preferences.putUChar("AUTOUPDATE",pSetting->bAutoupdate); //auto-update

  //GS Settings
  preferences.putFloat("GSLAT",pSetting->gs.lat);
  preferences.putFloat("GSLON",pSetting->gs.lon);
  preferences.putFloat("GSALT",pSetting->gs.alt);
  preferences.putFloat("GSGEOALT",pSetting->gs.geoidAlt);
  preferences.putUChar("GSSCR",pSetting->gs.SreenOption);
  preferences.putUChar("GSPS",pSetting->gs.PowerSave);
  preferences.putInt("GsSrO",pSetting->gs.sunriseOffset);
  preferences.putInt("GsSsO",pSetting->gs.sunsetOffset);
  preferences.putUChar("GSANEO",pSetting->wd.anemometer.AnemometerType);
  preferences.putUChar("WDANEOADSGAIN",pSetting->wd.anemometer.AnemometerAdsGain);
  preferences.putFloat("WDANEOADSVDIVR1",pSetting->wd.anemometer.AnemometerAdsVDivR1);
  preferences.putFloat("WDANEOADSVDIVR2",pSetting->wd.anemometer.AnemometerAdsVDivR2);
  preferences.putFloat("WDANEOADSWSMINV", pSetting->wd.anemometer.AnemometerAdsWSpeedMinVoltage);
  preferences.putFloat("WDANEOADSWSMAXV", pSetting->wd.anemometer.AnemometerAdsWSpeedMaxVoltage);
  preferences.putFloat("WDANEOADSWDMINV", pSetting->wd.anemometer.AnemometerAdsWDirMinVoltage);
  preferences.putFloat("WDANEOADSWDMAXV", pSetting->wd.anemometer.AnemometerAdsWDirMaxVoltage);
  preferences.putFloat("WDANEOADSWSMINS", pSetting->wd.anemometer.AnemometerAdsWSpeedMinSpeed);
  preferences.putFloat("WDANEOADSWSMAXS", pSetting->wd.anemometer.AnemometerAdsWSpeedMaxSpeed);
  preferences.putFloat("WDANEOADSWDMIND", pSetting->wd.anemometer.AnemometerAdsWDirMinDir);
  preferences.putFloat("WDANEOADSWDMAXD", pSetting->wd.anemometer.AnemometerAdsWDirMaxDir);
  preferences.putUChar("BattMinPerc",pSetting->minBattPercent);
  preferences.putUChar("restartBattPerc",pSetting->restartBattPercent);
  
  preferences.putULong("F2WuF0",pSetting->FntWuUpload[0].FanetId);
  preferences.putULong("F2WuF1",pSetting->FntWuUpload[1].FanetId);
  preferences.putULong("F2WuF2",pSetting->FntWuUpload[2].FanetId);
  preferences.putULong("F2WuF3",pSetting->FntWuUpload[3].FanetId);
  preferences.putULong("F2WuF4",pSetting->FntWuUpload[4].FanetId);
  preferences.putString("F2WuI0",pSetting->FntWuUpload[0].ID);
  preferences.putString("F2WuI1",pSetting->FntWuUpload[1].ID);
  preferences.putString("F2WuI2",pSetting->FntWuUpload[2].ID);
  preferences.putString("F2WuI3",pSetting->FntWuUpload[3].ID);
  preferences.putString("F2WuI4",pSetting->FntWuUpload[4].ID);
  preferences.putString("F2WuK0",pSetting->FntWuUpload[0].KEY);
  preferences.putString("F2WuK1",pSetting->FntWuUpload[1].KEY);
  preferences.putString("F2WuK2",pSetting->FntWuUpload[2].KEY);
  preferences.putString("F2WuK3",pSetting->FntWuUpload[3].KEY);
  preferences.putString("F2WuK4",pSetting->FntWuUpload[4].KEY);
  preferences.putULong("F2WiF0",pSetting->FntWiUpload[0].FanetId);
  preferences.putULong("F2WiF1",pSetting->FntWiUpload[1].FanetId);
  preferences.putULong("F2WiF2",pSetting->FntWiUpload[2].FanetId);
  preferences.putULong("F2WiF3",pSetting->FntWiUpload[3].FanetId);
  preferences.putULong("F2WiF4",pSetting->FntWiUpload[4].FanetId);
  preferences.putString("F2WiI0",pSetting->FntWiUpload[0].ID);
  preferences.putString("F2WiI1",pSetting->FntWiUpload[1].ID);
  preferences.putString("F2WiI2",pSetting->FntWiUpload[2].ID);
  preferences.putString("F2WiI3",pSetting->FntWiUpload[3].ID);
  preferences.putString("F2WiI4",pSetting->FntWiUpload[4].ID);
  preferences.putString("F2WiK0",pSetting->FntWiUpload[0].KEY);
  preferences.putString("F2WiK1",pSetting->FntWiUpload[1].KEY);
  preferences.putString("F2WiK2",pSetting->FntWiUpload[2].KEY);
  preferences.putString("F2WiK3",pSetting->FntWiUpload[3].KEY);
  preferences.putString("F2WiK4",pSetting->FntWiUpload[4].KEY);
  //live-tracking
  preferences.putUChar("OGN_LIVE",pSetting->OGNLiveTracking.mode);
  preferences.putUChar("Display",pSetting->displayType);
  preferences.putUChar("DispRot",pSetting->displayRotation);
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
  preferences.putFloat("vSigmaP",pSetting->vario.sigmaP);
  preferences.putFloat("vSigmaA",pSetting->vario.sigmaA);

  //weathersettings
  preferences.putUChar("WsMode",pSetting->wd.mode.mode);
  preferences.putUChar("FanetWeather",pSetting->wd.sendFanet);
  preferences.putFloat("wdTempOffset",pSetting->wd.tempOffset);
  preferences.putInt("wdWDirOffset",pSetting->wd.windDirOffset);
  preferences.putFloat("avgFanet",pSetting->wd.avgFactorFanet);
  preferences.putULong("FanetWDInt",pSetting->wd.FanetUploadInterval);
  preferences.putFloat("avgWU",pSetting->wd.avgFactorWU);
  preferences.putULong("WUIntervall",pSetting->wd.WUUploadIntervall);

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
  preferences.putUChar("GSMMODE",pSetting->gsm.NetworkMode);
  preferences.putUChar("GSMPREF",pSetting->gsm.PreferredMode);
  preferences.putString("NBIOT",pSetting->gsm.NB_IOT_Band);
  preferences.putString("CATM",pSetting->gsm.CAT_M_Band);
  //fuel-sensor
  preferences.putUChar("fuelSensor",pSetting->bHasFuelSensor);

  //mqtt
  preferences.putUChar("MqttEn",pSetting->mqtt.mode.mode);
  preferences.putString("MqttServer",pSetting->mqtt.server);
  preferences.putInt("MqttPort",pSetting->mqtt.port);
  preferences.putString("MqttPw",pSetting->mqtt.pw);

  preferences.end();

  //vario
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

void write_PilotName(void){
  preferences.begin("settings", false);
  preferences.putString("PILOTNAME",setting.PilotName);
  preferences.end();
}
void write_AircraftType(void){
  preferences.begin("settings", false);
  preferences.putUChar("AIRCRAFTTYPE",uint8_t(setting.AircraftType));
  preferences.end();
}

void write_AirMode(void){
  preferences.begin("settings", false);
  preferences.putUChar("fntMode",setting.fanetMode);
  preferences.end();
}

void write_Mode(void){
  preferences.begin("settings", false);
  preferences.putUChar("Mode",setting.Mode);
  preferences.end();
}

void write_RFMode(void){
  preferences.begin("settings", false);
  preferences.putUChar("RFM",setting.RFMode);
  preferences.end();
}

void write_OutputMode(void){
  preferences.begin("settings", false);
  preferences.putUChar("OutputMode",setting.outputMode);
  preferences.end();
}

void write_fuelsensor(void){
  preferences.begin("settings", false);
  preferences.putUChar("fuelSensor",setting.bHasFuelSensor);
  preferences.end();
}

void write_battOffset(void){
  preferences.begin("settings", false);
  preferences.putFloat("BATOFFS",setting.BattVoltOffs);
  preferences.end();
}

void write_battMinPerc(void){
  preferences.begin("settings", false);
  preferences.putUChar("BattMinPerc",setting.minBattPercent);
  preferences.end();
}

void write_gpsBaud(void){
  log_i("write new GPS-Baudrate=%d",setting.gps.Baud);
  preferences.begin("settings", false);
  preferences.putULong("GPSBAud",setting.gps.Baud);
  preferences.end();
}

void write_wifiModeBits(void){
  log_i("write new wifi mode bits=%d",setting.wifi.uMode.mode);
  preferences.begin("settings", false);
  preferences.putUChar("WIFI_MODE",setting.wifi.uMode.mode);
  preferences.end(); 
}

void write_CPUFrequency(void){
  preferences.begin("settings", false);
  preferences.putUChar("CPUFREQU",setting.CPUFrequency); //
  preferences.end(); 
}
