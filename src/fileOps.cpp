#include "fileOps.h"

Preferences preferences;  

void load_configFile(void){
    log_i("LOAD CONFIG FILE");
    preferences.begin("settings", false);                         //Ordner settings anlegen/verwenden
    setting.appw = preferences.getString("APPW","12345678");
    setting.boardType = preferences.getUChar("BOARDTYPE",BOARD_T_BEAM); //
    setting.band = preferences.getUChar("BAND",BAND868); //
    setting.LoraPower = preferences.getUChar("LORA_POWER",10);//
    setting.awLiveTracking = preferences.getUChar("AWLIVE",0); //
    setting.outputLK8EX1 = preferences.getUChar("OLK8EX1",1); //
    setting.outputFLARM = preferences.getUChar("OFLARM",1); //
    setting.outputGPS = preferences.getUChar("OGPS",1); //
    setting.outputFANET = preferences.getUChar("OFANET",1); //
    setting.PilotName = preferences.getString("PILOTNAME","");
    setting.ssid = preferences.getString("WIFI_SSID","");
    setting.password = preferences.getString("WIFI_PW","");
    setting.AircraftType = (FanetLora::aircraft_t)preferences.getUChar("AIRCRAFTTYPE",1);
    setting.bSwitchWifiOff3Min = preferences.getBool("SWOFF3MIN",false);
    setting.UDPServerIP = preferences.getString("UDP_SERVER","192.168.4.2"); //UDP-IP-Adress to match connected device
    setting.UDPSendPort = preferences.getUInt("UDP_PORT",10110); //Port of udp-server
    setting.outputMode = preferences.getUChar("OutputMode",OUTPUT_SERIAL); //output-mode
    setting.testMode = preferences.getUChar("TESTMODE",0); //testmode
    setting.GSMode = preferences.getUChar("GSMode",0);
    setting.GSLAT = preferences.getFloat("GSLAT",0.0);
    setting.GSLON = preferences.getFloat("GSLON",0.0);
    setting.GSAlt = preferences.getFloat("GSALT",0.0);
    setting.GSAWID = preferences.getString("GSAWID","");
    setting.OGNLiveTracking = preferences.getUChar("OGN_LIVE",0);
    setting.screenNumber = preferences.getUChar("SCREEN",0);
    preferences.end(); 
}

void write_configFile(void){
    log_i("WRITE CONFIG FILE");
    preferences.begin("settings", false);                         //Ordner settings anlegen/verwenden
    preferences.putString("APPW",setting.appw);
    preferences.putUChar("BOARDTYPE",setting.boardType); //
    preferences.putUChar("BAND",setting.band); //
    preferences.putUChar("LORA_POWER",setting.LoraPower);//
    preferences.putUChar("AWLIVE",setting.awLiveTracking); //
    preferences.putUChar("OLK8EX1",setting.outputLK8EX1); //
    preferences.putUChar("OFLARM",setting.outputFLARM); //
    preferences.putUChar("OGPS",setting.outputGPS); //
    preferences.putUChar("OFANET",setting.outputFANET); //
    preferences.putString("PILOTNAME",setting.PilotName);
    preferences.putString("WIFI_SSID",setting.ssid);
    preferences.putString("WIFI_PW",setting.password);
    preferences.putUChar("AIRCRAFTTYPE",uint8_t(setting.AircraftType));
    preferences.putBool("SWOFF3MIN",setting.bSwitchWifiOff3Min);    
    preferences.putString("UDP_SERVER",setting.UDPServerIP); //UDP-IP-Adress for sending Pakets
    preferences.putUInt("UDP_PORT",setting.UDPSendPort); //Port of udp-server
    preferences.putUChar("OutputMode",setting.outputMode);
    preferences.putUChar("TESTMODE",setting.testMode);
    preferences.putUChar("GSMode",setting.GSMode);
    preferences.putFloat("GSLAT",setting.GSLAT);
    preferences.putFloat("GSLON",setting.GSLON);
    preferences.putFloat("GSALT",setting.GSAlt);
    preferences.putString("GSAWID",setting.GSAWID);
    preferences.putUChar("OGN_LIVE",setting.OGNLiveTracking);
    ESP.restart();
}

void write_screenNumber(void){
    log_i("WRITE CONFIG FILE");
    preferences.begin("settings", false);
    preferences.putUChar("SCREEN",setting.screenNumber);
    preferences.end();
}