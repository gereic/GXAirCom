#include "fileOps.h"

Preferences preferences;  

void load_configFile(void){
    log_i("LOAD CONFIG FILE");
    preferences.begin("settings", false);                         //Ordner settings anlegen/verwenden
    setting.band = preferences.getUChar("BAND",BAND868); //
    setting.outputLK8EX1 = preferences.getUChar("OLK8EX1",1); //
    setting.outputFLARM = preferences.getUChar("OFLARM",1); //
    setting.outputGPS = preferences.getUChar("OGPS",1); //
    setting.outputFANET = preferences.getUChar("OFANET",1); //
    setting.PilotName = preferences.getString("PILOTNAME","");
    setting.ssid = preferences.getString("WIFI_SSID","");
    setting.password = preferences.getString("WIFI_PW","");
    setting.AircraftType = (eFanetAircraftType)preferences.getUChar("AIRCRAFTTYPE",1);
    setting.bSwitchWifiOff3Min = preferences.getBool("SWOFF3MIN",false);
    setting.UDPServerIP = preferences.getString("UDP_SERVER",""); //UDP-IP-Adress for sending Pakets
    setting.UDPSendPort = preferences.getUInt("UDP_PORT",10110); //Port of udp-server
    setting.outputMode = preferences.getUChar("OutputMode",OUTPUT_SERIAL); //output-mode
    setting.testMode = preferences.getUChar("TESTMODE",0); //testmode
    preferences.end(); 
}

void write_configFile(void){
    log_i("WRITE CONFIG FILE");
    preferences.begin("settings", false);                         //Ordner settings anlegen/verwenden
    preferences.putUChar("BAND",setting.band); //
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
    preferences.end(); 
    ESP.restart();
}