#include <FanetLora.h>


#ifndef __MAIN_H__
#define __MAIN_H__

#define MAIN_FIRMWARE_VERSION "0.9"
#define MAIN_IDENT            "ESP32FANET-"
#define HOSTNAME            "ESP32FANET-"

#define ARDUINO_RUNNING_CORE0 0
#define ARDUINO_RUNNING_CORE1 1

struct SettingsData{
  String myDevId; //my device-ID
  String ssid; //WIFI SSID
  String password; //WIFI PASSWORD
  String PilotName; //Pilotname
  eFanetAircraftType AircraftType; //Aircrafttype
  bool bSwitchWifiOff3Min; //switch off wifi after 3min.
  uint32_t wifiDownTime;
  String UDPServerIP; //UDP-IP-Adress for sending Pakets
  uint16_t UDPSendPort; //Port of udp-server
};




#endif