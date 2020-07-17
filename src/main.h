#include <FanetLora.h>
#include <string.h>


#ifndef __MAIN_H__
#define __MAIN_H__

#define VERSION "1.0.1"
#define APPNAME "GXAirCom"

#define BLE_LOW_HEAP 10000
#define MAX_BLE_LOW_HEAP_TIME 30000


#define ARDUINO_RUNNING_CORE0 0
#define ARDUINO_RUNNING_CORE1 1

#define OUTPUT_SERIAL 0
#define OUTPUT_UDP 1
#define OUTPUT_BLUETOOTH 2
#define OUTPUT_BLE 3

#define BOARD_T_BEAM 0
#define BOARD_HELTEC_LORA 1

#define BAND868 0
#define BAND915 1

struct SettingsData{
  String appw; //access-point-Password
  uint8_t boardType;
  String myDevId; //my device-ID
  uint8_t band;
  uint8_t outputLK8EX1;
  uint8_t outputFLARM;
  uint8_t outputGPS;
  uint8_t outputFANET;
  uint8_t awLiveTracking; //airwhere live-tracking
  String ssid; //WIFI SSID
  String password; //WIFI PASSWORD
  String PilotName; //Pilotname
  eFanetAircraftType AircraftType; //Aircrafttype
  bool bSwitchWifiOff3Min; //switch off wifi after 3min.
  uint32_t wifiDownTime;
  String UDPServerIP; //UDP-IP-Adress for sending Pakets
  uint16_t UDPSendPort; //Port of udp-server
  uint8_t outputMode; //output-mode
  uint8_t testMode;
  uint8_t GSMode; //ground-station-mode
  float GSLAT; //Ground-Station Latitude
  float GSLON; //Ground-Station Longitude
  float GSAlt; //Ground-Station altitude
  String GSAWID; //Ground-Station ID
};

struct statusData{
  String myIP; //my IP-Adress
  float vBatt; //battery-voltage
  uint8_t GPS_Fix;
  double GPS_Lat;
  double GPS_Lon;
  float GPS_alt;
  float GPS_speed;
  float GPS_course;
  uint8_t GPS_NumSat;
  float ClimbRate;
  uint16_t fanetTx;
  uint16_t fanetRx;
  bool bHasAXP192;
  uint32_t tGPSCycle;
  uint32_t tLoop; //current Loop-time
  uint32_t tMaxLoop; //max Loop-time
};

#endif