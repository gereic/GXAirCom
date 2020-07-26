#include <FanetLora.h>
#include <string.h>


#ifndef __MAIN_H__
#define __MAIN_H__

#define VERSION "V1.2.1"
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
#define BOARD_T_BEAM_V07 2

#define BAND868 0
#define BAND915 1

#define BATTFULL 4050
#define BATTEMPTY 3400

#define MAXSCREENS 2

#define RADAR_CLOSEST 0
#define RADAR_LIST 1
#define RADAR_FRIENDS 2

#define MIN_FLIGHT_SPEED 15.0 //min speed for flying-detection 
// > at least for 10sec --> takeoff
// < at least for 60sec --> landing
#define MIN_FLIGHT_TIME 10000
#define MIN_GROUND_TIME 60000

#define BUTTON2 38

#define DISPLAY_UPDATE_RATE 500

#define FLARM_UPDATE_RATE 1000
#define FLARM_UPDATE_STATE 60000

struct SettingsData{
  String appw; //access-point-Password
  uint8_t boardType;
  String myDevId; //my device-ID
  uint8_t band;
  uint8_t LoraPower; //output-Power 5-20db
  uint8_t outputLK8EX1;
  uint8_t outputFLARM;
  uint8_t outputGPS;
  uint8_t outputFANET;
  uint8_t awLiveTracking; //airwhere live-tracking
  String ssid; //WIFI SSID
  String password; //WIFI PASSWORD
  String PilotName; //Pilotname
  FanetLora::aircraft_t AircraftType; //Aircrafttype
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
  uint8_t screenNumber; //number of default-screen
};

struct statusData{
  String myIP; //my IP-Adress
  uint16_t vBatt; //battery-voltage 1/1000V
  uint8_t BattPerc; //battery-percent
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
  bool flying;
};

#endif