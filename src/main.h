#include <string.h>
#include <FanetLora.h>


/*
************* Pins for TTGO-TBeam V1.0 ***************
******* LORA ********
SCK 5
MISO 19
MOSI 27
SS 18
RST 23
DIO0 26

******* AXP192 *******
SDA 21
SCL 22 
******* OLED ********
SDA 21
SCL 22 

******* E-Ink *********
BUSY     33
RST      4
DC       32
CS       15
CLK      25
DIN      2

******** BARO *********
SDA 13
SCL 14 

******** GPS *********
TXD 34 
RXD 12

******** BUZZER *******
BUZZER 0


************* Pins for TTGO-TBeam V07 ***************
******* LORA ********
SCK 5
MISO 19
MOSI 27
SS 18
RST 23
DIO0 26

******* OLED ********
SDA 21
SCL 22 

******* E-Ink *********
BUSY     33
RST      4
DC       32
CS       15
CLK      25
DIN      2

******** BARO *********
SDA 13
SCL 14 

******** BUZZER *******
BUZZER 0


************* Pins for TTGO Lora32 ***************
******* LORA ********
SCK 5
MISO 19
MOSI 27
SS 18
RST 14
DIO0 26

******* OLED ********
SDA 21
SCL 22 
RST 16

******** BARO *********
SDA 13
SCL 23 

******** GPS *********
TXD 12 
RXD 15

******** BUZZER *******
BUZZER 0

************* Pins for TTGO Lora32 V2.1 1.6 ***************
******* LORA ********
SCK 5
MISO 19
MOSI 27
SS 18
RST 23
DIO0 26

******* OLED ********
SDA 21
SCL 22 

******** BARO *********
SDA 13
SCL 14 

******** GPS *********
TXD 34 
RXD 12

******** BUZZER *******
BUZZER 0




*/

#ifndef __MAIN_H__
#define __MAIN_H__

#define VERSION "V1.4.6"
#define APPNAME "GXAirCom"

#define BLE_LOW_HEAP 10000
#define MAX_BLE_LOW_HEAP_TIME 30000


#define ARDUINO_RUNNING_CORE0 0
#define ARDUINO_RUNNING_CORE1 1

#define RADAR_SCREEN_CENTER_X 32
#define RADAR_SCREEN_CENTER_Y 38

#define OUTPUT_SERIAL 0
#define OUTPUT_UDP 1
#define OUTPUT_BLUETOOTH 2
#define OUTPUT_BLE 3

#define BOARD_T_BEAM 0
#define BOARD_HELTEC_LORA 1
#define BOARD_T_BEAM_V07 2
#define BOARD_TTGO_T3_V1_6 3

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

#define WIFI_RECONNECT_TIME 60000

#define BUTTON2 38
#define LONGPRESSTIME 250

#define LOWVOLUME 50
#define MIDVOLUME 100
#define HIGHVOLUME 250

#define DISPLAY_UPDATE_RATE 500
#define DISPLAY_UPDATE_RATE2 2000
#define DISPLAY_UPDATE_RATE_GS 2000

#define FLARM_UPDATE_RATE 1000
#define FLARM_UPDATE_STATE 60000

//defines for display
#define OLED0_96 0
#define EINK2_9  1

//defines for Mode
#define MODE_AIR_MODULE 0
#define MODE_GROUND_STATION 1
#define MODE_FANET_INTERFACE 2
#define MODE_DEVELOPER 100

//defines for wifi connect
#define WIFI_CONNECT_NONE 0
#define WIFI_CONNECT_ONCE 1
#define WIFI_CONNECT_ALWAYS 2

#define FANET_CMD_START			"#FN"
#define FANET_CMD_TRANSMIT	"#FNT"

#define WIFI_DISABLED 0
#define WIFI_STA 1
#define WIFI_CONNECTED 2


struct VarioSettings{
  float sinkingThreshold;
  float climbingThreshold;
  float nearClimbingSensitivity;
  uint8_t volume;
  bool BeepOnlyWhenFlying;
};

struct GSSettings{
  float lat; //Ground-Station Latitude
  float lon; //Ground-Station Longitude
  float alt; //Ground-Station altitude
  String AWID; //Ground-Station ID
};

struct WifiSettings{
  String appw; //access-point-Password
  String ssid; //WIFI SSID
  String password; //WIFI PASSWORD
  uint8_t connect; //1 connect to wifi, 2 connect to wifi and try to stay connected
  uint32_t tWifiStop; //time after wifi will be stopped to save energy 0 --> never
};



struct SettingsData{
  uint8_t boardType;
  uint8_t Mode; //Air-Module, GS-Station,
  uint8_t displayType;
  String myDevId; //my device-ID
  uint8_t band;
  uint8_t LoraPower; //output-Power 5-20db
  uint8_t outputLK8EX1;
  uint8_t outputFLARM;
  uint8_t outputGPS;
  uint8_t outputFANET;
  uint8_t awLiveTracking; //airwhere live-tracking
  WifiSettings wifi;
  String PilotName; //Pilotname
  FanetLora::aircraft_t AircraftType; //Aircrafttype
  String UDPServerIP; //UDP-IP-Adress for sending Pakets
  uint16_t UDPSendPort; //Port of udp-server
  uint8_t outputMode; //output-mode
  GSSettings gs;
  VarioSettings vario; //variosettings
  uint8_t OGNLiveTracking; //OGN-Live-Tracking
  uint8_t screenNumber; //number of default-screen
};

struct statusData{
  String myIP; //my IP-Adress
  uint16_t vBatt; //battery-voltage 1/1000V
  uint8_t BattPerc; //battery-percent
  bool BattCharging;
  uint8_t GPS_Fix;
  double GPS_Lat;
  double GPS_Lon;
  float GPS_alt;
  float GPS_speed;
  float GPS_course;
  float pressure;
  float varioAlt;
  float varioTemp;
  float varioHeading;
  uint8_t GPS_NumSat;
  float ClimbRate;
  uint16_t fanetTx;
  uint16_t fanetRx;
  bool bHasAXP192;
  bool bHasVario;
  uint32_t tGPSCycle;
  uint32_t tLoop; //current Loop-time
  uint32_t tMaxLoop; //max Loop-time
  bool flying;
  uint8_t wifiStat;
  uint8_t bluetoothStat;
  uint32_t flightTime; //flight-time in sek.
  bool bMuting; //muting beeper
};

#endif