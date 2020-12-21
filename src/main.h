#include <string.h>
#include <FanetLora.h>

#ifndef __MAIN_H__
#define __MAIN_H__

/*
************* Pins for TTGO-TBeam V1.0 / V1.1 ***************
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
******* BUTTON ********
BUTTON_PIN 0

******* LED *********
LED_PIN 2


******* LORA ********
SCK 5
MISO 19
MOSI 27
SS 18
RST 14
DIO0 26

******* OLED ********
SDA 4
SCL 15
RST 16

******** BARO *********
SDA 13
SCL 23 

******** GPS *********
TXD 12 
RXD 15

******** BUZZER *******
BUZZER 17

******** BATTERY-Voltate *******
                       ____           ____
BATT_VOLT_PIN 34   ---|____|----|----|____|-----
                  GND  100K   GPIO34   27K     3.3V

******** Temperature-sensor DS18B20 for Ground-station *********
TEMP_PIN 22

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


************* Pins for TTGO-TSIM7000G ***************
******* LORA ********
SCK 18
MISO 19
MOSI 23
SS 5
RST 12
DIO0 32

******** GPS *********
TXD 27 
RXD 26

******* OLED ********
SDA 21
SCL 22 

******** BARO *********
SDA 13
SCL 14 




*/

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
#define BOARD_TTGO_TSIM_7000 4

#define BAND868 0
#define BAND915 1

#define BATTFULL 4050
#define BATTEMPTY 3300
#define BATTPINOK 1000 //1 Volt for Batt-pinOK

#define MAXSCREENS 2

#define RADAR_CLOSEST 0
#define RADAR_LIST 1
#define RADAR_FRIENDS 2

#define MIN_FLIGHT_SPEED 15.0 //min speed for flying-detection 
// > at least for 10sec --> takeoff
// < at least for 60sec --> landing
#define MIN_FLIGHT_TIME 10000
#define MIN_GROUND_TIME 60000

#define WIFI_RECONNECT_TIME 600000

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

#define MODE_WIFI_DISABLED 0
#define MODE_WIFI_STA 1
#define MODE_WIFI_CONNECTED 2

#define WEATHER_UPDATE_RATE 40000 //update every 40 seconds weatherdata
#define WEATHER_UNDERGROUND_UPDATE_RATE 300000 //update 5min

#define MODEM_DISCONNECTED 0
#define MODEM_CONNECTING 1
#define MODEM_CONNECTED 2

#define SCREEN_ALWAYS_ON 0
#define SCREEN_ON_WHEN_TRAFFIC 1
#define SCREEN_ALWAYS_OFF 2

#define DISPLAY_STAT_OFF 0
#define DISPLAY_STAT_ON 1

#define GS_POWER_ALWAYS_ON 0
#define GS_POWER_SAFE 1



struct weatherupload{
  bool enable;
  String ID;
  String KEY;
};

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
  uint8_t SreenOption; //energy-option for display
  uint8_t PowerSave; //powersave-option
};

struct WeatherSettings{
  float tempOffset;
  uint8_t sendFanet;  
};

struct GsmSettings{
  String apn;
  String user;
  String pwd;
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
  uint8_t LegacyTxEnable; //OGN-Live-Tracking
  uint8_t traccarLiveTracking; //Traccar live-tracking
  String TraccarSrv; //OGN-Live-Tracking  
  WeatherSettings wd;
  weatherupload WUUpload; //weather-underground upload-settings
  weatherupload WindyUpload; //weather-underground upload-settings
  GsmSettings gsm; //settings for GSM
};

struct weatherStatus{
  float temp; //temp [°C]
  float Humidity; // [%rH]
  float Pressure; // [hPa]
  float WindDir; //[Deg]
  float WindSpeed; //[km/h]
  float WindGust; //[km/h]
  float rain1h; // rain this hour [l/h]
  float rain1d; // rain this day [l/h]
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
  bool bHasBME;
  bool bWUBroadCast;
  uint32_t tGPSCycle;
  uint32_t tLoop; //current Loop-time
  uint32_t tMaxLoop; //max Loop-time
  bool flying;
  uint8_t wifiStat;
  int8_t wifiRssi;
  uint8_t bluetoothStat;
  uint32_t flightTime; //flight-time in sek.
  bool bMuting; //muting beeper
  bool bPowerOff;
  weatherStatus weather;
  uint8_t modemstatus; //status of mobile-device (sim800)
  bool bInternetConnected;
  bool bTimeOk;
  bool bHasGSM;
  int16_t GSMSignalQuality;
  uint8_t displayStat; //stat of display
};

#endif