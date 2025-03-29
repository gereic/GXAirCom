#include <string.h>
#include "enums.h"
//#include "../lib/FANETLORA/FanetLora.h"

#ifndef __MAIN_H__
#define __MAIN_H__

#define MAXSTRING 255
#define MAXSIZEBLE 256 // same as local mtu

#define MAXFNTUPLOADSTATIONS 5

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
SDA 3
SCL 4 

******** GPS *********
TXD 34 
RXD --

******** BUZZER *******
BUZZER 25

******** EXTRA BUTTON (Volume) *******
GND >> BUTTON >> GPIO 0

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

#define GSM_CHECK_TIME_CON 60000
#define GSM_CHECK_TIME_SMS 5000
#define GSM_MAX_BAUD 230400

#define OLED_SLAVE_ADDRESS (0x3C)


#define ARDUINO_RUNNING_CORE0 0
#define ARDUINO_RUNNING_CORE1 1

#define RADAR_SCREEN_CENTER_X 32
#define RADAR_SCREEN_CENTER_Y 38

#define BATTPINOK 2000 //2 Volt for Batt-pinOK
#define BATTSLEEPTIME 3600 //check every 1h if Battery is full again
#define BATTPERCSTART 10 //percent, where esp starts normal in addition to min. percentage

#define MAXSCREENS 2

#define MIN_FLIGHT_SPEED 15.0 //min speed for flying-detection 
// > at least for 5sec --> takeoff
// < at least for 60sec --> landing
#define MIN_FLIGHT_TIME 5000
#define MIN_GROUND_TIME 60000

#define WIFI_RECONNECT_TIME 600000 //10min.

#define NUMBUTTONS 2
//#define BUTTON2 38
#define LONGPRESSTIME 250

#define LOWVOLUME 50
#define MIDVOLUME 80
#define HIGHVOLUME 127

#define DISPLAY_UPDATE_RATE 500
#define DISPLAY_UPDATE_RATE2 2000
#define DISPLAY_UPDATE_RATE_GS 2000

#define FLARM_UPDATE_RATE 1000
#define FLARM_UPDATE_STATE 60000

#define FANET_CMD_START			"#FN"
#define FANET_CMD_TRANSMIT	"#FNT"
#define FANET_CMD_GROUND_TYPE	"#FNG"
#define SYSTEM_CMD	"#SYC"
#define GPS_STATE	"$G"

#define FUELSENDINTERVALL 10000

#define GETNTPINTERVALL 1800000 //refresh timer from NTP every 30min.

struct weatherupload{
  bool enable;
  String ID;
  String KEY;
};

struct Fanetweatherupload{
  uint32_t FanetId = 0;
  String ID;
  String KEY;
};

struct weatherAvg{
  float Winddir;
  float sinWinddir;
  float cosWinddir;
  float WindSpeed; //[km/h]
  float WindGust; //[km/h]
  float Pressure; // [hPa]
  float Humidity; // [%rH]
  float temp; //temp [°C]
};

struct VarioSettings{
  float sinkingThreshold;
  float climbingThreshold;
  float nearClimbingSensitivity;
  uint8_t volume;
  bool BeepOnlyWhenFlying;
  bool useMPU;
  int16_t accel[3];
  int16_t gyro[3];
  float tValues[2];
  float zValues[2];    
  float tempOffset;
  float sigmaP;
  float sigmaA;
};

	struct MqttModeBits
	{
			unsigned enable:1, sendWeather:1, b2:1, b3:1, b4:1, b5:1, b6:1, b7:1;
	};

  	union uMqttMode
	{
			MqttModeBits bits;
			uint8_t mode;
	};

	struct WeatherStationModeBits
	{
			unsigned enable:1, hasBME:1, rainSensor:1, ds18B20:1, SHT20:1, b5:1, b6:1, b7:1;
	};

  	union uWeatherStationMode
	{
			WeatherStationModeBits bits;
			uint8_t mode;
	};

struct AnemometerSettings {
  eAnemometer AnemometerType; //Anemometer
  uint8_t AnemometerAdsGain;
  float AnemometerAdsWSpeedMinVoltage;
  float AnemometerAdsWSpeedMaxVoltage;
  float AnemometerAdsWDirMinVoltage;
  float AnemometerAdsWDirMaxVoltage;
  float AnemometerAdsWSpeedMinSpeed;
  float AnemometerAdsWSpeedMaxSpeed;
  float AnemometerAdsWDirMinDir;
  float AnemometerAdsWDirMaxDir;
  float AnemometerAdsVDivR1;
  float AnemometerAdsVDivR2;
};

struct MqttSettings{
  uMqttMode mode;
  String server = "";
  uint16_t port;
  String pw = "";
};

struct statusVario{
  bool bHasVario;
  bool bHasMPU;
  int16_t accel[3];
  int16_t gyro[3];
  float acc_Z;
  float pressure = NAN;
  float alt;
  float temp = NAN;
  float Heading;
  float ClimbRate;
};

struct GSSettings{
  float lat; //Ground-Station Latitude
  float lon; //Ground-Station Longitude
  float alt; //Ground-Station altitude
  float geoidAlt; //geoidaltitude for Legacy
  eScreenOption SreenOption; //energy-option for display
  eGsPower PowerSave; //powersave-option
  int sunriseOffset; //sunrise offset [min]
  int sunsetOffset; //sunrise offset [min]
};

struct WeatherSettings{
  
  uWeatherStationMode mode;
  AnemometerSettings anemometer;
  float tempOffset;
  int16_t windDirOffset;
  uint8_t sendFanet;  
  float avgFactorFanet;
  uint32_t FanetUploadInterval;
  float avgFactorWU;
  uint32_t WUUploadIntervall;
};

struct GsmSettings{
  String apn;
  String user;
  String pwd;
  eGsmNetworkMode NetworkMode;
  eGsmPreferedMode PreferredMode;
  String NB_IOT_Band;
  String CAT_M_Band;
};

	struct WifiModeBits
	{
			unsigned switchOffApWhenStaConnected:1, disableWifiAtStartup:1, b2:1, b3:1, b4:1, b5:1, b6:1, b7:1;
	};
	union uWifiMode
	{
			WifiModeBits bits;
			uint8_t mode;
	};

struct WifiSettings{
  String appw; //access-point-Password
  String ssid; //WIFI SSID
  String password; //WIFI PASSWORD
  eWifiMode connect; //1 connect to wifi, 2 connect to wifi and try to stay connected
  uint32_t tWifiStop; //time after wifi will be stopped to save energy 0 --> never
  uWifiMode uMode; //wifi-mode
};

struct GPSSettings{
  bool customGPSConfig; // When enable, GPS is configured with additional like SBAS, 5HZ, Galileo and a few others from the default
  uint32_t Baud; //GPS-Baudrate
};

	struct OgnModeBits
	{
			unsigned liveTracking:1, sendWeather:1, fwdWeather:1, fwdName:1, b4:1, b5:1, b6:1, b7:1;
	};
	union uOgnMode
	{
			OgnModeBits bits;
			uint8_t mode;
	};


struct SettingsData{
  uint8_t settingsView; //view of settings (basic, advanced, expert)
  eBoard boardType;
  uint8_t CPUFrequency; //CPU-Frequency
  int32_t FrqCor; //frequency-correction [kHz]
  eMode Mode; //Air-Module, GS-Station,
  eDisplay displayType;
  uint8_t displayRotation; //displayrotation 0-3;
  float BattVoltOffs; //offset for Battery-multiplier
  uint8_t minBattPercent;
  uint8_t restartBattPercent;
  uint8_t myDevIdType; // Address type 0=Stateless Random, 1=Official ICAO, 2=stableFlarm, FF=Not overridden
  String myDevId; //my device-ID
  GPSSettings gps;
  eOutputVario outputModeVario;
  uint8_t outputFLARM;
  uint8_t outputGPS;
  uint8_t outputFANET;
  bool bOutputSerial; //additional output over serial-interface
  uint8_t awLiveTracking; //airwhere live-tracking
  WifiSettings wifi;
  String PilotName; //Pilotname
  uint8_t AircraftType; //Aircrafttype
  String UDPServerIP; //UDP-IP-Adress for sending Pakets
  uint16_t UDPSendPort; //Port of udp-server
  eOutput outputMode; //output-mode
  GSSettings gs;
  VarioSettings vario; //variosettings
  uOgnMode OGNLiveTracking; //OGN-Live-Tracking
  uint8_t screenNumber; //number of default-screen
  uint8_t RFMode; //RF-Mode
  uint8_t traccarLiveTracking; //Traccar live-tracking
  String TraccarSrv; //OGN-Live-Tracking  
  WeatherSettings wd;
  weatherupload WUUpload; //weather-underground upload-settings
  weatherupload WindyUpload; //weather-underground upload-settings
  GsmSettings gsm; //settings for GSM
  eFnMode fanetMode; //fanet tracking-mode 0 ... switch between online-tracking and ground-tracking 1 ... always online-tracking
  uint16_t fanetpin; //pin for fanet (4 signs)
  bool bHasExtPowerSw; //has external power-switch
  bool bHasFuelSensor; //has fuel-Sensor
  MqttSettings mqtt;
  Fanetweatherupload FntWuUpload[MAXFNTUPLOADSTATIONS]; //Fanet WU Upload
  Fanetweatherupload FntWiUpload[MAXFNTUPLOADSTATIONS]; //Fanet Wi Upload
  bool bAutoupdate; //auto-update to new Version
};

struct wdErrorBits
{
    unsigned VanevalueInvalid:1, b1:1, b2:1, b3:1, b4:1, b5:1, b6:1, notStarted:1;
};
union uWdError
{
    wdErrorBits bits;
    uint8_t value;
};

struct weatherStatus{
  bool bTemp; //temp exists
  float temp = NAN; //temp [°C]
  bool bHumidity; //humidity exists
  float Humidity = NAN; // [%rH]
  bool bPressure; // pressure exists
  float Pressure = NAN; // [hPa]
  float WindDir = NAN; //[Deg]
  float WindSpeed = NAN; //[km/h]
  float WindGust = NAN; //[km/h]
  bool bRain; //rain-sensor exists
  float rain1h = NAN; // rain this hour [l/h]
  float rain1d = NAN; // rain this day [l/h]
  int vaneVAlue = 0;
  uWdError error;
};

struct commandData{
  uint8_t ConfigGPS; //command to configure GPS
  uint8_t CalibAcc;  //command calibrate acc
  uint8_t CalibGyro; //command calibrate Gyro
  uint8_t getGpsPos; //command to get GPS-Position
};

struct gsmStatus{
  bool bHasGSM;
  int16_t SignalQuality; //signalquality
  int16_t networkstat; //networkstatus
  String sOperator; //Operator
  uint32_t baud;
};

struct statusNetwork{
  eConnectionState state = IDLE;
  String ip = "";
};

struct statusWifi{
  eConnectionState state = IDLE;
  String ip = "";
  int8_t Rssi;
};

struct statusGPS{
  bool bHasGPS;
  bool bExtGps = false;
  uint8_t NumSat;
  uint8_t Fix;
  double Lat;
  double Lon;
  float alt;
  float speed;
  float course;
  float geoidAlt;
  uint8_t hdop; //hdop in 1/10m
  char Date[10];
  char Time[10];  
  uint32_t tCycle;  
};

struct statusBat{
  bool charging;  
  uint16_t voltage; //battery-voltage 1/1000V
  uint8_t percent; //battery-percent
};

struct statusRestart{
  uint32_t time;
  bool doRestart;
};

struct statusRtc{
  eRTC module = RTC_NONE;
  float voltage = 0.0;
  float temp = 0;
};

struct statusData{
  uint8_t displayType;
  statusNetwork wifiAP;
  statusWifi wifiSTA;
  bool bWifiOn;
  statusBat battery;
  statusGPS gps;
  uint16_t fanetTx;
  uint16_t fanetRx;
  uint16_t legTx;
  uint16_t legRx;
  ePMU PMU;
  statusVario vario;
  bool bWUBroadCast;
  uint32_t tLoop; //current Loop-time
  uint32_t tMaxLoop; //max Loop-time
  bool flying;
  uint8_t bluetoothStat;
  uint32_t flightTime; //flight-time in sek.
  bool bMuting; //muting beeper
  bool bPowerOff;
  weatherStatus weather;
  eConnectionState modemstatus; //status of mobile-device (sim800)
  bool bInternetConnected;
  bool bTimeOk;
  gsmStatus gsm;
  eDisplayState displayStat; //stat of display
  uint8_t updateState; //state of update
  String sNewVersion;
  uint32_t tRestart;
  float fuelSensor; //ads of fuel-sensor
  uint8_t calibAccStat;
  uint8_t FanetMsgCount;
  String lastFanetMsg;
  uint8_t MqttStat;
  statusRestart restart;
  statusRtc rtc;
};

//define wifi-events on for old framework
#if ESP_IDF_VERSION_MAJOR < 4
  #define ARDUINO_EVENT_WIFI_READY SYSTEM_EVENT_WIFI_READY
  #define ARDUINO_EVENT_WIFI_STA_START SYSTEM_EVENT_STA_START
  #define ARDUINO_EVENT_WIFI_STA_STOP SYSTEM_EVENT_STA_STOP
  #define ARDUINO_EVENT_WIFI_STA_CONNECTED SYSTEM_EVENT_STA_CONNECTED
  #define ARDUINO_EVENT_WIFI_STA_DISCONNECTED SYSTEM_EVENT_STA_DISCONNECTED
  #define ARDUINO_EVENT_WIFI_STA_GOT_IP SYSTEM_EVENT_STA_GOT_IP
  #define ARDUINO_EVENT_WIFI_STA_LOST_IP SYSTEM_EVENT_STA_LOST_IP
  #define ARDUINO_EVENT_WIFI_AP_START SYSTEM_EVENT_AP_START
  #define ARDUINO_EVENT_WIFI_AP_STOP SYSTEM_EVENT_AP_STOP
  #define ARDUINO_EVENT_WIFI_AP_STACONNECTED SYSTEM_EVENT_AP_STACONNECTED
  #define ARDUINO_EVENT_WIFI_AP_STADISCONNECTED SYSTEM_EVENT_AP_STADISCONNECTED
  #define ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED SYSTEM_EVENT_AP_STAIPASSIGNED
#endif

#endif