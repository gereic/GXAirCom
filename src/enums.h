#ifndef __ENUMS_H__
#define __ENUMS_H__

/* Boards */
enum eBoard {
    T_BEAM = 0,
    HELTEC_LORA = 1,
    T_BEAM_V07 = 2,
    TTGO_T3_V1_6 = 3,    
    TTGO_TSIM_7000 = 4,
    T_BEAM_SX1262 = 5,
    TTGO_TCALL_800 = 6,
    HELTEC_WIRELESS_STICK_LITE = 7,
    HELTEC_WIRELESS_STICK = 8,
    T_BEAM_S3CORE = 9,
    HELTEC_WIRELESS_STICK_LITE_V3 = 10,
    HELTEC_LORA_V3 = 11,
    UNKNOWN = 255
};

/* PMU */
enum ePMU {
  NOPMU = 0,  
  AXP192 = 1,
  AXP2101 = 2
};

/* aneometer */
enum eAnemometer {
    DAVIS = 0,
    TX20 = 1,
    ADS_A1015 = 2, // don't use ADS1015 or things will break.
    PEETBROS = 3,
    MISOL = 4,
    WS90 = 5,
    WS85_serial = 6
};

/* Mode of operation */
enum eMode{
  AIR_MODULE = 0,
  GROUND_STATION = 1,
  FANET_INTERFACE = 2,
  DEVELOPER = 100
};

/* output-mode */
enum eOutput{
  oSERIAL = 0,
  oUDP = 1,
  oBLUETOOTH = 2,
  oBLE = 3
};

/* Displays */
enum eDisplay{
  NO_DISPLAY = 0,
  OLED0_96 = 1,
  EINK2_9 = 2,
  EINK2_9_V2 = 3
};

/* Wifi-Connect-Mode */
enum eWifiMode{
  CONNECT_NONE = 0,
  CONNECT_ONCE = 1,
  CONNECT_ALWAYS = 2
};

/* GSMPreferedMode */
enum eGsmNetworkMode{
  GSM_NW_AUTO = 2,
  GSM_NW_GSM = 13,
  GSM_NW_LTE = 38,
  GSM_NW_GSM_LTE = 51
};

/* GSMPreferedMode */
enum eGsmPreferedMode{
  GSM_PREF_NOT_SET = 0,
  GSM_PREF_CAT_M = 1,
  GSM_PREF_NB_IOT = 2,
  GSM_PREF_CAT_M_NB_IOT = 3
};

/* Screen Option */
enum eScreenOption{
  ALWAYS_ON = 0,
  ON_WHEN_TRAFFIC = 1,
  ALWAYS_OFF = 2,
  WEATHER_DATA = 3
};

/* Vario Output Option */
enum eOutputVario{
  OVARIO_NONE = 0,
  OVARIO_LK8EX1 = 1,
  OVARIO_LXPW = 2

};

/* ground-station power-mode */
enum eGsPower{
  GS_POWER_ALWAYS_ON = 0,
  GS_POWER_SAFE = 1,
  GS_POWER_BATT_LIFE = 2

};

/* Fanet-mode */
enum eFnMode{
  FN_GROUNT_AIR_TRACKING = 0,
  FN_AIR_TRACKING = 1,
};

//connection-state of wifi and AP
enum eConnectionState {
    IDLE = 0,
    STARTED = 10,
    CONNECTING = 20,
    CONNECTED = 30,
    FULL_CONNECTED = 50,
    DISCONNECTED = 100
};

/* state of display */
enum eDisplayState{
  DISPLAY_STAT_OFF = 0,
  DISPLAY_STAT_ON = 1
};

/* state of display */
enum eRadarDispMode{
  CLOSEST = 0,
  LIST = 1,
  FRIENDS = 2
};

enum eAdsAneometerPin{
  ADS_WINDSPEED_PIN = 0,
  ADS_WINDDIR_PIN = 1,
  ADS_VREF_PIN = 3
};

enum eRTC{
  RTC_NONE = 0,
  RTC_3031 = 1,
  RTC_3231 = 2
};

enum wdService {
  weatherUnderground = 0,  
  holfuy = 1
};




#endif
