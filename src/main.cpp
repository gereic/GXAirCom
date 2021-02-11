#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <WiFiClient.h>
#include <SPI.h>
//#include <LoRa.h>
#include <FanetLora.h>
#include <Flarm.h>
#include <axp20x.h>
#include <main.h>
#include <config.h>
#include "WebHelper.h"
#include "fileOps.h"
#include <SPIFFS.h>
#include <ble.h>
#include <icons.h>
#include <Ogn.h>
#include "SparkFun_Ublox_Arduino_Library.h"
#include <TimeLib.h>
#include <sys/time.h>
//#include <HTTPClient.h>
#include <ArduinoHttpClient.h>

#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_bt_main.h"
#include "driver/adc.h"
//#include "Update.h"
#include "gxUpdater.h"

//#define TEST

#ifdef GSM_MODULE

// Set serial for debug console (to the SerialMon Monitor, default speed 115200)
//#define MODEMDEBUG

#include <TinyGsmClient.h>
  HardwareSerial GsmSerial(2);

#if defined(MODEMDEBUG)
  #define SerialMon Serial
  #define TINY_GSM_DEBUG SerialMon
  #include <StreamDebugger.h>
  StreamDebugger debugger(GsmSerial, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(GsmSerial);
#endif
  //TinyGsm modem(GsmSerial);
  TinyGsmClient GsmOGNClient(modem,0); //client number 0 for OGN
  TinyGsmClient GsmWUClient(modem,1); //client number 1 for weather-underground
  TinyGsmClient GsmUpdaterClient(modem,2); //client number 2 for updater
#endif

#ifdef EINK
#include <Screen.h>
#endif

#ifdef GSMODULE

#include <Dusk2Dawn.h> //for sunset and sunrise-functions
#include <Weather.h>
#include <WeatherUnderground.h>
#include <Windy.h>

#define uS_TO_S_FACTOR 1000000uL  /* Conversion factor for micro seconds to seconds */
//#define uS_TO_ms_FACTOR 1000000uL  /* Conversion factor for micro seconds to seconds */
//#define TIME_TO_SLEEP  5uL //5 seconds
//#define TIME_TO_SLEEP  1800uL //1/2 Stunde
#define TIME_TO_SLEEP  60uL //1 min
//#define TIME_TO_SLEEP  10uL //10 sec
//#define TIME_TO_SLEEP  21600uL //6 Stunden
//#define TIME_TO_SLEEP  43200uL //12 Stunden
//#define TIME_TO_SLEEP  3600uL //1 Stunde
//#define TIME_TO_SLEEP  600uL //10 min

RTC_DATA_ATTR int iSunRise = -1;
RTC_DATA_ATTR int iSunSet = -1;

#endif

//Libraries for OLED Display
#include <Wire.h>
TwoWire i2cOLED = TwoWire(1);


#ifdef OLED
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &i2cOLED);

#endif

#ifdef BLUETOOTHSERIAL

#include <BluetoothSerial.h>
BluetoothSerial SerialBT;

#endif

#ifdef AIRMODULE
#include <MicroNMEA.h>
//Libraries for Vario
#include <Baro.h>
#include <beeper.h>
#include <toneAC.h>

#define USE_BEEPER

char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

Baro baro;
beeper Beeper(BEEP_VELOCITY_DEFAULT_SINKING_THRESHOLD,BEEP_VELOCITY_DEFAULT_CLIMBING_THRESHOLD,BEEP_VELOCITY_DEFAULT_NEAR_CLIMBING_SENSITIVITY,BEEP_DEFAULT_VOLUME);
int freq = 2000;
int channel = 0;
int resolution = 8;
uint8_t wifiCMD = 0;


#endif

bool WebUpdateRunning = false;
bool bPowerOff = false;
bool bGsmOff= false;

struct SettingsData setting;
struct statusData status;
String host_name = "";
String bleMsg = "";

const char compile_date[] = __DATE__ " " __TIME__;

//WebServer server(80);

FanetLora fanet;
//NmeaOut nmeaout;
Flarm flarm;
#ifdef AIRMODULE
HardwareSerial NMeaSerial(2);
#endif
//MicroNMEA library structures

uint16_t battFull = 4050;
uint16_t battEmpty = 3300;

Ogn ogn;

gxUpdater updater;  

FanetLora::trackingData MyFanetData;  


FanetLora::trackingData testTrackingData;
FanetLora::weatherData testWeatherData;
bool sendWeatherData = false;
String testString;
uint32_t fanetReceiver;
uint8_t sendTestData = 0;

IPAddress local_IP(192,168,4,1);
IPAddress gateway(192,168,4,1);
IPAddress subnet(255,255,255,0);

volatile bool ppsTriggered = false;

AXP20X_Class axp;
#define AXP_IRQ 35
volatile bool AXP192_Irq = false;
volatile float BattCurrent = 0.0;

//bool newStationConnected = false;

#define AIRWHERE_UDP_PORT 5555
String airwhere_web_ip = "37.128.187.9";



 unsigned long ble_low_heap_timer=0;
 String ble_data="";
 bool ble_mutex=false;


static RTC_NOINIT_ATTR uint8_t startOption;
static RTC_NOINIT_ATTR uint8_t detectOption;
uint32_t psRamSize = 0;

uint32_t fanetDstId = 0;


//PIN-Definition

//LED
int8_t PinUserLed = -1;

//ADC-Voltage
int8_t PinADCVoltage = -1;

//LORA-Module
int8_t PinLoraRst = -1;
int8_t PinLoraDI0 = -1;
int8_t PinLora_SS = -1;
int8_t PinLora_MISO = -1;
int8_t PinLora_MOSI = -1;
int8_t PinLora_SCK = -1;

//GSM-Module
int8_t PinGsmRst = -1;
int8_t PinGsmTx = -1;
int8_t PinGsmRx = -1;
int8_t PinGsmGPSPower = -1;

//OLED-Display / AXP192
int8_t PinOledRst = -1;
int8_t PinOledSDA = -1;
int8_t PinOledSCL = -1;

//BARO
int8_t PinBaroSDA = -1;
int8_t PinBaroSCL = -1;

//BUZZER
int8_t PinBuzzer = -1;

//ONE-WIRE
int8_t PinOneWire = -1;

//aneometer
int8_t PinWindDir = -1;
int8_t PinWindSpeed = -1;
int8_t PinRainGauge = -1;

//external Power on/off
int8_t PinExtPowerOnOff = -1;

float adcVoltageMultiplier = 0.0;
String sNmeaIn = "";



TaskHandle_t xHandleBaro = NULL;
TaskHandle_t xHandleStandard = NULL;
TaskHandle_t xHandleBackground = NULL;
TaskHandle_t xHandleBluetooth = NULL;
TaskHandle_t xHandleMemory = NULL;
TaskHandle_t xHandleEInk = NULL;
TaskHandle_t xHandleWeather = NULL;

SemaphoreHandle_t xOutputMutex;
String sOutputData;

#ifdef GSM_MODULE
TaskHandle_t xHandleGsm = NULL;
SemaphoreHandle_t xGsmMutex;
#endif

/********** function prototypes ******************/
#ifdef GSMODULE
void taskWeather(void *pvParameters);
void sendAWGroundStationdata(uint32_t tAct);
void enterDeepsleep();
int isDayTime();
uint32_t calcSleepTime();
#endif
#ifdef AIRMODULE
void readGPS();
void taskBaro(void *pvParameters);
#endif
#ifdef EINK
void taskEInk(void *pvParameters);
#endif
#ifdef GSM_MODULE
void taskGsm(void *pvParameters);
#endif
#ifdef OLED
void startOLED();
void add2OutputString(String s);
void DrawRadarScreen(uint32_t tAct,uint8_t mode);
void DrawRadarPilot(uint8_t neighborIndex);
void printGSData(uint32_t tAct);
void printBattVoltage(uint32_t tAct);
void printScanning(uint32_t tAct);
void printWeather(uint32_t tAct);
void printGPSData(uint32_t tAct);
void DrawAngleLine(int16_t x,int16_t y,int16_t length,float deg);
void drawBatt(int16_t x, int16_t y,uint8_t value);
void drawSignal(int16_t x, int16_t y,uint8_t strength);
void drawflying(int16_t x, int16_t y, bool flying);
void drawAircraftType(int16_t x, int16_t y, FanetLora::aircraft_t AircraftType);
void drawSatCount(int16_t x, int16_t y,uint8_t value);
void drawspeaker(int16_t x, int16_t y);
void drawBluetoothstat(int16_t x, int16_t y);
void drawWifiStat(int wifiStat);
void oledPowerOff();
void oledPowerOn();
void checkBoardType();

void checkBoardType(){
  #ifdef TINY_GSM_MODEM_SIM7000
    setting.displayType = NO_DISPLAY;
    setting.boardType = BOARD_TTGO_TSIM_7000;
    log_i("TTGO-T-Sim7000 found");
    write_configFile(&setting);
    delay(1000);
    esp_restart(); //we need to restart
  #endif
  log_i("start checking if board is T-Beam");
  PinOledSDA = 21;
  PinOledSCL = 22;
  i2cOLED.begin(PinOledSDA, PinOledSCL);
  log_i("init i2c %d,%d OK",PinOledSDA,PinOledSCL);
  i2cOLED.beginTransmission(AXP192_SLAVE_ADDRESS);
  if (i2cOLED.endTransmission() == 0) {
    //ok we have a T-Beam !! 
    //check, if we have an OLED
    setting.boardType = BOARD_T_BEAM;
    log_i("AXP192 found");
    i2cOLED.beginTransmission(OLED_SLAVE_ADDRESS);
    if (i2cOLED.endTransmission() == 0) {
      //we have found also the OLED
      setting.displayType = OLED0_96;
      log_i("OLED found");
      write_configFile(&setting);
      delay(1000);
      esp_restart(); //we need to restart
    }else{
      setting.displayType = NO_DISPLAY;
      log_i("no OLED found");
      write_configFile(&setting);
      delay(1000);
      esp_restart(); //we need to restart
    }
  }else{
    //no AXP192 --> maybe T-Beam V07 or T3 V1.6
    i2cOLED.beginTransmission(OLED_SLAVE_ADDRESS);
    if (i2cOLED.endTransmission() == 0) {
      //we have found the OLED
      setting.boardType = BOARD_T_BEAM_V07;
      setting.displayType = OLED0_96;
      log_i("OLED found");
      write_configFile(&setting);
      delay(1000);
      esp_restart(); //we need to restart
    }
  }

  log_i("start checking if board is HELTEC/TTGO");
  PinOledSDA = 4;
  PinOledSCL = 15;
  PinOledRst = 16;
  i2cOLED.begin(PinOledSDA, PinOledSCL);
  log_i("init i2c %d,%d OK",PinOledSDA,PinOledSCL);
  pinMode(PinOledRst, OUTPUT);
  digitalWrite(PinOledRst, LOW);
  delay(100);
  digitalWrite(PinOledRst, HIGH);
  delay(100);
  i2cOLED.beginTransmission(OLED_SLAVE_ADDRESS);
  if (i2cOLED.endTransmission() == 0) {
    //we have found also the OLED
    setting.boardType = BOARD_HELTEC_LORA;
    setting.displayType = OLED0_96;
    log_i("OLED found");
    write_configFile(&setting);
    delay(1000);
    esp_restart(); //we need to restart
  }

}

void add2OutputString(String s){
  xSemaphoreTake( xOutputMutex, portMAX_DELAY );
  sOutputData += s;
  xSemaphoreGive(xOutputMutex);
}

void oledPowerOn(){
  if (status.displayStat == DISPLAY_STAT_ON){
    return;
  }
  //reset OLED display via software
  if (PinOledRst >= 0){
    log_i("Heltec-board");
    pinMode(PinOledRst, OUTPUT);
    digitalWrite(PinOledRst, LOW);
    delay(100);
    digitalWrite(PinOledRst, HIGH);
    delay(100);
  }

  //initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_SLAVE_ADDRESS, false, false)) { // Address 0x3C for 128x32
    log_e("SSD1306 allocation failed");
    for(;;); // Don't proceed, loop forever
  }
  //set display to full contrast
  display.ssd1306_command(SSD1306_SETCONTRAST);
  display.ssd1306_command(0xFF);
  status.displayStat = DISPLAY_STAT_ON;
}

void oledPowerOff(){
  if (status.displayStat == DISPLAY_STAT_OFF){
    return;
  }
  display.clearDisplay();
  log_i("set display to off");
  //display.ssd1306_command(SSD1306_DISPLAYOFF);
  display.ssd1306_command(0x8D); //into charger pump set mode
  display.ssd1306_command(0x10); //turn off charger pump
  display.ssd1306_command(0xAE); //set OLED sleep  
  status.displayStat = DISPLAY_STAT_OFF; //Display if off now
}

void drawWifiStat(int wifiStat)
{
  if (wifiStat==1) 
  {
    
    WIFI_bits[2]=0xC4;
    WIFI_bits[6]=0xC9;
    display.drawXBitmap(85,0,WIFI_bits,WIFI_width,WIFI_height,WHITE);
  }
  if (wifiStat==2) 
  { 
    WIFI_bits[2]=0x4;
    WIFI_bits[6]=0x9;
    display.drawXBitmap(85,0,WIFI_bits,WIFI_width,WIFI_height,WHITE);
  }
}
#endif

void setupAXP192();
void taskStandard(void *pvParameters);
void taskBackGround(void *pvParameters);
void taskBluetooth(void *pvParameters);
void taskMemory(void *pvParameters);
void setupWifi();
void IRAM_ATTR ppsHandler(void);
void printSettings();
void listSpiffsFiles();
String setStringSize(String s,uint8_t sLen);
//void writeTrackingData(uint32_t tAct);
void sendData2Client(String data);
eFlarmAircraftType Fanet2FlarmAircraft(FanetLora::aircraft_t aircraft);
void Fanet2FlarmData(FanetLora::trackingData *FanetData,FlarmtrackingData *FlarmDataData);
void sendLK8EX(uint32_t tAct);
void powerOff();
esp_sleep_wakeup_cause_t print_wakeup_reason();
void WiFiEvent(WiFiEvent_t event);
//void listConnectedStations();
float readBattvoltage();
void sendAWTrackingdata(FanetLora::trackingData *FanetData);
void sendTraccarTrackingdata(FanetLora::trackingData *FanetData);
void sendAWUdp(String msg);
void checkFlyingState(uint32_t tAct);
void sendFlarmData(uint32_t tAct);
void handleButton(uint32_t tAct);
char* readSerial();
void checkReceivedLine(char *ch_str);
void checkSystemCmd(char *ch_str);
void setWifi(bool on);
void handleUpdate(uint32_t tAct);
#ifdef BLUETOOTHSERIAL
void serialBtCallBack(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
char* readBtSerial();
#endif
void printChipInfo(void);
void setAllTime(tm &timeinfo);
void checkExtPowerOff(uint32_t tAct);
#ifdef AIRMODULE
bool setupUbloxConfig(void);
#endif

void setAllTime(tm &timeinfo){
  tmElements_t tm;          // a cache of time elements
  //fill time-structure
  tm.Year = timeinfo.tm_year+1900 - 1970;
  tm.Month = timeinfo.tm_mon+1;
  tm.Day = timeinfo.tm_mday;
  tm.Hour = timeinfo.tm_hour;
  tm.Minute = timeinfo.tm_min;
  tm.Second = timeinfo.tm_sec;
  time_t t =  makeTime(tm);  
  setTime(t); //set time of timelib
  timeval epoch = {(int32_t)t, 0};
  settimeofday((const timeval*)&epoch, 0); //set time on RTC of ESP32

}

void printChipInfo(void){
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  log_i("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
          chip_info.cores,
          (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
          (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
  log_i("silicon revision %d, ", chip_info.revision);
  log_i("%dMB %s flash", spi_flash_get_chip_size() / (1024 * 1024),
          (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
}


bool printLocalTime()
{
  struct tm now;
  getLocalTime(&now,0);
  if (now.tm_year >= 117){
    //Serial.println(&now, "%B %d %Y %H:%M:%S (%A)");
    log_i("%04d %02d %02d %02d:%02d:%02d",now.tm_year+1900,now.tm_mon+1,now.tm_mday,now.tm_hour,now.tm_min,now.tm_sec);
    return true;
  }else{
    return false;
  }
  //struct tm timeinfo;
  /*
  if (timeStatus() == timeSet){
    log_i("%d %d %d %d:%d:%d",year(),month(),day(),hour(),minute(),second());
    return true;
  }
  log_i("Failed to obtain time");
  return false;
  */
  /*
  if(!getLocalTime(&timeinfo)){
    
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
    return false;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  return true;
  */
}

#ifdef BLUETOOTHSERIAL
void serialBtCallBack(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  if (event == ESP_SPP_SRV_OPEN_EVT){
    status.bluetoothStat = 2; //client connected
  }else if (event == ESP_SPP_CLOSE_EVT){
    status.bluetoothStat = 1; //client disconnected
  }
  //log_i("Event=%d",event);
}
#endif

void handleButton(uint32_t tAct){
  static uint32_t buttonTimer = millis();
  //static uint32_t dblClkTimer = millis();
  //static bool dblClkActive = false;
  static bool buttonActive = false;
  static bool longPressActive = false;
  if (digitalRead(BUTTON2) == LOW){
    //Button pressed
		if (buttonActive == false) {
			buttonActive = true;
			buttonTimer = millis();
		}
		if ((millis() - buttonTimer > LONGPRESSTIME) && (longPressActive == false)) {
			longPressActive = true;
      if (setting.vario.volume == LOWVOLUME){
        setting.vario.volume = MIDVOLUME;  
      }else if (setting.vario.volume == MIDVOLUME){
        setting.vario.volume = HIGHVOLUME;  
      }else if (setting.vario.volume == HIGHVOLUME){
        setting.vario.volume = LOWVOLUME;  
      }
      write_Volume();
      //log_i("volume=%d",setting.vario.volume);
		}
  }else{
    //Button released
		if (!setting.bHasExtPowerSw){ //no muting when in kobo (for the moment)
      if (buttonActive == true) {
        if (longPressActive == false) {
          status.bMuting = !status.bMuting; //toggle muting
        }
      }
    }
		buttonActive = false;
		longPressActive = false;
  }
}


void sendFlarmData(uint32_t tAct){
  static uint32_t tSend = millis();
  static uint32_t tSendStatus = millis();
  FlarmtrackingData myFlarmData;
  FlarmtrackingData PilotFlarmData;
  FanetLora::trackingData tFanetData;  
  uint8_t countNeighbours = 0;

  if (!setting.outputFLARM) return;

  if (timeOver(tAct,tSendStatus,FLARM_UPDATE_STATE)){
    tSendStatus = tAct;
    sendData2Client(flarm.writeVersion());
    sendData2Client(flarm.writeSelfTestResult());
  }

  if (timeOver(tAct,tSend,FLARM_UPDATE_RATE)){
    tSend = tAct;
    if (status.GPS_Fix){
      Fanet2FlarmData(&fanet._myData,&myFlarmData);
      for (int i = 0; i < MAXNEIGHBOURS; i++){
        if (fanet.neighbours[i].devId){
          tFanetData.aircraftType = fanet.neighbours[i].aircraftType;
          tFanetData.altitude = fanet.neighbours[i].altitude;
          tFanetData.climb = fanet.neighbours[i].climb;
          tFanetData.devId = fanet.neighbours[i].devId;
          tFanetData.heading = fanet.neighbours[i].heading;
          tFanetData.lat = fanet.neighbours[i].lat;
          tFanetData.lon = fanet.neighbours[i].lon;
          tFanetData.speed = fanet.neighbours[i].speed;
          Fanet2FlarmData(&tFanetData,&PilotFlarmData);
          sendData2Client(flarm.writeFlarmData(&myFlarmData,&PilotFlarmData));    
          countNeighbours++;    
        }
      }
      flarm.GPSState = FLARM_GPS_FIX3d_AIR;
    }else{
      flarm.GPSState = FLARM_NO_GPS;
    }
    flarm.neighbors = countNeighbours;
    sendData2Client(flarm.writeDataPort());
  }
}

void checkFlyingState(uint32_t tAct){
  static uint32_t tOk = millis();
  static uint32_t tFlightTime = millis();
  if (status.flying){
    //flying
    if ((!status.GPS_Fix) || (status.GPS_speed < MIN_FLIGHT_SPEED)){
      if (timeOver(tAct,tOk,MIN_GROUND_TIME)){
        status.flying = false;
      }
    }else{
      tOk = tAct;
    }
    if (timeOver(tAct,tFlightTime,1000)){
      //1 sec. over --> inc. flight-time
      status.flightTime ++; //inc. flight-Time
      tFlightTime += 1000;
    }
  }else{
    //on ground
    if ((status.GPS_Fix) && (status.GPS_NumSat >= 4) && (status.GPS_speed > MIN_FLIGHT_SPEED)){ //minimum 4 satelites to get accurade position and speed
      if (timeOver(tAct,tOk,MIN_FLIGHT_TIME)){
        tFlightTime = tAct;
        status.flightTime = 0;
        status.flying = true;
      }
    }else{
      tOk = tAct;
    }
  }

}


#ifdef OLED
void drawAircraftType(int16_t x, int16_t y, FanetLora::aircraft_t AircraftType){
  switch (AircraftType)
  {
  case FanetLora::paraglider :
      display.drawXBitmap(x,y, Paraglider16_bits, 16, 16, WHITE);   //GxEPD_BLACK);
      break;
  case FanetLora::hangglider :
      display.drawXBitmap(x,y, Hangglider16_bits, 16, 16, WHITE);   //GxEPD_BLACK);
      break;
  case FanetLora::balloon :
      display.drawXBitmap(x,y, Ballon16_bits, 16, 16, WHITE);   //GxEPD_BLACK);
      break;
  case FanetLora::glider :
      display.drawXBitmap(x,y, Sailplane16_bits, 16, 16, WHITE);   //GxEPD_BLACK);
      break;
  case FanetLora::poweredAircraft :
      display.drawXBitmap(x,y, Airplane16_bits, 16, 16, WHITE);   //GxEPD_BLACK);
      break;
  case FanetLora::helicopter :
      display.drawXBitmap(x,y, Helicopter16_bits, 16, 16, WHITE);   //GxEPD_BLACK);
      break;
  case FanetLora::uav :
      display.drawXBitmap(x,y, UAV16_bits, 16, 16, WHITE);   //GxEPD_BLACK);
      break;
  
  default:
      display.drawXBitmap(x,y, UFO16_bits, 16, 16, WHITE);   //GxEPD_BLACK);
      break;
  }
}

void drawflying(int16_t x, int16_t y, bool flying){
    if (flying){
        display.drawXBitmap(x, y,flying_bits,  16, 16, WHITE);
    }else{
        display.drawXBitmap(x, y,not_flying_bits,  16, 16, WHITE);
    }
}


void drawBatt(int16_t x, int16_t y,uint8_t value){
    static uint8_t DrawValue = 0;
    if (value == 255){
        DrawValue = (DrawValue + 1) %5; 
    }else{
        DrawValue = value / 25;
    }
    //log_i("%d",DrawValue);
    display.fillRect(x,y,17,8,WHITE);
    switch (DrawValue)
    {
    case 1:
        display.drawBitmap(x, y, bat1icons, 17, 8, BLACK);   //GxEPD_BLACK);    
        break;
    case 2:
        display.drawBitmap(x, y, bat2icons, 17, 8, BLACK);   //GxEPD_BLACK);    
        break;
    case 3:
        display.drawBitmap(x, y, bat3icons, 17, 8, BLACK);   //GxEPD_BLACK);    
        break;
    case 4:
        display.drawBitmap(x, y, bat4icons, 17, 8, BLACK);   //GxEPD_BLACK);    
        break;
    default:
        display.drawBitmap(x, y, bat0icons, 17, 8, BLACK);   //GxEPD_BLACK);    
        break;
    }
}

void drawBluetoothstat(int16_t x, int16_t y){
    if (status.bluetoothStat == 1){
     display.drawXBitmap(x,y,BT_bits,8,10,WHITE);
    }else if (status.bluetoothStat == 2){
      display.fillRect(x,y,8,10,WHITE);
      display.drawXBitmap(x,y,BT_bits,8,10,BLACK);
    }
}

void printGSData(uint32_t tAct){
  static uint8_t index = 0;
  char buf[10];
  oledPowerOn();
  display.clearDisplay();
  drawWifiStat(status.wifiStat);
  drawBluetoothstat(101,0);
  drawBatt(111, 0,(status.BattCharging) ? 255 : status.BattPerc);


  //show rx-count
  display.setTextSize(1);

  display.setCursor(78,0);
  sprintf(buf, "%d", uint8_t(status.fanetRx) % 10);
  display.print(buf);


  //get next index
  for (int i = 0;i <= MAXNEIGHBOURS;i++){
    if (fanet.neighbours[index].devId) break;
    index ++;
    if (index >= MAXNEIGHBOURS) index = 0;
  }
  if (!fanet.neighbours[index].devId){
    //no more neighbours --> return false
    display.display();
    return;
  } 



  display.setCursor(0,0);
  display.print(fanet.getDevId(fanet.neighbours[index].devId));
  display.setCursor(35,0);
  sprintf(buf, "%4ddb", fanet.neighbours[index].rssi);
  display.print(buf);

  display.setCursor(0,10);
  display.print(fanet.getNeighbourName(fanet.neighbours[index].devId));

  switch (fanet.neighbours[index].aircraftType)
  {
  case FanetLora::aircraft_t ::paraglider :
    display.drawXBitmap(88, 12, PGRX_bits,PGRX_width, PGRX_height,WHITE);      
    break;
  case FanetLora::aircraft_t::hangglider :
    display.drawXBitmap(88, 12, HGRX_bits,HGRX_width, HGRX_height,WHITE);      
    break;
  case FanetLora::aircraft_t::balloon :
    display.drawXBitmap(88, 12, BLRX_bits,BLRX_width, BLRX_height,WHITE);      
    break;
  case FanetLora::aircraft_t::glider :
    display.drawXBitmap(88, 12, SPRX_bits,SPRX_width, SPRX_height,WHITE);      
    break;
  case FanetLora::aircraft_t::poweredAircraft :
    display.drawXBitmap(88, 12, Airplane40_bits,Airplane40_width, Airplane40_height,WHITE);      
    break;
  case FanetLora::aircraft_t::helicopter :
    display.drawXBitmap(88, 10, Helicopter40_bits,Helicopter40_width, Helicopter40_height,WHITE);      
    break;
  case FanetLora::aircraft_t::uav:
    display.drawXBitmap(88, 12, UAVRX_bits,UAVRX_width, UAVRX_height,WHITE);      
    break;
  
  default:
    display.drawXBitmap(88, 12, UFORX_bits,UFORX_width, UFORX_height,WHITE);      
    break;
  }
  
  display.setCursor(0,20);
  display.print("alt:");
  display.print(fanet.neighbours[index].altitude,0);
  display.print("m");

  display.setCursor(0,32);
  display.print("speed:");
  display.print(fanet.neighbours[index].speed,0);
  display.print("kmh");

  display.setCursor(0,44);
  display.print(fanet.neighbours[index].climb,1);
  display.print("m/s");

  display.setCursor(50,44);
  display.print(fanet.neighbours[index].heading,0);
  display.print("deg");

  display.setCursor(0,56);
  display.print(fanet.neighbours[index].lat,6);
  display.setCursor(64,56);
  display.print(fanet.neighbours[index].lon,6);


  display.display();
  index ++;
  if (index >= MAXNEIGHBOURS) index = 0;
}
#endif

#ifdef GSMODULE
uint32_t calcSleepTime(){
  uint32_t iRet = 0;
  struct tm now;
  getLocalTime(&now,0);
  if (now.tm_year < 117) return -3; //time not set yet
  int iAct = (now.tm_hour * 60) + now.tm_min;
  char time1[] = "00:00";
  char time2[] = "00:00";
  char time3[] = "00:00";
  Dusk2Dawn::min2str(time1, iSunRise);
  Dusk2Dawn::min2str(time2, iSunSet);
  Dusk2Dawn::min2str(time3, iAct);
  log_i("sunrise=%s sunset=%s iAct=%s",time1,time2,time3);
  log_i("sunrise=%d sunset=%d iAct=%d",iSunRise, iSunSet, iAct);
  if (iAct > iSunSet){
    iRet = (1440 - iAct + iSunRise) * 60; //we have to calc until midnight and then until sunrise
  }else if (iAct < iSunRise){
    iRet = (iSunRise - iAct) * 60; //we have to calc until from now to sunrise
  }
  return iRet;  
}


int isDayTime(){
  if ((iSunRise >= 0) && (iSunSet >= 0)) {
    struct tm now;
    getLocalTime(&now,0);
    if (now.tm_year < 117) return -3; //time not set yet    
    int iAct = (now.tm_hour * 60) + now.tm_min;
    char time1[] = "00:00";
    char time2[] = "00:00";
    char time3[] = "00:00";
    Dusk2Dawn::min2str(time1, iSunRise);
    Dusk2Dawn::min2str(time2, iSunSet);
    Dusk2Dawn::min2str(time3, iAct);
    if (iSunRise < iSunSet){
      if ((iAct >= iSunRise) && (iAct <= iSunSet)){
        // all ok. We are between sunrise and sunset
        return 1;
      }else{
        printLocalTime();
        log_i("%02d:%02d:%02d sunrise=%s sunset=%s iAct=%s",now.tm_hour,now.tm_min,now.tm_sec,time1,time2,time3);
        return 0;
      }
    }else{
      if ((iAct >= iSunSet) && (iAct <= iSunRise)){
        return 0;
      }else{
        // all ok. We are between sunrise and sunset
        return 1;
      }
    }
  }
  log_e("sunrise or sunset not set");
  return -1; //error sunrise and sunset not set

}

void enterDeepsleep(){
  uint32_t tSleep = calcSleepTime();
  log_i("time to sleep = %d",tSleep);
  log_i("time to sleep = %02d:%02d:%02d",tSleep/60/60,(tSleep/60)%60,tSleep%60);
  esp_sleep_enable_timer_wakeup((uint64_t)tSleep * uS_TO_S_FACTOR); //set Timer for wakeup
  log_i("not day --> Power off --> enter deep-sleep");
  powerOff();
}

void sendAWGroundStationdata(uint32_t tAct){
  static uint32_t tSend = millis() - 290000; //10sec. delay, to get wifi working
  if ((tAct - tSend) < 300000) return; //every 5min.
  if (!setting.awLiveTracking) return;
  tSend = tAct;
  char chs[20];
  String msg = ">GS Location,"
            + setting.myDevId + ",";  
  sprintf(chs,"%02.6f",setting.gs.lat);
  msg += String(chs) + ",";
  sprintf(chs,"%02.6f",setting.gs.lon);
  msg += String(chs) + ",";
  sprintf(chs,"%d",uint16_t(setting.gs.alt));
  msg += String(chs);
  //log_i("%s",msg.c_str());
  sendAWUdp(msg);
}
#endif

void sendTraccarTrackingdata(FanetLora::trackingData *FanetData){

  if ((!setting.traccarLiveTracking) || (!setting.TraccarSrv.startsWith("http://")) || (WiFi.status() != WL_CONNECTED) || (!status.bTimeOk)) return;

  //HTTPClient http;
  //WiFiClient c;
  //HttpClient http(c);
  String server = setting.TraccarSrv.substring(strlen("http://"),setting.TraccarSrv.length());
  int iPort = 80;
  int posPortStart = server.indexOf(':');  
  if (posPortStart > 0){
    posPortStart++;
    int posPortEnd = server.indexOf('/',posPortStart);
    if (posPortEnd > 0){
    }else{
      posPortEnd = server.length();
    }
    String port = server.substring(posPortStart,posPortEnd);
    iPort = server.substring(posPortStart,posPortEnd).toInt();
    server = server.substring(0,posPortStart-1);
    //log_i("port=%s, %d",port.c_str(),iPort);
  }
  //log_i("server=%s port=%d",server.c_str(),iPort);
  time_t now;
  time(&now);
  char chs[120];
  String msg="/?id=FANET_";
  msg +=fanet.getDevId(FanetData->devId) + "&lat=";

  sprintf(chs,"%02.6f",FanetData->lat);
  msg += String(chs) + "&lon="; //
  sprintf(chs,"%02.6f",FanetData->lon);
  msg += String(chs) + "&timestamp="; //unix-timestamp
  sprintf(chs,"%0ld",(long int)now);
  msg += String(chs) + "&altitude="; //alt in m
  sprintf(chs,"%0.2f",FanetData->altitude);
  msg += String(chs) + "&speed=";
  sprintf(chs,"%0.2f",FanetData->speed / 1.852); //speed in knots
  msg += String(chs) + "&heading=";
  sprintf(chs,"%0.2f",FanetData->heading); //heading in degrees
  msg += String(chs);
  
  //log_i("server=%s port=%d",server.c_str(),iPort);
  //log_i("%s",msg.c_str());

  WiFiClient c;
  HttpClient http(c, server,iPort);
  int httpResponseCode = http.get(msg);
  if (httpResponseCode == 0){
    httpResponseCode = http.responseStatusCode();
    if (httpResponseCode != 200){
      log_e("resp=%d",httpResponseCode);
    } 
  }else{
    log_e("failed to connect=%d",httpResponseCode);
  }
  http.stop();
  //log_i("ready");
}

void sendAWTrackingdata(FanetLora::trackingData *FanetData){
  if (!setting.awLiveTracking) return;
  char chs[20];
  String msg;
  #ifdef GSMODULE
    if (setting.Mode == MODE_GROUND_STATION){
      msg = "000000,";
    }
  #endif
  #ifdef AIRMODULE
  if (setting.Mode == MODE_AIR_MODULE){
    if (nmea.getFixTime().length() == 0){
      msg = "000000,";
    }else{
      msg = nmea.getFixTime() + ",";
    }
  }
  #endif
  msg += setting.myDevId + ","
         + fanet.getDevId(FanetData->devId) + ","
         + String((uint8_t)FanetData->aircraftType) + ",";
  sprintf(chs,"%02.6f",FanetData->lat);
  msg += String(chs) + ",";
  sprintf(chs,"%02.6f",FanetData->lon);
  msg += String(chs) + ",0,0,";
  sprintf(chs,"%0.2f",FanetData->heading);
  msg += String(chs) + ",";
  sprintf(chs,"%0.2f",FanetData->speed * 0.621371); //kmh --> mph
  msg += String(chs) + ",";
  sprintf(chs,"%0.2f",FanetData->altitude);
  msg += String(chs) + "," + String(FanetData->rssi);
  //log_i("%s",msg.c_str());
  sendAWUdp(msg);
}

void sendAWUdp(String msg){
  if (WiFi.status() == WL_CONNECTED){
    //log_i("%s",msg.c_str());
    WiFiUDP udp;
    udp.beginPacket(airwhere_web_ip.c_str(),AIRWHERE_UDP_PORT);
    udp.write((uint8_t *)msg.c_str(),msg.length());
    udp.endPacket();
  }else{
    //log_e("can't send AW-Data WIFI not connected state=%d",WiFi.status());
  }      
}

void sendData2Client(String data){
  if (setting.outputMode == OUTPUT_UDP){
    //output via udp
    if ((WiFi.status() == WL_CONNECTED) || (WiFi.softAPgetStationNum() > 0)){ //connected to wifi or a client is connected to me
      //log_i("sending udp");
      WiFiUDP udp;
      udp.beginPacket(setting.UDPServerIP.c_str(),setting.UDPSendPort);
      udp.write((uint8_t *)data.c_str(),data.length());
      udp.endPacket();    
    }
  }
  if ((setting.outputMode == OUTPUT_SERIAL) || (setting.bOutputSerial)){//output over serial-connection
    Serial.print(data); 
  }
  #ifdef BLUETOOTHSERIAL
  if (setting.outputMode == OUTPUT_BLUETOOTH){//output over bluetooth serial
    if (status.bluetoothStat == 2){
      //if (SerialBT.hasClient()){
        //log_i("sending to bt-device %s",data.c_str());
      SerialBT.print(data);
      //}    
    }
  }
  #endif
  if (setting.outputMode == OUTPUT_BLE){ //output over ble-connection
    if (xHandleBluetooth){
      if ((ble_data.length() + data.length()) <512){
        while(ble_mutex){
          delay(100);
        };
        ble_data=ble_data+data;
      }else{
        if (!ble_mutex){
          ble_data="";
        }
      }
    }else{
      ble_data = "";
      ble_mutex = false;
    }
  }
}

/*
void writeTrackingData(uint32_t tAct){
  static uint32_t tSend = millis();
  if (status.GPS_Fix == 1){
    if ((tAct - tSend) >= 5000){
      MyFanetData.aircraftType = fanet.getAircraftType();
      fanet.setMyTrackingData(&MyFanetData);
      tSend = tAct;
    }
  }else{
    tSend = tAct;
  }
}
*/



static void IRAM_ATTR AXP192_Interrupt_handler() {
  AXP192_Irq = true;
  log_v("AXP192 IRQ");
}

void setupAXP192(){
  i2cOLED.beginTransmission(AXP192_SLAVE_ADDRESS);
  if (i2cOLED.endTransmission() == 0) {
    if (!axp.begin(i2cOLED, AXP192_SLAVE_ADDRESS)){
      axp.setChgLEDMode(AXP20X_LED_LOW_LEVEL);

      axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF); // GPS main power
      delay(500);
      axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
      axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
      axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
      axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON); // NC
      axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);

      axp.setDCDC1Voltage(3300); //       AXP192 power-on value: 3300
      axp.setLDO2Voltage (3300); // LoRa, AXP192 power-on value: 3300
      axp.setLDO3Voltage (3000); // GPS,  AXP192 power-on value: 2800

      pinMode(AXP_IRQ, INPUT_PULLUP);

      attachInterrupt(digitalPinToInterrupt(AXP_IRQ),
                      AXP192_Interrupt_handler, FALLING);

      axp.enableIRQ(AXP202_PEK_LONGPRESS_IRQ | AXP202_PEK_SHORTPRESS_IRQ, true);
      axp.clearIRQ();
      //axp.setChgLEDMode(AXP20X_LED_BLINK_1HZ);
      log_i("init AXP192 --> ready");
      status.bHasAXP192 = true;
    }else{
      log_e("AXP192 error begin");
      status.bHasAXP192 = false;
    }

  }else{
    log_e("AXP192 not found");
    status.bHasAXP192 = false;
  }
}

void IRAM_ATTR ppsHandler(void){
  ppsTriggered = true;
}

void WiFiEvent(WiFiEvent_t event){
  switch(event){
    case SYSTEM_EVENT_STA_START:
      log_i("SYSTEM_EVENT_STA_START");
      break;
    case SYSTEM_EVENT_ETH_START:
      log_i("SYSTEM_EVENT_ETH_START");
      break;
    case SYSTEM_EVENT_AP_START:
      log_d("AP started. IP: [%s]", WiFi.softAPIP().toString().c_str() );
      break;
    case SYSTEM_EVENT_AP_STOP:
      log_d("AP Stopped");
      break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
      //newStationConnected = true;
      log_d("WiFi Client Disconnected");
      break;
    case SYSTEM_EVENT_AP_STACONNECTED:
      log_d("SYSTEM_EVENT_AP_STACONNECTED");
      break;
    case SYSTEM_EVENT_STA_GOT_IP:
      //log_i("SYSTEM_EVENT_STA_GOT_IP!!!!!!!!!!!!");
      status.myIP = WiFi.localIP().toString();
      log_i("my IP=%s",status.myIP.c_str());
      status.wifiStat=2;
      break;
    case SYSTEM_EVENT_AP_STAIPASSIGNED:
      //newStationConnected = true;
      log_i("SYSTEM_EVENT_AP_STAIPASSIGNED!!!!!!!!!!!!");
      break;

    default:
      log_d("Unhandled WiFi Event: %d", event );
      break;
  }
}



void setupWifi(){
  while(host_name.length() == 0){
    delay(100); //wait until we have the devid
  }
  status.wifiStat = 0;
  WiFi.mode(WIFI_OFF);
  //delay(500);
  WiFi.persistent(false);
  WiFi.onEvent(WiFiEvent);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE); // call is only a workaround for bug in WiFi class
  log_i("Setting soft-AP ... ");
  //if (WiFi.softAP(host_name.c_str(), setting.appw.c_str(),rand() % 12 + 1,0,2)){
  if (WiFi.softAP(host_name.c_str(), setting.wifi.appw.c_str())){
    log_i("Ready");
  }else{
    log_i("Failed!");
  }
  delay(10);
  log_i("Setting soft-AP configuration ... ");
  if(WiFi.softAPConfig(local_IP, gateway, subnet)){
    log_i("Ready");
  }else{
    log_i("Failed!");
  }
  delay(10);

  //now configure access-point
  //so we have wifi connect and access-point at same time
  //we connecto to wifi
  if (setting.wifi.connect != WIFI_CONNECT_NONE){
    //esp_wifi_set_auto_connect(true);
    log_i("Try to connect to WiFi ...");
    WiFi.status();
    WiFi.mode(WIFI_MODE_APSTA);
    if ((WiFi.SSID() != setting.wifi.ssid || WiFi.psk() != setting.wifi.password)){
      // ... Try to connect to WiFi station.
      WiFi.begin(setting.wifi.ssid.c_str(), setting.wifi.password.c_str());
    } else {
      // ... Begin with sdk config.
      WiFi.begin();
    }
  }
  log_i("hostname=%s",host_name.c_str());
  WiFi.setHostname(host_name.c_str());

  log_i("my APIP=%s",local_IP.toString().c_str());
  status.wifiStat = 1;
  Web_setup();
}

#ifdef OLED
//Initialize OLED display
void startOLED(){
  /*
  //reset OLED display via software
  if (PinOledRst >= 0){
    log_i("Heltec-board");
    pinMode(PinOledRst, OUTPUT);
    digitalWrite(PinOledRst, LOW);
    delay(100);
    digitalWrite(PinOledRst, HIGH);
    delay(100);
  }

  //initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x32
    log_e("SSD1306 allocation failed");
    for(;;); // Don't proceed, loop forever
  }
  */
  oledPowerOn();
  if (startOption == 0){
    display.setTextColor(WHITE);
    display.clearDisplay();
    display.drawXBitmap(0,2,G_Logo_bits,G_Logo_width,G_Logo_height,WHITE);
    display.display();
    delay(1000);
    display.drawXBitmap(30,2,X_Logo_bits,X_Logo_width,X_Logo_height,WHITE);
    display.display();
    delay(1000);
    display.drawXBitmap(69,2,AirCom_Logo_bits,AirCom_Logo_width,AirCom_Logo_height,WHITE);
    display.setTextSize(1);
    display.setCursor(85,55);
    display.print(VERSION);
    display.display();
  }else{
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(2);
    display.setCursor(10,5);
    display.print("Switch to");
    display.setCursor(10,30);
    display.print("Bluetooth");
    display.display();

  }
}
#endif

void printSettings(){
  log_i("**** SETTINGS ****");
  log_i("Access-point password=%s",setting.wifi.appw.c_str());
  log_i("Board-Type=%d",setting.boardType);
  log_i("batt-type=%d",setting.BattType);
  log_i("Display-Type=%d",setting.displayType);
  //log_i("AXP192=%d",uint8_t(status.bHasAXP192));
  if (setting.band == 0){
    log_i("BAND=868mhz");
  }else{
    log_i("BAND=915mhz");
  }
  log_i("BAND=%d",setting.band);
  log_i("LORA_POWER=%d",setting.LoraPower);
  log_i("Mode=%d",setting.Mode);
  log_i("Fanet-Mode=%d",setting.fanetMode);
  log_i("Fanet-Pin=%d",setting.fanetpin);
  log_i("external power switch=%d",setting.bHasExtPowerSw);


  log_i("Serial-output=%d",setting.bOutputSerial);
  log_i("OUTPUT LK8EX1=%d",setting.outputLK8EX1);
  log_i("OUTPUT FLARM=%d",setting.outputFLARM);
  log_i("OUTPUT GPS=%d",setting.outputGPS);
  log_i("OUTPUT FANET=%d",setting.outputFANET);
  
  log_i("WIFI connect=%d",setting.wifi.connect);
  log_i("WIFI SSID=%s",setting.wifi.ssid.c_str());
  log_i("WIFI PW=%s",setting.wifi.password.c_str());
  log_i("Aircraft=%s",fanet.getAircraftType(setting.AircraftType).c_str());
  log_i("Pilotname=%s",setting.PilotName.c_str());
  log_i("Wifi-down-time=%d",setting.wifi.tWifiStop);
  log_i("Output-Mode=%d",setting.outputMode);
  log_i("UDP_SERVER=%s",setting.UDPServerIP.c_str());
  log_i("UDP_PORT=%d",setting.UDPSendPort);
  log_i("UDP_SERVER=%s",setting.UDPServerIP.c_str());
  log_i("UDP_PORT=%d",setting.UDPSendPort);
  
  //ground-station
  log_i("GS LAT=%02.6f",setting.gs.lat);
  log_i("GS LON=%02.6f",setting.gs.lon);
  log_i("GS ALT=%0.2f",setting.gs.alt);
  log_i("GS SCREEN OPTION=%d",setting.gs.SreenOption);
  log_i("GS POWERSAFE=%d",setting.gs.PowerSave);
  
  
  log_i("Airwhere-Livetracking=%d",setting.awLiveTracking);
  log_i("OGN-Livetracking=%d",setting.OGNLiveTracking);
  log_i("Traccar-Livetracking=%d",setting.traccarLiveTracking);
  log_i("Traccar-Address=%s",setting.TraccarSrv.c_str());
  log_i("Legacy-TX=%d",setting.LegacyTxEnable);
  
  //vario
  log_i("VarioSinkingThreshold=%0.2f",setting.vario.sinkingThreshold);
  log_i("VarioClimbingThreshold=%0.2f",setting.vario.climbingThreshold);
  log_i("VarioNearClimbingSensitivity=%0.2f",setting.vario.nearClimbingSensitivity);
  log_i("VarioVolume=%d",setting.vario.volume);
  log_i("Vario use MPU=%d",setting.vario.useMPU);
  log_i("Vario temp offset=%.02f",setting.vario.tempOffset);

  //weather-data
  log_i("WD Fanet-Weatherdata=%d",setting.wd.sendFanet);
  log_i("WD tempoffset=%.1f",setting.wd.tempOffset);
  log_i("WD windDirOffset=%.1f",setting.wd.windDirOffset);
  log_i("WD windDirAvgFactor=%d",setting.wd.windDirAvgFactor);
  log_i("WUUlEnable=%d",setting.WUUpload.enable);
  log_i("WUUlID=%s",setting.WUUpload.ID.c_str());
  log_i("WUUlKEY=%s",setting.WUUpload.KEY.c_str());
  log_i("WIUlEnable=%d",setting.WindyUpload.enable);
  log_i("WIUlID=%s",setting.WindyUpload.ID.c_str());
  log_i("WIUlKEY=%s",setting.WindyUpload.KEY.c_str());

  #ifdef GSM_MODULE
  //GSM
  log_i("GSM APN=%s",setting.gsm.apn.c_str());
  log_i("GSM USER=%s",setting.gsm.user.c_str());
  log_i("GSM PWD=%s",setting.gsm.pwd.c_str());
  #endif

}

void listSpiffsFiles(){
  File root = SPIFFS.open("/");
  File file = root.openNextFile();
  while(file){  
    log_i("FILE: %s",file.name());

    file = root.openNextFile();
  }  
}

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
esp_sleep_wakeup_cause_t print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_UNDEFINED : log_i("wakeup undefined --> possible by reset"); break;
    case ESP_SLEEP_WAKEUP_EXT0 : log_i("Wakeup EXT0"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : log_i("Wakeup EXIT1"); break;
    case ESP_SLEEP_WAKEUP_TIMER : log_i("Wakeup TIMER"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : log_i("Wakeup TOUCHPAD"); break;
    case ESP_SLEEP_WAKEUP_ULP : log_i("Wakeup ULP"); break;
    default : log_i("Wakeup was not caused by deep sleep: %d",wakeup_reason); break;
  }
  return wakeup_reason;
}

/*
void startBluetooth(void){
  if (setting.outputMode == OUTPUT_BLE){
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);    
    //log_i("currHeap:%d,minHeap:%d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
  }else if (setting.outputMode == OUTPUT_BLUETOOTH){
    //esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    //esp_bt_controller_init(&cfg);
    esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    log_i("starting bluetooth_serial %s",host_name.c_str());
    SerialBT.begin(host_name.c_str()); //Bluetooth device name
    log_i("currHeap:%d,minHeap:%d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
    status.bluetoothStat = 1;
  }
}
*/

void setup() {
  
  
  // put your setup code here, to run once:  
  //Serial.begin(57600);
  Serial.begin(115200);

  status.bPowerOff = false;
  status.vario.bHasBME = false;
  status.bWUBroadCast = false;
  status.bInternetConnected = false;
  status.bTimeOk = false;
  status.modemstatus = MODEM_DISCONNECTED;
  setting.bConfigGPS = false;
  status.bHasGPS = true; //we have our own GPS-Module
  status.tRestart = 0;

  //log_e("error");

  /*
  while(1){
    delay(10);
  }
  */


  log_i("SDK-Version=%s",ESP.getSdkVersion());
  log_i("CPU-Speed=%d",ESP.getCpuFreqMHz());
  log_i("Total heap: %d", ESP.getHeapSize());
  log_i("Free heap: %d", ESP.getFreeHeap());

  #ifdef BOARD_HAS_PSRAM
  if (psramFound()){
    psRamSize = ESP.getPsramSize();
    log_i("Total PSRAM: %d", psRamSize);
    log_i("Free PSRAM: %d", ESP.getFreePsram());
  }else{
    psRamSize = 0;
    log_i("No PSRAM found");
  }
  #endif
  printChipInfo();
  log_i("compiled at %s",compile_date);
  log_i("current free heap: %d, minimum ever free heap: %d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());

  //esp_sleep_wakeup_cause_t reason = print_wakeup_reason(); //print reason for wakeup
  esp_sleep_wakeup_cause_t reason2 = print_wakeup_reason(); //print reason for wakeup
  esp_reset_reason_t reason = esp_reset_reason();
  if (reason != ESP_RST_SW) {
    startOption = 0;
    detectOption = 0;
  }
  log_i("startOption=%d",startOption);

    // Make sure we can read the file system
  if( !SPIFFS.begin(true)){
    log_e("Error mounting SPIFFS");
    while(1);
  }
  log_i("SPIFFS total=%d used=%d free=%d",SPIFFS.totalBytes(),SPIFFS.usedBytes(),SPIFFS.totalBytes()-SPIFFS.usedBytes());

  //listSpiffsFiles();
  load_configFile(&setting); //load configuration
  if (setting.boardType == BOARD_UNKNOWN){
    checkBoardType();
  }  
  #if defined(OLED) && !defined(EINK)
  setting.displayType = OLED0_96;
  #elif defined(EINK) && !defined(OLED)
  setting.displayType = EINK2_9;
  #elif !defined(OLED) && !defined(EINK)
  setting.displayType = NO_DISPLAY;
  #endif
  status.displayType = setting.displayType; //we have to copy the display-type in case, setting is changing
  #if defined(GSMODULE)  && ! defined(AIRMODULE)
    log_i("only GS-Mode compiled");
    setting.Mode = MODE_GROUND_STATION;
  #endif
  #if defined(AIRMODULE) && ! defined(GSMODULE)
    log_i("only Air-Mode compiled");
    setting.Mode = MODE_AIR_MODULE;
  #endif
  status.bHasGSM = false;
  #ifdef GSM_MODULE
    status.bHasGSM = true;
  #endif

  //esp_wifi_stop();
  /*
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(12, OUTPUT);
  digitalWrite(12,HIGH); 
  delay(500);
  digitalWrite(4,LOW); 
  digitalWrite(5,LOW); 
  digitalWrite(12,LOW); 
  delay(500);
  log_i("start sleeping");
  delay(500);
  //Serial.end();
  esp_bluedroid_disable();
  esp_bluedroid_deinit();
  esp_bt_controller_disable();
  esp_bt_controller_deinit();
  esp_bt_mem_release(ESP_BT_MODE_BTDM);
  adc_power_off();  
  esp_deep_sleep_start();
  */


  #ifdef GSMODULE
  if (setting.Mode == MODE_GROUND_STATION){
    if ((setting.gs.PowerSave == GS_POWER_SAFE) && (reason2 == ESP_SLEEP_WAKEUP_TIMER)){
      printLocalTime();
      if (isDayTime() == 0){
        log_i("not day --> enter deep-sleep");
        uint32_t tSleep = calcSleepTime();
        //log_i("time to sleep = %d",tSleep);
        log_i("time to sleep = %d --> %02d:%02d:%02d",tSleep,tSleep/60/60,(tSleep/60)%60,(tSleep)%60);
        esp_sleep_enable_timer_wakeup((uint64_t)tSleep * uS_TO_S_FACTOR); //set Timer for wakeup      
        //esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR); //set Timer for wakeup
        esp_wifi_stop();
        esp_bluedroid_disable();
        esp_bluedroid_deinit();
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
        esp_bt_mem_release(ESP_BT_MODE_BTDM);
        adc_power_off();
        esp_deep_sleep_start();
      }
    }
  }
  #endif
  if ((setting.wifi.ssid.length() <= 0) || (setting.wifi.password.length() <= 0)){
    setting.wifi.connect = WIFI_CONNECT_NONE; //if no pw or ssid given --> don't connecto to wifi
  }

  pinMode(BUTTON2, INPUT_PULLUP);

  printSettings();
  analogReadResolution(10); // Default of 12 is not very linear. Recommended to use 10 or 11 depending on needed resolution.
  status.bHasAXP192 = false;    
  switch (setting.boardType)
  {
  case BOARD_T_BEAM: 
    log_i("Board=T_BEAM");
    PinLoraRst = 23;
    PinLoraDI0 = 26;
    PinLora_SS = 18;
    PinLora_MISO = 19;
    PinLora_MOSI = 27;
    PinLora_SCK = 5;

    PinOledRst = -1;
    PinOledSDA = 21;
    PinOledSCL = 22;

    PinBaroSDA = 13;
    PinBaroSCL = 14;

    
    PinUserLed = 4;

    //V3.0.0 changed from PIN 0 to PIN 25
    PinBuzzer = 25;

    if (setting.bHasExtPowerSw){
      PinExtPowerOnOff = 36;
    }

    i2cOLED.begin(PinOledSDA, PinOledSCL);
    setupAXP192();
    //for new T-Beam output 4 is red led

    break;
  case BOARD_T_BEAM_V07:
    log_i("Board=T_BEAM_V07/TTGO_T3_V1_6");
    PinLoraRst = 23;
    PinLoraDI0 = 26;
    PinLora_SS = 18;
    PinLora_MISO = 19;
    PinLora_MOSI = 27;
    PinLora_SCK = 5;

    PinOledRst = -1;
    PinOledSDA = 21;
    PinOledSCL = 22;

    PinBaroSDA = 13;
    PinBaroSCL = 14;

    PinADCVoltage = 35;

    PinBuzzer = 0;

    i2cOLED.begin(PinOledSDA, PinOledSCL);
    // voltage-divier 100kOhm and 100kOhm
    // vIn = (R1+R2)/R2 * VOut
    adcVoltageMultiplier = 2.0f * 3.76f; // not sure if it is ok ?? don't have this kind of board
    pinMode(PinADCVoltage, INPUT);
    break;
  /*
  case BOARD_TTGO_T3_V1_6:
    log_i("Board=TTGO_T3_V1_6");
    PinLoraRst = 23;
    PinLoraDI0 = 26;
    PinLora_SS = 18;
    PinLora_MISO = 19;
    PinLora_MOSI = 27;
    PinLora_SCK = 5;

    PinOledRst = -1;
    PinOledSDA = 21;
    PinOledSCL = 22;

    PinBaroSDA = 13;
    PinBaroSCL = 14;

    PinADCVoltage = 35;

    PinBuzzer = 0;

    i2cOLED.begin(PinOledSDA, PinOledSCL);
    // voltage-divier 100kOhm and 100kOhm
    // vIn = (R1+R2)/R2 * VOut
    adcVoltageMultiplier = 2.0f * 3.76f; // not sure if it is ok ?? don't have this kind of board
    pinMode(PinADCVoltage, INPUT);
    break;
  */
  case BOARD_HELTEC_LORA:
    log_i("Board=HELTEC_LORA");
    PinLoraRst = 14;
    PinLoraDI0 = 26;
    PinLora_SS = 18;
    PinLora_MISO = 19;
    PinLora_MOSI = 27;
    PinLora_SCK = 5;
    PinGsmRst = 25;
    PinGsmTx = 12;
    PinGsmRx = 21;

    PinOledRst = 16;
    PinOledSDA = 4;
    PinOledSCL = 15;

    PinBaroSDA = 13;
    PinBaroSCL = 23;

    PinOneWire = 22;    

    PinWindDir = 36;
    PinWindSpeed = 37;
    PinRainGauge = 38;

    PinADCVoltage = 34;

    PinBuzzer = 17;

    i2cOLED.begin(PinOledSDA, PinOledSCL);
    // voltage-divier 27kOhm and 100kOhm
    // vIn = (R1+R2)/R2 * VOut
    if (setting.BattType == BATT_TYPE_12V_LEAD){
      battFull = 12500;
      battEmpty = 10800;
      adcVoltageMultiplier = (100000.0f + 24000.0f) / 100000.0f * 15.6;
    }else{
      //1S LiPo
      adcVoltageMultiplier = (100000.0f + 27000.0f) / 100000.0f * 3.3;
    }
    pinMode(PinADCVoltage, INPUT); //input-Voltage on GPIO34
    break;
  case BOARD_TTGO_TSIM_7000:
    log_i("Board=TTGO_TSIM_7000");
    PinLoraRst = 12;
    PinLoraDI0 = 32;
    PinLora_SS = 5;
    //pinMode(5, OUTPUT);
    //digitalWrite(5,LOW); 
    //PinLora_SS = 16; //unused Pin, but pin5 is also for reset of GSM
    PinLora_MISO = 19;
    PinLora_MOSI = 23;
    PinLora_SCK = 18;
    PinGsmRst = 4; //is PowerKey, but IO5 is covered by Lora SS
    PinGsmTx = 27;
    PinGsmRx = 26;
    //PinGsmGPSPower = 4; //we don't need it.

    PinOledRst = -1; //no oled-support yet
    PinOledSDA = -1;
    PinOledSCL = -1;

    PinBaroSDA = 21;
    PinBaroSCL = 22;

    PinOneWire = 25;    

    PinWindDir = 33;
    PinWindSpeed = 34;
    PinRainGauge = 39;

    PinADCVoltage = 35;
    
    //PinUserLed = 12; //PinLoraRst
    
    PinBuzzer = 0;

    // voltage-divier 100kOhm and 100kOhm
    // vIn = (R1+R2)/R2 * VOut
    adcVoltageMultiplier = 2.0f * 3.76f; // not sure if it is ok ?? don't have this kind of board
    pinMode(PinADCVoltage, INPUT);

    break;
  case BOARD_UNKNOWN:
    log_e("unknown Board --> please correct");
    break;
  default:
    log_e("wrong-board-definition --> restart");
    setting.boardType = BOARD_UNKNOWN;    
    write_configFile(&setting);
    delay(1000);
    esp_restart(); //we need to restart    break;
  }
  if (PinUserLed >= 0){
    pinMode(PinUserLed, OUTPUT);
    digitalWrite(PinUserLed,HIGH); 
  }
  if (PinExtPowerOnOff >= 0){
    pinMode(PinExtPowerOnOff, INPUT);
    log_i("ext power-state=%d",digitalRead(PinExtPowerOnOff));
  }

  #ifdef OLED
  if (status.displayType == OLED0_96){
    startOLED();  
  }
  #endif
  setting.myDevId = "";
#ifdef GSM_MODULE
xGsmMutex = xSemaphoreCreateMutex();
#endif

xOutputMutex = xSemaphoreCreateMutex();

  //log_i("currHeap:%d,minHeap:%d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
#ifdef AIRMODULE
  if (setting.Mode == MODE_AIR_MODULE){
    xTaskCreatePinnedToCore(taskBaro, "taskBaro", 6500, NULL, 100, &xHandleBaro, ARDUINO_RUNNING_CORE1); //high priority task
  }
#endif  
  //log_i("currHeap:%d,minHeap:%d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
  xTaskCreatePinnedToCore(taskStandard, "taskStandard", 6500, NULL, 10, &xHandleStandard, ARDUINO_RUNNING_CORE1); //standard task
  //log_i("currHeap:%d,minHeap:%d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
#ifdef EINK
  xTaskCreatePinnedToCore(taskEInk, "taskEInk", 6500, NULL, 8, &xHandleEInk, ARDUINO_RUNNING_CORE1); //background EInk
#endif  
  //log_i("currHeap:%d,minHeap:%d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
  xTaskCreatePinnedToCore(taskBackGround, "taskBackGround", 6500, NULL, 5, &xHandleBackground, ARDUINO_RUNNING_CORE1); //background task
  //log_i("currHeap:%d,minHeap:%d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
  //xTaskCreatePinnedToCore(taskMemory, "taskMemory", 4096, NULL, 1, &xHandleMemory, ARDUINO_RUNNING_CORE1);
  //log_i("currHeap:%d,minHeap:%d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
  xTaskCreatePinnedToCore(taskBluetooth, "taskBluetooth", 4096, NULL, 7, &xHandleBluetooth, ARDUINO_RUNNING_CORE1);

#ifdef GSMODULE  
  if (setting.Mode == MODE_GROUND_STATION){
    //start weather-task
    xTaskCreatePinnedToCore(taskWeather, "taskWeather", 4096, NULL, 8, &xHandleWeather, ARDUINO_RUNNING_CORE1);
  }
#endif  
#ifdef GSM_MODULE
  //start Gsm-task
  xTaskCreatePinnedToCore(taskGsm, "taskGsm", 4096, NULL, 6, &xHandleGsm, ARDUINO_RUNNING_CORE1);
#endif

}


#ifdef GSM_MODULE

bool connectGPRS(){
  log_i("Connecting to internet");
  return modem.gprsConnect(setting.gsm.apn.c_str(), setting.gsm.user.c_str(), setting.gsm.pwd.c_str());

}


bool initModem(){
  //reset modem
  if (PinGsmRst >= 0){
    log_i("reset modem");
    digitalWrite(PinGsmRst,LOW);
    delay(500); //wait200ms
    digitalWrite(PinGsmRst,HIGH);
    delay(6000); //wait until modem is ok now
  }  
  log_i("test modem-connection");
  if (!modem.testAT()) return false;
  log_i("restarting modem...");  
  if (!modem.restart()) return false;
  #ifdef TINY_GSM_MODEM_SIM7000
    modem.disableGPS();
  #endif
  modem.sleepEnable(false); //set sleepmode off
  log_i("Waiting for network...");
  if (!modem.waitForNetwork(120000L)) return false;
  log_i("signal quality %d",modem.getSignalQuality());
  if (!connectGPRS()) return false;
  log_i("connected successfully");
  return true;
}

void taskGsm(void *pvParameters){  
  if (setting.wifi.connect != MODE_WIFI_DISABLED){
    //if (PinGsmRst >= 0){
    //  digitalWrite(PinGsmRst,LOW);
    //}
    log_i("stop task");
    vTaskDelete(xHandleGsm); //delete weather-task
    return;
  }
  
  if (PinGsmGPSPower >= 0){
    pinMode(PinGsmGPSPower, OUTPUT); //set GsmReset to output
    digitalWrite(PinGsmGPSPower,HIGH);
    delay(1000); //wait until hardware is stable
  }

  if (PinGsmRst >= 0){
    pinMode(PinGsmRst, OUTPUT); //set GsmReset to output
    log_i("reset modem");
    digitalWrite(PinGsmRst,LOW);
    delay(500); //wait200ms
    digitalWrite(PinGsmRst,HIGH);
    delay(3000); //wait until modem is ok now
  } 
  /*
  #ifdef TINY_GSM_MODEM_SIM7000
    GsmSerial.begin(115200,SERIAL_8N1,PinGsmRx,PinGsmTx,false); //baud, config, rx, tx, invert
  #else
    //GsmSerial.begin(9600,SERIAL_8N1,PinGsmRx,PinGsmTx,false); //baud, config, rx, tx, invert
  #endif
  */
  GsmSerial.begin(115200,SERIAL_8N1,PinGsmRx,PinGsmTx,false); //baud, config, rx, tx, invert
  const TickType_t xDelay = 5000 / portTICK_PERIOD_MS;   //only every 1sek.
  TickType_t xLastWakeTime = xTaskGetTickCount (); //get actual tick-count
  //bool status;
  while(1){
    xSemaphoreTake( xGsmMutex, portMAX_DELAY );
    if (modem.isGprsConnected()){
      status.modemstatus = MODEM_CONNECTED;
      xLastWakeTime = xTaskGetTickCount (); //get actual tick-count
      status.GSMSignalQuality = modem.getSignalQuality();
      /*
      modem.sendAT(GF("+CNSMOD?"));      
      String res;
      if (modem.waitResponse(GF("+CNSMOD:")) == 1){
        modem.streamSkipUntil(',');  // Skip context id
        String res = modem.stream.readStringUntil('\r');
        log_i("network system mode %s",res.c_str());
      }
      */
    }else{
      status.modemstatus = MODEM_CONNECTING;
      if (modem.isNetworkConnected()){  
        connectGPRS();
      }else{
        initModem(); //init modem
      }
    }
    xSemaphoreGive( xGsmMutex );
    //if ((WebUpdateRunning) || (bGsmOff)) break;
    if (bGsmOff) break; //we need GSM for webupdate
    //delay(1);
    vTaskDelayUntil( &xLastWakeTime, xDelay); //wait until next cycle
  }
  //modem.stop(15000L);
  status.modemstatus = MODEM_DISCONNECTED;
  xSemaphoreTake( xGsmMutex, portMAX_DELAY );
  
  log_i("stop gprs connection");
  modem.gprsDisconnect();
  log_i("switch radio off");
  modem.radioOff();  

  #ifdef TINY_GSM_MODEM_SIM7000
    digitalWrite(5,HIGH);
  #endif
  //digitalWrite(PinGsmRst,HIGH);
  
  #ifdef TINY_GSM_MODEM_SIM7000
    //Power-off SIM7000-module
    for (int i = 0; i < 10;i++){
      log_i("power off modem");
      if (modem.poweroff()) break;
      delay(1000);
    }
  #else
    //on SIM800L we can't complete power down the module
    //it restarts itself all the time --> we set the module to sleep-mode
    log_i("set modem to sleep-mode");
    modem.sendAT(GF("+CSCLK=2"));
    delay(1000);
  #endif
  xSemaphoreGive( xGsmMutex );
  //GsmSerial.end();
  //if (PinGsmRst >= 0){
  //  digitalWrite(PinGsmRst,LOW);
  //}
  log_i("stop task");
  vTaskDelete(xHandleGsm); //delete weather-task
}

#endif

#ifdef GSMODULE
void taskWeather(void *pvParameters){
  static uint32_t tUploadData =  0;
  static uint32_t tSendData = millis() - WEATHER_UPDATE_RATE + 30000; //first sending is in 10 seconds
  bool bDataOk = false;
  log_i("starting weather-task ");  
  Weather::weatherData wData;
  TwoWire i2cWeather = TwoWire(0);
  i2cWeather.begin(PinBaroSDA,PinBaroSCL,200000); //init i2cBaro for Baro
  Weather weather;
  weather.setTempOffset(setting.wd.tempOffset);
  weather.setWindDirOffset(setting.wd.windDirOffset);
  weather.setWindDirAvgFactor(setting.wd.windDirAvgFactor);
  if (weather.begin(&i2cWeather,setting.gs.alt,PinOneWire,PinWindDir,PinWindSpeed,PinRainGauge)){
    status.vario.bHasBME = true; //we have a bme-sensor
  }
  if ((setting.WUUpload.enable) && (!status.vario.bHasBME)){
    status.bWUBroadCast = true;
    log_i("wu broadcast enabled");
  }
  if ((!status.vario.bHasBME) && (!status.bWUBroadCast)){
    log_i("stopping task");
    vTaskDelete(xHandleWeather);
    return;    
  }
  const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;   //only every 1sek.
  TickType_t xLastWakeTime = xTaskGetTickCount (); //get actual tick-count
  while (1){
    uint32_t tAct = millis();
    if (tUploadData == 0){
      //first time sending if we have internet and time is ok
      if ((status.bInternetConnected) && (status.bTimeOk)){
        tUploadData = tAct - WEATHER_UNDERGROUND_UPDATE_RATE + 2000;
      }
    }
    if (status.vario.bHasBME){
      //station has BME --> we are a weather-station
      weather.run();
      weather.getValues(&wData);
      status.weather.temp = wData.temp;
      status.weather.Humidity = wData.Humidity;
      status.weather.Pressure = wData.Pressure;
      status.weather.WindDir = wData.WindDir;
      status.weather.WindSpeed = wData.WindSpeed;
      status.weather.WindGust = wData.WindGust;
      status.weather.rain1h = wData.rain1h;
      status.weather.rain1d = wData.rain1d;
      if (timeOver(tAct,tUploadData,WEATHER_UNDERGROUND_UPDATE_RATE)){
        tUploadData = tAct;
        if ((status.bInternetConnected) && (status.bTimeOk)){
          if (setting.WUUpload.enable){
            WeatherUnderground::wData wuData;
            WeatherUnderground wu;
            #ifdef GSM_MODULE
              if (setting.wifi.connect == MODE_WIFI_DISABLED){
                wu.setClient(&GsmWUClient);
                wu.setMutex(&xGsmMutex);
              }
            #endif
            //log_i("temp=%f,humidity=%f",testWeatherData.temp,testWeatherData.Humidity);
            wuData.bWind = true;
            wuData.winddir = wData.WindDir;
            wuData.windspeed = wData.WindSpeed;
            wuData.windgust = wData.WindGust;
            wuData.humidity = wData.Humidity;
            wuData.temp = wData.temp;
            wuData.pressure = wData.Pressure;
            wuData.bRain = true;
            wuData.rain1h = wData.rain1h ;
            wuData.raindaily = wData.rain1d;
            wu.sendData(setting.WUUpload.ID,setting.WUUpload.KEY,&wuData);
          }
          if (setting.WindyUpload.enable){
            Windy::wData wiData;
            Windy wi;
            #ifdef GSM_MODULE
              if (setting.wifi.connect == MODE_WIFI_DISABLED){
                wi.setClient(&GsmWUClient);
                wi.setMutex(&xGsmMutex);
              }
            #endif
            //log_i("temp=%f,humidity=%f",testWeatherData.temp,testWeatherData.Humidity);
            wiData.bWind = true;
            wiData.winddir = wData.WindDir;
            wiData.windspeed = wData.WindSpeed;
            wiData.windgust = wData.WindGust;
            wiData.humidity = wData.Humidity;
            wiData.temp = wData.temp;
            wiData.pressure = wData.Pressure;
            wiData.bRain = true;
            wiData.rain1h = wData.rain1h ;
            wiData.raindaily = wData.rain1d;
            wi.sendData(setting.WindyUpload.ID,setting.WindyUpload.KEY,&wiData);
          }

        }
        weather.resetWindGust();
      }
      
      if (setting.wd.sendFanet){
        if (timeOver(tAct,tSendData,WEATHER_UPDATE_RATE)){
          
          testWeatherData.lat = setting.gs.lat;
          testWeatherData.lon = setting.gs.lon;
          testWeatherData.bWind = true;
          testWeatherData.wHeading = wData.WindDir;
          testWeatherData.wSpeed = wData.WindSpeed;
          testWeatherData.wGust = wData.WindGust;      
          testWeatherData.bTemp = wData.bTemp;
          testWeatherData.bHumidity = wData.bHumidity;
          testWeatherData.bBaro = wData.bPressure;
          testWeatherData.temp = wData.temp;
          testWeatherData.Humidity = wData.Humidity;
          testWeatherData.Baro = wData.Pressure;      
          testWeatherData.bStateOfCharge = true;
          testWeatherData.Charge = status.BattPerc;
          //testWeatherData.Charge = 44;
          sendWeatherData = true;
          tSendData = tAct;
        }
      }
    }
    if (status.bWUBroadCast){
      //station should broadcast WU-Data over Fanet
      if (timeOver(tAct,tUploadData,WEATHER_UNDERGROUND_UPDATE_RATE)){ //get Data from WU        
        tUploadData = tAct;
        WeatherUnderground::wData wuData;
        WeatherUnderground wu;
        #ifdef GSM_MODULE
          if (setting.wifi.connect == MODE_WIFI_DISABLED){
            wu.setClient(&GsmWUClient);
            wu.setMutex(&xGsmMutex);
          }
        #endif
        bDataOk = wu.getData(setting.WUUpload.ID,setting.WUUpload.KEY,&wuData);
        if (bDataOk){
          status.weather.temp = wuData.temp;
          status.weather.Humidity = wuData.humidity;
          status.weather.Pressure = wuData.pressure;
          status.weather.WindDir = wuData.winddir;
          status.weather.WindSpeed = wuData.windspeed;
          status.weather.WindGust = wuData.windgust;
          status.weather.rain1h = wuData.rain1h;
          status.weather.rain1d = wuData.raindaily;
          testWeatherData.lat = wuData.lat;
          testWeatherData.lon = wuData.lon;
          testWeatherData.bWind = true;
          testWeatherData.wHeading = wuData.winddir;
          testWeatherData.wSpeed = wuData.windspeed;
          testWeatherData.wGust = wuData.windgust;      
          testWeatherData.bTemp = true;
          testWeatherData.bHumidity = true;
          testWeatherData.bBaro = true;
          testWeatherData.temp = wuData.temp;
          testWeatherData.Humidity = wuData.humidity;
          testWeatherData.Baro = wuData.pressure;      
          testWeatherData.bStateOfCharge = true;  
          log_i("winddir=%.1f speed=%.1f gust=%.1f temp=%.1f hum=%.1f press=%.1f",testWeatherData.wHeading,testWeatherData.wSpeed,testWeatherData.wGust,testWeatherData.temp,testWeatherData.Humidity,testWeatherData.Baro);          
        }else{
          log_e("no Data from WU");
        }
      }
      if (timeOver(tAct,tSendData,WEATHER_UPDATE_RATE)){
        tSendData = tAct;
        if (bDataOk){
          testWeatherData.Charge = status.BattPerc;
          sendWeatherData = true;
        }
      }

    }
    if ((WebUpdateRunning) || (bPowerOff)) break;
    vTaskDelayUntil( &xLastWakeTime, xDelay); //wait until next cycle
    //delay(1);
  }
  log_i("stop task");
  vTaskDelete(xHandleWeather); //delete weather-task
}
#endif


#ifdef AIRMODULE
void taskBaro(void *pvParameters){
  uint8_t u8Volume = setting.vario.volume;
  log_i("starting baro-task ");  
  TickType_t xLastWakeTime;
  // Block for 500ms.
  const TickType_t xDelay = 10 / portTICK_PERIOD_MS;  
  status.vario.bHasMPU = false;

  ledcSetup(channel, freq, resolution);
  if (PinBuzzer >= 0){
    pinMode(PinBuzzer, OUTPUT);
    digitalWrite(PinBuzzer,LOW);
    ledcAttachPin(PinBuzzer, channel);
  }  
  
  Wire.begin(PinBaroSDA,PinBaroSCL,400000);
  //TwoWire i2cBaro = TwoWire(0);
  //i2cBaro.begin(PinBaroSDA,PinBaroSCL,400000); //init i2cBaro for Baro
  //if (baro.begin(&i2cBaro)){
  baro.useMPU(setting.vario.useMPU);
  uint8_t baroSensor = baro.begin(&Wire);
  if (baroSensor > 0){
    if (baroSensor == 2){
      status.vario.bHasMPU = true;
    }
    status.vario.bHasVario = true;
    Beeper.setThresholds(setting.vario.sinkingThreshold,setting.vario.climbingThreshold,setting.vario.nearClimbingSensitivity);
    Beeper.setVolume(u8Volume);
    Beeper.setGlidingBeepState(true);
    //Beeper.setGlidingAlarmState(true);
  }else{
    log_i("no baro found --> end baro-task ");  
    status.vario.bHasVario = false;    
  }
  if (status.vario.bHasVario){
    xLastWakeTime = xTaskGetTickCount ();
    while (1){      
      if (setting.vario.bCalibGyro){
        baro.calibGyro();
        setting.vario.bCalibGyro = false;
      }
      if (setting.vario.bCalibAcc){
        baro.calibAcc();
        setting.vario.bCalibAcc = false;
      }
      if (((!status.flying) && (setting.vario.BeepOnlyWhenFlying)) || (status.bMuting)){
        Beeper.setVolume(0);
      }else{
        Beeper.setVolume(setting.vario.volume);
      }
      baro.run();
      if (setting.vario.useMPU){
        baro.getMPUValues(&status.vario.accel[0],&status.vario.gyro[0],&status.vario.acc_Z);
      }
      if (baro.isNewVAlues()){
        baro.getValues(&status.pressure,&status.varioAlt,&status.ClimbRate,&status.varioTemp);
        status.varioTemp = status.varioTemp + setting.vario.tempOffset;
        status.varioHeading = baro.getHeading();
        #ifdef USE_BEEPER
        Beeper.setVelocity(status.ClimbRate);
        #endif
      }
      #ifdef USE_BEEPER
        Beeper.update();
      #endif      
      //delay(10);
      // Wait for the next cycle.
      //vTaskDelayUntil( &xLastWakeTime, xDelay );
      delay(1);
      if ((WebUpdateRunning) || (bPowerOff)) break;
    }
  }
  baro.end();
  log_i("stop task");
  vTaskDelete(xHandleBaro); //delete baro-task
}
#endif

void loop() {
  // put your main code here, to run repeatedly:
  //delay(1000); //wait 1second
}

void taskMemory(void *pvParameters) {

	while (1){
    uint32_t freeHeap = xPortGetFreeHeapSize();
    if (freeHeap < 50000){
      log_i("current free heap: %d, minimum ever free heap: %d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelay(1 / portTICK_PERIOD_MS); 
  }
}

void taskBluetooth(void *pvParameters) {

	// BLEServer *pServer;
  //Put a 10 second delay before Seral BT start to allow settings to work.
  if ((setting.outputMode == OUTPUT_BLUETOOTH))
    delay(10000);

  //esp_coex_preference_set(ESP_COEX_PREFER_BT);
  while(host_name.length() == 0){
    delay(100); //wait until we have the devid
  }
  if ((setting.outputMode == OUTPUT_BLE) || (setting.outputMode == OUTPUT_BLUETOOTH)){
    /*
    if ((psRamSize > 0) || (startOption == 1)){
      //startBluetooth(); //start bluetooth
    }else{
      log_i("stop task");
      vTaskDelete(xHandleBluetooth);
      return;    
    }
    */
  }else{
    //stop bluetooth-controller --> save some memory
    esp_bt_controller_disable();
    esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
    //log_i("currHeap:%d,minHeap:%d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
    log_i("stop task");
    vTaskDelete(xHandleBluetooth);
    return;    
  }

  if (setting.outputMode == OUTPUT_BLE){
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);    
    //log_i("currHeap:%d,minHeap:%d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());    
    start_ble(host_name+"-LE");
    status.bluetoothStat = 1;
	 while (1)
	 {
	   // only send if we have more than 31k free heap space.
	   if (xPortGetFreeHeapSize()>BLE_LOW_HEAP)
	   {
		   ble_low_heap_timer = millis();
		   if (ble_data.length()>0)
           {
			   ble_mutex=true;
			   BLESendChunks(ble_data);
			   ble_data="";
			   ble_mutex=false;
           }
	   }
	   else
	   {
		   log_d( " BLE congested - Waiting - Current free heap: %d, minimum ever free heap: %d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
		   ble_mutex=true;
		   log_d("CLEARING BLEOUT");
		   ble_data="";
		   ble_mutex=false;
	   }
	   vTaskDelay(100);
	 }
  }else if (setting.outputMode == OUTPUT_BLUETOOTH){
  #ifdef BLUETOOTHSERIAL 
    //esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    //esp_bt_controller_init(&cfg);
    esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    esp_bt_controller_mem_release(ESP_BT_MODE_BLE);    
    log_i("starting bluetooth_serial %s",host_name.c_str());
    SerialBT.begin(host_name.c_str()); //Bluetooth device name
    SerialBT.register_callback(&serialBtCallBack);
    log_i("currHeap:%d,minHeap:%d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
  #endif
    status.bluetoothStat = 1; //client disconnected
    while (1)
    {
      delay(1);
    }
  }

}

String setStringSize(String s,uint8_t sLen){
  uint8_t actLen = (uint8_t)s.length();
  String sRet = "";
  for (uint8_t i = actLen;i < sLen;i++){
    sRet += " ";
  }
  sRet += s;
  return sRet;
}

float readBattvoltage(){
  // multisample ADC
  const byte NO_OF_SAMPLES = 5;
  uint32_t adc_reading = 0;
  float vBatt = 0.0;

  analogRead(PinADCVoltage); // First measurement has the biggest difference on my board, this line just skips the first measurement
  for (int i = 0; i < NO_OF_SAMPLES; i++) {
    uint16_t thisReading = analogRead(PinADCVoltage);
    adc_reading += thisReading;
  }
  adc_reading /= NO_OF_SAMPLES;

  vBatt = adcVoltageMultiplier * float(adc_reading) / 1023.0f;
  //log_i("adc=%d, vBatt=%.3f",adc_reading,vBatt);
  return vBatt;
}

void printBattVoltage(uint32_t tAct){
  static uint32_t tBatt = millis() - 5000;
  if ((tAct - tBatt) >= 5000){
    tBatt = tAct;
    if (status.bHasAXP192){
      status.BattCharging = axp.isChargeing();
      //log_i("chargingstate=%d",status.BattCharging);
      if (axp.isBatteryConnect()) {
        status.vBatt = (uint16_t)axp.getBattVoltage();
      }else{
        //log_w("no Batt");
        status.vBatt = 0;
      }
    }else{
      status.vBatt = uint16_t(readBattvoltage()*1000);      
    }
    //log_i("Batt =%dV",status.vBatt);
    status.BattPerc = scale(status.vBatt,battEmpty,battFull,0,100);
    //log_i("Batt =%d%%",status.BattPerc);
  }
}

#ifdef OLED

void drawSignal(int16_t x, int16_t y,uint8_t strength) {
  if ((strength <= 9) && (strength >= 3)){
      display.drawBitmap(x,y,signal_1, SIGNALWIDTH, SIGNALHEIGHT, BLACK,WHITE);
  }else if ((strength >= 10) && (strength <= 14)){
      display.drawBitmap(x,y,signal_2, SIGNALWIDTH, SIGNALHEIGHT, BLACK,WHITE);
  }else if ((strength >= 15) && (strength <= 19)){
      display.drawBitmap(x,y,signal_3, SIGNALWIDTH, SIGNALHEIGHT, BLACK,WHITE);
  }else if ((strength >= 19) && (strength <= 30)){
      display.drawBitmap(x,y,signal_4, SIGNALWIDTH, SIGNALHEIGHT, BLACK,WHITE);
  }else{
      display.drawBitmap(x,y,signal_0, SIGNALWIDTH, SIGNALHEIGHT, BLACK,WHITE);
  }
  /*
  display.setTextSize(1); //set textsize
  display.setTextColor(WHITE,BLACK); //Draw white text Background Black
  display.setCursor(x+22,y+2);
  if (strength == 0){
      display.print("  ");
  }else{
      if (strength < 10) display.print(" ");
      display.print(strength);
  }
  display.display();
  */
}


void printWeather(uint32_t tAct){
  static uint32_t tRefresh = millis();
  static uint8_t screen = 0;
  if (timeOver(tAct,tRefresh,3000)){
    tRefresh = tAct;
    String s = "";
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0,0);
    display.print(setting.PilotName);
    drawWifiStat(status.wifiStat);
    drawBluetoothstat(101,0);
    drawBatt(111, 0,(status.BattCharging) ? 255 : status.BattPerc);
    if (status.bHasGSM){
      drawSignal(60,0,status.GSMSignalQuality);
    }    
    display.setTextSize(3); //set Textsize
    display.setCursor(0,21);
    switch(screen)
    {
      case 0: //battery-voltage
        s = String(status.vBatt / 1000.,2) + "V";
        break;
      case 1: //wind dir
        s = "  " + getWDir(status.weather.WindDir);
        break;
      case 2: //speed and gust
        s = String(round(status.weather.WindSpeed),0) + "|" + String(round(status.weather.WindGust),0); // + "kh";
        break;
      case 3: //temp
        s = " " + String(status.weather.temp,1) + "C";
        break;
      case 4: //pressure
        display.setCursor(0,26);
        display.setTextSize(2); //set Textsize
        s = " " + String(status.weather.Pressure,1) + "hPa";
        break;
      case 5: //humidity
        s = " " + String(status.weather.Humidity,0) + "%";
        break;
    }
    screen++;
    if (screen > 5) screen = 0;
    display.print(s);
    display.setTextSize(1);
    display.setCursor(0,55);
    display.print(setting.myDevId);
    display.setCursor(85,55);
    display.print(VERSION);
    display.display();
  }
}

void printScanning(uint32_t tAct){
  static uint8_t icon = 0;
  if (setting.gs.SreenOption == SCREEN_ON_WHEN_TRAFFIC){
    oledPowerOff();
    return;
  }
  oledPowerOn();
  display.clearDisplay();
  drawWifiStat(status.wifiStat);
  drawBluetoothstat(101,0);
  drawBatt(111, 0,(status.BattCharging) ? 255 : status.BattPerc);
  switch (icon)
  {
  case 0: 
    display.drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    break;
  case 1: 
    display.drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    display.drawXBitmap(42,10,WFTX_bits,WFTX_width,WFTX_height,WHITE);
    break;
  case 2: 
    display.drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    break;
  case 3: 
    display.drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    display.drawXBitmap(42,10,WFTX_bits,WFTX_width,WFTX_height,WHITE);
    display.drawXBitmap(66, 34, WFRX_bits,WFRX_width, WFRX_height,WHITE );
    display.drawXBitmap(88, 10, PGRX_bits,PGRX_width, PGRX_height,WHITE);      
    break;
  case 4: 
    display.drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    break;
  case 5: 
    display.drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    display.drawXBitmap(42,10,WFTX_bits,WFTX_width,WFTX_height,WHITE);
    display.drawXBitmap(66, 34, WFRX_bits,WFRX_width, WFRX_height,WHITE );
    display.drawXBitmap(88, 10, HGRX_bits,HGRX_width, HGRX_height,WHITE);      
    break;
  case 6: 
    display.drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    break;
  case 7: 
    display.drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    display.drawXBitmap(42,10,WFTX_bits,WFTX_width,WFTX_height,WHITE);
    display.drawXBitmap(66, 34, WFRX_bits,WFRX_width, WFRX_height,WHITE );
    display.drawXBitmap(88, 10, BLRX_bits,BLRX_width, BLRX_height,WHITE);      
    break;
  case 8: 
    display.drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    break;
  case 9: 
    display.drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    display.drawXBitmap(42,10,WFTX_bits,WFTX_width,WFTX_height,WHITE);
    display.drawXBitmap(66, 34, WFRX_bits,WFRX_width, WFRX_height,WHITE );
    display.drawXBitmap(88, 10, SPRX_bits,SPRX_width, SPRX_height,WHITE);      
    break;
  case 10: 
    display.drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    break;
  case 11: 
    display.drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    display.drawXBitmap(42,10,WFTX_bits,WFTX_width,WFTX_height,WHITE);
    display.drawXBitmap(66, 34, WFRX_bits,WFRX_width, WFRX_height,WHITE );
    display.drawXBitmap(88, 10, Airplane40_bits,Airplane40_width, Airplane40_height,WHITE);      
    break;
  case 12: 
    display.drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    break;
  case 13: 
    display.drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    display.drawXBitmap(42,10,WFTX_bits,WFTX_width,WFTX_height,WHITE);
    display.drawXBitmap(66, 34, WFRX_bits,WFRX_width, WFRX_height,WHITE );
    display.drawXBitmap(88, 10, Helicopter40_bits,Helicopter40_width, Helicopter40_height,WHITE);      
    break;
  case 14: 
    display.drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    break;
  case 15: 
    display.drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    display.drawXBitmap(42,10,WFTX_bits,WFTX_width,WFTX_height,WHITE);
    display.drawXBitmap(66, 34, WFRX_bits,WFRX_width, WFRX_height,WHITE );
    display.drawXBitmap(88, 10, UAVRX_bits,UAVRX_width, UAVRX_height,WHITE);      
    break;
  case 16: 
    display.drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    break;
  case 17: 
    display.drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    display.drawXBitmap(42,10,WFTX_bits,WFTX_width,WFTX_height,WHITE);
    display.drawXBitmap(66, 34, WFRX_bits,WFRX_width, WFRX_height,WHITE );
    display.drawXBitmap(88, 10, UFORX_bits,UFORX_width, UFORX_height,WHITE);      
    break;



  
  default:
    break;
  }
  icon++;
  if (icon > 17) icon = 2;

  display.setTextSize(1);
  display.setCursor(0,54);
  display.print("Scanning the skyes...");

  display.display();
}
void DrawRadarPilot(uint8_t neighborIndex){
  float pilotDistance = 0.0;
  int bearing = 0;
  float rads;
  int relNorth;
  int relEast;

  
  //display.setCursor(95,0);
  //display.printf("%4d", fanet.neighbours[neighborIndex].rssi);
  display.setCursor(68,16);
  if (fanet.neighbours[neighborIndex].name.length() > 0){
    display.print(fanet.neighbours[neighborIndex].name.substring(0,10)); //max. 10 signs
  }else{
    display.print(fanet.getDevId(fanet.neighbours[neighborIndex].devId));
  }
  pilotDistance = distance(fanet._myData.lat, fanet._myData.lon,fanet.neighbours[neighborIndex].lat,fanet.neighbours[neighborIndex].lon, 'K') * 1000 ;
  bearing = CalcBearingA( fanet._myData.lat, fanet._myData.lon,fanet.neighbours[neighborIndex].lat,fanet.neighbours[neighborIndex].lon);
  rads = deg2rad(bearing + (fanet._myData.heading * -1));
  relEast=(int)((sin(rads) * 16) + RADAR_SCREEN_CENTER_X-8);
  relNorth=(int)(((cos(rads) * 16) * -1) + RADAR_SCREEN_CENTER_Y-8);
  //log_i("bearing=%i",bearing);
  //log_i("relNorth=%i",relNorth);
  //log_i("relEast=%i",relEast);
  switch (fanet.neighbours[neighborIndex].aircraftType)
  {
  case FanetLora::aircraft_t ::paraglider :
    display.drawXBitmap(relEast, relNorth, Paraglider16_bits,16, 16,WHITE);      
    break;
  case FanetLora::aircraft_t::hangglider :
    display.drawXBitmap(relEast, relNorth, Hangglider16_bits,16, 16,WHITE);      
    break;
  case FanetLora::aircraft_t::balloon :
    display.drawXBitmap(relEast, relNorth, Ballon16_bits,16, 16,WHITE);      
    break;
  case FanetLora::aircraft_t::glider :
    display.drawXBitmap(relEast, relNorth, Sailplane16_bits,16, 16,WHITE);      
    break;
  case FanetLora::aircraft_t::poweredAircraft :
    display.drawXBitmap(relEast, relNorth, Airplane16_bits,16, 16,WHITE);      
    break;
  case FanetLora::aircraft_t::helicopter :
    display.drawXBitmap(relEast, relNorth, Helicopter16_bits,16, 16,WHITE);      
    break;
  case FanetLora::aircraft_t::uav:
    display.drawXBitmap(relEast, relNorth, UAV16_bits,16, 16,WHITE);      
    break;
  
  default:
    display.drawXBitmap(relEast, relNorth, UFO16_bits,16, 16,WHITE);      
    break;
  }
  if (pilotDistance > 1000){
    pilotDistance /= 1000;
    display.setCursor(68,28);
    display.printf("%skm",setStringSize(String(pilotDistance,1),5).c_str());
  }else{
    display.setCursor(75,28);
    display.printf("%sm",setStringSize(String(pilotDistance,0),5).c_str());
  }
  display.setCursor(75,40); //display relative alt
  display.printf("%sm",setStringSize(String(fanet.neighbours[neighborIndex].altitude - fanet._myData.altitude,0),5).c_str());
  display.setCursor(75,52); //display climbing
  display.printf("%sms",setStringSize(String(fanet.neighbours[neighborIndex].climb,1),5).c_str());

}

void DrawAngleLine(int16_t x,int16_t y,int16_t length,float deg){
  int16_t xStart;
  int16_t yStart;
  int16_t xEnd;
  int16_t yEnd;
  float rads;
  rads = deg2rad(deg);
  xStart=(int)roundf(((sin(rads) * length/2) * 1) + x);
  yStart=(int)roundf(((cos(rads) * length/2) * -1) + y);
  xEnd=(int)roundf(((sin(rads) * length/2) * -1) + x);
  yEnd=(int)roundf(((cos(rads) * length/2) * 1) + y);  
  display.drawLine(xStart,yStart,xEnd,yEnd,WHITE);
  //log_i("x=%i,y=%i,deg=%0.1f,X-Start=%i,Y-Start=%i,X-End=%i,Y-End=%i",x,y,deg,xStart,yStart,xEnd,yEnd);
}

void DrawRadarScreen(uint32_t tAct,uint8_t mode){
  static uint8_t neighborIndex = 0;
  int index;
  int16_t xStart;
  int16_t yStart;
  float rads;
  
  String s = "";
  display.clearDisplay();
  drawAircraftType(0,0,setting.AircraftType);
  drawSatCount(18,0,(status.GPS_NumSat > 9) ? 9 : status.GPS_NumSat);
  //drawSatCount(18,0,9);
  drawWifiStat(status.wifiStat);
  drawBluetoothstat(101,0);
  drawBatt(111, 0,(status.BattCharging) ? 255 : status.BattPerc);

  
  display.setTextSize(1);
  display.drawCircle(RADAR_SCREEN_CENTER_X,RADAR_SCREEN_CENTER_Y,24,WHITE);

  DrawAngleLine(RADAR_SCREEN_CENTER_X,RADAR_SCREEN_CENTER_Y,30,fanet._myData.heading * -1);
  DrawAngleLine(RADAR_SCREEN_CENTER_X,RADAR_SCREEN_CENTER_Y,6,(fanet._myData.heading + 90) * -1);
  rads = deg2rad(fanet._myData.heading * -1);
  xStart=(int)(((sin(rads) * 19) * 1) + RADAR_SCREEN_CENTER_X);
  yStart=(int)(((cos(rads) * 19) * -1) + RADAR_SCREEN_CENTER_Y);
  display.setCursor(xStart-2,yStart-3);
  display.print("N");

  //display.drawFastHLine(0,RADAR_SCREEN_CENTER_X,64,WHITE);
  //display.drawFastVLine(RADAR_SCREEN_CENTER_X,8,56,WHITE);
  display.setTextSize(1);
  display.setCursor(42,0);
  switch (mode)
  {
  case RADAR_CLOSEST:
    display.print("CLOSEST");
    if (status.GPS_Fix == 0){
      display.setCursor(60,16);
      display.print("NO GPS-FIX");
      break;
    } 
    index = fanet.getNearestNeighborIndex();
    //log_i("index %i",index);
    if (index < 0) break;
    neighborIndex = index;
    DrawRadarPilot(neighborIndex);
    break;
  case RADAR_LIST:
    display.print("LIST");
    if (status.GPS_Fix == 0){
      display.setCursor(60,16);
      display.print("NO GPS-FIX");
      break;
    } 
    index = fanet.getNextNeighbor(neighborIndex);
    if (index < 0) break;
    neighborIndex = index;
    DrawRadarPilot(neighborIndex);
    break;
  case RADAR_FRIENDS:
    display.print("FRIENDS");
    if (status.GPS_Fix == 0){
      display.setCursor(50,16);
      display.print("NO GPS-FIX");
      break;
    } 
    break;    
  default:
    break;
  }
  display.display();
}

void drawSatCount(int16_t x, int16_t y,uint8_t value){
    //display.setFont(&FreeSansBold9pt7b);
    display.setTextSize(1);
    if (value == 0){
        display.drawXBitmap(x, y,gpsoff_bits,  16, 16, WHITE);
    }else{
        display.drawXBitmap(x, y,gpsOn_bits,  16, 16, WHITE);
        display.setCursor(x+18,y+4);
        display.print(String(value));
    }

}

void drawspeaker(int16_t x, int16_t y){
    uint8_t volume = 0;
    if (status.bMuting){
        volume = 0;
    }else{
        if (setting.vario.volume == LOWVOLUME){
            volume = 1;
        }else if (setting.vario.volume == MIDVOLUME){
            volume = 2;
        }else{
            volume = 3;
        }
    }
    switch (volume)
    {
    case 1:
        display.drawXBitmap(x, y,speakerlow_bits,  16, 16, WHITE);
        break;
    case 2:
        display.drawXBitmap(x, y,speakermid_bits,  16, 16, WHITE);
        break;
    case 3:
        display.drawXBitmap(x, y,speakerhigh_bits,  16, 16, WHITE);
        break;        
    default:
        display.drawXBitmap(x, y,speakeroff_bits,  16, 16, WHITE);
        break;
    }
}


void printGPSData(uint32_t tAct){
  String s = "";
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  //display.print(setStringSize(String(status.BattPerc) + "% ",4));
  drawAircraftType(0,0,setting.AircraftType);
  
  drawSatCount(18,0,(status.GPS_NumSat > 9) ? 9 : status.GPS_NumSat);
  drawspeaker(47,0);
  drawflying(67,0,status.flying);
  drawWifiStat(status.wifiStat);
  drawBluetoothstat(101,0);
  drawBatt(111, 0,(status.BattCharging) ? 255 : status.BattPerc);

  display.setTextSize(3);

  display.setCursor(0,20);
  display.print(setStringSize(String(status.ClimbRate,1) + "ms",7));

  display.setTextSize(2);

  display.setCursor(0,46);
  display.print(setStringSize(String(status.GPS_alt,0) + "m",4));

  display.setCursor(65,46);
  display.print(setStringSize(String(status.GPS_speed,0) + "kh",5));

  display.display();

}
#endif

void setWifi(bool on){
  if ((on) && (status.wifiStat == 0)){
    log_i("switch WIFI ACCESS-POINT ON");
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    WiFi.persistent(false);
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE); // call is only a workaround for bug in WiFi class
    WiFi.mode(WIFI_MODE_AP);
    WiFi.softAP(host_name.c_str(), setting.wifi.appw.c_str());
    WiFi.softAPConfig(local_IP, gateway, subnet);
    WiFi.setHostname(host_name.c_str());
    Web_setup();
    status.wifiStat = 1;      
  }
  if ((!on) && (status.wifiStat != 0)){
    log_i("switch WIFI OFF");
    Web_stop();
    WiFi.softAPdisconnect(true);
    WiFi.disconnect();
    WiFi.mode(WIFI_MODE_NULL);
    esp_wifi_set_mode(WIFI_MODE_NULL);
    esp_wifi_stop();
    setting.wifi.tWifiStop=0;
    status.wifiStat=0;
  }
  wifiCMD = 0;
}

void checkSystemCmd(char *ch_str){
	/* remove \r\n and any spaces */
	String line = ch_str;
  log_i("systemcmd=%s",line.c_str());
  String sRet = "";
  int32_t iPos;
  iPos = getStringValue(line,"#SYC WIFI=","\r",0,&sRet);
  if (iPos >= 0){
    uint8_t u8 = atoi(sRet.c_str());
    u8 = constrain(u8,0,1);
    if (u8 == 1){
      wifiCMD = 11;
    }else{
      wifiCMD = 10;
    }
    add2OutputString("#SYC OK\r\n");
  }
  if (line.indexOf("#SYC /?") >= 0){
    add2OutputString("VER\r\nNAME\r\nTYPE\r\nAIRMODE\r\nMODE\r\nOUTMODE\r\n");
  }
  if (line.indexOf("#SYC VER?") >= 0){
    //log_i("sending 2 client");
    add2OutputString("#SYC VER=" VERSION "\r\n");
  }
  if (line.indexOf("#SYC NAME?") >= 0){
    //log_i("sending 2 client");
    add2OutputString("#SYC NAME=" + setting.PilotName + "\r\n");
  }
  iPos = getStringValue(line,"#SYC NAME=","\r",0,&sRet);
  if (iPos >= 0){
    if (setting.PilotName != sRet){
      setting.PilotName = sRet;
      fanet.setPilotname(setting.PilotName);
      write_PilotName();      
    }
    add2OutputString("#SYC OK\r\n");
  }
  if (line.indexOf("#SYC TYPE?") >= 0){
    //log_i("sending 2 client");
    add2OutputString("#SYC TYPE=" + String(setting.AircraftType) + "\r\n");
  }
  iPos = getStringValue(line,"#SYC TYPE=","\r",0,&sRet);
  if (iPos >= 0){
    uint8_t u8 = atoi(sRet.c_str());
    u8 = constrain(u8,0,7);
    if (u8 != (uint8_t)setting.AircraftType){
      setting.AircraftType = FanetLora::aircraft_t(u8);
      fanet.setAircraftType(setting.AircraftType);
      write_AircraftType();
    }
    add2OutputString("#SYC OK\r\n");
  }
  if (line.indexOf("#SYC AIRMODE?") >= 0){
    //log_i("sending 2 client");
    add2OutputString("#SYC AIRMODE=" + String(setting.fanetMode) + "\r\n");
  }
  iPos = getStringValue(line,"#SYC AIRMODE=","\r",0,&sRet);
  if (iPos >= 0){
    uint8_t u8 = atoi(sRet.c_str());
    u8 = constrain(u8,0,1);
    if (u8 != setting.fanetMode){
      setting.fanetMode = u8;
      write_AirMode();
    }
    add2OutputString("#SYC OK\r\n");
  }
  if (line.indexOf("#SYC MODE?") >= 0){
    //log_i("sending 2 client");
    add2OutputString("#SYC MODE=" + String(setting.Mode) + "\r\n");
  }
  iPos = getStringValue(line,"#SYC MODE=","\r",0,&sRet);
  if (iPos >= 0){
    uint8_t u8 = atoi(sRet.c_str());
    u8 = constrain(u8,0,1);
    add2OutputString("#SYC OK\r\n");
    if (u8 != setting.Mode){
      setting.Mode = u8;
      write_Mode();
      delay(500); //we have to restart in case, the mode is changed
      log_e("ESP Restarting !");
      esp_restart();
    }
  }
  if (line.indexOf("#SYC OUTMODE?") >= 0){
    //log_i("sending 2 client");
    add2OutputString("#SYC OUTMODE=" + String(setting.outputMode) + "\r\n");
  }
  iPos = getStringValue(line,"#SYC OUTMODE=","\r",0,&sRet);
  if (iPos >= 0){
    uint8_t u8 = atoi(sRet.c_str());
    u8 = constrain(u8,0,3);
    add2OutputString("#SYC OK\r\n");
    if (u8 != setting.outputMode){
      setting.outputMode = u8;
      write_OutputMode();
      delay(500); //we have to restart in case, the mode is changed
      log_e("ESP Restarting !");
      esp_restart();
    }
  }
  if (line.indexOf("#SYC FNTPWR?") >= 0){
    //log_i("sending 2 client");
    add2OutputString("#SYC FNTPWR=" + String(setting.LoraPower) + "\r\n");
  }
  iPos = getStringValue(line,"#SYC FNTPWR=","\r",0,&sRet);
  if (iPos >= 0){
    uint8_t u8 = atoi(sRet.c_str());
    u8 = constrain(u8,0,14);
    add2OutputString("#SYC OK\r\n");
    if (u8 != setting.LoraPower){
      setting.LoraPower = u8;
      LoRa.setTxPower(setting.LoraPower);
      write_LoraPower();
    }
  }
}


void checkReceivedLine(char *ch_str){
  log_i("new serial msg=%s",ch_str);
  if(!strncmp(ch_str, FANET_CMD_TRANSMIT, 4)){
    fanet.fanet_cmd_transmit(ch_str+4);
  }else if(!strncmp(ch_str, FANET_CMD_GROUND_TYPE, 4)){
    fanet.fanet_cmd_setGroundTrackingType(ch_str+4);
  }else if (!strncmp(ch_str,SYSTEM_CMD,4)){
    checkSystemCmd(ch_str);
  }else if (!strncmp(ch_str,GPS_STATE,2)){
    //got GPS-Info
    if (sNmeaIn.length() == 0){
      sNmeaIn = String(ch_str);
    }
  }
  /*
  }else if(!strncmp(ch_str, "@", 1)){
    char *ptr = strchr(ch_str, '\r');
    if(ptr == nullptr)
      ptr = strchr(ch_str, '\n');
    if(ptr != nullptr)
      *ptr = '\0';

    char *p = (char *)ch_str + 1;
    uint32_t devId = strtol(p, NULL, 16);
    p = strchr(p, SEPARATOR)+1;
    String msg = p;
    //log_i("msg=%s",msg.c_str());
    fanet.writeMsgType3(devId,msg);
    //fanet.fanet_sendMsg(ch_str+1);
  }else{
    log_i("broadcast-msg %s",ch_str);
    String msg = ch_str;
    fanet.writeMsgType3(0,msg);
  }*/
}

#ifdef BLUETOOTHSERIAL 
char* readBtSerial(){
  static char lineBuffer[512];
  static uint16_t recBufferIndex = 0;
  if (status.bluetoothStat == 0){
    return NULL; //bluetooth not started yet.
  }
  if (status.bluetoothStat == 2){
    while(SerialBT.available()){
      if (recBufferIndex >= (512-1)) recBufferIndex = 0; //Buffer overrun
      lineBuffer[recBufferIndex] = SerialBT.read();
      if (lineBuffer[recBufferIndex] == '\n'){
        recBufferIndex++;
        lineBuffer[recBufferIndex] = 0; //zero-termination
        recBufferIndex = 0;
        return &lineBuffer[0];
      }else{
        recBufferIndex++;
      }  
    }
  }else{
    //status.bluetoothStat = 1;
    recBufferIndex = 0;
  }
  return NULL;
}
#endif

char* readSerial(){
  static char lineBuffer[512];
  static uint16_t recBufferIndex = 0;
  
  while(Serial.available()){
    if (recBufferIndex >= (512-1)) recBufferIndex = 0; //Buffer overrun
    lineBuffer[recBufferIndex] = Serial.read();
    if (lineBuffer[recBufferIndex] == '\n'){
      recBufferIndex++;
      lineBuffer[recBufferIndex] = 0; //zero-termination
      recBufferIndex = 0;
      return &lineBuffer[0];
    }else{
      recBufferIndex++;
    }  
  }
  return NULL;
}

#ifdef AIRMODULE
void readGPS(){
  static char lineBuffer[255];
  static uint16_t recBufferIndex = 0;
  static uint32_t tGpsOk = millis();
  
  if (sNmeaIn.length() > 0){
    if (!status.bHasGPS){
      char * cstr = new char [sNmeaIn.length()+1];
      strcpy (cstr, sNmeaIn.c_str());
      //log_i("process GPS-String:%s",cstr);
      uint16_t i = 0;
      //char c;
      //Serial.println();
      //Serial.print("s=");
      while (true){
        //Serial.print(cstr[i]);
        nmea.process(cstr[i]);
        if (cstr[i] == 0){
          //log_i("break %d",i);
          break;
        }
        if (i >= 255){
          //log_i("i >= 255 %d",i);
          break;
        }
        i++;
      }
      delete cstr; //delete allocated String
      if (setting.outputGPS) sendData2Client(sNmeaIn);
      sNmeaIn = "";
    }else{
      sNmeaIn = ""; //we have a gps --> don't take data from external GPS
    }
  }
  while(NMeaSerial.available()){
    if (recBufferIndex >= 255) recBufferIndex = 0; //Buffer overrun
    lineBuffer[recBufferIndex] = NMeaSerial.read();
    //log_i("GPS %c",lineBuffer[recBufferIndex]);
    nmea.process(lineBuffer[recBufferIndex]);
    if (lineBuffer[recBufferIndex] == '\n'){
      lineBuffer[recBufferIndex] = '\r';
      recBufferIndex++;
      lineBuffer[recBufferIndex] = '\n';
      recBufferIndex++;
      lineBuffer[recBufferIndex] = 0; //zero-termination
      String s = lineBuffer;
      if (setting.outputGPS) sendData2Client(s);
      recBufferIndex = 0;
      tGpsOk = millis();
      status.bHasGPS = true;
    }else{
      if (lineBuffer[recBufferIndex] != '\r'){
        recBufferIndex++;
      }
    }  
  }
  if (timeOver(millis(),tGpsOk,10000)){
    status.bHasGPS = false;
  }
}
#endif

eFlarmAircraftType Fanet2FlarmAircraft(FanetLora::aircraft_t aircraft){
  switch (aircraft)
  {
  case FanetLora::aircraft_t::paraglider :
    return eFlarmAircraftType::PARA_GLIDER;
  case FanetLora::aircraft_t::hangglider :
    return eFlarmAircraftType::HANG_GLIDER;
  case FanetLora::aircraft_t::balloon :
    return eFlarmAircraftType::BALLOON;
  case FanetLora::aircraft_t::glider :
    return eFlarmAircraftType::GLIDER_MOTOR_GLIDER;
  case FanetLora::aircraft_t::poweredAircraft :
    return eFlarmAircraftType::TOW_PLANE;
  case FanetLora::aircraft_t::helicopter :
    return eFlarmAircraftType::HELICOPTER_ROTORCRAFT;
  case FanetLora::aircraft_t::uav :
    return eFlarmAircraftType::UAV;
  default:
    return eFlarmAircraftType::UNKNOWN;
  }
}

void Fanet2FlarmData(FanetLora::trackingData *FanetData,FlarmtrackingData *FlarmDataData){
  FlarmDataData->aircraftType = Fanet2FlarmAircraft(FanetData->aircraftType);
  FlarmDataData->altitude = FanetData->altitude;
  FlarmDataData->climb = FanetData->climb;
  FlarmDataData->DevId = fanet.getDevId(FanetData->devId);
  FlarmDataData->heading = FanetData->heading;
  FlarmDataData->lat = FanetData->lat;
  FlarmDataData->lon = FanetData->lon;
  FlarmDataData->speed = FanetData->speed;
}

void sendLK8EX(uint32_t tAct){
  static uint32_t tOld = millis();
  if (!setting.outputLK8EX1) return; //not output
  if ((tAct - tOld) >= 250){
    //String s = "$LK8EX1,101300,99999,99999,99,999,";
    String s = "$LK8EX1,";

    if (status.vario.bHasVario){
      s += String(status.pressure,2) + ","; //raw pressure in hPascal: hPA*100 (example for 1013.25 becomes  101325) 
      if (status.GPS_Fix){
        s += String(status.GPS_alt,2) + ","; // altitude in meters, relative to QNH 1013.25
      }else{
        s += String(status.varioAlt,2) + ","; // altitude in meters, relative to QNH 1013.25
      }
      s += String((int32_t)(status.ClimbRate * 100.0)) + ","; //climbrate in cm/s
      s += String(status.varioTemp,1) + ","; //temperature
    }else{
      s += "999999,"; //raw pressure in hPascal: hPA*100 (example for 1013.25 becomes  101325) 
      s += String(status.GPS_alt,2) + ","; // altitude in meters, relative to QNH 1013.25
      s += String((int32_t)(status.ClimbRate * 100.0)) + ","; //climbrate in cm/s
      //s += String(status.) + ",";
      s += "99,"; //temperature
    }
    
    s += String((float)status.vBatt / 1000.,2) + ",";
    s = flarm.addChecksum(s);
    sendData2Client(s);
    tOld = tAct;
  }
}

#ifdef AIRMODULE
bool setupUbloxConfig(){
  SFE_UBLOX_GPS ublox;
  ublox.begin(NMeaSerial);
  ublox.factoryReset();
  delay(2000); //wait for hardware again !!
  for (int i = 0; i < 3; i++){
    if (!ublox.isConnected()){
      log_e("ublox: GPS not connected");
      continue;
    }
    if (!ublox.setUART1Output(COM_TYPE_NMEA)){
      log_e("ublox: error setting uart1 output");
      continue;
    }
    //disable nmea sentencess
    if (!ublox.disableNMEAMessage(UBX_NMEA_GLL,COM_PORT_UART1)){
      log_e("ublox: error setting parameter %d",UBX_NMEA_GLL);
      continue;
    }
    if (!ublox.disableNMEAMessage(UBX_NMEA_GSV,COM_PORT_UART1)){
      log_e("ublox: error setting parameter %d",UBX_NMEA_GSV);
      continue;
    }
    if (!ublox.disableNMEAMessage(UBX_NMEA_VTG,COM_PORT_UART1)){
      log_e("ublox: error setting parameter %d",UBX_NMEA_VTG);
      continue;
    }
    //xcguide uses GSA für GPS-Fix
    if (!ublox.disableNMEAMessage(UBX_NMEA_GSA,COM_PORT_UART1)){
      log_e("ublox: error setting parameter %d",UBX_NMEA_GSA);
      continue;
    }

    //enable nmea-sentences
    if (!ublox.enableNMEAMessage(UBX_NMEA_GGA,COM_PORT_UART1)){
      log_e("ublox: error setting parameter %d",UBX_NMEA_GGA);
      continue;
    }
    if (!ublox.enableNMEAMessage(UBX_NMEA_RMC,COM_PORT_UART1)){
      log_e("ublox: error setting parameter %d",UBX_NMEA_RMC);
      continue;
    }
    if (!ublox.saveConfiguration(3000)){
      log_e("ublox: error saving config");
      continue;
    }else{
      log_i("!!! setup ublox successfully");
      return true;
      break;      
    }
  }
  return false;

}
#endif


void taskStandard(void *pvParameters){
  static uint32_t tLoop = millis();
  static float oldAlt = 0.0;
  static uint32_t tOldPPS = millis();
  static uint32_t tLastPPS = millis();
  static uint32_t tDisplay = millis();
  #ifdef TEST
  static uint32_t tSend = millis();
  #endif
  char * pSerialLine = NULL;
  String sSerial = "";
  String s = "";
  FanetLora::trackingData myFanetData;  
  FanetLora::trackingData tFanetData;  
  
  uint8_t oldScreenNumber = 0;
  tFanetData.rssi = 0;
  MyFanetData.rssi = 0;

  #ifdef AIRMODULE
  if (status.bHasAXP192){
    NMeaSerial.begin(GPSBAUDRATE,SERIAL_8N1,34,12,false);
    log_i("GPS Baud=9600,8N1,RX=34,TX=12");
  }else{
    NMeaSerial.begin(GPSBAUDRATE,SERIAL_8N1,12,15,false);
    log_i("GPS Baud=9600,8N1,RX=12,TX=15");
  }
  
  //setupUbloxConfig();

  // Change the echoing messages to the ones recognized by the MicroNMEA library
  //MicroNMEA::sendSentence(NMeaSerial, "$PSTMSETPAR,1201,0x00000042");
  //MicroNMEA::sendSentence(NMeaSerial, "$PSTMSETPAR,1201,0x00000041"); //enable only $GPGGA and $GPRMC
  //MicroNMEA::sendSentence(NMeaSerial, "$PSTMSAVEPAR");

  //Reset the device so that the changes could take plaace
  //MicroNMEA::sendSentence(NMeaSerial, "$PSTMSRR");


  delay(1000);
  //clear serial buffer
  while (NMeaSerial.available())
    NMeaSerial.read();
  if (status.bHasAXP192){  
    //only on new boards we have an pps-pin
    pinMode(PPSPIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(PPSPIN), ppsHandler, FALLING);
  }
  #endif
  #ifdef GSMODULE
  if (setting.Mode == MODE_GROUND_STATION){
    //mode ground-station
    status.GPS_Lat = setting.gs.lat;
    status.GPS_Lon = setting.gs.lon;  
    status.GPS_alt = setting.gs.alt;
  }
  #endif


  // create a binary semaphore for task synchronization
  long frequency = FREQUENCY868;
  //bool begin(int8_t sck, int8_t miso, int8_t mosi, int8_t ss,int reset, int dio0,long frequency,uint8_t outputPower);
  if (setting.band == BAND915)frequency = FREQUENCY915; 
  fanet.setLegacy(setting.LegacyTxEnable);
  fanet.begin(PinLora_SCK, PinLora_MISO, PinLora_MOSI, PinLora_SS,PinLoraRst, PinLoraDI0,frequency,setting.LoraPower);
  fanet.setPilotname(setting.PilotName);
  fanet.setAircraftType(setting.AircraftType);
  //if (setting.Mode != MODE_DEVELOPER){ //
    fanet.autobroadcast = true;
  //}else{
  //  fanet.autobroadcast = false;
  //}
  setting.myDevId = fanet.getMyDevId();
  host_name = APPNAME "-" + setting.myDevId; //String((ESP32_getChipId() & 0xFFFFFF), HEX);
  #ifdef AIRMODULE
  if (setting.Mode == MODE_AIR_MODULE){
    flarm.begin();
  }
  #endif
  if (setting.OGNLiveTracking){
    #ifdef GSMODULE
      if (setting.Mode == MODE_GROUND_STATION){
        ogn.setAirMode(false); //set airmode
        #ifdef GSM_MODULE
          if (setting.wifi.connect == MODE_WIFI_DISABLED){
            ogn.setClient(&GsmOGNClient);
            ogn.setMutex(&xGsmMutex);
          }
        #endif
        if (setting.PilotName.length() > 0){
          ogn.begin(setting.PilotName,VERSION "." APPNAME);
        }else{
          ogn.begin("FNB" + setting.myDevId,VERSION "." APPNAME);
        }      
        ogn.setGPS(setting.gs.lat,setting.gs.lon,setting.gs.alt,0.0,0.0);
      }
    #endif
    #ifdef AIRMODULE
      if (setting.Mode == MODE_AIR_MODULE){
        ogn.setAirMode(true); //set airmode
        ogn.begin("FNB" + setting.myDevId,VERSION "." APPNAME);
      }
    #endif
  } 


#ifdef OLED
  if (status.displayType == OLED0_96){
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0,55);
    display.print(setting.myDevId);
    display.display();
    delay(3000);
    if (setting.gs.SreenOption == SCREEN_ALWAYS_OFF){
      oledPowerOff();
    }
  }
#endif

  //udp.begin(UDPPORT);
  tLoop = millis();
  while(1){    
    // put your main code here, to run repeatedly:
    uint32_t tAct = millis();
    #ifdef TEST
    if (timeOver(tAct,tSend,1000)){
      tSend = tSend;
      sendData2Client("$GPGGA,161640.00,4804.36384,N,01444.09266,E,1,04,11.84,320.6,M,43.5,M,,*69\n");
      sendData2Client("$GPRMC,161640.00,A,4804.36384,N,01444.09266,E,0.867,,090121,,,A*7A\n");
      sendData2Client("#FNF 8,C3CC,1,0,1,B,F33344467E0A0A12397CB8\n");
      sendData2Client("#FNF 8,C3CC,1,0,2,13,416E64726561732048696E74657265636B6572\n");
    }
    #endif

    status.tLoop = tAct - tLoop;
    tLoop = tAct;
    if (status.tMaxLoop < status.tLoop) status.tMaxLoop = status.tLoop;

    #ifdef AIRMODULE    
    if (setting.bConfigGPS){
      setupUbloxConfig();
      setting.bConfigGPS = false;
    }
    #endif
    handleButton(tAct);

    if (setting.OGNLiveTracking){
      if (status.vario.bHasVario){
        ogn.setStatusData(status.pressure ,status.varioTemp,NAN,(float)status.vBatt / 1000.);
      }else if ((status.vario.bHasBME) || (status.bWUBroadCast)){
        ogn.setStatusData(status.weather.Pressure ,status.weather.temp,status.weather.Humidity,(float)status.vBatt / 1000.);
      }else{
        ogn.setStatusData(NAN ,NAN, NAN, (float)status.vBatt / 1000.);
      }
      ogn.run(status.bInternetConnected);
    } 
    checkFlyingState(tAct);
    printBattVoltage(tAct);
    if (sOutputData.length() > 0){
      String s;
      xSemaphoreTake( xOutputMutex, portMAX_DELAY );
      s = sOutputData;
      sOutputData = "";
      xSemaphoreGive(xOutputMutex);
      //log_i("sending 2 client %s",s.c_str());
      if (fanetDstId > 0){ //send msg back to dst via fanet
        //log_i("sending back to %s:%s",fanet.getDevId(fanetDstId),s.c_str()); 
        fanet.writeMsgType3(fanetDstId,s);
        fanetDstId = 0;
      }
      sendData2Client(s);
    }

    #ifdef AIRMODULE
    if (setting.Mode == MODE_AIR_MODULE){
      readGPS();
    }
    #endif    
    sendFlarmData(tAct);
    #ifdef OLED
    if (status.displayType == OLED0_96){
    #ifdef GSMODULE
      if (setting.Mode == MODE_GROUND_STATION){
        if (setting.gs.SreenOption == SCREEN_WEATHER_DATA){
          printWeather(tAct);
        }else if (setting.gs.SreenOption != SCREEN_ALWAYS_OFF){
          if (fanet.getNeighboursCount() == 0){
            if (timeOver(tAct,tDisplay,500)){
              tDisplay = tAct;
              printScanning(tAct);
            }
          }else{
            if (timeOver(tAct,tDisplay,DISPLAY_UPDATE_RATE_GS)){
              tDisplay = tAct;
              printGSData(tAct);
            }
          }
        }
      }
    #endif
    #ifdef AIRMODULE
      if (setting.Mode == MODE_AIR_MODULE){
        switch (setting.screenNumber)
        {
        case 0: //main-Display
          if ((timeOver(tAct,tDisplay,DISPLAY_UPDATE_RATE)) || (oldScreenNumber != setting.screenNumber)){
            tDisplay = tAct;
            printGPSData(tAct);          
          }
          break;
        case 1: //radar-screen with list
          if ((timeOver(tAct,tDisplay,DISPLAY_UPDATE_RATE2)) || (oldScreenNumber != setting.screenNumber)){
            tDisplay = tAct;              
            DrawRadarScreen(tAct,RADAR_LIST);
          }
          break;
        case 2: //radar-screen with closest
          if ((timeOver(tAct,tDisplay,DISPLAY_UPDATE_RATE)) || (oldScreenNumber != setting.screenNumber)){
            tDisplay = tAct;
            DrawRadarScreen(tAct,RADAR_CLOSEST);
          }
          break;
        case 3: //list aircrafts
          if ((timeOver(tAct,tDisplay,DISPLAY_UPDATE_RATE)) || (oldScreenNumber != setting.screenNumber)){
            tDisplay = tAct;
            //DrawAircraftList();
          }
          break;
        default:
          break;
        }
        oldScreenNumber = setting.screenNumber;
      }
    #endif
    }
    #endif
    if (sendTestData == 1){
      log_i("sending msgtype 1");
      testTrackingData.devId = fanet._myData.devId;
      fanet.writeMsgType1(&testTrackingData);
      sendAWTrackingdata(&testTrackingData);
      sendTraccarTrackingdata(&testTrackingData);
      sendTestData = 0;
    }else if (sendTestData == 2){
      log_i("sending msgtype 2 %s",testString.c_str());
      fanet.writeMsgType2(testString);
      sendTestData = 0;
    }else if (sendTestData == 3){
      log_i("sending msgtype 3 to %s %s",fanet.getDevId(fanetReceiver).c_str(),testString.c_str());
      fanet.writeMsgType3(fanetReceiver,testString);
      sendTestData = 0;
    }else if (sendTestData == 4){
      log_i("sending msgtype 4");
      testWeatherData.bStateOfCharge = true;
      testWeatherData.bBaro = true;
      testWeatherData.bHumidity = true;
      testWeatherData.bWind = true;
      testWeatherData.bTemp = true;
      fanet.writeMsgType4(&testWeatherData);
      sendTestData = 0;
    }
    if (sendWeatherData){ //we have to send weatherdata
      //log_i("sending weatherdata");
      fanet.writeMsgType4(&testWeatherData);
      if (setting.OGNLiveTracking){
        Ogn::weatherData wData;
        wData.devId = fanet.getMyDevId();
        wData.lat = status.GPS_Lat;
        wData.lon = status.GPS_Lon;
        wData.bBaro = true;
        wData.Baro = status.weather.Pressure;
        wData.bHumidity = true;
        wData.Humidity = status.weather.Humidity;
        wData.bRain = true;
        wData.rain1h = status.weather.rain1h;
        wData.rain24h = status.weather.rain1d;
        wData.bTemp = true;
        wData.temp = status.weather.temp;
        wData.bWind = true;
        wData.wHeading = status.weather.WindDir;
        wData.wGust = status.weather.WindGust;
        wData.wSpeed = status.weather.WindSpeed;
        wData.snr = 0.0;      
        ogn.sendWeatherData(&wData);
      }      
      sendWeatherData = false;
    }
    pSerialLine = readSerial();
    if (pSerialLine != NULL){
      checkReceivedLine(pSerialLine);
    }
    #ifdef BLUETOOTHSERIAL 
    if (setting.outputMode == OUTPUT_BLUETOOTH){
      pSerialLine = readBtSerial();
      if (pSerialLine != NULL){
        checkReceivedLine(pSerialLine);
      }
    }
    #endif    
    if ((setting.fanetMode == FN_AIR_TRACKING) || (status.flying)){
      fanet.onGround = false; //online-tracking
    }else{
      fanet.onGround = true; //ground-tracking
    }
    fanet.run();
    status.fanetRx = fanet.rxCount;
    status.fanetTx = fanet.txCount;
    if (fanet.isNewMsg()){
      //write msg to udp !!
      String msg = fanet.getactMsg() + "\n";
      if (setting.outputFANET) sendData2Client(msg);
    }
    FanetLora::nameData nameData;
    if (fanet.getNameData(&nameData)){
      if (setting.OGNLiveTracking){
        ogn.sendNameData(fanet.getDevId(nameData.devId),nameData.name,(float)nameData.snr / 10.0);
      }
    }
    FanetLora::msgData msgData;
    if (fanet.getlastMsgData(&msgData)){
      if (msgData.dstDevId == fanet._myData.devId){
        String sRet = "";
        int pos = getStringValue(msgData.msg,"P","#",0,&sRet);
        if (pos >= 0){
          if (atoi(sRet.c_str()) == setting.fanetpin){
            fanetDstId = msgData.srcDevId; //we have to store the sender, to send back the value !!
            log_i("got fanet-cmd from %s:%s",fanet.getDevId(fanetDstId),msgData.msg.c_str()); 
            //log_i("msg=%s",msgData.msg.substring(pos).c_str());
            String s = msgData.msg.substring(pos) + "\r\n";
            checkReceivedLine((char *)s.c_str());
          }
        }
      }
    }
    FanetLora::weatherData weatherData;
    if (fanet.getWeatherData(&weatherData)){
      if (setting.OGNLiveTracking){
        Ogn::weatherData wData;
        wData.devId = fanet.getDevId(weatherData.devId);
        wData.lat = weatherData.lat;
        wData.lon = weatherData.lon;
        wData.bBaro = weatherData.bBaro;
        wData.Baro = weatherData.Baro;
        wData.bHumidity = weatherData.bHumidity;
        wData.Humidity = weatherData.Humidity;
        wData.bRain = false;
        wData.rain1h = 0;
        wData.rain24h = 0;
        wData.bTemp = weatherData.bTemp;
        wData.temp = weatherData.temp;
        wData.bWind = weatherData.bWind;
        wData.wGust = weatherData.wGust;
        wData.wHeading = weatherData.wHeading;
        wData.wSpeed = weatherData.wSpeed;
        wData.snr = (float)weatherData.snr / 10.0;      
        ogn.sendWeatherData(&wData);
      }
    }    
    if (fanet.getTrackingData(&tFanetData)){
        //log_i("new Tracking-Data");
        if (tFanetData.type == 0x11){ //online-tracking
          if (setting.OGNLiveTracking){
            ogn.sendTrackingData(tFanetData.lat ,tFanetData.lon,tFanetData.altitude,tFanetData.speed,tFanetData.heading,tFanetData.climb,fanet.getDevId(tFanetData.devId) ,(Ogn::aircraft_t)tFanetData.aircraftType,tFanetData.OnlineTracking,(float)tFanetData.snr / 10.0);
          } 
          sendAWTrackingdata(&tFanetData);
          sendTraccarTrackingdata(&tFanetData);
        }else if (tFanetData.type >= 0x70){ //ground-tracking
          if (setting.OGNLiveTracking){
            ogn.sendGroundTrackingData(tFanetData.lat,tFanetData.lon,fanet.getDevId(tFanetData.devId),tFanetData.type,(float)tFanetData.snr / 10.0);
          } 
        }
        
       
        //if (nmea.isValid()){
        //  fanet.getMyTrackingData(&myFanetData);
        //}
    }    
    flarm.run();
    sendLK8EX(tAct);
    #ifdef AIRMODULE
    if (setting.Mode == MODE_AIR_MODULE){
      if (!status.bHasAXP192){
        if ((tAct - tOldPPS) >= 1000){
          ppsTriggered = true;
        }
      }
      if (ppsTriggered){
        ppsTriggered = false;
        tLastPPS = tAct;
        //log_i("PPS-Triggered t=%d",status.tGPSCycle);
        //log_e("GPS-FixTime=%s",nmea.getFixTime().c_str());
        status.tGPSCycle = tAct - tOldPPS;
        if (nmea.isValid()){
          //log_i("nmea is valid");
          long alt = 0;
          nmea.getAltitude(alt);
          #ifdef AIRMODULE
          if (setting.Mode == MODE_AIR_MODULE){
            status.GPS_NumSat = nmea.getNumSatellites();
          }
          #endif
          status.GPS_Fix = 1;
          if (status.GPS_NumSat < 4){ //we need at least 4 satellites to get accurate position !!
            status.GPS_speed = 0;
            status.GPS_course = 0;
          }else{
            status.GPS_speed = nmea.getSpeed()*1.852/1000.; //speed in cm/s --> we need km/h
            status.GPS_course = nmea.getCourse()/1000.;
          }
          status.GPS_Lat = nmea.getLatitude() / 1000000.;
          status.GPS_Lon = nmea.getLongitude() / 1000000.;  
          status.GPS_alt = alt/1000.;
          setTime(nmea.getHour(), nmea.getMinute(), nmea.getSecond(), nmea.getDay(), nmea.getMonth(), nmea.getYear());
          if (oldAlt == 0) oldAlt = status.GPS_alt;        
          if (!status.vario.bHasVario) status.ClimbRate = (status.GPS_alt - oldAlt) / (float(status.tGPSCycle) / 1000.0);        
          oldAlt = status.GPS_alt;
          MyFanetData.climb = status.ClimbRate;
          MyFanetData.lat = status.GPS_Lat;
          MyFanetData.lon = status.GPS_Lon;
          MyFanetData.altitude = status.GPS_alt;
          MyFanetData.speed = status.GPS_speed; //speed in cm/s --> we need km/h
          if ((status.GPS_speed <= 5.0) && (status.vario.bHasVario)){
            MyFanetData.heading = status.varioHeading;
          }else{
            MyFanetData.heading = status.GPS_course;
          }        
          if (setting.OGNLiveTracking){
            ogn.setGPS(status.GPS_Lat,status.GPS_Lon,status.GPS_alt,status.GPS_speed,MyFanetData.heading);
            if (fanet.onGround){
              ogn.sendGroundTrackingData(status.GPS_Lat,status.GPS_Lon,fanet.getDevId(tFanetData.devId),fanet.state,0.0);
            }else{
              ogn.sendTrackingData(status.GPS_Lat,status.GPS_Lon,status.GPS_alt,status.GPS_speed,MyFanetData.heading,status.ClimbRate,fanet.getMyDevId() ,(Ogn::aircraft_t)fanet.getAircraftType(),fanet.doOnlineTracking,0.0);
            }
            
          } 

          fanet.setMyTrackingData(&MyFanetData); //set Data on fanet
          sendAWTrackingdata(&MyFanetData);
          sendTraccarTrackingdata(&MyFanetData);
        }else{
          status.GPS_Fix = 0;
          status.GPS_speed = 0.0;
          status.GPS_Lat = 0.0;
          status.GPS_Lon = 0.0;
          status.GPS_alt = 0.0;
          status.GPS_course = 0.0;
          status.GPS_NumSat = 0;
          if (!status.vario.bHasVario) status.ClimbRate = 0.0;
          oldAlt = 0.0;
        }
        tOldPPS = tAct;
      }
      if((tAct - tLastPPS) >= 10000){
        //no pps for more then 10sec. --> clear GPS-state
        status.GPS_Fix = 0;
        status.GPS_speed = 0.0;
        status.GPS_Lat = 0.0;
        status.GPS_Lon = 0.0;
        status.GPS_alt = 0.0;
        status.GPS_course = 0.0;
        status.GPS_NumSat = 0;
      }
    }
    #endif

    #ifdef GSMODULE
    if (setting.Mode == MODE_GROUND_STATION){
      sendAWGroundStationdata(tAct); //send ground-station-data    
    }
    #endif

    if (AXP192_Irq){
      if (axp.readIRQ() == AXP_PASS) {
        if (axp.isPEKLongtPressIRQ()) {
          log_v("Long Press IRQ");
          status.bPowerOff = true;
        }
        if (axp.isPEKShortPressIRQ()) {
          log_v("Short Press IRQ");
          setting.screenNumber ++;
          if (setting.screenNumber > MAXSCREENS) setting.screenNumber = 0;
          write_screenNumber(); //save screennumber in File
          tDisplay = tAct - DISPLAY_UPDATE_RATE;
        }
        axp.clearIRQ();
      }
      AXP192_Irq = false;
    }

    delay(1);
    if ((WebUpdateRunning) || (bPowerOff)) break;
  }
  log_i("stop task");
  #ifdef OLED
  if (status.displayType == OLED0_96){
    if (WebUpdateRunning){
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(10,5);
    display.print("FW-UPDATE");
    display.setCursor(10,30);
    display.print("wait...");
    display.display();
    }
  }
  #endif
  fanet.end();
  if (setting.OGNLiveTracking) ogn.end();
  vTaskDelete(xHandleStandard); //delete standard-task
}

void powerOff(){
  axp.clearIRQ();
  bPowerOff = true;
  delay(100);


  esp_wifi_set_mode(WIFI_MODE_NULL);
  esp_wifi_stop();
  //esp_bt_controller_disable();


  /*
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(4);
  display.setCursor(20,20);
  display.print("OFF");
  display.display(); 
  */

  log_i("wait until all tasks are stopped");
  eTaskState tBaro = eDeleted;
  eTaskState tEInk = eDeleted;
  eTaskState tStandard = eDeleted;
  eTaskState tGSM = eDeleted;
  eTaskState tWeather = eDeleted;
  while(1){
    //wait until all tasks are stopped
    if (xHandleBaro != NULL) tBaro = eTaskGetState(xHandleBaro);
    if (xHandleEInk != NULL) tEInk = eTaskGetState(xHandleEInk);
    if (xHandleStandard != NULL) tStandard = eTaskGetState(xHandleStandard);
    if (xHandleWeather != NULL) tWeather = eTaskGetState(xHandleWeather);    
    if ((tBaro == eDeleted) && (tEInk == eDeleted) && (tWeather == eDeleted) && (tStandard == eDeleted)) break; //now all tasks are stopped    
    log_i("baro=%d,eink=%d,standard=%d,weather=%d",tBaro,tEInk,tStandard,tWeather);
    delay(1000);
  }
  #ifdef GSM_MODULE
  bGsmOff = true; //now switch gsm off
  while(1){
    //wait until all tasks are stopped
    if (xHandleGsm != NULL) tGSM = eTaskGetState(xHandleGsm);
    if (tGSM == eDeleted) break; //now all tasks are stopped    
    log_i("Gsm=%d",tGSM);
    delay(1000);
  }
  #endif

  #ifdef OLED
  if (status.displayType == OLED0_96){
    display.setTextColor(WHITE);
    display.clearDisplay();
    display.drawXBitmap(0,2,G_Logo_bits,G_Logo_width,G_Logo_height,WHITE);
    display.drawXBitmap(30,2,X_Logo_bits,X_Logo_width,X_Logo_height,WHITE);
    display.drawXBitmap(69,2,AirCom_Logo_bits,AirCom_Logo_width,AirCom_Logo_height,WHITE);
    display.setTextSize(1);
    display.setCursor(85,55);
    display.print(VERSION);
    display.setCursor(0,55);
    display.print(setting.myDevId);
    display.display();
    delay(1000);
    display.clearDisplay();
    display.drawXBitmap(0,2,G_Logo_bits,G_Logo_width,G_Logo_height,WHITE);
    display.drawXBitmap(30,2,X_Logo_bits,X_Logo_width,X_Logo_height,WHITE);
    display.display();
    delay(1000);
    display.clearDisplay();
    display.drawXBitmap(0,2,G_Logo_bits,G_Logo_width,G_Logo_height,WHITE);
    display.display();
    delay(1000);
    oledPowerOff();
  }
  #endif
  if (PinUserLed >= 0){
    pinMode(PinUserLed, OUTPUT);
    digitalWrite(PinUserLed,HIGH); 
  }
  if (PinBuzzer >= 0){
    digitalWrite(PinBuzzer,LOW);     
  }  
  
  log_i("switch power-supply off");  
  delay(100);
  if (status.bHasAXP192){
    // switch all off
    axp.setChgLEDMode(AXP20X_LED_OFF);
    axp.setPowerOutPut(AXP192_LDO2, AXP202_OFF); //LORA
    axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF); //GPS
    
    axp.setPowerOutPut(AXP192_DCDC2, AXP202_OFF); // NC
    axp.setPowerOutPut(AXP192_EXTEN, AXP202_OFF);
    if (status.displayType == OLED0_96){
      axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON); //OLED-Display 3V3
    }else{
      axp.setPowerOutPut(AXP192_DCDC1, AXP202_OFF); //OLED-Display 3V3
    }
    
    delay(20);
    esp_sleep_enable_ext0_wakeup((gpio_num_t) AXP_IRQ, 0); // 1 = High, 0 = Low
  }
  if (PinExtPowerOnOff > 0){
    if (!digitalRead(PinExtPowerOnOff)){
      //pin has to be low, if not, somebody switched off with button !!
      uint64_t mask = uint64_t(1) << uint64_t(PinExtPowerOnOff);
      //uint64_t mask = 0x1000000000;
      esp_sleep_enable_ext1_wakeup(mask,ESP_EXT1_WAKEUP_ANY_HIGH); //wait until power is back again
    }
  }
  //Serial.end();
  esp_wifi_stop();
  esp_bluedroid_disable();
  esp_bluedroid_deinit();
  esp_bt_controller_disable();
  esp_bt_controller_deinit();
  esp_bt_mem_release(ESP_BT_MODE_BTDM);
  adc_power_off();
  esp_deep_sleep_start();

}

#ifdef EINK
void taskEInk(void *pvParameters){
  if (status.displayType != EINK2_9){
    log_i("stop task");
    vTaskDelete(xHandleEInk);
    return;
  }
  while(1){
    if (setting.myDevId.length() > 0) break; //now we are ready
    delay(100);
  }
  Screen screen;
  screen.begin();
  while(1){
    screen.run();
    delay(10);
    if ((WebUpdateRunning) || (bPowerOff)) break;
  }
  if (WebUpdateRunning) screen.webUpdate();
  if (bPowerOff) screen.end();
  log_i("stop task");
  vTaskDelete(xHandleEInk);
}
#endif

void checkExtPowerOff(uint32_t tAct){
  static uint32_t tPowerOff = millis();
  if (PinExtPowerOnOff > 0){
    if (!digitalRead(PinExtPowerOnOff)){
      if (timeOver(tAct,tPowerOff,500)){
        log_i("no external Power --> power-off");
        status.bPowerOff = true;
      }
    }else{
      tPowerOff = tAct;
    }
  }
}

void handleUpdate(uint32_t tAct){
  if (status.tRestart != 0){
    if (timeOver(tAct,status.tRestart,5000)){
      esp_restart();
    }
  }
  if (status.updateState == 1){
    status.updateState = 2;      
    if (updater.checkVersion()){
      status.sNewVersion = updater.getVersion();
      if (updater.checkVersionNewer()){
        status.updateState = 10; //we can update --> newer Version
      }
    }
  }else if (status.updateState == 100){
    status.updateState = 110;
    WebUpdateRunning = true;
    delay(500); //wait 1 second until tasks are stopped
    if (updater.updateVersion()){ //update to newest version  
      status.updateState = 200; //send status update complete
      status.tRestart = millis();
    }else{
      status.updateState = 255; //update failed
      status.tRestart = millis();
    }
    
  }
}

void taskBackGround(void *pvParameters){
  static uint32_t tWifiCheck = millis();
  static uint32_t warning_time=0;
  static uint8_t ntpOk = 0;
  static uint32_t tGetTime = millis();
  uint32_t tBattEmpty = millis();
  uint32_t tRuntime = millis();
  uint32_t tGetWifiRssi = millis();    
  bool bPowersaveOk = false;
  #ifdef GSMODULE
  static uint32_t tCheckDayTime = millis();
  bool bSunsetOk = false;
  Dusk2Dawn dusk2dawn(setting.gs.lat,setting.gs.lon, 0);
  #endif

  if (startOption != 0){ //we start wifi
    log_i("stop task");
    vTaskDelete(xHandleBackground);
    return;
  }

  setupWifi();
  #ifdef GSM_MODULE
    if (setting.wifi.connect == MODE_WIFI_DISABLED){      
      updater.setClient(&GsmUpdaterClient);
      updater.setMutex(&xGsmMutex);
    }
  #endif

  while (1){
    uint32_t tAct = millis();
    handleUpdate(tAct);
    #ifdef GSMODULE
    if (setting.Mode == MODE_GROUND_STATION){
      if (setting.gs.PowerSave == GS_POWER_SAFE){
        if (timeOver(tAct,tRuntime,180000)) bPowersaveOk = true; //after 3min. esp is allowed to go to sleep, so that anybody has a chance to change settings
        //bPowersaveOk = true; //after 3min. esp is allowed to go to sleep, so that anybody has a chance to change settings
        if (status.bTimeOk == true){
          if (!bSunsetOk){
            bSunsetOk = true;
            /*  Time is returned in minutes elapsed since midnight. If no sunrises or
            *  sunsets are expected, a "-1" is returned.
            */
            iSunRise = dusk2dawn.sunrise(year(), month(), day(), false);
            //iSunRise = 900;
            iSunSet = dusk2dawn.sunset(year(), month(), day(), false);
            if (iSunRise < 0) iSunRise += 1440;
            if (iSunSet < 0) iSunSet += 1440;
            char time1[] = "00:00";
            char time2[] = "00:00";
            Dusk2Dawn::min2str(time1, iSunRise);
            Dusk2Dawn::min2str(time2, iSunSet);
            log_i("sunrise=%d sunset=%d",iSunRise,iSunSet);
            log_i("sunrise=%s sunset=%s",time1,time2);
          }else{
            //check every 10 seconds
            
            if (timeOver(tAct,tCheckDayTime,10000)){
              tCheckDayTime = tAct;
              if (bPowersaveOk){              
                if (isDayTime() == 0){
                  enterDeepsleep();
                }
              }
            }
          }
        }
      }
    }
    #endif
    if ((WiFi.status() == WL_CONNECTED) || ((status.modemstatus == MODEM_CONNECTED) && (setting.wifi.connect == MODE_WIFI_DISABLED))){
      status.bInternetConnected = true;
    }else{
      status.bInternetConnected = false;
    }
    if (timeOver(tAct,tGetWifiRssi,5000)){
      tGetWifiRssi = tAct;
      status.wifiRssi = WiFi.RSSI();
      //log_i("wifi-strength=%d",status.wifiRssi);
    }
    if (WiFi.status() == WL_CONNECTED){
      if ((!ntpOk) && (timeOver(tAct,tGetTime,5000))){
        log_i("get ntp-time");
        configTime(0, 0, "pool.ntp.org");
        adjustTime(0);
        struct tm timeinfo;
        if(getLocalTime(&timeinfo)){
          //log_i("h=%d,min=%d,sec=%d,day=%d,month=%d,year=%d",timeinfo.tm_hour,timeinfo.tm_min, timeinfo.tm_sec, timeinfo.tm_mday,timeinfo.tm_mon+1, timeinfo.tm_year + 1900);
          setAllTime(timeinfo);
          struct tm now;
          getLocalTime(&now,0);
          if (now.tm_year >= 117) Serial.println(&now, "%B %d %Y %H:%M:%S (%A)");


        }
        tGetTime = tAct;
        if (printLocalTime() == true){
          ntpOk = 1;
          status.bTimeOk = true;
        } 
      }
    #ifdef GSM_MODULE
    }else if ((status.modemstatus == MODEM_CONNECTED) && (setting.wifi.connect == MODE_WIFI_DISABLED)){
      if ((!ntpOk) && (timeOver(tAct,tGetTime,5000))){
        log_i("get ntp-time");
        byte ret = -1;
        xSemaphoreTake( xGsmMutex, portMAX_DELAY );
        ret = modem.NTPServerSync("pool.ntp.org",0);
        xSemaphoreGive( xGsmMutex );
        //log_i("ret=%d",ret);
        if (ret >= 0){
          int year3 = 0;
          int month3 = 0;
          int day3 = 0;
          int hour3 = 0;
          int min3 = 0;
          int sec3 = 0;
          float timezone = 0;
          xSemaphoreTake( xGsmMutex, portMAX_DELAY );
          bool bret = modem.getNetworkTime(&year3, &month3, &day3, &hour3, &min3, &sec3,&timezone);
          xSemaphoreGive( xGsmMutex );
          log_i("h=%d,min=%d,sec=%d,day=%d,month=%d,year=%d,ret=%d",hour3,min3, sec3, day3,month3, year3,bret);
          if (bret){
            //log_i("set time");
            adjustTime(0);
            struct tm timeinfo;
            timeinfo.tm_year = year3 - 1900;
            timeinfo.tm_mon = month3-1;
            timeinfo.tm_mday = day3;
            timeinfo.tm_hour = hour3;
            timeinfo.tm_min = min3;
            timeinfo.tm_sec = sec3;
            setAllTime(timeinfo);

            //setTime(hour3,min3, sec3, day3,month3, year3);
            
            //log_i("timestatus = %d",timeStatus());
          }
        }        
        tGetTime = tAct;
        //log_i("print time");        
        if (printLocalTime() == true){
          ntpOk = 1;
          status.bTimeOk = true;
        } 
      }
    #endif
    }else{
      ntpOk = 0;
      tGetTime = tAct;
    }
    if ((setting.wifi.connect == WIFI_CONNECT_ALWAYS) && (WiFi.status() != WL_CONNECTED) && (status.wifiStat)){
      if (timeOver(tAct,tWifiCheck,WIFI_RECONNECT_TIME)){
        tWifiCheck = tAct;
        log_i("WiFi not connected. Try to reconnect");
        WiFi.disconnect();
        WiFi.mode(WIFI_OFF);
        WiFi.persistent(false);
        WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE); // call is only a workaround for bug in WiFi class
        WiFi.mode(WIFI_MODE_APSTA);
        WiFi.begin(setting.wifi.ssid.c_str(), setting.wifi.password.c_str()); 
        WiFi.setHostname(host_name.c_str());
      }
    }
    /*
    uint32_t freeHeap = xPortGetFreeHeapSize();
    if (freeHeap<30000){
      if (millis()>warning_time)
      {
        log_w( "*****LOOP current free heap: %d, minimum ever free heap: %d ******", freeHeap, xPortGetMinimumEverFreeHeapSize());
        warning_time=millis()+1000;
      }
    }
    if (freeHeap<10000)
    {
      log_e( "*****LOOP current free heap: %d, minimum ever free heap: %d ******", freeHeap, xPortGetMinimumEverFreeHeapSize());
      log_e("System Low on Memory - xPortGetMinimumEverFreeHeapSize < 2KB");
      //log_e("ESP Restarting !");
      //esp_restart();
    }
    */

    if  (status.wifiStat){
      Web_loop();
    }
    if (wifiCMD == 11) setWifi(true); //switch wifi on
    if (wifiCMD == 10) setWifi(false); //switch wifi off
    if (( tAct > (setting.wifi.tWifiStop * 1000)) && (setting.wifi.tWifiStop!=0) && (!WebUpdateRunning)){
      log_i("******************WEBCONFIG Setting - WIFI STOPPING*************************");
      log_i("currHeap:%d,minHeap:%d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
      setWifi(false);
      //delay(3000);
      //log_i("currHeap:%d,minHeap:%d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
      /*
      if ((setting.outputMode == OUTPUT_BLE) || (setting.outputMode == OUTPUT_BLUETOOTH)){
        if (psRamSize == 0){
          startOption = 1; //start without wifi, but with bluetooth enabled
          ESP.restart(); //we restart without wifi
        }
      }
      */
    }
    //yield();
    if (status.bPowerOff){
      powerOff(); //power off, when battery is empty !!
    }
    if (status.bHasAXP192){ //power off only if we have an AXP192, otherwise, we can't switch on again.
      if ((status.vBatt < battEmpty) && (status.vBatt >= BATTPINOK)) { // if Batt-voltage is below 1V, maybe the resistor is missing.
        if (timeOver(tAct,tBattEmpty,60000)){ //min 60sek. below
          log_i("Batt empty voltage=%d.%dV",status.vBatt/1000,status.vBatt%1000);
          powerOff(); //power off, when battery is empty !!
        }
      }else{
        tBattEmpty = millis();
      }
    }
    checkExtPowerOff(tAct);
    delay(1);
	}
}
