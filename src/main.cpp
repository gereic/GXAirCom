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
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "SparkFun_Ublox_Arduino_Library.h"
#include <TimeLib.h>
#include <HTTPClient.h>

#ifdef EINK
#include <Screen.h>

Screen screen;

#endif

#ifdef GSMODULE

#include <Weather.h>
#include <WeatherUnderground.h>
#endif

//Libraries for OLED Display
#include <Wire.h>
TwoWire i2cOLED = TwoWire(1);


#ifdef OLED
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &i2cOLED);

#endif

#include <BluetoothSerial.h>

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
//#define PINBUZZER 0
int freq = 2000;
int channel = 0;
int resolution = 8;


#endif

bool WebUpdateRunning = false;
bool bPowerOff = false;

struct SettingsData setting;
struct statusData status;
String host_name = "";
String bleMsg = "";

const char compile_date[] = __DATE__ " " __TIME__;

//WebServer server(80);

FanetLora fanet;
//NmeaOut nmeaout;
Flarm flarm;
HardwareSerial NMeaSerial(2);
//MicroNMEA library structures

Ogn ogn;

FanetLora::trackingData MyFanetData;  


FanetLora::trackingData testTrackingData;
FanetLora::weatherData testWeatherData;
bool sendWeatherData = false;
String testString;
uint32_t fanetReceiver;
uint8_t sendTestData = 0;

IPAddress local_IP(192,168,4,1);
IPAddress gateway(192,168,4,250);
IPAddress subnet(255,255,255,0);

volatile bool ppsTriggered = false;

AXP20X_Class axp;
#define AXP_IRQ 35
volatile bool AXP192_Irq = false;
volatile float BattCurrent = 0.0;

bool newStationConnected = false;

#define SDA2 13
#define SCL2 14

#define AIRWHERE_UDP_PORT 5555
String airwhere_web_ip = "37.128.187.9";

BluetoothSerial SerialBT;

//TTGO T-Beam V07
const byte ADCBOARDVOLTAGE_PIN = 35; // Prefer Use of ADC1 (8 channels, attached to GPIOs 32 - 39) . ADC2 (10 channels, attached to GPIOs 0, 2, 4, 12 - 15 and 25 - 27)
const byte ADC_BITS = 10; // 10 - 12 bits

#define HELTEC_BAT_PIN 34


 unsigned long ble_low_heap_timer=0;
 String ble_data="";
 bool ble_mutex=false;


static RTC_NOINIT_ATTR uint8_t startOption;
uint32_t psRamSize = 0;

TaskHandle_t xHandleBaro = NULL;
TaskHandle_t xHandleStandard = NULL;
TaskHandle_t xHandleBackground = NULL;
TaskHandle_t xHandleBluetooth = NULL;
TaskHandle_t xHandleMemory = NULL;
TaskHandle_t xHandleEInk = NULL;
TaskHandle_t xHandleWeather = NULL;

/********** function prototypes ******************/
#ifdef GSMODULE
void taskWeather(void *pvParameters);
void sendAWGroundStationdata(uint32_t tAct);

#endif
#ifdef AIRMODULE
void readGPS();
void taskBaro(void *pvParameters);
#endif
#ifdef EINK
void taskEInk(void *pvParameters);
#endif
#ifdef OLED
void startOLED();
void DrawRadarScreen(uint32_t tAct,uint8_t mode);
void DrawRadarPilot(uint8_t neighborIndex);
void printGSData(uint32_t tAct);
void printBattVoltage(uint32_t tAct);
void printScanning(uint32_t tAct);
void printGPSData(uint32_t tAct);
void DrawAngleLine(int16_t x,int16_t y,int16_t length,float deg);
void drawBatt(int16_t x, int16_t y,uint8_t value);
void drawflying(int16_t x, int16_t y, bool flying);
void drawAircraftType(int16_t x, int16_t y, FanetLora::aircraft_t AircraftType);
void drawSatCount(int16_t x, int16_t y,uint8_t value);
void drawspeaker(int16_t x, int16_t y);
void drawBluetoothstat(int16_t x, int16_t y);

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
char* readBtSerial();
void checkReceivedLine(char *ch_str);
void serialBtCallBack(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
void printChipInfo(void);

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
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return false;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  return true;
}

void serialBtCallBack(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  if (event == ESP_SPP_SRV_OPEN_EVT){
    status.bluetoothStat = 2; //client connected
  }else if (event == ESP_SPP_CLOSE_EVT){
    status.bluetoothStat = 1; //client disconnected
  }
  //log_i("Event=%d",event);
}

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
		if (buttonActive == true) {
			if (longPressActive == false) {
				status.bMuting = !status.bMuting; //toggle muting
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
    if ((status.GPS_Fix) && (status.GPS_speed > MIN_FLIGHT_SPEED)){
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
  display.clearDisplay();

  if (status.wifiStat==1) display.drawXBitmap(85,0,WIFI_AP_bits,WIFI_width,WIFI_height,WHITE);
  if (status.wifiStat==2) display.drawXBitmap(85,0,WIFI_Sta_bits,WIFI_width,WIFI_height,WHITE);
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
void sendAWGroundStationdata(uint32_t tAct){
  static uint32_t tSend = millis() - 290000; //10sec. delay, to get wifi working
  if ((tAct - tSend) < 300000) return; //every 5min.

  tSend = tAct;
  char chs[20];
  String msg = ">GS Location,"
            + setting.gs.AWID + ",";
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

  if ((!setting.traccarLiveTracking) || (!setting.TraccarSrv.startsWith("http://")) || (WiFi.status() != WL_CONNECTED)) return;

  HTTPClient http;
  time_t now;
  time(&now);
  char chs[120];
  String msg="?id=FANET_";
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
  msg = setting.TraccarSrv + msg;
  
  //log_i("%s",msg.c_str());
  http.begin(msg.c_str());
  int httpResponseCode = http.GET();
  if (httpResponseCode != 200){
    log_e("resp=%d %s",httpResponseCode,http.getString().c_str());
  }
  //
}

void sendAWTrackingdata(FanetLora::trackingData *FanetData){
  if (!setting.awLiveTracking) return;

  char chs[20];
  String msg;
  #ifdef GSMODULE
    msg = "000000,";
  #endif
  #ifdef AIRMODULE
  if (nmea.getFixTime().length() == 0){
    msg = "000000,";
  }else{
    msg = nmea.getFixTime() + ",";
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
  }else if (setting.outputMode == OUTPUT_SERIAL){//output over serial-connection
    Serial.print(data); 
  }else if (setting.outputMode == OUTPUT_BLUETOOTH){//output over bluetooth serial
    if (status.bluetoothStat == 2){
      //if (SerialBT.hasClient()){
        //log_i("sending to bt-device %s",data.c_str());
      SerialBT.print(data);
      //}    
    }
  }else if (setting.outputMode == OUTPUT_BLE){ //output over ble-connection
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
      newStationConnected = true;
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
      newStationConnected = true;
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
  //reset OLED display via software
  if (setting.boardType == BOARD_HELTEC_LORA){
    log_i("Heltec-board");
    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, LOW);
    delay(100);
    digitalWrite(OLED_RST, HIGH);
    delay(100);
  }

  //initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x32
    log_e("SSD1306 allocation failed");
    for(;;); // Don't proceed, loop forever
  }
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
  log_i("GS LAT=%02.6f",setting.gs.lat);
  log_i("GS LON=%02.6f",setting.gs.lon);
  log_i("GS ALT=%0.2f",setting.gs.alt);
  log_i("GS AWID=%s",setting.gs.AWID);
  log_i("OGN-Livetracking=%d",setting.OGNLiveTracking);
  log_i("Traccar-Livetracking=%d",setting.traccarLiveTracking);
  log_i("Traccar-Address=%s",setting.TraccarSrv.c_str());
  log_i("Legacy-TX=%d",setting.LegacyTxEnable);
  
  //vario
  log_i("VarioSinkingThreshold=%0.2f",setting.vario.sinkingThreshold);
  log_i("VarioClimbingThreshold=%0.2f",setting.vario.climbingThreshold);
  log_i("VarioNearClimbingSensitivity=%0.2f",setting.vario.nearClimbingSensitivity);
  log_i("VarioVolume=%d",setting.vario.volume);

  //weather-data
  log_i("WD Fanet-Weatherdata=%d",setting.wd.sendFanet);
  log_i("WD tempoffset=%.1f",setting.wd.tempOffset);
  log_i("WUUlEnable=%d",setting.WUUpload.enable);
  log_i("WUUlID=%s",setting.WUUpload.ID.c_str());
  log_i("WUUlKEY=%s",setting.WUUpload.KEY.c_str());

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
  Serial.begin(115200);

  status.bPowerOff = false;
  status.bHasBME = false;
  status.bWUBroadCast = false;

  //for new T-Beam output 4 is red led
  pinMode(4, OUTPUT);
  digitalWrite(4,HIGH); 

  log_e("error");
  
  log_i("SDK-Version=%s",ESP.getSdkVersion());
  log_i("CPU-Speed=%d",ESP.getCpuFreqMHz());
  log_i("Total heap: %d", ESP.getHeapSize());
  log_i("Free heap: %d", ESP.getFreeHeap());
  if (psramFound()){
    psRamSize = ESP.getPsramSize();
    log_i("Total PSRAM: %d", psRamSize);
    log_i("Free PSRAM: %d", ESP.getFreePsram());
  }else{
    psRamSize = 0;
    log_i("No PSRAM found");
  }
  printChipInfo();
  log_i("compiled at %s",compile_date);
  log_i("current free heap: %d, minimum ever free heap: %d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());

  //esp_sleep_wakeup_cause_t reason = print_wakeup_reason(); //print reason for wakeup
  print_wakeup_reason(); //print reason for wakeup
  esp_reset_reason_t reason = esp_reset_reason();
  if (reason != ESP_RST_SW) {
    startOption = 0;
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
  #ifdef GSMODULE
    setting.Mode = MODE_GROUND_STATION;
  #endif
  #ifdef AIRMODULE
    setting.Mode = MODE_AIR_MODULE;
  #endif
  if ((setting.wifi.ssid.length() <= 0) || (setting.wifi.password.length() <= 0)){
    setting.wifi.connect = WIFI_CONNECT_NONE; //if no pw or ssid given --> don't connecto to wifi
  }

  pinMode(BUTTON2, INPUT_PULLUP);

  printSettings();


  if (setting.boardType == BOARD_T_BEAM){    
    i2cOLED.begin(21, 22);
    setupAXP192();
  }else if ((setting.boardType == BOARD_T_BEAM_V07) || (setting.boardType == BOARD_TTGO_T3_V1_6)){
    i2cOLED.begin(21, 22);
    status.bHasAXP192 = false;
    analogReadResolution(ADC_BITS); // Default of 12 is not very linear. Recommended to use 10 or 11 depending on needed resolution.
    //analogSetAttenuation(ADC_11db); // Default is 11db which is very noisy. Recommended to use 2.5 or 6. Options ADC_0db (1.1V), ADC_2_5db (1.5V), ADC_6db (2.2V), ADC_11db (3.9V but max VDD=3.3V)
    pinMode(ADCBOARDVOLTAGE_PIN, INPUT);
  }else if (setting.boardType == BOARD_HELTEC_LORA){    
    i2cOLED.begin(4, 15);
    status.bHasAXP192 = false;
    analogReadResolution(12); //12 Bit resolution
    pinMode(HELTEC_BAT_PIN, INPUT); //input-Voltage on GPIO34
  }else{
    log_e("wrong-board-definition --> please correct");
    //wrong board-definition !!
  }

  #ifdef OLED
  startOLED();  
  #endif
  setting.myDevId = "";

  //log_i("currHeap:%d,minHeap:%d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
#ifdef AIRMODULE
  xTaskCreatePinnedToCore(taskBaro, "taskBaro", 6500, NULL, 100, &xHandleBaro, ARDUINO_RUNNING_CORE1); //high priority task
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
  //start weather-task
  xTaskCreatePinnedToCore(taskWeather, "taskWeather", 4096, NULL, 8, &xHandleWeather, ARDUINO_RUNNING_CORE1);
#endif  
}

#ifdef GSMODULE
void taskWeather(void *pvParameters){
  static uint32_t tSendData = millis() - WEATHER_UPDATE_RATE + 10000; //first sending is in 10 seconds
  static uint32_t tUploadData = millis() - WEATHER_UNDERGROUND_UPDATE_RATE + 10000; //first sending is in 10seconds
  bool bDataOk = false;
  log_i("starting weather-task ");  
  Weather::weatherData wData;
  TwoWire i2cWeather = TwoWire(0);
  uint8_t oneWirePin = -1;
  if (setting.boardType == BOARD_HELTEC_LORA){
    oneWirePin = 22;
    i2cWeather.begin(13,23,400000); //init i2cBaro for Baro
  }else{
    i2cWeather.begin(13,14,400000); //init i2cBaro for Baro
  }
  Weather weather;
  weather.setTempOffset(setting.wd.tempOffset);
  if (weather.begin(&i2cWeather,setting.gs.alt,oneWirePin)){
    status.bHasBME = true; //we have a bme-sensor
  }
  if (setting.WUUpload.enable){
    status.bWUBroadCast = true;
  }
  if ((!status.bHasBME) && (!status.bWUBroadCast)){
    log_i("stopping task");
    vTaskDelete(xHandleWeather);
    return;    
  }
  const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;   //only every 1sek.
  TickType_t xLastWakeTime = xTaskGetTickCount (); //get actual tick-count
  while (1){
    uint32_t tAct = millis();
    if (status.bHasBME){
      //station has BME --> we are a weather-station
      weather.run();
      if (setting.wd.sendFanet){
        if (timeOver(tAct,tSendData,WEATHER_UPDATE_RATE)){
          weather.getValues(&wData);
          testWeatherData.lat = setting.gs.lat;
          testWeatherData.lon = setting.gs.lon;
          testWeatherData.bWind = true;
          testWeatherData.wHeading = 0;
          testWeatherData.wSpeed = 0;
          testWeatherData.wGust = 0;      
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
      if (timeOver(tAct,tUploadData,WEATHER_UNDERGROUND_UPDATE_RATE)){
        tUploadData = tAct;
        if (setting.WUUpload.enable){
          WeatherUnderground::wData wuData;
          WeatherUnderground wu;
          weather.getValues(&wData);
          //log_i("temp=%f,humidity=%f",testWeatherData.temp,testWeatherData.Humidity);
          wuData.bWind = false;
          wuData.winddir = testWeatherData.wHeading;
          wuData.windspeed = testWeatherData.wSpeed;
          wuData.windgust = testWeatherData.wGust;
          wuData.humidity = testWeatherData.Humidity;
          wuData.temp = testWeatherData.temp;
          wuData.pressure = testWeatherData.Baro;
          wuData.bRain = false;
          wu.sendData(setting.WUUpload.ID,setting.WUUpload.KEY,&wuData);
        }
      }
    }
    if (status.bWUBroadCast){
      //station should broadcast WU-Data over Fanet
      if (timeOver(tAct,tUploadData,WEATHER_UNDERGROUND_UPDATE_RATE)){ //get Data from WU        
        tUploadData = tAct;
          WeatherUnderground::wData wuData;
          WeatherUnderground wu;
          bDataOk = wu.getData(setting.WUUpload.ID,setting.WUUpload.KEY,&wuData);
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
  ledcSetup(channel, freq, resolution);
  if (setting.boardType == BOARD_HELTEC_LORA){
    ledcAttachPin(17, channel); //Buzzer on Pin 17 --> Prog-Button is on PIN 0
  }else{
    ledcAttachPin(0, channel); //Buzzer on PIN 0
  }
  TwoWire i2cBaro = TwoWire(0);
  if (setting.boardType == BOARD_HELTEC_LORA){
    Wire.begin(13,23,400000);
    i2cBaro.begin(13,23,400000); //init i2cBaro for Baro
  }else{
    Wire.begin(13,14,400000);
    i2cBaro.begin(13,14,400000); //init i2cBaro for Baro
  }
  if (baro.begin(&i2cBaro)){
    status.bHasVario = true;
    Beeper.setThresholds(setting.vario.sinkingThreshold,setting.vario.climbingThreshold,setting.vario.nearClimbingSensitivity);
    Beeper.setVolume(u8Volume);
    //Beeper.setGlidingBeepState(true);
    //Beeper.setGlidingAlarmState(true);
  }else{
    log_i("no baro found --> end baro-task ");  
    status.bHasVario = false;    
  }
  if (status.bHasVario){
    xLastWakeTime = xTaskGetTickCount ();
    while (1){      
      
      //uint32_t tAct = millis();
      //if (u8Volume != setting.vario.volume){
      //  u8Volume = setting.vario.volume;
      //  Beeper.setVolume(u8Volume);
      //}
      if (((!status.flying) && (setting.vario.BeepOnlyWhenFlying)) || (status.bMuting)){
        Beeper.setVolume(0);
      }else{
        Beeper.setVolume(setting.vario.volume);
      }
      baro.run();
      if (baro.isNewVAlues()){
        baro.getValues(&status.pressure,&status.varioAlt,&status.ClimbRate,&status.varioTemp);
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
      vTaskDelayUntil( &xLastWakeTime, xDelay );
      if ((WebUpdateRunning) || (bPowerOff)) break;
    }
  }
  log_i("stop task");
  vTaskDelete(xHandleBaro); //delete baro-task
}
#endif

void loop() {
  // put your main code here, to run repeatedly:
  //delay(1000); //wait 1second
}

void taskMemory(void *pvParameters) {

	 while (1)
	 {
     log_i("current free heap: %d, minimum ever free heap: %d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
     vTaskDelay(1000 / portTICK_PERIOD_MS);
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
    if ((psRamSize > 0) || (startOption == 1)){
      //startBluetooth(); //start bluetooth
    }else{
      log_i("stop task");
      vTaskDelete(xHandleBluetooth);
      return;    
    }
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
    //esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    //esp_bt_controller_init(&cfg);
    esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    log_i("starting bluetooth_serial %s",host_name.c_str());
    SerialBT.begin(host_name.c_str()); //Bluetooth device name
    SerialBT.register_callback(&serialBtCallBack);
    log_i("currHeap:%d,minHeap:%d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
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
  if (setting.boardType == BOARD_HELTEC_LORA){
    analogRead(HELTEC_BAT_PIN); // First measurement has the biggest difference on my board, this line just skips the first measurement
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
      uint16_t thisReading = analogRead(HELTEC_BAT_PIN);
      adc_reading += thisReading;
    }
    adc_reading /= NO_OF_SAMPLES;
    // voltage-divier 27kOhm and 100kOhm
    // vIn = (R1+R2)/R2 * VOut
    vBatt = (100000.0f + 27000.0f) / 100000.0f * (float(adc_reading) / 4095.0f *3.3) ;
    //log_i("adc=%d, vBatt=%.3f",adc_reading,vBatt);
    return vBatt;
    //return (27f/100.0f) * 3.30f * float(analogRead(34)) / 4096.0f;  // LiPo battery
  }else{
    analogRead(ADCBOARDVOLTAGE_PIN); // First measurement has the biggest difference on my board, this line just skips the first measurement
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
      uint16_t thisReading = analogRead(ADCBOARDVOLTAGE_PIN);
      adc_reading += thisReading;
    }
    adc_reading /= NO_OF_SAMPLES;
    // Convert ADC reading to voltage in deciVolt, 1024/2048/4096 not hardcoded but calculated depending on the set ADC_BITS
    byte voltage = adc_reading * 39 * 2 / (1 << ADC_BITS); // 3.9V because of 11dB, 100K/100K Voltage Divider, maxResolution (1024/2048/4096) 
    return (float)voltage / 10.0; 
  }
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
        log_w("no Batt");
        status.vBatt = 0;
      }
    }else{
      status.vBatt = uint16_t(readBattvoltage()*1000);      
    }
    //log_i("Batt =%dV",status.vBatt);
    status.BattPerc = scale(status.vBatt,BATTEMPTY,BATTFULL,0,100);
    //log_i("Batt =%d%%",status.BattPerc);
  }
}

#ifdef OLED
void printScanning(uint32_t tAct){
  static uint8_t icon = 0;
  display.clearDisplay();
  if (status.wifiStat==1) display.drawXBitmap(85,0,WIFI_AP_bits,WIFI_width,WIFI_height,WHITE);
  if (status.wifiStat==2) display.drawXBitmap(85,0,WIFI_Sta_bits,WIFI_width,WIFI_height,WHITE);
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
  if (status.wifiStat==1) display.drawXBitmap(85,0,WIFI_AP_bits,WIFI_width,WIFI_height,WHITE);
  if (status.wifiStat==2) display.drawXBitmap(85,0,WIFI_Sta_bits,WIFI_width,WIFI_height,WHITE);
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
  if (status.wifiStat==1) display.drawXBitmap(85,0,WIFI_AP_bits,WIFI_width,WIFI_height,WHITE);
  if (status.wifiStat==2) display.drawXBitmap(85,0,WIFI_Sta_bits,WIFI_width,WIFI_height,WHITE);
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

void checkReceivedLine(char *ch_str){
  //log_i("new serial msg=%s",ch_str);
  if(!strncmp(ch_str, FANET_CMD_TRANSMIT, 4)){
    fanet.fanet_cmd_transmit(ch_str+4);
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
  }
}

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
    }else{
      if (lineBuffer[recBufferIndex] != '\r'){
        recBufferIndex++;
      }
    }  
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

    if (status.bHasVario){
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



void taskStandard(void *pvParameters){
  static uint32_t tLoop = millis();
  static float oldAlt = 0.0;
  static uint32_t tOldPPS = millis();
  static uint32_t tLastPPS = millis();
  static uint32_t tDisplay = millis();
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
  SFE_UBLOX_GPS ublox;
  bool isConnected = false;
  for (int i = 0; i < 3; i++){
    if (ublox.begin(NMeaSerial)){
      isConnected = true;
      break;
    }
    delay(500);
  }
  if (isConnected){
    log_i("Connected to UBLOX GPS successfully\n");
    ublox.setUART1Output(COM_TYPE_NMEA, 1000);
  }else{
    log_i("UBLOX GPS not connected\n");
  }
  // Change the echoing messages to the ones recognized by the MicroNMEA library
  MicroNMEA::sendSentence(NMeaSerial, "$PSTMSETPAR,1201,0x00000042");
  MicroNMEA::sendSentence(NMeaSerial, "$PSTMSAVEPAR");

  //Reset the device so that the changes could take plaace
  MicroNMEA::sendSentence(NMeaSerial, "$PSTMSRR");

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
  //mode ground-station
  status.GPS_Lat = setting.gs.lat;
  status.GPS_Lon = setting.gs.lon;  
  status.GPS_alt = setting.gs.alt;
  #endif
  // create a binary semaphore for task synchronization
  long frequency = FREQUENCY868;
  if (setting.band == BAND915)frequency = FREQUENCY915; 
  fanet.setLegacy(setting.LegacyTxEnable);
  if (setting.boardType == BOARD_HELTEC_LORA){
    fanet.begin(5, 19, 27, 18,14, 26,frequency,setting.LoraPower);

  }else{
    fanet.begin(SCK, MISO, MOSI, SS,RST, DIO0,frequency,setting.LoraPower);
  }
 
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
  flarm.begin();
  #endif
  if (setting.OGNLiveTracking){
    #ifdef GSMODULE
      if (setting.PilotName.length() > 0){
        ogn.begin(setting.PilotName,VERSION "." APPNAME);
      }else{
        ogn.begin("FNB" + setting.myDevId,VERSION "." APPNAME);
      }      
      ogn.setGPS(setting.gs.lat,setting.gs.lon,setting.gs.alt,0.0,0.0);
    #endif
    #ifdef AIRMODULE
      ogn.begin("FNB" + setting.myDevId,VERSION "." APPNAME);
    #endif
  } 


#ifdef OLED
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,55);
  display.print(setting.myDevId);
  display.display();
  delay(3000);
#endif

  //udp.begin(UDPPORT);
  tLoop = millis();
  while(1){    
    // put your main code here, to run repeatedly:
    uint32_t tAct = millis();
    status.tLoop = tAct - tLoop;
    tLoop = tAct;
    if (status.tMaxLoop < status.tLoop) status.tMaxLoop = status.tLoop;
    
    handleButton(tAct);

    if (setting.OGNLiveTracking) ogn.run();
    checkFlyingState(tAct);
    printBattVoltage(tAct);
    #ifdef AIRMODULE
    readGPS();
    #endif
    sendFlarmData(tAct);
    #ifdef OLED
    #ifdef GSMODULE
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
    #endif
    #ifdef AIRMODULE
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
    #endif
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
      log_i("sending msgtype 3 %s",testString.c_str());
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
      log_i("sending weatherdata");
      fanet.writeMsgType4(&testWeatherData);
      sendWeatherData = false;
    }
    pSerialLine = readSerial();
    if (pSerialLine != NULL){
      checkReceivedLine(pSerialLine);
    }
    if (setting.outputMode == OUTPUT_BLUETOOTH){
      pSerialLine = readBtSerial();
      if (pSerialLine != NULL){
        checkReceivedLine(pSerialLine);
      }
    }    
    fanet.run();
    status.fanetRx = fanet.rxCount;
    status.fanetTx = fanet.txCount;
    if (fanet.isNewMsg()){
      //write msg to udp !!
      String msg = fanet.getactMsg() + "\n";
      if (setting.outputFANET) sendData2Client(msg);
    }
    if (fanet.getTrackingData(&tFanetData)){
        //log_i("new Tracking-Data");
        if (setting.OGNLiveTracking){
          ogn.sendTrackingData(tFanetData.lat ,tFanetData.lon,tFanetData.altitude,tFanetData.speed,tFanetData.heading,tFanetData.climb,fanet.getDevId(tFanetData.devId) ,(Ogn::aircraft_t)tFanetData.aircraftType,(float)tFanetData.snr / 10.0);
        } 
        sendAWTrackingdata(&tFanetData);
        sendTraccarTrackingdata(&tFanetData);
        
       
        //if (nmea.isValid()){
        //  fanet.getMyTrackingData(&myFanetData);
        //}
    }    
    flarm.run();
    sendLK8EX(tAct);
    #ifdef AIRMODULE
    if (!status.bHasAXP192){
      if ((tAct - tOldPPS) >= 1000){
        ppsTriggered = true;
      }
    }
    if (ppsTriggered){
      ppsTriggered = false;
      tLastPPS = tAct;
      log_v("PPS-Triggered t=%d",status.tGPSCycle);
      //log_e("GPS-FixTime=%s",nmea.getFixTime().c_str());
      status.tGPSCycle = tAct - tOldPPS;
      if (nmea.isValid()){
        long alt = 0;
        nmea.getAltitude(alt);
        #ifdef AIRMODULE
        status.GPS_NumSat = nmea.getNumSatellites();
        #else
        status.GPS_NumSat = 0;
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
        if (!status.bHasVario) status.ClimbRate = (status.GPS_alt - oldAlt) / (float(status.tGPSCycle) / 1000.0);        
        oldAlt = status.GPS_alt;
        MyFanetData.climb = status.ClimbRate;
        MyFanetData.lat = status.GPS_Lat;
        MyFanetData.lon = status.GPS_Lon;
        MyFanetData.altitude = status.GPS_alt;
        MyFanetData.speed = status.GPS_speed; //speed in cm/s --> we need km/h
        if ((status.GPS_speed <= 5.0) && (status.bHasVario)){
          MyFanetData.heading = status.varioHeading;
        }else{
          MyFanetData.heading = status.GPS_course;
        }        
        if (setting.OGNLiveTracking){
          ogn.setGPS(status.GPS_Lat,status.GPS_Lon,status.GPS_alt,status.GPS_speed,MyFanetData.heading);
          ogn.sendTrackingData(status.GPS_Lat,status.GPS_Lon,status.GPS_alt,status.GPS_speed,MyFanetData.heading,status.ClimbRate,fanet.getMyDevId() ,(Ogn::aircraft_t)fanet.getAircraftType(),0.0);
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
        if (!status.bHasVario) status.ClimbRate = 0.0;
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
    #endif

    #ifdef GSMODULE
    sendAWGroundStationdata(tAct); //send ground-station-data    
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
  if (WebUpdateRunning){
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(10,5);
  display.print("FW-UPDATE");
  display.setCursor(10,30);
  display.print("wait...");
  display.display();
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
  while(1){
    //wait until all tasks are stopped
    if (xHandleBaro != NULL) tBaro = eTaskGetState(xHandleBaro);
    if (xHandleEInk != NULL) tEInk = eTaskGetState(xHandleEInk);
    if (xHandleStandard != NULL) tStandard = eTaskGetState(xHandleStandard);
    if ((tBaro == eDeleted) && (tEInk == eDeleted) && (tStandard == eDeleted)) break; //now all tasks are stopped    
    //log_i("baro=%d,eink=%d,standard=%d",tBaro,tEInk,tStandard);
    delay(500);
  }

  #ifdef OLED
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
  display.clearDisplay();
  display.ssd1306_command(SSD1306_DISPLAYOFF);
  #endif
  log_i("switch power-supply off");
  delay(100);
    if (status.bHasAXP192){
    // switch all off
    axp.setChgLEDMode(AXP20X_LED_OFF);
    axp.setPowerOutPut(AXP192_LDO2, AXP202_OFF); //LORA
    axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF); //GPS
    if (setting.displayType == OLED0_96){
      axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON); //OLED-Display 3V3
    }else{
      axp.setPowerOutPut(AXP192_DCDC1, AXP202_OFF); //OLED-Display 3V3
    }
    
    axp.setPowerOutPut(AXP192_DCDC2, AXP202_OFF); // NC
    axp.setPowerOutPut(AXP192_EXTEN, AXP202_OFF);
    delay(20);
  }

  esp_sleep_enable_ext0_wakeup((gpio_num_t) AXP_IRQ, 0); // 1 = High, 0 = Low

  esp_deep_sleep_start();

}

#ifdef EINK
void taskEInk(void *pvParameters){
  if (setting.displayType != EINK2_9){
    log_i("stop task");
    vTaskDelete(xHandleEInk);
    return;
  }
  while(1){
    if (setting.myDevId.length() > 0) break; //now we are ready
    delay(100);
  }
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

void taskBackGround(void *pvParameters){
  static uint32_t tWifiCheck = millis();
  static uint32_t warning_time=0;
  static uint8_t ntpOk = 0;
  static uint32_t tGetTime = millis();


  if (startOption != 0){ //we start wifi
    log_i("stop task");
    vTaskDelete(xHandleBackground);
    return;
  }

  setupWifi();
  while (1){
    uint32_t tAct = millis();
    if (WiFi.status() == WL_CONNECTED){
      if ((!ntpOk) && (timeOver(tAct,tGetTime,5000))){
        log_i("get ntp-time");
        configTime(0, 0, "pool.ntp.org");
        tGetTime = tAct;
        if (printLocalTime() == true) ntpOk = 1;
      }
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
        WiFi.mode(WIFI_MODE_APSTA);
        WiFi.begin(setting.wifi.ssid.c_str(), setting.wifi.password.c_str());        
      }
    }
    if (xPortGetMinimumEverFreeHeapSize()<100000)
    {
      if (millis()>warning_time)
      {
        //log_w( "*****LOOP current free heap: %d, minimum ever free heap: %d ******", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
        warning_time=millis()+1000;
      }
    }
	if (xPortGetMinimumEverFreeHeapSize()<5000)
	{
    log_e( "*****LOOP current free heap: %d, minimum ever free heap: %d ******", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
		log_e("System Low on Memory - xPortGetMinimumEverFreeHeapSize < 2KB");
		log_e("ESP Restarting !");
		esp_restart();
	}

    if  (status.wifiStat){
      Web_loop();
    }
    if (( tAct > (setting.wifi.tWifiStop * 1000)) && (setting.wifi.tWifiStop!=0) && (!WebUpdateRunning)){
      log_i("currHeap:%d,minHeap:%d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
      Web_stop();
      WiFi.softAPdisconnect(true);
      WiFi.disconnect();
      WiFi.mode(WIFI_MODE_NULL);
      //WiFi.forceSleepBegin(); //This also works
      setting.wifi.tWifiStop=0;
      status.wifiStat=0;
      esp_wifi_set_mode(WIFI_MODE_NULL);
      esp_wifi_stop();
      log_i("******************WEBCONFIG Setting - WIFI STOPPING*************************");
      log_i("currHeap:%d,minHeap:%d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
      if ((setting.outputMode == OUTPUT_BLE) || (setting.outputMode == OUTPUT_BLUETOOTH)){
        if (psRamSize == 0){
          startOption = 1; //start without wifi, but with bluetooth enabled
          ESP.restart(); //we restart without wifi
        }
      }
    }
    //yield();
    if (status.bPowerOff){
      powerOff(); //power off, when battery is empty !!
    }
    if ((status.vBatt < BATTEMPTY) && (status.vBatt >= BATTPINOK)) { // if Batt-voltage is below 1V, maybe the resistor is missing.
      log_i("Batt empty voltage=%d.%dV",status.vBatt/1000,status.vBatt%1000);
      powerOff(); //power off, when battery is empty !!
    }
    if (newStationConnected){
      //listConnectedStations();
      newStationConnected = false;
    }
    delay(1);
	}
}
