#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <WiFiClient.h>
#include <SPI.h>
//#include <LoRa.h>
#include <FanetLora.h>
#include <Flarm.h>
#include <MicroNMEA.h>
#include <axp20x.h>
#include <main.h>
#include <config.h>
#include "WebHelper.h"
#include "fileOps.h"
#include <SPIFFS.h>
#include <ble.h>
#include <icons.h>
//#include <U8g2lib.h>

//Libraries for OLED Display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BluetoothSerial.h>

//define programming OTA
//#define OTAPROGRAMMING
#ifdef OTAPROGRAMMING
  #include <ArduinoOTA.h>
#endif

#define LifeCount 5000

bool WebUpdateRunning = false;

struct SettingsData setting;
struct statusData status;
String host_name = APPNAME "-";

const char compile_date[] = __DATE__ " " __TIME__;

//WebServer server(80);

FanetLora fanet;
//NmeaOut nmeaout;
Flarm flarm;
HardwareSerial NMeaSerial(2);
//MicroNMEA library structures
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

Ogn ogn;

FanetLora::trackingData MyFanetData;  


FanetLora::trackingData testTrackingData;
FanetLora::weatherData testWeatherData;
String testString;
uint32_t fanetReceiver;
uint8_t sendTestData = 0;

uint8_t WifiConnectOk = 0;
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
uint8_t btOk;

const byte ADCBOARDVOLTAGE_PIN = 35; // Prefer Use of ADC1 (8 channels, attached to GPIOs 32 - 39) . ADC2 (10 channels, attached to GPIOs 0, 2, 4, 12 - 15 and 25 - 27)
const byte ADC_BITS = 10; // 10 - 12 bits

TwoWire i2cOLED = TwoWire(1);

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &i2cOLED);

 unsigned long ble_low_heap_timer=0;
 bool deviceConnected = false;
 String ble_data="";
 bool ble_mutex=false;

TaskHandle_t xHandleStandard = NULL;
TaskHandle_t xHandleBackground = NULL;
TaskHandle_t xHandleBle = NULL;
TaskHandle_t xHandleMemory = NULL;

/********** function prototypes ******************/
void setupAXP192();
void taskStandard(void *pvParameters);
void taskBackGround(void *pvParameters);
void taskBle(void *pvParameters);
void taskMemory(void *pvParameters);
#ifdef OTAPROGRAMMING
void setupOTAProgramming();
#endif
void setupWifi();
void IRAM_ATTR ppsHandler(void);
void startOLED();
void printSettings();
void listSpiffsFiles();
void readGPS();
void printGPSData(uint32_t tAct);
void DrawRadarScreen(uint32_t tAct,uint8_t mode);
void DrawRadarPilot(uint8_t neighborIndex);
void printGSData(uint32_t tAct);
void printBattVoltage(uint32_t tAct);
void printScanning(uint32_t tAct);
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
void sendAWGroundStationdata(uint32_t tAct);
void sendAWUdp(String msg);
void checkFlyingState(uint32_t tAct);
void DrawAngleLine(int16_t x,int16_t y,int16_t length,float deg);
void sendFlarmData(uint32_t tAct);


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
  if (status.flying){
    //flying
    if ((!status.GPS_Fix) || (status.GPS_speed < MIN_FLIGHT_SPEED)){
      if (timeOver(tAct,tOk,MIN_GROUND_TIME)){
        status.flying = false;
      }
    }else{
      tOk = tAct;
    }
  }else{
    //on ground
    if ((status.GPS_Fix) && (status.GPS_speed > MIN_FLIGHT_SPEED)){
      if (timeOver(tAct,tOk,MIN_FLIGHT_TIME)){
        status.flying = true;
      }
    }else{
      tOk = tAct;
    }
  }

}


void printGSData(uint32_t tAct){
  static uint32_t tRefresh = millis();
  static uint8_t index = 0;
  //if ((tAct - tRefresh) >= 1000){
    tRefresh = tAct;
    char buf[10];
    display.clearDisplay();

    if (WiFi.status() == WL_CONNECTED){
      display.drawXBitmap(114,0,WIFI_bits,WIFI_width,WIFI_height,WHITE);
    }

    //show rx-count
    display.setTextSize(1);

    display.setCursor(90,0);
    sprintf(buf, "%3d", uint8_t(status.fanetRx));
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
    display.setCursor(45,0);
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

  //}
}

void sendAWGroundStationdata(uint32_t tAct){
  static uint32_t tSend = millis() - 290000; //10sec. delay, to get wifi working
  if (!setting.GSMode) return;

  if ((tAct - tSend) < 300000) return; //every 5min.

  tSend = tAct;
  char chs[20];
  String msg = ">GS Location,"
            + setting.GSAWID + ",";
  sprintf(chs,"%02.6f",setting.GSLAT);
  msg += String(chs) + ",";
  sprintf(chs,"%02.6f",setting.GSLON);
  msg += String(chs) + ",";
  sprintf(chs,"%d",uint16_t(setting.GSAlt));
  msg += String(chs);
  //log_i("%s",msg.c_str());
  sendAWUdp(msg);
}

void sendAWTrackingdata(FanetLora::trackingData *FanetData){
  if ((!setting.awLiveTracking) && (!setting.GSMode )) return;

  char chs[20];
  String msg;
  if (setting.GSMode){
    msg = "000000,";
  }else{
    msg = nmea.getFixTime() + ",";
  }
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
  //log_e("%s",msg.c_str());
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
    if (btOk == 1){
      if (SerialBT.hasClient()){
        //log_i("sending to bt-device %s",data.c_str());
        SerialBT.print(data);
      }    
    }
  }else if (setting.outputMode == OUTPUT_BLE){ //output over ble-connection
    if (xHandleBle){
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
    log_v("init AXP192 -->");
    axp.begin(i2cOLED, AXP192_SLAVE_ADDRESS);

    axp.setChgLEDMode(AXP20X_LED_LOW_LEVEL);

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
    log_v("ready");
    status.bHasAXP192 = true;
  }else{
    log_e("AXP192 not found");
    status.bHasAXP192 = false;
  }
}

void IRAM_ATTR ppsHandler(void){
  ppsTriggered = true;
}

/*
void listConnectedStations(){
  wifi_sta_list_t wifi_sta_list;
  tcpip_adapter_sta_list_t adapter_sta_list;
 
  memset(&wifi_sta_list, 0, sizeof(wifi_sta_list));
  memset(&adapter_sta_list, 0, sizeof(adapter_sta_list));
 
  esp_wifi_ap_get_sta_list(&wifi_sta_list);
  tcpip_adapter_get_sta_list(&wifi_sta_list, &adapter_sta_list);
  if (adapter_sta_list.num > 0){
    WifiConnectOk = 2;
  }else{
    WifiConnectOk = 1;
  }
  for (int i = 0; i < adapter_sta_list.num; i++) {
 
 
    tcpip_adapter_sta_info_t station = adapter_sta_list.sta[i];
    log_v("station nr %d",i);
    log_v("MAC: ");
 
    log_v("%02X:%02X:%02X:%02X:%02X:%02X", station.mac[0], station.mac[1], station.mac[2], station.mac[3], station.mac[4], station.mac[5]);  
    String sIP = ip4addr_ntoa(&(station.ip));
    log_v("IP: %s",sIP);    
  }
}
*/

void WiFiEvent(WiFiEvent_t event){
  switch(event){
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
      log_d("SYSTEM_EVENT_STA_GOT_IP!!!!!!!!!!!!");
      break;
    case SYSTEM_EVENT_AP_STAIPASSIGNED:
      newStationConnected = true;
      log_d("SYSTEM_EVENT_AP_STAIPASSIGNED!!!!!!!!!!!!");
      break;

    default:
      log_d("Unhandled WiFi Event: %d", event );
      break;
  }
}



void setupWifi(){
  WifiConnectOk = 0;
  WiFi.mode(WIFI_OFF);
  delay(500);
  WiFi.persistent(false);
  log_i("Setting soft-AP ... ");
  //if (WiFi.softAP(host_name.c_str(), setting.appw.c_str(),rand() % 12 + 1,0,2)){
    if (WiFi.softAP(host_name.c_str(), setting.appw.c_str())){
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
  WiFi.onEvent(WiFiEvent);

  log_i("hostname=%s",host_name.c_str());
  WiFi.setHostname(host_name.c_str());
  //now configure access-point
  //so we have wifi connect and access-point at same time
  //we connecto to wifi
  if ((setting.ssid.length() > 0) && (setting.password.length() > 0)){
    //esp_wifi_set_auto_connect(true);
    log_i("Try to connect to WiFi ...");
    WiFi.status();
    WiFi.mode(WIFI_MODE_APSTA);
    //WiFi.mode(WIFI_STA);
    if ((WiFi.SSID() != setting.ssid || WiFi.psk() != setting.password)){
      // ... Try to connect to WiFi station.
      WiFi.begin(setting.ssid.c_str(), setting.password.c_str());
      delay(2000);
    } else {
      // ... Begin with sdk config.
      WiFi.begin();
      delay(2000);
    }
    log_i("Wait for WiFi connection.");
    uint32_t wifiTimeout = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - wifiTimeout < 10000) {
      delay(500);
      //log_i(".");
    }
    
  }
  if(WiFi.status() == WL_CONNECTED){
    // ... print IP Address
    status.myIP = WiFi.localIP().toString();
    log_i("my IP=%s",status.myIP.c_str());
    WifiConnectOk = 2;
  } else{
    log_i("Can not connect to WiFi station. Go into AP mode.");
    // Go into software AP mode.
    WiFi.status();
    WiFi.mode(WIFI_AP);
    delay(10);
    //WiFi.setHostname(host_name.c_str());
    WifiConnectOk = 1;
  }
  //status.myIP = WiFi.softAPIP().toString();
  log_i("my APIP=%s",local_IP.toString().c_str());
  Web_setup();
}

#ifdef OTAPROGRAMMING
void setupOTAProgramming(){
    ArduinoOTA
    .onStart([]() {
      String type;
      log_i("stopping standard-task");
      vTaskDelete(xHandleStandard); //delete standard-task
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";
        SPIFFS.end();

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      setting.wifiDownTime = 0; //don't switch off Wifi during upload
      log_i("Start updating %s",type);
    })
    .onEnd([]() {
      log_i("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      log_v("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      log_e("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) log_e("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) log_e("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) log_e("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) log_e("Receive Failed");
      else if (error == OTA_END_ERROR) log_e("End Failed");
    });

  ArduinoOTA.begin();  
}
#endif

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
  //delay(1000);

  /*
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(0,0);
  display.print(APPNAME);
  display.setCursor(0,18);
  display.print(VERSION);
  display.display();
  */
}

void printSettings(){
  log_i("**** SETTINGS ****");
  log_i("Access-point password=%s",setting.appw.c_str());
  log_i("Board-Type=%d",setting.boardType);
  log_i("AXP192=%d",uint8_t(status.bHasAXP192));
  if (setting.band == 0){
    log_i("BAND=868mhz");
  }else{
    log_i("BAND=915mhz");
  }
  log_i("BAND=%d",setting.band);
  log_i("LORA_POWER=%d",setting.LoraPower);
  log_i("OUTPUT LK8EX1=%d",setting.outputLK8EX1);
  log_i("OUTPUT FLARM=%d",setting.outputFLARM);
  log_i("OUTPUT GPS=%d",setting.outputGPS);
  log_i("OUTPUT FANET=%d",setting.outputFANET);
  log_i("WIFI SSID=%s",setting.ssid.c_str());
  log_i("WIFI PW=%s",setting.password.c_str());
  log_i("Aircraft=%s",fanet.getAircraftType(setting.AircraftType).c_str());
  log_i("Pilotname=%s",setting.PilotName.c_str());
  log_i("Switch WIFI OFF after 3 min=%d",setting.bSwitchWifiOff3Min);
  log_i("Wifi-down-time=%d",setting.wifiDownTime/1000.);
  log_i("Output-Mode=%d",setting.outputMode);
  log_i("UDP_SERVER=%s",setting.UDPServerIP.c_str());
  log_i("UDP_PORT=%d",setting.UDPSendPort);
  log_i("TESTMODE=%d",setting.testMode);
  log_i("UDP_SERVER=%s",setting.UDPServerIP.c_str());
  log_i("UDP_PORT=%d",setting.UDPSendPort);
  log_i("GS Mode=%d",setting.GSMode);
  log_i("GS LAT=%02.6f",setting.GSLAT);
  log_i("GS LON=%02.6f",setting.GSLON);
  log_i("GS LON=%0.2f",setting.GSAlt);
  log_i("GS AWID=%s",setting.GSAWID);
  log_i("OGN-Livetracking=%d",setting.OGNLiveTracking);
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

void setup() {
  
  
  // put your setup code here, to run once:  
  Serial.begin(115200);

  /*
  log_e("*********** LOG Error *********");
  log_w("*********** LOG Warning *********");
  log_i("*********** LOG Info *********");
  log_d("*********** LOG Debug *********");
  log_v("*********** LOG Verbose *********");
  
  */
  log_i("SDK-Version=%s",ESP.getSdkVersion());
  log_i("CPU-Speed=%d",ESP.getCpuFreqMHz());
  log_i("Total heap: %d", ESP.getHeapSize());
  log_i("Free heap: %d", ESP.getFreeHeap());
  log_i("Total PSRAM: %d", ESP.getPsramSize());
  log_i("Free PSRAM: %d", ESP.getFreePsram());
  log_i("compiled at %s",compile_date);



  //esp_sleep_wakeup_cause_t reason = print_wakeup_reason(); //print reason for wakeup
  print_wakeup_reason(); //print reason for wakeup


    // Make sure we can read the file system
  if( !SPIFFS.begin(true)){
    log_e("Error mounting SPIFFS");
    while(1);
  }

  //listSpiffsFiles();
  load_configFile(); //load configuration

  //setting.ssid = "WLAN_EICHLER";
  //setting.password = "magest172";

  pinMode(BUTTON2, INPUT_PULLUP);

  if (setting.GSMode){ // we are ground-station
    setting.outputMode = OUTPUT_SERIAL;
    /*
    setting.outputFANET = 0;
    setting.outputFLARM = 0;
    setting.outputGPS = 0;
    setting.outputLK8EX1 = 0;
    */
    setting.bSwitchWifiOff3Min = false;
  }
  

  //setting.ssid = "WLAN_EICHLER";
  //setting.password = "magest172";
  if (setting.outputMode == OUTPUT_UDP){
    setting.bSwitchWifiOff3Min = false;    
  }
  if (setting.bSwitchWifiOff3Min){
    setting.wifiDownTime = 180000; //switch off after 3 min.
  }else{
    setting.wifiDownTime = 0;
  }
  //setting.bSwitchWifiOff3Min = false;
  //setting.wifiDownTime = 0;

  printSettings();




  if (setting.boardType == BOARD_T_BEAM){    
    i2cOLED.begin(OLED_SDA, OLED_SCL);
    setupAXP192();
  }else if (setting.boardType == BOARD_T_BEAM_V07){
    i2cOLED.begin(OLED_SDA, OLED_SCL);
    status.bHasAXP192 = false;
  }else if (setting.boardType == BOARD_HELTEC_LORA){    
    i2cOLED.begin(4, 15);
    status.bHasAXP192 = false;
  }else{
    log_e("wrong-board-definition --> please correct");
    //wrong board-definition !!
  }

  if (!status.bHasAXP192){
    analogReadResolution(ADC_BITS); // Default of 12 is not very linear. Recommended to use 10 or 11 depending on needed resolution.
    //analogSetAttenuation(ADC_11db); // Default is 11db which is very noisy. Recommended to use 2.5 or 6. Options ADC_0db (1.1V), ADC_2_5db (1.5V), ADC_6db (2.2V), ADC_11db (3.9V but max VDD=3.3V)
    pinMode(ADCBOARDVOLTAGE_PIN, INPUT);
  }

  startOLED();

  btOk = 0;

  xTaskCreatePinnedToCore(taskStandard, "taskStandard", 6500, NULL, 10, &xHandleStandard, ARDUINO_RUNNING_CORE1); //standard task
  xTaskCreatePinnedToCore(taskBackGround, "taskBackGround", 6500, NULL, 5, &xHandleBackground, ARDUINO_RUNNING_CORE1); //background task
  if (setting.outputMode == OUTPUT_BLE){
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    xTaskCreatePinnedToCore(taskBle, "taskBle", 4096, NULL, 7, &xHandleBle, ARDUINO_RUNNING_CORE1);
  }else{
    esp_bt_controller_disable();
    esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
  }
  xTaskCreatePinnedToCore(taskMemory, "taskMemory", 4096, NULL, 1, &xHandleMemory, ARDUINO_RUNNING_CORE1);
  

}

void loop() {
  // put your main code here, to run repeatedly:
  //delay(1000); //wait 1second
}

void taskMemory(void *pvParameters) {

	 while (1)
	 {
     log_d("current free heap: %d, minimum ever free heap: %d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
     vTaskDelay(1000 / portTICK_PERIOD_MS);
   }
}

void taskBle(void *pvParameters) {

	// BLEServer *pServer;

	 delay(2000);
	 start_ble(host_name);
	 delay(1000);
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

void printBattVoltage(uint32_t tAct){
  static uint32_t tBatt = millis();
  if ((tAct - tBatt) >= 5000){
    tBatt = tAct;
    if (status.bHasAXP192){
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

void printScanning(uint32_t tAct){
  static uint32_t tPrint = millis();
  static uint8_t icon = 0;
  //if ((tAct - tPrint) >= 300){
    tPrint = tAct;
    display.clearDisplay();
    switch (icon)
    {
    case 0: 
      display.drawXBitmap(1,6,Antenna_bits,Antenna_width,Antenna_height,WHITE);
      break;
    case 1: 
      display.drawXBitmap(1,6,Antenna_bits,Antenna_width,Antenna_height,WHITE);
      display.drawXBitmap(42,6,WFTX_bits,WFTX_width,WFTX_height,WHITE);
      break;
    case 2: 
      display.drawXBitmap(1,6,Antenna_bits,Antenna_width,Antenna_height,WHITE);
      break;
    case 3: 
      display.drawXBitmap(1,6,Antenna_bits,Antenna_width,Antenna_height,WHITE);
      display.drawXBitmap(42,6,WFTX_bits,WFTX_width,WFTX_height,WHITE);
      display.drawXBitmap(66, 30, WFRX_bits,WFRX_width, WFRX_height,WHITE );
      display.drawXBitmap(88, 6, PGRX_bits,PGRX_width, PGRX_height,WHITE);      
      break;
    case 4: 
      display.drawXBitmap(1,6,Antenna_bits,Antenna_width,Antenna_height,WHITE);
      break;
    case 5: 
      display.drawXBitmap(1,6,Antenna_bits,Antenna_width,Antenna_height,WHITE);
      display.drawXBitmap(42,6,WFTX_bits,WFTX_width,WFTX_height,WHITE);
      display.drawXBitmap(66, 30, WFRX_bits,WFRX_width, WFRX_height,WHITE );
      display.drawXBitmap(88, 6, HGRX_bits,HGRX_width, HGRX_height,WHITE);      
      break;
    case 6: 
      display.drawXBitmap(1,6,Antenna_bits,Antenna_width,Antenna_height,WHITE);
      break;
    case 7: 
      display.drawXBitmap(1,6,Antenna_bits,Antenna_width,Antenna_height,WHITE);
      display.drawXBitmap(42,6,WFTX_bits,WFTX_width,WFTX_height,WHITE);
      display.drawXBitmap(66, 30, WFRX_bits,WFRX_width, WFRX_height,WHITE );
      display.drawXBitmap(88, 6, BLRX_bits,BLRX_width, BLRX_height,WHITE);      
      break;
    case 8: 
      display.drawXBitmap(1,6,Antenna_bits,Antenna_width,Antenna_height,WHITE);
      break;
    case 9: 
      display.drawXBitmap(1,6,Antenna_bits,Antenna_width,Antenna_height,WHITE);
      display.drawXBitmap(42,6,WFTX_bits,WFTX_width,WFTX_height,WHITE);
      display.drawXBitmap(66, 30, WFRX_bits,WFRX_width, WFRX_height,WHITE );
      display.drawXBitmap(88, 6, SPRX_bits,SPRX_width, SPRX_height,WHITE);      
      break;
    case 10: 
      display.drawXBitmap(1,6,Antenna_bits,Antenna_width,Antenna_height,WHITE);
      break;
    case 11: 
      display.drawXBitmap(1,6,Antenna_bits,Antenna_width,Antenna_height,WHITE);
      display.drawXBitmap(42,6,WFTX_bits,WFTX_width,WFTX_height,WHITE);
      display.drawXBitmap(66, 30, WFRX_bits,WFRX_width, WFRX_height,WHITE );
      display.drawXBitmap(88, 6, Airplane40_bits,Airplane40_width, Airplane40_height,WHITE);      
      break;
    case 12: 
      display.drawXBitmap(1,6,Antenna_bits,Antenna_width,Antenna_height,WHITE);
      break;
    case 13: 
      display.drawXBitmap(1,6,Antenna_bits,Antenna_width,Antenna_height,WHITE);
      display.drawXBitmap(42,6,WFTX_bits,WFTX_width,WFTX_height,WHITE);
      display.drawXBitmap(66, 30, WFRX_bits,WFRX_width, WFRX_height,WHITE );
      display.drawXBitmap(88, 6, Helicopter40_bits,Helicopter40_width, Helicopter40_height,WHITE);      
      break;
    case 14: 
      display.drawXBitmap(1,6,Antenna_bits,Antenna_width,Antenna_height,WHITE);
      break;
    case 15: 
      display.drawXBitmap(1,6,Antenna_bits,Antenna_width,Antenna_height,WHITE);
      display.drawXBitmap(42,6,WFTX_bits,WFTX_width,WFTX_height,WHITE);
      display.drawXBitmap(66, 30, WFRX_bits,WFRX_width, WFRX_height,WHITE );
      display.drawXBitmap(88, 6, UAVRX_bits,UAVRX_width, UAVRX_height,WHITE);      
      break;
    case 16: 
      display.drawXBitmap(1,6,Antenna_bits,Antenna_width,Antenna_height,WHITE);
      break;
    case 17: 
      display.drawXBitmap(1,6,Antenna_bits,Antenna_width,Antenna_height,WHITE);
      display.drawXBitmap(42,6,WFTX_bits,WFTX_width,WFTX_height,WHITE);
      display.drawXBitmap(66, 30, WFRX_bits,WFRX_width, WFRX_height,WHITE );
      display.drawXBitmap(88, 6, UFORX_bits,UFORX_width, UFORX_height,WHITE);      
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
  //}
}
void DrawRadarPilot(uint8_t neighborIndex){
  float pilotDistance = 0.0;
  int bearing = 0;
  float rads;
  int relNorth;
  int relEast;

  display.setCursor(95,0);
  display.printf("%4d", fanet.neighbours[neighborIndex].rssi);
  display.setCursor(68,16);
  if (fanet.neighbours[neighborIndex].name.length() > 0){
    display.print(fanet.neighbours[neighborIndex].name.substring(0,10)); //max. 10 signs
  }else{
    display.print(fanet.getDevId(fanet.neighbours[neighborIndex].devId));
  }
  pilotDistance = distance(fanet._myData.lat, fanet._myData.lon,fanet.neighbours[neighborIndex].lat,fanet.neighbours[neighborIndex].lon, 'K') * 1000 ;
  bearing = CalcBearingA( fanet._myData.lat, fanet._myData.lon,fanet.neighbours[neighborIndex].lat,fanet.neighbours[neighborIndex].lon);
  rads = deg2rad(bearing + (fanet._myData.heading * -1));
  relNorth=(int)(((cos(rads) * 16) * -1) + 32-8);
  relEast=(int)((sin(rads) * 16) + 32-8);
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
  /*
  log_i("deg=%0.1f",deg);
  log_i("X-Start=%i",xStart);
  log_i("Y-Start=%i",yStart);
  log_i("X-End=%i",xEnd);
  log_i("Y-End=%i",yEnd);    
  */
}

void DrawRadarScreen(uint32_t tAct,uint8_t mode){
  static uint32_t tPrint = millis();
  static float angle = 0.0;
  static uint8_t neighborIndex = 0;
  int index;
  int16_t xStart;
  int16_t yStart;
  float rads;
  
  //TODO delete !!
  /*
  fanet._myData.lat = 48.072900;
  fanet._myData.lon = 14.734774;
  fanet._myData.altitude = 1000;
  */
  String s = "";
  //if ((tAct - tPrint) >= 1000){
    tPrint = tAct;
    display.clearDisplay();
    //display.drawCircle(32,32,28,WHITE);
    //display.drawCircle(32,32,18,WHITE);
    display.setTextSize(1);
    display.setCursor(0,0);
    display.printf("%d%%",status.BattPerc);
    display.setCursor(0,56);
    if (status.flying){
      display.print("F");
    }else{
      display.print("G");
    }
    display.drawCircle(32,32,25,WHITE);
    //display.drawCircle(32,32,1,WHITE);

    /*
    if (digitalRead(BUTTON2)){
      fanet._myData.heading = 0.0;
    }else{
      fanet._myData.heading = 45.0;
    }
    */

    //fanet._myData.heading = angle;
    //angle += 15.0;
    //if (angle >= 360.0) angle = 0;
    DrawAngleLine(32,32,50,fanet._myData.heading * -1);
    DrawAngleLine(32,32,50,(fanet._myData.heading + 90) * -1);
    rads = deg2rad(fanet._myData.heading * -1);
    xStart=(int)(((sin(rads) * 29) * 1) + 32);
    yStart=(int)(((cos(rads) * 29) * -1) + 32);
    display.setCursor(xStart-2,yStart-3);
    display.print("N");

    //display.drawFastHLine(0,32,64,WHITE);
    //display.drawFastVLine(32,8,56,WHITE);
    display.setTextSize(1);
    display.setCursor(50,0);
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
  //}
}

void printGPSData(uint32_t tAct){
  static uint32_t tPrint = millis();
  String s = "";
  //if ((tAct - tPrint) >= 1000){
    tPrint = tAct;
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0,0);
    display.print(setStringSize(String(status.BattPerc) + "% ",4));
    if (status.flying){
      display.print("F");
    }else{
      display.print("G");
    }


    display.setTextSize(2);
    display.setCursor(55,0);
    //display.print(F("alt:"));
    s = "";
    if (status.GPS_Fix == 1){
      s = setStringSize(String(round(MyFanetData.altitude),0) + "m",6);
    }
    display.print(s);

    display.setTextSize(3);

    display.setCursor(0,20);
    display.print(setStringSize(String(status.ClimbRate,1) + "ms",7));

    display.setTextSize(2);

    display.setCursor(0,45);
    display.print(status.GPS_NumSat);

    display.setCursor(60,45);
    display.print(setStringSize(String(status.GPS_speed,0) + "kh",5));

    display.display();

  //}
}

void readGPS(){
  static char lineBuffer[255];
  static uint16_t recBufferIndex = 0;
  
  while(NMeaSerial.available()){
    if (recBufferIndex >= 255) recBufferIndex = 0; //Buffer overrun
    lineBuffer[recBufferIndex] = NMeaSerial.read();
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

    s += "999999,"; //raw pressure in hPascal: hPA*100 (example for 1013.25 becomes  101325) 
    s += String(status.GPS_alt,2) + ","; // altitude in meters, relative to QNH 1013.25
    s += String((int32_t)(status.ClimbRate * 100.0)) + ","; //climbrate in cm/s
    //s += String(status.) + ",";
    s += "99,"; //temperature
    s += String((float)status.vBatt / 1000.,2) + ",";
    s = flarm.addChecksum(s);
    sendData2Client(s);
    tOld = tAct;
  }
}



void taskStandard(void *pvParameters){
  static uint32_t tLife = millis();
  static uint32_t tLoop = millis();
  static uint8_t counter = 0;
  static uint32_t tFlarmState = millis();
  static float oldAlt = 0.0;
  static uint32_t tOldPPS = millis();
  static uint32_t tDisplay = millis();
  FanetLora::trackingData myFanetData;  
  FanetLora::trackingData tFanetData;  
  
  tFanetData.rssi = 0;


  if (!setting.GSMode){ // we are ground-station)
    if (!status.bHasAXP192){
      NMeaSerial.begin(GPSBAUDRATE,SERIAL_8N1,12,15,false);
    }else{
      NMeaSerial.begin(GPSBAUDRATE,SERIAL_8N1,34,12,false);
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

  }  
  // create a binary semaphore for task synchronization
  long frequency = FREQUENCY868;
  if (setting.band == BAND915)frequency = FREQUENCY915; 
  fanet.begin(SCK, MISO, MOSI, SS,RST, DIO0,frequency,setting.LoraPower);
  fanet.setPilotname(setting.PilotName);
  fanet.setAircraftType(setting.AircraftType);
  if ((setting.testMode == 0) && (!setting.GSMode)){ //not in testmode and not ground-station
    fanet.autobroadcast = true;
  }else{
    fanet.autobroadcast = false;
  }
  setting.myDevId = fanet.getMyDevId();
  host_name += setting.myDevId; //String((ESP32_getChipId() & 0xFFFFFF), HEX);
  if (!setting.GSMode){ // we are ground-station)
    flarm.begin();
  }
  if (setting.outputMode == OUTPUT_BLUETOOTH){
    log_i("starting bluetooth_serial %s",host_name.c_str());
    SerialBT.begin(host_name); //Bluetooth device name
    SerialBT.print(APPNAME " Bluetooth Starting");
    btOk = 1;
  }


  if (setting.OGNLiveTracking){
    ogn.begin(setting.myDevId,APPNAME " " VERSION);
    if (setting.GSMode){
      ogn.setGPS(setting.GSLAT,setting.GSLON,setting.GSAlt,0.0,0.0);
    }
    //ogn.setGPS(status.GPS_Lat,status.GPS_Lon,status.GPS_alt,status.GPS_speed,status.GPS_course);
  } 



  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,55);
  display.print(setting.myDevId);
  display.display();
  delay(3000);

  //udp.begin(UDPPORT);
  tLoop = millis();
  while(1){    
    // put your main code here, to run repeatedly:
    uint32_t tAct = millis();
    status.tLoop = tAct - tLoop;
    tLoop = tAct;
    if (status.tMaxLoop < status.tLoop) status.tMaxLoop = status.tLoop;
    

    if (setting.OGNLiveTracking) ogn.run();
    /*
    if (digitalRead(BUTTON2)){
      status.GPS_Fix = 1;
      status.GPS_speed = 0.0;
    }else{
      status.GPS_Fix = 1;
      status.GPS_speed = 22.0;
    }
    */
    checkFlyingState(tAct);
    printBattVoltage(tAct);
    readGPS();
    sendFlarmData(tAct);
    if (setting.GSMode){
      if (timeOver(tAct,tDisplay,DISPLAY_UPDATE_RATE_GS)){
        tDisplay = tAct;
        //log_i("neghbours=%u",fanet.getNeighboursCount());
        if (fanet.getNeighboursCount() == 0){
          printScanning(tAct);
        }else{
          printGSData(tAct);
        }
      }
    }else{
        switch (setting.screenNumber)
        {
        case 0: //main-Display
          if (timeOver(tAct,tDisplay,DISPLAY_UPDATE_RATE)){
            tDisplay = tAct;
            printGPSData(tAct);          
          }
          break;
        case 1: //radar-screen with list
          if (timeOver(tAct,tDisplay,DISPLAY_UPDATE_RATE2)){
            tDisplay = tAct;
            DrawRadarScreen(tAct,RADAR_LIST);
          }
          break;
        case 2: //radar-screen with closest
          if (timeOver(tAct,tDisplay,DISPLAY_UPDATE_RATE)){
            tDisplay = tAct;
            DrawRadarScreen(tAct,RADAR_CLOSEST);
          }
          break;
        case 3: //list aircrafts
          if (timeOver(tAct,tDisplay,DISPLAY_UPDATE_RATE)){
            tDisplay = tAct;
            //DrawAircraftList();
          }
          break;
        default:
          break;
        }

    }


    if (((timeOver(tAct,tDisplay,DISPLAY_UPDATE_RATE) && (setting.screenNumber != 1))) ||
       ((timeOver(tAct,tDisplay,2000) && (setting.screenNumber == 1)))) {
      tDisplay = tAct;
      if (setting.GSMode){
        //log_i("neghbours=%u",fanet.getNeighboursCount());
        if (fanet.getNeighboursCount() == 0){
          printScanning(tAct);
        }else{
          printGSData(tAct);
        }
      }else{
      }
    }
    /*
    if ((setting.testMode == 0) && (!setting.GSMode)){ //not in testmode and not ground-station
      //writeTrackingData(tAct);
      if ((tAct - tFlarmState) >= 1000){
          tFlarmState = tAct;
          if (setting.outputFLARM) sendData2Client(flarm.writeDataPort());
      }
    }
    */
    if (sendTestData == 1){
      log_i("sending msgtype 1");
      fanet.writeMsgType1(&testTrackingData);
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
      fanet.writeMsgType4(&testWeatherData);
      sendTestData = 0;
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
          ogn.sendTrackingData(tFanetData.lat ,tFanetData.lon,tFanetData.altitude,tFanetData.speed,tFanetData.heading,tFanetData.climb,fanet.getDevId(tFanetData.devId) ,(Ogn::aircraft_t)tFanetData.aircraftType);
        } 
        sendAWTrackingdata(&tFanetData);
        if (nmea.isValid()){
        fanet.getMyTrackingData(&myFanetData);
        //Serial.printf("LAT=%.6f\n",tFanetData.lat);
        //Serial.printf("LON=%.6f\n",tFanetData.lon);

      }
    }
    flarm.run();
    sendLK8EX(tAct);
    if (!status.bHasAXP192){
      if ((tAct - tOldPPS) >= 1000){
        ppsTriggered = true;
      }
    }
    if (ppsTriggered){
      ppsTriggered = false;
      log_v("PPS-Triggered t=%d",status.tGPSCycle);
      //log_e("GPS-FixTime=%s",nmea.getFixTime().c_str());
      status.tGPSCycle = tAct - tOldPPS;
      if (nmea.isValid()){
        long alt = 0;
        nmea.getAltitude(alt);
        status.GPS_Fix = 1;
        status.GPS_speed = nmea.getSpeed()*1.852/1000.; //speed in cm/s --> we need km/h
        status.GPS_Lat = nmea.getLatitude() / 1000000.;
        status.GPS_Lon = nmea.getLongitude() / 1000000.;  
        status.GPS_alt = alt/1000.;
        status.GPS_course = nmea.getCourse()/1000.;
        status.GPS_NumSat = nmea.getNumSatellites();
        if (oldAlt == 0) oldAlt = status.GPS_alt;        
        status.ClimbRate = (status.GPS_alt - oldAlt) / (float(status.tGPSCycle) / 1000.0);
        oldAlt = status.GPS_alt;
        MyFanetData.climb = status.ClimbRate;
        MyFanetData.lat = status.GPS_Lat;
        MyFanetData.lon = status.GPS_Lon;
        MyFanetData.altitude = status.GPS_alt;
        MyFanetData.speed = status.GPS_speed; //speed in cm/s --> we need km/h
        if (status.flying){
          MyFanetData.heading = status.GPS_course;
        }else{
          MyFanetData.heading = 0.0;
        }
        //MyFanetData.heading = status.GPS_course;
        if (setting.OGNLiveTracking){
          ogn.setGPS(status.GPS_Lat,status.GPS_Lon,status.GPS_alt,status.GPS_speed,status.GPS_course);
          ogn.sendTrackingData(status.GPS_Lat,status.GPS_Lon,status.GPS_alt,status.GPS_speed,status.GPS_course,status.ClimbRate,fanet.getMyDevId() ,(Ogn::aircraft_t)fanet.getAircraftType());
        } 

        fanet.setMyTrackingData(&MyFanetData); //set Data on fanet
        if (setting.awLiveTracking){
          char chs[20];
          String msg = nmea.getFixTime() + ","
                    + setting.myDevId + ","
                    + setting.myDevId + ","
                    + String((uint8_t)fanet.getAircraftType()) + ",";
          sprintf(chs,"%02.6f",status.GPS_Lat);
          msg += String(chs) + ",";
          sprintf(chs,"%02.6f",status.GPS_Lon);
          msg += String(chs) + ",0,0,";
          sprintf(chs,"%0.2f",status.GPS_course);
          msg += String(chs) + ",";
          sprintf(chs,"%0.2f",status.GPS_speed * 0.53996);
          msg += String(chs) + ",";
          sprintf(chs,"%0.2f",status.GPS_alt);
          msg += String(chs) + ",0";
          //log_e("%s",msg.c_str());
          sendAWUdp(msg);
        }
      }else{
        status.GPS_Fix = 0;
        status.GPS_speed = 0.0;
        status.GPS_Lat = 0.0;
        status.GPS_Lon = 0.0;
        status.GPS_alt = 0.0;
        status.GPS_course = 0.0;
        status.GPS_NumSat = 0;
        status.ClimbRate = 0.0;
        oldAlt = 0.0;
      }
      tOldPPS = tAct;
    }

    if ((tAct - tLife) >= LifeCount){
      tLife = tAct;
      counter++;
    }
    sendAWGroundStationdata(tAct); //send ground-station-data    
    delay(1);
    if (WebUpdateRunning) break;
  }
  log_i("stopp standard-task");
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(10,5);
  display.print("FW-UPDATE");
  display.setCursor(10,30);
  display.print("wait...");
  display.display();
  fanet.end();
  if (setting.OGNLiveTracking) ogn.end();
  vTaskDelete(xHandleStandard); //delete standard-task
}

void powerOff(){
  axp.clearIRQ();
  log_i("stopping standard-task");
  //vTaskDelete(xHandleStandard); //delete standard-task
  WebUpdateRunning = true;
  delay(100);
  fanet.end();


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

  // switch all off
  axp.setChgLEDMode(AXP20X_LED_OFF);
  axp.setPowerOutPut(AXP192_LDO2, AXP202_OFF); //LORA
  axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF); //GPS
  axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON); //OLED-Display 3V3
  axp.setPowerOutPut(AXP192_DCDC2, AXP202_OFF); // NC
  axp.setPowerOutPut(AXP192_EXTEN, AXP202_OFF);
  delay(20);

  esp_sleep_enable_ext0_wakeup((gpio_num_t) AXP_IRQ, 0); // 1 = High, 0 = Low

  esp_deep_sleep_start();

}


void taskBackGround(void *pvParameters){
  static uint32_t tLife = millis();
  static uint8_t counter = 0;
  static uint32_t warning_time=0;
  delay(1500);
  
  setupWifi();
  #ifdef OTAPROGRAMMING
  //if ((setting.outputMode != OUTPUT_BLUETOOTH) && (setting.outputMode != OUTPUT_BLE)){
    setupOTAProgramming();
  //}
  
  #endif
  while (1){
    uint32_t tAct = millis();
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

    if  (WifiConnectOk){
      Web_loop();
      #ifdef OTAPROGRAMMING
      //if ((setting.outputMode != OUTPUT_BLUETOOTH) && (setting.outputMode != OUTPUT_BLE)){
        ArduinoOTA.handle();
      //}
      #endif
    }
    if (( tAct > setting.wifiDownTime) && (setting.wifiDownTime!=0)){
      setting.wifiDownTime=0;
      WifiConnectOk=0;
      esp_wifi_set_mode(WIFI_MODE_NULL);
      esp_wifi_stop();
      log_i("******************WEBCONFIG Setting - WIFI STOPPING*************************");
    }
    if ((tAct - tLife) >= LifeCount){
      tLife = tAct;
      counter++;

    }
    //yield();
    if ((status.vBatt < BATTEMPTY) && (status.vBatt > 0)) {
      powerOff(); //power off, when battery is empty !!
    }
    if (AXP192_Irq){
      if (axp.readIRQ() == AXP_PASS) {
        if (axp.isPEKLongtPressIRQ()) {
          log_v("Long Press IRQ");
          powerOff();
        }
        if (axp.isPEKShortPressIRQ()) {
          log_v("Short Press IRQ");
          setting.screenNumber ++;
          if (setting.screenNumber > MAXSCREENS) setting.screenNumber = 0;
          write_screenNumber(); //save screennumber in File
        }
        axp.clearIRQ();
      }
      AXP192_Irq = false;
    }
    if (newStationConnected){
      //listConnectedStations();
      newStationConnected = false;
    }
    delay(1);
	}
}
