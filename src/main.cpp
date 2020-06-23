#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <WiFiClient.h>
#include <ArduinoOTA.h>
#include <SPI.h>
#include <LoRa.h>
#include <FanetLora.h>
#include <Flarm.h>
#include <MicroNMEA.h>
#include <axp20x.h>
#include <main.h>
#include <config.h>
#include "WebHelper.h"
#include "fileOps.h"
#include <SPIFFS.h>


//Libraries for OLED Display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//define programming OTA
#define OTAPROGRAMMING

#define LifeCount 5000

struct SettingsData setting;
struct statusData status;
String host_name = APPNAME "-";

//WebServer server(80);

FanetLora fanet;
//NmeaOut nmeaout;
Flarm flarm;
HardwareSerial NMeaSerial(2);
//MicroNMEA library structures
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

trackingData MyFanetData;  


trackingData testTrackingData;
weatherData testWeatherData;
String testString;
uint8_t sendTestData = 0;

bool WifiConnectOk = false;
IPAddress local_IP(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);
const char* ap_default_psk = "12345678"; ///< Default PSK.

WiFiUDP udp;

volatile bool ppsTriggered = false;

AXP20X_Class axp;
#define AXP_IRQ 35
volatile bool AXP192_Irq = false;
volatile float BattCurrent = 0.0;

#define SDA2 13
#define SCL2 14

TwoWire i2cOLED = TwoWire(1);

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &i2cOLED, OLED_RST);



TaskHandle_t xHandleStandard = NULL;
TaskHandle_t xHandleBackground = NULL;

/********** function prototypes ******************/
void setupAXP192();
void taskStandard(void *pvParameters);
void taskBackGround(void *pvParameters);
void setupOTAProgramming();
void setupWifi();
void IRAM_ATTR ppsHandler(void);
void startOLED();
void printSettings();
void listSpiffsFiles();
void readGPS();
void printGPSData(uint32_t tAct);
void printBattVoltage(uint32_t tAct);
String setStringSize(String s,uint8_t sLen);
void writeTrackingData(uint32_t tAct);
void sendData2Client(String data);
eFlarmAircraftType Fanet2FlarmAircraft(eFanetAircraftType aircraft);
void Fanet2FlarmData(trackingData *FanetData,FlarmtrackingData *FlarmDataData);
void sendLK8EX(uint32_t tAct);
void powerOff();
esp_sleep_wakeup_cause_t print_wakeup_reason();

void sendData2Client(String data){
  if (setting.outputMode == OUTPUT_UDP){
    //output via udp
    if (WiFi.status() == WL_CONNECTED){
      udp.beginPacket(setting.UDPServerIP.c_str(),setting.UDPSendPort);
      udp.write((uint8_t *)data.c_str(),data.length());
      udp.endPacket();    
    }
  }else{
    Serial.print(data); //output over serial-connection
  }
}


void writeTrackingData(uint32_t tAct){
  static uint32_t tSend = millis();
  if (status.GPS_Fix == 1){
    if ((tAct - tSend) >= 5000){
      MyFanetData.aircraftType = fanet.getAircraftType();
      fanet.writeTrackingData2FANET(&MyFanetData);
      tSend = tAct;
    }
  }else{
    tSend = tAct;
  }
}



static void IRAM_ATTR AXP192_Interrupt_handler() {
  AXP192_Irq = true;
  Serial.println("AXP192 IRQ");
}

void setupAXP192(){
  i2cOLED.beginTransmission(AXP192_SLAVE_ADDRESS);
  if (i2cOLED.endTransmission() == 0) {
    Serial.print("init AXP192 -->");
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
    Serial.println("ready");
  }else{
    Serial.println("AXP192 not found");
  }
}

void IRAM_ATTR ppsHandler(void){
  ppsTriggered = true;
}

void setupWifi(){
  WifiConnectOk = false;
  WiFi.mode(WIFI_STA);
  delay(10);
  if (WiFi.SSID() != setting.ssid || WiFi.psk() != setting.password)
    {
      Serial.println(F("WiFi config changed."));

      // ... Try to connect to WiFi station.
      WiFi.begin(setting.ssid.c_str(), setting.password.c_str());

      // ... Pritn new SSID
      Serial.print(F("new SSID: "));
      Serial.println(WiFi.SSID());

    }
    else
    {
      // ... Begin with sdk config.
      WiFi.begin();
    }
    WiFi.setHostname(host_name.c_str());
    Serial.println(F("Wait for WiFi connection."));
    uint32_t wifiTimeout = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - wifiTimeout < 10000) {
      delay(500);
      Serial.print(".");
    }Serial.println("");
  //}
  if(WiFi.status() == WL_CONNECTED){
    // ... print IP Address
    status.myIP = WiFi.localIP().toString();
  } else{
    Serial.println(F("Can not connect to WiFi station. Go into AP mode."));
    // Go into software AP mode.
    WiFi.mode(WIFI_AP);
    delay(10);
    //WiFi.setHostname(host_name.c_str());
    Serial.print(F("Setting soft-AP configuration ... "));
    Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ?
      F("Ready") : F("Failed!"));
    Serial.print(F("Setting soft-AP ... "));
    Serial.println(WiFi.softAP(host_name.c_str(), ap_default_psk) ?
      F("Ready") : F("Failed!"));    
    status.myIP = WiFi.softAPIP().toString();
  }
  Serial.print(F("IP address: "));
  Serial.println(status.myIP);
  WifiConnectOk = true;
  Web_setup();
}


void setupOTAProgramming(){
    ArduinoOTA
    .onStart([]() {
      String type;
      Serial.println(F("stopping standard-task"));
      vTaskDelete(xHandleStandard); //delete standard-task
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";
        SPIFFS.end();

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      setting.wifiDownTime = 0; //don't switch off Wifi during upload
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      //Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();  
}

//Initialize OLED display
void startOLED(){
  //reset OLED display via software
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);

  //initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(0,0);
  display.print(APPNAME);
  display.setCursor(0,16);
  display.print(VERSION);
  display.display();
  delay(500);
}

void printSettings(){
  Serial.println("**** SETTINGS ****");
  Serial.print("WIFI SSID=");Serial.println(setting.ssid);
  Serial.print("WIFI PW=");Serial.println(setting.password);
  Serial.print("Aircraft=");Serial.println(fanet.getAircraftType(setting.AircraftType));
  Serial.print("Pilotname=");Serial.println(setting.PilotName);
  Serial.print("Switch WIFI OFF after 3 min=");Serial.println(setting.bSwitchWifiOff3Min);
  Serial.print("Wifi-down-time=");Serial.println(setting.wifiDownTime/1000.);
  Serial.print("Output-Mode=");Serial.println(setting.outputMode);
  Serial.print("UDP_SERVER=");Serial.println(setting.UDPServerIP);
  Serial.print("UDP_PORT=");Serial.println(setting.UDPSendPort);
  Serial.print("TESTMODE=");Serial.println(setting.testMode);
}

void listSpiffsFiles(){
  File root = SPIFFS.open("/");
  File file = root.openNextFile();
  while(file){  
    Serial.print("FILE: ");
    Serial.println(file.name());

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
    case ESP_SLEEP_WAKEUP_UNDEFINED : Serial.println("wakeup undefined --> possible by reset"); break;
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup EXT0"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup EXIT1"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup TIMER"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup TOUCHPAD"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup ULP"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
  return wakeup_reason;
}

void setup() {
  
  
  // put your setup code here, to run once:  
  Serial.begin(115200);

  esp_sleep_wakeup_cause_t reason = print_wakeup_reason(); //print reason for wakeup
  
  i2cOLED.begin(OLED_SDA, OLED_SCL);

  setupAXP192();

    // Make sure we can read the file system
  if( !SPIFFS.begin(true)){
    Serial.println("Error mounting SPIFFS");
    while(1);
  }

  //listSpiffsFiles();

  startOLED();

  load_configFile(); //load configuration

  


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

  xTaskCreatePinnedToCore(taskStandard, "taskStandard", 6500, NULL, 10, &xHandleStandard, ARDUINO_RUNNING_CORE1); //standard task
  xTaskCreatePinnedToCore(taskBackGround, "taskBackGround", 6500, NULL, 1, &xHandleBackground, ARDUINO_RUNNING_CORE1); //background task

}

void loop() {
  // put your main code here, to run repeatedly:
  //delay(1000); //wait 1second
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

void printBattVoltage(uint32_t tAct){
  static uint32_t tBatt = millis();
  if ((tAct - tBatt) >= 5000){
    tBatt = tAct;
    if (axp.isBatteryConnect()) {
      status.vBatt = axp.getBattVoltage()/1000.;
    }else{
      Serial.println("no Batt");
      status.vBatt = 0.0;
    }
  }
}



void printGPSData(uint32_t tAct){
  static uint32_t tPrint = millis();
  String s = "";
  if ((tAct - tPrint) >= 1000){
    display.fillRect(0,0,128,64,BLACK);
    display.setTextSize(2);
    display.setCursor(0,0);
    display.print(setStringSize(String(status.vBatt,1) + "V",4));

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

    tPrint = tAct;
  }
}

void readGPS(){
  static char lineBuffer[255];
  static uint16_t recBufferIndex = 0;
  
  while(NMeaSerial.available()){
    if (recBufferIndex >= 255) recBufferIndex = 0; //Buffer overrun
    //Serial.write(NMeaSerial.read());
    //int c = NMeaSerial.read();
    lineBuffer[recBufferIndex] = NMeaSerial.read();
    nmea.process(lineBuffer[recBufferIndex]);
    //Serial.write(lineBuffer[recBufferIndex]);
    if (lineBuffer[recBufferIndex] == '\n'){
      lineBuffer[recBufferIndex] = '\r';
      recBufferIndex++;
      lineBuffer[recBufferIndex] = '\n';
      recBufferIndex++;
      lineBuffer[recBufferIndex] = 0; //zero-termination
      String s = lineBuffer;
      sendData2Client(s);
      recBufferIndex = 0;
    }else{
      if (lineBuffer[recBufferIndex] != '\r'){
        recBufferIndex++;
      }
    }  
  }
  //Serial.println("GPS ready");
}

eFlarmAircraftType Fanet2FlarmAircraft(eFanetAircraftType aircraft){
  switch (aircraft)
  {
  case eFanetAircraftType::UNKNOWN:
    return eFlarmAircraftType::UNKNOWN;
  case eFanetAircraftType::PARA_GLIDER:
    return eFlarmAircraftType::PARA_GLIDER;
  case eFanetAircraftType::HANG_GLIDER:
    return eFlarmAircraftType::HANG_GLIDER;
  case eFanetAircraftType::BALLOON:
    return eFlarmAircraftType::BALLOON;
  case eFanetAircraftType::GLIDER:
    return eFlarmAircraftType::GLIDER_MOTOR_GLIDER;
  case eFanetAircraftType::POWERED_AIRCRAFT:
    return eFlarmAircraftType::TOW_PLANE;
  case eFanetAircraftType::HELICOPTER_ROTORCRAFT:
    return eFlarmAircraftType::HELICOPTER_ROTORCRAFT;
  case eFanetAircraftType::UAV:
    return eFlarmAircraftType::UAV;
  }
  return eFlarmAircraftType::UNKNOWN;
}

void Fanet2FlarmData(trackingData *FanetData,FlarmtrackingData *FlarmDataData){
  FlarmDataData->aircraftType = Fanet2FlarmAircraft(FanetData->aircraftType);
  FlarmDataData->altitude = FanetData->altitude;
  FlarmDataData->climb = FanetData->climb;
  FlarmDataData->DevId = FanetData->DevId;
  FlarmDataData->heading = FanetData->heading;
  FlarmDataData->lat = FanetData->lat;
  FlarmDataData->lon = FanetData->lon;
  FlarmDataData->speed = FanetData->speed;
}

void sendLK8EX(uint32_t tAct){
  static uint32_t tOld = millis();
  
  if ((tAct - tOld) >= 250){
    //String s = "$LK8EX1,101300,99999,99999,99,999,";
    String s = "$LK8EX1,";

    s += "999999,"; //raw pressure in hPascal: hPA*100 (example for 1013.25 becomes  101325) 
    s += String((uint32_t)(status.GPS_alt,2)) + ","; // altitude in meters, relative to QNH 1013.25
    s += String((int32_t)(status.ClimbRate * 100.0)) + ","; //climbrate in cm/s
    //s += String(status.) + ",";
    s += "99,"; //temperature
    s += String(status.vBatt,2) + ",";
    s = flarm.addChecksum(s);
    sendData2Client(s);
    tOld = tAct;
  }
}



void taskStandard(void *pvParameters){
  static uint32_t tLife = millis();
  static uint8_t counter = 0;
  static uint32_t tPilotName = millis();
  static uint32_t tFlarmState = millis();
  static float oldAlt = 0.0;
  static uint32_t tOldPPS = millis();
  trackingData tFanetData;  
  trackingData myFanetData;  
  FlarmtrackingData myFlarmData;
  FlarmtrackingData PilotFlarmData;


  NMeaSerial.begin(GPSBAUDRATE,SERIAL_8N1,GPSRX,GPSTX,false);
  // Change the echoing messages to the ones recognized by the MicroNMEA library
  MicroNMEA::sendSentence(NMeaSerial, "$PSTMSETPAR,1201,0x00000042");
  MicroNMEA::sendSentence(NMeaSerial, "$PSTMSAVEPAR");

  //Reset the device so that the changes could take plaace
  MicroNMEA::sendSentence(NMeaSerial, "$PSTMSRR");

  delay(1000);
  //clear serial buffer
  while (NMeaSerial.available())
    NMeaSerial.read();  
  pinMode(PPSPIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPSPIN), ppsHandler, FALLING);

  // create a binary semaphore for task synchronization
  fanet.begin(SCK, MISO, MOSI, SS,RST, DIO0);
  fanet.setPilotname(setting.PilotName);
  fanet.setAircraftType(setting.AircraftType);
  setting.myDevId = fanet.getMyDevId();
  host_name += setting.myDevId; //String((ESP32_getChipId() & 0xFFFFFF), HEX);
  flarm.begin();

  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(0,40);
  display.print("ID:");
  display.print(setting.myDevId);
  display.display();
  delay(1000);

  //udp.begin(UDPPORT);

  while(1){    
    // put your main code here, to run repeatedly:
    uint32_t tAct = millis();
    printBattVoltage(tAct);
    readGPS();
    printGPSData(tAct);
    if (setting.testMode == 0){
      writeTrackingData(tAct);
      if ((tAct - tPilotName) >= 24000){
          tPilotName = tAct;
          fanet.sendPilotName();
      }
      if ((tAct - tFlarmState) >= 5000){
          tFlarmState = tAct;
          sendData2Client(flarm.writeDataPort());
      }
    }
    if (sendTestData == 1){
      fanet.writeTrackingData2FANET(&testTrackingData);
      sendTestData = 0;
    }else if (sendTestData == 2){
      fanet.writeMsgType2(testString);
      sendTestData = 0;
    }else if (sendTestData == 3){
      fanet.writeMsgType3(testString);
      sendTestData = 0;
    }else if (sendTestData == 4){
      fanet.writeMsgType4(&testWeatherData);
      sendTestData = 0;
    }
    fanet.run();
    if (fanet.isNewMsg()){
      //write msg to udp !!
      String msg = fanet.getactMsg() + "\n";
      sendData2Client(msg);
    }
    if (fanet.getTrackingData(&tFanetData)){
      if (nmea.isValid()){
        fanet.getMyTrackingData(&myFanetData);
        Fanet2FlarmData(&myFanetData,&myFlarmData);
        Fanet2FlarmData(&tFanetData,&PilotFlarmData);
        sendData2Client(flarm.writeFlarmData(&myFlarmData,&PilotFlarmData));        
      }
    }
    flarm.run();
    sendLK8EX(tAct);
    if (ppsTriggered){
      ppsTriggered = false;
      //Serial.println("PPS-Triggered");
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
        status.ClimbRate = (status.GPS_alt - oldAlt) / (float(tAct - tOldPPS) / 1000.0);
        oldAlt = status.GPS_alt;
        MyFanetData.climb = status.ClimbRate;
        MyFanetData.lat = status.GPS_Lat;
        MyFanetData.lon = status.GPS_Lon;
        MyFanetData.altitude = status.GPS_alt;
        MyFanetData.speed = status.GPS_speed; //speed in cm/s --> we need km/h
        MyFanetData.heading = status.GPS_course;
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
      //Serial.print(F("Standard-task running "));
      //Serial.println(counter);
      counter++;
    }    
    delay(1);
  }
}

void powerOff(){
  axp.clearIRQ();
  Serial.println(F("stopping standard-task"));
  vTaskDelete(xHandleStandard); //delete standard-task
  delay(100);
  fanet.end();


  esp_wifi_set_mode(WIFI_MODE_NULL);
  esp_wifi_stop();
  //esp_bt_controller_disable();


  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(4);
  display.setCursor(20,20);
  display.print("OFF");
  display.display();  


  delay(2000);

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
  setupWifi();
  setupOTAProgramming();
  while (1){
    uint32_t tAct = millis();
    if  (WifiConnectOk){
      Web_loop();
      #ifdef OTAPROGRAMMING
      ArduinoOTA.handle();
      #endif
    }
    if (( tAct > setting.wifiDownTime) && (setting.wifiDownTime!=0)){
      setting.wifiDownTime=0;
      WifiConnectOk=false;
      esp_wifi_set_mode(WIFI_MODE_NULL);
      esp_wifi_stop();
      Serial.println("******************WEBCONFIG Setting - WIFI STOPPING************************* ");
    }
    if ((tAct - tLife) >= LifeCount){
      tLife = tAct;
      //Serial.print(F("Background running wifi="));
      //Serial.print(WifiConnectOk);
      //Serial.print(F(" "));
      //Serial.println(counter);
      counter++;

    }
    //yield();
    if (AXP192_Irq){
      if (axp.readIRQ() == AXP_PASS) {
        if (axp.isPEKLongtPressIRQ()) {
          Serial.println(F("Long Press IRQ"));
          Serial.flush();
          powerOff();
        }
        if (axp.isPEKShortPressIRQ()) {
          Serial.println(F("Short Press IRQ"));
          Serial.flush();
        }
        axp.clearIRQ();
      }
      AXP192_Irq = false;
    }

    delay(1);
	}
}
