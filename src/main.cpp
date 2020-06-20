#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <WiFiClient.h>
#include <ArduinoOTA.h>
#include <SPI.h>
#include <LoRa.h>
#include <FanetLora.h>
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
String host_name = HOSTNAME;

//WebServer server(80);

FanetLora fanet;
//NmeaOut nmeaout;
//Flarm flarm;
HardwareSerial NMeaSerial(2);
//MicroNMEA library structures
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

trackingData MyFanetData;  
uint8_t u8Sat;
uint8_t u8Fix;

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
volatile float vBatt = 0.0;
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
    Serial.print(F("IP address: "));
    Serial.println(WiFi.localIP());
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
    Serial.print(F("IP address: "));
    Serial.println(WiFi.softAPIP());
  }
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
  display.setCursor(0,25);
  display.print("ESP FANET");
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
  Serial.print("UDP_SERVER=");Serial.println(setting.UDPServerIP);
  Serial.print("UDP_PORT=");Serial.println(setting.UDPSendPort);
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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  i2cOLED.begin(OLED_SDA, OLED_SCL);

  setupAXP192();

    // Make sure we can read the file system
  if( !SPIFFS.begin(true)){
    Serial.println("Error mounting SPIFFS");
    while(1);
  }

  listSpiffsFiles();

  startOLED();

  load_configFile(); //load configuration

  


  setting.ssid = "WLAN_EICHLER";
  setting.password = "magest172";
  setting.bSwitchWifiOff3Min = false;
  setting.wifiDownTime = 0;

  printSettings();

  xTaskCreatePinnedToCore(taskStandard, "taskStandard", 6500, NULL, 10, &xHandleStandard, ARDUINO_RUNNING_CORE1); //standard task
  xTaskCreatePinnedToCore(taskBackGround, "taskBackGround", 6500, NULL, 1, &xHandleBackground, ARDUINO_RUNNING_CORE1); //background task

}

void loop() {
  // put your main code here, to run repeatedly:
  //delay(1000); //wait 1second
}

void taskStandard(void *pvParameters){
  static uint32_t tLife = millis();
  static uint8_t counter = 0;
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
  //udp.begin(UDPPORT);

  while(1){    
    // put your main code here, to run repeatedly:
    uint32_t tAct = millis();
    if ((tAct - tLife) >= LifeCount){
      tLife = tAct;
      Serial.print(F("Standard-task running "));
      Serial.println(counter);
      counter++;
    }
    delay(1);
  }
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
      Serial.print(F("Background running wifi="));
      Serial.print(WifiConnectOk);
      Serial.print(F(" "));
      Serial.println(counter);
      counter++;

    }
    //yield();
    delay(1);
	}
}
