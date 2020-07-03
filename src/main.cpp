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
#include <ble.h>

//Libraries for OLED Display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BluetoothSerial.h>

//define programming OTA
#define OTAPROGRAMMING

#define LifeCount 5000

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

trackingData MyFanetData;  


trackingData testTrackingData;
weatherData testWeatherData;
String testString;
uint8_t sendTestData = 0;

uint8_t WifiConnectOk = 0;
IPAddress local_IP(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);
const char* ap_default_psk = "12345678"; ///< Default PSK.

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

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &i2cOLED, OLED_RST);

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
void WiFiEvent(WiFiEvent_t event);
void listConnectedStations();
float readBattvoltage();
void sendAWUdp(String msg);
void sendAWUdp(String msg){
  if (WiFi.status() == WL_CONNECTED){
    //log_e("%s",msg.c_str());
    WiFiUDP udp;
    udp.beginPacket(airwhere_web_ip.c_str(),AIRWHERE_UDP_PORT);
    udp.write((uint8_t *)msg.c_str(),msg.length());
    udp.endPacket();
  }      
}

void sendData2Client(String data){
  if (setting.outputMode == OUTPUT_UDP){
    //output via udp
    if (WifiConnectOk == 2){ //connected to wifi or a client is connected to me
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
    status.BoardVersion = 11;
  }else{
    log_e("AXP192 not found");
    status.BoardVersion = 07;
  }
}

void IRAM_ATTR ppsHandler(void){
  ppsTriggered = true;
}

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
  
  WiFi.onEvent(WiFiEvent);
  log_i("hostname=%s",host_name.c_str());
  WiFi.setHostname(host_name.c_str());
  delay(10);
  if ((setting.ssid.length() > 0) && (setting.password.length() > 0)){
    WiFi.mode(WIFI_STA);
    if ((WiFi.SSID() != setting.ssid || WiFi.psk() != setting.password)){
      // ... Try to connect to WiFi station.
      WiFi.begin(setting.ssid.c_str(), setting.password.c_str());
    } else {
      // ... Begin with sdk config.
      WiFi.begin();
    }
    log_i("Wait for WiFi connection.");
    uint32_t wifiTimeout = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - wifiTimeout < 10000) {
      delay(500);
      log_i(".");
    }
    
  }
  if(WiFi.status() == WL_CONNECTED){
    // ... print IP Address
    status.myIP = WiFi.localIP().toString();
    WifiConnectOk = 2;
  } else{
    log_i("Can not connect to WiFi station. Go into AP mode.");
    // Go into software AP mode.
    WiFi.mode(WIFI_AP);
    delay(10);
    //WiFi.setHostname(host_name.c_str());
    log_i("Setting soft-AP configuration ... ");
    if(WiFi.softAPConfig(local_IP, gateway, subnet)){
      log_i("Ready");
    }else{
      log_i("Failed!");
    }
    log_i("Setting soft-AP ... ");
    if (WiFi.softAP(host_name.c_str(), ap_default_psk)){
      log_i("Ready");
    }else{
      log_i("Failed!");
    }
    status.myIP = WiFi.softAPIP().toString();
    WifiConnectOk = 1;
  }
  log_i("IP address: %s",status.myIP.c_str());
  Web_setup();
}


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

//Initialize OLED display
void startOLED(){
  //reset OLED display via software
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);

  //initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x32
    log_e("SSD1306 allocation failed");
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
  log_i("**** SETTINGS ****");
  log_i("Board-Version=V%s",String((float)status.BoardVersion/10,1));
  if (setting.band == 0){
    log_i("BAND=868mhz");
  }else{
    log_i("BAND=915mhz");
  }
  log_i("BAND=%d",setting.band);
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

  esp_sleep_wakeup_cause_t reason = print_wakeup_reason(); //print reason for wakeup
  
  i2cOLED.begin(OLED_SDA, OLED_SCL);

  setupAXP192();

  if (status.BoardVersion == 07){
    analogReadResolution(ADC_BITS); // Default of 12 is not very linear. Recommended to use 10 or 11 depending on needed resolution.
    //analogSetAttenuation(ADC_11db); // Default is 11db which is very noisy. Recommended to use 2.5 or 6. Options ADC_0db (1.1V), ADC_2_5db (1.5V), ADC_6db (2.2V), ADC_11db (3.9V but max VDD=3.3V)
    pinMode(ADCBOARDVOLTAGE_PIN, INPUT);
  }


    // Make sure we can read the file system
  if( !SPIFFS.begin(true)){
    log_e("Error mounting SPIFFS");
    while(1);
  }

  //listSpiffsFiles();

  startOLED();

  load_configFile(); //load configuration
  btOk = 0;


  

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
  xTaskCreatePinnedToCore(taskBackGround, "taskBackGround", 6500, NULL, 5, &xHandleBackground, ARDUINO_RUNNING_CORE1); //background task
  if (setting.outputMode == OUTPUT_BLE){
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    xTaskCreatePinnedToCore(taskBle, "taskBle", 4096, NULL, 7, &xHandleBle, ARDUINO_RUNNING_CORE1);
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
    if (status.BoardVersion == 11){
      if (axp.isBatteryConnect()) {
        status.vBatt = axp.getBattVoltage()/1000.;
      }else{
        log_w("no Batt");
        status.vBatt = 0.0;
      }
    }else{
      status.vBatt = readBattvoltage();
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
    s += String(status.GPS_alt,2) + ","; // altitude in meters, relative to QNH 1013.25
    s += String((int32_t)(status.ClimbRate * 100.0)) + ","; //climbrate in cm/s
    //s += String(status.) + ",";
    s += "99,"; //temperature
    s += String(status.vBatt,2) + ",";
    s = flarm.addChecksum(s);
    if (setting.outputLK8EX1) sendData2Client(s);
    tOld = tAct;
  }
}



void taskStandard(void *pvParameters){
  static uint32_t tLife = millis();
  static uint32_t tLoop = millis();
  static uint8_t counter = 0;
  static uint32_t tPilotName = millis();
  static uint32_t tFlarmState = millis();
  static float oldAlt = 0.0;
  static uint32_t tOldPPS = millis();
  trackingData tFanetData;  
  trackingData myFanetData;  
  FlarmtrackingData myFlarmData;
  FlarmtrackingData PilotFlarmData;


  if (status.BoardVersion == 07){
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
  if (status.BoardVersion == 11){  
    //only on new boards we have an pps-pin
    pinMode(PPSPIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(PPSPIN), ppsHandler, FALLING);
  }

  // create a binary semaphore for task synchronization
  long frequency = FREQUENCY868;
  if (setting.band == BAND915)frequency = FREQUENCY915; 
  fanet.begin(SCK, MISO, MOSI, SS,RST, DIO0,frequency);
  fanet.setPilotname(setting.PilotName);
  fanet.setAircraftType(setting.AircraftType);
  setting.myDevId = fanet.getMyDevId();
  host_name += setting.myDevId; //String((ESP32_getChipId() & 0xFFFFFF), HEX);
  flarm.begin();

  if (setting.outputMode == OUTPUT_BLUETOOTH){
    log_i("starting bluetooth_serial %s",host_name.c_str());
    SerialBT.begin(host_name); //Bluetooth device name
    SerialBT.print(APPNAME " Bluetooth Starting");
    btOk = 1;
  }





  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(0,40);
  display.print("ID:");
  display.print(setting.myDevId);
  display.display();
  delay(1000);

  //udp.begin(UDPPORT);
  tLoop = millis();
  while(1){    
    // put your main code here, to run repeatedly:
    uint32_t tAct = millis();
    status.tLoop = tAct - tLoop;
    tLoop = tAct;
    if (status.tMaxLoop < status.tLoop) status.tMaxLoop = status.tLoop;
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
          if (setting.outputFLARM) sendData2Client(flarm.writeDataPort());
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
    status.fanetRx = fanet.rxCount;
    status.fanetTx = fanet.txCount;
    if (fanet.isNewMsg()){
      //write msg to udp !!
      String msg = fanet.getactMsg() + "\n";
      if (setting.outputFANET) sendData2Client(msg);
    }
    if (fanet.getTrackingData(&tFanetData)){
      if (nmea.isValid()){
        fanet.getMyTrackingData(&myFanetData);
        //Serial.printf("LAT=%.6f\n",tFanetData.lat);
        //Serial.printf("LON=%.6f\n",tFanetData.lon);
        if (setting.awLiveTracking){
          char chs[20];
          String msg = nmea.getFixTime() + ","
                    + setting.myDevId + ","
                    + tFanetData.DevId + ","
                    + String((uint8_t)tFanetData.aircraftType) + ",";
          sprintf(chs,"%02.6f",tFanetData.lat);
          msg += String(chs) + ",";
          sprintf(chs,"%02.6f",tFanetData.lon);
          msg += String(chs) + ",0,0,";
          sprintf(chs,"%0.2f",tFanetData.heading);
          msg += String(chs) + ",";
          sprintf(chs,"%0.2f",tFanetData.speed * 0.53996);
          msg += String(chs) + ",";
          sprintf(chs,"%0.2f",tFanetData.altitude);
          //msg += String(chs) + ",-50";
          msg += String(chs) + "," + String(tFanetData.rssi);
          //log_e("%s",msg.c_str());
          sendAWUdp(msg);
        }
        Fanet2FlarmData(&myFanetData,&myFlarmData);
        Fanet2FlarmData(&tFanetData,&PilotFlarmData);
        if (setting.outputFLARM) sendData2Client(flarm.writeFlarmData(&myFlarmData,&PilotFlarmData));        
      }
    }
    flarm.run();
    sendLK8EX(tAct);
    if (status.BoardVersion == 07){
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
        MyFanetData.heading = status.GPS_course;
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
    delay(1);
  }
}

void powerOff(){
  axp.clearIRQ();
  log_i("stopping standard-task");
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
        log_w( "*****LOOP current free heap: %d, minimum ever free heap: %d ******", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
        warning_time=millis()+1000;
      }
    }
	if (xPortGetMinimumEverFreeHeapSize()<1000)
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
    if ((status.vBatt < 3.3) && (status.vBatt > 1.0)) {
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
