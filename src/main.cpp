#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>
//#include <esp_wifi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <SPI.h>
//#include <LoRa.h>
#include <FanetLora.h>
#include <FlarmDataPort.h>
#include <main.h>
#include <config.h>
#include "WebHelper.h"
#include "fileOps.h"
#include <SPIFFS.h>
#ifdef BLUETOOTH
#include <ble.h>
#include "esp_bt_main.h"
#endif
#include <Ogn.h>
//#include "SparkFun_Ublox_Arduino_Library.h"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <TimeLib.h>
#include <sys/time.h>
//#include <HTTPClient.h>
#include <ArduinoHttpClient.h>

#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/adc.h"
//#include "Update.h"
#include "gxUpdater.h"
#include "gxUserLed.h"
#include <AceButton.h>
#include <ArduinoJson.h>
#include "../lib/GxMqtt/GxMqtt.h"

#include "XPowersLib.h"
#include "TimeDefs.h"

//#include <ws90.h>


//#define SEND_RAW_WIND_DATA

XPowersLibInterface *PMU = NULL;


//#define GXTEST
//#define FLARMTEST

#define TINY_GSM_RX_BUFFER 1024

#define WDT_TIMEOUT 15

//#define SSLCONNECTION

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
  TinyGsmClient GsmUpdaterClient(modem,0); //client number 2 for updater
  #ifdef SSLCONNECTION
  TinyGsmClientSecure  GsmWUClient(modem,1); //client number 1 for weather-underground
  #else
  TinyGsmClient  GsmWUClient(modem,1); //client number 1 for weather-underground
  #endif
  TinyGsmClient GsmOGNClient(modem,2); //client number 0 for OGN
  TinyGsmClient GsmMqttClient(modem,3); //client number 3 for MQTT
#endif

#ifdef EINK
#include <Screen.h>
#endif
#if defined(SSD1306) || defined(SH1106G)
#include <oled.h>
#endif

//#define FLARMLOGGER
#ifdef FLARMLOGGER
#include "FS.h"
#include <SD.h>
#include <SPI.h>
//#include <Logger.h>
//#include "driver/rtc_io.h"
#endif


#ifdef GSMODULE

#include <Dusk2Dawn.h> //for sunset and sunrise-functions
#include <Weather.h>
#include <WeatherUnderground.h>
#include <Windy.h>

#include <DFRobot_SD3031.h>
#include <RTClib.h>

RTC_DS3231 *pRtc3231 = NULL;
DFRobot_SD3031 *pRtc3031 = NULL;

#endif




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

#include <Wire.h>
TwoWire *pI2cZero = &Wire;
TwoWire *pI2cOne = &Wire1;
SemaphoreHandle_t xI2C1Mutex;
SemaphoreHandle_t xI2C0Mutex;
SemaphoreHandle_t *PMUMutex = NULL;

#include <MicroNMEA.h>
#ifdef AIRMODULE
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


#endif
uint8_t wifiCMD = 0;

bool WebUpdateRunning = false;
bool bPowerOff = false;
bool bGsmOff= false;

struct SettingsData setting;
struct statusData status;
struct commandData command;
String host_name = "";
String bleMsg = "";

String compile_date = String(BUILD_DATE_YEAR_CH0) + String(BUILD_DATE_YEAR_CH1) + String(BUILD_DATE_YEAR_CH2) + String(BUILD_DATE_YEAR_CH3) + "-" + String(BUILD_DATE_MONTH_CH0) + String(BUILD_DATE_MONTH_CH1) + "-" + String(BUILD_DATE_DAY_CH0) + String(BUILD_DATE_DAY_CH1) + "T" + String(BUILD_TIME_HOUR_CH0) + String(BUILD_TIME_HOUR_CH1) + ":" + String(BUILD_TIME_MIN_CH0) + String(BUILD_TIME_MIN_CH1) + ":" + String(BUILD_TIME_SEC_CH0) + String(BUILD_TIME_SEC_CH1);

FanetLora fanet;
//NmeaOut nmeaout;
FlarmDataPort flarmDataPort;
#ifdef AIRMODULE
HardwareSerial NMeaSerial(2);
#endif
//MicroNMEA library structures

uint16_t battFull = 4050;
uint16_t battEmpty = 3300;

Ogn ogn;

gxUpdater updater;  
gxUserLed userled;

FanetLora::trackingData MyFanetData;  


FanetLora::trackingData fanetTrackingData;
FanetLora::weatherData fanetWeatherData;
bool sendWeatherData = false;
String fanetString;
uint32_t fanetReceiver;
uint8_t sendFanetData = 0;

IPAddress local_IP(192,168,4,1);
//IPAddress gateway(192,168,4,1);
IPAddress gateway(0,0,0,0);
//IPAddress gateway(255,255,255,255); //clear gateway
//IPAddress gateway = INADDR_NONE;
IPAddress subnet(255,255,255,0);

volatile bool ppsTriggered = false;
volatile uint32_t gtPPS = 0;

volatile bool PMU_Irq = false;
volatile float BattCurrent = 0.0;

//bool newStationConnected = false;

#define AIRWHERE_UDP_PORT 5555
const char* airwhere_web_ip = "37.128.187.9";

#ifdef BLUETOOTH
SemaphoreHandle_t ble_queue = nullptr;
SemaphoreHandle_t RxBleQueue = nullptr;
#endif



uint32_t psRamSize = 0;

uint32_t fanetDstId = 0;


//PIN-Definition
int8_t PinPMU_Irq   =  35;

//e-ink
int8_t PinEink_Busy   =  33;
int8_t PinEink_Rst    =  0;
int8_t PinEink_Dc     =  32;
int8_t PinEink_Cs     =  15;
int8_t PinEink_Clk    =  4;
int8_t PinEink_Din    =  2;


//LED
int8_t PinUserLed = -1;
int8_t PinBeaconLed = -1;

//Power
int8_t PinExtPower = -1;

//ADC-Voltage
int8_t PinADCCtrl = -1;
int8_t PinADCVoltage = -1;

//LORA-Module
int8_t PinLoraRst = -1;
int8_t PinLoraDI0 = -1;
int8_t PinLoraGPIO = -1;
int8_t PinLora_SS = -1;
int8_t PinLora_MISO = -1;
int8_t PinLora_MOSI = -1;
int8_t PinLora_SCK = -1;

//GSM-Module
int8_t PinGsmPower = -1;
int8_t PinGsmRst = -1;
int8_t PinGsmTx = -1;
int8_t PinGsmRx = -1;

//GPS
int8_t PinGPSRX = -1;
int8_t PinGPSTX = -1;
int8_t PinPPS = -1;

//OLED-Display / PMU
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

//anemometer
int8_t PinWindDir = -1;
int8_t PinWindSpeed = -1;
int8_t PinRainGauge = -1;

//external Power on/off
int8_t PinExtPowerOnOff = -1;

//fuel-sensor
int8_t PinFuelSensor = -1;

//buttons
struct myButtons {
  int8_t PinButton = -1;
  uint8_t state = 0;
};

ace_button::AceButton buttons[NUMBUTTONS];
myButtons sButton[NUMBUTTONS];
//int8_t PinButton[NUMBUTTONS] = {-1,-1,};
//uint8_t buttonState[NUMBUTTONS] = {0,0,};

float adcVoltageMultiplier = 0.0;
String sNmeaIn = "";

char* pWd = NULL;
uint8_t wdCount = 0;

#ifdef SEND_RAW_WIND_DATA
char* pWdRaw = NULL;
uint8_t wdRawCount = 0;
#endif


TaskHandle_t xHandleBaro = NULL;
TaskHandle_t xHandleStandard = NULL;
TaskHandle_t xHandleBackground = NULL;
TaskHandle_t xHandleBluetooth = NULL;
TaskHandle_t xHandleMemory = NULL;
TaskHandle_t xHandleEInk = NULL;
TaskHandle_t xHandleOled = NULL;
TaskHandle_t xHandleLogger = NULL;
TaskHandle_t xHandleWeather = NULL;

SemaphoreHandle_t xOutputMutex;
String sOutputData = "";
char sMqttState[MAXSTRING];

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
#if defined(SSD1306) || defined(SH1106G)
void taskOled(void *pvParameters);
#endif
#ifdef FLARMLOGGER
SemaphoreHandle_t FlarmLogQueue = nullptr;
void taskLogger(void *pvParameters);
#endif
#ifdef GSM_MODULE
void taskGsm(void *pvParameters);
void PowerOffModem();
//void readSMS();
//void sendStatus();
#endif
#if defined(TINY_GSM_MODEM_SIM7000) || defined(TINY_GSM_MODEM_SIM7080)
void setupSim7000Gps();
#endif
void add2OutputString(String s);
bool printBattVoltage(uint32_t tAct);

void checkBoardType();
void checkLoraChip();
// Forward reference to prevent Arduino compiler becoming confused.
void handleEvent(ace_button::AceButton*, uint8_t, uint8_t);
void readFuelSensor(uint32_t tAct);
void setupPMU();
void taskStandard(void *pvParameters);
void taskBackGround(void *pvParameters);
#ifdef BLUETOOTH
void taskBluetooth(void *pvParameters);
void stopBluetooth(void);
#endif
void taskMemory(void *pvParameters);
void setupWifi();
void IRAM_ATTR ppsHandler(void);
void printSettings();
void listSpiffsFiles();
String setStringSize(String s,uint8_t sLen);
void sendData2Client(char *buffer,int iLen);
eFlarmAircraftType Fanet2FlarmAircraft(FanetLora::aircraft_t aircraft);
void Fanet2FlarmData(FanetLora::trackingData *FanetData,FlarmtrackingData *FlarmDataData);
void sendLK8EX(uint32_t tAct);
void sendLXPW(uint32_t tAct);
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
//void handleButton(uint32_t tAct);
char* readSerial();
void checkReceivedLine(const char *ch_str);
void checkSystemCmd(const char *ch_str);
size_t getNextString(char *ch_str,char *pSearch,char *buffer, size_t sizeBuffer);
void setWifi(bool on);
void handleUpdate(uint32_t tAct);
void printChipInfo(void);
void setAllTime(tm &timeinfo);
void checkExtPowerOff(uint32_t tAct);
void writePGXCFSentence();
double round2(double value);
#ifdef GSMODULE
void sendFanetWeatherData2WU(FanetLora::weatherData *weatherData,uint8_t wuIndex);
void sendFanetWeatherData2WI(FanetLora::weatherData *weatherData,uint8_t wiIndex);
void setRTCTime(tm &timeinfo);
void getRTC();
void getRTCTime();
#endif
#ifdef AIRMODULE
bool setupUbloxConfig(void);
bool setupQuectelGps(void);
bool checkGPSBaudrates(void);
bool checkGPSBaud(uint32_t baud);
#endif

constexpr uint32_t commonBaudRates[] = {9600, 19200, 38400, 57600, 115200};
constexpr uint8_t  commonBaudRatesSize = sizeof(commonBaudRates) / sizeof(commonBaudRates[0]);

/* customGPS Setup: set NMEA protocol version and numbering for u-blox 6 & 7 */
uint8_t setNMEA_U6_7[] PROGMEM = {0x00, 0x23, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00,  /* NMEA protocol v2.3 extended */
                                  0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00};

 /* https://content.arduino.cc/assets/Arduino-MKR-GPS-Shield_u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221_Public.pdf 198 */
uint8_t setNMEA_U8_9_10[] PROGMEM = {0x00, 0x40, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00,  /* NMEA protocol v4.00 extended for Galileo */
                                     0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
                                     0x00, 0x00, 0x00, 0x00};

/* UBX-CFG-MSG (NMEA Standard Messages) */
uint8_t setCFG[3][8] PROGMEM = {
	{0xF1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // Ublox - Lat/Long Position Data
	{0xF1, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // Ublox - Satellite Status
	{0xF1, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // Ublox - Time of Day and Clock Information
};


void radiotest(){
  LoRaClass radio;
  SPI.begin(5, 19, 27, 18);
  radio.setPins(&SPI,18,26,23);
  radio.begin();
  radio.switchFSK(869200000uL);
  radio.startReceive();	
  //radio.FSKRx(869200000uL);
  static uint8_t rxCount = 0;
  while(1){
    // put your main code here, to run repeatedly:
    //radio.run();
    if (radio.isRxMessage()){
      int16_t packetSize = radio.getPacketLength();
      rxCount ++;
      log_i("new message arrived %d len=%d",rxCount,packetSize);
      if (packetSize == 26){
        uint8_t rx_frame[26];	
        radio.readData(&rx_frame[0], packetSize);
        char Buffer[500];	
        int len = 0;
        for (int i = 0; i < 26; i++){
          len += sprintf(Buffer+len,"%02X", rx_frame[i]);
        }
        len += sprintf(Buffer+len,"\n");
        Serial.print(Buffer);
        //log_i("%d received:%s",rxCount,Buffer[0]);
      }
      //radio.FSKRx(869200000uL);
      radio.startReceive();	
    }
    delay(10);
  }
}

// rounds a number to 2 decimal places
// example: round(3.14159) -> 3.14
double round2(double value) {
   return (int)(value * 100 + 0.5) / 100.0;
}

void i2cScanner(){
  byte error, address;
  int nDevices;

  log_i("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    pI2cOne->beginTransmission(address);
    error = pI2cOne->endTransmission();

    if (error == 0)
    {
      log_i("I2C device found at address 0x%02X !",address);
      nDevices++;
    }
    else if (error==4)
    {
      log_i("Unknown error at address 0x%02X !",address);
    }
  }
  if (nDevices == 0)
    log_i("No I2C devices found");
  else
    log_i("done");
}

#ifdef GSMODULE
void sendFanetWeatherData2WU(FanetLora::weatherData *weatherData,uint8_t wuIndex){
  if ((status.bInternetConnected) && (status.bTimeOk)){
    WeatherUnderground wu;
    WeatherUnderground::wData wuData;
    #ifdef GSM_MODULE
      if (setting.wifi.connect == eWifiMode::CONNECT_NONE){
        wu.setClient(&GsmWUClient);
        wu.setMutex(&xGsmMutex);
      }
    #endif    
    wuData.bWind = weatherData->bWind;
    wuData.winddir = weatherData->wHeading;
    wuData.windspeed = weatherData->wSpeed;
    wuData.windgust = weatherData->wGust;
    wuData.bHum = weatherData->bHumidity;
    wuData.humidity = weatherData->Humidity;
    wuData.bTemp = weatherData->bTemp;
    wuData.temp = weatherData->temp;
    wuData.bPress = weatherData->bBaro;
    wuData.pressure = weatherData->Baro;
    wu.sendData(setting.FntWuUpload[wuIndex].ID ,setting.FntWuUpload[wuIndex].KEY,&wuData);
  }
}
void sendFanetWeatherData2WI(FanetLora::weatherData *weatherData,uint8_t wiIndex){
  if ((status.bInternetConnected) && (status.bTimeOk)){
    Windy wi;
    Windy::wData wiData;
    #ifdef GSM_MODULE
      if (setting.wifi.connect == eWifiMode::CONNECT_NONE){
        wi.setClient(&GsmWUClient);
        wi.setMutex(&xGsmMutex);
      }
    #endif
    wiData.bWind = weatherData->bWind;
    wiData.winddir = weatherData->wHeading;
    wiData.windspeed = weatherData->wSpeed;
    wiData.windgust = weatherData->wGust;
    wiData.bHum = weatherData->bHumidity;
    wiData.humidity = weatherData->Humidity;
    wiData.bTemp = weatherData->bTemp;
    wiData.temp = weatherData->temp;
    wiData.bPress = weatherData->bBaro;
    wiData.pressure = weatherData->Baro;
    wi.sendData(setting.FntWiUpload[wiIndex].ID,setting.FntWiUpload[wiIndex].KEY,&wiData);
  }
}
#endif

void readFuelSensor(uint32_t tAct){
  static uint32_t tRead = millis();
  static uint32_t tSend = millis();
  static bool bFirst = false;
  if (PinFuelSensor < 0) return;
  if (timeOver(tAct,tRead,100)){ //every 100ms
    tRead = tAct;
    // multisample ADC
    const byte NO_OF_SAMPLES = 4;
    uint32_t adc_reading = 0;
    analogRead(PinFuelSensor); // First measurement has the biggest difference on my board, this line just skips the first measurement
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
      uint16_t thisReading = analogRead(PinFuelSensor);
      adc_reading += thisReading;
    }
    adc_reading /= NO_OF_SAMPLES;
    float fFuel = 3.3 * float(adc_reading) / 1023.0f;
    if (!bFirst){
      status.fuelSensor = fFuel;
      bFirst = true;
    }
    status.fuelSensor = calcExpAvgf(status.fuelSensor,fFuel,128);
  }  
  if (timeOver(tAct,tSend,FUELSENDINTERVALL)){
    tSend = tAct;
    char sOut[MAXSTRING];
    int pos = 0;
    pos += snprintf(&sOut[pos],MAXSTRING-pos,"$FUEL,%.03f,",status.fuelSensor);
    pos = flarmDataPort.addChecksum(sOut,MAXSTRING);
    sendData2Client(sOut,pos);
  }
  
}


// The event handler for the button.
void handleEvent(ace_button::AceButton* button, uint8_t eventType, uint8_t buttonState) {

  // Print out a message for all events.
  /*
  Serial.print(F("handleEvent(): eventType: "));
  Serial.print(eventType);
  Serial.print(F("; buttonState: "));
  Serial.println(buttonState);
  */

  // Get the LED pin
  uint8_t id = button->getId();

  // Control the LED only for the Pressed and Released events.
  // Notice that if the MCU is rebooted while the button is pressed down, no
  // event is triggered and the LED remains off.
  switch (eventType) {
    case ace_button::AceButton::kEventClicked:
      sButton[id].state = eventType;
      log_i("button %d clicked",id);
      break;
    case ace_button::AceButton::kEventLongPressed:
      sButton[id].state = eventType;
      //log_i("button %d long pressed",id);
      break;
    case ace_button::AceButton::kEventDoubleClicked:
      sButton[id].state = eventType;
      log_i("button %d double clicked",id);
      break;
  }
}

void checkLoraChip(){
  //we check the Lora-Chip, if it is a SX1276
  PinLoraRst = 23;
  PinLoraDI0 = 26;
  PinLora_SS = 18;
  PinLora_MISO = 19;
  PinLora_MOSI = 27;
  PinLora_SCK = 5;

  SPI.begin(PinLora_SCK, PinLora_MISO, PinLora_MOSI, PinLora_SS);
  pinMode(PinLoraRst,OUTPUT);
  pinMode(PinLora_SS,OUTPUT);
  digitalWrite(PinLoraRst,LOW);
  delay(5);
  digitalWrite(PinLoraRst,HIGH);
  delay(5);
  SPISettings _spiSettings = SPISettings(8000000, MSBFIRST, SPI_MODE0);
  digitalWrite(PinLora_SS,LOW);
  SPI.beginTransaction(_spiSettings);  
  SPI.transfer(0x42); //read reg 0x42
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(PinLora_SS,HIGH);
  SPI.endTransaction();
  //log_i("Lora-Chip-Version=%02X",v);
  if (v == 0x12){
    log_i("Lora-Chip SX1276 found --> Board is a T-Beam with SX1276");    
  }else{
    setting.boardType = eBoard::T_BEAM_SX1262;
    log_i("Lora-Chip SX1262 found --> Board is a T-Beam with SX1262");
  }
}

uint8_t checkI2C(){
  uint8_t nDevices = 0;
  for (int i = 1;i < 127; i++){
    pI2cOne->beginTransmission(i);
    uint8_t err = pI2cOne->endTransmission();
    if (err == 0) {
      nDevices++;
      //log_i("I2C Device found at %02X",i);
    }
  }
  log_i("I2C found %d Devices",nDevices);
  return nDevices;
}

void checkBoardType(){
  #ifndef AIRMODULE //we are a ground-station --> set default-values
  setting.PilotName = "GS" + fanet.getMyDevId();
  log_i("set station-name %s",setting.PilotName.c_str());
  setting.wd.sendFanet = 1;
  setting.OGNLiveTracking.bits.fwdName = true;
  setting.OGNLiveTracking.bits.fwdWeather = true;
  setting.OGNLiveTracking.bits.liveTracking = true;
  setting.OGNLiveTracking.bits.sendWeather = true;
  setting.outputMode = eOutput::oSERIAL;
  setting.bOutputSerial = false;
  setting.outputGPS = false;
  setting.outputFLARM = false;
  setting.outputFANET = false;
  setting.outputModeVario = eOutputVario::OVARIO_NONE;
  #endif
  #ifdef TINY_GSM_MODEM_SIM7000
    setting.displayType = NO_DISPLAY;
    setting.boardType = eBoard::TTGO_TSIM_7000;
    log_i("TTGO-T-Sim7000 found");
    write_configFile(&setting);
    delay(1000);
    esp_restart(); //we need to restart
  #endif
  #ifdef TINY_GSM_MODEM_SIM800
    setting.displayType = NO_DISPLAY;
    setting.boardType = eBoard::TTGO_TCALL_800;
    log_i("TTGO-T-Sim800 found");
    write_configFile(&setting);
    delay(1000);
    esp_restart(); //we need to restart
  #endif
  log_i("start checking board-type");  
  PinOledSDA = 21;
  PinOledSCL = 22;
  pI2cOne->begin(PinOledSDA, PinOledSCL);
  uint8_t i2cDevices = checkI2C();
  if (i2cDevices < 10){
    pI2cOne->beginTransmission(AXP192_SLAVE_ADDRESS);
    if (pI2cOne->endTransmission() == 0) {
      //ok we have a T-Beam !! 
      //check, if we have an OLED
      setting.boardType = eBoard::T_BEAM;
      log_i("AXP192 has been found");
      checkLoraChip();
      setting.displayType = NO_DISPLAY;
      pI2cOne->beginTransmission(OLED_SLAVE_ADDRESS);
      if (pI2cOne->endTransmission() == 0) {
        //we have found also the OLED
        setting.displayType = OLED0_96;
        log_i("OLED found");
      }
      log_i("Board is T-Beam V1.0 or V1.1");
      write_configFile(&setting);
      delay(1000);
      esp_restart(); //we need to restart
    }
    //no AXP192 --> maybe T-Beam V07 or T3 V1.6
    pI2cOne->beginTransmission(OLED_SLAVE_ADDRESS);
    if (pI2cOne->endTransmission() == 0) {
      //we have found the OLED
      setting.boardType = eBoard::T_BEAM_V07;
      setting.displayType = OLED0_96;
      log_i("OLED found");
      log_i("Board is T-Beam v0.7");
      write_configFile(&setting);
      delay(1000);
      esp_restart(); //we need to restart
    }
  }
  setting.displayType = NO_DISPLAY;
  PinOledSDA = 4;
  PinOledSCL = 15;
  PinOledRst = 16;
  pI2cOne->begin(PinOledSDA, PinOledSCL);
  i2cDevices = checkI2C();
  pinMode(PinOledRst, OUTPUT);
  digitalWrite(PinOledRst, LOW);
  delay(100);
  digitalWrite(PinOledRst, HIGH);
  delay(100);
  pI2cOne->beginTransmission(OLED_SLAVE_ADDRESS);
  if (pI2cOne->endTransmission() == 0) {
    //we have found also the OLED
    setting.displayType = OLED0_96;
    log_i("OLED found");
    setting.boardType = eBoard::HELTEC_LORA;
    log_i("Board is Heltec/Lora");
    write_configFile(&setting);
    delay(1000);
    esp_restart(); //we need to restart
  }
  log_i("Board is Heltec Wireless Stick Lite");
  setting.boardType = eBoard::HELTEC_WIRELESS_STICK_LITE;
  write_configFile(&setting);
  delay(1000);
  esp_restart(); //we need to restart

}

void add2OutputString(String s){
  xSemaphoreTake( xOutputMutex, portMAX_DELAY );
  sOutputData += s;
  xSemaphoreGive(xOutputMutex);
}

void setAllTime(tm &timeinfo){
  tmElements_t tm;          // a cache of time elements
  tm.Year = uint8_t(timeinfo.tm_year + 1900 - 1970);
  tm.Month = timeinfo.tm_mon+1;
  tm.Day = timeinfo.tm_mday;
  tm.Hour = timeinfo.tm_hour;
  tm.Minute = timeinfo.tm_min;
  tm.Second = timeinfo.tm_sec;
  //log_i("%04d%02d%02d-%02d:%02d:%02d",timeinfo.tm_year,timeinfo.tm_mon,timeinfo.tm_mday,timeinfo.tm_hour,timeinfo.tm_min,timeinfo.tm_sec);
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
	uint64_t chipmacid = ESP.getEfuseMac();
  log_i("ESP32ChipID=%04X%08X",(uint16_t)(chipmacid>>32),(uint32_t)chipmacid);//print chip-id  
	uint8_t myDevId[3];
	myDevId[0] = ManuId;//Manufacturer GetroniX
	myDevId[1] = uint8_t(chipmacid >> 32); //last 2 Bytes of MAC
	myDevId[2] = uint8_t(chipmacid >> 40);
  log_i("dev_id=%02X%02X%02X",myDevId[0],myDevId[1],myDevId[2]);          
}


bool printLocalTime()
{
  struct tm now;
  getLocalTime(&now,0);
  if (now.tm_year >= 117){
    //log_i("%04d %02d %02d %02d:%02d:%02d",now.tm_year+1900,now.tm_mon+1,now.tm_mday,now.tm_hour,now.tm_min,now.tm_sec);
    return true;
  }else{
    return false;
  }
}


void sendFlarmData(uint32_t tAct){
  if (WebUpdateRunning) return;
  static uint32_t tSend = millis();
  static uint32_t tSendStatus = millis();
  FlarmtrackingData myFlarmData;
  FlarmtrackingData PilotFlarmData;
  FanetLora::trackingData tFanetData;  
  uint8_t countNeighbours = 0;

  if (!setting.outputFLARM) return;

  if (timeOver(tAct,tSendStatus,FLARM_UPDATE_STATE)){
    tSendStatus = tAct;
    char sOut[MAXSTRING];
    int pos = 0;
    pos = flarmDataPort.writeVersion(sOut,MAXSTRING);
    sendData2Client(sOut,pos);
    pos = flarmDataPort.writeSelfTestResult(sOut,MAXSTRING);
    sendData2Client(sOut,pos);
  }

  if (timeOver(tAct,tSend,FLARM_UPDATE_RATE)){
    tSend = tAct;
    if (status.gps.Fix){
      Fanet2FlarmData(&fanet._myData,&myFlarmData);
      for (int i = 0; i < MAXNEIGHBOURS; i++){
        //if ((fanet.neighbours[i].devId) && (fanet.neighbours[i].type == 0x11)){ //we have a ID an we are flying !!
        if (fanet.neighbours[i].devId){ //we have a ID an we are flying !!
          tFanetData.aircraftType = fanet.neighbours[i].aircraftType;
          tFanetData.altitude = fanet.neighbours[i].altitude;
          tFanetData.climb = fanet.neighbours[i].climb;
          tFanetData.devId = fanet.neighbours[i].devId;
          tFanetData.addressType = fanet.neighbours[i].addressType;
          tFanetData.heading = fanet.neighbours[i].heading;
          tFanetData.lat = fanet.neighbours[i].lat;
          tFanetData.lon = fanet.neighbours[i].lon;
          tFanetData.speed = fanet.neighbours[i].speed;
          tFanetData.OnlineTracking = fanet.neighbours[i].OnlineTracking;
          Fanet2FlarmData(&tFanetData,&PilotFlarmData);
          char sOut[MAXSTRING];
          int pos = flarmDataPort.writeFlarmData(sOut,MAXSTRING,&myFlarmData,&PilotFlarmData);
          sendData2Client(sOut,pos);
          countNeighbours++;    
        }
      }

      if (status.flying){
        flarmDataPort.GPSState = FLARM_GPS_FIX3d_AIR;
      }else{
        flarmDataPort.GPSState = FLARM_GPS_FIX3d_GROUND;
      }      
    }else{
      flarmDataPort.GPSState = FLARM_NO_GPS;
    }
    flarmDataPort.neighbors = countNeighbours;
    
    char sDataPort[MAXSTRING];
    int iLen = flarmDataPort.writeDataPort(&sDataPort[0],sizeof(sDataPort));
    sendData2Client(&sDataPort[0],iLen);
  }
}

void checkFlyingState(uint32_t tAct){
  static uint32_t tOk = millis();
  static uint32_t tFlightTime = millis();
  if (status.flying){
    //flying
    if ((!status.gps.Fix) || (status.gps.speed < MIN_FLIGHT_SPEED)){
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
    if ((status.gps.Fix) && (status.gps.NumSat >= 4) && (status.gps.speed > MIN_FLIGHT_SPEED)){ //minimum 4 satelites to get accurade position and speed
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
    if (now.tm_year < 117){
      log_e("time not set");
      return -3; //time not set yet    
    } 
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
        //log_i("day=1;%02d:%02d:%02d sunrise=%s sunset=%s iAct=%s",now.tm_hour,now.tm_min,now.tm_sec,time1,time2,time3);
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
    if (setting.Mode == eMode::GROUND_STATION){
      msg = "000000,";
    }
  #endif
  #ifdef AIRMODULE
  if (setting.Mode == eMode::AIR_MODULE){
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
    udp.beginPacket(airwhere_web_ip,AIRWHERE_UDP_PORT);
    udp.write((uint8_t *)msg.c_str(),msg.length());
    udp.endPacket();
  }else{
    //log_e("can't send AW-Data WIFI not connected state=%d",WiFi.status());
  }      
}

void sendData2Client(char *buffer,int iLen){
  //log_i("endChar=%02X;len=%d",buffer[iLen],iLen);
  //size_t bufLen = strlen(buffer);
  if (setting.outputMode == eOutput::oUDP){
    //output via udp
    if ((WiFi.status() == WL_CONNECTED) || (WiFi.softAPgetStationNum() > 0)){ //connected to wifi or a client is connected to me
      //log_i("sending udp");
      
      WiFiUDP udp;
      udp.beginPacket(setting.UDPServerIP.c_str(),setting.UDPSendPort);
      udp.write((uint8_t *)buffer,iLen);
      udp.endPacket();    
    }
  }
  if ((setting.outputMode == eOutput::oSERIAL) || (setting.bOutputSerial)){//output over serial-connection
    //Serial.print(buffer); 
    //log_i("serial write %s",buffer);
    Serial.write(buffer,iLen);
  }
  #ifdef BLUETOOTH
  if (setting.outputMode == eOutput::oBLE){ //output over ble-connection
    if (xHandleBluetooth && ble_queue){
      if (iLen < MAXSIZEBLE){
        ble_data data;
        std::copy_n(buffer,iLen,std::begin(data.data));
        data.size=iLen;
        BaseType_t status = xQueueSendToBack(ble_queue,&data,pdMS_TO_TICKS(100));
        if (status != pdTRUE){
          log_w("ble_queue full -> reset queue");
          xQueueReset(ble_queue);
        }
      }
      else{
        log_w("ble msg too long");
      }
    }
  }
  #endif
}


static void IRAM_ATTR PMU_Interrupt_handler() {
  PMU_Irq = true;
  //log_i("PMU IRQ");
}

void setupPMU(){
  status.PMU = ePMU::NOPMU;
  if (!PMU){
    if (setting.boardType == eBoard::T_BEAM_S3CORE){
      PMUMutex = &xI2C0Mutex;
      PMU = new XPowersAXP2101(*pI2cZero,42,41,AXP2101_SLAVE_ADDRESS);
    }else{
      PMUMutex = &xI2C1Mutex;
      PMU = new XPowersAXP2101(*pI2cOne,PinOledSDA,PinOledSCL,AXP2101_SLAVE_ADDRESS);
    }

    if (!PMU->init()) {
        //Serial.println("Warning: Failed to find AXP2101 power management");
        delete PMU;
        PMU = NULL;
    } else {
        log_i("AXP2101 PMU init succeeded, using AXP2101 PMU");
        status.PMU = ePMU::AXP2101;
    }
  }
  if (!PMU){
    PMUMutex = &xI2C1Mutex;
    PMU = new XPowersAXP192(*pI2cOne,PinOledSDA,PinOledSCL,AXP192_SLAVE_ADDRESS);
    if (!PMU->init()) {
        //Serial.println("Warning: Failed to find AXP192 power management");
        delete PMU;
        PMU = NULL;
    } else {
        log_i("AXP192 PMU init succeeded, using AXP192 PMU");
        status.PMU = ePMU::AXP192;
    }
  }
  if (!PMU) {
      status.PMU = ePMU::NOPMU;
      PMUMutex = NULL;
      log_e("failed to found any PMU (AXP192, AXP2101)");
      return;
  }

  if (status.PMU == ePMU::AXP2101){
    //t-beam m.2 inface
    //gps
    PMU->setPowerChannelVoltage(XPOWERS_ALDO4, 3300);
    PMU->enablePowerOutput(XPOWERS_ALDO4);

    // lora
    PMU->setPowerChannelVoltage(XPOWERS_ALDO3, 3300);
    PMU->enablePowerOutput(XPOWERS_ALDO3);

    // In order to avoid bus occupation, during initialization, the SD card and QMC sensor are powered off and restarted
    if (ESP_SLEEP_WAKEUP_UNDEFINED == esp_sleep_get_wakeup_cause()) {
        Serial.println("Power off and restart ALDO BLDO..");
        PMU->disablePowerOutput(XPOWERS_ALDO1);
        PMU->disablePowerOutput(XPOWERS_ALDO2);
        PMU->disablePowerOutput(XPOWERS_BLDO1);
        delay(250);
    }

    // Sensor
    PMU->setPowerChannelVoltage(XPOWERS_ALDO1, 3300);
    PMU->enablePowerOutput(XPOWERS_ALDO1);

    PMU->setPowerChannelVoltage(XPOWERS_ALDO2, 3300);
    PMU->enablePowerOutput(XPOWERS_ALDO2);

    //Sdcard

    PMU->setPowerChannelVoltage(XPOWERS_BLDO1, 3300);
    PMU->enablePowerOutput(XPOWERS_BLDO1);

    PMU->setPowerChannelVoltage(XPOWERS_BLDO2, 3300);
    PMU->enablePowerOutput(XPOWERS_BLDO2);

    //face m.2
    PMU->setPowerChannelVoltage(XPOWERS_DCDC3, 3300);
    PMU->enablePowerOutput(XPOWERS_DCDC3);

    PMU->setPowerChannelVoltage(XPOWERS_DCDC4, XPOWERS_AXP2101_DCDC4_VOL2_MAX);
    PMU->enablePowerOutput(XPOWERS_DCDC4);

    PMU->setPowerChannelVoltage(XPOWERS_DCDC5, 3300);
    PMU->enablePowerOutput(XPOWERS_DCDC5);


    //not use channel
    PMU->disablePowerOutput(XPOWERS_DCDC2);
    // PMU->disablePowerOutput(XPOWERS_DCDC4);
    // PMU->disablePowerOutput(XPOWERS_DCDC5);
    PMU->disablePowerOutput(XPOWERS_DLDO1);
    PMU->disablePowerOutput(XPOWERS_DLDO2);
    PMU->disablePowerOutput(XPOWERS_VBACKUP);

    // Set constant current charge current limit
    PMU->setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_500MA);

    // Set charge cut-off voltage
    PMU->setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2);

    pinMode(PinPMU_Irq, INPUT_PULLUP);
    attachInterrupt(PinPMU_Irq,PMU_Interrupt_handler, FALLING);

    // Disable all interrupts
    PMU->disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
    // Clear all interrupt flags
    PMU->clearIrqStatus();
    // Enable the required interrupt function
    PMU->enableIRQ(
        //XPOWERS_AXP2101_BAT_INSERT_IRQ    | XPOWERS_AXP2101_BAT_REMOVE_IRQ      |   //BATTERY
        //XPOWERS_AXP2101_VBUS_INSERT_IRQ   | XPOWERS_AXP2101_VBUS_REMOVE_IRQ     |   //VBUS
        XPOWERS_AXP2101_PKEY_SHORT_IRQ    | XPOWERS_AXP2101_PKEY_LONG_IRQ       //|   //POWER KEY
        //XPOWERS_AXP2101_BAT_CHG_DONE_IRQ  | XPOWERS_AXP2101_BAT_CHG_START_IRQ       //CHARGE
        // XPOWERS_AXP2101_PKEY_NEGATIVE_IRQ | XPOWERS_AXP2101_PKEY_POSITIVE_IRQ   |   //POWER KEY
    );


    PMU->enableSystemVoltageMeasure();
    PMU->enableVbusVoltageMeasure();
    PMU->enableBattVoltageMeasure();
    // It is necessary to disable the detection function of the TS pin on the board
    // without the battery temperature detection function, otherwise it will cause abnormal charging
    PMU->disableTSPinMeasure();

    // Set the time of pressing the button to turn off
    PMU->setPowerKeyPressOffTime(XPOWERS_POWEROFF_8S); //to switch really off without off-screen.

  }else if (status.PMU == ePMU::AXP192){
    PMU->setChargingLedMode(XPOWERS_CHG_LED_CTRL_CHG);
    PMU->disablePowerOutput(XPOWERS_LDO3);// GPS main power
    delay(500);

    PMU->setPowerChannelVoltage(XPOWERS_LDO2, 3300);// LoRa, AXP192 power-on value: 3300
    PMU->enablePowerOutput(XPOWERS_LDO2);

    PMU->setPowerChannelVoltage(XPOWERS_LDO3, 3300);// GPS,  AXP192 power-on value: 3000
    PMU->enablePowerOutput(XPOWERS_LDO3);

    PMU->setPowerChannelVoltage(XPOWERS_DCDC1, 3300);//       AXP192 power-on value: 3300
    //protected oled power source
    //PMU->setProtectedChannel(XPOWERS_DCDC1);
    // enable oled power
    PMU->enablePowerOutput(XPOWERS_DCDC1);

    //protected esp32 power source
    PMU->setProtectedChannel(XPOWERS_DCDC3);

    //disable not use channel
    PMU->disablePowerOutput(XPOWERS_DCDC2);

    pinMode(PinPMU_Irq, INPUT_PULLUP);
    attachInterrupt(PinPMU_Irq,PMU_Interrupt_handler, FALLING);

    // Disable all interrupts
    PMU->disableIRQ(XPOWERS_AXP192_ALL_IRQ);
    // Clear all interrupt flags
    PMU->clearIrqStatus();
    // Enable the required interrupt function
    PMU->enableIRQ(
        //XPOWERS_AXP2101_BAT_INSERT_IRQ    | XPOWERS_AXP2101_BAT_REMOVE_IRQ      |   //BATTERY
        //XPOWERS_AXP2101_VBUS_INSERT_IRQ   | XPOWERS_AXP2101_VBUS_REMOVE_IRQ     |   //VBUS
        XPOWERS_AXP192_PKEY_SHORT_IRQ    | XPOWERS_AXP192_PKEY_LONG_IRQ       //|   //POWER KEY
        //XPOWERS_AXP2101_BAT_CHG_DONE_IRQ  | XPOWERS_AXP2101_BAT_CHG_START_IRQ       //CHARGE
        // XPOWERS_AXP2101_PKEY_NEGATIVE_IRQ | XPOWERS_AXP2101_PKEY_POSITIVE_IRQ   |   //POWER KEY
    );
  }
}

void IRAM_ATTR ppsHandler(void){
  gtPPS = millis();
  ppsTriggered = true;
}

void WiFiEvent(WiFiEvent_t event){
  switch(event){
    case ARDUINO_EVENT_WIFI_READY:
      log_i("wifi ready");
      break;
    case ARDUINO_EVENT_WIFI_STA_START:
      log_i("station start");
      status.wifiSTA.state = STARTED;
      break;
    case ARDUINO_EVENT_WIFI_STA_STOP:
      log_i("station stop");
      status.wifiSTA.state = DISCONNECTED;
      status.wifiSTA.ip = "";
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      log_i("station connected to AP");
      status.wifiSTA.state = CONNECTED;
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      log_i("station lost connection to AP");   
      status.wifiSTA.state = DISCONNECTED;
      status.wifiSTA.ip = "";
      break; 
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      status.wifiSTA.state = FULL_CONNECTED;
      status.wifiSTA.ip = WiFi.localIP().toString();
      log_i("station got IP from connected AP IP:%s",status.wifiSTA.ip.c_str());
      break;
    case ARDUINO_EVENT_WIFI_STA_LOST_IP:
      status.wifiSTA.ip = "";            
      log_i("STA LOST IP");
      break;
    case ARDUINO_EVENT_WIFI_AP_START:
      status.wifiAP.state = STARTED;
      status.wifiAP.ip = local_IP.toString();
      log_i("soft-AP start");
      break;
    case ARDUINO_EVENT_WIFI_AP_STOP:
      status.wifiAP.state = IDLE;
      status.wifiAP.ip = "";
      log_i("soft-AP stop");
      break;
    case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
      log_i("a station connected to ESP32 soft-AP");
      break;
    case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
      log_i("a station disconnected from ESP32 soft-AP");
      break;
    case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED:
      log_i("soft-AP assign an IP to a connected station");
      break;
    default:
      log_e("Unhandled WiFi Event: %d", event );
      break;
  }
}

void setupSoftAp(){
  log_i("Set soft-AP settings");
  WiFi.softAP(host_name.c_str(), setting.wifi.appw.c_str());
  WiFi.softAPConfig(local_IP, gateway, subnet);
}

void setupWifi(){
  while(host_name.length() == 0){
    delay(100); //wait until we have the devid
  }
  //delay(2000); //wait 2 seconds until power is stable
  log_i("setup Wifi");
  status.bInternetConnected = false;
  status.wifiSTA.state = IDLE;
  status.bWifiOn = false;
  WiFi.disconnect(true,true);
  WiFi.mode(WIFI_MODE_NULL);  
  WiFi.persistent(false);
  WiFi.onEvent(WiFiEvent);
  //WiFi.config(IPADDR_ANY, IPADDR_ANY, IPADDR_ANY,IPADDR_ANY,IPADDR_ANY); // call is only a workaround for bug in WiFi class
  WiFi.setHostname(host_name.c_str());
  log_i("setup Wifi ready !");
}

void printSettings(){
  log_i("**** SETTINGS " VERSION " build:%s ******",&compile_date[0]);
  log_i("Access-point password=%s",setting.wifi.appw.c_str());
  log_i("Board-Type=%d",setting.boardType);
  log_i("Display-Type=%d",setting.displayType);
  log_i("Display-Rotation=%d",setting.displayRotation);
  log_i("Mode=%d",setting.Mode);
  log_i("Fanet-Mode=%d",setting.fanetMode);
  log_i("Fanet-Pin=%d",setting.fanetpin);
  log_i("external power switch=%d",setting.bHasExtPowerSw);


  log_i("Serial-output=%d",setting.bOutputSerial);
  log_i("OUTPUT Vario=%d",setting.outputModeVario);
  log_i("OUTPUT FLARM=%d",setting.outputFLARM);
  log_i("OUTPUT GPS=%d",setting.outputGPS);
  log_i("OUTPUT FANET=%d",setting.outputFANET);
  log_i("Device ID=%s",setting.myDevId);
  log_i("Device Address Type=%d",setting.myDevIdType);

  log_i("WIFI connect=%d",setting.wifi.connect);
  log_i("WIFI SSID=%s",setting.wifi.ssid.c_str());
  log_i("WIFI PW=%s",setting.wifi.password.c_str());
  log_i("Aircraft=%s",fanet.getAircraftType((FanetLora::aircraft_t)setting.AircraftType).c_str());
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

  log_i("BattVoltOffs=%.2f",setting.BattVoltOffs);
  log_i("minBattPercent=%d",setting.minBattPercent);
  log_i("restartBattPercent=%d",setting.restartBattPercent);
  
  log_i("AirWhere-Livetracking=%d",setting.awLiveTracking);
  log_i("OGN-Livetracking=%d",setting.OGNLiveTracking.mode);
  log_i("Traccar-Livetracking=%d",setting.traccarLiveTracking);
  log_i("Traccar-Address=%s",setting.TraccarSrv.c_str());
  log_i("RF-Mode=%d",setting.RFMode);
  
  //vario
  log_i("VarioSinkingThreshold=%0.2f",setting.vario.sinkingThreshold);
  log_i("VarioClimbingThreshold=%0.2f",setting.vario.climbingThreshold);
  log_i("VarioNearClimbingSensitivity=%0.2f",setting.vario.nearClimbingSensitivity);
  log_i("VarioVolume=%d",setting.vario.volume);
  log_i("Vario use MPU=%d",setting.vario.useMPU);
  log_i("Vario temp offset=%.02f",setting.vario.tempOffset);
  log_i("Vario sigmaP=%.02f",setting.vario.sigmaP);
  log_i("Vario sigmaA=%.02f",setting.vario.sigmaA);

  //weather-data
  //general
  log_i("WD mode=%d",setting.wd.mode.mode);
  log_i("WD tempoffset=%.1f [°]",setting.wd.tempOffset);
  log_i("WD windDirOffset=%d [°]",setting.wd.windDirOffset);
  // FANET
  log_i("WD FANET-Weatherdata=%d",setting.wd.sendFanet);
  log_i("WD FANET-Interval=%d [msec]",setting.wd.FanetUploadInterval);
  // Weather Underground
  log_i("WUUlEnable=%d",setting.WUUpload.enable);
  log_i("WUUlInterval=%d [msec]",setting.wd.WUUploadIntervall);
  log_i("WUUlID=%s",setting.WUUpload.ID.c_str());
  log_i("WUUlKEY=%s",setting.WUUpload.KEY.c_str());
  // Windy
  log_i("WIUlEnable=%d",setting.WindyUpload.enable);
  log_i("WIUlID=%s",setting.WindyUpload.ID.c_str());
  log_i("WIUlKEY=%s",setting.WindyUpload.KEY.c_str());

  // mqttt
  log_i("MqttMode=%d",setting.mqtt.mode.mode );
  log_i("MqttServer=%s",setting.mqtt.server.c_str());
  log_i("MqttPort=%d",setting.mqtt.port);
  log_i("MqttPw=%s",setting.mqtt.pw.c_str());

  #ifdef GSM_MODULE
  //GSM
  log_i("GSM APN=%s",setting.gsm.apn.c_str());
  log_i("GSM USER=%s",setting.gsm.user.c_str());
  log_i("GSM PWD=%s",setting.gsm.pwd.c_str());
  #endif

  log_i("fuel-sensor=%d",setting.bHasFuelSensor);

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

#ifdef TEST
void testLegacy(){
  log_i("test legacy");
  //uint8_t rx_frame[26] = {0x7A,0x32,0x4B,0x10,0x82,0xE5,0x59,0x4E,0x30,0x69,0x70,0xCB,0x4F,0xFC,0x5C,0x83,0xD1,0xB0,0xB7,0xF0,0x82,0x38,0xD4,0xC0,0xCD,0x34};
  uint8_t rx_frame[26] = {0x21,0x30,0x4B,0x10,0x99,0x63,0xA9,0x3B,0xAA,0x3A,0xAE,0x2A,0x12,0xEC,0xE9,0xC5,0xE4,0x6E,0x54,0xA7,0x2C,0x0E,0x70,0xD3,0x81,0x7E};
  uint16_t crc16 =  getLegacyCkSum(rx_frame,24);
  uint16_t crc16_2 = (uint16_t(rx_frame[24]) << 8) + uint16_t(rx_frame[25]);
  if (crc16 != crc16_2){
    log_e("Legacy: wrong Checksum %04X!=%04X",crc16,crc16_2);    
    //return;
  }
  ufo_t air={0};
  ufo_t myAircraft={0};
  myAircraft.latitude = setting.gs.lat;
  myAircraft.longitude = setting.gs.lon;
  myAircraft.geoid_separation = setting.gs.geoidAlt;
  myAircraft.timestamp = 1617554883;
  uint8_t newPacket[26];
  uint32_t tNow = myAircraft.timestamp;	
  uint32_t tOffset = 0;	
  bool bOk = false;
  char Buffer[500];
	int len = 0;
  for(int i = 0;i < 5; i++){
    memcpy(&newPacket[0],&rx_frame[0],26);
    decrypt_legacy(newPacket,tNow + tOffset);
    int8_t ret = legacy_decode(newPacket,&myAircraft,&air);
    if (ret == 0){
      float dist = distance(myAircraft.latitude,myAircraft.longitude,air.latitude,air.longitude, 'K');      
      //if ((dist <= 100.0) && (air.addr != 0) && (air.aircraft_type != 0)){
      if ((air.addr != 0) && (air.aircraft_type != 0)){
        len = sprintf(Buffer,"adr=%06X;adrType=%d;airType=%d,lat/lon=%.06f,%.06f,alt=%.01f,speed=%.01f,course=%.01f,climb=%.01f,dist=%.01f\n", air.addr,air.addr_type,air.aircraft_type,air.latitude,air.longitude,air.altitude,air.speed,air.course,air.vs,dist);
        log_i("%s",Buffer);

        bOk = true;
        break;
      }
    }else if (ret == -2){
      //unknown message
      break;
    }
    log_i("Legacy-Packet not valid ts=%d;offset=%d",tNow,tOffset);
    if (i == 0){
      tOffset = -1;
    }else if (i == 1){
      tOffset = 1;
    }else if (i == 2){
      tOffset = -2;
    }else if (i == 3){
      tOffset = 2;
    }
  }
}
#endif

/**
 * @brief Write a $PGXAC sentence to buffer, this can be used by Stratux (or other device) to validate
 * (part) of it's configuration.
 *
 */
void writePGXCFSentence() {
  char buffer[MAXSTRING];
  // $PGXCF,<version>,<Output Serial>,<eMode>,<eOutputVario>,<output Fanet>,<output GPS>,<output FLARM>,<customGPSConfig>,<Aircraft Type>,<Address>,<Pilot Name>
  int8_t version = 1;
  snprintf(buffer, MAXSTRING, "$PGXCF,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%s,%s",
    version,
    setting.outputMode,
    setting.Mode, setting.outputModeVario,
    setting.outputFANET, setting.outputGPS, setting.outputFLARM,
    setting.gps.customGPSConfig, setting.AircraftType,
    setting.myDevIdType, setting.myDevId.c_str(), setting.PilotName.c_str());
  size_t size = flarmDataPort.addChecksum(buffer, MAXSTRING);
  sendData2Client(buffer, size);
}

/**
 * @brief Parse a $PGXCF sentence and set the configuration accordingly
 * This is mainly used for automation of GxAirCom configuration
 * @param data incomming NMEA sentence
*/
void readPGXCFSentence(const char* data)
{
  char result[MAXSTRING];
  // Parse $PGXCF,<version>,<Output Serial>,<eMode>,<eOutputVario>,<output Fanet>,<output GPS>,<output FLARM>,<customGPSConfig>,<Aircraft Type (hex)>,<Address Type>,<Address (hex)>, <Pilot Name>
  // $PGXCF,1,0,0,1,0,1,1,1,5,1,F23456,Pilot Name*76 // enable customGPS config mode with ICAO address type
  // $PGXCF,1,3,0,1,0,1,1,0,5,2,,GXAirCom*4A  // Enable default GXAirCom with FLARM address type default hardware Id

  // NMEA type (PGXCF)
  data = MicroNMEA::parseField(data, &result[0], MAXSTRING); if (data == NULL) return;

  // Version
  data = MicroNMEA::parseField(data, &result[0], MAXSTRING); if (data == NULL) return;

  // only version 1 is supported
  if (strtol(result, NULL, 10) != 1) return;

  // Output mode
  data = MicroNMEA::parseField(data, &result[0], MAXSTRING); if (data == NULL) return;
  eOutput outputMode = (eOutput)strtol(result, NULL, 10);

  // GXAircomMode
  data = MicroNMEA::parseField(data, &result[0], MAXSTRING); if (data == NULL) return;
  eMode gxMode = (eMode)strtol(result, NULL, 10);

  // eOutputVario
  data = MicroNMEA::parseField(data, &result[0], MAXSTRING); if (data == NULL) return;
  eOutputVario outputModeVario = (eOutputVario)strtol(result, NULL, 10);

  // output Fanet
  data = MicroNMEA::parseField(data, &result[0], MAXSTRING); if (data == NULL) return;
  bool outputFANET = result[0] != '0';

  // output GPS
  data = MicroNMEA::parseField(data, &result[0], MAXSTRING); if (data == NULL) return;
  bool outputGPS = result[0] != '0';

  // output FLARM
  data = MicroNMEA::parseField(data, &result[0], MAXSTRING); if (data == NULL) return;
  bool outputFLARM = result[0] != '0';

  // customGPSConfig
  data = MicroNMEA::parseField(data, &result[0], MAXSTRING); if (data == NULL) return;
  bool customGPSConfig = result[0] != '0';

  // Aircraft type
  data = MicroNMEA::parseField(data, &result[0], MAXSTRING); if (data == NULL) return;
  uint8_t aircraftType = strtol(result, NULL, 16);

  // Address Type 1=ADDRESSTYPE_FLARM or 2=ADDRESSTYPE_ICAO
  data = MicroNMEA::parseField(data, &result[0], MAXSTRING); if (data == NULL) return;
  uint8_t myDevIdType = atoi(result) == 0x01?ADDRESSTYPE_ICAO:ADDRESSTYPE_FLARM;

  // Address
  data = MicroNMEA::parseField(data, &result[0], MAXSTRING); if (data == NULL) return;
  uint32_t devId = strtol(result, NULL, 16);
  MacAddr address = devId==0?fmac.readAddr(true):fanet.getMacFromDevId(devId);
  String myDevId = fanet.getDevId(fanet.getDevIdFromMac(&address));;

  // Pilot Name
  data = MicroNMEA::parseField(data, &result[0], MAXSTRING);
  const char* pilotName = result;

  // Configure settings
  setting.outputMode = outputMode;
  setting.Mode = gxMode;
  setting.outputModeVario = outputModeVario;
  setting.outputFANET = outputFANET;
  setting.outputGPS = outputGPS;
  setting.outputFLARM = outputFLARM;
  bool hasCustomGPSConfigSet = setting.gps.customGPSConfig;
  setting.gps.customGPSConfig = customGPSConfig;
  setting.AircraftType = aircraftType;
  setting.myDevId = myDevId;
  setting.myDevIdType = myDevIdType;
  setting.PilotName = pilotName;

  // Write settings, configure uBLox if needed and re boot
  log_i("write config-to file");
  write_configFile(&setting);
  // Do not restarts GPS  to avoid a long time beofre GPS constallation is found 
  if (!hasCustomGPSConfigSet) {
#if defined(AIRMODULE)
    log_i("Setup ublox");
    setupUbloxConfig();
#endif
  } else {
    log_i("Skipping setup ublox");
  }
  log_i("reboot");
  ESP.restart();
}

void setup() {

  Serial.begin(115200);
  /*
  pinMode(36, OUTPUT);
  digitalWrite(36,LOW); 
  delay(2000); 
  ws90_init();
  */
  status.restart.doRestart = false;
  status.bPowerOff = false;
  status.bWUBroadCast = false;
  status.bInternetConnected = false;
  status.bTimeOk = false;
  status.modemstatus = eConnectionState::DISCONNECTED;
  status.bWifiOn = false;
  command.ConfigGPS = 0;
  status.gps.bHasGPS = false;
  fanet.setGPS(false);
  status.tRestart = 0;
  sMqttState[0] = 0; //zero-Termination of String !
  status.MqttStat = 0;
  status.gps.Date[0] = 0; //clear string
  status.gps.Time[0] = 0; //clear string
  xI2C0Mutex = xSemaphoreCreateMutex(); // create Mutex for I2C0
  xI2C1Mutex = xSemaphoreCreateMutex(); // create Mutex for I2C1
  log_i("SDK-Version=%s",ESP.getSdkVersion());
  log_i("CPU-Speed=%dMHz",getCpuFrequencyMhz());
  log_i("XTAL-Freq=%dMHz",getXtalFrequencyMhz());
  log_i("APB-Freq=%dHz",getApbFrequency());
  log_i("Total heap: %d", ESP.getHeapSize());
  log_i("Free heap: %d", ESP.getFreeHeap());
  log_i("size of long %d",sizeof(long));
  //function takes the following frequencies as valid values:
  //  240, 160, 80    <<< For all XTAL types
  //  40, 20, 10      <<< For 40MHz XTAL
  //  26, 13          <<< For 26MHz XTAL
  //  24, 12          <<< For 24MHz XTAL
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
  log_i("compiled at %s",compile_date.c_str());
  log_i("current free heap: %d, minimum ever free heap: %d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());

  //esp_sleep_wakeup_cause_t reason = print_wakeup_reason(); //print reason for wakeup
  esp_sleep_wakeup_cause_t reason2 = print_wakeup_reason(); //print reason for wakeup
  //esp_reset_reason_t reason = esp_reset_reason();
  // Make sure we can read the file system
  if( !SPIFFS.begin(true)){
  //if( !SPIFFS.begin(false)){
    log_e("Error mounting SPIFFS");
    //while(1);
  }
  log_i("SPIFFS total=%d used=%d free=%d",SPIFFS.totalBytes(),SPIFFS.usedBytes(),SPIFFS.totalBytes()-SPIFFS.usedBytes());
  if (SPIFFS.usedBytes() == 0){
    log_e("SPIFFS empty");
  }
  //listSpiffsFiles();
  load_configFile(&setting); //load configuration
  //setting.RFMode = setting.RFMode & 0x03; //only FANET allowed !!
  //setting.wifi.connect = eWifiMode::CONNECT_NONE;
  #ifdef GSM_MODULE
  //minimum cpu-frequency is here 80Mhz cause of GSM-Module
  if (setting.CPUFrequency <  80){
    setting.CPUFrequency = 80;
  }
  #else
  if (setting.CPUFrequency <  10){
    setting.CPUFrequency = 10;
  }  
  #endif
  //setting.boardType = eBoard::UNKNOWN;
  #ifdef S3CORE
    setting.boardType = T_BEAM_S3CORE;
  #endif
  #ifdef WIRELESS_STICK_V3
    setting.boardType = HELTEC_WIRELESS_STICK_LITE_V3;
  #endif
  #ifdef Heltec_Lora_V3
    setting.boardType = HELTEC_LORA_V3;
  #endif
  if (setting.boardType == eBoard::UNKNOWN){
    checkBoardType();
  }  
  /*
  #if defined(OLED) && !defined(EINK)
  setting.displayType = OLED0_96;
  #elif defined(EINK) && !defined(OLED)
  setting.displayType = EINK2_9;
  #elif !defined(OLED) && !defined(EINK)
  setting.displayType = NO_DISPLAY;
  #endif
  */
  status.displayType = setting.displayType; //we have to copy the display-type in case, setting is changing
  #if defined(GSMODULE)  && ! defined(AIRMODULE)
    log_i("only GS-Mode compiled");
    setting.Mode = eMode::GROUND_STATION;
  #endif
  #if defined(AIRMODULE) && ! defined(GSMODULE)
    log_i("only Air-Mode compiled");
    setting.Mode = eMode::AIR_MODULE;
  #endif
  status.gsm.bHasGSM = false;
  #ifdef GSM_MODULE
    status.gsm.bHasGSM = true;
  #endif
  #ifdef GSMODULE
  if (setting.Mode == eMode::GROUND_STATION){
    if ((reason2 == ESP_SLEEP_WAKEUP_TIMER) && (setting.gs.PowerSave == eGsPower::GS_POWER_SAFE)){
      printLocalTime();
      if (isDayTime() == 0){
        log_i("not day --> enter deep-sleep");
        uint32_t tSleep = calcSleepTime();
        //log_i("time to sleep = %d",tSleep);
        log_i("time to sleep = %d --> %02d:%02d:%02d",tSleep,tSleep/60/60,(tSleep/60)%60,(tSleep)%60);
        esp_sleep_enable_timer_wakeup((uint64_t)tSleep * uS_TO_S_FACTOR); //set Timer for wakeup      
        //esp_wifi_stop();
        //#ifdef BLUETOOTH
        //stopBluetooth();
        //#endif
        //adc_power_off();
        //adc_power_release();
        esp_deep_sleep_start();
      }
    }
  }
  #endif
  if ((setting.wifi.ssid.length() <= 0) || (setting.wifi.password.length() <= 0)){
    setting.wifi.connect = eWifiMode::CONNECT_NONE; //if no pw or ssid given --> don't connecto to wifi
  }

  //pinMode(BUTTON2, INPUT_PULLUP);

  printSettings();
  if (setting.wifi.connect == eWifiMode::CONNECT_ALWAYS){
    log_i("set tWifiStop to 0 cause we have to be always connected");
    setting.wifi.tWifiStop = 0; //no stop on wifi in case we connect always
  }   
  analogReadResolution(10); // Default of 12 is not very linear. Recommended to use 10 or 11 depending on needed resolution.
  status.PMU = ePMU::NOPMU;
  for (uint8_t i = 0; i < NUMBUTTONS; i++) {
    //init buttons
    sButton[i].PinButton = -1;
  }

  #ifdef TEST
  testLegacy();
  #endif

  switch (setting.boardType)
  {
  case eBoard::T_BEAM: 
    log_i("Board=T_BEAM");
    PinGPSRX = 34;
    PinGPSTX = 12;
    PinPPS = 37;

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

    if (setting.Mode == eMode::GROUND_STATION){
      PinWindDir = 36;
      PinWindSpeed = 39;
    }else{
      if (setting.bHasExtPowerSw){
        PinExtPowerOnOff = 36;
      }
    }

    sButton[1].PinButton = 38;
    //PinButton[1] = 38;

    if (setting.bHasFuelSensor){
      PinFuelSensor = 39;
      pinMode(PinFuelSensor, INPUT);
    }      

    pI2cOne->begin(PinOledSDA, PinOledSCL);
    setupPMU();
    //for new T-Beam output 4 is red led

    break;
  case eBoard::T_BEAM_SX1262: 
    log_i("Board=T_BEAM SX1262");
    PinGPSRX = 34;
    PinGPSTX = 12;
    PinPPS = 37;
    
    PinLoraRst = 23;
    if ((setting.displayType == EINK2_9) || (setting.displayType == EINK2_9_V2)){
      PinLoraGPIO = 36;
      PinLoraDI0 = 39;
    }else{
      PinLoraGPIO = 32;
      PinLoraDI0 = 33;
      PinUserLed = 4;
    }
    PinLora_SS = 18;
    PinLora_MISO = 19;
    PinLora_MOSI = 27;
    PinLora_SCK = 5;

    PinOledRst = -1;
    PinOledSDA = 21;
    PinOledSCL = 22;

    PinBaroSDA = 13;
    PinBaroSCL = 14;


    //V3.0.0 changed from PIN 0 to PIN 25
    PinBuzzer = 25;

    if (setting.Mode == eMode::GROUND_STATION){
      PinWindDir = 36;
      PinWindSpeed = 39;
    }else{
      if (setting.bHasExtPowerSw){
        PinExtPowerOnOff = 36;
      }
    }
    sButton[1].PinButton = 38;
    //PinButton[1] = 38;

    if (setting.bHasFuelSensor){
      PinFuelSensor = 39;
      pinMode(PinFuelSensor, INPUT);
    }      

    pI2cOne->begin(PinOledSDA, PinOledSCL);
    setupPMU();
    //for new T-Beam output 4 is red led

    break;
  case eBoard::T_BEAM_V07:
    log_i("Board=T_BEAM_V07");
    //PinGPSRX = 34;
    //PinGPSTX = 39;
    PinGPSRX = 12; //T-Beam V07
    PinGPSTX = 15;

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
    // set gpio 4 as INPUT
    pinMode(PinBaroSCL, INPUT_PULLUP);

    if (setting.bHasFuelSensor){
      PinFuelSensor = 39;
      pinMode(PinFuelSensor, INPUT);
    }    

    PinBuzzer = 25;

    // Lilygo T3 v2.1.1.6 extra button on 0
    sButton[1].PinButton = 0;

    pI2cOne->begin(PinOledSDA, PinOledSCL);
    // voltage-divier 100kOhm and 100kOhm
    // vIn = (R1+R2)/R2 * VOut
    PinADCVoltage = 35;
    adcVoltageMultiplier = 2.12f;
    if (setting.Mode == eMode::GROUND_STATION){
      PinWindDir = 36;
      PinWindSpeed = 39;
    }    
    break;
  case eBoard::TTGO_T3_V1_6:
    log_i("Board=TTGO T3 V1.6");
    PinGPSRX = 36;
    PinGPSTX = 19;
    PinPPS = 39;

    PinLoraRst = 23;
    PinLoraDI0 = 26;
    PinLora_SS = 18;
    PinLora_MISO = 19;
    PinLora_MOSI = 27;
    PinLora_SCK = 5;

    PinBeaconLed = 25;

    PinOledSDA = 21;
    PinOledSCL = 22;

    // voltage-divier 100kOhm and 100kOhm
    // vIn = (R1+R2)/R2 * VOut
    PinADCVoltage = 35;
    adcVoltageMultiplier = 2.12f;    

    pI2cOne->begin(PinOledSDA, PinOledSCL);
    break;
  case eBoard::HELTEC_LORA:
    log_i("Board=HELTEC_LORA");
    //PinGPSRX = 34;
    //PinGPSTX = 39;
    PinGPSRX = 12;
    //PinGPSTX = 15; // no GPS-TX


    PinLoraRst = 14;
    PinLoraDI0 = 26;
    PinLora_SS = 18;
    PinLora_MISO = 19;
    PinLora_MOSI = 27;
    PinLora_SCK = 5;
    PinGsmRst = 25;
    PinGsmTx = 21;
    PinGsmRx = 35;

    if (setting.displayType == OLED0_96){
      PinOledRst = 16;
      PinOledSDA = 4;
      PinOledSCL = 15;
      pI2cOne->begin(PinOledSDA, PinOledSCL);
    }

    PinBuzzer = 17;

    PinBaroSDA = 13;
    PinBaroSCL = 23;

    PinOneWire = 22;    

    PinWindDir = 36;
    PinWindSpeed = 37;
    PinRainGauge = 38;

    #ifdef GXTEST
      PinPPS = 37;
    #endif

    
    if (setting.bHasFuelSensor){
      PinFuelSensor = 39;
      pinMode(PinFuelSensor, INPUT);
    }    



    sButton[0].PinButton = 0; //pin for program-Led
    //PinButton[0] = 0; //pin for Program-Led

    // voltage-divier 27kOhm and 100kOhm
    // vIn = (R1+R2)/R2 * VOut
    //1S LiPo
    PinADCVoltage = 34;
    adcVoltageMultiplier = (100000.0f + 27000.0f) / 100000.0f;
    break;
  case eBoard::HELTEC_WIRELESS_STICK_LITE:
    log_i("Board=Heltec Wireless Stick Lite");

#ifdef AIRMODULE
    PinBeaconLed = 25;
#endif

    PinGPSRX = 9;
    PinGPSTX = 10;
    PinPPS = 23;


    PinLoraRst = 14;
    PinLoraDI0 = 26;
    PinLora_SS = 18;
    PinLora_MISO = 19;
    PinLora_MOSI = 27;
    PinLora_SCK = 5;

    PinBaroSDA = 32;
    PinBaroSCL = 33;

    //PinOneWire = 22;    

    PinWindDir = 36;
    PinWindSpeed = 39;
    PinRainGauge = 38;

    sButton[0].PinButton = 0; //pin for program-button

    // voltage-divier 27kOhm and 100kOhm
    // vIn = (R1+R2)/R2 * VOut
    //1S LiPo
    PinExtPower = 21;
    //adcVoltageMultiplier =  (100000.0f + 220000.0f) / 100000.0f;
    PinADCVoltage = 37;
    adcVoltageMultiplier =  3.69f;
    break;
  case eBoard::HELTEC_WIRELESS_STICK:
    log_i("Board=HELTEC Wireless Stick");
#ifdef AIRMODULE
    PinBeaconLed = 25;
#endif
    sButton[0].PinButton = 0; //pin for program-button
    
    //PinGPSRX = 34;
    //PinGPSTX = 39;


    PinLoraRst = 14;
    PinLoraDI0 = 26;
    PinLora_SS = 18;
    PinLora_MISO = 19;
    PinLora_MOSI = 27;
    PinLora_SCK = 5;

    PinOledRst = 16;
    PinOledSDA = 4;
    PinOledSCL = 15;
    pI2cOne->begin(PinOledSDA, PinOledSCL);

    PinBaroSDA = 12;
    PinBaroSCL = 13;

    PinWindDir = 36;
    PinWindSpeed = 39;

    PinOneWire = 23;   

    PinExtPower = 21;
    break;
  case eBoard::TTGO_TSIM_7000:
    log_i("Board=TTGO_TSIM_7000");

    //E-Ink
    PinEink_Busy   =  39;
    PinEink_Rst    =  25;
    PinEink_Dc     =  15;
    PinEink_Cs     =  13;
    PinEink_Clk    =  14;
    PinEink_Din    =  2;


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

    PinOledRst = -1; //no oled-support yet
    PinOledSDA = -1;
    PinOledSCL = -1;

    PinBaroSDA = 21;
    PinBaroSCL = 22;

    if (setting.displayType > 1){
      PinOneWire = -1; //no one-wire if display is eink, cause we need that pin
    }else{
      PinOneWire = 25;    
    }
    

    PinWindDir = 33;
    PinWindSpeed = 34;
    PinRainGauge = 39;

    
    //PinUserLed = 12; //PinLoraRst
    
    PinBuzzer = 0;

    // voltage-divier 100kOhm and 100kOhm
    // vIn = (R1+R2)/R2 * VOut
    PinADCVoltage = 35;
    adcVoltageMultiplier = 2.2279f; // not sure if it is ok ?? don't have this kind of board

    break;
  case eBoard::TTGO_TCALL_800:

    PinLoraRst = 12;
    PinLoraDI0 = 32;
    PinLora_SS = 5;
    //pinMode(5, OUTPUT);
    //digitalWrite(5,LOW); 
    //PinLora_SS = 16; //unused Pin, but pin5 is also for reset of GSM
    PinLora_MISO = 19;
    PinLora_MOSI = 17;
    PinLora_SCK = 18;

    PinBaroSDA = 21;
    PinBaroSCL = 22;

    PinGsmPower = 23;
    PinGsmRst = 4; //is PowerKey, but IO5 is covered by Lora SS
    PinGsmTx = 27;
    PinGsmRx = 26;

    PinADCVoltage = 35;
    adcVoltageMultiplier = 2.12f; // not sure if it is ok ?? don't have this kind of board    

    break;
  case eBoard::T_BEAM_S3CORE:
    log_i("Board=T_BEAM-S3CORE");
    //setting.displayType = eDisplay::OLED0_96;
    PinGPSRX = 9;
    PinGPSTX = 8;
    //wakeup GPS
    pinMode(7,OUTPUT);
    digitalWrite(7,LOW);

    PinPPS = 6;

    PinLoraRst = 5;
    PinLoraDI0 = 1;
    PinLoraGPIO = 4;
    PinLora_SS = 10;
    PinLora_MISO = 13;
    PinLora_MOSI = 11;
    PinLora_SCK = 12;

    PinOledRst = -1;
    PinOledSDA = 17; //OLED + BARO + RTC
    PinOledSCL = 18; //OLED + BARO + RTC
    //PinOledSDA = 42; //PMU
    //PinOledSCL = 41; //PMU

    PinBaroSDA = 17;
    PinBaroSCL = 18;
    pI2cOne->begin(PinBaroSDA, PinBaroSCL);
    PinPMU_Irq = 40;
    PinBuzzer = 48;
    setupPMU();
    /*
    pI2cZero->begin(PinBaroSDA, PinBaroSCL);
    while(1){
      i2cScanner();
      delay(5000);
    }
    */

    break;
  case eBoard::HELTEC_WIRELESS_STICK_LITE_V3:
    log_i("Board=wireless-StickV3");
    sButton[0].PinButton = 0; //pin for program-button
    PinLoraRst = 12;
    PinLoraDI0 = 14;
    PinLoraGPIO = 13;
    PinLora_SS = 8;
    PinLora_MISO = 11;
    PinLora_MOSI = 10;
    PinLora_SCK = 9;

    PinBaroSDA = 33;
    PinBaroSCL = 34;
    pI2cOne->begin(PinBaroSDA, PinBaroSCL);

    PinWindDir = 2;
    PinWindSpeed = 3;    

    PinOneWire = 4; //pin for one-Wire

    #ifdef TINY_GSM_MODEM_SIM7080
      PinGsmRst = 7; //is PowerKey
      PinGsmTx = 5;
      PinGsmRx = 6;
    #endif

    pinMode(35,OUTPUT);
    digitalWrite(35,LOW); //switch user-LED off

    PinExtPower = 36; //pin for external Voltage-control
    PinADCCtrl = 37; //pin for reading battery-voltage
    PinADCVoltage = 1;
    adcVoltageMultiplier =  5.2636f;
    break;
  case eBoard::HELTEC_LORA_V3:
    log_i("Board=HELTEC_LORA_V3");

    PinLora_SS = 8;
    PinLora_SCK = 9;
    PinLora_MOSI = 10;
    PinLora_MISO = 11;
    PinLoraRst = 12;
    PinLoraGPIO = 13;
    PinLoraDI0 = 14;

    PinOledRst = 21;
    PinOledSDA = 17;
    PinOledSCL = 18;
    pI2cOne->begin(PinOledSDA, PinOledSCL);

    PinBaroSDA = 2;
    PinBaroSCL = 3;

    //pI2cOne->begin(PinBaroSDA, PinBaroSCL);

    PinWindDir = 6;
    PinWindSpeed =7;    

    pinMode(35,OUTPUT);
    digitalWrite(35,LOW); //switch user-LED off
    //digitalWrite(35,HIGH);

    PinExtPower = 36; //pin for external Voltage-control
    PinADCCtrl = 37; //pin for reading battery-voltage
    PinADCVoltage = 1;
    adcVoltageMultiplier =  5.2636f;
    break;
  case eBoard::UNKNOWN:
    log_e("unknown Board --> please correct");
    break;
  default:
    log_e("wrong-board-definition --> restart");
    setting.boardType = eBoard::UNKNOWN;    
    write_configFile(&setting);
    delay(1000);
    esp_restart(); //we need to restart    break;
  }

  for (uint8_t i = 0; i < NUMBUTTONS; i++) {
    // initialize built-in LED as an output
    if (sButton[i].PinButton >= 0){
      //set pin for button as input
      pinMode(sButton[i].PinButton, INPUT_PULLUP);
      // initialize the corresponding AceButton
      buttons[i].init(sButton[i].PinButton, HIGH, i);
    }
  }
  // Configure the ButtonConfig with the event handler, and enable all higher
  // level events.
  ace_button::ButtonConfig* buttonConfig = ace_button::ButtonConfig::getSystemButtonConfig();
  buttonConfig->setEventHandler(handleEvent);
  buttonConfig->setFeature(ace_button::ButtonConfig::kFeatureClick);
  buttonConfig->setFeature(ace_button::ButtonConfig::kFeatureDoubleClick);
  buttonConfig->setFeature(ace_button::ButtonConfig::kFeatureLongPress);
  buttonConfig->setFeature(ace_button::ButtonConfig::kFeatureRepeatPress);
  buttonConfig->setDebounceDelay(20); //set debounce-delay to 20ms
  buttonConfig->setLongPressDelay(1000); //set long-press-delay to 500ms
  buttonConfig->setClickDelay(500); //set click-delay to 200ms


  if (PinBeaconLed >= 0) {
    pinMode(PinBeaconLed, OUTPUT);
    digitalWrite(PinBeaconLed,LOW); 
  }
  if (PinUserLed >= 0){
    pinMode(PinUserLed, OUTPUT);
    digitalWrite(PinUserLed,HIGH); 
  }
  if (PinExtPowerOnOff >= 0){
    pinMode(PinExtPowerOnOff, INPUT);
    log_i("ext power-state=%d",digitalRead(PinExtPowerOnOff));
  }
  if (PinExtPower >= 0){
    pinMode(PinExtPower, OUTPUT); //we have to set pin 21 to measure voltage of Battery
    digitalWrite(PinExtPower,LOW); //set output to Low, so we can measure the voltage
    log_i("set ext-power");
    delay(500); //wait until devices are on
  }
  if (PinADCCtrl >= 0){
    pinMode(PinADCCtrl, OUTPUT); //we have to set pin to measure voltage of Battery
    digitalWrite(PinADCCtrl,LOW); //set output to Low, so we can measure the voltage  
    log_i("set adcCtrl"); 
    delay(100); 
  }
  if (PinADCVoltage >= 0){
    pinMode(PinADCVoltage, INPUT); //set pin ADC-Voltage as input
  }


  
  #ifdef GSMODULE
  if (setting.Mode == eMode::GROUND_STATION){
    if ((setting.gs.PowerSave == eGsPower::GS_POWER_BATT_LIFE) || (setting.gs.PowerSave == eGsPower::GS_POWER_SAFE)){
      printBattVoltage(millis()); //read Battery-level
      log_i("Batt %dV; %d%%",status.battery.voltage,status.battery.percent);
      if ((status.battery.voltage >= BATTPINOK) && (status.battery.percent < (setting.minBattPercent + setting.restartBattPercent)) && (status.battery.percent < 80)){
        //go again to sleep
        printLocalTime();
        log_i("batt-voltage to less (%d%%) --> going to sleep again for 60min.",status.battery.percent);
        esp_sleep_enable_timer_wakeup((uint64_t)BATTSLEEPTIME * uS_TO_S_FACTOR); //set Timer for wakeup      
        //esp_wifi_stop();        
        #ifdef BLUETOOTH
        log_i("disable bluetooth for power-saving");
        stopBluetooth();
        #endif
        esp_deep_sleep_start();
      }
    }
  }
  #endif


  // setting.myDevId = ""; //MyDevID is now stored on file because it's used together with the addressType
#ifdef GSM_MODULE
xGsmMutex = xSemaphoreCreateMutex();
#endif

xOutputMutex = xSemaphoreCreateMutex();

  //log_i("currHeap:%d,minHeap:%d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
#ifdef AIRMODULE

  if (setting.Mode == eMode::AIR_MODULE){

    //adding logger task
    #ifdef FLARMLOGGER
      xTaskCreatePinnedToCore(taskLogger, "taskLogger", 6500, NULL, 4, &xHandleLogger, ARDUINO_RUNNING_CORE1); //background Logger
    #endif
    //#ifndef S3CORE  //still in progress
    xTaskCreatePinnedToCore(taskBaro, "taskBaro", 6500, NULL, 9, &xHandleBaro, ARDUINO_RUNNING_CORE1); //high priority task
    //#endif
  }
#endif  
  //log_i("currHeap:%d,minHeap:%d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
  xTaskCreatePinnedToCore(taskStandard, "taskStandard", 6500, NULL, 10, &xHandleStandard, ARDUINO_RUNNING_CORE1); //standard task
  //log_i("currHeap:%d,minHeap:%d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());

#ifdef EINK
  xTaskCreatePinnedToCore(taskEInk, "taskEInk", 6500, NULL, 8, &xHandleEInk, ARDUINO_RUNNING_CORE1); //background EInk
#endif
#if defined(SSD1306) || defined(SH1106G)
xTaskCreatePinnedToCore(taskOled, "taskOled", 6500, NULL, 8, &xHandleOled, ARDUINO_RUNNING_CORE1); //background Oled
#endif
  xTaskCreatePinnedToCore(taskBackGround, "taskBackGround", 6500, NULL, 5, &xHandleBackground, ARDUINO_RUNNING_CORE1); //background task
  #ifdef BLUETOOTH
  xTaskCreatePinnedToCore(taskBluetooth, "taskBluetooth", 4096, NULL, 7, &xHandleBluetooth, ARDUINO_RUNNING_CORE1);
  #endif

#ifdef GSMODULE  
  if (setting.Mode == eMode::GROUND_STATION){
    if ((setting.WUUpload.enable) && (!setting.wd.mode.bits.enable)){
      status.bWUBroadCast = true;
      log_i("wu broadcast enabled");
    }
    if ((status.bWUBroadCast) || (setting.wd.mode.bits.enable)){
      //start weather-task
      xTaskCreatePinnedToCore(taskWeather, "taskWeather", 13000, NULL, 8, &xHandleWeather, ARDUINO_RUNNING_CORE1);
    }
  }
#endif  
#ifdef GSM_MODULE
  //start Gsm-task
  xTaskCreatePinnedToCore(taskGsm, "taskGsm", 4096, NULL, 3, &xHandleGsm, ARDUINO_RUNNING_CORE1);
#endif
}


#ifdef GSM_MODULE

bool connectGPRS(){
  log_i("Connecting to internet apn=%s,user=%s,pw=%s",setting.gsm.apn.c_str(),setting.gsm.user.c_str(),setting.gsm.pwd.c_str());
  return modem.gprsConnect(setting.gsm.apn.c_str(), setting.gsm.user.c_str(), setting.gsm.pwd.c_str());
}

void readModem2Serial(const char* GsmConstStr,uint32_t timeout_ms){
  modem.streamWrite(GsmConstStr);
  for (uint32_t start = millis(); millis() - start < timeout_ms;) {
    Serial.println(modem.stream.readString());  
    delay(100);
  }
  /*
  uint32_t tStart = millis();
  int i = 0;
  while(i < 2){
    String s = modem.stream.readStringUntil('\n');
    Serial.println(s);  
    i++;
  }
  */  
}

bool connectModem(){
  log_i("Waiting for network...");
  if (!modem.waitForNetwork(600000L)){
    log_e("no network");
    return false;
  } 
  
  //readModem2Serial("AT+CPSI?\r\n",2000);
  //readModem2Serial("AT+IPR=?\r\n",2000); //read baud-rates
  //readModem2Serial("AT+IPR?\r\n",2000); //read actual baud-rate
  
  status.gsm.SignalQuality = modem.getSignalQuality();
  //log_i("signal quality %d",status.gsm.SignalQuality);
  status.gsm.sOperator = modem.getOperator();
  //log_i("operator=%s",status.gsm.sOperator.c_str());
  #if defined(TINY_GSM_MODEM_SIM7000) || defined(TINY_GSM_MODEM_SIM7080)
  bool bAutoreport;
  if (modem.getNetworkSystemMode(bAutoreport,status.gsm.networkstat)){        
    //log_i("network system mode %d",status.gsm.networkstat);
  }else{
    log_e("can't get Networksystemmode");
  }
  #endif
  if (!connectGPRS()) return false;
  status.wifiSTA.ip = modem.getLocalIP();
  log_i("connected successfully IP:%s",status.wifiSTA.ip.c_str());
  return true;
}

bool factoryResetModem(){
  //reset modem
  if (PinGsmRst >= 0){
    log_i("reset modem");
    digitalWrite(PinGsmRst,HIGH);
    delay(100);
    digitalWrite(PinGsmRst,LOW);
    delay(1200);
    digitalWrite(PinGsmRst,HIGH);
    delay(6000); //wait until modem is ok now
  }  
  log_i("test modem-connection");
  if (!modem.testAT()){
    log_e("modem-connection not OK");
    return false;
  } 
  //log_i("set factory-settings");
  modem.sendAT(GF("&F"));  // Factory settings
  modem.waitResponse();
  if (!modem.testAT()){
    log_e("modem-connection not OK");
    return false;
  } 
  modem.sendAT(GF("&W"));  // Factory settings
  modem.waitResponse();
  return true;
}

bool TestModemconnection(unsigned long baud){
  log_i("Modem switch baudrate to %d",baud);
  GsmSerial.begin(baud,SERIAL_8N1,PinGsmRx,PinGsmTx,false); //baud, config, rx, tx, invert
  log_i("test modem connection");
  if (modem.testAT()){
    status.gsm.baud = baud; //set baudrate in status
    log_i("modem connection ok");
    return true;
  }else{
    return false;
  }
}

bool initModem(){
  //reset modem
  command.getGpsPos = 0;
  if (PinGsmRst >= 0){
    log_i("reset modem");
    digitalWrite(PinGsmRst,HIGH);
    delay(100);
    digitalWrite(PinGsmRst,LOW);
    delay(1200);
    digitalWrite(PinGsmRst,HIGH);
    delay(3000); //wait until modem is ok now
  }
  #if defined(TINY_GSM_MODEM_SIM7000) || defined(TINY_GSM_MODEM_SIM7080)
    //on SIM7000 we use other baudrate !!    
    if (!TestModemconnection(GSM_MAX_BAUD)){
      if (!TestModemconnection(115200)){      
        log_e("modem-connection not OK");
        return false;
      }
    }
  #else
    if (!TestModemconnection(115200)){
      log_e("modem-connection not OK");
      return false;
    }
  #endif
  String sVer = modem.getModemInfo();
  log_i("Version of SIM7000 %s",sVer.c_str());
  bool bRet = false;
  for (int i = 0;i <3;i++){
    log_i("restarting modem...");
    if (modem.restart()){
      bRet = true;
      break;
    }
  }
  if (!bRet){
    log_e("restarting modem failed");
    return false; 
  }
  
  #if defined(TINY_GSM_MODEM_SIM7000) || defined(TINY_GSM_MODEM_SIM7080)

  if (status.gsm.baud != GSM_MAX_BAUD){
    log_i("GSM-Modem switch baudrate to %d",GSM_MAX_BAUD);
    modem.setBaud(GSM_MAX_BAUD); //set Baudrate to 921600
    GsmSerial.begin(GSM_MAX_BAUD,SERIAL_8N1,PinGsmRx,PinGsmTx,false); //baud, config, rx, tx, invert
    status.gsm.baud = GSM_MAX_BAUD;
  }
  //readModem2Serial("AT+COPS=?\r\n",50000);
  log_i("set NetworkMode to %d",setting.gsm.NetworkMode);
  modem.setNetworkMode(setting.gsm.NetworkMode); //set mode
  if (setting.gsm.PreferredMode > 0){
    delay(500);
    log_i("set PreferredMode to %d",setting.gsm.PreferredMode);
    modem.setPreferredMode(setting.gsm.PreferredMode); //set preferred mode
  }else{
    modem.setPreferredMode(3); //set preferred mode to CAT-M and NB-IoT
  }

  if (setting.gsm.NB_IOT_Band.length() > 0){
    modem.sendAT(String("+CBANDCFG=\"NB-IOT\"," + setting.gsm.NB_IOT_Band).c_str());
  }else{
    modem.sendAT("+CBANDCFG=\"NB-IOT\",3,5,8,20,28"); //set to default bands
  }
  modem.waitResponse();
  if (setting.gsm.CAT_M_Band.length() > 0){
    modem.sendAT(String("+CBANDCFG=\"CAT-M\"," + setting.gsm.CAT_M_Band).c_str());
  }else{
    modem.sendAT("+CBANDCFG=\"CAT-M\",3,5,8,20,28"); //set to default bands
  }
  modem.waitResponse();
  //readModem2Serial("AT+CBANDCFG=?\r\n",2000);
  //readModem2Serial("AT+CBANDCFG?\r\n",2000);  
  #endif
  modem.sleepEnable(false); //set sleepmode off
  modem.sendAT(GF("+CMGF=1"));
  modem.waitResponse();  
  modem.sendAT("+CNETLIGHT=0"); //turn off net-light to redure Power
  modem.waitResponse();  
  return true;
}

void PowerOffModem(){
  status.modemstatus = eConnectionState::DISCONNECTED;
  while(1){
    if (xSemaphoreTake( xGsmMutex, ( TickType_t ) 500 ) == pdTRUE ){
      log_i("stop gprs connection");
      modem.gprsDisconnect();
      log_i("switch radio off");
      modem.radioOff();  
      #if defined(TINY_GSM_MODEM_SIM7000) || defined(TINY_GSM_MODEM_SIM7080)
        #ifdef TINY_GSM_MODEM_SIM7000
          digitalWrite(5,HIGH);
        #endif
        //Power-off SIM7000-module
        modem.sleepEnable(false); // required in case sleep was activated and will apply after reboot
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
      if (PinGsmRst >= 0){
        pinMode(PinGsmRst,INPUT); //switch to input, so it is floating        
        /*
        #if defined(TINY_GSM_MODEM_SIM7080)
          
        #else
          digitalWrite(PinGsmRst,LOW);
        #endif
        */
      }      
      if (PinGsmPower >= 0){
        // Turn off the Modem power first
        digitalWrite(PinGsmPower, LOW);
      }
      xSemaphoreGive( xGsmMutex );
      break;
    }else{
      log_i("can't get Mutex for GSM");
    }
  }
}

#if defined(TINY_GSM_MODEM_SIM7000) || defined(TINY_GSM_MODEM_SIM7080)
void setupSim7000Gps(){
  xSemaphoreTake( xGsmMutex, portMAX_DELAY );
  modem.sendAT("+SGPIO=0,4,1,1");
  modem.waitResponse(10000L);
  modem.enableGPS(); //enable GPS
  xSemaphoreGive( xGsmMutex );
  command.getGpsPos = 2; //setup GPS finished --> waiting for GPS-Position
}
#endif

void taskGsm(void *pvParameters){  
  if (PinGsmPower >= 0){
    pinMode(PinGsmPower, OUTPUT);
    // Turn on the Modem power first
    digitalWrite(PinGsmPower, HIGH);
  }
  if (PinGsmRst >= 0){
    pinMode(PinGsmRst, OUTPUT); //set GsmReset to output
    digitalWrite(PinGsmRst,HIGH);
  }
  status.gsm.sOperator = ""; 
  GsmSerial.begin(115200,SERIAL_8N1,PinGsmRx,PinGsmTx,false); //baud, config, rx, tx, invert
  //const TickType_t xDelay = 60000 / portTICK_PERIOD_MS;   //only every 60sek.
  //TickType_t xLastWakeTime = xTaskGetTickCount (); //get actual tick-count
  //bool status;
  uint32_t tAct = millis();
  static uint32_t tCheckConn = millis() - GSM_CHECK_TIME_CON; //check every 60sec.
  //static uint32_t tCheckSms = millis() - GSM_CHECK_TIME_SMS;
  // Set preferred message format to text mode
  delay(1000); //wait 2 seconds until power is stable
  xSemaphoreTake( xGsmMutex, portMAX_DELAY );
  //factoryResetModem();
  //we try it twice because on sim7080, the power-pin switches the modem on and off
  //so it can be, that we switch it off
  for (int i = 0;i<2;i++){ 
    if (initModem()){
      break;
    }
  }  
  xSemaphoreGive( xGsmMutex );
  if (setting.wifi.connect != eWifiMode::CONNECT_NONE){
    log_i("stop task");
    //xSemaphoreTake( xGsmMutex, portMAX_DELAY );
    PowerOffModem();  
    //xSemaphoreGive( xGsmMutex );  
    vTaskDelete(xHandleGsm); //delete weather-task
    return;
  }
  xSemaphoreTake( xGsmMutex, portMAX_DELAY );
  status.modemstatus = eConnectionState::CONNECTING;
  connectModem(); //connect modem to network  
  /*
  if (modem.isNetworkConnected()){  
    connectGPRS();
  }
  */
  xSemaphoreGive( xGsmMutex );
  tCheckConn = millis() - GSM_CHECK_TIME_CON + 1000;
  while(1){
    tAct = millis();
    
    #if defined(TINY_GSM_MODEM_SIM7000) || defined(TINY_GSM_MODEM_SIM7080)
    if (command.getGpsPos == 1){
      setupSim7000Gps();
    }else if (command.getGpsPos == 2){
      float lat,  lon, speed, alt, accuracy;
      int vsat, usat;
      xSemaphoreTake( xGsmMutex, portMAX_DELAY );
      if (modem.getGPS(&lat, &lon, &speed, &alt, &vsat, &usat, &accuracy)) {
        log_i("GPS-Position: lat:%.6f lon:%.6f alt:%.2f vsat:%d usat:%d accuracy:%.2f",lat,lon,alt,vsat,usat,accuracy);
        if ((usat >= 6) || ((accuracy < 2.0) && (vsat >= 10))){ //only if we have more then 6 satellites in view or accuracy < 2.0m on SIM7080 we get only accuracy
          setting.gs.lat = lat;
          setting.gs.lon = lon;
          setting.gs.alt = alt;
          command.getGpsPos = 3;
        }        
      }
      xSemaphoreGive( xGsmMutex );
    }else if (command.getGpsPos == 3){
      write_configFile(&setting); //write config-File
      ESP.restart();
    }
    #endif
    //String gps_raw = modem.getGPSraw();
    //log_i("GPS-Position:%s",gps_raw.c_str());
    //xSemaphoreGive( xGsmMutex );
    if ((timeOver(tAct,tCheckConn,GSM_CHECK_TIME_CON)) && (status.modemstatus != eConnectionState::CONNECTED)){
      tCheckConn = tAct;
      //log_i("%d Check GSM-Connection",tCheckConn);
      if (xSemaphoreTake( xGsmMutex, ( TickType_t ) 500) == pdTRUE ){
        //log_i("%d Check GSM-Connection",tCheckConn);
        if (modem.isGprsConnected()){
          status.modemstatus = eConnectionState::CONNECTED;
          //xLastWakeTime = xTaskGetTickCount (); //get actual tick-count
          tCheckConn = millis();
          status.gsm.SignalQuality = modem.getSignalQuality();
          if (status.gsm.sOperator.length() == 0){
            status.gsm.sOperator = modem.getOperator();
          }
          #if defined(TINY_GSM_MODEM_SIM7000) || defined(TINY_GSM_MODEM_SIM7080)
          bool bAutoreport;
          if (modem.getNetworkSystemMode(bAutoreport,status.gsm.networkstat)){        
            //log_i("network system mode %d",status.gsm.networkstat);
          }else{
            log_e("can't get Networksystemmode");
          }
          #endif
        }else{
          status.modemstatus = eConnectionState::CONNECTING;
          if (modem.isNetworkConnected()){  
            connectGPRS();
            status.wifiSTA.ip = modem.getLocalIP();
            log_i("connected successfully IP:%s",status.wifiSTA.ip.c_str());
          }else{
            initModem(); //init modem
            connectModem(); //connect modem to network
          }
        }          
        xSemaphoreGive( xGsmMutex );
      }
    }
    if (bGsmOff) break; //we need GSM for webupdate
    //delay(1);
    delay(1000);
    //vTaskDelayUntil( &xLastWakeTime, xDelay); //wait until next cycle
  }
  PowerOffModem();
  log_i("stop task");
  vTaskDelete(xHandleGsm); //delete weather-task
}

#endif



#ifdef GSMODULE
void setRTCTime(tm &timeinfo){
  //log_i("set time of RTC to %04d%02d%02d-%02d:%02d:%02d",timeinfo.tm_year + 1900,timeinfo.tm_mon+1,timeinfo.tm_mday,timeinfo.tm_hour,timeinfo.tm_min,timeinfo.tm_sec);
  if ((status.rtc.module == RTC_3031) && (pRtc3031)){
    pRtc3031->setTime(timeinfo.tm_year + 1900,timeinfo.tm_mon+1,timeinfo.tm_mday,timeinfo.tm_hour,timeinfo.tm_min,timeinfo.tm_sec);
  }else if ((status.rtc.module == RTC_3231) && (pRtc3231)){
    DateTime tAct = pRtc3231->now(); //read time and check time-difference
    DateTime tNew = DateTime(timeinfo.tm_year + 1900, timeinfo.tm_mon+1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    uint32_t tDiff = tNew.unixtime() - tAct.unixtime();    
    if (tDiff == 0){
      return; //nothing to do
    }
    for (int i = 0; i < 3; i++){
      pRtc3231->adjust(tNew);  
      log_i("timediff to big (%d) --> adjust rtc-time to %04d-%02d-%02d_%d:%02d:%02d",tDiff,timeinfo.tm_year + 1900, timeinfo.tm_mon+1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);      
      tAct = pRtc3231->now(); //read time and check time-difference
      tDiff = tNew.unixtime() - tAct.unixtime();
      if (tDiff == 0) return; //ready
    }
    delete pRtc3231; //delete again
    pRtc3231 = NULL;
    log_e("time can't be set --> remove rtc");
    status.rtc.module = RTC_NONE;
  }
}

void getRTCTime(){
  if ((status.rtc.module == RTC_3031) && (pRtc3031)){
    status.rtc.temp = float(pRtc3031->getTemperatureC());
    status.rtc.voltage = pRtc3031->getVoltage();
    //log_i("RTC temp=%dC;Batt=%.2fV",status.rtc.temp,status.rtc.voltage);  
    sTimeData_t rtcTime =  pRtc3031->getRTCTime();
    //log_i("%04d%02d%02d-%02d:%02d:%02d",rtcTime.year,rtcTime.month,rtcTime.day,rtcTime.hour,rtcTime.minute,rtcTime.second);
    struct tm timeinfo;
    timeinfo.tm_year = rtcTime.year - 1900;
    timeinfo.tm_mon = rtcTime.month - 1; //month begin with 0
    timeinfo.tm_mday = rtcTime.day;
    timeinfo.tm_hour = rtcTime.hour;
    timeinfo.tm_min = rtcTime.minute; 
    timeinfo.tm_sec = rtcTime.second;
    setAllTime(timeinfo); //we have to set time to GPS-Time for decoding.  
    status.bTimeOk = true;
  }else if ((status.rtc.module == RTC_3231) && (pRtc3231)){
    status.rtc.temp = int(pRtc3231->getTemperature());
    status.rtc.voltage = 0.0;
    DateTime now = pRtc3231->now();
    //log_i("%04d%02d%02d-%02d:%02d:%02d",now.year(),now.month(),now.day(),now.hour(),now.minute(),now.second());
    struct tm timeinfo;
    timeinfo.tm_year = now.year() - 1900;
    timeinfo.tm_mon = now.month() - 1; //month begin with 0
    timeinfo.tm_mday = now.day();
    timeinfo.tm_hour = now.hour();
    timeinfo.tm_min = now.minute(); 
    timeinfo.tm_sec = now.second();
    //log_i("set time from rtc-time to %04d-%02d-%02d_%d:%02d:%02d",timeinfo.tm_year + 1900, timeinfo.tm_mon+1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    setAllTime(timeinfo); //we have to set time to GPS-Time for decoding.  
    status.bTimeOk = true;    
  }
}

void getRTC(){
  log_i("searching for RTC");
  if (pRtc3031 == NULL) pRtc3031 = new DFRobot_SD3031(pI2cZero);
  if (!pRtc3031->begin()){
    log_i("found SD3031 RTC");
    pRtc3031->disable32k(); //we don't need the 32k output --> disable it
    pRtc3031->clearAlarm();
    pRtc3031->setAlarm(pRtc3031->eEveryDay,4,0,0);
    status.rtc.module = RTC_3031;
    getRTCTime();
    printLocalTime();    
    return;
  }
  delete pRtc3031; //delete again
  pRtc3031 = NULL;  
  if (pRtc3231 == NULL) pRtc3231 = new RTC_DS3231();
  if (pRtc3231->begin(pI2cZero)) {
    log_i("found DS3231 RTC");
    pRtc3231->disable32K(); //we don't need the 32k output --> disable it
    // set alarm 1, 2 flag to false (so alarm 1, 2 didn't happen so far)
    // if not done, this easily leads to problems, as both register aren't reset on reboot/recompile
    pRtc3231->clearAlarm(1);
    pRtc3231->clearAlarm(2);    
    status.rtc.module = RTC_3231;
    // stop oscillating signals at SQW Pin
    // otherwise setAlarm1 will fail
    pRtc3231->writeSqwPinMode(DS3231_OFF);
    // turn off alarm 2 (in case it isn't off already)
    // again, this isn't done at reboot, so a previously set alarm could easily go overlooked
    pRtc3231->disableAlarm(1);
    pRtc3231->disableAlarm(2);
    // schedule an alarm on 04:00:00 UTC
    //if (!pRtc3231->setAlarm1(pRtc3231->now() + TimeSpan(10),DS3231_A1_Second)){ // this mode triggers the alarm when the seconds match. See Doxygen for other options
    if (!pRtc3231->setAlarm1(DateTime(2024,6,17,4,00,00),DS3231_A1_Hour)){
      log_e("Error, alarm wasn't set!");
    }
    // schedule an alarm on 16:00:00 UTC
    if (!pRtc3231->setAlarm2(DateTime(2024,6,17,16,00,00),DS3231_A2_Hour)){
      log_e("Error, alarm wasn't set!");
    }    
    getRTCTime();
    printLocalTime();
    return;  
  }
  log_i("no RTC found");
  delete pRtc3231; //delete again
  pRtc3231 = NULL;
}

#ifdef SEND_RAW_WIND_DATA
void sendRawWeatherData(Weather::weatherData *weather){
  static uint32_t tsend = millis();
  if (!timeOver(millis(),tsend,1000)){
    return;
  }
  tsend = millis();
  static char msg_buf[500];
  pWdRaw = &msg_buf[0];
  StaticJsonDocument<500> doc;                      //Memory pool
  char buff[30];
  snprintf (buff,sizeof(buff)-1,"%04d-%02d-%02dT%02d:%02d:%02d+00:00",year(),month(),day(),hour(),minute(),second()); // ISO 8601
  doc["DT"] = buff;
  doc["wDir"] = round2(weather->WindDir);
  doc["wSpeed"] = round2(weather->WindSpeed);
  doc["wGust"] = round2(weather->WindGust);
  serializeJson(doc, pWdRaw, 500);
  wdRawCount++;
}
#endif

void taskWeather(void *pvParameters){
  static uint32_t tUploadData; //first sending is in Sending-intervall to have steady-values
  static uint32_t tSendData; //first sending is in FANET-Sending-intervall to have steady-values
  static uint32_t tLastWindSpeed; //
  static uint32_t tWindOk;
  static uint32_t tGetRTCTime;
  bool bDataOk = false;
  log_i("starting weather-task ");  
  Weather::weatherData wData;
  Weather::weatherData wFntData;
  WeatherUnderground::wData wuData;
  Windy::wData wiData;
  weatherAvg avg[2];
  bool bFirstWData = false;
  pI2cZero->begin(PinBaroSDA,PinBaroSCL,200000u); //init i2c
  Weather weather;
  weather.setTempOffset(setting.wd.tempOffset);
  weather.setWindDirOffset(setting.wd.windDirOffset);
  if (!weather.begin(pI2cZero,setting,PinOneWire,PinWindDir,PinWindSpeed,PinRainGauge,setting.wd.frequency)){
  
  }
  if ((!setting.wd.mode.bits.enable) && (!status.bWUBroadCast)){
    log_i("stopping task");
    vTaskDelete(xHandleWeather);
    return;    
  }
  //const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;   //only every 1sek.
  //const TickType_t xDelay = 10 / portTICK_PERIOD_MS;   //only every 10ms.
  //TickType_t xLastWakeTime = xTaskGetTickCount (); //get actual tick-count
  tUploadData = millis();
  tSendData = millis();
  tLastWindSpeed = millis();
  tWindOk = millis();
  status.weather.error.value = 0;
  status.weather.error.bits.notStarted = true;
  getRTC();
  tGetRTCTime = millis();
  while (1){
    uint32_t tAct = millis();
    if (timeOver(tAct,tGetRTCTime,10000)){
      tGetRTCTime = tAct;
      getRTCTime();
    }
    if (setting.wd.mode.bits.enable){
      //station has BME --> we are a weather-station
      weather.run();
      if (weather.getValues(&wData)){
        //log_i("wdata:wDir=%f;wSpeed=%f,temp=%f,h=%f,p=%f",wData.WindDir,wData.WindSpeed,wData.temp,wData.Humidity,wData.Pressure);
        //if ((status.weather.vaneVAlue != wData.vaneValue) || (wData.vaneValue < 1023)){ //we check, if analog-value is changing, when there is an error, we get always 1023 for analog-read
        #ifdef SEND_RAW_WIND_DATA
        sendRawWeatherData(&wData);
        #endif

        if ((wData.vaneValue != 0x03FF) || (setting.wd.anemometer.AnemometerType != eAnemometer::DAVIS)){ //0x03FF=1023 we check, if analog-value is changing, when there is an error, we get always 1023 for analog-read
          tWindOk = tAct;
          status.weather.error.bits.VanevalueInvalid = false;
          status.weather.error.bits.notStarted = false; //we got the first valid value
        }
        status.weather.vaneVAlue =  wData.vaneValue;
        if (timeOver(tAct,tWindOk,3600000uL)) { //when more than one hour we get always the same value --> problem with wind-sensor
          status.weather.error.bits.VanevalueInvalid = true;
        }
        if (wData.WindSpeed > 0.01){
          tLastWindSpeed = tAct;
        }
        if (!bFirstWData){
          for (int i = 0;i <2; i++){
            avg[i].sinWinddir = sin(wData.WindDir * DEG2RAD);
            avg[i].cosWinddir = cos(wData.WindDir * DEG2RAD);
            avg[i].WindSpeed = wData.WindSpeed;
            avg[i].Humidity = wData.Humidity;
            avg[i].Pressure = wData.Pressure;
            avg[i].temp = wData.temp;
          }
          bFirstWData = true;
        }
        if ((tAct - tLastWindSpeed) > 10000){
          bFirstWData = false; //when we have not wind for at least 10 seconds, we initialize the average
        }
        //wuData.winddir calcExpAvgf
        float fAvg = 0;
        for (int i = 0;i <2; i++){
          if (i == 0){
            fAvg = setting.wd.avgFactorFanet;
          }else{
            fAvg = setting.wd.avgFactorWU;
          }
          //log_i("fAvg%d=%f",i,fAvg);
          avg[i].sinWinddir = calcExpAvgf(avg[i].sinWinddir,sin(wData.WindDir * DEG2RAD),fAvg);
          avg[i].cosWinddir = calcExpAvgf(avg[i].cosWinddir, cos(wData.WindDir * DEG2RAD),fAvg);
          //log_i("%d,cos=%f,sin=%f",i,avg[i].sinWinddir,avg[i].cosWinddir);
          avg[i].Winddir = atan2(avg[i].sinWinddir,avg[i].cosWinddir) * RAD2DEG; //get deg back vom sin and cos-value
          //log_i("wdir%d=%f",i,avg[i].Winddir);
          while (avg[i].Winddir < 0){
            avg[i].Winddir += 360;
          }
          while (avg[i].Winddir > 360){
            avg[i].Winddir -= 360;
          }
          avg[i].WindSpeed = calcExpAvgf(avg[i].WindSpeed,wData.WindSpeed,fAvg); 
          if (wData.WindSpeed > avg[i].WindGust) avg[i].WindGust = wData.WindSpeed;
          if (wData.bHumidity) avg[i].Humidity = calcExpAvgf(avg[i].Humidity,wData.Humidity,fAvg);
          if (wData.bPressure) avg[i].Pressure = calcExpAvgf(avg[i].Pressure,wData.Pressure,fAvg);
          if (wData.bTemp) avg[i].temp = calcExpAvgf(avg[i].temp,wData.temp,fAvg);
        }
        //log_i("wDir=%f,wDir0=%f,wDir1=%f",wData.WindDir,avg[0].Winddir,avg[1].Winddir);
        status.weather.bRain = setting.wd.mode.bits.rainSensor;
        if (setting.wd.mode.bits.rainSensor){
          status.weather.rain1h = wData.rain1h;
          status.weather.rain1d = wData.rain1d;
        }else{
          status.weather.rain1h = 0;
          status.weather.rain1d = 0;
        }
        status.weather.bTemp = wData.bTemp;
        status.weather.temp = avg[0].temp;
        status.weather.bHumidity = wData.bHumidity;
        status.weather.Humidity = avg[0].Humidity;
        status.weather.bPressure = wData.bPressure;
        status.weather.Pressure = avg[0].Pressure;
        status.weather.WindDir = avg[0].Winddir;
        status.weather.WindSpeed = avg[0].WindSpeed; //we use the Fanet-Weather-Speed
        status.weather.WindGust = avg[0].WindGust; //we use the Fanet-Weather-Gust
      }
      if (timeOver(tAct,tUploadData,setting.wd.WUUploadIntervall)){
        tUploadData = tAct;
        if ((status.bInternetConnected) && (status.bTimeOk) && (status.weather.error.value == 0)){
          if (setting.WUUpload.enable){
            WeatherUnderground wu;
            #ifdef GSM_MODULE
              if (setting.wifi.connect == eWifiMode::CONNECT_NONE){
                wu.setClient(&GsmWUClient);
                wu.setMutex(&xGsmMutex);
              }
            #endif
            wuData.bWind = true;
            wuData.winddir = avg[1].Winddir;
            wuData.windspeed = avg[1].WindSpeed;
            wuData.windgust = avg[1].WindGust;
            wuData.bHum = status.weather.bHumidity;
            wuData.humidity = avg[1].Humidity;
            wuData.bTemp = status.weather.bTemp;
            wuData.temp = avg[1].temp;
            wuData.bPress = status.weather.bPressure;
            wuData.pressure = avg[1].Pressure;
            wuData.bRain = setting.wd.mode.bits.rainSensor;
            wuData.rain1h = wData.rain1h ;
            wuData.raindaily = wData.rain1d;
            //log_i("wuData:wDir=%f;wSpeed=%f,gust=%f,temp=%f,h=%f,p=%f",wuData.winddir,wuData.windspeed,wuData.windgust,wuData.temp,wuData.humidity,wuData.pressure);
            wu.sendData(setting.WUUpload.ID,setting.WUUpload.KEY,&wuData);
          }
          if (setting.WindyUpload.enable){
            Windy wi;
            #ifdef GSM_MODULE
              if (setting.wifi.connect == eWifiMode::CONNECT_NONE){
                wi.setClient(&GsmWUClient);
                wi.setMutex(&xGsmMutex);
              }
            #endif
            //log_i("temp=%f,humidity=%f",testWeatherData.temp,testWeatherData.Humidity);
            wiData.bWind = true;
            wiData.winddir = avg[1].Winddir;
            wiData.windspeed = avg[1].WindSpeed;
            wiData.windgust = avg[1].WindGust;
            wiData.bHum = status.weather.bHumidity;
            wiData.humidity = avg[1].Humidity;
            wiData.bTemp = status.weather.bTemp;
            wiData.temp = avg[1].temp;
            wiData.bPress = status.weather.bPressure;
            wiData.pressure = avg[1].Pressure;
            wiData.bRain = setting.wd.mode.bits.rainSensor;
            wiData.rain1h = wData.rain1h ;
            wiData.raindaily = wData.rain1d;
            //log_i("send weather-data to windy");
            wi.sendData(setting.WindyUpload.ID,setting.WindyUpload.KEY,&wiData);
          }

        }
        //weather.resetWindGust();
        avg[1].WindGust = 0;
      }
      
      if (timeOver(tAct,tSendData,setting.wd.FanetUploadInterval)){
        //print weather-data to serial
        //log_i("send Fanet-weatherData");
        if (timeStatus() == timeSet){
          StaticJsonDocument<500> doc;                      //Memory pool
          char buff[30];
          static char msg_buf[500];
          pWd = &msg_buf[0];
          snprintf (buff,sizeof(buff)-1,"%04d-%02d-%02dT%02d:%02d:%02d+00:00",year(),month(),day(),hour(),minute(),second()); // ISO 8601
          doc["DT"] = buff;
          doc["wDir"] = round2(avg[0].Winddir);
          doc["vaneValue"] = status.weather.vaneVAlue;
          doc["error"] = status.weather.error.value;
          doc["wSpeed"] = round2(avg[0].WindSpeed);
          doc["wGust"] = round2(avg[0].WindGust);
          if (wData.bTemp) doc["temp"] = round2(avg[0].temp);
          if (wData.bHumidity) doc["hum"] = round2(avg[0].Humidity);
          if (wData.bPressure) doc["press"] = round2(avg[0].Pressure);
          serializeJson(doc, msg_buf);
          //Serial.print("WD=");Serial.println(pWd);
          wdCount++;
        }
        if ((setting.wd.sendFanet) && (status.weather.error.value == 0)){
          
          fanetWeatherData.lat = setting.gs.lat;
          fanetWeatherData.lon = setting.gs.lon;
          fanetWeatherData.bWind = true;
          fanetWeatherData.wHeading = avg[0].Winddir;
          fanetWeatherData.wSpeed = avg[0].WindSpeed;
          fanetWeatherData.wGust = avg[0].WindGust;      
          fanetWeatherData.bTemp = wData.bTemp;
          fanetWeatherData.bHumidity = wData.bHumidity;
          fanetWeatherData.bBaro = wData.bPressure;
          fanetWeatherData.temp = avg[0].temp;
          fanetWeatherData.Humidity = avg[0].Humidity;
          fanetWeatherData.Baro = avg[0].Pressure;      
          fanetWeatherData.bStateOfCharge = true;
          fanetWeatherData.Charge = status.battery.percent;
          //testWeatherData.Charge = 44;
          sendWeatherData = true;
        }
        avg[0].WindGust = 0;
        tSendData = tAct;
      }
    }
    if (status.bWUBroadCast){
      //station should broadcast WU-Data over Fanet
      if (timeOver(tAct,tUploadData,setting.wd.WUUploadIntervall)){ //get Data from WU        
        tUploadData = tAct;
        if (status.bInternetConnected){
          WeatherUnderground::wData wuData;
          WeatherUnderground wu;
          #ifdef GSM_MODULE
            if (setting.wifi.connect == eWifiMode::CONNECT_NONE){
              wu.setClient(&GsmWUClient);
              wu.setMutex(&xGsmMutex);
            }
          #endif
          bDataOk = wu.getData(setting.WUUpload.ID,setting.WUUpload.KEY,&wuData);
          if (bDataOk){
            status.weather.bTemp = wuData.bTemp;
            status.weather.temp = wuData.temp;
            status.weather.bHumidity = wuData.bHum;
            status.weather.Humidity = wuData.humidity;
            status.weather.bPressure = wuData.bPress;
            status.weather.Pressure = wuData.pressure;            
            status.weather.WindDir = wuData.winddir;
            status.weather.WindSpeed = wuData.windspeed;
            status.weather.WindGust = wuData.windgust;
            status.weather.bRain = wuData.bRain;
            status.weather.rain1h = wuData.rain1h;
            status.weather.rain1d = wuData.raindaily;
            fanetWeatherData.lat = wuData.lat;
            fanetWeatherData.lon = wuData.lon;
            fanetWeatherData.bWind = true;
            fanetWeatherData.wHeading = wuData.winddir;
            fanetWeatherData.wSpeed = wuData.windspeed;
            fanetWeatherData.wGust = wuData.windgust;      
            fanetWeatherData.bTemp = true;
            fanetWeatherData.bHumidity = true;
            fanetWeatherData.bBaro = true;
            fanetWeatherData.temp = wuData.temp;
            fanetWeatherData.Humidity = wuData.humidity;
            fanetWeatherData.Baro = wuData.pressure;      
            fanetWeatherData.bStateOfCharge = true;  
            log_i("winddir=%.1f speed=%.1f gust=%.1f temp=%.1f hum=%.1f press=%.1f",fanetWeatherData.wHeading,fanetWeatherData.wSpeed,fanetWeatherData.wGust,fanetWeatherData.temp,fanetWeatherData.Humidity,fanetWeatherData.Baro);          
          }else{
            log_e("no Data from WU");
          }
        }
      }
      if (timeOver(tAct,tSendData,setting.wd.FanetUploadInterval)){
        tSendData = tAct;
        if (bDataOk){
          fanetWeatherData.Charge = status.battery.percent;
          sendWeatherData = true;
        }
      }

    }
    if ((WebUpdateRunning) || (bPowerOff)) break;
    //vTaskDelayUntil( &xLastWakeTime, xDelay); //wait until next cycle
    delay(100);
  }
  log_i("stop task");
  vTaskDelete(xHandleWeather); //delete weather-task
}
#endif

#ifdef AIRMODULE
void taskBaro(void *pvParameters){
  uint32_t tMax = 0;
  uint32_t tStart;
  uint32_t tCycle;
  uint8_t u8Volume = setting.vario.volume;
  static bool bInitCalib = true;
  log_i("starting baro-task ");  
  //TickType_t xLastWakeTime;
  // Block for 500ms.
  //const TickType_t xDelay = 10 / portTICK_PERIOD_MS;  
  status.vario.bHasVario = false;    
  status.vario.bHasMPU = false;

  ledcSetup(channel, freq, resolution);
  if (PinBuzzer >= 0){
    pinMode(PinBuzzer, OUTPUT);
    digitalWrite(PinBuzzer,LOW);
    ledcAttachPin(PinBuzzer, channel);
  }
  baro.useMPU(setting.vario.useMPU);
  #ifdef S3CORE
  uint8_t baroSensor = baro.begin(pI2cOne,&xI2C1Mutex);
  #else
  pI2cZero->begin(PinBaroSDA,PinBaroSCL,400000u); //init i2c
  uint8_t baroSensor = baro.begin(pI2cZero,&xI2C0Mutex);
  #endif
  baro.setKalmanSettings(setting.vario.sigmaP,setting.vario.sigmaA);
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
  }
  //status.vario.bHasVario = false; //only for testing
  if (status.vario.bHasVario){
    //xLastWakeTime = xTaskGetTickCount();
    while (1){      
      tStart = millis();
      if (command.CalibGyro == 1){
        baro.calibGyro();
        command.CalibGyro = 2;
        delay(2000);
        esp_restart();
      }
      if (command.CalibAcc == 1){
        status.calibAccStat = 0;
        command.CalibAcc = 10;
        bInitCalib = true;
      }
      if (command.CalibAcc == 11){
        baro.calibrate(bInitCalib,&status.calibAccStat);
        bInitCalib = false;
        if (status.calibAccStat == 0x3F){ //all sides calibratet --> ready !!           
          command.CalibAcc = 2; //calibrating finished --> reboot
          delay(2000);
          esp_restart();
        }
        command.CalibAcc = 10;
      }
        /*
        if (baro.calibAcc()){
          command.CalibAcc = 2;
          delay(2000);
          esp_restart();
        }else{
          command.CalibAcc = 255; //error
        } 
        */       
      //}
      
      if (command.CalibAcc > 0){
        delay(1);
        continue;
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
        baro.getValues(&status.vario.pressure,&status.vario.alt,&status.vario.ClimbRate,&status.vario.temp);
        status.vario.temp = status.vario.temp + setting.vario.tempOffset;
        status.vario.Heading = baro.getHeading();
        #ifdef USE_BEEPER
        Beeper.setVelocity(status.vario.ClimbRate);
        #endif
      }
      #ifdef USE_BEEPER
        Beeper.update();
      #endif      
      //delay(10);
      // Wait for the next cycle.
      //vTaskDelayUntil( &xLastWakeTime, xDelay );
      tCycle = millis() - tStart;
      if (tCycle >= tMax){
        tMax = tCycle;
      }
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
  //log_i("Resetting WDT...");
  //esp_task_wdt_reset();  
  //delay(10000);
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

#ifdef BLUETOOTH

void stopBluetooth(void){
  log_i("stop bluetooth-controller");
  //btStop();
  esp_bluedroid_disable();
  esp_bluedroid_deinit();
  esp_bt_controller_disable();
  esp_bt_controller_deinit();
  esp_bt_mem_release(ESP_BT_MODE_BTDM); 
  btStop(); 
}

void taskBluetooth(void *pvParameters) {
  log_i("start bluetooth task");
  // BLEServer *pServer;
  //Put a 10 second delay before Seral BT start to allow settings to work.
  if ((setting.outputMode == eOutput::oBLUETOOTH)) delay(10000);
  //esp_coex_preference_set(ESP_COEX_PREFER_BT);
  while(host_name.length() == 0){
    delay(100); //wait until we have the devid
  }
  if ((setting.outputMode == eOutput::oBLE) || (setting.outputMode == eOutput::oBLUETOOTH)){

  }else{
    //stop bluetooth-controller --> save some memory
    //stopBluetooth();
    log_i("stop task");
    vTaskDelete(xHandleBluetooth);
    return;    
  }

  if (setting.outputMode == eOutput::oBLE){
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);    
    //log_i("currHeap:%d,minHeap:%d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());    

    ble_queue = xQueueCreate(3, sizeof(ble_data));
    RxBleQueue = xQueueCreate(5, sizeof(ble_data));

    start_ble(host_name+"-LE");
    status.bluetoothStat = 1;
    while (1)
    {
      // only send if we have more than 31k free heap space.
      if (xPortGetFreeHeapSize()>BLE_LOW_HEAP){
        ble_data data;
        BaseType_t status = xQueueReceive(ble_queue, &data, pdMS_TO_TICKS(50));
        if (status == pdTRUE) {
          BLESendChunks(data.data, data.size);
        }
      }
      else
      {
        log_w( " BLE congested - Waiting - Current free heap: %d, minimum ever free heap: %d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
        log_w("CLEARING ble_queue");
        xQueueReset(ble_queue);
        delay(1);
      }
      if ((WebUpdateRunning) || (bPowerOff)){
        stop_ble();
        break;
      } 
    }
  }else if (setting.outputMode == eOutput::oBLUETOOTH){
    status.bluetoothStat = 1; //client disconnected
    while (1)
    {
      delay(1);
      if ((WebUpdateRunning) || (bPowerOff)){
        break;
      } 
    }
  }
  log_i("stop task"); 
  vTaskDelete(xHandleBluetooth); //delete bluetooth task
}
#endif


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
  if (PinADCVoltage < 0){
    return 0.0; //not voltage-pin defined !!
  }

  analogRead(PinADCVoltage); // First measurement has the biggest difference on my board, this line just skips the first measurement
  for (int i = 0; i < NO_OF_SAMPLES; i++) {
    uint16_t thisReading = analogRead(PinADCVoltage);
    adc_reading += thisReading;
  }
  adc_reading /= NO_OF_SAMPLES;

  vBatt = (adcVoltageMultiplier * float(adc_reading) / 1023.0f * 3.3f) + setting.BattVoltOffs;
  //Serial.println(adc_reading);
  //log_i("adc=%d, vADC=%.3f, vBatt=%.3f",adc_reading,float(adc_reading) / 1023.0f * 3.3f,vBatt);
  return vBatt;
}

bool printBattVoltage(uint32_t tAct){
  static uint32_t tBatt = millis() - 5000;
  if ((tAct - tBatt) >= 5000){
    tBatt = tAct;
    if (status.PMU != ePMU::NOPMU){
      //log_i("read Batt-voltage");
      xSemaphoreTake( *PMUMutex, portMAX_DELAY );
      status.battery.charging = PMU->isCharging();
      status.battery.voltage = PMU->getBattVoltage();
      xSemaphoreGive( *PMUMutex );
    }else{
      status.battery.voltage = uint16_t(readBattvoltage()*1000);      
    }
    //log_i("Batt =%dV",status.battery.voltage);
    status.battery.percent = scale(status.battery.voltage,battEmpty,battFull,0,100);
    //log_i("Batt =%d%%",status.battery.percent);
    //log_i("Batt %dV; %d%%",status.battery.voltage,status.battery.percent);

    /*
    if (status.battery.voltage >= battFull){
      log_i("Battery Full: %d mV Temp:%s C",status.battery.voltage,String(status.vario.temp,1));
        
        // if (!status.bMuting){
        //   ledcWriteTone(channel,2000);
        //   delay(500);
        // }
    }
    */
    return true; //bat-status readed
  }else{
    return false;
  }
}

void setWifi(bool on){
  if ((on) && (!status.bWifiOn)){
    log_i("switch WIFI ON");
    if (setting.CPUFrequency <  80){ //set to min. 80Mhz, because of Wifi
      setCpuFrequencyMhz(uint32_t(80));
    }else{
      setCpuFrequencyMhz(uint32_t(setting.CPUFrequency));
    }
    log_i("set CPU-Speed to %dMHz",getCpuFrequencyMhz());
    delay(10); //wait 10ms
    WiFi.config(IPADDR_ANY, IPADDR_ANY, IPADDR_ANY,IPADDR_ANY,IPADDR_ANY); // call is only a workaround for bug in WiFi class
    WiFi.disconnect(true,true);
    WiFi.mode(WIFI_MODE_NULL);
    WiFi.persistent(false);

    //now configure access-point
    //so we have wifi connect and access-point at same time
    //we connecto to wifi
    if (setting.wifi.connect != eWifiMode::CONNECT_NONE){
      //esp_wifi_set_auto_connect(true);
      log_i("Try to connect to WiFi ");
      WiFi.status();
      if (setting.wifi.uMode.bits.switchOffApWhenStaConnected){
        WiFi.mode(WIFI_MODE_STA);
      }else{
        setupSoftAp();
        WiFi.mode(WIFI_MODE_APSTA);
      }    
      if ((WiFi.SSID() != setting.wifi.ssid || WiFi.psk() != setting.wifi.password)){
        // ... Try to connect to WiFi station.
        WiFi.begin(setting.wifi.ssid.c_str(), setting.wifi.password.c_str());
      } else {
        // ... Begin with sdk config.
        WiFi.begin();
      }
      uint32_t tStart = millis();    
      while(WiFi.status() != WL_CONNECTED){
        if (timeOver(millis(),tStart,60000)){ //wait max. 60seconds
          break;
        }
        //delay(1000);
        //yield();
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
      if (WiFi.status() == WL_CONNECTED){
        log_i("wifi connected");
      }else{
        log_i("wifi not connected --> switch on AP");
        WiFi.disconnect();
        WiFi.mode(WIFI_MODE_AP); //start AP
        setupSoftAp();
      }
    }else{
      WiFi.mode(WIFI_MODE_AP);
      setupSoftAp();
    }
    log_i("hostname=%s",host_name.c_str());  
    log_i("my APIP=%s",local_IP.toString().c_str());
    if((WiFi.getMode() & WIFI_MODE_STA)){
      if (setting.outputMode != eOutput::oBLE){
        WiFi.setSleep(false); //disable power-save-mode !! will increase ping-time
        WiFi.setTxPower(WIFI_POWER_19_5dBm); //maximum wifi-power
      }
    }
    status.bWifiOn = true; 
    log_i("switch WIFI ON ready !");
    Web_setup(); 
  }
  if ((!on) && (status.bWifiOn)){
    Web_stop();
    log_i("switch WIFI OFF");
    setting.wifi.tWifiStop=0;
    //WiFi.softAPdisconnect(true);
    WiFi.disconnect(true,true);
    WiFi.mode(WIFI_MODE_NULL);
    //adc_power_release();
    status.bWifiOn = false;
    log_i("switch WIFI OFF ready !");
    
  }
  if (!status.bWifiOn){
    if ((setting.outputMode == eOutput::oBLE) && (setting.CPUFrequency <  80)){ //output over ble-connection and frequency < 80Mhz --> set to 80Mhz min.
      setCpuFrequencyMhz(uint32_t(80)); //minimum 80Mhz if bluetooth-output is on
    }else{
      setCpuFrequencyMhz(uint32_t(setting.CPUFrequency));
    }
    log_i("set CPU-Speed to %dMHz",getCpuFrequencyMhz());    
  }
  wifiCMD = 0;
}

size_t getNextString(char *ch_str,char *pSearch,char *buffer, size_t sizeBuffer){
  char *pChar = strstr(ch_str,pSearch);

  if (pChar != NULL){
    if (pChar > ch_str){
      size_t count = pChar - ch_str;
      if (count > sizeBuffer){
        strncpy(buffer,ch_str,sizeBuffer-1);
        buffer[sizeBuffer-1] = 0; //zero-termination
      }else{
        strncpy(buffer,ch_str,count);
      }      
      return count;
    }else{
      return 0;
    }
  }else{
    return 0;
  }
  
}

void checkSystemCmd(const char *ch_str){
	/* remove \r\n and any spaces */
  //log_i("system CMD=%s",ch_str);
  //Serial.printf("%s\n,",ch_str);
	String line(ch_str);
  //log_i("systemcmd=%s",line.c_str());
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
    return;
  }
  if (line.indexOf("#SYC /?") >= 0){
    add2OutputString("VER\r\nNAME\r\nTYPE\r\nAIRMODE\r\nMODE\r\nOUTMODE\r\n");
    return;
  }
  if (line.indexOf("#SYC VER?") >= 0){
    //log_i("sending 2 client");
    add2OutputString("#SYC VER=" VERSION "\r\n");
    return;
  }
  if (line.indexOf("#SYC NAME?") >= 0){
    //log_i("sending 2 client");
    add2OutputString("#SYC NAME=" + setting.PilotName + "\r\n");
    return;
  }
  iPos = getStringValue(line,"#SYC NAME=","\r",0,&sRet);
  if (iPos >= 0){
    if (setting.PilotName != sRet){
      setting.PilotName = sRet;
      fanet.setPilotname(setting.PilotName);
      write_PilotName();      
    }

    add2OutputString("#SYC OK\r\n");
    return;
  }
  if (line.indexOf("#SYC TYPE?") >= 0){
    //log_i("sending 2 client");
    add2OutputString("#SYC TYPE=" + String(setting.AircraftType) + "\r\n");
    return;
  }
  iPos = getStringValue(line,"#SYC TYPE=","\r",0,&sRet);
  if (iPos >= 0){
    uint8_t u8 = atoi(sRet.c_str());
    u8 = constrain(u8,0,7);
    if (u8 != (uint8_t)setting.AircraftType){
      setting.AircraftType = u8;
      fanet.setAircraftType(FanetLora::aircraft_t(setting.AircraftType));
      write_AircraftType();
    }
    add2OutputString("#SYC OK\r\n");
    return;
  }
  if (line.indexOf("#SYC Wifi?") >= 0){
    char msg_buf[500];
    StaticJsonDocument<500> doc;
    doc.clear();
    doc["appw"] = setting.wifi.appw;
    doc["wificonnect"] = (uint8_t)setting.wifi.connect;
    doc["WIFI_MODE"] = setting.wifi.uMode.mode;
    doc["ssid"] = setting.wifi.ssid;
    doc["password"] = setting.wifi.password;
    doc["wifioff"] = setting.wifi.tWifiStop;
    serializeJson(doc, msg_buf);
    add2OutputString(String(msg_buf));
    return;
  }
  iPos = getStringValue(line,"#SYC WIFI_MODE=","\r",0,&sRet);
  if (iPos >= 0){
    uint8_t u8 = atoi(sRet.c_str());
    if (u8 != (uint8_t)setting.wifi.uMode.mode){
      setting.wifi.uMode.mode = u8;
      write_wifiModeBits();
      Serial.println("WIFI_MODE changed --> need restart");
      status.restart.doRestart = true;
    }
    add2OutputString("#SYC OK\r\n");
    return;
  }
  if (line.indexOf("#SYC AIRMODE?") >= 0){
    //log_i("sending 2 client");
    add2OutputString("#SYC AIRMODE=" + String(setting.fanetMode) + "\r\n");
    return;
  }
  iPos = getStringValue(line,"#SYC AIRMODE=","\r",0,&sRet);
  if (iPos >= 0){
    uint8_t u8 = atoi(sRet.c_str());
    u8 = constrain(u8,0,1);
    if (u8 != setting.fanetMode){
      setting.fanetMode = eFnMode(u8);
      write_AirMode();
    }
    add2OutputString("#SYC OK\r\n");
    return;
  }
  if (line.indexOf("#SYC FUEL_SENSOR?") >= 0){
    //log_i("sending 2 client");
    add2OutputString("#SYC FUEL_SENSOR=" + String((uint8_t)setting.bHasFuelSensor) + "\r\n");
    return;
  }
  iPos = getStringValue(line,"#SYC FUEL_SENSOR=","\r",0,&sRet);
  if (iPos >= 0){
    uint8_t u8 = atoi(sRet.c_str());
    u8 = constrain(u8,0,1);
    if (u8 != (uint8_t)setting.bHasFuelSensor){
      if (u8 == 1){
        setting.bHasFuelSensor = true;
      }else{
        setting.bHasFuelSensor = false;
      }      
      write_fuelsensor();
      Serial.println("FUEL_SENSOR changed --> need restart");
      status.restart.doRestart = true;
    }
    add2OutputString("#SYC OK\r\n");
    return;
  }
  if (line.indexOf("#SYC FCPU?") >= 0){
    add2OutputString("#SYC FCPU=" + String(setting.CPUFrequency) + "\r\n");
    return;
  }
  iPos = getStringValue(line,"#SYC FCPU=","\r",0,&sRet);
  if (iPos >= 0){
    uint8_t u8 = atoi(sRet.c_str());
    u8 = constrain(u8,20,240);
    add2OutputString("#SYC OK\r\n");
    if (u8 != setting.CPUFrequency){
      setting.CPUFrequency = u8;
      log_i("set CPU-frequency to %dMhz",setting.CPUFrequency);
      write_CPUFrequency();
      Serial.println("CPU-frequency changed --> need restart");
      status.restart.doRestart = true;
    }
    return;
  }
  if (line.indexOf("#SYC MODE?") >= 0){
    //log_i("sending 2 client");
    add2OutputString("#SYC MODE=" + String(setting.Mode) + "\r\n");
    return;
  }
  iPos = getStringValue(line,"#SYC MODE=","\r",0,&sRet);
  if (iPos >= 0){
    uint8_t u8 = atoi(sRet.c_str());
    u8 = constrain(u8,0,1);
    add2OutputString("#SYC OK\r\n");
    if (u8 != setting.Mode){
      setting.Mode = eMode(u8);
      write_Mode();
      Serial.println("MODE changed --> need restart");
      status.restart.doRestart = true;
    }
    return;
  }
  if (line.indexOf("#SYC RESTART") >= 0){
      Serial.println("restart received");
      status.restart.doRestart = true;
      return;
  }
  if (line.indexOf("#SYC OUTMODE?") >= 0){
    //log_i("sending 2 client");
    add2OutputString("#SYC OUTMODE=" + String(setting.outputMode) + "\r\n");
    return;
  }
  iPos = getStringValue(line,"#SYC OUTMODE=","\r",0,&sRet);
  if (iPos >= 0){
    uint8_t u8 = atoi(sRet.c_str());
    u8 = constrain(u8,0,3);
    add2OutputString("#SYC OK\r\n");
    if (u8 != setting.outputMode){
      setting.outputMode = eOutput(u8);
      write_OutputMode();
      Serial.println("OUTMODE changed --> need to restart");
      status.restart.doRestart = true;
    }
    return;
  }
  if (line.indexOf("#SYC NETSTAT?") >= 0){
    //log_i("sending 2 client");
    add2OutputString("#SYC INET=" + String(status.bInternetConnected) + ",MODEM=" + String(status.modemstatus) + ",MQTT=" + String(status.MqttStat) + "\r\n");
    return;
  }
  if (line.indexOf("#SYC RFMODE?") >= 0){
    //log_i("sending 2 client");
    add2OutputString("#SYC RFMODE=" + String(setting.RFMode) + "\r\n");
    return;
  }
  iPos = getStringValue(line,"#SYC RFMODE=","\r",0,&sRet);
  if (iPos >= 0){
    uint8_t u8 = atoi(sRet.c_str());
    u8 = constrain(u8,0,15);
    add2OutputString("#SYC OK\r\n");
    if (u8 != setting.RFMode){
      setting.RFMode = u8;
      write_RFMode();
      Serial.println("RF-Mode changed --> need to restart");
      status.restart.doRestart = true;
    }
    return;
  }
  if (line.indexOf("#SYC DOUPDATE") >= 0){
    status.updateState = 50; //check for Update automatic
    add2OutputString("Check for update\r\n");
    return;
  }
  iPos = getStringValue(line,"#SYC UPDATE=","\r",0,&sRet);
  if (iPos >= 0){
    status.sNewVersion = sRet;
    status.updateState = 60; //check for Update automatic
    add2OutputString("#SYC UPDATE NEW VERSION\r\n");
    return;
  }
  //Query BattVoltage
  if (line.indexOf("#SYC VBAT?") >= 0){
    float vbat = status.battery.voltage / 1000.0;
    add2OutputString("#SYC VBAT=" + String(vbat, 2) + "\r\n");
    return;
  }

  #ifdef GSMODULE  
  //Query Rain values
  if (line.indexOf("#SYC RAIN?") >= 0){
    add2OutputString("#SYC RAIN1H=" + String(status.weather.rain1h, 2) + ",RAIN1D=" + String(status.weather.rain1d, 2) + "\r\n");
    return;
  }
  //Query and set FanetUploadInterval
  if (line.indexOf("#SYC FANETWDINT?") >= 0){
    // Return the interval in seconds, not milliseconds
    add2OutputString("#SYC FANETWDINT=" + String(setting.wd.FanetUploadInterval / 1000) + "\r\n");
    return;
  }
  iPos = getStringValue(line,"#SYC FANETWDINT=","\r",0,&sRet);
  if (iPos >= 0){
    uint32_t intervalSec = atol(sRet.c_str());
    intervalSec = constrain(intervalSec, 10, 600); // Accept 10–600 seconds
    uint32_t intervalMs = intervalSec * 1000;
    add2OutputString("#SYC OK\r\n");
    if (intervalMs != setting.wd.FanetUploadInterval){
      setting.wd.FanetUploadInterval = intervalMs;
      write_FanetUploadInterval();
      Serial.println("FANETWDINT changed --> need to restart");
      status.restart.doRestart = true;
    }
    return;
  }
  //Query and set windDirOffset
  if (line.indexOf("#SYC WINDOFFSET?") >= 0){
    add2OutputString("#SYC WINDOFFSET=" + String(setting.wd.windDirOffset) + "\r\n");
    return;
  }
  iPos = getStringValue(line, "#SYC WINDOFFSET=", "\r", 0, &sRet);
  if (iPos >= 0){
    int16_t offset = atoi(sRet.c_str());
    offset = constrain(offset, -180, 180); // wind dir offset should be in -180° to +180°
    add2OutputString("#SYC OK\r\n");
    if (offset != setting.wd.windDirOffset){
      setting.wd.windDirOffset = offset;
      write_windDirOffset();
      Serial.println("WINDOFFSET changed --> need to restart");
      status.restart.doRestart = true;
    }
    return;
  }
  //Query and set RTC time
  if (line.indexOf("#SYC GETTIME?") >= 0){
    getRTCTime(); // sets system time using RTC
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
      add2OutputString("#SYC ERROR: Failed to get local time\r\n");
      return;
    }
    char buf[32];
    snprintf(buf, sizeof(buf), "#SYC TIME=%04d-%02d-%02d_%02d:%02d:%02d\r\n",
             timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    add2OutputString(String(buf));
    return;
  }
  iPos = getStringValue(line, "#SYC SETTIME=", "\r", 0, &sRet);
  if (iPos >= 0){
    struct tm timeinfo;
    memset(&timeinfo, 0, sizeof(struct tm));
    // Expect format: YYYY-MM-DD_HH:MM:SS
    if (sscanf(sRet.c_str(), "%4d-%2d-%2d_%2d:%2d:%2d",
               &timeinfo.tm_year, &timeinfo.tm_mon, &timeinfo.tm_mday,
               &timeinfo.tm_hour, &timeinfo.tm_min, &timeinfo.tm_sec) == 6) {
      timeinfo.tm_year -= 1900; // tm struct expects years since 1900
      timeinfo.tm_mon -= 1;     // tm_mon is 0-based (0=Jan)

      setRTCTime(timeinfo);
      add2OutputString("#SYC OK\r\n");
    } else {
      add2OutputString("#SYC ERROR: Invalid time format\r\n");
    }
    return;
  }
  // Query anemometer type
  if (line.indexOf("#SYC ANEMO?") >= 0){
    add2OutputString("#SYC ANEMO=" + String((uint8_t)setting.wd.anemometer.AnemometerType) + "\r\n");
    return;
  }

  // Set anemometer type
  iPos = getStringValue(line, "#SYC ANEMO=", "\r", 0, &sRet);
  if (iPos >= 0){
    uint8_t u8 = atoi(sRet.c_str());
    u8 = constrain(u8, 0, 5); // valid enum range
    add2OutputString("#SYC OK\r\n");
    if (u8 != (uint8_t)setting.wd.anemometer.AnemometerType){
      setting.wd.anemometer.AnemometerType = eAnemometer(u8);
      write_AnemometerType();
      Serial.println("ANEMO changed --> need restart");
      status.restart.doRestart = true;
    }
    return;
  }
  #endif  
  return;  
}


void checkReceivedLine(const char *ch_str){
  //log_i("new serial msg=%s",ch_str);
  if(!strncmp(ch_str, FANET_CMD_TRANSMIT, 4)){
    fanet.fanet_cmd_transmit(ch_str+4);
    return;
  }else if(!strncmp(ch_str, FANET_CMD_GROUND_TYPE, 4)){
    fanet.fanet_cmd_setGroundTrackingType(ch_str+4);
    return;
  }else if (!strncmp(ch_str,SYSTEM_CMD,4)){
    checkSystemCmd(ch_str);
    return;
  }else if (!strncmp(ch_str,GPS_STATE,2)){
    //got GPS-Info
    //log_i("GPS-Info:%s",ch_str);
    if (sNmeaIn.length() == 0){
      sNmeaIn = String(ch_str);
    }
    return;
  #ifndef GXTEST
  }else if (!strncmp(ch_str,"$PPS",4)){
    //received a PFLAG-Message --> start pps
    //log_i("trigger pps");
    status.gps.bExtGps = true;
    //ppsTriggered = true;
    return;
  #endif
  }else if (!strncmp(ch_str,"$CL,",4)){
    float climb = atof(&ch_str[4]);
    status.vario.bHasVario = true;
    status.vario.ClimbRate = climb;
    //log_i("Climb=%.1f",climb);
    return;
  }else if (!strncmp(ch_str,"$PGXCF,?",8)){
    // Handle request for configuration request
    writePGXCFSentence();
    return;
  }else if (!strncmp(ch_str,"$PFLAC,R,RADIOID",16)){
    // Handle request ID
    char buffer[MAXSTRING];
    snprintf(buffer,MAXSTRING,"$PFLAC,A,RADIOID,%d,%s", MyFanetData.addressType, fanet.getMyDevId().c_str());
    int pos = flarmDataPort.addChecksum(buffer,MAXSTRING);
    sendData2Client(buffer, pos);
    return;
  }else if (!strncmp(ch_str,"$PGXCF,",7)){
    // Handle configuration setting
    readPGXCFSentence(ch_str);
    return;
  }else{
    log_i("unknown message=%s",ch_str);
  }
}

char* readSerial(){
  static char lineBuffer[512];
  static uint16_t recBufferIndex = 0;
  
  int16_t thisChar;
  while((thisChar = Serial.read())!=-1){
    if ((recBufferIndex >= (512-1)) || thisChar=='$' || thisChar=='!') recBufferIndex = 0; //Buffer overrun, $ and ! are start of NMEA-String
    lineBuffer[recBufferIndex] = thisChar;
    if (lineBuffer[recBufferIndex] == '\n' && recBufferIndex>10){
      recBufferIndex++;
      lineBuffer[recBufferIndex] = 0; //zero-termination
      recBufferIndex = 0;
      return &lineBuffer[0];
    }
    recBufferIndex++;
  }
  return NULL;
}

#ifdef AIRMODULE
void readGPS(){
  static char lineBuffer[255];
  static uint16_t recBufferIndex = 0;
  static uint32_t tGpsOk = millis();
  
  if (sNmeaIn.length() > 0){ //String received by Bluetooth or serial, ...
    #ifdef GXTEST
    if (true){
        tGpsOk = millis();
        if (!status.gps.bHasGPS){
          status.gps.bHasGPS = true;
          fanet.setGPS(status.gps.bHasGPS);          
        }
    #else
    if (!status.gps.bHasGPS){
    #endif
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
      if ((setting.outputGPS) && (!WebUpdateRunning)) sendData2Client(cstr,sNmeaIn.length()); //sendData2Client(sNmeaIn);
      delete cstr; //delete allocated String
      sNmeaIn = "";
    }else{
      sNmeaIn = ""; //we have a gps --> don't take data from external GPS
    }
  }
  if ((setting.Mode == eMode::AIR_MODULE) || ((abs(setting.gs.lat) <= 0.1) && (abs(setting.gs.lon) <= 0.1))) {
    //in Ground-Station-Mode you have to setup GPS-Position manually
    int16_t thisChar;
    while((thisChar = NMeaSerial.read())!=-1){
      if ((recBufferIndex >= (255-2)) || thisChar=='$' || thisChar=='!') recBufferIndex = 0; //Buffer overrun, $ and ! are start of NMEA-String
      lineBuffer[recBufferIndex] = thisChar;
      //log_i("GPS %c",lineBuffer[recBufferIndex]);
      nmea.process(lineBuffer[recBufferIndex]);
      if ((lineBuffer[recBufferIndex] == '\n' || lineBuffer[recBufferIndex] == '\r') && recBufferIndex>10){ // Each message is minimum 10 characters
        lineBuffer[recBufferIndex] = '\r';
        recBufferIndex++;
        lineBuffer[recBufferIndex] = '\n';
        recBufferIndex++;
        lineBuffer[recBufferIndex] = 0; //zero-termination
        if ((setting.outputGPS) && (!WebUpdateRunning)) sendData2Client(lineBuffer,recBufferIndex);
        recBufferIndex = 0;
        tGpsOk = millis();
        if (!status.gps.bHasGPS){
          status.gps.bHasGPS = true;
          fanet.setGPS(status.gps.bHasGPS);          
        }
      }
      recBufferIndex++;
    }  
  }
  if (timeOver(millis(),tGpsOk,10000)){
    if (status.gps.bHasGPS) {      
      status.gps.bHasGPS = false;
      fanet.setGPS(status.gps.bHasGPS);
    }
  }
}
#endif

eFlarmAircraftType Fanet2FlarmAircraft(FanetLora::aircraft_t aircraft){
  if (aircraft >= 0x80) return (eFlarmAircraftType)(aircraft - 0x80);
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
  FlarmDataData->addressType = (FanetData->addressType) & 0x7F; //clear highest bit (is set for FANET-Msg)
  FlarmDataData->noTrack = !FanetData->OnlineTracking;
}

void sendLXPW(uint32_t tAct){
  if (WebUpdateRunning) return;
  // $LXWP0,logger_stored, airspeed, airaltitude,
  //   v1[0],v1[1],v1[2],v1[3],v1[4],v1[5], hdg, windspeed*CS<CR><LF>
  //
  // 0 loger_stored : [Y|N] (not used in LX1600)
  // 1 IAS [km/h] ----> Condor uses TAS!
  // 2 baroaltitude [m]
  // 3-8 vario values [m/s] (last 6 measurements in last second)
  // 9 heading of plane (not used in LX1600)
  // 10 windcourse [deg] (not used in LX1600)
  // 11 windspeed [km/h] (not used in LX1600)
  //
  // e.g.:
  // $LXWP0,Y,222.3,1665.5,1.71,,,,,,239,174,10.1
  static uint32_t tOld = millis();
  if ((tAct - tOld) >= 250){
    char sOut[MAXSTRING];
    int pos = 0;
    pos += snprintf(&sOut[pos],MAXSTRING-pos,"$LXWP0,N,");
    if (status.gps.Fix){
      pos += snprintf(&sOut[pos],MAXSTRING-pos,"%.01f",status.gps.speed);
    }
    pos += snprintf(&sOut[pos],MAXSTRING-pos,",");
    if (status.vario.bHasVario){
      pos += snprintf(&sOut[pos],MAXSTRING-pos,"%.01f,%.02f",status.vario.alt,status.vario.ClimbRate); // altitude in meters, relative to QNH 1013.25
    }else{
      pos += snprintf(&sOut[pos],MAXSTRING-pos,",");
    }
    pos += snprintf(&sOut[pos],MAXSTRING-pos,",,,,,,");
    if (status.gps.Fix){
      pos += snprintf(&sOut[pos],MAXSTRING-pos,"%.01f",status.gps.course);
    }
    pos += snprintf(&sOut[pos],MAXSTRING-pos,",,");
    pos = flarmDataPort.addChecksum(sOut,MAXSTRING);
    sendData2Client(sOut,pos);
    /*
    String s = "$LXWP0,N,";
    if (status.gps.Fix){
      s += String(status.gps.speed,1);
    }
    s += ",";
    if (status.vario.bHasVario){
      s += String(status.vario.alt,1) + "," + String(status.vario.ClimbRate,2); // altitude in meters, relative to QNH 1013.25
    }
    s+=  + ",,,,,,";
    if (status.gps.Fix){
      s += String(status.gps.course,1);
    }
    s +=  ",,";
    s = flarmDataPort.addChecksum(s);
    sendData2Client(s);
    */
    tOld = tAct;
  }
}

template<typename Tp>
static inline
Tp Clamp(Tp value, Tp min, Tp max) {
  return std::min<Tp>(std::max<Tp>(value, min), max);
}

void sendLK8EX(uint32_t tAct){
  if (WebUpdateRunning) return;
  static uint32_t tSend = millis();
  if ((timeOver(tAct,tSend,100) && status.vario.bHasVario) || //update every 100ms if baro exists
     (timeOver(tAct,tSend,1000) && !status.vario.bHasVario)) { //update every 1 second of no baro exists (only battery-state)
    //            $LK8EX1,pressure [1/100mbar],altVario [m],temp [°C],batt [percent],chksum crlf
    //String s = "$LK8EX1,101300,99999,99999,99,999,";
    char sOut[MAXSTRING];
    int pos = snprintf(sOut,MAXSTRING,"$LK8EX1,");
    if (status.vario.bHasVario){
      pos += snprintf(&sOut[pos],MAXSTRING-pos,"%d,",(uint32_t)status.vario.pressure);
      pos += snprintf(&sOut[pos],MAXSTRING-pos,"%.02f,",status.vario.alt);
      pos += snprintf(&sOut[pos],MAXSTRING-pos,"%d,%.01f,",(int32_t)roundf(status.vario.ClimbRate * 100.0),status.vario.temp);
    }else{
      // Don't send altitude nor baro data if baro sensor is not available.
      pos += snprintf(&sOut[pos],MAXSTRING-pos,"999999,99999,9999,99,");      
    }
    pos += snprintf(&sOut[pos],MAXSTRING-pos,"%u,", Clamp<uint32_t>(status.battery.percent, 0, 100) + 1000U);
    pos = flarmDataPort.addChecksum(sOut,MAXSTRING);
    sendData2Client(sOut,pos);
    tSend = tAct;
  }
}

#ifdef AIRMODULE

bool checkGPSBaudrates(void){  
  for (int i = 0; i < 2;i++) { //try 2 times with baud in settings
    if (checkGPSBaud(setting.gps.Baud)){
      log_i("gps connection ok with baud %d",setting.gps.Baud);
      return true; //found with gps-baudrate which is set in settings
    } 
  } 
  //try other baudrates
  for (int i = 0; i < commonBaudRatesSize;i++) {
    log_i("gps: Trying baudrate=%d %d/%d",commonBaudRates[i],i,commonBaudRatesSize);
    if (checkGPSBaud(commonBaudRates[i])){
      setting.gps.Baud = commonBaudRates[i];
      write_gpsBaud();
      return true;
    } 
  }  
  return false;
}

bool checkGPSBaud(uint32_t baud){
  NMeaSerial.updateBaudRate(baud);  
  while(NMeaSerial.available()) NMeaSerial.read(); //clear input-buffer
  uint32_t tStart = millis();
  while((millis() - tStart) < 1100){
    if (NMeaSerial.available()){
      if (nmea.process(NMeaSerial.read())){
        log_i("gps: baudrate with %d ok",baud);
        return true;
      }
    }      
  }
  return false;
}
bool sendCmd2NMEA(const char* s,const char* sRet){
  size_t lLen = strlen(sRet);
  char buffer[MAXSTRING];
  int pos = 0;
  pos += snprintf(&buffer[pos],MAXSTRING-pos,s);
  pos = flarmDataPort.addChecksum(buffer,MAXSTRING);
  log_i("%s",buffer);
  while(NMeaSerial.available()) NMeaSerial.read(); //clear receive buffer
  NMeaSerial.write(&buffer[0]);  
  uint32_t tstart = millis();
  //wait for response
  pos = 0;
  buffer[pos] = 0;
  char c;
  while((millis() - tstart) < 1000){
    while(NMeaSerial.available()){
      c = NMeaSerial.read();
      //Serial.printf("%c",c);
      if (c == '$') pos = 0;
      /*
      if (c == '\r'){
        Serial.println(buffer);
      }
      */
      buffer[pos] = c;
      pos++;
      buffer[pos] = 0; //zero-termination !!
      if (pos == lLen){
        if (strcmp(sRet,buffer) == 0){
          log_i("successful transmitted");
          return true;
        }
      }
    }
    delay(1);
  }
  log_e("error sending cmd to GPS");
  return false;
}

bool setupQuectelGps(void){
  log_i("************ config GPS *************");
  checkGPSBaudrates();
  NMeaSerial.begin(setting.gps.Baud,SERIAL_8N1,PinGPSRX,PinGPSTX,false); //reinit TX-Pin, cause it is used twice
  bool bOk = false;
  char sQuery[20];
  sprintf(sQuery,"$PQVERNO,R");
  flarmDataPort.addChecksum(sQuery,20);
  for (int i = 0;i < 3;i++){
    if (sendCmd2NMEA(sQuery,"$PQVERNO,R,")){ //set GPTXT to off
    //if (sendCmd2NMEA("$PQTXT,W,0,1","$PQTXT,W,OK*0A")){ //set GPTXT to off
      bOk = true;
      break;
    }
    delay(500);
  }
  if (bOk == false) return false;
  //we got response --> Quectel-chip
  sendCmd2NMEA("$PQTXT,W,0,1","$PQTXT,W,OK*0A");
  //sendCmd2NMEA("$PMTK353,1,1,1,0,0","$PMTK001,353,3,1,1,1,0,0,15*00"); //enable GPS & Glonass & Galileo
  sendCmd2NMEA("$PMTK353,1,1,0,0,0","$PMTK001,353,3"); //enable GPS & Glonass
  //sendCmd2NMEA("$PMTK353,1,0,0,0,0","$PMTK001,353,3,1,0,0,0,0,1*35"); //enable GPS
  sendCmd2NMEA("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0","$PMTK001,314,3"); //set nmea-output to GPRMC and GPGGA
  //sendCmd2NMEA("$PMTK225,0*2B","$PMTK001,225,3*35"); //power save mode normal mode

  //sendCmd2NMEA("$PMTK605*31",""); //read Firmware
  //sendCmd2NMEA("$PMTK104*37",""); //factory settings
  //sendCmd2NMEA("$PMTK251,9600",""); //set Baudrate to 9600
  log_i("update Baudrate to %d",GPSBAUDRATE);
  char chs[20];
  sprintf(chs,"$PMTK251,%d",GPSBAUDRATE);    
  sendCmd2NMEA(chs,""); //set Baudrate to 57600
  //clear serial buffer
  NMeaSerial.updateBaudRate(GPSBAUDRATE);
  setting.gps.Baud = GPSBAUDRATE;
  write_gpsBaud();
  //delay(100); 
  return true;   
}

bool setupUbloxConfig(){
  SFE_UBLOX_GNSS ublox;
  NMeaSerial.begin(setting.gps.Baud,SERIAL_8N1,PinGPSRX,PinGPSTX,false); //reinit TX-Pin, cause maybe it is used twice
  ublox.begin(NMeaSerial);
  ublox.factoryReset();

  delay(2000); //wait for hardware again !!
  for (int i = 0; i < 3; i++){
    uint8_t tryBaudPos=0;
    do {
      log_i("ublox: Trying baudRate=%d %d/%d",commonBaudRates[tryBaudPos],tryBaudPos,commonBaudRatesSize);
      NMeaSerial.updateBaudRate(commonBaudRates[tryBaudPos]);      
      delay(500);
    } while (!ublox.isConnected() && (++tryBaudPos < commonBaudRatesSize));
    if (tryBaudPos >= commonBaudRatesSize) {
      log_e("ublox: Could not connect to GPS with any baudRate");
      continue;
    }
    if (!ublox.isConnected()){
      log_e("ublox: GPS not connected");
      continue;
    }
    log_i("ublox: GPS connected at %d", commonBaudRates[tryBaudPos]);
    if (!ublox.setUART1Output(COM_TYPE_NMEA)){
      log_e("ublox: error setting uart1 output");
      continue;
    }

    bool shouldHardReset = false;
    if (!setting.gps.customGPSConfig){
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
      /*
      if (!ublox.disableNMEAMessage(UBX_NMEA_GSA,COM_PORT_UART1)){
        log_e("ublox: error setting parameter %d",UBX_NMEA_GSA);
        continue;
      }
      */
      ublox.enableNMEAMessage(UBX_NMEA_GSA,COM_PORT_UART1, 0x05, defaultMaxWait);

      //enable nmea-sentences
      if (!ublox.enableNMEAMessage(UBX_NMEA_GGA,COM_PORT_UART1)){
        log_e("ublox: error setting parameter %d",UBX_NMEA_GGA);
        continue;
      }
      if (!ublox.enableNMEAMessage(UBX_NMEA_RMC,COM_PORT_UART1)){
        log_e("ublox: error setting parameter %d",UBX_NMEA_RMC);
        continue;
      }
    } else {
      // Configure uBlox based on the attached version
      log_i("ublox: Getting version");
      uint8_t payloadCfg[MAX_PAYLOAD_SIZE];
      ubxPacket customCfg = {UBX_CLASS_MON, UBX_MON_VER, 0, 0, 0, payloadCfg, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
      if (ublox.sendCommand(&customCfg) != SFE_UBLOX_STATUS_DATA_RECEIVED){
        log_e("ublox: error getting version");
        continue;
      }

      uint8_t ubloxType = payloadCfg[33] - '0';
      log_i("ublox: Version %d detected", ubloxType);
      if (ubloxType<6) {
        ubloxType = 6;
      } else if (ubloxType>10) {
        ubloxType = 10;
      }
      log_i("ublox: Version %d detected", ubloxType);


      // Set required constellations, if it can be set it's tried, else it's ok if it fails
      log_i("ublox: Retreive constellations that are enabled");
      customCfg = {UBX_CLASS_CFG, UBX_CFG_GNSS, 0, 0, 0, payloadCfg, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
      bool constellationsReceived = true;
      sfe_ublox_status_e ret = ublox.sendCommand(&customCfg,3000);
      if (ret != SFE_UBLOX_STATUS_DATA_RECEIVED){
        log_e("ublox: error getting parameter UBX-CFG-GNSS, also not trying to set it ret=%d",ret);
        constellationsReceived=false;
      }

      if (constellationsReceived) {
        log_i("ublox: GNSS payload 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x", customCfg.cls, customCfg.id, customCfg.len, customCfg.counter, customCfg.startingSpot);
        for (uint8_t ncb = 0; ncb < payloadCfg[3]; ncb++){
          log_i("ublox: GNSS block 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x",
            payloadCfg[4+8*ncb], payloadCfg[5+8*ncb], payloadCfg[6+8*ncb], payloadCfg[7+8*ncb],
            payloadCfg[8+8*ncb], payloadCfg[9+8*ncb], payloadCfg[10+8*ncb], payloadCfg[11+8*ncb]);

            uint8_t gnssId = payloadCfg[4+8*ncb];
            if (gnssId == 0x02 && payloadCfg[8+8*ncb] == 0x00) {
              shouldHardReset = true;
            }
            // Enable GPS=0x00 SBAS=0x01 Galileo=0x02 Beidou=0x03 QZSS=0x05 Disable: Glonass=0x06
            if (gnssId == 0x00 || gnssId == 0x01 || gnssId == 0x02 || gnssId == 0x03 || gnssId == 0x05)  {
              payloadCfg[8+8*ncb] = 0x01;
            } else {
              payloadCfg[8+8*ncb] = 0x00;
            }
        }
        log_i("ublox: enable required constellations");
        if (ublox.sendCommand(&customCfg) != SFE_UBLOX_STATUS_DATA_SENT){
          log_e("ublox: error setting parameter UBX_CFG_GNSS");
        }
        delay(550);
      }


      // Configure ublox based on the attached version
      uint8_t *setNMEA=NULL;
      uint8_t lenNMEA=0;
      if (ubloxType == 6 || ubloxType == 7){
        setNMEA = setNMEA_U6_7;
        lenNMEA = sizeof(setNMEA_U6_7);
      } else {
        setNMEA = setNMEA_U8_9_10;
        lenNMEA = sizeof(setNMEA_U8_9_10);
      }

      if (setNMEA != NULL){
        log_i("ublox: setting parameter UBX-CFG-NMEA");
        customCfg = {UBX_CLASS_CFG, UBX_CFG_NMEA, lenNMEA, 0, 0, setNMEA, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
        ret = ublox.sendCommand(&customCfg);
        if (ret != SFE_UBLOX_STATUS_DATA_SENT){
          log_e("ublox: error setting parameter UBX-CFG-NMEA ret=%d",ret);
        }
      }

      // Set on best efford
      ublox.enableNMEAMessage(UBX_NMEA_GGA,COM_PORT_UART1, 0x01, defaultMaxWait);
      ublox.disableNMEAMessage(UBX_NMEA_GLL,COM_PORT_UART1);
      ublox.enableNMEAMessage(UBX_NMEA_GSA,COM_PORT_UART1, 0x05, defaultMaxWait);
      ublox.enableNMEAMessage(UBX_NMEA_GSV,COM_PORT_UART1, 0x05, defaultMaxWait);
      ublox.enableNMEAMessage(UBX_NMEA_RMC,COM_PORT_UART1, 0x01, defaultMaxWait);
      ublox.enableNMEAMessage(UBX_NMEA_VTG,COM_PORT_UART1, 0x01, defaultMaxWait);
      ublox.disableNMEAMessage(UBX_NMEA_GRS,COM_PORT_UART1);
      ublox.disableNMEAMessage(UBX_NMEA_GST,COM_PORT_UART1);
      ublox.disableNMEAMessage(UBX_NMEA_ZDA,COM_PORT_UART1);
      ublox.disableNMEAMessage(UBX_NMEA_GBS,COM_PORT_UART1);
      ublox.disableNMEAMessage(UBX_NMEA_DTM,COM_PORT_UART1);
      ublox.disableNMEAMessage(UBX_NMEA_GNS,COM_PORT_UART1);
      ublox.disableNMEAMessage(UBX_NMEA_VLW,COM_PORT_UART1);

      for (uint8_t i=0; i<sizeof(setCFG) / 8; i++) {
        log_i("ublox: setting parameter UBX-CFG-MSG 0x%02x", setCFG[i][1]);
        customCfg = {UBX_CLASS_CFG, UBX_CFG_MSG, sizeof(setCFG) / 3, 0, 0, setCFG[i], 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
        ublox.sendCommand(&customCfg);
      }

      log_i("ublox: setting parameter UBX-CFG-RATE");
      if (!ublox.setNavigationFrequency(5, defaultMaxWait)){
        log_e("ublox: error setting parameter UBX-CFG-RATE");
      }

      if (!ublox.setDynamicModel(DYN_MODEL_AIRBORNE2g)) {
          log_e("ublox: error setting parameter UBX-CFG-NAV5");
          continue;
      }

      log_i("ublox: Configured for customGPS");
    }

    // Set baudrate faster because we handle more messages
    ublox.setSerialRate(GPSBAUDRATE, COM_PORT_UART1);
    NMeaSerial.updateBaudRate(GPSBAUDRATE);
    setting.gps.Baud = GPSBAUDRATE;
    write_gpsBaud();
    delay(1500);

    if (!ublox.saveConfiguration(3000)){
      log_e("ublox: error saving config");
      continue;
    }else{
      if(shouldHardReset) {
        // log_e("ublox:Should hard reset after ebable of Galileo");
        // Not quite sure yet if this is really needed.
        // ublox.hardReset();
      }
      log_i("!!! setup ublox successfully");
      return true;
    }
  }
  return false;

}
#endif


void taskStandard(void *pvParameters){
  static uint32_t tLoop = micros();
  static float oldAlt = 0.0;
  static uint32_t tOldPPS = millis();
  static uint32_t tLastPPS = millis();
  static uint32_t tMaxLoop = millis();
  bool bBatPowerOk = false;
  bool bShowBattPower = true;
  #ifdef TEST
  static uint32_t tSend = millis();
  #endif
  userled.setUserLed(PinBeaconLed,true);
  char * pSerialLine = NULL;
  String sSerial = "";
  String s = "";
  FanetLora::trackingData tFanetData;  
  
  tFanetData.rssi = 0;
  MyFanetData.rssi = 0;
  MyFanetData.OnlineTracking = true; //this could be a setting

  GxMqtt *pMqtt = NULL;
  static uint8_t myWdCount = 0;
  #ifdef SEND_RAW_WIND_DATA
  static uint8_t myWdRawCount = 0;
  #endif
  if (setting.mqtt.mode.bits.enable){
    pMqtt = new(GxMqtt);
    #ifdef GSM_MODULE
    if (setting.wifi.connect == eWifiMode::CONNECT_NONE){
      pMqtt->begin(&xGsmMutex,&GsmMqttClient);
      //pMqtt->begin(&xGsmMutex,NULL);
    }else{
      pMqtt->begin();
    }
    #else
    pMqtt->begin();
    #endif
    
  }  

  #ifdef AIRMODULE
  if (PinGPSRX >= 0){
    NMeaSerial.begin(setting.gps.Baud,SERIAL_8N1,PinGPSRX,-1,false); //clear Tx-Pin, cause maybe it is used twice
    log_i("GPS Baud=%d,8N1,RX=%d,TX=%d",setting.gps.Baud,PinGPSRX,PinGPSTX);
    delay(2000); //wait 1 second until power is stable
    checkGPSBaudrates();
    //clear serial buffer
    //while (NMeaSerial.available())
    //  NMeaSerial.read();
    //delay(100);
    //sendCmd2NMEA("$PMTK353,1,0,1,0,0","$PMTK001,353,3,1,0,1,0,0,13*07"); //enable GPS & Galileo
  }
  
  if (PinPPS > 0){  
    //only on new boards we have an pps-pin
    log_i("setup PPS-Pin for GPS");
    pinMode(PinPPS, INPUT);
    //attachInterrupt(digitalPinToInterrupt(PinPPS), ppsHandler, FALLING);
    attachInterrupt(digitalPinToInterrupt(PinPPS), ppsHandler, RISING);
  }
  #ifdef GXTEST
    status.gps.bExtGps = true;
    fanet.setGPS(true);
    //only on new boards we have an pps-pin
    pinMode(PinPPS, INPUT);
    //attachInterrupt(digitalPinToInterrupt(PinPPS), ppsHandler, FALLING);
    attachInterrupt(digitalPinToInterrupt(PinPPS), ppsHandler, RISING);
  #endif
  #endif
  // create a binary semaphore for task synchronization
  fanet.setRFMode(setting.RFMode);
  uint8_t radioChip = RADIO_SX1276;
  if ((setting.boardType == eBoard::T_BEAM_SX1262) || (setting.boardType == eBoard::T_BEAM_S3CORE) || (setting.boardType == eBoard::HELTEC_WIRELESS_STICK_LITE_V3) || (setting.boardType == eBoard::HELTEC_LORA_V3)) radioChip = RADIO_SX1262;

  // When the requested Address type is ICAO then the devId of the device must be set to your mode-s address
  // See Flarm Dataport Specification for details
  // When devId is not set, take the HW Address (default) and this is done automatically
  fanet.setAddressType(setting.myDevIdType);
  if (setting.myDevId.length() == 6) {
    fmac.setAddr(strtol(setting.myDevId.c_str(), NULL, 16));
  }

  fanet.begin(PinLora_SCK, PinLora_MISO, PinLora_MOSI, PinLora_SS,PinLoraRst, PinLoraDI0,PinLoraGPIO,setting.FrqCor * 1000,14,radioChip);
  fanet.setGPS(status.gps.bHasGPS);
  #ifdef GSMODULE
  if (setting.Mode == eMode::GROUND_STATION){
    //mode ground-station
    status.gps.Lat = setting.gs.lat;
    status.gps.Lon = setting.gs.lon;  
    status.gps.alt = setting.gs.alt;
    status.gps.geoidAlt = setting.gs.geoidAlt;
    MyFanetData.lat = status.gps.Lat;
    MyFanetData.lon = status.gps.Lon;
    MyFanetData.altitude = status.gps.alt;
    fanet.setMyTrackingData(&MyFanetData,setting.gs.geoidAlt,0); //set Data on fanet
  }
  #endif
  fanet.setPilotname(setting.PilotName);
  fanet.setAircraftType(FanetLora::aircraft_t(setting.AircraftType));
  fanet.autoSendName = true;
  if (setting.Mode == eMode::GROUND_STATION){
    fanet.autobroadcast = false;
  }else{
    fanet.autobroadcast = true;
  }
  setting.myDevId = fanet.getMyDevId();
  host_name = APPNAME "-" + setting.myDevId; //String((ESP32_getChipId() & 0xFFFFFF), HEX);
  #ifdef AIRMODULE
  if (setting.Mode == eMode::AIR_MODULE){
    flarmDataPort.begin();
  }
  #endif
  if (setting.OGNLiveTracking.mode > 0){
    #ifdef GSMODULE
      if (setting.Mode == eMode::GROUND_STATION){
        ogn.setAirMode(false); //set airmode
        #ifdef GSM_MODULE
          if (setting.wifi.connect == eWifiMode::CONNECT_NONE){
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
      if (setting.Mode == eMode::AIR_MODULE){
        ogn.setAirMode(true); //set airmode
        ogn.begin("FNB" + setting.myDevId,VERSION "." APPNAME);
      }
    #endif
  }

  tLoop = micros();
  while(1){    
    // put your main code here, to run repeatedly:
    uint32_t tAct = millis();
    if ((bShowBattPower) && (bBatPowerOk)){
      //log_i("show batt-percent");
      userled.setBattPower(status.battery.percent);
      userled.setState(gxUserLed::showBattPower); //blink slow 1-5 times for batt-percent 0-100percent
      bShowBattPower = false; 
      //log_i("show Batt ok");     
    }else if (status.gps.Fix){
      userled.setBlinkFast(1); //blink fast 1 times
    }else{
      userled.setBlinkFast(2); //blink fast 2 times
    }
    #ifdef TEST
    if (timeOver(tAct,tSend,1000)){
      tSend = tSend;
      sendData2Client("$GPGGA,161640.00,4804.36384,N,01444.09266,E,1,04,11.84,320.6,M,43.5,M,,*69\n");
      sendData2Client("$GPRMC,161640.00,A,4804.36384,N,01444.09266,E,0.867,,090121,,,A*7A\n");
      sendData2Client("#FNF 8,C3CC,1,0,1,B,F33344467E0A0A12397CB8\n");
      sendData2Client("#FNF 8,C3CC,1,0,2,13,416E64726561732048696E74657265636B6572\n");
    }
    #endif

    status.tLoop = micros() - tLoop;
    tLoop = micros();
    if (timeOver(tAct,tMaxLoop,10000)){
      //Serial.printf("tMax=%d\r\n",status.tMaxLoop);
      status.tMaxLoop = 0; //reset every 10seconds max-looptime
      tMaxLoop = tAct;
    }
    if (status.tMaxLoop < status.tLoop) status.tMaxLoop = status.tLoop;
    #ifdef AIRMODULE    
    if (command.ConfigGPS == 1){    
      bool bCheckQuectelGps = true;
      if ((setting.boardType ==  T_BEAM) || (setting.boardType ==  T_BEAM_V07)){
        bCheckQuectelGps = false;
      }
      if (bCheckQuectelGps){
        if (setupQuectelGps()){
          command.ConfigGPS = 2; //setting ok
          delay(2000);
          esp_restart();
        }
      }
      if (setupUbloxConfig()){
        command.ConfigGPS = 2; //setting ok
        delay(2000);
        esp_restart();
      }else{
        command.ConfigGPS = 255; //error Setting GPS-konfig
      }
    }
    #endif
    //handleButton(tAct);
    // Should be called every 4-5ms or faster, for the default debouncing time
    // of ~20ms.
    for (uint8_t i = 0; i < NUMBUTTONS; i++) {
      if (sButton[i].PinButton >= 0){
        buttons[i].check();
      }
    }
    //check Button 0
    if (sButton[0].state == ace_button::AceButton::kEventClicked){
      //log_v("Short Press IRQ");
      setting.screenNumber ++;
      bShowBattPower = true; //show battery-state again
      if (setting.screenNumber > MAXSCREENS) setting.screenNumber = 0;
      write_screenNumber(); //save screennumber in File
    }else if (sButton[0].state == ace_button::AceButton::kEventLongPressed){
      //log_v("Long Press IRQ");
      status.bPowerOff = true;
    }else if (sButton[0].state == ace_button::AceButton::kEventDoubleClicked){
      //log_v("double clicked IRQ"); --> switch wifi off or on
      log_i("double clicked wifi on/off");
      userled.setState(gxUserLed::off); //switch to state off --> so state is working
      if (status.bWifiOn){
        userled.setState(gxUserLed::showWifiDis);
        wifiCMD = 10; //switch off wifi
      }else{
        userled.setState(gxUserLed::showWifiEn);
        wifiCMD = 11; //switch on wifi
      }
    }
    sButton[0].state =  0;
    //check Button 1
    if (sButton[1].state == ace_button::AceButton::kEventClicked){
      if (status.bMuting){
        status.bMuting = false; //undo muting
      }else{
        if (setting.vario.volume == LOWVOLUME){
          setting.vario.volume = MIDVOLUME;  
        }else if (setting.vario.volume == MIDVOLUME){
          setting.vario.volume = HIGHVOLUME;  
        }else if (setting.vario.volume == HIGHVOLUME){
          setting.vario.volume = LOWVOLUME;  
        }
        write_Volume();
      }
      //log_i("volume=%d",setting.vario.volume);
    }else if (sButton[1].state == ace_button::AceButton::kEventLongPressed){
      status.bMuting = !status.bMuting; //toggle muting
    }
    sButton[1].state = 0;

    if (setting.bHasFuelSensor) readFuelSensor(tAct);

    if (setting.OGNLiveTracking.mode > 0){
      if (status.vario.bHasVario){
        ogn.setStatusData(status.vario.pressure ,status.vario.temp,NAN,(float)status.battery.voltage / 1000.,status.battery.percent);
      }else if ((setting.wd.mode.bits.enable) || (status.bWUBroadCast)){
        ogn.setStatusData(status.weather.Pressure ,status.weather.temp,status.weather.Humidity,(float)status.battery.voltage / 1000.,status.battery.percent);
      }else{
        ogn.setStatusData(NAN ,NAN, NAN, (float)status.battery.voltage / 1000.,status.battery.percent);
      }
      ogn.run(status.bInternetConnected);
    } 
    if (pMqtt){
      pMqtt->run(status.bInternetConnected);
      char cmd[100];
      if (pMqtt->getLastCmd(cmd,100)){
        checkReceivedLine(cmd);
      }
      if (sMqttState[0]){
        xSemaphoreTake( xOutputMutex, portMAX_DELAY );
        pMqtt->sendState(sMqttState);
        sMqttState[0] = 0;
        xSemaphoreGive(xOutputMutex);
      }      
      if ((myWdCount != wdCount) && (setting.mqtt.mode.bits.sendWeather)) {
        pMqtt->sendTopic("WD",pWd,false);
        myWdCount = wdCount;
      }
      #ifdef SEND_RAW_WIND_DATA
        if (myWdRawCount != wdRawCount){
          pMqtt->sendTopic("WD_Raw",pWdRaw,false);
          myWdRawCount = wdRawCount;
        }
      #endif
    } 
    #ifdef BLUETOOTH    
    if (RxBleQueue) {
      ble_data data;
      BaseType_t status = xQueueReceive(RxBleQueue, &data, 0);
      if (status == pdTRUE) {
        checkReceivedLine(data.data);
      }      
    }
    #endif  
    checkFlyingState(tAct);
    if (printBattVoltage(tAct)){
      bBatPowerOk = true;
    }
    if (sOutputData.length() > 0){
      String s;
      xSemaphoreTake( xOutputMutex, portMAX_DELAY );
      s = sOutputData;
      snprintf(sMqttState,sizeof(sMqttState),sOutputData.c_str());
      sOutputData = "";
      xSemaphoreGive(xOutputMutex);
      //log_i("sending 2 client %s",s.c_str());
      if (fanetDstId > 0){ //send msg back to dst via fanet
        log_i("sending back to %s:%s",fanet.getDevId(fanetDstId),s.c_str()); 
        log_i("updatestate=%d",status.updateState);
        fanet.writeMsgType3(fanetDstId,s);
        if (status.updateState != 50){
          fanetDstId = 0;
        }        
      }
      char * cstr = new char [s.length()+1];
      strcpy (cstr, s.c_str());      
      sendData2Client(cstr,s.length());
      delete cstr; //delete allocated String
    }

    #ifdef AIRMODULE
    //if (setting.Mode == eMode::AIR_MODULE){
    readGPS();
    //}
    #endif
    sendFlarmData(tAct);
    if (sendFanetData == 1){
      log_i("sending msgtype 1");
      fanetTrackingData.devId = fanet._myData.devId;
      fanet.writeMsgType1(&fanetTrackingData);
      sendAWTrackingdata(&fanetTrackingData);
      sendTraccarTrackingdata(&fanetTrackingData);
      sendFanetData = 0;
    }else if (sendFanetData == 2){
      log_i("sending msgtype 2 %s",fanetString.c_str());
      fanet.writeMsgType2(fanetString);
      sendFanetData = 0;
    }else if (sendFanetData == 3){
      log_i("sending msgtype 3 to %s %s",fanet.getDevId(fanetReceiver).c_str(),fanetString.c_str());
      fanet.writeMsgType3(fanetReceiver,fanetString);
      sendFanetData = 0;
    }else if (sendFanetData == 4){
      log_i("sending msgtype 4");
      fanetWeatherData.bStateOfCharge = true;
      fanetWeatherData.bBaro = true;
      fanetWeatherData.bHumidity = true;
      fanetWeatherData.bWind = true;
      fanetWeatherData.bTemp = true;
      fanet.writeMsgType4(&fanetWeatherData);
      sendFanetData = 0;
    }
    if (sendWeatherData){ //we have to send weatherdata
      //log_i("sending weatherdata %d,%d,%d,%d,%d",fanetWeatherData.bTemp,fanetWeatherData.bHumidity,fanetWeatherData.bStateOfCharge,fanetWeatherData.bWind,fanetWeatherData.bBaro);
      fanet.writeMsgType4(&fanetWeatherData);
      if (setting.OGNLiveTracking.bits.sendWeather) {
        Ogn::weatherData wData;
        wData.devId = fanet.getMyDevId();
        wData.lat = status.gps.Lat;
        wData.lon = status.gps.Lon;
        wData.bBaro = status.weather.bPressure;
        wData.Baro = status.weather.Pressure;
        wData.bHumidity = status.weather.bHumidity;
        wData.Humidity = status.weather.Humidity;
        wData.bRain = setting.wd.mode.bits.rainSensor;
        wData.rain1h = status.weather.rain1h;
        wData.rain24h = status.weather.rain1d;
        wData.bTemp = status.weather.bTemp;
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
    if ((setting.fanetMode == eFnMode::FN_AIR_TRACKING) || (status.flying)){
      fanet.onGround = false; //online-tracking
    }else{
      fanet.onGround = true; //ground-tracking
    }
    if (!WebUpdateRunning){
      fanet.run();
    }
    userled.run();
    //status.fanetRx = fanet.rxCount;
    //status.fanetTx = fanet.txCount;
    fanet.getRxTxCount(&status.fanetRx,&status.fanetTx,&status.legRx,&status.legTx);
    if (fanet.isNewMsg()){
      //write msg to udp !!      
      String msg = fanet.getactMsg() + "\n";
      if (setting.outputFANET){
        char * cstr = new char [msg.length()+1];
        strcpy (cstr, msg.c_str());      
        sendData2Client(cstr,msg.length());//sendData2Client(msg);
        delete cstr; //delete allocated String
      } 
    }
    FanetLora::nameData nameData;
    if (fanet.getNameData(&nameData)){
      if (setting.OGNLiveTracking.bits.fwdName){
        ogn.sendNameData(fanet.getDevId(nameData.devId),nameData.name,(float)nameData.snr / 10.0);
      }
    }
    FanetLora::msgData msgData;
    if (fanet.getlastMsgData(&msgData)){
      status.lastFanetMsg = msgData.msg;
      status.FanetMsgCount++;
      if ((pMqtt) && (setting.mqtt.mode.bits.enable)) {
        StaticJsonDocument<300> doc;
        char msg_buf[300];
        char timestamp[30];
        snprintf(timestamp, sizeof(timestamp), "%04d-%02d-%02dT%02d:%02d:%02d+00:00",
                 year(), month(), day(), hour(), minute(), second());
        doc["DT"] = timestamp;
        doc["ID"] = fanet.getDevId(msgData.srcDevId);
        doc["msg"] = msgData.msg;
        doc["rssi"] = msgData.rssi;
        serializeJson(doc, msg_buf);
        pMqtt->sendTopic("RxMsg", msg_buf, false);
      }      
      if (msgData.dstDevId == fanet._myData.devId){
        String sRet = "";
        int pos = getStringValue(msgData.msg,"P","#",0,&sRet);
        if (pos >= 0){
          if ((atoi(sRet.c_str()) == setting.fanetpin) && (setting.fanetpin != 0)){
            fanetDstId = msgData.srcDevId; //we have to store the sender, to send back the value !!
            log_i("got fanet-cmd from %s:%s",fanet.getDevId(fanetDstId),msgData.msg.c_str()); 
            //log_i("msg=%s",msgData.msg.substring(pos).c_str());
            String s = msgData.msg.substring(pos) + "\r\n";
            checkReceivedLine(s.c_str());
          }
        }
      }
    }
    FanetLora::weatherData weatherData;
    if (fanet.getWeatherData(&weatherData)){
#ifdef GSMODULE
      if (setting.Mode == eMode::GROUND_STATION){
        //check if we forward the weather-data to WU
        for (int i = 0; i < MAXFNTUPLOADSTATIONS;i++){
          if (setting.FntWuUpload[i].FanetId == weatherData.devId){
            sendFanetWeatherData2WU(&weatherData,i);
          }
        }
        //check if we forward the weather-data to Windy
        for (int i = 0; i < MAXFNTUPLOADSTATIONS;i++){
          if (setting.FntWiUpload[i].FanetId == weatherData.devId){
            sendFanetWeatherData2WI(&weatherData,i);
          }
        }
        if ((pMqtt) && (setting.mqtt.mode.bits.sendWeather)){ //we use MQTT
          StaticJsonDocument<500> doc;                      //Memory pool
          char buff[30];
          char msg_buf[500];
          snprintf (buff,sizeof(buff)-1,"%04d-%02d-%02dT%02d:%02d:%02d+00:00",year(),month(),day(),hour(),minute(),second()); // ISO 8601
          doc["DT"] = buff;
          doc["ID"] = fanet.getDevId(weatherData.devId);
          doc["rssi"] = weatherData.rssi;
          if (weatherData.bWind){
            doc["wDir"] = round2(weatherData.wHeading);
            doc["wSpeed"] = round2(weatherData.wSpeed);
            doc["wGust"] = round2(weatherData.wGust);
          } 
          if (weatherData.bTemp) doc["temp"] = round2(weatherData.temp);
          if (weatherData.bHumidity) doc["hum"] = round2(weatherData.Humidity);
          if (weatherData.bBaro) doc["press"] = round2(weatherData.Baro);
          if (weatherData.bStateOfCharge) doc["soc"] = round2(weatherData.Charge);
          serializeJson(doc, msg_buf);
          pMqtt->sendTopic("RxWd",&msg_buf[0],false);
        }
      }
#endif
      if (setting.OGNLiveTracking.bits.fwdWeather){
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
        if ((wData.wHeading >= 0) && (wData.wHeading <= 360) && (wData.bWind) && (wData.wSpeed >= 0) && (wData.wSpeed <= 300) && (wData.wGust >= 0) && (wData.wGust <= 300)){
          ogn.sendWeatherData(&wData);
        }
        
      }
    }    
    if (fanet.getTrackingData(&tFanetData)){
      //log_i("new Tracking-Data");
      if (tFanetData.type == 0x11){ //online-tracking
        if (setting.OGNLiveTracking.bits.liveTracking){
          ogn.sendTrackingData(tFanetData.timestamp ,tFanetData.lat ,tFanetData.lon,tFanetData.altitude,tFanetData.speed,tFanetData.heading,tFanetData.climb,fanet.getDevId(tFanetData.devId) ,(Ogn::aircraft_t)fanet.getFlarmAircraftType(&tFanetData),tFanetData.addressType,tFanetData.OnlineTracking,(float)tFanetData.snr);
        } 
        sendAWTrackingdata(&tFanetData);
        sendTraccarTrackingdata(&tFanetData);
      }else if (tFanetData.type >= 0x70){ //ground-tracking
        if (setting.OGNLiveTracking.bits.liveTracking){
          ogn.sendGroundTrackingData(tFanetData.timestamp,tFanetData.lat,tFanetData.lon,tFanetData.altitude,fanet.getDevId(tFanetData.devId),tFanetData.type,tFanetData.addressType,(float)tFanetData.snr);
        } 
      }
    }    
    flarmDataPort.run();
    if (setting.outputModeVario == eOutputVario::OVARIO_LK8EX1) sendLK8EX(tAct);
    if (setting.outputModeVario == eOutputVario::OVARIO_LXPW) sendLXPW(tAct); //not output 
    #ifdef AIRMODULE
    if ((setting.Mode == eMode::AIR_MODULE) || ((abs(setting.gs.lat) <= 0.1) && (abs(setting.gs.lon) <= 0.1))){ //in GS-Mode we use the GPS, if in settings disabled
      /*
      if ((PinPPS < 0) && (!status.gps.bExtGps)){
        if ((tAct - tOldPPS) >= 1000){
          gtPPS = millis();
          ppsTriggered = true;
        }
      }
      if (ppsTriggered){
        ppsTriggered = false;        
        tLastPPS = tAct;      
        //log_i("PPS-Triggered t=%d",status.gps.tCycle);
      }
      */
      if (nmea.isNewMsgValid()){
        nmea.clearNewMsgValid();
        if ((PinPPS < 0) && (!status.gps.bExtGps)){
          gtPPS = millis() - 110; //we set PPS to actual-time -100ms, measured on T-Beam V1.0
        }
        //log_i("diff=%d",millis()-gtPPS);
        tLastPPS = tAct;
        //log_i("lat=%d;lon=%d",nmea.getLatitude(),nmea.getLongitude());
        status.gps.tCycle = tAct - tOldPPS;
        if (nmea.isValid()){
          //log_i("nmea is valid");
          long alt = 0;
          nmea.getAltitude(alt);
          #ifdef AIRMODULE
          if (setting.Mode == eMode::AIR_MODULE){
            status.gps.NumSat = nmea.getNumSatellites();
          }
          #endif
          status.gps.Fix = 1;
          if (status.gps.NumSat < 4){ //we need at least 4 satellites to get accurate position !!
            status.gps.speed = 0;
            status.gps.course = 0;
          }else{
            status.gps.speed = nmea.getSpeed()*1.852/1000.; //speed in cm/s --> we need km/h
            status.gps.course = nmea.getCourse()/1000.;
          status.gps.hdop = nmea.getHDOP();
          //log_i("hdop=%d",status.gps.hdop);
          // create a global variable for logger igc file name based on GPS datetime
            //set today date
            // if got a correct date i.e. with year
            if (nmea.getYear()>0){
              snprintf (status.gps.Date,sizeof(status.gps.Date)-1,"%02d%02d%02d",nmea.getDay(),nmea.getMonth(),nmea.getYear() - 2000);
              snprintf (status.gps.Time,sizeof(status.gps.Time)-1,"%02d%02d%02d",nmea.getHour(),nmea.getMinute(),nmea.getSecond());
            }
          }
          long geoidalt = 0;
          nmea.getGeoIdAltitude(geoidalt);
          //log_i("latlon=%d,%d",nmea.getLatitude(),nmea.getLongitude());
          status.gps.Lat = nmea.getLatitude() / 1000000.;
          status.gps.Lon = nmea.getLongitude() / 1000000.;
          #ifdef FLARMTEST
            //status.gps.Lat = setting.gs.lat;
            //status.gps.Lon = setting.gs.lon;
            //only for testing NZ
            //status.gps.Lat = -41.0605988;
            //status.gps.Lon = 175.3534596;  
            //only for testing EU
            //status.gps.Lat = 48.387;
            //status.gps.Lon = 14.4;  
            //USA
            //status.gps.Lat = 42.16;
            //status.gps.Lon = -106.1;  
            //australia
            status.gps.Lat = -22;
            status.gps.Lon = 134;  
            #endif
          status.gps.alt = alt/1000.;
          status.gps.geoidAlt = geoidalt/1000.;
          //setTime(nmea.getHour(), nmea.getMinute(), nmea.getSecond(), nmea.getDay(), nmea.getMonth(), nmea.getYear());
          struct tm timeinfo;
          timeinfo.tm_year = nmea.getYear() - 1900; //from 1900
          timeinfo.tm_mon = nmea.getMonth() - 1; //month begin with 0
          timeinfo.tm_mday = nmea.getDay();
          timeinfo.tm_hour = nmea.getHour();
          timeinfo.tm_min = nmea.getMinute(); 
          timeinfo.tm_sec = nmea.getSecond();
          setAllTime(timeinfo); //we have to set time to GPS-Time for decoding.

          MyFanetData.timestamp = now();
          if (oldAlt == 0) oldAlt = status.gps.alt;        
          if (!status.vario.bHasVario) status.vario.ClimbRate = (status.gps.alt - oldAlt) / (float(status.gps.tCycle) / 1000.0);        
          oldAlt = status.gps.alt;          
          MyFanetData.climb = status.vario.ClimbRate;
          MyFanetData.lat = status.gps.Lat;
          MyFanetData.lon = status.gps.Lon;
          MyFanetData.altitude = status.gps.alt;
          MyFanetData.hdop = status.gps.hdop;
          //log_i("lat=%.6f;lon=%.6f;alt=%.1f;geoAlt=%.1f",status.gps.Lat,status.gps.Lon,status.gps.alt,geoidalt/1000.);
          if (fanet.onGround){
            MyFanetData.speed = 0.0;
          }else{
            MyFanetData.speed = status.gps.speed; //speed in cm/s --> we need km/h
            if (MyFanetData.speed < 1.0) MyFanetData.speed = 1.0;
          }
          MyFanetData.addressType = fanet.getAddressType();
          if ((status.gps.speed <= 5.0) && (status.vario.bHasVario)){
            MyFanetData.heading = status.vario.Heading;
          }else{
            MyFanetData.heading = status.gps.course;
          }
          MyFanetData.aircraftType = fanet.getAircraftType();        
          if (setting.Mode == eMode::AIR_MODULE){
            if (setting.OGNLiveTracking.bits.liveTracking){
              ogn.setGPS(status.gps.Lat,status.gps.Lon,status.gps.alt,status.gps.speed,MyFanetData.heading);
              if (fanet.onGround){
                ogn.sendGroundTrackingData(now(),status.gps.Lat,status.gps.Lon,status.gps.alt,fanet.getDevId(tFanetData.devId),fanet.state,MyFanetData.addressType,0.0);
              }else{
                ogn.sendTrackingData(now(),status.gps.Lat,status.gps.Lon,status.gps.alt,status.gps.speed,MyFanetData.heading,status.vario.ClimbRate,fanet.getMyDevId() ,(Ogn::aircraft_t)fanet.getFlarmAircraftType(&MyFanetData),MyFanetData.addressType,MyFanetData.OnlineTracking,0.0);
              }
              
            } 
            sendAWTrackingdata(&MyFanetData);
            sendTraccarTrackingdata(&MyFanetData);
          }
          fanet.setMyTrackingData(&MyFanetData,geoidalt/1000.,gtPPS); //set Data on fanet
        }else{
          status.gps.Fix = 0;
          status.gps.speed = 0.0;
          status.gps.Lat = 0.0;
          status.gps.Lon = 0.0;
          status.gps.alt = 0.0;
          status.gps.course = 0.0;
          status.gps.NumSat = 0;
          status.gps.geoidAlt = 0;
          status.gps.hdop = 0;
          if (!status.vario.bHasVario) status.vario.ClimbRate = 0.0;
          oldAlt = 0.0;
        }
        tOldPPS = tAct;
      }
      if((tAct - tLastPPS) >= 10000){
        //no pps for more then 10sec. --> clear GPS-state
        status.gps.Fix = 0;
        status.gps.speed = 0.0;
        status.gps.Lat = 0.0;
        status.gps.Lon = 0.0;
        status.gps.alt = 0.0;
        status.gps.geoidAlt = 0.0;
        status.gps.course = 0.0;
        status.gps.NumSat = 0;
        status.gps.hdop = 0;
      }
    /*
    }else{
      if ((PinPPS < 0) && (!status.gps.bExtGps)){
        if ((tAct - tOldPPS) >= 1000){
          gtPPS = millis();
          ppsTriggered = true;
          
        }
      }
      if (ppsTriggered){
        ppsTriggered = false;
        tOldPPS = tAct;
        MyFanetData.climb = status.vario.ClimbRate;
        MyFanetData.lat = status.gps.Lat;
        MyFanetData.lon = status.gps.Lon;
        MyFanetData.altitude = status.gps.alt;
        MyFanetData.speed = 0; //speed in cm/s --> we need km/h
        MyFanetData.addressType = ADDRESSTYPE_OGN;
        MyFanetData.heading = 0;
        MyFanetData.aircraftType = fanet.getAircraftType();        
        fanet.setMyTrackingData(&MyFanetData,status.gps.geoidAlt,gtPPS); //set Data on fanet

      }
      */
    }
    #endif

    #ifdef GSMODULE
    if (setting.Mode == eMode::GROUND_STATION){
      sendAWGroundStationdata(tAct); //send ground-station-data    
    }
    #endif

    if (PMU_Irq){
      xSemaphoreTake( *PMUMutex, portMAX_DELAY );
      PMU->getIrqStatus(); //read irqStatus
      if (PMU->isPekeyLongPressIrq()){
        sButton[0].state = ace_button::AceButton::kEventLongPressed;
        //log_i("PMU LongPress");
      }
      if (PMU->isPekeyShortPressIrq()){
        sButton[0].state = ace_button::AceButton::kEventClicked;
        //log_i("PMU ShortPress");
      }
      PMU->clearIrqStatus();
      xSemaphoreGive( *PMUMutex );
      PMU_Irq = false;
    }

    delay(1);
    //if ((WebUpdateRunning) || (bPowerOff)) break;
    if (bPowerOff) break;
    if (status.restart.doRestart){
      if (timeOver(millis(),status.restart.time,2000)){
        Serial.println("ESP Restarting !");
        esp_restart();
      }
    }else{
      status.restart.time = millis();
    }
  }
  log_i("stop task");
  fanet.end();
  if (setting.OGNLiveTracking.mode > 0) ogn.end();
  vTaskDelete(xHandleStandard); //delete standard-task
}

void powerOff(){
  if (status.PMU != ePMU::NOPMU){
    xSemaphoreTake( *PMUMutex, portMAX_DELAY );
    PMU->clearIrqStatus();
    xSemaphoreGive(*PMUMutex);
  }
  bPowerOff = true;
  delay(100);

  //esp_wifi_set_mode(WIFI_MODE_NULL);
  //esp_wifi_stop();
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
  eTaskState tOled = eDeleted;
  eTaskState tStandard = eDeleted;
  #ifdef GSM_MODULE
  eTaskState tGSM = eDeleted;
  #endif
  eTaskState tWeather = eDeleted;
  eTaskState tLogger = eDeleted;
  while(1){
    //wait until all tasks are stopped
    if (xHandleLogger != NULL) tLogger = eTaskGetState(xHandleLogger);
    if (xHandleBaro != NULL) tBaro = eTaskGetState(xHandleBaro);
    if (xHandleEInk != NULL) tEInk = eTaskGetState(xHandleEInk);
    if (xHandleOled != NULL) tOled = eTaskGetState(xHandleOled);
    if (xHandleStandard != NULL) tStandard = eTaskGetState(xHandleStandard);
    if (xHandleWeather != NULL) tWeather = eTaskGetState(xHandleWeather);    
    if ((tLogger == eDeleted) && (tBaro == eDeleted) && (tEInk == eDeleted) && (tWeather == eDeleted) && (tStandard == eDeleted) && (tOled == eDeleted)) break; //now all tasks are stopped
    log_i("logger=%d,baro=%d,eink=%d,oled=%d,standard=%d,weather=%d",tLogger,tBaro,tEInk,tOled,tStandard,tWeather);
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

  if (PinUserLed >= 0){
    pinMode(PinUserLed, OUTPUT);
    digitalWrite(PinUserLed,HIGH); 
  }
  if (PinBuzzer >= 0){
    digitalWrite(PinBuzzer,LOW);     
  }  
  if (PinBeaconLed >= 0){
    digitalWrite(PinBeaconLed,LOW);    
  }
  if (PinExtPower >= 0){
    digitalWrite(PinExtPower,HIGH);
    //pinMode(PinExtPower,INPUT); //set ext-power-pin to input (if low --> ext-power is enabled)  
  }
  
  log_i("switch power-supply off");  
  delay(100);
  if (status.PMU != ePMU::NOPMU){
    xSemaphoreTake( *PMUMutex, portMAX_DELAY );
    if (status.PMU == ePMU::AXP192){
      PMU->setChargingLedMode(XPOWERS_CHG_LED_OFF);
      PMU->disablePowerOutput(XPOWERS_LDO2); //LORA
      PMU->disablePowerOutput(XPOWERS_LDO3); //GPS
      PMU->disablePowerOutput(XPOWERS_DCDC2); // NC
      if (status.displayType != OLED0_96){
        PMU->disablePowerOutput(XPOWERS_DCDC1);//OLED-Display 3V3
      }
      esp_sleep_enable_ext0_wakeup((gpio_num_t) PinPMU_Irq, 0); // 1 = High, 0 = Low
    }else{
      PMU->shutdown(); //shutdown disables all power supply
    }
    xSemaphoreGive(*PMUMutex);
  }else if ((sButton[0].PinButton >= 0) && (PinExtPowerOnOff < 0)){
    while (!digitalRead(sButton[0].PinButton)){
      log_i("wait until power-button is released");
      delay(500); //wait 500ms
    }
    uint64_t mask = uint64_t(1) << uint64_t(sButton[0].PinButton); //prg-Button has a pull-up-resistor --> Pin has to go low, if pressed
    #ifdef CONFIG_IDF_TARGET_ESP32
      esp_sleep_enable_ext1_wakeup(mask,ESP_EXT1_WAKEUP_ALL_LOW); //wait until power is back again
    #else
      esp_sleep_enable_ext1_wakeup(mask,ESP_EXT1_WAKEUP_ANY_LOW); //wait until power is back again
    #endif
  }
  if (PinExtPowerOnOff >= 0){
    if (!digitalRead(PinExtPowerOnOff)){
      //pin has to be low, if not, somebody switched off with button !!
      uint64_t mask = uint64_t(1) << uint64_t(PinExtPowerOnOff);
      //uint64_t mask = 0x1000000000;
      esp_sleep_enable_ext1_wakeup(mask,ESP_EXT1_WAKEUP_ANY_HIGH); //wait until power is back again
    }
  }
  

  //Serial.end();
  //esp_wifi_stop();
  #ifdef BLUETOOTH
  #ifndef S3CORE
  #ifndef WIRELESS_STICK_V3
  stopBluetooth();
  #endif
  #endif
  #endif
  if (PinLora_SCK >= 0) pinMode(PinLora_SCK,INPUT);
  if (PinLoraRst >= 0) pinMode(PinLoraRst,INPUT);
  if (PinLora_MISO >= 0) pinMode(PinLora_MISO,INPUT);
  if (PinLora_MOSI>= 0) pinMode(PinLora_MOSI,INPUT);
  if (PinLora_SS >= 0) pinMode(PinLora_SS,INPUT);
  if (PinLoraDI0 >= 0) pinMode(PinLoraDI0,INPUT);
  esp_deep_sleep_start();
  

}

#ifdef FLARMLOGGER
void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if(!root){
    Serial.println("Failed to open directory");
    return;
  }
  if(!root.isDirectory()){
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while(file){
    if(file.isDirectory()){
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if(levels){
        listDir(fs, file.name(), levels -1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void createDir(fs::FS &fs, const char * path){
  Serial.printf("Creating Dir: %s\n", path);
  if(fs.mkdir(path)){
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char * path){
  Serial.printf("Removing Dir: %s\n", path);
  if(fs.rmdir(path)){
    Serial.println("Dir removed");
  } else {
    Serial.println("rmdir failed");
  }
}

void readFile(fs::FS &fs, const char * path){
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if(!file){
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while(file.available()){
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
  //log_i("Appending to file: %s\n", path);
  File file = fs.open(path, FILE_APPEND);
  if(!file){
    log_e("Failed to open file for appending");
    return;
  }
  if(file.print(message)){
    //log_i("Message appended");
  } else {
    log_e("Append failed");
  }
  file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
  //log_i("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    //log_i("File renamed");
  } else {
    log_e("Rename failed");
  }
}

void deleteFile(fs::FS &fs, const char * path){
 //log_i("Deleting file: %s\n", path);
  if(fs.remove(path)){
    //log_i("File deleted");
  } else {
    log_e("Delete failed");
  }
}

void testFileIO(fs::FS &fs, const char * path){
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if(file){
    len = file.size();
    size_t flen = len;
    start = millis();
    while(len){
      size_t toRead = len;
      if(toRead > 512){
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    Serial.printf("%u bytes read for %u ms\n", flen, end);
    file.close();
  } else {
    Serial.println("Failed to open file for reading");
  }


  file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file for writing");
    return;
  }

  size_t i;
  start = millis();
  for(i=0; i<2048; i++){
    file.write(buf, 512);
  }
  end = millis() - start;
  Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
  file.close();
}

// logger task to manage new and update of igc track log
void taskLogger(void * pvPArameters){

  #define SD_CS 13
  #define SD_SCK 14
  #define SD_MOSI 15
  #define SD_MISO 2
  pinMode(SD_CS, OUTPUT);
  pinMode(SD_SCK, OUTPUT);
  pinMode(SD_MOSI, OUTPUT);
  pinMode(SD_MISO, INPUT_PULLUP);
  //SPIClass sd_spi(VSPI);
  SPIClass sd_spi(HSPI);
  // SD Card
  sd_spi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  //if (!SD.begin(SD_CS, sd_spi,4000000,"/sd",5,true)){
  if (!SD.begin(SD_CS,sd_spi)){ 
    log_e("SD Card: mounting failed.");
    log_i("stop task");
    vTaskDelete(xHandleLogger);    
    return;    
  }else{
    log_i("SD Card: mounted.");  
  } 
  sdcard_type_t cardType = SD.cardType();
  if(cardType == CARD_MMC){
      log_i("SD Card Type: MMC");
  } else if(cardType == CARD_SD){
      log_i("SD Card Type: SDSC");
  } else if(cardType == CARD_SDHC){
      log_i("SD Card Type: SDHC");
  } else {
      log_i("SD Card Type: UNKNOWN");
  }  
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  log_i("SD Card Size: %dMB", cardSize);

  /*
  listDir(SD, "/", 0);
  createDir(SD, "/mydir");
  listDir(SD, "/", 0);
  removeDir(SD, "/mydir");
  listDir(SD, "/", 2);
  writeFile(SD, "/hello.txt", "Hello ");
  appendFile(SD, "/hello.txt", "World!\n");
  readFile(SD, "/hello.txt");
  deleteFile(SD, "/foo.txt");
  renameFile(SD, "/hello.txt", "/foo.txt");
  readFile(SD, "/foo.txt");
  testFileIO(SD, "/test.txt");
  */
  log_i("Total space: %lluMB", SD.totalBytes() / (1024 * 1024));
  log_i("Used space: %lluMB", SD.usedBytes() / (1024 * 1024));

  char sFilename[20];
  /*
  strcpy(&sFilename[0],"/test.log");  
  File file = SD.open(sFilename, FILE_WRITE);
  if(file){
    if(file.print("test\r\n")){
        log_i("File written");
    } else {
        log_i("Write failed");
    }  
    file.close();
  }else{
      log_i("Failed to open file for writing");
  }
  */
  //Logger logger;

  /*
  logger.begin();
  */
  //delay(10);
  bool bFileNameOk = false;
  char sRec[MAXSTRING];
  FlarmLogQueue = xQueueCreate(5, sizeof(sRec));
  while(1){
    //logger.run();
    if (!bFileNameOk){
      if ((status.gps.Fix == 1) && (status.gps.NumSat >= 4)){
        time_t now;
        struct tm timeinfo;
        time(&now);
        gmtime_r(&now, &timeinfo);
        strftime(sFilename, sizeof(sFilename), "/%y%m%d-%H%M%S.log", &timeinfo);
        //strcpy(&sFilename[0],"/test.log");
        //writeFile(SD, &sFilename[0], "firstline\r\n"); //create file
        log_i("filename=%s",sFilename);   
        bFileNameOk = true;
        log_i("GPS-FIX OK --> start logging");
      }
    }else{
      if ((status.gps.Fix != 1) || (status.gps.NumSat < 4)){
        bFileNameOk = false;
        log_i("no GPS-Fix stop logging");
      }
    }
    //check log-queue
    if (FlarmLogQueue) {
      BaseType_t status = xQueueReceive(FlarmLogQueue, &sRec[0], 0);
      if (status == pdTRUE) {
        if (bFileNameOk){
          appendFile(SD, &sFilename[0], &sRec[0]);
        }        
        //log_i("Flarm-Debug received:%s",sRec);
      }      
    }    
    
    delay(1);
    if ((WebUpdateRunning) || (bPowerOff)) break;
  }
  //if (bPowerOff) logger.end();
  log_i("stop task");
  vTaskDelete(xHandleLogger);
}
#endif

#if defined(SSD1306) || defined(SH1106G)
void taskOled(void *pvParameters){
  log_i("start Oled-task");
  if (status.displayType != OLED0_96){
    log_i("stop task");
    vTaskDelete(xHandleOled);
    return;
  }
  Oled display;
  display.begin(pI2cOne,PinOledRst,&xI2C1Mutex);
  while(1){
    display.run();
    delay(10);
    if ((WebUpdateRunning) || (bPowerOff)) break;
  }
  if (WebUpdateRunning) display.webUpdate();
  if (bPowerOff) display.end();
  log_i("stop task");
  vTaskDelete(xHandleOled);
}
#endif

#ifdef EINK
void taskEInk(void *pvParameters){
  if ((status.displayType != EINK2_9) && (status.displayType != EINK2_9_V2)){
    log_i("stop task");
    vTaskDelete(xHandleEInk);
    return;
  }
  Screen screen;
  if (setting.displayType == EINK2_9_V2){
    screen.begin(1,PinEink_Cs,PinEink_Dc,PinEink_Rst,PinEink_Busy,PinEink_Clk,PinEink_Din); //display-type 1
  }else{
    screen.begin(0,PinEink_Cs,PinEink_Dc,PinEink_Rst,PinEink_Busy,PinEink_Clk,PinEink_Din);
  }  
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
  //static uint32_t tStartAutoUpdate = millis();
  static uint32_t tWait = 0;
  if (status.tRestart != 0){
    if (timeOver(tAct,status.tRestart,5000)){
      esp_restart();
    }
  }
  /*
  if ((setting.bAutoupdate) && (status.bInternetConnected)){
    if ((status.updateState == 0) && (timeOver(tAct,tStartAutoUpdate,10000))){
      status.updateState = 50; //auto-update
    }
  }else{
    tStartAutoUpdate = tAct;
  }
  */
  if (status.updateState == 1){
    status.updateState = 2;      
    if (updater.checkVersion()){
      status.sNewVersion = updater.getVersion();
      if (updater.checkVersionNewer()){
        status.updateState = 10; //we can update --> newer Version
      }
    }
  }else if (status.updateState == 50){ //autoupdate
    if (updater.checkVersion()){
      status.sNewVersion = updater.getVersion();
      if (updater.checkVersionNewer()){
        status.updateState = 61;
        /*
        add2OutputString("update to " + status.sNewVersion + "\r\n");
        WebUpdateRunning = true;
        delay(500); //wait 1 second until tasks are stopped
        if (updater.updateVersion()){ //update to newest version  
          add2OutputString("update complete\r\n");
          status.updateState = 200;
          status.tRestart = millis();
        }else{
          add2OutputString("update failed\r\n");
          status.updateState = 254;
          status.tRestart = millis();
        }
        */
      }else{
        add2OutputString("no new Version avaiable\r\n");
        status.updateState = 253;
      }
    }else{
      add2OutputString("no connection to server\r\n");
      status.updateState = 252;
    }
  }else if (status.updateState == 60){ //update to version
    updater.setVersion(status.sNewVersion);
    status.updateState = 61;
  }else if (status.updateState == 61){ //update to version
    add2OutputString("update to " + status.sNewVersion + "\r\n");
    tWait = millis();
    status.updateState = 62;
  }else if ((status.updateState == 62) && (timeOver(millis(),tWait,500))) { //update to version
    WebUpdateRunning = true;
    status.updateState = 63;
  }else if ((status.updateState == 63) && (timeOver(millis(),tWait,500))) { //update to version
    if (updater.updateVersion()){ //update to newest version
      add2OutputString("update complete\r\n");
      status.updateState = 200;
      status.tRestart = millis();
    }else{
      add2OutputString("update failed\r\n");
      status.updateState = 254;
      status.tRestart = millis();
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
  //static uint32_t warning_time=0;
  static uint32_t tGetTime = millis();
  uint32_t tBattEmpty = millis();
  uint32_t tRuntime = millis();
  uint32_t tGetWifiRssi = millis();    
  bool bPowersaveOk = false;
  #ifdef GSMODULE
  static uint32_t tCheckDayTime = millis();
  bool bSunsetOk = false;
  Dusk2Dawn dusk2dawn(setting.gs.lat,setting.gs.lon, 0);
  uint8_t actDay = 0;
  #endif
  WiFiUDP ntpUDP;
  NTPClient timeClient(ntpUDP);  
  
  setupWifi();
  setWifi(!setting.wifi.uMode.bits.disableWifiAtStartup);
  #ifdef GSM_MODULE
    if (setting.wifi.connect == eWifiMode::CONNECT_NONE){      
      updater.setClient(&GsmUpdaterClient);
      updater.setMutex(&xGsmMutex);
    }
  #endif

  tBattEmpty = millis();
  tGetTime = millis() - GETNTPINTERVALL + 5000; //we refresh NTP-Time 5 sec. after internet is connected
  timeClient.begin();
  while (1){
    uint32_t tAct = millis();
    if  (status.wifiSTA.state != IDLE){
      Web_loop();
    }
    handleUpdate(tAct);
    /*
    if (pMqtt){
      pMqtt->run(status.bInternetConnected);
      char cmd[100];
      if (pMqtt->getLastCmd(cmd,100)){
        checkReceivedLine(cmd);
      }
      if (sMqttState[0]){
        xSemaphoreTake( xOutputMutex, portMAX_DELAY );
        pMqtt->sendState(sMqttState);
        sMqttState[0] = 0;
        xSemaphoreGive(xOutputMutex);
      }      
      if ((myWdCount != wdCount) && (setting.mqtt.mode.bits.sendWeather)) {
        pMqtt->sendTopic("WD",pWd,false);
        myWdCount = wdCount;
      }
    }
    */ 
    #ifdef GSMODULE  
    if (setting.Mode == eMode::GROUND_STATION){
      if (status.bTimeOk == true){
        struct tm now;
        getLocalTime(&now,0);
        if ((now.tm_mday != actDay) && (now.tm_hour > 0)){
          //restart every new day --> to get new NTP-Time
          //every day at 01:00am
          log_i("day changed %d->%d hour:%d",actDay,now.tm_mday,now.tm_hour);
          if (actDay != 0){
            printLocalTime();
            log_i("new day --> restart ESP");
            delay(500);
            esp_restart();
          }
          actDay = day();
        }
      }
      if (setting.gs.PowerSave == eGsPower::GS_POWER_SAFE){
        if (timeOver(tAct,tRuntime,180000)) bPowersaveOk = true; //after 3min. esp is allowed to go to sleep, so that anybody has a chance to change settings
        //bPowersaveOk = true; //after 3min. esp is allowed to go to sleep, so that anybody has a chance to change settings
        if (status.bTimeOk == true){
          if (!bSunsetOk){
            bSunsetOk = true;
            /*  Time is returned in minutes elapsed since midnight. If no sunrises or
            *  sunsets are expected, a "-1" is returned.
            */
            iSunRise = dusk2dawn.sunrise(year(), month(), day(), false) + setting.gs.sunriseOffset;
            //iSunRise = 900;
            iSunSet = dusk2dawn.sunset(year(), month(), day(), false) + setting.gs.sunsetOffset;
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
            
            if (timeOver(tAct,tCheckDayTime,60000)){
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
    if ((status.wifiSTA.state == FULL_CONNECTED) || (status.modemstatus == eConnectionState::CONNECTED)){
      status.bInternetConnected = true;
    }else{
      status.bInternetConnected = false;
    }
    if (timeOver(tAct,tGetWifiRssi,5000)){
      tGetWifiRssi = tAct;
      status.wifiSTA.Rssi = WiFi.RSSI();
      //log_i("wifi-strength=%d",status.wifiSTA.Rssi);
    }
    if (WiFi.status() == WL_CONNECTED){
      if (timeOver(tAct,tGetTime,GETNTPINTERVALL)){
        tGetTime = tAct;
        log_i("get ntp-time");
        uint32_t tStart = millis();
        uint32_t tBeforeNtp = now();
        //configTime(0, 0, "pool.ntp.org");
        //adjustTime(0);
        struct tm timeinfo;
        //delay(5000); //wait 5 seconds until ntp-time is set.
        //if(getLocalTime(&timeinfo)){
        if (timeClient.forceUpdate()){
          time_t tNow = timeClient.getEpochTime();
          localtime_r(&tNow,&timeinfo);
          uint32_t tEnd = millis();
          log_i("%04d%02d%02d-%02d:%02d:%02d",timeinfo.tm_year + 1900,timeinfo.tm_mon+1,timeinfo.tm_mday,timeinfo.tm_hour,timeinfo.tm_min, timeinfo.tm_sec);
          setAllTime(timeinfo);
          uint32_t tAfterNtp = now(); 
          tBeforeNtp += ((tEnd - tStart) / 1000);
          uint32_t tDist = tAfterNtp - tBeforeNtp;          
          if (tDist > 1){
            log_e("%ds delay timediff to big --> try again in 5 seconds",tDist);
            tGetTime = tAct - GETNTPINTERVALL + 5000; //try again in 5 second
            status.bTimeOk = false; //reset time ok.
          }else{
            log_i("get ntp-time OK");
            #ifdef GSMODULE
              setRTCTime(timeinfo);
            #endif
            if (printLocalTime() == true){
              status.bTimeOk = true;
            } 
          }
        }else{
          log_e("failed to get time from NTP-Server --> try again in 5 seconds");
          tGetTime = tAct - GETNTPINTERVALL + 5000; //try again in 5 second
        }        
      }
    #ifdef GSM_MODULE
    }else if ((status.modemstatus == eConnectionState::CONNECTED) && (setting.wifi.connect == eWifiMode::CONNECT_NONE)){
      if (timeOver(tAct,tGetTime,GETNTPINTERVALL)){
        log_i("get ntp-time");
        byte ret = -1;
        if (xSemaphoreTake( xGsmMutex, ( TickType_t ) 500) == pdTRUE ){
          ret = modem.NTPServerSync("pool.ntp.org",0);
          //log_i("ret=%d",ret);
          if (ret >= 0){
            int year3 = 0;
            int month3 = 0;
            int day3 = 0;
            int hour3 = 0;
            int min3 = 0;
            int sec3 = 0;
            float timezone = 0;
            bool bret = modem.getNetworkTime(&year3, &month3, &day3, &hour3, &min3, &sec3,&timezone);
            log_i("tz=%f,h=%d,min=%d,sec=%d,day=%d,month=%d,year=%d,ret=%d",timezone,hour3,min3, sec3, day3,month3, year3,bret);
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
              #ifdef GSMODULE
                setRTCTime(timeinfo);
              #endif
              //setTime(hour3,min3, sec3, day3,month3, year3);
              
              //log_i("timestatus = %d",timeStatus());
            }
          }        
          xSemaphoreGive( xGsmMutex );
          tGetTime = tAct;
          //log_i("print time");        
          if (printLocalTime() == true){
            status.bTimeOk = true;
          } 
        }
      }
    #endif
    }else{
      tGetTime = tAct - GETNTPINTERVALL + 5000; //we refresh NTP-Time 5 sec. after internet is connected
    }
    if (timeOver(tAct,tWifiCheck,WIFI_RECONNECT_TIME)){
      tWifiCheck = tAct;
      if ((setting.wifi.connect == eWifiMode::CONNECT_ALWAYS) && (status.bWifiOn)){
        //log_i("check Wifi-status %d ",status.wifiSTA.state);
        if (status.wifiSTA.state != FULL_CONNECTED){
          log_i("WiFi not connected. Try to reconnect");
          WiFi.disconnect(true);
          WiFi.mode(WIFI_MODE_NULL);
          #if ESP_IDF_VERSION_MAJOR < 4
          WiFi.config(IPADDR_ANY, IPADDR_ANY, IPADDR_ANY,IPADDR_ANY,IPADDR_ANY); // call is only a workaround for bug in WiFi class
          #endif
          //WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE,INADDR_NONE,INADDR_NONE); // call is only a workaround for bug in WiFi class
          //WiFi.setHostname(host_name.c_str()); //set hostname          
          if (setting.wifi.uMode.bits.switchOffApWhenStaConnected){
            WiFi.mode(WIFI_MODE_STA);
          }else{
            WiFi.mode(WIFI_MODE_APSTA);
          }
          WiFi.begin(setting.wifi.ssid.c_str(), setting.wifi.password.c_str()); 
          
        }
      }
    }
    uint32_t actFreeHeap = xPortGetFreeHeapSize();
    uint32_t minFreeHeap = xPortGetMinimumEverFreeHeapSize();
    /*
    if (actFreeHeap<30000){
      if (millis()>warning_time)
      {
        log_w( "*****LOOP current free heap: %d, minimum ever free heap: %d ******", actFreeHeap, minFreeHeap);
        warning_time=millis()+10000;
      }
    }
    */
    if (minFreeHeap<2000)
    {
      log_e( "*****LOOP current free heap: %d, minimum ever free heap: %d ******", actFreeHeap, minFreeHeap);
      log_e("System Low on Memory - xPortGetMinimumEverFreeHeapSize < 2KB");
      log_e("ESP Restarting !");
      esp_restart();
    }

    if (wifiCMD == 11) setWifi(true); //switch wifi on
    if (wifiCMD == 10) setWifi(false); //switch wifi off
    if (( tAct > (setting.wifi.tWifiStop * 1000)) && (setting.wifi.tWifiStop!=0) && (!WebUpdateRunning)){
      log_i("******************WEBCONFIG Setting - WIFI STOPPING*************************");
      log_i("currHeap:%d,minHeap:%d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
      setWifi(false);
    }
    //yield();
    if (status.bPowerOff){
      powerOff(); //power off, when battery is empty !!
    }
    //if ((status.battery.percent < setting.minBattPercent) && (status.battery.voltage >= BATTPINOK)) { // if Batt-voltage is below 1V, maybe the resistor is missing.
    if ((setting.gs.PowerSave == eGsPower::GS_POWER_BATT_LIFE) || (setting.gs.PowerSave == eGsPower::GS_POWER_SAFE)){
      if ((status.battery.percent <= setting.minBattPercent) && (status.battery.voltage >= BATTPINOK)){
        if (timeOver(tAct,tBattEmpty,60000)){ //min 60sek. below
          log_i("Batt empty voltage=%d.%dV",status.battery.voltage/1000,status.battery.voltage%1000);
          log_i("sleep for 30min. --> then check again batt-voltage");
          esp_sleep_enable_timer_wakeup((uint64_t)BATTSLEEPTIME * uS_TO_S_FACTOR); //set Timer for wakeup        
          powerOff();
        }
      }else{
        tBattEmpty = millis();
      }
    }else{
      if (status.PMU != ePMU::NOPMU){ //power off only if we have an AXP192, otherwise, we can't switch on again.
        if ((status.battery.voltage < battEmpty) && (status.battery.voltage >= BATTPINOK)) { // if Batt-voltage is below 1V, maybe the resistor is missing.
          if (timeOver(tAct,tBattEmpty,60000)){ //min 60sek. below
            log_i("Batt empty voltage=%d.%dV",status.battery.voltage/1000,status.battery.voltage%1000);
            powerOff(); //power off, when battery is empty !!
          }
        }else{
          tBattEmpty = millis();
        }
      }
    }
    checkExtPowerOff(tAct);
    delay(1);
	}
}
