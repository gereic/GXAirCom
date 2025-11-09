#ifndef __TFT_H__
#define __TFT_H__

#include <Arduino.h>
#include "enums.h"
#include "main.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

extern struct SettingsData setting;
extern struct statusData status;

// TFT Display dimensions for Wireless Tracker
#define TFT_WIDTH  160
#define TFT_HEIGHT 80

class Tft {
public:
  Tft(); //constructor
  bool begin(int8_t cs, int8_t dc, int8_t rst, int8_t mosi, int8_t sck, int8_t bl);
  void end(void);
  void run(void); //has to be called cyclic
  void webUpdate(void);

private:
  void PowerOn(void);
  void PowerOff(void);
  
  // Page display functions
  void printGPSData(uint32_t tAct, bool forceRedraw = false);      // Page 0: Main GPS data
  void printSystemInfo(uint32_t tAct, bool forceRedraw = false);   // Page 1: System info
  void printDetailedGPS(uint32_t tAct, bool forceRedraw = false);  // Page 2: Detailed GPS data
  
  // Helper drawing functions
  void drawSatCount(int16_t x, int16_t y, uint8_t value);
  void drawBatt(int16_t x, int16_t y, uint8_t value);
  void drawWifiStat(eConnectionState wifiStat);
  void drawBluetooth(int16_t x, int16_t y, bool connected);
  void drawFlying(int16_t x, int16_t y, bool isFlying);
  String setStringSize(String s, uint8_t sLen);
  
  Adafruit_ST7735 *display = NULL;
  SPIClass *tftSPI = NULL;
  int8_t pinCS = -1;
  int8_t pinDC = -1;
  int8_t pinRst = -1;
  int8_t pinBL = -1;
  bool bDisplayOn = false;
  uint8_t oldScreenNumber = 0;
};

#endif
