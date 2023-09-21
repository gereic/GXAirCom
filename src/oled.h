#include "FanetLora.h"

extern struct SettingsData setting;
extern struct statusData status;
extern FanetLora fanet;

#ifndef __OLED_H__
#define __OLED_H__

#include <Arduino.h>
#include "enums.h"
#include "main.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#ifdef SH1106G
#include <Adafruit_SH110X.h>
#define BLACK 0
#define WHITE 1
#else
#include <Adafruit_SSD1306.h>
#endif




class Oled {
public:
  Oled(); //constructor
  bool begin(TwoWire *pi2c,int8_t rst,SemaphoreHandle_t *_xMutex);
  void end(void);
  void run(void); //has to be called cyclic
  void webUpdate(void);

private:
  
  void PowerOn(void);
  void PowerOff(void);
  void drawWifiStat(eConnectionState wifiStat);
  void printGPSData(uint32_t tAct);
  String setStringSize(String s,uint8_t sLen);
  void drawAircraftType(int16_t x, int16_t y, uint8_t AircraftType);
  void drawSatCount(int16_t x, int16_t y,uint8_t value);
  void drawspeaker(int16_t x, int16_t y);
  void drawflying(int16_t x, int16_t y, bool flying);
  void drawBluetoothstat(int16_t x, int16_t y);
  void drawBatt(int16_t x, int16_t y,uint8_t value);
  void DrawRadarScreen(uint32_t tAct,eRadarDispMode mode);
  void DrawAngleLine(int16_t x,int16_t y,int16_t length,float deg);
  void DrawRadarPilot(uint8_t neighborIndex);
  void printWeather(uint32_t tAct);
  void drawSignal(int16_t x, int16_t y,uint8_t strength);
  void printScanning(uint32_t tAct);
  void printGSData(uint32_t tAct);
  #ifdef SH1106G
  Adafruit_SH1106G *display = NULL;
  #else
  Adafruit_SSD1306 *display = NULL;
  #endif
  int8_t pinRst = -1;
  bool bDisplayOn = false;
  uint8_t oldScreenNumber = 0;
  SemaphoreHandle_t *xMutex;
};

#endif