/*!
 * @file Screen.h
 *
 *
 */

extern struct SettingsData setting;
extern struct statusData status;



#ifndef __SCREEN_H__
#define __SCREEN_H__

#include <Arduino.h>
#include <GxEPD2_BW.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold24pt7b.h>
#include "Fonts/NotoSans6pt7b.h"
#include "Fonts/NotoSansBold6pt7b.h"
#include "Fonts/gnuvarioe14pt7b.h"
#include "Fonts/gnuvarioe18pt7b.h"
#include "Fonts/gnuvarioe23pt7b.h"
#include "main.h"
#include <string.h>
#include "../lib/FANETLORA/FanetLora.h"

/*
#define EINK_BUSY     33
#define EINK_RST      0
#define EINK_DC       32
#define EINK_CS       15
#define EINK_CLK      4
#define EINK_DIN      2
*/

#define EINK_FULL_UPDATE 300000 //every 5min do a full-update because of ghosting

class Screen {
public:
  Screen(); //constructor
  bool begin(uint8_t type,int8_t cs,int8_t dc,int8_t rst,int8_t busy,int8_t clk, int8_t din);
  void end(void);
  void run(void); //has to be called cyclic
  void webUpdate(void);

private:
  bool bInit;
  void doInitScreen(void);
  void drawMainScreen(void);
  void drawWeatherScreen(void);
  void drawFlightTime(int16_t x, int16_t y, int16_t width, int16_t height,uint32_t tTime);
  void drawValue(int16_t x, int16_t y, int16_t width, int16_t height,float value,unsigned int decimals);
  void drawCompass(int16_t x, int16_t y, int16_t width, int16_t height,float value);
  void drawBatt(int16_t x, int16_t y, int16_t width, int16_t height,uint8_t value);
  void drawSatCount(int16_t x, int16_t y, int16_t width, int16_t height,uint8_t value);
  void getTextPositions(int16_t *posx, int16_t *posy,int16_t x, int16_t y, int16_t width, int16_t height,String sText);
  void drawspeaker(int16_t x, int16_t y, int16_t width, int16_t height,uint8_t volume);
  void drawflying(int16_t x, int16_t y, int16_t width, int16_t height,bool flying);
  String getWDir(float dir);
  uint8_t stepCount;
  GxEPD2_GFX_BASE_CLASS *pEInk = NULL;
  uint32_t countFullRefresh = 0;
  struct screenMainData{
    uint8_t battPercent;
    float alt;
    float vario;
    float speed;
    float compass;
    uint8_t SatCount;
    uint32_t flightTime;
    uint8_t volume; //muting beeper
    bool flying;
    bool wifi;
    uint8_t bluetooth;
  };
  //GxEPD2_EPD *pDisplay;

};

#endif