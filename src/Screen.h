/*!
 * @file Screen.h
 *
 *
 */

#ifndef __SCREEN_H__
#define __SCREEN_H__

#include <Arduino.h>
#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>
#include <GxEPD2_7C.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include "main.h"
#include <icons.h>

#define EINK_BUSY     33
#define EINK_RST      4
#define EINK_DC       23
#define EINK_CS       15
#define EINK_CLK      25
#define EINK_DIN      2

extern struct SettingsData setting;
extern struct statusData status;

class Screen {
public:
  Screen(); //constructor
  bool begin(void);
  void end(void);
  void run(void); //has to be called cyclic

private:
  bool bInit;
  void doInitScreen(void);
};

#endif