/*!
 * @file gxUserLed.h
 *
 *
 */

#ifndef __USERLED_H__
#define __USERLED_H__

#include <Arduino.h>
#include <inttypes.h>

class gxUserLed {
public:
  enum ledState : uint8_t
  {
    off = 0,
    on = 1,
    blink1 = 11,
    blink2 = 12,
    blink_05s = 50,
    blink_1s = 60,
    blink_fast = 70,
    showBattPower = 10,
  };
  gxUserLed(); //constructor
  void setUserLed(int8_t pinLed,bool bHighActive);
  void setBattPower(uint8_t power);
  void setState(ledState newState);
  void setBlinkFast(uint8_t blinkCount);
  void run();
private:
  void setLed(uint8_t val);
  bool _bHighActive;
  int8_t _pinLed = -1;
  uint8_t _power = 0;
  ledState state = off;
  ledState _newState = off;
  ledState oldState = off;
  uint32_t tWait = 0;
  uint8_t blinkCnt = 0;
  uint8_t maxBlinkCnt = 0;
  uint8_t ledVal = 0;
};

#endif
