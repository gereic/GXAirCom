/*!
 * @file gxUserLed.cpp
 *
 *
 */

#include "gxUserLed.h"

gxUserLed::gxUserLed(){

}

void gxUserLed::setLed(uint8_t val){
  if (_bHighActive){
    digitalWrite(_pinLed,val);
  }else{
    digitalWrite(_pinLed,(!val) & 0x01);
  }
}

void gxUserLed::resetLed(void){
  ledVal = LOW; //LED OFF
  setLed(ledVal);
  tWait = millis();
  blinkCnt = 0;
  bReady = false;
}

bool gxUserLed::blinkLed(uint8_t blinkCount,uint32_t tOn, uint32_t tOff,uint32_t _tWait){
  uint32_t tAct = millis();
  if (ledVal == LOW){
    if (((tAct - tWait) >= _tWait) && (blinkCnt == 0)){
      ledVal = HIGH; //LED ON
      setLed(ledVal);
      tWait = millis();    
    }else if (((tAct - tWait) >= tOff) && (blinkCnt > 0)){
      ledVal = HIGH; //LED ON
      setLed(ledVal);
      tWait = millis(); 
    }
  }else{
    if ((tAct - tWait) >= tOn){
      ledVal = LOW; //LED OFF
      setLed(ledVal);
      tWait = millis();
      if (blinkCnt >= (blinkCount-1)){
        blinkCnt = 0;
        return true; //ready !!
      }else{
        blinkCnt++;
      }        
    }
  }
  return false;
  
}

void gxUserLed::setUserLed(int8_t pinLed,bool bHighActive){
  _pinLed = pinLed;
  _bHighActive = bHighActive;
  if (_pinLed >= 0){
    pinMode(_pinLed, OUTPUT);
    ledVal = LOW;
    setLed(ledVal);
  }
}

void gxUserLed::setBattPower(uint8_t power){
  _power = constrain(power,0,100);
  _power /= 20; //only 1 to 5
}

void gxUserLed::setState(ledState newState){
  _newState = newState;
  if (_newState == off){
    resetLed();
    bReady = true;
  }
}

void gxUserLed::setBlinkFast(uint8_t blinkCount){
  _newState = blink_fast;
  maxBlinkCnt = blinkCount;
}

gxUserLed::ledState gxUserLed::getState(void){
  return state;
}

void gxUserLed::run(){
  if (_pinLed < 0){
    return; //nothing to do --> return;
  }
  if (bReady){
  //if ((state != showBattPower) && (state != showWifiEn) && (state != showWifiDis)){
    if (_newState != state){
      state = _newState;      
    }    
  }
  switch (state)
  {
  case off:    
    ledVal = LOW; //LED OFF
    setLed(ledVal);
    bReady = true;
    break;
  case on:
    ledVal = HIGH; //LED ON
    setLed(ledVal);
    bReady = true;
    break;
  case blink_05s:
    if (oldState != state){
      resetLed();
    }
    blinkLed(0,500,500,0);
    bReady = true;
    break;
  case blink_1s:
    if (oldState != state){
      resetLed();
    }
    blinkLed(0,1000,1000,0);
    bReady = true;
    break;
  case blink_fast:
    if (oldState != state){
      resetLed();
    }
    blinkLed(maxBlinkCnt,200,200,1000);
    bReady = true;
    break;
  case showBattPower:
    if (oldState != state){
      resetLed();
    }
    if (blinkLed(_power,1000,1000,200)){
      bReady = true;
      state = off; //switch back to off
    } 
    break;
  case showWifiDis:
    if (oldState != state){
      resetLed();
    }
    if (blinkLed(5,100,100,0)){
      bReady = true;
      state = off; //switch back to off
    }
    break;
  case showWifiEn:
    if (oldState != state){
      resetLed();
    }
    if (blinkLed(1,3000,0,0)){
      bReady = true;
      state = off; //switch back to off      
    }
    break;



  default:
    break;
  }
  oldState = state;
}