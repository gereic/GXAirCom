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
}

void gxUserLed::setBlinkFast(uint8_t blinkCount){
  _newState = blink_fast;
  maxBlinkCnt = blinkCount;
}

void gxUserLed::run(){
  if (_pinLed < 0){
    return; //nothing to do --> return;
  }
  uint32_t tAct = millis();
  if (state != showBattPower){
    if (_newState != state){
      state = _newState;      
    }
    
  }
  switch (state)
  {
  case off:
    ledVal = LOW; //LED OFF
    setLed(ledVal);
    //digitalWrite(_pinLed,ledVal); //Led OFF
    break;
  case on:
    ledVal = HIGH; //LED ON
    setLed(ledVal);
    break;
  case blink_05s:
    if (oldState != state){
      ledVal = LOW; //LED OFF
      setLed(ledVal);
      tWait = millis();
    }
    if ((tAct - tWait) >= 500){
      if (ledVal == LOW){ //Led off
        ledVal = HIGH;
      }else{
        ledVal = LOW;
      }
      setLed(ledVal);
      tWait = tAct;
    }
    break;
  case blink_1s:
    if (oldState != state){
      ledVal = LOW; //LED OFF
      setLed(ledVal);
      tWait = millis();
    }
    if ((tAct - tWait) >= 1000){
      if (ledVal == LOW){ //Led off
        ledVal = HIGH;
      }else{
        ledVal = LOW;
      }
      setLed(ledVal);
      tWait = tAct;
    }
    break;
  case blink1:
    if (oldState != state){
      ledVal = LOW; //LED OFF
      setLed(ledVal);
      maxBlinkCnt = 1;
      tWait = millis();
    }
    if (ledVal == LOW){
      if ((tAct - tWait) >= 1000){
        ledVal = HIGH; //LED ON
        setLed(ledVal);
        tWait = millis();
      }
    }else{
      if ((tAct - tWait) >= 200){
        ledVal = LOW; //LED OFF
        setLed(ledVal);
        tWait = millis();
      }
    }
  case blink2: // 1000ms low ... 200ms high ... 200ms low ... 200ms high ... return
    if (oldState != state){
      ledVal = LOW; //LED OFF
      setLed(ledVal);
      blinkCnt = 0;
      maxBlinkCnt = 2;
      tWait = millis();
    }
    if (ledVal == LOW){
      if (((tAct - tWait) >= 1000) && (blinkCnt == 0)){
        ledVal = HIGH; //LED ON
        setLed(ledVal);
        tWait = millis();    
      }else if (((tAct - tWait) >= 200) && (blinkCnt > 0)){
        ledVal = HIGH; //LED ON
        setLed(ledVal);
        tWait = millis(); 
      }
    }else{
      if ((tAct - tWait) >= 200){
        ledVal = LOW; //LED OFF
        setLed(ledVal);
        tWait = millis();
        if (blinkCnt >= (maxBlinkCnt-1)){
          blinkCnt = 0;
        }else{
          blinkCnt++;
        }        
      }
    }
    break;
  case blink_fast:
    if (oldState != state){
      ledVal = LOW; //LED OFF
      setLed(ledVal);
      blinkCnt = 0;
      tWait = millis();
    }
    if (ledVal == LOW){
      if (((tAct - tWait) >= 1000) && (blinkCnt == 0)){
        ledVal = HIGH; //LED ON
        setLed(ledVal);
        tWait = millis();    
      }else if (((tAct - tWait) >= 200) && (blinkCnt > 0)){
        ledVal = HIGH; //LED ON
        setLed(ledVal);
        tWait = millis(); 
      }
    }else{
      if ((tAct - tWait) >= 200){
        ledVal = LOW; //LED OFF
        setLed(ledVal);
        tWait = millis();
        if (blinkCnt >= (maxBlinkCnt-1)){
          blinkCnt = 0;
        }else{
          blinkCnt++;
        }        
      }
    }
    break;
  case showBattPower:
    if (oldState != state){
      ledVal = LOW; //LED OFF
      setLed(ledVal);
      blinkCnt = 0;
      tWait = millis();
    }
    if ((tAct - tWait) >= 1000){
      if (ledVal == LOW){ //Led off
        if (blinkCnt >= _power){
          ledVal = LOW;
          state = off; //switch back to off
        }else{
          ledVal = HIGH;
        }        
      }else{
        ledVal = LOW;
        blinkCnt += 1;
      }
      setLed(ledVal);
      tWait = tAct;
    }
    break;


  default:
    break;
  }
  oldState = state;
}