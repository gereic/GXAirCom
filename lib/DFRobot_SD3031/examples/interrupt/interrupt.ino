/*!
 * @file interrupt.ino
 * @brief Run this routine, set internal clock and interrupt trigger first, interrupt will be triggered when the set time is reached
 * @copyright    Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license      The MIT License (MIT)
 * @author       [TangJie](jie.tang@dfrobot.com)
 * @version      V1.0.0
 * @date         2022-07-27
 * @url         https://github.com/DFRobot/DFRobot_SD3031
 */

#include "DFRobot_SD3031.h"
volatile  int8_t alarmFlag = 0;
DFRobot_SD3031 rtc;

#define PIN 2
void setup()
{
    Serial.begin(115200);
    /*Wait for the chip to be initialized completely, and then exit*/
    while(rtc.begin() != 0){
        Serial.println("Failed to init chip, please check if the chip connection is fine. ");
        delay(1000);
    }
    rtc.setHourSystem(rtc.e24hours);//Set display format
    rtc.setTime(2022,7,27,23,59,55);//Set default time
    //Countdown timer, schedule alarm and daily regular alarm can't be used at the same time
    //rtc.countDown(3);//Countdown
    //rtc.setAlarm(2022,7,28);//Set schedule alarm
    rtc.setAlarm(rtc.eEveryDay,0,0,0);//Set daily regular alarm

    #if defined(ESP32)||defined(ARDUINO_SAM_ZERO)
      attachInterrupt(digitalPinToInterrupt(D7)/*Query the interrupt number of the D6 pin*/,interrupt,FALLING);
    #elif defined(ESP8266)
      attachInterrupt(digitalPinToInterrupt(D5)/*Query the interrupt number of the D6 pin*/,interrupt,FALLING);
    #else
    /*    The Correspondence Table of AVR Series Arduino Interrupt Pins And Terminal Numbers
    * ---------------------------------------------------------------------------------------
    * |                                        |  DigitalPin  | 2  | 3  |                   |
    * |    Uno, Nano, Mini, other 328-based    |--------------------------------------------|
    * |                                        | Interrupt No | 0  | 1  |                   |
    * |-------------------------------------------------------------------------------------|
    * |                                        |    Pin       | 2  | 3  | 21 | 20 | 19 | 18 |
    * |               Mega2560                 |--------------------------------------------|
    * |                                        | Interrupt No | 0  | 1  | 2  | 3  | 4  | 5  |
    * |-------------------------------------------------------------------------------------|
    * |                                        |    Pin       | 3  | 2  | 0  | 1  | 7  |    |
    * |    Leonardo, other 32u4-based          |--------------------------------------------|
    * |                                        | Interrupt No | 0  | 1  | 2  | 3  | 4  |    |
    * |--------------------------------------------------------------------------------------
    */
    /*                      The Correspondence Table of micro:bit Interrupt Pins And Terminal Numbers
    * ---------------------------------------------------------------------------------------------------------------------------------------------
    * |             micro:bit                       | DigitalPin |P0-P20 can be used as an external interrupt                                     |
    * |  (When using as an external interrupt,      |---------------------------------------------------------------------------------------------|
    * |no need to set it to input mode with pinMode)|Interrupt No|Interrupt number is a pin digital value, such as P0 interrupt number 0, P1 is 1 |
    * |-------------------------------------------------------------------------------------------------------------------------------------------|
    */
    attachInterrupt(digitalPinToInterrupt(2), interrupt, FALLING);
    #endif
}

void loop()
{ 
    sTimeData_t sTime;
    sTime = rtc.getRTCTime();
    Serial.print(sTime.year, DEC);//year
    Serial.print('/');
    Serial.print(sTime.month, DEC);//month
    Serial.print('/');
    Serial.print(sTime.day, DEC);//day
    Serial.print(" (");
    Serial.print(sTime.week);//week
    Serial.print(") ");
    Serial.print(sTime.hour, DEC);//hour
    Serial.print(':');
    Serial.print(sTime.minute, DEC);//minute
    Serial.print(':');
    Serial.print(sTime.second, DEC);//second
    Serial.println(' ');
    /*Enable 12-hour time format*/
    // Serial.print(rtc.getAMorPM());
    // Serial.println();
     if(alarmFlag == 1){
        rtc.clearAlarm();
        alarmFlag = 0;
        Serial.println("Alarm clock is triggered.");
        delay(1000);
        
    }else{
      delay(1000);
    }  
}

void interrupt(void)
{
    alarmFlag = 1;
}
