
/*!
 * @file getTime.ino
 * @brief Run this routine, set internal clock first, and then circularly get clock, temperature and voltage data
 * @copyright    Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license      The MIT License (MIT)
 * @author       [TangJie](jie.tang@dfrobot.com)
 * @version      V1.0.0
 * @date         2022-07-27
 * @url         https://github.com/DFRobot/DFRobot_SD3031
 */
#include "DFRobot_SD3031.h"

DFRobot_SD3031 rtc;
void setup()
{
    Serial.begin(115200);
    /*Wait for the chip to be initialized completely, and then exit*/
    while(rtc.begin() != 0){
        Serial.println("Failed to init chip, please check if the chip connection is fine. ");
        delay(1000);
    }
    rtc.setHourSystem(rtc.e24hours);//Set display format
    rtc.setTime(2021,7,27,14,59,0);//Initialize time
    // //Get internal temperature
    // Serial.print(rtc.getTemperatureC());
    // Serial.println(" C");
    // //Get battery voltage
    // Serial.print(rtc.getVoltage());
    // Serial.println(" V");
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
    delay(1000);
}
