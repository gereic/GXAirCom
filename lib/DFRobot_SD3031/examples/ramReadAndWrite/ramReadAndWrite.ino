/*!
 * @file ramReadAndWrite.ino
 * @brief Run this routine to read and write RAM data in the RTC module
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
    uint8_t data= 0;
    Serial.begin(115200);
    /*Wait for the chip to be initialized completely, and then exit*/
    while(rtc.begin() != 0){
        Serial.println("Failed to init chip, please check if the chip connection is fine. ");
        delay(1000);
    }
    rtc.writeSRAM(0x2D,2);//Address Range 0x2c~0x71
    delay(1000);
    data = rtc.readSRAM(0x2D);
    Serial.print("data:");
    Serial.println(data);
    delay(100);
    rtc.clearSRAM(0x2D);
    delay(100);
    data = rtc.readSRAM(0x2D);
    Serial.print("data:");
    Serial.println(data);
}

void loop()
{
    delay(1000);
}
