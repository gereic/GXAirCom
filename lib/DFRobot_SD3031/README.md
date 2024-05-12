# DFRobot_SD3031
* [中文版](./README_CN.md)

The SD3031 is a low-cost, extremely accurate I2C real-time clock(RTC) module. It can provide ±5%ppm accuracy (no more than 0.432s error) within the whole temperature range and lifespan, and is compatible with 2.5-5.5V wide voltage range. The current consumption of the module is only 2µA when powered by batteries. This module also can be used to measure ambient temperature with ±3℃ accuracy.


![产品效果图片](./resources/images/DFR0998.png)


## Product Link（https://www.dfrobot.com）

    SKU：DFR0998

## Table of Contents

  * [Summary](#Summary)
  * [Installation](#Installation)
  * [Methods](#Methods)
  * [Compatibility](#Compatibility)
  * [History](#History)
  * [Credits](#Credits)

## Summary

  * Get real time data. The initial time needs to be set by users. Users can set to get the compiling time. Minimum unit: s 
  * Alarm clock. Users can set alarm time and get falling-edge pulse at interrupt pin to trigger the alarm clock
  * Measure the chip temperature, error: ±0.5℃

## Installation

To use this library, first download the library file, paste it into the \Arduino\libraries directory, then open the examples folder and run the demo in the folder.

## Methods

```C++
  /**
   * @fn begin
   * @brief Initialize sensor
   * @return Return init status
   */
  uint8_t begin(void);

  /**
   * @fn getRTCTime
   * @brief Get information of year in RTC module
   * @return Return the obtained year
   */
  sTimeData_t getRTCTime(void);

  /**
   * @brief Set clock as 24-hour or 12-hour format
   * @param mode Clock time format
   */
  void setHourSystem(eHours_t mode){ _mode = mode; };

  /**
   * @fn setTime
   * @brief Set time into RTC and take effect immediately
   * @param year 2000~2099
   * @param month 1~12
   * @param day 1~31
   * @param week 0~6
   * @param hour 0~23
   * @param minute 0~59
   * @param second 0~59
   * @return None
   */
  void setTime(uint16_t year, uint8_t month, uint8_t day,eWeek_t week,uint8_t hour, uint8_t minute, uint8_t second);


  /**
   * @fn setAlarmnumber
   * @brief Set the data for triggering alarm
   * @param trigger Interrupt select
   * @param year 2000-2099
   * @param month 1-12
   * @param day 1-31
   * @param week 0-6
   * @param hour 0-23
   * @param minute 0-59
   * @param second 0-59
   * @return None
   */
  void setAlarmnumber(eTrigger_t trigger, uint16_t year, uint8_t month, uint8_t day,eWeek_t week,uint8_t hour, uint8_t minute, uint8_t second);

  /**
   * @brief Get internal temperature of the clock
   * @return Return the obtained temperature, unit: ℃
   */
  int8_t getTemperatureC(void);

  /**
   * @brief Get voltage of onboard battery
   * @return float Return the obtained voltage
   */
  float getVoltage(void);

  /**
   * @brief Clear alarm flag bit
   */
  void clearAlarm(void);
  /**
   * @fn getAMorPM
   * @brief Output AM or PM of time
   * @return AM or PM, return empty string for 24 hours mode
   */
  String getAMorPM();

  /**
   * @fn enable32k
   * @brief Enable 32k frequency output
   * @return None
   */
  void enable32k();

  /**
   * @fn disable32k
   * @brief Disable 32k frequency output
   * @return None
   */
  void disable32k();
  /**
   * @fn writeSRAM
   * @brief Write SRAM
   * @param addr 0x14-0xFF
   * @param data Write data
   * @return true indicates writing SRAM succeeded, false indicates writing SRAM failed
   */
  uint8_t writeSRAM(uint8_t addr, uint8_t data);

  /**
   * @fn readSRAM
   * @brief Read SRAM
   * @param addr 0x14~0xFF
   * @return Data stored in SRAM
   */
  uint8_t readSRAM(uint8_t addr);

  /**
   * @fn clearSRAM
   * @brief Clear SRAM
   * @param addr 0x14~0xFF
   * @return true indicates clearing SRAM succeeded, false indicates clearing SRAM failed
   */
  uint8_t clearSRAM(uint8_t addr);
  
  /**
   * @fn countDown
   * @brief Countdown
   * @param second  countdown time 0-0xffffff
   */
  void countDown(uint32_t second);
```

## Compatibility

MCU                | Work Well    | Work Wrong   | Untested    | Remarks
------------------ | :----------: | :----------: | :---------: | :----:
Arduino Uno        |      √       |              |             |
Arduino MEGA2560   |      √       |              |             |
Arduino Leonardo   |      √       |              |             |
FireBeetle-ESP8266 |      √       |              |             |
FireBeetle-ESP32   |      √       |              |             |
FireBeetle-M0      |      √       |              |             |
Micro:bit          |      √       |              |             |


## History

- 2022/07/29 - 1.0.0 version

## Credits

Written by TangJie(jie.tang@dfrobot.com), 2021. (Welcome to our [website](https://www.dfrobot.com/))





