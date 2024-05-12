# DFRobot_SD3031
* [English Version](./README.md)

SD3031是一款低成本、极其精确的I2C实时时钟(RTC)模块。在整个温度范围和寿命范围内，可提供±5%ppm的精度(误差不超过0.432s)，并兼容2.5-5.5V宽电压范围。电池供电时，该模块的电流消耗低至2uA。该模块可用于测量环境温度，精度为±3℃。


![产品效果图片](./resources/images/DFR0998.png)


## 产品链接（https://www.dfrobot.com.cn）

    SKU：DFR0998

## 目录

  * [概述](#概述)
  * [库安装](#库安装)
  * [方法](#方法)
  * [兼容性](#兼容性)
  * [历史](#历史)
  * [创作者](#创作者)

## 概述

  * 获取实时，初始时间需要用户设定，我们可以获取编译时间，自己设定，最小单位:s 
  * 设置闹钟，用户可以设置闹钟，可以在中断引脚处得到下降沿脉冲触发闹钟
  * 测量芯片温度，误差:±0.5℃

## 库安装

使用此库前，请首先下载库文件，将其粘贴到\Arduino\libraries目录中，然后打开examples文件夹并在该文件夹中运行演示。

## 方法

```C++
  /**
   * @fn begin
   * @brief 初始化传感器
   * @return 返回初始化状态
   */
  uint8_t begin(void);

  /**
   * @fn getRTCTime
   * @brief 获取时钟模块中的年
   * @return 返回获取的年份
   */
  sTimeData_t getRTCTime(void);

  /**
   * @brief 设置时钟是24小时制还是12小时制
   * @param mode 时钟计算方式
   */
  void setHourSystem(eHours_t mode){ _mode = mode; };

  /**
   * @fn setTime
   * @brief 设置时钟时间
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
   * @brief 设置触发报警的数据
   * @param trigger 中断选择
   * @param year 2000~2099
   * @param month 1~12
   * @param day 1~31
   * @param week 0~6
   * @param hour 0~23
   * @param minute 0~59
   * @param second 0~59
   * @return None
   */
  void setAlarmnumber(eTrigger_t trigger, uint16_t year, uint8_t month, uint8_t day,eWeek_t week,uint8_t hour, uint8_t minute, uint8_t second);

  /**
   * @brief 获取时钟内部温度
   * @return 返回获取得温度，单位：℃
   */
  int8_t getTemperatureC(void);

  /**
   * @brief 获取板载电池电压
   * @return float 返回获取得电压
   */
  float getVoltage(void);

  /**
   * @brief 清除报警标志位
   */
  void clearAlarm(void);
  /**
   * @fn getAMorPM
   * @brief 输出上午或下午的时间
   * @return 上午或下午的时间，24小时模式返回空字符串
   */
  String getAMorPM();

  /**
   * @fn enable32k
   * @brief 开启32k频率输出
   * @return 无
   */
  void enable32k();

  /**
   * @fn disable32k
   * @brief 关闭32k输出
   * @return 无
   */
  void disable32k();
  /**
   * @fn writeSRAM
   * @brief 写 SRAM
   * @param addr 0x14~0xFF
   * @param data 写数据
   * @return true 意味着写SRAM是成功的, false 意味着写SRAM是失败的
   */
  uint8_t writeSRAM(uint8_t addr, uint8_t data);

  /**
   * @fn readSRAM
   * @brief 读 SRAM
   * @param addr 0x14~0xFF
   * @return 存储在SRAM中的数据
   */
  uint8_t readSRAM(uint8_t addr);

  /**
   * @fn clearSRAM
   * @brief 清除SRAM
   * @param addr 0x14~0xFF
   * @return true 意味着清除SRAM是成功的, false 意味着清除SRAM是失败的
   */
  uint8_t clearSRAM(uint8_t addr);
  
  /**
   * @fn countDown
   * @brief 倒计时
   * @param second  倒计时时间 0~0xffffff
   */
  void countDown(uint32_t second);
```

## 兼容性

MCU                | Work Well    | Work Wrong   | Untested    | Remarks
------------------ | :----------: | :----------: | :---------: | :----:
Arduino Uno        |      √       |              |             |
Arduino MEGA2560   |      √       |              |             |
Arduino Leonardo   |      √       |              |             |
FireBeetle-ESP8266 |      √       |              |             |
FireBeetle-ESP32   |      √       |              |             |
FireBeetle-M0      |      √       |              |             |
Micro:bit          |      √       |              |             |


## 历史

- 2022/07/29 - 1.0.0 版本

## 创作者

Written by TangJie(jie.tang@dfrobot.com), 2021. (Welcome to our [website](https://www.dfrobot.com/))





