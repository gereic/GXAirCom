
#ifndef LORA_H
#define LORA_H

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <Module.h>
#include "GxModule.h"

#define USE_GXMODULE
//#define USE_GXMODULESX1262

// SEND FRAME RETURN VALUES
#define	TX_OK						0
#define	TX_TX_ONGOING					-1
#define	TX_RX_ONGOING					-2
#define	TX_FSK_ONGOING					-3
#define TX_ERROR					-100

#define RADIO_NULL 0
#define RADIO_SX1262 1
#define RADIO_SX1276 2

//SX126X_CMD_SET_STANDBY
#define SX126X_STANDBY_RC                             0x00        //  7     0     standby mode: 13 MHz RC oscillator
#define SX126X_STANDBY_XOSC                           0x01        //  7     0                   32 MHz crystal oscillator

#define FSK_PACKET_LENGTH 26

#define FSK_FREQUENCY 868.2

#define LORA_RX_DEBUG 0

class LoRaClass {
public:
  //LoRaClass(SPIClass *_spi,uint8_t cs, uint8_t irq, uint8_t rst, uint8_t gpio);
  LoRaClass();
  void setPins(SPIClass *spi,uint8_t cs, uint8_t irq, uint8_t rst, uint8_t gpio = RADIOLIB_NC);
  int16_t begin(float freq = 434.0, float bw = 125.0, uint8_t sf = 9, uint8_t cr = 7, uint8_t syncWord = SX126X_SYNC_WORD_PRIVATE, int8_t power = 10,uint8_t radioChip = RADIO_SX1276);
  int16_t readData(uint8_t* data, size_t len);
  float getRSSI();
  float getSNR();
  int16_t startReceive();
  size_t getPacketLength();
  bool isReceiving();
  int16_t setCodingRate(uint8_t cr);
  int16_t transmit(uint8_t* data, size_t len);
  bool isRxMessage();
  int16_t switchFSK(float frequency);
  int16_t switchLORA(float frequency);
  float get_airlimit(void);
  
  //int16_t setFrequency(float frequency);
  uint8_t gain = 0; //0 --> auto-gain, 1--> highest gain; 6 --> lowest gain
  void run(); //has to be called cyclic
  void end();

private:
  enum tActMode : uint8_t
  {
    UNKNOWN = 0,
    LORA = 1,
    FSK = 2,
  };
  float expectedAirTime_ms(int pktLen);
  void printReg(uint8_t reg);
  float rssiValue = 0.0;
  void invertba(byte* ba, int len);
  int16_t sx1276setOpMode(uint8_t mode);
  int16_t sx1276setRxBandwidth(float rxBw);
  Module *pModule = NULL;
  GxModule *pGxModule = NULL;
  #ifndef USE_GXMODULESX1262
	SX1262 *pSx1262Radio = NULL;
  #endif
  #ifndef USE_GXMODULE
	SX1276 *pSx1276Radio = NULL;
  #endif
  SPIClass *_spi = NULL;
  uint8_t radioType = RADIO_NULL;
  uint8_t _power = 10;
  float _freq = 868.2;
  float _bw;
  uint8_t _cs;
  uint8_t _irq;
  uint8_t _rst;
  uint8_t _gpio;
  uint8_t _sf;
  uint8_t _cr;
  float _br = 100.0;
  uint8_t _syncWord;
  uint16_t _preambleLength = 8;
  bool _fskMode;
  tActMode _actMode;
	float sx_airtime = 0.0f;
  uint8_t rxCount = 0;
  bool bCalibrated = false;
  uint8_t prevIrqFlags;
  int16_t sx1262_standby(uint8_t mode = SX126X_STANDBY_RC);
  uint32_t sx1262GetPacketStatus();
  int16_t sx1262CalibrateImage();
  int16_t writeRegister(uint16_t addr, uint8_t* data, uint8_t numBytes);
  int16_t readRegister(uint16_t addr, uint8_t* data, uint8_t numBytes);
  int16_t SPIwriteCommand(uint8_t cmd, uint8_t* data, uint8_t numBytes, bool waitForBusy = true);
  int16_t SPIwriteCommand(uint8_t* cmd, uint8_t cmdLen, uint8_t* data, uint8_t numBytes, bool waitForBusy = true);
  int16_t SPIreadCommand(uint8_t cmd, uint8_t* data, uint8_t numBytes, bool waitForBusy = true);
  int16_t SPIreadCommand(uint8_t* cmd, uint8_t cmdLen, uint8_t* data, uint8_t numBytes, bool waitForBusy = true);
  int16_t SPItransfer(uint8_t* cmd, uint8_t cmdLen, bool write, uint8_t* dataOut, uint8_t* dataIn, uint8_t numBytes, bool waitForBusy, uint32_t timeout = 5000);


};
#endif