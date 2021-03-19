
#ifndef LORA_H
#define LORA_H

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <Module.h>

// SEND FRAME RETURN VALUES
#define	TX_OK						0
#define	TX_TX_ONGOING					-1
#define	TX_RX_ONGOING					-2
#define	TX_FSK_ONGOING					-3
#define TX_ERROR					-100

#define RADIO_NULL 0
#define RADIO_SX1262 1
#define RADIO_SX1276 2

#define FSK_PACKET_LENGTH 26

class LoRaClass {
public:
  //LoRaClass(SPIClass *_spi,uint8_t cs, uint8_t irq, uint8_t rst, uint8_t gpio);
  LoRaClass();
  void setPins(SPIClass *_spi,uint8_t cs, uint8_t irq, uint8_t rst, uint8_t gpio = RADIOLIB_NC);
  int16_t begin(float freq = 434.0, float bw = 125.0, uint8_t sf = 9, uint8_t cr = 7, uint8_t syncWord = SX126X_SYNC_WORD_PRIVATE, int8_t power = 10,uint8_t radioChip = RADIO_SX1276);
  int16_t readData(uint8_t* data, size_t len);
  float getRSSI();
  float getSNR();
  int16_t startReceive();
  size_t getPacketLength();
  int16_t setCodingRate(uint8_t cr);
  int16_t transmit(uint8_t* data, size_t len);
  bool isRxMessage();
  int16_t switchFSK();
  int16_t switchLORA();
  float get_airlimit(void);  

private:
	float expectedAirTime_ms(int pktLen);
  void invertba(byte* ba, int len);
  Module *pModule = NULL;
	SX1262 *pSx1262Radio = NULL;
	SX1276 *pSx1276Radio = NULL;
  uint8_t radioType = RADIO_NULL;
  uint8_t _power = 10;
  float _freq = 868.2;
  float _bw;
  uint8_t _sf;
  uint8_t _cr;
  uint8_t _syncWord;
  uint16_t _preambleLength = 8;
  bool _fskMode;
	float sx_airtime = 0.0f;



};
#endif