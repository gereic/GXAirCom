
#ifndef LORA_H
#define LORA_H

//#define RADIOLIB_DEBUG
//#define RADIOLIB_VERBOSE


#include <Arduino.h>
#include <SPI.h>

// SX126X SPI commands
// operational modes commands
#define SX126X_CMD_NOP                                0x00
#define SX126X_CMD_SET_SLEEP                          0x84
#define SX126X_CMD_SET_STANDBY                        0x80
#define SX126X_CMD_SET_FS                             0xC1
#define SX126X_CMD_SET_TX                             0x83
#define SX126X_CMD_SET_RX                             0x82
#define SX126X_CMD_STOP_TIMER_ON_PREAMBLE             0x9F
#define SX126X_CMD_SET_RX_DUTY_CYCLE                  0x94
#define SX126X_CMD_SET_CAD                            0xC5
#define SX126X_CMD_SET_TX_CONTINUOUS_WAVE             0xD1
#define SX126X_CMD_SET_TX_INFINITE_PREAMBLE           0xD2
#define SX126X_CMD_SET_REGULATOR_MODE                 0x96
#define SX126X_CMD_CALIBRATE                          0x89
#define SX126X_CMD_CALIBRATE_IMAGE                    0x98
#define SX126X_CMD_SET_PA_CONFIG                      0x95
#define SX126X_CMD_SET_RX_TX_FALLBACK_MODE            0x93

// register and buffer access commands
#define SX126X_CMD_WRITE_REGISTER                     0x0D
#define SX126X_CMD_READ_REGISTER                      0x1D
#define SX126X_CMD_WRITE_BUFFER                       0x0E
#define SX126X_CMD_READ_BUFFER                        0x1E


// SX126X SPI command variables
//SX126X_CMD_SET_SLEEP                                                MSB   LSB   DESCRIPTION
#define SX126X_SLEEP_START_COLD                       0b00000000  //  2     2     sleep mode: cold start, configuration is lost (default)
#define SX126X_SLEEP_START_WARM                       0b00000100  //  2     2                 warm start, configuration is retained
#define SX126X_SLEEP_RTC_OFF                          0b00000000  //  0     0     wake on RTC timeout: disabled
#define SX126X_SLEEP_RTC_ON                           0b00000001  //  0     0                          enabled


//SX126X_CMD_SET_DIO_IRQ_PARAMS
#define SX126X_IRQ_TIMEOUT                          0b1000000000  //  9     9     Rx or Tx timeout
#define SX126X_IRQ_CAD_DETECTED                     0b0100000000  //  8     8     channel activity detected
#define SX126X_IRQ_CAD_DONE                         0b0010000000  //  7     7     channel activity detection finished
#define SX126X_IRQ_CRC_ERR                          0b0001000000  //  6     6     wrong CRC received
#define SX126X_IRQ_HEADER_ERR                       0b0000100000  //  5     5     LoRa header CRC error
#define SX126X_IRQ_HEADER_VALID                     0b0000010000  //  4     4     valid LoRa header received
#define SX126X_IRQ_SYNC_WORD_VALID                  0b0000001000  //  3     3     valid sync word detected
#define SX126X_IRQ_PREAMBLE_DETECTED                0b0000000100  //  2     2     preamble detected
#define SX126X_IRQ_RX_DONE                          0b0000000010  //  1     1     packet received
#define SX126X_IRQ_TX_DONE                          0b0000000001  //  0     0     packet transmission completed
#define SX126X_IRQ_ALL                              0b1111111111  //  9     0     all interrupts
#define SX126X_IRQ_NONE                             0b0000000000  //  9     0     no interrupts

//SX126X_CMD_CALIBRATE_IMAGE
#define SX126X_CAL_IMG_430_MHZ_1                      0x6B
#define SX126X_CAL_IMG_430_MHZ_2                      0x6F
#define SX126X_CAL_IMG_470_MHZ_1                      0x75
#define SX126X_CAL_IMG_470_MHZ_2                      0x81
#define SX126X_CAL_IMG_779_MHZ_1                      0xC1
#define SX126X_CAL_IMG_779_MHZ_2                      0xC5
#define SX126X_CAL_IMG_863_MHZ_1                      0xD7
#define SX126X_CAL_IMG_863_MHZ_2                      0xDB
#define SX126X_CAL_IMG_902_MHZ_1                      0xE1
#define SX126X_CAL_IMG_902_MHZ_2                      0xE9

// SX127x series common LoRa registers
#define SX127X_REG_FIFO                               0x00
#define SX127X_REG_OP_MODE                            0x01
#define SX127X_REG_FSKBitrateMsb                      0x02
#define SX127X_REG_FSKBitrateLsb                      0x03
#define SX127X_REG_FSKFdevMsb                         0x04
#define SX127X_REG_FSKFdevLsb                         0x05
#define SX127X_REG_FRF_MSB                            0x06
#define SX127X_REG_FRF_MID                            0x07
#define SX127X_REG_FRF_LSB                            0x08
#define SX127X_REG_PA_CONFIG                          0x09
#define SX127X_REG_PA_RAMP                            0x0A
#define SX127X_REG_OCP                                0x0B
#define SX127X_REG_LNA                                0x0C
#define SX127X_REG_FIFO_ADDR_PTR                      0x0D
#define SX127X_REG_FIFO_TX_BASE_ADDR                  0x0E
#define SX127X_REG_FIFO_RX_BASE_ADDR                  0x0F
#define SX127X_REG_FIFO_RX_CURRENT_ADDR               0x10
#define SX127X_REG_IRQ_FLAGS_MASK                     0x11
#define SX127X_REG_IRQ_FLAGS                          0x12
#define SX127X_REG_RX_NB_BYTES                        0x13
#define SX127X_REG_RX_HEADER_CNT_VALUE_MSB            0x14
#define SX127X_REG_RX_HEADER_CNT_VALUE_LSB            0x15
#define SX127X_REG_RX_PACKET_CNT_VALUE_MSB            0x16
#define SX127X_REG_RX_PACKET_CNT_VALUE_LSB            0x17
#define SX127X_REG_MODEM_STAT                         0x18
#define SX127X_REG_PKT_SNR_VALUE                      0x19
#define SX127X_REG_PKT_RSSI_VALUE                     0x1A
#define SX127X_REG_RSSI_VALUE                         0x1B
#define SX127X_REG_HOP_CHANNEL                        0x1C
#define SX127X_REG_MODEM_CONFIG_1                     0x1D
#define SX127X_REG_MODEM_CONFIG_2                     0x1E
#define SX127X_REG_SYMB_TIMEOUT_LSB                   0x1F
#define SX127X_REG_PREAMBLE_MSB                       0x20
#define SX127X_REG_PREAMBLE_LSB                       0x21
#define SX127X_REG_PAYLOAD_LENGTH                     0x22
#define SX127X_REG_MAX_PAYLOAD_LENGTH                 0x23
#define SX127X_REG_HOP_PERIOD                         0x24
#define SX127X_REG_FIFO_RX_BYTE_ADDR                  0x25
#define SX127X_REG_FSKPreambleMsb                     0x26
#define SX127X_REG_FSKPreambleLsb                     0x26
#define SX127X_REG_FSKSyncConfig                      0x27
#define SX127X_REG_FEI_MSB                            0x28
#define SX127X_REG_FEI_MID                            0x29
#define SX127X_REG_FEI_LSB                            0x2A
#define SX127X_REG_RSSI_WIDEBAND                      0x2C
#define SX127X_REG_FSKPacketConfig1                   0x30
#define SX127X_REG_FSKPacketConfig2                   0x31
#define SX127X_REG_DETECT_OPTIMIZE                    0x31
#define SX127X_REG_FSKPayloadLength                   0x32
#define SX127X_REG_INVERT_IQ                          0x33
#define SX127X_REG_FifoThresh                         0x35
#define SX127X_REG_DETECTION_THRESHOLD                0x37
#define SX127X_REG_SYNC_WORD                          0x39
#define SX127X_REG_FSKImageCal                        0x3B
#define SX127X_REG_DIO_MAPPING_1                      0x40
#define SX127X_REG_DIO_MAPPING_2                      0x41
#define SX127X_REG_VERSION                            0x42
#define SX127X_REG_PaDac                              0x4D
#define SX127X_REG_BitRateFrac                        0x5D

#define SX127X_RF_FIFOTHRESH_TXSTARTCONDITION_FIFONOTEMPTY 0x80

// SX127X_REG_IRQ_FLAGS
#define SX127X_CLEAR_IRQ_FLAG_RX_TIMEOUT              0b10000000  //  7     7     timeout
#define SX127X_CLEAR_IRQ_FLAG_RX_DONE                 0b01000000  //  6     6     packet reception complete
#define SX127X_CLEAR_IRQ_FLAG_PAYLOAD_CRC_ERROR       0b00100000  //  5     5     payload CRC error
#define SX127X_CLEAR_IRQ_FLAG_VALID_HEADER            0b00010000  //  4     4     valid header received
#define SX127X_CLEAR_IRQ_FLAG_TX_DONE                 0b00001000  //  3     3     payload transmission complete
#define SX127X_CLEAR_IRQ_FLAG_CAD_DONE                0b00000100  //  2     2     CAD complete
#define SX127X_CLEAR_IRQ_FLAG_FHSS_CHANGE_CHANNEL     0b00000010  //  1     1     FHSS change channel
#define SX127X_CLEAR_IRQ_FLAG_CAD_DETECTED            0b00000001  //  0     0     valid LoRa signal detected during CAD operation

//SX126X_CMD_GET_STATUS
#define SX126X_STATUS_MODE_STDBY_RC                   0b00100000  //  6     4     current chip mode: STDBY_RC
#define SX126X_STATUS_MODE_STDBY_XOSC                 0b00110000  //  6     4                        STDBY_XOSC
#define SX126X_STATUS_MODE_FS                         0b01000000  //  6     4                        FS
#define SX126X_STATUS_MODE_RX                         0b01010000  //  6     4                        RX
#define SX126X_STATUS_MODE_TX                         0b01100000  //  6     4                        TX
#define SX126X_STATUS_DATA_AVAILABLE                  0b00000100  //  3     1     command status: packet received and data can be retrieved
#define SX126X_STATUS_CMD_TIMEOUT                     0b00000110  //  3     1                     SPI command timed out
#define SX126X_STATUS_CMD_INVALID                     0b00001000  //  3     1                     invalid SPI command
#define SX126X_STATUS_CMD_FAILED                      0b00001010  //  3     1                     SPI command failed to execute
#define SX126X_STATUS_TX_DONE                         0b00001100  //  3     1                     packet transmission done
#define SX126X_STATUS_SPI_FAILED                      0b11111111  //  7     0     SPI transaction failed

// SX127X_REG_IRQ_FLAGS_1
#define SX127X_FLAG_MODE_READY                        0b10000000  //  7     7     requested mode is ready
#define SX127X_FLAG_RX_READY                          0b01000000  //  6     6     reception ready (after RSSI, AGC, AFC)
#define SX127X_FLAG_TX_READY                          0b00100000  //  5     5     transmission ready (after PA ramp-up)
#define SX127X_FLAG_PLL_LOCK                          0b00010000  //  4     4     PLL locked
#define SX127X_FLAG_RSSI                              0b00001000  //  3     3     RSSI value exceeds RSSI threshold
#define SX127X_FLAG_TIMEOUT                           0b00000100  //  2     2     timeout occurred
#define SX127X_FLAG_PREAMBLE_DETECT                   0b00000010  //  1     1     valid preamble was detected
#define SX127X_FLAG_SYNC_ADDRESS_MATCH                0b00000001  //  0     0     sync address matched

// SX127X FSK ImageCal defines
#define SX127X_RF_IMAGECAL_IMAGECAL_START      0x40
#define SX127X_RF_IMAGECAL_IMAGECAL_RUNNING    0x20

//SX127X_IRQ FLAGS Lora
#define SX127X_IRQ_RX_TIMEOUT					0x80
#define SX127X_IRQ_RX_DONE					0x40
#define SX127X_IRQ_PAYLOAD_CRC_ERROR				0x20
#define SX127X_IRQ_VALID_HEADER				0x10
#define SX127X_IRQ_TX_DONE					0x08
#define SX127X_IRQ_CAD_DONE					0x04
#define SX127X_IRQ_FHSS_CHANGE_CHANNEL				0x02
#define SX127X_IRQ_CAD_DETECTED				0x01

#define	SX127X_LORA_SLEEP_MODE					0x80
#define SX127X_LORA_TX_MODE					0x83
#define SX127X_LORA_RXCONT_MODE				0x85
#define SX127X_LORA_RXSINGLE_MODE				0x86
#define SX127X_LORA_MODE_MASK 0x87

#include "GxModule.h"

#define LORAMAXSTRING 500



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
  void setPins(SPIClass *spi,uint8_t cs, uint8_t irq, uint8_t rst, uint8_t gpio = GXMODULE_NC);
  int16_t begin(float bw = 125.0, uint8_t sf = 9, uint8_t cr = 7, uint8_t syncWord = 0x12, int8_t power = 10,uint8_t radioChip = RADIO_SX1276);
  int16_t readData(uint8_t* data, size_t len);
  float getRSSI();
  int16_t startReceive();
  size_t getPacketLength();
  bool isReceiving();
  int16_t setCodingRate(uint8_t cr);
  int16_t transmit(uint8_t* data, size_t len);
  bool isRxMessage();
  int16_t switchFSK(uint32_t frequency);
  int16_t switchLORA(uint32_t frequency,uint16_t loraBandwidth);
  float get_airlimit(void);
  bool isFskMode(void);
  
  //int16_t setFrequency(float frequency);
  uint8_t gain = 0; //0 --> auto-gain, 1--> highest gain; 6 --> lowest gain
  int8_t maxLoraPower = 14;
  int8_t maxFskPower = 14;
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
  GxModule *pGxModule = NULL;
  SPIClass *_spi = NULL;
  uint8_t radioType = RADIO_NULL;
  uint8_t _power = 10;
  uint32_t _freq = 868200000;
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
  int16_t sx1262_standby(uint8_t mode = 0x00);
  uint32_t sx1262GetPacketStatus();
  int16_t sx1262CalibrateImage();
  void sx1262CheckAndClearErrors();
  uint16_t sx1262GetDeviceErrors();
  int16_t sx1262ClearDeviceErrors();
  int16_t sx1262ClearIrqStatus();
  int16_t sx1262ClearIrqFlags();
  int16_t sx1262Transmit(uint8_t* data, size_t len, uint8_t addr = 0);
  uint32_t sx1262GetTimeOnAir(size_t len);
  int16_t sx1262SetFrequency(uint32_t freq);
  uint8_t sx1262GetStatus();
  int16_t sx1262GetStats();
  float getSNR();
  int16_t sx1262SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask = SX126X_IRQ_NONE, uint16_t dio3Mask = SX126X_IRQ_NONE);
  int16_t sx1262SetBufferBaseAddress(uint8_t txBaseAddress = 0x00, uint8_t rxBaseAddress = 0x00);
  int16_t sx1262ReadData(uint8_t* buffer, size_t len);
  uint16_t sx1262ReadIrQ();
  void sx1262SetCmdTx();  
  void sx1262SetCmdRx();  
  int16_t writeRegister(uint16_t addr, uint8_t* data, uint8_t numBytes);
  int16_t readRegister(uint16_t addr, uint8_t* data, uint8_t numBytes);
  int16_t SPIwriteCommand(uint8_t cmd, uint8_t* data, uint8_t numBytes, bool waitForBusy = true);
  int16_t SPIwriteCommand(uint8_t* cmd, uint8_t cmdLen, uint8_t* data, uint8_t numBytes, bool waitForBusy = true);
  int16_t SPIreadCommand(uint8_t cmd, uint8_t* data, uint8_t numBytes, bool waitForBusy = true);
  int16_t SPIreadCommand(uint8_t* cmd, uint8_t cmdLen, uint8_t* data, uint8_t numBytes, bool waitForBusy = true);
  int16_t SPItransfer(uint8_t* cmd, uint8_t cmdLen, bool write, uint8_t* dataOut, uint8_t* dataIn, uint8_t numBytes, bool waitForBusy, uint32_t timeout = 5000);
  void checkRet(int16_t value);
  int sx_channel_free4tx();
  void configChannel (uint32_t frequency);
  void sx1276_setPower(int8_t power);
  bool calibrated = false;

};
#endif