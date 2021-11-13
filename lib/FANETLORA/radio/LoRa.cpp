// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "LoRa.h"
#include "manchester.h"


// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// disable interrupt when it's not needed
volatile bool enableInterrupt = true;

void IRAM_ATTR setFlag(void)
{
    // check if the interrupt is enabled
    if (!enableInterrupt) {
        return;
    }

    // we got a packet, set the flag
    receivedFlag = true;
}

void LoRaClass::run(){
  static uint32_t tLast = millis();
  uint32_t tAct = millis();
  if ((radioType == RADIO_SX1276) && (_fskMode)){
    uint8_t irqFlags = pGxModule->SPIgetRegValue(0x3E); //SX127X_REG_IRQ_FLAGS_1
    irqFlags &= SX127X_FLAG_SYNC_ADDRESS_MATCH;
    if (prevIrqFlags != irqFlags){
      prevIrqFlags = irqFlags;
      if (irqFlags){
        #ifdef USE_GXMODULE
        rssiValue = (float)pGxModule->SPIgetRegValue(0x11) / -2.0; //REG_RSSI_VALUE_FSK
        #else
        rssiValue = pSx1276Radio->getRSSI();
        #endif
        //log_i("rssiValue=%.02f %d",rssiValue,regValue);
      }
    }
  }
  if ((tAct - tLast) >= 100){ //check every 100ms
    tLast = tAct;
    switch (radioType){
      case RADIO_SX1262:
        break;
      case RADIO_SX1276:
        if (_fskMode){
          if (bCalibrated){
            uint8_t ret = pGxModule->SPIgetRegValue(0x3B); //SX127X_REG_IMAGE_CAL
            if (ret != 0) log_i("REG_IMAGE_CAL=%d",ret);
            if (ret & 0x08){
              bCalibrated = false; //we calibrate the image on next start receive
            }
          }
        }
        break;
    }
  }
}

bool LoRaClass::isRxMessage(){
  if (receivedFlag){
    rxCount++;
	  #if LORA_RX_DEBUG > 10
    log_i("new message arrived %d",rxCount);
    #endif
    enableInterrupt = false;
    receivedFlag = false;
    return true;
  }
  return false;

}


LoRaClass::LoRaClass(){
  #ifndef USE_GXMODULE
  pSx1276Radio = NULL;
  #endif
  pSx1262Radio = NULL;
}

//LoRaClass::LoRaClass(SPIClass *_spi,uint8_t cs, uint8_t irq, uint8_t rst, uint8_t gpio){
void LoRaClass::setPins(SPIClass *spi,uint8_t cs, uint8_t irq, uint8_t rst, uint8_t gpio){
  //log_i("cs=%d,irq=%d,rst=%d,gpio=%d",cs, irq, rst, gpio);
  //SPISettings _spiSettings(8E6, MSBFIRST, SPI_MODE0);
  SPISettings _spiSettings = SPISettings(8000000, MSBFIRST, SPI_MODE0);
  _spi = spi;
  _cs = cs;
  _irq = irq;
  _rst = rst;
  _gpio = gpio;
  _actMode = UNKNOWN;
  pModule = new Module(cs, irq, rst, gpio,*_spi,_spiSettings);
  pGxModule = new GxModule(cs, irq, rst, gpio,*_spi,_spiSettings);
  #ifndef USE_GXMODULE
  pSx1276Radio = NULL;
  #endif
  pSx1262Radio = NULL;
}
int16_t LoRaClass::begin(float freq, float bw, uint8_t sf, uint8_t cr, uint8_t syncWord, int8_t power,uint8_t radioChip){
	receivedFlag = false;
	//enableInterrupt = false;
  bCalibrated = false;
  radioType = radioChip;
  int state = 0;
  _freq = freq;
  _bw = bw;
  _sf = sf;
  _cr = cr;
  _br = 100.0;
  _syncWord = syncWord;
  _power = uint8_t(power);
  _fskMode = false;
  _actMode = UNKNOWN;
  //delay(1500); //wait until Hardware is stable
  //check which radio
  switch (radioType){
    case RADIO_SX1262:
      pSx1262Radio = new SX1262(pModule);
      state = pSx1262Radio->begin(_freq,_bw,_sf,_cr,_syncWord,_power,_preambleLength);
      if (state == ERR_NONE) {
        log_i("SX1262 Radio found !");
        pSx1262Radio->setDio1Action(setFlag);	
        pSx1262Radio->standby();
        //uint8_t rxGain = 0x96; //Rx Boosted gain
        //pModule->SPIwriteRegister(SX126X_REG_RX_GAIN, &rxGain);
        //uint8_t cmd[] = { SX126X_CMD_WRITE_REGISTER, (uint8_t)((SX126X_REG_RX_GAIN >> 8) & 0xFF), (uint8_t)(SX126X_REG_RX_GAIN & 0xFF) };
        //pModule->SPItransfer(cmd, 3, &rxGain, 1));


        return state;
      } else {
          log_i("failed, code %d",state);
          return state;
      }
      break;
    case RADIO_SX1276:
      #ifdef USE_GXMODULE
      pGxModule->init(RADIOLIB_USE_SPI); //we use SPI
      GxModule::pinMode(pGxModule->getIrq(), INPUT); //set IRQ as input
      GxModule::pinMode(pGxModule->getGpio(), INPUT); //set GPIO as input
      // run the reset sequence
      GxModule::pinMode(pGxModule->getRst(), OUTPUT);
      GxModule::digitalWrite(pGxModule->getRst(), LOW);
      GxModule::delayMicroseconds(200);
      GxModule::digitalWrite(pGxModule->getRst(), HIGH);
      GxModule::delay(6); //wait min 5 ms until module is ready
      GxModule::attachInterrupt(digitalPinToInterrupt(pGxModule->getIrq()), setFlag, RISING);
      return state;
      #else
      pSx1276Radio = new SX1276(pModule);
      //log_i("freq=%f,bw=%f,sf=%d,cr=%d",freq,bw,sf,cr);

      state = pSx1276Radio->begin(freq,bw,sf,cr,syncWord,power,_preambleLength);
      if (state == ERR_NONE) {
        log_i("SX1276 Radio found !");
        pSx1276Radio->setDio0Action(setFlag);	
        //calib image
        pSx1276Radio->setActiveModem(SX127X_FSK_OOK);
        pModule->SPIsetRegValue(SX127X_REG_IMAGE_CAL, 0x00); //set Temperature monitoring on 5°C#
        pSx1276Radio->setActiveModem(SX127X_LORA);
        return state;
      } else {
          log_e("failed, code %d",state);
          return state;
      }
      #endif
      break;
  }
  return 0;
}

int16_t LoRaClass::readData(uint8_t* data, size_t len){
  int16_t ret = 0;
  switch (radioType){
    case RADIO_SX1262:
      if (_fskMode){
        if (len == 26){
          uint8_t rx_frame[len*2];
          int16_t ret = pSx1262Radio->readData(rx_frame,len*2);
          uint8_t val1, val2;
          for (int i = 0;i < len * 2; i++){
            val1 = ManchesterDecode[rx_frame[i]];
            val2 = ManchesterDecode[rx_frame[i+1]];
            data[i>>1] = ((val1 & 0x0F) << 4) | (val2 & 0x0F);
            //data[i>>1] = ~data[i>>1];
            i++;
          }
          return ret;
        }else{
          return ERR_UNKNOWN;
        }
      }else{
        return pSx1262Radio->readData(data,len);
      }
      
    case RADIO_SX1276:

      if (_fskMode){
        // put module to standby
        //we have to read the register in order to read data
        if (len == 26){
          uint8_t rx_frame[len*2];
          pGxModule->SPIreadRegisterBurst(0x00, len*2, rx_frame);
          uint8_t val1, val2;
          for (int i = 0;i < len * 2; i++){
            val1 = ManchesterDecode[rx_frame[i]];
            val2 = ManchesterDecode[rx_frame[i+1]];
            data[i>>1] = ((val1 & 0x0F) << 4) | (val2 & 0x0F);
            //data[i>>1] = ~data[i>>1];
            i++;
          }
          return 0;
        }else{
          return ERR_UNKNOWN;
        }
      }else{
        #ifdef USE_GXMODULE
          sx1276setOpMode(SX1276_MODE_STANDBY); //RegOpMode --> set Module to standby
          // read packet data
          pGxModule->SPIreadRegisterBurst(0x00, len, data);     //REG_FIFO      
        #else
        ret = pSx1276Radio->readData(data,len);
        #endif
      }
      return ret;
  }
  return -1;
}

float LoRaClass::getRSSI(){
  switch (radioType){
    case RADIO_SX1262:
      return pSx1262Radio->getRSSI();
      break;
    case RADIO_SX1276:
      if (_fskMode){
        return rssiValue;
      }else{
        #ifdef USE_GXMODULE
        float lastPacketRSSI;
        // RSSI calculation uses different constant for low-frequency and high-frequency ports
        if(_freq < 868.0) {
          lastPacketRSSI = -164 + pGxModule->SPIgetRegValue(0x1A); //REG_PKT_RSSI_VALUE
        } else {
          lastPacketRSSI = -157 + pGxModule->SPIgetRegValue(0x1A); //REG_PKT_RSSI_VALUE
        }
        float lastPacketSNR = getSNR();
        if(lastPacketSNR < 0.0) {
          lastPacketRSSI += lastPacketSNR;
        }
        return lastPacketRSSI;
        #else
        return pSx1276Radio->getRSSI();
        #endif
      }
  }
  return 0.0;
}

int16_t LoRaClass::sx1276setOpMode(uint8_t mode){
  int16_t ret = 0;
  ret = pGxModule->SPIsetRegValue(0x01, mode, 2, 0, 10); //RegOpMode --> set op-mode
  /*
  for (int i = 0; i < 3; i++){
    ret = pGxModule->SPIsetRegValue(0x01, mode, 2, 0, 200); //RegOpMode --> set op-mode
    if (ret == 0) break;
  } 
  */
  //if (ret){
  //  log_e("sx1276 error set OP-Mode %d",ret);    
  //}
  return ret;
}

float LoRaClass::getSNR(){
  switch (radioType){
    case RADIO_SX1262:
      return pSx1262Radio->getSNR();
    case RADIO_SX1276:
      #ifdef USE_GXMODULE
        if (_fskMode) return 0.0;
        // spread-spectrum modulation signal can be received below noise floor
        // check last packet SNR and if it's less than 0, add it to reported RSSI to get the correct value
        // get SNR value
        return (int8_t)pGxModule->SPIgetRegValue(0x19) / 4.0; //REG_PKT_SNR_VALUE
      #else
      return pSx1276Radio->getSNR();
      #endif
  }
  return 0.0;
}

void LoRaClass::printReg(uint8_t reg){
	uint8_t regVal = pModule->SPIreadRegister(reg);
	Serial.printf("%02X:%02X\n",reg,regVal);
}

int16_t LoRaClass::sx1276setRxBandwidth(float rxBw){
  int16_t state = 0;
  // calculate exponent and mantissa values
  for(uint8_t e = 7; e >= 1; e--) {
    for(int8_t m = 2; m >= 0; m--) {
      float point = (SX127X_CRYSTAL_FREQ * 1000000.0)/(((4 * m) + 16) * ((uint32_t)1 << (e + 2)));
      if(abs(rxBw - ((point / 1000.0) + 0.05)) <= 0.5) {
        // set Rx bandwidth during AFC
        state = pGxModule->SPIsetRegValue(SX127X_REG_AFC_BW, (m << 3) | e, 4, 0);
        RADIOLIB_ASSERT(state);

        // set Rx bandwidth
        state = pGxModule->SPIsetRegValue(SX127X_REG_RX_BW, (m << 3) | e, 4, 0);
        return(state);
      }
    }
  }
}

int16_t LoRaClass::switchFSK(float frequency){
  //uint32_t tBegin = micros();
  _freq = frequency;
  int16_t ret = 0;
  uint8_t syncWord[] = {0x99, 0xA5, 0xA9, 0x55, 0x66, 0x65, 0x96};	
  switch (radioType){
    case RADIO_SX1262:
      ret = pSx1262Radio->switchFSK(_br,50.0,156.2,8,0.0,false);
      //log_i("switchFSK %d",ret);
      ret = pSx1262Radio->setFrequency(_freq,false); //don't calibrate Frequency
      //log_i("setFrequency %f ret=%d",frequency,ret);
      ret = pSx1262Radio->setSyncWord(syncWord,sizeof(syncWord));
      //log_i("setSyncWord %d",ret);
      //preamble-size = 1Byte --> 8Bits
      //no CRC --> 0x01
      //payload-size = 26 * 2 --> Manchester encoding
      ret = pSx1262Radio->setPacketParaFSK(1,SX126X_GFSK_CRC_OFF,sizeof(syncWord),0x00,0x00,0x00,26*2,SX126X_GFSK_PREAMBLE_DETECT_8);
      //log_i("setPacketParaFSK %d",ret);
      ret = pSx1262Radio->setDataShaping(RADIOLIB_SHAPING_0_5); //set gaussian filter to 0.5)

      break;
    case RADIO_SX1276:
      #ifdef USE_GXMODULE
      sx1276setOpMode(SX1276_MODE_SLEEP); //RegOpMode --> set Module to sleep
      ret = pGxModule->SPIsetRegValue(0x01, 0b00000000, 6, 5, 200); //set modulation to FSK
      ret = pGxModule->SPIsetRegValue(0x01, 0b00000000, 3, 3, 200); //clear low frequency-mode
      ret = pGxModule->SPIsetRegValue(0x01, 0b00000000, 7, 7, 5); //RegOpMode --> set modem to FSK
      if (ret){
        log_e("sx1276 error set OP-Mode %d",ret);    
      }

      sx1276setOpMode(SX1276_MODE_STANDBY); //RegOpMode --> set Module to standby   
      //set bitrate to 100kBps
      pGxModule->SPIwriteRegister(0x02,0x01); //RegBitrateMsb 100kbps
      pGxModule->SPIwriteRegister(0x03,0x40); //RegBitrateLsb
      //set deviation to 50kHz
      pGxModule->SPIwriteRegister(0x04,0x03); //RegFdevMsb Frequency deviation +/- 50kHz
      pGxModule->SPIwriteRegister(0x05,0x33); //RegFdevLsb
      //calculate register values
      uint32_t FRF = (_freq * (uint32_t(1) << 19)) / 32.0;
      // write registers
      pGxModule->SPIsetRegValue(0x06, (FRF & 0xFF0000) >> 16); //RegFrMsb
      pGxModule->SPIsetRegValue(0x07, (FRF & 0x00FF00) >> 8);  //RegFrMid
      pGxModule->SPIsetRegValue(0x08, FRF & 0x0000FF); //RegFrLsb

      pGxModule->SPIwriteRegister(0x09,0xFC); //RegPaConfig
      pGxModule->SPIwriteRegister(0x0A,0x49); //RegPaRamp
      pGxModule->SPIwriteRegister(0x0B,0x2B); //RegOcp
      pGxModule->SPIwriteRegister(0x0C,0x23); //RegLna max gain, default LNA current
      //pGxModule->SPIwriteRegister(0x0D,0x0E); //RegRxConfig AFC off, AGC on, trigger on preamble?!?
      pGxModule->SPIwriteRegister(0x0D,0x1E); //RegRxConfig Auto AFC on, AGC on, trigger on preamble?!?
      pGxModule->SPIwriteRegister(0x0E,0x02); //RegRssiConfig
      pGxModule->SPIwriteRegister(0x0F,0x0A); //RegRssiCollision
      pGxModule->SPIwriteRegister(0x10,0xFF); //RegRssiThresh
      sx1276setRxBandwidth(125.0); //set receiver-bandwidth 125khz
      //pGxModule->SPIwriteRegister(0x12,0x02); //RegRxBw 125kHz SSb; BW >= (DR + 2 X FDEV)
      //pGxModule->SPIwriteRegister(0x13,0x11); //RegAfcBw 166.6kHz SSB
      pGxModule->SPIwriteRegister(0x14,0x28); //RegOokPeak
      pGxModule->SPIwriteRegister(0x15,0x0C); //RegOokFix
      pGxModule->SPIwriteRegister(0x16,0x12); //RegOokAvg
      pGxModule->SPIwriteRegister(0x1B,0x47); //RegAfcMsb
      pGxModule->SPIwriteRegister(0x1C,0x00); //RegAfcLsb
      pGxModule->SPIwriteRegister(0x1D,0x00); //RegFeiMsb
      pGxModule->SPIwriteRegister(0x1E,0x00); //RegFeiLsb
      pGxModule->SPIwriteRegister(0x1F,0x85); //RegPreambleDetect enable, 1 bytes, 5 chip errors
      pGxModule->SPIwriteRegister(0x20,0x00); //RegRxTimeout1
      pGxModule->SPIwriteRegister(0x21,0x00); //RegRxTimeout2
      pGxModule->SPIwriteRegister(0x22,0x00); //RegRxTimeout3
      pGxModule->SPIwriteRegister(0x23,0x00); //RegRxDelay
      pGxModule->SPIwriteRegister(0x24,0x05); //RegOsc
      pGxModule->SPIwriteRegister(0x25,0x00); //RegPreambleMsb
      pGxModule->SPIwriteRegister(0x26,0x02); //RegPreambleLsb //preamble-size 2
      pGxModule->SPIwriteRegister(0x27,0x36); //RegSyncConfig Sync-Word-Size = 6 Preamble-Type = 55
      pGxModule->SPIwriteRegister(0x28,0x99); //RegSyncValue1
      pGxModule->SPIwriteRegister(0x29,0xA5); //RegSyncValue2
      pGxModule->SPIwriteRegister(0x2A,0xA9); //RegSyncValue3
      pGxModule->SPIwriteRegister(0x2B,0x55); //RegSyncValue4
      pGxModule->SPIwriteRegister(0x2C,0x66); //RegSyncValue5
      pGxModule->SPIwriteRegister(0x2D,0x65); //RegSyncValue6
      pGxModule->SPIwriteRegister(0x2E,0x96); //RegSyncValue7
      pGxModule->SPIwriteRegister(0x2F,0X00); //RegSyncValue8
      pGxModule->SPIwriteRegister(0x30,0x00); //RegPacketConfig1 WHITENING_NONE
      pGxModule->SPIwriteRegister(0x31,0x40); //RegPacketConfig2 packet mode
      pGxModule->SPIwriteRegister(0x32,0x34); //RegPayloadLength
      pGxModule->SPIwriteRegister(0x33,0x00); //RegNodeAdrs
      pGxModule->SPIwriteRegister(0x34,0x00); //RegBroadcastAdrs
      pGxModule->SPIwriteRegister(0x35,0x0F); //RegFifoThresh
      pGxModule->SPIwriteRegister(0x36,0x00); //RegSeqConfig1
      pGxModule->SPIwriteRegister(0x37,0x00); //RegSeqConfig2
      pGxModule->SPIwriteRegister(0x38,0x00); //RegTimerResol
      pGxModule->SPIwriteRegister(0x39,0xF5); //RegTimer1Coef
      pGxModule->SPIwriteRegister(0x3A,0x20); //RegTimer2Coef
      pGxModule->SPIwriteRegister(0x3B,0x00); //RegImageCal
      pGxModule->SPIwriteRegister(0x40,0x00); //RegDioMapping1
      pGxModule->SPIwriteRegister(0x41,0x00); //RegDioMapping2
      pGxModule->SPIwriteRegister(0x4B,0x09); //RegTcxo Power up tcxo
      pGxModule->SPIwriteRegister(0x61,0x19); //RegAgcRef
      pGxModule->SPIwriteRegister(0x62,0x0C); //RegAgcThresh1
      pGxModule->SPIwriteRegister(0x63,0x4B); //RegAgcThresh2
      pGxModule->SPIwriteRegister(0x64,0xCC); //RegAgcThresh3
      pGxModule->SPIwriteRegister(0x70,0xD0); //RegPll
      sx1276setOpMode(SX1276_MODE_STANDBY); //RegOpMode --> set Module to stand-by

      #else
      /*
      ret = pSx1276Radio->beginFSK(frequency,100.0,50.0,125.0,_power,8,false);
      //ret = pSx1276Radio->beginFSK();
      if (ret) log_e("beginFSK %d",ret);
      //ret = pSx1276Radio->setFrequency(frequency);
      //if (ret) log_e("setFrequency %d",ret);
      //ret = pSx1276Radio->setBitRate(100.0);
      //if (ret) log_e("setBitRate %d",ret);
      //ret = pSx1276Radio->setFrequencyDeviation(50.0);
      //if (ret) log_e("setBandwidth %d",ret);
      //ret = pSx1276Radio->setRxBandwidth(125.0);
      //if (ret) log_e("setRxBandwidth %d",ret);
      //ret = pSx1276Radio->setOutputPower(_power);
      //if (ret) log_e("setOutputPower %d",ret);
      //ret = pSx1276Radio->setCurrentLimit(100);
      //if (ret) log_e("setCurrentLimit %d",ret);
      ret = pSx1276Radio->setDataShaping(RADIOLIB_SHAPING_0_5); //set gaussian filter to 0.5
      if (ret) log_e("setDataShaping %d",ret);
      ret = pSx1276Radio->setSyncWord(syncWord,sizeof(syncWord));
      if (ret) log_e("setSyncWord %d",ret);
      ret = pSx1276Radio->setOOK(false);
      if (ret) log_e("setOOK %d",ret);
      //ret = pSx1276Radio->setGain(1); //set highest Gain
      //if (ret) log_e("setGain %d",ret);
      ret = pSx1276Radio->fixedPacketLengthMode(FSK_PACKET_LENGTH);
      if (ret) log_e("fixedPacketLengthMode %d",ret);
      ret = pSx1276Radio->setCRC(false);
      if (ret) log_e("setCRC %d",ret);
      ret = pSx1276Radio->setRSSIConfig(2);
      if (ret) log_e("setRSSIConfig %d",ret);
      ret = pSx1276Radio->setEncoding(RADIOLIB_ENCODING_MANCHESTER);      
      if (ret) log_e("setWhitening %d",ret);
      //ret = pSx1276Radio->setPreambleLength(2);
      //if (ret) log_e("setPreambleLength %d",ret);
      */


      /*
      ret = pSx1276Radio->setActiveModem(SX127X_FSK_OOK);
      //log_i("setActiveModem %d",ret);
      
      //set module to standby
      pModule->SPIsetRegValue(SX127X_REG_OP_MODE, 0x00);
      if (pModule->SPIreadRegister(SX127X_REG_RX_NB_BYTES) != 0x00){
        log_e("failed set modem to sleep");
      }

      pSx1276Radio->standby(); //switch to standby

      // set Frequency 
      //calculate register values
      uint32_t FRF = (frequency * (uint32_t(1) << SX127X_DIV_EXPONENT)) / SX127X_CRYSTAL_FREQ;

      // write registers
      pModule->SPIsetRegValue(SX127X_REG_FRF_MSB, (FRF & 0xFF0000) >> 16);
      pModule->SPIsetRegValue(SX127X_REG_FRF_MID, (FRF & 0x00FF00) >> 8);
      pModule->SPIsetRegValue(SX127X_REG_FRF_LSB, FRF & 0x0000FF);

      // set LNA gain
      //pModule->SPIsetRegValue(SX127X_REG_LNA, LNA_RX_GAIN);
      pModule->SPIsetRegValue(SX127X_REG_LNA, 0x20 | 0x03); // max gain, boost enable
      //pModule->SPIsetRegValue(SX127X_REG_LNA, 0x20 | 0x00); // max gain, default LNA current
      //pModule->SPIsetRegValue(SX127X_REG_LNA, 0x60 | 0x00); // -12dB gain, default LNA current
      //pModule->SPIsetRegValue(SX127X_REG_LNA, 0x80 | 0x00); // -24dB gain, default LNA current
      //pModule->SPIsetRegValue(SX127X_REG_LNA, 0xC0 | 0x00); // -48dB gain, default LNA current

      // configure receiver
      //pModule->SPIsetRegValue(SX127X_REG_RX_CONFIG, 0x1E); // AFC auto, AGC, trigger on preamble?!?
      pModule->SPIsetRegValue(SX127X_REG_RX_CONFIG, 0x0E); // AFC off, AGC on, trigger on preamble?!?
      //pModule->SPIsetRegValue(SX127X_REG_RX_CONFIG, 0x06); // AFC off, AGC off, trigger on preamble?!?

      pModule->SPIsetRegValue(SX127X_REG_RX_BW, 0x02); // 125kHz SSb; BW >= (DR + 2 X FDEV)

    // set AFC bandwidth
//    pModule->SPIsetRegValue(SX127X_REG_AFC_BW, 0x0B); // 50kHz SSB  // PAW
//    pModule->SPIsetRegValue(SX127X_REG_AFC_BW, 0x12); // 83.3kHz SSB
      pModule->SPIsetRegValue(SX127X_REG_AFC_BW, 0x11); // 166.6kHz SSB
//    pModule->SPIsetRegValue(SX127X_REG_AFC_BW, 0x09); // 200kHz SSB
//    pModule->SPIsetRegValue(SX127X_REG_AFC_BW, 0x01); // 250kHz SSB

      pModule->SPIsetRegValue(SX127X_REG_PREAMBLE_DETECT, 0x85); // enable, 1 bytes, 5 chip errors
      
      // set sync config
      uint8_t SyncConfig = (sizeof(syncWord) - 1);
      //set preambletype to 55
      pModule->SPIsetRegValue(SX127X_REG_SYNC_CONFIG, (0x30 | SyncConfig));

      //set Manchester encoding and fixed length
      pModule->SPIsetRegValue(SX127X_REG_PACKET_CONFIG_1, 0x20);
      //set packet-mode
      pModule->SPIsetRegValue(SX127X_REG_PACKET_CONFIG_2, 0x40); // packet mode


      pModule->SPIsetRegValue(SX127X_REG_PAYLOAD_LENGTH_FSK,26);

      // set sync word
      pModule->SPIwriteRegisterBurst(SX127X_REG_SYNC_VALUE_1, syncWord, sizeof(syncWord));

      //set bitrate to 100kBps
      pModule->SPIsetRegValue(SX127X_REG_BITRATE_MSB, 0x01); // 100kbps
      pModule->SPIsetRegValue(SX127X_REG_BITRATE_LSB, 0x40);      

      //set deviation to 50kHz
      pModule->SPIsetRegValue(SX127X_REG_FDEV_MSB, 0x03); // +/- 50kHz
      pModule->SPIsetRegValue(SX127X_REG_FDEV_LSB, 0x33);

      // frame and packet handler settings
      pModule->SPIsetRegValue(SX127X_REG_PREAMBLE_MSB_FSK, 0x00);
      // add extra preamble symbol at Tx to ease reception on partner's side 
      pModule->SPIsetRegValue(SX127X_REG_PREAMBLE_LSB_FSK, 2);

      // configure output power
      pModule->SPIsetRegValue(SX127X_REG_PA_RAMP, 0x49); 
      */

      

      /*
      ret = pSx1276Radio->setActiveModem(SX127X_FSK_OOK);
      if (ret) log_e("setActiveModem %d",ret);
      //ret = pSx1276Radio->setOOK(false);
      //log_i("setOOK %d",ret);
      ret = pSx1276Radio->setFrequency(frequency);
      if (ret) log_e("setOOK %d",ret);
      ret = pSx1276Radio->setBitRate(100.0);
      if (ret) log_e("setBitRate %d",ret);
      ret = pSx1276Radio->setFrequencyDeviation(50.0);
      if (ret) log_e("setBandwidth %d",ret);
      ret = pSx1276Radio->setRxBandwidth(125.0);
      if (ret) log_e("setRxBandwidth %d",ret);
      // set LNA gain
      //pModule->SPIsetRegValue(SX127X_REG_LNA, LNA_RX_GAIN);
      pModule->SPIsetRegValue(SX127X_REG_LNA, 0x20 | 0x03); // max gain, boost enable
      //pModule->SPIsetRegValue(SX127X_REG_LNA, 0x20 | 0x00); // max gain, default LNA current
      //pModule->SPIsetRegValue(SX127X_REG_LNA, 0x60 | 0x00); // -12dB gain, default LNA current
      //pModule->SPIsetRegValue(SX127X_REG_LNA, 0x80 | 0x00); // -24dB gain, default LNA current
      //pModule->SPIsetRegValue(SX127X_REG_LNA, 0xC0 | 0x00); // -48dB gain, default LNA current

      // configure receiver
      //pModule->SPIsetRegValue(SX127X_REG_RX_CONFIG, 0x1E); // AFC auto, AGC, trigger on preamble?!?
      pModule->SPIsetRegValue(SX127X_REG_RX_CONFIG, 0x0E); // AFC off, AGC on, trigger on preamble?!?
      //pModule->SPIsetRegValue(SX127X_REG_RX_CONFIG, 0x06); // AFC off, AGC off, trigger on preamble?!?
      //pModule->SPIsetRegValue(SX127X_REG_AFC_BW, 0x11); // 166.6kHz SSB
      //ret = pSx1276Radio->setGain(1); //set highest Gain
      //if (ret) log_e("setGain %d",ret);
      ret = pSx1276Radio->setSyncWord(syncWord,sizeof(syncWord));
      if (ret) log_e("setSyncWord %d",ret);
      ret = pSx1276Radio->setOutputPower(_power);
      if (ret) log_e("setOutputPower %d",ret);
      ret = pSx1276Radio->fixedPacketLengthMode(FSK_PACKET_LENGTH);
      if (ret) log_e("fixedPacketLengthMode %d",ret);
      ret = pSx1276Radio->setCRC(false);
      if (ret) log_e("setCRC %d",ret);
      ret = pSx1276Radio->setRSSIConfig(2);
      if (ret) log_e("setRSSIConfig %d",ret);
      ret = pSx1276Radio->setEncoding(RADIOLIB_ENCODING_MANCHESTER);      
      if (ret) log_e("setWhitening %d",ret);
      ret = pSx1276Radio->setPreambleLength(2);
      if (ret) log_e("setPreambleLength %d",ret);
      ret = pSx1276Radio->setDataShaping(RADIOLIB_SHAPING_0_5); //set gaussian filter to 0.5
      if (ret) log_e("setDataShaping %d",ret);
      //pModule->SPIsetRegValue(SX127X_REG_AFC_BW, 0x11); // 166.6kHz SSB
      */
      ret = pSx1276Radio->setActiveModem(SX127X_FSK_OOK);
      if (ret) log_e("setActiveModem %d",ret);
      //ret = pSx1276Radio->setOOK(false);
      //log_i("setOOK %d",ret);
      ret = pSx1276Radio->setFrequency(frequency);
      if (ret) log_e("setFrequency %d",ret);
      ret = pSx1276Radio->setBitRate(_br);
      if (ret) log_e("setBitRate %d",ret);
      ret = pSx1276Radio->setFrequencyDeviation(50.0);
      if (ret) log_e("setFrequencyDeviation %d",ret);
      ret = pSx1276Radio->setRxBandwidth(125.0);
      if (ret) log_e("setRxBandwidth %d",ret);
      ret = pSx1276Radio->setSyncWord(syncWord,sizeof(syncWord));
      if (ret) log_e("setSyncWord %d",ret);
      ret = pSx1276Radio->setOutputPower(_power);
      if (ret) log_e("setOutputPower %d",ret);
      //ret = pSx1276Radio->fixedPacketLengthMode(FSK_PACKET_LENGTH);
      ret = pSx1276Radio->fixedPacketLengthMode(FSK_PACKET_LENGTH*2);
      if (ret) log_e("fixedPacketLengthMode %d",ret);
      ret = pSx1276Radio->setCRC(false);
      if (ret) log_e("setCRC %d",ret);
      ret = pSx1276Radio->setRSSIConfig(2);
      if (ret) log_e("setRSSIConfig %d",ret);
      //ret = pSx1276Radio->setEncoding(RADIOLIB_ENCODING_MANCHESTER);      
      ret = pSx1276Radio->setEncoding(RADIOLIB_ENCODING_NRZ);
      if (ret) log_e("setEncoding %d",ret);
      ret = pSx1276Radio->setPreambleLength(2);
      if (ret) log_e("setPreambleLength %d",ret);
      ret = pSx1276Radio->setDataShaping(RADIOLIB_SHAPING_0_5); //set gaussian filter to 0.5     
      if (ret) log_e("setDataShaping %d",ret);
      ret = pSx1276Radio->setCurrentLimit(100);
      if (ret) log_e("setCurrentLimit %d",ret);
      // set LNA gain
      pModule->SPIsetRegValue(SX127X_REG_LNA, 0x20 | 0x03); // max gain, boost enable
      // configure receiver
      pModule->SPIsetRegValue(SX127X_REG_RX_CONFIG, 0x0E); // AFC off, AGC on, trigger on preamble?!?
      pModule->SPIsetRegValue(SX127X_REG_AFC_BW, 0x11); // 166.6kHz SSB
      pModule->SPIsetRegValue(SX127X_REG_PREAMBLE_DETECT,  0x85);  // enable, 1 bytes, 5 chip errors
      pModule->SPIsetRegValue(SX127X_REG_SYNC_CONFIG, 0x30 + sizeof(syncWord) - 1);
      #endif

      /*
      for (int i = 0x00;i <= 0x44;i++){
        printReg(i);
      }
      printReg(0x4B);
      printReg(0x4D);
      printReg(0x5B);
      printReg(0x5D);
      printReg(0x61);
      printReg(0x62);
      printReg(0x63);
      printReg(0x64);
      printReg(0x70);
      */

      //calib image
      //pModule->SPIsetRegValue(SX127X_REG_IMAGE_CAL, 0x40);
      //while ( pModule->SPIgetRegValue(SX127X_REG_IMAGE_CAL) & 0x20 );      

      break;
  }
  enableInterrupt = false;
  receivedFlag = false;
  _fskMode = true;
  //log_i("FSK-Mode On %d",micros()-tBegin);
  return 0;
}

bool LoRaClass::isReceiving(){
  switch (radioType){
    case RADIO_SX1262:
      return false;
      break;
    case RADIO_SX1276:
      uint8_t reg = pModule->SPIgetRegValue(SX127X_REG_IRQ_FLAGS_1);
      log_i("regIRQ=%d",reg);
      return false;
      break;
  }
  return false;
}

int16_t LoRaClass::switchLORA(float frequency){
  _freq = frequency;
  int16_t ret = -1;
  switch (radioType){
    case RADIO_SX1262:
      ret = pSx1262Radio->switchLoRa(_bw,_sf,_cr,_syncWord,8,0.0,false);
      //log_i("switchLoRa %d",ret);
      ret = pSx1262Radio->setFrequency(_freq,false); //don't calibrate Frequency
      //log_i("setFrequency %d",ret);
      break;
    case RADIO_SX1276:
      #ifdef USE_GXMODULE
      sx1276setOpMode(SX1276_MODE_SLEEP); //RegOpMode --> set Module to sleep
      ret = pGxModule->SPIsetRegValue(0x01, 0b00000000, 6, 6, 200); //AccessSharedReg
      ret = pGxModule->SPIsetRegValue(0x01, 0b00000000, 3, 3, 200); //clear low frequency-mode

      int16_t ret = pGxModule->SPIsetRegValue(0x01, 0b10000000, 7, 7, 5); //RegOpMode --> set modem to LORA
      if (ret){
        log_e("sx1276 error set OP-Mode %d",ret);    
      }      
      //calculate register values
      uint32_t FRF = (_freq * (uint32_t(1) << 19)) / 32.0;
      // write registers
      pGxModule->SPIwriteRegister(0x06, (FRF & 0xFF0000) >> 16); //RegFrMsb
      pGxModule->SPIwriteRegister(0x07, (FRF & 0x00FF00) >> 8);  //RegFrMid
      pGxModule->SPIwriteRegister(0x08, FRF & 0x0000FF); //RegFrLsb
      //pGxModule->SPIwriteRegister(0x06,0xD9); //RegFrMsb
      //pGxModule->SPIwriteRegister(0x07,0x0C); //RegFrMid
      //pGxModule->SPIwriteRegister(0x08,0xCD); //RegFrLsb
      pGxModule->SPIwriteRegister(0x09,0xFC); //RegPaConfig
      pGxModule->SPIwriteRegister(0x0A,0x49); //RegPaRamp
      pGxModule->SPIwriteRegister(0x0B,0x2B); //RegOcp
      pGxModule->SPIwriteRegister(0x0C,0x23); //RegLna
      pGxModule->SPIwriteRegister(0x0D,0x01); //RegFifoAddrPtr
      pGxModule->SPIwriteRegister(0x0E,0x00); //RegFifoTxBaseAddr
      pGxModule->SPIwriteRegister(0x0F,0x00); //RegFifoRxBaseAddr
      pGxModule->SPIwriteRegister(0x11,0x00); //RegIrqFlags
      pGxModule->SPIwriteRegister(0x1D,0x88); //RegModemConfig1
      pGxModule->SPIwriteRegister(0x1E,0x74); //RegModemConfig2
      pGxModule->SPIwriteRegister(0x1F,0x64); //RegSymbTimeoutLsb
      pGxModule->SPIwriteRegister(0x20,0x00); //RegPreambleMsb
      pGxModule->SPIwriteRegister(0x21,0x07); //RegPreambleLsb
      pGxModule->SPIwriteRegister(0x22,0x13); //RegPayloadLength
      pGxModule->SPIwriteRegister(0x23,0xFF); //RegMaxPayloadLength
      pGxModule->SPIwriteRegister(0x24,0x00); //RegHopPeriod
      pGxModule->SPIwriteRegister(0x26,0x04); //RegModemConfig3
      pGxModule->SPIwriteRegister(0x27,0x00); //Data rate offset value, used in conjunction with AFC
      pGxModule->SPIwriteRegister(0x31,0x43); //RegDetectOptimize
      pGxModule->SPIwriteRegister(0x33,0x27); //RegInvertIQ
      pGxModule->SPIwriteRegister(0x36,0x03); //RegHighBWOptimize1
      pGxModule->SPIwriteRegister(0x37,0x0A); //RegDetectionThreshold
      pGxModule->SPIwriteRegister(0x39,0xF1); //set sync-word
      pGxModule->SPIwriteRegister(0x3A,0x52); //RegHighBWOptimize1
      pGxModule->SPIwriteRegister(0x3B,0x1D); //RegInvertIQ2
      pGxModule->SPIwriteRegister(0x40,0x00); //RegDioMapping1
      pGxModule->SPIwriteRegister(0x41,0x00); //RegDioMapping2
      pGxModule->SPIwriteRegister(0x4B,0x09); //RegTcxo
      pGxModule->SPIwriteRegister(0x4D,0x84); //RegPaDac
      pGxModule->SPIwriteRegister(0x61,0x19); //RegAgcRef
      pGxModule->SPIwriteRegister(0x62,0x0C); //RegAgcThresh1
      pGxModule->SPIwriteRegister(0x63,0x4B); //RegAgcThresh2
      pGxModule->SPIwriteRegister(0x64,0xCC); //RegAgcThresh3
      pGxModule->SPIwriteRegister(0x70,0xD0); //RegPll
      sx1276setOpMode(SX1276_MODE_STANDBY);  //RegOpMode --> set Module to stand-by
      #else
      ret = pSx1276Radio->setActiveModem(SX127X_LORA);
      //log_i("setActiveModem %d",ret);
      ret = pSx1276Radio->setSyncWord(_syncWord);
      //log_i("setSyncWord %d",ret);
      ret = pSx1276Radio->setPreambleLength(7);
      //log_i("setPreambleLength %d",ret);
      ret = pSx1276Radio->setBandwidth(_bw);
      //log_i("setBandwidth %d",ret);
      ret = pSx1276Radio->setFrequency(_freq);
      //log_i("setFrequency %d",ret);
      ret = pSx1276Radio->setSpreadingFactor(_sf);
      //log_i("setSpreadingFactor %d",ret);
      ret = pSx1276Radio->setCodingRate(_cr);
      //log_i("setCodingRate %d",ret);
      ret = pSx1276Radio->setOutputPower(_power);
      //log_i("setOutputPower %d",ret);
      #endif

      /*
      for (int i = 0x00;i <= 0x44;i++){
        printReg(i);
      }
      printReg(0x4B);
      printReg(0x4D);
      printReg(0x5B);
      printReg(0x5D);
      printReg(0x61);
      printReg(0x62);
      printReg(0x63);
      printReg(0x64);
      printReg(0x70);
      */

      break;
  }
  enableInterrupt = false;
  receivedFlag = false;
  _fskMode = false;
  return ret;
}

float LoRaClass::expectedAirTime_ms(int pktLen)
{
	if (_fskMode){
    return 0; //not in FSK-Mode !!
  }
  /* LORA */
  float bw = _bw * 1000;
  // Symbol rate : time for one symbol (secs)
  //float sf = _sf<6 ? 6.0f : _sf;
  float rs = bw / ( 1 << _sf);
  float ts = 1 / rs;
  // time of preamble
  float tPreamble = ( _preambleLength + 4.25f ) * ts;		//note: assuming preamble < 256
  float tmp = ceil( (8 * pktLen - 4 * _sf + 28 + 16 - 20) / (float)( 4 * ( _sf ) ) ) * _cr;
  float nPayload = 8 + ( ( tmp > 0 ) ? tmp : 0 );
  float tPayload = nPayload * ts;
  // Time on air
  float tOnAir = (tPreamble + tPayload) * 1000.0f;
  //log_i("tOnAir=%f",tOnAir);
  return tOnAir;
}



float LoRaClass::get_airlimit(void)
{
  static uint32_t last = 0;
	uint32_t current = millis();
	uint32_t dt = current - last;
	last = current;

	// reduce airtime by 1% 
	sx_airtime -= dt*0.01f;
	if(sx_airtime < 0.0f)
		sx_airtime = 0.0f;

	// air time over 3min average -> 1800ms air time allowed 
	return sx_airtime / 1800.0f;
}



int16_t LoRaClass::startReceive(){
  #if LORA_RX_DEBUG > 0
  uint32_t tBegin = micros();
  char Buffer[500];
  #endif
  int16_t iRet = 0;
  //delay(5);
  prevIrqFlags = 0;
  switch (radioType){
    case RADIO_SX1262:
      iRet = pSx1262Radio->startReceive();
      break;
    case RADIO_SX1276:
      #ifdef USE_GXMODULE
        if (_fskMode){
          sx1276setOpMode(SX1276_MODE_STANDBY);//RegOpMode --> set Module to standby
          if (!bCalibrated){
            log_i("start calib image");
            //calib image
            pGxModule->SPIwriteRegister(0x3B, 0x40); //REG_IMAGE_CAL
            while ( pGxModule->SPIgetRegValue(0x3B) & 0x20 );      
            bCalibrated = true;
          }
          pGxModule->SPIsetRegValue(0x40, 0x00, 7, 6); //REG_DIO_MAPPING_1 --> DIO0_PACK_PAYLOAD_READY
          pGxModule->SPIwriteRegister(0x3E, 0b11111111); //REG_IRQ_FLAGS_1
          pGxModule->SPIwriteRegister(0x3F, 0b11111111); //REG_IRQ_FLAGS_2
          // set RF switch (if present)
          pGxModule->setRfSwitchState(HIGH, LOW);
          //iRet = sx1276setOpMode(SX1276_MODE_FS_MODE_RX); //RegOpMode --> set Module to RXCONTINUOUS
          iRet = sx1276setOpMode(SX1276_MODE_RX_CONTINUOUS); //RegOpMode --> set Module to RXCONTINUOUS
          /*
          if (iRet){
            log_e("FSK-Mode set OP-Mode RX_Continous failed");
          }
          */
        }else{
          sx1276setOpMode(SX1276_MODE_STANDBY); //RegOpMode --> set Module to standby
          pGxModule->SPIsetRegValue(0x40, 0x00, 7, 4); //REG_DIO_MAPPING_1
          pGxModule->SPIwriteRegister(0x12, 0b11111111); //REG_IRQ_FLAGS //clear IRQ
          pGxModule->SPIsetRegValue(0x0F, 0x00); //REG_FIFO_RX_BASE_ADDR
          pGxModule->SPIsetRegValue(0x0D, 0x00); //REG_FIFO_ADDR_PTR
          // set RF switch (if present)
          pGxModule->setRfSwitchState(HIGH, LOW);
          iRet = sx1276setOpMode(SX1276_MODE_RX_CONTINUOUS); //RegOpMode --> set Module to RXCONTINUOUS
          if (iRet){
            log_e("LORA-Mode set OP-Mode RX_Continous failed");
          }
        }
      #else
      if ((!bCalibrated) && (_fskMode)){
        log_i("start calib image");
        pSx1276Radio->setActiveModem(SX127X_FSK_OOK);
        //calib image
        pModule->SPIsetRegValue(SX127X_REG_IMAGE_CAL, 0x40);
        while ( pModule->SPIgetRegValue(SX127X_REG_IMAGE_CAL) & 0x20 );      
        bCalibrated = true;
      }
      iRet = pSx1276Radio->startReceive();
      #endif
      break;
  }
  receivedFlag = false;
  enableInterrupt = true;
  #if LORA_RX_DEBUG > 10
  //log_i("%d start Receive",millis());
  #endif
  #if LORA_RX_DEBUG > 0
  sprintf(Buffer,"startReceive %dus\n",int(micros()-tBegin));
  Serial.print(Buffer);
  #endif
  return iRet;
}

size_t LoRaClass::getPacketLength(){
  size_t tRet = 0;
  //log_i("getPacketLength %d",radioType);
  if (_fskMode) return FSK_PACKET_LENGTH;
  switch (radioType){
    case RADIO_SX1262:
      tRet = pSx1262Radio->getPacketLength();
      if (_fskMode){
        tRet /= 2; //cause of manchester-decoding we get always double of the Length
      }      
      break;
    case RADIO_SX1276:
      #ifdef USE_GXMODULE
      if (_fskMode){
        tRet = pGxModule->SPIreadRegister(0x32); //REG_PAYLOAD_LENGTH_FSK
      }else{
        tRet = pGxModule->SPIreadRegister(0x13); //REG_RX_NB_BYTES
      }
      
      #else
      tRet = pSx1276Radio->getPacketLength();
      #endif
      break;
  }
  return tRet;
}

int16_t LoRaClass::setCodingRate(uint8_t cr){
  int16_t iRet = 0;
  if (_cr != cr){
    _cr = cr;
    switch (radioType){
      case RADIO_SX1262:
        log_i("setCodingRate SX1262");
        iRet = pSx1262Radio->setCodingRate(cr);
        break;
      case RADIO_SX1276:
        log_i("setCodingRate SX1276");
        #ifdef USE_GXMODULE
          uint8_t newCodingRate;
          // check allowed coding rate values
          switch(cr) {
            case 5:
              newCodingRate = 0b00000010;
              break;
            case 6:
              newCodingRate = 0b00000100;
              break;
            case 7:
              newCodingRate = 0b00000110;
              break;
            case 8:
              newCodingRate = 0b00001000;
              break;
            default:
              return(-10);
          }
          sx1276setOpMode(SX1276_MODE_STANDBY); //RegOpMode --> set Module to standby
          pGxModule->SPIsetRegValue(0x1D, newCodingRate, 3, 1); //REG_MODEM_CONFIG_1
        #else
        iRet = pSx1276Radio->setCodingRate(cr);
        #endif
        break;
    }
  }
  return iRet;
}

int16_t LoRaClass::transmit(uint8_t* data, size_t len){
  int16_t ret = 0;
  enableInterrupt = false;
  receivedFlag = false;
  uint8_t tx_frame[FSK_PACKET_LENGTH*2];
  if (_fskMode){
    for (int i = 0;i < FSK_PACKET_LENGTH * 2; i++){            
      tx_frame[i] = ManchesterEncode[(data[i>>1] >> 4) & 0x0F];
      tx_frame[i+1] = ManchesterEncode[(data[i>>1]) & 0x0F];
      i++;
    }    
  }else{
    /* update air time */
    sx_airtime += expectedAirTime_ms(len);
  } 	
  switch (radioType){
    case RADIO_SX1262:
      if (_fskMode){
        if (len == 26){
          return pSx1262Radio->transmit(tx_frame,FSK_PACKET_LENGTH*2);
        }else{
          return -1;
        }        
      }else{
        return pSx1262Radio->transmit(data,len);
      }      
    case RADIO_SX1276:
      if (_fskMode){
        if (len == 26){
          #ifdef USE_GXMODULE
          sx1276setOpMode(SX1276_MODE_STANDBY); //RegOpMode --> set Module to standby
          // calculate timeout (5ms + 500 % of expected time-on-air)
          uint32_t timeout = 5000000 + (uint32_t)((((float)(FSK_PACKET_LENGTH*2 * 8)) / (_br * 1000.0)) * 5000000.0);
          // set DIO mapping
          pGxModule->SPIsetRegValue(0x40, 0b00000000, 7, 6); //REG_DIO_MAPPING_1 --> DIO0_PACK_PACKET_SENT

          // clear interrupt flags
          pGxModule->SPIwriteRegister(0x3E, 0b11111111); //REG_IRQ_FLAGS_1
          pGxModule->SPIwriteRegister(0x3F, 0b11111111); //REG_IRQ_FLAGS_2
          // write packet to FIFO
          pGxModule->SPIwriteRegisterBurst(0x00, tx_frame, FSK_PACKET_LENGTH*2); //REG_FIFO

          // set RF switch (if present)
          pGxModule->setRfSwitchState(LOW, HIGH);
          // start transmission
          sx1276setOpMode(SX1276_MODE_TX);//RegOpMode --> set Module to TX
          // start transmission
          uint32_t start = GxModule::micros();
          while(!GxModule::digitalRead(pGxModule->getIrq())) {
            GxModule::yield();
            if(GxModule::micros() - start > timeout) {
              ret = -5;
              break;
            }
          }
          pGxModule->SPIwriteRegister(0x3E, 0b11111111); //REG_IRQ_FLAGS_1
          pGxModule->SPIwriteRegister(0x3F, 0b11111111); //REG_IRQ_FLAGS_2
          sx1276setOpMode(SX1276_MODE_STANDBY);//RegOpMode --> set Module to standby
          return ret;
          #else
          return pSx1276Radio->transmit(tx_frame,FSK_PACKET_LENGTH*2);
          #endif
        }else{
          return -1;
        }        
      }else{
        #ifdef USE_GXMODULE
          sx1276setOpMode(SX1276_MODE_STANDBY);//RegOpMode --> set Module to standby
          // calculate timeout (150 % of expected time-one-air)
          float symbolLength = (float)(uint32_t(1) <<_sf) / (float)_bw;
          float de = 0;
          if(symbolLength >= 16.0) {
            de = 1;
          }
          float ih = (float)pGxModule->SPIgetRegValue(0x1D, 0, 0); //REG_MODEM_CONFIG_1
          float crc = (float)(pGxModule->SPIgetRegValue(0x1E, 2, 2) >> 2); //REG_MODEM_CONFIG_2
          float n_pre = (float)((pGxModule->SPIgetRegValue(0x20) << 8) | pGxModule->SPIgetRegValue(0x21)); //REG_PREAMBLE_MSB
          float n_pay = 8.0 + max(ceil((8.0 * (float)len - 4.0 * (float)_sf + 28.0 + 16.0 * crc - 20.0 * ih)/(4.0 * (float)_sf - 8.0 * de)) * (float)_cr, 0.0);
          uint32_t timeout = ceil(symbolLength * (n_pre + n_pay + 4.25) * 1500.0);

          // set DIO mapping
          pGxModule->SPIsetRegValue(0x40, 0b01000000, 7, 6); //REG_DIO_MAPPING_1 --> DIO0_TX_DONE

          // clear interrupt flags
          pGxModule->SPIwriteRegister(0x12, 0b11111111);

          // set packet length
          pGxModule->SPIsetRegValue(0x22, len);  //REG_PAYLOAD_LENGTH

          // set FIFO pointers
          pGxModule->SPIsetRegValue(0x0E, 0x00); //REG_FIFO_TX_BASE_ADDR
          pGxModule->SPIsetRegValue(0x0D, 0x00); //REG_FIFO_ADDR_PTR

          // write packet to FIFO
          pGxModule->SPIwriteRegisterBurst(0x00, data, len); //REG_FIFO

          // set RF switch (if present)
          pGxModule->setRfSwitchState(LOW, HIGH);

          // start transmission
          sx1276setOpMode(SX1276_MODE_TX); //RegOpMode --> set Module to TX
          // start transmission
          uint32_t start = GxModule::micros();
          while(!GxModule::digitalRead(pGxModule->getIrq())) {
            GxModule::yield();
            if(GxModule::micros() - start > timeout) {
              ret = -5;
              break;
           }
          }
          pGxModule->SPIwriteRegister(0x12, 0b11111111); //clear irq-flags
          sx1276setOpMode(SX1276_MODE_STANDBY); //RegOpMode --> set Module to standby
          return ret;
        #else
        return pSx1276Radio->transmit(data,len);
        #endif
      }      
  }
  return -1;
}

void LoRaClass::invertba(byte* ba, int len)
{
  for (int i =0;i<len;i++)
    ba[i] =~ba[i];
} 

void LoRaClass::end(){
  //set radio to sleep-mode
  switch (radioType){
    case RADIO_SX1262:
      pSx1262Radio->sleep();
      break;
    case RADIO_SX1276:
      #ifdef USE_GXMODULE
      sx1276setOpMode(SX1276_MODE_SLEEP);//RegOpMode --> set Module to sleep
      #else
      pSx1276Radio->sleep();
      #endif
      break;
  }
}

