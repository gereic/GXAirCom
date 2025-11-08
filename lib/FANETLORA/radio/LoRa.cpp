// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "LoRa.h"
#include "manchester.h"


// flag to indicate that a packet was received
volatile bool receivedFlag = false;
volatile uint32_t gtReceived = 0;

// disable interrupt when it's not needed
volatile bool enableInterrupt = true;

void IRAM_ATTR setFlag(void)
{
    // check if the interrupt is enabled
    if (!enableInterrupt) {
        return;
    }
    if (!receivedFlag) gtReceived = millis();
    receivedFlag = true;
}

int16_t LoRaClass::sx1262CalibrateImage(){
  uint8_t data[2];
  if(_freq > 900000000) {
    data[0] = SX126X_CAL_IMG_902_MHZ_1;
    data[1] = SX126X_CAL_IMG_902_MHZ_2;
  } else if(_freq > 850000000) {
    data[0] = SX126X_CAL_IMG_863_MHZ_1;
    data[1] = SX126X_CAL_IMG_863_MHZ_2;
  } else if(_freq > 770000000) {
    data[0] = SX126X_CAL_IMG_779_MHZ_1;
    data[1] = SX126X_CAL_IMG_779_MHZ_2;
  } else if(_freq > 460000000) {
    data[0] = SX126X_CAL_IMG_470_MHZ_1;
    data[1] = SX126X_CAL_IMG_470_MHZ_2;
  } else {
    data[0] = SX126X_CAL_IMG_430_MHZ_1;
    data[1] = SX126X_CAL_IMG_430_MHZ_2;
  }
  return(SPIwriteCommand(0x98, data, 2));
}

void LoRaClass::run(){
  //log_i("run");
  static uint32_t tLast = millis();
  uint32_t tAct = millis();
  if ((radioType == RADIO_SX1276) && (_fskMode)){
    uint8_t irqFlags = pGxModule->SPIgetRegValue(0x3E); //SX127X_REG_IRQ_FLAGS_1
    irqFlags &= SX127X_FLAG_SYNC_ADDRESS_MATCH;
    if (prevIrqFlags != irqFlags){
      prevIrqFlags = irqFlags;
      if (irqFlags){
        rssiValue = (float)pGxModule->SPIgetRegValue(0x11) / -2.0; //REG_RSSI_VALUE_FSK
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
}

//LoRaClass::LoRaClass(SPIClass *_spi,uint8_t cs, uint8_t irq, uint8_t rst, uint8_t gpio){
void LoRaClass::setPins(SPIClass *spi,uint8_t cs, uint8_t irq, uint8_t rst, uint8_t gpio){
  //log_i("setPins cs=%d,irq=%d,rst=%d,gpio=%d",cs, irq, rst, gpio);
  //SPISettings _spiSettings(8E6, MSBFIRST, SPI_MODE0);
  SPISettings _spiSettings = SPISettings(8000000, MSBFIRST, SPI_MODE0);
  _spi = spi;
  _cs = cs;
  _irq = irq;
  _rst = rst;
  _gpio = gpio;
  _actMode = UNKNOWN;
  pGxModule = new GxModule(cs, irq, rst, gpio,*_spi,_spiSettings);
}

int16_t LoRaClass::sx1262_standby(uint8_t mode){
  // set RF switch (if present)
  pGxModule->setRfSwitchState(LOW, LOW);

  uint8_t data[] = {mode};
  return(SPIwriteCommand(0x80, data, 1));

}

int16_t LoRaClass::SPIwriteCommand(uint8_t* cmd, uint8_t cmdLen, uint8_t* data, uint8_t numBytes, bool waitForBusy) {
  return(LoRaClass::SPItransfer(cmd, cmdLen, true, data, NULL, numBytes, waitForBusy));
}

int16_t LoRaClass::SPIwriteCommand(uint8_t cmd, uint8_t* data, uint8_t numBytes, bool waitForBusy) {
  return(LoRaClass::SPItransfer(&cmd, 1, true, data, NULL, numBytes, waitForBusy));
}

int16_t LoRaClass::SPIreadCommand(uint8_t* cmd, uint8_t cmdLen, uint8_t* data, uint8_t numBytes, bool waitForBusy) {
  return(LoRaClass::SPItransfer(cmd, cmdLen, false, NULL, data, numBytes, waitForBusy));
}

int16_t LoRaClass::SPIreadCommand(uint8_t cmd, uint8_t* data, uint8_t numBytes, bool waitForBusy) {
  return(LoRaClass::SPItransfer(&cmd, 1, false, NULL, data, numBytes, waitForBusy));
}


int16_t LoRaClass::SPItransfer(uint8_t* cmd, uint8_t cmdLen, bool write, uint8_t* dataOut, uint8_t* dataIn, uint8_t numBytes, bool waitForBusy, uint32_t timeout) {
  // get pointer to used SPI interface and the settings
  SPIClass* spi = pGxModule->getSpi();
  SPISettings spiSettings = pGxModule->getSpiSettings();

  #ifdef RADIOLIB_VERBOSE
    uint8_t debugBuff[256];
  #endif

  // pull NSS low
  GxModule::digitalWrite(pGxModule->getCs(), LOW);

  // ensure BUSY is low (state machine ready)
  uint32_t start = GxModule::millis();
  while(GxModule::digitalRead(pGxModule->getGpio())) {
    GxModule::yield();
    if(GxModule::millis() - start >= timeout) {
      GxModule::digitalWrite(pGxModule->getCs(), HIGH);
      return(ERR_SPI_CMD_TIMEOUT);
    }
  }

  // start transfer
  spi->beginTransaction(spiSettings);

  // send command byte(s)
  for(uint8_t n = 0; n < cmdLen; n++) {
    spi->transfer(cmd[n]);
  }

  // variable to save error during SPI transfer
  uint8_t status = 0;

  // send/receive all bytes
  if(write) {
    for(uint8_t n = 0; n < numBytes; n++) {
      // send byte
      uint8_t in = spi->transfer(dataOut[n]);
      #ifdef RADIOLIB_VERBOSE
        debugBuff[n] = in;
      #endif

      // check status
      if(((in & 0b00001110) == SX126X_STATUS_CMD_TIMEOUT) ||
         ((in & 0b00001110) == SX126X_STATUS_CMD_INVALID) ||
         ((in & 0b00001110) == SX126X_STATUS_CMD_FAILED)) {
        status = in & 0b00001110;
        break;
      } else if(in == 0x00 || in == 0xFF) {
        status = SX126X_STATUS_SPI_FAILED;
        break;
      }
    }

  } else {
    // skip the first byte for read-type commands (status-only)
    uint8_t in = spi->transfer(SX126X_CMD_NOP);
    #ifdef RADIOLIB_VERBOSE
      debugBuff[0] = in;
    #endif

    // check status
    if(((in & 0b00001110) == SX126X_STATUS_CMD_TIMEOUT) ||
       ((in & 0b00001110) == SX126X_STATUS_CMD_INVALID) ||
       ((in & 0b00001110) == SX126X_STATUS_CMD_FAILED)) {
      status = in & 0b00001110;
    } else if(in == 0x00 || in == 0xFF) {
      status = SX126X_STATUS_SPI_FAILED;
    } else {
      for(uint8_t n = 0; n < numBytes; n++) {
        dataIn[n] = spi->transfer(SX126X_CMD_NOP);
      }
    }
  }

  // stop transfer
  spi->endTransaction();
  GxModule::digitalWrite(pGxModule->getCs(), HIGH);

  // wait for BUSY to go high and then low
  if(waitForBusy) {
    GxModule::delayMicroseconds(1);
    start = GxModule::millis();
    while(GxModule::digitalRead(pGxModule->getGpio())) {
      GxModule::yield();
      if(GxModule::millis() - start >= timeout) {
        status = SX126X_STATUS_CMD_TIMEOUT;
        break;
      }
    }
  }

  // print debug output
  #ifdef RADIOLIB_VERBOSE
    // print command byte(s)
    RADIOLIB_VERBOSE_PRINT("CMD\t");
    for(uint8_t n = 0; n < cmdLen; n++) {
      RADIOLIB_VERBOSE_PRINT(cmd[n], HEX);
      RADIOLIB_VERBOSE_PRINT('\t');
    }
    RADIOLIB_VERBOSE_PRINTLN();

    // print data bytes
    RADIOLIB_VERBOSE_PRINT("DAT");
    if(write) {
      RADIOLIB_VERBOSE_PRINT("W\t");
      for(uint8_t n = 0; n < numBytes; n++) {
        RADIOLIB_VERBOSE_PRINT(dataOut[n], HEX);
        RADIOLIB_VERBOSE_PRINT('\t');
        RADIOLIB_VERBOSE_PRINT(debugBuff[n], HEX);
        RADIOLIB_VERBOSE_PRINT('\t');
      }
      RADIOLIB_VERBOSE_PRINTLN();
    } else {
      RADIOLIB_VERBOSE_PRINT("R\t");
      // skip the first byte for read-type commands (status-only)
      RADIOLIB_VERBOSE_PRINT(SX126X_CMD_NOP, HEX);
      RADIOLIB_VERBOSE_PRINT('\t');
      RADIOLIB_VERBOSE_PRINT(debugBuff[0], HEX);
      RADIOLIB_VERBOSE_PRINT('\t')

      for(uint8_t n = 0; n < numBytes; n++) {
        RADIOLIB_VERBOSE_PRINT(SX126X_CMD_NOP, HEX);
        RADIOLIB_VERBOSE_PRINT('\t');
        RADIOLIB_VERBOSE_PRINT(dataIn[n], HEX);
        RADIOLIB_VERBOSE_PRINT('\t');
      }
      RADIOLIB_VERBOSE_PRINTLN();
    }
    RADIOLIB_VERBOSE_PRINTLN();
  #else
  #endif

  if (status){
    char sOut[LORAMAXSTRING];
    int pos = 0;
    pos += snprintf(&sOut[pos],LORAMAXSTRING-pos,"error ");
    if (write){
      pos += snprintf(&sOut[pos],LORAMAXSTRING-pos,"writing ");
    }else{
      pos += snprintf(&sOut[pos],LORAMAXSTRING-pos,"reading ");
    }
    pos += snprintf(&sOut[pos],LORAMAXSTRING-pos,"state=%d cmd=0X",status);
    for(uint8_t n = 0; n < cmdLen; n++) {
      pos += snprintf(&sOut[pos],LORAMAXSTRING-pos,"%02X",cmd[n]);      
    }
    log_e("%s",sOut);
  }
  // parse status
  switch(status) {
    case SX126X_STATUS_CMD_TIMEOUT:
      return(ERR_SPI_CMD_TIMEOUT);
    case SX126X_STATUS_CMD_INVALID:
      return(ERR_SPI_CMD_INVALID);
    case SX126X_STATUS_CMD_FAILED:
      return(ERR_SPI_CMD_FAILED);
    case SX126X_STATUS_SPI_FAILED:
      return(ERR_CHIP_NOT_FOUND);
    default:
      return(ERR_NONE);
  }
}

int16_t LoRaClass::writeRegister(uint16_t addr, uint8_t* data, uint8_t numBytes) {
  uint8_t cmd[] = { SX126X_CMD_WRITE_REGISTER, (uint8_t)((addr >> 8) & 0xFF), (uint8_t)(addr & 0xFF) };
  return(SPIwriteCommand(cmd, 3, data, numBytes));
}

int16_t LoRaClass::readRegister(uint16_t addr, uint8_t* data, uint8_t numBytes) {
  uint8_t cmd[] = { SX126X_CMD_READ_REGISTER, (uint8_t)((addr >> 8) & 0xFF), (uint8_t)(addr & 0xFF) };
  return(LoRaClass::SPItransfer(cmd, 3, false, NULL, data, numBytes, true));
}

void LoRaClass::sx1262CheckAndClearErrors(){
  int16_t err = sx1262GetDeviceErrors();
  if(err) {
    log_e("device-error %d",err);
    sx1262ClearDeviceErrors();
  }

}

int16_t LoRaClass::begin(float bw, uint8_t sf, uint8_t cr, uint8_t syncWord, int8_t power,uint8_t radioChip){
	//log_i("begin");
  receivedFlag = false;
	//enableInterrupt = false;
  bCalibrated = false;
  radioType = radioChip;
  int state = 0;
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
      {
      pGxModule->init(0x00); //we use SPI
      GxModule::pinMode(pGxModule->getIrq(), INPUT); //set IRQ as input
      GxModule::pinMode(pGxModule->getGpio(), INPUT); //set GPIO as input
      // run the reset sequence
      GxModule::pinMode(pGxModule->getRst(), OUTPUT);
      GxModule::digitalWrite(pGxModule->getRst(), LOW);
      GxModule::delay(1);
      GxModule::digitalWrite(pGxModule->getRst(), HIGH);
      //GxModule::delay(20); //wait 10ms until module is up
      // set mode to standby - SX126x often refuses first few commands after reset
      uint32_t start = GxModule::millis();
      uint8_t data[10];
      while(true) {
        // try to set mode to standby
        int16_t state = sx1262_standby();
        if(state == ERR_NONE) {
          // standby command successful
          break;
        }
        // standby command failed, check timeout and try again
        if(GxModule::millis() - start >= 3000) {
          // timed out, possibly incorrect wiring
          return(state);
        }
        // wait a bit to not spam the module
        GxModule::delay(10);
      }
      sx1262_standby(); //set Modele to Standby
      sx1262SetBufferBaseAddress(); //Set Buffer-Address to 0
      //set Modem to Lora
      data[0] = 0x01;
      state = SPIwriteCommand(0x8A, data, 1); 
      //SetRxTxFallbackMode to The radio goes into STDBY_RC mode after Tx or Rx
      data[0] = 0x20;
      state = SPIwriteCommand(0x93, data, 1);
      // set CAD parameters
      data[0] = 0x03;
      data[1] = 0x14;
      data[2] = 0x0A;
      data[3] = 0x00;
      data[4] = 0x00;
      data[5] = 0x00;
      data[6] = 0x00;
      state = SPIwriteCommand(0x88, data, 7);
      sx1262ClearIrqStatus(); //clear IRQ-Status
      sx1262SetDioIrqParams(0x00,0x00);//clear IRQ-Params
      //calibrate all Blocks
      data[0] = 0x7F;
      state = SPIwriteCommand(0x89, data, 1);
      sx1262_standby();
      // check SX126X_XOSC_START_ERR flag and clear it
      sx1262CheckAndClearErrors();
      //SetDIO3AsTcxoCtrl
      data[0] = 0x00; //DIO3 outputs 1.6 V to supply the TCXO
      // calculate delay
      uint32_t delayValue = (float)5000 / 15.625;
      data[1] = (uint8_t)((delayValue >> 16) & 0xFF);
      data[2] = (uint8_t)((delayValue >> 8) & 0xFF);
      data[3] = (uint8_t)(delayValue & 0xFF);      
      state = SPIwriteCommand(0x97, data, 4);
      //modulation-params
      data[0] = _sf; //<spreadingFact:SF=7>
      data[1] = 0x05; //<bw:5=250kHz>
      data[2] = _cr - 4; //<cr=4/8>
      data[3] = 0x00; //<lowDrOpt=off>
      SPIwriteCommand(0x8B, data, 4);
      //Sync Word
      data[0] = 0xF4;
      data[1] = 0x14;
      writeRegister(0x0740, data, 2);
      // read current clamping configuration
      uint8_t invertIq = 0;
      readRegister(0x0736, &invertIq, 1);
      // update with the new value
      invertIq |= 0x04;
      //invertIq = 0x09;
      writeRegister(0x0736, &invertIq, 1);  
      //packet-params
      data[0] = 0x00; // 12-symbol Präambel, expliziter header
      data[1] = 0x0C;
      data[2] = 0x00;
      data[3] = 0xFF; // Payload length - nachher setzen!
      data[4] = 0x01; // CRC on, Standard InvertIQ, 3x unused
      data[5] = 0x00; 
      data[6] = 0x00; 
      data[7] = 0x00;
      data[8] = 0x00; 
      SPIwriteCommand(0x8C, data, 9);
      sx1262CheckAndClearErrors();
      // calculate raw value for current
      uint8_t rawLimit = (uint8_t)(60.0 / 2.5);
      // update register
      writeRegister(0x08E7, &rawLimit, 1);
      //set DIO2 as rf-switch
      data[0] = 0x01;
      state = SPIwriteCommand(0x9D, data, 1);
      //set regulator to DC_DC
      data[0] = 0x01;
      SPIwriteCommand(0x96, data, 1);
      sx1262CalibrateImage(); //calibrate image for the frequency
      delay(4); //wait for calibration ready !!  Dauert 3.5ms, p.75
      sx1262SetFrequency(_freq);
      // SetPaConfig, p.76
      data[0] = 0x04; // +22 dBm
      data[1] = 0x07;
      data[2] = 0x00;
      data[3] = 0x01;
      SPIwriteCommand(0x95, data, 4);  
      //set output-power and ramp-time
      data[0] = power;
      data[1] = 0x04;
      SPIwriteCommand(0x8E, data, 2);
      GxModule::attachInterrupt(digitalPinToInterrupt(pGxModule->getIrq()), setFlag, RISING);
      sx1262CheckAndClearErrors();
      // read current clamping configuration
      uint8_t clampConfig = 0;
      readRegister(0x08D8, &clampConfig, 1);
      // update with the new value
      clampConfig |= 0x1E;
      writeRegister(0x08D8, &clampConfig, 1);  
      sx1262CheckAndClearErrors();
      sx1262_standby(); //switch to stand-by STDBY_XOSC
      return 0;
      break;
      }      
    case RADIO_SX1276:
      pGxModule->init(0x00); //we use SPI
      GxModule::pinMode(pGxModule->getIrq(), INPUT); //set IRQ as input
      GxModule::pinMode(pGxModule->getGpio(), INPUT); //set GPIO as input
      // run the reset sequence
      GxModule::pinMode(pGxModule->getRst(), OUTPUT);
      GxModule::digitalWrite(pGxModule->getRst(), LOW);
      GxModule::delayMicroseconds(200);
      GxModule::digitalWrite(pGxModule->getRst(), HIGH);
      GxModule::delay(6); //wait min 5 ms until module is ready
      GxModule::attachInterrupt(digitalPinToInterrupt(pGxModule->getIrq()), setFlag, RISING);
      sx1276setOpMode(SX1276_OPMODE_FSK_SLEEP);
      uint8_t rVersion = pGxModule->SPIreadRegister(SX127X_REG_VERSION);
      log_i("sx1276-Version %d",rVersion);
      return state;
      break;
  }
  return 0;
}

int16_t LoRaClass::sx1262ClearIrqStatus(){
  uint8_t data[] = { 0x03,0xFF };
  return (SPIwriteCommand(0x02, data, 2));
}

uint16_t LoRaClass::sx1262ReadIrQ(){
  uint8_t data[] = { 0x00, 0x00 };
  SPIreadCommand(0x12, data, 2);
  uint16_t irq = ((uint16_t)(data[0]) << 8) | data[1];
  return irq;
}

int16_t LoRaClass::sx1262ReadData(uint8_t* buffer, size_t len){
  int16_t ret = 0;
  sx1262_standby(0x01);

  uint16_t irq = sx1262ReadIrQ();
  if((irq & SX126X_IRQ_CRC_ERR) || (irq & SX126X_IRQ_HEADER_ERR)) {
    ret = -1;
  }

  uint8_t cmd[] = { SX126X_CMD_READ_BUFFER, SX126X_CMD_NOP };
  SPIreadCommand(cmd, 2, buffer, len);          
  sx1262ClearIrqStatus();
  return ret;
}

int16_t LoRaClass::readData(uint8_t* data, size_t len){
  //log_i("readData");
  int16_t ret = 0;
  switch (radioType){
    case RADIO_SX1262:
      if (_fskMode){
        if (len == 26){
          uint8_t rx_frame[len*2];
          int16_t ret = 0;
          ret = sx1262ReadData(rx_frame,len*2);
          uint8_t val1, val2;
          for (int i = 0;i < len * 2; i++){
            val1 = ManchesterDecode[rx_frame[i]];
            val2 = ManchesterDecode[rx_frame[i+1]];
            data[i>>1] = ((val1 & 0x0F) << 4) | (val2 & 0x0F);
            i++;
          }
          return ret;
        }else{
          return ERR_UNKNOWN;
        }
      }else{
        return sx1262ReadData(data,len);;
      }
    case RADIO_SX1276:

      if (_fskMode){
        // put module to standby
        //we have to read the register in order to read data
        if (len == 26){
          //pGxModule->SPIreadRegisterBurst(0x00, len, data,true);
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
        sx1276setOpMode(SX1276_MODE_STANDBY); //RegOpMode --> set Module to standby
        // read packet data
        pGxModule->SPIreadRegisterBurst(0x00, len, data);     //REG_FIFO      
        if(pGxModule->SPIgetRegValue(SX127X_REG_IRQ_FLAGS, 5, 5) == SX127X_CLEAR_IRQ_FLAG_PAYLOAD_CRC_ERROR) {
          ret = ERR_CRC_MISMATCH;
        }
      }
      return ret;
  }
  return -1;
}

uint32_t LoRaClass::sx1262GetPacketStatus() {
  uint8_t data[3] = {0, 0, 0};
  SPIreadCommand(0x14, data, 3);
  uint32_t retData = (((uint32_t)data[0]) << 16) | (((uint32_t)data[1]) << 8) | (uint32_t)data[2];
  //log_i("Packetstatus = 0X%06X",retData);
  return(retData);
}

int16_t LoRaClass::sx1262GetStats(){
  uint8_t data[6] = {0, 0, 0, 0, 0, 0};
  SPIreadCommand(0x10, data, 6);
  log_i("Packetstatus = 0X%02X 0X%02X 0X%02X 0X%02X 0X%02X 0X%02X",data[0],data[1],data[2],data[3],data[4],data[5]);
  return(0);
}


float LoRaClass::getRSSI(){
  //log_i("getRssi");
  switch (radioType){
    case RADIO_SX1262:
      {
      // get last packet RSSI from packet status
      uint32_t packetStatus = sx1262GetPacketStatus();
      uint8_t rssiPkt = packetStatus & 0xFF;
      return(-1.0 * rssiPkt/2.0);
      break;
      }
    case RADIO_SX1276:
      if (_fskMode){
        return rssiValue;
      }else{
        float lastPacketRSSI;
        // RSSI calculation uses different constant for low-frequency and high-frequency ports
        if(_freq < 868000000) {
          lastPacketRSSI = -164 + pGxModule->SPIgetRegValue(0x1A); //REG_PKT_RSSI_VALUE
        } else {
          lastPacketRSSI = -157 + pGxModule->SPIgetRegValue(0x1A); //REG_PKT_RSSI_VALUE
        }
        float lastPacketSNR = getSNR();
        if(lastPacketSNR < 0.0) {
          lastPacketRSSI += lastPacketSNR;
        }
        return lastPacketRSSI;
      }
  }
  return 0.0;
}

int16_t LoRaClass::sx1276setOpMode(uint8_t mode){
  int16_t ret = 0;
  if (mode == SX1276_MODE_RX_CONTINUOUS){
    ret = pGxModule->SPIsetRegValue(SX127X_REG_OP_MODE, mode, 2, 0, 200); //set Register wait max. 200ms
  }else{
    ret = pGxModule->SPIsetRegValue(SX127X_REG_OP_MODE, mode, 2, 0, 10); //RegOpMode --> set op-mode wait max. 10ms
  }
  if (ret) log_e("sx1276 mode=%d error=%d",mode,ret);
  /*
  for (int i = 0; i < 3; i++){
    ret = pGxModule->SPIsetRegValue(SX127X_REG_OP_MODE, mode, 2, 0, 200); //RegOpMode --> set op-mode
    if (ret == 0) break;
  } 
  */
  //if (ret){
  //  log_e("sx1276 error set OP-Mode %d",ret);    
  //}
  //delay(10); //wait 10ms.
  return ret;
}

float LoRaClass::getSNR(){
  switch (radioType){
    case RADIO_SX1276:
      if (_fskMode) return 0.0;
      // spread-spectrum modulation signal can be received below noise floor
      // check last packet SNR and if it's less than 0, add it to reported RSSI to get the correct value
      // get SNR value
      return (int8_t)pGxModule->SPIgetRegValue(0x19) / 4.0; //REG_PKT_SNR_VALUE
  }
  return 0.0;
}

void LoRaClass::printReg(uint8_t reg){
	uint8_t regVal = pGxModule->SPIreadRegister(reg);
  Serial.printf("REG=%02X;%02X\n",reg,regVal);
}

int16_t LoRaClass::sx1276setRxBandwidth(float rxBw){
  int16_t state = 0;
  // calculate exponent and mantissa values
  for(uint8_t e = 7; e >= 1; e--) {
    for(int8_t m = 2; m >= 0; m--) {
      float point = (SX127X_CRYSTAL_FREQ * 1000000.0)/(((4 * m) + 16) * ((uint32_t)1 << (e + 2)));
      if(abs(rxBw - ((point / 1000.0) + 0.05)) <= 0.5) {
        // set Rx bandwidth during AFC
        state = pGxModule->SPIsetRegValue(0x13, (m << 3) | e, 4, 0);
        GXMODULE_ASSERT(state);

        // set Rx bandwidth
        state = pGxModule->SPIsetRegValue(0x12, (m << 3) | e, 4, 0);
        return(state);
      }
    }
  }
  return 0;
}

int16_t LoRaClass::sx1262SetFrequency(uint32_t freq){
  //log_i("frequ=%dHz",_freq);
  uint64_t frf = ((uint64_t)_freq << 25) / 32000000;
  uint8_t data[4];
  data[0] = (uint8_t)((frf >> 24) & 0xFF);
  data[1] = (uint8_t)((frf >> 16) & 0xFF);
  data[2] = (uint8_t)((frf >> 8) & 0xFF);
  data[3] = (uint8_t)(frf & 0xFF);
  return SPIwriteCommand(0x86, data, 4);
}

void LoRaClass::checkRet(int16_t value){
  if (value){
    log_e("ret-error:%d",value);
  }
}

void LoRaClass::sx1276_setPower(int8_t power){
  if (power > 17) { // use high-power +20dBm option
    if (power > 20) {
        power = 20;
    }
    pGxModule->SPIwriteRegister(SX127X_REG_PaDac, 0x87); // high power
    pGxModule->SPIwriteRegister(SX127X_REG_PA_CONFIG, 0x80 | (power - 5)); // BOOST (5..20dBm)
  } else {
    if (power < 2) {
        power = 2;
    }
    pGxModule->SPIwriteRegister(SX127X_REG_PaDac, 0x84); // normal power
    pGxModule->SPIwriteRegister(SX127X_REG_PA_CONFIG, 0x80 | (power - 2)); // BOOST (2..17dBm)
  }
  // set 50us PA ramp-up time
  pGxModule->SPIwriteRegister(SX127X_REG_PA_RAMP, 0b00011000); // unused=000, LowPnTxPllOff=1, PaRamp=1000    
}

int16_t LoRaClass::switchFSK(uint32_t frequency){
  //log_i("switchFSK frequ=%dHz,power=%d",frequency,maxFskPower);
  //Bitrate=100kHz
  //BT0.5
  //BW=117khz
  // preamble len 24 Bit
  //Sync-Word-Len 7Byte --> 56 Bit
  //Msg-Len 52 bytes --> 416 Bit
  //total 496Bit --> airtime 0.00496s (~5ms)
  
  //uint32_t tBegin = micros();
  _freq = frequency;
  int16_t ret = 0;
  uint8_t syncWord[] = {0x99, 0xA5, 0xA9, 0x55, 0x66, 0x65, 0x96};	
  //log_i("switchFSK %d frequ=%.2f",millis(),_freq);
  switch (radioType){
    case RADIO_SX1262:
      {
      uint8_t data[10];
      sx1262_standby(0x01); //switch to stand-by STDBY_XOSC
      sx1262SetBufferBaseAddress();
      data[0] = 0x00;
      SPIwriteCommand(0x8A, data, 1); //set Modem to GFSK
      // set CAD parameters
      data[0] = 0x03;
      data[1] = 0x14;
      data[2] = 0x0A;
      data[3] = 0x00;
      data[4] = 0x00;
      data[5] = 0x00;
      data[6] = 0x00;
      SPIwriteCommand(0x88, data, 7);
      sx1262ClearIrqStatus();
      sx1262SetDioIrqParams(0x00,0x00);//clear IRQ-Params
      // calculate raw value for current
      uint8_t rawLimit = (uint8_t)(60.0 / 2.5);
      // update register
      writeRegister(0x08E7, &rawLimit, 1);
      //set regulator to DC_DC
      data[0] = 0x01;
      SPIwriteCommand(0x96, data, 1);
      //modulation-params
      // br = 32 * 32000000 / 100000 = 10240 = 0x002800
      data[0] = 0x00;
      data[1] = 0x28;
      data[2] = 0x00;
      // 0x09=BT0.5, 0x0b=bw117kHz,
      data[3] = 0x09;
      data[4] = 0x0B; //bw=117khz
      //data[4] = 0x1A; //bw=156.2khz
      // fdev = (50 kHz * 2**25) / 32000000 = 52428 = 0xcccc
      data[5] = 0x00;
      data[6] = 0xCC;
      data[7] = 0xCC;
      SPIwriteCommand(0x8B, data, 8);

      //packet-params
      // preamble len 24, detector 0x05 16 bits
      data[0] = 0x00;
      data[1] = 24;
      data[2] = 0x04;
      //data[1] = 0x08;
      //data[2] = 0x05;
      // sync word len (56 bits), addr comp off, fixed len
      data[3] = 56;
      data[4] = 0x00;
      data[5] = 0x00;
      // payload len = 52 bytes (2 * (24+2)), includes CRC 
      data[6] = 52; 
      // no integrated CRC check (not possible due to manchester encoding)
      data[7] = 0x01;
      data[8] = 0x00; 
      SPIwriteCommand(0x8C, data, 9);

      sx1262SetBufferBaseAddress();
      //setFrequency
      sx1262SetFrequency(_freq);
      //Sync Word GFSK
      data[0] = 0x99;
      data[1] = 0xA5;
      data[2] = 0xA9;
      data[3] = 0x55;
      data[4] = 0x66;
      data[5] = 0x65;
      data[6] = 0x96;
      writeRegister(0x06C0, syncWord, 7);
      ret = 0;
      break;
      }
    case RADIO_SX1276:
      sx1276setOpMode(SX1276_MODE_SLEEP); //RegOpMode --> set Module to sleep      
      ret = pGxModule->SPIsetRegValue(SX127X_REG_OP_MODE, 0b00000000, 6, 5, 200); //set modulation to FSK
      if (ret) log_e("sx1276 error set OP-Mode 1 %d",ret);
      ret = pGxModule->SPIsetRegValue(SX127X_REG_OP_MODE, 0b00000000, 3, 3, 200); //clear low frequency-mode
      if (ret) log_e("sx1276 error set OP-Mode 2 %d",ret);
      ret = pGxModule->SPIsetRegValue(SX127X_REG_OP_MODE, 0b00000000, 7, 7, 200); //RegOpMode --> set modem to FSK
      if (ret) log_e("sx1276 error set OP-Mode 3 %d",ret);

      sx1276setOpMode(SX1276_MODE_STANDBY); //RegOpMode --> set Module to standby   
      //set bitrate to 100kBps
      pGxModule->SPIwriteRegister(0x02,0x01); //RegBitrateMsb 100kbps
      pGxModule->SPIwriteRegister(0x03,0x40); //RegBitrateLsb
      pGxModule->SPIwriteRegister(0x5D,0x00); //RegBitrateFrac
      //set deviation to 50kHz
      pGxModule->SPIwriteRegister(0x04,0x03); //RegFdevMsb Frequency deviation +/- 50kHz
      pGxModule->SPIwriteRegister(0x05,0x33); //RegFdevLsb Fdev = 50000Hz / 61Hz = 819 (0x333)
      //calculate register values
      configChannel(_freq);

      sx1276_setPower(maxFskPower);

      pGxModule->SPIwriteRegister(0x0B,0x2B); //RegOcp
      pGxModule->SPIwriteRegister(0x0C,0x23); //RegLna max gain, default LNA current
      //pGxModule->SPIwriteRegister(0x0D,0x0E); //RegRxConfig AFC off, AGC on, trigger on preamble?!?
      //pGxModule->SPIwriteRegister(0x0D,0x1E); //RegRxConfig Auto AFC on, AGC on, trigger on preamble?!?
      pGxModule->SPIwriteRegister(0x0D,0x09); //RegRxConfig Auto AFC off, LNA controlled by AGC
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
      pGxModule->SPIwriteRegister(0x24,0x05); //RegOsc FXOSC/32 FXOSC = 32Mhz --> 1Mhz
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

      /*
      int i = 0;
      for (i = 0x00;i <= 0x19;i++){
        printReg(i);
      }
      for (i = 0x1A; i <= 0x44;i++ ){
        printReg(i);
      }
      i = 0x4B; printReg(i);
      i = 0x4D; printReg(i);
      i = 0x5B; printReg(i);
      i = 0x5D; printReg(i);
      for (i = 0x61; i <= 0x64;i++ ){
        printReg(i);
      }
      */

      break;
  }
  enableInterrupt = false;
  receivedFlag = false;
  _fskMode = true;
  //log_i("FSK-Mode On %d",micros()-tBegin);
  return 0;
}

bool LoRaClass::isFskMode(void){
  return _fskMode;
}

bool LoRaClass::isReceiving(){
  //log_i("is receiving");
  switch (radioType){
    case RADIO_SX1262:
      return false;
      break;
    case RADIO_SX1276:
      uint8_t reg = pGxModule->SPIgetRegValue(0x3E);
      log_i("regIRQ=%d",reg);
      return false;
      break;
  }
  return false;
}

uint16_t LoRaClass::sx1262GetDeviceErrors() {
  uint8_t data[2] = {0, 0};
  SPIreadCommand(0x17, data, 2);
  uint16_t opError = (((uint16_t)data[0] & 0xFF) << 8) & ((uint16_t)data[1]);
  return(opError);
}

int16_t LoRaClass::sx1262ClearDeviceErrors() {
  uint8_t data[2] = {0x00, 0x00};
  return(SPIwriteCommand(0x08, data, 2));
}

int16_t LoRaClass::sx1262SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask) {
  uint8_t data[8] = {(uint8_t)((irqMask >> 8) & 0xFF), (uint8_t)(irqMask & 0xFF),
                     (uint8_t)((dio1Mask >> 8) & 0xFF), (uint8_t)(dio1Mask & 0xFF),
                     (uint8_t)((dio2Mask >> 8) & 0xFF), (uint8_t)(dio2Mask & 0xFF),
                     (uint8_t)((dio3Mask >> 8) & 0xFF), (uint8_t)(dio3Mask & 0xFF)};
  return(SPIwriteCommand(0x08, data, 8));
}

int16_t LoRaClass::switchLORA(uint32_t frequency,uint16_t loraBandwidth){
  //log_i("switchLora");
  _freq = frequency;
  _bw = float(loraBandwidth);
  //log_i("frequ=%.1f,bw=%d",frequency,loraBandwidth);
  int16_t ret = -1;
  uint8_t regData = 0x00;
  //log_i("switchLora %d frequ=%.2f",millis(),_freq);
  switch (radioType){
    case RADIO_SX1262:
      {
      uint8_t data[10];

      sx1262_standby(0x01); //switch to stand-by STDBY_XOSC
      data[0] = 0x01;
      SPIwriteCommand(0x8A, data, 1); //set Modem to Lora

      sx1262SetBufferBaseAddress();

      sx1262SetFrequency(_freq);

      //modulation-params
      data[0] = _sf; //<spreadingFact:SF=7>
      switch (loraBandwidth){
        case 125:
          data[1] = 0x04; //<bw:4=125kHz>
          break;
        case 250:
          data[1] = 0x05; //<bw:5=250kHz>
          break;
        case 500:
          data[1] = 0x06; //<bw:6=500kHz>
          break;
      }
      data[2] = _cr - 4; //<cr=4/8>
      data[3] = 0x00; //<lowDrOpt=off>
      SPIwriteCommand(0x8B, data, 4);

      //packet-params
      data[0] = 0x00; // 12-symbol Präambel, expliziter header
      data[1] = 0x0C;
      data[2] = 0x00;
      data[3] = 0xFF; // Payload length - nachher setzen!
      data[4] = 0x01; // CRC on, Standard InvertIQ, 3x unused
      data[5] = 0x00; 
      data[6] = 0x00; 
      data[7] = 0x00;
      data[8] = 0x00; 
      SPIwriteCommand(0x8C, data, 9);

      //set IRQ to RX-Done
      sx1262SetDioIrqParams(SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_CRC_ERR | SX126X_IRQ_HEADER_ERR | SX126X_IRQ_HEADER_VALID | SX126X_IRQ_SYNC_WORD_VALID | SX126X_IRQ_PREAMBLE_DETECTED, SX126X_IRQ_RX_DONE);

      //Sync Word
      data[0] = 0xF4;
      data[1] = 0x14;
      writeRegister(0x0740, data, 2);

      // clear IRQ
      sx1262ClearIrqStatus();

      ret = 0;
      break;
      }
    case RADIO_SX1276:
      sx1276setOpMode(SX1276_MODE_SLEEP); //RegOpMode --> set Module to sleep
      ret = pGxModule->SPIsetRegValue(SX127X_REG_OP_MODE, 0b00000000, 6, 6, 200); //AccessSharedReg
      if (ret) log_e("sx1276 error set OP-Mode 1 %d",ret);
      ret = pGxModule->SPIsetRegValue(SX127X_REG_OP_MODE, 0b00000000, 3, 3, 200); //clear low frequency-mode
      if (ret) log_e("sx1276 error set OP-Mode 2 %d",ret);
      ret = pGxModule->SPIsetRegValue(SX127X_REG_OP_MODE, 0b10000000, 7, 7, 5); //RegOpMode --> set modem to LORA
      if (ret) log_e("sx1276 error set OP-Mode 3 %d",ret);
      configChannel(_freq);
      sx1276_setPower(maxLoraPower);

      pGxModule->SPIwriteRegister(0x0B,0x2B); //RegOcp OCP enabled, max. 100mA
      pGxModule->SPIwriteRegister(0x0C,0x23); //RegLna G1 (max gain), Boost on (150% LNA current)
      pGxModule->SPIwriteRegister(0x0D,0x01); //RegFifoAddrPtr
      pGxModule->SPIwriteRegister(0x0E,0x00); //RegFifoTxBaseAddr
      pGxModule->SPIwriteRegister(0x0F,0x00); //RegFifoRxBaseAddr
      pGxModule->SPIwriteRegister(0x11,0x00); //RegIrqFlags
      regData = 0x08; //RegModemConfig1 Explicit Header, CR 4/8
      switch (loraBandwidth){
        case 125:
          regData += 0x70; //<bw 125kHz>
          break;
        case 250:
          regData += 0x80; //<bw 250kHz>
          break;
        case 500:
          regData += 0x90; //<bw 500kHz>
          break;
      }
      pGxModule->SPIwriteRegister(0x1D,regData); //RegModemConfig1 Explicit Header, CR 4/8, BW 250kHz
      pGxModule->SPIwriteRegister(0x1E,0x74); //RegModemConfig2 CRC enabled, 128 chips / symbol
      pGxModule->SPIwriteRegister(0x1F,0x64); //RegSymbTimeoutLsb
      pGxModule->SPIwriteRegister(0x20,0x00); //RegPreambleMsb
      pGxModule->SPIwriteRegister(0x21,0x07); //RegPreambleLsb
      pGxModule->SPIwriteRegister(0x22,0x13); //RegPayloadLength
      pGxModule->SPIwriteRegister(0x23,0xFF); //RegMaxPayloadLength
      pGxModule->SPIwriteRegister(0x24,0x00); //RegHopPeriod
      pGxModule->SPIwriteRegister(0x26,0x04); //RegModemConfig3 AGC on
      pGxModule->SPIwriteRegister(0x27,0x00); //Data rate offset value, used in conjunction with AFC
      pGxModule->SPIwriteRegister(0x31,0x43); //RegDetectOptimize
      pGxModule->SPIwriteRegister(0x33,0x27); //RegInvertIQ
      pGxModule->SPIwriteRegister(0x36,0x03); //RegHighBWOptimize1
      pGxModule->SPIwriteRegister(0x37,0x0A); //RegDetectionThreshold SF7
      pGxModule->SPIwriteRegister(0x39,0xF1); //set sync-word Sync-Word F1
      pGxModule->SPIwriteRegister(0x3A,0x52); //RegHighBWOptimize1
      pGxModule->SPIwriteRegister(0x3B,0x1D); //RegInvertIQ2
      pGxModule->SPIwriteRegister(0x40,0x00); //RegDioMapping1
      pGxModule->SPIwriteRegister(0x41,0x00); //RegDioMapping2
      pGxModule->SPIwriteRegister(0x4B,0x09); //RegTcxo
      //pGxModule->SPIwriteRegister(0x4D,0x84); //RegPaDac
      pGxModule->SPIwriteRegister(0x61,0x19); //RegAgcRef
      pGxModule->SPIwriteRegister(0x62,0x0C); //RegAgcThresh1
      pGxModule->SPIwriteRegister(0x63,0x4B); //RegAgcThresh2
      pGxModule->SPIwriteRegister(0x64,0xCC); //RegAgcThresh3
      pGxModule->SPIwriteRegister(0x70,0xD0); //RegPll
      sx1276setOpMode(SX1276_MODE_STANDBY);  //RegOpMode --> set Module to stand-by

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
  //log_i("getAirLimit");
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

int16_t LoRaClass::sx1262SetBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress) {
  uint8_t data[2] = {txBaseAddress, rxBaseAddress};
  return(SPIwriteCommand(0x8F, data, 2));
}

int16_t LoRaClass::startReceive(){
  //log_i("start receive");
  #if LORA_RX_DEBUG > 0
  uint32_t tBegin = micros();
  char Buffer[500];
  #endif
  int16_t iRet = 0;
  //delay(5);
  prevIrqFlags = 0;
  
  switch (radioType){
    case RADIO_SX1262:
      {
      //set IRQ to RX-Done
      sx1262SetDioIrqParams(SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_CRC_ERR | SX126X_IRQ_HEADER_ERR | SX126X_IRQ_HEADER_VALID | SX126X_IRQ_SYNC_WORD_VALID | SX126X_IRQ_PREAMBLE_DETECTED, SX126X_IRQ_RX_DONE);

      sx1262SetBufferBaseAddress();

      sx1262ClearIrqStatus(); //clear irq-status

      // set RF switch (if present)
      pGxModule->setRfSwitchState(HIGH, LOW);
      receivedFlag = false;
      enableInterrupt = true;
      //start RX
      sx1262SetCmdRx();

      break;
      }
    case RADIO_SX1276:
        if (_fskMode){
          sx1276setOpMode(SX1276_MODE_STANDBY);//RegOpMode --> set Module to standby
          if (!bCalibrated){
            log_i("start calib image");
            //calib image
            uint32_t start = millis();
            pGxModule->SPIwriteRegister(0x3B, 0x40); //REG_IMAGE_CAL            
            while(pGxModule->SPIgetRegValue(0x3B) & 0x20) {
              GxModule::yield();
              if(millis() - start > 10) {
                log_e("timeout calibrating image");
                break;
              }
            }
            bCalibrated = true;
          }
          pGxModule->SPIsetRegValue(0x40, 0x00, 7, 6); //REG_DIO_MAPPING_1 --> DIO0_PACK_PAYLOAD_READY
          pGxModule->SPIwriteRegister(0x3E, 0b11111111); //REG_IRQ_FLAGS_1
          pGxModule->SPIwriteRegister(0x3F, 0b11111111); //REG_IRQ_FLAGS_2
          // set RF switch (if present)
          pGxModule->setRfSwitchState(HIGH, LOW);
          //iRet = sx1276setOpMode(SX1276_MODE_FS_MODE_RX); //RegOpMode --> set Module to RXCONTINUOUS
          receivedFlag = false;
          enableInterrupt = true;
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
          receivedFlag = false;
          enableInterrupt = true;
          iRet = sx1276setOpMode(SX1276_MODE_RX_CONTINUOUS); //RegOpMode --> set Module to RXCONTINUOUS
          if (iRet){
            log_e("LORA-Mode set OP-Mode RX_Continous failed");
          }
        }
      break;
  }
  #if LORA_RX_DEBUG > 10
    log_i("%d start Receive",millis());
  #endif
  #if LORA_RX_DEBUG > 0
  sprintf(Buffer,"startReceive %dus\n",int(micros()-tBegin));
  Serial.print(Buffer);
  #endif
  return iRet;
}

size_t LoRaClass::getPacketLength(){
  //log_i("getPacketLength");
  size_t tRet = 0;
  //log_i("getPacketLength %d",radioType);
  if (_fskMode) return FSK_PACKET_LENGTH;
  switch (radioType){
    case RADIO_SX1262:
      {
      uint8_t rxBufStatus[2] = {0, 0};
      SPIreadCommand(0x13, rxBufStatus, 2);
      tRet = (size_t)rxBufStatus[0];
      if (_fskMode){
        tRet /= 2; //cause of manchester-decoding we get always double of the Length
      }    
      break;
      }
    case RADIO_SX1276:
      if (_fskMode){
        tRet = pGxModule->SPIreadRegister(0x32); //REG_PAYLOAD_LENGTH_FSK
      }else{
        tRet = pGxModule->SPIreadRegister(0x13); //REG_RX_NB_BYTES
      }
      break;
  }
  return tRet;
}

int16_t LoRaClass::setCodingRate(uint8_t cr){
  //log_i("setCodingRate %d",cr);
  int16_t iRet = 0;
  if (_cr != cr){
    _cr = cr;
    switch (radioType){
      case RADIO_SX1262:
        log_i("setCodingRate SX1262");
        //modulation-params
        uint8_t data[4];
        data[0] = _sf; //<spreadingFact:SF=7>
        data[1] = 0x05; //<bw:5=250kHz>
        data[2] = _cr - 4; //<cr=4/8>
        data[3] = 0x00; //<lowDrOpt=off>
        iRet = SPIwriteCommand(0x8B, data, 4);
        break;
      case RADIO_SX1276:
        //log_i("setCodingRate SX1276");
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
        break;
    }
  }
  return iRet;
}

uint32_t LoRaClass::sx1262GetTimeOnAir(size_t len) {
  // everything is in microseconds to allow integer arithmetic
  // some constants have .25, these are multiplied by 4, and have _x4 postfix to indicate that fact
  if(!_fskMode) {
    uint32_t symbolLength_us = ((uint32_t)(1000 * 10) << _sf) / (_bw * 10) ;
    uint8_t sfCoeff1_x4 = 17; // (4.25 * 4)
    uint8_t sfCoeff2 = 8;
    if(_sf == 5 || _sf == 6) {
      sfCoeff1_x4 = 25; // 6.25 * 4
      sfCoeff2 = 0;
    }
    uint8_t sfDivisor = 4*_sf;
    if(symbolLength_us >= 16000) {
      sfDivisor = 4*(_sf - 2);
    }
    const int8_t bitsPerCrc = 16;
    const int8_t N_symbol_header = 20; //explicit header

    // numerator of equation in section 6.1.4 of SX1268 datasheet v1.1 (might not actually be bitcount, but it has len * 8)
    //crc-on
    int16_t bitCount = (int16_t) 8 * len + 1 * bitsPerCrc - 4 * _sf  + sfCoeff2 + N_symbol_header;
    if(bitCount < 0) {
      bitCount = 0;
    }
    // add (sfDivisor) - 1 to the numerator to give integer CEIL(...)
    uint16_t nPreCodedSymbols = (bitCount + (sfDivisor - 1)) / (sfDivisor);

    // preamble can be 65k, therefore nSymbol_x4 needs to be 32 bit
    uint32_t nSymbol_x4 = (_preambleLength + 8) * 4 + sfCoeff1_x4 + nPreCodedSymbols * (_cr + 4) * 4;

    return((symbolLength_us * nSymbol_x4) / 4);
  } else {
    //bitrate of FSK is 100kbps
    // calculate raw bit rate value
    uint32_t brRaw = (uint32_t)((32.0 * 1000000.0 * 32.0) / (_br * 1000.0));
    return((len * 8 * brRaw) / (32.0 * 32));
  }
}

uint8_t LoRaClass::sx1262GetStatus() {
  uint8_t data = 0;
  SPIreadCommand(0xC0, &data, 1);
  //log_i("Status = 0X%02X",data);
  return(data);
}

void LoRaClass::sx1262SetCmdTx(){
  uint8_t data[] = { 0x00,0x00,0x00 };
  SPIwriteCommand(SX126X_CMD_SET_TX, data, 3);  
}

void LoRaClass::sx1262SetCmdRx(){
  uint8_t data[] = { 0xFF,0xFF,0xFF };
  SPIwriteCommand(SX126X_CMD_SET_RX, data, 3);  
}

int16_t LoRaClass::sx1262Transmit(uint8_t* buffer, size_t len, uint8_t addr){
  int16_t state = 0;
  uint8_t data[10];
  uint32_t timeout = 0;

  sx1262_standby(0x01);

  // get currently active modem
  if(_fskMode) {
    // calculate timeout (500% of expected time-on-air)
    timeout = sx1262GetTimeOnAir(len) * 5;
  } else {
    // calculate timeout (150% of expected time-on-air)
    timeout = (sx1262GetTimeOnAir(len) * 3) / 2;
  }
  //log_i("timeout=%d",timeout);
  sx1262SetDioIrqParams(SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT, SX126X_IRQ_TX_DONE);
  sx1262SetBufferBaseAddress(); //clear Buffer-Base-address
  // write packet to buffer
  uint8_t cmd[] = { 0x0E, 0x00 };
  SPIwriteCommand(cmd, 2, buffer, len);  

  //packet-params
  if(!_fskMode) {
    // Modulation Quality with 500 kHz LoRa® Bandwidth
    uint8_t value = 0;
    readRegister(0x0889, &value, 1);
    if (_bw == 500.0){
      // set Bit 2    
      value |= 0x04;
    }else{
      // set Bit 2    
      value &= 0xFD; //clear bit 2
    }
    writeRegister(0x0889, &value, 1);  

    data[0] = 0x00; // 12-symbol Präambel, expliziter header
    data[1] = 0x0C;
    data[2] = 0x00;
    data[3] = len; // Payload length - nachher setzen!
    data[4] = 0x01; // CRC on, Standard InvertIQ, 3x unused
    data[5] = 0x00; 
    data[6] = 0x00; 
    data[7] = 0x00;
    data[8] = 0x00; 
    SPIwriteCommand(0x8C, data, 9);
  }

  sx1262ClearIrqStatus();


  // SetPaConfig, p.76
  data[0] = 0x04; // +22 dBm
  data[1] = 0x07;
  data[2] = 0x00;
  data[3] = 0x01;
  SPIwriteCommand(0x95, data, 4);  

  //SetTxParams, p.84
  data[0] = 0x16; // +22 dBm, 200us ramp
  data[1] = 0x04;
  SPIwriteCommand(0x8E, data, 2);  



  // set RF switch (if present)
  pGxModule->setRfSwitchState(LOW, HIGH);

  // start transmission
  sx1262SetCmdTx();

  // wait for BUSY to go low (= PA ramp up done)
  while(GxModule::digitalRead(pGxModule->getGpio())) {
    GxModule::yield();
  }

  // wait for packet transmission or timeout
  uint32_t start = GxModule::micros();
  while(!GxModule::digitalRead(pGxModule->getIrq())) {
    GxModule::yield();
    if(GxModule::micros() - start > timeout) {
      sx1262ClearIrqStatus();
      sx1262_standby(0x01);
      return(ERR_TX_TIMEOUT);
    }
  }

  // clear interrupt flags
  state = sx1262ClearIrqStatus();
  GXMODULE_ASSERT(state);

  // set mode to standby to disable transmitter
  state = sx1262_standby(0x01);

  return(state);
}

int LoRaClass::sx_channel_free4tx(){
	/* in case of receiving, is it ongoing? */
  uint8_t mode = 0;
  uint16_t irq = 0;
  uint8_t sx1262Cmd = 0;
  uint8_t chipMode = 0;
  if (_fskMode){ //todo --> check also in FSK-Mode
    return ERR_NONE;
  }
  switch (radioType){
    case RADIO_SX1262:
      mode = sx1262GetStatus(); //get Status
      /* are we transmitting anyway? */
      chipMode = mode & 0xF0; //we neee the chip-mode
      if ((chipMode != SX126X_STATUS_MODE_RX) && (chipMode != SX126X_STATUS_MODE_STDBY_XOSC)){
        log_i("ChipMode=0X%02X",chipMode);
      }      
      if(chipMode == SX126X_STATUS_MODE_TX)
        return ERR_TX_TX_ONGOING;

      /* in case of receiving, is it ongoing? */
      for(uint16_t i=0; i<4 && (chipMode == SX126X_STATUS_MODE_RX); i++){
        irq = sx1262ReadIrQ();
        if((irq & SX126X_IRQ_PREAMBLE_DETECTED) || (irq & SX126X_IRQ_SYNC_WORD_VALID || (irq & SX126X_IRQ_HEADER_VALID))) {
          return ERR_TX_RX_ONGOING;
        }
        delay(1);
      }
      break;
    case RADIO_SX1276:
      mode = pGxModule->SPIreadRegister(SX127X_REG_OP_MODE); //read OP-Mode
      mode &= SX127X_LORA_MODE_MASK;
      /* are we transmitting anyway? */
      if(mode == SX127X_LORA_TX_MODE)
        return ERR_TX_TX_ONGOING;
      /* in case of receiving, is it ongoing? */
      for(uint16_t i=0; i<4 && (mode == SX127X_LORA_RXCONT_MODE || mode == SX127X_LORA_RXSINGLE_MODE); i++){
        if(pGxModule->SPIreadRegister(SX127X_REG_MODEM_STAT) & 0x0B)
          return ERR_TX_RX_ONGOING;
        delay(1);
      }
      break;
  }
  /* CAD not required */
  if(_fskMode)
    return ERR_NONE;

	/*
	 * CAD
	 */

  switch (radioType){
    case RADIO_SX1262:
      sx1262_standby(); //set Module to Standby
      //set IRQ to RX-Done
      sx1262SetDioIrqParams(SX126X_IRQ_CAD_DETECTED | SX126X_IRQ_CAD_DONE, SX126X_IRQ_NONE);
      pGxModule->SPIwriteRegister(SX127X_REG_IRQ_FLAGS,SX127X_IRQ_CAD_DETECTED | SX127X_IRQ_CAD_DONE);	// clearing flags
      // clear IRQ
      sx1262ClearIrqStatus();
      // set RF switch (if present)
      pGxModule->setRfSwitchState(HIGH, LOW);
      SPIwriteCommand(SX126X_CMD_SET_CAD, &sx1262Cmd, 0, false); //set to CAD-Mode

      // wait for CAD completion
      for(uint16_t i = 0; i<10 && ((irq=sx1262ReadIrQ()) & SX126X_IRQ_CAD_DONE) == 0; i++)
        delay(1);
      if(irq & SX126X_IRQ_CAD_DETECTED)
      {
        // re-establish old mode
        if(chipMode == SX126X_STATUS_MODE_RX){
          sx1262SetCmdRx();
        }else if (chipMode == SX126X_STATUS_MODE_STDBY_XOSC){
          sx1262_standby(); //set Module to Standby
        }
          
        return ERR_TX_RX_ONGOING;
      }
      break;
    case RADIO_SX1276:
      sx1276setOpMode(SX1276_MODE_STANDBY);
      pGxModule->SPIwriteRegister(SX127X_REG_IRQ_FLAGS,SX127X_IRQ_CAD_DETECTED | SX127X_IRQ_CAD_DONE);	// clearing flags
      sx1276setOpMode(SX1276_MODE_CAD);

      // wait for CAD completion
      uint8_t iflags;
      for(uint16_t i = 0; i<10 && ((iflags=pGxModule->SPIreadRegister(SX127X_REG_IRQ_FLAGS)) & SX127X_IRQ_CAD_DONE) == 0; i++)
        delay(1);
      if(iflags & SX127X_IRQ_CAD_DETECTED)
      {
        // re-establish old mode
        if(mode == SX127X_LORA_RXCONT_MODE || mode == SX127X_LORA_RXSINGLE_MODE || mode == SX127X_LORA_SLEEP_MODE)
          sx1276setOpMode(mode);

        return ERR_TX_RX_ONGOING;
      }
      break;
  }
	return ERR_NONE;
}

void LoRaClass::configChannel (uint32_t frequency){
  // set frequency: FQ = (FRF * 32 Mhz) / (2 ^ 19)
  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

  pGxModule->SPIwriteRegister(SX127X_REG_FRF_MSB, frf >> 16);
  pGxModule->SPIwriteRegister(SX127X_REG_FRF_MID, frf >> 8);
  pGxModule->SPIwriteRegister(SX127X_REG_FRF_LSB, frf >> 0);

  // run one-time receiver chain calibration (in STANDBY mode!)
  if (!bCalibrated) {
    log_i("calibrate sx1276");
    bCalibrated = true;
    pGxModule->SPIwriteRegister(SX127X_REG_FSKImageCal, SX127X_RF_IMAGECAL_IMAGECAL_START);
    while ( pGxModule->SPIreadRegister(SX127X_REG_FSKImageCal) & SX127X_RF_IMAGECAL_IMAGECAL_RUNNING );
    log_i("calibration done");
  }
}

int16_t LoRaClass::transmit(uint8_t* data, size_t len){
  //log_i("transmit l=%d",len);
	/* channel accessible? */
  int state = sx_channel_free4tx();
	if(state != ERR_NONE)
    return state;

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
          return sx1262Transmit(tx_frame,FSK_PACKET_LENGTH*2);
        }else{
          return -1;
        }        
      }else{
        return sx1262Transmit(data,len);
      }    
    case RADIO_SX1276:
      if (_fskMode){
        if (len == 26){
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
          //uint32_t tSend = GxModule::micros() - start;
          //log_i("sending took %d",tSend);
          pGxModule->SPIwriteRegister(0x3E, 0b11111111); //REG_IRQ_FLAGS_1
          pGxModule->SPIwriteRegister(0x3F, 0b11111111); //REG_IRQ_FLAGS_2
          sx1276setOpMode(SX1276_MODE_STANDBY);//RegOpMode --> set Module to standby
          return ret;
        }else{
          return -1;
        }        
      }else{
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
      {
      // set RF switch (if present)
      pGxModule->setRfSwitchState(LOW, LOW);
      uint8_t sleepMode = SX126X_SLEEP_START_COLD | SX126X_SLEEP_RTC_OFF;
      SPIwriteCommand(SX126X_CMD_SET_SLEEP, &sleepMode, 1, false);      
      GxModule::delay(1);
      break;
      }
    case RADIO_SX1276:
      sx1276setOpMode(SX1276_MODE_SLEEP);//RegOpMode --> set Module to sleep
      break;
  }
}

