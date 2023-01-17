#include "GxModule.h"

GxModule::GxModule(uint8_t cs, uint8_t irq, uint8_t rst):
  _cs(cs),
  _irq(irq),
  _rst(rst),
  _rx(GXMODULE_NC),
  _tx(GXMODULE_NC),
  _spiSettings(SPISettings(2000000, MSBFIRST, SPI_MODE0))
{
  _spi = &SPI;
  _initInterface = true;
}  

GxModule::GxModule(uint8_t cs, uint8_t irq, uint8_t rst, uint8_t gpio):
  _cs(cs),
  _irq(irq),
  _rst(rst),
  _rx(gpio),
  _tx(GXMODULE_NC),
  _spiSettings(SPISettings(2000000, MSBFIRST, SPI_MODE0))
{
  _spi = &SPI;
  _initInterface = true;
}

GxModule::GxModule(uint8_t cs, uint8_t irq, uint8_t rst, SPIClass& spi, SPISettings spiSettings):
  _cs(cs),
  _irq(irq),
  _rst(rst),
  _rx(GXMODULE_NC),
  _tx(GXMODULE_NC),
  _spiSettings(spiSettings)
{
  _spi = &spi;
  _initInterface = false;
}

GxModule::GxModule(uint8_t cs, uint8_t irq, uint8_t rst, uint8_t gpio, SPIClass& spi, SPISettings spiSettings):
  _cs(cs),
  _irq(irq),
  _rst(rst),
  _rx(gpio),
  _tx(GXMODULE_NC),
  _spiSettings(spiSettings)
{
  _spi = &spi;
  _initInterface = false;
}

GxModule::GxModule(const GxModule& mod) {
  *this = mod;
}

GxModule& GxModule::operator=(const GxModule& mod) {
  this->SPIreadCommand = mod.SPIreadCommand;
  this->SPIwriteCommand = mod.SPIwriteCommand;
  this->_cs = mod.getCs();
  this->_irq = mod.getIrq();
  this->_rst = mod.getRst();
  this->_rx = mod.getRx();
  this->_tx = mod.getTx();
  this->_spiSettings = mod.getSpiSettings();
  this->_spi = mod.getSpi();

  return(*this);
}

void GxModule::init(uint8_t interface) {
  // select interface
  GxModule::pinMode(_cs, OUTPUT);
  GxModule::digitalWrite(_cs, HIGH);
  if(_initInterface) {
    _spi->begin();
  }
}

void GxModule::term(uint8_t interface) {
  // stop hardware interfaces (if they were initialized by the library)
  if(!_initInterface) {
    return;
  }

  if((interface == 0x00) && (_spi != nullptr)) {
    _spi->end();
  }

}

int16_t GxModule::SPIgetRegValue(uint8_t reg, uint8_t msb, uint8_t lsb) {
  if((msb > 7) || (lsb > 7) || (lsb > msb)) {
    return(ERR_INVALID_BIT_RANGE);
  }

  uint8_t rawValue = SPIreadRegister(reg);
  uint8_t maskedValue = rawValue & ((0b11111111 << lsb) & (0b11111111 >> (7 - msb)));
  return(maskedValue);
}

int16_t GxModule::SPIsetRegValue(uint8_t reg, uint8_t value, uint8_t msb, uint8_t lsb, uint8_t checkInterval) {
  if((msb > 7) || (lsb > 7) || (lsb > msb)) {
    return(ERR_INVALID_BIT_RANGE);
  }

  uint8_t currentValue = SPIreadRegister(reg);
  uint8_t mask = ~((0b11111111 << (msb + 1)) | (0b11111111 >> (8 - lsb)));
  uint8_t newValue = (currentValue & ~mask) | (value & mask);
  SPIwriteRegister(reg, newValue);

  // check register value each millisecond until check interval is reached
  // some registers need a bit of time to process the change (e.g. SX127X_REG_OP_MODE)
  uint32_t start = GxModule::micros();
  uint8_t readValue = 0;
  if (checkInterval > 0){
    while(GxModule::micros() - start < (checkInterval * 1000)) {
      readValue = SPIreadRegister(reg);
      if(readValue == newValue) {
        // check passed, we can stop the loop
        return(ERR_NONE);
      }
    }
  }else{
    return(ERR_NONE);
  }

  // check failed, print debug info
  GX_MODULE_DEBUG_PRINTLN();
  GX_MODULE_DEBUG_PRINT(F("address:\t0x"));
  GX_MODULE_DEBUG_PRINTLN(reg, HEX);
  GX_MODULE_DEBUG_PRINT(F("bits:\t\t"));
  GX_MODULE_DEBUG_PRINT(msb);
  GX_MODULE_DEBUG_PRINT(' ');
  GX_MODULE_DEBUG_PRINTLN(lsb);
  GX_MODULE_DEBUG_PRINT(F("value:\t\t0b"));
  GX_MODULE_DEBUG_PRINTLN(value, BIN);
  GX_MODULE_DEBUG_PRINT(F("current:\t0b"));
  GX_MODULE_DEBUG_PRINTLN(currentValue, BIN);
  GX_MODULE_DEBUG_PRINT(F("mask:\t\t0b"));
  GX_MODULE_DEBUG_PRINTLN(mask, BIN);
  GX_MODULE_DEBUG_PRINT(F("new:\t\t0b"));
  GX_MODULE_DEBUG_PRINTLN(newValue, BIN);
  GX_MODULE_DEBUG_PRINT(F("read:\t\t0b"));
  GX_MODULE_DEBUG_PRINTLN(readValue, BIN);
  GX_MODULE_DEBUG_PRINTLN();

  //log_e("spi-write failed %02X:%02X",reg,value);
  return(ERR_SPI_WRITE_FAILED);
}

void GxModule::SPIreadRegisterBurst(uint8_t reg, uint8_t numBytes, uint8_t* inBytes) {
  SPItransfer(SPIreadCommand, reg, NULL, inBytes, numBytes);
}

uint8_t GxModule::SPIreadRegister(uint8_t reg) {
  uint8_t resp = 0;
  SPItransfer(SPIreadCommand, reg, NULL, &resp, 1);
  return(resp);
}

void GxModule::SPIwriteRegisterBurst(uint8_t reg, uint8_t* data, uint8_t numBytes) {
  SPItransfer(SPIwriteCommand, reg, data, NULL, numBytes);
}

void GxModule::SPIwriteRegister(uint8_t reg, uint8_t data) {
  SPItransfer(SPIwriteCommand, reg, &data, NULL, 1);
}

void GxModule::SPItransfer(uint8_t cmd, uint8_t reg, uint8_t* dataOut, uint8_t* dataIn, uint8_t numBytes) {
  // start SPI transaction
  _spi->beginTransaction(_spiSettings);

  // pull CS low
  GxModule::digitalWrite(_cs, LOW);

  // send SPI register address with access command
  _spi->transfer(reg | cmd);
  #ifdef GX_MODULE_VERBOSE
    if(cmd == SPIwriteCommand) {
      GX_MODULE_VERBOSE_PRINT('W');
    } else if(cmd == SPIreadCommand) {
      GX_MODULE_VERBOSE_PRINT('R');
    }
    GX_MODULE_VERBOSE_PRINT('\t')
    GX_MODULE_VERBOSE_PRINT(reg, HEX);
    GX_MODULE_VERBOSE_PRINT('\t');
  #endif

  // send data or get response
  if(cmd == SPIwriteCommand) {
    if(dataOut != NULL) {
      for(size_t n = 0; n < numBytes; n++) {
        _spi->transfer(dataOut[n]);
        GX_MODULE_VERBOSE_PRINT(dataOut[n], HEX);
        GX_MODULE_VERBOSE_PRINT('\t');
      }
    }
  } else if (cmd == SPIreadCommand) {
    if(dataIn != NULL) {
      for(size_t n = 0; n < numBytes; n++) {
        dataIn[n] = _spi->transfer(0x00);
        GX_MODULE_VERBOSE_PRINT(dataIn[n], HEX);
        GX_MODULE_VERBOSE_PRINT('\t');
      }
    }
  }
  GX_MODULE_VERBOSE_PRINTLN();

  // release CS
  GxModule::digitalWrite(_cs, HIGH);

  // end SPI transaction
  _spi->endTransaction();
}

void GxModule::pinMode(uint8_t pin, uint8_t mode) {
  if(pin != GXMODULE_NC) {
    ::pinMode(pin, mode);
  }
}

void GxModule::digitalWrite(uint8_t pin, uint8_t value) {
  if(pin != GXMODULE_NC) {
    ::digitalWrite(pin, value);
  }
}

uint8_t GxModule::digitalRead(uint8_t pin) {
  if(pin != GXMODULE_NC) {
    return(::digitalRead(pin));
  }
  return(LOW);
}

void GxModule::attachInterrupt(uint8_t interruptNum, void (*userFunc)(void), uint8_t mode) {
  ::attachInterrupt(interruptNum, userFunc, mode);
}

void GxModule::detachInterrupt(uint8_t interruptNum) {
  ::detachInterrupt(interruptNum);
}

void GxModule::yield() {
  ::yield();
}

void GxModule::delay(uint32_t ms) {
  ::delay(ms);
}

void GxModule::delayMicroseconds(uint32_t us) {
  ::delayMicroseconds(us);
}

uint32_t GxModule::millis() {
  return(::millis());
}

uint32_t GxModule::micros() {
  return(::micros());
}

void GxModule::setRfSwitchPins(uint8_t rxEn, uint8_t txEn) {
  _useRfSwitch = true;
  _rxEn = rxEn;
  _txEn = txEn;
  GxModule::pinMode(rxEn, OUTPUT);
  GxModule::pinMode(txEn, OUTPUT);
}

void GxModule::setRfSwitchState(uint8_t rxPinState, uint8_t txPinState) {
  // check RF switch control is enabled
  if(!_useRfSwitch) {
    return;
  }

  // set pins
  GxModule::digitalWrite(_rxEn, rxPinState);
  GxModule::digitalWrite(_txEn, txPinState);
}
