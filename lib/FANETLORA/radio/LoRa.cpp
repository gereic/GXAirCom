// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "LoRa.h"
#include "manchester.h"


// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// disable interrupt when it's not needed
volatile bool enableInterrupt = true;

void setFlag(void)
{
    // check if the interrupt is enabled
    if (!enableInterrupt) {
        return;
    }

    // we got a packet, set the flag
    receivedFlag = true;
}

bool LoRaClass::isRxMessage(){
  if (receivedFlag){
    rxCount++;
    //log_i("new message arrived %d",rxCount);
    enableInterrupt = false;
    receivedFlag = false;
    return true;
  }
  return false;

}


LoRaClass::LoRaClass(){
  pSx1276Radio = NULL;
  pSx1262Radio = NULL;
}

//LoRaClass::LoRaClass(SPIClass *_spi,uint8_t cs, uint8_t irq, uint8_t rst, uint8_t gpio){
void LoRaClass::setPins(SPIClass *_spi,uint8_t cs, uint8_t irq, uint8_t rst, uint8_t gpio){
  log_i("cs=%d,irq=%d,rst=%d,gpio=%d",cs, irq, rst, gpio);
  //SPISettings _spiSettings(8E6, MSBFIRST, SPI_MODE0);
  SPISettings _spiSettings = SPISettings(8000000, MSBFIRST, SPI_MODE0);
  pModule = new Module(cs, irq, rst, gpio,*_spi,_spiSettings);
  pSx1276Radio = NULL;
  pSx1262Radio = NULL;
}
int16_t LoRaClass::begin(float freq, float bw, uint8_t sf, uint8_t cr, uint8_t syncWord, int8_t power,uint8_t radioChip){
	receivedFlag = false;
	//enableInterrupt = false;
  radioType = radioChip;
  int state = 0;
  _freq = freq;
  _bw = bw;
  _sf = sf;
  _cr = cr;
  _syncWord = syncWord;
  _power = uint8_t(power);
  _fskMode = false;
  //delay(1500); //wait until Hardware is stable
  //check which radio
  switch (radioType){
    case RADIO_SX1262:
      pSx1262Radio = new SX1262(pModule);
      state = pSx1262Radio->begin(_freq,_bw,_sf,_cr,_syncWord,_power,_preambleLength);
      if (state == ERR_NONE) {
        log_i("SX1262 Radio found !");
        pSx1262Radio->setDio1Action(setFlag);	
        radioType = RADIO_SX1262;
        return state;
      } else {
          log_i("failed, code %d",state);
          return state;
      }
      break;
    case RADIO_SX1276:
      pSx1276Radio = new SX1276(pModule);
      log_i("freq=%f,bw=%f,sf=%d,cr=%d",freq,bw,sf,cr);
      state = pSx1276Radio->begin(freq,bw,sf,cr,syncWord,power,_preambleLength);
      if (state == ERR_NONE) {
        log_i("SX1276 Radio found !");
        pSx1276Radio->setDio0Action(setFlag);	
        radioType = RADIO_SX1276;
        return state;
      } else {
          log_i("failed, code %d",state);
          return state;
      }
      break;
  }
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
      ret = pSx1276Radio->readData(data,len);

      if (_fskMode){
        invertba(data,len); //invert complete Frame
      }
      return ret;
  }
}

float LoRaClass::getRSSI(){
  switch (radioType){
    case RADIO_SX1262:
      return pSx1262Radio->getRSSI();
      break;
    case RADIO_SX1276:
      return pSx1276Radio->getRSSI();

  }
}

float LoRaClass::getSNR(){
  switch (radioType){
    case RADIO_SX1262:
      return pSx1262Radio->getSNR();
    case RADIO_SX1276:
      return pSx1276Radio->getSNR();
  }
}

int16_t LoRaClass::switchFSK(){
  int16_t ret = 0;
  uint8_t syncWord[] = {0x99, 0xA5, 0xA9, 0x55, 0x66, 0x65, 0x96};	
  switch (radioType){
    case RADIO_SX1262:
      ret = pSx1262Radio->switchFSK(100.0,50.0,156.2,8,0.0,false);
      //log_i("switchFSK %d",ret);
      ret = pSx1262Radio->setFrequency(868.199950,false); //don't calibrate Frequency
      //log_i("setFrequency %d",ret);
      ret = pSx1262Radio->setSyncWord(syncWord,sizeof(syncWord));
      //log_i("setSyncWord %d",ret);
      //preamble-size = 1Byte --> 8Bits
      //no CRC --> 0x01
      //payload-size = 26 * 2 --> Manchester encoding
      ret = pSx1262Radio->setPacketParaFSK(1,SX126X_GFSK_CRC_OFF,sizeof(syncWord),0x00,0x00,0x00,26*2,SX126X_GFSK_PREAMBLE_DETECT_8);
      //log_i("setPacketParaFSK %d",ret);

      break;
    case RADIO_SX1276:
      ret = pSx1276Radio->setActiveModem(SX127X_FSK_OOK);
      //log_i("setActiveModem %d",ret);
      ret = pSx1276Radio->setOOK(false);
      //log_i("setOOK %d",ret);
      ret = pSx1276Radio->setBitRate(100.0);
      //log_i("setBitRate %d",ret);
      ret = pSx1276Radio->setFrequencyDeviation(50.0);
      //log_i("setBandwidth %d",ret);
      ret = pSx1276Radio->setRxBandwidth(125.0);
      //log_i("setRxBandwidth %d",ret);
      ret = pSx1276Radio->setSyncWord(syncWord,sizeof(syncWord));
      //log_i("setSyncWord %d",ret);
      ret = pSx1276Radio->setOutputPower(_power);
      //log_i("setOutputPower %d",ret);
      ret = pSx1276Radio->fixedPacketLengthMode(FSK_PACKET_LENGTH);
      //log_i("fixedPacketLengthMode %d",ret);
      ret = pSx1276Radio->setCRC(false);
      //log_i("setCRC %d",ret);
      ret = pSx1276Radio->setRSSIConfig(2);
      //log_i("setRSSIConfig %d",ret);
      ret = pSx1276Radio->setEncoding(RADIOLIB_ENCODING_MANCHESTER);      
      //log_i("setWhitening %d",ret);
      ret = pSx1276Radio->setPreambleLength(2);
      //log_i("setPreambleLength %d",ret);
      break;
  }
  enableInterrupt = false;
  receivedFlag = false;
  _fskMode = true;
  return ret;
}
int16_t LoRaClass::switchLORA(){
  int16_t ret;
  switch (radioType){
    case RADIO_SX1262:
      ret = pSx1262Radio->switchLoRa(_bw,_sf,_cr,_syncWord,8,0.0,false);
      //log_i("switchLoRa %d",ret);
      ret = pSx1262Radio->setFrequency(_freq,false); //don't calibrate Frequency
      //log_i("setFrequency %d",ret);
      break;
    case RADIO_SX1276:
      //ret = pSx1276Radio->startReceive();
      ret = pSx1276Radio->standby();
      //log_i("standby %d",ret);
      ret = pSx1276Radio->setActiveModem(SX127X_LORA);
      //log_i("setActiveModem %d",ret);
      ret = pSx1276Radio->setSyncWord(_syncWord);
      //log_i("setSyncWord %d",ret);
      ret = pSx1276Radio->setPreambleLength(7);
      //log_i("setPreambleLength %d",ret);
      ret = pSx1276Radio->setFrequency(_freq);
      //log_i("setFrequency %d",ret);
      ret = pSx1276Radio->setBandwidth(_bw);
      //log_i("setBandwidth %d",ret);
      ret = pSx1276Radio->setSpreadingFactor(_sf);
      //log_i("setSpreadingFactor %d",ret);
      ret = pSx1276Radio->setCodingRate(_cr);
      //log_i("setCodingRate %d",ret);
      ret = pSx1276Radio->setOutputPower(_power);
      //log_i("setOutputPower %d",ret);
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
  int16_t iRet = 0;
  switch (radioType){
    case RADIO_SX1262:
      iRet = pSx1262Radio->startReceive();
      break;
    case RADIO_SX1276:
      iRet = pSx1276Radio->startReceive();
      //delay(10);
      //iRet = pSx1276Radio->startReceive();
      break;
  }
  //log_i("startReceive %d,ret=%d",radioType,iRet);  
  enableInterrupt = true;
  return iRet;
}

size_t LoRaClass::getPacketLength(){
  size_t tRet = 0;
  //log_i("getPacketLength %d",radioType);
  switch (radioType){
    case RADIO_SX1262:
      tRet = pSx1262Radio->getPacketLength();
      if (_fskMode){
        tRet /= 2; //cause of manchester-decoding we get always double of the Length
      }      
      break;
    case RADIO_SX1276:
      tRet = pSx1276Radio->getPacketLength();
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
        iRet = pSx1276Radio->setCodingRate(cr);
        break;
    }
  }
  return iRet;
}

int16_t LoRaClass::transmit(uint8_t* data, size_t len){
  if (!_fskMode){
    /* update air time */
    sx_airtime += expectedAirTime_ms(len);
  } 	
  switch (radioType){
    case RADIO_SX1262:
      if (_fskMode){
        if (len == 26){
          uint8_t tx_frame[len*2];
          for (int i = 0;i < len * 2; i++){            
            tx_frame[i] = ManchesterEncode[(data[i>>1] >> 4) & 0x0F];
            tx_frame[i+1] = ManchesterEncode[(data[i>>1]) & 0x0F];
            i++;
          }
          return pSx1262Radio->transmit(tx_frame,len*2);
        }else{
          return ERR_UNKNOWN;
        }        
      }else{
        return pSx1262Radio->transmit(data,len);
      }      
    case RADIO_SX1276:
      if (_fskMode){
        invertba(data,len);        
      //}else{
      //  return 0; // GE only for testing FSK_MODE !!
      } 
      return pSx1276Radio->transmit(data,len);
  }
}

void LoRaClass::invertba(byte* ba, int len)
{
  for (int i =0;i<len;i++)
    ba[i] =~ba[i];
} 

