/*!
 * @file FanetLora.cpp
 *
 *
 */

#include "FanetLora.h"

#define MAXPACKETSIZE 50

int PacketSize;
static uint8_t arRec[MAXPACKETSIZE];
static uint8_t u8ewMessage;
static int i16Packetsize;
//static uint8_t u8PacketOverflow;
//static uint8_t u8PacketError;



FanetLora::FanetLora(){
    initCount = 0;
}

String FanetLora::uint64ToString(uint64_t input) {
  String result = "";
  uint8_t base = 16;

  do {
    char c = input % base;
    input /= base;

    if (c < 10)
      c +='0';
    else
      c += 'A' - 10;
    result = c + result;
  } while (input);
  return result;
}

void FanetLora::checkMyDevId(){
  uint64_t chipmacid = ESP.getEfuseMac();
  //Serial.print("MAC:");Serial.println(uint64ToString(chipmacid)); // six octets
  myDevId[0] = ManuId;//Manufacturer GetroniX
  myDevId[1] = uint8_t(chipmacid >> 40);
  myDevId[2] = uint8_t(chipmacid >> 32); //last 2 Bytes of MAC
  _myData.devId = ((uint32_t)myDevId[0] << 16) | ((uint32_t)myDevId[1] << 8) | (uint32_t)myDevId[2];
  //_myData.DevId = getHexFromByte(myDevId[0],true) + getHexFromByte(myDevId[2],true) + getHexFromByte(myDevId[1],true);
  //_myData.DevId = String(uint8_t(chipmacid >> 24),HEX) + String(uint8_t(chipmacid >> 32),HEX) + String(uint8_t(chipmacid >> 40),HEX);
  //Serial.print("MAC:");Serial.println(((uint32_t)chipmacid & 0xFFFFFF), HEX);
  //Serial.print("MAC:");Serial.println(((uint32_t)(chipmacid >> 32) & 0xFFFFFF), HEX);
}

void FanetLora::end(void){
  LoRa.end();
}

String FanetLora::getMyDevId(void){
    return getDevId(_myData.devId);
}

void FanetLora::onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return
  log_v("onReceive");
  int i = 0;
  i16Packetsize = 0;
  while (LoRa.available()) {
      uint8_t rec = (uint8_t)LoRa.read();
      if (i < (MAXPACKETSIZE-1)){
          arRec[i] = rec;
          i++;
      }      
  }
  i16Packetsize = i;
}


bool FanetLora::begin(int8_t sck, int8_t miso, int8_t mosi, int8_t ss,int reset, int dio0,long frequency){
  checkMyDevId(); //get Dev-Id out of MAC
  log_i("MyDevId:%s",getMyDevId());
  //SPI LoRa pins
  SPI.begin(sck, miso, mosi, ss);
  //setup LoRa transceiver module
  LoRa.setPins(ss, reset, dio0);
  log_i("Start Lora Frequency=%d",frequency);
  while (!LoRa.begin(frequency) && counter < 10) {
    log_i(".");
    counter++;
    delay(500);
  }
  if (counter == 10) {
    log_e("Starting LoRa failed!"); 
  }
  LoRa.setSignalBandwidth(250E3); //set Bandwidth to 250kHz
  LoRa.setSpreadingFactor(7); //set spreading-factor to 7
  LoRa.setCodingRate4(8);
  LoRa.setSyncWord(0xF1);
  LoRa.enableCrc();
  //LoRa.setTxPower(10); //10dbm + 4dbm antenna --> max 14dbm
  LoRa.setTxPower(20); //full Power
  log_i("LoRa Initialization OK!");



  initCount = 0;
  _PilotName = "";  
  _myData.aircraftType = eFanetAircraftType::PARA_GLIDER; //default Paraglider
  newMsg = false;
  u8ewMessage = 0; //now we have new processed the msg
  // put the radio into receive mode
  //LoRa.onReceive(onReceive);
  //LoRa.receive();
  for (int i = 0; i < MAXNEIGHBOURS; i++){ //clear neighbours list
    neighbours[i].devId = 0; 
  }

  return true;
}

String FanetLora::getNeighbourName(uint32_t devId){
  for (int i = 0; i < MAXNEIGHBOURS; i++){
    if (neighbours[i].devId == devId){
      return neighbours[i].name; //found entry
    }
  }
  return "";
}

int16_t FanetLora::getneighbourIndex(uint32_t devId){
  int16_t iRet = -1;
  for (int i = 0; i < MAXNEIGHBOURS; i++){
    if (neighbours[i].devId == devId){
      return i; //found entry
    }
    if ((neighbours[i].devId == 0) && (iRet < 0)){
      iRet = i; //found empty one
    }
  }
  return iRet;
}

void FanetLora::insertNameToNeighbour(uint32_t devId, String name){
  int16_t index = getneighbourIndex(devId);
  //log_i("devId=%i",devId);
  //log_i("name=%s",name.c_str());
  //log_i("index=%i",index);
  if (index < 0) return;
  neighbours[index].devId = devId;
  neighbours[index].tLastMsg = millis();
  neighbours[index].name = name;
}

void FanetLora::insertDataToNeighbour(uint32_t devId, trackingData *Data){
  int16_t index = getneighbourIndex(devId);
  //log_i("devId=%i",devId);
  //log_i("index=%i",index);
  if (index < 0) return;
  neighbours[index].devId = devId;
  neighbours[index].tLastMsg = millis();
  neighbours[index].aircraftType = Data->aircraftType;
  neighbours[index].lat = Data->lat;
  neighbours[index].lon = Data->lon;
  neighbours[index].altitude = Data->altitude;
  neighbours[index].speed = Data->speed;
  neighbours[index].climb = Data->climb;
  neighbours[index].heading = Data->heading;
  neighbours[index].rssi = Data->rssi;
}

void FanetLora::clearNeighbours(uint32_t tAct){
  static uint32_t tCheck = millis();
  if ((tAct - tCheck) >= 5000){ //check only every 5 seconds
    tCheck = tAct;
    for (int i = 0; i < MAXNEIGHBOURS; i++){
      if (neighbours[i].devId){
        if ((tCheck - neighbours[i].tLastMsg) >= NEIGHBOURSLIFETIME){ //if we get no msg in 4min --> del neighbour
          //log_i("clear slot %i devId %s",i,getDevId(neighbours[i].devId).c_str());
          neighbours[i].devId = 0; //clear slot
        }
      }
    }
  }
}

void FanetLora::setPilotname(String name){
    _PilotName = name;
}

void FanetLora::setAircraftType(eFanetAircraftType type){
    _myData.aircraftType = type;
}

uint8_t FanetLora::getNeighboursCount(void){
  uint8_t countRet = 0;
  for (int i = 0; i < MAXNEIGHBOURS; i++){
    if (neighbours[i].devId){
      countRet++;
    }
  }
  return countRet;
}

String FanetLora::CreateFNFMSG(char *recBuff,uint8_t size){
  String msg = ""; 
  uint32_t devId =  ((uint32_t)recBuff[1] << 16) | ((uint32_t)recBuff[2] << 8) | (uint32_t)recBuff[3];
  fanet_header_t *tHeader = (fanet_header_t *)&recBuff[0];
  if ((myDevId[0] == recBuff[1]) && (myDevId[1] == recBuff[2]) && (myDevId[2] == recBuff[3])){
      //this is me --> abort
      log_v("source is my id --> abort");
      return "";
  }
  uint8_t offset = 4;
  if (tHeader->ext_header){
      offset ++;
      if (tHeader->unicast) offset += 3; //if unicast add 3 Byte für dest-addr
      if (tHeader->signature) offset += 4; //if signature add 4 Byte for signature
  } 
  uint8_t msgLen = (size - offset);
  msg = "#FNF " + getHexFromByte(recBuff[1]) + "," + getHexFromWord(tHeader->address) + ",";
  if ((tHeader->ext_header) && (tHeader->unicast)){
      msg += "0,";
  }else{
      msg += "1,";
  }
  msg += "0,"; //signature always 0
  msg += getHexFromByte(tHeader->type) + ",";
  msg += getHexFromByte(msgLen) + ",";
  String payload = ""; 
  String msg2 = ""; 
  for (int i = offset;i < size;i++){
      payload += getHexFromByte(recBuff[i],true);
      msg2 += recBuff[i];
  }
  msg += payload;
  if (tHeader->type == 1){
      //actTrackingData.DevId = getHexFromByte(recBuff[1],true) + getHexFromWord(tHeader->address,true);
      actTrackingData.devId = ((uint32_t)recBuff[1] << 16) | ((uint32_t)recBuff[2] << 8) | (uint32_t)recBuff[3];
      actTrackingData.rssi = actrssi;
      getTrackingInfo(payload,msgLen);
      insertDataToNeighbour(devId,&actTrackingData);
  }else if (tHeader->type == 2){
      insertNameToNeighbour(devId,msg2);
  }
  actMsg = msg;
  newMsg = true;
  rxCount++;
  return msg;

}

String FanetLora::getactMsg(){
    return actMsg;
}

bool FanetLora::isNewMsg(){
    bool ret = newMsg;
    newMsg = false;
    return ret;
}

void FanetLora::getLoraMsg(void){
  CreateFNFMSG((char *)&arRec[0],(uint8_t)i16Packetsize);
  u8ewMessage = 0; //now we have new processed the msg
}

void FanetLora::run(void){    
  onReceive(LoRa.parsePacket());
  if (i16Packetsize > 0){
      actrssi = LoRa.packetRssi();
      getLoraMsg();
      i16Packetsize = 0;
  }
  clearNeighbours(millis());
}

void FanetLora::sendPilotName(void){
    if (_PilotName.length() > 0){
        log_v("sending fanet-name:%s",_PilotName);
        LoRa.beginPacket();
        LoRa.write(0x42);
        LoRa.write(myDevId[0]);
        LoRa.write(myDevId[1]);
        LoRa.write(myDevId[2]);
        LoRa.print(_PilotName);
        LoRa.endPacket();   
        txCount++;
    }
}

void FanetLora::sendPilotName(uint32_t tAct){
    static uint32_t tPilotName = millis();
    if ((tAct - tPilotName) >= 24000){
        tPilotName = tAct;
        sendPilotName();
    }
}

String FanetLora::getAircraftType(eFanetAircraftType type){
  if (type == eFanetAircraftType::PARA_GLIDER){
    return "PARA_GLIDER";
  }else if (type == eFanetAircraftType::HANG_GLIDER){
    return "HANG_GLIDER";
  }else if (type == eFanetAircraftType::BALLOON){
    return "BALLOON";
  }else if (type == eFanetAircraftType::GLIDER){
    return "GLIDER";
  }else if (type == eFanetAircraftType::POWERED_AIRCRAFT){
    return "POWERED_AIRCRAFT";
  }else if (type == eFanetAircraftType::HELICOPTER_ROTORCRAFT){
    return "HELICOPTER_ROTORCRAFT";
  }else if (type == eFanetAircraftType::UAV){
    return "UAV";
  }
  return "UNKNOWN";

}

String FanetLora::getHexFromWord(uint16_t val,bool leadingZero){
    char myHexString[5];
    if (leadingZero){
        sprintf(myHexString,"%04X",val);
    }else{
        sprintf(myHexString,"%X",val);
    }
    //myHexString[0] = (val >> 4) + 0x30;
    //if (myHexString[0] > 0x39) myHexString[0] +=7;


    //myHexString[1] = (val & 0x0f) + 0x30;
    //if (myHexString[1] > 0x39) myHexString[1] +=7;   
    //myHexString[2] = 0;
    return String(myHexString);

}

String FanetLora::getHexFromByte(uint8_t val,bool leadingZero){
    char myHexString[3];
    if (leadingZero){
        sprintf(myHexString,"%02X",val);
    }else{
        sprintf(myHexString,"%X",val);
    }
    

    //myHexString[0] = (val >> 4) + 0x30;
    //if (myHexString[0] > 0x39) myHexString[0] +=7;


    //myHexString[1] = (val & 0x0f) + 0x30;
    //if (myHexString[1] > 0x39) myHexString[1] +=7;   
    //myHexString[2] = 0;
    return String(myHexString);; 
}

int FanetLora::getByteFromHex(char in[]){
  int tens;
  int digits;
   
  if (!isxdigit(in[0]) || !isxdigit(in[1]))   // Valid hex digit character?
    return -1;

  in[0] = toupper(in[0]);   // Use upper case
  in[1] = toupper(in[1]);
 
  tens = in[0] >= 'A' ? (in[0] - 'A' + 10) : in[0] - '0';
  digits = in[1] >= 'A' ? (in[1] - 'A' + 10) : in[1] - '0';
  return tens * 16 + digits;
}

bool FanetLora::getTrackingData(trackingData *tData){
    bool bRet = newData;
    *tData = actTrackingData; //copy tracking-data
    newData = false; //clear new Data flag
    return bRet;
}

void FanetLora::getTrackingInfo(String line,uint16_t length){
    char arPayload[23];
    
    line.toCharArray(arPayload,sizeof(arPayload));

    // integer values /
    int32_t lati = getByteFromHex(&arPayload[4])<<16 | getByteFromHex(&arPayload[2])<<8 | getByteFromHex(&arPayload[0]);
    if(lati & 0x00800000)
      lati |= 0xFF000000;
    int32_t loni = getByteFromHex(&arPayload[10])<<16 | getByteFromHex(&arPayload[8])<<8 | getByteFromHex(&arPayload[6]);
    if(loni & 0x00800000)
      loni |= 0xFF000000;
    actTrackingData.lat = (float)lati / 93206.0f;
    actTrackingData.lon = (float)loni / 46603.0f;
    //Serial.print("FANETlat=");Serial.println(actTrackingData.lat);
    //Serial.print("FANETlon=");Serial.println(actTrackingData.lon);

    uint16_t Type = (uint16_t(getByteFromHex(&arPayload[14])) << 8) + uint16_t(getByteFromHex(&arPayload[12]));
    uint16_t altitude = Type & 0x7FF;
    if  (Type & 0x0800){
        altitude *= 4;
    } 
    actTrackingData.altitude = altitude;
    actTrackingData.aircraftType = (eFanetAircraftType)((Type >> 12) & 0x07);

    uint16_t speed = uint16_t(getByteFromHex(&arPayload[16]));
    if (speed & 0x80){
        speed = (speed & 0x007F) * 5;
    }
    actTrackingData.speed = float(speed) * 0.5;

    int8_t climb = int8_t(getByteFromHex(&arPayload[18]));
    int8_t climb2 = (climb & 0x7F) | (climb&(1<<6))<<1; //create 2-complement
    if (climb & 0x80){
        actTrackingData.climb = float(climb2) * 5.0 / 10.0;
    }else{
        actTrackingData.climb = float(climb2) / 10.0;
    }

    actTrackingData.heading = float(getByteFromHex(&arPayload[20])) * 360 / 255;
    if (actTrackingData.heading  < 0) actTrackingData.heading += 360.0;
    newData = true;
}
void FanetLora::coord2payload_absolut(float lat, float lon, uint8_t *buf)
{
	if(buf == NULL)
		return;

	int32_t lat_i = roundf(lat * 93206.0f);
	int32_t lon_i = roundf(lon * 46603.0f);

	buf[0] = ((uint8_t*)&lat_i)[0];
	buf[1] = ((uint8_t*)&lat_i)[1];
	buf[2] = ((uint8_t*)&lat_i)[2];

	buf[3] = ((uint8_t*)&lon_i)[0];
	buf[4] = ((uint8_t*)&lon_i)[1];
	buf[5] = ((uint8_t*)&lon_i)[2];
}

void FanetLora::writeMsgType2(String name){
    LoRa.beginPacket();
    LoRa.write(0x42);
    LoRa.write(myDevId[0]);
    LoRa.write(myDevId[1]);
    LoRa.write(myDevId[2]);
    LoRa.print(name);
    LoRa.endPacket(); 
    txCount++;  
}


void FanetLora::writeMsgType3(String msg){
    LoRa.beginPacket();
    LoRa.write(0x43);
    LoRa.write(myDevId[0]);
    LoRa.write(myDevId[1]);
    LoRa.write(myDevId[2]);
    LoRa.write(0); //extended header
    LoRa.print(msg);
    LoRa.endPacket(); 
    txCount++;
}


eFanetAircraftType FanetLora::getAircraftType(void){
    return _myData.aircraftType;
}

void FanetLora::writeMsgType4(weatherData *wData){
    uint8_t sendBuffer[30];
    uint8_t sendindex = 0;
    fanet_packet_t4 *pkt = (fanet_packet_t4 *)&sendBuffer[0];
    pkt->ext_header     = 0;
    pkt->forward        = 1;
    pkt->type           = 4;  /* weather-data  */
    pkt->vendor         = myDevId[0];
    pkt->address        = ((uint16_t)myDevId[2] << 8) + (uint16_t)myDevId[1];
    
    coord2payload_absolut(wData->lat,wData->lon, ((uint8_t *) pkt) + FANET_HEADER_SIZE + 1);
    pkt->bExt_header2 = false;
    pkt->bStateOfCharge = true;
    pkt->bRemoteConfig = false;
    pkt->bBaro = true;
    pkt->bHumidity = true;
    pkt->bWind = true;
    pkt->bTemp = true;
    pkt->bInternetGateway = false;

    int iTemp = (int)(round(wData->temp * 2)); //Temperature (+1byte in 0.5 degree, 2-Complement)
    pkt->temp = iTemp & 0xFF;
    pkt->heading = uint8_t(round(wData->wHeading * 256.0 / 360.0)); //Wind (+3byte: 1byte Heading in 360/256 degree, 1byte speed and 1byte gusts in 0.2km/h (each: bit 7 scale 5x or 1x, bit 0-6))

    int speed = (int)roundf(wData->wSpeed * 5.0f);
    if(speed > 127) {
        pkt->speed_scale  = 1;
        pkt->speed        = (speed / 5);
    } else {
        pkt->speed_scale  = 0;
        pkt->speed        = speed & 0x7F;
    }
    speed = (int)roundf(wData->wGust * 5.0f);
    if(speed > 127) {
        pkt->gust_scale  = 1;
        pkt->gust        = (speed / 5);
    } else {
        pkt->gust_scale  = 0;
        pkt->gust        = speed & 0x7F;
    }

    pkt->humidity = uint8_t(round(wData->Humidity * 10 / 4)); //Humidity (+1byte: in 0.4% (%rh*10/4))

    pkt->baro = int16_t(round((wData->Baro - 430.0) * 10));  //Barometric pressure normailized (+2byte: in 10Pa, offset by 430hPa, unsigned little endian (hPa-430)*10)

    pkt->charge = constrain(roundf(float(wData->Charge) / 100.0 * 15.0),0,15); //State of Charge  (+1byte lower 4 bits: 0x00 = 0%, 0x01 = 6.666%, .. 0x0F = 100%)

    sendindex = 19;
    LoRa.beginPacket();
    for (int i = 0;i < sendindex;i++){
        LoRa.write(sendBuffer[i]);
    }
    LoRa.endPacket();  
    txCount++; 
}

void FanetLora::writeTrackingData2FANET(trackingData *tData){
    uint8_t sendBuffer[20];
    uint8_t sendindex = 0;
    fanet_packet_t *pkt = (fanet_packet_t *)&sendBuffer[0];
    pkt->ext_header     = 0;
    pkt->forward        = 1;
    pkt->type           = 1;  /* Tracking  */
    pkt->vendor         = myDevId[0];
    pkt->address        = ((uint16_t)myDevId[2] << 8) + (uint16_t)myDevId[1];
    
    coord2payload_absolut(tData->lat,tData->lon, ((uint8_t *) pkt) + FANET_HEADER_SIZE);
    pkt->track_online = 1;
    //pkt->aircraft_type  = uint16_t(_myData.aircraftType);
    pkt->aircraft_type  = uint16_t(tData->aircraftType);
    int altitude        = constrain(tData->altitude, 0, 8190);
    pkt->altitude_scale = altitude > 2047 ? (altitude = (altitude + 2) / 4, 1) : 0;
    pkt->altitude_msb   = (altitude & 0x700) >> 8;
    pkt->altitude_lsb   = (altitude & 0x0FF);

    int speed2          = constrain((int)roundf(tData->speed * 2.0f), 0, 635);
    if(speed2 > 127) {
        pkt->speed_scale  = 1;
        pkt->speed        = ((speed2 + 2) / 5);
    } else {
        pkt->speed_scale  = 0;
        pkt->speed        = speed2 & 0x7F;
    }

    int climb10         = constrain((int)roundf(tData->climb * 10.0f), -315, 315);
    if(abs(climb10) > 63) {
        pkt->climb_scale  = 1;
        pkt->climb        = ((climb10 + (climb10 >= 0 ? 2 : -2)) / 5);
    } else {
        pkt->climb_scale  = 0;
        pkt->climb        = climb10 & 0x7F;
    }

    pkt->heading        = constrain((int)roundf(tData->heading * 256.0f)/360.0f, 0, 255);
    pkt->turn_rate = 0;
    pkt->turn_scale = 0;
    /*
    sendindex++;
    for (int i = 0;i < 3; i++){
        sendBuffer[sendindex] = myDevId[i];
        sendindex++;
    }
    //coord2payload_absolut(tData->lat,tData->lon,&sendBuffer[sendindex]);
    */
    _myData.altitude = tData->altitude;
    _myData.climb = tData->climb;
    _myData.heading = tData->heading;
    _myData.lat = tData->lat;
    _myData.lon = tData->lon;
    _myData.speed = tData->speed;

    sendindex = 15;
    LoRa.beginPacket();
    for (int i = 0;i < sendindex;i++){
        LoRa.write(sendBuffer[i]);
    }
    LoRa.endPacket();  
    txCount++; 
}

bool FanetLora::getMyTrackingData(trackingData *tData){
    *tData = _myData;
    return true;
}

void FanetLora::printAircraftType(eFanetAircraftType type){
  Serial.print("Aircraft-type=");
  if (type == eFanetAircraftType::PARA_GLIDER){
    Serial.println("PARA_GLIDER");
  }else if (type == eFanetAircraftType::HANG_GLIDER){
    Serial.println("HANG_GLIDER");
  }else if (type == eFanetAircraftType::BALLOON){
    Serial.println("BALLOON");
  }else if (type == eFanetAircraftType::GLIDER){
    Serial.println("GLIDER");
  }else if (type == eFanetAircraftType::POWERED_AIRCRAFT){
    Serial.println("POWERED_AIRCRAFT");
  }else if (type == eFanetAircraftType::HELICOPTER_ROTORCRAFT){
    Serial.println("HELICOPTER_ROTORCRAFT");
  }else if (type == eFanetAircraftType::UAV){
    Serial.println("UAV");
  }else{
      Serial.println("UNKNOWN");
  }
}

String FanetLora::getDevId(uint32_t devId){
  return getHexFromByte((devId >> 16) & 0xFF,true) + getHexFromByte((devId) & 0xFF,true) + getHexFromByte((devId >> 8) & 0xFF,true);
}

void FanetLora::printFanetData(trackingData tData){
    Serial.print("id=");
    Serial.println(getDevId(tData.devId));
    printAircraftType(tData.aircraftType);
    Serial.print("lat=");
    Serial.println(tData.lat,5);
    Serial.print("lon=");
    Serial.println(tData.lon,5);
    Serial.print("alt=");
    Serial.println(tData.altitude);
    Serial.print("heading=");
    Serial.println(tData.heading);
    Serial.print("speed=");
    Serial.println(tData.speed,2);
    Serial.print("climb=");
    Serial.println(tData.climb,2);

}




/*
void FanetLora::coord2payload_absolut(float lat, float lon, uint8_t *buf)
{
	if(buf == NULL)
		return;

	int32_t lat_i = roundf(lat * 93206.0f);
	int32_t lon_i = roundf(lon * 46603.0f);

	buf[0] = ((uint8_t*)&lat_i)[0];
	buf[1] = ((uint8_t*)&lat_i)[1];
	buf[2] = ((uint8_t*)&lat_i)[2];

	buf[3] = ((uint8_t*)&lon_i)[0];
	buf[4] = ((uint8_t*)&lon_i)[1];
	buf[5] = ((uint8_t*)&lon_i)[2];
}

void FanetLora::encodeTrackingData(trackingData tData,uint8_t *buf){
    //first 6 Bytes are the position
    coord2payload_absolut(tData.lat,tData.lon,buf);
    uint16_t type = 0x8000; //always online tracking on
    type += ((uint16_t)tData.aircraftType) << 12;
    if (tData.altitude > 0x7FF){
        type += 0x800; //altidude * 4
        type += (tData.altitude / 4);
    }else{
        type += tData.altitude;
    }  
    buf[6] = uint8_t(type);
    buf[7] = uint8_t(type >> 8);    
    //speed
    if (tData.speed > 0x7F){
        buf[8] = 0x80 + (uint8_t)((uint16_t)(round(tData.speed)) / 5);
    }else{
        buf[8] = (uint8_t)(round(tData.speed));
    }
    //climb
    buf[9] = 0;
    //heading
     buf[10] = (uint8_t)(round(tData.heading * 255 / 360));
    
}


*/