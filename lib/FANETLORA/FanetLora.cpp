/*!
 * @file FanetLora.cpp
 *
 *
 */

#include "FanetLora.h"

int PacketSize;
static uint8_t arRec[50];
static uint8_t u8ewMessage;
static int i16Packetsize;
static uint8_t u8PacketOverflow;
static uint8_t u8PacketError;


FanetLora::FanetLora(){
    initCount = 0;
}

/*
void FanetLora::setNMEAOUT(NmeaOut *_pNmeaOut){
    pNmeaOut = _pNmeaOut;
}
*/

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
  _myData.DevId = String(myDevId[0],HEX) + String(myDevId[2],HEX) + String(myDevId[1],HEX);
  //_myData.DevId = String(uint8_t(chipmacid >> 24),HEX) + String(uint8_t(chipmacid >> 32),HEX) + String(uint8_t(chipmacid >> 40),HEX);
  //Serial.print("MAC:");Serial.println(((uint32_t)chipmacid & 0xFFFFFF), HEX);
  //Serial.print("MAC:");Serial.println(((uint32_t)(chipmacid >> 32) & 0xFFFFFF), HEX);
}

String FanetLora::getMyDevId(void){
    return _myData.DevId;
}

void FanetLora::onReceive(int packetSize) {
  PacketSize = packetSize;
  //uint8_t arRec[50];
  // received a packet
  if (packetSize > 50){
    u8PacketOverflow ++;
    //Serial.print("error packetSize to big ");
    //Serial.print(packetSize);
    //Serial.println();
    for (int i = 0; i < packetSize; i++) {
      LoRa.read();
    }
    return;
  }
  if (u8ewMessage){
    u8PacketError++;
    //Serial.print("error buffer not empty");
    //Serial.println();
    for (int i = 0; i < packetSize; i++) {
      LoRa.read();
    }
    return;
  }
  //counter++;

  // read packet
  for (int i = 0; i < packetSize; i++) {
    arRec[i] = LoRa.read();
    //Serial.print(getHexFromByte(arRec[i]));
  }
  i16Packetsize = packetSize;
  u8ewMessage = 1;
  //Serial.println();
  //decodePacket(&arRec[0],packetSize);
  //display.display();


  
  /*
  // received a packet
  Serial.print("Received packet '");
  // read packet
  for (int i = 0; i < packetSize; i++) {
    Serial.print((char)LoRa.read());
  }

  // print RSSI of packet
  Serial.print("' with RSSI ");
  Serial.println(LoRa.packetRssi());
  */
}


bool FanetLora::begin(int8_t sck, int8_t miso, int8_t mosi, int8_t ss,int reset, int dio0){
  checkMyDevId(); //get Dev-Id out of MAC
  Serial.print("MyDevId:");Serial.println(getMyDevId());
  //SPI LoRa pins
  SPI.begin(sck, miso, mosi, ss);
  //setup LoRa transceiver module
  LoRa.setPins(ss, reset, dio0);

  while (!LoRa.begin(BAND) && counter < 10) {
    Serial.print(".");
    counter++;
    delay(500);
  }
  if (counter == 10) {
    Serial.println("Starting LoRa failed!"); 
  }
  LoRa.setSignalBandwidth(250E3); //set Bandwidth to 250kHz
  LoRa.setSpreadingFactor(7); //set spreading-factor to 7
  LoRa.setCodingRate4(8);
  LoRa.setSyncWord(0xF1);
  LoRa.enableCrc();
  //LoRa.setTxPower(10); //10dbm + 4dbm antenna --> max 14dbm
  LoRa.setTxPower(20); //full Power
  Serial.println("LoRa Initialization OK!");



  initCount = 0;
  _PilotName = "";  
  _myData.aircraftType = eFanetAircraftType::PARA_GLIDER; //default Paraglider
  newMsg = false;
  u8ewMessage = 0; //now we have new processed the msg
  // put the radio into receive mode
  //LoRa.onReceive(onReceive);
  //LoRa.receive();
  return true;
}


void FanetLora::setPilotname(String name){
    _PilotName = name;
}

void FanetLora::setAircraftType(eFanetAircraftType type){
    _myData.aircraftType = type;
}

String FanetLora::CreateFNFMSG(char *recBuff,uint8_t size){
  String msg = "";  
  fanet_header_t *tHeader = (fanet_header_t *)&recBuff[0];
  if ((myDevId[0] == recBuff[1]) && (myDevId[1] == recBuff[2]) && (myDevId[2] == recBuff[3])){
      //this is me --> abort
      Serial.println("source is my id --> abort");
      return "";
  }

  //Serial.print("type:");Serial.println(tHeader->type);
  //Serial.print("forward:");Serial.println(tHeader->forward);
  //Serial.print("exHeader:");Serial.println(tHeader->ext_header);
  //Serial.print("manu:");Serial.println(getHexFromByte(tHeader->vendor));
  //Serial.print("uniqueId:");Serial.println(getHexFromWord(tHeader->address));

  uint8_t offset = 4;
  if (tHeader->ext_header){
      offset ++;
      if (tHeader->unicast) offset += 3; //if unicast add 3 Byte für dest-addr
      if (tHeader->signature) offset += 4; //if signature add 4 Byte for signature
  } 
  uint8_t msgLen = (size - offset);
  //Serial.print("LEN=");Serial.println(msgLen);
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
  for (int i = offset;i < size;i++){
      payload += getHexFromByte(recBuff[i],true);
  }
  msg += payload;
  if (tHeader->type == 1){
      actTrackingData.DevId = getHexFromByte(recBuff[1],true) + getHexFromWord(tHeader->address,true);
      getTrackingInfo(payload,msgLen);
  }else if (tHeader->type == 2){
      Serial.println("************ name received *********************");
  }
  Serial.println(msg);
  actMsg = msg;
  newMsg = true;
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
  //char recBuff[30];
  //char c;
  // received a packet
  
  /*
  Serial.print("Received packet size=");
  Serial.print(PacketSize);
  Serial.print(" with RSSI ");
  Serial.println(LoRa.packetRssi());
  // read packet
  for (int i = 0; i < PacketSize; i++) {
    c = (char)LoRa.read();
    //Serial.print(getHexFromByte(c));
    if (i < 30){
        recBuff[i] = c;
    }    
    //Serial.print(getHexFromByte((char)LoRa.read()));
  }
  //Serial.println();
  if (PacketSize < 30)   CreateFNFMSG(&recBuff[0],PacketSize);

  // print RSSI of packet

  for (int i = 0; i < PacketSize; i++) {
    c = (char)LoRa.read();
    Serial.print(c);
  }
  */
  CreateFNFMSG((char *)&arRec[0],(uint8_t)i16Packetsize);
  u8ewMessage = 0; //now we have new processed the msg
}

void FanetLora::run(void){    
    uint32_t tAct = millis();
    //if (PacketSize > 0){
    i16Packetsize = LoRa.parsePacket();
    if (i16Packetsize > 0){
        Serial.print("new lora-msg size=");
        Serial.println(i16Packetsize);
        for (int i = 0; i < i16Packetsize; i++) {
            arRec[i] = LoRa.read();
            Serial.print(getHexFromByte(arRec[i]));
        }
        Serial.println("");
        getLoraMsg();
    }
    
    /*    
    if (u8ewMessage){
        getLoraMsg();
        //PacketSize = 0;
    }
    */
    /*
    while (pFanetSerial->available()){
        if (recBufferIndex >= FANET_MAXRECBUFFER) recBufferIndex = 0; //Buffer overrun
        lineBuffer[recBufferIndex] = pFanetSerial->read();
        if (lineBuffer[recBufferIndex] == '\n'){
            //Serial.print("length=");Serial.println(recBufferIndex);
            lineBuffer[recBufferIndex] = 0; //zero-termination
            //Serial.println(lineBuffer);
            DecodeLine(String(lineBuffer));    
            recBufferIndex = 0;
        }else{
            if (lineBuffer[recBufferIndex] != '\r'){
                recBufferIndex++;
            }
        }    
        
        
        //String line = pFanetSerial->readStringUntil('\n');
        //Serial.println(line);
        //DecodeLine(line);
    }
    */
    sendPilotName(tAct);
}

void FanetLora::sendPilotName(uint32_t tAct){
    static uint32_t tPilotName = millis();
    if ((tAct - tPilotName) >= 24000){
        tPilotName = tAct;
        if (_PilotName.length() > 0){
            //Serial.println("sending fanet-name");
            LoRa.beginPacket();
            LoRa.write(0x42);
            LoRa.write(myDevId[0]);
            LoRa.write(myDevId[1]);
            LoRa.write(myDevId[2]);
            LoRa.print(_PilotName);
            LoRa.endPacket();   
            LoRa.parsePacket();
            while (LoRa.available()){
                LoRa.read();  //empty buffer
            }
            //LoRa.receive(); //switch back to receive-mode         
        }
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
    
    int32_t lat_i = 0;
    int32_t lon_i = 0;

    line.toCharArray(arPayload,sizeof(arPayload));

    ((uint8_t*)&lat_i)[0] = getByteFromHex(&arPayload[0]);
    ((uint8_t*)&lat_i)[1] = getByteFromHex(&arPayload[2]);
    ((uint8_t*)&lat_i)[2] = getByteFromHex(&arPayload[4]);

    ((uint8_t*)&lon_i)[0] = getByteFromHex(&arPayload[6]);
    ((uint8_t*)&lon_i)[1] = getByteFromHex(&arPayload[8]);
    ((uint8_t*)&lon_i)[2] = getByteFromHex(&arPayload[10]);

    actTrackingData.lat = (float) lat_i / 93206.0f;
    actTrackingData.lon = (float) lon_i / 46603.0f;

    uint16_t Type = (uint16_t(getByteFromHex(&arPayload[14])) << 8) + uint16_t(getByteFromHex(&arPayload[12]));
    uint16_t altitude = Type & 0x7FF;
    if  (Type & 0x0800){
        altitude *= 4;
    } 
    actTrackingData.altitude = altitude;
    actTrackingData.aircraftType = (eFanetAircraftType)((Type >> 12) & 0x07);

    uint16_t speed = uint16_t(getByteFromHex(&arPayload[16]));
    if (speed & 0x80){
        speed = (speed & 0x80) * 5;
    }else{
        speed = (speed & 0x80);
    }
    actTrackingData.speed = float(speed) * 0.5;

    int8_t climb = int8_t(getByteFromHex(&arPayload[18]));
    if (climb & 0x80){
        climb = (climb & 0x80) * 5;
    }else{
        climb = (climb & 0x80);
    }
    actTrackingData.climb = float(climb) * 0.1;

    actTrackingData.heading = float(getByteFromHex(&arPayload[20])) * 360 / 255;
    if (actTrackingData.heading  < 0) actTrackingData.heading += 360.0;
    newData = true;
}




/*
void FanetLora::resetModule(){
    digitalWrite(_ResetPin, LOW);
    delay(500);
    digitalWrite(_ResetPin, HIGH);
    delay(500);
}

void FanetLora::initModule(uint32_t tAct){
    static uint32_t timeout = millis();
    bool btimeout = false;
    if ((tAct - timeout) >= 1000){
        btimeout = true;
    }
    switch (initCount)
    {
    case 0:
        bFNAOk = false;
        //Serial.print("#FNA\n");
        pFanetSerial->print("#FNA\n"); //get module-addr
        timeout = tAct; //reset timeout
        initCount++;
        break;
    case 1:
        if (btimeout){
            initCount--; //back on step ask module again
        }
        if (bFNAOk){
            bFAXOk = false;
            pFanetSerial->print("#FAX\n"); //flarm expiration
            timeout = tAct; //reset timeout
            initCount++;
        }
        break;
    case 2:
        if (btimeout){
            initCount--; //back on step ask module again
        }
        if (bFAXOk){
            bFNCOk = false;
            //Serial.print("#FNC 1,1\n");
            String sMsg = "#FNC " + String(int(_myData.aircraftType)) + ",1\n";
            //pFanetSerial->print("#FNC 1,1\n"); //PG, online tracking
            Serial.print(sMsg);
            pFanetSerial->print(sMsg); //PG, online tracking
            timeout = tAct; //reset timeout
            initCount++;
        }
        break;
    case 3:
        if (btimeout){
            initCount--; //back on step ask module again
        }
        if (bFNCOk){
            bDGPOk = false;
            //Serial.print("#DGP 1\n");
            pFanetSerial->print("#DGP 1\n"); //Enable receiver
            timeout = tAct; //reset timeout
            initCount++;
        }
        break;
    case 4:
        if (btimeout){
            initCount--; //back on step ask module again
        }
        if (bDGPOk){
            bFAPOk = false;
            //Serial.print("#FAP 1\n");
            pFanetSerial->print("#FAP 1\n"); //Enable FLARM
            timeout = tAct; //reset timeout
            initCount++;
        }
        break;
    case 5:
        if (btimeout){
            initCount--; //back on step ask module again
        }
        if (bFAPOk){
            initCount = 100; //we are ready !!
            //Serial.println("**** INIT OK ****");
            bInitOk = true;
        }
        break;
    
    default:
        break;
    }
}

String FanetLora::getFlarmExp(void){
    return FlarmExp;
}


void FanetLora::getMyID(String line){
    _myData.DevId = line.substring(5,7) + line.substring(8,12);
    //Serial.print("*******myID=");
    //Serial.println(_myData.DevId);

}

void FanetLora::getFAX(String line){
    String s1;
    int ret = 0;
    FlarmExp = "";
    ret = getStringValue(line,&s1,ret,"#FAX ",",");
    if (ret >= 0){
        FlarmExp = String(atoi(s1.c_str()) + 1900);
    }
    ret = getStringValue(line,&s1,ret,",",",");
    if (ret >= 0){
        FlarmExp += "-" + String(atoi(s1.c_str()));
    }
    ret = getStringValue(line,&s1,ret,",","");
    if (ret >= 0){
        FlarmExp += "-" + String(atoi(s1.c_str()));
    }
    Serial.println(FlarmExp);
}

bool FanetLora::initOk(void){
    return bInitOk;
}

void FanetLora::DecodeLine(String line){
    Serial.println(line);
    if (line.startsWith("#DGV")){
    }else if (line.startsWith("#FNA")){
        getMyID(line);
        bFNAOk = true;
    }else if (line.startsWith("#FAX")){
        getFAX(line);
        bFAXOk = true;
    }else if (line.startsWith("#FNR OK")){
        bFNCOk = true;
    }else if (line.startsWith("#DGR OK")){
        bDGPOk = true;
    }else if (line.startsWith("#FAR OK")){
        bFAPOk = true;
    }else if (line.startsWith("#FNF")){
        //we have received tracking-information
        CheckReceivedPackage(line);
    }else{
        Serial.println("UNKNOWN MSG:");
        Serial.println(line);
    }

}

void FanetLora::sendWeather(void){
    String sSend = "#FNT 4,00,0000,1,0,A,A0965E44687A0A7F4B96\n";
    Serial.println("sending weather-data");
    Serial.println(sSend);
    pFanetSerial->print(sSend);
}

void FanetLora::CheckReceivedPackage(String line){
    String s1;
    String devId = "";
    int ret;
    ret = getStringValue(line,&s1,0,"#FNF ",",");
    //Serial.print("ret=");
    //Serial.println(ret);
    //Serial.print("src_manufacturer=");
    //Serial.println(s1);
    if (s1 == "7"){
        devId = "F1";
    }else{
        devId = s1;
    }
    ret = getStringValue(line,&s1,ret,",",",");
    devId += s1;
    //Serial.print("src_id=");
    //Serial.println(s1);

    ret = getStringValue(line,&s1,ret,",",",");
    //Serial.print("broadcast=");
    //Serial.println(s1);

    ret = getStringValue(line,&s1,ret,",",",");
    //Serial.print("signature=");
    //Serial.println(s1);

    ret = getStringValue(line,&s1,ret,",",",");
    //Serial.print("type=");
    //Serial.println(s1);
    uint8_t msgType = s1.toInt();
    ret = getStringValue(line,&s1,ret,",",",");
    //Serial.print("length=");
    //Serial.println(s1);
    long x = strtol(s1.c_str(),NULL,16) * 2;
    //Serial.println(x);
    String payload = line.substring(ret+1);
    //Serial.println(payload.length());
    if (msgType == 1){
        if (payload.length() == x){
            actTrackingData.DevId = devId;
            getTrackingInfo(payload,uint16_t(x));                
        }else{
            Serial.println("length not ok");
        }
        //Serial.println(line); //directly to serial out
    }else if ((msgType == 2) || (msgType == 3) || (msgType == 4)){
        //Type 2 --> Device-Name
        //Type 3 --> MSG
        //Type 4 --> weather-data
        if (pNmeaOut != NULL){
            pNmeaOut->write(line + "\r\n"); //directly to serial out
        }
    }
}



bool Fanet::newTrackingDataAvaiable(void){
    return newData;
}

String Fanet::getStringValue(String s,String keyword){
  String sRet = "";
  int pos = s.indexOf(keyword);
  if (pos < 0) return sRet; //not found
  sRet = s.substring(pos + keyword.length());
  return sRet;
}

int Fanet::getStringValue(String s,String *sRet,unsigned int fromIndex,String keyword,String delimiter){
  *sRet = "";
  int pos = s.indexOf(keyword,fromIndex);
  if (pos < 0) return -1; //not found
  pos += keyword.length();
  int pos2 = 0;
  if (delimiter.length() > 0){
    pos2 = s.indexOf(delimiter,pos);
  }else{
      pos2 = s.length();
  }
  if (pos2 < 0) return -1; //not found
  *sRet = s.substring(pos,pos2);
  return pos2;
}
void Fanet::writeStateData2FANET(stateData *tData){
    String sFNS = "#FNS " + String(tData->lat,4) + ","
                 + String(tData->lon,4) + ","
                 + String(tData->altitude,0) + "," 
                 + String(int(tData->speed)) + "," 
                 + String(tData->climb,2) + "," 
                 + String(tData->heading,2) + "," 
                 + String(tData->year) + "," 
                 + String(tData->month-1) + "," 
                 + String(tData->day)+ "," 
                 + String(tData->hour) +  "," 
                 + String(tData->minute) +  "," 
                 + String(tData->second) + "," 
                 + String(tData->geoIdAltitude,0) + ",0\n"; //todo check geoid-height
    //Serial.println(sFNS);
    _myData.altitude = tData->altitude;
    _myData.climb = tData->climb;
    _myData.heading = tData->heading;
    _myData.lat = tData->lat;
    _myData.lon = tData->lon;
    _myData.speed = tData->speed;
    if (bInitOk){
        pFanetSerial->print(sFNS);
    }
}
*/

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

void FanetLora::writeTrackingData2FANET(trackingData *tData){
    uint8_t sendBuffer[20];
    uint8_t sendindex = 0;
    //Serial.println("sending tracking-data");
    fanet_packet_t *pkt = (fanet_packet_t *)&sendBuffer[0];
    pkt->ext_header     = 0;
    pkt->forward        = 1;
    pkt->type           = 1;  /* Tracking  */
    pkt->vendor         = myDevId[0];
    pkt->address        = ((uint16_t)myDevId[2] << 8) + (uint16_t)myDevId[1];
    
    coord2payload_absolut(tData->lat,tData->lon, ((uint8_t *) pkt) + FANET_HEADER_SIZE);
    pkt->track_online = 1;
    pkt->aircraft_type  = uint16_t(_myData.aircraftType);
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
    if(climb10 > 63) {
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
    LoRa.parsePacket();
    while (LoRa.available()){
        LoRa.read();  //empty buffer
    }
    //LoRa.receive(); //switch back to receive-mode
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


void FanetLora::printFanetData(trackingData tData){
    Serial.print("id=");
    Serial.println(tData.DevId);
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