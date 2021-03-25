/*!
 * @file FanetLora.cpp
 *
 *
 */

#include "FanetLora.h"
#include "Legacy/Legacy.h"

FanetLora::FanetLora(){
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

void FanetLora::end(void){
  fmac.end();
}

String FanetLora::getMyDevId(void){
    return getDevId(_myData.devId);
}

bool FanetLora::begin(int8_t sck, int8_t miso, int8_t mosi, int8_t ss,int reset, int dio0,long frequency,uint8_t outputPower,uint8_t radio){
  valid_until = millis() - 1000; //set Data to not valid
  _myData.lat = 0.0;
  _myData.lon = 0.0;
  Fapp * fa = this;
  //_myData.devId = getDevIdFromMac(fmac.myAddr);
  

  _PilotName = "";  
  _myData.aircraftType = FanetLora::aircraft_t::paraglider; //default Paraglider
  _myData.lat = NAN;
  _myData.lon = NAN;
  newMsg = false;
    for (int i = 0; i < MAXNEIGHBOURS; i++){ //clear neighbours list
    neighbours[i].devId = 0; 
  }

  bool bRet = fmac.begin(sck, miso, mosi, ss, reset, dio0,*fa,frequency,outputPower,radio);
  _myData.devId = ((uint32_t)fmac.myAddr.manufacturer << 16) + (uint32_t)fmac.myAddr.id;
  log_i("myDevId:%02X%04X",fmac.myAddr.manufacturer,fmac.myAddr.id);
  if (!bRet){
    log_e("radio failed");
    return false;
  }

  fmac.setLegacy(_enableLegacyTx);
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

/* send msg: @typedest_manufacturer,msg */
/*
void FanetLora::fanet_sendMsg(char *ch_str){
	char *ptr = strchr(ch_str, '\r');
	if(ptr == nullptr)
		ptr = strchr(ch_str, '\n');
	if(ptr != nullptr)
		*ptr = '\0';

  char *p = (char *)ch_str;
  uint32_t devId = strtol(p, NULL, 16);
  p = strchr(p, SEPARATOR)+1;
  String msg = p;
  log_i("msg=%s",msg.c_str());
  Frame *frm = new Frame(fmac.myAddr);
  frm->type = FRM_TYPE_MESSAGE;
  frm->dest = getMacFromDevId(devId);
  frm->payload_length = serialize_msg(msg,frm->payload);
  log_i("sending fanet-msg:%s length=%d",msg.c_str(),frm->payload_length);
  if (frm2txBuffer(frm)) log_i("#FNR OK");
}
*/

// set groundtrackingtype 
// #FNG groundType ( 0 . . F i n hex )\r\n
void FanetLora::fanet_cmd_setGroundTrackingType(char *ch_str){
	/* remove \r\n and any spaces */
	char *ptr = strchr(ch_str, '\r');
	if(ptr == nullptr)
		ptr = strchr(ch_str, '\n');
	if(ptr != nullptr)
		*ptr = '\0';
	while(*ch_str == ' ')
		ch_str++;

	char *p = (char *)ch_str;
	state = (status_t)strtol(p, NULL, 16);
  //log_i("set ground-tracking-status:%d",(uint8_t)state);
  add2ActMsg("#FNR OK");
}

/* Transmit: #FNT type,dest_manufacturer,dest_id,forward,ack_required,length,length*2hex[,signature] */
//note: all in HEX
void FanetLora::fanet_cmd_transmit(char *ch_str)
{
#if SERIAL_debug_mode > 0
  log_i("### Packet %s", ch_str);
#endif

	/* remove \r\n and any spaces */
	char *ptr = strchr(ch_str, '\r');
	if(ptr == nullptr)
		ptr = strchr(ch_str, '\n');
	if(ptr != nullptr)
		*ptr = '\0';
	while(*ch_str == ' ')
		ch_str++;

	/* integrity check */
  char lastChar = ' ';
	for(char *ptr = ch_str; *ptr != '\0'; ptr++)
	{
		lastChar = *ptr;
    if(*ptr >= '0' && *ptr <= '9')
			continue;
		if(*ptr >= 'A' && *ptr <= 'F')
			continue;
		if(*ptr >= 'a' && *ptr <= 'f')
			continue;
		if(*ptr == ',')
			continue;

		/* not allowed char */
		//log_e("frm broken");
    add2ActMsg("#FNR ERR,34,frm broken");
		return;
	}
  if (lastChar == ','){
		//log_e("frm broken");
    add2ActMsg("#FNR ERR,34,frm broken");
		return;
  }

	/* w/o an address we can not tx */
	if(fmac.myAddr == MacAddr())
	{
		//log_e("no source address");
    add2ActMsg("#FNR ERR,10,no source address");
		return;
	}

	/* no need to generate a package. tx queue is full */
	if(!fmac.txQueueHasFreeSlots())
	{
		//log_e("tx buffer full");
    add2ActMsg("#FNR ERR,14,tx buffer full");
		return;
	}

  Frame *frm = new Frame(fmac.myAddr);

	/* header */
	char *p = (char *)ch_str;
	frm->type = strtol(p, NULL, 16);
	p = strchr(p, SEPARATOR)+1;
	frm->dest.manufacturer = strtol(p, NULL, 16);
	p = strchr(p, SEPARATOR)+1;
	frm->dest.id = strtol(p, NULL, 16);
	p = strchr(p, SEPARATOR)+1;
	frm->forward = !!strtol(p, NULL, 16);
	p = strchr(p, SEPARATOR)+1;
	/* ACK required */
	if(strtol(p, NULL, 16))
	{
		frm->ack_requested = frm->forward?FRM_ACK_TWOHOP:FRM_ACK_SINGLEHOP;
		frm->num_tx = MAC_TX_RETRANSMISSION_RETRYS;
	}
	else
	{
		frm->ack_requested = FRM_NOACK;
		frm->num_tx = 0;
	}

	/* payload */
	p = strchr(p, SEPARATOR)+1;
	frm->payload_length = strtol(p, NULL, 16);
	if(frm->payload_length >= 128)
	{
		delete frm;
		//log_e("frm too long");
    add2ActMsg("#FNR ERR,33,frm too long");
		return;
	}
	frm->payload = new uint8_t[frm->payload_length];

	p = strchr(p, SEPARATOR)+1;
	for(int i=0; i<frm->payload_length; i++)
	{
		char sstr[3] = {p[i*2], p[i*2+1], '\0'};
		if(strlen(sstr) != 2)
		{
			//log_e("too short");      
      add2ActMsg("#FNR ERR,30,too short");
			delete frm;
			return;
		}
		frm->payload[i] = strtol(sstr,  NULL,  16);
	}

	/* signature */
	if((p = strchr(p, SEPARATOR)) != NULL)
		frm->signature = ((uint32_t)strtoll(++p, NULL, 16));

	/* pass to mac */
  if (frm2txBuffer(frm)){
    //log_i("%s",actMsg.c_str());
    add2ActMsg("#FNR OK");    
  }else{
    add2ActMsg("#FNR ERR,14,tx buffer full");
  } 
	/*
  if(fmac.transmit(frm) == 0)
	{
		//if(!Lora.isArmed())
		//	log_e("power down");
		//else
		log_i("#FNR OK");
#ifdef FANET_NAME_AUTOBRDCAST
		if(frm->type == FRM_TYPE_NAME)
			app.allow_brdcast_name(false);
#endif
	}
	else
	{
		delete frm;
		log_e("tx buffer full");
	}*/
}
int16_t FanetLora::getWeatherIndex(uint32_t devId,bool getEmptyEntry){
  int16_t iRet = -1;
  uint32_t tElapsed = 0;
  uint32_t tMaxElapsed = 0;
  int16_t iOldestEntry = -1;
  uint32_t tAct = millis();
  for (int i = 0; i < MAXWEATHERDATAS; i++){
    if (!weatherDatas[i].devId){
      tElapsed = gettimeElapsed(tAct,weatherDatas[i].tLastMsg);
      if (tElapsed > tMaxElapsed){
        tMaxElapsed = tElapsed;
        iOldestEntry = i;
      }
    }    
    if (weatherDatas[i].devId == devId){
      return i; //found entry
    }
    
    if ((weatherDatas[i].devId == 0) && (iRet < 0)){
      iRet = i; //found empty one
    }
  }
  if (!getEmptyEntry) return -1; //not found in list
  if (iRet >= 0){
    memset(&weatherDatas[iRet],0,sizeof(weatherDatas[iRet])); //clear slot
    return iRet; //we give back an empty slot
  }else{
    return iOldestEntry; //we tell the oldest entry to override !! (only if we to much traffic)
  }
}

int16_t FanetLora::getneighbourIndex(uint32_t devId,bool getEmptyEntry){
  int16_t iRet = -1;
  uint32_t tElapsed = 0;
  uint32_t tMaxElapsed = 0;
  int16_t iOldestEntry = -1;
  uint32_t tAct = millis();
  for (int i = 0; i < MAXNEIGHBOURS; i++){
    if (!neighbours[i].devId){
      tElapsed = gettimeElapsed(tAct,neighbours[i].tLastMsg);
      if (tElapsed > tMaxElapsed){
        tMaxElapsed = tElapsed;
        iOldestEntry = i;
      }
    }    
    if (neighbours[i].devId == devId){
      return i; //found entry
    }
    
    if ((neighbours[i].devId == 0) && (iRet < 0)){
      iRet = i; //found empty one
    }
  }
  if (!getEmptyEntry) return -1; //not found in list
  if (iRet >= 0){
    memset(&neighbours[iRet],0,sizeof(neighbours[iRet])); //clear slot
    return iRet; //we give back an empty slot
  }else{
    return iOldestEntry; //we tell the oldest entry to override !! (only if we to much traffic)
  }
}
void FanetLora::insertNameToWeather(uint32_t devId, String name){
  int16_t index = getWeatherIndex(devId,false);
  //log_i("devId=%06X",devId);
  //log_i("name=%s",name.c_str());
  //log_i("index=%i",index);
  if (index < 0) return;
  weatherDatas[index].tLastMsg = millis();
  weatherDatas[index].name = name;
}

void FanetLora::insertNameToNeighbour(uint32_t devId, String name){
  int16_t index = getneighbourIndex(devId,false);
  //log_i("devId=%06X",devId);
  //log_i("name=%s",name.c_str());
  //log_i("index=%i",index);
  if (index < 0) return;
  neighbours[index].tLastMsg = millis();
  neighbours[index].name = name;
}

void FanetLora::insertDataToWeatherStation(uint32_t devId, weatherData *Data){
  int16_t index = getWeatherIndex(devId,true);
  if (index < 0) return;
  Data->name = weatherDatas[index].name; // we have to use existing name !!
  weatherDatas[index] = *Data;  
  weatherDatas[index].tLastMsg = millis();
}

void FanetLora::insertDataToNeighbour(uint32_t devId, trackingData *Data){
  int16_t index = getneighbourIndex(devId,true);
  //log_i("devId=%06X",devId);
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
  neighbours[index].type = Data->type;
  neighbours[index].addressType = Data->addressType;
}

void FanetLora::clearNeighboursWeather(uint32_t tAct){
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
    for (int i = 0; i < MAXWEATHERDATAS; i++){
      if (weatherDatas[i].devId){
        if ((tCheck - weatherDatas[i].tLastMsg) >= NEIGHBOURSLIFETIME){ //if we get no msg in 4min --> del neighbour
          //log_i("clear slot %i devId %s",i,getDevId(neighbours[i].devId).c_str());
          weatherDatas[i].devId = 0; //clear slot
        }
      }
    }
  }
}

void FanetLora::setPilotname(String name){
    _PilotName = name;
}

void FanetLora::setAircraftType(aircraft_t type){
    _myData.aircraftType = type;
}

int16_t FanetLora::getNextNeighbor(uint8_t index){
  uint8_t actIndex = constrain(index,0,MAXNEIGHBOURS);
  uint8_t Ret = 0;
  for (int i = 0; i < MAXNEIGHBOURS; i++){
    actIndex++;
    if (actIndex >= MAXNEIGHBOURS) actIndex = 0;
    if (neighbours[actIndex].devId){
      return actIndex;
    }
  }
  return -1;
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

String FanetLora::getactMsg(){
    String ret = actMsg;
    actMsg = "";
    return ret;
}

bool FanetLora::getlastMsgData(msgData *data){
  if (lastMsgData.srcDevId == 0) return false;
  *data = lastMsgData;
  lastMsgData.srcDevId = 0;
  return true;
}

bool FanetLora::isNewMsg(){
    bool ret = newMsg;
    newMsg = false;
    return ret;
}

void FanetLora::run(void){    
  uint32_t tAct = millis();
  clearNeighboursWeather(millis());
  if (autobroadcast){
    sendPilotName(tAct);
  }
  fmac.handle();
}

void FanetLora::setLegacy(uint8_t enableTx){ 
    _enableLegacyTx = enableTx;
}

Frame *FanetLora::get_frame()
{
	/* prepare frame */
	Frame *frm = new Frame(fmac.myAddr);
		/* broadcast tracking information */
		if(onGround == false)
		{
			frm->type = FRM_TYPE_TRACKING;
			frm->payload_length = serialize_tracking(&_myData,frm->payload);

      if (_enableLegacyTx>0)
        createLegacyPkt(&_myData,fmac.geoidAlt,Legacy_Buffer);
        fmac.bSendLegacy = true; //send Legacy-Information
		}
		else
		{
			frm->type = FRM_TYPE_GROUNDTRACKING;
			frm->payload_length = serialize_GroundTracking(&_myData,frm->payload);
		}
	/* in case of a busy channel, ensure that frames from the fifo gets also a change */
	next_tx = millis() + FANET_LORA_TYPE1OR7_TAU_MS;

	return frm;
}

void FanetLora::handle_acked(bool ack, MacAddr &addr) 
{ 
	//log_i("Handle ACK");
	char buf[64];
	snprintf(buf, sizeof(buf), "%s,%X,%X", ack?"#FNR ACK":"#FNR NACK", addr.manufacturer, addr.id);
  //log_i("%s",buf);
  add2ActMsg(buf);
  rxCount++;
}

String FanetLora::CreateFNFMSG(Frame *frm){
  String msg;
  msg = "#FNF " + getHexFromByte(frm->src.manufacturer) + "," + getHexFromWord(frm->src.id) + "," + getHexFromByte(frm->dest==MacAddr()) + "," + getHexFromByte(frm->signature) + "," + getHexFromByte(frm->type) + "," + getHexFromByte(frm->payload_length) + ",";
  String payload = ""; 
	for(int i=0; i<frm->payload_length; i++)
	{
		payload += getHexFromByte(frm->payload[i],true);
	}
  msg += payload;
  return msg;  
}

float FanetLora::getSpeedFromByte(uint8_t speed){
  if (speed & 0x80){
    return float(speed & 0x7F);
  }else{
    return float(speed) / 5.0;
  }
}

void FanetLora::getWeatherinfo(uint8_t *buffer,uint16_t length){
  uint8_t index = 0;
  uint8_t header = buffer[index];
  //log_i("header=%d",header);
  index ++;
  if (header & 0x01) index ++; //additional header direct after first byte
  // integer values /
  int32_t lati = int32_t(buffer[index+2])<<16 | int32_t(buffer[index+1])<<8 | int32_t(buffer[index]);
  index += 3;
  if(lati & 0x00800000)
    lati |= 0xFF000000;
  int32_t loni = int32_t(buffer[index+2])<<16 | int32_t(buffer[index+1])<<8 | int32_t(buffer[index]);
  index += 3;
  if(loni & 0x00800000)
    loni |= 0xFF000000;    
  lastWeatherData.lat = (float)lati / 93206.0f;
  lastWeatherData.lon = (float)loni / 46603.0f;
  //log_i("lat=%.6f;lon=%.6f",lastWeatherData.lat,lastWeatherData.lon);
  if (header & 0x40){ //temp
    int8_t temp = (int8_t)buffer[index];
    lastWeatherData.bTemp = true;
    lastWeatherData.temp = float(temp) / 2;
    index ++;
    //log_i("temp=%.1f",lastWeatherData.temp);
  }else{
    lastWeatherData.bTemp = false;
    lastWeatherData.temp = NAN;
  }
  if (header & 0x20){ //wind
    lastWeatherData.bWind = true;
    lastWeatherData.wHeading = float(buffer[index]) * 360 / 256;
    index ++;
    lastWeatherData.wSpeed = getSpeedFromByte(buffer[index]);
    index ++;
    lastWeatherData.wGust = getSpeedFromByte(buffer[index]);
    index ++;
    //log_i("wdir=%.1f,wspeed=%.1f,wgust=%.1f",lastWeatherData.wHeading,lastWeatherData.wSpeed,lastWeatherData.wGust);    
  }else{
    lastWeatherData.bWind = false;
    lastWeatherData.wHeading = NAN;
    lastWeatherData.wSpeed = NAN;
    lastWeatherData.wGust = NAN;
  }
  if (header & 0x10){ //humiditiy
    lastWeatherData.bHumidity = true;
    lastWeatherData.Humidity = float(buffer[index]) * 4 / 10;
    index ++;
    //log_i("%d,hum=%.1f",index,lastWeatherData.Humidity);
  }else{
    lastWeatherData.Humidity = NAN;
    lastWeatherData.bHumidity = false;
  }
  if (header & 0x8){ //pressure
    uint16_t press = uint16_t(buffer[index+1])<<8 | uint16_t(buffer[index]);
    index +=2;
    lastWeatherData.bBaro = true;
    lastWeatherData.Baro = (float(press) / 10) + 430;
    //log_i("baro=%.1f",lastWeatherData.Baro);
  }else{
    lastWeatherData.bBaro = false;
    lastWeatherData.Baro = NAN;
  }
  if (header & 0x2){ //state of charge
    uint8_t charge = buffer[index];
    index +=1;
    lastWeatherData.bStateOfCharge = true;
    lastWeatherData.Charge = float(charge) * 100 / 15; //State of Charge  (+1byte lower 4 bits: 0x00 = 0%, 0x01 = 6.666%, .. 0x0F = 100%)
  }else{
    lastWeatherData.bStateOfCharge = false;
    lastWeatherData.Charge = NAN;
  }
  newWData = true;
}



void FanetLora::getGroundTrackingInfo(uint8_t *buffer,uint16_t length){
    char arPayload[23];
    uint8_t index = 0;
      // integer values /
    int32_t lati = int32_t(buffer[index+2])<<16 | int32_t(buffer[index+1])<<8 | int32_t(buffer[index]);
    index += 3;
    if(lati & 0x00800000)
      lati |= 0xFF000000;
    int32_t loni = int32_t(buffer[index+2])<<16 | int32_t(buffer[index+1])<<8 | int32_t(buffer[index]);
    index += 3;
    if(loni & 0x00800000)
      loni |= 0xFF000000;    
    actTrackingData.lat = (float)lati / 93206.0f;
    actTrackingData.lon = (float)loni / 46603.0f;
    //Serial.print("FANETlat=");Serial.println(actTrackingData.lat);
    //Serial.print("FANETlon=");Serial.println(actTrackingData.lon);

    uint8_t type = buffer[index];
    if (type & 0x01){
      actTrackingData.OnlineTracking = true;  
    }else{
      actTrackingData.OnlineTracking = false;
    }
    //log_i("online-tracking=%d",actTrackingData.OnlineTracking);
    actTrackingData.type = 0x70 + (type >> 4);
    //log_i("type=%d,groundType=%d",type,actTrackingData.type);
    newData = true;

}

bool FanetLora::getNameData(nameData *nameData){
    bool bRet = newName;
    *nameData = lastNameData; //copy tracking-data
    memset(&lastNameData,0,sizeof(lastNameData));
    newName = false; //clear new Data flag
    return bRet;
}

bool FanetLora::getWeatherData(weatherData *weather){
    bool bRet = newWData;
    *weather = lastWeatherData; //copy tracking-data
    memset(&lastWeatherData,0,sizeof(lastWeatherData));
    newWData = false; //clear new Data flag
    return bRet;
}

String FanetLora::getType(uint8_t type){
  switch (type)
  {
  case 0x11:
    return "flying";
    break;
  case 0x70:
    return "other";
    break;
  case 0x71:
    return "walking";
    break;
  case 0x72:
    return "vehicle";
    break;
  case 0x73:
    return "bike";
    break;
  case 0x74:
    return "boat";
    break;
  case 0x78:
    return "need a ride";
    break;
  case 0x79:
    return "landed well";
    break;
  case 0x7C:
    return "need technical support";
    break;
  case 0x7D:
    return "need medical help";
    break;
  case 0x7E:
    return "distress call";
    break;
  case 0x7F:
    return "distress call automatically";
    break;
  default:
    char s1[20];
    sprintf(s1,"unknown %01X",type);
    return String(s1);
    break;
  }
}

void FanetLora::handle_frame(Frame *frm){
  //log_i("Handle Frame %d",frm->rssi);
  /*
  String s = "";
  uint8_t* c = frm->payload;
  for (int i = 0; i < frm->payload_length;i++){
    s += getHexFromByte(*c) + ":";
    c++;
  }
  log_i("payload=%s",s.c_str());
  */
  //frm->payload

  //log_i("new msg");

	String msg = CreateFNFMSG(frm);
  String payload = ""; 
  String msg2 = ""; 
	for(int i=0; i<frm->payload_length; i++)
	{
		payload += getHexFromByte(frm->payload[i],true);
    msg2 += (char)frm->payload[i];
	}
  uint32_t devId = getDevIdFromMac(&frm->src);
  if (frm->type == 1){
    //online-tracking
    actTrackingData.devId = ((uint32_t)frm->src.manufacturer << 16) + (uint32_t)frm->src.id;
    actTrackingData.rssi = frm->rssi;
    actTrackingData.snr = frm->snr;
    actTrackingData.type = 0x11;
    actTrackingData.addressType = frm->AddressType;
    getTrackingInfo(payload,frm->payload_length);
    insertDataToNeighbour(actTrackingData.devId,&actTrackingData);
  }else if (frm->type == 2){      
      //log_i("name=%s",msg2.c_str());
      lastNameData.devId = ((uint32_t)frm->src.manufacturer << 16) + (uint32_t)frm->src.id;
      lastNameData.rssi = frm->rssi;
      lastNameData.snr = frm->snr;
      lastNameData.name = msg2;
      newName = true;
      insertNameToWeather(devId,msg2); //insert name in weather-list
      insertNameToNeighbour(devId,msg2); //insert name in neighbour-list
  }else if (frm->type == 3){
    //msg-data --> check if msg is explicit for us.
    //if (frm->dest)
    lastMsgData.rssi = frm->rssi;
    lastMsgData.snr = frm->snr;
    lastMsgData.srcDevId = ((uint32_t)frm->src.manufacturer << 16) + (uint32_t)frm->src.id;
    lastMsgData.dstDevId = ((uint32_t)frm->dest.manufacturer << 16) + (uint32_t)frm->dest.id;
    lastMsgData.msg = "";
    for(int i=1; i<frm->payload_length; i++)
    {
      lastMsgData.msg += (char)frm->payload[i];
    }
  }else if (frm->type == 4){   
    //weather-data   
      lastWeatherData.devId = ((uint32_t)frm->src.manufacturer << 16) + (uint32_t)frm->src.id;
      lastWeatherData.rssi = frm->rssi;
      lastWeatherData.snr = frm->snr;
      getWeatherinfo(frm->payload,frm->payload_length);    
      insertDataToWeatherStation(devId,&lastWeatherData);
  }else if (frm->type == 7){      
    //ground-tracking
    actTrackingData.devId = ((uint32_t)frm->src.manufacturer << 16) + (uint32_t)frm->src.id;
    actTrackingData.rssi = frm->rssi;
    actTrackingData.snr = frm->snr;
    actTrackingData.addressType = frm->AddressType;
    getGroundTrackingInfo(frm->payload,frm->payload_length);
    insertDataToNeighbour(actTrackingData.devId,&actTrackingData);
  }
  //log_i("%s",msg.c_str());
  add2ActMsg(msg);
  rxCount++;
}

void FanetLora::add2ActMsg(String s){
  if (actMsg.length() != 0){
    actMsg += "\n";
  }
  actMsg += s;
  newMsg = true;
}

uint32_t FanetLora::getDevIdFromMac(MacAddr *adr){
  return  ((uint32_t)adr->manufacturer << 16) + (uint32_t)adr->id;
}

MacAddr FanetLora::getMacFromDevId(uint32_t devId){
  MacAddr adr;
  adr.manufacturer = (devId >> 16) & 0xFF;
  adr.id = devId & 0xFFFF;
  return adr;
}

bool FanetLora::is_broadcast_ready(int num_neighbors)
{
  if (!autobroadcast) //autobroadcast not enabled
    return false;
	//log_i("start");
	/* is the state valid? */
	if(millis() > valid_until || isnan(_myData.lat) || isnan(_myData.lon))
		return false;

	/* in case of a busy channel, ensure that frames from the fifo get also a change */
	if(next_tx > millis())
		return false;

	/* determine if its time to send something (again) */
	const int tau_add = (num_neighbors/10 + 1) * FANET_LORA_TYPE1OR7_TAU_MS;
	if(last_tx + tau_add > millis())
		return false;
	//log_i("ready");
	return true;
}

int FanetLora::serialize_msg(String name,uint8_t*& buffer){
		const int namelength = name.length();
		buffer = new uint8_t[namelength+1];
		buffer[0] = 0; //normal msg
    memcpy(&buffer[1], name.c_str(), namelength);
		return namelength+1;
}

int FanetLora::serialize_name(String name,uint8_t*& buffer){
		const int namelength = name.length();
		buffer = new uint8_t[namelength];
		memcpy(buffer, name.c_str(), namelength);
		return namelength;
}

bool FanetLora::frm2txBuffer(Frame *frm){
  //log_i("payload_length=%i",frm->payload_length);
  //log_i("%s",CreateFNFMSG(frm).c_str());
  if(!fmac.txQueueHasFreeSlots()){
    log_e("TX-buffer full");
    return false;
  }
  /* pass to mac */
  if(fmac.transmit(frm) == 0){
		//if(!LoRa.isArmed()) log_e("power down");
    txCount++;
    return true;
  }else{
		delete frm;
		log_e("TX-buffer full");
    return false;
  }
  //log_i("ready");
}

void FanetLora::sendMSG(String msg){
    if (msg.length() > 0){
        Frame *frm = new Frame(fmac.myAddr);
        frm->type = FRM_TYPE_MESSAGE;
        frm->payload_length = serialize_msg(msg,frm->payload);
        //log_i("sending fanet-msg:%s length=%d",msg.c_str(),frm->payload_length);
        frm2txBuffer(frm);
    }
}

void FanetLora::sendName(String name){
    if (name.length() > 0){
        //log_i("sending fanet-name:%s",name.c_str());
        Frame *frm = new Frame(fmac.myAddr);
        frm->type = FRM_TYPE_NAME;
        frm->payload_length = serialize_name(name,frm->payload);
        frm2txBuffer(frm);
    }
}

void FanetLora::sendPilotName(uint32_t tAct){
  static uint32_t tSend = millis() - SENDNAMEINTERVAL + 10000; //send after 10seconds first name
  if (timeOver(tAct,tSend,SENDNAMEINTERVAL)){
    tSend = tAct;
    //log_i("tAct=%lu;tSend=%lu",tAct,tSend);
    sendName(_PilotName);   
  }
}

void FanetLora::broadcast_successful(int type){ 
  last_tx = millis(); 
  //log_i("ready");
  txCount++; 
}

String FanetLora::getAircraftType(aircraft_t type){
  if (type == aircraft_t::paraglider){
    return "PARA_GLIDER";
  }else if (type == aircraft_t::hangglider){
    return "HANG_GLIDER";
  }else if (type == aircraft_t::balloon){
    return "BALLOON";
  }else if (type == aircraft_t::glider){
    return "GLIDER";
  }else if (type == aircraft_t::poweredAircraft){
    return "POWERED_AIRCRAFT";
  }else if (type == aircraft_t::helicopter){
    return "HELICOPTER_ROTORCRAFT";
  }else if (type == aircraft_t::uav){
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
    return String(myHexString);

}

String FanetLora::getHexFromByte(uint8_t val,bool leadingZero){
    char myHexString[3];
    if (leadingZero){
        sprintf(myHexString,"%02X",val);
    }else{
        sprintf(myHexString,"%X",val);
    }
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
    memset(&actTrackingData,0,sizeof(actTrackingData)); //clear tracking-data
    newData = false; //clear new Data flag
    return bRet;
}

void FanetLora::getTrackingInfo(String line,uint16_t length){
    char arPayload[23];

    //log_i("getTrackingInfo");
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

    uint16_t Type = (uint16_t(getByteFromHex(&arPayload[14])) << 8) + uint16_t(getByteFromHex(&arPayload[12]));
    uint16_t altitude = Type & 0x7FF;
    if (Type & 0x8000){
      actTrackingData.OnlineTracking = true;      
    }else{
      actTrackingData.OnlineTracking = false;
    }
    //log_i("online-tracking=%d",actTrackingData.OnlineTracking);
    if  (Type & 0x0800){
        altitude *= 4;
    } 
    actTrackingData.altitude = altitude;
    actTrackingData.aircraftType = (aircraft_t)((Type >> 12) & 0x07);

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

int16_t FanetLora::getNearestNeighborIndex(){
  int16_t Ret = -1;
  float pilotDistance;
  float actDist = 0.0;
  for (int i = 0; i < MAXNEIGHBOURS; i++){
    if (neighbours[i].devId){
      pilotDistance = distance(_myData.lat, _myData.lon,neighbours[i].lat,neighbours[i].lon, 'K') * 1000 ;
      if ((pilotDistance < actDist) || (Ret < 0)){
        actDist = pilotDistance;
        Ret = i;
      }
    }
  }
  return Ret;
}

void FanetLora::sendTracking(trackingData *tData){
  Frame *frm = new Frame(fmac.myAddr);
  frm->type = FRM_TYPE_TRACKING;
  frm->payload_length = serialize_tracking(tData,frm->payload);

  if (_enableLegacyTx>0)
  {
    int oldid = tData->devId;
    tData->devId=_myData.devId;
    createLegacyPkt(tData,fmac.geoidAlt,Legacy_Buffer);
    tData->devId=oldid;
  }

  frm2txBuffer(frm);
}

void FanetLora::writeMsgType1(trackingData *tData){
  sendTracking(tData);
}

void FanetLora::writeMsgType2(String name){
    sendName(name);
}


void FanetLora::writeMsgType3(uint32_t devId,String msg){
    if (msg.length() > 0){
        //log_i("sending fanet-msg:%s",msg.c_str());
        Frame *frm = new Frame(fmac.myAddr);
        frm->type = FRM_TYPE_MESSAGE;
        frm->dest = getMacFromDevId(devId);
        frm->payload_length = serialize_msg(msg,frm->payload);
        //log_i("sending fanet-msg:%s length=%d",msg.c_str(),frm->payload_length);
        frm2txBuffer(frm);
    }
}


FanetLora::aircraft_t FanetLora::getAircraftType(void){
    return _myData.aircraftType;
}

int FanetLora::serialize_GroundTracking(trackingData *Data,uint8_t*& buffer){
	buffer = new uint8_t[FANET_LORA_TYPE7_SIZE];

	/* position */
	Frame::coord2payload_absolut(Data->lat, Data->lon, buffer);

	/* state */
	buffer[6] = (state&0x0F)<<4 | (!!doOnlineTracking);

	return FANET_LORA_TYPE7_SIZE;
}

int FanetLora::serialize_tracking(trackingData *Data,uint8_t*& buffer){
  int msgSize = sizeof(fanet_packet_t1);
  buffer = new uint8_t[msgSize];
  coord2payload_absolut(Data->lat,Data->lon, &buffer[0]);

	/* altitude set the lower 12bit */
	int alt = constrain(Data->altitude, 0, 8190);
	if(alt > 2047)
		((uint16_t*)buffer)[3] = ((alt+2)/4) | (1<<11);				//set scale factor
	else
		((uint16_t*)buffer)[3] = alt;
	/* online tracking */
	((uint16_t*)buffer)[3] |= !!doOnlineTracking<<15;
	/* aircraft type */
	((uint16_t*)buffer)[3] |= (Data->aircraftType&0x7)<<12;

	/* Speed */
	int speed2 = constrain((int)roundf(Data->speed *2.0f), 0, 635);
	if(speed2 > 127)
		buffer[8] = ((speed2+2)/5) | (1<<7);					//set scale factor
	else
		buffer[8] = speed2;

	/* Climb */
	int climb10 = constrain((int)roundf(Data->climb *10.0f), -315, 315);
	if(std::abs(climb10) > 63)
		buffer[9] = ((climb10 + (climb10>=0?2:-2))/5) | (1<<7);			//set scale factor
	else
		buffer[9] = climb10 & 0x7F;

	/* Heading */
	buffer[10] = constrain((int)roundf(Data->heading *256.0f/360.0f), 0, 255);

	return FANET_LORA_TYPE1_SIZE - 2;
}

int FanetLora::serialize_service(weatherData *wData,uint8_t*& buffer){
  int msgSize = sizeof(fanet_packet_t4);
  buffer = new uint8_t[msgSize];
  fanet_packet_t4 *pkt = (fanet_packet_t4 *)&buffer[0];
  pkt->bExt_header2 = false;
  pkt->bStateOfCharge = wData->bStateOfCharge;
  pkt->bRemoteConfig = false;
  pkt->bBaro = wData->bBaro;
  pkt->bHumidity = wData->bHumidity;
  pkt->bWind = wData->bWind;
  pkt->bTemp = wData->bTemp;
  pkt->bInternetGateway = false;
  coord2payload_absolut(wData->lat,wData->lon, &buffer[1]);
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
  return msgSize;
}

void FanetLora::writeMsgType4(weatherData *wData){
  Frame *frm = new Frame(fmac.myAddr);
  frm->type = FRM_TYPE_SERVICE;
  frm->forward = true;
  frm->payload_length = serialize_service(wData,frm->payload);
  frm2txBuffer(frm);
}

void FanetLora::setMyTrackingData(trackingData *tData,float geoidAlt){
    _myData.altitude = tData->altitude;
    _myData.climb = tData->climb;
    _myData.heading = tData->heading;
    _myData.lat = tData->lat;
    _myData.lon = tData->lon;
    _myData.speed = tData->speed;
    fmac.lat = _myData.lat;
    fmac.lon = _myData.lon;
    fmac.geoidAlt = geoidAlt;
    fmac.bPPS = true;
    valid_until = millis() + FANET_LORA_VALID_STATE_MS;
    //log_i("valid_until %lu",valid_until);
}

bool FanetLora::getMyTrackingData(trackingData *tData){
    *tData = _myData;
    return true;
}

void FanetLora::printAircraftType(aircraft_t type){
  Serial.print("Aircraft-type=");
  if (type == aircraft_t::paraglider){
    Serial.println("PARA_GLIDER");
  }else if (type == aircraft_t::hangglider){
    Serial.println("HANG_GLIDER");
  }else if (type == aircraft_t::balloon){
    Serial.println("BALLOON");
  }else if (type == aircraft_t::glider){
    Serial.println("GLIDER");
  }else if (type == aircraft_t::poweredAircraft){
    Serial.println("POWERED_AIRCRAFT");
  }else if (type == aircraft_t::helicopter){
    Serial.println("HELICOPTER_ROTORCRAFT");
  }else if (type == aircraft_t::uav){
    Serial.println("UAV");
  }else{
      Serial.println("UNKNOWN");
  }
}

String FanetLora::getDevId(uint32_t devId){
  return getHexFromByte((devId >> 16) & 0xFF,true) + getHexFromByte((devId >> 8) & 0xFF,true) + getHexFromByte((devId) & 0xFF,true);
}

void FanetLora::printFanetData(FanetLora::trackingData tData){
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