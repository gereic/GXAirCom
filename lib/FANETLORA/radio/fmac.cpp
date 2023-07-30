/*
 * mac.cpp
 *
 *  Created on: 30 Sep 2016
 *      Author: sid
 */

#include <stdlib.h>
#include <math.h>

#include "lib/random.h"
#include "LoRa.h"
#include "fmac.h"
#include <TimeLib.h>
//#include "Legacy/Legacy.h"
#include "../FLARM/FlarmDataPort.h"
#include "../FLARM/lib_crc.h"
#include "WiFi.h"
#include "Udp.h"

int serialize_legacyTracking(ufo_t *Data,uint8_t*& buffer);
int serialize_legacyGroundTracking(ufo_t *Data,uint8_t*& buffer);

/* get next frame which can be sent out */
//todo: this is potentially dangerous, as frm may be deleted in another place.
Frame* MacFifo::get_nexttx()
{
	int next;
	//noInterrupts();
	for (next = 0; next < fifo.size(); next++)
		if (fifo.get(next)->next_tx < millis())
			break;
	Frame *frm;
	if (next == fifo.size())
		frm = NULL;
	else
		frm = fifo.get(next);
	//interrupts();
	return frm;
}

Frame* MacFifo::frame_in_list(Frame *frm)
{
	//noInterrupts();

	for (int i = 0; i < fifo.size(); i++)
	{
		Frame *frm_list = fifo.get(i);
		if (*frm_list == *frm)
		{
			//interrupts();
			return frm_list;
		}
	}

	//interrupts();

	return NULL;
}

Frame* MacFifo::front()
{
	//noInterrupts();
	Frame *frm = fifo.shift();
	//interrupts();

	return frm;
}

/* add frame to fifo */
int MacFifo::add(Frame *frm)
{
	//noInterrupts();

	/* buffer full */
	/* note: ACKs will always fit */
	if (fifo.size() >= MAC_FIFO_SIZE && frm->type != FRM_TYPE_ACK)
	{
		//interrupts();
		return -1;
	}

	/* only one ack_requested from us to a specific address at a time is allowed in the queue */
	//in order not to screw with the awaiting of ACK
	if (frm->ack_requested)
	{
		for (int i = 0; i < fifo.size(); i++)
		{
			//note: this never succeeds for received packets -> tx condition only
			Frame *ffrm = fifo.get(i);
			if (ffrm->ack_requested && ffrm->src == fmac.myAddr && ffrm->dest == frm->dest)
			{
				//interrupts();
				return -2;
			}
		}
	}

	if (frm->type == FRM_TYPE_ACK)
		/* add to front */
		fifo.unshift(frm);
	else
		/* add to tail */
		fifo.add(frm);

	//interrupts();
	return 0;
}

/* remove frame from linked list and delete it */
bool MacFifo::remove_delete(Frame *frm)
{
	bool found = false;

	//noInterrupts();
	for (int i = 0; i < fifo.size() && !found; i++)
		if (frm == fifo.get(i))
		{
			delete fifo.remove(i);
			found = true;
		}
	//interrupts();

	return found;
}

/* remove any pending frame that waits on an ACK from a host */
bool MacFifo::remove_delete_acked_frame(MacAddr dest)
{
	bool found = false;
	//noInterrupts();

	for (int i = 0; i < fifo.size(); i++)
	{
		Frame* frm = fifo.get(i);
		if (frm->ack_requested && frm->dest == dest)
		{
			delete fifo.remove(i);
			found = true;
		}
	}
	//interrupts();
	return found;
}

void coord2payload_absolut(float lat, float lon, uint8_t *buf)
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

int serialize_legacyGroundTracking(ufo_t *Data,uint8_t*& buffer){
  int msgSize = 7; //sizeof(FanetLora::fanet_packet_t1);
  buffer = new uint8_t[msgSize];
  coord2payload_absolut(Data->latitude,Data->longitude, &buffer[0]);
  buffer[6] = 1 << 4; //set mode to walking
	if (!Data->no_track){
		buffer[6] += 1; //set online-tracking
	}
	return 7;
}

uint8_t Flarm2FanetAircraft(eFlarmAircraftType aircraft){
  switch (aircraft)
  {
  case eFlarmAircraftType::PARA_GLIDER :
    return 1; //FanetLora::aircraft_t::paraglider;
  case eFlarmAircraftType::HANG_GLIDER :
    return 2; //FanetLora::aircraft_t::hangglider;
  case eFlarmAircraftType::BALLOON :
    return 3; //FanetLora::aircraft_t::balloon;
  case eFlarmAircraftType::GLIDER_MOTOR_GLIDER :
    return 4; //FanetLora::aircraft_t::glider;
  case eFlarmAircraftType::TOW_PLANE :
    return 5; //FanetLora::aircraft_t::poweredAircraft;
  case eFlarmAircraftType::HELICOPTER_ROTORCRAFT :
    return 6; //FanetLora::aircraft_t::helicopter;
  case eFlarmAircraftType::UAV :
    return 7; //FanetLora::aircraft_t::uav;
  default:
    return 0; //FanetLora::aircraft_t::otherAircraft;
  }
}



int serialize_legacyTracking(ufo_t *Data,uint8_t*& buffer){
  int msgSize = 11; //sizeof(FanetLora::fanet_packet_t1);
  buffer = new uint8_t[msgSize];
  coord2payload_absolut(Data->latitude,Data->longitude, &buffer[0]);

	/* altitude set the lower 12bit */
	int alt = constrain(Data->altitude, 0, 8190);
	if(alt > 2047)
		((uint16_t*)buffer)[3] = ((alt+2)/4) | (1<<11);				//set scale factor
	else
		((uint16_t*)buffer)[3] = alt;
	/* online tracking */
	((uint16_t*)buffer)[3] |= !Data->stealth<<15;
	/* aircraft type */
	//((uint16_t*)buffer)[3] |= (LP_Flarm2FanetAircraft((eFlarmAircraftType)Data->aircraft_type)&0x7)<<12;
	((uint16_t*)buffer)[3] |= (Flarm2FanetAircraft((eFlarmAircraftType)Data->aircraft_type)&0x7)<<12;

	/* Speed */
	int speed2 = constrain((int)roundf(Data->speed *2.0f), 0, 635);
	if(speed2 > 127)
		buffer[8] = ((speed2+2)/5) | (1<<7);					//set scale factor
	else
		buffer[8] = speed2;

	/* Climb */
	int climb10 = constrain((int)roundf(Data->vs *10.0f), -315, 315);
	if(std::abs(climb10) > 63)
		buffer[9] = ((climb10 + (climb10>=0?2:-2))/5) | (1<<7);			//set scale factor
	else
		buffer[9] = climb10 & 0x7F;

	/* Heading */
	buffer[10] = constrain((int)roundf(Data->course *256.0f/360.0f), 0, 255);

	return 11; //FANET_LORA_TYPE1_SIZE - 2;
}


uint8_t FanetMac::getAddressType(uint8_t manuId){
	//  GxAircom            Skytraxx          XCTracer
	if (manuId == 0x08 || manuId == 0x11 || manuId == 0x20 || manuId == 0xDD || manuId == 0xDE || manuId == 0xDF){
			return 2; //address type FLARM
			//log_i("address=%s Flarm",devId.c_str());
	/*
	}else if (manuId == 0x08){
			return 3; //address type OGN        
			//log_i("address=%s OGN",devId.c_str());
	*/
	}else{
			return 0; //address type unknown
	}
}

void FanetMac::sendUdpData(const uint8_t *buffer,int len){
	if ((WiFi.status() == WL_CONNECTED) || (WiFi.softAPgetStationNum() > 0)){ //connected to wifi or a client is connected to me
		//log_i("sending udp");
		WiFiUDP udp;
		udp.beginPacket("192.168.0.178",10110);
		udp.write(buffer,len);
		udp.endPacket();    
	}
}

/* this is executed in a non-linear fashion */
void FanetMac::frameReceived(int length)
{
	/* quickly read registers */
	num_received = length;
	int state = radio.readData(rx_frame, length);
	if (state != ERR_NONE) {
		if (state == ERR_CRC_MISMATCH) {
			//log_e("CRC error!");
		}else{
			log_e("failed, code %d",state);
		}
		return;
	}
	int rssi = radio.getRSSI();
	//log_i("%d rssi=%d",millis(),rssi);
	int snr = 0;	
	snr = rssi + 120;
	if (snr < 0) snr = 0;
	

	/* build frame from stream */
	Frame *frm;
  if (_actMode != MODE_LORA){			
		time_t tUnix;
		time(&tUnix);
		#if RX_DEBUG > 0
			static uint32_t tmax = 0;
			static uint32_t tmin = 1000;
			uint32_t tPPS = millis()-_ppsMillis;
			if (tPPS > 400){
				if (tmin > tPPS) tmin = tPPS;
			}else{
				if (tmax < tPPS) tmax = tPPS;
			}
			char Buffer[500];	
			int len = 0;
      char strftime_buf[64];
      struct tm timeinfo;      
			len += sprintf(Buffer+len,"min=%d;max=%d;%d;T=%d;",tmin,tmax,tPPS,tUnix);
			//len += sprintf(Buffer+len,"%d;",tPPS);
			//len += sprintf(Buffer+len,"T=%d;",tUnix);
			//log_i("l=%d,%s",len,Buffer);
      localtime_r(&tUnix, &timeinfo);
      strftime(strftime_buf, sizeof(strftime_buf), "%F %T", &timeinfo);   
			len += sprintf(Buffer+len,"%s;",strftime_buf);
			//log_i("l=%d,%s",len,Buffer);
			len += sprintf(Buffer+len,"F=%.1f;",fmac.actflarmFreq);
			len += sprintf(Buffer+len,"Rx=%d;rssi=%d;", num_received, rssi);

			for(int i=0; i<num_received; i++)
			{
				len += sprintf(Buffer+len,"%02X", rx_frame[i]);
				if (i >= 26) break;
				//if(i<num_received-1)
				//	len += sprintf(Buffer+len,",");
			}
			len += sprintf(Buffer+len,"\n");
			Serial.print(Buffer);
			//fmac.sendUdpData((uint8_t *)Buffer,len);
			//log_i("%s",Buffer);
			//log_i("l=%d;%s",len,Buffer);
		#endif
    if (length != 26){ //FSK-Frame is fixed 26Bytes long
			#if RX_DEBUG > 0
			log_e("rx size 26!=%d",length);
			#endif
			return;
		}
    // The magic number is a 1-byte unencrypted value at the 4th byte. 
		// high-nibble --> Address-Type
		// ICAO = 1;
		// FLARM = 2;
		// ANONYMOUS = 3;
		// low-nibble --> unknown (must be zero)

		uint8_t magic = rx_frame[3] & 0x0F;
		if (magic != 0x00){
			//the unknown-Bits have to be zero !!
			#if RX_DEBUG > 0
      log_e("Legacy: unknown-Bits have to be zero %d",magic);
			#endif
      return;			
		}
		/*
		magic = rx_frame[3] >> 8;
    if (magic != 0x10 && magic != 0x20 && magic != 0x30) {
			#if RX_DEBUG > 0
      log_e("%d Legacy: wrong message %d!=0x10!=0x20",millis(),magic);
			#endif
      return;			
    }
		*/

    //check if Checksum is OK
  	uint16_t crc16 =  flarm_getCkSum(rx_frame,24);
    uint16_t crc16_2 = (uint16_t(rx_frame[24]) << 8) + uint16_t(rx_frame[25]);
    if (crc16 != crc16_2){
			/*
			time_t tUnix;
      char strftime_buf[64];
      struct tm timeinfo;
      time(&tUnix);
			len += sprintf(Buffer+len,"T=%d;",tUnix);
			//log_i("l=%d,%s",len,Buffer);
      localtime_r(&tUnix, &timeinfo);
      strftime(strftime_buf, sizeof(strftime_buf), "%F %T", &timeinfo);   
			len += sprintf(Buffer+len,"%s;",strftime_buf);
			//log_i("l=%d,%s",len,Buffer);
			if (_actMode == MODE_FSK_8684){
				len += sprintf(Buffer+len,"F=868.4;");
			}else{
				len += sprintf(Buffer+len,"F=868.2;");
			}
			len += sprintf(Buffer+len,"Rx=%d;rssi=%d;", num_received, rssi);

			for(int i=0; i<num_received; i++)
			{
				len += sprintf(Buffer+len,"%02X", rx_frame[i]);
				if (i >= 26) break;
				//if(i<num_received-1)
				//	len += sprintf(Buffer+len,",");
			}
			len += sprintf(Buffer+len,"\n");
			Serial.print(Buffer);
			*/
			//fmac.sendUdpData((uint8_t *)Buffer,len);
			//log_i("%s",Buffer);
			//log_i("l=%d;%s",len,Buffer);
      #if RX_DEBUG > 0
			log_e("%d Legacy: wrong Checksum %04X!=%04X",millis(),crc16,crc16_2);
			#endif
      return;
    }
    ufo_t air={0};
    ufo_t myAircraft={0};
    myAircraft.latitude = lat;
    myAircraft.longitude = lon;
    myAircraft.geoid_separation = geoidAlt;
    myAircraft.timestamp = now();
    //myAircraft.latitude =    
		uint8_t newPacket[26];
		//uint32_t tNow = now();	
		uint32_t tOffset = 0;	
		bool bOk = false;
		int8_t ret = 0;
		for(int i = 0;i < 5; i++){
			memcpy(&newPacket[0],&rx_frame[0],26);
			flarm_decrypt(newPacket,tUnix + tOffset);
			ret = flarm_decode(newPacket,&myAircraft,&air);
			//if (legacy_decode(newPacket,&myAircraft,&air) == 0){
			if (ret == 0){				
				//float dist = distance(myAircraft.latitude,myAircraft.longitude,air.latitude,air.longitude, 'K');      
				//len = sprintf(Buffer,"T=%dadr=%06X;adrType=%d;airType=%d,lat=%.06f,lon=%.06f,alt=%.01f,speed=%.01f,course=%.01f,climb=%.01f\n", millis()-fmac._ppsMillis,air.addr,air.addr_type,air.aircraft_type,air.latitude,air.longitude,air.altitude,air.speed,air.course,air.vs);
				/*
				len = sprintf(Buffer,"T=%dadr=%06X;adrType=%d;airType=%d,lat=%.06f,lon=%.06f,alt=%.01f,speed=%.01f,course=%.01f,vs=%.01f,geoid=%.01f", air.timestamp,air.addr,air.addr_type,air.aircraft_type,air.latitude,air.longitude,air.altitude,air.speed,air.course,air.vs,air.geoid_separation);
				Serial.print(Buffer);
				legacy_packet_t *pkt = (legacy_packet_t *) newPacket;
				len = sprintf(Buffer,",Turnrate=%d\n", pkt->turnrate);
				Serial.print(Buffer);
				*/

				//fmac.sendUdpData((uint8_t *)Buffer,len);
				
				//legacy_packet_t *pkt = (legacy_packet_t *) newPacket;
				//len = sprintf(Buffer,"unk0=%d,unk1=%d,unk2=%d,unk3=%d,\n", pkt->zero0,pkt->zero1,pkt->_unk2,pkt->zero2);
				//fmac.sendUdpData((uint8_t *)Buffer,len);
				//Serial.print(Buffer);

      	//if ((dist <= 100) && (air.addr != 0) && (air.aircraft_type != 0)){
				//if ((air.addr != 0) && (air.aircraft_type != 0)){
				//if ((pkt->zero0 == 0) && (pkt->zero1 == 0) && (pkt->_unk2 == 0) && (pkt->zero2 == 0)){
			  bOk = true;
				break;
			//}else if (ret == -2){
				//unknown message
			//	break;
			}
			//log_i("Legacy-Packet not valid ts=%d;offset=%d",tNow,tOffset);
			if (i == 0){
				tOffset = -1;
			}else if (i == 1){
				tOffset = 1;
			}else if (i == 2){
				tOffset = -2;
			}else if (i == 3){
				tOffset = 2;
			}
		}
		if (bOk){
			frm = new Frame();
			frm->src.manufacturer = uint8_t(air.addr >> 16);
			frm->src.id = uint16_t(air.addr & 0x0000FFFF);
			frm->dest = MacAddr();
		  frm->altitude = air.altitude;
			frm->forward = false;
			if (!air.airborne){
				frm->type = FRM_TYPE_GROUNDTRACKING_LEGACY;
				frm->payload_length = serialize_legacyGroundTracking(&air,frm->payload);
			}else{
				frm->type = FRM_TYPE_TRACKING_LEGACY;
				frm->payload_length = serialize_legacyTracking(&air,frm->payload);
			}			
			frm->AddressType = air.addr_type;
			frm->legacyAircraftType = air.aircraft_type;
			frm->timeStamp = tUnix + tOffset;
			
			rxLegCount++;
		}else{
			//log_e("error decoding legacy");
			#if RX_DEBUG > 0
			log_e("error decoding legacy %d",ret);
			#endif
			return;
		}
  }else{
    frm = new Frame(num_received, rx_frame);
		//time_t now;
		//time(&now);
		//frm->timeStamp = now;
		frm->timeStamp = uint32_t(now());
		//log_i("%d",frm->timeStamp);
		frm->AddressType = getAddressType(frm->src.manufacturer) + 0x80; //set highest Bit, so we know, that it was a Fanet-MSG
		rxFntCount++;
  }  
	frm->rssi = rssi;
	frm->snr = snr;
	if ((frm->src.id == 0) || (frm->src.manufacturer == 0)){
    //log_e("frmId=0");
    delete frm;
    return;
  }
	/* add to fifo */
	if (rx_fifo.add(frm) < 0)
		delete frm;
}

/* wrapper to fit callback into c++ */
void FanetMac::frameRxWrapper(int length)
{
	log_i("received %d",length);
	fmac.frameReceived(length);
}

void FanetMac::end()
{
  // stop LoRa class
  radio.end();
  SPI.end();
  //if (_ss >= 0) digitalWrite(_ss,LOW);
  //if (_reset >= 0) digitalWrite(_reset,HIGH);
}


bool FanetMac::begin(int8_t sck, int8_t miso, int8_t mosi, int8_t ss,int8_t reset, int8_t dio0,int8_t gpio,Fapp &app,long frequency,uint8_t level,uint8_t radioChip)
{
	myApp = &app;
	setup_frequency=frequency;
	_ss = ss;
	_reset = reset;
	_actMode = 0;

	// address 
	_myAddr = readAddr();


	/* configure phy radio */
	//SPI LoRa pins
	log_i("sck=%d,miso=%d,mosi=%d,ss=%d,reset=%d,dio0=%d,gpio=%d",sck,miso,mosi,ss,reset,dio0,gpio);
	SPI.begin(sck, miso, mosi, ss);
	if (radioChip == RADIO_SX1262){
		radio.setPins(&SPI,ss,dio0,reset,gpio);
	}else{
		radio.setPins(&SPI,ss,dio0,reset);
	}
	//radio.setPins(&SPI,ss,dio0,reset,32);
	//radio.setPins(&SPI,ss,dio0,reset);
	radio.gain = 1; //highest gain setting for FSK
  int state = radio.begin(float(frequency) / 1000000.,250.0,7,8,0xF1,int8_t(level),radioChip);
  if (state == ERR_NONE) {
    //log_i("success!");
  } else {
    log_e("failed, code %d",state);
		return 0;
  }
	// start listening for LoRa packets
	/*	
	if (_RfMode.bits.FntRx){
		switchMode(MODE_LORA);
	}else{
		switchMode(MODE_FSK_8682);
	}
	*/
	log_i("LoRa Initialization OK!");


	/* start state machine */
	myTimer.Start();
	randomSeed(millis());
	
	return true;
}

void FanetMac::switchMode(uint8_t mode,bool bStartReceive){
  time_t tUnix = 0;
	uint32_t channel = 0;
	#if RX_DEBUG > 1
	uint32_t tBegin = micros();
	bool bChanged = false;
	#endif
	if (mode == MODE_LORA){
		radio.switchLORA(loraFrequency,loraBandwidth);
		actflarmFreq = 0.0;
	}else if (mode == MODE_FSK_8682){
		actflarmFreq = 868.2;
		radio.switchFSK(actflarmFreq);
	}else if (mode == MODE_FSK_8684){
		actflarmFreq = 868.4;
		radio.switchFSK(actflarmFreq);
	}else if (mode == MODE_FSK){		
		time(&tUnix);
		channel = flarm_calculate_freq_channel(tUnix,(uint32_t)flarmChannels);
		float frequ = 0.0;
		frequ = flarmFrequency + (float(channel * 400) / 1000.0);
		if (actflarmFreq != frequ){
			#if RX_DEBUG > 1
			bChanged = true;
			#endif
			actflarmFreq = frequ;
			radio.switchFSK(actflarmFreq);
		}
	}
	if (bStartReceive) radio.startReceive();	
	#if RX_DEBUG > 1
	if (_actMode != mode){
		bChanged = true;
	}
	#endif
	_actMode = mode;
	#if RX_DEBUG > 1
	if (bChanged){
		char Buffer[500];
		int len = 0;
		len += sprintf(Buffer+len,"%d switch to mode ",millis());
		if (mode == MODE_LORA){
			len += sprintf(Buffer+len,"LORA %0.1fMhz BW=%d",loraFrequency,loraBandwidth);
		}else if (mode == MODE_FSK_8682){
			len += sprintf(Buffer+len,"FSK %0.1fMhz ",actflarmFreq);
		}else if (mode == MODE_FSK_8684){
			len += sprintf(Buffer+len,"FSK %0.1fMhz ",actflarmFreq);
		}else if (mode == MODE_FSK){
			len += sprintf(Buffer+len,"FSK c=%d,f=%0.2f,t=%d",channel,actflarmFreq,tUnix);
			//log_i("channel=%d,frequ=%.2f,time=%d",channel,frequ,tUnix);
		}
		len += sprintf(Buffer+len,"in %dus pps=%d\n",int(micros()-tBegin),int(millis() - _ppsMillis));
		//log_i("%d switch to mode %d in %dus pps=%d",millis(),mode,micros()-tBegin,_ppsMillis);
		Serial.print(Buffer);
	}
	#endif
}

/* wrapper to fit callback into c++ */
void FanetMac::stateWrapper()
{
	//return;
	static uint32_t tSwitch = millis();
	//static uint32_t tMsg = millis();
	static uint8_t ppsCount = 0;
	static uint8_t ppsCountFlarm = 0;
	static uint32_t ppsMillis = 0;
	static uint8_t actPPsDiff = 3; 
	uint32_t tAct = millis();
	fmac.handleIRQ();
	/*
	if ((tAct - tMsg) >= 1000){
		tMsg = tAct;
		log_i("hasGPS=%d",fmac.bHasGPS);
	}
	*/
	uint8_t ppsDiff = fmac._ppsCount - ppsCount;  //we have to calc diff here, because in if, it will be calculated as int
	if (fmac.bHasGPS){
		if (fmac._RfMode.bits.FntRx && fmac._RfMode.bits.LegRx){
			if (fmac._actMode == MODE_LORA){
				if ((ppsDiff) > actPPsDiff){
					if (fmac.flarmZone == 1){
						if ((millis() - fmac._ppsMillis) >= (LEGACY_8682_BEGIN - LEGACY_RANGE)){
							actPPsDiff ++; //we count from 3 to 5 with pps-diff, so we are never in same cylce and will receive more Lora
							if (actPPsDiff > 5) actPPsDiff = 3;
							//log_i("actPPsDiff=%d",actPPsDiff);
							ppsMillis = fmac._ppsMillis;
							ppsCount = fmac._ppsCount;
							fmac.switchMode(MODE_FSK_8682); //start now with FSK868200
							#if RX_DEBUG > 5
							log_i("**** %d start FSK 868.2 %d %d",millis(),millis() - ppsMillis,fmac._ppsCount);
							#endif
						}
					}else if (fmac.flarmZone > 0){
							actPPsDiff ++; //we count from 3 to 5 with pps-diff, so we are never in same cylce and will receive more Lora
							if (actPPsDiff > 5) actPPsDiff = 3;
							//log_i("actPPsDiff=%d",actPPsDiff);
							ppsMillis = fmac._ppsMillis;
							ppsCount = fmac._ppsCount;
							ppsCountFlarm = fmac._ppsCount;
							fmac.switchMode(MODE_FSK); //start now with FSK
					}
				}
			}else if (fmac._actMode == MODE_FSK_8682){
				if ((millis() - ppsMillis) >= (LEGACY_8684_BEGIN - LEGACY_RANGE)){
					fmac.switchMode(MODE_FSK_8684); //start now with FSK868400
					#if RX_DEBUG > 5
					log_i("**** %d start FSK 868.4 %d %d",millis(),millis() - ppsMillis,fmac._ppsCount);
					#endif
				}
			}else if (fmac._actMode == MODE_FSK_8684){
				if (((millis() - ppsMillis) >= (LEGACY_8684_END + LEGACY_RANGE)) && ((ppsDiff) >= 1)){
					fmac.switchMode(MODE_LORA); //Back to Lora again
					#if RX_DEBUG > 5
					log_i("**** %d finisched FSK-Mode %d",millis(),millis() - ppsMillis,fmac._ppsCount);
					#endif
					ppsCount = fmac._ppsCount;
				}
			}else if (fmac._actMode == MODE_FSK){
				if (ppsDiff >= 2){
					fmac.switchMode(MODE_LORA); //Back to Lora again
					ppsCount = fmac._ppsCount;
				}else{
					if (ppsCountFlarm != fmac._ppsCount){
						ppsCountFlarm = fmac._ppsCount;
						fmac.switchMode(MODE_FSK); //start now with FSK //switch to right channel !!
					}
				}
			}
		}else if (!fmac._RfMode.bits.FntRx && fmac._RfMode.bits.LegRx){
			//only Legacy and we have a GPS-Sensor (PPS-Signal)
			if (fmac.flarmZone == 1){ //zone 1: f0=868.2, f1=868.4
				if (fmac._actMode != MODE_FSK_8682){
						if ((millis() - fmac._ppsMillis) >= (LEGACY_8682_BEGIN - LEGACY_RANGE) && ((ppsDiff) >= 1)){
							ppsMillis = fmac._ppsMillis;
							ppsCount = fmac._ppsCount;
							if (fmac._actMode != MODE_FSK_8682){
								fmac.switchMode(MODE_FSK_8682); //start now with FSK8682
							}
							#if RX_DEBUG > 5
							log_i("**** %d start FSK 868.2 %d %d",millis(),millis() - ppsMillis,fmac._ppsCount);
							#endif
						}
				}else{
					if ((millis() - ppsMillis) >= (LEGACY_8684_BEGIN - LEGACY_RANGE)){
						if (fmac._actMode != MODE_FSK_8684){
							fmac.switchMode(MODE_FSK_8684); //start now with FSK8684
						}	
						#if RX_DEBUG > 5
						log_i("**** %d start FSK 868.4 %d %d",millis(),millis() - ppsMillis,fmac._ppsCount);
						#endif
					}
				}				
			}else if (fmac.flarmZone > 0){
				if (ppsCount !=fmac._ppsCount){
					ppsCount =fmac._ppsCount;
					fmac.switchMode(MODE_FSK); //start now with FSK //switch to right channel !!
				}
			}
		}else{

		} 
	}else{
		if (fmac.flarmZone == 1){
			if (fmac._RfMode.bits.FntRx && fmac._RfMode.bits.LegRx){
				//no GPS-Sensor but Flarm and Fanet-RX --> 5.3sec. for Fanet, 2.3sec. Legacy 868.2, 2.3sec. Legacy 868.4
				if (fmac._actMode == MODE_LORA){
					if ((tAct - tSwitch) >= 5300){
						fmac.switchMode(MODE_FSK_8682);
						tSwitch = millis();
					}
				}else if (fmac._actMode == MODE_FSK_8682){
					if ((tAct - tSwitch) >= 2300){					
						fmac.switchMode(MODE_FSK_8684);
						tSwitch = millis();
					}
				}else{
					if ((tAct - tSwitch) >= 2300){					
						fmac.switchMode(MODE_LORA);
						tSwitch = millis();
					}
				}
			}else if (!fmac._RfMode.bits.FntRx && fmac._RfMode.bits.LegRx){			
				//only Legacy-Receive --> 1sec. 868.2 1sec. 868.4
				//only in zone 1
				if ((tAct - tSwitch) >= 1000){ //switch every 1sec.
					if (fmac._actMode != MODE_FSK_8682){
						fmac.switchMode(MODE_FSK_8682);
					}else{
						fmac.switchMode(MODE_FSK_8684);
					}
					tSwitch = tAct;
				}
			}
		}
	}
  fmac.handleRx();
	if ((fmac._RfMode.bits.FntTx) && (!fmac._RfMode.bits.FntRx) && (fmac.eLoraRegion != NONE)){
		if (millis() >= fmac.csma_next_tx){
			if ((fmac.tx_fifo.size() > 0) || (fmac.myApp->is_broadcast_ready(fmac.neighbors.size()))){
				uint8_t oldMode = fmac._actMode;
				if (fmac._actMode != MODE_LORA){
					fmac.switchMode(MODE_LORA); //switch to Lora to send Messages
				}								
				#if RX_DEBUG > 8
				log_i("******************* %d handle FanetTx *****************",millis());
				#endif
				fmac.handleTx();
				if (oldMode != MODE_LORA){
					fmac.switchMode(oldMode); //switch back to old mode
				}				
			}
		}
	}
	if ((fmac._actMode == MODE_LORA) && (fmac._RfMode.bits.FntTx) && (fmac.eLoraRegion != NONE)){  	
    fmac.handleTx();
  }
	fmac.handleTxLegacy();	
}

bool FanetMac::isNeighbor(MacAddr addr)
{
	for (int i = 0; i < neighbors.size(); i++)
		if (neighbors.get(i)->addr == addr)
			return true;

	return false;
}

/*
 * Generates ACK frame
 */
void FanetMac::ack(Frame* frm)
{
#if MAC_debug_mode > 0
	Serial.printf("### generating ACK\n");
#endif

	/* generate reply */
	Frame *ack = new Frame(myAddr);
	ack->type = FRM_TYPE_ACK;
	ack->dest = frm->src;

	/* only do a 2 hop ACK in case it was requested and we received it via a two hop link (= forward bit is not set anymore) */
	if (frm->ack_requested == FRM_ACK_TWOHOP && !frm->forward)
		ack->forward = true;

	/* add to front of fifo */
	//note: this will not fail by define
	if (tx_fifo.add(ack) != 0)
		delete ack;
}

/*
 * Processes irq
 */
void FanetMac::handleIRQ(){
	// check if the flag is set
	if (radio.isRxMessage()) {	
		
		int16_t packetSize = radio.getPacketLength();
		#if RX_DEBUG > 1
		//log_i("new package arrived %d",packetSize);
		#endif
		if (packetSize > 0){
			//log_i("packet receive %d",packetSize);			
			fmac.frameReceived(packetSize);
			
		}
		radio.startReceive();
	}
	return;
}

/*
 * Processes stuff from rx_fifo
 */
void FanetMac::handleRx()
{
	/* nothing to do */
	if (rx_fifo.size() == 0)
	{
		/* clean neighbors list */
		for (int i = 0; i < neighbors.size(); i++)
		{
			if (neighbors.get(i)->isAround() == false)
				delete neighbors.remove(i);
		}

		return;
	}

	Frame *frm = rx_fifo.front();
	if(frm == nullptr)
		return;
	/* build up neighbors list */
	bool neighbor_known = false;
	for (int i = 0; i < neighbors.size(); i++)
	{
		if (neighbors.get(i)->addr == frm->src)
		{
			/* update presents */
			neighbors.get(i)->seen();
			if(frm->type == FRM_TYPE_TRACKING || frm->type == FRM_TYPE_GROUNDTRACKING)
				neighbors.get(i)->hasTracking = true;
			neighbor_known = true;
			break;
		}
	}
  //log_i("src=%06X,dest=%06X,type=%d",frm->src,frm->dest,frm->type);
	/* neighbor unknown until now, add to list */
	if (neighbor_known == false)
	{
		/* too many neighbors, delete oldest member */
		if (neighbors.size() > MAC_NEIGHBOR_SIZE)
			delete neighbors.shift();

		neighbors.add(new NeighborNode(frm->src, frm->type == FRM_TYPE_TRACKING || frm->type == FRM_TYPE_GROUNDTRACKING));
	}
  if (frm->type == FRM_TYPE_TRACKING_LEGACY) frm->type = FRM_TYPE_TRACKING; //if we have a Legacy-Tracking, we don't count it in neighbors, so switch back tracking-type

	if (frm->type == FRM_TYPE_GROUNDTRACKING_LEGACY) frm->type = FRM_TYPE_GROUNDTRACKING; //if we have a Legacy-GroundTracking, we don't count it in neighbors, so switch back tracking-type

	//if (frm->type == FRM_TYPE_GROUNDTRACKING_LEGACY) frm->type = FRM_TYPE_GROUNDTRACKING; //if we have a Legacy-Tracking, we don't count it in neighbors, so switch back tracking-type


	/* is the frame a forwarded one and is it still in the tx queue? */
	Frame *frm_list = tx_fifo.frame_in_list(frm);
	if (frm_list != NULL)
	{
		/* frame already in tx queue */

		if (frm->rssi > frm_list->rssi + MAC_FORWARD_MIN_DB_BOOST)
		{
			/* somebody broadcasted it already towards our direction */
#if MAC_debug_mode > 0
			Serial.printf("### rx frame better than org. dropping both.\n");
#endif
			/* received frame is at least 20dB better than the original -> no need to rebroadcast */
			tx_fifo.remove_delete(frm_list);
		}
		else
		{
#if MAC_debug_mode >= 2
			Serial.printf("### adjusting tx time");
#endif
			/* adjusting new departure time */
			frm_list->next_tx = millis() + random(MAC_FORWARD_DELAY_MIN, MAC_FORWARD_DELAY_MAX);
		}
	}
	else
	{
		if ((frm->dest == MacAddr() || frm->dest == myAddr) && frm->src != myAddr)
		{
			/* a relevant frame */
			if (frm->type == FRM_TYPE_ACK)
			{
				if (tx_fifo.remove_delete_acked_frame(frm->src) && myApp != NULL)
					myApp->handle_acked(true, frm->src);
			}
			else
			{
				/* generate ACK */
				if (frm->ack_requested)
					ack(frm);

				/* forward frame */
				if (myApp != NULL)
					myApp->handle_frame(frm);
			}
		}

		/* Forward frame */
		if (doForward && frm->forward && tx_fifo.size() < MAC_FIFO_SIZE - 3 && frm->rssi <= MAC_FORWARD_MAX_RSSI_DBM
				&& (frm->dest == MacAddr() || isNeighbor(frm->dest)) && radio.get_airlimit() < 0.5f)
		{
#if MAC_debug_mode >= 2
			Serial.printf("### adding new forward frame\n");
#endif
			/* prevent from re-forwarding */
			frm->forward = false;

			/* generate new tx time */
			frm->next_tx = millis() + random(MAC_FORWARD_DELAY_MIN, MAC_FORWARD_DELAY_MAX);
			frm->num_tx = !!frm->ack_requested;

			/* add to list */
			tx_fifo.add(frm);
			return;
		}
	}

	/* discard frame */
	delete frm;
}



void dumpBuffer(uint8_t * data, int len,Stream& out)
{
  
   int regnum=0;
   do {
     for (int i = 0; i < 16; i++) {
        
        uint8_t reg = data[regnum++];
        if (reg < 16) {out.print("0");}
        out.print(reg,HEX);
        out.print(" ");
     }
    out.println();
   } while (regnum <len);
}


void FanetMac::setRfMode(uint8_t mode){
    _RfMode.mode = mode;
}

void FanetMac::setRegion(float lat, float lon){
	bool bstartRec = false;
	//find out LoRa regions for Fanet
	if (eLoraRegion == NONE){
		if (-169.0f < lon && lon < -30.0f){
			eLoraRegion = US920;
			loraFrequency = 920.8;
			loraBandwidth = 500;
			log_i("set Lora-Region to US920");
		}	else if (110.0f < lon && lon < 179.0f && -48.0f < lat && lat < 10.0f){
			eLoraRegion = AU920;
			loraFrequency = 920.8;
			loraBandwidth = 500;
			log_i("set Lora-Region to AU920");
		} else if (69.0f < lon && lon < 89.0f && 5.0f < lat && lat < 40.0f){
			eLoraRegion = IN866;
			loraFrequency = 866.2;
			loraBandwidth = 250;
			log_i("set Lora-Region to IN866");
		} else if (124.0f < lon && lon < 130.0f && 34.0f < lat && lat < 39.0f){
			eLoraRegion = KR923;
			loraFrequency = 923.2;
			loraBandwidth = 125;
			log_i("set Lora-Region to KR923");
		} else if (89.0f < lon && lon < 146.0f && 21.0f < lat && lat < 47.0f){
			eLoraRegion = AS920;
			loraFrequency = 923.2;
			loraBandwidth = 125;
			log_i("set Lora-Region to AS920");
		} else if (34.0f < lon && lon < 36.0f && 29.0f < lat && lat < 34.0f){
			eLoraRegion = IL918;
			loraFrequency = 918.5;
			loraBandwidth = 125;
			log_i("set Lora-Region to IL918");
		} else{
			eLoraRegion = EU868;
			log_i("set Lora-Region to EU868");
			loraFrequency = 868.2;
			loraBandwidth = 250;
		} 
		bstartRec = true;
	}
	if (flarmZone == 0){
		flarmZone = flarm_get_zone(lat,lon);		
		flarm_getFrequencyChannels(flarmZone,&flarmFrequency,&flarmChannels);
		log_i("zone=%d,freq=%.1f,channels=%d",flarmZone,flarmFrequency,flarmChannels);		
		bstartRec = true;
	}
	if (bstartRec){
		if (_RfMode.bits.FntRx){
			switchMode(MODE_LORA);
		}else{
			switchMode(MODE_FSK_8682);
		}
	}
}

void FanetMac::setPps(uint32_t *ppsMillis){
	_ppsMillis = *ppsMillis;
	_ppsCount++;
	//log_i("%d ppsCount=%d",millis(),_ppsCount);
}

void FanetMac::handleTxLegacy()
{
	static uint32_t tMillis = 0;
	static bool bSend8682 = false;
	static uint8_t ppsCount = 0;
	if ((!_RfMode.bits.LegTx) || (flarmZone < 1)){
		return; //Flarm disabled or no active zone
	} 
	if (!fmac.bHasGPS){
		return; //no Flarm when no GPS
	}
	if (legacy_next_tx == 0){
		if (flarmZone == 1){
			if (_RfMode.bits.FntTx){
				//send only on 868.4 if Fanet is enabled
				if (((millis() - _ppsMillis) >= LEGACY_8684_BEGIN-LEGACY_RANGE) && (ppsCount != _ppsCount)){
					tMillis = _ppsMillis;		
					ppsCount = _ppsCount;		
					if (myApp->createLegacy(LegacyBuffer)){
						//send flarm 868.4 Mhz
						legacy_next_tx = tMillis + uint32_t(random(LEGACY_8684_BEGIN,LEGACY_8684_END - LEGACY_SEND_TIME));
						bSend8682 = false;
					}
				}
			}else{
				//send on 868.2 and 868.4 (PowerFlarm)
				if (((millis() - _ppsMillis) >= LEGACY_8682_BEGIN-LEGACY_RANGE) && (ppsCount != _ppsCount)){
					tMillis = _ppsMillis;	
					ppsCount = _ppsCount;			
					if (myApp->createLegacy(LegacyBuffer)){
						//send flarm 868.2 Mhz
						legacy_next_tx = tMillis + uint32_t(random(LEGACY_8682_BEGIN,LEGACY_8682_END - LEGACY_SEND_TIME));
						bSend8682 = true;
					}
				}
			}
		}else{ //not in Flarm-zone 1
			if (((millis() - _ppsMillis) >= 200) && (ppsCount != _ppsCount)){
				tMillis = _ppsMillis;		
				ppsCount = _ppsCount;		
				if (myApp->createLegacy(LegacyBuffer)){
					legacy_next_tx = tMillis + uint32_t(random(200,800));
					bSend8682 = false;
				}
			}
		}
		/*
		if (((millis() - _ppsMillis) <= LEGACY_8684_BEGIN-100) && (_RfMode.bits.FntTx)){
			tMillis = _ppsMillis;				
			if (myApp->createLegacy(LegacyBuffer)){
				//send Legacy 868.4 Mhz
				legacy_next_tx = tMillis + uint32_t(random(LEGACY_8684_BEGIN,LEGACY_8684_END));
				bSend8682 = false;
			}
		}
		else if ((millis() - _ppsMillis) <= LEGACY_8682_BEGIN-100){
			tMillis = _ppsMillis;		
			if (!_RfMode.bits.FntTx){ 		
				if (myApp->createLegacy(LegacyBuffer)){
					//send Legacy 868.2 Mhz
					legacy_next_tx = tMillis + uint32_t(random(LEGACY_8682_BEGIN,LEGACY_8682_END));
					bSend8682 = true;
					//log_i("calc legNextTx=%d,pps=%d 868.2=%d",legacy_next_tx,tMillis,bSend8682);
				}else{
					log_e("error creating legacy-packet");
				}
			}			
		}
		*/
	}else if (millis() >= legacy_next_tx){		
		#if TX_DEBUG > 0
			uint32_t tStart = micros();
		#endif
		
		uint8_t oldMode = _actMode;
		if (bSend8682){
			//log_i("sending legacy 868.2 %d",millis() - tMillis);
			if (_actMode != MODE_FSK_8682){				
				fmac.switchMode(MODE_FSK_8682,false);
			}
		}else{
			//log_i("sending legacy 868.4 %d",millis() - tMillis);
			if (flarmZone == 1){
				if (_actMode != MODE_FSK_8684){				
					fmac.switchMode(MODE_FSK_8684,false);
				}
			}else{
				fmac.switchMode(MODE_FSK,false);
			}
		}
		//radio.isReceiving(); //check radio is receiving
		int16_t state = radio.transmit(LegacyBuffer, sizeof(LegacyBuffer));
		if (state != ERR_NONE){
			log_e("error TX state=%d",state);
		}else{
			txLegCount++;
		}
		//log_i("%d sending Legacy %d",millis(),fmac._ppsCount);
		if (_actMode != oldMode){
			fmac.switchMode(oldMode,false); //switch back to old mode and receive
		}
		radio.startReceive();		
		#if TX_DEBUG > 0
		Serial.printf("ppsCount=%d,leg_Tx=%d\n",ppsCount,micros()-tStart);
		#endif
		if (bSend8682){
			//we have sent on 868.2 --> calc send-time on 868.4
		  //send also on 8684
			legacy_next_tx = tMillis + uint32_t(random(LEGACY_8684_BEGIN,LEGACY_8684_END - LEGACY_SEND_TIME));
			bSend8682 = false;
			//log_i("calc legNextTx=%d,pps=%d 868.2=%d",legacy_next_tx,tMillis,bSend8682);
		}else{
			legacy_next_tx = 0; //legacy sent!!
		}
		
	}
}

/*
 * get a from from tx_fifo (or the app layer) and transmit it
 */
void FanetMac::handleTx()
{
	
	/* still in backoff or chip turned off*/
	//if (millis() < csma_next_tx  || !LoRa.isArmed())
	if (millis() < csma_next_tx)
		return;

	/* find next send-able packet */
	/* this breaks the layering. however, this approach is much more efficient as the app layer now has a much higher priority */
	Frame* frm;
	bool app_tx = false;
	if (myApp->is_broadcast_ready(neighbors.size()))
	{
		/* the app wants to broadcast the glider state */
		frm = myApp->get_frame();
		if (frm == NULL)
			return;

		if (neighbors.size() <= MAC_MAXNEIGHBORS_4_TRACKING_2HOP)
			frm->forward = true;
		else
			frm->forward = false;

		app_tx = true;
	}
	else if(radio.get_airlimit() < 0.9f)
	{
#if MAC_debug_mode >= 2
		static int queue_length = 0;
		int current_qlen = tx_fifo.size();
		if(current_qlen != queue_length)
		{
			Serial.printf("### tx queue: %d\n", current_qlen);
			queue_length = current_qlen;
		}
#endif

		/* get a frame from the fifo */
		frm = tx_fifo.get_nexttx();
		if (frm == nullptr)
			return;
		/* frame w/o a received ack and no more re-transmissions left */
		if (frm->ack_requested && frm->num_tx <= 0)
		{
#if MAC_debug_mode > 0
			Serial.printf("### Frame, 0x%02X NACK!\n", frm->type);
#endif
			if (myApp != nullptr && frm->src == myAddr)
				myApp->handle_acked(false, frm->dest);
			tx_fifo.remove_delete(frm);
			return;
		}

		/* unicast frame w/o forwarding and it is not a direct neighbor */
		if (frm->forward == false && frm->dest != MacAddr() && isNeighbor(frm->dest) == false)
			frm->forward = true;

		app_tx = false;
	}
	else
	{
		return;
	}

	/* serialize frame */
	uint8_t* buffer;
	int blength = frm->serialize(buffer);
	if (blength < 0)
	{
#if MAC_debug_mode > 0
		Serial.printf("### Problem serialization type 0x%02X. removing.\n", frm->type);
#endif
		/* problem while assembling the frame */
		if (app_tx)
			delete frm;
		else
			tx_fifo.remove_delete(frm);
		return;
	}

#if MAC_debug_mode > 0
	Serial.printf("### Sending, 0x%02X... ", frm->type);
#endif

#if MAC_debug_mode >= 4
	/* print hole packet */
	Serial.printf(" ");
	for(int i=0; i<blength; i++)
	{
		Serial.printf("%02X", buffer[i]);
		if(i<blength-1)
			Serial.printf(":");
	}
	Serial.printf("\n");
#endif

	/* channel free and transmit? */
	//note: for only a few nodes around, increase the coding rate to ensure a more robust transmission
	if (neighbors.size() < MAC_CODING48_THRESHOLD){
		radio.setCodingRate(8);
	}else{
		radio.setCodingRate(5);
	}	
	//log_i("sending Frame");
	int16_t state = radio.transmit(buffer, blength);
	//int tx_ret = LoRa.sendFrame(buffer, blength, neighbors.size() < MAC_CODING48_THRESHOLD ? 8 : 5);
	int tx_ret=TX_OK;
	if (state != ERR_NONE){
		if (state == ERR_TX_TX_ONGOING){
			tx_ret = TX_RX_ONGOING;
			//log_e("error TX_RX_ONGOING");
		}else if (state == ERR_TX_RX_ONGOING){
			tx_ret = TX_RX_ONGOING;
			//log_e("error TX_RX_ONGOING");
		}else if (state == ERR_TX_FSK_ONGOING){
			tx_ret = TX_FSK_ONGOING;
			//log_e("error TX_FSK_ONGOING");
		}else{
			log_e("error TX state=%d",state);
			tx_ret = TX_ERROR;
		} 
	}else{
		//log_i("TX OK");
		txFntCount++;
	}	
	state = radio.startReceive();
	if (state != ERR_NONE) {
			log_e("startReceive failed, code %d",state);
	}
	delete[] buffer;

	if (tx_ret == TX_OK){
#if MAC_debug_mode > 0
		Serial.printf("done.\n");
#endif

		if (app_tx)
		{
			/* app tx */
			myApp->broadcast_successful(frm->type);
			delete frm;
		}
		else
		{
			/* fifo tx */

			/* transmission successful */
			if (!frm->ack_requested || frm->src != myAddr)
			{
				/* remove frame from FIFO */
				tx_fifo.remove_delete(frm);
			}
			else
			{
				/* update next transmission */
				if (--frm->num_tx > 0)
					frm->next_tx = millis() + (MAC_TX_RETRANSMISSION_TIME * (MAC_TX_RETRANSMISSION_RETRYS - frm->num_tx));
				else
					frm->next_tx = millis() + MAC_TX_ACKTIMEOUT;
			}
		}

		/* ready for a new transmission in */
		csma_backoff_exp = MAC_TX_BACKOFF_EXP_MIN;
		csma_next_tx = millis() + MAC_TX_MINPREAMBLEHEADERTIME_MS + (blength * MAC_TX_TIMEPERBYTE_MS);
	}
	else if (tx_ret == TX_RX_ONGOING || tx_ret == TX_TX_ONGOING)
	{
#if MAC_debug_mode > 0
		if(tx_ret == TX_RX_ONGOING)
			Serial.printf("rx, abort.\n");
		else
			Serial.printf("tx not done yet, abort.\n");
#endif

		if (app_tx)
			delete frm;

		/* channel busy, increment backoff exp */
		if (csma_backoff_exp < MAC_TX_BACKOFF_EXP_MAX)
			csma_backoff_exp++;

		/* next tx try */
		csma_next_tx = millis() + random(1 << (MAC_TX_BACKOFF_EXP_MIN - 1), 1 << csma_backoff_exp);

#if MAC_debug_mode > 1
		Serial.printf("### backoff %lums\n", csma_next_tx - millis());
#endif
	}
	else
	{
		/* ignoring TX_TX_ONGOING */
#if MAC_debug_mode > 2
		Serial.printf("## WAT: %d\n", tx_ret);
#endif

		if (app_tx)
			delete frm;
	}
}

uint16_t FanetMac::numTrackingNeighbors(void)
{
	uint16_t num = 0;
	for (uint16_t i = 0; i < neighbors.size(); i++)
		if (neighbors.get(i)->hasTracking)
			num++;

	return num;
}

MacAddr FanetMac::readAddr(bool getHwAddr)
{
	// If the address is already set, return it
	if ((_myAddr.id != 0 || _myAddr.manufacturer != 0) && !getHwAddr) {
		return _myAddr;
	}
	uint64_t chipmacid = ESP.getEfuseMac();
  log_i("ESP32ChipID=%04X%08X",(uint16_t)(chipmacid>>32),(uint32_t)chipmacid);//print chip-id
	//Serial.printf("MAC:");Serial.println(uint64ToString(chipmacid)); // six octets
	uint8_t myDevId[3];
	myDevId[0] = ManuId;//Manufacturer GetroniX
	myDevId[1] = uint8_t(chipmacid >> 32); //last 2 Bytes of MAC
	myDevId[2] = uint8_t(chipmacid >> 40);
    log_i("dev_id=%02X%02X%02X",myDevId[0],myDevId[1],myDevId[2]);
	return MacAddr(myDevId[0],((uint32_t)myDevId[1] << 8) | (uint32_t)myDevId[2]);	
	//return MacAddr();	
}

bool FanetMac::setAddr(MacAddr addr)
{
	/*
	 test for clean storage 
	if(*(__IO uint64_t*)MAC_ADDR_BASE != UINT64_MAX)
		return false;

	// build config
	uint64_t addr_container = MAC_ADDR_MAGIC | (addr.manufacturer&0xFF)<<16 | (addr.id&0xFFFF);

	HAL_FLASH_Unlock();
	HAL_StatusTypeDef flash_ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, MAC_ADDR_BASE, addr_container);
	HAL_FLASH_Lock();

	if(flash_ret == HAL_OK)
		_myAddr = addr;

	return (flash_ret == HAL_OK);
	*/
	return false;
}

bool FanetMac::setAddr(uint32_t addr)
{
	_myAddr = MacAddr((addr >> 16) & 0xff, addr & 0xffff);
	return true;
}

FanetMac fmac = FanetMac();
