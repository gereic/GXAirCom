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
#include "Legacy/Legacy.h"
#include "CRC/lib_crc.h"

int serialize_legacyTracking(ufo_t *Data,uint8_t*& buffer);

/* get next frame which can be sent out */
//todo: this is potentially dangerous, as frm may be deleted in another place.
Frame* MacFifo::get_nexttx()
{
	int next;
	noInterrupts();
	for (next = 0; next < fifo.size(); next++)
		if (fifo.get(next)->next_tx < millis())
			break;
	Frame *frm;
	if (next == fifo.size())
		frm = NULL;
	else
		frm = fifo.get(next);
	interrupts();
	return frm;
}

Frame* MacFifo::frame_in_list(Frame *frm)
{
	noInterrupts();

	for (int i = 0; i < fifo.size(); i++)
	{
		Frame *frm_list = fifo.get(i);
		if (*frm_list == *frm)
		{
			interrupts();
			return frm_list;
		}
	}

	interrupts();

	return NULL;
}

Frame* MacFifo::front()
{
	noInterrupts();
	Frame *frm = fifo.shift();
	interrupts();

	return frm;
}

/* add frame to fifo */
int MacFifo::add(Frame *frm)
{
	noInterrupts();

	/* buffer full */
	/* note: ACKs will always fit */
	if (fifo.size() >= MAC_FIFO_SIZE && frm->type != FRM_TYPE_ACK)
	{
		interrupts();
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
				interrupts();
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

	interrupts();
	return 0;
}

/* remove frame from linked list and delete it */
bool MacFifo::remove_delete(Frame *frm)
{
	bool found = false;

	noInterrupts();
	for (int i = 0; i < fifo.size() && !found; i++)
		if (frm == fifo.get(i))
		{
			delete fifo.remove(i);
			found = true;
		}
	interrupts();

	return found;
}

/* remove any pending frame that waits on an ACK from a host */
bool MacFifo::remove_delete_acked_frame(MacAddr dest)
{
	bool found = false;
	noInterrupts();

	for (int i = 0; i < fifo.size(); i++)
	{
		Frame* frm = fifo.get(i);
		if (frm->ack_requested && frm->dest == dest)
		{
			delete fifo.remove(i);
			found = true;
		}
	}
	interrupts();
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



int serialize_legacyTracking(ufo_t *Data,uint8_t*& buffer){
  int msgSize = sizeof(FanetLora::fanet_packet_t1);
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
	((uint16_t*)buffer)[3] |= (LP_Flarm2FanetAircraft((eFlarmAircraftType)Data->aircraft_type)&0x7)<<12;

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

	return FANET_LORA_TYPE1_SIZE - 2;
}


uint8_t FanetMac::getAddressType(uint8_t manuId){
	if (manuId == 0x11 || manuId == 0x20 || manuId == 0xDD || manuId == 0xDE || manuId == 0xDF){
			return 2; //address type FLARM
			//log_i("address=%s Flarm",devId.c_str());
	}else{
			return 3; //address type OGN        
			//log_i("address=%s OGN",devId.c_str());
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
	int snr = radio.getSNR();

	/* build frame from stream */
	Frame *frm;
  if (_fskMode){
    if (length != 26){ //FSK-Frame is fixed 26Bytes long
			return;
		}
		frm = new Frame();
		#if 0
			Serial.printf("### Mac Rx:%d @ %d ", num_received, rssi);

			for(int i=0; i<num_received; i++)
			{
				Serial.printf("%02X", rx_frame[i]);
				if(i<num_received-1)
					Serial.printf(":");
			}
			Serial.printf("\n");
		#endif

    //invertba(rx_frame,26); //invert complete Frame


    //check if Checksum is OK
  	uint16_t crc16 =  getLegacyCkSum(rx_frame,24);
    uint16_t crc16_2 = (uint16_t(rx_frame[24]) << 8) + uint16_t(rx_frame[25]);
    if (crc16 != crc16_2){
      log_e("wrong Checksum %04X!=%04X",crc16,crc16_2);
      delete frm;
      return;
    }
    ufo_t air={0};
    ufo_t myAircraft={0};
    myAircraft.latitude = lat;
    myAircraft.longitude = lon;
    myAircraft.geoid_separation = geoidAlt;
    myAircraft.timestamp = now();
    //myAircraft.latitude =     
    decrypt_legacy(rx_frame,now());
    if (legacy_decode(rx_frame,&myAircraft,&air)){
      //legacyLogAircraft(&air);
      //float dist = distance(myAircraft.latitude,myAircraft.longitude,air.latitude,air.longitude, 'K');      
      //if (dist <= 100.0){
        //only, if dist <= 100km        
        frm->src.manufacturer = uint8_t(air.addr >> 16);
        frm->src.id = uint16_t(air.addr & 0x0000FFFF);
        frm->dest = MacAddr();
        frm->forward = false;
        frm->type = FRM_TYPE_TRACKING_LEGACY;
				frm->AddressType = air.addr_type;
        frm->payload_length = serialize_legacyTracking(&air,frm->payload);
        //log_i("src=%02X%04X,dest=%02X%04X,type=%d",frm->src.manufacturer,frm->src.id,frm->dest.manufacturer,frm->dest.id,frm->type);
      //}else{
      //  delete frm;
      //  return;
      //}
    }else{
      delete frm;
      return;
    }
  }else{
    frm = new Frame(num_received, rx_frame);
		frm->AddressType = getAddressType(frm->src.manufacturer) + 0x80; //set highest Bit, so we know, that it was a Fanet-MSG
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
  //LoRa.end();
  SPI.end();
  //if (_ss >= 0) digitalWrite(_ss,LOW);
  //if (_reset >= 0) digitalWrite(_reset,HIGH);
}


bool FanetMac::begin(int8_t sck, int8_t miso, int8_t mosi, int8_t ss,int reset, int dio0,Fapp &app,long frequency,uint8_t level,uint8_t radioChip)
{
	myApp = &app;
	setup_frequency=frequency;
	_ss = ss;
	_reset = reset;
	_fskMode = false;

	// address 
	_myAddr = readAddr();


	/* configure phy radio */
	//SPI LoRa pins
	log_i("sck=%d,miso=%d,mosi=%d,ss=%d,reset=%d,dio0=%d",sck,miso,mosi,ss,reset,dio0);
	SPI.begin(sck, miso, mosi, ss);
	
	//setup LoRa transceiver module
	//LoRa.setPins(ss, reset, dio0);
	//LoRa.setPins(18, 14, 26);
	//long frequency = FREQUENCY868;
	//if (setting.band == BAND915)frequency = FREQUENCY915; 
	log_i("Start Lora Frequency=%d",frequency);
  //pModule = new Module(ss,33,reset,32);
	//LoRaClass radio = new LoRaClass(&SPI,ss,33,reset,32);
	//radio.setPins(&SPI,ss,33,reset,32);
	
	//_fskMode = true;
	if (radioChip == RADIO_SX1262){
		radio.setPins(&SPI,ss,dio0,reset,32);
	}else{
		radio.setPins(&SPI,ss,dio0,reset);
	}
	//radio.setPins(&SPI,ss,dio0,reset,32);
	//radio.setPins(&SPI,ss,dio0,reset);
  int state = radio.begin(float(frequency) / 1000000.,250.0,7,8,0xF1,int8_t(level),radioChip);
  if (state == ERR_NONE) {
    log_i("success!");
  } else {
    log_i("failed, code %d",state);
		return 0;
  }
	// start listening for LoRa packets
	state = radio.startReceive();
	if (state != ERR_NONE) {
			log_i("startReceive failed, code %d",state);
			return 0;
	}	
	log_i("LoRa Initialization OK!");


	/* start state machine */
	myTimer.Start();
	randomSeed(millis());
	
	return true;
}

void FanetMac::switchLora(){
	uint32_t tBegin = millis();
	radio.switchLORA();
	radio.startReceive();
  _fskMode = false;
  //log_i("LORA-Mode On %d",millis()-tBegin);
}

void FanetMac::switchFSK(){

  uint32_t tBegin = millis();
	radio.switchFSK();
	if (bSendLegacy) sendLegacy(); //we send the legacy-msg, and switch then to receive
	radio.startReceive();
  _fskMode = true;
  //log_i("FSK-Mode On %d",millis()-tBegin);
}



/* wrapper to fit callback into c++ */
void FanetMac::stateWrapper()
{
	//return;
	static uint32_t tSwitch = millis();
	static uint32_t tNewTime = 3000 + random(MAC_FORWARD_DELAY_MIN, MAC_FORWARD_DELAY_MAX);
	uint32_t tAct = millis();
	fmac.handleIRQ();
	if (fmac._enableLegacyTx == 2){
		if (fmac._fskMode){
			if ((tAct - tSwitch) >= tNewTime){ // 1.9sec in FSK-Mode
				fmac.switchLora();
				tSwitch = tAct;
				tNewTime = 5500 + random(MAC_FORWARD_DELAY_MIN, MAC_FORWARD_DELAY_MAX);
			}
		}else{
			if ((tAct - tSwitch) >= tNewTime){ // 3sec. in FANET-Mode
				fmac.switchFSK();
				tSwitch = tAct;
				tNewTime = 2200 + random(MAC_FORWARD_DELAY_MIN, MAC_FORWARD_DELAY_MAX);
			}
		}
	}
  fmac.handleRx();
	if (!fmac._fskMode){  	
    fmac.handleTx();
  }
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
		//log_i("new package arrived %d",packetSize);
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


void FanetMac::setLegacy(uint8_t enableTx){
    _enableLegacyTx = enableTx;
}

void FanetMac::sendLegacy(){
	//LoRa.dumpRegisters(Serial);
	//log_i("sending legacy");
	
	encrypt_legacy(Legacy_Buffer,now());
	
	uint16_t crc16 =  getLegacyCkSum(Legacy_Buffer,24);

	byte byteArr[26];
	for (int i=0;i<24;i++)
		byteArr[i]=Legacy_Buffer[i];
	
	byteArr[24]=(crc16 >>8);
  	byteArr[25]=crc16;

/*
  ufo_t air={0};
  ufo_t myAircraft={0};
  myAircraft.latitude = lat;
  myAircraft.longitude = lon;
  myAircraft.timestamp = now();
  //myAircraft.latitude = 
  
  byte byteArr2[26];
	for (int i=0;i<26;i++)
		byteArr2[i]=byteArr[i];
  decrypt_legacy(byteArr2,now());
  legacy_decode(byteArr2,&myAircraft,&air);
  //decodeFrame(byteArr2,26,&air);
  legacyLogAircraft(&air);
*/  
	
	int16_t state = radio.transmit(byteArr, 26);
	if (state != ERR_NONE){
		log_e("error TX state=%d",state);
	}	
	log_i("sending Legacy");
	bSendLegacy = false; //legacy sent !!
}

void FanetMac::handleTxLegacy()
{
	//switchFSK();
	radio.switchFSK();
	sendLegacy();
	radio.switchLORA();
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
	int16_t state = radio.transmit(buffer, blength);
	//int tx_ret = LoRa.sendFrame(buffer, blength, neighbors.size() < MAC_CODING48_THRESHOLD ? 8 : 5);
	int tx_ret=TX_OK;
	if (state != ERR_NONE){
		log_e("error TX state=%d",state);
		tx_ret = TX_ERROR;
	}else{
		//log_i("TX OK");
	}	
	state = radio.startReceive();
	if (state != ERR_NONE) {
			log_e("startReceive failed, code %d",state);
	}
	//enableInterrupt = true;	

	bool bSend = false;
	if ((buffer[0]&0x1f)==1){
		//my tracking-data
		if (_enableLegacyTx == 1){
			bSend = true;
		}else if (_enableLegacyTx == 2){
			bSendLegacy = true; //send legacy, when switching to legacy !! (not now)
		}
	}
	//if ((_enableLegacyTx == 1) && ((buffer[0]&0x1f)==1)) bSend = true;	
	delete[] buffer;

	if (tx_ret == TX_OK){
		if (bSend == 1){
				handleTxLegacy();
    }
		

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

MacAddr FanetMac::readAddr(void)
{
	uint64_t chipmacid = ESP.getEfuseMac();
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

FanetMac fmac = FanetMac();
