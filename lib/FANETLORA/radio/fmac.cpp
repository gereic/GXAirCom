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

void invertba(byte* ba, int len);
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
	((uint16_t*)buffer)[3] |= !!Data->no_track<<15;
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


/* this is executed in a non-linear fashion */
void FanetMac::frameReceived(int length)
{
	/* quickly read registers */
	num_received = LoRa.getFrame(rx_frame, sizeof(rx_frame));

	int rssi = LoRa.getRssi();
	int snr = (120 + rssi) * 10; //we use the rssi as snr-value cause snr is normally -10 to 20db on Lora. so we use rssi which is 0 - -120 in invert it to 0 - 120
	if (snr < 0) snr = 0;	
  //log_i("rssi=%d,snr=%d",rssi,snr);

#if MAC_debug_mode > 0
	Serial.printf("### Mac Rx:%d @ %d ", num_received, rssi);

	for(int i=0; i<num_received; i++)
	{
		Serial.printf("%02X", rx_frame[i]);
		if(i<num_received-1)
			Serial.printf(":");
	}
	Serial.printf("\n");
#endif

	/* build frame from stream */
	Frame *frm;
  if (_fskMode){
    frm = new Frame();
    //uint16_t crc16 =  getLegacyCkSum(rx_frame,24);
    //uint32_t tNow = now();
    //log_i("now=%d",tNow);
    invertba(rx_frame,26);
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
 			frm->src.manufacturer = uint8_t(air.addr >> 16);
      frm->src.id = uint16_t(air.addr & 0x0000FFFF);
      frm->dest = MacAddr();
      frm->forward = false;
      frm->type = FRM_TYPE_TRACKING;
			frm->payload_length = serialize_legacyTracking(&air,frm->payload);
      //log_i("src=%02X%04X,dest=%02X%04X,type=%d",frm->src.manufacturer,frm->src.id,frm->dest.manufacturer,frm->dest.id,frm->type);
    }else{
      delete frm;
      return;
    }
  }else{
    frm = new Frame(num_received, rx_frame);
  }  
	frm->rssi = rssi;
	frm->snr = snr;
	
	/* add to fifo */
	if (rx_fifo.add(frm) < 0)
		delete frm;
}

/* wrapper to fit callback into c++ */
void FanetMac::frameRxWrapper(int length)
{
	//log_i("received %d",length);
	fmac.frameReceived(length);
}

void FanetMac::end()
{
  // stop LoRa class
  LoRa.end();
  SPI.end();
  //if (_ss >= 0) digitalWrite(_ss,LOW);
  //if (_reset >= 0) digitalWrite(_reset,HIGH);
}


bool FanetMac::begin(int8_t sck, int8_t miso, int8_t mosi, int8_t ss,int reset, int dio0,Fapp &app,long frequency,uint8_t level)
{
	myApp = &app;
	setup_frequency=frequency;
	_ss = ss;
	_reset = reset;
	_fskMode = false;

	/* configure phy radio */
	//SPI LoRa pins
	log_i("sck=%d,miso=%d,mosi=%d,ss=%d,reset=%d,dio0=%d",sck,miso,mosi,ss,reset,dio0);
	SPI.begin(sck, miso, mosi, ss);
	
	//setup LoRa transceiver module
	LoRa.setPins(ss, reset, dio0);
	//LoRa.setPins(18, 14, 26);
	//long frequency = FREQUENCY868;
	//if (setting.band == BAND915)frequency = FREQUENCY915; 
	log_i("Start Lora Frequency=%d",frequency);
	uint8_t counter = 0;
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
	LoRa.setSyncWord(MAC_SYNCWORD);
	LoRa.enableCrc();
	//LoRa.setTxPower(10); //10dbm + 4dbm antenna --> max 14dbm
	//LoRa.setTxPower(10); //+4dB antenna gain (skytraxx/lynx) -> max allowed output (14dBm) (20 //full Power)
	//level = 20;
	log_i("set tx Power=%d",level);
	LoRa.setTxPower(level); //+4dB antenna gain (skytraxx/lynx) -> max allowed output (14dBm) (20 //full Power)
	//LoRa.setTxPower(20); //+4dB antenna gain (skytraxx/lynx) -> max allowed output (14dBm) (20 //full Power)
	//LoRa.onReceive(frameRxWrapper);
	//LoRa.receive();
	LoRa.setArmed(true,frameRxWrapper); //set receiver armed
	log_i("LoRa Initialization OK!");

	// address 
	_myAddr = readAddr();

	/* start state machine */
	myTimer.Start();

	/* start random machine */
#if defined(ARDUINO_SAMD_ZERO) || defined(ARDUINO_SAMD_VARIANT_COMPLIANCE)
	/* use the device ID */
	volatile uint32_t *ptr1 = (volatile uint32_t *)0x0080A00C;
	volatile uint32_t *ptr2 = (volatile uint32_t *)0x0080A040;
	volatile uint32_t *ptr3 = (volatile uint32_t *)0x0080A044;
	volatile uint32_t *ptr4 = (volatile uint32_t *)0x0080A048;
	randomSeed(millis() + *ptr1 + *ptr2 + *ptr3 + *ptr4);
#else
	randomSeed(millis());
#endif
	return true;
}

void FanetMac::switchLora(){
  uint32_t tBegin = millis();
  if(!LoRa.setLoRa())
		Serial.println("LoRA Set Error");
	delay(10);
	LoRa.setFrequency(setup_frequency);
	LoRa.setSignalBandwidth(250E3); //set Bandwidth to 250kHz
	LoRa.setSpreadingFactor(7); //set spreading-factor to 7
	LoRa.setCodingRate4(8);
	LoRa.setSyncWord(MAC_SYNCWORD);
	LoRa.enableCrc();
	//LoRa.dumpRegisters(Serial);
	LoRa.setArmed(true,frameRxWrapper); 
	//LoRa.irqEnable(true);
  _fskMode = false;
  //log_i("LORA-Mode On %d",millis()-tBegin);
}

void FanetMac::switchFSK(){

  uint32_t tBegin = millis();
	uint8_t syncWord[] = {0x99, 0xA5, 0xA9, 0x55, 0x66, 0x65, 0x96};
  LoRa.ClearIRQ();	
	LoRa.setArmed(false,frameRxWrapper); 
	if (!LoRa.setFSK())
		Serial.println("FSK Set Error");

	LoRa.setBitRate(100.0);
	LoRa.setFrequencyDeviation(50.0);
	LoRa.setPreambleLength(2);
	LoRa.setPreamblePolarity(true);
	LoRa.setEncoding(1);
	LoRa.setSyncWordFSK(syncWord,sizeof(syncWord));
	LoRa.setFrequency(868199950);
  //LoRa.setFrequency(868200000);

	LoRa.setPaRamp(8);	
	LoRa.disableCrc();
	LoRa.setRxBandwidth(125.0);

  //log_i("t=%d",millis()-tBegin);

  sendLegacy(); //we send always a legacy-package, when we swtich to legacy !!

  LoRa.setPacketMode(0,26);
  LoRa.setArmed(true,frameRxWrapper); 

  _fskMode = true;
  //log_i("FSK-Mode On %d",millis()-tBegin);
}



/* wrapper to fit callback into c++ */
void FanetMac::stateWrapper()
{
	static uint32_t tSwitch = millis();
	uint32_t tAct = millis();
	if (fmac._enableLegacyTx == 2){
		if (fmac._fskMode){
			if ((tAct - tSwitch) >= 1900){ // 1.9sec in FSK-Mode
				fmac.switchLora();
				tSwitch = tAct;
			}
		}else{
			if ((tAct - tSwitch) >= 3000){ // 3sec. in FANET-Mode
				fmac.switchFSK();
				tSwitch = tAct;
			}
		}
	}
	fmac.handleIRQ();
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
	int packetSize = LoRa.parsePacket();
	if (packetSize > 0){
		//log_i("packet receive %d",packetSize);
    frameRxWrapper(packetSize);
	}
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
				&& (frm->dest == MacAddr() || isNeighbor(frm->dest)) && LoRa.get_airlimit() < 0.5f)
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



void invertba(byte* ba, int len)
{
  for (int i =0;i<len;i++)
    ba[i] =~ba[i];
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

	invertba(byteArr,26);

	LoRa.SetTxIRQ();
	LoRa.SetFifoTresh();

	LoRa.write(byteArr,26);
	LoRa.setPacketMode(0,26);
	LoRa.setTXFSK();

	LoRa.WaitTxDone();
	LoRa.ClearIRQ();

}

void FanetMac::handleTxLegacy()
{
	//switchFSK();
  uint8_t syncWord[] = {0x99, 0xA5, 0xA9, 0x55, 0x66, 0x65, 0x96};	
  LoRa.ClearIRQ();	
	LoRa.setArmed(false,frameRxWrapper); 
	//LoRa.dumpRegisters(Serial);
	//LoRa.irqEnable(false);
	if (!LoRa.setFSK())
		Serial.println("FSK Set Error");

	LoRa.setBitRate(100.0);
	LoRa.setFrequencyDeviation(50.0);
	LoRa.setPreambleLength(2);
	LoRa.setPreamblePolarity(true);
	LoRa.setEncoding(1);
	LoRa.setSyncWordFSK(syncWord,sizeof(syncWord));
	LoRa.setFrequency(868199950);

	LoRa.setPaRamp(8);	
	LoRa.disableCrc();
	LoRa.setRxBandwidth(125.0);

  sendLegacy();

	switchLora();
	
//	Serial.println("Lora Set Radio End ");
}




/*
 * get a from from tx_fifo (or the app layer) and transmit it
 */
void FanetMac::handleTx()
{
	/* still in backoff or chip turned off*/
	if (millis() < csma_next_tx  || !LoRa.isArmed())
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
	else if(LoRa.get_airlimit() < 0.9f)
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
	int tx_ret = LoRa.sendFrame(buffer, blength, neighbors.size() < MAC_CODING48_THRESHOLD ? 8 : 5);
	//int tx_ret=TX_OK;
	bool bSendLegacy = false;
  if ((_enableLegacyTx == 1) && ((buffer[0]&0x1f)==1)) bSendLegacy = true;
	delete[] buffer;

	if (tx_ret == TX_OK)
	{
		if (bSendLegacy){
			//if ((buffer[0]&0x1f)==1)  //only my traking data
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
