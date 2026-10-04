/*
 * mac.cpp
 *
 * Created on: 30 Sep 2016
 * Author: sid
 *
 * ADS-L M-Band additions — feature/adsl-protocol:
 * - #include AdslProtocol.h
 * - switchMode(): MODE_ADSL_8682 / MODE_ADSL_8684 cases
 * - handleTxAdsl(): new method, transmits one ADS-L packet per 4-second slot
 * - stateWrapper(): calls fmac.handleTxAdsl() after handleTxLegacy()
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

// ── ADS-L NEU ────────────────────────────────────────────────────────────────
#include "../ADSL/AdslProtocol.h"
#include "manchester.h"
// ─────────────────────────────────────────────────────────────────────────────

int serialize_legacyTracking(ufo_t *Data,uint8_t*& buffer);
int serialize_legacyGroundTracking(ufo_t *Data,uint8_t*& buffer);

/* get next frame which can be sent out */
Frame* MacFifo::get_nexttx()
{
	int next;
	for (next = 0; next < fifo.size(); next++)
		if (fifo.get(next)->next_tx < millis())
			break;
	Frame *frm;
	if (next == fifo.size())
		frm = NULL;
	else
		frm = fifo.get(next);
	return frm;
}

Frame* MacFifo::frame_in_list(Frame *frm)
{
	for (int i = 0; i < fifo.size(); i++)
	{
		Frame *frm_list = fifo.get(i);
		if (*frm_list == *frm)
		{
			return frm_list;
		}
	}
	return NULL;
}

Frame* MacFifo::front()
{
	Frame *frm = fifo.shift();
	return frm;
}

/* add frame to fifo */
int MacFifo::add(Frame *frm)
{
	/* buffer full */
	if (fifo.size() >= MAC_FIFO_SIZE && frm->type != FRM_TYPE_ACK)
	{
		return -1;
	}

	if (frm->ack_requested)
	{
		for (int i = 0; i < fifo.size(); i++)
		{
			Frame *ffrm = fifo.get(i);
			if (ffrm->ack_requested && ffrm->src == fmac.myAddr && ffrm->dest == frm->dest)
			{
				return -2;
			}
		}
	}

	if (frm->type == FRM_TYPE_ACK)
		fifo.unshift(frm);
	else
		fifo.add(frm);

	return 0;
}

/* remove frame from linked list and delete it */
bool MacFifo::remove_delete(Frame *frm)
{
	bool found = false;

	for (int i = 0; i < fifo.size() && !found; i++)
		if (frm == fifo.get(i))
		{
			delete fifo.remove(i);
			found = true;
		}

	return found;
}

/* remove any pending frame that waits on an ACK from a host */
bool MacFifo::remove_delete_acked_frame(MacAddr dest)
{
	bool found = false;

	for (int i = 0; i < fifo.size(); i++)
	{
		Frame* frm = fifo.get(i);
		if (frm->ack_requested && frm->dest == dest)
		{
			delete fifo.remove(i);
			found = true;
		}
	}
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
  int msgSize = 7;
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
    return 1;
  case eFlarmAircraftType::HANG_GLIDER :
    return 2;
  case eFlarmAircraftType::BALLOON :
    return 3;
  case eFlarmAircraftType::GLIDER_MOTOR_GLIDER :
    return 4;
  case eFlarmAircraftType::TOW_PLANE :
    return 5;
  case eFlarmAircraftType::HELICOPTER_ROTORCRAFT :
    return 6;
  case eFlarmAircraftType::UAV :
    return 7;
  default:
    return 0;
  }
}

int serialize_legacyTracking(ufo_t *Data,uint8_t*& buffer){
  int msgSize = 11;
  buffer = new uint8_t[msgSize];
  coord2payload_absolut(Data->latitude,Data->longitude, &buffer[0]);

	int alt = constrain(Data->altitude, 0, 8190);
	if(alt > 2047)
		((uint16_t*)buffer)[3] = ((alt+2)/4) | (1<<11);
	else
		((uint16_t*)buffer)[3] = alt;
	((uint16_t*)buffer)[3] |= !(Data->stealth || Data->no_track)<<15;
	((uint16_t*)buffer)[3] |= (Flarm2FanetAircraft((eFlarmAircraftType)Data->aircraft_type)&0x7)<<12;

	int speed2 = constrain((int)roundf(Data->speed *2.0f), 0, 635);
	if(speed2 > 127)
		buffer[8] = ((speed2+2)/5) | (1<<7);
	else
		buffer[8] = speed2;

	int climb10 = constrain((int)roundf(Data->vs *10.0f), -315, 315);
	if(std::abs(climb10) > 63)
		buffer[9] = ((climb10 + (climb10>=0?2:-2))/5) | (1<<7);
	else
		buffer[9] = climb10 & 0x7F;

	buffer[10] = constrain((int)roundf(Data->course *256.0f/360.0f), 0, 255);

	return 11;
}

void FanetMac::sendUdpData(const uint8_t *buffer,int len){
	if ((WiFi.status() == WL_CONNECTED) || (WiFi.softAPgetStationNum() > 0)){
		WiFiUDP udp;
		udp.beginPacket("192.168.0.178",10110);
		udp.write(buffer,len);
		udp.endPacket();    
	}
}

/* this is executed in a non-linear fashion */
void FanetMac::frameReceived(int length)
{
	int num_received = length;
	uint8_t rx_frame[MAC_FRAME_LENGTH];	
	int state = radio.readData(&rx_frame[0], num_received);
	if (state != ERR_NONE) {
		if (state == ERR_CRC_MISMATCH) {
			//log_e("CRC error!");
		}else{
			log_e("failed, code %d",state);
		}
		return;
	}
	#if RX_DEBUG > 1
  if (_actMode != MODE_LORA && num_received >= 26) {
      char hexDump[256];
      int pos = sprintf(hexDump, "RAW RX (%d bytes): ", num_received);
      for(int i = 0; i < num_received && i < 64; i++) {
          pos += sprintf(hexDump + pos, "%02X ", rx_frame[i]);
      }
      Serial.println(hexDump);
  }
  #endif
	int rssi = radio.getRSSI();
	int snr = 0;	
	snr = rssi + 120;
	if (snr < 0) snr = 0;
	
	Frame *frm;
  if (_actMode != MODE_LORA){			
		time_t tUnix;
		time(&tUnix);
		#ifdef FREQUENCYTEST
		if (gtPPS != 0){
			static uint32_t tWait = millis();
			static uint32_t tmax = 0;
			static uint32_t tmin = 1000;
			uint32_t tdiff = 0;
			uint32_t tPPS = millis()-_ppsMillis;
			uint32_t tDiff2 = gtReceived - gtPPS;
			tPPS = tDiff2;
			if (tPPS < 2000){
				if (_actMode == MODE_FSK_8682){
					if (tmin > tPPS) tmin = tPPS;
					if (tmax < tPPS) tmax = tPPS;
				}else{
					if (tPPS > 400){
						if (tmin > tPPS) tmin = tPPS;
					}else{
						if (tmax < tPPS) tmax = tPPS;
					}	
					tdiff = tmin - tmax;				
				}
				if (tmin > tmax){
					tdiff = tmin - tmax;
				}else{
					tdiff = tmax - tmin;
				}
			}
			char Buffer[500];	
			int len = 0;	
			len += sprintf(Buffer+len,"min=%d;max=%d;diff=%d,%d,%d,%d,%d",tmin,tmax,tdiff,tDiff2,tPPS,gtPPS,gtReceived);		
			len += sprintf(Buffer+len,"\\n");
			Serial.print(Buffer);
		}
		#endif

		// ── ADS-L Rx 24-byte check + tracking data extraction ────────────────
		// With the 8-byte hardware sync the radio chip strips Manchester(0x18),
		// so the FIFO delivers 24 bytes (net_header + data + CRC-24).
		// Reconstruct the full 25-byte packet by prepending the known 0x18 length
		// byte before passing to adsl_decode_packet().
		bool decoded = false;
		if (num_received >= 24 && _RfMode.bits.AdslRx) {
			adsl_iconspicuity_t adslData;
			uint8_t adsl_full[ADSL_PACKET_SIZE];
			adsl_full[0] = 0x18; // length byte absorbed by 8-byte sync; restore it
			memcpy(&adsl_full[1], rx_frame, 24);
			if (adsl_decode_packet(adsl_full, ADSL_PACKET_SIZE, &adslData)) {
				// Create Frame for ADS-L data
				frm = new Frame();
				frm->src.manufacturer = (adslData.address >> 16) & 0xFF;
				frm->src.id = adslData.address & 0xFFFF;
				frm->altitude = (int32_t)round(adslData.altitude_wgs84_m);
				frm->type = FRM_TYPE_TRACKING_LEGACY;  // Legacy tracking packet
				frm->AddressType = 0x84;  // Distinct marker for ADS-L (NOT 0x80=FANET)
				frm->legacyAircraftType = (uint8_t)adslData.aircraft_category;
				frm->rssi = rssi;
				frm->snr = snr;
				frm->timeStamp = now();
				
				// Store decoded ADS-L data in Frame for later processing by FanetLora
				frm->adslData = new adsl_iconspicuity_t(adslData);
				
				rxFntCount++;
				decoded = true;
				
				#if RX_DEBUG > 1
				uint32_t devId = adslData.address & 0x00FFFFFF;
				log_i("ADS-L RX: id=%06X lat=%.6f lon=%.6f alt=%.1f spd=%.1f clb=%.2f hdg=%.0f rssi=%d",
				      devId, adslData.latitude, adslData.longitude, adslData.altitude_wgs84_m,
				      adslData.speed_ms * 3.6, adslData.vertical_rate_ms, adslData.track_deg, rssi);
				#endif
			} else {
				#if RX_DEBUG > 1
				Serial.println(">>> Not valid ADS-L (CRC failed). Trying FLARM...");
				#endif
			}
		} 
		// ── Legacy FLARM Rx 26 byte check ───────────────────────────────────
		if (!decoded && num_received >= 26) {
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
				localtime_r(&tUnix, &timeinfo);
				strftime(strftime_buf, sizeof(strftime_buf), "%F %T", &timeinfo);   
				len += sprintf(Buffer+len,"%s;",strftime_buf);
				len += sprintf(Buffer+len,"F=%d;",fmac.actflarmFreq);
				len += sprintf(Buffer+len,"Rx=%d;rssi=%d;", num_received, rssi);
				for(int i=0; i<num_received; i++)
				{
					len += sprintf(Buffer+len,"%02X", rx_frame[i]);
					if (i >= 26) break;
				}
				len += sprintf(Buffer+len,"\\n");
				Serial.print(Buffer);
			#endif

			uint16_t crc16_2 = (uint16_t(rx_frame[24]) << 8) + uint16_t(rx_frame[25]);
			uint16_t crc16 =  flarm_getCkSum(rx_frame,24);
			if (crc16 != crc16_2){
				#if RX_DEBUG > 0
				log_e("%d Flarm: wrong Checksum %04X!=%04X",millis(),crc16,crc16_2);
				#endif
				return;
			}

			ufo_t air={0};
			ufo_t myAircraft={0};
			myAircraft.latitude = lat;
			myAircraft.longitude = lon;
			myAircraft.geoid_separation = geoidAlt;
			myAircraft.timestamp = tUnix; 
			uint8_t newPacket[26];
			uint32_t tOffset = 0;	
			bool bOk = false;
			int i = 0;
			for(i = 0;i < 5; i++){
				memcpy(&newPacket[0],&rx_frame[0],26);
				myAircraft.timestamp = tUnix + tOffset;
				bOk = flarm_decode(&newPacket[0],&myAircraft,&air);
				if (bOk){				
					float dist = distance(myAircraft.latitude,myAircraft.longitude,air.latitude,air.longitude, 'K');
					if (dist > 100.0){
						flarm_debugAircraft(&air,&myAircraft);
						log_e("distance %.1f > 100km --> error ",dist);
						bOk = false;
					}else{
						break;
					}
				}
				if (i == 0){
					tOffset = 1;
				}else if (i == 1){
					tOffset = -1;
				}else if (i == 2){
					tOffset = 2;
				}else if (i == 3){
					tOffset = -2;
				}				
			}
			
			#if RX_DEBUG > 0
			if ((bOk == true) && (i > 0)){
				flarm_v7_packet_t *pkt = (flarm_v7_packet_t *)&newPacket[0];
				log_i("id=%06X,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",pkt->addr,bOk,i,pkt->_unk1,pkt->_unk2,pkt->_unk3,pkt->_unk4,pkt->_unk5,pkt->_unk6,pkt->_unk7,pkt->_unk8,pkt->_unk9,pkt->_unk10);
			}
			#endif
			#ifdef DEBUG_FLARM_RX
			if (bOk == true){
				flarm_v7_debugBuffer(&rx_frame[0],&myAircraft);
			}
			#endif
			if (bOk){
				#ifdef FLARMLOGGER
				if (fmac.actflarmFreq == 868400000){
					flarm_debugLog(_ppsMillis,&rx_frame[0],&myAircraft);
				}
				#endif
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
				decoded = true;
			}else{
				return;
			}
		} else if (!decoded) {
      return;
    }
  }else{
    frm = new Frame(num_received, rx_frame);
		frm->timeStamp = uint32_t(now());
		frm->AddressType = 2 + 0x80; //Fanet is always type 2
		rxFntCount++;
  }  
	frm->rssi = rssi;
	frm->snr = snr;
	if ((frm->src.id == 0) || (frm->src.manufacturer == 0)){
    delete frm;
    return;
  }
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
  radio.end();
  SPI.end();
}


bool FanetMac::begin(int8_t sck, int8_t miso, int8_t mosi, int8_t ss,int8_t reset, int8_t dio0,int8_t gpio,Fapp &app,long frequCor,uint8_t level,uint8_t radioChip)
{
	myApp = &app;
	_ss = ss;
	_reset = reset;
	_actMode = 0;
	_frequencyCorrection = frequCor;

	_myAddr = readAddr();

	log_i("sck=%d,miso=%d,mosi=%d,ss=%d,reset=%d,dio0=%d,gpio=%d,chip=%d",sck,miso,mosi,ss,reset,dio0,gpio,radioChip);
	SPI.begin(sck, miso, mosi, ss);
	if (radioChip == RADIO_SX1262){
		radio.setPins(&SPI,ss,dio0,reset,gpio);
	}else{
		radio.setPins(&SPI,ss,dio0,reset);
	}
	radio.gain = 1; //highest gain setting for FSK
  int state = radio.begin(250.0,7,8,0xF1,int8_t(level),radioChip);
  if (state == ERR_NONE) {
  } else {
    log_e("failed, code %d",state);
		return 0;
  }
	log_i("LoRa Initialization OK!");

	myTimer.Start();
	randomSeed(millis());

	// ── ADS-L NEU: initialise ADS-L state ──────────────────────────────────
	_adslFreqToggle  = false;
	_adsl_last_tx    = 0;
	_flarm8682TxDone = 0;
	memset(_adslBuffer, 0, sizeof(_adslBuffer));
	// ─────────────────────────────────────────────────────────────────────────
	
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
		radio.switchLORA(loraFrequency + _frequencyCorrection,loraBandwidth);
	}else if (mode == MODE_FSK_8682){
		flarmFrequency = 868200000;
		actflarmFreq = flarmFrequency + _frequencyCorrection;
		radio.switchFSK(actflarmFreq);
	}else if (mode == MODE_FSK_8684){
		flarmFrequency = 868400000;
		actflarmFreq = flarmFrequency + _frequencyCorrection;
		radio.switchFSK(actflarmFreq);
	}else if (mode == MODE_FSK){		
		time(&tUnix);
		channel = flarm_calculate_freq_channel(tUnix,(uint32_t)flarmChannels);
		uint32_t frequ = 0;
		frequ = flarmFrequency + (channel * ChanSepar) + _frequencyCorrection;
		if ((actflarmFreq != frequ) || (!radio.isFskMode())) {
			#if RX_DEBUG > 1
			bChanged = true;
			#endif
			actflarmFreq = frequ;			
			radio.switchFSK(actflarmFreq);
		}
	// ── ADS-L NEU: M-Band modes ─────────────────────────────────────────────
	// M-Band uses identical RF parameters to FLARM FSK (2-GFSK, 100kbps raw)
	// PLUS Manchester encoding (enabled via LoRa.setManchesterEncoding()).
	// The frequency is fixed (no PPS-synchronised hopping).
	}else if (mode == MODE_ADSL_8682){
		uint32_t adslFreq = (uint32_t)ADSL_FREQ_MBAND_1 + (uint32_t)_frequencyCorrection;
		actflarmFreq = adslFreq;
		radio.switchFSK(adslFreq, 48);
		radio.setADSLSyncWord();           // Configure ADS-L Manchester sync word
		#if TX_DEBUG > 0
		log_i("ADS-L: switchMode ADSL_8682 freq=%lu", adslFreq);
		#endif
	}else if (mode == MODE_ADSL_8684){
		uint32_t adslFreq = (uint32_t)ADSL_FREQ_MBAND_2 + (uint32_t)_frequencyCorrection;
		actflarmFreq = adslFreq;
		radio.switchFSK(adslFreq, 48);
		radio.setADSLSyncWord();           // Configure ADS-L Manchester sync word
		#if TX_DEBUG > 0
		log_i("ADS-L: switchMode ADSL_8684 freq=%lu", adslFreq);
		#endif
	}
	// ─────────────────────────────────────────────────────────────────────────
	
	if (bStartReceive){
		radio.startReceive();	
	} 
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
			len += sprintf(Buffer+len,"LORA %dHz BW=%d",loraFrequency,loraBandwidth);
		}else if (mode == MODE_FSK_8682){
			len += sprintf(Buffer+len,"FSK %dHz ",actflarmFreq);
		}else if (mode == MODE_FSK_8684){
			len += sprintf(Buffer+len,"FSK %dHz ",actflarmFreq);
		}else if (mode == MODE_FSK){
			len += sprintf(Buffer+len,"FSK c=%d,f=%d,t=%d",channel,actflarmFreq,tUnix);
		}else if (mode == MODE_ADSL_8682){
			len += sprintf(Buffer+len,"ADSL 868.2MHz");
		}else if (mode == MODE_ADSL_8684){
			len += sprintf(Buffer+len,"ADSL 868.4MHz");
		}
		len += sprintf(Buffer+len,"in %dus pps=%d\\n",int(micros()-tBegin),int(millis() - _ppsMillis));
		Serial.print(Buffer);
	}
	#endif
}

// ── ADS-L NEU: handleTxAdsl() ────────────────────────────────────────────────
/**
 * @brief Transmit one ADS-L M-Band iConspicuity packet.
 *
 * Called every MAC timer tick from stateWrapper().
 * Respects the ADSL_TX_INTERVAL_MS (4 s) between transmissions.
 * Alternates between 868.2 and 868.4 MHz as required by the spec.
 * Restores the previous radio mode after transmission.
 */
void FanetMac::handleTxAdsl()
{
  static uint8_t ppsCount = 0;

  if (!_RfMode.bits.AdslTx || !myApp || !fmac.bHasGPS) return;

  // --- PHASE 1: SCHEDULING ---
  if (adsl_next_tx == 0) {
    if ((millis() - _adsl_last_tx) >= ADSL_TX_INTERVAL_MS) {
      if (_RfMode.bits.LegTx) {
        // FLARM active: fire ~30ms after FLARM 8682 TX so we inherit its random slot.
        // Each device already picks a random FLARM slot per cycle; ADS-L rides on that.
        if (_flarm8682TxDone != 0) {
          memset(_adslBuffer, 0, sizeof(_adslBuffer));
          if (myApp->createAdsl(_adslBuffer, _adslFreqToggle)) {
            adsl_next_tx = _flarm8682TxDone + 30;
          } else {
            _adsl_last_tx = millis();
          }
          _flarm8682TxDone = 0;
        }
      } else {
        // ADS-L only (no FLARM TX): pick a new random slot each PPS cycle,
        // same logic as FLARM uses for its own window.
        if (ppsCount != _ppsCount) {
          ppsCount = _ppsCount;
          memset(_adslBuffer, 0, sizeof(_adslBuffer));
          if (myApp->createAdsl(_adslBuffer, _adslFreqToggle)) {
            adsl_next_tx = _ppsMillis + random(50, 950);
          } else {
            _adsl_last_tx = millis();
          }
        }
      }
    }
  }
  // --- PHASE 2: EXECUTION ---
  else if (millis() >= adsl_next_tx) {
    uint8_t oldMode = _actMode;

    if (_adslFreqToggle) {
      switchMode(MODE_ADSL_8684, false);
    } else {
      switchMode(MODE_ADSL_8682, false);
    }

    // Software Manchester (G.E. Thomas: 0→10, 1→01) via ManchesterEncode[] LUT.
    // buf[0] = 0x18 (length byte) is absorbed into the 8-byte hardware sync word
    // (last 2 bytes = Manchester(0x18) = {0xA9, 0x6A}) — matches SoftRF interop.
    // So encode only buf[1..24] → 48 bytes in _adslEncodedBuffer.
    for (int i = 0; i < (ADSL_PACKET_SIZE - 1) * 2; i++){
        int src = 1 + (i >> 1);
        _adslEncodedBuffer[i]   = ManchesterEncode[(_adslBuffer[src] >> 4) & 0x0F];
        _adslEncodedBuffer[i+1] = ManchesterEncode[(_adslBuffer[src])      & 0x0F];
        i++;
    }
    // Transmit the 48-byte encoded packet (length byte lives in the 8-byte sync).
    int16_t txState = radio.transmit(_adslEncodedBuffer, (ADSL_PACKET_SIZE - 1) * 2);
    
    if (txState == ERR_NONE) {
      txAdslCount++; // Internal MAC counter
    } else {
      log_e("ADS-L TX error: code=%d", txState);
    }

    _adslFreqToggle = !_adslFreqToggle;
    _adsl_last_tx = millis();
    adsl_next_tx = 0;

    if (oldMode != _actMode) {
      switchMode(oldMode, false);
    }
    radio.startReceive();
  }
}
// ─────────────────────────────────────────────────────────────────────────────

/* wrapper to fit callback into c++ */
void FanetMac::stateWrapper()
{
	static uint32_t tSwitch = millis();
	static uint8_t ppsCount = 0;
	static uint8_t ppsCountFlarm = 0;
	static uint32_t ppsMillis = 0;
	static uint8_t actPPsDiff = 3; 
	uint32_t tAct = millis();
	fmac.handleIRQ();

	uint8_t ppsDiff = fmac._ppsCount - ppsCount;
	if (fmac.bHasGPS){
		#ifdef FREQUENCYTEST
		fmac._RfMode.bits.LegRx = true;
		fmac._RfMode.bits.LegTx = false;
		fmac._RfMode.bits.FntRx = false;
		fmac._RfMode.bits.FntTx = false;
		if (1 == 1){
			if (fmac._actMode != MODE_FSK_8682){
				fmac.switchMode(MODE_FSK_8682);
			}
		}else
		#endif 
		if (fmac._RfMode.bits.FntRx && fmac._RfMode.bits.LegRx){
			if (fmac._actMode == MODE_LORA){
				if ((ppsDiff) > actPPsDiff){
					if (fmac.flarmZone == 1){
						if ((millis() - fmac._ppsMillis) >= (LEGACY_8682_BEGIN - LEGACY_RANGE)){
							actPPsDiff ++;
							if (actPPsDiff > 5) actPPsDiff = 3;
							ppsMillis = fmac._ppsMillis;
							ppsCount = fmac._ppsCount;
							// ── ADS-L NEU: Rx ──────────────────────────────────
							fmac.switchMode(fmac._RfMode.bits.AdslRx ? MODE_ADSL_8682 : MODE_FSK_8682);
							#if RX_DEBUG > 5
							log_i("**** %d start FSK/ADSL 868.2 %d %d",millis(),millis() - ppsMillis,fmac._ppsCount);
							#endif
						}
					}else if (fmac.flarmZone > 0){
							actPPsDiff ++;
							if (actPPsDiff > 5) actPPsDiff = 3;
							ppsMillis = fmac._ppsMillis;
							ppsCount = fmac._ppsCount;
							ppsCountFlarm = fmac._ppsCount;
							fmac.switchMode(MODE_FSK);
					}
				}
			}else if (fmac._actMode == MODE_FSK_8682 || fmac._actMode == MODE_ADSL_8682){
				if ((millis() - ppsMillis) >= (LEGACY_8684_BEGIN - LEGACY_RANGE)){
					// ── ADS-L NEU: Rx ──────────────────────────────────────
					fmac.switchMode(fmac._RfMode.bits.AdslRx ? MODE_ADSL_8684 : MODE_FSK_8684);
					#if RX_DEBUG > 5
					log_i("**** %d start FSK/ADSL 868.4 %d %d",millis(),millis() - ppsMillis,fmac._ppsCount);
					#endif
				}
			}else if (fmac._actMode == MODE_FSK_8684 || fmac._actMode == MODE_ADSL_8684){
				if (((millis() - ppsMillis) >= (LEGACY_8684_END + LEGACY_RANGE)) && ((ppsDiff) >= 1)){
					fmac.switchMode(MODE_LORA);
					#if RX_DEBUG > 5
					log_i("**** %d finisched FSK-Mode %d",millis(),millis() - ppsMillis,fmac._ppsCount);
					#endif
					ppsCount = fmac._ppsCount;
				}
			}else if (fmac._actMode == MODE_FSK){
				if (ppsDiff >= 2){
					fmac.switchMode(MODE_LORA);
					ppsCount = fmac._ppsCount;
				}else{
					if (ppsCountFlarm != fmac._ppsCount){
						ppsCountFlarm = fmac._ppsCount;
						fmac.switchMode(MODE_FSK);
					}
				}
			}
		}else if (!fmac._RfMode.bits.FntRx && fmac._RfMode.bits.LegRx){
			if (fmac.flarmZone == 1){
				if (fmac._actMode != MODE_FSK_8682 && fmac._actMode != MODE_ADSL_8682){
						if ((millis() - fmac._ppsMillis) >= (LEGACY_8682_BEGIN - LEGACY_RANGE) && ((ppsDiff) >= 1)){
							ppsMillis = fmac._ppsMillis;
							ppsCount = fmac._ppsCount;
							fmac.switchMode(fmac._RfMode.bits.AdslRx ? MODE_ADSL_8682 : MODE_FSK_8682);
							#if RX_DEBUG > 5
							log_i("**** %d start FSK 868.2 %d %d",millis(),millis() - ppsMillis,fmac._ppsCount);
							#endif
						}
				}else{
					if ((millis() - ppsMillis) >= (LEGACY_8684_BEGIN - LEGACY_RANGE)){
						fmac.switchMode(fmac._RfMode.bits.AdslRx ? MODE_ADSL_8684 : MODE_FSK_8684);
						#if RX_DEBUG > 5
						log_i("**** %d start FSK 868.4 %d %d",millis(),millis() - ppsMillis,fmac._ppsCount);
						#endif
					}
				}				
			}else if (fmac.flarmZone > 0){
				if (ppsCount !=fmac._ppsCount){
					ppsCount =fmac._ppsCount;
					fmac.switchMode(MODE_FSK);
				}
			}
		}else if (!fmac._RfMode.bits.FntRx && !fmac._RfMode.bits.LegRx && fmac._RfMode.bits.AdslRx){
			if (fmac.flarmZone == 1){
				if (fmac._actMode != MODE_ADSL_8682 && fmac._actMode != MODE_ADSL_8684){
					if ((millis() - fmac._ppsMillis) >= (LEGACY_8682_BEGIN - LEGACY_RANGE) && (ppsDiff >= 1)){
						ppsMillis = fmac._ppsMillis;
						ppsCount = fmac._ppsCount;
						fmac.switchMode(MODE_ADSL_8682);
					}
				}else if (fmac._actMode == MODE_ADSL_8682){
					if ((millis() - ppsMillis) >= (LEGACY_8684_BEGIN - LEGACY_RANGE)){
						fmac.switchMode(MODE_ADSL_8684);
					}
				}
			}
		}else{

		}
	}else{
		if (fmac.flarmZone == 1){
			if (fmac._RfMode.bits.FntRx && fmac._RfMode.bits.LegRx){
				if (fmac._actMode == MODE_LORA){
					if ((tAct - tSwitch) >= 5300){
						fmac.switchMode(fmac._RfMode.bits.AdslRx ? MODE_ADSL_8682 : MODE_FSK_8682);
						tSwitch = millis();
					}
				}else if (fmac._actMode == MODE_FSK_8682 || fmac._actMode == MODE_ADSL_8682){
					if ((tAct - tSwitch) >= 2300){					
						fmac.switchMode(fmac._RfMode.bits.AdslRx ? MODE_ADSL_8684 : MODE_FSK_8684);
						tSwitch = millis();
					}
				}else{
					if ((tAct - tSwitch) >= 2300){					
						fmac.switchMode(MODE_LORA);
						tSwitch = millis();
					}
				}
			}else if (!fmac._RfMode.bits.FntRx && fmac._RfMode.bits.LegRx){
				if ((tAct - tSwitch) >= 1000){
					if (fmac._actMode != MODE_FSK_8682 && fmac._actMode != MODE_ADSL_8682){
						fmac.switchMode(fmac._RfMode.bits.AdslRx ? MODE_ADSL_8682 : MODE_FSK_8682);
					}else{
						fmac.switchMode(fmac._RfMode.bits.AdslRx ? MODE_ADSL_8684 : MODE_FSK_8684);
					}
					tSwitch = tAct;
				}
			}else if (!fmac._RfMode.bits.FntRx && !fmac._RfMode.bits.LegRx && fmac._RfMode.bits.AdslRx){
				if ((tAct - tSwitch) >= 1000){
					fmac.switchMode(fmac._actMode == MODE_ADSL_8682 ? MODE_ADSL_8684 : MODE_ADSL_8682);
					tSwitch = tAct;
				}
			}
		}else if (fmac.flarmZone == 3){
			if (fmac._RfMode.bits.FntRx && fmac._RfMode.bits.LegRx){
				if (fmac._actMode == MODE_LORA){
					if ((tAct - tSwitch) >= 5300){
						fmac.switchMode(MODE_FSK);
						tSwitch = millis();
					}
				}else{
					if ((tAct - tSwitch) >= 2300){					
						fmac.switchMode(MODE_LORA);
						tSwitch = millis();
					}
				}
			}else if (!fmac._RfMode.bits.FntRx && fmac._RfMode.bits.LegRx){			
				if (fmac._actMode != MODE_FSK){
					fmac.switchMode(MODE_FSK);
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
					fmac.switchMode(MODE_LORA);
				}								
				#if RX_DEBUG > 8
				log_i("******************* %d handle FanetTx *****************",millis());
				#endif
				fmac.handleTx();
				if (oldMode != MODE_LORA){
					fmac.switchMode(oldMode);
				}				
			}
		}
	}
	if ((fmac._actMode == MODE_LORA) && (fmac._RfMode.bits.FntTx) && (fmac.eLoraRegion != NONE)){  	
    fmac.handleTx();
  }
	fmac.handleTxLegacy();

	// ── ADS-L NEU: ADS-L M-Band TX slot ───────────────────────────────────
	// handleTxAdsl() is self-timing (ADSL_TX_INTERVAL_MS guard inside).
	// It borrows the radio briefly, then restores the previous mode.
	// Safe to call every MAC timer tick regardless of current radio mode.
	fmac.handleTxAdsl();
	// ─────────────────────────────────────────────────────────────────────────
}

bool FanetMac::isNeighbor(MacAddr addr)
{
	for (int i = 0; i < neighbors.size(); i++)
		if (neighbors.get(i)->addr == addr)
			return true;

	return false;
}

void FanetMac::ack(Frame* frm)
{
	Frame *ack = new Frame(myAddr);
	ack->type = FRM_TYPE_ACK;
	ack->dest = frm->src;

	if (frm->ack_requested == FRM_ACK_TWOHOP && !frm->forward)
		ack->forward = true;

	if (tx_fifo.add(ack) != 0)
		delete ack;
}

void FanetMac::handleIRQ(){
	static uint8_t rxCount = 0;
	if (radio.isRxMessage()) {	
		int16_t packetSize = radio.getPacketLength();
		rxCount++;
		#if RX_DEBUG > 1
		  //log_i("new package arrived %d",packetSize);
		#endif
		if (packetSize > 0){
			fmac.frameReceived(packetSize);
		}
  	radio.startReceive();
	}
	return;
}

void FanetMac::handleRx()
{
	/* nothing to do */
	if (rx_fifo.size() == 0)
	{
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

	bool neighbor_known = false;
	for (int i = 0; i < neighbors.size(); i++)
	{
		if (neighbors.get(i)->addr == frm->src)
		{
			neighbors.get(i)->seen();
			if(frm->type == FRM_TYPE_TRACKING || frm->type == FRM_TYPE_GROUNDTRACKING)
				neighbors.get(i)->hasTracking = true;
			neighbor_known = true;
			break;
		}
	}

	if (neighbor_known == false)
	{
		if (neighbors.size() > MAC_NEIGHBOR_SIZE)
			delete neighbors.shift();

		neighbors.add(new NeighborNode(frm->src, frm->type == FRM_TYPE_TRACKING || frm->type == FRM_TYPE_GROUNDTRACKING));
	}
  if (frm->type == FRM_TYPE_TRACKING_LEGACY) frm->type = FRM_TYPE_TRACKING;
	if (frm->type == FRM_TYPE_GROUNDTRACKING_LEGACY) frm->type = FRM_TYPE_GROUNDTRACKING;

	Frame *frm_list = tx_fifo.frame_in_list(frm);
	if (frm_list != NULL)
	{
		if (frm->rssi > frm_list->rssi + MAC_FORWARD_MIN_DB_BOOST)
		{
			tx_fifo.remove_delete(frm_list);
		}
		else
		{
			frm_list->next_tx = millis() + random(MAC_FORWARD_DELAY_MIN, MAC_FORWARD_DELAY_MAX);
		}
	}
	else
	{
		if ((frm->dest == MacAddr() || frm->dest == myAddr) && frm->src != myAddr)
		{
			if (frm->type == FRM_TYPE_ACK)
			{
				if (tx_fifo.remove_delete_acked_frame(frm->src) && myApp != NULL)
					myApp->handle_acked(true, frm->src);
			}
			else
			{
				if (frm->ack_requested)
					ack(frm);

				if (myApp != NULL)
					myApp->handle_frame(frm);
			}
		}

		if (doForward && frm->forward && tx_fifo.size() < MAC_FIFO_SIZE - 3 && frm->rssi <= MAC_FORWARD_MAX_RSSI_DBM
				&& (frm->dest == MacAddr() || isNeighbor(frm->dest)) && radio.get_airlimit() < 0.5f)
		{
			frm->forward = false;
			frm->next_tx = millis() + random(MAC_FORWARD_DELAY_MIN, MAC_FORWARD_DELAY_MAX);
			frm->num_tx = !!frm->ack_requested;
			tx_fifo.add(frm);
			return;
		}
	}

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
	if (eLoraRegion == NONE){
		if (-169.0f < lon && lon < -30.0f){
			eLoraRegion = US920;
			loraFrequency = 920800000;
			loraBandwidth = 500;
			log_i("set Lora-Region to US920");
		}	else if (110.0f < lon && lon < 179.0f && -48.0f < lat && lat < 10.0f){
			eLoraRegion = AU920;
			loraFrequency = 920800000;
			loraBandwidth = 500;
			log_i("set Lora-Region to AU920");
		} else if (69.0f < lon && lon < 89.0f && 5.0f < lat && lat < 40.0f){
			eLoraRegion = IN866;
			loraFrequency = 866200000;
			loraBandwidth = 250;
			log_i("set Lora-Region to IN866");
		} else if (124.0f < lon && lon < 130.0f && 34.0f < lat && lat < 39.0f){
			eLoraRegion = KR923;
			loraFrequency = 923200000;
			loraBandwidth = 125;
			log_i("set Lora-Region to KR923");
		} else if (89.0f < lon && lon < 146.0f && 21.0f < lat && lat < 47.0f){
			eLoraRegion = AS920;
			loraFrequency = 923200000;
			loraBandwidth = 125;
			log_i("set Lora-Region to AS920");
		} else if (34.0f < lon && lon < 36.0f && 29.0f < lat && lat < 34.0f){
			eLoraRegion = IL918;
			loraFrequency = 918500000;
			loraBandwidth = 125;
			log_i("set Lora-Region to IL918");
		} else{
			eLoraRegion = EU868;
			log_i("set Lora-Region to EU868");
			loraFrequency = 868200000;
			loraBandwidth = 250;
		} 
		bstartRec = true;
	}
	if (flarmZone == 0){
		flarmZone = flarm_get_zone(lat,lon);		
		flarm_getFrequencyChannels(flarmZone,&flarmFrequency,&ChanSepar,&flarmChannels,&radio.maxFskPower);
		log_i("zone=%d,freq=%d,seperation=%d,channels=%d,maxTxPower=%d",flarmZone,flarmFrequency,ChanSepar,flarmChannels,radio.maxFskPower);		
		bstartRec = true;
	}
	if (bstartRec){
		if (_RfMode.bits.FntRx){
			switchMode(MODE_LORA);
		}else if (_RfMode.bits.LegRx){
			if (flarmZone == 1){
				switchMode(_RfMode.bits.AdslRx ? MODE_ADSL_8682 : MODE_FSK_8682);
			}else{
				switchMode(MODE_FSK);
			}
		}else if (_RfMode.bits.AdslRx && flarmZone == 1){
			switchMode(MODE_ADSL_8682);
		}
	}
}

void FanetMac::setPps(uint32_t *ppsMillis){
	_ppsMillis = *ppsMillis;
	_ppsCount++;
}

void FanetMac::handleTxLegacy()
{
	static uint32_t tMillis = 0;
	static bool bSend8682 = false;
	static uint8_t ppsCount = 0;
	if ((!_RfMode.bits.LegTx) || (flarmZone < 1)){
		return;
	} 
	if (!fmac.bHasGPS){
		return;
	}
	if (legacy_next_tx == 0){
		if (flarmZone == 1){
			if (_RfMode.bits.FntTx){
				if (((millis() - _ppsMillis) >= LEGACY_8684_BEGIN-LEGACY_RANGE) && (ppsCount != _ppsCount)){
					tMillis = _ppsMillis;		
					ppsCount = _ppsCount;		
					if (myApp->createLegacy(&FlarmBuffer[0])){
						legacy_next_tx = tMillis + uint32_t(random(LEGACY_8684_BEGIN,LEGACY_8684_END - LEGACY_SEND_TIME));
						bSend8682 = false;
					}
				}
			}else{
				if (((millis() - _ppsMillis) >= LEGACY_8682_BEGIN-LEGACY_RANGE) && (ppsCount != _ppsCount)){
					tMillis = _ppsMillis;	
					ppsCount = _ppsCount;			
					if (myApp->createLegacy(&FlarmBuffer[0])){
						legacy_next_tx = tMillis + uint32_t(random(LEGACY_8682_BEGIN,LEGACY_8682_END - LEGACY_SEND_TIME));
						bSend8682 = true;
					}
				}
			}
		}else{
			if (((millis() - _ppsMillis) >= 200) && (ppsCount != _ppsCount)){
				tMillis = _ppsMillis;		
				ppsCount = _ppsCount;		
				if (myApp->createLegacy(&FlarmBuffer[0])){
					legacy_next_tx = tMillis + uint32_t(random(200,800));
					bSend8682 = false;
				}
			}
		}
	}else if (millis() >= legacy_next_tx){		
		#if TX_DEBUG > 0
			uint32_t tStart = micros();
		#endif		

		uint8_t oldMode = _actMode;
		if (bSend8682){
			if (_actMode != MODE_FSK_8682){				
				fmac.switchMode(MODE_FSK_8682,false);
			}
		}else{
			if (flarmZone == 1){
				if (_actMode != MODE_FSK_8684){				
					fmac.switchMode(MODE_FSK_8684,false);
				}
			}else{
				fmac.switchMode(MODE_FSK,false);
			}
		}
		int16_t state = radio.transmit(FlarmBuffer, sizeof(FlarmBuffer));
		if (state != ERR_NONE){
			log_e("error TX state=%d",state);
		}else{
			txLegCount++;
		}
		if (_actMode != oldMode){
			fmac.switchMode(oldMode,false);
		}
		radio.startReceive();
		#if TX_DEBUG > 0
		Serial.printf("ppsCount=%d,leg_Tx=%d\\n",ppsCount,micros()-tStart);
		#endif
		if (bSend8682){
			_flarm8682TxDone = millis(); // ADS-L uses this to fire just after FLARM 8682
		  legacy_next_tx = tMillis + uint32_t(random(LEGACY_8684_BEGIN,LEGACY_8684_END - LEGACY_SEND_TIME));
			bSend8682 = false;
		}else{
			legacy_next_tx = 0;
		}
	}
}

void FanetMac::handleTx()
{
	if (millis() < csma_next_tx)
		return;

	Frame* frm;
	bool app_tx = false;
	if (myApp->is_broadcast_ready(neighbors.size()))
	{
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
		frm = tx_fifo.get_nexttx();
		if (frm == nullptr)
			return;
		if (frm->ack_requested && frm->num_tx <= 0)
		{
			if (myApp != nullptr && frm->src == myAddr)
				myApp->handle_acked(false, frm->dest);
			tx_fifo.remove_delete(frm);
			return;
		}

		if (frm->forward == false && frm->dest != MacAddr() && isNeighbor(frm->dest) == false)
			frm->forward = true;

		app_tx = false;
	}
	else
	{
		return;
	}

	uint8_t* buffer;
	int blength = frm->serialize(buffer);
	if (blength < 0)
	{
		if (app_tx)
			delete frm;
		else
			tx_fifo.remove_delete(frm);
		return;
	}

	if (neighbors.size() < MAC_CODING48_THRESHOLD){
		radio.setCodingRate(8);
	}else{
		radio.setCodingRate(5);
	}	
	int16_t state = radio.transmit(buffer, blength);
	int tx_ret=TX_OK;
	if (state != ERR_NONE){
		if (state == ERR_TX_TX_ONGOING){
			tx_ret = TX_RX_ONGOING;
		}else if (state == ERR_TX_RX_ONGOING){
			tx_ret = TX_RX_ONGOING;
		}else if (state == ERR_TX_FSK_ONGOING){
			tx_ret = TX_FSK_ONGOING;
		}else{
			log_e("error TX state=%d",state);
			tx_ret = TX_ERROR;
		} 
	}else{
		txFntCount++;
	}
	state = radio.startReceive();
	if (state != ERR_NONE) {
			log_e("startReceive failed, code %d",state);
	}
	delete[] buffer;

	if (tx_ret == TX_OK){
		if (app_tx)
		{
			myApp->broadcast_successful(frm->type);
			delete frm;
		}
		else
		{
			if (!frm->ack_requested || frm->src != myAddr)
			{
				tx_fifo.remove_delete(frm);
			}
			else
			{
				if (--frm->num_tx > 0)
					frm->next_tx = millis() + (MAC_TX_RETRANSMISSION_TIME * (MAC_TX_RETRANSMISSION_RETRYS - frm->num_tx));
				else
					frm->next_tx = millis() + MAC_TX_ACKTIMEOUT;
			}
		}

		csma_backoff_exp = MAC_TX_BACKOFF_EXP_MIN;
		csma_next_tx = millis() + MAC_TX_MINPREAMBLEHEADERTIME_MS + (blength * MAC_TX_TIMEPERBYTE_MS);
	}
	else if (tx_ret == TX_RX_ONGOING || tx_ret == TX_TX_ONGOING)
	{
		if (app_tx)
			delete frm;

		if (csma_backoff_exp < MAC_TX_BACKOFF_EXP_MAX)
			csma_backoff_exp++;

		csma_next_tx = millis() + random(1 << (MAC_TX_BACKOFF_EXP_MIN - 1), 1 << csma_backoff_exp);
	}
	else
	{
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
	if ((_myAddr.id != 0 || _myAddr.manufacturer != 0) && !getHwAddr) {
		return _myAddr;
	}
	uint64_t chipmacid = ESP.getEfuseMac();
  log_i("ESP32ChipID=%04X%08X",(uint16_t)(chipmacid>>32),(uint32_t)chipmacid);
	uint8_t myDevId[3];
	myDevId[0] = ManuId;
	myDevId[1] = uint8_t(chipmacid >> 32);
	myDevId[2] = uint8_t(chipmacid >> 40);
    log_i("dev_id=%02X%02X%02X",myDevId[0],myDevId[1],myDevId[2]);
	return MacAddr(myDevId[0],((uint32_t)myDevId[1] << 8) | (uint32_t)myDevId[2]);	
}

bool FanetMac::setAddr(MacAddr addr)
{
	return false;
}

bool FanetMac::setAddr(uint32_t addr)
{
	_myAddr = MacAddr((addr >> 16) & 0xff, addr & 0xffff);
	return true;
}

FanetMac fmac = FanetMac();

float FanetMac::getAirtime() {
	return radio.get_airlimit();
}

int FanetMac::getTxQueueLength() {
	return tx_fifo.size();
}