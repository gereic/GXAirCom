/*
 * mac.h
 *
 *  Created on: 30 Sep 2016
 *      Author: sid
 */

#ifndef FANET_STACK_FMAC_H_
#define FANET_STACK_FMAC_H_

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <Arduino.h>

//define the pins used by the LoRa transceiver module
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 23
#define DIO0 26

#define FREQUENCY868 868200012
#define FREQUENCY915 916039978

#define ManuId 0x07 // for the moment we use soft-Rf, cause then airwhere is showing it on the map
//#define ManuId 0xFA 



/*
 * Hard coded tx time assumption:
 * -SR7
 * -BW250
 * -CR8 (worst case)
 *
 * payload (byte) -> airtime (ms) -> airtime per byte payload (ms)
 * 0		9.43		0
 * 1		13.44		4.1
 * 2-5		17.54		4-1.63
 * 10		25.73		1.63
 * 64		87.17		1.2
 * 201		246.91		1.18
 * (number according to LoRa calculator)
 *
 * -> tx time assumption:
 * 15ms + 2*payload(bytes)
 * MAC_TX_MINHEADERTIME_MS + (blength * MAC_TX_TIMEPERBYTE_MS)
 */

/*
 * Timing defines
 * ONLY change if you know what you are doing. Can destroy the hole nearby network!
 */

#define MAC_SLOT_MS				20

#define MAC_TX_MINPREAMBLEHEADERTIME_MS		15
#define MAC_TX_TIMEPERBYTE_MS			2
#define MAC_TX_ACKTIMEOUT			1000
#define MAC_TX_RETRANSMISSION_TIME		1000
#define MAC_TX_RETRANSMISSION_RETRYS		3
#define MAC_TX_BACKOFF_EXP_MIN			7
#define MAC_TX_BACKOFF_EXP_MAX			12

#define MAC_FORWARD_MAX_RSSI_DBM		-90		//todo test
#define MAC_FORWARD_MIN_DB_BOOST		20
#define MAC_FORWARD_DELAY_MIN			100
#define MAC_FORWARD_DELAY_MAX			300

#define NEIGHBOR_MAX_TIMEOUT_MS			250000		//4min + 10sek

#define MAC_SYNCWORD				0xF1

/*
 * Number defines
 */
#define MAC_NEIGHBOR_SIZE			64
#define MAC_MAXNEIGHBORS_4_TRACKING_2HOP	5
#define MAC_CODING48_THRESHOLD			8

#define MAC_FIFO_SIZE				8
#define MAC_FRAME_LENGTH			254


/* Debug */
#define MAC_debug_mode				0



//#include "main.h"
#include "lib/LinkedList2.h"
#include "lib/TimerObject.h"

#include "frame.h"

/* note: zero copy stack might be faster and more memory efficient, but who cares @ 9kBaud and 64Ks of ram... */

class NeighborNode
{
private:
	unsigned long last_seen;
public:
	const MacAddr addr;
	bool hasTracking;

	NeighborNode(MacAddr addr, bool tracking = false) : addr(addr), hasTracking(tracking) { last_seen = millis(); }
	void seen(void) { last_seen = millis(); }
	bool isAround(void) { return last_seen + NEIGHBOR_MAX_TIMEOUT_MS > millis(); }
};

class Fapp
{
public:
	Fapp() { }
	virtual ~Fapp() { }

	/* device -> air */
	virtual bool is_broadcast_ready(int num_neighbors) = 0;
	virtual void broadcast_successful(int type) = 0;
	virtual Frame *get_frame() = 0;

	/* air -> device */
	virtual void handle_acked(bool ack, MacAddr &addr) = 0;
	virtual void handle_frame(Frame *frm) = 0;
};

class MacFifo
{
private:
	LinkedList2<Frame*> fifo;
public:
	/* not usable in async mode */
	Frame* get_nexttx();
	Frame* frame_in_list(Frame *frm);
	Frame* front();

	/* usable in async mode */
	bool remove_delete_acked_frame(MacAddr dest);
	bool remove_delete(Frame *frm);
	int add(Frame *frm);
	int size() { return fifo.size(); }
};

class FanetMac
{
private:
	TimerObject myTimer;
	MacFifo tx_fifo;
	MacFifo rx_fifo;
	LinkedList2<NeighborNode *> neighbors;
	Fapp *myApp = NULL;
	MacAddr _myAddr;

	unsigned long csma_next_tx = 0;
	int csma_backoff_exp = MAC_TX_BACKOFF_EXP_MIN;

	/* used for interrupt handler */
	uint8_t rx_frame[MAC_FRAME_LENGTH];
	int num_received = 0;

	static void frameRxWrapper(int length);
	void frameReceived(int length);

	void ack(Frame* frm);

	static void stateWrapper();
	void handleTx();
	void handleRx();

	bool isNeighbor(MacAddr addr);

public:
	bool doForward = true;

	FanetMac() : myTimer(MAC_SLOT_MS, stateWrapper), myAddr(_myAddr) { }
	~FanetMac() { }

	bool begin(Fapp &app,long frequency,uint8_t level);
	void end();
	void handle() { myTimer.Update(); }

	bool txQueueDepleted(void) { return (tx_fifo.size() == 0); }
	bool txQueueHasFreeSlots(void){ return (tx_fifo.size() < MAC_FIFO_SIZE); }
	int transmit(Frame *frm) { return tx_fifo.add(frm); }

	uint16_t numNeighbors(void) { return neighbors.size(); }
	uint16_t numTrackingNeighbors(void);

	/* Addr */
	const MacAddr &myAddr;
	bool setAddr(MacAddr addr);
	bool eraseAddr(void);
	MacAddr readAddr();
};

extern FanetMac fmac;

#endif /* FANET_STACK_FMAC_H_ */
