/*
 * frame.h
 *
 *  Created on: Sep 29, 2018
 *      Author: sid
 */

#ifndef FANET_RADIO_LIB_FRAME_H_
#define FANET_RADIO_LIB_FRAME_H_

/*
 * Frame
 */
#define MAC_FRM_MIN_HEADER_LENGTH		4
#define MAC_FRM_ADDR_LENGTH			3
#define MAC_FRM_SIGNATURE_LENGTH		4

/* Header Byte */
#define MAC_FRM_HEADER_EXTHEADER_BIT		7
#define MAC_FRM_HEADER_FORWARD_BIT		6
#define MAC_FRM_HEADER_TYPE_MASK		0x3F

/* Extended Header Byte */
#define MAC_FRM_EXTHEADER_ACK_BIT1		7
#define MAC_FRM_EXTHEADER_ACK_BIT0		6
#define MAC_FRM_EXTHEADER_UNICAST_BIT		5
#define MAC_FRM_EXTHEADER_SIGNATURE_BIT		4
//bits 3-0 reserved

#define FRM_NOACK				0
#define FRM_ACK_SINGLEHOP			1
#define FRM_ACK_TWOHOP				2

/* frame Types */
#define FRM_TYPE_ACK				0
#define FRM_TYPE_TRACKING			1
#define FRM_TYPE_NAME				2
#define FRM_TYPE_MESSAGE			3
#define FRM_TYPE_SERVICE			4
#define FRM_TYPE_LANDMARK			5
#define FRM_TYPE_REMOTECONFIG		6
#define FRM_TYPE_GROUNDTRACKING		7
#define FRM_TYPE_GROUNDTRACKING_LEGACY	254
#define FRM_TYPE_TRACKING_LEGACY	255

#include "macaddr.h"

class Frame
{
public:
	/* general stuff */
	static uint16_t coord2payload_compressed(float deg);
	static void coord2payload_absolut(float lat, float lon, uint8_t *buf);

	/* addresses */
	MacAddr src;
	MacAddr dest;

	//note: ack and forwards (also geo based) will be handled by mac layer
	int ack_requested = FRM_NOACK;
	bool forward = false;

	uint32_t signature = 0;

	/* payload */
	int type = 0;
	int payload_length = 0;
	uint8_t *payload = nullptr;

	/* Transmit stuff */
	int num_tx = 0;
	unsigned long next_tx = 0;					//used for backoff

	/* Received stuff */
	int rssi = 0;
	int snr = 0;
	float altitude = 0.0;
	uint8_t legacyAircraftType = 0;
	uint8_t AddressType = 0; //address-Type of Flarm
	uint32_t timeStamp = 0; //unix-Timestamp from Legacy-Msg

	int serialize(uint8_t*& buffer);

	Frame(MacAddr addr) : src(addr) { }
	Frame();
	Frame(int length, uint8_t *data);				// deserialize packet
	~Frame() { delete [] payload; }

	bool operator== (const Frame& frm) const;
};


#endif /* FANET_RADIO_LIB_FRAME_H_ */
