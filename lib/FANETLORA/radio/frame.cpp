/*
 * frame.cpp
 *
 *  Created on: Sep 29, 2018
 *      Author: sid
 */
#include <stdint.h>
#include <string.h>
#include <cmath>
#include <math.h>
//#include "constrain.h"
#include <Arduino.h>

#include "fmac.h"
#include "frame.h"
#include "macaddr.h"

/*
 * Frame
 */

uint16_t Frame::coord2payload_compressed(float deg)
{
	float deg_round =  round(deg);
	bool deg_odd = ((int)deg_round) & 1;
	const float decimal = deg-deg_round;
	const int dec_int = constrain((int)(decimal*32767), -16383, 16383);

	return ((dec_int&0x7FFF) | (!!deg_odd<<15));
}

void Frame::coord2payload_absolut(float lat, float lon, uint8_t *buf)
{
	if(buf == nullptr)
		return;

	int32_t lat_i = round(lat * 93206.0f);
	int32_t lon_i = round(lon * 46603.0f);

	buf[0] = ((uint8_t*)&lat_i)[0];
	buf[1] = ((uint8_t*)&lat_i)[1];
	buf[2] = ((uint8_t*)&lat_i)[2];

	buf[3] = ((uint8_t*)&lon_i)[0];
	buf[4] = ((uint8_t*)&lon_i)[1];
	buf[5] = ((uint8_t*)&lon_i)[2];
}

int Frame::serialize(uint8_t*& buffer)
{
	if(src.id <= 0 || src.id >= 0xFFFF || src.manufacturer <= 0 || src.manufacturer>=0xFE)
		return -2;

	int blength = MAC_FRM_MIN_HEADER_LENGTH + payload_length;

	/* extended header? */
	if(ack_requested || dest.id != 0 || dest.manufacturer != 0 || signature != 0)
		blength++;

	/* none broadcast frame */
	if(dest.id != 0 || dest.manufacturer != 0)
		blength += MAC_FRM_ADDR_LENGTH;

	/* signature */
	if(signature != 0)
		blength += MAC_FRM_SIGNATURE_LENGTH;

	/* frame to long */
	if(blength > 255)
		return -1;

	/* get memory */
	buffer = new uint8_t[blength];
	int idx = 0;

	/* header */
	buffer[idx++] = !!(ack_requested || dest.id != 0 || dest.manufacturer != 0 || signature != 0)<<MAC_FRM_HEADER_EXTHEADER_BIT |
			!!forward<<MAC_FRM_HEADER_FORWARD_BIT | (type & MAC_FRM_HEADER_TYPE_MASK);
	buffer[idx++] = src.manufacturer & 0x000000FF;
	buffer[idx++] = src.id & 0x000000FF;
	buffer[idx++] = (src.id>>8) & 0x000000FF;

	/* extended header */
	if(buffer[0] & 1<<7)
		buffer[idx++] = (ack_requested & 3)<<MAC_FRM_EXTHEADER_ACK_BIT0 |
				!!(dest.id != 0 || dest.manufacturer != 0)<<MAC_FRM_EXTHEADER_UNICAST_BIT |
				!!signature<<MAC_FRM_EXTHEADER_SIGNATURE_BIT;

	/* extheader and unicast -> add destination addr */
	if((buffer[0] & 1<<7) && (buffer[4] & 1<<5))
	{
		buffer[idx++] = dest.manufacturer & 0x000000FF;
		buffer[idx++] = dest.id & 0x000000FF;
		buffer[idx++] = (dest.id>>8) & 0x000000FF;
	}

	/* extheader and signature -> add signature */
	if((buffer[0] & 1<<7) && (buffer[4] & 1<<4))
	{
		buffer[idx++] = signature & 0x000000FF;
		buffer[idx++] = (signature>>8) & 0x000000FF;
		buffer[idx++] = (signature>>16) & 0x000000FF;
		buffer[idx++] = (signature>>24) & 0x000000FF;
	}

	/* fill payload */
	for(int i=0; i<payload_length && idx<blength; i++)
		buffer[idx++] = payload[i];

	return blength;
}

Frame::Frame(int length, uint8_t *data)
{
	int payload_start = MAC_FRM_MIN_HEADER_LENGTH;

	/* header */
	forward = !!(data[0] & (1<<MAC_FRM_HEADER_FORWARD_BIT));
	type = data[0] & MAC_FRM_HEADER_TYPE_MASK;
	src.manufacturer = data[1];
	src.id = data[2] | (data[3]<<8);

	/* extended header */
	if(data[0] & 1<<MAC_FRM_HEADER_EXTHEADER_BIT)
	{
		payload_start++;

		/* ack type */
		ack_requested = (data[4] >> MAC_FRM_EXTHEADER_ACK_BIT0) & 3;

		/* unicast */
		if(data[4] & (1<<MAC_FRM_EXTHEADER_UNICAST_BIT))
		{
			dest.manufacturer = data[5];
			dest.id = data[6] | (data[7]<<8);

			payload_start += MAC_FRM_ADDR_LENGTH;
		}

		/* signature */
		if(data[4] & (1<<MAC_FRM_EXTHEADER_SIGNATURE_BIT))
		{
			signature = data[payload_start] | (data[payload_start+1]<<8) | (data[payload_start+2]<<16) | (data[payload_start+3]<<24);
			payload_start += MAC_FRM_SIGNATURE_LENGTH;
		}
	}

	/* payload */
	payload_length = length - payload_start;
	if(payload_length > 0)
	{
		payload = new uint8_t[payload_length];
		memcpy(payload, &data[payload_start], payload_length);
	}
}

Frame::Frame()
{
	src = fmac.myAddr;
}

bool Frame::operator== (const Frame& frm) const
{
	if(src != frm.src)
		return false;

	if(dest != frm.dest)
		return false;

	if(type != frm.type)
		return false;

	if(payload_length != frm.payload_length)
		return false;

	for(int i=0; i<payload_length; i++)
		if(payload[i] != frm.payload[i])
			return false;

	return true;
}

