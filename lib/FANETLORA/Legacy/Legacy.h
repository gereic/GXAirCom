#include <stdint.h>
#include "../FanetLora.h"
#include "../CRC/lib_crc.h"
#include "../../FLARM/Flarm.h"

#ifndef PROTOCOL_LEGACY_H
#define PROTOCOL_LEGACY_H

#define LEGACY_KEY1 { 0xe43276df, 0xdca83759, 0x9802b8ac, 0x4675a56b, \
                      0xfc78ea65, 0x804b90ea, 0xb76542cd, 0x329dfa32 }
#define LEGACY_KEY2 0x045d9f3b
#define LEGACY_KEY3 0x87b562f4

#define LEGACY_SYNCWORD        {0x99, 0xA5, 0xA9, 0x55, 0x66, 0x65, 0x96}
#define LEGACY_SYNCWORD_SIZE   7
#define RF_PROTOCOL_LEGACY 1

#define _GPS_KMH_PER_KNOT 1.852
#define _GPS_MPH_PER_KNOT 1.15077945
#define _GPS_MPS_PER_KNOT 0.51444444
#define _GPS_KMH_2_MPS 0.277778
#define _GPS_FEET_PER_METER 3.2808399
#define StdOut  Serial


/* FTD-12 Version: 7.00 */
enum
{
	ADDR_TYPE_RANDOM,
	ADDR_TYPE_ICAO,
	ADDR_TYPE_FLARM,
	ADDR_TYPE_ANONYMOUS, /* FLARM stealth, OGN */
	ADDR_TYPE_P3I,
	ADDR_TYPE_FANET
};

enum
{
	TURN_RATE_0,
	TURN_RATE_ON_GROUND, //1 (plane on ground)
	TURN_RATE_2,
	TURN_RATE_3,
	TURN_RATE_RIGHT_TURN, //4 (right turn >14deg)
	TURN_RATE_NO_TURN, //5 (no/slow turn)
	TURN_RATE_6,
	TURN_RATE_LEFT_TURN //7 (left turn >14deg)
};


typedef struct {
    /********************/
    unsigned int addr:24;
    unsigned int zero0:4;
    unsigned int addr_type:3;
    unsigned int zero1:1;
    // unsigned int magic:8;
    /********************/
    int vs:10;
    //unsigned int onGround:1;
    //unsigned int _unk2:1;
    //unsigned int airborne:1;
    unsigned int turnrate:3; //1 (plane on ground), 5 (no/slow turn), 4 (right turn >14deg), 7 (left turn >14deg)
    unsigned int stealth:1;
    unsigned int no_track:1;
    unsigned int parity:1;
    unsigned int gps:12;
    unsigned int aircraft_type:4;
    /********************/
    unsigned int lat:19;
    unsigned int alt:13;
    /********************/
    unsigned int lon:20;
    unsigned int zero2:10;
    unsigned int smult:2;
    /********************/
    int8_t ns[4];
    int8_t ew[4];
    /********************/
} __attribute__((packed)) legacy_packet_t;


typedef struct UFO {
    long    timestamp;
    uint32_t  addr;
    uint8_t   addr_type;
    float     latitude;
    float     longitude;
    float     altitude;
    float     course;     /* CoG */
    float     speed;      /* ground speed in knots */
    uint8_t   aircraft_type;

    float     vs; /* feet per minute */

    bool      stealth;
    bool      no_track;
    bool      onGround;
    bool      airborne;

    int8_t    ns[4];
    int8_t    ew[4];

    float     geoid_separation; /* metres */
} ufo_t;

void createLegacyPkt(FanetLora::trackingData *Data,float geoidAlt,bool onGround,uint8_t * buffer);
extern uint8_t Legacy_Buffer [24];

size_t encrypt_legacy(void *legacy_pkt, long timestamp);
size_t decrypt_legacy(void *legacy_pkt, long timestamp);
int8_t decodeFrame(void *legacy_pkt,uint32_t len,ufo_t *fop);
int8_t legacy_decode(void *legacy_pkt, ufo_t *this_aircraft, ufo_t *fop);
unsigned short getLegacyCkSum(byte* ba, int len);
void legacyLogAircraft(ufo_t *air);
FanetLora::aircraft_t LP_Flarm2FanetAircraft(eFlarmAircraftType aircraft);
class Legacy {
public:

};
#endif /* PROTOCOL_LEGACY_H */