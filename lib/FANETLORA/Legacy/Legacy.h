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


typedef struct {
    /********************/
    unsigned int addr:24;
    unsigned int _unk0:4;
    unsigned int addr_type:3;
    unsigned int _unk1:1;
    // unsigned int magic:8;
    /********************/
    int vs:10;
    unsigned int _unk2:2;
    unsigned int airborne:1;
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
    unsigned int _unk3:10;
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

    int8_t    ns[4];
    int8_t    ew[4];

    float     geoid_separation; /* metres */
} ufo_t;

void createLegacyPkt(FanetLora::trackingData *Data,float geoidAlt,uint8_t * buffer);
extern uint8_t Legacy_Buffer [24];

size_t encrypt_legacy(void *legacy_pkt, long timestamp);
size_t decrypt_legacy(void *legacy_pkt, long timestamp);
int8_t decodeFrame(void *legacy_pkt,uint32_t len,ufo_t *fop);
bool legacy_decode(void *legacy_pkt, ufo_t *this_aircraft, ufo_t *fop);
unsigned short getLegacyCkSum(byte* ba, int len);
void legacyLogAircraft(ufo_t *air);
FanetLora::aircraft_t LP_Flarm2FanetAircraft(eFlarmAircraftType aircraft);
class Legacy {
public:

};
#endif /* PROTOCOL_LEGACY_H */