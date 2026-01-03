/*!
 * @file FlarmRadio.h
 *
 *
 */

#ifndef __FLARMRADIO_H__
#define __FLARMRADIO_H__

#include <inttypes.h>
#include <math.h>
#include <stdlib.h>
#include <Arduino.h>
#include "lib_crc.h"

#define DEBUG_FLARM_ERRORS


/*
#ifndef M_PI
  #define M_PI		3.14159265358979323846
#endif
#ifndef min
  #define min(a, b) (((a) < (b)) ? (a) : (b))
#endif
*/

#define MAXSTRING 255

#ifdef FLARMLOGGER
extern SemaphoreHandle_t FlarmLogQueue;
#endif

#define _GPS_KMH_2_MPS 0.277778
#define MS_KMH 3.6


#define FLARM_KEY1 { 0xe43276df, 0xdca83759, 0x9802b8ac, 0x4675a56b, \
                      0xfc78ea65, 0x804b90ea, 0xb76542cd, 0x329dfa32 }
#define FLARM_KEY2 0x045d9f3b
#define FLARM_KEY3 0x87b562f4

#define FLARM_KEY4 0x956F6C77
#define FLARM_KEY5 { 0xA5F9B21C, 0xAB3F9D12, 0xC6F34E34, 0xD72FA378 }

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



typedef struct {
    /********************/
    unsigned int addr:24;
    unsigned int type:4; /* 2 new protocol, 0 old protocol, 1 other messages */
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
} __attribute__((packed)) flarm_packet_t;


typedef struct {
    /********************/
    unsigned int addr:24;
    unsigned int type:4;     /* 2 new protocol, 0 old protocol, 1 other messages */
    unsigned int addr_type:3;
    unsigned int _unk1:1;
    /********************/
    unsigned int _unk2:22;   /* always 0 ? */
    unsigned int stealth:1;
    unsigned int no_track:1;
    unsigned int _unk3:2;    /* always 1 ? */
    unsigned int _unk4:2;    /* always 0 ? */
    unsigned int _unk5:2;    /* always 1 ? */
    unsigned int _unk6:2;    /* always 0 ? */
    /********************/
    unsigned int _unk7:2;    /* always 0 ? */
    unsigned int tstamp:4;   /* the LS 4 bits of timestamp (unix epoch) */
    unsigned int aircraft_type:4;
    unsigned int _unk8:1;    /* always 0 ? */
    unsigned int alt:13;     /* meters + 1000, enscaled(12,1) */

    unsigned int lat:20;     /* rounded and with MS bits removed */
    unsigned int lon:20;     /* rounded and with MS bits removed */
    int          turn:9;     /* degs/sec times 20, enscaled(6,2) */
    unsigned int hs:10;      /* m/s times 10, enscaled(8,2) */
    int          vs:9;       /* m/s times 10, enscaled(6,2) */
    unsigned int course:10;  /* degrees (0-360) times 2 */
    unsigned int airborne:2; /* 1 when stationary, 2 when moving, 3 when circling */

    unsigned int hp:6;       /* meters times 10, enscaled(3,3) */
    unsigned int vp:5;       /* meters times  4, enscaled(2,3) */
    unsigned int _unk9:5;
    unsigned int _unk10:8;   /* always 0 ? */
    /********************/
} __attribute__((packed)) flarm_v7_packet_t;


typedef struct {
    uint8_t identifier[3];
    uint8_t addressType; // Address type 0=Stateless Random, 1=Official ICAO, 2=stableFlarm
    int type; // 1=Glider, ...
    int thre_m_per_sec; // typ. 2m/s
    int no_tracking_mode; // 0=off
    int private_mode; // 0=off
    int airborne_mode; // 0=calc, 1=onGround, 2=inAir
} AircraftConfig;

typedef struct {
    int32_t lat_deg_e7; // Latitude
    int32_t lon_deg_e7; // Longitude
    int32_t height_m; // Height above Ellipsoid
    uint32_t hacc_cm; // Horizontal Accuracy Estimate
    uint32_t vacc_cm; // Vertical Accuracy Estimate
    int32_t vel_n_cm_s; // NED north velocity
    int32_t vel_e_cm_s; // NED east velocity
    int32_t vel_u_cm_s; // NED up! velocity
    uint32_t gspeed_cm_s; // Ground Speed (2-D)
    int32_t heading_deg_e1; // Heading 2-D
    uint32_t sacc_cm_s; // Speed Accuracy Estimate
    uint32_t cacc_deg_e1; // Course / Heading Accuracy Estimate
} GpsData;

typedef struct {
    AircraftConfig *config;
    GpsData *gps;
    int turn_state; // 1=onGround, 5=no, 4=rightTurn, 7=leftTurn
    int is_airborne; // 0=onGround, 1=airborne
    int32_t extrapolated_lat_deg_e7;
    int32_t extrapolated_lon_deg_e7;
    int32_t extrapolated_heading_deg_e1;
    uint32_t extrapolated_height_m; // 0 instead of negative
    int32_t heading_deg_e1;
    int32_t delta_heading_deg_e1;
    int32_t filtered_delta_heading_deg_e1;
    int32_t vel_u_filtered_cm_s; // up
    uint32_t gspeed_filtered_cm_s;
    int airborne_ctr; // -2..0..10
    int old_is_airborne;
    int32_t old_heading_deg_e1;
    int speed_scaling; // 0,1,2,3
    uint8_t hacc_m; // 0..62
    uint8_t vacc_m; // 0..62
    // extrapolated 4s time slots 2-5s, 6-9s, ...
    uint8_t extrapolated_avg_vel_n[4];
    uint8_t extrapolated_avg_vel_e[4];
} AircraftState;
uint8_t flarm_get_zone(float lat, float lon);
void flarm_getFrequencyChannels(uint8_t zone,uint32_t *frequency,uint32_t *ChanSepar, uint8_t *channels,int8_t *MaxTxPower);
uint32_t flarm_calculate_freq_channel(uint32_t timestamp, uint32_t nch);
void flarm_init_aircraft_state(AircraftState *aircraft);
void flarm_update_aircraft_state(AircraftState *aircraft);
void flarm_update_position_speed_direction(AircraftState *aircraft);
void flarm_update_airborne(AircraftState *aircraft);
void flarm_update_turn_state(AircraftState *aircraft);
void flarm_calculate_speed_scaling(AircraftState *aircraft);
void flarm_update_height(AircraftState *aircraft);
void flarm_update_accuracy(AircraftState *aircraft);
void flarm_extrapolate_velocity_vector(AircraftState *aircraft);
size_t flarm_decrypt(void *flarm_pkt, long timestamp);
unsigned short flarm_getCkSum(byte* ba, int len);

bool flarm_decode(void *flarm_pkt, ufo_t *this_aircraft, ufo_t *fop);
size_t flarm_encode(void *flarm_pkt, AircraftState *aircraft, long timestamp);

void flarm_v7_debugBuffer(uint8_t *flarm_pkt,ufo_t *this_aircraft);
void flarm_debugLog(uint32_t tPps,uint8_t *flarm_pkt,ufo_t *this_aircraft);
void flarm_debugAircraft(ufo_t *air,ufo_t *this_aircraft);

#endif
