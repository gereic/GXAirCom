#include <math.h>
#include <stdint.h>
#include <Arduino.h> //
#include "FanetLora.h"
#include "Legacy.h"


uint8_t Legacy_Buffer [24];
uint8_t parity(uint32_t x);

void legacyLogAircraft(ufo_t *air){
  log_i("addr=%06X,adrT=%d,airT=%d,lat=%.6f,lon=%.6f,alt=%.2f,speed=%.2f,vario=%.2f",air->addr,air->addr_type,air->aircraft_type,air->latitude,air->longitude,air->altitude,air->speed,air->vs);
}

void btea(uint32_t *v, int8_t n, const uint32_t key[4]) {
    uint32_t y, z, sum;
    uint32_t p, rounds, e;

    #define DELTA 0x9e3779b9
    // #define ROUNDS (6 + 52 / n)
    #define ROUNDS 6
    #define MX (((z >> 5 ^ y << 2) + (y >> 3 ^ z << 4)) ^ ((sum ^ y) + (key[(p & 3) ^ e] ^ z)))

    if (n > 1) {
        /* Coding Part */
        rounds = ROUNDS;
        sum = 0;
        z = v[n - 1];
        do {
            sum += DELTA;
            e = (sum >> 2) & 3;
            for (p = 0; p < n - 1; p++) {
                y = v[p + 1];
                z = v[p] += MX;
            }
            y = v[0];
            z = v[n - 1] += MX;
        } while (--rounds);
    } else if (n < -1) {
        /* Decoding Part */
        n = -n;
        rounds = ROUNDS;
        sum = rounds * DELTA;
        y = v[0];
        do {
            e = (sum >> 2) & 3;
            for (p = n - 1; p > 0; p--) {
                z = v[p - 1];
                y = v[p] -= MX;
            }
            z = v[n - 1];
            y = v[0] -= MX;
            sum -= DELTA;
        } while (--rounds);
    }
}

/* http://pastebin.com/YK2f8bfm */
long obscure(uint32_t key, uint32_t seed) {
    uint32_t m1 = seed * (key ^ (key >> 16));
    uint32_t m2 = (seed * (m1 ^ (m1 >> 16)));
    return m2 ^ (m2 >> 16);
}

static const uint32_t table[8] = LEGACY_KEY1;

void make_key(uint32_t key[4], uint32_t timestamp, uint32_t address) {
    int8_t i, ndx;
    for (i = 0; i < 4; i++) {
        ndx = ((timestamp >> 23) & 1) ? i+4 : i ;
        key[i] = obscure(table[ndx] ^ ((timestamp >> 6) ^ address), LEGACY_KEY2) ^ LEGACY_KEY3;
    }
}

void invert_buffer(uint8_t *buffer,uint32_t len){
    for (int i =0;i<len;i++)
        buffer[i] =~buffer[i];
}

unsigned short getLegacyCkSum(byte* ba, int len)
{
  uint16_t crc16 = 0xffff;  /* seed value */
  crc16 = update_crc_ccitt(crc16, 0x31);
  crc16 = update_crc_ccitt(crc16, 0xFA);
  crc16 = update_crc_ccitt(crc16, 0xB6);
    
  for (int i =0;i<len;i++)
    crc16 = update_crc_ccitt(crc16, (uint8_t)(ba[i]));

 return crc16;    
} 



int8_t decodeFrame(void *legacy_pkt,uint32_t len,ufo_t *fop){
  legacy_packet_t *pkt = (legacy_packet_t *) legacy_pkt;
  int ndx;
  uint8_t pkt_parity=0;

  
  for (ndx = 0; ndx < sizeof (legacy_packet_t); ndx++) {
    pkt_parity += parity(*(((unsigned char *) pkt) + ndx));
  }
  
  int32_t lat = (pkt->lat) % (uint32_t) 0x080000;
  if (lat >= 0x040000) lat -= 0x080000;
  lat = (lat << 7); // + 0x40 
  
  int32_t lon = (pkt->lon) % (uint32_t) 0x080000;
  if (lon >= 0x040000) lon -= 0x080000;
  lon = (lon << 7); // + 0x40 
  log_i("lat=%d,lon=%d",pkt->lat,pkt->lon);

  int32_t ns = (pkt->ns[0] + pkt->ns[1] + pkt->ns[2] + pkt->ns[3]) / 4;
  int32_t ew = (pkt->ew[0] + pkt->ew[1] + pkt->ew[2] + pkt->ew[3]) / 4;
  float speed4 = sqrtf(ew * ew + ns * ns) * (1 << pkt->smult);  

  float direction = 0;
  if (speed4 > 0) {
    direction = atan2f(ns,ew) * 180.0 / PI;  // -180 ... 180 
    // convert from math angle into course relative to north 
    direction = (direction <= 90.0 ? 90.0 - direction :
                                    450.0 - direction);
  }

  uint16_t vs_u16 = pkt->vs;
  int16_t vs_i16 = (int16_t) (vs_u16 | (vs_u16 & (1<<9) ? 0xFC00U : 0));
  int16_t vs10 = vs_i16 << pkt->smult;

  int16_t alt = pkt->alt ; // relative to WGS84 ellipsoid   

  fop->addr = pkt->addr;
  fop->addr_type = pkt->addr_type;
  //fop->timestamp = now();
  fop->latitude = (float)lat / 1e7;
  fop->longitude = (float)lon / 1e7;
  fop->altitude = (float) alt; // - geo_separ;
  fop->speed = speed4 / (4 * _GPS_KMH_2_MPS);
  fop->course = direction;
  //fop->vs = ((float) vs10) * (_GPS_FEET_PER_METER * 6.0);
  fop->vs = ((float) vs10) / 10.0;
  fop->aircraft_type = pkt->aircraft_type;
  fop->stealth = pkt->stealth;
  fop->no_track = pkt->no_track;
  fop->ns[0] = pkt->ns[0]; fop->ns[1] = pkt->ns[1];
  fop->ns[2] = pkt->ns[2]; fop->ns[3] = pkt->ns[3];
  fop->ew[0] = pkt->ew[0]; fop->ew[1] = pkt->ew[1];
  fop->ew[2] = pkt->ew[2]; fop->ew[3] = pkt->ew[3];
 return 0;
}

uint8_t parity(uint32_t x) {
    uint8_t parity=0;
    while (x > 0) {
      if (x & 0x1) {
          parity++;
      }
      x >>= 1;
    }
    return (parity % 2);
}
 

int8_t legacy_decode(void *legacy_pkt, ufo_t *this_aircraft, ufo_t *fop) {

  legacy_packet_t *pkt = (legacy_packet_t *) legacy_pkt;

  float ref_lat = this_aircraft->latitude;
  float ref_lon = this_aircraft->longitude;
  float geo_separ = this_aircraft->geoid_separation;
  uint32_t timestamp = (uint32_t) this_aircraft->timestamp;
  int ndx;
  uint8_t pkt_parity=0;

  
  //log_i("unk0=%02X,unk1=%02X",pkt->_unk0,pkt->_unk1);
  if (pkt->_unk0 != 0){
    log_e("unknown message unk0=%02X",pkt->_unk0);
    return -2;
  }

  for (ndx = 0; ndx < sizeof (legacy_packet_t); ndx++) {
    pkt_parity += parity(*(((unsigned char *) pkt) + ndx));
  }
  
  if (pkt_parity % 2) {
    log_i("bad parity of decoded packet: %02X",pkt_parity % 2);        
    return -1;
  }



  int32_t round_lat = (int32_t) (ref_lat * 1e7) >> 7;
  int32_t lat = (pkt->lat - round_lat) % (uint32_t) 0x080000;
  if (lat >= 0x040000) lat -= 0x080000;
  lat = ((lat + round_lat) << 7) /* + 0x40 */;

  int32_t round_lon = (int32_t) (ref_lon * 1e7) >> 7;
  int32_t lon = (pkt->lon - round_lon) % (uint32_t) 0x100000;
  if (lon >= 0x080000) lon -= 0x100000;
  lon = ((lon + round_lon) << 7) /* + 0x40 */;

  int32_t ns = (pkt->ns[0] + pkt->ns[1] + pkt->ns[2] + pkt->ns[3]) / 4;
  int32_t ew = (pkt->ew[0] + pkt->ew[1] + pkt->ew[2] + pkt->ew[3]) / 4;
  float speed4 = sqrtf(ew * ew + ns * ns) * (1 << pkt->smult);

  float direction = 0;
  if (speed4 > 0) {
    direction = atan2f(ns,ew) * 180.0 / PI;  /* -180 ... 180 */
    /* convert from math angle into course relative to north */
    direction = (direction <= 90.0 ? 90.0 - direction :
                                    450.0 - direction);
  }

  uint16_t vs_u16 = pkt->vs;
  int16_t vs_i16 = (int16_t) (vs_u16 | (vs_u16 & (1<<9) ? 0xFC00U : 0));
  int16_t vs10 = vs_i16 << pkt->smult;

  int16_t alt = pkt->alt ; /* relative to WGS84 ellipsoid */

  fop->addr = pkt->addr;
  fop->addr_type = pkt->addr_type;
  fop->timestamp = timestamp;
  fop->latitude = (float)lat / 1e7;
  fop->longitude = (float)lon / 1e7;
  fop->altitude = (float) alt - geo_separ;
  fop->speed = speed4 / (4 * _GPS_KMH_2_MPS);
  fop->course = direction;
  //log_i("vs10=%d",vs10);
  //fop->vs = ((float) vs10) * (_GPS_FEET_PER_METER * 6.0);
  fop->vs = ((float) vs10) / 10.0;
  fop->aircraft_type = pkt->aircraft_type;
  fop->stealth = pkt->stealth;
  fop->no_track = pkt->no_track;
  fop->ns[0] = pkt->ns[0]; fop->ns[1] = pkt->ns[1];
  fop->ns[2] = pkt->ns[2]; fop->ns[3] = pkt->ns[3];
  fop->ew[0] = pkt->ew[0]; fop->ew[1] = pkt->ew[1];
  fop->ew[2] = pkt->ew[2]; fop->ew[3] = pkt->ew[3];

  return 0;
}

size_t legacy_encode(void *legacy_pkt, ufo_t *this_aircraft) {

    legacy_packet_t *pkt = (legacy_packet_t *) legacy_pkt;

    int ndx;
    uint8_t pkt_parity=0;
    

    uint32_t id = this_aircraft->addr;
    float lat = this_aircraft->latitude;
    float lon = this_aircraft->longitude;
    int16_t alt = (int16_t) (this_aircraft->altitude + this_aircraft->geoid_separation);
 
    float course = this_aircraft->course;
    float speedf = this_aircraft->speed * _GPS_KMH_2_MPS; /* m/s */
    //float vsf = this_aircraft->vs / (_GPS_FEET_PER_METER * 60.0); /* m/s */
    float vsf = this_aircraft->vs ; /* m/s */

    uint16_t speed4 = (uint16_t) roundf(speedf * 4.0f);
    if (speed4 > 0x3FF) {
      speed4 = 0x3FF;
    }

    if        (speed4 & 0x200) {
      pkt->smult = 3;
    } else if (speed4 & 0x100) {
      pkt->smult = 2;
    } else if (speed4 & 0x080) {
      pkt->smult = 1;
    } else {
      pkt->smult = 0;
    }

    uint8_t speed = speed4 >> pkt->smult;

    int8_t ns = (int8_t) (speed * cosf(radians(course)));
    int8_t ew = (int8_t) (speed * sinf(radians(course)));

    int16_t vs10 = (int16_t) roundf(vsf * 10.0f);
    pkt->vs = vs10 >> pkt->smult;

    pkt->addr = id & 0x00FFFFFF;

//pkt->addr_type = ADDR_TYPE_ICAO;
pkt->addr_type = ADDR_TYPE_ANONYMOUS;
//pkt->addr_type = ADDR_TYPE_FLARM;


//#if !defined(SOFTRF_ADDRESS)
//    pkt->addr_type = ADDR_TYPE_ANONYMOUS; /* ADDR_TYPE_ANONYMOUS */
//#else
//    pkt->addr_type = (pkt->addr == SOFTRF_ADDRESS ?
//                      ADDR_TYPE_ICAO : ADDR_TYPE_FLARM); /* ADDR_TYPE_ANONYMOUS */
//#endif

    pkt->parity = 0;

    pkt->stealth = this_aircraft->stealth;
    pkt->no_track = this_aircraft->no_track;

    pkt->aircraft_type = this_aircraft->aircraft_type;

    pkt->gps = 323;

    pkt->lat = (uint32_t ( lat * 1e7) >> 7) & 0x7FFFF;
    pkt->lon = (uint32_t ( lon * 1e7) >> 7) & 0xFFFFF;
    pkt->alt = alt;

    pkt->airborne = speed > 0 ? 1 : 0;
    pkt->ns[0] = ns; pkt->ns[1] = ns; pkt->ns[2] = ns; pkt->ns[3] = ns;
    pkt->ew[0] = ew; pkt->ew[1] = ew; pkt->ew[2] = ew; pkt->ew[3] = ew;

    pkt->_unk0 = 0;
    pkt->_unk1 = 0;
    pkt->_unk2 = 0;
    pkt->_unk3 = 0;
//    pkt->_unk4 = 0;

    for (ndx = 0; ndx < sizeof (legacy_packet_t); ndx++) {
      pkt_parity += parity(*(((unsigned char *) pkt) + ndx));
    }
    pkt->parity = (pkt_parity % 2);
    return (sizeof(legacy_packet_t));
}


size_t encrypt_legacy(void *legacy_pkt, long timestamp)
{
    legacy_packet_t *pkt = (legacy_packet_t *) legacy_pkt;
    uint32_t key[4];
    make_key(key, timestamp , (pkt->addr << 8) & 0xffffff);

#if 0
    Serial.print(key[0]);   Serial.print(", ");
    Serial.print(key[1]);   Serial.print(", ");
    Serial.print(key[2]);   Serial.print(", ");
    Serial.println(key[3]);
#endif
    btea((uint32_t *) pkt + 1, 5, key);

    return (sizeof(legacy_packet_t));
}

size_t decrypt_legacy(void *legacy_pkt, long timestamp)
{
	legacy_packet_t *pkt = (legacy_packet_t *) legacy_pkt;
    uint32_t key[4];
    make_key(key, timestamp, (pkt->addr << 8) & 0xffffff);
#if 0
    Serial.print(key[0]);   Serial.print(", ");
    Serial.print(key[1]);   Serial.print(", ");
    Serial.print(key[2]);   Serial.print(", ");
    Serial.println(key[3]);
#endif
    btea((uint32_t *) pkt + 1, -5, key);
    return (sizeof(legacy_packet_t));
}


eFlarmAircraftType LP_Fanet2FlarmAircraft(FanetLora::aircraft_t aircraft){
  switch (aircraft)
  {
  case FanetLora::aircraft_t::paraglider :
    return eFlarmAircraftType::PARA_GLIDER;
  case FanetLora::aircraft_t::hangglider :
    return eFlarmAircraftType::HANG_GLIDER;
  case FanetLora::aircraft_t::balloon :
    return eFlarmAircraftType::BALLOON;
  case FanetLora::aircraft_t::glider :
    return eFlarmAircraftType::GLIDER_MOTOR_GLIDER;
  case FanetLora::aircraft_t::poweredAircraft :
    return eFlarmAircraftType::TOW_PLANE;
  case FanetLora::aircraft_t::helicopter :
    return eFlarmAircraftType::HELICOPTER_ROTORCRAFT;
  case FanetLora::aircraft_t::uav :
    return eFlarmAircraftType::UAV;
  default:
    return eFlarmAircraftType::UNKNOWN;
  }
}

 FanetLora::aircraft_t LP_Flarm2FanetAircraft(eFlarmAircraftType aircraft){
  switch (aircraft)
  {
  case eFlarmAircraftType::PARA_GLIDER :
    return FanetLora::aircraft_t::paraglider;
  case eFlarmAircraftType::HANG_GLIDER :
    return FanetLora::aircraft_t::hangglider;
  case eFlarmAircraftType::BALLOON :
    return FanetLora::aircraft_t::balloon;
  case eFlarmAircraftType::GLIDER_MOTOR_GLIDER :
    return FanetLora::aircraft_t::glider;
  case eFlarmAircraftType::TOW_PLANE :
    return FanetLora::aircraft_t::poweredAircraft;
  case eFlarmAircraftType::HELICOPTER_ROTORCRAFT :
    return FanetLora::aircraft_t::helicopter;
  case eFlarmAircraftType::UAV :
    return FanetLora::aircraft_t::uav;
  default:
    return FanetLora::aircraft_t::otherAircraft;
  }
}


void createLegacyPkt(FanetLora::trackingData *Data,float geoidAlt,uint8_t * buffer)
{
    FlarmtrackingData FlarmDataData;
    ufo_t air={0};

    //FlarmDataData.heading = Data->heading;
    air.stealth = false;
    air.no_track = false;
    air.aircraft_type=(uint8_t)LP_Fanet2FlarmAircraft(Data->aircraftType);
    air.altitude= Data->altitude;
    air.geoid_separation = geoidAlt;
    air.addr = Data->devId;
    air.latitude=Data->lat;
    air.longitude=Data->lon;
    air.vs=Data->climb;
    //air.vs=-12.3;
    air.speed= Data->speed;
    //air.speed = 37.5;
    //legacyLogAircraft(&air);
    legacy_encode(buffer,&air);

}




/*
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
}*/