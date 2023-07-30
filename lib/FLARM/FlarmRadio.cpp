/*!
 * @file FlarmRadio.cpp
 *
 *
 */

#include "FlarmRadio.h"

uint8_t flarm_get_zone(float lat, float lon){
/*
- Zone 1: Europe, Africa, Russia, China (30W to 110E, excl. zone 5)
- Zone 2: North America (west of 30W, north of 10N)
- Zone 3: New Zealand (east of 160E)
- Zone 4: Australia (110E to 160E)
- Zone 5: Israel (34E to 54E and 29.25N to 33.5N)
- Zone 6: South America (west of 30W, south of 10N)
*/

  if (34.0f <= lon && lon <= 54.0f && 29.25f <= lat && lat <= 33.5f){
    log_i("Zone 5: Israel (34E to 54E and 29.25N to 33.5N)");
    return 5; //Zone 5: Israel (34E to 54E and 29.25N to 33.5N)
  }	else if (-30.0f <= lon && lon <= 110.0f){
    log_i("Zone 1: Europe, Africa, Russia, China (30W to 110E, excl. zone 5)");
    return 1; //Zone 1: Europe, Africa, Russia, China (30W to 110E, excl. zone 5)
  }	else if (lon < -30.0f && 10.0f < lat){
    log_i("Zone 2: North America (west of 30W, north of 10N)");
    return 2; //Zone 2: North America (west of 30W, north of 10N)
  }	else if (160.0f < lon){
    log_i("Zone 3: New Zealand (east of 160E)");
    return 3; //Zone 3: New Zealand (east of 160E)
  }	else if (110.0f <= lon && lon <= 160.0f){
    log_i("Zone 4: Australia (110E to 160E)");
    return 4; //Zone 4: Australia (110E to 160E)
  }	else if (lon < -30.0f && lat < 10.0f){
    log_i("Zone 6: South America (west of 30W, south of 10N)");
    return 6; //Zone 6: South America (west of 30W, south of 10N)
  } 
  log_i("Zone not detected lat=%.6f, lon=%.6f",lat,lon);
  return 0; //not defined
}

void flarm_getFrequencyChannels(uint8_t zone,float *frequency, uint8_t *channels){
  if ((zone < 1) || (zone > 6)) return; //zone not defined
/*
- Zone 1: f0=868.2, f1=868.4
- Zone 2, 3, 5, 6: f0=902.2, nch=65
- Zone 4: f0=917.0, nch=24
*/
  if (zone == 1){
    //Zone 1: f0=868.2, f1=868.4
    *frequency = 868.2;
    *channels = 0;
  }else if (zone == 4){
    //Zone 4: f0=917.0, nch=24
    *frequency = 917.0;
    *channels = 24;
  }else{
    //Zone 2, 3, 5, 6: f0=902.2, nch=65
    *frequency = 902.2;
    *channels = 65;
  }
}

uint32_t flarm_calculate_freq_channel(uint32_t timestamp, uint32_t nch) {
  uint32_t nts = ~timestamp;
  uint32_t ts16 = timestamp * uint32_t(32768) + nts;
  uint32_t v4096 = (ts16 >> 12) ^ ts16;
  uint32_t v5 = uint32_t(5) * v4096;
  uint32_t v16 = (v5 >> uint32_t(4)) ^ v5;
  uint32_t v2057 = uint32_t(2057) * v16;
  uint32_t v9 = (v2057 >> uint32_t(16)) ^ v2057;
  //log_i("%04X,%04X,%04X,%04X,%04X,%04X,%04X,%04X",timestamp,nts,ts16,v4096,v5,v16,v2057,v9);
  return v9 % nch;
}

void flarm_init_aircraft_state(AircraftState *aircraft) {
  aircraft->turn_state = 1;
  aircraft->is_airborne = 0;
  aircraft->extrapolated_lat_deg_e7 = 0;
  aircraft->extrapolated_lon_deg_e7 = 0;
  aircraft->extrapolated_heading_deg_e1 = 0;
  aircraft->extrapolated_height_m = 0;
  aircraft->heading_deg_e1 = 0;
  aircraft->delta_heading_deg_e1 = 0;
  aircraft->filtered_delta_heading_deg_e1 = 0;
  aircraft->vel_u_filtered_cm_s = 0;
  aircraft->gspeed_filtered_cm_s = 0;
  aircraft->airborne_ctr = 0;
  aircraft->old_heading_deg_e1 = 0;
  aircraft->speed_scaling = 0;
  aircraft->hacc_m = 0;
  aircraft->vacc_m = 0;
  for (int i = 0; i < 4; i++) {
    aircraft->extrapolated_avg_vel_n[i] = 0;
    aircraft->extrapolated_avg_vel_e[i] = 0;
  }
}

void flarm_update_aircraft_state(AircraftState *aircraft) {
  flarm_update_position_speed_direction(aircraft);
  flarm_update_airborne(aircraft);
  flarm_update_turn_state(aircraft);    
  flarm_calculate_speed_scaling(aircraft);
  flarm_update_height(aircraft);
  flarm_update_accuracy(aircraft);
  flarm_extrapolate_velocity_vector(aircraft);
}

void flarm_create_packet(AircraftState *aircraft, uint8_t *packet) {
  flarm_packet_t *pkt = (flarm_packet_t *) packet;
  uint32_t lat0 = *(uint32_t*)&aircraft->extrapolated_lat_deg_e7;
  uint32_t lat128 = lat0 >> 7;
  if (lat0 & 0x40) {
    lat128 += 1;
  }
  lat128 &= 0x7ffff;
  
  uint32_t lon0 = *(uint32_t*)&aircraft->extrapolated_lon_deg_e7;
  uint32_t lon128 = lon0 >> 7;
  if (lon0 & 0x40) {
    lon128 += 1;
  }
  lon128 &= 0xfffff;

  // Fill bytes 0 to 23 of the packet with data.
  packet[0] = aircraft->config->identifier[0];
  packet[1] = aircraft->config->identifier[1];
  packet[2] = aircraft->config->identifier[2];
  packet[3] = (aircraft->config->addressType << 4) & 0x30; // Shift left and ensure within range
  // Scale the vertical speed; scaled result can be in the range -512..511.
  // 511 corresponds to 51.1m/s (scaling=0) ... 408.8m/s (scaling=3).
  int velUScaled = aircraft->gps->vel_u_cm_s / 10;
  for (int i = 0; i < aircraft->speed_scaling; i++) {
    velUScaled /= 2;
  }
  packet[4] = (uint8_t)(velUScaled & 0xff);
  packet[5] = (aircraft->config->no_tracking_mode ? 0x40 : 0x00)
          | (aircraft->config->private_mode ? 0x20 : 0x00)
          | (aircraft->turn_state << 2)
          | ((velUScaled >> 8) & 0x03);
  // Horizontal and vertical accuracy are cropped at 62m.
  packet[6] = ((aircraft->vacc_m & 0x03) << 6) | aircraft->hacc_m;
  packet[7] = (aircraft->config->type << 4) | (aircraft->vacc_m >> 2);
  packet[8] = (uint8_t)(lat128 & 0xff);
  packet[9] = (uint8_t)((lat128 >> 8) & 0xff);
  uint32_t h = aircraft->extrapolated_height_m;
  if (h >= 8192){
    h = 0x1fff;
  }
  packet[10] = (uint8_t)((h & 0x1f) << 3)
            | (uint8_t)((lat128 >> 16) & 0x07);
  packet[11] = (uint8_t)(h >> 5);
  packet[12] = (uint8_t)(lon128 & 0xff);
  packet[13] = (uint8_t)((lon128 >> 8) & 0xff);
  packet[14] = (uint8_t)((lon128 >> 16) & 0x0f);
  packet[15] = (uint8_t)(aircraft->speed_scaling << 6);
  for (int i = 0; i < 4; i++) {
    packet[16 + i] = aircraft->extrapolated_avg_vel_n[i];
    packet[20 + i] = aircraft->extrapolated_avg_vel_e[i];
  }
  //create parity for packet
  int ndx;
  uint8_t pkt_parity=0;
  //create parity for packet
  for (ndx = 0; ndx < sizeof (flarm_packet_t); ndx++) {
    pkt_parity += flarm_parity(*(((unsigned char *) pkt) + ndx));
  }
  pkt->parity = (pkt_parity % 2);
}

uint8_t flarm_parity(uint32_t x) {
  uint8_t parity=0;
  while (x > 0) {
    if (x & 0x1) {
      parity++;
    }
    x >>= 1;
  }
  return (parity % 2);
}
 


void flarm_update_position_speed_direction(AircraftState *aircraft) {
  // Extrapolate the lattitude.
  float lat = (float)aircraft->gps->lat_deg_e7;
  float latRad = lat / 1e7 / 180.0 * M_PI;
  float cosLat = cosf(latRad);
  float absCosLat = fabsf(cosLat);
  float dLat = (float)aircraft->gps->vel_n_cm_s * 1.8; // cm/s to km/h*50
  float newLat = lat + dLat;
  aircraft->extrapolated_lat_deg_e7 = (int32_t)floorf(newLat);
  
  // Extrapolate the longitude.
  float lon = (float)aircraft->gps->lon_deg_e7;
  float velE = (float)aircraft->gps->vel_e_cm_s;
  float dLon = (velE * 1.8) / absCosLat;
  aircraft->extrapolated_lon_deg_e7 = (int32_t)floorf(lon + dLon);

  // Heading
  if (aircraft->gps->gspeed_cm_s >= 101
    && aircraft->gps->cacc_deg_e1 < 300) {
    aircraft->heading_deg_e1 = aircraft->gps->heading_deg_e1;
  }
  
  // Ground speed, low-pass filtered
  uint32_t prevGspeed = aircraft->gspeed_filtered_cm_s;
  uint32_t curGspeed = aircraft->gps->gspeed_cm_s;
  uint32_t newGspeed = (7 * prevGspeed + curGspeed) / 8;
  aircraft->gspeed_filtered_cm_s = newGspeed;

  // Vertical speed, low-pass filtered
  int32_t prevVelU = aircraft->vel_u_filtered_cm_s;
  int32_t curVelU = aircraft->gps->vel_u_cm_s;
  int32_t newVelU = (7 * prevVelU + curVelU) / 8;
  aircraft->vel_u_filtered_cm_s = newVelU;
  
  // Heading difference
  int32_t prevHeading = aircraft->old_heading_deg_e1;
  int32_t curHeading = aircraft->heading_deg_e1;
  int32_t deltaHeading = curHeading - prevHeading;
  if (deltaHeading >= 1801) {
    deltaHeading -= 3600;
  } else if (deltaHeading < -1799) {
    deltaHeading += 3600;
  }
  aircraft->delta_heading_deg_e1 = deltaHeading;
  
  // Only update the filtered heading difference if the current heading
  // difference is 50 degrees or less, otherwise use the filtered value
  // as current heading difference.
  int32_t absDeltaHeading = abs(deltaHeading);
  if (absDeltaHeading >= 501) {
    aircraft->delta_heading_deg_e1 = aircraft->filtered_delta_heading_deg_e1;
  } else {
    aircraft->filtered_delta_heading_deg_e1 =
      (7 * aircraft->filtered_delta_heading_deg_e1 + aircraft->delta_heading_deg_e1) / 8;
  }
  
  // Remember the current heading for the next iteration.
  aircraft->old_heading_deg_e1 = aircraft->heading_deg_e1;
  
  int32_t curDeltaHeading = aircraft->delta_heading_deg_e1;
  int32_t extrapolatedHeading = aircraft->gps->heading_deg_e1 + curDeltaHeading;
  if (extrapolatedHeading > 3600) {
    extrapolatedHeading -= 3600;
  }
  aircraft->extrapolated_heading_deg_e1 = extrapolatedHeading;
}

void flarm_update_airborne(AircraftState *aircraft) {
  int airborneMode = aircraft->config->airborne_mode;
  if (airborneMode < 0 || airborneMode > 2) {
    return;
  }
  if (airborneMode == 2) {
    aircraft->is_airborne = 1;
    return;
  }
  if (airborneMode == 1) {
    aircraft->is_airborne = 0;
    return;
  }
  
  int configThres = aircraft->config->thre_m_per_sec;
  int speedThresholdCmPerSec = aircraft->gps->sacc_cm_s + 100 * configThres;
  
  // Fast aircraft, or config threshold 0; set is_airborne after a while.
  if (configThres == 0
    || aircraft->gps->gspeed_cm_s >= speedThresholdCmPerSec
    || abs(aircraft->gps->vel_u_cm_s) >= 2 * speedThresholdCmPerSec) {
      
    // Fast or no threshold configured; set isAirborne after a while.
    if (aircraft->airborne_ctr < 0) {
      aircraft->airborne_ctr++;
    } else {
      aircraft->airborne_ctr = 10;
      aircraft->is_airborne = 1;
    }
    return;
  }
  
  // Slow aircraft, config threshold not 0; clear is_airborne after a while.
  if (aircraft->airborne_ctr > 0) {
    aircraft->airborne_ctr--;
  } else {
    aircraft->airborne_ctr = -2;
    aircraft->is_airborne = 0;
  }
}

void flarm_update_turn_state(AircraftState *aircraft) {
  if (!aircraft->is_airborne) {
    aircraft->turn_state = 1;
    return;
  }
  // No turn if this is not a glider or if moving faster than 36m/s.
  if (aircraft->config->type != 1 || aircraft->gspeed_filtered_cm_s > 3600) {
    aircraft->turn_state = 5;
    return;
  }
  // The aircraft is a glider, check if it's making a turn of 14 degrees or more.
  int dh = aircraft->delta_heading_deg_e1;
  if (dh > -140 && dh < 140) {
    aircraft->turn_state = 5;
    return;
  }
  // 4 for right turn, 7 for left turn
  aircraft->turn_state = dh < 0 ? 7 : 4;
}

void flarm_calculate_speed_scaling(AircraftState *aircraft) {
  int velU = aircraft->gps->vel_u_cm_s; // abs?
  int gspeed = aircraft->gps->gspeed_cm_s;
  if (velU < 2400 && gspeed < 3000) {
    // < 86.4 km/h, < 108 km/h
    aircraft->speed_scaling = 0;
  } else if (velU < 4800 && gspeed < 6200) {
    // < 172.8 km/h, < 223.2 km/h
    aircraft->speed_scaling = 1;
  } else if (velU < 10000 && gspeed < 12400) {
    // < 360.0 km/h, < 446.4 km/h
    aircraft->speed_scaling = 2;
  } else {
    aircraft->speed_scaling = 3;
  }
}

void flarm_update_height(AircraftState *aircraft) {
  int32_t velU = aircraft->gps->vel_u_cm_s;
  int32_t deltaHeight = velU / 50; // m/2s
  int32_t height = aircraft->gps->height_m;
  int32_t newHeight = height + deltaHeight;
  if (newHeight < 0) {
    newHeight = 0;
  }
  aircraft->extrapolated_height_m = (uint32_t)newHeight;
}

void flarm_update_accuracy(AircraftState *aircraft) {
  int hacc = aircraft->gps->hacc_cm / 100;
  aircraft->hacc_m = min(62, hacc);
  int vacc = aircraft->gps->vacc_cm / 100;
  aircraft->vacc_m = min(62, vacc);
}

// Intermediate extrapolated velocity vectors.
#define VEL_XPL_CNT 18
int16_t vel_e_i[VEL_XPL_CNT];
int16_t vel_n_i[VEL_XPL_CNT];

void flarm_extrapolate_velocity_vector(AircraftState *aircraft) {
  int32_t deltaHeading = aircraft->delta_heading_deg_e1;
  
  // Convert delta heading from decidegrees to radian.
  float deltaHeadingRad = (float)deltaHeading / 10 / 180 * M_PI;
  float dhSin = sin(deltaHeadingRad);
  float dhCos = cos(deltaHeadingRad);
  
  // Initial velocity vector.
  float curVelN = (float)aircraft->gps->vel_n_cm_s;
  float curVelE = (float)aircraft->gps->vel_e_cm_s;

  // Extrapolate future velocity vectors by applying the delta heading.
  for (int i = 0; i < VEL_XPL_CNT; i++) {
    // Rotate curVel by delta heading to get newVel.
    float newVelN = curVelN * dhCos - curVelE * dhSin;
    float newVelE = curVelN * dhSin + curVelE * dhCos;

    // Store newVel in the corresponding slot.
    vel_n_i[i] = (int16_t)newVelN;
    vel_e_i[i] = (int16_t)newVelE;

    // Use newVel as start vector for the next iteration.
    curVelN = newVelN;
    curVelE = newVelE;
  }
  
  // Average the extrapolated velocity vectors to get 4 vectors at t=2-5s, t=6-9s, ...
  int ix = 1; // 1 to skip t=1s and start at t=2s
  for (int i = 0; i < 4; i++) {      
    // Average over groups of 4 extrapolated velocity vectors.
    int avgVelN = 0;
    int avgVelE = 0;
    for (int j = 0; j < 4; j++) {
      avgVelN += vel_n_i[ix] / 2; // summing 4x cm/s gives cm/4s, /2
      avgVelE += vel_e_i[ix] / 2;
      ix++;
    }
    
    // Scale and store averages for transmission in co-operation message.
    avgVelN /= 50; // from cm/4s/2 to m/4s
    avgVelE /= 50;
    for (int j = 0; j < aircraft->speed_scaling; j++) {
      avgVelN /= 2;
      avgVelE /= 2;
    }
    aircraft->extrapolated_avg_vel_n[i] = (uint8_t)avgVelN;
    aircraft->extrapolated_avg_vel_e[i] = (uint8_t)avgVelE;
  }
}

/* http://pastebin.com/YK2f8bfm */
long flarm_obscure(uint32_t key, uint32_t seed) {
  uint32_t m1 = seed * (key ^ (key >> 16));
  uint32_t m2 = (seed * (m1 ^ (m1 >> 16)));
  return m2 ^ (m2 >> 16);
}

static const uint32_t flarm_table[8] = FLARM_KEY1;


void flarm_make_key(uint32_t key[4], uint32_t timestamp, uint32_t address) {
  int8_t i, ndx;
  for (i = 0; i < 4; i++) {
    ndx = ((timestamp >> 23) & 1) ? i+4 : i ;
    key[i] = flarm_obscure(flarm_table[ndx] ^ ((timestamp >> 6) ^ address), FLARM_KEY2) ^ FLARM_KEY3;
  }
}

void flarm_btea(uint32_t *v, int8_t n, const uint32_t key[4]) {
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



size_t flarm_encrypt(void *flarm_pkt, long timestamp)
{
  flarm_packet_t *pkt = (flarm_packet_t *) flarm_pkt;
  uint32_t key[4];
  flarm_make_key(key, timestamp , (pkt->addr << 8) & 0xffffff);
  flarm_btea((uint32_t *) pkt + 1, 5, key);

  return (sizeof(flarm_packet_t));
}



size_t flarm_decrypt(void *flarm_pkt, long timestamp)
{
  flarm_packet_t *pkt = (flarm_packet_t *) flarm_pkt;
  uint32_t key[4];
  flarm_make_key(key, timestamp, (pkt->addr << 8) & 0xffffff);
  flarm_btea((uint32_t *) pkt + 1, -5, key);
  return (sizeof(flarm_packet_t));
}

unsigned short flarm_getCkSum(byte* ba, int len)
{
  uint16_t crc16 = 0xffff;  /* seed value */
  crc16 = update_crc_ccitt(crc16, 0x31);
  crc16 = update_crc_ccitt(crc16, 0xFA);
  crc16 = update_crc_ccitt(crc16, 0xB6);
    
  for (int i =0;i<len;i++)
    crc16 = update_crc_ccitt(crc16, (uint8_t)(ba[i]));

 return crc16;    
} 

int8_t flarm_decode(void *flarm_pkt, ufo_t *this_aircraft, ufo_t *fop) {

  flarm_packet_t *pkt = (flarm_packet_t *) flarm_pkt;

  float ref_lat = this_aircraft->latitude;
  float ref_lon = this_aircraft->longitude;
  float geo_separ = this_aircraft->geoid_separation;
  uint32_t timestamp = (uint32_t) this_aircraft->timestamp;
  int ndx;
  uint8_t pkt_parity=0;

  //check parity of frame !!
  for (ndx = 0; ndx < sizeof (flarm_packet_t); ndx++) {
    pkt_parity += flarm_parity(*(((unsigned char *) pkt) + ndx));
  }
  if (pkt_parity % 2) {
    log_i("bad parity of decoded packet: %02X",pkt_parity % 2);        
    //char Buffer[500];	
    //sprintf(Buffer,"adr=%06X;adrType=%d,lat=%.06f,lon=%.06f,alt=%.01f,speed=%.01f,course=%.01f,climb=%.01f\n", fop->addr,fop->addr_type,fop->latitude,fop->longitude,fop->altitude,fop->speed,fop->course,fop->vs);
    //log_e("%s",&Buffer[0]);
    return -1;
  }

  
  if (pkt->addr == 0){
    log_e("addr = 0");
    return -8;    
  }
  if (pkt->aircraft_type == 0){
    //log_e("aircraft_type = 0");
    return -9;    
  }
  if (pkt->zero0 != 0){
    log_e("unknown message zero0=%02X",pkt->zero0);
    return -10;
  }
  if (pkt->zero1 != 0){
    log_e("unknown message zero1=%02X",pkt->zero1);
    return -11;
  }
  if (pkt->zero2 != 0){
    //log_e("unknown message zero2=%02X",pkt->zero2);
    return -13;
  }
  //log_i("%d unk0=%02X,unk1=%02X,unk2=%02X,unk3=%02X,onground=%02X,AirBorne=%d",millis(),pkt->zero0,pkt->zero1,pkt->_unk2,pkt->zero2,pkt->onGround,pkt->airborne);

  //Serial.printf("onGround=%d,unk0=%d,unk1=%d,unk2=%d,unk3=%d\r\n",pkt->onGround,pkt->_unk0,pkt->_unk1,pkt->_unk2,pkt->zero2);

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

  //log_i("vs=%d,mult=%d",pkt->vs,pkt->smult);
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
  if (pkt->turnrate == TURN_RATE_ON_GROUND){
    fop->onGround = true;
    fop->airborne = false;
  }else{
    fop->onGround = false;
    fop->airborne = true;
  }
  fop->stealth = pkt->stealth;
  fop->no_track = pkt->no_track;
  fop->ns[0] = pkt->ns[0]; fop->ns[1] = pkt->ns[1];
  fop->ns[2] = pkt->ns[2]; fop->ns[3] = pkt->ns[3];
  fop->ew[0] = pkt->ew[0]; fop->ew[1] = pkt->ew[1];
  fop->ew[2] = pkt->ew[2]; fop->ew[3] = pkt->ew[3];

  /*
  if (pkt->parity != 0){
    log_i("bad parity of decoded packet");
    char Buffer[500];	
    sprintf(Buffer,"adr=%06X;adrType=%d,lat=%.06f,lon=%.06f,alt=%.01f,speed=%.01f,course=%.01f,climb=%.01f\n", fop->addr,fop->addr_type,fop->latitude,fop->longitude,fop->altitude,fop->speed,fop->course,fop->vs);
    log_e("%s",&Buffer[0]);
    return -1;
  }
  */

  if ((pkt->turnrate != TURN_RATE_ON_GROUND) && (pkt->turnrate != TURN_RATE_RIGHT_TURN) && (pkt->turnrate != TURN_RATE_NO_TURN) && (pkt->turnrate != TURN_RATE_LEFT_TURN)){
    log_e("unknown message turnrate=%02X",pkt->turnrate);
    char Buffer[500];	
    sprintf(Buffer,"adr=%06X;adrType=%d,lat=%.06f,lon=%.06f,alt=%.01f,speed=%.01f,course=%.01f,climb=%.01f\n", fop->addr,fop->addr_type,fop->latitude,fop->longitude,fop->altitude,fop->speed,fop->course,fop->vs);
    log_e("%s",&Buffer[0]);
    return -12;
  }


  return 0;
}




