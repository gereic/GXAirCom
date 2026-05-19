/*!
 * @file FlarmRadio.cpp
 *
 *
 */

#include "FlarmRadio.h"


void flarm_create_packet(AircraftState *aircraft, uint8_t *packet);
size_t flarm_encrypt(void *flarm_pkt, long timestamp);
size_t flarm_v7_encode(AircraftState *aircraft, uint8_t *packet, long timestamp);
uint8_t flarm_parity(uint32_t x);
bool flarm_v6_decode(void *flarm_pkt, ufo_t *this_aircraft, ufo_t *fop);


//static bool use_v6_on_tx = true;

size_t flarm_encode(void *flarm_pkt, AircraftState *aircraft, long timestamp) {
    /*
    if (use_v6_on_tx) {
      use_v6_on_tx = false;
      log_i("Tx_V6");
      flarm_create_packet(aircraft, (uint8_t *)flarm_pkt);
      return flarm_encrypt(flarm_pkt,timestamp);
    }
    use_v6_on_tx = true;
    */
    //flarm_create_packet(aircraft, (uint8_t *)flarm_pkt);
    //return flarm_encrypt(flarm_pkt,timestamp);   
    //log_i("Tx_V7");
    return flarm_v7_encode(aircraft,(uint8_t *)flarm_pkt,timestamp);

}

void printbuffer(uint8_t *buffer){
  char hexBuffer[3];
  for(int i=0; i<26; i++){
    sprintf(hexBuffer,"%02X", buffer[i]);
    Serial.print(hexBuffer);
  }
  Serial.print("\n");
}

static const uint16_t lon_div_table[] = {
   53,  53,  54,  54,  55,  55,
   56,  56,  57,  57,  58,  58,  59,  59,  60,  60,
   61,  61,  62,  62,  63,  63,  64,  64,  65,  65,
   67,  68,  70,  71,  73,  74,  76,  77,  79,  80,
   82,  83,  85,  86,  88,  89,  91,  94,  98, 101,
  105, 108, 112, 115, 119, 122, 126, 129, 137, 144,
  152, 159, 167, 174, 190, 205, 221, 236, 252,
  267, 299, 330, 362, 425, 489, 552, 616, 679, 743, 806, 806
};

static int descale(unsigned int value, unsigned int mbits, unsigned int ebits)
{
    unsigned int offset   = (1 << mbits);
    unsigned int signbit  = (offset << ebits);
    unsigned int negative = (value & signbit);

    value &= (signbit - 1);

    if (value >= offset) {
        unsigned int exp = value >> mbits;
        value &= (offset - 1);
        value += offset;
        value <<= exp;
        value -= offset;
    }

    return (negative ? -(int)value : value);
}

static unsigned int enscale_unsigned(unsigned int value,
                                     unsigned int mbits,
                                     unsigned int ebits)
{
    unsigned int offset  = (1 << mbits);
    unsigned int max_val = (offset << ebits) - 1;

    if (value >= offset) {
      unsigned int e      = 0;
      unsigned int m      = offset + value;
      unsigned int mlimit = offset + offset - 1;

      while (m > mlimit) {
          m >>= 1;
          e += offset;
          if (e > max_val) {
              return (max_val);
          }
      }
      m -= offset;
      value = (e | m);
    }

    return (value);
}

static unsigned int enscale_signed(  signed int value,
                                   unsigned int mbits,
                                   unsigned int ebits)
{
    unsigned int offset  = (1 << mbits);
    unsigned int signbit = (offset << ebits);
    unsigned int max_val = signbit - 1;
    unsigned int sign    = 0;

    if (value < 0) {
      value = -value;
      sign  = signbit;
    }

    unsigned int rval;
    if (value >= offset) {
      unsigned int e      = 0;
      unsigned int m      = offset + (unsigned int) value;
      unsigned int mlimit = offset + offset - 1;

      while (m > mlimit) {
        m >>= 1;
        e += offset;
        if (e > max_val) {
          return (sign | max_val);
        }
      }
      m -= offset;
      rval = (sign | e | m);
    } else {
      rval = (sign | (unsigned int) value);
    }

    return (rval);
}



uint8_t flarm_get_zone(float lat, float lon){
/*
- Zone 1: Europe, Africa, Russia, China (30W to 110E, excl. zone 5)
- Zone 2: North America (west of 30W, north of 10N)
- Zone 3: New Zealand (east of 160E)
- Zone 4: Australia (110E to 160E)
- Zone 5: Israel (34E to 54E and 29.25N to 33.5N)
- Zone 6: South America (west of 30W, south of 10N)
*/
//    log_i("Zone 3: New Zealand (east of 160E)");
//    return 3; //Zone 3: New Zealand (east of 160E)

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

void flarm_getFrequencyChannels(uint8_t zone,uint32_t *frequency,uint32_t *ChanSepar, uint8_t *channels,int8_t *MaxTxPower){
  if ((zone < 1) || (zone > 6)) return; //zone not defined
/*
- Zone 1: f0=868200000, f1=868400000
- Zone 2, 3, 5, 6: f0=902200000, nch=65
- Zone 4: f0=917000000, nch=24
*/
  if (zone == 1){
    //Zone 1: f0=868200000, f1=868400000
    *frequency = 868200000;
    *ChanSepar = 200000;
    *channels = 2;
    *MaxTxPower = 14;
  }else if (zone == 3){
    //Zone 3: f0=869250000, nch=1 //tried with softrf 1.7.1 --> correct Frequency is 869200000
    *frequency = 869200000;
    *ChanSepar = 200000;
    *channels = 1;
    *MaxTxPower = 10;
  }else if (zone == 4){
    //Zone 4: f0=917000000, nch=24
    *frequency = 917000000;
    *ChanSepar = 400000;
    *channels = 24;
    *MaxTxPower = 30;
  }else{
    //Zone 2, 3, 5, 6: f0=902200000, nch=65
    *frequency = 902200000;
    *ChanSepar = 400000;
    *channels = 65;
    *MaxTxPower = 30;
  }
}

uint32_t flarm_calculate_freq_channel(uint32_t timestamp, uint32_t nch) {
  timestamp = timestamp - 1;
  timestamp = (timestamp<<1);
  timestamp  = (timestamp<<15) + (~timestamp);
  timestamp ^= timestamp>>12;
  timestamp += timestamp<<2;
  timestamp ^= timestamp>>4;
  timestamp *= 2057;
  return (timestamp ^ (timestamp>>16))  % nch;

  if (nch <= 1) return 0;
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
  //pkt->parity = 0; //from V7 it is always 0
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

void make_v7_key(uint32_t key[4]) {
  uint8_t *bkeys;
  int p, q, x, y, z, sum;

  bkeys = (uint8_t *) &key[0];
  x = bkeys[15];
  sum = 0;
  q = 2;

  do {
    sum += DELTA;
    for (p=0; p<16; p++) {
      z = x & 0xFF;
      y = bkeys[(p+1) % 16];
      x = bkeys[p];
      x += ((((z >> 5) ^ (y << 2)) + ((y >> 3) ^ (z << 4))) ^ (sum ^ y));
      bkeys[p] = (uint8_t)x;
    }
  } while (--q > 0);
}

bool flarm_decode(void *flarm_pkt, ufo_t *this_aircraft, ufo_t *fop){
  const uint32_t xxtea_key[4] = FLARM_KEY5;
  uint32_t key_v7[4];
  flarm_v7_packet_t *pkt = (flarm_v7_packet_t *) flarm_pkt;
  if (pkt->type == 0){
    #ifdef DEBUG_FLARM_ERRORS
    log_i("V6 protocol");
    #endif
    return flarm_v6_decode(flarm_pkt,this_aircraft,fop);
  }else if (pkt->type != 2){
    #ifdef DEBUG_FLARM_ERRORS
    log_e("wrong Flarm-protocol %d --> discarding",pkt->type);
    #endif
    return false; //wrong protocoll --> discarding
  }
  uint32_t *wpkt     = (uint32_t *) flarm_pkt;
  uint32_t timestamp = (uint32_t) this_aircraft->timestamp;  

  flarm_btea(&wpkt[2], -4, xxtea_key);
  key_v7[0]          = wpkt[0];
  key_v7[1]          = wpkt[1];
  key_v7[2]          = timestamp >> 4;
  key_v7[3]          = FLARM_KEY4;

  make_v7_key(key_v7);

  wpkt[2] ^= key_v7[0];
  wpkt[3] ^= key_v7[1];
  wpkt[4] ^= key_v7[2];
  wpkt[5] ^= key_v7[3];

  #ifdef DEBUG_FLARM_ERRORS
  //if ((pkt->_unk1 == 0) && (pkt->_unk2 == 0) && (pkt->_unk3 == 3) && (pkt->_unk4 == 0) && (pkt->_unk5 == 3) && (pkt->_unk6 == 0) && (pkt->_unk7 == 0) && (pkt->_unk8 == 0) && ((pkt->_unk9 == 0) || (pkt->_unk9 == 2) || (pkt->_unk9 == 11)) && (pkt->_unk10 == 0)){
  if ((pkt->_unk1 == 0) && (pkt->_unk2 == 0) && (pkt->_unk3 == 3) && (pkt->_unk4 == 0) && (pkt->_unk5 == 3) && (pkt->_unk6 == 0)){
    //ok
  }else{
    log_i("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",pkt->_unk1,pkt->_unk2,pkt->_unk3,pkt->_unk4,pkt->_unk5,pkt->_unk6,pkt->_unk7,pkt->_unk8,pkt->_unk9,pkt->_unk10);
    //return false;
  }
  #endif
  //log_i("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",pkt->_unk1,pkt->_unk2,pkt->_unk3,pkt->_unk4,pkt->_unk5,pkt->_unk6,pkt->_unk7,pkt->_unk8,pkt->_unk9,pkt->_unk10);

  if (pkt->_unk1 != 0){
    #ifdef DEBUG_FLARM_ERRORS
    log_e("_unk1");
    #endif
    return false;
  }  
  if (pkt->_unk2 != 0){
    #ifdef DEBUG_FLARM_ERRORS
    log_e("_unk2");
    #endif
    return false;
  }  
  if (pkt->_unk3 != 3){
    #ifdef DEBUG_FLARM_ERRORS
    log_e("_unk3");
    #endif
    return false;
  }  
  if (pkt->_unk4 != 0){
    #ifdef DEBUG_FLARM_ERRORS
    log_e("_unk4");
    #endif
    return false;
  }  
  if (pkt->_unk5 != 3){
    #ifdef DEBUG_FLARM_ERRORS
    log_e("_unk5");
    #endif
    return false;
  }  
  if (pkt->_unk6 != 0){
    #ifdef DEBUG_FLARM_ERRORS
    log_e("_unk6");
    #endif
    return false;
  }  
  /*
  if (pkt->_unk7 != 0){
    #ifdef DEBUG_FLARM_ERRORS
    log_e("_unk7");
    #endif
    return false;
  }  
  if (pkt->_unk8 != 0){
    #ifdef DEBUG_FLARM_ERRORS
    log_e("_unk8");
    #endif
    return false;
  }
  if ((pkt->_unk9 != 0) && (pkt->_unk9 != 2) && (pkt->_unk9 != 11)){
    #ifdef DEBUG_FLARM_ERRORS
    log_e("_unk9");
    #endif
    return false;
  }
  if (pkt->_unk10 != 0){
    #ifdef DEBUG_FLARM_ERRORS
    log_e("_unk10");
    #endif
    return false;
  }  
  */     

  fop->addr          = pkt->addr;
  fop->addr_type     = pkt->addr_type;
  fop->timestamp     = timestamp;

  fop->stealth       = pkt->stealth;
  fop->no_track      = pkt->no_track;
  fop->aircraft_type = pkt->aircraft_type;
  if (pkt->airborne == 1){
    fop->airborne = false;
    fop->onGround = true;
  }else{
    fop->airborne = true;
    fop->onGround = false;
  }
  

  float ref_lat      = this_aircraft->latitude;
  float ref_lon      = this_aircraft->longitude;
  float geo_separ    = this_aircraft->geoid_separation;

  int16_t alt = descale(pkt->alt, 12, 1) - 1000 ; /* relative to WGS84 ellipsoid */
  fop->altitude      = (float) alt - geo_separ;

  int32_t round_lat  = (int32_t) (ref_lat * 1e7) / 52;
  int32_t lat        = (pkt->lat - round_lat) % (uint32_t) 0x100000;
  if (lat >= 0x080000) lat -= 0x100000;
  lat                = ((lat + round_lat) * 52) /* + 0x40 */;
  fop->latitude      = (float)lat / 1e7;

  int ilat           = (int)fabs(fop->latitude);
  if (ilat > 89) { ilat = 89; }
  int32_t lon_div    = (ilat < 14) ? 52 : lon_div_table[ilat-14];

  int32_t round_lon  = (int32_t) (ref_lon * 1e7) / lon_div;
  int32_t lon        = (pkt->lon - round_lon) % (uint32_t) 0x100000;
  if (lon >= 0x080000) lon -= 0x100000;
  lon                = ((lon + round_lon) * lon_div) /* + 0x40 */;
  fop->longitude     = (float)lon / 1e7;

  uint16_t speed10   = (uint16_t) descale(pkt->hs, 8, 2);

  fop->speed         = speed10 * MS_KMH / 10;

  int16_t vs10       = (int16_t) descale(pkt->vs, 6, 2);
  fop->vs            = ((float) vs10) / 10.0;

  float course       = pkt->course;
  fop->course        = course / 2;

  //log_i("sp10=%d:%.1f,vs10=%d",speed10,fop->speed,vs10);
    /* TODO */
  //log_i("lat=%d,lon=%d,ts=%d",pktpkt->tstamp);  
  //flarm_v7_debugBuffer((uint8_t *)flarm_pkt,this_aircraft);
  return true;
}

size_t flarm_v7_encode(AircraftState *aircraft, uint8_t *packet, long timestamp){
		//time_t tUnix;
		//time(&tUnix);
    long tUnix = timestamp;
    const uint32_t xxtea_key[4] = FLARM_KEY5;
    uint32_t key_v7[4];

    flarm_v7_packet_t *pkt = (flarm_v7_packet_t *) packet;
    uint32_t *wpkt     = (uint32_t *) packet;

    //uint32_t id        = this_aircraft->addr;
    //uint8_t acft_type  = this_aircraft->aircraft_type > AIRCRAFT_TYPE_STATIC ?
    //                     AIRCRAFT_TYPE_UNKNOWN : this_aircraft->aircraft_type;

    //int32_t lat        = (aircraft->extrapolated_lat_deg_e7);
    //int32_t lon        = (aircraft->extrapolated_lon_deg_e7);
    //int16_t alt = (int16_t) (aircraft->extrapolated_height_m);
    int32_t lat        = (aircraft->gps->lat_deg_e7);
    int32_t lon        = (aircraft->gps->lon_deg_e7);
    int16_t alt = (int16_t) (aircraft->gps->height_m);

    float course       = (float)aircraft->gps->heading_deg_e1/10;
    float speedf       = (float)aircraft->gspeed_filtered_cm_s / 100.; //aircraft->gps.; /* m/s */
    float vsf          = (float)aircraft->vel_u_filtered_cm_s / 100.; //this_aircraft->vs / (_GPS_FEET_PER_METER * 60.0); /* m/s */

    //log_i("course=%d,speed=%d,vsf=%d",aircraft->extrapolated_heading_deg_e1,aircraft->gspeed_filtered_cm_s,aircraft->vel_u_filtered_cm_s);

  // Fill bytes 0 to 23 of the packet with data.
    packet[0] = aircraft->config->identifier[0];
    packet[1] = aircraft->config->identifier[1];
    packet[2] = aircraft->config->identifier[2];
    pkt->type          = 2; /* Air V7 position */

    pkt->addr_type     = aircraft->config->addressType;

    pkt->stealth       = aircraft->config->private_mode;
    pkt->no_track      = aircraft->config->no_tracking_mode;


    pkt->tstamp        = tUnix & 0xF;
    pkt->aircraft_type = aircraft->config->type;

    alt += 1000;
    if (alt < 0) { alt = 0; }
    pkt->alt           = enscale_unsigned(alt, 12, 1); /* 0 ... 12286 */

    pkt->lat = ((lat / 52) + (lat & 0x40 /* TBD */ ? (lat < 0 ? -1 : 1) : 0)) & 0xFFFFF;

    int ilat           = abs(lat / 10000000); //(int)fabs(this_aircraft->latitude);
    //log_i("ilat=%d",ilat);
    if (ilat > 89) { ilat = 89; }
    int32_t lon_div    = (ilat < 14) ? 52 : lon_div_table[ilat-14];

    pkt->lon           = (lon / lon_div) & 0xFFFFF;

    pkt->turn          = enscale_signed(aircraft->filtered_delta_heading_deg_e1 * 2,6,2); /* TBD */

    uint16_t speed10   = (uint16_t) roundf(speedf * 10.0f);
    pkt->hs            = enscale_unsigned(speed10, 8, 2); /* 0 ... 3832 */

    int16_t vs10       = (int16_t) roundf(vsf * 10.0f);
    pkt->vs = aircraft->config->private_mode ? 0 : enscale_signed(vs10, 6, 2); /* 0 ... 952 */

    pkt->course        = (int) (course * 2);
    pkt->airborne      = aircraft->is_airborne ? 2 : 1;

    //pkt->hp = enscale_unsigned(aircraft->gps->hacc_cm / 10,3,3); //GNSS horizontal accuracy, meters times 10, enscaled(3,3)
    //pkt->vp = enscale_unsigned(aircraft->gps->vacc_cm * 4 / 100 ,2,3); //GNSS vertical accuracy, meters times 4, enscaled(2,3)
    pkt->hp = 30; //FANET+
    pkt->vp = 17; //FANET+

/*
 * TODO
 * Volunteer contributors are welcome:
 * https://pastebin.com/YB1ppAbt
 */
    pkt->_unk1         = 0;
    pkt->_unk2         = 0;
    pkt->_unk3         = 3;
    pkt->_unk4         = 0;
    pkt->_unk5         = 3;
    pkt->_unk6         = 0;
    pkt->_unk7         = 0;
    pkt->_unk8         = 0;
    pkt->_unk9         = 0; /* TBD */
    pkt->_unk10        = 0;

    key_v7[0]          = wpkt[0];
    key_v7[1]          = wpkt[1];
    key_v7[2]          = tUnix >> 4;
    key_v7[3]          = FLARM_KEY4;

    make_v7_key(key_v7);

    wpkt[2] ^= key_v7[0];
    wpkt[3] ^= key_v7[1];
    wpkt[4] ^= key_v7[2];
    wpkt[5] ^= key_v7[3];

    flarm_btea(&wpkt[2], 4, xxtea_key);
    //printbuffer(packet);
    return (sizeof(flarm_v7_packet_t));

}

size_t flarm_decrypt(void *flarm_pkt, long timestamp)
{
  flarm_packet_t *pkt = (flarm_packet_t *) flarm_pkt;
  uint32_t key[4];
  flarm_make_key(key, timestamp, (pkt->addr << 8) & 0xffffff);
  flarm_btea((uint32_t *) pkt + 1, -5, key);
  return (sizeof(flarm_packet_t));
}

unsigned short flarm_getCkSum(uint8_t* ba, int len)
{
  uint16_t crc16 = 0xffff;  /* seed value */
  crc16 = update_crc_ccitt(crc16, 0x31);
  crc16 = update_crc_ccitt(crc16, 0xFA);
  crc16 = update_crc_ccitt(crc16, 0xB6);
    
  for (int i =0;i<len;i++)
    crc16 = update_crc_ccitt(crc16, (ba[i]));

 return crc16;    
} 

bool flarm_v6_decode(void *flarm_pkt, ufo_t *this_aircraft, ufo_t *fop) {

  flarm_packet_t *pkt = (flarm_packet_t *) flarm_pkt;

  float ref_lat = this_aircraft->latitude;
  float ref_lon = this_aircraft->longitude;
  float geo_separ = this_aircraft->geoid_separation;
  uint32_t timestamp = (uint32_t) this_aircraft->timestamp;
  int ndx;
  //check parity of frame !! --> don't include parity-bit
  uint32_t key[4];
  uint8_t pkt_parity=0;
  flarm_make_key(key, timestamp, (pkt->addr << 8) & 0xffffff);
  flarm_btea((uint32_t *) pkt + 1, -5, key);

  for (ndx = 0; ndx < sizeof (flarm_packet_t); ndx++) {
    pkt_parity += flarm_parity(*(((unsigned char *) pkt) + ndx));
  }
  if (pkt_parity % 2) {
    log_e("bad parity of decoded packet: %02X",pkt_parity);  
    return false;
  } 

  if (pkt->addr == 0){
    #ifdef DEBUG_FLARM_ERRORS
    log_e("addr = 0");
    #endif
    return false;    
  }
  if (pkt->aircraft_type == 0){
    #ifdef DEBUG_FLARM_ERRORS
    log_e("aircraft_type = 0");
    #endif
    return false;    
  }
  if (pkt->type != 0){
    #ifdef DEBUG_FLARM_ERRORS
    log_e("unknown message type=%02X",pkt->type);
    #endif
    return false;
  }
  if (pkt->zero1 != 0){
    #ifdef DEBUG_FLARM_ERRORS
    log_e("unknown message zero1=%02X",pkt->zero1);
    #endif
    return false;
  }
  if (pkt->zero2 != 0){
    #ifdef DEBUG_FLARM_ERRORS
    log_e("unknown message zero2=%02X",pkt->zero2);
    #endif
    return false;
  }

  /*
  bool bParityErr = false;
  //if (origParity) log_i("adr:%06X parity orig:%02X,calc:%02X",pkt->addr,origParity,pkt_parity);
  //V7 sends always 0
  //V6 and below sends parity-bit (could be 0 or 1)
  if ((origParity != 0) && (origParity != pkt_parity)) { 
  //if (origParity != pkt_parity) {
    #ifdef DEBUG_FLARM_ERRORS
    log_e("bad parity of decoded packet: %02X!=%02X",origParity,pkt_parity);        
    #endif
    bParityErr = true;
    //return -1;
  }
  */
  

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

  if ((pkt->turnrate != TURN_RATE_0) && (pkt->turnrate != TURN_RATE_ON_GROUND) && (pkt->turnrate != TURN_RATE_RIGHT_TURN) && (pkt->turnrate != TURN_RATE_NO_TURN) && (pkt->turnrate != TURN_RATE_LEFT_TURN)){
    log_e("unknown message turnrate=%02X",pkt->turnrate);
    char Buffer[500];	
    sprintf(Buffer,"adr=%06X;adrType=%d,lat=%.06f,lon=%.06f,alt=%.01f,speed=%.01f,course=%.01f,climb=%.01f\n", fop->addr,fop->addr_type,fop->latitude,fop->longitude,fop->altitude,fop->speed,fop->course,fop->vs);
    log_e("%s",&Buffer[0]);
    return false;
  }


  return true;
}

void flarm_v7_debugBuffer(uint8_t *flarm_pkt,ufo_t *this_aircraft){
  uint8_t newPacket[26];
  memcpy(&newPacket[0],flarm_pkt,26);
  uint16_t crc16_2 = (uint16_t(newPacket[24]) << 8) + uint16_t(newPacket[25]);
  uint16_t crc16 =  flarm_getCkSum(newPacket,24);
  if (crc16 != crc16_2){
    log_e("wrong Checksum %04X!=%04X",crc16,crc16_2);
    return;
  } 
  flarm_v7_packet_t *pkt = (flarm_v7_packet_t *)newPacket;
  if (pkt->type != 2){
    log_e("packet type %d != 2",pkt->type);
    return;
  } 
  ufo_t air={0};
  bool bOk = flarm_decode(newPacket,this_aircraft,&air);
  if (!bOk){
    log_e("flarm-package not ok");
    return;
  }
  log_i("%d,id=%06X,addr_type=%d,lat=%d,lon=%D,alt=%d,ab=%d,turn=%d,hs=%d,vs=%d,hp=%d,vp=%d",millis(),pkt->addr,pkt->addr_type,pkt->lat,pkt->lon,pkt->alt,pkt->airborne,pkt->turn,pkt->hs,pkt->vs,pkt->hp,pkt->vp);
  log_i("aircraft=%d,course=%d,nt=%d,st=%d,tstmp=%d",pkt->aircraft_type,pkt->course,pkt->no_track,pkt->stealth,pkt->tstamp);
  log_i("u1=%d,u2=%d,u3=%d,u4=%d,u5=%d,u6=%d,u7=%d,u8=%d,u9=%d,u10=%d,",pkt->_unk1,pkt->_unk2,pkt->_unk3,pkt->_unk4,pkt->_unk5,pkt->_unk6,pkt->_unk7,pkt->_unk8,pkt->_unk9,pkt->_unk10);
  log_i("flarm package ok");

}

void flarm_debugLog(uint32_t tPps,uint8_t *flarm_pkt,ufo_t *this_aircraft){
  uint8_t newPacket[26];
  memcpy(&newPacket[0],flarm_pkt,26);
  uint16_t crc16_2 = (uint16_t(newPacket[24]) << 8) + uint16_t(newPacket[25]);
  uint16_t crc16 =  flarm_getCkSum(newPacket,24);
  if (crc16 != crc16_2){
    log_e("wrong Checksum %04X!=%04X",crc16,crc16_2);
    return;
  } 
  flarm_v7_packet_t *pkt = (flarm_v7_packet_t *)newPacket;
  if (pkt->type != 2){
    log_e("packet type %d != 2",pkt->type);
    return;
  } 
  ufo_t air={0};
  bool bOk = flarm_decode(newPacket,this_aircraft,&air);
  if (!bOk){
    log_e("flarm-package not ok");
    return;
  }
  char sLine[MAXSTRING];
  //sprintf(sLine,"%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d\n",millis()-tPps,pkt->addr,pkt->type,pkt->addr_type,pkt->_unk1,pkt->_unk2,pkt->stealth,pkt->no_track,pkt->_unk3,pkt->_unk4,pkt->_unk5,pkt->_unk6,pkt->_unk7,pkt->tstamp,pkt->aircraft_type,pkt->_unk8,pkt->alt,pkt->lat,pkt->lon,pkt->turn,pkt->hs,pkt->vs,pkt->course,pkt->airborne,pkt->hp,pkt->vp,pkt->_unk9,pkt->_unk10);
  int pos = 0;
  pos += snprintf(&sLine[pos],MAXSTRING-pos,"%d;%06X;",int(millis()-tPps),air.addr);
  pos += snprintf(&sLine[pos],MAXSTRING-pos,"%d;%d;%d;%d;%d;",air.addr_type,air.stealth,air.no_track,pkt->tstamp,air.aircraft_type);
  pos += snprintf(&sLine[pos],MAXSTRING-pos,"%.2f;%.6f;%.6f;%d;%d;",air.altitude,air.latitude,air.longitude,pkt->turn,pkt->hs);
  pos += snprintf(&sLine[pos],MAXSTRING-pos,"%d;%.1f;%d;%d;%d",pkt->vs,air.course,pkt->airborne,pkt->hp,pkt->vp);
  //log_i("%s",sLine);
  pos += snprintf(&sLine[pos],MAXSTRING-pos,"\r\n");
  #ifdef FLARMLOGGER
    if (FlarmLogQueue) {
		  BaseType_t status = xQueueSendToBack(FlarmLogQueue,&sLine,0);
			if (status != pdTRUE){
			  log_w("FlarmLogQueue full -> reset queue");
				xQueueReset(FlarmLogQueue);
			} 
    }   
  #endif
}

void flarm_debugAircraft(ufo_t *air,ufo_t *this_aircraft){
  char sLine[MAXSTRING];
  int pos = 0;
  pos += snprintf(&sLine[pos],MAXSTRING-pos,"adr=%06X;",air->addr);
  pos += snprintf(&sLine[pos],MAXSTRING-pos,"adrType=%d;aircraftType=%d;",air->addr_type,air->aircraft_type);
  pos += snprintf(&sLine[pos],MAXSTRING-pos,"alt=%.2f;lat=%.6f;lon=%.6f;",air->altitude,air->latitude,air->longitude);
  log_i("%s",sLine);  
}




