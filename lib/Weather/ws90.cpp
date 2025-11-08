/*
Fine Offset Electronics WS90 weather station.

The WS90 is a WS80 with the addition of a piezoelectric rain gauge.
Data bytes 1-13 are the same between the two models.  The new rain data
is in bytes 16-20, with bytes 19 and 20 reporting total rain.  Bytes
17 and 18 are affected by rain, but it is unknown what they report.  Byte
21 reports the voltage of the super cap. And the checksum and CRC
have been moved to bytes 30 and 31.  What is reported in the other
bytes is unknown at this time.

Also sold by EcoWitt.

Preamble is aaaa aaaa aaaa, sync word is 2dd4.

Packet layout:

     0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31
    YY II II II LL LL BB FF TT HH WW DD GG VV UU UU R0 R1 R2 R3 R4 SS UU UU UU UU UU UU UU ZZ AA XX
    90 00 34 2b 00 77 a4 82 62 39 00 3e 00 00 3f ff 20 00 ba 00 00 26 02 00 ff 9f f8 00 00 82 92 4f

- Y = fixed sensor type 0x90
- I = device ID, might be less than 24 bit?
- L = light value, unit of 10 lux
- B = battery voltage, unit of 20 mV, we assume a range of 3.0V to 1.4V
- F = flags and MSBs, 0x03: temp MSB, 0x10: wind MSB, 0x20: bearing MSB, 0x40: gust MSB
      0x80 or 0x08: maybe battery good? seems to be always 0x88
- T = temperature, lowest 8 bits of temperature, offset 40, scale 10
- H = humidity
- W = wind speed, lowest 8 bits of wind speed, m/s, scale 10
- D = wind bearing, lowest 8 bits of wind bearing, range 0-359 deg, 0x1ff if invalid
- G = wind gust, lowest 8 bits of wind gust, m/s, scale 10
- V = uv index, scale 10
- U = unknown (bytes 14 and 15 appear to be fixed at 3f ff)
- R = rain total (R3 << 8 | R4) * 0.1 mm
- S = super cap voltage, unit of 0.1V, lower 6 bits, mask 0x3f
- Z = Firmware version. 0x82 = 130 = 1.3.0
- A = checksum
- X = CRC

Fine Offset Electronics WS80 weather station.

Also sold by EcoWitt, used with the weather station GW1000.

Preamble is aaaa aaaa aaaa, sync word is 2dd4.

Packet layout:

     0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17
    YY II II II LL LL BB FF TT HH WW DD GG VV UU UU AA XX
    80 0a 00 3b 00 00 88 8a 59 38 18 6d 1c 00 ff ff d8 df

- Y = fixed sensor type 0x80
- I = device ID, might be less than 24 bit?
- L = light value, unit of 10 Lux (or 0.078925 W/m2)
- B = battery voltage, unit of 20 mV, we assume a range of 3.0V to 1.4V
- F = flags and MSBs, 0x03: temp MSB, 0x10: wind MSB, 0x20: bearing MSB, 0x40: gust MSB
      0x80 or 0x08: maybe battery good? seems to be always 0x88
- T = temperature, lowest 8 bits of temperature, offset 40, scale 10
- H = humidity
- W = wind speed, lowest 8 bits of wind speed, m/s, scale 10
- D = wind bearing, lowest 8 bits of wind bearing, range 0-359 deg, 0x1ff if invalid
- G = wind gust, lowest 8 bits of wind gust, m/s, scale 10
- V = uv index, scale 10
- U = unknown, might be rain option
- A = checksum
- X = CRC

*/

#include <ws90.h>

#define RF_MODULE_CS 48     //pin to be used as chip select
#define RF_MODULE_GDO0 42   // CC1101 pin GDO0
#define RF_MODULE_GDO2 41   // CC1101 pin GDO2
#define RF_MODULE_MISO 40
#define RF_MODULE_SCK 47  
#define RF_MODULE_MOSI 39

#define RF_MODULE_FREQUENCY 868.350

#define WS90_PACKET_LEN 32



// flag to indicate that a packet was received
volatile bool ws90ReceivedFlag = false;

SPIClass newSPI(HSPI);
//SPIClass newSPI(FSPI);
CC1101 radio = NULL;
//CC1101 radio = new Module(RF_MODULE_CS, RF_MODULE_GDO0, RADIOLIB_NC, RF_MODULE_GDO2,newSPI);

ws90Data ws90ActData;

uint8_t crc8(uint8_t const message[], unsigned nBytes, uint8_t polynomial, uint8_t init)
{
    uint8_t remainder = init;
    unsigned byte, bit;

    for (byte = 0; byte < nBytes; ++byte) {
        remainder ^= message[byte];
        for (bit = 0; bit < 8; ++bit) {
            if (remainder & 0x80) {
                remainder = (remainder << 1) ^ polynomial;
            } else {
                remainder = (remainder << 1);
            }
        }
    }
    return remainder;
}

int add_bytes(uint8_t const message[], unsigned num_bytes)
{
    int result = 0;
    for (unsigned i = 0; i < num_bytes; ++i) {
        result += message[i];
    }
    return result;
}



ICACHE_RAM_ATTR void ws90RxComplete(void) {
  // we got a packet, set the flag
  ws90ReceivedFlag = true;
}

void ws90DebugData(){
  log_i("id=0x%06X;rssi=%.1f;fw=%d;flags=0x%02X;bat=%d;hum=%d;temp=%.1f;wDir=%d;wSpeed=%.1f,wGust=%.1f",ws90ActData.id,ws90ActData.rssi,ws90ActData.firmware,ws90ActData.flags,ws90ActData.battery_lvl,ws90ActData.humidity,ws90ActData.temp_c,ws90ActData.wind_dir,ws90ActData.wind_avg,ws90ActData.wind_max);
}

void ws90GetCommondata(uint8_t *pData){
  ws90ActData.rssi = radio.getRSSI(); //get rssi of last package
  ws90ActData.id          = (pData[1] << 16) | (pData[2] << 8) | (pData[3]);
  ws90ActData.light_raw   = (pData[4] << 8) | (pData[5]);
  ws90ActData.light_lux = ws90ActData.light_raw * 10;        // Lux
  //float light_wm2 = light_raw * 0.078925f; // W/m2
  ws90ActData.battery_mv  = (pData[6] * 20);            // mV
  ws90ActData.battery_lvl = ws90ActData.battery_mv < 1400 ? 0 : (ws90ActData.battery_mv - 1400) / 16; // 1.4V-3.0V is 0-100
  ws90ActData.flags       = pData[7]; // to find the wind msb
  ws90ActData.temp_raw    = ((pData[7] & 0x03) << 8) | (pData[8]);
  ws90ActData.temp_c    = (ws90ActData.temp_raw - 400) * 0.1f;
  ws90ActData.humidity    = (pData[9]);
  ws90ActData.wind_avg    = float(((pData[7] & 0x10) << 4) | (pData[10]))* 0.1 * 3.6; //m/s --> km/h
  ws90ActData.wind_dir    = ((pData[7] & 0x20) << 3) | (pData[11]);
  ws90ActData.wind_max    = float(((pData[7] & 0x40) << 2) | (pData[12])) * 0.1 * 3.6; //m/s --> km/h
  ws90ActData.uv_index    = (pData[13]);
  ws90ActData.bTemp = (ws90ActData.temp_raw != 0x3ff) ? true : false;
  ws90ActData.bHum = (ws90ActData.humidity != 0xff) ? true : false;
  if ((ws90ActData.wind_dir != 0x1FF) && (ws90ActData.wind_avg != 0x1FF) && (ws90ActData.wind_max != 0x1FF)){
    ws90ActData.bWind = true;
  }else{
    ws90ActData.bWind = false;
  }

  if (ws90ActData.battery_lvl > 100) // More then 100%?
      ws90ActData.battery_lvl = 100;  
  ws90ActData.bValid = true;

}

void ws85GetData(uint8_t *pData){
  log_i("get data");
    // Verify checksum and CRC
  uint8_t crc = crc8(pData, 27, 0x31, 0x00);
  uint8_t chk = add_bytes(pData, 27);
  if (crc != 0 || chk != pData[27]) {
      log_e("Checksum error: %02x %02x (%02x)", crc, chk, pData[27]);
      return;
  }
  ws90ActData.rssi = radio.getRSSI(); //get rssi of last package
  ws90ActData.id          = (pData[1] << 16) | (pData[2] << 8) | (pData[3]);
  ws90ActData.battery_mv  = (pData[4] * 20);            // mV
  ws90ActData.battery_lvl = ws90ActData.battery_mv < 1400 ? 0 : (ws90ActData.battery_mv - 1400) / 16; // 1.4V-3.0V is 0-100
  ws90ActData.flags       = pData[5]; // to find the wind msb
  ws90ActData.wind_avg    = float(((pData[5] & 0x10) << 4) | (pData[7]))* 0.1 * 3.6; //m/s --> km/h
  ws90ActData.wind_dir    = ((pData[5] & 0x20) << 3) | (pData[8]);
  ws90ActData.wind_max    = float(((pData[5] & 0x40) << 2) | (pData[9])) * 0.1 * 3.6; //m/s --> km/h
  ws90ActData.supercap_V  = pData[6];
  ws90ActData.firmware    = pData[25];
  if ((ws90ActData.wind_dir != 0x1FF) && (ws90ActData.wind_avg != 0x1FF) && (ws90ActData.wind_max != 0x1FF)){
    ws90ActData.bWind = true;
  }else{
    ws90ActData.bWind = false;
  }
  ws90ActData.bTemp = false;
  ws90ActData.bHum = false;
  if (ws90ActData.battery_lvl > 100) // More then 100%?
      ws90ActData.battery_lvl = 100;  
  ws90ActData.bValid = true;
  ws90DebugData();
}


void ws90GetData(uint8_t *pData){
  log_i("get data");
    // Verify checksum and CRC
  uint8_t crc = crc8(pData, 31, 0x31, 0x00);
  uint8_t chk = add_bytes(pData, 31);
  if (crc != 0 || chk != pData[31]) {
      log_e("Checksum error: %02x %02x (%02x)", crc, chk, pData[31]);
      return;
  }
  ws90GetCommondata(pData);
  ws90ActData.rain_raw    = (pData[19] << 8 ) | (pData[20]);
  ws90ActData.supercap_V  = (pData[21] & 0x3f);
  ws90ActData.firmware    = pData[29];
  ws90DebugData();
}

void ws90_init(float frequency){   
  //uint8_t data[] = {0x85,0x00,0x35,0xC7,0x9D,0x8A,0x44,0x00,0x6F,0x05,0x07,0xFF,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0xFF,0xDF,0xFD,0x00,0x00,0x71,0xED,0xA0,0x00,0x00,0x00,0x00};
  uint8_t data[] = {0x85,0x00,0x35,0xC7,0x9D,0x8A,0x44,0x00,0x6F,0x05,0x07,0xFF,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0xFF,0xDF,0xFD,0x00,0x00,0x71,0xED,0xA0,0x00,0x00,0x00,0x00};
  ws85GetData(&data[0]);
  delay(3000);//wait until power is stable
  log_i("****** init ws90 ******");
  newSPI.begin(RF_MODULE_SCK, RF_MODULE_MISO, RF_MODULE_MOSI, RF_MODULE_CS);
  radio = new Module(RF_MODULE_CS, RF_MODULE_GDO0, RADIOLIB_NC, RF_MODULE_GDO2,newSPI);
  int state = 0;
  for (int i = 0; i < 5;i++){ //try 5 times, if CC1101 is present
    state = radio.begin();
    if (state == RADIOLIB_ERR_NONE) {
      log_i("success!");
      break;
    } else {
      log_e("failed, code %d",state);
      delay(1000);
    }
  }
  if (state != RADIOLIB_ERR_NONE){
    log_e("can't init ws90 radio");
    return;
  } 
  radio.setPacketReceivedAction(ws90RxComplete);
  log_i("set CC1101 frequency=%.3f",frequency);
  radio.setFrequency(frequency);
  radio.setOOK(false);
  //radio.setCrcFiltering(true);
  radio.setCrcFiltering(false);
  radio.setFrequencyDeviation(40);
  radio.setBitRate(17.24);
  radio.setRxBandwidth(270);
  uint8_t const sync_word[] = {0x2d, 0xd4};
  radio.setSyncWord(&sync_word[0],2);
  radio.setEncoding(RADIOLIB_ENCODING_NRZ);
  radio.setDataShaping(RADIOLIB_SHAPING_0_5);
  radio.fixedPacketLengthMode(WS90_PACKET_LEN);
  log_i("[CC1101] Starting to listen ... ");
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    log_i("success!");
  } else {
    log_e("failed, code ",state);
    return;
  }
  

}

void getData(uint8_t *pData){
  if (pData[0] == 0x90){ //WS90
    ws90GetData(pData);
  }else if (pData[0] == 0x85){ //WS85
    ws85GetData(pData);
  }
}

void ws90Run(){
  if (!ws90ReceivedFlag) return;
  byte byteArr[WS90_PACKET_LEN];
  int state = radio.readData(&byteArr[0], WS90_PACKET_LEN);
  ws90ReceivedFlag = false;
  if (state != RADIOLIB_ERR_NONE) {
    log_e("failed, code ",state);
    radio.startReceive();
    return;
  }  
  getData(&byteArr[0]);
  //ws90GetData(&byteArr[0]);
  char msg[1024];
  int next = 0;
  for (int i = 0;i < WS90_PACKET_LEN;i++){
    next += sprintf(&msg[next],"0x%02X;",byteArr[i]);
  }
  log_i("msg=%s",msg);
  // put module back to listen mode
  radio.startReceive();
}