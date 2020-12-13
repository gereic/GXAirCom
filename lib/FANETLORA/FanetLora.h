/*!
 * @file FanetLora.h
 *
 *
 */

#ifndef __FANETLORA_H__
#define __FANETLORA_H__


#include <Arduino.h>
#include <string.h>
#include <SPI.h>
#include <tools.h>
#include "./radio/fmac.h"
//#include "app.h"
#include "./radio/LoRa.h"
#include "CalcTools.h"



#define FANET_DEBUG

#define FANET_LORA_TYPE1_SIZE				13		//11+2
#define FANET_LORA_TYPE4_SIZE				15
#define FANET_LORA_TYPE7_SIZE				7

#define FANET_LORA_TYPE1OR7_AIRTIME_MS			40		//actually 20-30ms
#define	FANET_LORA_TYPE1OR7_MINTAU_MS			250
#define	FANET_LORA_TYPE1OR7_TAU_MS			5000

#define SEPARATOR			','

#define FANET_LORA_VALID_STATE_MS 10000 //10 seconds positions valid



#define MAXNEIGHBOURS 64
#define NEIGHBOURSLIFETIME 240000ul //4min
#define SENDNAMEINTERVAL 240000ul //every 4min
//#define NEIGHBOURSLIFETIME 60000 //4min

//433E6 for Asia
//866E6 for Europe
//915E6 for North America
//#define BAND 866E6
//#define FREQUENCY868 868200012
//#define FREQUENCY868 868200000
//#define FREQUENCY915 916039978
#define FANET_HEADER_SIZE 4
//see fmac.h
//#define ManuId 0x11 // for the moment we use soft-Rf, cause then airwhere is showing it on the map
//#define ManuId 0x07 // for the moment we use soft-Rf, cause then airwhere is showing it on the map
//#define ManuId 0xFA 
/*
Manufacturer IDs:
0x00		[reserved]
0x01		Skytraxx
0x03		BitBroker.eu
0x04		AirWhere
0x05		Windline
0x06		Burnair.ch
0x07		SoftRF
...
0x11		FANET+ (incl FLARM. Currently Skytraxx, and Naviter)
...
0xE0		OGN Tracker
...
0xFA		Various
		0x0001-0x00FF		GetroniX
0xFB		Espressif based base stations, address is last 2bytes of MAC 
0xFC		Unregistered Devices
0xFD		Unregistered Devices
0xFE		[Multicast]
0xFF		[reserved]
*/


class FanetLora : public Fapp{
public:
    enum aircraft_t : uint8_t
    {
      otherAircraft = 0,
      paraglider = 1,
      hangglider = 2,
      balloon = 3,
      glider = 4,
      poweredAircraft = 5,
      helicopter = 6,
      uav = 7,
    };

    enum status_t : uint16_t
    {
      otherStatus = 0,
      hiking = 1,
      vehicle = 2,
      bike = 3,
      boot = 4,
      needAride = 8,

      needTechnicalAssistance = 12,
      needMedicalHelp = 13,
      distressCall = 14,
      distressCallAuto = 15,				//max number
    };
  typedef struct {
    uint32_t devId;
    float lat; //latitude
    float lon; //longitude
    float altitude; //altitude [m]
    aircraft_t aircraftType; //
    float speed; //km/h
    float climb; //m/s
    float heading; //deg
    int rssi; //rssi
    int snr; //signal to noise ratio
  } trackingData;

  typedef struct {
    float lat; //latitude
    float lon; //longitude
    float altitude; //altitude [m]
    float geoIdAltitude; //altitude [m]
    float speed; //km/h
    float climb; //m/s
    float heading; //deg
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
  } stateData;

  typedef struct {
    uint32_t tLastMsg; //timestamp of neighbour (if 0 --> empty slot)
    uint32_t devId; //devId
    String name; //name of neughbour
    aircraft_t aircraftType; //
    float lat; //latitude
    float lon; //longitude
    float altitude; //altitude [m]
    float speed; //km/h
    float climb; //m/s
    float heading; //deg
    int rssi; //rssi
  } neighbour;

  typedef struct {
    float lat; //latitude
    float lon; //longitude
    bool bTemp;
    float temp; //temp [°C]
    float wHeading; //wind heading [°]
    bool bWind;
    float wSpeed; //km/h
    float wGust; //km/h
    bool bHumidity;
    float Humidity;
    bool bBaro;
    float Baro;
    bool bStateOfCharge;
    uint8_t Charge; //+1byte lower 4 bits: 0x00 = 0%, 0x01 = 6.666%, .. 0x0F = 100%
  } weatherData;

  FanetLora(); //constructor
  bool begin(int8_t sck, int8_t miso, int8_t mosi, int8_t ss,int reset, int dio0,long frequency,uint8_t outputPower);
  void end(void);
  String getMyDevId(void);
  String getDevId(uint32_t devId);
  String getNeighbourName(uint32_t devId);
  uint8_t getNeighboursCount(void);
  int16_t getNextNeighbor(uint8_t index);
  int16_t getNearestNeighborIndex();
  void setPilotname(String name);
  void setAircraftType(aircraft_t type);
  void setMyTrackingData(trackingData *tData);
  void writeMsgType1(trackingData *tData);
  void writeMsgType2(String name);
  void writeMsgType3(uint32_t devId,String msg);
  void writeMsgType4(weatherData *wData);
  String getAircraftType(aircraft_t type);
  aircraft_t getAircraftType(void);
  void run(void); //has to be called cyclic
  bool isNewMsg();
  String getactMsg(); 
  bool getTrackingData(trackingData *tData);
  bool getMyTrackingData(trackingData *tData);
  void printFanetData(trackingData tData);
  void sendTracking(trackingData *tData);
  void sendName(String name);
  void sendMSG(String msg);
  uint16_t txCount;
  uint16_t rxCount;
  neighbour neighbours[MAXNEIGHBOURS];
  trackingData _myData;
  bool autobroadcast = false; //autobroadcast
  bool doOnlineTracking = true; //online-tracking
  bool onGround = false;
  status_t state = hiking;

	/* device -> air */
	bool is_broadcast_ready(int num_neighbors);
	void broadcast_successful(int type);

	/* air -> device */
	void handle_acked(bool ack, MacAddr &addr);
	void handle_frame(Frame *frm);
  Frame *get_frame();
  void fanet_cmd_transmit(char *ch_str);
  //void fanet_sendMsg(char *ch_str);

  /* Legacy Switch */
  void setLegacy(uint8_t enableTx);

protected:
private:  
  uint8_t _enableLegacyTx;
  String _PilotName;  
  uint32_t valid_until;
  bool newMsg;
  String actMsg;
  void sendPilotName(uint32_t tAct);
  String uint64ToString(uint64_t input);
  void coord2payload_absolut(float lat, float lon, uint8_t *buf);
  int getByteFromHex(char in[]);
  String getHexFromByte(uint8_t val,bool leadingZero = false);    
  String getHexFromWord(uint16_t val,bool leadingZero = false);
  trackingData actTrackingData;
  bool newData = false;
  void getTrackingInfo(String line,uint16_t length);
  void printAircraftType(aircraft_t type);
  String CreateFNFMSG(Frame *frm);
  int16_t getneighbourIndex(uint32_t devId,bool getEmptyEntry);
  void insertNameToNeighbour(uint32_t devId, String name);
  void insertDataToNeighbour(uint32_t devId, trackingData *Data);
  void clearNeighbours(uint32_t tAct);
  uint32_t getDevIdFromMac(MacAddr *adr);
  MacAddr getMacFromDevId(uint32_t devId);
  int serialize_name(String name,uint8_t*& buffer);
  int serialize_msg(String name,uint8_t*& buffer);
  int serialize_service(weatherData *wData,uint8_t*& buffer);
  int serialize_tracking(trackingData *Data,uint8_t*& buffer);
  int serialize_GroundTracking(trackingData *Data,uint8_t*& buffer);
  bool frm2txBuffer(Frame *frm);
  int actrssi;
	/* determines the tx rate */
	unsigned long last_tx = 0;
	unsigned long next_tx = 0;

  typedef struct fanet_header_t {
    unsigned int type           :6;
    unsigned int forward        :1;
    unsigned int ext_header     :1;

    unsigned int vendor         :8;
    unsigned int address        :16;

    unsigned int reserved       :3;
    unsigned int forwarded      :1;
    unsigned int signature      :1;
    unsigned int unicast        :1;
    unsigned int ack            :2;

    unsigned int DestVendor     :8;
    unsigned int DestAddress    :16;

  } __attribute__((packed)) fanet_header_t;

  /*
  * Tracking frame type (#1),
  * Standard header,
  * No signature,
  * Broadcast
  */
  typedef struct {
    unsigned int latitude       :24;
    unsigned int longitude      :24;

    /* units are degrees, seconds, and meter */
    unsigned int altitude_lsb   :8; /* FANET+ reported alt. comes from ext. source */
    unsigned int altitude_msb   :3; /* I assume that it is geo (GNSS) altitude */
    unsigned int altitude_scale :1;
    unsigned int aircraft_type  :3;
    unsigned int track_online   :1;

    unsigned int speed          :7;
    unsigned int speed_scale    :1;

    unsigned int climb          :7;
    unsigned int climb_scale    :1;

    unsigned int heading        :8;

    unsigned int turn_rate      :7;
    unsigned int turn_scale     :1;

  } __attribute__((packed)) fanet_packet_t1;
    
  /*
  * Tracking frame type (#4),
  * Standard header,
  * No signature,
  * Broadcast
  */
  typedef struct {
    /*
    unsigned int type           :6;
    unsigned int forward        :1;
    unsigned int ext_header     :1;

    unsigned int vendor         :8;
    unsigned int address        :16;
    */
    unsigned int bExt_header2     :1;
    unsigned int bStateOfCharge   :1;
    unsigned int bRemoteConfig    :1;
    unsigned int bBaro            :1;
    unsigned int bHumidity        :1;
    unsigned int bWind            :1;
    unsigned int bTemp            :1;
    unsigned int bInternetGateway :1;

    unsigned int latitude       :24;
    unsigned int longitude      :24;

    int8_t temp           :8;

    unsigned int heading        :8;

    unsigned int speed          :7;
    unsigned int speed_scale    :1;
    
    unsigned int gust          :7;
    unsigned int gust_scale    :1;

    unsigned int humidity      :8;

    int baro          :16;

    unsigned int charge        :8;
  } __attribute__((packed)) fanet_packet_t4;
    
};



#endif