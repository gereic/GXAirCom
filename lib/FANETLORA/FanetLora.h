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
#include <LoRa.h>


#define FANET_DEBUG

//433E6 for Asia
//866E6 for Europe
//915E6 for North America
//#define BAND 866E6
#define FREQUENCY868 868200012
#define FREQUENCY915 916039978
#define FANET_HEADER_SIZE 4
#define ManuId 0x07 // for the moment we use soft-Rf, cause then airwhere is showing it on the map
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


enum class eFanetAircraftType {
  UNKNOWN = 0,
  PARA_GLIDER = 1,
  HANG_GLIDER = 2,
  BALLOON = 3,
  GLIDER = 4,
  POWERED_AIRCRAFT = 5,
  HELICOPTER_ROTORCRAFT = 6,
  UAV = 7
};

typedef struct {
  String DevId;
  float lat; //latitude
  float lon; //longitude
  float altitude; //altitude [m]
  eFanetAircraftType aircraftType; //
  float speed; //km/h
  float climb; //m/s
  float heading; //deg
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
  float lat; //latitude
  float lon; //longitude
  float temp; //temp [°C]
  float wHeading; //wind heading [°]
  float wSpeed; //km/h
  float wGust; //km/h
  float Humidity;
  float Baro;
  uint8_t Charge; //+1byte lower 4 bits: 0x00 = 0%, 0x01 = 6.666%, .. 0x0F = 100%
} weatherData;

class FanetLora {
public:
    FanetLora(); //constructor
    bool begin(int8_t sck, int8_t miso, int8_t mosi, int8_t ss,int reset, int dio0,long frequency);
    void end(void);
    String getMyDevId(void);
    void setPilotname(String name);
    void setAircraftType(eFanetAircraftType type);
    void writeTrackingData2FANET(trackingData *tData);
    void writeMsgType2(String name);
    void writeMsgType3(String msg);
    void writeMsgType4(weatherData *wData);
    String getAircraftType(eFanetAircraftType type);
    eFanetAircraftType getAircraftType(void);
    void run(void); //has to be called cyclic
    bool isNewMsg();
    String getactMsg(); 
    bool getTrackingData(trackingData *tData);
    bool getMyTrackingData(trackingData *tData);
    void printFanetData(trackingData tData);
    void sendPilotName(void);
    uint16_t txCount;
    uint16_t rxCount;

    /*
    String getFlarmExp(void);
    void setNMEAOUT(NmeaOut *_pNmeaOut);
    
    bool newTrackingDataAvaiable(void);
    void writeTrackingData2FANET(trackingData *tData);
    
    void writeStateData2FANET(stateData *tData);
    String getAircraftType(eFanetAircraftType type);
    bool initOk(void);
    */
protected:
    uint8_t initCount;
    uint8_t counter = 0;
    String _PilotName;
    trackingData _myData;
    uint8_t myDevId[3];
    uint8_t LoraReceive[64];
    bool newMsg;
    String actMsg;
    void sendPilotName(uint32_t tAct);
    void getLoraMsg(void);
    void checkMyDevId();   
    String uint64ToString(uint64_t input);
    void coord2payload_absolut(float lat, float lon, uint8_t *buf);
    int getByteFromHex(char in[]);
    String getHexFromByte(uint8_t val,bool leadingZero = false);    
    String getHexFromWord(uint16_t val,bool leadingZero = false);
    trackingData actTrackingData;
    bool newData = false;
    void getTrackingInfo(String line,uint16_t length);
    void printAircraftType(eFanetAircraftType type);


    /*
    String FlarmExp;
    HardwareSerial *pFanetSerial;  
    NmeaOut *pNmeaOut;  
    
    void initModule(uint32_t tAct);
    void DecodeLine(String line);
    void CheckReceivedPackage(String line);
    
    uint32_t tCheckModuleState;
    String getStringValue(String s,String keyword);
    int getStringValue(String s,String *sRet,unsigned int fromIndex,String keyword,String delimiter);
    uint8_t _ResetPin;
    
    void sendWeather(void);
    uint32_t tWeather;
    void coord2payload_absolut(float lat, float lon, uint8_t *buf);
    void encodeTrackingData(trackingData tData,uint8_t *buf);
    
    void resetModule();
    
    void getMyID(String line);
    void getFAX(String line);
    char lineBuffer[FANET_MAXRECBUFFER];
    uint8_t recBufferIndex;
    
    bool bFNAOk;
    bool bFNCOk;
    bool bDGPOk;
    bool bFAPOk;
    bool bFAXOk;
    bool bInitOk;
    */
private:
  static void onReceive(int packetSize);
  String CreateFNFMSG(char *recBuff,uint8_t size);

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
    unsigned int type           :6;
    unsigned int forward        :1;
    unsigned int ext_header     :1;

    unsigned int vendor         :8;
    unsigned int address        :16;

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

  } __attribute__((packed)) fanet_packet_t;
    
  /*
  * Tracking frame type (#4),
  * Standard header,
  * No signature,
  * Broadcast
  */
  typedef struct {
    unsigned int type           :6;
    unsigned int forward        :1;
    unsigned int ext_header     :1;

    unsigned int vendor         :8;
    unsigned int address        :16;

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