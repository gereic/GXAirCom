/*
 * fmac.h  — GXAirCom FANET MAC layer
 *
 * Original author: sid (30 Sep 2016)
 * ADS-L M-Band additions: feature/adsl-protocol PR
 * - MODE_ADSL_8682 / MODE_ADSL_8684 mode defines
 * - ADSL_TIME timing constant
 * - rfModeBits.AdslTx / .AdslRx (previously reserved bits b4/b5)
 * - Fapp::createAdsl() virtual method
 */

#ifndef FANET_STACK_FMAC_H_
#define FANET_STACK_FMAC_H_

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <Arduino.h>
#include "LoRa.h"
#include "../FLARM/FlarmRadio.h"

extern volatile uint32_t gtPPS;
extern volatile uint32_t gtReceived;


/* Debug */
#define MAC_debug_mode                          0
//#define MAC_debug_mode                                100
#define RX_DEBUG 0
//#define RX_DEBUG 1
#define TX_DEBUG 0
//#define FREQUENCYTEST
//#define DEBUG_FLARM_RX



//define the pins used by the LoRa transceiver module
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 23
#define DIO0 26

//#define FREQUENCY868 868200012
#define FREQUENCY868 868200000
#define FREQUENCY915 916039978

//#define ManuId 0x07 // for the moment we use soft-Rf, cause then airwhere is showing it on the map
#define ManuId 0x08 //08 ... GxAircom



/*
 * Hard coded tx time assumption:
 * -SR7
 * -BW250
 * -CR8 (worst case)
 *
 * payload (byte) -> airtime (ms) -> airtime per byte payload (ms)
 * 0            9.43            0
 * 1            13.44           4.1
 * 2-5          17.54           4-1.63
 * 10           25.73           1.63
 * 64           87.17           1.2
 * 201          246.91          1.18
 * (number according to LoRa calculator)
 *
 * -> tx time assumption:
 * 15ms + 2*payload(bytes)
 * MAC_TX_MINHEADERTIME_MS + (blength * MAC_TX_TIMEPERBYTE_MS)
 */

/*
 * Timing defines
 * ONLY change if you know what you are doing. Can destroy the hole nearby network!
 */

#define MAC_SLOT_MS                             20

#define MAC_TX_MINPREAMBLEHEADERTIME_MS         15
#define MAC_TX_TIMEPERBYTE_MS                   2
#define MAC_TX_ACKTIMEOUT                       1000
#define MAC_TX_RETRANSMISSION_TIME              1000
#define MAC_TX_RETRANSMISSION_RETRYS            3
#define MAC_TX_BACKOFF_EXP_MIN                  7
#define MAC_TX_BACKOFF_EXP_MAX                  12

#define MAC_FORWARD_MAX_RSSI_DBM                -90             //todo test
#define MAC_FORWARD_MIN_DB_BOOST                20
#define MAC_FORWARD_DELAY_MIN                   100
#define MAC_FORWARD_DELAY_MAX                   300

#define NEIGHBOR_MAX_TIMEOUT_MS                 250000          //4min + 10sek

#define MAC_SYNCWORD                            0xF1

/*
 * Number defines
 */
#define MAC_NEIGHBOR_SIZE                       64
#define MAC_MAXNEIGHBORS_4_TRACKING_2HOP        5
#define MAC_CODING48_THRESHOLD                  8

#define MAC_FIFO_SIZE                           8
#define MAC_FRAME_LENGTH                        254

#define ADDRESSTYPE_RANDOM 0
#define ADDRESSTYPE_ICAO 1
#define ADDRESSTYPE_FLARM 2
#define ADDRESSTYPE_OGN 3

// SEND FRAME RETURN VALUES
#define TX_OK                                           0
#define TX_TX_ONGOING                                   -1
#define TX_RX_ONGOING                                   -2
#define TX_FSK_ONGOING                                  -3
#define TX_ERROR                                        -100

#define LORA_TIME 5000
#define FSK_TIME 2000

/*
 * ADS-L M-Band timing slot [ms].
 * Inserted after the FLARM/OGN FSK window.
 * Sized to cover one packet transmission + guard time.
 * At 100 kbps raw with 27-byte packet: ToA ≈ 2.2 ms → 250 ms gives ample guard.
 */
#define ADSL_TIME 250

#define LEGACY_RANGE 30
#define LEGACY_SEND_TIME 30

#define LEGACY_8682_BEGIN               480
#define LEGACY_8682_END                 820
#define LEGACY_8684_BEGIN               870
#define LEGACY_8684_END                 1230

// ── Radio mode defines ───────────────────────────────────────────────────────
#define MODE_LORA       1   ///< FANET+ (LoRa SF7 BW250 868.2 MHz)
#define MODE_FSK_8682   2   ///< FLARM/OGN FSK (868.2 MHz)
#define MODE_FSK_8684   3   ///< FLARM/OGN FSK (868.4 MHz)
#define MODE_FSK        4   ///< Generic FSK
#define MODE_ADSL_8682  5   ///< ADS-L M-Band (868.2 MHz, 2-GFSK 100 kbps raw)
#define MODE_ADSL_8684  6   ///< ADS-L M-Band (868.4 MHz, 2-GFSK 100 kbps raw)


//#include "main.h"
#include "lib/LinkedList2.h"
#include "lib/TimerObject.h"

#include "frame.h"

/* note: zero copy stack might be faster and more memory efficient, but who cares @ 9kBaud and 64Ks of ram... */

class NeighborNode
{
private:
        unsigned long last_seen;
public:
        const MacAddr addr;
        bool hasTracking;

        NeighborNode(MacAddr addr, bool tracking = false) : addr(addr), hasTracking(tracking) { last_seen = millis(); }
        void seen(void) { last_seen = millis(); }
        bool isAround(void) { return last_seen + NEIGHBOR_MAX_TIMEOUT_MS > millis(); }
};

// ── Fapp — application interface (pure virtual) ──────────────────────────────
class Fapp
{
public:
        Fapp() { }
        virtual ~Fapp() { }

        /* device -> air */
        virtual bool is_broadcast_ready(int num_neighbors) = 0;
        virtual void broadcast_successful(int type) = 0;
        virtual Frame *get_frame() = 0;
        virtual bool createLegacy(uint8_t *buffer);

        /**
         * @brief Fill buffer with a complete, ready-to-transmit ADS-L M-Band
         * packet (ADSL_PACKET_SIZE = 27 bytes).
         *
         * @param buffer      Caller-provided buffer of at least ADSL_PACKET_SIZE.
         * @param freqToggle  false = use 868.2 MHz, true = use 868.4 MHz.
         * Implementer stores this in the struct for the caller
         * to read after the call.
         * @return true if data is valid and buffer was filled, false to skip TX.
         *
         * Default implementation returns false (ADS-L disabled unless overridden).
         */
        virtual bool createAdsl(uint8_t *buffer, bool freqToggle) { return false; }

        /* air -> device */
        virtual void handle_acked(bool ack, MacAddr &addr) = 0;
        virtual void handle_frame(Frame *frm) = 0;
};

class MacFifo
{
private:
        LinkedList2<Frame*> fifo;
public:
        /* not usable in async mode */
        Frame* get_nexttx();
        Frame* frame_in_list(Frame *frm);
        Frame* front();

        /* usable in async mode */
        bool remove_delete_acked_frame(MacAddr dest);
        bool remove_delete(Frame *frm);
        int add(Frame *frm);
        int size() { return fifo.size(); }
};

class FanetMac
{
private:
        /* lora region */
        enum eLoraRegion {
                        NONE = 0,
                        US920 = 1,
                        AU920 = 2,
                        IN866 = 3,
                        KR923 = 4,
                        AS920 = 5,
                        IL918 = 6,
                        EU868 = 7
        };

        struct rfModeBits
        {
                // Bits 0..3: original protocol flags
                unsigned FntRx:1;   ///< FANET receive enabled
                unsigned FntTx:1;   ///< FANET transmit enabled
                unsigned LegRx:1;   ///< FLARM/OGN receive enabled
                unsigned LegTx:1;   ///< FLARM/OGN transmit enabled
                // Bits 4..5: ADS-L (previously reserved b4/b5)
                unsigned AdslTx:1;  ///< ADS-L M-Band transmit enabled  [NEW]
                unsigned AdslRx:1;  ///< ADS-L M-Band receive enabled   [NEW]
                unsigned b6:1;
                unsigned b7:1;
        };
        union uRfMode
        {
                        rfModeBits bits;
                        uint8_t mode;
        };
        TimerObject myTimer;
        MacFifo tx_fifo;
        MacFifo rx_fifo;
        LinkedList2<NeighborNode *> neighbors;
        Fapp *myApp = NULL;
        MacAddr _myAddr;

        int8_t _ss;
        int8_t _reset;

        unsigned long csma_next_tx = 0;
        int csma_backoff_exp = MAC_TX_BACKOFF_EXP_MIN;

        /* used for interrupt handler */

        static void frameRxWrapper(int length);
        void frameReceived(int length);

        void ack(Frame* frm);

        static void stateWrapper();
        void handleIRQ();
        void handleTx();
        void handleTxLegacy();
        void handleTxAdsl();    ///< NEW: transmit one ADS-L packet if ready
        void handleRx();
        void switchMode(uint8_t mode,bool bStartReceive = true);

        bool isNeighbor(MacAddr addr);
        uint8_t _actMode = 0;
        LoRaClass radio;

        void sendUdpData(const uint8_t *buffer,int len);
        uint32_t long legacy_next_tx = 0;
        uint32_t long adsl_next_tx = 0;
        uint8_t FlarmBuffer [26];

        // ── ADS-L state ──────────────────────────────────────────────────────
        /// Buffer for one outbound ADS-L packet (ADSL_PACKET_SIZE = 27 bytes)
        uint8_t  _adslBuffer[32];
        uint8_t  _adslEncodedBuffer[64]; // Manchester Buffer
        /// Toggle flag: false=868.2 MHz, true=868.4 MHz (alternates each TX)
        bool     _adslFreqToggle = false;
        /// Timestamp of last ADS-L transmission [ms]
        unsigned long _adsl_last_tx = 0;
        /// Timestamp when FLARM 8682 TX completed; ADS-L fires shortly after.
        uint32_t _flarm8682TxDone = 0;

        uint32_t _ppsMillis = 0;
        uint8_t _ppsCount = 0;
        eLoraRegion eLoraRegion = NONE;
        uint8_t flarmZone = 0;
        uint16_t loraBandwidth = 0;
        uint8_t flarmChannels = 0;
        uint32_t ChanSepar = 200000;

public:

  bool doForward = true;
  float lat = 0;
  float lon = 0;
  float geoidAlt = 0;
  bool bPPS = false;
        bool bHasGPS = false;
        uRfMode _RfMode;
        uint16_t txFntCount = 0;
        uint16_t rxFntCount = 0;
        uint16_t txLegCount = 0;
        uint16_t txAdslCount = 0;  ///< NEW: ADS-L TX packet counter
        uint16_t rxLegCount = 0;
        uint32_t loraFrequency = 0;
        uint32_t flarmFrequency = 0;
        uint32_t actflarmFreq = 0;
        long _frequencyCorrection = 0;

        FanetMac() : myTimer(MAC_SLOT_MS, stateWrapper), myAddr(_myAddr) { }
        ~FanetMac() { }

        bool begin(int8_t sck, int8_t miso, int8_t mosi, int8_t ss,int8_t reset, int8_t dio0,int8_t gpio,Fapp &app,long frequCor,uint8_t level,uint8_t radioChip);
        void end();
        void handle() { radio.run(); myTimer.Update();  }
        float getAirtime();
        int getTxQueueLength();
        bool txQueueDepleted(void) { return (tx_fifo.size() == 0); }
        bool txQueueHasFreeSlots(void){ return (tx_fifo.size() < MAC_FIFO_SIZE); }
        int transmit(Frame *frm) { return tx_fifo.add(frm); }

        uint16_t numNeighbors(void) { return neighbors.size(); }
        uint16_t numTrackingNeighbors(void);
        void setRfMode(uint8_t mode);
        void setRegion(float lat, float lon);
        void setPps(uint32_t *ppsMillis);
        /* Addr */
        const MacAddr &myAddr;
        bool setAddr(MacAddr addr);
        bool setAddr(uint32_t addr);
        bool eraseAddr(void);
        MacAddr readAddr(bool getHwAddr=false);
};

extern FanetMac fmac;

#endif /* FANET_STACK_FMAC_H_ */