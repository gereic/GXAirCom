# GxAirCom

GxAircom aims to be a complete and open source implementation of the [FANET+ (Fanet + Flarm) protocol](https://github.com/3s1d/fanet-stm32/blob/master/Src/fanet/radio/protocol.txt) running on readily available cheap lora modules and interfacing with mobile phones via Bluetooth. It can also act as a Fanet ground station and broadcast recieved FANET information to [OGN](http://wiki.glidernet.org).

For information and documentation see:

- [The PDF quick guide](doc/20200908%20-%20GXAirCom%20-%20Quick%20Guide.pdf) and the [The PDF Documentation](doc/20200723%20-%20GXAirCom%20-%20A%20LoRa%20communication%20device%20for%20free%20flying.pdf)
- Further information can be found in [the wiki](https://github.com/gereic/GXAirCom/wiki) and [the docs folder](doc/). E.g.:
    - [The list of supported hardware](https://github.com/gereic/GXAirCom/wiki/Hardware-supported)
    - [The list of supported smartphone software](https://github.com/gereic/GXAirCom/wiki/Software)
    - [How to update the firmware](https://github.com/gereic/GXAirCom/wiki/Upgrading---updating-the-firmware-using-the-internal-web-interface-and-a-cellphone)
    - [See the video tutorials](https://github.com/gereic/GXAirCom/wiki/Video-Tutorials)

## ADS-L (SRD-860 M-Band) Support

GXAirCom implements the EASA **ADS-L 4 SRD-860** iConspicuity protocol (Issue 1 / Issue 2, December 2025).  
ADS-L lets non-certified aircraft broadcast and receive position data on a standardised EU frequency plan, independently of FLARM.

### Protocol overview

| Parameter | Value |
|---|---|
| Frequencies | 868.200 MHz and 868.400 MHz (M-Band), alternated per transmission |
| Modulation | 2-GFSK, 100 kbps chip rate (50 kbps net after Manchester) |
| Deviation | ±50 kHz |
| RX bandwidth | 125 kHz |
| Encoding | Software Manchester — G.E. Thomas convention (`0`→`10`, `1`→`01`) |
| Hardware Manchester | Disabled (`RegPacketConfig1 = 0x00`) |

### Packet structure (25 decoded bytes)

```
Byte  0      Length byte = 0x18 (24 bytes follow)
Byte  1      Network byte: Protocol=0, SigFlag=0, KeyIdx=0, ECM=0
Bytes 2–21   ADS-L Data (20 bytes) — XXTEA-scrambled
               Byte  2     Payload Type = 0x02 (Traffic / iConspicuity)
               Bytes 3–5   Sender address (24-bit, MSB first)
               Byte  6     AMT (6 bits, bits[7:2]) | Relay (bit 0)
               Bytes 7–21  iConspicuity payload (15 bytes = 120 bits)
Bytes 22–24  CRC-24 (big-endian, covers bytes 1–21)
```

On air the 25-byte packet is Manchester-encoded to **50 bytes**, transmitted after the sync word.

### iConspicuity payload bit layout (120 bits, MSB-first)

| Bits | Width | Field | Unit / notes |
|---|---|---|---|
| 0–5 | 6 | Timestamp | Quarter-seconds since UTC hour, mod 60 (0–59) |
| 6–7 | 2 | Flight State | 0=undefined, 1=ground, 2=airborne |
| 8–12 | 5 | Aircraft Category | 0=none … 13=UAS certified (EASA Issue 2 table) |
| 13–15 | 3 | Emergency Status | 0=none, 2=general, 3=medical … |
| 16–39 | 24 | Latitude | Signed, LSB = 1°/93 206 (~1.2 m) |
| 40–63 | 24 | Longitude | Signed, LSB = 1°/46 603 (~1.2 m at equator) |
| 64–71 | 8 | Ground Speed | Exp-unsigned N=6, unit 0.25 m/s |
| 72–85 | 14 | Altitude WGS-84 | Exp-unsigned N=12, +320 m offset |
| 86–94 | 9 | Vertical Rate | Exp-signed N=6, unit 0.125 m/s (sign in bit 8) |
| 95–103 | 9 | Ground Track | Cyclic, 360°/512 per LSB |
| 104–105 | 2 | SIL | Source Integrity Level |
| 106–107 | 2 | DAL | Design Assurance Level |
| 108–111 | 4 | NIC | Navigation Integrity Category |
| 112–114 | 3 | HFOM | Horizontal Figure of Merit |
| 115–116 | 2 | VFOM | Vertical Figure of Merit |
| 117–118 | 2 | NACv | Navigation Accuracy for Velocity |
| 119 | 1 | Reserved | Always 0 |

Non-certified devices transmit SIL=0, DAL=0 (conservative defaults per spec appendix).

### Scrambling — XXTEA

ADS-L Data (bytes 2–21, 20 bytes = 5 × uint32\_t LE words) is scrambled with **XXTEA**, 6 rounds, all-zero 128-bit key.  
This provides basic obfuscation but not cryptographic security (the spec acknowledges this).

### CRC-24

Same polynomial as Mode-S / ADS-B: **0xFFFA0480** (generator degree 24).  
The CRC is computed over 21 bytes (Network byte + XXTEA-encrypted ADS-L Data) and appended big-endian.

### Sync word

The radio sync word is a **6-byte Manchester-encoded** pattern derived from raw bytes `0xF5 0x72 0x4B`:

```
55 99 95 A6 9A 65
```

This 6-byte form matches the **OpenACE** and **SoftRF** on-air pattern.  
The leading `55 99` = Manchester(`0xF5`) acts as the explicit preamble byte required on SX1262 hardware (see [issue #210](https://github.com/gereic/GXAirCom/issues/210)).  
The spec-defined sync bytes `0x72 0x4B` are encoded as `95 A6 9A 65`.

| Chip | Sync config | SyncWordLength |
|---|---|---|
| SX1276 | `RegSyncConfig = 0x35` (SyncOn, SyncSize=6) | 6 bytes |
| SX1262 | SetPacketParams, 48 bits | 6 bytes written to register `0x06C0` |

### Address Mapping Table (AMT)

GXAirCom uses AMT = **8 (FANET)** by default. Other notable values: 5=ICAO, 6=FLARM, 7=OGN, 0=Random/Privacy.

### Interoperability

| System | Status |
|---|---|
| OpenACE | ✅ Compatible — same 6-byte sync, same packet structure |
| SoftRF (moshe-braner fork) | ✅ Compatible — same sync word pattern confirmed |
| OGN receivers | ✅ CRC-24 + packet structure matches Mode-S tooling |

Similar/ related projects are:

- The [SoftRF](https://github.com/lyusupov/SoftRF) project, which has wider hardware and protocol support, but implements only the subset of the FANET protocol that broadcasts and receives locations. It also cannot broadcast FANET and FLARM at the same time.
- The [Skytraxx FANET Source](https://github.com/3s1d/fanet-stm32). The original reference implementation of the FANET standard and upstream of the standard protocol specification and documentation.

