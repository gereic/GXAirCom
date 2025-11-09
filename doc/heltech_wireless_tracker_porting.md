# GXAirCom - Heltec Wireless Tracker Board Porting Guide

## 🔌 Quick Start - Hardware Connections

### Recommended External Components

#### 📊 Barometric Sensor (Essential for Altitude)
Connect a **BMP280** or **BME280** I2C sensor:

| Sensor Pin | ESP32-S3 GPIO | Wire Color (suggested) |
|------------|---------------|------------------------|
| VCC        | 3.3V          | Red                    |
| GND        | GND           | Black                  |
| SDA        | **GPIO 45**   | Blue/Green             |
| SCL        | **GPIO 46**   | Yellow/White           |

**Recommended Modules to Buy:**
- **BMP280**: ~$2-3 USD
  - Adafruit BMP280 I2C/SPI Barometric Pressure & Altitude Sensor
  - GY-BMP280-3.3 Module (Amazon, eBay, AliExpress)
  - Sparkfun Atmospheric Sensor Breakout - BMP280
  
- **BME280**: ~$4-6 USD (includes humidity sensor)
  - Adafruit BME280 I2C or SPI Temperature Humidity Pressure Sensor
  - GY-BME280-3.3 Module
  - Bosch BME280 Breakout Board

**I2C Address:** Default 0x76 or 0x77 (check your module's documentation)

#### 🔊 Buzzer (Optional - for audio alerts)
Connect a **piezo buzzer** or **active buzzer**:

| Buzzer Pin | ESP32-S3 GPIO | Wire Color (suggested) |
|------------|---------------|------------------------|
| Positive   | **GPIO 37**   | Red                    |
| Negative   | GND           | Black                  |

**Recommended Buzzers to Buy:**
- **Passive Piezo Buzzer**: ~$0.50-1 USD (requires PWM)
  - 5V/3.3V Piezo Buzzer Module (KY-006 or similar)
  - Murata 7BB-20-6L0 or PKM13EPYH4000-A0
  
- **Active Buzzer**: ~$1-2 USD (simpler, just on/off)
  - 3.3V Active Buzzer Module
  - HXD Buzzer Module
  - TMB09A05 or TMB12A05

**Alternative GPIO Pins for Buzzer:** 4, 7, 15, 16, 17, 19, 20

---

## 📋 Table of Contents

1. [Hardware Overview](#hardware-overview)
2. [Pin Mapping Reference](#pin-mapping-reference)
3. [Critical Power Configuration](#critical-power-configuration)
4. [Software Configuration](#software-configuration)
5. [Common Issues & Solutions](#common-issues--solutions)
6. [Testing & Validation](#testing--validation)
7. [Complete Wiring Diagram](#complete-wiring-diagram)

---

## 🔧 Hardware Overview

### Heltec Wireless Tracker V1.1 Specifications

**CPU:** ESP32-S3FN8
- Dual-core Xtensa LX7 @ 240MHz
- 320KB SRAM
- 8MB Flash (no PSRAM)

**Integrated Peripherals:**
- **LoRa Radio:** SX1262 (868/915MHz with TCXO 1.8V)
- **GPS Module:** UC6580 (multi-GNSS: GPS, GLONASS, Galileo, BeiDou)
- **Display:** ST7735S 1.14" TFT LCD (160x80 pixels, 65K colors)
- **Battery Monitoring:** Built-in ADC with voltage divider
- **Antenna:** Integrated LoRa antenna boost amplifier

**Physical Features:**
- Compact size: ~42mm x 32mm
- USB-C connector for programming and power
- BOOT button for user input
- White LED (GPIO 18)

---

## 📍 Pin Mapping Reference

### Complete GPIO Assignment

#### System & Control Pins

| Function        | GPIO | Direction | Notes                                    |
|----------------|------|-----------|------------------------------------------|
| BOOT Button    | 0    | Input     | User button, page navigation             |
| Battery ADC    | 1    | Input     | Voltage measurement (ADC1_GPIO1_CHANNEL) |
| ADC Control    | 2    | Output    | HIGH = Enable battery voltage divider    |
| **Vext Power** | **3** | **Output** | **HIGH = Power GPS/LoRa antenna (CRITICAL!)** |
| White LED      | 18   | Output    | Built-in LED (disabled in code)          |

#### LoRa SX1262 (SPI2_HOST/HSPI - Default SPI Bus)

| Function       | GPIO | Direction | Notes                        |
|---------------|------|-----------|------------------------------|
| LORA_SS       | 8    | Output    | SPI Chip Select              |
| LORA_SCK      | 9    | Output    | SPI Clock                    |
| LORA_MOSI     | 10   | Output    | SPI Master Out Slave In      |
| LORA_MISO     | 11   | Input     | SPI Master In Slave Out      |
| LORA_RESET    | 12   | Output    | LoRa module reset            |
| LORA_BUSY     | 13   | Input     | SX1262 BUSY (DIO2)           |
| LORA_IRQ      | 14   | Input     | SX1262 DIO1 (interrupt)      |

**CRITICAL:** LoRa antenna power is controlled by Vext (pin 3). Must be HIGH!

#### TFT Display ST7735S (HSPI - Separate SPI Bus)

| Function       | GPIO | Direction | Notes                        |
|---------------|------|-----------|------------------------------|
| TFT_CS        | 38   | Output    | SPI Chip Select              |
| TFT_RST       | 39   | Output    | Display Reset                |
| TFT_DC        | 40   | Output    | Data/Command (RS)            |
| TFT_SCK       | 41   | Output    | SPI Clock                    |
| TFT_MOSI      | 42   | Output    | SPI Master Out Slave In      |
| TFT_BL        | 21   | Output    | Backlight Control            |

**NOTE:** TFT uses HSPI bus (separate from LoRa's default SPI) to avoid conflicts.

#### GPS UC6580 (UART)

| Function       | GPIO | Direction | Notes                        |
|---------------|------|-----------|------------------------------|
| GPS_RX        | 33   | Input     | ESP32 RX ← GPS TX            |
| GPS_TX        | 34   | Output    | ESP32 TX → GPS RX            |
| GPS_RESET     | 35   | Output    | GPS module reset (optional)  |
| **GPS_PPS**   | **36** | **Input** | **Pulse Per Second (DO NOT DRIVE!)** |
| GPS_EN        | 37   | -         | V1.0 only (unused on V1.1)   |

**CRITICAL:** Pin 36 is GPS PPS OUTPUT. Never configure as OUTPUT or drive LOW!

**GPS Baudrate:** 115200 bps (UC6580 default)

#### I2C Bus (External Sensors)

| Function       | GPIO | Direction | Notes                        |
|---------------|------|-----------|------------------------------|
| **I2C_SDA**   | **45** | Bi-dir  | **Official I2C Data (recommended)** |
| **I2C_SCL**   | **46** | Output  | **Official I2C Clock (recommended)** |
| I2C_SDA_ALT   | 5    | Bi-dir    | Alternative if 45/46 needed  |
| I2C_SCL_ALT   | 6    | Output    | Alternative if 45/46 needed  |

**For BMP280/BME280:** Use GPIO 45 (SDA) and 46 (SCL)

#### PWM/GPIO (Buzzer & Expansion)

| Function       | GPIO | Type      | Notes                        |
|---------------|------|-----------|------------------------------|
| **BUZZER**    | **37** | **PWM**  | **Best choice (unused on V1.1)** |
| Alternative   | 4    | GPIO/PWM  | Available for expansion      |
| Alternative   | 7    | GPIO/PWM  | Available for expansion      |
| Alternative   | 15   | GPIO/PWM  | Available for expansion      |
| Alternative   | 16   | GPIO/PWM  | Available for expansion      |
| Alternative   | 17   | GPIO/PWM  | Available for expansion      |
| Alternative   | 19   | GPIO/PWM  | Available for expansion      |
| Alternative   | 20   | GPIO/PWM  | Available for expansion      |

#### USB Serial (Use with Caution)

| Function       | GPIO | Direction | Notes                        |
|---------------|------|-----------|------------------------------|
| USB_TX        | 43   | Output    | USB serial TX (programming)  |
| USB_RX        | 44   | Input     | USB serial RX (programming)  |

**WARNING:** These pins are used for USB programming. Do not connect external devices.

---

## ⚡ Critical Power Configuration

### Vext Power Rail (Pin 3) - MOST IMPORTANT!

**Pin 3 controls THREE critical subsystems on V1.1:**

```
GPIO 3 (Vext) = HIGH  →  Powers:
    ├─ GPS Module (UC6580 VDD_IO, DCDC_IN)
    ├─ GPS LNA (SW7125DE amplifier)
    └─ LoRa Antenna LNA (boost amplifier)
```

**In Code:**
```cpp
// CRITICAL: Enable Vext (pin 3) - active HIGH
pinMode(3, OUTPUT);
digitalWrite(3, HIGH);
delay(200); // Wait for power rails to stabilize
```

### Power Sequence

1. **Power On (Boot):**
   - Pin 3 → HIGH (enables GPS + LoRa antenna)
   - Pin 2 → HIGH (enables battery ADC divider)
   - Delay 200ms for rail stabilization
   - Initialize LoRa radio
   - Initialize GPS at 115200 baud
   - Initialize TFT display (after LoRa to avoid conflicts)

2. **Important Delays:**
   - After Vext enable: 200ms
   - Before TFT init: 5000ms (taskTft startup delay)
   - TFT refresh rate: 50ms

### Pin 36 Warning ⚠️

**NEVER configure GPIO 36 as OUTPUT!**

Pin 36 is the GPS PPS (Pulse Per Second) OUTPUT signal from the GPS module. Driving this pin LOW or configuring it as OUTPUT will:
- Short-circuit the GPS PPS output
- Potentially damage the GPS module
- Cause erratic GPS behavior

**Correct Configuration:**
```cpp
// Pin 36 is GPS PPS INPUT - do NOT initialize!
// Leave it as high-impedance input (default state)
```

**Wrong Configuration (DO NOT DO THIS!):**
```cpp
// WRONG! This will damage the GPS!
pinMode(36, OUTPUT);
digitalWrite(36, LOW);
```

---

## 💻 Software Configuration

### PlatformIO Configuration

**File:** `platformio.ini`

```ini
[env:Wireless_Tracker]
platform = espressif32@6.7.0
board = esp32-s3-devkitc-1
framework = arduino
board_build.arduino.memory_type = qio_opi
board_build.f_flash = 80000000L
board_build.flash_mode = qio
board_build.partitions = min_spiffs.csv
build_flags = 
    -D ARDUINO_ESP32S3_DEV
    -D BOARD_HAS_PSRAM=0
    -D HELTEC_WIRELESS_TRACKER
lib_deps = 
    adafruit/Adafruit ST7735 and ST7789 Library@^1.11.0
    adafruit/Adafruit GFX Library@^1.11.11
```

### Main Code Configuration

**File:** `src/main.cpp`

#### Board Detection (Lines ~2437-2520)

```cpp
case eBoard::HELTEC_WIRELESS_TRACKER:
    log_i("Board=HELTEC_WIRELESS_TRACKER");

    // Set TFT display type
    setting.displayType = TFT160x80;
    
    // Set GPS baudrate for UC6580 module
    setting.gps.Baud = 115200;

    // SX1262 LoRa pins
    PinLora_SS = 8;
    PinLora_SCK = 9;
    PinLora_MOSI = 10;
    PinLora_MISO = 11;
    PinLoraRst = 12;
    PinLoraGPIO = 13; // SX1262 BUSY
    PinLoraDI0 = 14;  // SX1262 IRQ

    // ST7735S TFT LCD pins (HSPI)
    PinTftCS = 38;
    PinTftDC = 40;
    PinTftRst = 39;
    PinTftMOSI = 42;
    PinTftSCK = 41;
    PinTftBL = 21;

    // GPS UC6580 pins
    PinGPSRX = 33;
    PinGPSTX = 34;

    // I2C pins (for BMP280/BME280)
    PinBaroSDA = 45;  // Use official I2C pins
    PinBaroSCL = 46;

    // User button
    sButton[0].PinButton = 0;
    PinUserLed = -1; // Disable LED (annoying)
    
    // Disable white LED
    pinMode(18, OUTPUT);
    digitalWrite(18, LOW);

    // Battery monitoring
    PinADCVoltage = 1;
    adcVoltageMultiplier = 4.9f * 1.045f;
    
    // Enable ADC voltage divider
    pinMode(2, OUTPUT);
    digitalWrite(2, HIGH);

    // CRITICAL: Enable Vext (pin 3) - active HIGH
    pinMode(3, OUTPUT);
    digitalWrite(3, HIGH);
    delay(200); // Power stabilization
    
    // NOTE: Pin 36 is GPS PPS - do NOT initialize!
    
    // Enable TFT backlight
    pinMode(PinTftBL, OUTPUT);
    digitalWrite(PinTftBL, HIGH);

    pI2cOne->begin(PinBaroSDA, PinBaroSCL);
    break;
```

### TFT Display Driver

**File:** `src/tft.cpp`

**Critical Fix:** Use HSPI bus to avoid conflict with LoRa:

```cpp
bool Tft::begin(int8_t cs, int8_t dc, int8_t rst, int8_t mosi, int8_t sck, int8_t bl){
    pinCS = cs;
    pinDC = dc;
    pinRst = rst;
    pinBL = bl;
    
    // CRITICAL: Use HSPI (separate from LoRa's default SPI)
    tftSPI = new SPIClass(HSPI);
    
    // Initialize HSPI bus with custom pins
    tftSPI->begin(sck, -1, mosi, -1);
    
    // Create ST7735 display instance
    display = new Adafruit_ST7735(tftSPI, cs, dc, rst);
    
    // Initialize ST7735S for 160x80 display
    display->initR(INITR_MINI160x80_PLUGIN);
    display->setRotation(3); // Landscape
    
    // Turn on backlight
    if (pinBL >= 0) {
        pinMode(pinBL, OUTPUT);
        digitalWrite(pinBL, HIGH);
    }
    
    bDisplayOn = true;
    display->fillScreen(ST77XX_BLACK);
    
    return true;
}
```

**Why HSPI?**
- LoRa uses default SPI (SPI2_HOST/VSPI)
- TFT must use HSPI to avoid bus conflicts
- Original code used undefined `FSPI` constant, causing both peripherals to share same bus
- Result: LoRa worked until TFT initialized, then corrupted LoRa communications

### FANET/LoRa TCXO Configuration

**File:** `lib/FANETLORA/radio/LoRa.cpp` (Line 394)

```cpp
// Set TCXO voltage for SX1262
data[0] = 0x02; // DIO3 outputs 1.8V (correct for Heltec hardware)
```

**Important:** Heltec Wireless Tracker uses 1.8V TCXO. Do not change this value!

### RTOS Task Configuration

**File:** `src/main.cpp`

```cpp
// Task stack sizes and priorities
xTaskCreatePinnedToCore(taskTft, "taskTft", 6500, NULL, 8, NULL, ARDUINO_RUNNING_CORE1);
xTaskCreatePinnedToCore(taskBackGround, "taskBackGround", 8192, NULL, 5, NULL, ARDUINO_RUNNING_CORE1);
xTaskCreatePinnedToCore(taskStandard, "taskStandard", 6500, NULL, 10, NULL, ARDUINO_RUNNING_CORE1);
```

**Task Details:**
- **taskTft:** 6500 bytes, priority 8, 50ms refresh, 5000ms startup delay
- **taskBackGround:** 8192 bytes, priority 5 (increased to prevent stack overflow)
- **taskStandard:** 6500 bytes, priority 10

---

## 🔍 Common Issues & Solutions

### Issue 1: LoRa TX Timeout (`state=-5` errors)

**Symptoms:**
```
[E] error TX state=-5
[E] error writing state=255 cmd=0X80
```

**Root Cause:** LoRa antenna not powered (Vext pin 3 not HIGH)

**Solution:**
```cpp
pinMode(3, OUTPUT);
digitalWrite(3, HIGH); // Enable LoRa antenna LNA
delay(200);
```

### Issue 2: LoRa Works at Boot, Fails After TFT Init

**Symptoms:**
```
[I] LoRa Initialization OK!
[I] TFT Display initialized: 160x80
[E] error writing state=255 cmd=0X80  // After TFT init
```

**Root Cause:** TFT and LoRa sharing same SPI bus due to incorrect `FSPI` constant

**Solution:** Use `HSPI` instead:
```cpp
tftSPI = new SPIClass(HSPI); // NOT FSPI!
```

### Issue 3: GPS Not Working or Erratic Behavior

**Symptoms:**
- No GPS fix
- GPS resets randomly
- Satellites appear/disappear

**Root Causes:**
1. GPS not powered (Vext pin 3 not HIGH)
2. Wrong baudrate (should be 115200 for UC6580)
3. **Pin 36 configured as OUTPUT** (shorting GPS PPS!)

**Solution:**
```cpp
// Power GPS
pinMode(3, OUTPUT);
digitalWrite(3, HIGH);

// Set correct baudrate
setting.gps.Baud = 115200;

// DO NOT touch pin 36!
// pinMode(36, OUTPUT);  // WRONG!
// digitalWrite(36, LOW); // WRONG!
```

### Issue 4: TFT Display Flickering

**Symptoms:**
- Screen updates cause full refresh
- Text flickers when changing pages

**Solution:** Implemented partial screen updates with change detection in `tft.cpp`

### Issue 5: Stack Overflow in taskBackGround

**Symptoms:**
```
***ERROR*** A stack overflow in task taskBackGround has been detected.
```

**Solution:** Increase stack size from 6500 to 8192 bytes:
```cpp
xTaskCreatePinnedToCore(taskBackGround, "taskBackGround", 8192, NULL, 5, ...);
```

### Issue 6: Compilation Error - `SPI3_HOST` not declared

**Symptoms:**
```
error: 'SPI3_HOST' was not declared in this scope
```

**Solution:** ESP32-S3 Arduino framework doesn't define `SPI3_HOST`. Use `HSPI` instead:
```cpp
tftSPI = new SPIClass(HSPI); // Correct
// tftSPI = new SPIClass(SPI3_HOST); // Wrong - not defined
```

---

## ✅ Testing & Validation

### Step 1: Power-On Test

Connect USB-C and monitor serial output:

```
[I] Board=HELTEC_WIRELESS_TRACKER
[I] gps: baudrate with 115200 ok
[I] LoRa Initialization OK!
[I] TFT Display initialized: 160x80
[I] GPS: Sats=X Valid=1
```

**Expected:** All subsystems initialize without errors.

### Step 2: GPS Lock Test

Place device near window or outdoors:

```
[I] GPS: Sats=6 Valid=1 Lat=XX.XXXXXX Lon=X.XXXXXX Alt=XXX.X
```

**Expected:** 
- 4+ satellites within 30 seconds (cold start)
- Valid fix within 60 seconds
- 10+ satellites within 2 minutes

### Step 3: TFT Display Test

Press BOOT button to cycle through pages:

**Page 0:** Main GPS
- Satellite count
- GPS coordinates
- Altitude
- Speed

**Page 1:** System Info
- Battery voltage
- WiFi status
- BLE status
- Flying status

**Page 2:** Detailed GPS
- HDOP
- Fix type
- Satellite details

**Expected:** Pages switch instantly without flickering.

### Step 4: LoRa Transmission Test

Monitor serial for successful transmissions:

```
[I] set Lora-Region to EU868
[I] Zone 1: Europe, Africa, Russia, China
```

**Expected:** No `state=255` or `state=-5` errors.

### Step 5: Web Interface Test

1. Connect to WiFi AP: `GXAirCom-XXXXXX`
2. Open browser: `http://192.168.4.1`
3. Check configuration page

**Expected:** 
- TFT display option visible
- Wireless Tracker board option visible
- All settings accessible

### Step 6: BMP280/BME280 Test (if connected)

Monitor serial for barometric data:

```
[I] Baro: Pressure=XXXX.XX hPa Temp=XX.X°C Alt=XXX.Xm
```

**Expected:** Valid pressure/temperature readings.

### Step 7: Buzzer Test (if connected)

Enable buzzer in settings and trigger alert:

**Expected:** Buzzer sounds on specified events.

---

## 📐 Complete Wiring Diagram

### External Sensor Connections

```
┌─────────────────────────────────────────────────────────┐
│         Heltec Wireless Tracker V1.1                    │
│                                                          │
│  ┌──────────────┐                                       │
│  │  ESP32-S3    │                                       │
│  │              │                                       │
│  │  GPIO 45 ────┼────────────────┐ SDA (Blue)          │
│  │  GPIO 46 ────┼─────────────┐  │ SCL (Yellow)        │
│  │              │             │  │                      │
│  │  GPIO 37 ────┼──────┐      │  │                     │
│  │              │      │      │  │                     │
│  │  3.3V    ────┼───┐  │      │  │                     │
│  │  GND     ────┼─┐ │  │      │  │                     │
│  └──────────────┘ │ │  │      │  │                     │
└───────────────────┼─┼──┼──────┼──┼─────────────────────┘
                    │ │  │      │  │
                    │ │  │      │  │
    ┌───────────────┼─┼──┘      │  │
    │ GND (Black)   │ │         │  │
    │ VCC (Red)  ───┼─┘         │  │
    │ SCL (Yellow) ─┼───────────┘  │
    │ SDA (Blue)  ──┼──────────────┘
    │               │
    │  ┌─────────┐  │
    │  │ BMP280/ │  │
    │  │ BME280  │  │
    │  │ Module  │  │
    │  └─────────┘  │
    │               │
    └───────────────┘

                    ┌──────────────┐
                    │   Buzzer     │
                    │  (Active or  │
                    │   Passive)   │
                    └──────┬───┬───┘
                           │   │
                     (+) GPIO 37│
                           GND (-)
```

### Full Pin Assignment Summary

```
╔════════════════════════════════════════════════════════╗
║          Heltec Wireless Tracker V1.1                  ║
║              Complete Pin Assignment                    ║
╠════════════════════════════════════════════════════════╣
║ POWER & CONTROL                                        ║
║  GPIO 0  : BOOT Button (User Input)                    ║
║  GPIO 1  : Battery ADC (Voltage Measurement)           ║
║  GPIO 2  : ADC Control (Enable Divider)                ║
║ ⚡GPIO 3 : Vext Power (GPS + LoRa Antenna) - HIGH!    ║
║  GPIO 18 : White LED (Disabled in Code)                ║
╠════════════════════════════════════════════════════════╣
║ LORA SX1262 (Default SPI - SPI2_HOST)                  ║
║  GPIO 8  : SS (Chip Select)                            ║
║  GPIO 9  : SCK (SPI Clock)                             ║
║  GPIO 10 : MOSI (SPI Data Out)                         ║
║  GPIO 11 : MISO (SPI Data In)                          ║
║  GPIO 12 : RESET                                       ║
║  GPIO 13 : BUSY (SX1262 DIO2)                          ║
║  GPIO 14 : IRQ (SX1262 DIO1)                           ║
╠════════════════════════════════════════════════════════╣
║ TFT ST7735S (HSPI - Separate SPI Bus)                  ║
║  GPIO 38 : CS (Chip Select)                            ║
║  GPIO 39 : RESET                                       ║
║  GPIO 40 : DC (Data/Command)                           ║
║  GPIO 41 : SCK (SPI Clock)                             ║
║  GPIO 42 : MOSI (SPI Data Out)                         ║
║  GPIO 21 : BL (Backlight Control)                      ║
╠════════════════════════════════════════════════════════╣
║ GPS UC6580 (UART @ 115200 baud)                        ║
║  GPIO 33 : RX (ESP32 RX ← GPS TX)                      ║
║  GPIO 34 : TX (ESP32 TX → GPS RX)                      ║
║  GPIO 35 : RESET (Optional)                            ║
║ ⚠️GPIO 36: PPS (Input Only - DO NOT DRIVE!)           ║
║  GPIO 37 : EN (V1.0 only - unused on V1.1)             ║
╠════════════════════════════════════════════════════════╣
║ I2C BUS (Barometric Sensor)                            ║
║ 📊GPIO 45: SDA (I2C Data) ← BMP280/BME280 SDA         ║
║ 📊GPIO 46: SCL (I2C Clock) ← BMP280/BME280 SCL        ║
║   Alt: 5/6 if 45/46 needed for other purposes         ║
╠════════════════════════════════════════════════════════╣
║ PWM/GPIO (Buzzer & Expansion)                          ║
║ 🔊GPIO 37: BUZZER (Recommended - PWM)                 ║
║   GPIO 4 : Available                                   ║
║   GPIO 7 : Available                                   ║
║   GPIO 15: Available                                   ║
║   GPIO 16: Available                                   ║
║   GPIO 17: Available                                   ║
║   GPIO 19: Available                                   ║
║   GPIO 20: Available                                   ║
╠════════════════════════════════════════════════════════╣
║ USB SERIAL (Programming - Do Not Use!)                 ║
║  GPIO 43 : USB TX                                      ║
║  GPIO 44 : USB RX                                      ║
╚════════════════════════════════════════════════════════╝
```

---

## 📚 Additional Resources

### Official Heltec Documentation
- [Heltec Wireless Tracker Product Page](https://heltec.org/project/wireless-tracker/)
- [ESP32-S3 Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf)
- [SX1262 Datasheet](https://www.semtech.com/products/wireless-rf/lora-core/sx1262)
- [UC6580 GPS Module Documentation](https://www.unicorecomm.com/products/detail/24)

### Reference Implementations
- [Meshtastic Firmware - Heltec Wireless Tracker](https://github.com/meshtastic/firmware/tree/master/variants/esp32s3/heltec_wireless_tracker)
- [Arduino ESP32 Core](https://github.com/espressif/arduino-esp32)

### Sensor Datasheets
- [BMP280 Datasheet (Bosch)](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf)
- [BME280 Datasheet (Bosch)](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf)

### Where to Buy Components

**BMP280/BME280 Modules:**
- Adafruit: https://www.adafruit.com/
- SparkFun: https://www.sparkfun.com/
- Amazon: Search "BMP280 module" or "BME280 module"
- AliExpress: Search "GY-BMP280" or "GY-BME280"

**Buzzers:**
- Amazon: Search "3.3V buzzer module"
- eBay: Search "piezo buzzer 3.3V"
- AliExpress: Search "active buzzer module"

---

## 🎯 Summary Checklist

Before declaring the port complete, verify:

- [x] Pin 3 (Vext) set to HIGH for GPS/LoRa antenna power
- [x] Pin 36 (GPS PPS) left as input, never driven
- [x] TFT uses HSPI bus (not default SPI)
- [x] GPS baudrate set to 115200
- [x] LoRa TCXO voltage set to 1.8V
- [x] Task stack sizes adequate (taskBackGround = 8192)
- [x] TFT initialization delayed 5000ms after boot
- [x] I2C pins 45/46 configured for barometric sensor
- [x] Buzzer pin 37 available for PWM
- [x] All subsystems initialize without errors
- [x] GPS achieves satellite lock
- [x] LoRa transmissions succeed without timeouts
- [x] TFT display pages switch without flickering
- [x] Web interface accessible and functional

---

## 🏆 Success Criteria

Your Heltec Wireless Tracker port is successful when:

1. **GPS:** 10+ satellites, valid fix, accurate position
2. **LoRa:** Successful initialization, no `state=255` or `state=-5` errors
3. **Display:** 3 pages, instant switching, no flickering
4. **System:** No crashes, no stack overflows, stable operation
5. **Sensors:** BMP280/BME280 reporting valid pressure/temperature
6. **Buzzer:** PWM tones working correctly (if implemented)

**Congratulations!** You have successfully ported GXAirCom to the Heltec Wireless Tracker! 🎉

---

## 📝 Version History

- **v1.0** (2025-11-09): Initial porting documentation
  - Complete pin mapping
  - Critical power configuration identified
  - SPI bus conflict resolution (TFT HSPI vs LoRa default SPI)
  - GPS PPS pin 36 warning added
  - BMP280/BME280 and buzzer connection guide
  - Complete troubleshooting section

---

## 📧 Support & Contributions

For issues, improvements, or questions:
- GitHub Repository: [GXAirCom](https://github.com/Martenz/GXAirCom)
- Branch: `heltech-wireless-tracker-s3`

**Key Contributors:**
- Original GXAirCom firmware by community
- Heltec Wireless Tracker port by @Martenz (2025)

---

**Last Updated:** 2025-11-09  
**Board:** Heltec Wireless Tracker V1.1 (ESP32-S3)  
**Firmware:** GXAirCom v8.3.0
