#include "tft.h"
#include "tools.h"
#include "icons.h"

Tft::Tft(){
  oldScreenNumber = 0;
  splashShown = false;
  splashStartTime = 0;
  autoAdvanceDone = false;
}

bool Tft::begin(int8_t cs, int8_t dc, int8_t rst, int8_t mosi, int8_t sck, int8_t bl){
  pinCS = cs;
  pinDC = dc;
  pinRst = rst;
  pinBL = bl;
  
  // CRITICAL FIX: Use HSPI (separate from default VSPI) to avoid conflict with LoRa
  // On ESP32-S3, we need to use a different SPI peripheral than LoRa
  // HSPI = bus ID 2, VSPI = bus ID 3 (default)
  // LoRa uses default SPI, TFT must use HSPI with custom pins
  tftSPI = new SPIClass(HSPI);
  
  // Initialize HSPI bus with custom pins: SCK, MISO (not used), MOSI, SS (not used)
  tftSPI->begin(sck, -1, mosi, -1);
  
  // Create ST7735 display instance with CS, DC, RST pins
  display = new Adafruit_ST7735(tftSPI, cs, dc, rst);
  
  if (!display) {
    log_e("Failed to create TFT display");
    return false;
  }
  
  // Initialize ST7735S
  display->initR(INITR_MINI160x80_PLUGIN); // For 160x80 display
  display->setRotation(3); // Landscape orientation
  
  // Turn on backlight
  if (pinBL >= 0) {
    pinMode(pinBL, OUTPUT);
    digitalWrite(pinBL, HIGH);
  }
  
  bDisplayOn = true;
  
  // Clear screen with black background
  display->fillScreen(ST77XX_BLACK);
  
  log_i("TFT Display initialized: 160x80");
  
  return true;
}

void Tft::end(void){
  PowerOff();
}

void Tft::PowerOn(void){
  if (bDisplayOn){
    return; // already on
  }
  
  if (pinBL >= 0){
    digitalWrite(pinBL, HIGH);
  }
  
  bDisplayOn = true;
  display->fillScreen(ST77XX_BLACK);
}

void Tft::PowerOff(void){
  if (!bDisplayOn){
    return; // already off
  }
  
  if (pinBL >= 0){
    digitalWrite(pinBL, LOW);
  }
  
  bDisplayOn = false;
}

String Tft::setStringSize(String s, uint8_t sLen){
  uint8_t actLen = (uint8_t)s.length();
  String sRet = "";
  for (uint8_t i = actLen; i < sLen; i++){
    sRet += " ";
  }
  sRet += s;
  return sRet;
}

void Tft::drawSatCount(int16_t x, int16_t y, uint8_t value){
  // Draw satellite icon and count
  display->setTextSize(1);
  display->setCursor(x, y);
  display->setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  display->print("SAT:");
  display->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  display->print(value);
  display->print("  "); // Clear old values
}

void Tft::drawBatt(int16_t x, int16_t y, uint8_t value){
  // Draw battery percentage
  display->setTextSize(1);
  display->setCursor(x, y);
  
  // Check if battery is connected (voltage > 4500mV indicates USB without battery)
  if (status.battery.voltage > 4500) {
    // No battery connected, running on USB
    display->setTextColor(ST77XX_CYAN, ST77XX_BLACK);
    display->print("PWR: USB    ");  // Extra spaces to clear old percentage
  } else {
    // Battery connected, show percentage
    uint16_t color = ST77XX_GREEN;
    if (value < 20) color = ST77XX_RED;
    else if (value < 50) color = ST77XX_YELLOW;
    
    display->setTextColor(color, ST77XX_BLACK);
    display->print("BAT:");
    display->print(value);
    display->print("%   ");  // Extra spaces to clear old text
  }
}

void Tft::drawWifiStat(eConnectionState wifiStat){
  int16_t x = TFT_WIDTH - 30;
  int16_t y = 2;
  
  display->setTextSize(1);
  display->setCursor(x, y);
  
  if (wifiStat == eConnectionState::CONNECTED) {
    display->setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    display->print("WiFi");
  } else {
    display->setTextColor(ST77XX_RED, ST77XX_BLACK);
    display->print("----");
  }
}

void Tft::drawBluetooth(int16_t x, int16_t y, bool connected){
  display->setTextSize(1);
  display->setCursor(x, y);
  
  if (connected) {
    display->setTextColor(ST77XX_BLUE, ST77XX_BLACK);
    display->print("BT");
  } else {
    display->setTextColor(ST77XX_RED, ST77XX_BLACK);
    display->print("--");
  }
}

void Tft::drawFlying(int16_t x, int16_t y, bool isFlying){
  display->setTextSize(1);
  display->setCursor(x, y);
  
  if (isFlying) {
    display->setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    display->print("FLY");
  } else {
    display->setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    display->print("GND");
  }
}

// PAGE 0: Splash Screen with GXAirCom Logo and Version
void Tft::printSplashScreen(uint32_t tAct, bool forceRedraw){
  if (!bDisplayOn) return;
  
  static bool logoDrawn = false;
  
  // Record when splash screen first displayed
  if (!splashShown || forceRedraw) {
    splashStartTime = millis();
    splashShown = true;
  }
  
  // Only draw once or on force redraw
  if (!logoDrawn || forceRedraw) {
    display->fillScreen(ST77XX_BLACK);
    
    // Draw GXAirCom logo (scaled down to fit 160x80 display)
    // Original OLED uses: G at (1,2), X at (30,2), AirCom at (69,2)
    // For TFT 160x80, we'll center it better
    
    // Draw "G" logo (35x45 pixels) - position at x=10
    display->drawXBitmap(10, 10, G_Logo_bits, G_Logo_width, G_Logo_height, ST77XX_CYAN);
    
    // Draw "X" logo (49x60 pixels) - position at x=48 (10 + 35 + 3)
    // Scale down by only drawing part of it or use smaller Y offset
    display->drawXBitmap(48, 5, X_Logo_bits, X_Logo_width, X_Logo_height, ST77XX_CYAN);
    
    // Draw "AirCom" logo (59x42 pixels) - position at x=100 (48 + 49 + 3)
    display->drawXBitmap(100, 15, AirCom_Logo_bits, AirCom_Logo_width, AirCom_Logo_height, ST77XX_CYAN);
    
    // Draw device ID at bottom area
    display->setTextSize(1);
    display->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    display->setCursor(2, TFT_HEIGHT - 18);
    display->print("ID: ");
    display->setTextColor(ST77XX_CYAN, ST77XX_BLACK);
    if (setting.myDevId.length() > 0) {
      display->print(setting.myDevId.c_str());
      display->print("      ");  // Clear any old text
    } else {
      display->print("------      ");  // Placeholder if not yet initialized
    }
    
    // Draw version at bottom
    display->setTextSize(1);
    display->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    display->setCursor(2, TFT_HEIGHT - 8);
    display->print("Ver: ");
    display->setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    display->print(VERSION);
    display->print("      ");  // Clear any old text
    
    logoDrawn = true;
  }
  
  // Reset logoDrawn when leaving splash screen so it redraws next time
  if (forceRedraw) {
    logoDrawn = false;
  }
}

// PAGE 1: Main GPS Data (was Page 0)
void Tft::printGPSData(uint32_t tAct, bool forceRedraw){
  if (!bDisplayOn) return;
  
  static uint8_t lastNumSat = 0;  // Start with 0 instead of 255
  static uint8_t lastBattPercent = 255;
  static uint16_t lastBattVoltage = 0;
  static float lastClimbRate = 999.0;
  static float lastAlt = -9999.0;
  static float lastSpeed = -9999.0;
  static bool lastFlying = false;
  static eConnectionState lastWifiState = eConnectionState::IDLE;
  static uint8_t lastBluetooth = 255;
  
  // Check if this is first time on this page or forced redraw
  bool pageJustChanged = forceRedraw || (lastBattPercent == 255 && lastClimbRate == 999.0);
  
  // Draw static elements if page just loaded
  if (pageJustChanged || oldScreenNumber == 1) {
    display->setTextSize(1);
    display->setCursor(2, 2);
    display->setTextColor(ST77XX_CYAN, ST77XX_BLACK);
    display->print("P1:MAIN        ");
  }
  
  // Always update satellite count - show it always, keep last value
  if (lastNumSat != status.gps.NumSat || pageJustChanged) {
    lastNumSat = status.gps.NumSat;  // Update before drawing to keep last value
  }
  // Draw satellite count every time to ensure it stays visible
  drawSatCount(60, 2, lastNumSat);
  
  // Always update WiFi status
  if (lastWifiState != status.wifiSTA.state || pageJustChanged) {
    drawWifiStat(status.wifiSTA.state);
    lastWifiState = status.wifiSTA.state;
  }
  
  // Always update Bluetooth
  if (lastBluetooth != status.bluetoothStat || pageJustChanged) {
    drawBluetooth(TFT_WIDTH - 50, 2, (status.bluetoothStat > 0));
    lastBluetooth = status.bluetoothStat;
  }
  
  // Climb rate - BIG (always show, even if 0.0)
  if (abs(lastClimbRate - status.vario.ClimbRate) > 0.05 || pageJustChanged) {
    display->setTextSize(3);
    display->setCursor(2, 18);
    display->setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    display->print(setStringSize(String(status.vario.ClimbRate, 1) + "ms", 7));
    lastClimbRate = status.vario.ClimbRate;
  }
  
  // Altitude (always show)
  if (abs(lastAlt - status.gps.alt) > 0.5 || pageJustChanged) {
    display->setTextSize(2);
    display->setCursor(2, 46);
    display->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    display->print(setStringSize(String(status.gps.alt, 0) + "m", 6));
    lastAlt = status.gps.alt;
  }
  
  // Speed (always show)
  if (abs(lastSpeed - status.gps.speed) > 0.5 || pageJustChanged) {
    display->setTextSize(2);
    display->setCursor(95, 46);
    display->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    display->print(setStringSize(String(status.gps.speed, 0) + "kh", 5));
    lastSpeed = status.gps.speed;
  }
  
  // Battery (always show)
  if (lastBattPercent != status.battery.percent || lastBattVoltage != status.battery.voltage || pageJustChanged) {
    drawBatt(2, TFT_HEIGHT - 10, status.battery.percent);
    lastBattPercent = status.battery.percent;
    lastBattVoltage = status.battery.voltage;
  }
  
  // Flying status (always show)
  if (lastFlying != status.flying || pageJustChanged) {
    drawFlying(TFT_WIDTH - 30, TFT_HEIGHT - 10, status.flying);
    lastFlying = status.flying;
  }
}

// PAGE 1: System Info
void Tft::printSystemInfo(uint32_t tAct, bool forceRedraw){
  if (!bDisplayOn) return;
  
  static eConnectionState lastWifiState = eConnectionState::IDLE;
  static String lastIP = "";
  static uint16_t lastBattVoltage = 0;
  static uint8_t lastBattPercent = 255;
  static float lastTemp = -999.0;
  static float lastPressure = -999.0;
  static uint32_t lastHeap = 0;
  
  bool pageJustChanged = forceRedraw || (lastBattPercent == 255 && lastBattVoltage == 0);
  
  // Draw static elements
  if (pageJustChanged || oldScreenNumber == 2) {
    display->setTextSize(1);
    display->setCursor(2, 2);
    display->setTextColor(ST77XX_CYAN, ST77XX_BLACK);
    display->print("P2:SYSTEM INFO  ");
  }
  
  // Always update WiFi status
  if (lastWifiState != status.wifiSTA.state || pageJustChanged) {
    drawWifiStat(status.wifiSTA.state);
    lastWifiState = status.wifiSTA.state;
  }
  
  // WiFi IP (always show)
  if (lastIP != status.wifiSTA.ip || lastWifiState != status.wifiSTA.state || pageJustChanged) {
    display->setCursor(2, 14);
    display->setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    display->print("IP:");
    display->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    if (status.wifiSTA.state == eConnectionState::CONNECTED) {
      display->print(status.wifiSTA.ip);
      display->print("          "); // Clear old text
    } else {
      display->print("Not connected   ");
    }
    lastIP = status.wifiSTA.ip;
  }
  
  // Battery voltage (always show)
  if (lastBattVoltage != status.battery.voltage || lastBattPercent != status.battery.percent || pageJustChanged) {
    display->setCursor(2, 26);
    display->setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    display->print("Batt:");
    display->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    if (status.battery.voltage > 4500) {
      display->print("USB Power        ");
    } else {
      display->print(status.battery.voltage);
      display->print("mV (");
      display->print(status.battery.percent);
      display->print("%)     ");
    }
    lastBattVoltage = status.battery.voltage;
    lastBattPercent = status.battery.percent;
  }
  
  // Temperature (always show)
  if (abs(lastTemp - status.vario.temp) > 0.1 || isnan(lastTemp) != isnan(status.vario.temp) || pageJustChanged) {
    display->setCursor(2, 38);
    display->setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    display->print("Temp:");
    display->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    if (!isnan(status.vario.temp)) {
      display->print(status.vario.temp, 1);
      display->print("C     ");
    } else {
      display->print("N/A     ");
    }
    lastTemp = status.vario.temp;
  }
  
  // Pressure (always show)
  if (abs(lastPressure - status.vario.pressure) > 0.1 || pageJustChanged) {
    display->setCursor(2, 50);
    display->setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    display->print("Press:");
    display->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    if (status.vario.pressure > 0) {
      display->print(status.vario.pressure, 1);
      display->print("hPa     ");
    } else {
      display->print("N/A        ");
    }
    lastPressure = status.vario.pressure;
  }
  
  // Heap memory (always show)
  uint32_t currentHeap = ESP.getFreeHeap() / 1024;
  if (abs((int32_t)(lastHeap - currentHeap)) > 1 || pageJustChanged) {
    display->setCursor(2, 62);
    display->setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    display->print("Heap:");
    display->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    display->print(currentHeap);
    display->print("KB     ");
    lastHeap = currentHeap;
  }
}

// PAGE 2: Detailed GPS Info
void Tft::printDetailedGPS(uint32_t tAct, bool forceRedraw){
  if (!bDisplayOn) return;
  
  static uint8_t lastFix = 255;
  static uint8_t lastNumSat = 255;
  static double lastLat = -999.0;
  static double lastLon = -999.0;
  static float lastAlt = -9999.0;
  static float lastSpeed = -9999.0;
  static float lastCourse = -9999.0;
  static eConnectionState lastWifiState = eConnectionState::IDLE;
  
  bool pageJustChanged = forceRedraw || (lastFix == 255 && lastNumSat == 255);
  
  // Draw static elements
  if (pageJustChanged || oldScreenNumber == 3) {
    display->setTextSize(1);
    display->setCursor(2, 2);
    display->setTextColor(ST77XX_CYAN, ST77XX_BLACK);
    display->print("P3:GPS DETAIL   ");
  }
  
  // Always update WiFi and satellite count
  if (lastWifiState != status.wifiSTA.state || pageJustChanged) {
    drawWifiStat(status.wifiSTA.state);
    lastWifiState = status.wifiSTA.state;
  }
  
  if (lastNumSat != status.gps.NumSat || pageJustChanged) {
    drawSatCount(100, 2, status.gps.NumSat);
    lastNumSat = status.gps.NumSat;
  }
  
  // GPS Fix status (always show)
  if (lastFix != status.gps.Fix || pageJustChanged) {
    display->setCursor(2, 14);
    display->setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    display->print("Fix:");
    if (status.gps.Fix >= 2) {
      display->setTextColor(ST77XX_GREEN, ST77XX_BLACK);
      display->print("YES (");
      display->print(status.gps.Fix);
      display->print(")  ");
    } else {
      display->setTextColor(ST77XX_RED, ST77XX_BLACK);
      display->print("NO (");
      display->print(status.gps.Fix);
      display->print(")  ");
    }
    lastFix = status.gps.Fix;
  }
  
  // Latitude (always show)
  if (abs(lastLat - status.gps.Lat) > 0.000001 || pageJustChanged) {
    display->setCursor(2, 26);
    display->setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    display->print("Lat:");
    display->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    if (status.gps.Fix >= 2) {
      display->print(status.gps.Lat, 6);
      display->print("     ");
    } else {
      display->print("--          ");
    }
    lastLat = status.gps.Lat;
  }
  
  // Longitude (always show)
  if (abs(lastLon - status.gps.Lon) > 0.000001 || pageJustChanged) {
    display->setCursor(2, 38);
    display->setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    display->print("Lon:");
    display->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    if (status.gps.Fix >= 2) {
      display->print(status.gps.Lon, 6);
      display->print("     ");
    } else {
      display->print("--          ");
    }
    lastLon = status.gps.Lon;
  }
  
  // Altitude (always show)
  if (abs(lastAlt - status.gps.alt) > 0.5 || pageJustChanged) {
    display->setCursor(2, 50);
    display->setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    display->print("Alt:");
    display->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    if (status.gps.Fix >= 2) {
      display->print((int)status.gps.alt);
      display->print("m      ");
    } else {
      display->print("--       ");
    }
    lastAlt = status.gps.alt;
  }
  
  // Speed & Course (always show)
  if (abs(lastSpeed - status.gps.speed) > 0.5 || abs(lastCourse - status.gps.course) > 0.5 || pageJustChanged) {
    display->setCursor(2, 62);
    display->setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    display->print("Spd:");
    display->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    if (status.gps.Fix >= 2) {
      display->print((int)status.gps.speed);
      display->print("km/h ");
      display->setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
      display->print("Crs:");
      display->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
      display->print((int)status.gps.course);
      display->print("deg   ");
    } else {
      display->print("--              ");
    }
    lastSpeed = status.gps.speed;
    lastCourse = status.gps.course;
  }
}

void Tft::run(void){
  if (!bDisplayOn) return;
  
  static uint32_t tDisplay = millis();
  uint32_t tAct = millis();
  
  // Auto-advance from splash screen (page 0) to GPS page (page 1) ONLY ONCE at boot when:
  // 1. GPS is fixed AND
  // 2. At least 3 seconds have passed since splash was first shown AND
  // 3. Auto-advance hasn't already happened (so user can navigate back to page 0)
  if (setting.screenNumber == 0 && status.gps.Fix >= 2 && splashShown && !autoAdvanceDone) {
    uint32_t splashDuration = tAct - splashStartTime;
    if (splashDuration >= 3000) {  // 3 seconds minimum
      setting.screenNumber = 1;  // Move to GPS data page
      autoAdvanceDone = true;  // Mark that we've done the auto-advance
      log_i("GPS fixed and 3s elapsed - auto-switching from splash to GPS page");
    }
  }
  
  // Check if screen number changed
  bool screenChanged = (oldScreenNumber != setting.screenNumber);
  
  // When page changes, force full redraw
  if (screenChanged) {
    display->fillScreen(ST77XX_BLACK);
    oldScreenNumber = setting.screenNumber;
    // Force immediate update on page change
    tDisplay = 0;
  }
  
  // Update display every 1000ms or immediately on screen change
  if (screenChanged || ((tAct - tDisplay) >= 1000)) {
    tDisplay = tAct;
    
    // Switch between pages based on screenNumber
    // Pass screenChanged flag to force redraw of all fields
    switch (setting.screenNumber) {
      case 0:
        printSplashScreen(tAct, screenChanged);
        break;
      case 1:
        printGPSData(tAct, screenChanged);
        break;
      case 2:
        printSystemInfo(tAct, screenChanged);
        break;
      case 3:
        printDetailedGPS(tAct, screenChanged);
        break;
      default:
        // Invalid page, reset to splash
        printSplashScreen(tAct, true);
        setting.screenNumber = 0;
        oldScreenNumber = 0;
        break;
    }
  }
}

void Tft::webUpdate(void){
  // Trigger immediate update
  if (bDisplayOn) {
    run();
  }
}
