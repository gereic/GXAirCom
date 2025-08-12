#include <ws85.h>
#include <Arduino.h>
#include <string.h>

HardwareSerial WS85Serial(2);

static char serialBytes[512]; // Buffer for incoming serial data
static float lastDir = 0;
static float lastSpeed = 0;
static float lastGust = 0;
static float lastRain = 0;
static float lastBatVolt = 0;
static float lastCapVolt = 0;
static float lastTemp = -100;
static bool hasNewData = false;

struct ParsedLine {
    String name;
    String value;
};

ParsedLine parseLine(const char *line) {
    ParsedLine result;
    const char *eq = strchr(line, '=');
    if (eq && eq != line) {
        //result.name = String(line, eq - line);
        result.name = String(line).substring(0,eq - line);
        result.name.trim();
        result.value = String(eq + 1);
        result.value.trim();
    }
    return result;
}

void ws85_init(int rxPin) {
    //WS85Serial.begin(115200, SERIAL_8N1, rxPin, -1, 512, true); // RX only, 512 byte buffer, enable RX interrupt
    WS85Serial.begin(115200, SERIAL_8N1, rxPin, -1); // RX only
}

void ws85_run() {
    while (WS85Serial.available()) {
        // Clear buffer before reading
        memset(serialBytes, '\0', sizeof(serialBytes));
        size_t serialPayloadSize = WS85Serial.readBytes(serialBytes, sizeof(serialBytes) - 1);
        // Make sure it is null-terminated
        serialBytes[serialPayloadSize] = '\0';

        if (serialPayloadSize > 0) {
            // Walk through buffer splitting lines
            size_t lineStart = 0;
            for (size_t i = 0; i <= serialPayloadSize; ++i) {
                // Check newline or end of buffer
                if (serialBytes[i] == '\n' || serialBytes[i] == '\0') {
                    size_t lineLength = i - lineStart;

                    if (lineLength > 0 && lineLength < 128) {
                        char line[128];
                        memset(line, '\0', sizeof(line));
                        memcpy(line, &serialBytes[lineStart], lineLength);

                        // Log each line clearly
                        //log_i("WS85 line: %s", line);

                        // Parse the line
                        ParsedLine parsed = parseLine(line);
                        if (parsed.name.length() > 0) {
                            if (parsed.name == "WindDir") {
                                lastDir = parsed.value.toFloat();
                                hasNewData = true;
                            } else if (parsed.name == "WindSpeed") {
                                lastSpeed = parsed.value.toFloat();
                                hasNewData = true;
                            } else if (parsed.name == "WindGust") {
                                lastGust = parsed.value.toFloat();
                                hasNewData = true;
                            } else if (parsed.name == "GXTS04Temp" || parsed.name == "Temperature") {
                                lastTemp = parsed.value.toFloat();
                                hasNewData = true;
                            } else if (parsed.name == "Rain" && parsed.name != "RainIntSum") {
                                lastRain = parsed.value.toFloat();
                                hasNewData = true;
                            } else if (parsed.name == "BatVoltage") {
                                lastBatVolt = parsed.value.toFloat();
                                hasNewData = true;
                            } else if (parsed.name == "CapVoltage") {
                                lastCapVolt = parsed.value.toFloat();
                                hasNewData = true;
                            }
                        }
                    }
                    // Move start to after the newline
                    lineStart = i + 1;
                }
            }
            break; // Done with this batch
        }
    }
}

bool ws85_getData(float *dir, float *speed, float *gust, float *temp, float *rain1h, float *batVolt, float *capVolt) {
    if (!hasNewData) return false;
    *dir = lastDir;
    *speed = lastSpeed;
    *gust = lastGust;
    *temp = lastTemp;
    *rain1h = lastRain;
    *batVolt = lastBatVolt;
    *capVolt = lastCapVolt;
    hasNewData = false;
    return true;
}