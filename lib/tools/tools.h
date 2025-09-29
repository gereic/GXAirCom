/*!
 * @file gxtools.h
 *
 *
 */

#ifndef __GXTOOLS_H__
#define __GXTOOLS_H__

#include <inttypes.h>
#include <Arduino.h>
#include <ArduinoJson.h>

bool timeOver(uint32_t tAct,uint32_t timestamp,uint32_t tTime);
uint32_t gettimeElapsed(uint32_t tAct,uint32_t timestamp);
int32_t scale(int32_t value,int32_t inLow, int32_t inHigh, int32_t outLow, int32_t outHigh);
int32_t getStringValue(String s,String begin,String end,int32_t fromIndex,String *sRet);
String urldecode(String str);
String urlencode(String str);
unsigned char h2int(char c);
String getWDir(float dir);
float kmh2mph(float f);
float deg2f(float f);
double dewPointFast(double celsius, double humidity);
bool checkValueDiff(float f1, float f2, uint8_t decimals);
// Helper function: assigns the value if the key exists
template<typename T>
bool assignIfExists(JsonObject& obj, const char* key, T& target) {
    if (obj.containsKey(key)) {
        target = obj[key].as<T>();
        return true;
    }
    return false;
}
// Special version for enums or casts
template<typename T, typename U>
bool assignIfExistsCast(JsonObject& root, const char* key, T& target) {
  if (root.containsKey(key)) {
    target = static_cast<T>(root[key].as<U>());
    return true;
  }
  return false;
}
#endif