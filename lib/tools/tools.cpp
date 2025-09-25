#include <tools.h>


// Helper function: assigns the value if the key exists
template<typename T>
bool assignIfExists(JsonObject& obj, const char* key, T& target) {
    if (obj.containsKey(key)) {
        target = obj[key].as<T>();
        return true;
    }
    return false;
}

bool timeOver(uint32_t tAct,uint32_t timestamp,uint32_t tTime){
    uint32_t tDiff = tAct - timestamp;
    if (tDiff >= tTime){
        return true;
    }else{
        return false;
    }
}

bool checkValueDiff(float f1, float f2, uint8_t decimals){
    int32_t i1 = int32_t(round(f1 * pow(10,decimals)));
    int32_t i2 = int32_t(round(f2 * pow(10,decimals)));
    //log_i("%.02f,%.02f,%d,%d",f1,f2,i1,i2);
    if (i1 != i2) return true;
    return false;
}

uint32_t gettimeElapsed(uint32_t tAct,uint32_t timestamp){
    return (tAct - timestamp);
}

int32_t scale(int32_t value,int32_t inLow, int32_t inHigh, int32_t outLow, int32_t outHigh){
    if (value <= inLow) return outLow;
    if (value >= inHigh) return outHigh;
    return map(value,inLow,inHigh,outLow,outHigh);
}

int32_t getStringValue(String s,String begin,String end,int32_t fromIndex,String *sRet){
    int pos = s.indexOf(begin,fromIndex);
    if (pos < 0) return -1;
    pos += begin.length();
    int pos2 = s.indexOf(end,pos);
    if (pos2 < 0) return -1;
    *sRet = s.substring(pos,pos2);
    return pos2;
}

String getWDir(float dir){
    uint8_t uDir = uint8_t(dir/22.5);
    switch (uDir)
    {
    case 1:
        return "NNE";
        break;
    case 2:
        return "NE";
        break;
    case 3:
        return "ENE";
        break;
    case 4:
        return "E";
        break;
    case 5:
        return "ESE";
        break;
    case 6:
        return "SE";
        break;
    case 7:
        return "SSE";
        break;
    case 8:
        return "S";
        break;
    case 9:
        return "SSW";
        break;
    case 10:
        return "SW";
        break;
    case 11:
        return "WSW";
        break;
    case 12:
        return "W";
        break;
    case 13:
        return "WNW";
        break;
    case 14:
        return "NW";
        break;
    case 15:
        return "NNW";
        break;
    
    default:
        return "N";
        break;
    }
}



String urldecode(String str)
{
    
    String encodedString="";
    char c;
    char code0;
    char code1;
    for (int i =0; i < str.length(); i++){
        c=str.charAt(i);
      if (c == '+'){
        encodedString+=' ';  
      }else if (c == '%') {
        i++;
        code0=str.charAt(i);
        i++;
        code1=str.charAt(i);
        c = (h2int(code0) << 4) | h2int(code1);
        encodedString+=c;
      } else{
        
        encodedString+=c;  
      }
      
      yield();
    }
    
   return encodedString;
}

String urlencode(String str)
{
    String encodedString="";
    char c;
    char code0;
    char code1;
    //char code2;
    for (int i =0; i < str.length(); i++){
      c=str.charAt(i);
      if (c == ' '){
        encodedString+= '+';
      } else if (isalnum(c)){
        encodedString+=c;
      } else{
        code1=(c & 0xf)+'0';
        if ((c & 0xf) >9){
            code1=(c & 0xf) - 10 + 'A';
        }
        c=(c>>4)&0xf;
        code0=c+'0';
        if (c > 9){
            code0=c - 10 + 'A';
        }
        //code2='\0';
        encodedString+='%';
        encodedString+=code0;
        encodedString+=code1;
        //encodedString+=code2;
      }
      yield();
    }
    return encodedString;
    
}

float kmh2mph(float f){
  return f / 1.609;
}

float deg2f(float f){
  return (f * 9/5) + 32;
}

double dewPointFast(double celsius, double humidity)
{
        double a = 17.271;
        double b = 237.7;
        double temp = (a * celsius) / (b + celsius) + log(humidity*0.01);
        double Td = (b * temp) / (a - temp);
        return Td;
}

unsigned char h2int(char c)
{
    if (c >= '0' && c <='9'){
        return((unsigned char)c - '0');
    }
    if (c >= 'a' && c <='f'){
        return((unsigned char)c - 'a' + 10);
    }
    if (c >= 'A' && c <='F'){
        return((unsigned char)c - 'A' + 10);
    }
    return(0);
}