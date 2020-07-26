#include <tools.h>


bool timeOver(uint32_t tAct,uint32_t timestamp,uint32_t tTime){
    if ((tAct - timestamp) >= tTime){
        return true;
    }else{
        return false;
    }
}

uint32_t gettimeElapsed(uint32_t tAct,uint32_t timestamp){
    return (tAct - timestamp);
}

int32_t scale(int32_t value,int32_t inLow, int32_t inHigh, int32_t outLow, int32_t outHigh){
    if (value <= inLow) return outLow;
    if (value >= inHigh) return outHigh;
    return map(value,inLow,inHigh,outLow,outHigh);
}