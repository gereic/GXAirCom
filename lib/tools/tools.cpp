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