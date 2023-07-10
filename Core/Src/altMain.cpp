#include "altMain.h"
#include "slcan.hpp"

SLCAN::CANIface slCan;

int altMain(){





    while(1){

    }
}

int8_t usbReceive(uint8_t* Buf, uint32_t *Len){
    slCan.receiveSerial(Buf,Len);
    return 0;
}

int8_t canReceive(){
    slCan.receiveCan();
    return 0;
}
