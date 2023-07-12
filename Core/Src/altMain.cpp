#include "altMain.h"
#include "slcan.hpp"

SLCAN::CANIface slCan;

int altMain(){

    HAL_CAN_RegisterCallback(&hcan, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID,
    canRxInt);
    HAL_CAN_Start(&hcan);
    SLCAN::CANFrame frame;
    SLCAN::CanIOFlags flags;
    uint64_t time;
    while(1){
        slCan.receive(frame,time,flags);
    }
}

int8_t usbReceive(uint8_t* Buf, uint32_t *Len){
    slCan.receiveSerial(Buf,Len);
    return 0;
}

void canRxInt(CAN_HandleTypeDef *_hcan){
    slCan.canRxInt(0, SLCAN::CANIface::native_micros64());
    slCan.canRxInt(1, SLCAN::CANIface::native_micros64());
}
