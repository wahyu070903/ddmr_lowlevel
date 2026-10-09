#ifndef CAN_MANAGER_H
#define CAN_MANAGER_H

#include <Arduino.h>

union CanMsg_32t {
    int32_t value;
    uint8_t bytes[4];
};

typedef struct{
    unsigned long id;
    byte len;
    byte data[8];
}CanRcv_t;

void can_init();
void can_send(int, int, byte*);
bool can_receive(CanRcv_t &result);

#endif