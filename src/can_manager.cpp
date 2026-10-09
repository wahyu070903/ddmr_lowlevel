
#include "can_manager.h"
#include <SPI.h>
#include <mcp_can.h>

#define CAN_CS 53

MCP_CAN CAN(CAN_CS);

void can_init() {
    SPI.begin();
    while (CAN_OK != CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ)) {
        Serial.println("MCP2515 gagal init...");
        delay(1000);
    }

    CAN.setMode(MCP_NORMAL);
    Serial.println("MCP2515 OK!");
}

void can_send(int can_id, int length, byte* data) {
    byte result = CAN.sendMsgBuf(can_id, 0, length, data);

    if (result != CAN_OK) {
        Serial.print("CAN SEND ERROR: ");
        Serial.println(result);
    }
}

bool can_receive(CanRcv_t &result) {
    if (CAN.checkReceive() != CAN_MSGAVAIL) {
        return false;
    }
    Serial.println("heere");
    CAN.readMsgBuf(&result.id, &result.len, result.data);

    Serial.print("ID: 0x");
    Serial.println(result.id, HEX);

    Serial.print("DATA: ");

    for (byte i = 0; i < result.len; i++) {
        if (result.data[i] < 0x10) Serial.print("0");
        Serial.print(result.data[i], HEX);
        Serial.print(" ");
    }

    Serial.println();

    return true;
}