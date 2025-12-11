/*
 * MAVLink Protocol Handler
 * Lightweight implementation for Teensy 4.1
 */

#include "mavlink.h"
#include <string.h>

static const uint8_t crc_extra[256] = {
    50, 124, 137, 0, 237, 217, 104, 119, 0, 0, 0, 89, 0, 0, 0, 0,
    0, 0, 0, 0, 214, 69, 101, 0, 24, 23, 170, 144, 67, 115, 39, 246,
    185, 104, 237, 244, 222, 212, 9, 254, 230, 28, 28, 132, 255, 159, 14, 106
};

static uint16_t crc16_accumulate(uint8_t b, uint16_t crc) {
    uint8_t tmp = b ^ (crc & 0xFF);
    tmp ^= (tmp << 4);
    return (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

static uint16_t crc16_calculate(const uint8_t* buf, uint16_t len, uint8_t extra) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc = crc16_accumulate(buf[i], crc);
    }
    crc = crc16_accumulate(extra, crc);
    return crc;
}

void MAVLinkHandler::begin(Stream* port) {
    serial = port;
    sequence = 0;
    rxIndex = 0;
    cmdPending = false;
}

void MAVLinkHandler::update() {
    if (!serial) return;
    
    while (serial->available()) {
        uint8_t c = serial->read();
        
        if (rxIndex == 0) {
            if (c == MAVLink::HEADER_V2) {
                rxBuffer[rxIndex++] = c;
            }
        } else {
            rxBuffer[rxIndex++] = c;
            
            if (rxIndex >= 10) {
                uint8_t len = rxBuffer[1];
                uint16_t total_len = 12 + len;
                
                if (rxIndex >= total_len) {
                    parseMessage();
                    rxIndex = 0;
                }
            }
            
            if (rxIndex >= sizeof(rxBuffer)) {
                rxIndex = 0;
            }
        }
    }
}

void MAVLinkHandler::parseMessage() {
    uint8_t len = rxBuffer[1];
    uint32_t msgId = rxBuffer[7] | (rxBuffer[8] << 8) | (rxBuffer[9] << 16);
    
    if (msgId == MAVLink::COMMAND_LONG) {
        uint16_t cmd = rxBuffer[10] | (rxBuffer[11] << 8);
        lastCommand = cmd;
        cmdPending = true;
        
        uint8_t ack[12];
        ack[0] = cmd & 0xFF;
        ack[1] = (cmd >> 8) & 0xFF;
        ack[2] = 0;
        ack[3] = 0;
        ack[4] = MAVLink::SYS_ID;
        ack[5] = MAVLink::COMP_ID;
        sendMessage(MAVLink::COMMAND_ACK, ack, 6);
    }
}

void MAVLinkHandler::sendMessage(uint16_t msgId, const uint8_t* payload, uint8_t len) {
    if (!serial) return;
    
    txBuffer[0] = MAVLink::HEADER_V2;
    txBuffer[1] = len;
    txBuffer[2] = 0;
    txBuffer[3] = 0;
    txBuffer[4] = sequence++;
    txBuffer[5] = MAVLink::SYS_ID;
    txBuffer[6] = MAVLink::COMP_ID;
    txBuffer[7] = msgId & 0xFF;
    txBuffer[8] = (msgId >> 8) & 0xFF;
    txBuffer[9] = (msgId >> 16) & 0xFF;
    
    memcpy(&txBuffer[10], payload, len);
    
    uint8_t crc_extra_val = (msgId < 48) ? crc_extra[msgId] : 0;
    uint16_t crc = crc16_calculate(&txBuffer[1], 9 + len, crc_extra_val);
    
    txBuffer[10 + len] = crc & 0xFF;
    txBuffer[11 + len] = (crc >> 8) & 0xFF;
    
    serial->write(txBuffer, 12 + len);
}

void MAVLinkHandler::sendHeartbeat(uint8_t mode, uint8_t state) {
    uint8_t payload[9];
    payload[0] = 0;
    payload[1] = 0;
    payload[2] = 0;
    payload[3] = 0;
    payload[4] = MAVLink::QUADROTOR;
    payload[5] = MAVLink::GENERIC;
    payload[6] = mode;
    payload[7] = 0;
    payload[8] = state;
    sendMessage(MAVLink::HEARTBEAT, payload, 9);
}

void MAVLinkHandler::sendAttitude(float roll, float pitch, float yaw,
                                   float rollRate, float pitchRate, float yawRate) {
    uint8_t payload[28];
    uint32_t time_ms = millis();
    
    memcpy(&payload[0], &time_ms, 4);
    
    float roll_rad = roll * 0.0174533f;
    float pitch_rad = pitch * 0.0174533f;
    float yaw_rad = yaw * 0.0174533f;
    float p_rad = rollRate * 0.0174533f;
    float q_rad = pitchRate * 0.0174533f;
    float r_rad = yawRate * 0.0174533f;
    
    memcpy(&payload[4], &roll_rad, 4);
    memcpy(&payload[8], &pitch_rad, 4);
    memcpy(&payload[12], &yaw_rad, 4);
    memcpy(&payload[16], &p_rad, 4);
    memcpy(&payload[20], &q_rad, 4);
    memcpy(&payload[24], &r_rad, 4);
    
    sendMessage(MAVLink::ATTITUDE, payload, 28);
}

void MAVLinkHandler::sendGlobalPosition(double lat, double lon, float alt,
                                         float vx, float vy, float vz) {
    uint8_t payload[28];
    uint32_t time_ms = millis();
    
    memcpy(&payload[0], &time_ms, 4);
    
    int32_t lat_e7 = (int32_t)(lat * 1e7);
    int32_t lon_e7 = (int32_t)(lon * 1e7);
    int32_t alt_mm = (int32_t)(alt * 1000);
    int32_t rel_alt = alt_mm;
    int16_t vx_cm = (int16_t)(vx * 100);
    int16_t vy_cm = (int16_t)(vy * 100);
    int16_t vz_cm = (int16_t)(vz * 100);
    uint16_t hdg = 0xFFFF;
    
    memcpy(&payload[4], &lat_e7, 4);
    memcpy(&payload[8], &lon_e7, 4);
    memcpy(&payload[12], &alt_mm, 4);
    memcpy(&payload[16], &rel_alt, 4);
    memcpy(&payload[20], &vx_cm, 2);
    memcpy(&payload[22], &vy_cm, 2);
    memcpy(&payload[24], &vz_cm, 2);
    memcpy(&payload[26], &hdg, 2);
    
    sendMessage(MAVLink::GLOBAL_POSITION, payload, 28);
}
