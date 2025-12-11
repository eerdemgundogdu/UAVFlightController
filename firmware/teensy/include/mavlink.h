#pragma once

#include <stdint.h>

namespace MAVLink {
    constexpr uint8_t HEADER_V2 = 0xFD;
    constexpr uint8_t SYS_ID = 1;
    constexpr uint8_t COMP_ID = 1;
    
    enum MsgId : uint16_t {
        HEARTBEAT = 0,
        SYS_STATUS = 1,
        GPS_RAW = 24,
        ATTITUDE = 30,
        GLOBAL_POSITION = 33,
        RC_CHANNELS = 65,
        VFR_HUD = 74,
        COMMAND_LONG = 76,
        COMMAND_ACK = 77,
        ALTITUDE = 141,
        BATTERY_STATUS = 147
    };
    
    enum Type : uint8_t {
        QUADROTOR = 2
    };
    
    enum Autopilot : uint8_t {
        GENERIC = 0
    };
    
    enum Mode : uint8_t {
        STABILIZE = 0,
        ALT_HOLD = 2,
        LOITER = 5,
        RTL = 6,
        AUTO = 3
    };
    
    enum State : uint8_t {
        UNINIT = 0,
        BOOT = 1,
        STANDBY = 3,
        ACTIVE = 4,
        CRITICAL = 5,
        EMERGENCY = 6
    };
    
    struct Header {
        uint8_t magic;
        uint8_t len;
        uint8_t incompatFlags;
        uint8_t compatFlags;
        uint8_t seq;
        uint8_t sysId;
        uint8_t compId;
        uint32_t msgId : 24;
    } __attribute__((packed));
}

class MAVLinkHandler {
public:
    void begin(Stream* port);
    void update();
    void sendHeartbeat(uint8_t mode, uint8_t state);
    void sendAttitude(float roll, float pitch, float yaw, 
                      float rollRate, float pitchRate, float yawRate);
    void sendGlobalPosition(double lat, double lon, float alt, 
                            float vx, float vy, float vz);
    bool hasCommand() const { return cmdPending; }
    uint16_t getCommand() const { return lastCommand; }
    
private:
    Stream* serial = nullptr;
    uint8_t txBuffer[280];
    uint8_t rxBuffer[280];
    uint16_t rxIndex = 0;
    uint8_t sequence = 0;
    bool cmdPending = false;
    uint16_t lastCommand = 0;
    
    void parseMessage();
    void sendMessage(uint16_t msgId, const uint8_t* payload, uint8_t len);
    uint16_t calcCRC(const uint8_t* data, uint8_t len, uint8_t crcExtra);
};
