#pragma once

#include <string>
#include <termios.h>

class SerialPort {
public:
    SerialPort();
    ~SerialPort();
    bool open(const std::string& port, int baudrate);
    void close();
    bool isOpen() const { return fd >= 0; }
    int write(const uint8_t* data, int len);
    int read(uint8_t* data, int maxLen);
    int available();
    void sendCommand(float roll, float pitch, float yaw);
    
private:
    int fd = -1;
    struct termios tty;
};
