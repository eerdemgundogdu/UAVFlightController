#include "serial_comm.h"
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <cstdio>
#include <sys/ioctl.h>

SerialPort::SerialPort() : fd(-1) {}

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open(const std::string& port, int baudrate) {
    fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) return false;
    
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        ::close(fd);
        fd = -1;
        return false;
    }
    
    speed_t baud;
    switch (baudrate) {
        case 9600: baud = B9600; break;
        case 19200: baud = B19200; break;
        case 38400: baud = B38400; break;
        case 57600: baud = B57600; break;
        case 115200: baud = B115200; break;
        default: baud = B115200;
    }
    
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);
    
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ISIG;
    
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        ::close(fd);
        fd = -1;
        return false;
    }
    
    return true;
}

void SerialPort::close() {
    if (fd >= 0) {
        ::close(fd);
        fd = -1;
    }
}

int SerialPort::write(const uint8_t* data, int len) {
    if (fd < 0) return -1;
    return ::write(fd, data, len);
}

int SerialPort::read(uint8_t* data, int maxLen) {
    if (fd < 0) return -1;
    return ::read(fd, data, maxLen);
}

int SerialPort::available() {
    if (fd < 0) return 0;
    int bytes;
    ioctl(fd, FIONREAD, &bytes);
    return bytes;
}

void SerialPort::sendCommand(float roll, float pitch, float yaw) {
    char buf[64];
    int len = snprintf(buf, sizeof(buf), "T%.2f,%.2f,%.2f\n", roll, pitch, yaw);
    write((uint8_t*)buf, len);
}
