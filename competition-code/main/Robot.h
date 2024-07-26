#ifndef ROBO_H
#define ROBO_H

#include <Arduino.h>

class Robot{
public:
    Robot(const String& ip, uint16_t port, uint8_t num);
    String getIp() const;
    uint16_t getPort() const;
    uint8_t getNum() const;

private:
    String ip_;
    uint16_t port_;
    uint8_t num_;
    void initializeRobot();
};


#endif // ROBO_H
