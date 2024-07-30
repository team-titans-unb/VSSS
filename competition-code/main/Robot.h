#ifndef ROBO_H
#define ROBO_H

#include <Arduino.h>
#include "motor.h"

class Robot{
public:
    Robot(uint8_t pin_R1, uint8_t pin_L1,  uint8_t pin_R2, uint8_t pin_L2);
    void initializeRobot();
    void setMotorRight(int speedL, int direction);
    void setMotorLeft(int speedL, int direction);
    void Stop();

    Motor motorRight;
    Motor motorLeft;
private:
    uint8_t pin_R1_;
    uint8_t pin_L1_; 
    uint8_t channelR1_;
    uint8_t channelL1_;
    uint8_t pin_R2_;
    uint8_t pin_L2_;
    uint8_t channelR2_;
    uint8_t channelL2_;
    uint8_t state_;
    
};


#endif // ROBO_H
