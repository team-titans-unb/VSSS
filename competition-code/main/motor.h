/**********************************************************************************************/
/*                                                                                            */
/*                                                                                            */
/*        motor.h                                      Author  : Luiz Felipe                  */
/*                                                     Email   :                              */
/*                                                     address : DF, BRAZIL                   */
/*        Created: 2023/02/26          by Luiz F.                                             */
/*        Updated: 2023/03/31          by Luiz F.                                             */
/*                                                                       All rights reserved  */
/**********************************************************************************************/
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "config.h"

class Motor {
  /*
    This class was been create to makes Motors objects and control them
  */
private:
    int pin_R_;                
    int pin_L_;                       
    bool stateMotor_;
    int channelR_;
    int channelL_;
public:
    Motor(uint8_t pin_R, uint8_t pin_L, uint8_t channelR, uint8_t channelL);
    void moveForward(int speed, int direction);
    void stop();
};

#endif