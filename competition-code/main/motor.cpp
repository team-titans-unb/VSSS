/**********************************************************************************************/
/*                                                                                            */
/*                                                                                            */
/*        motor.cpp                                    Author  : Luiz Felipe                  */
/*                                                     Email   :                              */
/*                                                     address : DF, BRAZIL                   */
/*        Created: 2023/02/26          by Luiz F.                                             */
/*        Updated: 2024/09/13          by Luiz F.                                             */
/*                                                                       All rights reserved  */
/**********************************************************************************************/
#include "motor.h"

Motor::Motor(uint8_t pin_R, uint8_t pin_L, uint8_t channelR, uint8_t channelL) {
    this->pin_R_ = pin_R;
    this->pin_L_ = pin_L;
    this->channelR_ = channelR;
    this->channelL_ = channelL;

    ledcAttachChannel(pin_R_, PWM_MOTOR_FREQUENCY, PWM_MOTOR_RESOLUTION, channelR_);
    ledcAttachChannel(pin_L_, PWM_MOTOR_FREQUENCY, PWM_MOTOR_RESOLUTION, channelL_);

    ledcAttach(pin_R_, PWM_MOTOR_FREQUENCY, PWM_MOTOR_RESOLUTION);
    ledcAttach(pin_L_, PWM_MOTOR_FREQUENCY, PWM_MOTOR_RESOLUTION);
}

void Motor::moveForward(int speed, int direction) {
    if (direction == 1){
        ledcWrite(pin_R_, LOW);
        ledcWrite(pin_L_, speed);        
    }
    else if (direction == -1) {
        ledcWrite(pin_R_, speed);
        ledcWrite(pin_L_, LOW);              
    }
}

void Motor::stop() {
    ledcWrite(pin_R_, 0x00);
    ledcWrite(pin_L_, 0x00);
}
