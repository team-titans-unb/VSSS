#include "motor.h"
#include <Arduino.h>

Motor::Motor(uint8_t pin_R, uint8_t pin_L, uint8_t channelR, uint8_t channelL) {
    this->pin_R_ = pin_R;
    this->pin_L_ = pin_L;
    this->channelR_ = channelR;
    this->channelL_ = channelL;

    ledcAttachChannel(pin_R_, PWM_MOTOR_FREQUENCY, PWM_MOTOR_RESOLUTION, channelR_);
    ledcAttachChannel(pin_L_, PWM_MOTOR_FREQUENCY, PWM_MOTOR_RESOLUTION, channelL_);

    // Associar os pinos aos canais PWM
    ledcAttach(pin_R_, PWM_MOTOR_FREQUENCY, PWM_MOTOR_RESOLUTION);
    ledcAttach(pin_L_, PWM_MOTOR_FREQUENCY, PWM_MOTOR_RESOLUTION);
}

void Motor::moveForward(int speed, int direction) {
    /*
        Este método inicia o motor e define a velocidade e direção.
        - A velocidade é dada em um valor hexadecimal de 8 bits para modificar o duty cycle do motor.
        - A direção 'forward' é dada por um número, 1 para direção R e -1 para direção L.
    */
    if (direction == 1){
        //Move o motor para a direção R
        ledcWrite(pin_R_, LOW);
        ledcWrite(pin_L_, speed);        
    }
    else if (direction == -1) {
        // Move o motor para a direção L
        ledcWrite(pin_R_, speed);
        ledcWrite(pin_L_, LOW);              
    }
}

void Motor::stop() {
    // Este método para ambas as direções definindo o duty cycle como zero
    ledcWrite(pin_R_, 0x00);
    ledcWrite(pin_L_, 0x00);
}
