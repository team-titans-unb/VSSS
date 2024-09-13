/**********************************************************************************************/
/*                                                                                            */
/*                                                                                            */
/*        Robot.cpp                                    Author  : Luiz Felipe                  */
/*                                                     Email   :                              */
/*                                                     address : DF, BRAZIL                   */
/*        Created: 2023/02/26          by Luiz F.                                             */
/*        Updated: 2024/09/13          by Luiz F.                                             */
/*                                                                       All rights reserved  */
/**********************************************************************************************/
#include "Robot.h"

Robot::Robot(uint8_t pin_R1, uint8_t pin_L1, uint8_t pin_R2, uint8_t pin_L2)
    : motorRight(pin_R1, pin_L1, ROBOT_CHANEL_1R, ROBOT_CHANEL_1L),
      motorLeft(pin_R2, pin_L2, ROBOT_CHANEL_2R, ROBOT_CHANEL_2L) {
    this->pin_R1_ = pin_R1;
    this->pin_L1_ = pin_L1; 
    this->channelR1_ = ROBOT_CHANEL_1R;
    this->channelL1_ = ROBOT_CHANEL_1L;
    this->pin_R2_ = pin_R2;
    this->pin_L2_ = pin_L2;
    this->channelR2_ = ROBOT_CHANEL_2R;
    this->channelL2_ = ROBOT_CHANEL_2L;
    this->state_ = 0;
}

void Robot::initializeRobot(){
    // Faz uma rotina de testes para saber se o robo se conectou com o controle
}

void Robot::setMotorRight(int speedR, int direction){
    // Coloca a velocidade no motor direito
    this->motorRight.moveForward(speedR, direction);
}

void Robot::setMotorLeft(int speedL, int direction){
    // Coloca a velocidade no motor esquerdo
    this->motorLeft.moveForward(speedL, direction);
}

void Robot::Stop(int Motor1, int Motor2){
    int motorState = (Motor1 << 1) | Motor2; 

    switch (motorState) {
        case 0b01:  // 0 e 1: Para apenas o motor esquerdo
            motorLeft.stop();
            break;
        case 0b10:  // 1 e 0: Para apenas o motor direito
            motorRight.stop();
            break;
        case 0b11:  // 1 e 1: Para ambos os motores
            motorRight.stop();
            motorLeft.stop();
            break;
        default:    // Nenhum motor é parado se ambos forem 0
            break;
    }
}

