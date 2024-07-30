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
    // Coloca a velocidade nas motor direito
    this->motorRight.moveForward(speedR, direction);
}

void Robot::setMotorLeft(int speedL, int direction){
    // Coloca a velocidade no motor esquerdo
    this->motorLeft.moveForward(speedL, direction);
}

void Robot::Stop(){
    // Para o robo
    motorRight.stop();
    motorLeft.stop();
}
