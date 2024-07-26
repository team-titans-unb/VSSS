#include "Robot.h"

Robot::Robot(const String& ip, uint16_t port, uint8_t num){
    this->ip_ = ip;
    this->port_ = port;
    this->num_ = num;
}

String Robot::getIp() const {
    return this->ip_;
}

uint16_t Robot::getPort() const {
    return this->port_;
}

uint8_t Robot::getNum() const {
    return this->num_;
}

void Robot::initializeRobot(){
    // Faz uma rotina de testes para saber se o robo se conectou com o controle
    if(this->num_1 == 1){
        // faz configuração do robo 1
    }
}

void Robot::setSpeed(){
    // Coloca a velocidade nas rodas do robo
    if(state==1){

    }
}

void Robot::Stop(){
    // Para o robo
}