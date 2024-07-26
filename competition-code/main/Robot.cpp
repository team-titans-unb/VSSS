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
    // Se recebe dados deixa o robo em estado de espera para iniciar, caso n, emite um 
    // alerta

}

void Robot::setSpeed(){
    // Coloca a velocidade nas rodas do robo
}

void Robot::Stop(){
    // Para o robo
}