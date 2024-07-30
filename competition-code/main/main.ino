#include "Robot.h"
#include "communication.h"

Robot corobeu(ROBOT_MOTOR_1R, ROBOT_MOTOR_1L, ROBOT_MOTOR_2R, ROBOT_MOTOR_2L);
Communication mensseger(NETWORK,PASSWORD, 8080);

void setup() {
    Serial.begin(9600);
    mensseger.begin();
}

void loop() {
    corobeu.setMotorRight(255,1);
    corobeu.setMotorLeft(255,1);
}