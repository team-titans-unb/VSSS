#include "Robot.h"

Robot corobeu(ROBOT_MOTOR_1R, ROBOT_MOTOR_1L, ROBOT_MOTOR_2R, ROBOT_MOTOR_2L);

void setup() {
    Serial.begin(9600);
}

void loop() {
    corobeu.setMotorRight(255,1);
    corobeu.setMotorLeft(255,1);
}