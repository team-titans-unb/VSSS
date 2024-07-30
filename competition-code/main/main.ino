#include "Robot.h"
#include "communication.h"

Robot corobeu(ROBOT_MOTOR_1R, ROBOT_MOTOR_1L, ROBOT_MOTOR_2R, ROBOT_MOTOR_2L);
Communication messenger(NETWORK, PASSWORD, 80);
int val = 0;

void setup() {
    Serial.begin(9600);
    messenger.begin();
}

void loop() {
    int receivedValue = messenger.receiveInt();
    if (receivedValue != -1) {
        val = receivedValue;
        corobeu.setMotorRight(val, 1);
        corobeu.setMotorLeft(val, 1);
        Serial.print("Valor recebido: ");
        Serial.println(val);
    }
    delay(100); // Adiciona um pequeno delay para evitar travamentos
}
