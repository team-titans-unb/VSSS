/**********************************************************************************************/
/*                                                                                            */
/*                                                                                            */
/*        motor.h                                      Author  : Luiz Felipe                  */
/*                                                     Email   :                              */
/*                                                     address : DF, BRAZIL                   */
/*        Created: 2023/02/26          by Luiz F.                                             */
/*        Updated: 2024/09/13          by Luiz F.                                             */
/*                                                                       All rights reserved  */
/**********************************************************************************************/
#include "Robot.h"
#include "communication.h"

// Cria um robô corobeu para controle
Robot corobeu(ROBOT_MOTOR_1R, ROBOT_MOTOR_1L, ROBOT_MOTOR_2R, ROBOT_MOTOR_2L);

// Inicializa a comunicação
Communication messenger(NETWORK, PASSWORD, 80);
uint32_t combinedValue = 0;

TaskHandle_t communicationTaskHandle = NULL;
TaskHandle_t motorControlTaskHandle = NULL;

// Função de tarefa para lidar com a comunicação
void communicationTask(void* parameter) {
    while (true) {
        uint32_t receivedValue = messenger.receiveData();
        if (receivedValue != 0xFFFFFFFF) { // Verifica se o valor é válido
            combinedValue = receivedValue;
            Serial.print("Valor recebido: ");
            Serial.println(combinedValue, HEX);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // Delay para evitar travamentos
    }
}

// Função de tarefa para controle dos motores
void motorControlTask(void* parameter) {
    while (true) {
        // Decodifica o valor combinado em velocidade e direção
        int speed1 = ((combinedValue & 0xFF000000) >> 24); // Extrai os 8 bits mais significativos
        int direction1 = ((combinedValue & 0x0000FF00) >> 16);     // Extrai os 8 bits menos significativos

        int speed2 = ((combinedValue & 0x00FF0000) >> 8); // Extrai os 8 bits mais significativos
        int direction2 = combinedValue & 0x000000FF;     // Extrai os 8 bits menos significativos

        // Atualiza o controle do robô com o valor recebido
        corobeu.setMotorRight(speed1, direction1);
        corobeu.setMotorLeft(speed2, direction2);

        vTaskDelay(10 / portTICK_PERIOD_MS); // Delay para atualização periódica
    }
}

void setup() {
    Serial.begin(9600);
    messenger.begin();

    // Cria a tarefa de comunicação
    xTaskCreate(
        communicationTask,        // Função da tarefa
        "Communication Task",     // Nome da tarefa
        2048,                     // Tamanho da pilha
        NULL,                     // Parâmetro da tarefa
        1,                        // Prioridade da tarefa
        &communicationTaskHandle  // Handle da tarefa
    );

    // Cria a tarefa de controle dos motores
    xTaskCreate(
        motorControlTask,         // Função da tarefa
        "Motor Control Task",     // Nome da tarefa
        2048,                     // Tamanho da pilha
        NULL,                     // Parâmetro da tarefa
        1,                        // Prioridade da tarefa
        &motorControlTaskHandle   // Handle da tarefa
    );
}

void loop() {
    // loop vazio, tarefas são executadas em segundo plano
}
