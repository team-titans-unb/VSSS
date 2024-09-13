#include "Robot.h"
#include "communication.h"

// Cria um robo corobeu para controle
Robot corobeu(ROBOT_MOTOR_1R, ROBOT_MOTOR_1L, ROBOT_MOTOR_2R, ROBOT_MOTOR_2L);


Communication messenger(NETWORK, PASSWORD, 80);
int val = 0;


TaskHandle_t communicationTaskHandle = NULL;
TaskHandle_t motorControlTaskHandle = NULL;

void communicationTask(void* parameter) {
    while (true) {
        int receivedValue = messenger.receiveInt();
        if (receivedValue != -1) {
            val = receivedValue;
            Serial.print("Valor recebido: ");
            Serial.println(val);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // Delay para evitar travamentos
    }
}

void motorControlTask(void* parameter) {
    while (true) {
        // Rotina de controle do robo
        corobeu.setMotorRight(val, 1);
        corobeu.setMotorLeft(val, 1);
        vTaskDelay(10 / portTICK_PERIOD_MS); // Delay para atualização periódica
    }
}

void setup() {
    Serial.begin(9600);
    messenger.begin();

    xTaskCreate(
        communicationTask,        // Função da tarefa
        "Communication Task",     // Nome da tarefa
        2048,                     // Tamanho da pilha
        NULL,                     // Parâmetro da tarefa
        1,                        // Prioridade da tarefa
        &communicationTaskHandle  // Handle da tarefa
    );

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
