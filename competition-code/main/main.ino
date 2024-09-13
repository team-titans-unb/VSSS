#include "Robot.h"
#include "communication.h"

// Cria um robô corobeu para controle
Robot corobeu(ROBOT_MOTOR_1R, ROBOT_MOTOR_1L, ROBOT_MOTOR_2R, ROBOT_MOTOR_2L);

// Inicializa a comunicação
Communication messenger(NETWORK, PASSWORD, 80);
uint16_t combinedValue = 0;

TaskHandle_t communicationTaskHandle = NULL;
TaskHandle_t motorControlTaskHandle = NULL;

// Função de tarefa para lidar com a comunicação
void communicationTask(void* parameter) {
    while (true) {
        uint16_t receivedValue = messenger.receiveData();
        if (receivedValue != 0xFFFF) { // Verifica se o valor é válido
            combinedValue = receivedValue;
            Serial.print("Valor recebido: ");
            Serial.println(combinedValue);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // Delay para evitar travamentos
    }
}

// Função de tarefa para controle dos motores
void motorControlTask(void* parameter) {
    while (true) {
        // Decodifica o valor combinado em velocidade e direção
        int speed = (combinedValue >> 8) & 0xFF; // Extrai os 8 bits mais significativos
        int direction = combinedValue & 0xFF;     // Extrai os 8 bits menos significativos

        // Atualiza o controle do robô com o valor recebido
        corobeu.setMotorRight(speed, direction);
        corobeu.setMotorLeft(speed, direction);

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
