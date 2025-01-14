/**********************************************************************************************/
/*                                                                                            */
/*        main.ino                                     Author  : Luiz Felipe                  */
/*                                                     Email   :                              */
/*                                                     address : DF, BRAZIL                   */
/*        Created: 2023/02/26          by Luiz F.                                             */
/*        Updated: 2024/09/13          by Luiz F.                                             */
/*                                                                       All rights reserved  */
/**********************************************************************************************/
#include "Robot.h"
#include "communication.h"

// Configurações do controle PID
float kpr = 1.3; // Ganho proporcional
float kir = 0.000005; // Ganho integral
float kdr = 1.0; // Ganho derivativo
// Acima parece certo
float kpl = 1; // Ganho proporcional
float kil = 0.000005; // Ganho integral
float kdl = 0.0; // Ganho derivativo

// Variáveis para controle PID de cada roda
volatile int setPointRight = 0;
volatile int setPointLeft = 0;
float integralRight = 0.0, integralLeft = 0.0;
float prevErrorRight = 0.0, prevErrorLeft = 0.0;

// Create a robot object named corobeu for motor control
Robot corobeu(ROBOT_MOTOR_1R, ROBOT_MOTOR_1L, ROBOT_MOTOR_2R, ROBOT_MOTOR_2L, LEFT_ENCODER_A, LEFT_ENCODER_B, RIGHT_ENCODER_A, RIGHT_ENCODER_B);

//Initialize communication
Communication messenger(NETWORK, PASSWORD, 80);
uint32_t combinedValue = 0xFFFFFFFF;

TaskHandle_t communicationTaskHandle = NULL;
TaskHandle_t motorControlTaskHandle = NULL;
TaskHandle_t encoderReadingTaskHandle = NULL;
unsigned long lastUpdateTime; // Última atualização de velocidade

/**
 * @brief Task to handle communication with the host.
 * 
 * This task listens for incoming data from the host and stores the received value in a global variable.
 * It runs indefinitely in the background.
 * 
 * @param parameter Task parameter (not used in this case).
 */
void communicationTask(void* parameter) {
    while (true) {
        // Poll for data with a reduced frequency by adding idle time
        uint32_t receivedValue = messenger.receiveData();
        if (receivedValue != 0xFFFFFFFF) { // Check if the value is valid
            // Atualizar set-points com os novos dados
            setPointRight = ((receivedValue & 0xFF000000) >> 24);
            setPointLeft = ((receivedValue & 0x00FF0000) >> 16);
            Serial.print("Received value: ");
            Serial.println(receivedValue, HEX);
            receivedValue = 0xFFFFFFFF;
        }
        // messenger.sendData(corobeu.encoderRight.getRPM());
        // messenger.sendData(corobeu.encoderLeft.getRPM());
        vTaskDelay(10 / portTICK_PERIOD_MS); // Reduced delay to make communication more responsive
    }
}

/**
 * @brief Task to control the motors of the robot.
 * 
 * This task decodes the received combined value into speed and direction, and updates the robot's motor control accordingly.
 * It runs indefinitely in the background.
 * 
 * @param parameter Task parameter (not used in this case).
 */
void motorControlTask(void* parameter) {
    while (true) {
          // Leituras dos encoders (feedback em RPM)
          float currentSpeedRight = corobeu.encoderRight.getRPM();
          float currentSpeedLeft = corobeu.encoderLeft.getRPM();

          // Calcular erro
          float errorRight = setPointRight - currentSpeedRight;
          float errorLeft = setPointLeft - currentSpeedLeft;

          // Atualizar integração (acúmulo do erro para ação integral)
          integralRight += errorRight;
          integralLeft += errorLeft;

          // Derivada do erro (mudança do erro para ação derivativa)
          float derivativeRight = errorRight - prevErrorRight;
          float derivativeLeft = errorLeft - prevErrorLeft;

          // Atualizar valores anteriores
          prevErrorRight = errorRight;
          prevErrorLeft = errorLeft;

          // Calcular o sinal de controle PID
          float controlRight = (kpr * errorRight) + (kir * integralRight) + (kdr * derivativeRight);
          float controlLeft = (kpl * errorLeft) + (kil * integralLeft) + (kdl * derivativeLeft);

          // Saturar o sinal de controle dentro dos limites do PWM
          controlRight = constrain(controlRight, 0, 255);
          controlLeft = constrain(controlLeft, 0, 255);

          // Aplicar o controle nos motores
          // Serial.print("PWM R: ");
          // Serial.print(controlRight);
          // Serial.print("  |   PWM L: ");
          // Serial.println(controlLeft);
          corobeu.setMotorRight((int)controlRight, setPointRight >= 0 ? 1 : -1); // Definir direção com base no set-point
          corobeu.setMotorLeft((int)controlLeft, setPointLeft >= 0 ? 1 : -1);
          // Serial.print("RPM R: ");
          // Serial.print(corobeu.encoderRight.getRPM());
          // Serial.print(" : ");
          // Serial.print(corobeu.encoderRight.getDirection());
          // Serial.print("  |   RPM L: ");
          // Serial.print(corobeu.encoderLeft.getRPM());
          // Serial.print(" : ");
          // Serial.println(corobeu.encoderLeft.getDirection());
        }
        vTaskDelay(500 / portTICK_PERIOD_MS); // Delay para atualização suave
}

void encoderTask(void* parameter){
    while (true){
        corobeu.encoderLeft.updateSpeed();
        corobeu.encoderRight.updateSpeed();
        Serial.print("RPM R: ");
        Serial.print(corobeu.encoderRight.getRPM());
        Serial.print(" : ");
        Serial.print(corobeu.encoderRight.getDirection());
        Serial.print("  |   RPM L: ");
        Serial.print(corobeu.encoderLeft.getRPM());
        Serial.print(" : ");
        Serial.println(corobeu.encoderLeft.getDirection());
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
}

/**
 * @brief Setup function to initialize communication and create tasks.
 * 
 * This function is called once when the ESP32 starts. It initializes serial communication, 
 * starts WiFi communication, and creates two FreeRTOS tasks: one for communication and one for motor control.
 */
void setup() {
    corobeu.initializeRobot();
    Serial.begin(19200);  // Initialize serial communication
    messenger.begin();   // Start WiFi communication

    // Create the communication task
    xTaskCreate(
        communicationTask,        // Task function
        "Communication Task",     // Task name
        2048,                     // Stack size
        NULL,                     // Task parameter
        2,                        // Higher priority for communication
        &communicationTaskHandle  // Task handle
    );

    // Create the motor control task
    xTaskCreate(
        motorControlTask,         // Task function
        "Motor Control Task",     // Task name
        2048,                     // Stack size
        NULL,                     // Task parameter
        1,                        // Lower priority for motor control
        &motorControlTaskHandle   // Task handle
    );

    xTaskCreate(
        encoderTask,
        "Encoder Reading Task",
        2048,
        NULL,
        1,
        &encoderReadingTaskHandle
    );
}

/**
 * @brief Main loop function.
 * 
 * This loop remains empty as all operations are handled by FreeRTOS tasks in the background.
 */
void loop() {

}
