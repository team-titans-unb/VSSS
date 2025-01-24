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

// Create a robot object named corobeu for motor control
Robot corobeu(ROBOT_MOTOR_1R, ROBOT_MOTOR_1L, ROBOT_MOTOR_2R, ROBOT_MOTOR_2L, LEFT_ENCODER_A, LEFT_ENCODER_B, RIGHT_ENCODER_A, RIGHT_ENCODER_B);

//Initialize communication
Communication messenger(NETWORK, PASSWORD, 80);
uint32_t combinedValue = 0xFFFFFFFF;

TaskHandle_t communicationTaskHandle = NULL;
TaskHandle_t motorControlTaskHandle = NULL;
TaskHandle_t encoderReadingTaskHandle = NULL;
unsigned long lastUpdateTime; // Última atualização de velocidade
volatile int setPointRight = 0;
volatile int setPointLeft = 0;

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
      setPointRight = ((receivedValue & 0xFFFF0000) >> 16);
      setPointLeft = ((receivedValue & 0x0000FFFF));
      Serial.print("Received value: ");
      Serial.println(receivedValue, HEX);
      receivedValue = 0xFFFFFFFF;
    }
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
    corobeu.encoderLeft.updateSpeed();
    corobeu.encoderRight.updateSpeed();
    Serial.print("RPM R: ");
    Serial.print(corobeu.encoderRight.getRPM());
    Serial.print("  |   RPM L: ");
    Serial.println(corobeu.encoderLeft.getRPM());

    // Aplicar a velocidade nos motores
    corobeu.setMotorRight((int)setPointRight, setPointRight >= 0 ? 1 : -1); // Definir direção com base no set-point
    corobeu.setMotorLeft((int)setPointLeft, setPointLeft >= 0 ? 1 : -1);
  }
  vTaskDelay(100 / portTICK_PERIOD_MS); // Delay para atualização suave
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
}

/**
 * @brief Main loop function.
 * 
 * This loop remains empty as all operations are handled by FreeRTOS tasks in the background.
 */
void loop() {

}
