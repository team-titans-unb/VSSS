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
int counter = 0;
TaskHandle_t communicationTaskHandle = NULL;
TaskHandle_t motorControlTaskHandle = NULL;

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
            combinedValue = receivedValue;
            counter += 1;
            Serial.print("Received value: ");
            Serial.println(combinedValue, HEX);
            Serial.println(counter);
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
        if (combinedValue != 0xFFFFFFFF) { // Check if there's new data to process
            // Decode the combined value into speed and direction
            int speed1 = ((combinedValue & 0xFF000000) >> 24);   // Extract the 8 most significant bits
            int direction1 = ((combinedValue & 0x0000FF00) >> 8); // Extract the direction for motor 1

            int speed2 = ((combinedValue & 0x00FF0000) >> 16);    // Extract the speed for motor 2
            int direction2 = combinedValue & 0x000000FF;         // Extract the direction for motor 2

            // Update the robot's motor control with the received values
            // Serial.print("Speed1:");
            // Serial.println(speed1);
            // Serial.print("Speed2:");
            // Serial.println(speed2);
            // Serial.print("Direction1:");
            // Serial.println(direction1);
            // Serial.print("Direction2:");
            // Serial.println(direction2);
            corobeu.setMotorRight(speed1, direction1);
            corobeu.setMotorLeft(speed2, direction2);
            combinedValue = 0xFFFFFFFF;
        }

        vTaskDelay(20 / portTICK_PERIOD_MS); // Delay kept for smoother motor updates
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
}

/**
 * @brief Main loop function.
 * 
 * This loop remains empty as all operations are handled by FreeRTOS tasks in the background.
 */
void loop() {

}
