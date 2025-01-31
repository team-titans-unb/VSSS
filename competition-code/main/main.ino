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
float kpr = 0.4145; // Ganho proporcional
float kir = 2.1283; // Ganho integral
float kdr = 0.0146; // Ganho derivativo
float alfar = 0;
float umalfar = 0;
float fr = 0;
float fanteriorr = 0;

float kpl = 0.1587; // Ganho proporcional
float kil = 2.0705; // Ganho integral
float kdl = 0.0129; // Ganho derivativo
float alfal = 0;
float umalfal = 0;
float fl = 0;
float fanteriorl = 0;

// Variáveis para controle PID de cada roda
volatile int cnt = 0;
volatile int pid_anteriorR = 0;
volatile int pid_anteriorL = 0;
volatile int setPointRight = 150;
volatile int setPointLeft = 150;
volatile float controlRight = 0;
volatile float controlLeft = 0;
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
            setPointRight = ((receivedValue & 0xFFFF0000) >> 16);
            setPointLeft = ((receivedValue & 0x0000FFFF));
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
    int auxcontrolRight = 0;
    int auxcontrolLeft = 0;
    while (true) {
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
       if(cnt<24){
        auxcontrolRight += 5;
        auxcontrolLeft += 5;
        cnt++;
        float currentSpeedRight = corobeu.encoderRight.getRPM();
        float currentSpeedLeft = corobeu.encoderLeft.getRPM();

        // Calcular erro
        float errorRight = setPointRight - currentSpeedRight;
        float errorLeft = setPointLeft - currentSpeedLeft;
        Serial.print("Erro vel PID\n\r");
        Serial.println(errorRight);
        Serial.println(errorLeft);
        pid_anteriorR = auxcontrolRight;
        pid_anteriorL = auxcontrolLeft;
       }
       else{
        // Leituras dos encoders (feedback em RPM)
        float currentSpeedRight = corobeu.encoderRight.getRPM();
        float currentSpeedLeft = corobeu.encoderLeft.getRPM();

        // Calcular erro
        float errorRight = setPointRight - currentSpeedRight;
        float errorLeft = setPointLeft - currentSpeedLeft;

        // Atualizar integração (acúmulo do erro para ação integral)
        integralRight += 0.5*errorRight;
        integralLeft += 0.5*errorLeft;

        if(integralRight>250){
            integralRight = 250;
        }
        if(integralLeft>250){
            integralLeft = 250;
        }
        if(integralRight<-250){
            integralRight = -250;
        }
        if(integralLeft<-250){
            integralLeft = -250;
        }

        // Derivada do erro (mudança do erro para ação derivativa)
        float derivativeRight = 2*(errorRight - prevErrorRight);
        float derivativeLeft = 2*(errorLeft - prevErrorLeft);

        // Atualizar valores anteriores
        prevErrorRight = errorRight;
        prevErrorLeft = errorLeft;

        // Calcular o sinal de controle PID
        controlRight = (kpr * errorRight) + (kir * integralRight) + (kdr * derivativeRight);
        controlLeft = (kpl * errorLeft) + (kil * integralLeft) + (kdl * derivativeLeft);
        if(controlRight>50 || controlRight<-50){
            if((controlRight > 1.5*pid_anteriorR)){
              controlRight = 1.5*pid_anteriorR;
            }
            if((controlRight < 0.5*pid_anteriorR)){
              controlRight = 0.5*pid_anteriorR;
            }
            pid_anteriorR = controlRight;
        }
        
        if(controlLeft>50 || controlLeft<-50){
          if((controlLeft > 1.5*pid_anteriorL)){
            controlLeft = 1.5*pid_anteriorL;
          }
          if((controlLeft < 0.5*pid_anteriorL)){
            controlLeft = 0.5*pid_anteriorL;
          }
          pid_anteriorL = controlLeft;
        }
      Serial.print("Erro vel PID\n\r");
      Serial.println(errorRight);
      Serial.println(errorLeft);
      Serial.print("saidas PID\n\r");
      Serial.println(controlRight);
      Serial.println(controlLeft);
      // controlRight = 150;
      // controlLeft = 150;
      auxcontrolRight = constrain(controlRight, 50, 255);
      auxcontrolLeft = constrain(controlLeft, 50, 255);
      }

      // Saturar o sinal de controle dentro dos limites do PWM
      // if(auxcontrolRight<50){
      //   auxcontrolRight = 0;
      // }
      // if(auxcontrolLeft<50){
      //   auxcontrolLeft = 0;
      // }

      // Aplicar o controle nos motores
      Serial.print("PWM R: ");
      Serial.print(auxcontrolRight);
      Serial.print("  |   PWM L: ");
      Serial.println(auxcontrolLeft);
      corobeu.setMotorRight((int)auxcontrolRight, setPointRight >= 0 ? 1 : -1); // Definir direção com base no set-point
      corobeu.setMotorLeft((int)auxcontrolLeft, setPointLeft >= 0 ? 1 : -1);
      // Serial.print("RPM R: ");
      // Serial.print(corobeu.encoderRight.getRPM());
      // Serial.print(" : ");
      // Serial.print(corobeu.encoderRight.getDirection());
      // Serial.print("  |   RPM L: ");
      // Serial.print(corobeu.encoderLeft.getRPM());
      // Serial.print(" : ");
      // Serial.println(corobeu.encoderLeft.getDirection());
    }
  vTaskDelay(100 / portTICK_PERIOD_MS); // Delay para atualização suave
}

// void encoderTask(void* parameter){
//     while (true){
//         corobeu.encoderLeft.updateSpeed();
//         corobeu.encoderRight.updateSpeed();
//         // Serial.print("RPM R: ");
//         // Serial.print(corobeu.encoderRight.getRPM());
//         // Serial.print(" : ");
//         // Serial.print(corobeu.encoderRight.getDirection());
//         // Serial.print("  |   RPM L: ");
//         // Serial.print(corobeu.encoderLeft.getRPM());
//         // Serial.print(" : ");
//         // Serial.println(corobeu.encoderLeft.getDirection());
//         vTaskDelay(pdMS_TO_TICKS(100));
//     }
    
// }

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

    // xTaskCreate(
    //     encoderTask,
    //     "Encoder Reading Task",
    //     2048,
    //     NULL,
    //     1,
    //     &encoderReadingTaskHandle
    // );

  // uint32_t PIDR = messenger.waitData();
  // if (PIDR != 0xFFFFFFFF) { // Check if the value is valid
  //           // Atualizar set-points com os novos dados
  //           kpr = ((PIDR & 0xFF000000) >> 24);
  //           kir = ((PIDR & 0x00FF0000) >> 16);
  //           kdr = ((PIDR & 0x0000FF00) >> 8);
  // }
  // uint32_t PIDL = messenger.waitData();
  // if (PIDL != 0xFFFFFFFF) { // Check if the value is valid
  //           // Atualizar set-points com os novos dados
  //           kpl = ((PIDR & 0xFF000000) >> 24);
  //           kil = ((PIDR & 0x00FF0000) >> 16);
  //           kdl = ((PIDR & 0x0000FF00) >> 8);
  // }
}

/**
 * @brief Main loop function.
 * 
 * This loop remains empty as all operations are handled by FreeRTOS tasks in the background.
 */
void loop() {

}
