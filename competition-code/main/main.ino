#include "Robot.h"
#include "communication.h"

// Configurações do controle PID
const float kpr = 0.4145; // Ganho proporcional
const float kir = 2.1283; // Ganho integral
const float kdr = 0.0146; // Ganho derivativo
const float alfar = 0.00046;
const float umalfar = 0.99954;


const float kpl = 0.1587; // Ganho proporcional
const float kil = 2.0705; // Ganho integral
const float kdl = 0.0129; // Ganho derivativo
const float alfal = 0.00079;
const float umalfal = 0.99921;

// Limites para ação integral (anti-windup)
const float INTEGRAL_LIMIT = 250.0;

// Rampa de aceleração
const float RAMP_PWM_LIMIT = 100.0;
const float RAMP_STEP = 10;

// Variáveis de controle
volatile int setPointRight = 0;
volatile int setPointLeft = 0;
volatile float controlRight = 0, controlLeft = 0;
float integralRight = 0.0, integralLeft = 0.0;
float prevErrorRight = 0.0, prevErrorLeft = 0.0;
float fr = 0, fanteriorr = 0, fl = 0, fanteriorl = 0;
float currentPWMRight = 0.0, currentPWMLeft = 0.0;

// Criação do objeto do robô
Robot corobeu(ROBOT_MOTOR_1R, ROBOT_MOTOR_1L, ROBOT_MOTOR_2R, ROBOT_MOTOR_2L,
              LEFT_ENCODER_A, LEFT_ENCODER_B, RIGHT_ENCODER_A, RIGHT_ENCODER_B);

// Comunicação Wi-Fi
Communication messenger(NETWORK, PASSWORD, 80);
TaskHandle_t communicationTaskHandle = NULL;
TaskHandle_t motorControlTaskHandle = NULL;

/**
 * @brief Tarefa de comunicação Wi-Fi
 */
void communicationTask(void* parameter) {
    while (true) {
        uint32_t receivedValue = messenger.receiveData();
        if (receivedValue != 0xFFFFFFFF) {
            setPointRight = (receivedValue & 0xFFFF0000) >> 16;
            setPointLeft = receivedValue & 0x0000FFFF;
            Serial.printf("Setpoints -> Direita: %d, Esquerda: %d\n", setPointRight, setPointLeft);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Tarefa de controle dos motores
 */
void motorControlTask(void* parameter) {
    int auxcontrolRight = 0;
    int auxcontrolLeft = 0;
    //Filtro da parte derivativa
    float fl = 0;
    float fanteriorl = 0;
    float fr = 0;
    float fanteriorr = 0;

    while (true) {
      corobeu.encoderLeft.updateSpeed();
      corobeu.encoderRight.updateSpeed();
      // Serial.print("RPM R: ");
      // Serial.print(corobeu.encoderRight.getRPM());
      // Serial.print(" : ");
      // Serial.print(corobeu.encoderRight.getDirection());
      // Serial.print("  |   RPM L: ");
      // Serial.print(corobeu.encoderLeft.getRPM());
      // Serial.print(" : ");
      // Serial.println(corobeu.encoderLeft.getDirection());
       if(cnt<24){
        auxcontrolRight += 5;
        auxcontrolLeft += 5;
        cnt++;
        float currentSpeedRight = corobeu.encoderRight.getRPM();
        float currentSpeedLeft = corobeu.encoderLeft.getRPM();

        // Controle PID para motor direito
        if (setPointRight == 0 || abs(setPointRight) < 70) {
            controlRight = integralRight = currentPWMRight = 0;
            corobeu.setMotorRight(0, 0);
        } else {
            float errorRight = setPointRight - currentSpeedRight;
            integralRight += 0.1 * errorRight;
            integralRight = constrain(integralRight, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
            fr = umalfar * fanteriorr + alfar * errorRight;
            float derivativeRight = 10 * (fr - fanteriorr);
            fanteriorr = fr;

            float pidOutputRight = (kpr * errorRight) + (kir * integralRight) + (kdr * derivativeRight);
            pidOutputRight = constrain(pidOutputRight, RAMP_PWM_LIMIT, 255);

        // Atualizar integração (acúmulo do erro para ação integral)
        integralRight += 0.5*errorRight;
        integralLeft += 0.5*errorLeft;

        integralRight = constrain(integralRight, -250, 250);
        integralLeft = constrain(integralLeft, -250, 250);
        
        fr = umalfar * fanteriorr + alfar * errorRight;
        float derivativeRight = 2*(fr - fanteriorr);

        fl = umalfal * fanteriorl + alfal * errorLeft;
        float derivativeLeft = 2*(fl - fanteriorl);

        // Derivada do erro (mudança do erro para ação derivativa)
        // float derivativeRight = 2*(errorRight - prevErrorRight);
        // float derivativeLeft = 2*(errorLeft - prevErrorLeft);

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

            corobeu.setMotorRight((int)controlRight, setPointRight >= 0 ? 1 : -1);
        }

        // Controle PID para motor esquerdo
        if (setPointLeft == 0 || abs(setPointLeft) < 70) {
            controlLeft = integralLeft = currentPWMLeft = 0;
            corobeu.setMotorLeft(0, 0);
        } else {
            float errorLeft = setPointLeft - currentSpeedLeft;
            integralLeft += 0.1 * errorLeft;
            integralLeft = constrain(integralLeft, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
            fl = umalfal * fanteriorl + alfal * errorLeft;
            float derivativeLeft = 10 * (fl - fanteriorl);
            fanteriorl = fl;

            float pidOutputLeft = (kpl * errorLeft) + (kil * integralLeft) + (kdl * derivativeLeft);
            pidOutputLeft = constrain(pidOutputLeft, RAMP_PWM_LIMIT, 255);

            // Rampa de aceleração
            if (currentPWMLeft < RAMP_PWM_LIMIT) {
                currentPWMLeft += RAMP_STEP;
                currentPWMLeft = min(currentPWMLeft, RAMP_PWM_LIMIT);
                controlLeft = currentPWMLeft;
            } else {
                controlLeft = pidOutputLeft;
            }

            corobeu.setMotorLeft((int)controlLeft, setPointLeft >= 0 ? 1 : -1);
        }

        // Debug
        Serial.printf("PWM -> Direita: %.2f | Esquerda: %.2f\n", controlRight, controlLeft);
        
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Setup do ESP32
 */
void setup() {
    corobeu.initializeRobot();
    Serial.begin(19200);
    messenger.begin();

    // Criar tarefas do FreeRTOS
    xTaskCreate(communicationTask, "Communication Task", 2048, NULL, 2, &communicationTaskHandle);
    xTaskCreate(motorControlTask, "Motor Control Task", 2048, NULL, 1, &motorControlTaskHandle);
}

/**
 * @brief Loop principal (vazio, pois usamos FreeRTOS)
 */
void loop() {
}
