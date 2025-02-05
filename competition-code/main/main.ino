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
        if (receivedValue != 0xFFFFFFFF) { // Se houver novos dados válidos
            // Extrai velocidade e direção de cada motor
            int speedRight = (receivedValue & 0x00FF0000) >> 16;  // Bits 16-23 -> Velocidade motor direito
            int directionRight = (receivedValue & 0x000000FF);    // Bits 0-7  -> Direção motor direito
            
            int speedLeft = (receivedValue & 0xFF000000) >> 24;   // Bits 24-31 -> Velocidade motor esquerdo
            int directionLeft = (receivedValue & 0x0000FF00) >> 8;// Bits 8-15  -> Direção motor esquerdo
            
            // Ajusta o sinal conforme a direção (exemplo: 1 = frente, 0 = ré)
            setPointRight = (directionRight == 1) ? speedRight : -speedRight;
            setPointLeft = (directionLeft == 1) ? speedLeft : -speedLeft;

            Serial.printf("Setpoints -> Direita: %d (Dir: %d), Esquerda: %d (Dir: %d)\n", 
                          setPointRight, directionRight, setPointLeft, directionLeft);
        }
        // receivedValue = 0xFFFFFFFF;
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
        // Atualizar velocidades dos encoders
        corobeu.encoderLeft.updateSpeed();
        corobeu.encoderRight.updateSpeed();
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

            // Rampa de aceleração
            if (currentPWMRight < RAMP_PWM_LIMIT) {
                currentPWMRight += RAMP_STEP;
                currentPWMRight = min(currentPWMRight, RAMP_PWM_LIMIT);
                controlRight = currentPWMRight;
            } else {
                controlRight = pidOutputRight;
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
    xTaskCreate(communicationTask, "Communication Task", 4096, NULL, 2, &communicationTaskHandle);
    xTaskCreate(motorControlTask, "Motor Control Task", 2048, NULL, 1, &motorControlTaskHandle);
}

/**
 * @brief Loop principal (vazio, pois usamos FreeRTOS)
 */
void loop() {
}
