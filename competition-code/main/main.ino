#include "Robot.h"
#include "communication.h"

// Configurações do controle PID
float kpr = 35; // Ganho proporcional
float kir = 0.0; // Ganho integral
float kdr = 0.0; // Ganho derivativo

float kpl = 35; // Ganho proporcional
float kil = 0.0; // Ganho integral
float kdl = 0.0; // Ganho derivativo

// Limites para ação integral (anti-windup)
const float INTEGRAL_LIMIT = 250.0;

// Variáveis para controle PID de cada roda
volatile int setPointRight = 0;
volatile int setPointLeft = 0;
volatile float controlRight = 0;
volatile float controlLeft = 0;
float integralRight = 0.0, integralLeft = 0.0;
float prevErrorRight = 0.0, prevErrorLeft = 0.0;

// Rampa de aceleração
const float RAMP_PWM_LIMIT = 100.0; // Limite da rampa (100 PWM)
const float RAMP_STEP = 10;        // Incremento da rampa
float currentPWMRight = 0.0;        // PWM atual do motor direito
float currentPWMLeft = 0.0;         // PWM atual do motor esquerdo

// Criação do objeto para controle do robô
Robot corobeu(ROBOT_MOTOR_1R, ROBOT_MOTOR_1L, ROBOT_MOTOR_2R, ROBOT_MOTOR_2L, LEFT_ENCODER_A, LEFT_ENCODER_B, RIGHT_ENCODER_A, RIGHT_ENCODER_B);

// Inicializar comunicação
Communication messenger(NETWORK, PASSWORD, 80);
TaskHandle_t communicationTaskHandle = NULL;
TaskHandle_t motorControlTaskHandle = NULL;

/**
 * @brief Tarefa para comunicação com o host.
 */
void communicationTask(void* parameter) {
    while (true) {
        uint32_t receivedValue = messenger.receiveData();
        if (receivedValue != 0xFFFFFFFF) {
            // Atualizar setpoints
            setPointRight = ((receivedValue & 0xFFFF0000) >> 16);
            setPointLeft = (receivedValue & 0x0000FFFF);
            Serial.print("Setpoints atualizados: ");
            Serial.print(setPointRight);
            Serial.print(", ");
            Serial.println(setPointLeft);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Tarefa para controle dos motores.
 */
void motorControlTask(void* parameter) {
    while (true) {
        // Atualizar velocidades atuais dos encoders
        corobeu.encoderLeft.updateSpeed();
        corobeu.encoderRight.updateSpeed();
        float currentSpeedRight = corobeu.encoderRight.getRPM();
        float currentSpeedLeft = corobeu.encoderLeft.getRPM();

        // **Controle para o motor direito**
        if (setPointRight == 0 || abs(setPointRight) < 70) {
            // Se o setpoint for zero ou menor que 70, zere o PWM e reinicie o estado
            controlRight = 0;
            integralRight = 0; // Zerar a ação integral
            currentPWMRight = 0; // Resetar rampa de aceleração
            corobeu.setMotorRight(0, 0); // Parar motor direito
        } else {
            // Calcular erro e PID mesmo durante a rampa
            float errorRight = setPointRight - currentSpeedRight;
            float derivativeRight = 10 * (errorRight - prevErrorRight);
            integralRight += 0.1 * errorRight;
            integralRight = constrain(integralRight, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
            prevErrorRight = errorRight;

            float pidOutputRight = (kpr * errorRight) + (kir * integralRight) + (kdr * derivativeRight);
            pidOutputRight = constrain(pidOutputRight, RAMP_PWM_LIMIT, 255);

            // Implementação da rampa de aceleração
            if (currentPWMRight < RAMP_PWM_LIMIT) {
                currentPWMRight += RAMP_STEP; // Incremento gradual
                currentPWMRight = min(currentPWMRight, RAMP_PWM_LIMIT); // Limitar a 100
                controlRight = currentPWMRight; // Durante a rampa, usar PWM direto
            } else {
                controlRight = pidOutputRight; // Após a rampa, usar PID
            }

            // Aplicar PWM ao motor direito
            corobeu.setMotorRight((int)controlRight, setPointRight >= 0 ? 1 : -1);
        }

        // **Controle para o motor esquerdo**
        if (setPointLeft == 0 || abs(setPointLeft) < 70) {
            // Se o setpoint for zero ou menor que 70, zere o PWM e reinicie o estado
            controlLeft = 0;
            integralLeft = 0; // Zerar a ação integral
            currentPWMLeft = 0; // Resetar rampa de aceleração
            corobeu.setMotorLeft(0, 0); // Parar motor esquerdo
        } else {
            // Calcular erro e PID mesmo durante a rampa
            float errorLeft = setPointLeft - currentSpeedLeft;
            float derivativeLeft = 10 * (errorLeft - prevErrorLeft);
            integralLeft += 0.1 * errorLeft;
            integralLeft = constrain(integralLeft, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
            prevErrorLeft = errorLeft;

            float pidOutputLeft = (kpl * errorLeft) + (kil * integralLeft) + (kdl * derivativeLeft);
            pidOutputLeft = constrain(pidOutputLeft, RAMP_PWM_LIMIT, 255);

            // Implementação da rampa de aceleração
            if (currentPWMLeft < RAMP_PWM_LIMIT) {
                currentPWMLeft += RAMP_STEP; // Incremento gradual
                currentPWMLeft = min(currentPWMLeft, RAMP_PWM_LIMIT); // Limitar a 100
                controlLeft = currentPWMLeft; // Durante a rampa, usar PWM direto
            } else {
                controlLeft = pidOutputLeft; // Após a rampa, usar PID
            }

            // Aplicar PWM ao motor esquerdo
            corobeu.setMotorLeft((int)controlLeft, setPointLeft >= 0 ? 1 : -1);
        }

        // Debug
        Serial.print("PWM Right: ");
        Serial.print(controlRight);
        Serial.print(" | PWM Left: ");
        Serial.println(controlLeft);

        vTaskDelay(100 / portTICK_PERIOD_MS); // Tempo de controle
    }
}


/**
 * @brief Função de setup.
 */
void setup() {
    corobeu.initializeRobot();
    Serial.begin(19200);
    messenger.begin();

    // Criar tarefas FreeRTOS
    xTaskCreate(
                communicationTask,
                "Communication Task",
                2048,
                NULL,
                2,
                &communicationTaskHandle
    );

    xTaskCreate(
                motorControlTask,
                "Motor Control Task",
                2048, 
                NULL,
                1,
                &motorControlTaskHandle
    );
}

/**
 * @brief Loop principal (vazio, já que usamos FreeRTOS).
 */
void loop() {
}
