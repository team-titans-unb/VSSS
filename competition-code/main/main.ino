#include "Robot.h"
#include "communication.h"

// Configurações do controle PID
const float kpr = 1.2, kir = 0.813, kdr = 0.01;
const float alfar = 1, umalfar = 0;

const float kpl = 1.4958, kil = 0.816, kdl = 0.0632;
const float alfal = 1, umalfal = 0;

// const float kpl = 0.1587, kil = 2.0705, kdl = 0.0129;
// const float alfal = 0.00079, umalfal = 0.99921;

const float INTEGRAL_LIMIT = 250.0;
const float RAMP_PWM_LIMIT = 100.0;
const float RAMP_STEP = 10;

// Variáveis de controle
volatile int setPointRight = 0, setPointLeft = 0, directionRight = 0, directionLeft = 0;
float controlRight = 0, controlLeft = 0;
float integralRight = 0, integralLeft = 0;
float fr = 0, fanteriorr = 0, fl = 0, fanteriorl = 0;
float currentPWMRight = 0.0, currentPWMLeft = 0.0;

// Criação do objeto do robô
Robot corobeu(ROBOT_MOTOR_1R, ROBOT_MOTOR_1L, ROBOT_MOTOR_2R, ROBOT_MOTOR_2L,
              LEFT_ENCODER_A, LEFT_ENCODER_B, RIGHT_ENCODER_A, RIGHT_ENCODER_B);

// Comunicação Wi-Fi
Communication messenger(NETWORK, PASSWORD, 80);

// Variáveis de controle de tempo
unsigned long lastMotorUpdate = 0;
unsigned long lastCommUpdate = 0;
const unsigned long motorUpdateInterval = 700; // Atualiza os motores a cada 100ms
const unsigned long commUpdateInterval = 10;   // Verifica comunicação a cada 10ms

void setup() {
    corobeu.initializeRobot();
    Serial.begin(19200);
    messenger.begin();
}

void loop() {
    unsigned long currentTime = millis();

    // Comunicação Wi-Fi
    if (currentTime - lastCommUpdate >= commUpdateInterval) {
        lastCommUpdate = currentTime;
        uint32_t receivedValue = messenger.receiveData();
        if (receivedValue != 0xFFFFFFFF) {
            // Extrai velocidade e direção de cada motor
            int speedRight = (receivedValue & 0x00FF0000) >> 16;
            directionRight = (receivedValue & 0x000000FF);
            
            int speedLeft = (receivedValue & 0xFF000000) >> 24;
            directionLeft = (receivedValue & 0x0000FF00) >> 8;

            setPointRight = speedRight;
            setPointLeft = speedLeft;

            Serial.printf("Setpoints -> Direita: %d (Dir: %d), Esquerda: %d (Dir: %d)\n", 
                          setPointRight, directionRight, setPointLeft, directionLeft);
        }
    }

    // Controle dos motores
    if (currentTime - lastMotorUpdate >= motorUpdateInterval) {
        lastMotorUpdate = currentTime;
        
        // Atualizar velocidades dos encoders
        corobeu.encoderLeft.updateSpeed();
        corobeu.encoderRight.updateSpeed();
        float currentSpeedRight = corobeu.encoderRight.getRPM();
        float currentSpeedLeft = corobeu.encoderLeft.getRPM();
        Serial.printf("RPM R: %.2f | RPM L: %.2f\n", currentSpeedRight, currentSpeedLeft);
        // Controle PID para motor direito
        if (setPointRight == 0 || abs(setPointRight) < 70) {
            controlRight = integralRight = currentPWMRight = 0;
            corobeu.setMotorRight(0, 0);
        } else {
            float errorRight = setPointRight - currentSpeedRight;
            integralRight += errorRight * 0.7;
            // Serial.println(integralRight);
            integralRight = constrain(integralRight, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
            fr = umalfar * fanteriorr + alfar * errorRight;
            float derivativeRight = 1.4285714286 * (fr - fanteriorr);
            fanteriorr = fr;

            float pidOutputRight = (kpr * errorRight) + (kir * integralRight) + (kdr * derivativeRight);
            pidOutputRight = constrain(pidOutputRight, RAMP_PWM_LIMIT, 255);

            // Rampa de aceleração
            // if (currentPWMRight < RAMP_PWM_LIMIT) {
            //     currentPWMRight += RAMP_STEP;
            //     currentPWMRight = min(currentPWMRight, RAMP_PWM_LIMIT);
            //     controlRight = currentPWMRight;
            // } else {
                controlRight = pidOutputRight;
            // }

            corobeu.setMotorRight((int)controlRight, (int)directionRight);
        }

        //Controle PID para motor esquerdo
        if (setPointLeft == 0 || abs(setPointLeft) < 70) {
            controlLeft = integralLeft = currentPWMLeft = 0;
            corobeu.setMotorLeft(0, 0);
        } else {
            float errorLeft = setPointLeft - currentSpeedLeft;
            integralLeft += 0.7 * errorLeft;
            integralLeft = constrain(integralLeft, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
            fl = umalfal * fanteriorl + alfal * errorLeft;
            float derivativeLeft = 1.4285714286 * (fl - fanteriorl);
            fanteriorl = fl;

            float pidOutputLeft = (kpl * errorLeft) + (kil * integralLeft) + (kdl * derivativeLeft);
            pidOutputLeft = constrain(pidOutputLeft, RAMP_PWM_LIMIT, 255);

            // Rampa de aceleração
            // if (currentPWMLeft < RAMP_PWM_LIMIT) {
            //     currentPWMLeft += RAMP_STEP;
            //     currentPWMLeft = min(currentPWMLeft, RAMP_PWM_LIMIT);
            //     controlLeft = currentPWMLeft;
            // } else {
                controlLeft = pidOutputLeft;
            // }

            corobeu.setMotorLeft((int)controlLeft, (int)directionLeft);
        }

        // Debug
        // Serial.printf("PWM -> Direita: %.2f | Esquerda: %.2f\n", controlRight, controlLeft);
        
    }
}
