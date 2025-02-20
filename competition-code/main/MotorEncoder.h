#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

#include <Arduino.h>

class MotorEncoder {
private:
    uint8_t encoderPinA;   // Pino A do encoder
    uint8_t encoderPinB;   // Pino B do encoder (opcional)
    volatile int pulseCount; // Contagem de pulsos
    volatile int direction; //Direção de rotação: 1 = horária, -1 = anti-horária
    unsigned long lastUpdateTime; // Última atualização de velocidade
    float rpm; // Rotação por minuto (RPM)
    static constexpr int pulsesPerRevolution = 24; // Pulsos por rotação do encoder
    static void IRAM_ATTR handleInterrupt(void* arg); // Rotina de interrupção estática

public:
    MotorEncoder(uint8_t pinA, uint8_t pinB = 255); // Construtor
    void begin();         // Inicializa o encoder
    void updateSpeed();   // Atualiza a velocidade do motor
    float getRPM();       // Retorna a velocidade atual em RPM
    int getDirection();   // Retorna a direção de rotação
};

#endif // MOTOR_ENCODER_H
