#include "MotorEncoder.h"

// Construtor
MotorEncoder::MotorEncoder(uint8_t pinA, uint8_t pinB)
    : encoderPinA(pinA), encoderPinB(pinB), pulseCount(0), direction(0), lastUpdateTime(0), rpm(0.0) {}

// Rotina de interrupção estática
void IRAM_ATTR MotorEncoder::handleInterrupt(void* arg) {
    MotorEncoder* instance = static_cast<MotorEncoder*>(arg);
    if(!digitalRead(instance->encoderPinA)) {
      int bState = digitalRead(instance->encoderPinB);
      instance->direction = (bState == HIGH) ? 1 : -1;
      instance->pulseCount++;    
    }
}

// Inicializa o encoder
void MotorEncoder::begin() {
    pinMode(encoderPinA, INPUT_PULLUP);
    pinMode(encoderPinB, INPUT_PULLUP);
    attachInterruptArg(digitalPinToInterrupt(encoderPinA), handleInterrupt, this, FALLING);
    lastUpdateTime = millis();
}

// Atualiza a velocidade do motor
void MotorEncoder::updateSpeed() {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastUpdateTime;

    if (elapsedTime >= 700) { // Atualiza a cada 100 ms
        noInterrupts();
        int count = pulseCount;
        pulseCount = 0;
        interrupts();

        // Calcula a velocidade em RPS
        rpm = (count / (float)pulsesPerRevolution) * (60000.0 / elapsedTime);
        // Serial.println(count);
        if(count == 0){
            direction = 0;
        }

        lastUpdateTime = currentTime;
    }
}

// Retorna a velocidade atual em RPM
float MotorEncoder::getRPM() {
    return rpm;
}

int MotorEncoder::getDirection(){
    return direction;
}
