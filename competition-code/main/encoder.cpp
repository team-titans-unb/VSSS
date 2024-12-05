#include "encoder.h"

Encoder::Encoder(uint8_t pinA, uint8_t pinB)
    : pinA_(pinA), pinB_(pinB), pulses_(0), lastTime_(0), rpm_(0.0), angle_(0.0) {}

void Encoder::begin() {
    pinMode(pinA_, INPUT_PULLUP);
    pinMode(pinB_, INPUT_PULLUP);

    attachInterruptArg(digitalPinToInterrupt(pinA_), handleInterrupt, this, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(pinB_), handleInterrupt, this, CHANGE);
}

void IRAM_ATTR Encoder::handleInterrupt(void* arg) {
    Encoder* self = static_cast<Encoder*>(arg);
    bool aState = digitalRead(self->pinA_);
    bool bState = digitalRead(self->pinB_);

    if (aState != bState) {
        self->pulses_++;
    } else {
        self->pulses_--;
    }
}

void Encoder::calculateSpeed(uint32_t intervalMs) {
    uint32_t now = millis();
    uint32_t elapsedTime = now - lastTime_;
    if (elapsedTime >= intervalMs) {
        float rotations = pulses_ / 12.0; // 12 pulses per rotation
        rpm_ = (rotations / elapsedTime) * 60000.0;
        angle_ += (pulses_ % 12) * (360.0 / 12.0);
        angle_ = fmod(angle_, 360.0);
        pulses_ = 0;
        lastTime_ = now;
    }
}

float Encoder::getRPM() const {
    return rpm_;
}

float Encoder::getAngle() const {
    return angle_;
}
