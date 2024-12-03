#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

/**
 * @brief Class to handle quadrature encoder readings.
 */
class Encoder {
private:
    uint8_t pinA_;       /**< GPIO pin for channel A. */
    uint8_t pinB_;       /**< GPIO pin for channel B. */
    volatile int32_t pulses_; /**< Accumulated pulse count. */
    uint32_t lastTime_;  /**< Last timestamp for speed calculation. */
    float rpm_;          /**< Current speed in rotations per minute (RPM). */
    float angle_;        /**< Current angular position in degrees. */

    /**
     * @brief ISR for the encoder signal.
     */
    static void IRAM_ATTR handleInterrupt(void* arg);

public:
    /**
     * @brief Constructor for the Encoder class.
     * @param pinA GPIO pin for encoder channel A.
     * @param pinB GPIO pin for encoder channel B.
     */
    Encoder(uint8_t pinA, uint8_t pinB);

    /**
     * @brief Initializes the encoder pins and attaches interrupts.
     */
    void begin();

    /**
     * @brief Calculates the speed (RPM) based on elapsed time and pulses.
     * @param intervalMs Time interval in milliseconds for calculation.
     */
    void calculateSpeed(uint32_t intervalMs);

    /**
     * @brief Gets the current speed in RPM.
     * @return Current speed in RPM.
     */
    float getRPM() const;

    /**
     * @brief Gets the current angular position in degrees.
     * @return Angular position in degrees.
     */
    float getAngle() const;
};

#endif
