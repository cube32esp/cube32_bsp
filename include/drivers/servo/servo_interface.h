/**
 * @file servo_interface.h
 * @brief Abstract Servo Driver Interface
 * 
 * Provides a hardware-agnostic interface for servo control.
 * Any servo driver (PWM LEDC, one-wire bus servo, etc.) can implement
 * this interface to be used with higher-level automation layers
 * such as the Robot Head action engine.
 */

#ifndef CUBE32_DRIVERS_SERVO_SERVO_INTERFACE_H
#define CUBE32_DRIVERS_SERVO_SERVO_INTERFACE_H

#include "utils/common.h"
#include <cstdint>

#ifdef __cplusplus

namespace cube32 {

/**
 * @brief Abstract servo driver interface
 * 
 * Higher-level layers (e.g. RobotHead) program against this interface,
 * allowing the underlying servo hardware to be swapped without 
 * changing the automation logic.
 */
class IServoDriver {
public:
    virtual ~IServoDriver() = default;

    /**
     * @brief Set servo angle immediately
     * @param channel Servo channel index
     * @param angle Target angle in degrees
     * @return CUBE32_OK on success
     */
    virtual cube32_result_t setAngle(uint8_t channel, float angle) = 0;

    /**
     * @brief Get current servo angle
     * @param channel Servo channel index
     * @return Current angle in degrees, or -1 on error
     */
    virtual float getAngle(uint8_t channel) const = 0;

    /**
     * @brief Smooth move to target angle
     * @param channel Servo channel index
     * @param target_angle Target angle in degrees
     * @param speed_pct Speed percentage (1.0 - 100.0)
     * @return CUBE32_OK on success, CUBE32_NOT_SUPPORTED if not applicable
     */
    virtual cube32_result_t smoothMove(uint8_t channel, float target_angle, float speed_pct = 50.0f) = 0;

    /**
     * @brief Check if a smooth move is in progress
     * @param channel Servo channel index
     * @return true if moving
     */
    virtual bool isMoving(uint8_t channel) const = 0;

    /**
     * @brief Stop any ongoing smooth move
     * @param channel Servo channel index
     * @return CUBE32_OK on success
     */
    virtual cube32_result_t stopMove(uint8_t channel) = 0;

    /**
     * @brief Get number of available channels
     */
    virtual uint8_t getNumChannels() const = 0;

    /**
     * @brief Get max rotation for a channel (90, 180, or 360)
     * @param channel Servo channel index
     */
    virtual uint16_t getMaxRotation(uint8_t channel) const = 0;

    /**
     * @brief Check if driver is initialized / ready
     */
    virtual bool isReady() const = 0;
};

} // namespace cube32

#endif // __cplusplus
#endif // CUBE32_DRIVERS_SERVO_SERVO_INTERFACE_H
