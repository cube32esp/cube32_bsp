/**
 * @file pwm_servo_adapter.h
 * @brief Adapter: PwmServo → IServoDriver interface
 * 
 * Wraps the CUBE32 PwmServo singleton to implement the abstract
 * IServoDriver interface. This allows the Robot Head action engine
 * (and any future automation layer) to use PWM servos without
 * depending on the concrete PwmServo class directly.
 */

#ifndef CUBE32_DRIVERS_SERVO_PWM_SERVO_ADAPTER_H
#define CUBE32_DRIVERS_SERVO_PWM_SERVO_ADAPTER_H

#include "drivers/servo/servo_interface.h"

#ifdef CONFIG_CUBE32_SERVO_ENABLED
#include "drivers/servo/pwm_servo.h"
#endif

#ifdef __cplusplus

namespace cube32 {

#ifdef CONFIG_CUBE32_SERVO_ENABLED

/**
 * @brief Adapts PwmServo singleton to the IServoDriver interface
 */
class PwmServoAdapter : public IServoDriver {
public:
    PwmServoAdapter() : m_servo(PwmServo::instance()) {}

    cube32_result_t setAngle(uint8_t channel, float angle) override {
        return m_servo.setAngle(channel, angle);
    }

    float getAngle(uint8_t channel) const override {
        return m_servo.getAngle(channel);
    }

    cube32_result_t smoothMove(uint8_t channel, float target_angle, float speed_pct) override {
        return m_servo.smoothMove(channel, target_angle, speed_pct);
    }

    bool isMoving(uint8_t channel) const override {
        return m_servo.isMoving(channel);
    }

    cube32_result_t stopMove(uint8_t channel) override {
        return m_servo.stopMove(channel);
    }

    uint8_t getNumChannels() const override {
        return m_servo.getNumChannels();
    }

    uint16_t getMaxRotation(uint8_t channel) const override {
        return m_servo.getMaxRotation(channel);
    }

    bool isReady() const override {
        return m_servo.isInitialized();
    }

private:
    PwmServo& m_servo;
};

#endif // CONFIG_CUBE32_SERVO_ENABLED

} // namespace cube32

#endif // __cplusplus
#endif // CUBE32_DRIVERS_SERVO_PWM_SERVO_ADAPTER_H
