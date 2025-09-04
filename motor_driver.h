// motor_driver.h
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the GPIO/PWM pins for your two motors.
void motor_init(void);

// Drive the motors given a linear and angular velocity.
// - `linear` and `angular` are in the same units you used on the host 
void motor_drive(float linear, float angular);

/// Immediately stop both motors.
void motor_stop(void);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_DRIVER_H