#include "motor_driver.h"
#include "pico/stdlib.h"
#include <hardware/pwm.h>
#include <hardware/gpio.h>

// Pins (your mapping)
#define RIGHT_MOTOR_DIR_PIN 17   // Pin 22 → base of DIR NPN (Right)
#define RIGHT_MOTOR_PWM_PIN 16   // Pin 21 → RC node (Right)
#define LEFT_MOTOR_DIR_PIN  19   // Pin 25 → base of DIR NPN (Left)
#define LEFT_MOTOR_PWM_PIN  18   // Pin 24 → RC node (Left)

// If forward on your WS55 requires F/R SINKED to COM, set to 1. If it’s the opposite on your unit, set 0.
#define RIGHT_FORWARD_SINKS 1
#define LEFT_FORWARD_SINKS  1

// speed “gains” for your /cmd_vel mix 
static int baseSpeed      = 120;
static int turnSpeed      = 102; 
static const int tankTurn =  92;

static inline uint16_t clamp_u8(int v) {
    if (v < 0)   return 0;
    if (v > 255) return 255;
    return (uint16_t)v;
}

// Map |speed| (0…255) to PWM level with **inversion** for the SV sink:
// 0 (stop)  -> 255 (SV≈0 V)   ; 255 (full) -> 0 (SV≈10 V)
static inline void set_pwm_inverted(uint pin, uint16_t mag_0_255) {
    uint slice = pwm_gpio_to_slice_num(pin);
    uint16_t level = 255 - clamp_u8(mag_0_255);
    pwm_set_gpio_level(pin, level);
}

// DIR helper: ‘sink’ = drive GPIO HIGH to turn on 2N2222A (pull F/R→COM)
static inline void set_dir_sink(uint pin, bool sink) {
    gpio_put(pin, sink ? 1 : 0);
}

// Initialize GPIO/PWM for both motors
void motor_init(void) {
    // DIR pins
    gpio_init(RIGHT_MOTOR_DIR_PIN);
    gpio_set_dir(RIGHT_MOTOR_DIR_PIN, GPIO_OUT);
    gpio_put(RIGHT_MOTOR_DIR_PIN, 0);  // default: open (not sinking)

    gpio_init(LEFT_MOTOR_DIR_PIN);
    gpio_set_dir(LEFT_MOTOR_DIR_PIN, GPIO_OUT);
    gpio_put(LEFT_MOTOR_DIR_PIN, 0);

    // PWM pins
    gpio_set_function(RIGHT_MOTOR_PWM_PIN, GPIO_FUNC_PWM);
    gpio_set_function(LEFT_MOTOR_PWM_PIN,  GPIO_FUNC_PWM);

    uint right_slice = pwm_gpio_to_slice_num(RIGHT_MOTOR_PWM_PIN);
    uint left_slice  = pwm_gpio_to_slice_num(LEFT_MOTOR_PWM_PIN);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_wrap(&cfg, 255);          // 8-bit
    // ~2 kHz: f = 125e6/(clkdiv*(255+1)) → clkdiv ≈ 244
    pwm_config_set_clkdiv(&cfg, 244.0f);

    pwm_init(right_slice, &cfg, true);
    pwm_init(left_slice,  &cfg, true);

    // Safe default: motors “stopped” (SV≈0 V) with our inverted mapping
    pwm_set_gpio_level(RIGHT_MOTOR_PWM_PIN, 255);
    pwm_set_gpio_level(LEFT_MOTOR_PWM_PIN,  255);
}

// Drive helper with signed 8-bit magnitudes 
static void DriveInt(int left, int right) {
    // clamp
    if (left  > 255) left  = 255; if (left  < -255) left  = -255;
    if (right > 255) right = 255; if (right < -255) right = -255;

    // Direction whether we must SINK F/R to get “forward”
    bool left_forward_sink  = LEFT_FORWARD_SINKS;
    bool right_forward_sink = RIGHT_FORWARD_SINKS;

    // LEFT motor
    bool L_forward = (left >= 0);
    // If forward requires sink, sink when L_forward==true; else sink when false
    set_dir_sink(LEFT_MOTOR_DIR_PIN,  L_forward ? left_forward_sink : !left_forward_sink);
    set_pwm_inverted(LEFT_MOTOR_PWM_PIN,  (uint16_t) (left >= 0 ? left : -left));

    // RIGHT motor
    bool R_forward = (right >= 0);
    set_dir_sink(RIGHT_MOTOR_DIR_PIN, R_forward ? right_forward_sink : !right_forward_sink);
    set_pwm_inverted(RIGHT_MOTOR_PWM_PIN, (uint16_t) (right >= 0 ? right : -right));
}

void motor_drive(float linear, float angular) {
    int L = (int)(linear * baseSpeed) - (int)(angular * turnSpeed);
    int R = (int)(linear * baseSpeed) + (int)(angular * turnSpeed);

    if (angular >  0.9f) { L = -tankTurn; R = +tankTurn; }
    if (angular < -0.9f) { L = +tankTurn; R = -tankTurn; }

    DriveInt(L, R);
}

void motor_stop(void) { DriveInt(0, 0); }
