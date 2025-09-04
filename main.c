#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include <rmw_microros/rmw_microros.h>

// micro-ROS transport
#include "pico_uart_transports.h"

// micro-ROS client API
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

// your motor driver
#include "motor_driver.h"


// Global ROS Objects
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

// Error Handling 
// If any ROS setup step fails, the Pico will enter this panic loop
void error_loop() {
    while (true) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(50);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(50);
    }
}

// --- Subscriber Callback ---
// This function is called every time a message is received on /cmd_vel
void subscription_callback(const void *msgin) {
    // --- ADD THESE LINES FOR THE TEST ---
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1); // Turn LED ON
    sleep_ms(100);                             // Keep it on for a moment
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0); // Turn LED OFF
    // --- END OF TEST LINES ---

    const geometry_msgs__msg__Twist *twist_msg = (const geometry_msgs__msg__Twist *)msgin;
    // Call your motor drive function with the received velocities
    motor_drive(twist_msg->linear.x, twist_msg->angular.z);
}


int main() {
    // Standard Pico Setup 
    stdio_init_all();
    
    // Initialize the Wi-Fi/LED driver
    if (cyw43_arch_init()) {
        printf("CYW43 init failed\n");
        return -1;
    }
    
    // Initialize your motor driver pins
    motor_init();

    // Add a delay to wait for the ROS Agent to be ready on the Pi
    sleep_ms(3000);

    // Set up micro-ROS transport
    init_pico_uart_transports();

    // Wait for Agent 
    // Keep blinking slowly until the agent is connected
    while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(500);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(500);
    }
    // If we get here, the agent is connected. Turn the LED on solid for a moment.
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(500);

    // micro-ROS Initialization with Error Checking 
    allocator = rcl_get_default_allocator();

    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) { error_loop(); }
    if (rclc_node_init_default(&node, "pico_motor_controller_node", "", &support) != RCL_RET_OK) { error_loop(); }
    if (rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel") != RCL_RET_OK) { error_loop(); }
    if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) { error_loop(); }
    if (rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA) != RCL_RET_OK) { error_loop(); }

    //    Main Loop 
    while (true) {
        // A quick double-blink heartbeat to show the main loop is running
        static uint32_t last_blink_time = 0;
        if (to_ms_since_boot(get_absolute_time()) - last_blink_time > 1000) {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
            sleep_ms(50);
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
            sleep_ms(50);
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
            sleep_ms(50);
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
            last_blink_time = to_ms_since_boot(get_absolute_time());
        }

        // Spin the ROS executor to handle incoming messages
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        sleep_ms(10);
    }

    return 0;
}