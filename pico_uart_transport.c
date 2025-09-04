#include "pico_uart_transports.h"       // your transport API header
#include <rmw_microros/rmw_microros.h>  // for rmw_uros_set_custom_transport
#include <stdio.h>
#include <time.h>
#include "pico/stdlib.h"

#include <uxr/client/profile/transport/custom/custom_transport.h>

/// Hook up UART0 as the micro-ROS transport
void init_pico_uart_transports(void)
{
    // Register our four callbacks with the micro-ROS client
    rmw_ret_t ret = rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    if (ret != RMW_RET_OK) {
        // Optional: indicate failure (e.g., blink the LED or halt)
    }
}

void usleep(uint64_t us)
{
    sleep_us(us);
}

int clock_gettime(clockid_t unused, struct timespec *tp)
{
    uint64_t m = time_us_64();
    tp->tv_sec  = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;
    return 0;
}

bool pico_serial_transport_open(struct uxrCustomTransport * transport)
{
    // Ensure stdio_init_all() is only called once
    static bool first_time = true;
    if (first_time) {
        stdio_init_all();
        first_time = false;
    }
    return true;
}

bool pico_serial_transport_close(struct uxrCustomTransport * transport)
{
    // Nothing to do for UART close on Pico
    return true;
}

size_t pico_serial_transport_write(
    struct uxrCustomTransport * transport,
    const uint8_t *buf,
    size_t len,
    uint8_t *errcode)
{
    for (size_t i = 0; i < len; i++) {
        if (putchar(buf[i]) != buf[i]) {
            *errcode = 1;
            return i;
        }
    }
    return len;
}

size_t pico_serial_transport_read(
    struct uxrCustomTransport * transport,
    uint8_t *buf,
    size_t len,
    int timeout,
    uint8_t *errcode)
{
    uint64_t start_us = time_us_64();
    for (size_t i = 0; i < len; i++) {
        int64_t remaining_us = (int64_t)timeout * 1000 - (time_us_64() - start_us);
        if (remaining_us < 0) {
            *errcode = 1;
            return i;
        }
        int c = getchar_timeout_us(remaining_us);
        if (c == PICO_ERROR_TIMEOUT) {
            *errcode = 1;
            return i;
        }
        buf[i] = (uint8_t)c;
    }
    return len;
}