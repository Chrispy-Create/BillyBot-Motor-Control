#ifndef PICO_UART_TRANSPORTS_H
#define PICO_UART_TRANSPORTS_H

#ifdef __cplusplus
extern "C" {
#endif

/// Call this once at startup to hook up the UART transport
/// Implemented in pico_uart_transport.c
void init_pico_uart_transports(void);

#include <stdio.h>
#include <stdint.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>

// The low-level callbacks:
bool pico_serial_transport_open(uxrCustomTransport * transport);
bool pico_serial_transport_close(uxrCustomTransport * transport);
size_t pico_serial_transport_write(
  uxrCustomTransport* transport,
  const uint8_t * buf,
  size_t len,
  uint8_t * err);
size_t pico_serial_transport_read(
  uxrCustomTransport* transport,
  uint8_t* buf,
  size_t len,
  int timeout,
  uint8_t* err);

#ifdef __cplusplus
}
#endif

#endif // PICO_UART_TRANSPORTS_H