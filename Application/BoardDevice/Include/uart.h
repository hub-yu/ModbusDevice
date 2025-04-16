#ifndef UART_H
#define UART_H

#include <stddef.h>

void uart_init();
void uart_snd(const void *array, size_t len);
void uart_snd_isr(const void *array, size_t len);

__attribute__((weak)) size_t uart_rcv_override(const void *array, size_t len);

#endif