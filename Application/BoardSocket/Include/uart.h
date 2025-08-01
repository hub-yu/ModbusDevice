#ifndef UART_H
#define UART_H

#include <stddef.h>

#define UART_BAUD_RATE 115200
#define UART_RCV_BUFFER_SIZE 64
#define UART_SND_BUFFER_SIZE 800
#define UART_TASK_NAME "uart_task"
#define UART_TASK_PRIORITY 2
#define UART_TASK_STACK_SIZE 300

void uart_init();
void uart_snd(const void *array, size_t len);
void uart_snd_isr(const void *array, size_t len);

__attribute__((weak)) size_t uart_rcv_override(const void *array, size_t len);

#endif