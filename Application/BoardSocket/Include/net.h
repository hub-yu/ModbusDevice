#ifndef NET_H
#define NET_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define NET_RCV_BUFFER_SIZE 256
#define NET_SND_BUFFER_SIZE 256

typedef enum
{
    SOCKET_CHANNEL_0,
    SOCKET_CHANNEL_1,
    SOCKET_CHANNEL_2,
    SOCKET_CHANNEL_3,
    SOCKET_CHANNEL_4,
    SOCKET_CHANNEL_5,
    SOCKET_CHANNEL_6,
    SOCKET_CHANNEL_7,
    SOCKET_END,
} SOCKET_INDEX;

#define NET_TASK_NAME "net_task"
#define NET_TASK_PRIORITY 1
#define NET_TASK_STACK_SIZE 1024

__attribute__((weak)) size_t net_rcv_override(uint8_t sn, const void *array, size_t len);

void net_init();
void net_snd(uint8_t sn, const void *array, size_t len);

#endif