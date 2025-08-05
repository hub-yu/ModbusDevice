#ifndef NET_H
#define NET_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define NET_RCV_BUFFER_SIZE 64
#define NET_SND_BUFFER_SIZE 64

typedef enum
{
    SOCKET_UDP,
    SOCKET_TCP_SERVER,
    SOCKET_OTHER_2,
    SOCKET_OTHER_3,
    SOCKET_OTHER_4,
    SOCKET_OTHER_5,
    SOCKET_OTHER_6,
    SOCKET_OTHER_7,
    SOCKET_END,
} SOCKET_INDEX;

#define NET_TASK_NAME "net_task"
#define NET_TASK_PRIORITY 1
#define NET_TASK_STACK_SIZE 1024

__attribute__((weak)) size_t net_rcv_override(uint8_t sn, const void *array, size_t len);

void net_init();
void net_snd(uint8_t sn, const void *array, size_t len);

#endif