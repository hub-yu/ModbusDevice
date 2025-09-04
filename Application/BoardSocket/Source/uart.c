#include "uart.h"
#include "stm32f10x.h"

#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"

#include <string.h>

static StreamBufferHandle_t xStreamBufferSnd, xStreamBufferRcv;

void USART1_IRQHandler()
{
    if (SET == USART_GetITStatus(USART1, USART_IT_RXNE))
    {
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        uint8_t data = (uint8_t)USART_ReceiveData(USART1);
        xStreamBufferSendFromISR(xStreamBufferRcv, (void *)&data, 1, 0);
    }

    if (SET == USART_GetITStatus(USART1, USART_IT_TXE))
    {
        USART_ClearITPendingBit(USART1, USART_IT_TXE);
        uint8_t data;
        if (xStreamBufferReceiveFromISR(xStreamBufferSnd, &data, 1, NULL) == pdPASS)
        {
            // USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
            GPIO_SetBits(GPIOA, GPIO_Pin_8);
            USART_SendData(USART1, (uint16_t)data);
        }
        else
        {
            USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
            USART_ITConfig(USART1, USART_IT_TC, ENABLE);
        }
    }

    if (SET == USART_GetITStatus(USART1, USART_IT_TC))
    {
        USART_ITConfig(USART1, USART_IT_TC, DISABLE);
        USART_ClearITPendingBit(USART1, USART_IT_TC);
        GPIO_ResetBits(GPIOA, GPIO_Pin_8);
        // USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        // USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    }

    if (SET == USART_GetITStatus(USART1, USART_IT_ORE_RX))
        USART_ClearITPendingBit(USART1, USART_IT_ORE_RX);

    if (SET == USART_GetITStatus(USART1, USART_IT_ORE_ER))
        USART_ClearITPendingBit(USART1, USART_IT_ORE_ER);

    if (SET == USART_GetITStatus(USART1, USART_IT_NE))
        USART_ClearITPendingBit(USART1, USART_IT_NE);

    if (SET == USART_GetITStatus(USART1, USART_IT_FE))
        USART_ClearITPendingBit(USART1, USART_IT_FE);

    if (SET == USART_GetITStatus(USART1, USART_IT_PE))
        USART_ClearITPendingBit(USART1, USART_IT_PE);
}

static void uart_task(void *param)
{

    uint8_t array[UART_RCV_BUFFER_SIZE];
    volatile size_t len_received = 0;

    for (;;)
    {

        volatile UBaseType_t uxHighWaterMark; // 70
        uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

        size_t len_used = len_received ? uart_rcv_override(array, len_received) : 0;
        if (len_used)
        {
            for (size_t i = 0; i < (len_received - len_used); i++)
                array[i] = array[i + len_used];

            len_received -= len_used;
            continue;
        }

        size_t len_rcv = xStreamBufferReceive(xStreamBufferRcv, (void *)(array + len_received), sizeof(array) - len_received, pdMS_TO_TICKS(1000));
        if (len_rcv)
        {
            len_received += len_rcv;
            continue;
        }

        // 长时间无接收清空缓存
        len_received = 0;
    }
}

void uart_init(uint32_t baudrate)
{
    xStreamBufferSnd = xStreamBufferCreate(UART_SND_BUFFER_SIZE, 1);
    xStreamBufferRcv = xStreamBufferCreate(UART_RCV_BUFFER_SIZE, 1);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct = {
        .GPIO_Pin = GPIO_Pin_9,        // TX
        .GPIO_Mode = GPIO_Mode_AF_PP,  // 复用推挽输出
        .GPIO_Speed = GPIO_Speed_50MHz // 50MHz
    };
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;            // RX
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING; // GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_ResetBits(GPIOA, GPIO_Pin_8); // 默认接收

    USART_DeInit(USART1);
    USART_InitTypeDef USART_InitStruct = {
        .USART_BaudRate = baudrate,                                  // 设置波特率
        .USART_WordLength = USART_WordLength_8b,                     // 8位数据位
        .USART_StopBits = USART_StopBits_1,                          // 1个停止位
        .USART_Parity = USART_Parity_No,                             // 无奇偶校验
        .USART_HardwareFlowControl = USART_HardwareFlowControl_None, // 无硬件流控制
        .USART_Mode = USART_Mode_Tx | USART_Mode_Rx                  // 发送和接收模式
    };
    USART_Init(USART1, &USART_InitStruct);

    NVIC_InitTypeDef NVIC_InitStruct = {
        .NVIC_IRQChannel = USART1_IRQn,
        .NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1,
        .NVIC_IRQChannelSubPriority = 0, // 子优先级0
        .NVIC_IRQChannelCmd = ENABLE     // Enable the USART1 interrupt
    };
    NVIC_Init(&NVIC_InitStruct);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART1, USART_IT_ERR, DISABLE);

    USART_Cmd(USART1, ENABLE);

    xTaskCreate(uart_task, UART_TASK_NAME, UART_TASK_STACK_SIZE, NULL, UART_TASK_PRIORITY, NULL);
}

void uart_snd(const void *array, size_t len)
{
    size_t len_snd = (len <= 0) ? strlen(array) : len;
    taskENTER_CRITICAL();
    xStreamBufferSend(xStreamBufferSnd, array, len_snd, 0);
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    taskEXIT_CRITICAL();
}

void uart_snd_isr(const void *array, size_t len)
{
    size_t len_snd = (len <= 0) ? strlen(array) : len;
    UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
    xStreamBufferSendFromISR(xStreamBufferSnd, array, len_snd, 0);
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}