#include "uart.h"
#include "stm32f0xx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"

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
            GPIO_SetBits(GPIOA, GPIO_Pin_1);
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
        GPIO_ResetBits(GPIOA, GPIO_Pin_1);
    }
}

static void uart_task(void *param)
{

    uint8_t array[UART_RCV_BUFFER_SIZE];
    volatile size_t len_received = 0;

    for (;;)
    {

        // volatile UBaseType_t uxHighWaterMark;//70
        // uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

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

void uart_init()
{
    xStreamBufferSnd = xStreamBufferCreate(UART_SND_BUFFER_SIZE, 1);
    xStreamBufferRcv = xStreamBufferCreate(UART_RCV_BUFFER_SIZE, 1);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    // 默认接收
    GPIO_ResetBits(GPIOA, GPIO_Pin_1);

    USART_DeInit(USART1);
    USART_InitTypeDef USART_InitStruct;
    USART_InitStruct.USART_BaudRate = UART_BAUD_RATE;                            // 设置波特率
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;                     // 8位数据位
    USART_InitStruct.USART_StopBits = USART_StopBits_1;                          // 1个停止位
    USART_InitStruct.USART_Parity = USART_Parity_No;                             // 无奇偶校验
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件流控制
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;                 // 发送和接收模式
    USART_Init(USART1, &USART_InitStruct);

    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);

    xTaskCreate(uart_task, UART_TASK_NAME, UART_TASK_STACK_SIZE, NULL, UART_TASK_PRIORITY, NULL);
}

void uart_snd(const void *array, size_t len)
{
    taskENTER_CRITICAL();
    xStreamBufferSend(xStreamBufferSnd, array, len, 0);
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    taskEXIT_CRITICAL();
}

void uart_snd_isr(const void *array, size_t len)
{
    UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
    xStreamBufferSendFromISR(xStreamBufferSnd, array, len, 0);
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}