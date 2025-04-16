#include "uart.h"
#include "config.h"

#include "stm32f0xx_rcc.h"
#include "stm32f0xx_usart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"

static StreamBufferHandle_t xStreamBufferSnd, xStreamBufferRcv;

void UART_IRQHANDLER()
{
    if (SET == USART_GetITStatus(UART_PERIPH, USART_IT_RXNE))
    {
        USART_ClearITPendingBit(UART_PERIPH, USART_IT_RXNE);
        uint8_t data = (uint8_t)USART_ReceiveData(UART_PERIPH);
        xStreamBufferSendFromISR(xStreamBufferRcv, (void *)&data, 1, 0);
    }

    if (SET == USART_GetITStatus(UART_PERIPH, USART_IT_TXE))
    {
        USART_ClearITPendingBit(UART_PERIPH, USART_IT_TXE);
        uint8_t data;
        if (xStreamBufferReceiveFromISR(xStreamBufferSnd, &data, 1, NULL) == pdPASS)
        {
            GPIO_SetBits(UART_GPIO_PORT, UART_GPIO_DIR_PIN);
            USART_SendData(UART_PERIPH, (uint16_t)data);
        }
        else
        {
            USART_ITConfig(UART_PERIPH, USART_IT_TXE, DISABLE);
            USART_ITConfig(UART_PERIPH, USART_IT_TC, ENABLE);
        }
    }

    if (SET == USART_GetITStatus(UART_PERIPH, USART_IT_TC))
    {
        USART_ITConfig(UART_PERIPH, USART_IT_TC, DISABLE);
        USART_ClearITPendingBit(UART_PERIPH, USART_IT_TC);
        GPIO_ResetBits(UART_GPIO_PORT, UART_GPIO_DIR_PIN);
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

    UART_CLK_CMD(UART_CLK, ENABLE);
    RCC_AHBPeriphClockCmd(UART_GPIO_CLK, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = UART_GPIO_TX_PIN | UART_GPIO_RX_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(UART_GPIO_PORT, &GPIO_InitStruct);
    GPIO_PinAFConfig(UART_GPIO_PORT, GPIO_PinSource2, GPIO_AF_1);
    GPIO_PinAFConfig(UART_GPIO_PORT, GPIO_PinSource3, GPIO_AF_1);

    GPIO_InitStruct.GPIO_Pin = UART_GPIO_DIR_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(UART_GPIO_PORT, &GPIO_InitStruct);
    // 默认接收
    GPIO_ResetBits(UART_GPIO_PORT, UART_GPIO_DIR_PIN);

    USART_DeInit(UART_PERIPH);
    USART_InitTypeDef USART_InitStruct;
    USART_InitStruct.USART_BaudRate = UART_BAUD_RATE;                            // 设置波特率
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;                     // 8位数据位
    USART_InitStruct.USART_StopBits = USART_StopBits_1;                          // 1个停止位
    USART_InitStruct.USART_Parity = USART_Parity_No;                             // 无奇偶校验
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件流控制
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;                 // 发送和接收模式
    USART_Init(UART_PERIPH, &USART_InitStruct);

    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = UART_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    USART_ITConfig(UART_PERIPH, USART_IT_RXNE, ENABLE);
    USART_Cmd(UART_PERIPH, ENABLE);

    xTaskCreate(uart_task, UART_TASK_NAME, UART_TASK_STACK_SIZE, NULL, UART_TASK_PRIORITY, NULL);
}

void uart_snd(const void *array, size_t len)
{
    taskENTER_CRITICAL();
    xStreamBufferSend(xStreamBufferSnd, array, len, 0);
    USART_ITConfig(UART_PERIPH, USART_IT_TXE, ENABLE);
    taskEXIT_CRITICAL();
}

void uart_snd_isr(const void *array, size_t len)
{
    UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
    xStreamBufferSendFromISR(xStreamBufferSnd, array, len, 0);
    USART_ITConfig(UART_PERIPH, USART_IT_TXE, ENABLE);
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}