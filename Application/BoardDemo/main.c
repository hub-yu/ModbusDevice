#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"

static volatile uint64_t ticks = 0;
static volatile int val = 0;
void SysTick_Handler()
{
    ticks += 1;
}

void USART2_IRQHandler()
{
    if (SET == USART_GetITStatus(USART2, USART_IT_RXNE))
    {
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
        USART_SendData(USART2, USART_ReceiveData(USART2));
    }

    if (SET == USART_GetITStatus(USART2, USART_IT_TXE))
    {
        USART_ClearITPendingBit(USART2, USART_IT_TXE);

        USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    }
}

// 延时函数
void delay(volatile uint64_t t)
{
    uint64_t tick = ticks + t;
    do
    {

    } while (tick > ticks);
}

int main(void)
{
    // 初始化滴答定时器
    SysTick_Config(SystemCoreClock / 1000); // 设置为1ms的滴答定时器

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // 使能USART2时钟
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    // 配置PA2为USART2的TX
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 连接PA2和PA3到USART2
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);

    USART_DeInit(USART2);
    USART_InitTypeDef USART_InitStruct;
    USART_InitStruct.USART_BaudRate = 115200;                                    // 设置波特率
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;                     // 8位数据位
    USART_InitStruct.USART_StopBits = USART_StopBits_1;                          // 1个停止位
    USART_InitStruct.USART_Parity = USART_Parity_No;                             // 无奇偶校验
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件流控制
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;                 // 发送和接收模式
    USART_Init(USART2, &USART_InitStruct);

    // 配置USART2中断优先级
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn; // USART2中断
    NVIC_InitStruct.NVIC_IRQChannelPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE; // 使能中断
    NVIC_Init(&NVIC_InitStruct);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // 使能接收中断
    USART_Cmd(USART2, ENABLE);                     // 使能USART2

    // LED
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // KEY
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    while (1)
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_4);   // 点亮LED
        delay(1000);                       // 延时
        GPIO_ResetBits(GPIOA, GPIO_Pin_4); // 熄灭LED
        delay(1000);                       // 延时
    }
}