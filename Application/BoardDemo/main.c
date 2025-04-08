#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"

// 延时函数
void delay(volatile uint32_t delay)
{
    while (delay--)
    {
        __asm("nop");
    }
}

int main(void)
{
    // // 初始化系统
    // SystemInit();

    // 配置为输出模式
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    {
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOA, &GPIO_InitStruct);
    }

    {
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOA, &GPIO_InitStruct);
    }

    // unsigned char number_buffer[26] = { 0 };
    // int length = sprintf(number_buffer, "%d", 123);  // 测试是否正常
    // double a = 3.14;
    // length = sprintf(number_buffer, "%f", 3.14f);
    // GPIO_ResetBits(GPIOA, GPIO_Pin_4); // 点亮LED
    while (1)
    {

        for (int i = 0; i < 10000; i++)
        {
            GPIO_SetBits(GPIOA, GPIO_Pin_4); // 点亮LED
            delay(100);                    // 延时
        }
        for (int i = 0; i < 10000; i++)
        {
        GPIO_ResetBits(GPIOA, GPIO_Pin_4); // 熄灭LED
            delay(100);                      // 延时
        }
    }
}