#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"

static volatile uint64_t ticks = 0;
void SysTick_Handler()
{
    ticks += 1;
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

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    // LED
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
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