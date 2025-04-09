#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

static void led_init(void *param)
{

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void led_task(void *param)
{
    led_init(param);

    for (;;)
    {

        vTaskDelay(pdMS_TO_TICKS(500));
        GPIO_SetBits(GPIOA, GPIO_Pin_4);
        vTaskDelay(pdMS_TO_TICKS(500));
        GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    }
}

int main(void)
{
    SystemCoreClockUpdate();
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
    SysTick_Config(SystemCoreClock / configTICK_RATE_HZ);

    xTaskCreate(led_task, "led_task", 256, NULL, 3, NULL);
    vTaskStartScheduler();
    while (1)
    {
    }
}