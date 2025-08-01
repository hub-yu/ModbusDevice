#include "led.h"
#include "stm32f0xx.h"

#include "FreeRTOS.h"
#include "task.h"


static void led_task(void *param)
{
    for (;;)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_1);
        vTaskDelay(pdMS_TO_TICKS(LED_TASK_DELAY));
        GPIO_ResetBits(GPIOB, GPIO_Pin_1);
        vTaskDelay(pdMS_TO_TICKS(LED_TASK_DELAY));
    }
}


void led_init()
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    xTaskCreate(led_task, LED_TASK_NAME, LED_TASK_STACK_SIZE, NULL, LED_TASK_PRIORITY, NULL);
}