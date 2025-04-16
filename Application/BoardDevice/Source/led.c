#include "led.h"

#include "config.h"

#include "FreeRTOS.h"
#include "task.h"

#include "stm32f0xx_rcc.h"
#include "stm32f0xx_gpio.h"

static void led_task(void *param)
{
    // volatile val = 0;
    for (;;)
    {
        // volatile UBaseType_t uxHighWaterMark;//70
        // uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

        vTaskDelay(pdMS_TO_TICKS(LED_TASK_DELAY));
        GPIO_SetBits(LED_GPIO_PORT, LED_GPIO_PIN);
        // val = xPortGetFreeHeapSize();
        vTaskDelay(pdMS_TO_TICKS(LED_TASK_DELAY));
        GPIO_ResetBits(LED_GPIO_PORT, LED_GPIO_PIN);
        // val = xPortGetMinimumEverFreeHeapSize();
    }
}

void led_init()
{
    RCC_AHBPeriphClockCmd(LED_GPIO_CLK, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = LED_GPIO_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);

    xTaskCreate(led_task, LED_TASK_NAME, LED_TASK_STACK_SIZE, NULL, LED_TASK_PRIORITY, NULL);
}