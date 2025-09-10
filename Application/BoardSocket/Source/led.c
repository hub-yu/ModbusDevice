#include "led.h"
#include "stm32f10x.h"

#include "FreeRTOS.h"
#include "task.h"

static void led_task(void *param)
{
    for (;;)
    {       
        volatile UBaseType_t uxHighWaterMark; // 70
        uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        // IWDG_ReloadCounter();// 喂狗

        GPIO_SetBits(GPIOB, GPIO_Pin_3);
        vTaskDelay(pdMS_TO_TICKS(LED_TASK_DELAY));
        GPIO_ResetBits(GPIOB, GPIO_Pin_3);
        vTaskDelay(pdMS_TO_TICKS(LED_TASK_DELAY));
    }
}

void led_init()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct = {
        .GPIO_Pin = GPIO_Pin_3,
        .GPIO_Mode = GPIO_Mode_Out_PP,
        .GPIO_Speed = GPIO_Speed_50MHz
    };

    GPIO_Init(GPIOB, &GPIO_InitStruct);

    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);   //使能看门狗
    IWDG_SetPrescaler(IWDG_Prescaler_32);     //独立看门狗主频约40Khz  32分频后 约 0.8ms
    IWDG_SetReload(1000);// 3设置重装载值（0~0xFFF） 3750 * 0.8ms = 3s
    IWDG_ReloadCounter(); // 喂狗（防止立即复位）
    // IWDG_Enable();

    xTaskCreate(led_task, LED_TASK_NAME, LED_TASK_STACK_SIZE, NULL, LED_TASK_PRIORITY, NULL);
}
