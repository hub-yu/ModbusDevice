#include "led.h"

#include "config.h"

#include "FreeRTOS.h"
#include "task.h"

#include "stm32f0xx_rcc.h"
#include "stm32f0xx_gpio.h"


static uint16_t slaveId = MODBUS_ADDR;

static void led_on() {
    GPIO_ResetBits(LED_GPIO_PORT, LED_GPIO_PIN);
}

static void led_off() {
    GPIO_SetBits(LED_GPIO_PORT, LED_GPIO_PIN);
}

static void led_0() {

    for(int32_t i = 0; i < 1; i++) {
        led_on();
        vTaskDelay(pdMS_TO_TICKS(400));
        led_off();
        vTaskDelay(pdMS_TO_TICKS(400));
    }

}

static void led_1() {

    for(int32_t i = 0; i < 2; i++) {
        led_on();
        vTaskDelay(pdMS_TO_TICKS(200));
        led_off();
        vTaskDelay(pdMS_TO_TICKS(200));
    }

}


/**
 * 
 * @brief LED task
 * 
 * @param param 
 * @return None
 */
static void led_task(void *param)
{
    slaveId = *(uint16_t *)(0x8003c00);
    for (;;)
    {
        // // volatile UBaseType_t uxHighWaterMark;//70
        // // uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

        // vTaskDelay(pdMS_TO_TICKS(LED_TASK_DELAY));
        // GPIO_SetBits(LED_GPIO_PORT, LED_GPIO_PIN);
        // // val = xPortGetFreeHeapSize();
        // vTaskDelay(pdMS_TO_TICKS(LED_TASK_DELAY));
        // GPIO_ResetBits(LED_GPIO_PORT, LED_GPIO_PIN);
        // // val = xPortGetMinimumEverFreeHeapSize();
      

        led_on();
        vTaskDelay(pdMS_TO_TICKS(2000));
        led_off();

        for(int i = 0; i < 8; i++) {
            vTaskDelay(pdMS_TO_TICKS(500));

            if(slaveId & (0x80 >> i)) 
                led_1();            
            else 
                led_0();     
        }
        vTaskDelay(pdMS_TO_TICKS(1000));

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