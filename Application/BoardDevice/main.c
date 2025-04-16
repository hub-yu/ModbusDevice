#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

#include "led.h"
#include "uart.h"
#include "modbus.h"

int main(void)
{
    SystemCoreClockUpdate();
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
    SysTick_Config(SystemCoreClock / configTICK_RATE_HZ);

    led_init();

    uart_init();
    modbus_init();
    vTaskStartScheduler();
    for (;;)
    {
    }
}