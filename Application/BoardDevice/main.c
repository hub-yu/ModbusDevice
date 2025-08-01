#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

#include "led.h"
#include "uart.h"
#include "device.h"


int main(void)
{
    SystemCoreClockUpdate();
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
    SysTick_Config(SystemCoreClock / configTICK_RATE_HZ);

    device_init();
    led_init();
    uart_init();

    vTaskStartScheduler();
    while (1)
    {
    }
}


void NMI_Handler(void)
{
    while (1)
    {
    }
}

void HardFault_Handler(void)
{
    while (1)
    {
    }
}


void MemManage_Handler(void)
{
    while (1)
    {
    }
}

void BusFault_Handler(void)
{
    while (1)
    {
    }
}

void UsageFault_Handler(void)
{
    while (1)
    {
    }
}

