#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

#include "led.h"
#include "uart.h"
#include "device.h"
// #include "log.h"

// static void consoleHook(const uint8_t *d, int32_t l)
// {
//     uart_snd(d, l);
// }

int main(void)
{
    SystemCoreClockUpdate();
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
    SysTick_Config(SystemCoreClock / configTICK_RATE_HZ);

    device_init();
    led_init();
    const uint32_t baudrate[] = {115200, 57600, 19200, 9600};
    uart_init(baudrate[FLASH_REG_CONFIG & REG_CONFIG_BAUDRATE]);

    // 是否开启串口日志输出
    // if (FLASH_REG_CONFIG & REG_CONFIG_LOG)
    //     LogSetHook(consoleHook);

    // LOG_INFO("##############################\r\n");
    // LOG_INFO("#### build %s %s\r\n", __DATE__, __TIME__);
    // LOG_INFO("#### version %d.%d.%d\r\n", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
    // LOG_INFO("##############################\r\n");

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

