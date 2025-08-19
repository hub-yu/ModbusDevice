#include "stm32f10x.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

#include "led.h"
#include "uart.h"
#include "net.h"
#include "log.h"
#include "modbus.h"
#include "device.h"

static void consoleHook(const uint8_t *d, int32_t l)
{
    uart_snd(d, l);
}

int main(void)
{
    SystemCoreClockUpdate();
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
    SysTick_Config(SystemCoreClock / configTICK_RATE_HZ);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    device_init();

    led_init();
    const uint32_t baudrate[] = {115200, 57600, 19200, 9600};
    uart_init(baudrate[FLASH_REG_CONFIG & REG_CONFIG_BAUDRATE]);
    net_init();

    // 是否开启串口日志输出
    if ((FLASH_REG_CONFIG & REG_CONFIG_LOG) || (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15) == Bit_RESET))
        LogSetHook(consoleHook);

    LOG_INFO("##############################\r\n");
    LOG_INFO("#### build %s %s\r\n", __DATE__, __TIME__);
    LOG_INFO("#### version %d.%d.%d\r\n", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
    LOG_INFO("##############################\r\n");

    // 这里是必要的 强制链接器保留strtok()函数
    char str[] = "hello,world!";
    char *token = strtok(str, ",");

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
