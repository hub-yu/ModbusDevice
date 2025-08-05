#include "net.h"
#include "log.h"
#include "stm32f10x.h"

#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"
#include "semphr.h"

#include "w5500.h"
#include "socket.h"
#include "wizchip_conf.h"
#include "device.h"

static SemaphoreHandle_t xSemaphore;
static StreamBufferHandle_t xStreamBufferSnd[SOCKET_END];
static DeviceMap deviceMap;

void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line10) != RESET)
    {
        // 清除中断标志位（重要！）
        EXTI_ClearITPendingBit(EXTI_Line10);
        xSemaphoreGiveFromISR(xSemaphore, NULL);
    }
}

static uint16_t spi_rw(uint16_t data)
{
    uint32_t count = 10000;
    while ((count) && (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET))
        count--;
    if (count == 0)
        return 0xffff;

    SPI_I2S_SendData(SPI2, data);
    count = 10000;
    while ((count) && (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET))
        count--;
    if (count == 0)
        return 0xffff;
    uint16_t recv = SPI_I2S_ReceiveData(SPI2);
    return recv;
}

static void cris_en()
{
}

static void cris_ex()
{
}

static void cs_sel()
{
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
}

static void cs_desel()
{
    GPIO_SetBits(GPIOB, GPIO_Pin_12);
}

static uint8_t spi_read()
{
    return spi_rw(0x00);
}

static void spi_write(uint8_t data)
{
    spi_rw(data);
}

static void reset_net(void)
{
    GPIO_ResetBits(GPIOB, GPIO_Pin_11); // min 500us
    vTaskDelay(pdMS_TO_TICKS(10));
    GPIO_SetBits(GPIOB, GPIO_Pin_11); // max 1ms
    vTaskDelay(pdMS_TO_TICKS(10));

    setPHYCFGR(0xb8); // reset PHY
    vTaskDelay(pdMS_TO_TICKS(10));
}

static void get_common_regs()
{
    LOG_INFO("Mode: 0x%02x\r\n", getMR()); // 配置模式寄存器

    uint8_t mac[6] = {0};
    getSHAR(mac);
    LOG_INFO("mac: %02x.%02x.%02x.%02x.%02x.%02x\r\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]); // 网卡MAC地址

    uint8_t ip[4] = {0};
    getSIPR(ip);
    LOG_INFO("ip: %d.%d.%d.%d\r\n", ip[0], ip[1], ip[2], ip[3]); // 网卡IP地址

    uint8_t mask[4] = {0};
    getSUBR(mask);
    LOG_INFO("mask: %d.%d.%d.%d\r\n", mask[0], mask[1], mask[2], mask[3]); // 网卡子网掩码

    uint8_t gateway[4] = {0};
    getGAR(gateway);
    LOG_INFO("gateway: %d.%d.%d.%d\r\n", gateway[0], gateway[1], gateway[2], gateway[3]); // 网卡网关地址

    // vTaskDelay(pdMS_TO_TICKS(1000));
    LOG_INFO("INTLEVEL: 0x%02x\r\n", getINTLEVEL()); // 中断触发延时 𝐼𝐴𝑊𝑇 = (𝐼𝑁𝑇𝐿𝐸𝑉𝐸𝐿 + 1) ×(1/𝑃𝐿𝐿𝑐𝑙𝑘3 × 4) (when INTLEVEL > 0)
    LOG_INFO("IR: 0x%02x\r\n", getIR());             // 状态中断标志
    LOG_INFO("IMR: 0x%02x\r\n", getIMR());           // 状态中断掩码
    LOG_INFO("SIR: 0x%02x\r\n", getSIR());           // socket中断标志
    setSIMR(0xff);
    LOG_INFO("SIMR: 0x%02x\r\n", getSIMR()); // socket中断掩码
    LOG_INFO("RTR: 0x%04x\r\n", getRTR());   // 重发延时 100us × RTR
    LOG_INFO("RCR: 0x%02x\r\n", getRCR());   // 重发次数 RCR + 1

    vTaskDelay(pdMS_TO_TICKS(200));
    LOG_INFO("PTIMER: 0x%02x\r\n", getPTIMER()); // ppp链路控制协议请求定时器  25ms × PTIMER
    LOG_INFO("PMAGIC: 0x%02x\r\n", getPMAGIC()); // PMAGIC configures the 4bytes magic number to be used in LCP echo request
    uint8_t pppoe_mac[6] = {0x00, 0x22, 0x33, 0x44, 0x55, 0x66};
    getPHAR(pppoe_mac); // PHAR should be written to the PPPoE server hardware address acquired in PPPoE connection process
    LOG_INFO("PHAR: %02x.%02x.%02x.%02x.%02x.%02x\r\n", pppoe_mac[0], pppoe_mac[1], pppoe_mac[2], pppoe_mac[3], pppoe_mac[4], pppoe_mac[5]);
    LOG_INFO("PSID: 0x%04x\r\n", getPSID()); // PSID should be written to the PPPoE sever session ID acquired in PPPoE connectionprocess.
    LOG_INFO("PMRU: 0x%02x\r\n", getPMRU()); // PMRU configures the maximum receive unit of PPPoE

    uint8_t unreachable_ip[4] = {0, 0, 0, 0};
    getUIPR(unreachable_ip);
    LOG_INFO("UIPR: %d.%d.%d.%d\r\n", unreachable_ip[0], unreachable_ip[1], unreachable_ip[2], unreachable_ip[3]); // 不可达 IP 地址寄存器
    LOG_INFO("UPORTR: %d\r\n", getUPORTR());                                                                       // 不可达端口寄存器

    LOG_INFO("PHYCFGR: 0x%02x\r\n", getPHYCFGR()); // configures PHY operation
    LOG_INFO("version: %d\r\n", getVERSIONR());    // W5500 版本号 always 0x04
}

static void get_socket_regs(uint8_t ch)
{
    LOG_INFO("ch[%d] ##############\r\n", ch);
    LOG_INFO("ch[%d] MR: 0x%02x\r\n", ch, getSn_MR(ch)); // Socket n Mode Register
    LOG_INFO("ch[%d] CR: 0x%02x\r\n", ch, getSn_CR(ch)); // Socket n Command Register

    // setSn_IR(ch, Sn_IR_RECV);
    LOG_INFO("ch[%d] IR: 0x%02x\r\n", ch, getSn_IR(ch)); // Socket n Interrupt Register
    LOG_INFO("ch[%d] SR: 0x%02x\r\n", ch, getSn_SR(ch)); // Socket n Status Register
    setSn_IMR(ch, Sn_IR_RECV | Sn_IR_DISCON | Sn_IR_CON);
    // setSn_IMR(ch, Sn_IR_RECV);
    LOG_INFO("ch[%d] IMR: 0x%02x\r\n", ch, getSn_IMR(ch)); // Socket n Interrupt Mask
    LOG_INFO("ch[%d] PORT: %d\r\n", ch, getSn_PORT(ch));   // Socket n Source Port Register

    uint8_t dhar[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    getSn_DHAR(ch, dhar); // Socket n Destination Hardware Address Register
    LOG_INFO("ch[%d] DHAR: %02x.%02x.%02x.%02x.%02x.%02x\r\n", ch, dhar[0], dhar[1], dhar[2], dhar[3], dhar[4], dhar[5]);

    uint8_t dipr[4] = {0x00, 0x00, 0x00, 0x00};
    getSn_DIPR(ch, dipr); // Socket n Destination IP Address Register
    LOG_INFO("ch[%d] DIPR: %d.%d.%d.%d\r\n", ch, dipr[0], dipr[1], dipr[2], dipr[3]);

    LOG_INFO("ch[%d] DPORT: %d\r\n", ch, getSn_DPORT(ch)); // Socket n Destination Port Register

    LOG_INFO("ch[%d] MSSR: 0x%04x\r\n", ch, getSn_MSSR(ch));         // Socket n Maximum Segment Size Register
    LOG_INFO("ch[%d] TOS: %d\r\n", ch, getSn_TOS(ch));               // Socket n IP Type of Service Register
    LOG_INFO("ch[%d] TTL: %d\r\n", ch, getSn_TTL(ch));               // Socket n TTL Register
    LOG_INFO("ch[%d] RXBUF_SIZE: %d\r\n", ch, getSn_RXBUF_SIZE(ch)); // Socket n RX Buffer Size Register
    LOG_INFO("ch[%d] TXBUF_SIZE: %d\r\n", ch, getSn_TXBUF_SIZE(ch)); // Socket n TX Buffer Size Register

    LOG_INFO("ch[%d] TX_FSR: 0X%04x\r\n", ch, getSn_TX_FSR(ch)); // Socket n TX Free Size Register

    LOG_INFO("ch[%d] TX_RD: 0X%04x\r\n", ch, getSn_TX_RD(ch)); // Socket n TX Read Pointer Register
    LOG_INFO("ch[%d] TX_WR: 0X%04x\r\n", ch, getSn_TX_WR(ch)); // Socket n TX Write Pointer Register

    LOG_INFO("ch[%d] RX_RSR: 0X%04x\r\n", ch, getSn_RX_RSR(ch)); // Socket n Received Size Register
    LOG_INFO("ch[%d] RX_RD: 0X%04x\r\n", ch, getSn_RX_RD(ch));   // Socket n RX Read Data Pointer Register

    LOG_INFO("ch[%d] RX_RX_WR: 0X%04x\r\n", ch, getSn_RX_WR(ch));     // Socket n RX Write Pointer Register
    LOG_INFO("ch[%d] RX_FRAG: 0X%04x\r\n", ch, getSn_FRAG(ch));       // Socket n Fragment Register
    LOG_INFO("ch[%d] RX_KPALVTR: 0X%02x\r\n", ch, getSn_KPALVTR(ch)); // Socket n Keep Alive Time Register

    LOG_INFO("ch[%d] RxMAX: 0X%02x\r\n", ch, getSn_RxMAX(ch)); //
    LOG_INFO("ch[%d] TxMAX: 0X%02x\r\n", ch, getSn_TxMAX(ch)); //
}


static void net_update()
{
    uint8_t mac[6] = {deviceMap.regs[REG_NET_MAC_0], deviceMap.regs[REG_NET_MAC_1], deviceMap.regs[REG_NET_MAC_2], deviceMap.regs[REG_NET_MAC_3], deviceMap.regs[REG_NET_MAC_4], deviceMap.regs[REG_NET_MAC_5]};
    uint8_t gateway[4] = {deviceMap.regs[REG_NET_GATEWAY_0], deviceMap.regs[REG_NET_GATEWAY_1], deviceMap.regs[REG_NET_GATEWAY_2], deviceMap.regs[REG_NET_GATEWAY_3]};
    uint8_t mask[4] = {deviceMap.regs[REG_NET_MASK_0], deviceMap.regs[REG_NET_MASK_1], deviceMap.regs[REG_NET_MASK_2], deviceMap.regs[REG_NET_MASK_3]};
    uint8_t ip[4] = {deviceMap.regs[REG_NET_IP_0], deviceMap.regs[REG_NET_IP_1], deviceMap.regs[REG_NET_IP_2], deviceMap.regs[REG_NET_IP_3]};
    setSHAR(mac);
    setGAR(gateway);
    setSUBR(mask);
    setSIPR(ip);
}

static void udp(uint8_t sn, BaseType_t taked)
{
    static uint8_t array[NET_RCV_BUFFER_SIZE];
    static size_t len_received = 0;
    static uint8_t remote_ip[4] = {192, 168, 1, 4};
    static uint16_t remote_port = 5000;

    LOG_INFO("sn: %d, SR: 0x%02x\r\n", sn, getSn_SR(sn));

    uint8_t snd[NET_SND_BUFFER_SIZE];
    size_t len_snd = xStreamBufferReceive(xStreamBufferSnd[sn], snd, NET_SND_BUFFER_SIZE, 0);

    switch (getSn_SR(sn))
    {

    case SOCK_UDP:
    {

        if (len_snd > 0)
            sendto(sn, snd, len_snd, remote_ip, remote_port);

        if (getSn_IR(sn) & Sn_IR_RECV)
            setSn_IR(sn, Sn_IR_RECV);

        if (taked != pdTRUE)
            len_received = 0;

        do
        {
            size_t len_used = len_received ? net_rcv_override(sn, array, len_received) : 0;
            if (len_used)
            {
                for (size_t i = 0; i < (len_received - len_used); i++)
                    array[i] = array[i + len_used];

                len_received -= len_used;
                continue;
            }

            int32_t ret = getSn_RX_RSR(sn);
            if (ret <= 0)
                break;
            uint16_t max_size = NET_RCV_BUFFER_SIZE - len_received;
            int32_t len_rcv = recvfrom(sn, array + len_received, (ret >= max_size ? max_size : ret), remote_ip, &remote_port);
            len_received += len_rcv;

        } while (1);
    }
    break;

    case SOCK_CLOSED:
        socket(sn, Sn_MR_UDP, deviceMap.regs[REG_NET_PORT_0 + sn], 0x00);
        xSemaphoreGive(xSemaphore);
        break;

    default:
        break;
    }
}

static void tcp_server(uint8_t sn, BaseType_t taked)
{
    static uint8_t array[NET_RCV_BUFFER_SIZE];
    static size_t len_received = 0;

    uint8_t snd[NET_SND_BUFFER_SIZE];
    size_t len_snd = xStreamBufferReceive(xStreamBufferSnd[sn], snd, NET_SND_BUFFER_SIZE, 0);

    LOG_INFO("sn: %d, SR: 0x%02x\r\n", sn, getSn_SR(sn));

    switch (getSn_SR(sn))
    {

    case SOCK_ESTABLISHED:

        if (len_snd > 0)
            send(sn, snd, len_snd);

        if (getSn_IR(sn) & Sn_IR_CON)
            setSn_IR(sn, Sn_IR_CON);

        if (getSn_IR(sn) & Sn_IR_RECV)
            setSn_IR(sn, Sn_IR_RECV);

        if (taked != pdTRUE)
            len_received = 0;

        do
        {
            size_t len_used = len_received ? net_rcv_override(sn, array, len_received) : 0;
            if (len_used)
            {
                for (size_t i = 0; i < (len_received - len_used); i++)
                    array[i] = array[i + len_used];

                len_received -= len_used;
                continue;
            }

            int32_t ret = getSn_RX_RSR(sn);
            if (ret <= 0)
                break;
            uint16_t max_size = NET_RCV_BUFFER_SIZE - len_received;
            int32_t len_rcv = recv(sn, array + len_received, (ret >= max_size ? max_size : ret));
            len_received += len_rcv;

        } while (1);

        break;
    case SOCK_CLOSE_WAIT:
        if (getSn_IR(sn) & Sn_IR_DISCON)
            setSn_IR(sn, Sn_IR_DISCON);
        disconnect(sn);
        break;

    case SOCK_INIT:
        listen(sn);
        xSemaphoreGive(xSemaphore);
        break;

    case SOCK_CLOSED:
        socket(sn, Sn_MR_TCP, deviceMap.regs[REG_NET_PORT_0 + sn], 0x00);
        xSemaphoreGive(xSemaphore);
        break;

    default:
        break;
    }
}

static void net_task(void *arg)
{

    reg_wizchip_cris_cbfunc(cris_en, cris_ex);   // 注册用于进入和退出临界区的回调函数
    reg_wizchip_cs_cbfunc(cs_sel, cs_desel);     // 注册用于选择和取消选择SPI时钟的回调函数
    reg_wizchip_spi_cbfunc(spi_read, spi_write); // 注册用于通过SPI接口读写字节的回调函数

    reset_net();

    while (getVERSIONR() != 0x04)
    {
        LOG_ERROR("W5500 version error: 0x%02x\r\n", getVERSIONR());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    net_update();

    get_common_regs();
    for (uint8_t i = 0; i < 8; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
        get_socket_regs(i);
    }

    xSemaphoreGive(xSemaphore);
    while (1)
    {

        BaseType_t taked = xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(3000));
        // for (uint8_t i = 0; i < 8; i++)
        //     udp(i, taked);
        udp(0, taked);
        tcp_server(1, taked);
    }
}

void net_init()
{
    deviceMap = *(DeviceMap *)(FLASH_ADDR);

    xSemaphore = xSemaphoreCreateBinary();
    for (int32_t i = 0; i < SOCKET_END; i++)
        xStreamBufferSnd[i] = xStreamBufferCreate(NET_SND_BUFFER_SIZE, 1);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure = {
        .GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13, // SCK, MISO, MOSI
        .GPIO_Mode = GPIO_Mode_AF_PP,                        // Alternate function push-pull
        .GPIO_Speed = GPIO_Speed_50MHz,                      // 50 MHz
    };
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    SPI_InitTypeDef SPI_InitStructure = {
        .SPI_Direction = SPI_Direction_2Lines_FullDuplex, // Bidirectional mode
        .SPI_Mode = SPI_Mode_Master,                      // Master mode
        .SPI_DataSize = SPI_DataSize_8b,                  // 8 bits per transfer
        .SPI_CPOL = SPI_CPOL_Low,                         // Clock polarity low
        .SPI_CPHA = SPI_CPHA_1Edge,                       // Clock phase first edge
        .SPI_NSS = SPI_NSS_Soft,                          // Software NSS management
        .SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2, // 36 MHz clock (72 MHz / 2)
        .SPI_FirstBit = SPI_FirstBit_MSB,                 // MSB first
        .SPI_CRCPolynomial = 7                            // CRC polynomial
    };

    SPI_Init(SPI2, &SPI_InitStructure);
    SPI_Cmd(SPI2, ENABLE);

    // 定义RES CS引脚
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_11; // 选择要控制的GPIO引脚
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;        // 设置引脚速率为50MHz
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;         // 设置引脚模式为通用推挽输出
    GPIO_Init(GPIOB, &GPIO_InitStructure);                   // 调用库函数，初始化GPIO
    GPIO_SetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_11);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_ResetBits(GPIOB, GPIO_Pin_9);

    // 定义INT引脚
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;        // 选择要控制的GPIO引脚
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 设置引脚速率为50MHz
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;     // 设置引脚模式为通用推挽模拟上拉输入
    GPIO_Init(GPIOB, &GPIO_InitStructure);            // 调用库函数，初始化GPIO

    // 配置外部中断线
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource10); // 连接PB10到EXTI10

    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line10;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // 下降沿触发（可选 Rising/Both）
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // 配置NVIC（中断控制器）
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;      // PB10使用EXTI15_10中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5; // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        // 子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    xTaskCreate(net_task, NET_TASK_NAME, NET_TASK_STACK_SIZE, NULL, NET_TASK_PRIORITY, NULL);
}

void net_snd(uint8_t sn, const void *array, size_t len)
{
    size_t len_snd = (len <= 0) ? strlen(array) : len;
    taskENTER_CRITICAL();
    xStreamBufferSend(xStreamBufferSnd[sn], array, len_snd, 0);
    xSemaphoreGive(xSemaphore);
    taskEXIT_CRITICAL();
}