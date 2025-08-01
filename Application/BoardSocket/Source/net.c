#include "net.h"
#include "log.h"
#include "stm32f10x.h"

#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"

#include "w5500.h"
#include "socket.h"
#include "wizchip_conf.h"

#include <string.h>

void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line10) != RESET)
    {
        // 此处添加中断处理代码（例如翻转LED、发送信号等）
        GPIO_SetBits(GPIOB, GPIO_Pin_9);
        // 清除中断标志位（重要！）
        EXTI_ClearITPendingBit(EXTI_Line10);
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

void reset_net(void)
{
    GPIO_ResetBits(GPIOB, GPIO_Pin_11); // min 500us
    vTaskDelay(pdMS_TO_TICKS(10));
    GPIO_SetBits(GPIOB, GPIO_Pin_11); // max 1ms
    vTaskDelay(pdMS_TO_TICKS(10));

    setPHYCFGR(0xb8); // reset PHY
    vTaskDelay(pdMS_TO_TICKS(10));
}

void get_common_regs()
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
    LOG_INFO("SIMR: 0x%02x\r\n", getSIMR());         // socket中断掩码
    LOG_INFO("RTR: 0x%04x\r\n", getRTR());           // 重发延时 100us × RTR
    LOG_INFO("RCR: 0x%02x\r\n", getRCR());           // 重发次数 RCR + 1

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

void get_socket_regs(uint8_t ch)
{
    LOG_INFO("ch[%d] ##############\r\n", ch);
    LOG_INFO("ch[%d] MR: 0x%02x\r\n", ch, getSn_MR(ch));   // Socket n Mode Register
    LOG_INFO("ch[%d] CR: 0x%02x\r\n", ch, getSn_CR(ch));   // Socket n Command Register
    LOG_INFO("ch[%d] IR: 0x%02x\r\n", ch, getSn_IR(ch));   // Socket n Interrupt Register
    LOG_INFO("ch[%d] SR: 0x%02x\r\n", ch, getSn_SR(ch));   // Socket n Status Register
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

void net_task(void *arg)
{

    reg_wizchip_cris_cbfunc(cris_en, cris_ex);   // 注册用于进入和退出临界区的回调函数
    reg_wizchip_cs_cbfunc(cs_sel, cs_desel);     // 注册用于选择和取消选择SPI时钟的回调函数
    reg_wizchip_spi_cbfunc(spi_read, spi_write); // 注册用于通过SPI接口读写字节的回调函数

    reset_net();

    if(getVERSIONR() != 0x04) {
        LOG_ERROR("W5500 version error: 0x%02x\r\n", getVERSIONR());
        return;
    }

    uint8_t mac[6] = {0x00, 0x22, 0x33, 0x44, 0x55, 0x66};
    uint8_t gateway[4] = {192, 168, 1, 1};
    uint8_t mask[4] = {255, 255, 255, 0};
    uint8_t ip[4] = {192, 168, 1, 49};
    setGAR(gateway);
    setSUBR(mask);
    setSHAR(mac);
    setSIPR(ip);
    get_common_regs();
    for(uint8_t i = 0; i < 1; i++) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        get_socket_regs(i);
    }

    uint8_t remote_ip[4] = {192, 168, 1, 4};

    LOG_INFO("socket %d\r\n", socket(1, Sn_MR_TCP, 0, Sn_MR_ND));
    LOG_INFO("connect %d\r\n", connect(1, remote_ip, 5000));

    //   switch (getSn_SR(0))
    //         {
    //         case SOCK_CLOSED:
    //             socket(0, Sn_MR_TCP, 5000, Sn_MR_ND);
    //             break;

    //         case SOCK_INIT:
    //             connect(0, remote_ip, remote_port);
    //             break;

    //         case SOCK_ESTABLISHED:
    //             if (getSn_IR(0) & Sn_IR_CON)
    //             {
    //                 setSn_IR(0, Sn_IR_CON);
    //             }

    //             len = getSn_RX_RSR(0);
    //             if (len > 0)
    //             {
    //                 recv(0, buff, len);
    //                 buff[len] = 0x00;
    //                 printf("%s\r\n", buff);
    //                 send(0, buff, len);
    //             }
    //             break;

    //         case SOCK_CLOSE_WAIT:
    //             close(0);
    //             break;
    //         }

    uint8_t buff[100];

    while (1)
    {
        // LOG_INFO("PHYCFGR: 0x%02x\r\n", getPHYCFGR());
        // get_socket_regs(0);

        // GPIO_ResetBits(GPIOA, GPIO_Pin_8);
        // vTaskDelay(pdMS_TO_TICKS(500));

        // GPIO_SetBits(GPIOA, GPIO_Pin_8);
        vTaskDelay(pdMS_TO_TICKS(100));

        if (getSn_IR(1) & Sn_IR_CON)
            setSn_IR(1, Sn_IR_CON);

        uint16_t len = getSn_RX_RSR(1);
        if(len == 0)
            continue;
        memset(buff, 0, sizeof(buff));
        recv(1, buff, len);
        LOG_INFO("%s\r\n", buff);
        send(1, buff, len);
    }
}

void net_init()
{

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

    // spi_init();

    // 配置外部中断线
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource10); // 连接PB10到EXTI10

    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line10;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // 下降沿触发（可选 Rising/Both）
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // 4. 配置NVIC（中断控制器）
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;      // PB10使用EXTI15_10中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5; // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        // 子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // NVIC_EnableIRQ(IRQn_Type IRQn);

    xTaskCreate(net_task, NET_TASK_NAME, NET_TASK_STACK_SIZE, NULL, NET_TASK_PRIORITY, NULL);
}