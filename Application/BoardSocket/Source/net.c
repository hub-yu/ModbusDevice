#include "net.h"
#include "log.h"
#include "stm32f10x.h"

#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"
#include "semphr.h"
#include "timers.h"

#include "w5500.h"
#include "socket.h"
#include "wizchip_conf.h"
#include "dhcp.h"
#include "dns.h"
#include "httpUtil.h"
#include "MQTTClient.h"

#include "device.h"
#include "modbus.h"

static TimerHandle_t xTimer, xTimerMQTT;
static SemaphoreHandle_t xSemaphore;
static StreamBufferHandle_t xStreamBufferSnd[SOCKET_END];
static uint8_t rcv_buf[SOCKET_END][NET_RCV_BUFFER_SIZE] = {};
static size_t rcv_len[SOCKET_END] = {};
static uint64_t ms[SOCKET_END] = {};

static uint8_t html[500] = {};

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
    // taskENTER_CRITICAL();
}

static void cris_ex()
{
    // taskEXIT_CRITICAL();
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
    setSHAR(deviceMap.regs.net_mac);
    setGAR(deviceMap.regs.net_gateway);
    setSUBR(deviceMap.regs.net_mask);
    setSIPR(deviceMap.regs.net_ip);
}

static void timer_callback(TimerHandle_t xTimer)
{
    DHCP_time_handler();       // 处理DHCP超时
    DNS_time_handler();        // 处理DNS超时
    httpServer_time_handler(); // 处理httpserver超时
}
static void timerMQTT_callback(TimerHandle_t xTimerMQTT)
{
    for (int32_t i = 0; i < 10; i++)
        MilliTimer_Handler();
}

static void snd_Device(const Modbus *modbus)
{
    uint8_t data[20] = {};
    Modbus_RTU rtu = {
        .addr = modbus->addr,
        .pdu = modbus->pdu,
    };

    int32_t len = modebus_serilize_rtu(&rtu, data);
    net_rcv_override(modbus->from, data, len);
}

static int32_t dhcp(uint8_t n)
{
    static uint8_t buf[RIP_MSG_SIZE];
    static uint8_t need_init = 1;

    if (need_init)
    {
        // 关闭该通道中断
        uint8_t simr = getSIMR();
        simr &= ~(1 << n);
        setSIMR(simr);

        DHCP_init(n, buf);
        need_init = 0;
    }
    LOG_INFO("channel: %d, SR: 0x%02x\r\n", n, getSn_SR(n));

    uint8_t state = DHCP_run();
    LOG_INFO("DHCP state %d, Leasetime %d\r\n", state, getDHCPLeasetime());

    uint8_t ip[4] = {};
    uint8_t gw[4] = {};
    uint8_t sn[4] = {};
    uint8_t dns[4] = {};

    getIPfromDHCP(ip);
    getGWfromDHCP(gw);
    getSNfromDHCP(sn);
    getDNSfromDHCP(dns);

    if ((ip[0] || ip[1] || ip[2] || ip[3]) &&
        ((deviceMap.regs.net_ip[0] != ip[0]) ||
         (deviceMap.regs.net_ip[1] != ip[1]) ||
         (deviceMap.regs.net_ip[2] != ip[2]) ||
         (deviceMap.regs.net_ip[3] != ip[3])))
    {
        getIPfromDHCP(deviceMap.regs.net_ip);
        LOG_INFO("DHCP IP %d.%d.%d.%d\r\n", deviceMap.regs.net_ip[0], deviceMap.regs.net_ip[1], deviceMap.regs.net_ip[2], deviceMap.regs.net_ip[3]);

        Modbus modbus = {
            .from = 255,
            .addr = 0,
            .pdu = {
                .cmd = MODBUS_CMD_SET_REG_MULT,
                .reg = REG_IP,
                .num = 2,
                .length = 4,
                .data = {ip[0], ip[1], ip[2], ip[3]}}};

        snd_Device(&modbus);
    }

    if ((gw[0] || gw[1] || gw[2] || gw[3]) &&
        ((deviceMap.regs.net_gateway[0] != gw[0]) ||
         (deviceMap.regs.net_gateway[1] != gw[1]) ||
         (deviceMap.regs.net_gateway[2] != gw[2]) ||
         (deviceMap.regs.net_gateway[3] != gw[3])))
    {
        getGWfromDHCP(deviceMap.regs.net_gateway);
        LOG_INFO("DHCP GW %d.%d.%d.%d\r\n", deviceMap.regs.net_gateway[0], deviceMap.regs.net_gateway[1], deviceMap.regs.net_gateway[2], deviceMap.regs.net_gateway[3]);

        Modbus modbus = {
            .from = 255,
            .addr = 0,
            .pdu = {
                .cmd = MODBUS_CMD_SET_REG_MULT,
                .reg = REG_GATEWAY,
                .num = 2,
                .length = 4,
                .data = {gw[0], gw[1], gw[2], gw[3]}}};

        snd_Device(&modbus);
    }

    if ((sn[0] || sn[1] || sn[2] || sn[3]) &&
        ((deviceMap.regs.net_mask[0] != sn[0]) ||
         (deviceMap.regs.net_mask[1] != sn[1]) ||
         (deviceMap.regs.net_mask[2] != sn[2]) ||
         (deviceMap.regs.net_mask[3] != sn[3])))
    {
        getSNfromDHCP(deviceMap.regs.net_mask);
        LOG_INFO("DHCP SN %d.%d.%d.%d\r\n", deviceMap.regs.net_mask[0], deviceMap.regs.net_mask[1], deviceMap.regs.net_mask[2], deviceMap.regs.net_mask[3]);
        Modbus modbus = {
            .from = 255,
            .addr = 0,
            .pdu = {
                .cmd = MODBUS_CMD_SET_REG_MULT,
                .reg = REG_MASK,
                .num = 2,
                .length = 4,
                .data = {sn[0], sn[1], sn[2], sn[3]}}};

        snd_Device(&modbus);
    }

    if ((dns[0] || dns[1] || dns[2] || dns[3]) &&
        ((deviceMap.regs.net_dns[0] != dns[0]) ||
         (deviceMap.regs.net_dns[1] != dns[1]) ||
         (deviceMap.regs.net_dns[2] != dns[2]) ||
         (deviceMap.regs.net_dns[3] != dns[3])))
    {
        getDNSfromDHCP(deviceMap.regs.net_dns);
        LOG_INFO("DHCP DNS %d.%d.%d.%d\r\n", deviceMap.regs.net_dns[0], deviceMap.regs.net_dns[1], deviceMap.regs.net_dns[2], deviceMap.regs.net_dns[3]);

        Modbus modbus = {
            .from = 255,
            .addr = 0,
            .pdu = {
                .cmd = MODBUS_CMD_SET_REG_MULT,
                .reg = REG_DNS,
                .num = 2,
                .length = 4,
                .data = {dns[0], dns[1], dns[2], dns[3]}}};

        snd_Device(&modbus);
    }

    return state == DHCP_IP_LEASED ? 0 : 1;
}


void generate(uint8_t n, uint8_t *buf, uint16_t *len)
{

    static char *types[] = {
        "关闭",
        "UDP",
        "TCP-SERVER",
        "TCP-CLIENT",
        "MQTT",
    };

    static char *protocols[] = {
        "RTU",
        "ASCII",
        "IP协议"};

    sprintf(buf, "<html><head><meta charset='utf-8'><title>通道:%d 信息</title></head><body>"
                 "(%d)[3:0] 通讯:%s <br>"
                 "(%d)[6:4] 协议:%s<br>"
                 "(%d)[7] 目标域名:%s<br>"
                 "(%d)[15:0] 超时重连: %dms<br>"
                 "(%d)[15:0] 本地端口: %d<br>"
                 "(%d)[15:0] 目标端口: %d<br>"
                 "(%d-%d) 目标IP: %d.%d.%d.%d<br>"
                 "(%d-%d) 目标域名: %s<br>"
                 "<a href=\"/\">返回</a>"
                 "</body></html>",
            n,
            REG_SOCKET_TYPE(n),
            types[deviceMap.regs.netMap[n].type & REG_SOCKET_TYPE_STYLE],
            REG_SOCKET_TYPE(n),
            protocols[(deviceMap.regs.netMap[n].type & REG_SOCKET_TYPE_PROTOCOL) >> 4],
            REG_SOCKET_TYPE(n),
            (deviceMap.regs.netMap[n].type & REG_SOCKET_TYPE_DOMAIN ? "开" : "关"),
            REG_SOCKET_TIMEOUT(n),
            deviceMap.regs.netMap[n].timeout_ms,
            REG_SOCKET_LOCAL_PORT(n),
            deviceMap.regs.netMap[n].local_port,
            REG_SOCKET_REMOTE_PORT(n),
            deviceMap.regs.netMap[n].remote_port,
            REG_SOCKET_REMOTE_IP(n),
            REG_SOCKET_REMOTE_IP(n) + 1,
            deviceMap.regs.netMap[n].remote_ip[0], deviceMap.regs.netMap[n].remote_ip[1], deviceMap.regs.netMap[n].remote_ip[2], deviceMap.regs.netMap[n].remote_ip[3],
            REG_SOCKET_REMOTE_DOMAIN(n),
            REG_SOCKET_REMOTE_DOMAIN(n) + 15,
            deviceMap.regs.netMap[n].remote_domain

    );
    *len = strlen((char *)buf);
    // LOG_INFO("%d %d\r\n", *len, (*len + strlen(RES_CGIHEAD_OK) + 8));
}

uint8_t predefined_get_cgi_processor(uint8_t *uri_name, uint8_t *buf, uint16_t *len)
{

    if (strcmp((const char *)uri_name, "n.cgi") == 0)
    {

        uint32_t baudrate[] = {115200, 57600, 19200, 9600};

        sprintf(buf, "<html><head><meta charset='utf-8'><title>基本信息</title></head><body>"
                     "(0)[7:0] 设备地址: %d<br>"
                     "(2)[5] DHCP: %s<br>"
                     "(2)[4] 输出掉电保存: %s<br>"
                     "(2)[3] 串口日志: %s<br>"
                     "(2)[2] 串口协议: %s<br>"
                     "(2)[1:0] 串口波特率: %d<br>"
                     "<hr>"
                     "(3:5) MAC: %02x.%02x.%02x.%02x.%02x.%02x<br>"
                     "(6:7) IP: %d.%d.%d.%d<br>"
                     "(8:9) MASK: %d.%d.%d.%d<br>"
                     "(10:11) GATEWAY: %d.%d.%d.%d<br>"
                     "(12:13) DNS: %d.%d.%d.%d<br>"
                     "<a href=\"/\">返回</a>"
                     "</body></html>",
                deviceMap.regs.id,
                (deviceMap.regs.config & REG_CONFIG_DHCP ? "开" : "关"),
                (deviceMap.regs.config & REG_CONFIG_OUTKEEP ? "开" : "关"),
                (deviceMap.regs.config & REG_CONFIG_LOG ? "开" : "关"),
                (deviceMap.regs.config & REG_CONFIG_PROTOCOL ? "ASCII" : "RTU"),
                baudrate[(deviceMap.regs.config & REG_CONFIG_BAUDRATE)],
                deviceMap.regs.net_mac[0], deviceMap.regs.net_mac[1], deviceMap.regs.net_mac[2], deviceMap.regs.net_mac[3], deviceMap.regs.net_mac[4], deviceMap.regs.net_mac[5],
                deviceMap.regs.net_ip[0], deviceMap.regs.net_ip[1], deviceMap.regs.net_ip[2], deviceMap.regs.net_ip[3],
                deviceMap.regs.net_mask[0], deviceMap.regs.net_mask[1], deviceMap.regs.net_mask[2], deviceMap.regs.net_mask[3],
                deviceMap.regs.net_gateway[0], deviceMap.regs.net_gateway[1], deviceMap.regs.net_gateway[2], deviceMap.regs.net_gateway[3],
                deviceMap.regs.net_dns[0], deviceMap.regs.net_dns[1], deviceMap.regs.net_dns[2], deviceMap.regs.net_dns[3]);
        *len = strlen((char *)buf);
        // LOG_INFO("%d\r\n", (*len + strlen(RES_CGIHEAD_OK) + 8));
        return HTTP_OK;
    }
    else if (strcmp((const char *)uri_name, "0.cgi") == 0)
    {
        generate(0, buf, len);
        return HTTP_OK;
    }
    else if (strcmp((const char *)uri_name, "1.cgi") == 0)
    {
        generate(1, buf, len);
        return HTTP_OK;
    }
    else if (strcmp((const char *)uri_name, "2.cgi") == 0)
    {
        generate(2, buf, len);
        return HTTP_OK;
    }
    else if (strcmp((const char *)uri_name, "3.cgi") == 0)
    {
        generate(3, buf, len);
        return HTTP_OK;
    }
    else if (strcmp((const char *)uri_name, "4.cgi") == 0)
    {
        generate(4, buf, len);
        return HTTP_OK;
    }
    else if (strcmp((const char *)uri_name, "5.cgi") == 0)
    {
        generate(5, buf, len);
        return HTTP_OK;
    }
    return HTTP_FAILED;
}

uint8_t predefined_set_cgi_processor(uint8_t *uri_name, uint8_t *uri, uint8_t *buf, uint16_t *len)
{
    // 复位
    if (strcmp((const char *)uri_name, "rs.cgi") == 0)
    {
        Modbus modbus = {
            .from = 255,
            .addr = 0,
            .pdu = {
                .cmd = MODBUS_CMD_SET_REG,
                .reg = 1,
                .num = 1}};
        snd_Device(&modbus);

        sprintf(buf, "<html><head><meta charset='utf-8'><title>设备重启</title></head><body>"
                     "<p>%s</p>"
                     "<a href=\"/\">返回</a>"
                     "</body></html>",
                "重启完成");
        *len = strlen((char *)buf);

        return HTTP_OK;
    }
    // 恢复出厂
    else if (strcmp((const char *)uri_name, "rst.cgi") == 0)
    {
        Modbus modbus = {
            .from = 255,
            .addr = 0,
            .pdu = {
                .cmd = MODBUS_CMD_SET_REG,
                .reg = 1,
                .num = 2}};
        snd_Device(&modbus);

        sprintf(buf, "<html><head><meta charset='utf-8'><title>设备恢复出厂</title></head><body>"
                     "<p>%s</p>"
                     "<a href=\"/\">返回</a>"
                     "</body></html>",
                "恢复出厂完成");
        *len = strlen((char *)buf);

        return HTTP_OK;
    }

    return HTTP_FAILED;
}

static int32_t web()
{
    static uint8_t tx[520] = {};
    static uint8_t rx[520] = {};

    static uint8_t n = SOCKET_CHANNEL_6;
    static uint8_t need_init = 1;
    if (need_init)
    {
        // 关闭该通道中断
        uint8_t simr = getSIMR();
        simr &= ~(1 << n);
        setSIMR(simr);
        httpServer_init(tx, rx, 1, &n);
        need_init = 0;
    }
    // if (getSn_IR(n) & Sn_IR_RECV)
    //     setSn_IR(n, Sn_IR_RECV);

    httpServer_run(0);
    get_httpServer_timecount();
}

void messageArrived_0(MessageData *md)
{
    MQTTMessage *message = md->message;
    net_rcv_override(0, message->payload, message->payloadlen);
}

void messageArrived_1(MessageData *md)
{
    MQTTMessage *message = md->message;
    net_rcv_override(1, message->payload, message->payloadlen);
}

void messageArrived_2(MessageData *md)
{
    MQTTMessage *message = md->message;
    net_rcv_override(2, message->payload, message->payloadlen);
}

void messageArrived_3(MessageData *md)
{
    MQTTMessage *message = md->message;
    net_rcv_override(3, message->payload, message->payloadlen);
}

void messageArrived_4(MessageData *md)
{
    MQTTMessage *message = md->message;
    net_rcv_override(4, message->payload, message->payloadlen);
}

void messageArrived_5(MessageData *md)
{
    MQTTMessage *message = md->message;
    net_rcv_override(5, message->payload, message->payloadlen);
}

// void messageArrived_6(MessageData *md)
// {
//     MQTTMessage *message = md->message;
//     net_rcv_override(6, message->payload, message->payloadlen);
// }

// void messageArrived_7(MessageData *md)
// {
//     MQTTMessage *message = md->message;
//     net_rcv_override(7, message->payload, message->payloadlen);
// }

static messageHandler messageArrived[SOCKET_END] = {
    messageArrived_0,
    messageArrived_1,
    messageArrived_2,
    messageArrived_3,
    messageArrived_4,
    messageArrived_5,
    // messageArrived_6,
    // messageArrived_7,
};

static int32_t mqtt(uint8_t n)
{
    static Network network[SOCKET_END];
    static MQTTClient client[SOCKET_END];
    static unsigned char buf[200] = {};
    static unsigned char rcv[200] = {};
    static uint8_t topic[30] = {};
    static uint8_t topicAck[30] = {};

    if (topic[0] == 0)
        sprintf(topic, "device/%d", deviceMap.regs.id);
    if (topicAck[0] == 0)
        sprintf(topicAck, "device/ack/%d", deviceMap.regs.id);

    uint8_t snd[NET_SND_BUFFER_SIZE];
    size_t len_snd = xStreamBufferReceive(xStreamBufferSnd[n], snd, NET_SND_BUFFER_SIZE, 0);

    LOG_INFO("[%d] mqtt, SR: 0x%02x\r\n", n, getSn_SR(n));

    switch (getSn_SR(n))
    {

    case SOCK_ESTABLISHED:
    {
        if (getSn_IR(n) & Sn_IR_RECV)
            setSn_IR(n, Sn_IR_RECV);

        if (!client[n].isconnected)
        {
            uint8_t client_id[30] = {};
            sprintf((char *)client_id, "device_%02x-%02x-%02x-%02x-%02x-%02x",
                    deviceMap.regs.net_mac[0],
                    deviceMap.regs.net_mac[1],
                    deviceMap.regs.net_mac[2],
                    deviceMap.regs.net_mac[3],
                    deviceMap.regs.net_mac[4],
                    deviceMap.regs.net_mac[5]);

            MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
            data.willFlag = 0;
            data.MQTTVersion = 3;
            data.clientID.cstring = client_id;
            data.username.cstring = NULL;
            data.password.cstring = NULL;
            data.keepAliveInterval = 60;
            data.cleansession = 1;
            int32_t rc = MQTTConnect(&client[n], &data);
            // LOG_INFO("MQTTConnect %d\r\n", rc);
            // static uint8_t topic[30] = {};
            // sprintf(topic, "device/%d", deviceMap.regs.id);
            rc = MQTTSubscribe(&client[n], topic, QOS0, messageArrived[n]);
            LOG_INFO("[%d] topic: %s  ret: %d\r\n", n, topic, rc);
        }

        if (len_snd > 0)
        {
            // LOG_INFO("[%d] send: %d\r\n", n, len_snd);
            MQTTMessage message = {
                .qos = QOS0,
                .retained = 0,
                .dup = 0,
                .id = 0,
                .payload = snd,
                .payloadlen = len_snd};
            // static uint8_t topic[30] = {};
            // sprintf(topic, "device/ack/%d", deviceMap.regs.id);
            // LOG_INFO("[%d] topic: %s  publish: %d\r\n", n, topic,
            MQTTPublish(&client[n], topicAck, &message);
            // );
        }

        MQTTYield(&client[n], 60);
        // LOG_INFO("MQTTYield %d\r\n", );
    }

    break;
    case SOCK_CLOSE_WAIT:
        if (getSn_IR(n) & Sn_IR_DISCON)
            setSn_IR(n, Sn_IR_DISCON);
        disconnect(n);
        break;

    case SOCK_INIT:
        connect(n, deviceMap.regs.netMap[n].remote_ip, deviceMap.regs.netMap[n].remote_port);
        break;

    case SOCK_SYNSENT:
        break;

    case SOCK_CLOSED:
        close(n);
        NewNetwork(&network[n], n);
        MQTTClientInit(&client[n], &network[n], 5000, buf, 200, rcv, 200);
        socket(n, Sn_MR_TCP, deviceMap.regs.netMap[n].local_port, 0x00);
        break;

    default:
        break;
    }
}

static void udp(uint8_t n, uint8_t *rcv_buf, size_t *rcv_len)
{

    LOG_INFO("[%d] udp, SR: 0x%02x\r\n", n, getSn_SR(n));

    uint8_t snd[NET_SND_BUFFER_SIZE];
    size_t len_snd = xStreamBufferReceive(xStreamBufferSnd[n], snd, NET_SND_BUFFER_SIZE, 0);

    switch (getSn_SR(n))
    {

    case SOCK_UDP:
    {

        if (len_snd > 0)
            sendto(n, snd, len_snd, deviceMap.regs.netMap[n].remote_ip, deviceMap.regs.netMap[n].remote_port);

        if (getSn_IR(n) & Sn_IR_RECV)
            setSn_IR(n, Sn_IR_RECV);

        do
        {
            size_t len_used = *rcv_len ? net_rcv_override(n, rcv_buf, *rcv_len) : 0;
            if (len_used)
            {
                for (size_t i = 0; i < (*rcv_len - len_used); i++)
                    rcv_buf[i] = rcv_buf[i + len_used];

                *rcv_len -= len_used;
                continue;
            }

            int32_t ret = getSn_RX_RSR(n);
            if (ret <= 0)
                break;
            uint16_t max_size = NET_RCV_BUFFER_SIZE - *rcv_len;
            int32_t len_rcv = recvfrom(n, rcv_buf + *rcv_len, (ret >= max_size ? max_size : ret), deviceMap.regs.netMap[n].remote_ip, &deviceMap.regs.netMap[n].remote_port);
            *rcv_len += len_rcv;

        } while (1);
    }
    break;

    case SOCK_CLOSED:
        socket(n, Sn_MR_UDP, deviceMap.regs.netMap[n].local_port, 0x00);
        // xSemaphoreGive(xSemaphore);
        break;

    default:
        break;
    }
}

static void tcp_server(uint8_t n, uint8_t *rcv_buf, size_t *rcv_len, uint64_t *ms)
{
    uint8_t snd[NET_SND_BUFFER_SIZE];
    size_t len_snd = xStreamBufferReceive(xStreamBufferSnd[n], snd, NET_SND_BUFFER_SIZE, 0);

    LOG_INFO("[%d] server, SR: 0x%02x\r\n", n, getSn_SR(n));

    switch (getSn_SR(n))
    {

    case SOCK_ESTABLISHED:

        if (len_snd > 0)
            send(n, snd, len_snd);

        if (getSn_IR(n) & Sn_IR_CON)
            setSn_IR(n, Sn_IR_CON);

        if (getSn_IR(n) & Sn_IR_RECV)
            setSn_IR(n, Sn_IR_RECV);

        do
        {
            size_t len_used = *rcv_len ? net_rcv_override(n, rcv_buf, *rcv_len) : 0;
            if (len_used)
            {
                for (size_t i = 0; i < (*rcv_len - len_used); i++)
                    rcv_buf[i] = rcv_buf[i + len_used];

                *rcv_len -= len_used;
                continue;
            }

            int32_t ret = getSn_RX_RSR(n);
            if (ret <= 0)
            {
                if (pdTICKS_TO_MS(xTaskGetTickCount()) >= (*ms + deviceMap.regs.netMap[n].timeout_ms))
                {
                    disconnect(n);
                    *rcv_len = 0;
                }
                break;
            }
            uint16_t max_size = NET_RCV_BUFFER_SIZE - *rcv_len;
            int32_t len_rcv = recv(n, rcv_buf + *rcv_len, (ret >= max_size ? max_size : ret));
            *rcv_len += len_rcv;

            *ms = pdTICKS_TO_MS(xTaskGetTickCount());

        } while (1);

        break;
    case SOCK_CLOSE_WAIT:
        if (getSn_IR(n) & Sn_IR_DISCON)
            setSn_IR(n, Sn_IR_DISCON);
        disconnect(n);
        break;

    case SOCK_INIT:
        listen(n);
        // xSemaphoreGive(xSemaphore);
        break;
    case SOCK_LISTEN:
        *ms = pdTICKS_TO_MS(xTaskGetTickCount());
        break;

    case SOCK_CLOSED:
        socket(n, Sn_MR_TCP, deviceMap.regs.netMap[n].local_port, 0x00);
        // xSemaphoreGive(xSemaphore);
        break;

    default:
        break;
    }
}

static void tcp_client(uint8_t n, uint8_t *rcv_buf, size_t *rcv_len, uint64_t *ms)
{
    uint8_t snd[NET_SND_BUFFER_SIZE];
    size_t len_snd = xStreamBufferReceive(xStreamBufferSnd[n], snd, NET_SND_BUFFER_SIZE, 0);

    LOG_INFO("[%d] client, SR: 0x%02x\r\n", n, getSn_SR(n));

    switch (getSn_SR(n))
    {

    case SOCK_ESTABLISHED:

        if (len_snd > 0)
            send(n, snd, len_snd);

        if (getSn_IR(n) & Sn_IR_CON)
            setSn_IR(n, Sn_IR_CON);

        if (getSn_IR(n) & Sn_IR_RECV)
            setSn_IR(n, Sn_IR_RECV);

        do
        {
            size_t len_used = *rcv_len ? net_rcv_override(n, rcv_buf, *rcv_len) : 0;
            if (len_used)
            {
                for (size_t i = 0; i < (*rcv_len - len_used); i++)
                    rcv_buf[i] = rcv_buf[i + len_used];

                *rcv_len -= len_used;
                continue;
            }

            int32_t ret = getSn_RX_RSR(n);
            if (ret <= 0)
            {
                if (pdTICKS_TO_MS(xTaskGetTickCount()) >= (*ms + deviceMap.regs.netMap[n].timeout_ms))
                {
                    disconnect(n);
                    // close(n);
                    *rcv_len = 0;
                }
                break;
            }
            uint16_t max_size = NET_RCV_BUFFER_SIZE - *rcv_len;
            int32_t len_rcv = recv(n, rcv_buf + *rcv_len, (ret >= max_size ? max_size : ret));
            *rcv_len += len_rcv;

            *ms = pdTICKS_TO_MS(xTaskGetTickCount());
        } while (1);

        break;
    case SOCK_CLOSE_WAIT:
        if (getSn_IR(n) & Sn_IR_DISCON)
            setSn_IR(n, Sn_IR_DISCON);
        disconnect(n);
        break;

    case SOCK_INIT:
        *ms = pdTICKS_TO_MS(xTaskGetTickCount());
        connect(n, deviceMap.regs.netMap[n].remote_ip, deviceMap.regs.netMap[n].remote_port);
        // xSemaphoreGive(xSemaphore);
        break;

    case SOCK_SYNSENT:
        *ms = pdTICKS_TO_MS(xTaskGetTickCount());
        break;

    case SOCK_CLOSED:
        close(n);
        socket(n, Sn_MR_TCP, deviceMap.regs.netMap[n].local_port, 0x00);
        // xSemaphoreGive(xSemaphore);
        break;

    default:
        break;
    }
}

static void dns_domain(uint8_t n)
{
    LOG_INFO("[%d] dns domain \r\n", n);
    // 关闭该通道中断
    uint8_t simr = getSIMR();
    simr &= ~(1 << n);
    setSIMR(simr);
    // 0xf7
    uint8_t buf[MAX_DNS_BUF_SIZE];
    DNS_init(n, buf);
    int8_t ret = DNS_run(deviceMap.regs.net_dns, deviceMap.regs.netMap[n].remote_domain, deviceMap.regs.netMap[n].remote_ip);
    if (ret == 1)
    {
        // 开启该通道中断
        uint8_t simr = getSIMR();
        simr |= (1 << n);
        setSIMR(simr);
        LOG_INFO("DNS ret %d, %s IP %d.%d.%d.%d\r\n", ret, deviceMap.regs.netMap[n].remote_domain, deviceMap.regs.netMap[n].remote_ip[0], deviceMap.regs.netMap[n].remote_ip[1], deviceMap.regs.netMap[n].remote_ip[2], deviceMap.regs.netMap[n].remote_ip[3]);
        deviceMap.regs.netMap[n].type &= ~REG_SOCKET_TYPE_DOMAIN;
    }
}

static void sokit(int32_t n, uint8_t *rcv_buf, size_t *rcv_len, uint64_t *ms)
{
    if (deviceMap.regs.netMap[n].type & REG_SOCKET_TYPE_DOMAIN)
    {
        dns_domain(n);
        return;
    }

    switch (deviceMap.regs.netMap[n].type & REG_SOCKET_TYPE_STYLE)
    {
    case REG_SOCKET_TYPE_STYLE_UDP:
        udp(n, rcv_buf, rcv_len);
        break;
    case REG_SOCKET_TYPE_STYLE_TCPSERVER:
        tcp_server(n, rcv_buf, rcv_len, ms);
        break;
    case REG_SOCKET_TYPE_STYLE_TCPCLIENT:
        tcp_client(n, rcv_buf, rcv_len, ms);
        break;
    case REG_SOCKET_TYPE_STYLE_MQTT:
        mqtt(n);
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

    setSIMR(0xff);
    // setRTR(1000);
    // setRCR(2);
    // When RTR = 2000(0x07D0), RCR = 8(0x0008),
    // ARPTO = 2000 X 0.1ms X 9 = 1800ms = 1.8s
    // TCPTO = (0x07D0+0x0FA0+0x1F40+0x3E80+0x7D00+0xFA00+0xFA00+0xFA00+0xFA00) X 0.1ms
    //  = (2000 + 4000 + 8000 + 16000 + 32000 + ((8 - 4) X 64000)) X 0.1ms
    //  = 318000 X 0.1ms = 31.8s
    // get_common_regs();
    for (uint8_t i = 0; i < SOCKET_END; i++)
    {
        setSn_IMR(i, Sn_IR_RECV); // | Sn_IR_DISCON | Sn_IR_CON);
        // setSn_KPALVTR(i, 1);
        // vTaskDelay(pdMS_TO_TICKS(100));
        // get_socket_regs(i);
    }
    xTimerStart(xTimer, 0);
    xTimerStart(xTimerMQTT, 0);

    xSemaphoreGive(xSemaphore);

    while (1)
    {

        volatile UBaseType_t uxHighWaterMark; // 70
        uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        
        if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(2000)) != pdTRUE)
            memset(rcv_len, 0, SOCKET_END); // 清空接收缓冲区

        if ((deviceMap.regs.config & REG_CONFIG_DHCP) && dhcp(SOCKET_CHANNEL_7))
            continue;

        web();
        for (int32_t i = 0; i < SOCKET_CHANNEL_6; i++)
            sokit(i, rcv_buf[i], rcv_len + i, ms + i);
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

    xTimer = xTimerCreate("dhcp_timer", pdMS_TO_TICKS(1000), pdTRUE, NULL, timer_callback);
    xTimerMQTT = xTimerCreate("mqtt_timer", pdMS_TO_TICKS(10), pdTRUE, NULL, timerMQTT_callback);

    sprintf(html, "<html><head><meta charset='utf-8'><title>地址 %d</title></head><body>"
                  "<form action='/rst.cgi' method='POST'>"
                  "<input type='submit' value='恢复出厂'></form>"
                  "<form action='/rs.cgi' method='POST'>"
                  "<input type='submit' value='重启'></form>"
                  "<a href='/n.cgi'>基本信息</a><br>"
                  "<a href='/0.cgi'>0</a><br>"
                  "<a href='/1.cgi'>1</a><br>"
                  "<a href='/2.cgi'>2</a><br>"
                  "<a href='/3.cgi'>3</a><br>"
                  "<a href='/4.cgi'>4</a><br>"
                  "<a href='/5.cgi'>5</a><br>"
                  "<a href='https://xiaopj.com:444'>帮助</a><br>"
                  //   "<a href='/'>刷新</a>"
                  "</body></html>",
            deviceMap.regs.id);

    // LOG_INFO("html len:%d\r\n", strlen(html));
    reg_httpServer_webContent("index.html", html);
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