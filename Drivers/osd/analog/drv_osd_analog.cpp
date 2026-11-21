#include "drv_osd_analog.hpp"
#include "Basic.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "FreeRTOS.h"
#include "HardWareConfig.hpp"
#include "MeasurementSystem.hpp"
#include "Modes.hpp"
#include "StorageSystem.hpp"
#include "task.h"

#include "drv_spi.hpp"

#include <stdio.h>
#include <string.h>

// =====================================================================
// SPI 及 CS 引脚（运行时从硬件配置读取）
// =====================================================================
#ifdef DCACHE_SIZE
#define TX_BUF_SIZE DCACHE_SIZE
#define RX_BUF_SIZE TX_BUF_SIZE
Aligned_DMABuf uint8_t tx_buf[TX_BUF_SIZE];
uint8_t *rx_buf = tx_buf;
#else
#define TX_BUF_SIZE 32
#define RX_BUF_SIZE TX_BUF_SIZE
uint8_t tx_buf[TX_BUF_SIZE];
uint8_t *rx_buf = tx_buf;
#endif

static SPI_CLASS *s_spi = nullptr;
static GPIO_TypeDef *s_cs_gpio = nullptr;
static uint16_t s_cs_pin = 0;

static inline void pu_CS()
{
    s_cs_gpio->BSRR = 1u << (s_cs_pin + 0);
}
static inline void pd_CS()
{
    s_cs_gpio->BSRR = 1u << (s_cs_pin + 16);
}

// 从硬件配置初始化 SPI 实例和 CS 引脚
// 返回 false 表示配置不存在或硬件未使能
static bool osd_spi_init(void)
{
    HW_ConfigEntry entry;

    // 读取 SPI 编号，若不存在则不初始化
    if (!HW_ReadConfigEntry(HW_DEV_OSD_ANALOG, HW_CFG_SPI_REF, &entry))
        return false;
    s_spi = get_spi_instance(entry.value_low);
    if (!s_spi)
        return false;

    // 读取 CS 引脚 (HW_CFG_PIN1 = cs_pin)
    if (!HW_ReadConfigEntry(HW_DEV_OSD_ANALOG, HW_CFG_PIN1, &entry))
        return false;
    s_cs_gpio = HW_GpioTable[entry.value_high];
    s_cs_pin = entry.value_low;

    // 使能对应 GPIO 时钟并配置 CS 为推挽输出
    // AHB4ENR: bit0~7 对应 GPIOA~GPIOH
    RCC->AHB4ENR |= (1u << entry.value_high);
    os_delay(1e-2);
    set_register(s_cs_gpio->MODER, 0b01, 2 * s_cs_pin, 2);
    set_register(s_cs_gpio->OTYPER, 0, 1 * s_cs_pin, 1);
    set_register(s_cs_gpio->OSPEEDR, 2, 2 * s_cs_pin, 2);
    pu_CS();

    return true;
}

// =====================================================================
// AT7456E SPI 底层读写
// =====================================================================

static void write_at7456_addr_data(unsigned char addr, unsigned char dat)
{
    tx_buf[0] = addr;
    tx_buf[1] = dat;
    pd_CS();
    s_spi->write(tx_buf, 2, TX_BUF_SIZE);
    pu_CS();
}

static unsigned char read_at7456_addr_data(unsigned char addr)
{
    addr |= 0x80;
    tx_buf[0] = addr;
    tx_buf[1] = 0xff;
    pd_CS();
    s_spi->writeNread(tx_buf, 2, TX_BUF_SIZE, rx_buf, RX_BUF_SIZE);
    pu_CS();
    return rx_buf[1];
}

static void write_at7456_data(unsigned char dat)
{
    tx_buf[0] = dat;
    pd_CS();
    s_spi->write(tx_buf, 1, TX_BUF_SIZE);
    pu_CS();
}

static unsigned char read_at7456_data(void)
{
    tx_buf[0] = 0xff;
    pd_CS();
    s_spi->writeNread(tx_buf, 1, TX_BUF_SIZE, rx_buf, RX_BUF_SIZE);
    pu_CS();
    return rx_buf[0];
}

// =====================================================================
// AT7456E 功能实现
// =====================================================================

static VER7456 newAt7456;

static void wait_memory_wr_ok(void)
{
    while ((read_at7456_addr_data(STAT) & (1 << 5)) != 0)
        os_delay(0.001);
    while ((read_at7456_addr_data(DMM) & (1 << 2)) != 0)
        os_delay(0.001);
}

static void at7456_reset(void)
{
    write_at7456_addr_data(VM0, 2);
    os_delay(0.1);
    read_at7456_addr_data(STAT);
    read_at7456_addr_data(VM0);
}

static void turnOff_osd(void)
{
    unsigned char k;
    k = read_at7456_addr_data(VM0);
    write_at7456_addr_data(VM0, k & ~(1 << 3));
    os_delay(0.001);
}

static void turnOn_osd(void)
{
    unsigned char k;
    k = read_at7456_addr_data(VM0);
    write_at7456_addr_data(VM0, k | (1 << 3));
    os_delay(0.001);
}

static void clear_screen(void)
{
    unsigned int i;
    write_at7456_addr_data(DMAH, 0x00);
    write_at7456_addr_data(DMAL, 0);
    write_at7456_addr_data(DMM, 0x01);
    for (i = 0; i < (16 * 30); i++)
        write_at7456_data(0);
    write_at7456_data(0xff);
}

static VER7456 checkAT7456(void)
{
    unsigned char r1, r2;
    r1 = read_at7456_addr_data(VM0);
    r2 = (r1 & ~(1 << 1)) | 0x88;
    write_at7456_addr_data(VM0, r2);
    os_delay(0.001);
    r2 = read_at7456_addr_data(VM0) & 0x88;
    write_at7456_addr_data(VM0, r1);
    os_delay(0.001);
    if (r2 == 0x88)
        return NEW7456;
    else if (r2 == 0x08)
        return OLD7456;
    else
        return BAD7456;
}

static void configOSDBL(unsigned char autoLevelCtrl)
{
    unsigned char k;
    k = read_at7456_addr_data(OSDBL);
    k |= (1 << 4);
    write_at7456_addr_data(OSDBL, k);
    read_at7456_addr_data(OSDBL);
}

static void write_at7456_font(unsigned char addr, unsigned char *dt)
{
    unsigned char n;
    write_at7456_addr_data(CMAH, addr);
    for (n = 0; n < 54; n++)
    {
        write_at7456_addr_data(CMAL, n);
        write_at7456_addr_data(CMDI, *dt++);
    }
    write_at7456_addr_data(CMM, RAM_NVM);
    wait_memory_wr_ok();
}

static void read_at7456_font(unsigned char addr, unsigned char *dt)
{
    unsigned char n;
    write_at7456_addr_data(CMAH, addr);
    write_at7456_addr_data(CMM, NVM_RAM);
    os_delay(0.001);
    for (n = 0; n < 54; n++)
    {
        write_at7456_addr_data(CMAL, n);
        *dt++ = read_at7456_addr_data(CMDO);
    }
}

#define AUTO_INC_DIS 0

static void write_at7456E_font(unsigned short addr, const unsigned char *dt)
{
    unsigned char addr_h, n;
    addr_h = (addr >> 8);
    write_at7456_addr_data(CMAH, addr);
    if (AUTO_INC_DIS)
        write_at7456_addr_data(DMM, 0);
    else
    {
        write_at7456_addr_data(CMAL, (addr_h << 6));
        write_at7456_addr_data(DMM, 0x80);
    }
    for (n = 0; n < 54; n++)
    {
        if (AUTO_INC_DIS)
        {
            write_at7456_addr_data(CMAL, n | (addr_h << 6));
            write_at7456_addr_data(CMDI, *dt++);
        }
        else
            write_at7456_data(*dt++);
    }
    write_at7456_addr_data(CMM, RAM_NVM);
    wait_memory_wr_ok();
    read_at7456_addr_data(STAT);
}

static void read_at7456E_font(unsigned short addr, unsigned char *dt)
{
    unsigned char addr_h, n;
    addr_h = (addr >> 8);
    write_at7456_addr_data(CMAH, addr);
    write_at7456_addr_data(CMAL, (addr_h << 6));
    write_at7456_addr_data(CMM, NVM_RAM);
    os_delay(0.001);
    for (n = 0; n < 54; n++)
    {
        write_at7456_addr_data(CMAL, n);
        *dt++ = read_at7456_addr_data(CMDO);
    }
}

static void at7456_read_char(unsigned char row, unsigned char col, CHAR_ATTRIB *ca)
{
    unsigned short k;
    k = row * 30 + col;
    write_at7456_addr_data(DMM, (0 << 6));
    write_at7456_addr_data(DMAH, k >> 8);
    write_at7456_addr_data(DMAL, k);
    ca->index = read_at7456_addr_data(DMDO);
    ca->attrib = read_at7456_data() >> 4;
}

static void at7456_write_char(unsigned char row, unsigned char col, CHAR_ATTRIB *ca)
{
    unsigned short k;
    write_at7456_addr_data(DMM, ca->attrib << 3);
    k = row * 30 + col;
    write_at7456_addr_data(DMAH, k >> 8);
    write_at7456_addr_data(DMAL, k);
    write_at7456_addr_data(DMDI, ca->index);
}

static void at7456E_read_char(unsigned char row, unsigned char col, CHAR_ATTRIB *ca)
{
    unsigned short k;
    unsigned char addr_h, j;
    k = row * 30 + col;
    addr_h = k >> 8;
    write_at7456_addr_data(DMM, (1 << 6));
    write_at7456_addr_data(DMAH, addr_h);
    write_at7456_addr_data(DMAL, k);
    ca->index = read_at7456_addr_data(DMDO);
    write_at7456_addr_data(DMAH, addr_h | 2);
    write_at7456_addr_data(DMAL, k);
    j = read_at7456_addr_data(DMDO);
    ca->attrib = j >> 5;
    if (j & (1 << 4))
        ca->index |= 0x100;
}

static void at7456E_write_char(unsigned char row, unsigned char col, CHAR_ATTRIB *ca)
{
    unsigned short k;
    unsigned char addr_h, j;
    k = row * 30 + col;
    addr_h = k >> 8;
    write_at7456_addr_data(DMM, (1 << 6));
    write_at7456_addr_data(DMAH, addr_h | 0x2);
    write_at7456_addr_data(DMAL, k);
    j = ca->attrib << 5;
    if ((ca->index >> 8) != 0)
        j |= 0x10;
    write_at7456_addr_data(DMDI, j);
    write_at7456_addr_data(DMAH, addr_h);
    write_at7456_addr_data(DMAL, k);
    write_at7456_addr_data(DMDI, ca->index);
}

static void DisplayChar(unsigned char row, unsigned char col, unsigned char c)
{
    unsigned int kk;
    kk = row * 30 + col;
    write_at7456_addr_data(DMAH, kk / 256);
    write_at7456_addr_data(DMAL, kk % 256);
    write_at7456_addr_data(DMDI, c);
}

static void DisplayNum(unsigned char row, unsigned char col, unsigned char c)
{
    unsigned int kk;
    kk = row * 30 + col;
    write_at7456_addr_data(DMAH, kk / 256);
    write_at7456_addr_data(DMAL, kk % 256);
    write_at7456_addr_data(DMDI, (c == 0) ? 10 : c);
}

static void DisplayString(unsigned char row, unsigned char col, unsigned char *s)
{
    unsigned char c;
    unsigned char d_col;
    CHAR_ATTRIB ca;

    ca.attrib = 0;
    d_col = col;
    c = *s++;
    while (c != 0)
    {
        if ((c >= '0') && (c <= '9'))
        {
            ca.index = 0x130 + (c - '0');
            at7456E_write_char(row, d_col, &ca);
        }
        else if ((c >= 'A') && (c <= 'Z'))
        {
            ca.index = 0x141 + (c - 'A');
            at7456E_write_char(row, d_col, &ca);
        }
        else if ((c >= 'a') && (c <= 'z'))
        {
            ca.index = 0x161 + (c - 'a');
            at7456E_write_char(row, d_col, &ca);
        }
        else if (c == ' ')
        {
            ca.index = 0x00;
            at7456E_write_char(row, d_col, &ca);
        }
        else if (c == ':')
        {
            ca.index = 0x13a;
            at7456E_write_char(row, d_col, &ca);
        }
        else if (c == '-')
        {
            ca.index = 0x12d;
            at7456E_write_char(row, d_col, &ca);
        }
        else if (c == '.')
        {
            ca.index = 0x12e;
            at7456E_write_char(row, d_col, &ca);
        }
        d_col++;
        c = *s++;
    }
}

static void Display_CHN(unsigned char row, unsigned char col, unsigned char *s)
{
    unsigned int kk;
    unsigned char c;
    kk = row * 30 + col;
    write_at7456_addr_data(DMAH, kk / 256);
    write_at7456_addr_data(DMAL, kk % 256);
    write_at7456_addr_data(DMM, 0x01);
    c = *s++;
    while (c != 0)
    {
        if ((c >= '0') && (c <= '9'))
            write_at7456_data((c == '0') ? 10 : c - '1' + 1);
        else if ((c >= 'A') && (c <= 'Z'))
            write_at7456_data(c - 'A' + 11);
        else if ((c >= 'a') && (c <= 'z'))
            write_at7456_data(c - 'a' + 37);
        else if (c == ' ')
            write_at7456_data(0x00);
        else if (c == '(')
            write_at7456_data(0x3f);
        else if (c == ')')
            write_at7456_data(0x40);
        else if (c == '.')
            write_at7456_data(0x41);
        else if (c == '?')
            write_at7456_data(0x42);
        else if (c == ';')
            write_at7456_data(0x43);
        else if (c == ':')
            write_at7456_data(0x44);
        else if (c == ',')
            write_at7456_data(0x45);
        else if (c == '\'')
            write_at7456_data(0x46);
        else if (c == '/')
            write_at7456_data(0x47);
        else if (c == '"')
            write_at7456_data(0x48);
        else if (c == '-')
            write_at7456_data(0x49);
        else if (c == '<')
            write_at7456_data(0x4a);
        else if (c == '>')
            write_at7456_data(0x4b);
        else if (c == '@')
            write_at7456_data(0x4c);
        else
            write_at7456_data(0xc7);
        c = *s++;
        kk++;
    }
    write_at7456_data(0xff);
}

static void at7456_init(void)
{
    at7456_reset();
    unsigned char n = 0;
    configOSDBL(!n);
    newAt7456 = checkAT7456();
    n = read_at7456_addr_data(VM0);
    n |= PAL;
    write_at7456_addr_data(VM0, n);
    os_delay(0.001);
    n = read_at7456_addr_data(VM0);
}

const unsigned char charset_array1[] = {
    // 地址:0x1E0
    0x55, 0x55, 0x55, 0x54, 0x54, 0x55, 0x52, 0x12, 0x15, 0x52, 0x08, 0x55, 0x52, 0x21, 0x55, 0x52, 0x85, 0x55, 0x52, 0x21, 0x55, 0x52, 0x08, 0x55, 0x52, 0x12, 0x15,
    0x54, 0x54, 0x55, 0x55, 0x05, 0x41, 0x54, 0xA1, 0x28, 0x54, 0x88, 0x88, 0x54, 0x82, 0x08, 0x54, 0x84, 0x48, 0x54, 0x85, 0x48, 0x54, 0x85, 0x48, 0x55, 0x15, 0x51};

const unsigned char charset_array2[] = {
    // 地址:0x1E1
    0x55, 0x55, 0x55, 0x55, 0x41, 0x55, 0x55, 0x28, 0x55, 0x55, 0x21, 0x55, 0x55, 0x28, 0x55, 0x55, 0x21, 0x55, 0x55, 0x28, 0x55, 0x55, 0x21, 0x55, 0x55, 0x28, 0x55,
    0x55, 0x21, 0x55, 0x55, 0x28, 0x55, 0x55, 0x28, 0x55, 0x54, 0xAA, 0x15, 0x54, 0xAA, 0x15, 0x54, 0xAA, 0x15, 0x55, 0x28, 0x55, 0x55, 0x41, 0x55, 0x55, 0x55, 0x55};

const unsigned char charset_array3[] = {
    // 地址:0x1E2
    0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x41, 0x55, 0x55, 0x28, 0x55, 0x54, 0xAA, 0x15, 0x52, 0x82, 0x85,
    0x52, 0xAA, 0x85, 0x4A, 0xAA, 0xA1, 0x2A, 0x28, 0xA8, 0x28, 0x28, 0x28, 0x41, 0x41, 0x41, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55};

const unsigned char charset_array4[] = {
    // 地址:0x1E3
    0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x50, 0x55, 0x41, 0x4A, 0x15, 0x28, 0x52, 0x84, 0xA1, 0x54, 0xA2, 0x85, 0x55, 0x2A, 0x15,
    0x05, 0x28, 0x55, 0xA0, 0xA1, 0x55, 0x2A, 0x85, 0x55, 0x4A, 0x15, 0x55, 0x50, 0x55, 0x55, 0x00, 0x00, 0x00, 0xAA, 0xAA, 0xAA, 0x00, 0x00, 0x00, 0x55, 0x55, 0x55};

struct DriverInfo
{
    uint32_t param;
    Port port;
};
float test1, test2;

static void OSD_Server(void *pvParameters)
{
    DriverInfo driver_info = *(DriverInfo *) pvParameters;
    delete (DriverInfo *) pvParameters;

    if (!osd_spi_init())
    {
        vTaskDelete(NULL);
        return;
    }
    at7456_init();

    uint8_t a = read_at7456_addr_data(VM0);
    write_at7456_addr_data(VM0, OSD_ENABLE | a);
    os_delay(0.001);
    clear_screen(); // 手动清屏

    // 更新字库
    turnOff_osd();
    write_at7456E_font(0x1e0, charset_array1);
    write_at7456E_font(0x1e1, charset_array2);
    write_at7456E_font(0x1e2, charset_array3);
    write_at7456E_font(0x1e3, charset_array4);
    turnOn_osd();
    wait_memory_wr_ok();

    CHAR_ATTRIB ca;
    uint8_t buff[24];
    // 模拟地平线测标尺
    ca.index = 0x1c9;
    ca.attrib = 0;
    at7456E_write_char(6, 9, &ca);
    at7456E_write_char(7, 9, &ca);
    at7456E_write_char(8, 9, &ca);
    at7456E_write_char(9, 9, &ca);
    at7456E_write_char(10, 9, &ca);

    ca.index = 0x13e;
    ca.attrib = 0;
    at7456E_write_char(8, 10, &ca);

    ca.index = 0x1c8;
    ca.attrib = 0;
    at7456E_write_char(6, 21, &ca);
    at7456E_write_char(7, 21, &ca);
    at7456E_write_char(8, 21, &ca);
    at7456E_write_char(9, 21, &ca);
    at7456E_write_char(10, 21, &ca);

    ca.index = 0x13c;
    ca.attrib = 0;
    at7456E_write_char(8, 20, &ca);

    while (1)
    {
        // 模式显示
        AFunc mode = getCurrentFlyMode();
        switch (mode)
        {
            case AFunc_AltHold:
            {
                sprintf((char *) buff, "AltH");
                break;
            }
            case AFunc_PosHold:
            {
                sprintf((char *) buff, "PosH");
                break;
            }
            case AFunc_Mission:
            {
                sprintf((char *) buff, "Miss");
                break;
            }
            case AFunc_RTL:
            {
                sprintf((char *) buff, "RTL ");
                break;
            }
            case AFunc_Land:
            {
                sprintf((char *) buff, "Land");
                break;
            }
            case AFunc_Manual:
            { // 手动
                sprintf((char *) buff, "Manu");
                break;
            }
            case AFunc_Stabilize:
            { // 自稳
                sprintf((char *) buff, "Stab");
                break;
            }
            case AFunc_Rattitude:
            { // 半自稳
                sprintf((char *) buff, "Ratt");
                break;
            }
            default:
            {
                sprintf((char *) buff, "Null");
                break;
            }
        }
        DisplayString(2, 2, buff);

        uint16_t mav_mode, mav_main_mode, mav_sub_mode;
        get_mav_modes(mav_mode, mav_main_mode, mav_sub_mode);
        if (mav_mode & 0x80)
        { // 解锁图标
            ca.index = 0x110;
            ca.attrib = 0;
            at7456E_write_char(2, 6, &ca);
        }
        else
        { // 上锁图标
            ca.index = 0x111;
            ca.attrib = 0;
            at7456E_write_char(2, 6, &ca);
        }

        // 电池
        BatteryInfo batInfo;
        getCurrentBatteryInfo(&batInfo);

        if (batInfo.totalPercent >= 75.0)
        {
            ca.index = 0x1b8;
            ca.attrib = 0;
            at7456E_write_char(12, 28, &ca);
        }
        else if (batInfo.totalPercent >= 50.0)
        {
            ca.index = 0x1b7;
            ca.attrib = 0;
            at7456E_write_char(12, 28, &ca);
        }
        else if (batInfo.totalPercent >= 25.0)
        {
            ca.index = 0x1b6;
            ca.attrib = 0;
            at7456E_write_char(12, 28, &ca);
        }
        else if (batInfo.totalPercent >= 10.0)
        {
            ca.index = 0x1b5;
            ca.attrib = 0;
            at7456E_write_char(12, 28, &ca);
        }
        else
        {
            ca.index = 0x1b4;
            ca.attrib = CHAR_BLK;
            at7456E_write_char(12, 28, &ca);
        }

        ca.index = 0x18e;
        ca.attrib = 0;
        at7456E_write_char(13, 28, &ca);
        ca.index = 0x18f;
        ca.attrib = 0;
        at7456E_write_char(14, 28, &ca);

        sprintf((char *) buff, "%4.1f", batInfo.totalVoltRawFilted);
        DisplayString(13, 24, buff);
        sprintf((char *) buff, "%6.2f", batInfo.totalCurrent);
        DisplayString(14, 22, buff);

        // 获取最优全球定位传感器信息
        PosSensorHealthInf3 global_inf;
        if (get_OptimalGlobal_XYZ(&global_inf) == true)
        {
            // 获取GPS信息
            Position_Sensor gps_sensor;
            if (GetPositionSensor(global_inf.sensor_ind, &gps_sensor))
            { // 显示GPS信息  离家距离
                ca.index = 0x12a;
                ca.attrib = 0;
                at7456E_write_char(10, 1, &ca);
                ca.index = 0x112;
                ca.attrib = 0;
                at7456E_write_char(11, 1, &ca);
                //			ca.index = 0x184;
                //			ca.attrib = 0;
                //			at7456E_write_char(13, 1, &ca);
                //			ca.index = 0x183;
                //			ca.attrib = 0;
                //			at7456E_write_char(12, 1, &ca);
                ca.index = 0x185;
                ca.attrib = 0;
                at7456E_write_char(14, 1, &ca);
                ca.index = 0x18d;
                ca.attrib = 0;
                at7456E_write_char(14, 7, &ca);

                sprintf((char *) buff, "%3.0f", gps_sensor.inf.addition_inf[0]); // 卫星数
                DisplayString(10, 2, buff);
                sprintf((char *) buff, "%-11.2f", gps_sensor.inf.addition_inf[4] * 0.01); // 定位精度
                DisplayString(11, 2, buff);

                bool inFlight;
                get_is_inFlight(&inFlight);
                static double last_global_x = 0, last_global_y = 0;
                static bool last_global_init = false;
                static double flight_distance = 0;

                double x, y;
                map_projection_project(&global_inf.mp, gps_sensor.data.position_Global.x, gps_sensor.data.position_Global.y, &x, &y);
                if (last_global_init == false)
                { // 初始化
                    last_global_init = true;

                    last_global_x = x;
                    last_global_y = y;

                    ca.index = 0x18d;
                    ca.attrib = 0;
                    at7456E_write_char(12, 17, &ca);

                    uint8_t pLen = sprintf((char *) buff, "%.0f", 0.0f); // 总距离
                    DisplayString(12, 17 - pLen, buff);
                }
                else // if(inFlight)
                {
                    double error_global_x = fabs(x - last_global_x);
                    double error_global_y = fabs(y - last_global_y);
                    double distance = sqrt((error_global_x * error_global_x) + (error_global_y * error_global_y));

                    if (distance > 100)
                    { // 变化大于1m，累加显示
                        flight_distance += (distance / 100);

                        last_global_x = x;
                        last_global_y = y;

                        if (flight_distance < 1000)
                        { // 显示单位米
                            ca.index = 0x18d;
                            ca.attrib = 0;
                            at7456E_write_char(12, 17, &ca);

                            uint8_t pLen = sprintf((char *) buff, "%.0f", flight_distance); // 总距离
                            DisplayString(12, 17 - pLen, buff);
                        }
                        else
                        { // 显示单位千米
                            ca.index = 0x1e0;
                            ca.attrib = 0;
                            at7456E_write_char(12, 17, &ca);

                            uint8_t pLen = sprintf((char *) buff, "%.2f", flight_distance / 1000); // 总距离
                            DisplayString(12, 17 - pLen, buff);
                        }
                    }
                }

                //			sprintf((char *)buff, "%-11.6f", gps_sensor.data.position_Global.x);	//纬度
                //			DisplayString(12, 2, buff);
                //			sprintf((char *)buff, "%-11.6f", gps_sensor.data.position_Global.y);	//经度
                //			DisplayString(13, 2, buff);
                sprintf((char *) buff, "%5.0f", gps_sensor.data.position_Global.z * 0.01); // 海拔
                DisplayString(14, 2, buff);

                vector2<double> homePoint;
                if (getHomePoint(&homePoint))
                {
                    vector3<double> Position;
                    get_Position_Ctrl(&Position);

                    // 离家距离
                    ca.index = 0x11f;
                    ca.attrib = 0;
                    at7456E_write_char(2, 22, &ca);
                    ca.index = 0x1e0;
                    ca.attrib = 0;
                    at7456E_write_char(2, 28, &ca);

                    vector2<double> Point;
                    Point.x = Position.x - homePoint.x;
                    Point.y = Position.y - homePoint.y;

                    double Gs = safe_sqrt(Point.x * Point.x + Point.y * Point.y) * 0.01;
                    sprintf((char *) buff, "%5.3f", Gs * 0.001); // 离家距离
                    DisplayString(2, 23, buff);

                    // 距家角度
                    ca.index = 0x17f;
                    ca.attrib = 0;
                    at7456E_write_char(4, 22, &ca);

                    double angle;
                    if (inFlight)
                    {
                        // 距家角度
                        Point.x = Position.x - homePoint.x;
                        Point.y = Position.y - homePoint.y;

                        angle = rad2degree(atan2(Point.y, Point.x));
                        angle = -angle + 90; // enu转ned
                        if (angle < 0)
                            angle += 360;
                    }
                    else
                    {
                        angle = 0;
                    }

                    sprintf((char *) buff, "%4.0f", angle); // 距家角度
                    DisplayString(4, 23, buff);
                }
            }
            else
            {                                    // 清屏
                sprintf((char *) buff, "     "); // 卫星数
                DisplayString(10, 1, buff);
                sprintf((char *) buff, "       "); // 定位精度
                DisplayString(11, 1, buff);
                sprintf((char *) buff, "            "); // 经度
                DisplayString(12, 1, buff);
                sprintf((char *) buff, "            "); // 纬度
                DisplayString(13, 1, buff);
                sprintf((char *) buff, "          "); // 海拔
                DisplayString(14, 1, buff);
                sprintf((char *) buff, "       "); // 离家距离
                DisplayString(2, 22, buff);
                sprintf((char *) buff, "      "); // 距家角度
                DisplayString(4, 22, buff);
            }
        }

        // 飞行时间
        static TIME inFlight_TIME(false);
        bool inFlight;
        get_is_inFlight(&inFlight);

        static uint32_t inFlight_time = 0;
        if (inFlight && inFlight_TIME.get_pass_time() > 1)
        {
            inFlight_TIME = TIME::now();
            inFlight_time++;
        }

        ca.index = 0x1b3;
        ca.attrib = 0;
        at7456E_write_char(14, 14, &ca);

        sprintf((char *) buff, "%3.1d:%2.2d", inFlight_time / 60, inFlight_time % 60); // 运行时间
        DisplayString(14, 15, buff);

        // 离家高度
        static bool last_inFlight = false;
        static double homeZ = 0;
        get_is_inFlight(&inFlight);
        if (inFlight != last_inFlight)
        {
            getHomeLocalZ(&homeZ, 0, 0.01);
            last_inFlight = inFlight;
        }

        vector3<double> position;
        get_Position_Ctrl(&position);
        double heightAboveGround = position.z - homeZ;

        ca.index = 0x1e3;
        ca.attrib = 0;
        at7456E_write_char(3, 22, &ca);
        ca.index = 0x18d;
        ca.attrib = 0;
        at7456E_write_char(3, 28, &ca);

        sprintf((char *) buff, "%5.0f", heightAboveGround * 0.01); // 离家高度
        DisplayString(3, 23, buff);

        // 飞行速度
        vector3<double> vel;
        get_VelocityENU_Ctrl(&vel);
        double Gs = safe_sqrt(vel.x * vel.x + vel.y * vel.y) * 0.01;

        ca.index = 0x186;
        ca.attrib = 0;
        at7456E_write_char(3, 2, &ca);
        ca.index = 0x181;
        ca.attrib = 0;
        at7456E_write_char(3, 6, &ca);

        sprintf((char *) buff, "%3.0f", Gs * 3.6); // 飞行速度
        DisplayString(3, 3, buff);

        // 电调温度
        extern float esc_temperature;
        if (esc_temperature != -1)
        {
            ca.index = 0x1e1;
            ca.attrib = 0;
            at7456E_write_char(4, 2, &ca);

            sprintf((char *) buff, "%3.0fc", esc_temperature); // 电调温度
            DisplayString(4, 3, buff);
        }

        // 姿态获取
        Quaternion quat;
        get_history_AirframeQuatY(&quat, 0.1);
        int roll = rad2degree(quat.getRoll());
        int pitch = rad2degree(quat.getPitch());
        int yaw = rad2degree(quat.getYaw());

        // 俯仰
        ca.index = 0x1b0;
        ca.attrib = 0;
        at7456E_write_char(2, 12, &ca);
        ca.index = 0x1b1;
        ca.attrib = 0;
        at7456E_write_char(2, 13, &ca);

        sprintf((char *) buff, "%4.1d", pitch); // 俯仰
        DisplayString(2, 8, buff);

        // 横滚
        ca.index = 0x1b0;
        ca.attrib = 0;
        at7456E_write_char(2, 19, &ca);
        ca.index = 0x1b2;
        ca.attrib = 0;
        at7456E_write_char(2, 20, &ca);

        sprintf((char *) buff, "%4.1d", roll); // 横滚
        DisplayString(2, 15, buff);

        // 偏航
        ca.index = 0x1e2;
        ca.attrib = 0;
        at7456E_write_char(3, 13, &ca);
        ca.index = 0x1b0;
        ca.attrib = 0;
        at7456E_write_char(3, 18, &ca);

        sprintf((char *) buff, "%4.1d", yaw); // 偏航
        DisplayString(3, 14, buff);

        // 模拟地平线
        for (int i = 6; i <= 10; i++)
        { // 清空
            for (int j = 11; j <= 19; j++)
            {
                ca.index = 0x00;
                ca.attrib = 0;
                at7456E_write_char(i, j, &ca);
            }
        }

        uint8_t row;
        uint16_t roll_buff[9];
        uint8_t row_buff[9];

        if (pitch >= 45)
            row = 6;
        else if (pitch >= 25)
            row = 7;
        else if (pitch >= -5)
            row = 8;
        else if (pitch >= -25)
            row = 9;
        else
            row = 10;

        if (roll >= 45)
        {
            roll_buff[0] = 0x106;
            row_buff[0] = row + 1;
            roll_buff[1] = 0x10a;
            row_buff[1] = row + 1;
            roll_buff[2] = 0x10e;
            row_buff[2] = row + 1;
            roll_buff[3] = 0x108;
            row_buff[3] = row;
            roll_buff[4] = 0x10a;
            row_buff[4] = row;
            roll_buff[5] = 0x10c;
            row_buff[5] = row;
            roll_buff[6] = 0x106;
            row_buff[6] = row - 1;
            roll_buff[7] = 0x10a;
            row_buff[7] = row - 1;
            roll_buff[8] = 0x10e;
            row_buff[8] = row - 1;
        }
        else if (roll >= 25)
        {
            roll_buff[0] = 0x106;
            row_buff[0] = row;
            roll_buff[1] = 0x107;
            row_buff[1] = row;
            roll_buff[2] = 0x108;
            row_buff[2] = row;
            roll_buff[3] = 0x109;
            row_buff[3] = row;
            roll_buff[4] = 0x10a;
            row_buff[4] = row;
            roll_buff[5] = 0x10b;
            row_buff[5] = row;
            roll_buff[6] = 0x10c;
            row_buff[6] = row;
            roll_buff[7] = 0x10d;
            row_buff[7] = row;
            roll_buff[8] = 0x10e;
            row_buff[8] = row;
        }
        else if (roll >= -5)
        {
            roll_buff[0] = 0x10a;
            row_buff[0] = row;
            roll_buff[1] = 0x10a;
            row_buff[1] = row;
            roll_buff[2] = 0x10a;
            row_buff[2] = row;
            roll_buff[3] = 0x10a;
            row_buff[3] = row;
            roll_buff[4] = 0x10a;
            row_buff[4] = row;
            roll_buff[5] = 0x10a;
            row_buff[5] = row;
            roll_buff[6] = 0x10a;
            row_buff[6] = row;
            roll_buff[7] = 0x10a;
            row_buff[7] = row;
            roll_buff[8] = 0x10a;
            row_buff[8] = row;
        }
        else if (roll >= -25)
        {
            roll_buff[0] = 0x10e;
            row_buff[0] = row;
            roll_buff[1] = 0x10d;
            row_buff[1] = row;
            roll_buff[2] = 0x10c;
            row_buff[2] = row;
            roll_buff[3] = 0x10b;
            row_buff[3] = row;
            roll_buff[4] = 0x10a;
            row_buff[4] = row;
            roll_buff[5] = 0x109;
            row_buff[5] = row;
            roll_buff[6] = 0x108;
            row_buff[6] = row;
            roll_buff[7] = 0x107;
            row_buff[7] = row;
            roll_buff[8] = 0x106;
            row_buff[8] = row;
        }
        else
        {
            roll_buff[0] = 0x10e;
            row_buff[0] = row - 1;
            roll_buff[1] = 0x10a;
            row_buff[1] = row - 1;
            roll_buff[2] = 0x106;
            row_buff[2] = row - 1;
            roll_buff[3] = 0x10e;
            row_buff[3] = row;
            roll_buff[4] = 0x10a;
            row_buff[4] = row;
            roll_buff[5] = 0x106;
            row_buff[5] = row;
            roll_buff[6] = 0x10e;
            row_buff[6] = row + 1;
            roll_buff[7] = 0x10a;
            row_buff[7] = row + 1;
            roll_buff[8] = 0x106;
            row_buff[8] = row + 1;
        }

        for (int i = 0; i < 9; i++)
        {
            ca.index = roll_buff[i];
            if (row_buff[i] >= 6 && row_buff[i] <= 10)
                at7456E_write_char(row_buff[i], 11 + i, &ca);
        }

        // 中心标尺
        ca.index = 0x1c7;
        ca.attrib = 0;
        at7456E_write_char(8, 15, &ca);

        os_delay(0.1);
    }
}

static bool OSD_DriverInit()
{
    DriverInfo *driver_info = new DriverInfo;
    xTaskCreate(OSD_Server, "OSD", 812, (void *) driver_info, SysPriority_ExtSensor, NULL);
    return true;
}

void init_drv_osd_analog()
{
    HW_ConfigEntry entry;

    // 读取 SPI 编号，若不存在则不初始化
    if (!HW_ReadConfigEntry(HW_DEV_OSD_ANALOG, HW_CFG_SPI_REF, &entry))
        return;

    OSD_DriverInit();
}