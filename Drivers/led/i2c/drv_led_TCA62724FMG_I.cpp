#include "drv_led_TCA62724FMG_I.hpp"
#include "Basic.hpp"
#include "Commulink.hpp"
#include "drv_i2c.hpp"

#define i2c_speed I2C_SPEED_50k

#define ADDR 0x55              /**< I2C adress of TCA62724FMG */
#define SUB_ADDR_START 0x01    /**< write everything (with auto-increment) */
#define SUB_ADDR_PWM0 0x81     /**< blue     (without auto-increment) */
#define SUB_ADDR_PWM1 0x82     /**< green    (without auto-increment) */
#define SUB_ADDR_PWM2 0x83     /**< red      (without auto-increment) */
#define SUB_ADDR_SETTINGS 0x84 /**< settings (without auto-increment)*/

#define SETTING_NOT_POWERSAVE 0x01 /**< power-save mode not off */
#define SETTING_ENABLE 0x02        /**< on */

static void led_TCA62724FMG_Server(void *pvParameters)
{
    I2C_CLASS *i2c_port = (I2C_CLASS *) pvParameters;
#ifdef DCACHE_SIZE
    Aligned_DMABuf uint8_t tx_buf[DCACHE_SIZE];
#else
    Aligned_DMABuf uint8_t tx_buf[32];
#endif
    uint16_t tx_buf_size = sizeof(tx_buf);

    I2C_Result res;
reTry:
    while (1)
    {
        os_delay(1.0);

        tx_buf[0] = SUB_ADDR_SETTINGS;
        tx_buf[1] = SETTING_NOT_POWERSAVE;
        res = i2c_port->write(ADDR, tx_buf, 2, tx_buf_size, i2c_speed);
        if (res != I2C_Result_Success)
            continue;
        os_delay(2);

        tx_buf[0] = SUB_ADDR_PWM0;
        tx_buf[1] = 0;
        tx_buf[2] = SUB_ADDR_PWM1;
        tx_buf[3] = 0;
        tx_buf[4] = SUB_ADDR_PWM2;
        tx_buf[5] = 0;
        res = i2c_port->write(ADDR, tx_buf, 6, tx_buf_size, i2c_speed);
        if (res != I2C_Result_Success)
            continue;

        tx_buf[0] = SUB_ADDR_SETTINGS;
        tx_buf[1] = SETTING_NOT_POWERSAVE | SETTING_ENABLE;
        res = i2c_port->write(ADDR, tx_buf, 2, tx_buf_size, i2c_speed);
        os_delay(1);
        if (res != I2C_Result_Success)
            continue;

        break;
    }

    extern volatile float ExtLedR;
    extern volatile float ExtLedG;
    extern volatile float ExtLedB;

    int retry_cnt = 0;
    while (1)
    {
        tx_buf[0] = SUB_ADDR_PWM0;
        tx_buf[1] = ExtLedB * 15.0f / 100;
        tx_buf[2] = SUB_ADDR_PWM1;
        tx_buf[3] = ExtLedG * 15.0f / 100;
        tx_buf[4] = SUB_ADDR_PWM2;
        tx_buf[5] = ExtLedR * 15.0f / 100;
        res = i2c_port->write(ADDR, tx_buf, 6, tx_buf_size, i2c_speed);
        if (res != I2C_Result_Success)
        {
            if (++retry_cnt >= 3)
                goto reTry;
        }
        else
            retry_cnt = 0;

        os_delay(0.05);
    }
}

static bool led_TCA62724FMG_DriverInit(I2C_CLASS *i2c)
{
    return true;
}
static bool led_TCA62724FMG_DriverRun(I2C_CLASS *i2c)
{
    xTaskCreate(led_TCA62724FMG_Server, "led_TCA62724FMG", 800, (void *) i2c, SysPriority_UserTask, NULL);
    return true;
}

void init_drv_led_TCA62724FMG_I()
{
    I2CFunc_Register(1, led_TCA62724FMG_DriverInit, led_TCA62724FMG_DriverRun);
}
