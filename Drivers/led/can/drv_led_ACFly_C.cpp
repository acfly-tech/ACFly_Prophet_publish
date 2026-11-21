#include "drv_led_ACFly_C.hpp"

#include "Basic.hpp"
#include "Commulink.hpp"
#include "drv_can.hpp"

struct DriverInfo
{
    CanMailBox *mail_box;
    uint32_t sensor_key;
};

static void led_ACFly_Server(void *pvParameters)
{
    DriverInfo driver_info = *(DriverInfo *) pvParameters;
    delete (DriverInfo *) pvParameters;

    extern volatile float ExtLedR;
    extern volatile float ExtLedG;
    extern volatile float ExtLedB;
    while (1)
    {
        CanPacket mail;
        mail.IdType = 0;
        mail.FrameType = 0;
        mail.FDFormat = 0;
        mail.Identifier = 0x4AC;

        mail.DataLength = 8;
        *((uint16_t *) &mail.data[0]) = __REV16((uint16_t) (ExtLedR * 10));
        *((uint16_t *) &mail.data[2]) = __REV16((uint16_t) (ExtLedG * 10));
        *((uint16_t *) &mail.data[4]) = __REV16((uint16_t) (ExtLedB * 10));

        driver_info.mail_box->SendMail(mail);

        os_delay(0.01);
    }
}

static bool led_ACFly_DriverInit()
{
    return true;
}
static bool led_ACFly_DriverRun(CanMailBox *mailBox)
{
    CanId Ids[1];

    DriverInfo *driver_info = new DriverInfo;
    driver_info->mail_box = mailBox;
    xTaskCreate(led_ACFly_Server, "led_ACFly", 800, (void *) driver_info, SysPriority_UserTask, NULL);
    return true;
}

void init_drv_led_ACFly_C(void)
{
    CanFunc_Register(1, led_ACFly_DriverInit, led_ACFly_DriverRun);
}