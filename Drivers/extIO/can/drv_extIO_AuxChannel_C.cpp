#include "drv_extIO_AuxChannel_C.hpp"

#include "Basic.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "Parameters.hpp"
#include "StorageSystem.hpp"
#include "drv_CAN.hpp"
#include "drv_PWMOut.hpp"

struct DriverInfo
{
    CanMailBox *mail_box;
    uint32_t sensor_key;
    uint8_t pwm_base;
};

static void extIO_AuxChannel_Server(void *pvParameters)
{
    DriverInfo driver_info = *(DriverInfo *) pvParameters;
    delete (DriverInfo *) pvParameters;

    uint16_t pwm_send_counter = 0;
    uint16_t param_send_counter = 0;
    uint16_t param_counter = 0;

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        // 800hz
        vTaskDelayUntil(&xLastWakeTime, 1);

        if (++pwm_send_counter >= 400)
        {
            pwm_send_counter = 0;

            CanPacket mail;
            mail.IdType = 1;
            mail.FrameType = 0;
            mail.FDFormat = 0;
            mail.DataLength = 8;

            uint8_t base = driver_info.pwm_base;
            mail.Identifier = 0xacac101;
            *(uint16_t *) &mail.data[0] = (uint16_t) getPWMus(base + 0);
            *(uint16_t *) &mail.data[2] = (uint16_t) getPWMus(base + 1);
            *(uint16_t *) &mail.data[4] = (uint16_t) getPWMus(base + 2);
            *(uint16_t *) &mail.data[6] = (uint16_t) getPWMus(base + 3);
            driver_info.mail_box->SendMail(mail);

            mail.Identifier = 0xacac102;
            *(uint16_t *) &mail.data[0] = (uint16_t) getPWMus(base + 4);
            *(uint16_t *) &mail.data[2] = (uint16_t) getPWMus(base + 5);
            *(uint16_t *) &mail.data[4] = (uint16_t) getPWMus(base + 6);
            *(uint16_t *) &mail.data[6] = (uint16_t) getPWMus(base + 7);
            driver_info.mail_box->SendMail(mail);
        }

        for (int rx_n = 0; rx_n < 10; ++rx_n)
        {
            CanPacket mail;
            if (!driver_info.mail_box->receiveMail(&mail, 0))
                break;
            switch (mail.Identifier)
            {
                case 0xacac200:
                { // 通道频率
                    uint32_t ExtIO_Period[2];

                    ExtIO_Period[0] = *(uint32_t *) &mail.data[0];
                    ExtIO_Period[1] = *(uint32_t *) &mail.data[4];
                    break;
                }
            }
        }
    }
}

static bool extIO_AuxChannel_DriverInit()
{
    return true;
}
static bool extIO_AuxChannel_DriverRun(CanMailBox *mailBox)
{
    CanId Ids[1];

    Ids[0].Identifier = 0xacac200;
    Ids[0].IdType = 1;

    mailBox->init(5, Ids, 1);
    DriverInfo *driver_info = new DriverInfo;
    driver_info->mail_box = mailBox;
    driver_info->pwm_base = get_PWMChannelsCount();
    xTaskCreate(extIO_AuxChannel_Server, "extIO_AuxChannel", 1024, (void *) driver_info, SysPriority_ExtSensor, NULL);
    return true;
}

void init_drv_extIO_AuxChannel_C()
{
    CanFunc_Register(15, extIO_AuxChannel_DriverInit, extIO_AuxChannel_DriverRun);
}