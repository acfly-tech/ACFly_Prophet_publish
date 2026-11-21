#pragma once

#include "Basic.h"
#include "freertos.h"
#include "queue.h"
#include <stdint.h>

struct CanPacket
{
    uint8_t IdType; // 0-standard 1-extended
    uint8_t DataLength;
    uint8_t FrameType;   // 0-data_frame 1-remote_frame
    uint8_t FDFormat;    // 0-classic 1-FD
    uint32_t Identifier; // 0~0x7FF (standard) or 0~0x1FFFFFFF (extended)
    uint8_t data[64];
};

struct CanId
{
    uint8_t IdType; // 0-standard 1-extended
    uint32_t Identifier;
};

struct CanIdWithMask
{
    uint8_t IdType; // 0-standard 1-extended
    uint32_t Identifier;
    uint32_t mask; // 0-无mask
};

class CanMailBox
{
  public:
    virtual bool init(uint16_t Capacity, CanIdWithMask *Ids, uint16_t Ids_count) = 0;
    virtual bool init(uint16_t Capacity, CanId *Ids, uint16_t Ids_count) = 0;
    virtual uint32_t get_mail_overflow_cnt() = 0;
    virtual bool receiveMail(CanPacket *mail, double waitTime) = 0;
    virtual bool SendMail(CanPacket mail, double Send_waitTime = 0, double Sync_waitTime = -1) = 0;
    virtual ~CanMailBox() = default;
};

// 内部实现的统一邮箱类 (forward-declare CanBusState defined in drv_Can.cpp)
struct CanBusState;

class CanBusMailBox : public CanMailBox
{
  private:
    CanBusState *drv;
    uint16_t rx_idCount;
    uint32_t *id_masks;
    uint32_t *ids;
    QueueHandle_t packet_queue;
    uint32_t mail_overflow_cnt;

    bool checkMail(CanId id);

  public:
    explicit CanBusMailBox(uint8_t can_num);
    CanBusMailBox(uint8_t can_num, uint16_t Capacity, CanIdWithMask *Ids, uint16_t Ids_count);
    CanBusMailBox(uint8_t can_num, uint16_t Capacity, CanId *Ids, uint16_t Ids_count);

    bool init(uint16_t Capacity, CanIdWithMask *Ids, uint16_t Ids_count) override;
    bool init(uint16_t Capacity, CanId *Ids, uint16_t Ids_count) override;

    uint32_t get_mail_overflow_cnt() override
    {
        return mail_overflow_cnt;
    }
    bool receiveMail(CanPacket *mail, double waitTime) override;
    bool SendMail(CanPacket mail, double Send_waitTime = 0, double Sync_waitTime = -1) override;

    // ISR回调 (仅内部使用)
    static void CanRxTCB(void *pvParameter1, uint32_t ulParameter2);
};

// 向后兼容: Can1MailBox / Can2MailBox
class Can1MailBox : public CanBusMailBox
{
  public:
    Can1MailBox()
        : CanBusMailBox(1)
    {
    }
    Can1MailBox(uint16_t C, CanIdWithMask *Ids, uint16_t n)
        : CanBusMailBox(1, C, Ids, n)
    {
    }
    Can1MailBox(uint16_t C, CanId *Ids, uint16_t n)
        : CanBusMailBox(1, C, Ids, n)
    {
    }
};

class Can2MailBox : public CanBusMailBox
{
  public:
    Can2MailBox()
        : CanBusMailBox(2)
    {
    }
    Can2MailBox(uint16_t C, CanIdWithMask *Ids, uint16_t n)
        : CanBusMailBox(2, C, Ids, n)
    {
    }
    Can2MailBox(uint16_t C, CanId *Ids, uint16_t n)
        : CanBusMailBox(2, C, Ids, n)
    {
    }
};

// 根据硬件配置INI初始化所有CAN总线
void init_drv_can(void);

// 查询指定CAN总线是否已初始化 (can_num: 1=CAN1, 2=CAN2)
bool can_is_inited(uint8_t can_num);

// 为指定CAN总线创建新邮箱实例 (总线未初始化返回nullptr)
CanMailBox *create_can_mailbox(uint8_t can_num);
