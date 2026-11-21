#include "mavlink.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define CUSTOM_CHANS 2

static SemaphoreHandle_t m_mavlink_TxMutex[MAVLINK_COMM_NUM_BUFFERS + CUSTOM_CHANS] = {0};

mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS + CUSTOM_CHANS];

mavlink_message_t m_mavlink_buffer[MAVLINK_COMM_NUM_BUFFERS + CUSTOM_CHANS];

extern "C"
{
    void mavlink_init_chan(uint8_t chan)
    {
        if (m_mavlink_TxMutex[chan] == 0)
            m_mavlink_TxMutex[chan] = xSemaphoreCreateRecursiveMutex();
    }
    bool mavlink_lock_chan(uint8_t chan, double TIMEOUT)
    {
        if (m_mavlink_TxMutex[chan] == 0)
            return false;
        TickType_t TIMEOUT_Ticks;
        if (TIMEOUT >= 0)
            TIMEOUT_Ticks = TIMEOUT * configTICK_RATE_HZ;
        else
            TIMEOUT_Ticks = portMAX_DELAY;
        if (xSemaphoreTakeRecursive(m_mavlink_TxMutex[chan], TIMEOUT_Ticks))
            return true;
        return false;
    }
    void mavlink_unlock_chan(uint8_t chan)
    {
        xSemaphoreGiveRecursive(m_mavlink_TxMutex[chan]);
    }

    int8_t reg_custom_mavlink_chan()
    {
        for (uint8_t i = MAVLINK_COMM_NUM_BUFFERS; i < MAVLINK_COMM_NUM_BUFFERS + CUSTOM_CHANS; ++i)
        {
            if (m_mavlink_TxMutex[i] == 0)
            {
                m_mavlink_TxMutex[i] = xSemaphoreCreateRecursiveMutex();
                return i;
            }
        }
        return -1;
    }
}