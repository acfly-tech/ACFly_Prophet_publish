#pragma once
#include <canard.h>
#include <stdbool.h>
#include <stdint.h>

#define REMOTEID_SELFID_MAX_SIZE 46
#define REMOTEID_SELFID_SIGNATURE (0xD2EDF61E84404C5AULL)
#define REMOTEID_SELFID_ID 20032

#define REMOTEID_SELFID_ODID_DESC_TYPE_TEXT 0

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class remoteid_SelfID_cxx_iface;
#endif

struct remoteid_SelfID
{
#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = remoteid_SelfID_cxx_iface;
#endif
    struct
    {
        uint8_t len;
        uint8_t data[20];
    } id_or_mac;
    uint8_t description_type;
    struct
    {
        uint8_t len;
        uint8_t data[23];
    } description;
};

#ifdef __cplusplus
extern "C"
{
#endif

    uint32_t remoteid_SelfID_encode(struct remoteid_SelfID *msg, uint8_t *buffer
#if CANARD_ENABLE_TAO_OPTION
                                    ,
                                    bool tao
#endif
    );
    bool remoteid_SelfID_decode(const CanardRxTransfer *transfer, struct remoteid_SelfID *msg);

#if defined(CANARD_DSDLC_INTERNAL)
    static inline void _remoteid_SelfID_encode(uint8_t *buffer, uint32_t *bit_ofs, struct remoteid_SelfID *msg, bool tao);
    static inline bool _remoteid_SelfID_decode(const CanardRxTransfer *transfer, uint32_t *bit_ofs, struct remoteid_SelfID *msg, bool tao);
    void _remoteid_SelfID_encode(uint8_t *buffer, uint32_t *bit_ofs, struct remoteid_SelfID *msg, bool tao)
    {
        (void) buffer;
        (void) bit_ofs;
        (void) msg;
        (void) tao;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
        const uint8_t id_or_mac_len = msg->id_or_mac.len > 20 ? 20 : msg->id_or_mac.len;
#pragma GCC diagnostic pop
        canardEncodeScalar(buffer, *bit_ofs, 5, &id_or_mac_len);
        *bit_ofs += 5;
        for (size_t i = 0; i < id_or_mac_len; i++)
        {
            canardEncodeScalar(buffer, *bit_ofs, 8, &msg->id_or_mac.data[i]);
            *bit_ofs += 8;
        }
        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->description_type);
        *bit_ofs += 8;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
        const uint8_t description_len = msg->description.len > 23 ? 23 : msg->description.len;
#pragma GCC diagnostic pop
        if (!tao)
        {
            canardEncodeScalar(buffer, *bit_ofs, 5, &description_len);
            *bit_ofs += 5;
        }
        for (size_t i = 0; i < description_len; i++)
        {
            canardEncodeScalar(buffer, *bit_ofs, 8, &msg->description.data[i]);
            *bit_ofs += 8;
        }
    }

    /*
     decode remoteid_SelfID, return true on failure, false on success
    */
    bool _remoteid_SelfID_decode(const CanardRxTransfer *transfer, uint32_t *bit_ofs, struct remoteid_SelfID *msg, bool tao)
    {
        (void) transfer;
        (void) bit_ofs;
        (void) msg;
        (void) tao;
        canardDecodeScalar(transfer, *bit_ofs, 5, false, &msg->id_or_mac.len);
        *bit_ofs += 5;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
        if (msg->id_or_mac.len > 20)
        {
            return true; /* invalid value */
        }
#pragma GCC diagnostic pop
        for (size_t i = 0; i < msg->id_or_mac.len; i++)
        {
            canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->id_or_mac.data[i]);
            *bit_ofs += 8;
        }

        canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->description_type);
        *bit_ofs += 8;

        if (!tao)
        {
            canardDecodeScalar(transfer, *bit_ofs, 5, false, &msg->description.len);
            *bit_ofs += 5;
        }
        else
        {
            msg->description.len = ((transfer->payload_len * 8) - *bit_ofs) / 8;
        }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
        if (msg->description.len > 23)
        {
            return true; /* invalid value */
        }
#pragma GCC diagnostic pop
        for (size_t i = 0; i < msg->description.len; i++)
        {
            canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->description.data[i]);
            *bit_ofs += 8;
        }

        return false; /* success */
    }
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
    struct remoteid_SelfID sample_remoteid_SelfID_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
BROADCAST_MESSAGE_CXX_IFACE(remoteid_SelfID, REMOTEID_SELFID_ID, REMOTEID_SELFID_SIGNATURE, REMOTEID_SELFID_MAX_SIZE);
#endif
#endif
