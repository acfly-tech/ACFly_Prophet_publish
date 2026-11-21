#pragma once
#include <canard.h>
#include <stdbool.h>
#include <stdint.h>

#define REMOTEID_SECURECOMMAND_REQUEST_MAX_SIZE 230
#define REMOTEID_SECURECOMMAND_REQUEST_SIGNATURE (0x2035E2EAE81D2E53ULL)
#define REMOTEID_SECURECOMMAND_REQUEST_ID 64

#define REMOTEID_SECURECOMMAND_REQUEST_SECURE_COMMAND_GET_SESSION_KEY 0
#define REMOTEID_SECURECOMMAND_REQUEST_SECURE_COMMAND_GET_REMOTEID_SESSION_KEY 1
#define REMOTEID_SECURECOMMAND_REQUEST_SECURE_COMMAND_REMOVE_PUBLIC_KEYS 2
#define REMOTEID_SECURECOMMAND_REQUEST_SECURE_COMMAND_GET_PUBLIC_KEYS 3
#define REMOTEID_SECURECOMMAND_REQUEST_SECURE_COMMAND_SET_PUBLIC_KEYS 4
#define REMOTEID_SECURECOMMAND_REQUEST_SECURE_COMMAND_GET_REMOTEID_CONFIG 5
#define REMOTEID_SECURECOMMAND_REQUEST_SECURE_COMMAND_SET_REMOTEID_CONFIG 6

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class remoteid_SecureCommand_cxx_iface;
#endif

struct remoteid_SecureCommandRequest
{
#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = remoteid_SecureCommand_cxx_iface;
#endif
    uint32_t sequence;
    uint32_t operation;
    uint8_t sig_length;
    struct
    {
        uint8_t len;
        uint8_t data[220];
    } data;
};

#ifdef __cplusplus
extern "C"
{
#endif

    uint32_t remoteid_SecureCommandRequest_encode(struct remoteid_SecureCommandRequest *msg, uint8_t *buffer
#if CANARD_ENABLE_TAO_OPTION
                                                  ,
                                                  bool tao
#endif
    );
    bool remoteid_SecureCommandRequest_decode(const CanardRxTransfer *transfer, struct remoteid_SecureCommandRequest *msg);

#if defined(CANARD_DSDLC_INTERNAL)
    static inline void _remoteid_SecureCommandRequest_encode(uint8_t *buffer, uint32_t *bit_ofs, struct remoteid_SecureCommandRequest *msg, bool tao);
    static inline bool _remoteid_SecureCommandRequest_decode(const CanardRxTransfer *transfer, uint32_t *bit_ofs, struct remoteid_SecureCommandRequest *msg, bool tao);
    void _remoteid_SecureCommandRequest_encode(uint8_t *buffer, uint32_t *bit_ofs, struct remoteid_SecureCommandRequest *msg, bool tao)
    {
        (void) buffer;
        (void) bit_ofs;
        (void) msg;
        (void) tao;

        canardEncodeScalar(buffer, *bit_ofs, 32, &msg->sequence);
        *bit_ofs += 32;
        canardEncodeScalar(buffer, *bit_ofs, 32, &msg->operation);
        *bit_ofs += 32;
        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->sig_length);
        *bit_ofs += 8;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
        const uint8_t data_len = msg->data.len > 220 ? 220 : msg->data.len;
#pragma GCC diagnostic pop
        if (!tao)
        {
            canardEncodeScalar(buffer, *bit_ofs, 8, &data_len);
            *bit_ofs += 8;
        }
        for (size_t i = 0; i < data_len; i++)
        {
            canardEncodeScalar(buffer, *bit_ofs, 8, &msg->data.data[i]);
            *bit_ofs += 8;
        }
    }

    /*
     decode remoteid_SecureCommandRequest, return true on failure, false on success
    */
    bool _remoteid_SecureCommandRequest_decode(const CanardRxTransfer *transfer, uint32_t *bit_ofs, struct remoteid_SecureCommandRequest *msg, bool tao)
    {
        (void) transfer;
        (void) bit_ofs;
        (void) msg;
        (void) tao;
        canardDecodeScalar(transfer, *bit_ofs, 32, false, &msg->sequence);
        *bit_ofs += 32;

        canardDecodeScalar(transfer, *bit_ofs, 32, false, &msg->operation);
        *bit_ofs += 32;

        canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->sig_length);
        *bit_ofs += 8;

        if (!tao)
        {
            canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->data.len);
            *bit_ofs += 8;
        }
        else
        {
            msg->data.len = ((transfer->payload_len * 8) - *bit_ofs) / 8;
        }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
        if (msg->data.len > 220)
        {
            return true; /* invalid value */
        }
#pragma GCC diagnostic pop
        for (size_t i = 0; i < msg->data.len; i++)
        {
            canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->data.data[i]);
            *bit_ofs += 8;
        }

        return false; /* success */
    }
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
    struct remoteid_SecureCommandRequest sample_remoteid_SecureCommandRequest_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
#endif
#endif
