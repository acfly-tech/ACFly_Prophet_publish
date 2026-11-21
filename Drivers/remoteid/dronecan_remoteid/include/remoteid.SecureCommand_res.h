#pragma once
#include <canard.h>
#include <stdbool.h>
#include <stdint.h>

#define REMOTEID_SECURECOMMAND_RESPONSE_MAX_SIZE 230
#define REMOTEID_SECURECOMMAND_RESPONSE_SIGNATURE (0x2035E2EAE81D2E53ULL)
#define REMOTEID_SECURECOMMAND_RESPONSE_ID 64

#define REMOTEID_SECURECOMMAND_RESPONSE_RESULT_ACCEPTED 0
#define REMOTEID_SECURECOMMAND_RESPONSE_RESULT_TEMPORARILY_REJECTED 1
#define REMOTEID_SECURECOMMAND_RESPONSE_RESULT_DENIED 2
#define REMOTEID_SECURECOMMAND_RESPONSE_RESULT_UNSUPPORTED 3
#define REMOTEID_SECURECOMMAND_RESPONSE_RESULT_FAILED 4

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class remoteid_SecureCommand_cxx_iface;
#endif

struct remoteid_SecureCommandResponse
{
#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = remoteid_SecureCommand_cxx_iface;
#endif
    uint32_t sequence;
    uint32_t operation;
    uint8_t result;
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

    uint32_t remoteid_SecureCommandResponse_encode(struct remoteid_SecureCommandResponse *msg, uint8_t *buffer
#if CANARD_ENABLE_TAO_OPTION
                                                   ,
                                                   bool tao
#endif
    );
    bool remoteid_SecureCommandResponse_decode(const CanardRxTransfer *transfer, struct remoteid_SecureCommandResponse *msg);

#if defined(CANARD_DSDLC_INTERNAL)
    static inline void _remoteid_SecureCommandResponse_encode(uint8_t *buffer, uint32_t *bit_ofs, struct remoteid_SecureCommandResponse *msg, bool tao);
    static inline bool _remoteid_SecureCommandResponse_decode(const CanardRxTransfer *transfer, uint32_t *bit_ofs, struct remoteid_SecureCommandResponse *msg, bool tao);
    void _remoteid_SecureCommandResponse_encode(uint8_t *buffer, uint32_t *bit_ofs, struct remoteid_SecureCommandResponse *msg, bool tao)
    {
        (void) buffer;
        (void) bit_ofs;
        (void) msg;
        (void) tao;

        canardEncodeScalar(buffer, *bit_ofs, 32, &msg->sequence);
        *bit_ofs += 32;
        canardEncodeScalar(buffer, *bit_ofs, 32, &msg->operation);
        *bit_ofs += 32;
        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->result);
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
     decode remoteid_SecureCommandResponse, return true on failure, false on success
    */
    bool _remoteid_SecureCommandResponse_decode(const CanardRxTransfer *transfer, uint32_t *bit_ofs, struct remoteid_SecureCommandResponse *msg, bool tao)
    {
        (void) transfer;
        (void) bit_ofs;
        (void) msg;
        (void) tao;
        canardDecodeScalar(transfer, *bit_ofs, 32, false, &msg->sequence);
        *bit_ofs += 32;

        canardDecodeScalar(transfer, *bit_ofs, 32, false, &msg->operation);
        *bit_ofs += 32;

        canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->result);
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
    struct remoteid_SecureCommandResponse sample_remoteid_SecureCommandResponse_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
#endif
#endif
