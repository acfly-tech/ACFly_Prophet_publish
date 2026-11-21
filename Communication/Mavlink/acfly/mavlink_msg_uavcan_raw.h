#pragma once
// MESSAGE UAVCAN_RAW PACKING

#define MAVLINK_MSG_ID_UAVCAN_RAW 312


typedef struct __mavlink_uavcan_raw_t {
 uint32_t can_id; /*<  CAN id, bit0-bit28, bit29: [0:standard frame, 1:extended frame], bit30: [1:require feedback].*/
 uint8_t target_system; /*<  System ID.*/
 uint8_t target_component; /*<  Component ID.*/
 uint8_t sequence; /*<  Message sequence, auto +1 for every CAN raw frame.*/
 uint8_t can_port; /*<  CAN port id, e.g. 0, 1, 2 (0xFF send to all ports).*/
 uint8_t payloadLen; /*<  Payload length.*/
 uint8_t payload[64]; /*<  CAN payload.*/
} mavlink_uavcan_raw_t;

#define MAVLINK_MSG_ID_UAVCAN_RAW_LEN 73
#define MAVLINK_MSG_ID_UAVCAN_RAW_MIN_LEN 73
#define MAVLINK_MSG_ID_312_LEN 73
#define MAVLINK_MSG_ID_312_MIN_LEN 73

#define MAVLINK_MSG_ID_UAVCAN_RAW_CRC 31
#define MAVLINK_MSG_ID_312_CRC 31

#define MAVLINK_MSG_UAVCAN_RAW_FIELD_PAYLOAD_LEN 64

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_UAVCAN_RAW { \
    312, \
    "UAVCAN_RAW", \
    7, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_uavcan_raw_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_uavcan_raw_t, target_component) }, \
         { "sequence", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_uavcan_raw_t, sequence) }, \
         { "can_port", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_uavcan_raw_t, can_port) }, \
         { "payloadLen", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_uavcan_raw_t, payloadLen) }, \
         { "can_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_uavcan_raw_t, can_id) }, \
         { "payload", NULL, MAVLINK_TYPE_UINT8_T, 64, 9, offsetof(mavlink_uavcan_raw_t, payload) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_UAVCAN_RAW { \
    "UAVCAN_RAW", \
    7, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_uavcan_raw_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_uavcan_raw_t, target_component) }, \
         { "sequence", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_uavcan_raw_t, sequence) }, \
         { "can_port", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_uavcan_raw_t, can_port) }, \
         { "payloadLen", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_uavcan_raw_t, payloadLen) }, \
         { "can_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_uavcan_raw_t, can_id) }, \
         { "payload", NULL, MAVLINK_TYPE_UINT8_T, 64, 9, offsetof(mavlink_uavcan_raw_t, payload) }, \
         } \
}
#endif

/**
 * @brief Pack a uavcan_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param sequence  Message sequence, auto +1 for every CAN raw frame.
 * @param can_port  CAN port id, e.g. 0, 1, 2 (0xFF send to all ports).
 * @param payloadLen  Payload length.
 * @param can_id  CAN id, bit0-bit28, bit29: [0:standard frame, 1:extended frame], bit30: [1:require feedback].
 * @param payload  CAN payload.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavcan_raw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t sequence, uint8_t can_port, uint8_t payloadLen, uint32_t can_id, const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVCAN_RAW_LEN];
    _mav_put_uint32_t(buf, 0, can_id);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 6, sequence);
    _mav_put_uint8_t(buf, 7, can_port);
    _mav_put_uint8_t(buf, 8, payloadLen);
    _mav_put_uint8_t_array(buf, 9, payload, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVCAN_RAW_LEN);
#else
    mavlink_uavcan_raw_t packet;
    packet.can_id = can_id;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.sequence = sequence;
    packet.can_port = can_port;
    packet.payloadLen = payloadLen;
    mav_array_assign_uint8_t(packet.payload, payload, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVCAN_RAW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVCAN_RAW;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UAVCAN_RAW_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_RAW_LEN, MAVLINK_MSG_ID_UAVCAN_RAW_CRC);
}

/**
 * @brief Pack a uavcan_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param sequence  Message sequence, auto +1 for every CAN raw frame.
 * @param can_port  CAN port id, e.g. 0, 1, 2 (0xFF send to all ports).
 * @param payloadLen  Payload length.
 * @param can_id  CAN id, bit0-bit28, bit29: [0:standard frame, 1:extended frame], bit30: [1:require feedback].
 * @param payload  CAN payload.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavcan_raw_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t sequence, uint8_t can_port, uint8_t payloadLen, uint32_t can_id, const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVCAN_RAW_LEN];
    _mav_put_uint32_t(buf, 0, can_id);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 6, sequence);
    _mav_put_uint8_t(buf, 7, can_port);
    _mav_put_uint8_t(buf, 8, payloadLen);
    _mav_put_uint8_t_array(buf, 9, payload, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVCAN_RAW_LEN);
#else
    mavlink_uavcan_raw_t packet;
    packet.can_id = can_id;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.sequence = sequence;
    packet.can_port = can_port;
    packet.payloadLen = payloadLen;
    mav_array_memcpy(packet.payload, payload, sizeof(uint8_t)*64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVCAN_RAW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVCAN_RAW;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UAVCAN_RAW_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_RAW_LEN, MAVLINK_MSG_ID_UAVCAN_RAW_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UAVCAN_RAW_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_RAW_LEN);
#endif
}

/**
 * @brief Pack a uavcan_raw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param sequence  Message sequence, auto +1 for every CAN raw frame.
 * @param can_port  CAN port id, e.g. 0, 1, 2 (0xFF send to all ports).
 * @param payloadLen  Payload length.
 * @param can_id  CAN id, bit0-bit28, bit29: [0:standard frame, 1:extended frame], bit30: [1:require feedback].
 * @param payload  CAN payload.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavcan_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint8_t sequence,uint8_t can_port,uint8_t payloadLen,uint32_t can_id,const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVCAN_RAW_LEN];
    _mav_put_uint32_t(buf, 0, can_id);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 6, sequence);
    _mav_put_uint8_t(buf, 7, can_port);
    _mav_put_uint8_t(buf, 8, payloadLen);
    _mav_put_uint8_t_array(buf, 9, payload, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVCAN_RAW_LEN);
#else
    mavlink_uavcan_raw_t packet;
    packet.can_id = can_id;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.sequence = sequence;
    packet.can_port = can_port;
    packet.payloadLen = payloadLen;
    mav_array_assign_uint8_t(packet.payload, payload, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVCAN_RAW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVCAN_RAW;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UAVCAN_RAW_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_RAW_LEN, MAVLINK_MSG_ID_UAVCAN_RAW_CRC);
}

/**
 * @brief Encode a uavcan_raw struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param uavcan_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavcan_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_uavcan_raw_t* uavcan_raw)
{
    return mavlink_msg_uavcan_raw_pack(system_id, component_id, msg, uavcan_raw->target_system, uavcan_raw->target_component, uavcan_raw->sequence, uavcan_raw->can_port, uavcan_raw->payloadLen, uavcan_raw->can_id, uavcan_raw->payload);
}

/**
 * @brief Encode a uavcan_raw struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param uavcan_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavcan_raw_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_uavcan_raw_t* uavcan_raw)
{
    return mavlink_msg_uavcan_raw_pack_chan(system_id, component_id, chan, msg, uavcan_raw->target_system, uavcan_raw->target_component, uavcan_raw->sequence, uavcan_raw->can_port, uavcan_raw->payloadLen, uavcan_raw->can_id, uavcan_raw->payload);
}

/**
 * @brief Encode a uavcan_raw struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param uavcan_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavcan_raw_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_uavcan_raw_t* uavcan_raw)
{
    return mavlink_msg_uavcan_raw_pack_status(system_id, component_id, _status, msg,  uavcan_raw->target_system, uavcan_raw->target_component, uavcan_raw->sequence, uavcan_raw->can_port, uavcan_raw->payloadLen, uavcan_raw->can_id, uavcan_raw->payload);
}

/**
 * @brief Send a uavcan_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param sequence  Message sequence, auto +1 for every CAN raw frame.
 * @param can_port  CAN port id, e.g. 0, 1, 2 (0xFF send to all ports).
 * @param payloadLen  Payload length.
 * @param can_id  CAN id, bit0-bit28, bit29: [0:standard frame, 1:extended frame], bit30: [1:require feedback].
 * @param payload  CAN payload.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_uavcan_raw_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t sequence, uint8_t can_port, uint8_t payloadLen, uint32_t can_id, const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVCAN_RAW_LEN];
    _mav_put_uint32_t(buf, 0, can_id);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 6, sequence);
    _mav_put_uint8_t(buf, 7, can_port);
    _mav_put_uint8_t(buf, 8, payloadLen);
    _mav_put_uint8_t_array(buf, 9, payload, 64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVCAN_RAW, buf, MAVLINK_MSG_ID_UAVCAN_RAW_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_RAW_LEN, MAVLINK_MSG_ID_UAVCAN_RAW_CRC);
#else
    mavlink_uavcan_raw_t packet;
    packet.can_id = can_id;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.sequence = sequence;
    packet.can_port = can_port;
    packet.payloadLen = payloadLen;
    mav_array_assign_uint8_t(packet.payload, payload, 64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVCAN_RAW, (const char *)&packet, MAVLINK_MSG_ID_UAVCAN_RAW_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_RAW_LEN, MAVLINK_MSG_ID_UAVCAN_RAW_CRC);
#endif
}

/**
 * @brief Send a uavcan_raw message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_uavcan_raw_send_struct(mavlink_channel_t chan, const mavlink_uavcan_raw_t* uavcan_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_uavcan_raw_send(chan, uavcan_raw->target_system, uavcan_raw->target_component, uavcan_raw->sequence, uavcan_raw->can_port, uavcan_raw->payloadLen, uavcan_raw->can_id, uavcan_raw->payload);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVCAN_RAW, (const char *)uavcan_raw, MAVLINK_MSG_ID_UAVCAN_RAW_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_RAW_LEN, MAVLINK_MSG_ID_UAVCAN_RAW_CRC);
#endif
}

#if MAVLINK_MSG_ID_UAVCAN_RAW_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_uavcan_raw_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t sequence, uint8_t can_port, uint8_t payloadLen, uint32_t can_id, const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, can_id);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 6, sequence);
    _mav_put_uint8_t(buf, 7, can_port);
    _mav_put_uint8_t(buf, 8, payloadLen);
    _mav_put_uint8_t_array(buf, 9, payload, 64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVCAN_RAW, buf, MAVLINK_MSG_ID_UAVCAN_RAW_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_RAW_LEN, MAVLINK_MSG_ID_UAVCAN_RAW_CRC);
#else
    mavlink_uavcan_raw_t *packet = (mavlink_uavcan_raw_t *)msgbuf;
    packet->can_id = can_id;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->sequence = sequence;
    packet->can_port = can_port;
    packet->payloadLen = payloadLen;
    mav_array_assign_uint8_t(packet->payload, payload, 64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVCAN_RAW, (const char *)packet, MAVLINK_MSG_ID_UAVCAN_RAW_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_RAW_LEN, MAVLINK_MSG_ID_UAVCAN_RAW_CRC);
#endif
}
#endif

#endif

// MESSAGE UAVCAN_RAW UNPACKING


/**
 * @brief Get field target_system from uavcan_raw message
 *
 * @return  System ID.
 */
static inline uint8_t mavlink_msg_uavcan_raw_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field target_component from uavcan_raw message
 *
 * @return  Component ID.
 */
static inline uint8_t mavlink_msg_uavcan_raw_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field sequence from uavcan_raw message
 *
 * @return  Message sequence, auto +1 for every CAN raw frame.
 */
static inline uint8_t mavlink_msg_uavcan_raw_get_sequence(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field can_port from uavcan_raw message
 *
 * @return  CAN port id, e.g. 0, 1, 2 (0xFF send to all ports).
 */
static inline uint8_t mavlink_msg_uavcan_raw_get_can_port(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field payloadLen from uavcan_raw message
 *
 * @return  Payload length.
 */
static inline uint8_t mavlink_msg_uavcan_raw_get_payloadLen(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field can_id from uavcan_raw message
 *
 * @return  CAN id, bit0-bit28, bit29: [0:standard frame, 1:extended frame], bit30: [1:require feedback].
 */
static inline uint32_t mavlink_msg_uavcan_raw_get_can_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field payload from uavcan_raw message
 *
 * @return  CAN payload.
 */
static inline uint16_t mavlink_msg_uavcan_raw_get_payload(const mavlink_message_t* msg, uint8_t *payload)
{
    return _MAV_RETURN_uint8_t_array(msg, payload, 64,  9);
}

/**
 * @brief Decode a uavcan_raw message into a struct
 *
 * @param msg The message to decode
 * @param uavcan_raw C-struct to decode the message contents into
 */
static inline void mavlink_msg_uavcan_raw_decode(const mavlink_message_t* msg, mavlink_uavcan_raw_t* uavcan_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    uavcan_raw->can_id = mavlink_msg_uavcan_raw_get_can_id(msg);
    uavcan_raw->target_system = mavlink_msg_uavcan_raw_get_target_system(msg);
    uavcan_raw->target_component = mavlink_msg_uavcan_raw_get_target_component(msg);
    uavcan_raw->sequence = mavlink_msg_uavcan_raw_get_sequence(msg);
    uavcan_raw->can_port = mavlink_msg_uavcan_raw_get_can_port(msg);
    uavcan_raw->payloadLen = mavlink_msg_uavcan_raw_get_payloadLen(msg);
    mavlink_msg_uavcan_raw_get_payload(msg, uavcan_raw->payload);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_UAVCAN_RAW_LEN? msg->len : MAVLINK_MSG_ID_UAVCAN_RAW_LEN;
        memset(uavcan_raw, 0, MAVLINK_MSG_ID_UAVCAN_RAW_LEN);
    memcpy(uavcan_raw, _MAV_PAYLOAD(msg), len);
#endif
}
