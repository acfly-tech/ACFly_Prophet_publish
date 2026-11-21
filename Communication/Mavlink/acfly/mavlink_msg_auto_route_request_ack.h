#pragma once
// MESSAGE AUTO_ROUTE_REQUEST_ACK PACKING

#define MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK 415


typedef struct __mavlink_auto_route_request_ack_t {
 uint8_t target_system; /*<  System ID.*/
 uint8_t target_component; /*<  Component ID.*/
 uint8_t type; /*<  0: takeoff, 1: return to launch.*/
 uint8_t mode; /*<  0: efficiency mode, 1: simple mode.*/
 uint8_t progress; /*< [%] 0-100.*/
} mavlink_auto_route_request_ack_t;

#define MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_LEN 5
#define MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_MIN_LEN 5
#define MAVLINK_MSG_ID_415_LEN 5
#define MAVLINK_MSG_ID_415_MIN_LEN 5

#define MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_CRC 80
#define MAVLINK_MSG_ID_415_CRC 80



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AUTO_ROUTE_REQUEST_ACK { \
    415, \
    "AUTO_ROUTE_REQUEST_ACK", \
    5, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_auto_route_request_ack_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_auto_route_request_ack_t, target_component) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_auto_route_request_ack_t, type) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_auto_route_request_ack_t, mode) }, \
         { "progress", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_auto_route_request_ack_t, progress) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AUTO_ROUTE_REQUEST_ACK { \
    "AUTO_ROUTE_REQUEST_ACK", \
    5, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_auto_route_request_ack_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_auto_route_request_ack_t, target_component) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_auto_route_request_ack_t, type) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_auto_route_request_ack_t, mode) }, \
         { "progress", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_auto_route_request_ack_t, progress) }, \
         } \
}
#endif

/**
 * @brief Pack a auto_route_request_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param type  0: takeoff, 1: return to launch.
 * @param mode  0: efficiency mode, 1: simple mode.
 * @param progress [%] 0-100.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_auto_route_request_ack_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t type, uint8_t mode, uint8_t progress)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, type);
    _mav_put_uint8_t(buf, 3, mode);
    _mav_put_uint8_t(buf, 4, progress);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_LEN);
#else
    mavlink_auto_route_request_ack_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.type = type;
    packet.mode = mode;
    packet.progress = progress;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_MIN_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_CRC);
}

/**
 * @brief Pack a auto_route_request_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param type  0: takeoff, 1: return to launch.
 * @param mode  0: efficiency mode, 1: simple mode.
 * @param progress [%] 0-100.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_auto_route_request_ack_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t type, uint8_t mode, uint8_t progress)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, type);
    _mav_put_uint8_t(buf, 3, mode);
    _mav_put_uint8_t(buf, 4, progress);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_LEN);
#else
    mavlink_auto_route_request_ack_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.type = type;
    packet.mode = mode;
    packet.progress = progress;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_MIN_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_MIN_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_LEN);
#endif
}

/**
 * @brief Pack a auto_route_request_ack message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param type  0: takeoff, 1: return to launch.
 * @param mode  0: efficiency mode, 1: simple mode.
 * @param progress [%] 0-100.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_auto_route_request_ack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint8_t type,uint8_t mode,uint8_t progress)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, type);
    _mav_put_uint8_t(buf, 3, mode);
    _mav_put_uint8_t(buf, 4, progress);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_LEN);
#else
    mavlink_auto_route_request_ack_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.type = type;
    packet.mode = mode;
    packet.progress = progress;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_MIN_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_CRC);
}

/**
 * @brief Encode a auto_route_request_ack struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param auto_route_request_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_auto_route_request_ack_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_auto_route_request_ack_t* auto_route_request_ack)
{
    return mavlink_msg_auto_route_request_ack_pack(system_id, component_id, msg, auto_route_request_ack->target_system, auto_route_request_ack->target_component, auto_route_request_ack->type, auto_route_request_ack->mode, auto_route_request_ack->progress);
}

/**
 * @brief Encode a auto_route_request_ack struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param auto_route_request_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_auto_route_request_ack_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_auto_route_request_ack_t* auto_route_request_ack)
{
    return mavlink_msg_auto_route_request_ack_pack_chan(system_id, component_id, chan, msg, auto_route_request_ack->target_system, auto_route_request_ack->target_component, auto_route_request_ack->type, auto_route_request_ack->mode, auto_route_request_ack->progress);
}

/**
 * @brief Encode a auto_route_request_ack struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param auto_route_request_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_auto_route_request_ack_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_auto_route_request_ack_t* auto_route_request_ack)
{
    return mavlink_msg_auto_route_request_ack_pack_status(system_id, component_id, _status, msg,  auto_route_request_ack->target_system, auto_route_request_ack->target_component, auto_route_request_ack->type, auto_route_request_ack->mode, auto_route_request_ack->progress);
}

/**
 * @brief Send a auto_route_request_ack message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param type  0: takeoff, 1: return to launch.
 * @param mode  0: efficiency mode, 1: simple mode.
 * @param progress [%] 0-100.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_auto_route_request_ack_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t type, uint8_t mode, uint8_t progress)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, type);
    _mav_put_uint8_t(buf, 3, mode);
    _mav_put_uint8_t(buf, 4, progress);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK, buf, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_MIN_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_CRC);
#else
    mavlink_auto_route_request_ack_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.type = type;
    packet.mode = mode;
    packet.progress = progress;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK, (const char *)&packet, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_MIN_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_CRC);
#endif
}

/**
 * @brief Send a auto_route_request_ack message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_auto_route_request_ack_send_struct(mavlink_channel_t chan, const mavlink_auto_route_request_ack_t* auto_route_request_ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_auto_route_request_ack_send(chan, auto_route_request_ack->target_system, auto_route_request_ack->target_component, auto_route_request_ack->type, auto_route_request_ack->mode, auto_route_request_ack->progress);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK, (const char *)auto_route_request_ack, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_MIN_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_CRC);
#endif
}

#if MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_auto_route_request_ack_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t type, uint8_t mode, uint8_t progress)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, type);
    _mav_put_uint8_t(buf, 3, mode);
    _mav_put_uint8_t(buf, 4, progress);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK, buf, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_MIN_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_CRC);
#else
    mavlink_auto_route_request_ack_t *packet = (mavlink_auto_route_request_ack_t *)msgbuf;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->type = type;
    packet->mode = mode;
    packet->progress = progress;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK, (const char *)packet, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_MIN_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_CRC);
#endif
}
#endif

#endif

// MESSAGE AUTO_ROUTE_REQUEST_ACK UNPACKING


/**
 * @brief Get field target_system from auto_route_request_ack message
 *
 * @return  System ID.
 */
static inline uint8_t mavlink_msg_auto_route_request_ack_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_component from auto_route_request_ack message
 *
 * @return  Component ID.
 */
static inline uint8_t mavlink_msg_auto_route_request_ack_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field type from auto_route_request_ack message
 *
 * @return  0: takeoff, 1: return to launch.
 */
static inline uint8_t mavlink_msg_auto_route_request_ack_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field mode from auto_route_request_ack message
 *
 * @return  0: efficiency mode, 1: simple mode.
 */
static inline uint8_t mavlink_msg_auto_route_request_ack_get_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field progress from auto_route_request_ack message
 *
 * @return [%] 0-100.
 */
static inline uint8_t mavlink_msg_auto_route_request_ack_get_progress(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Decode a auto_route_request_ack message into a struct
 *
 * @param msg The message to decode
 * @param auto_route_request_ack C-struct to decode the message contents into
 */
static inline void mavlink_msg_auto_route_request_ack_decode(const mavlink_message_t* msg, mavlink_auto_route_request_ack_t* auto_route_request_ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    auto_route_request_ack->target_system = mavlink_msg_auto_route_request_ack_get_target_system(msg);
    auto_route_request_ack->target_component = mavlink_msg_auto_route_request_ack_get_target_component(msg);
    auto_route_request_ack->type = mavlink_msg_auto_route_request_ack_get_type(msg);
    auto_route_request_ack->mode = mavlink_msg_auto_route_request_ack_get_mode(msg);
    auto_route_request_ack->progress = mavlink_msg_auto_route_request_ack_get_progress(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_LEN? msg->len : MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_LEN;
        memset(auto_route_request_ack, 0, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_LEN);
    memcpy(auto_route_request_ack, _MAV_PAYLOAD(msg), len);
#endif
}
