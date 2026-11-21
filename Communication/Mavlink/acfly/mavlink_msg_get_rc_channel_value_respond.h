#pragma once
// MESSAGE GET_RC_CHANNEL_VALUE_RESPOND PACKING

#define MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND 21169


typedef struct __mavlink_get_rc_channel_value_respond_t {
 uint16_t channel[16]; /*<  RC channels 1-16.*/
 uint16_t channelExt[16]; /*<  Extended RC channels 17-32.*/
} mavlink_get_rc_channel_value_respond_t;

#define MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_LEN 64
#define MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_MIN_LEN 64
#define MAVLINK_MSG_ID_21169_LEN 64
#define MAVLINK_MSG_ID_21169_MIN_LEN 64

#define MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_CRC 178
#define MAVLINK_MSG_ID_21169_CRC 178

#define MAVLINK_MSG_GET_RC_CHANNEL_VALUE_RESPOND_FIELD_CHANNEL_LEN 16
#define MAVLINK_MSG_GET_RC_CHANNEL_VALUE_RESPOND_FIELD_CHANNELEXT_LEN 16

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GET_RC_CHANNEL_VALUE_RESPOND { \
    21169, \
    "GET_RC_CHANNEL_VALUE_RESPOND", \
    2, \
    {  { "channel", NULL, MAVLINK_TYPE_UINT16_T, 16, 0, offsetof(mavlink_get_rc_channel_value_respond_t, channel) }, \
         { "channelExt", NULL, MAVLINK_TYPE_UINT16_T, 16, 32, offsetof(mavlink_get_rc_channel_value_respond_t, channelExt) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GET_RC_CHANNEL_VALUE_RESPOND { \
    "GET_RC_CHANNEL_VALUE_RESPOND", \
    2, \
    {  { "channel", NULL, MAVLINK_TYPE_UINT16_T, 16, 0, offsetof(mavlink_get_rc_channel_value_respond_t, channel) }, \
         { "channelExt", NULL, MAVLINK_TYPE_UINT16_T, 16, 32, offsetof(mavlink_get_rc_channel_value_respond_t, channelExt) }, \
         } \
}
#endif

/**
 * @brief Pack a get_rc_channel_value_respond message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param channel  RC channels 1-16.
 * @param channelExt  Extended RC channels 17-32.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_get_rc_channel_value_respond_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const uint16_t *channel, const uint16_t *channelExt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_LEN];

    _mav_put_uint16_t_array(buf, 0, channel, 16);
    _mav_put_uint16_t_array(buf, 32, channelExt, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_LEN);
#else
    mavlink_get_rc_channel_value_respond_t packet;

    mav_array_assign_uint16_t(packet.channel, channel, 16);
    mav_array_assign_uint16_t(packet.channelExt, channelExt, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_MIN_LEN, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_LEN, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_CRC);
}

/**
 * @brief Pack a get_rc_channel_value_respond message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param channel  RC channels 1-16.
 * @param channelExt  Extended RC channels 17-32.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_get_rc_channel_value_respond_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               const uint16_t *channel, const uint16_t *channelExt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_LEN];

    _mav_put_uint16_t_array(buf, 0, channel, 16);
    _mav_put_uint16_t_array(buf, 32, channelExt, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_LEN);
#else
    mavlink_get_rc_channel_value_respond_t packet;

    mav_array_memcpy(packet.channel, channel, sizeof(uint16_t)*16);
    mav_array_memcpy(packet.channelExt, channelExt, sizeof(uint16_t)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_MIN_LEN, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_LEN, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_MIN_LEN, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_LEN);
#endif
}

/**
 * @brief Pack a get_rc_channel_value_respond message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param channel  RC channels 1-16.
 * @param channelExt  Extended RC channels 17-32.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_get_rc_channel_value_respond_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const uint16_t *channel,const uint16_t *channelExt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_LEN];

    _mav_put_uint16_t_array(buf, 0, channel, 16);
    _mav_put_uint16_t_array(buf, 32, channelExt, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_LEN);
#else
    mavlink_get_rc_channel_value_respond_t packet;

    mav_array_assign_uint16_t(packet.channel, channel, 16);
    mav_array_assign_uint16_t(packet.channelExt, channelExt, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_MIN_LEN, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_LEN, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_CRC);
}

/**
 * @brief Encode a get_rc_channel_value_respond struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param get_rc_channel_value_respond C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_get_rc_channel_value_respond_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_get_rc_channel_value_respond_t* get_rc_channel_value_respond)
{
    return mavlink_msg_get_rc_channel_value_respond_pack(system_id, component_id, msg, get_rc_channel_value_respond->channel, get_rc_channel_value_respond->channelExt);
}

/**
 * @brief Encode a get_rc_channel_value_respond struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param get_rc_channel_value_respond C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_get_rc_channel_value_respond_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_get_rc_channel_value_respond_t* get_rc_channel_value_respond)
{
    return mavlink_msg_get_rc_channel_value_respond_pack_chan(system_id, component_id, chan, msg, get_rc_channel_value_respond->channel, get_rc_channel_value_respond->channelExt);
}

/**
 * @brief Encode a get_rc_channel_value_respond struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param get_rc_channel_value_respond C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_get_rc_channel_value_respond_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_get_rc_channel_value_respond_t* get_rc_channel_value_respond)
{
    return mavlink_msg_get_rc_channel_value_respond_pack_status(system_id, component_id, _status, msg,  get_rc_channel_value_respond->channel, get_rc_channel_value_respond->channelExt);
}

/**
 * @brief Send a get_rc_channel_value_respond message
 * @param chan MAVLink channel to send the message
 *
 * @param channel  RC channels 1-16.
 * @param channelExt  Extended RC channels 17-32.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_get_rc_channel_value_respond_send(mavlink_channel_t chan, const uint16_t *channel, const uint16_t *channelExt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_LEN];

    _mav_put_uint16_t_array(buf, 0, channel, 16);
    _mav_put_uint16_t_array(buf, 32, channelExt, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND, buf, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_MIN_LEN, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_LEN, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_CRC);
#else
    mavlink_get_rc_channel_value_respond_t packet;

    mav_array_assign_uint16_t(packet.channel, channel, 16);
    mav_array_assign_uint16_t(packet.channelExt, channelExt, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND, (const char *)&packet, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_MIN_LEN, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_LEN, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_CRC);
#endif
}

/**
 * @brief Send a get_rc_channel_value_respond message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_get_rc_channel_value_respond_send_struct(mavlink_channel_t chan, const mavlink_get_rc_channel_value_respond_t* get_rc_channel_value_respond)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_get_rc_channel_value_respond_send(chan, get_rc_channel_value_respond->channel, get_rc_channel_value_respond->channelExt);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND, (const char *)get_rc_channel_value_respond, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_MIN_LEN, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_LEN, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_CRC);
#endif
}

#if MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_get_rc_channel_value_respond_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const uint16_t *channel, const uint16_t *channelExt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_uint16_t_array(buf, 0, channel, 16);
    _mav_put_uint16_t_array(buf, 32, channelExt, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND, buf, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_MIN_LEN, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_LEN, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_CRC);
#else
    mavlink_get_rc_channel_value_respond_t *packet = (mavlink_get_rc_channel_value_respond_t *)msgbuf;

    mav_array_assign_uint16_t(packet->channel, channel, 16);
    mav_array_assign_uint16_t(packet->channelExt, channelExt, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND, (const char *)packet, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_MIN_LEN, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_LEN, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_CRC);
#endif
}
#endif

#endif

// MESSAGE GET_RC_CHANNEL_VALUE_RESPOND UNPACKING


/**
 * @brief Get field channel from get_rc_channel_value_respond message
 *
 * @return  RC channels 1-16.
 */
static inline uint16_t mavlink_msg_get_rc_channel_value_respond_get_channel(const mavlink_message_t* msg, uint16_t *channel)
{
    return _MAV_RETURN_uint16_t_array(msg, channel, 16,  0);
}

/**
 * @brief Get field channelExt from get_rc_channel_value_respond message
 *
 * @return  Extended RC channels 17-32.
 */
static inline uint16_t mavlink_msg_get_rc_channel_value_respond_get_channelExt(const mavlink_message_t* msg, uint16_t *channelExt)
{
    return _MAV_RETURN_uint16_t_array(msg, channelExt, 16,  32);
}

/**
 * @brief Decode a get_rc_channel_value_respond message into a struct
 *
 * @param msg The message to decode
 * @param get_rc_channel_value_respond C-struct to decode the message contents into
 */
static inline void mavlink_msg_get_rc_channel_value_respond_decode(const mavlink_message_t* msg, mavlink_get_rc_channel_value_respond_t* get_rc_channel_value_respond)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_get_rc_channel_value_respond_get_channel(msg, get_rc_channel_value_respond->channel);
    mavlink_msg_get_rc_channel_value_respond_get_channelExt(msg, get_rc_channel_value_respond->channelExt);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_LEN? msg->len : MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_LEN;
        memset(get_rc_channel_value_respond, 0, MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_LEN);
    memcpy(get_rc_channel_value_respond, _MAV_PAYLOAD(msg), len);
#endif
}
