#pragma once
// MESSAGE ACFLY_MISSION_STATUS PACKING

#define MAVLINK_MSG_ID_ACFLY_MISSION_STATUS 205


typedef struct __mavlink_acfly_mission_status_t {
 float heading; /*< [deg] */
 float vector_x; /*< [m] */
 float vector_y; /*< [m] */
 float vector_z; /*< [m] */
 float distance; /*< [m] */
 float trigger_interval; /*< [m] */
 float speed; /*< [m/s] */
 float radius; /*< [m] */
 uint16_t seq; /*<  Current mission sequence number (pairs with MISSION_CURRENT.seq).*/
 uint16_t resumeSeq; /*<  */
 uint16_t flag; /*<  */
} mavlink_acfly_mission_status_t;

#define MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_LEN 38
#define MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_MIN_LEN 38
#define MAVLINK_MSG_ID_205_LEN 38
#define MAVLINK_MSG_ID_205_MIN_LEN 38

#define MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_CRC 86
#define MAVLINK_MSG_ID_205_CRC 86



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ACFLY_MISSION_STATUS { \
    205, \
    "ACFLY_MISSION_STATUS", \
    11, \
    {  { "seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_acfly_mission_status_t, seq) }, \
         { "resumeSeq", NULL, MAVLINK_TYPE_UINT16_T, 0, 34, offsetof(mavlink_acfly_mission_status_t, resumeSeq) }, \
         { "flag", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_acfly_mission_status_t, flag) }, \
         { "heading", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_acfly_mission_status_t, heading) }, \
         { "vector_x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_acfly_mission_status_t, vector_x) }, \
         { "vector_y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_acfly_mission_status_t, vector_y) }, \
         { "vector_z", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_acfly_mission_status_t, vector_z) }, \
         { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_acfly_mission_status_t, distance) }, \
         { "trigger_interval", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_acfly_mission_status_t, trigger_interval) }, \
         { "speed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_acfly_mission_status_t, speed) }, \
         { "radius", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_acfly_mission_status_t, radius) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ACFLY_MISSION_STATUS { \
    "ACFLY_MISSION_STATUS", \
    11, \
    {  { "seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_acfly_mission_status_t, seq) }, \
         { "resumeSeq", NULL, MAVLINK_TYPE_UINT16_T, 0, 34, offsetof(mavlink_acfly_mission_status_t, resumeSeq) }, \
         { "flag", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_acfly_mission_status_t, flag) }, \
         { "heading", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_acfly_mission_status_t, heading) }, \
         { "vector_x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_acfly_mission_status_t, vector_x) }, \
         { "vector_y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_acfly_mission_status_t, vector_y) }, \
         { "vector_z", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_acfly_mission_status_t, vector_z) }, \
         { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_acfly_mission_status_t, distance) }, \
         { "trigger_interval", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_acfly_mission_status_t, trigger_interval) }, \
         { "speed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_acfly_mission_status_t, speed) }, \
         { "radius", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_acfly_mission_status_t, radius) }, \
         } \
}
#endif

/**
 * @brief Pack a acfly_mission_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param seq  Current mission sequence number (pairs with MISSION_CURRENT.seq).
 * @param resumeSeq  
 * @param flag  
 * @param heading [deg] 
 * @param vector_x [m] 
 * @param vector_y [m] 
 * @param vector_z [m] 
 * @param distance [m] 
 * @param trigger_interval [m] 
 * @param speed [m/s] 
 * @param radius [m] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_acfly_mission_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t seq, uint16_t resumeSeq, uint16_t flag, float heading, float vector_x, float vector_y, float vector_z, float distance, float trigger_interval, float speed, float radius)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_LEN];
    _mav_put_float(buf, 0, heading);
    _mav_put_float(buf, 4, vector_x);
    _mav_put_float(buf, 8, vector_y);
    _mav_put_float(buf, 12, vector_z);
    _mav_put_float(buf, 16, distance);
    _mav_put_float(buf, 20, trigger_interval);
    _mav_put_float(buf, 24, speed);
    _mav_put_float(buf, 28, radius);
    _mav_put_uint16_t(buf, 32, seq);
    _mav_put_uint16_t(buf, 34, resumeSeq);
    _mav_put_uint16_t(buf, 36, flag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_LEN);
#else
    mavlink_acfly_mission_status_t packet;
    packet.heading = heading;
    packet.vector_x = vector_x;
    packet.vector_y = vector_y;
    packet.vector_z = vector_z;
    packet.distance = distance;
    packet.trigger_interval = trigger_interval;
    packet.speed = speed;
    packet.radius = radius;
    packet.seq = seq;
    packet.resumeSeq = resumeSeq;
    packet.flag = flag;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACFLY_MISSION_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_LEN, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_CRC);
}

/**
 * @brief Pack a acfly_mission_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param seq  Current mission sequence number (pairs with MISSION_CURRENT.seq).
 * @param resumeSeq  
 * @param flag  
 * @param heading [deg] 
 * @param vector_x [m] 
 * @param vector_y [m] 
 * @param vector_z [m] 
 * @param distance [m] 
 * @param trigger_interval [m] 
 * @param speed [m/s] 
 * @param radius [m] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_acfly_mission_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint16_t seq, uint16_t resumeSeq, uint16_t flag, float heading, float vector_x, float vector_y, float vector_z, float distance, float trigger_interval, float speed, float radius)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_LEN];
    _mav_put_float(buf, 0, heading);
    _mav_put_float(buf, 4, vector_x);
    _mav_put_float(buf, 8, vector_y);
    _mav_put_float(buf, 12, vector_z);
    _mav_put_float(buf, 16, distance);
    _mav_put_float(buf, 20, trigger_interval);
    _mav_put_float(buf, 24, speed);
    _mav_put_float(buf, 28, radius);
    _mav_put_uint16_t(buf, 32, seq);
    _mav_put_uint16_t(buf, 34, resumeSeq);
    _mav_put_uint16_t(buf, 36, flag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_LEN);
#else
    mavlink_acfly_mission_status_t packet;
    packet.heading = heading;
    packet.vector_x = vector_x;
    packet.vector_y = vector_y;
    packet.vector_z = vector_z;
    packet.distance = distance;
    packet.trigger_interval = trigger_interval;
    packet.speed = speed;
    packet.radius = radius;
    packet.seq = seq;
    packet.resumeSeq = resumeSeq;
    packet.flag = flag;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACFLY_MISSION_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_LEN, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_LEN);
#endif
}

/**
 * @brief Pack a acfly_mission_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param seq  Current mission sequence number (pairs with MISSION_CURRENT.seq).
 * @param resumeSeq  
 * @param flag  
 * @param heading [deg] 
 * @param vector_x [m] 
 * @param vector_y [m] 
 * @param vector_z [m] 
 * @param distance [m] 
 * @param trigger_interval [m] 
 * @param speed [m/s] 
 * @param radius [m] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_acfly_mission_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t seq,uint16_t resumeSeq,uint16_t flag,float heading,float vector_x,float vector_y,float vector_z,float distance,float trigger_interval,float speed,float radius)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_LEN];
    _mav_put_float(buf, 0, heading);
    _mav_put_float(buf, 4, vector_x);
    _mav_put_float(buf, 8, vector_y);
    _mav_put_float(buf, 12, vector_z);
    _mav_put_float(buf, 16, distance);
    _mav_put_float(buf, 20, trigger_interval);
    _mav_put_float(buf, 24, speed);
    _mav_put_float(buf, 28, radius);
    _mav_put_uint16_t(buf, 32, seq);
    _mav_put_uint16_t(buf, 34, resumeSeq);
    _mav_put_uint16_t(buf, 36, flag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_LEN);
#else
    mavlink_acfly_mission_status_t packet;
    packet.heading = heading;
    packet.vector_x = vector_x;
    packet.vector_y = vector_y;
    packet.vector_z = vector_z;
    packet.distance = distance;
    packet.trigger_interval = trigger_interval;
    packet.speed = speed;
    packet.radius = radius;
    packet.seq = seq;
    packet.resumeSeq = resumeSeq;
    packet.flag = flag;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACFLY_MISSION_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_LEN, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_CRC);
}

/**
 * @brief Encode a acfly_mission_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param acfly_mission_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_acfly_mission_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_acfly_mission_status_t* acfly_mission_status)
{
    return mavlink_msg_acfly_mission_status_pack(system_id, component_id, msg, acfly_mission_status->seq, acfly_mission_status->resumeSeq, acfly_mission_status->flag, acfly_mission_status->heading, acfly_mission_status->vector_x, acfly_mission_status->vector_y, acfly_mission_status->vector_z, acfly_mission_status->distance, acfly_mission_status->trigger_interval, acfly_mission_status->speed, acfly_mission_status->radius);
}

/**
 * @brief Encode a acfly_mission_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param acfly_mission_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_acfly_mission_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_acfly_mission_status_t* acfly_mission_status)
{
    return mavlink_msg_acfly_mission_status_pack_chan(system_id, component_id, chan, msg, acfly_mission_status->seq, acfly_mission_status->resumeSeq, acfly_mission_status->flag, acfly_mission_status->heading, acfly_mission_status->vector_x, acfly_mission_status->vector_y, acfly_mission_status->vector_z, acfly_mission_status->distance, acfly_mission_status->trigger_interval, acfly_mission_status->speed, acfly_mission_status->radius);
}

/**
 * @brief Encode a acfly_mission_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param acfly_mission_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_acfly_mission_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_acfly_mission_status_t* acfly_mission_status)
{
    return mavlink_msg_acfly_mission_status_pack_status(system_id, component_id, _status, msg,  acfly_mission_status->seq, acfly_mission_status->resumeSeq, acfly_mission_status->flag, acfly_mission_status->heading, acfly_mission_status->vector_x, acfly_mission_status->vector_y, acfly_mission_status->vector_z, acfly_mission_status->distance, acfly_mission_status->trigger_interval, acfly_mission_status->speed, acfly_mission_status->radius);
}

/**
 * @brief Send a acfly_mission_status message
 * @param chan MAVLink channel to send the message
 *
 * @param seq  Current mission sequence number (pairs with MISSION_CURRENT.seq).
 * @param resumeSeq  
 * @param flag  
 * @param heading [deg] 
 * @param vector_x [m] 
 * @param vector_y [m] 
 * @param vector_z [m] 
 * @param distance [m] 
 * @param trigger_interval [m] 
 * @param speed [m/s] 
 * @param radius [m] 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_acfly_mission_status_send(mavlink_channel_t chan, uint16_t seq, uint16_t resumeSeq, uint16_t flag, float heading, float vector_x, float vector_y, float vector_z, float distance, float trigger_interval, float speed, float radius)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_LEN];
    _mav_put_float(buf, 0, heading);
    _mav_put_float(buf, 4, vector_x);
    _mav_put_float(buf, 8, vector_y);
    _mav_put_float(buf, 12, vector_z);
    _mav_put_float(buf, 16, distance);
    _mav_put_float(buf, 20, trigger_interval);
    _mav_put_float(buf, 24, speed);
    _mav_put_float(buf, 28, radius);
    _mav_put_uint16_t(buf, 32, seq);
    _mav_put_uint16_t(buf, 34, resumeSeq);
    _mav_put_uint16_t(buf, 36, flag);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS, buf, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_LEN, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_CRC);
#else
    mavlink_acfly_mission_status_t packet;
    packet.heading = heading;
    packet.vector_x = vector_x;
    packet.vector_y = vector_y;
    packet.vector_z = vector_z;
    packet.distance = distance;
    packet.trigger_interval = trigger_interval;
    packet.speed = speed;
    packet.radius = radius;
    packet.seq = seq;
    packet.resumeSeq = resumeSeq;
    packet.flag = flag;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS, (const char *)&packet, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_LEN, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_CRC);
#endif
}

/**
 * @brief Send a acfly_mission_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_acfly_mission_status_send_struct(mavlink_channel_t chan, const mavlink_acfly_mission_status_t* acfly_mission_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_acfly_mission_status_send(chan, acfly_mission_status->seq, acfly_mission_status->resumeSeq, acfly_mission_status->flag, acfly_mission_status->heading, acfly_mission_status->vector_x, acfly_mission_status->vector_y, acfly_mission_status->vector_z, acfly_mission_status->distance, acfly_mission_status->trigger_interval, acfly_mission_status->speed, acfly_mission_status->radius);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS, (const char *)acfly_mission_status, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_LEN, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_acfly_mission_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t seq, uint16_t resumeSeq, uint16_t flag, float heading, float vector_x, float vector_y, float vector_z, float distance, float trigger_interval, float speed, float radius)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, heading);
    _mav_put_float(buf, 4, vector_x);
    _mav_put_float(buf, 8, vector_y);
    _mav_put_float(buf, 12, vector_z);
    _mav_put_float(buf, 16, distance);
    _mav_put_float(buf, 20, trigger_interval);
    _mav_put_float(buf, 24, speed);
    _mav_put_float(buf, 28, radius);
    _mav_put_uint16_t(buf, 32, seq);
    _mav_put_uint16_t(buf, 34, resumeSeq);
    _mav_put_uint16_t(buf, 36, flag);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS, buf, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_LEN, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_CRC);
#else
    mavlink_acfly_mission_status_t *packet = (mavlink_acfly_mission_status_t *)msgbuf;
    packet->heading = heading;
    packet->vector_x = vector_x;
    packet->vector_y = vector_y;
    packet->vector_z = vector_z;
    packet->distance = distance;
    packet->trigger_interval = trigger_interval;
    packet->speed = speed;
    packet->radius = radius;
    packet->seq = seq;
    packet->resumeSeq = resumeSeq;
    packet->flag = flag;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS, (const char *)packet, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_LEN, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE ACFLY_MISSION_STATUS UNPACKING


/**
 * @brief Get field seq from acfly_mission_status message
 *
 * @return  Current mission sequence number (pairs with MISSION_CURRENT.seq).
 */
static inline uint16_t mavlink_msg_acfly_mission_status_get_seq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  32);
}

/**
 * @brief Get field resumeSeq from acfly_mission_status message
 *
 * @return  
 */
static inline uint16_t mavlink_msg_acfly_mission_status_get_resumeSeq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  34);
}

/**
 * @brief Get field flag from acfly_mission_status message
 *
 * @return  
 */
static inline uint16_t mavlink_msg_acfly_mission_status_get_flag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  36);
}

/**
 * @brief Get field heading from acfly_mission_status message
 *
 * @return [deg] 
 */
static inline float mavlink_msg_acfly_mission_status_get_heading(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field vector_x from acfly_mission_status message
 *
 * @return [m] 
 */
static inline float mavlink_msg_acfly_mission_status_get_vector_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field vector_y from acfly_mission_status message
 *
 * @return [m] 
 */
static inline float mavlink_msg_acfly_mission_status_get_vector_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field vector_z from acfly_mission_status message
 *
 * @return [m] 
 */
static inline float mavlink_msg_acfly_mission_status_get_vector_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field distance from acfly_mission_status message
 *
 * @return [m] 
 */
static inline float mavlink_msg_acfly_mission_status_get_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field trigger_interval from acfly_mission_status message
 *
 * @return [m] 
 */
static inline float mavlink_msg_acfly_mission_status_get_trigger_interval(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field speed from acfly_mission_status message
 *
 * @return [m/s] 
 */
static inline float mavlink_msg_acfly_mission_status_get_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field radius from acfly_mission_status message
 *
 * @return [m] 
 */
static inline float mavlink_msg_acfly_mission_status_get_radius(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a acfly_mission_status message into a struct
 *
 * @param msg The message to decode
 * @param acfly_mission_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_acfly_mission_status_decode(const mavlink_message_t* msg, mavlink_acfly_mission_status_t* acfly_mission_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    acfly_mission_status->heading = mavlink_msg_acfly_mission_status_get_heading(msg);
    acfly_mission_status->vector_x = mavlink_msg_acfly_mission_status_get_vector_x(msg);
    acfly_mission_status->vector_y = mavlink_msg_acfly_mission_status_get_vector_y(msg);
    acfly_mission_status->vector_z = mavlink_msg_acfly_mission_status_get_vector_z(msg);
    acfly_mission_status->distance = mavlink_msg_acfly_mission_status_get_distance(msg);
    acfly_mission_status->trigger_interval = mavlink_msg_acfly_mission_status_get_trigger_interval(msg);
    acfly_mission_status->speed = mavlink_msg_acfly_mission_status_get_speed(msg);
    acfly_mission_status->radius = mavlink_msg_acfly_mission_status_get_radius(msg);
    acfly_mission_status->seq = mavlink_msg_acfly_mission_status_get_seq(msg);
    acfly_mission_status->resumeSeq = mavlink_msg_acfly_mission_status_get_resumeSeq(msg);
    acfly_mission_status->flag = mavlink_msg_acfly_mission_status_get_flag(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_LEN? msg->len : MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_LEN;
        memset(acfly_mission_status, 0, MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_LEN);
    memcpy(acfly_mission_status, _MAV_PAYLOAD(msg), len);
#endif
}
