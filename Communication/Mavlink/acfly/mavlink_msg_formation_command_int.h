#pragma once
// MESSAGE FORMATION_COMMAND_INT PACKING

#define MAVLINK_MSG_ID_FORMATION_COMMAND_INT 601


typedef struct __mavlink_formation_command_int_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time since 1.1.1970).*/
 uint64_t cmdExTime; /*< [ms] Timestamp that the command should be executed (UNIX Epoch time since 1.1.1970).*/
 float params[7]; /*<  PARAM1-7.*/
 int32_t x; /*<  local: x position in meters * 1e4; global: latitude in degrees * 1e7.*/
 int32_t y; /*<  local: y position in meters * 1e4; global: longitude in degrees * 1e7.*/
 float z; /*<  z position: global: altitude in meters (relative or absolute, depending on frame).*/
 uint16_t target_system; /*<  System ID.*/
 uint16_t target_component; /*<  Component ID.*/
 uint16_t command; /*<  The scheduled action for the mission item.*/
 uint8_t frame; /*<  The coordinate system of the COMMAND.*/
 uint8_t flag; /*<  Flag.*/
} mavlink_formation_command_int_t;

#define MAVLINK_MSG_ID_FORMATION_COMMAND_INT_LEN 64
#define MAVLINK_MSG_ID_FORMATION_COMMAND_INT_MIN_LEN 64
#define MAVLINK_MSG_ID_601_LEN 64
#define MAVLINK_MSG_ID_601_MIN_LEN 64

#define MAVLINK_MSG_ID_FORMATION_COMMAND_INT_CRC 127
#define MAVLINK_MSG_ID_601_CRC 127

#define MAVLINK_MSG_FORMATION_COMMAND_INT_FIELD_PARAMS_LEN 7

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FORMATION_COMMAND_INT { \
    601, \
    "FORMATION_COMMAND_INT", \
    11, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_formation_command_int_t, time_usec) }, \
         { "cmdExTime", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_formation_command_int_t, cmdExTime) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT16_T, 0, 56, offsetof(mavlink_formation_command_int_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT16_T, 0, 58, offsetof(mavlink_formation_command_int_t, target_component) }, \
         { "frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 62, offsetof(mavlink_formation_command_int_t, frame) }, \
         { "flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 63, offsetof(mavlink_formation_command_int_t, flag) }, \
         { "command", NULL, MAVLINK_TYPE_UINT16_T, 0, 60, offsetof(mavlink_formation_command_int_t, command) }, \
         { "params", NULL, MAVLINK_TYPE_FLOAT, 7, 16, offsetof(mavlink_formation_command_int_t, params) }, \
         { "x", NULL, MAVLINK_TYPE_INT32_T, 0, 44, offsetof(mavlink_formation_command_int_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_INT32_T, 0, 48, offsetof(mavlink_formation_command_int_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_formation_command_int_t, z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FORMATION_COMMAND_INT { \
    "FORMATION_COMMAND_INT", \
    11, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_formation_command_int_t, time_usec) }, \
         { "cmdExTime", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_formation_command_int_t, cmdExTime) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT16_T, 0, 56, offsetof(mavlink_formation_command_int_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT16_T, 0, 58, offsetof(mavlink_formation_command_int_t, target_component) }, \
         { "frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 62, offsetof(mavlink_formation_command_int_t, frame) }, \
         { "flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 63, offsetof(mavlink_formation_command_int_t, flag) }, \
         { "command", NULL, MAVLINK_TYPE_UINT16_T, 0, 60, offsetof(mavlink_formation_command_int_t, command) }, \
         { "params", NULL, MAVLINK_TYPE_FLOAT, 7, 16, offsetof(mavlink_formation_command_int_t, params) }, \
         { "x", NULL, MAVLINK_TYPE_INT32_T, 0, 44, offsetof(mavlink_formation_command_int_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_INT32_T, 0, 48, offsetof(mavlink_formation_command_int_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_formation_command_int_t, z) }, \
         } \
}
#endif

/**
 * @brief Pack a formation_command_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time since 1.1.1970).
 * @param cmdExTime [ms] Timestamp that the command should be executed (UNIX Epoch time since 1.1.1970).
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param frame  The coordinate system of the COMMAND.
 * @param flag  Flag.
 * @param command  The scheduled action for the mission item.
 * @param params  PARAM1-7.
 * @param x  local: x position in meters * 1e4; global: latitude in degrees * 1e7.
 * @param y  local: y position in meters * 1e4; global: longitude in degrees * 1e7.
 * @param z  z position: global: altitude in meters (relative or absolute, depending on frame).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_formation_command_int_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint64_t cmdExTime, uint16_t target_system, uint16_t target_component, uint8_t frame, uint8_t flag, uint16_t command, const float *params, int32_t x, int32_t y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FORMATION_COMMAND_INT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, cmdExTime);
    _mav_put_int32_t(buf, 44, x);
    _mav_put_int32_t(buf, 48, y);
    _mav_put_float(buf, 52, z);
    _mav_put_uint16_t(buf, 56, target_system);
    _mav_put_uint16_t(buf, 58, target_component);
    _mav_put_uint16_t(buf, 60, command);
    _mav_put_uint8_t(buf, 62, frame);
    _mav_put_uint8_t(buf, 63, flag);
    _mav_put_float_array(buf, 16, params, 7);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FORMATION_COMMAND_INT_LEN);
#else
    mavlink_formation_command_int_t packet;
    packet.time_usec = time_usec;
    packet.cmdExTime = cmdExTime;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.command = command;
    packet.frame = frame;
    packet.flag = flag;
    mav_array_assign_float(packet.params, params, 7);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FORMATION_COMMAND_INT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FORMATION_COMMAND_INT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FORMATION_COMMAND_INT_MIN_LEN, MAVLINK_MSG_ID_FORMATION_COMMAND_INT_LEN, MAVLINK_MSG_ID_FORMATION_COMMAND_INT_CRC);
}

/**
 * @brief Pack a formation_command_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time since 1.1.1970).
 * @param cmdExTime [ms] Timestamp that the command should be executed (UNIX Epoch time since 1.1.1970).
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param frame  The coordinate system of the COMMAND.
 * @param flag  Flag.
 * @param command  The scheduled action for the mission item.
 * @param params  PARAM1-7.
 * @param x  local: x position in meters * 1e4; global: latitude in degrees * 1e7.
 * @param y  local: y position in meters * 1e4; global: longitude in degrees * 1e7.
 * @param z  z position: global: altitude in meters (relative or absolute, depending on frame).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_formation_command_int_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t time_usec, uint64_t cmdExTime, uint16_t target_system, uint16_t target_component, uint8_t frame, uint8_t flag, uint16_t command, const float *params, int32_t x, int32_t y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FORMATION_COMMAND_INT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, cmdExTime);
    _mav_put_int32_t(buf, 44, x);
    _mav_put_int32_t(buf, 48, y);
    _mav_put_float(buf, 52, z);
    _mav_put_uint16_t(buf, 56, target_system);
    _mav_put_uint16_t(buf, 58, target_component);
    _mav_put_uint16_t(buf, 60, command);
    _mav_put_uint8_t(buf, 62, frame);
    _mav_put_uint8_t(buf, 63, flag);
    _mav_put_float_array(buf, 16, params, 7);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FORMATION_COMMAND_INT_LEN);
#else
    mavlink_formation_command_int_t packet;
    packet.time_usec = time_usec;
    packet.cmdExTime = cmdExTime;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.command = command;
    packet.frame = frame;
    packet.flag = flag;
    mav_array_memcpy(packet.params, params, sizeof(float)*7);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FORMATION_COMMAND_INT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FORMATION_COMMAND_INT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_FORMATION_COMMAND_INT_MIN_LEN, MAVLINK_MSG_ID_FORMATION_COMMAND_INT_LEN, MAVLINK_MSG_ID_FORMATION_COMMAND_INT_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_FORMATION_COMMAND_INT_MIN_LEN, MAVLINK_MSG_ID_FORMATION_COMMAND_INT_LEN);
#endif
}

/**
 * @brief Pack a formation_command_int message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time since 1.1.1970).
 * @param cmdExTime [ms] Timestamp that the command should be executed (UNIX Epoch time since 1.1.1970).
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param frame  The coordinate system of the COMMAND.
 * @param flag  Flag.
 * @param command  The scheduled action for the mission item.
 * @param params  PARAM1-7.
 * @param x  local: x position in meters * 1e4; global: latitude in degrees * 1e7.
 * @param y  local: y position in meters * 1e4; global: longitude in degrees * 1e7.
 * @param z  z position: global: altitude in meters (relative or absolute, depending on frame).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_formation_command_int_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint64_t cmdExTime,uint16_t target_system,uint16_t target_component,uint8_t frame,uint8_t flag,uint16_t command,const float *params,int32_t x,int32_t y,float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FORMATION_COMMAND_INT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, cmdExTime);
    _mav_put_int32_t(buf, 44, x);
    _mav_put_int32_t(buf, 48, y);
    _mav_put_float(buf, 52, z);
    _mav_put_uint16_t(buf, 56, target_system);
    _mav_put_uint16_t(buf, 58, target_component);
    _mav_put_uint16_t(buf, 60, command);
    _mav_put_uint8_t(buf, 62, frame);
    _mav_put_uint8_t(buf, 63, flag);
    _mav_put_float_array(buf, 16, params, 7);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FORMATION_COMMAND_INT_LEN);
#else
    mavlink_formation_command_int_t packet;
    packet.time_usec = time_usec;
    packet.cmdExTime = cmdExTime;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.command = command;
    packet.frame = frame;
    packet.flag = flag;
    mav_array_assign_float(packet.params, params, 7);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FORMATION_COMMAND_INT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FORMATION_COMMAND_INT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FORMATION_COMMAND_INT_MIN_LEN, MAVLINK_MSG_ID_FORMATION_COMMAND_INT_LEN, MAVLINK_MSG_ID_FORMATION_COMMAND_INT_CRC);
}

/**
 * @brief Encode a formation_command_int struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param formation_command_int C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_formation_command_int_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_formation_command_int_t* formation_command_int)
{
    return mavlink_msg_formation_command_int_pack(system_id, component_id, msg, formation_command_int->time_usec, formation_command_int->cmdExTime, formation_command_int->target_system, formation_command_int->target_component, formation_command_int->frame, formation_command_int->flag, formation_command_int->command, formation_command_int->params, formation_command_int->x, formation_command_int->y, formation_command_int->z);
}

/**
 * @brief Encode a formation_command_int struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param formation_command_int C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_formation_command_int_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_formation_command_int_t* formation_command_int)
{
    return mavlink_msg_formation_command_int_pack_chan(system_id, component_id, chan, msg, formation_command_int->time_usec, formation_command_int->cmdExTime, formation_command_int->target_system, formation_command_int->target_component, formation_command_int->frame, formation_command_int->flag, formation_command_int->command, formation_command_int->params, formation_command_int->x, formation_command_int->y, formation_command_int->z);
}

/**
 * @brief Encode a formation_command_int struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param formation_command_int C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_formation_command_int_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_formation_command_int_t* formation_command_int)
{
    return mavlink_msg_formation_command_int_pack_status(system_id, component_id, _status, msg,  formation_command_int->time_usec, formation_command_int->cmdExTime, formation_command_int->target_system, formation_command_int->target_component, formation_command_int->frame, formation_command_int->flag, formation_command_int->command, formation_command_int->params, formation_command_int->x, formation_command_int->y, formation_command_int->z);
}

/**
 * @brief Send a formation_command_int message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time since 1.1.1970).
 * @param cmdExTime [ms] Timestamp that the command should be executed (UNIX Epoch time since 1.1.1970).
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param frame  The coordinate system of the COMMAND.
 * @param flag  Flag.
 * @param command  The scheduled action for the mission item.
 * @param params  PARAM1-7.
 * @param x  local: x position in meters * 1e4; global: latitude in degrees * 1e7.
 * @param y  local: y position in meters * 1e4; global: longitude in degrees * 1e7.
 * @param z  z position: global: altitude in meters (relative or absolute, depending on frame).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_formation_command_int_send(mavlink_channel_t chan, uint64_t time_usec, uint64_t cmdExTime, uint16_t target_system, uint16_t target_component, uint8_t frame, uint8_t flag, uint16_t command, const float *params, int32_t x, int32_t y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FORMATION_COMMAND_INT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, cmdExTime);
    _mav_put_int32_t(buf, 44, x);
    _mav_put_int32_t(buf, 48, y);
    _mav_put_float(buf, 52, z);
    _mav_put_uint16_t(buf, 56, target_system);
    _mav_put_uint16_t(buf, 58, target_component);
    _mav_put_uint16_t(buf, 60, command);
    _mav_put_uint8_t(buf, 62, frame);
    _mav_put_uint8_t(buf, 63, flag);
    _mav_put_float_array(buf, 16, params, 7);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION_COMMAND_INT, buf, MAVLINK_MSG_ID_FORMATION_COMMAND_INT_MIN_LEN, MAVLINK_MSG_ID_FORMATION_COMMAND_INT_LEN, MAVLINK_MSG_ID_FORMATION_COMMAND_INT_CRC);
#else
    mavlink_formation_command_int_t packet;
    packet.time_usec = time_usec;
    packet.cmdExTime = cmdExTime;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.command = command;
    packet.frame = frame;
    packet.flag = flag;
    mav_array_assign_float(packet.params, params, 7);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION_COMMAND_INT, (const char *)&packet, MAVLINK_MSG_ID_FORMATION_COMMAND_INT_MIN_LEN, MAVLINK_MSG_ID_FORMATION_COMMAND_INT_LEN, MAVLINK_MSG_ID_FORMATION_COMMAND_INT_CRC);
#endif
}

/**
 * @brief Send a formation_command_int message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_formation_command_int_send_struct(mavlink_channel_t chan, const mavlink_formation_command_int_t* formation_command_int)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_formation_command_int_send(chan, formation_command_int->time_usec, formation_command_int->cmdExTime, formation_command_int->target_system, formation_command_int->target_component, formation_command_int->frame, formation_command_int->flag, formation_command_int->command, formation_command_int->params, formation_command_int->x, formation_command_int->y, formation_command_int->z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION_COMMAND_INT, (const char *)formation_command_int, MAVLINK_MSG_ID_FORMATION_COMMAND_INT_MIN_LEN, MAVLINK_MSG_ID_FORMATION_COMMAND_INT_LEN, MAVLINK_MSG_ID_FORMATION_COMMAND_INT_CRC);
#endif
}

#if MAVLINK_MSG_ID_FORMATION_COMMAND_INT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_formation_command_int_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint64_t cmdExTime, uint16_t target_system, uint16_t target_component, uint8_t frame, uint8_t flag, uint16_t command, const float *params, int32_t x, int32_t y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, cmdExTime);
    _mav_put_int32_t(buf, 44, x);
    _mav_put_int32_t(buf, 48, y);
    _mav_put_float(buf, 52, z);
    _mav_put_uint16_t(buf, 56, target_system);
    _mav_put_uint16_t(buf, 58, target_component);
    _mav_put_uint16_t(buf, 60, command);
    _mav_put_uint8_t(buf, 62, frame);
    _mav_put_uint8_t(buf, 63, flag);
    _mav_put_float_array(buf, 16, params, 7);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION_COMMAND_INT, buf, MAVLINK_MSG_ID_FORMATION_COMMAND_INT_MIN_LEN, MAVLINK_MSG_ID_FORMATION_COMMAND_INT_LEN, MAVLINK_MSG_ID_FORMATION_COMMAND_INT_CRC);
#else
    mavlink_formation_command_int_t *packet = (mavlink_formation_command_int_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->cmdExTime = cmdExTime;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->command = command;
    packet->frame = frame;
    packet->flag = flag;
    mav_array_assign_float(packet->params, params, 7);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION_COMMAND_INT, (const char *)packet, MAVLINK_MSG_ID_FORMATION_COMMAND_INT_MIN_LEN, MAVLINK_MSG_ID_FORMATION_COMMAND_INT_LEN, MAVLINK_MSG_ID_FORMATION_COMMAND_INT_CRC);
#endif
}
#endif

#endif

// MESSAGE FORMATION_COMMAND_INT UNPACKING


/**
 * @brief Get field time_usec from formation_command_int message
 *
 * @return [us] Timestamp (UNIX Epoch time since 1.1.1970).
 */
static inline uint64_t mavlink_msg_formation_command_int_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field cmdExTime from formation_command_int message
 *
 * @return [ms] Timestamp that the command should be executed (UNIX Epoch time since 1.1.1970).
 */
static inline uint64_t mavlink_msg_formation_command_int_get_cmdExTime(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field target_system from formation_command_int message
 *
 * @return  System ID.
 */
static inline uint16_t mavlink_msg_formation_command_int_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  56);
}

/**
 * @brief Get field target_component from formation_command_int message
 *
 * @return  Component ID.
 */
static inline uint16_t mavlink_msg_formation_command_int_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  58);
}

/**
 * @brief Get field frame from formation_command_int message
 *
 * @return  The coordinate system of the COMMAND.
 */
static inline uint8_t mavlink_msg_formation_command_int_get_frame(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  62);
}

/**
 * @brief Get field flag from formation_command_int message
 *
 * @return  Flag.
 */
static inline uint8_t mavlink_msg_formation_command_int_get_flag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  63);
}

/**
 * @brief Get field command from formation_command_int message
 *
 * @return  The scheduled action for the mission item.
 */
static inline uint16_t mavlink_msg_formation_command_int_get_command(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  60);
}

/**
 * @brief Get field params from formation_command_int message
 *
 * @return  PARAM1-7.
 */
static inline uint16_t mavlink_msg_formation_command_int_get_params(const mavlink_message_t* msg, float *params)
{
    return _MAV_RETURN_float_array(msg, params, 7,  16);
}

/**
 * @brief Get field x from formation_command_int message
 *
 * @return  local: x position in meters * 1e4; global: latitude in degrees * 1e7.
 */
static inline int32_t mavlink_msg_formation_command_int_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  44);
}

/**
 * @brief Get field y from formation_command_int message
 *
 * @return  local: y position in meters * 1e4; global: longitude in degrees * 1e7.
 */
static inline int32_t mavlink_msg_formation_command_int_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  48);
}

/**
 * @brief Get field z from formation_command_int message
 *
 * @return  z position: global: altitude in meters (relative or absolute, depending on frame).
 */
static inline float mavlink_msg_formation_command_int_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Decode a formation_command_int message into a struct
 *
 * @param msg The message to decode
 * @param formation_command_int C-struct to decode the message contents into
 */
static inline void mavlink_msg_formation_command_int_decode(const mavlink_message_t* msg, mavlink_formation_command_int_t* formation_command_int)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    formation_command_int->time_usec = mavlink_msg_formation_command_int_get_time_usec(msg);
    formation_command_int->cmdExTime = mavlink_msg_formation_command_int_get_cmdExTime(msg);
    mavlink_msg_formation_command_int_get_params(msg, formation_command_int->params);
    formation_command_int->x = mavlink_msg_formation_command_int_get_x(msg);
    formation_command_int->y = mavlink_msg_formation_command_int_get_y(msg);
    formation_command_int->z = mavlink_msg_formation_command_int_get_z(msg);
    formation_command_int->target_system = mavlink_msg_formation_command_int_get_target_system(msg);
    formation_command_int->target_component = mavlink_msg_formation_command_int_get_target_component(msg);
    formation_command_int->command = mavlink_msg_formation_command_int_get_command(msg);
    formation_command_int->frame = mavlink_msg_formation_command_int_get_frame(msg);
    formation_command_int->flag = mavlink_msg_formation_command_int_get_flag(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FORMATION_COMMAND_INT_LEN? msg->len : MAVLINK_MSG_ID_FORMATION_COMMAND_INT_LEN;
        memset(formation_command_int, 0, MAVLINK_MSG_ID_FORMATION_COMMAND_INT_LEN);
    memcpy(formation_command_int, _MAV_PAYLOAD(msg), len);
#endif
}
