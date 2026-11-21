#pragma once
// MESSAGE AUTO_ROUTE_REQUEST PACKING

#define MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST 414


typedef struct __mavlink_auto_route_request_t {
 int32_t lat1; /*< [degE7] Latitude (WGS84).*/
 int32_t lon1; /*< [degE7] Longitude (WGS84).*/
 int32_t alt1; /*< [m] Altitude (MSL).*/
 int32_t lat2; /*< [degE7] Latitude (WGS84).*/
 int32_t lon2; /*< [degE7] Longitude (WGS84).*/
 int32_t alt2; /*< [m] Altitude (MSL).*/
 uint8_t target_system; /*<  System ID.*/
 uint8_t target_component; /*<  Component ID.*/
 uint8_t type; /*<  0: takeoff, 1: return to launch.*/
 uint8_t mode; /*<  0: efficiency mode, 1: simple mode.*/
} mavlink_auto_route_request_t;

#define MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_LEN 28
#define MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_MIN_LEN 28
#define MAVLINK_MSG_ID_414_LEN 28
#define MAVLINK_MSG_ID_414_MIN_LEN 28

#define MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_CRC 222
#define MAVLINK_MSG_ID_414_CRC 222



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AUTO_ROUTE_REQUEST { \
    414, \
    "AUTO_ROUTE_REQUEST", \
    10, \
    {  { "lat1", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_auto_route_request_t, lat1) }, \
         { "lon1", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_auto_route_request_t, lon1) }, \
         { "alt1", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_auto_route_request_t, alt1) }, \
         { "lat2", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_auto_route_request_t, lat2) }, \
         { "lon2", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_auto_route_request_t, lon2) }, \
         { "alt2", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_auto_route_request_t, alt2) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_auto_route_request_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_auto_route_request_t, target_component) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_auto_route_request_t, type) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_auto_route_request_t, mode) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AUTO_ROUTE_REQUEST { \
    "AUTO_ROUTE_REQUEST", \
    10, \
    {  { "lat1", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_auto_route_request_t, lat1) }, \
         { "lon1", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_auto_route_request_t, lon1) }, \
         { "alt1", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_auto_route_request_t, alt1) }, \
         { "lat2", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_auto_route_request_t, lat2) }, \
         { "lon2", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_auto_route_request_t, lon2) }, \
         { "alt2", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_auto_route_request_t, alt2) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_auto_route_request_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_auto_route_request_t, target_component) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_auto_route_request_t, type) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_auto_route_request_t, mode) }, \
         } \
}
#endif

/**
 * @brief Pack a auto_route_request message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param lat1 [degE7] Latitude (WGS84).
 * @param lon1 [degE7] Longitude (WGS84).
 * @param alt1 [m] Altitude (MSL).
 * @param lat2 [degE7] Latitude (WGS84).
 * @param lon2 [degE7] Longitude (WGS84).
 * @param alt2 [m] Altitude (MSL).
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param type  0: takeoff, 1: return to launch.
 * @param mode  0: efficiency mode, 1: simple mode.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_auto_route_request_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t lat1, int32_t lon1, int32_t alt1, int32_t lat2, int32_t lon2, int32_t alt2, uint8_t target_system, uint8_t target_component, uint8_t type, uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_LEN];
    _mav_put_int32_t(buf, 0, lat1);
    _mav_put_int32_t(buf, 4, lon1);
    _mav_put_int32_t(buf, 8, alt1);
    _mav_put_int32_t(buf, 12, lat2);
    _mav_put_int32_t(buf, 16, lon2);
    _mav_put_int32_t(buf, 20, alt2);
    _mav_put_uint8_t(buf, 24, target_system);
    _mav_put_uint8_t(buf, 25, target_component);
    _mav_put_uint8_t(buf, 26, type);
    _mav_put_uint8_t(buf, 27, mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_LEN);
#else
    mavlink_auto_route_request_t packet;
    packet.lat1 = lat1;
    packet.lon1 = lon1;
    packet.alt1 = alt1;
    packet.lat2 = lat2;
    packet.lon2 = lon2;
    packet.alt2 = alt2;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.type = type;
    packet.mode = mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_MIN_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_CRC);
}

/**
 * @brief Pack a auto_route_request message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param lat1 [degE7] Latitude (WGS84).
 * @param lon1 [degE7] Longitude (WGS84).
 * @param alt1 [m] Altitude (MSL).
 * @param lat2 [degE7] Latitude (WGS84).
 * @param lon2 [degE7] Longitude (WGS84).
 * @param alt2 [m] Altitude (MSL).
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param type  0: takeoff, 1: return to launch.
 * @param mode  0: efficiency mode, 1: simple mode.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_auto_route_request_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               int32_t lat1, int32_t lon1, int32_t alt1, int32_t lat2, int32_t lon2, int32_t alt2, uint8_t target_system, uint8_t target_component, uint8_t type, uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_LEN];
    _mav_put_int32_t(buf, 0, lat1);
    _mav_put_int32_t(buf, 4, lon1);
    _mav_put_int32_t(buf, 8, alt1);
    _mav_put_int32_t(buf, 12, lat2);
    _mav_put_int32_t(buf, 16, lon2);
    _mav_put_int32_t(buf, 20, alt2);
    _mav_put_uint8_t(buf, 24, target_system);
    _mav_put_uint8_t(buf, 25, target_component);
    _mav_put_uint8_t(buf, 26, type);
    _mav_put_uint8_t(buf, 27, mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_LEN);
#else
    mavlink_auto_route_request_t packet;
    packet.lat1 = lat1;
    packet.lon1 = lon1;
    packet.alt1 = alt1;
    packet.lat2 = lat2;
    packet.lon2 = lon2;
    packet.alt2 = alt2;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.type = type;
    packet.mode = mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_MIN_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_MIN_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_LEN);
#endif
}

/**
 * @brief Pack a auto_route_request message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lat1 [degE7] Latitude (WGS84).
 * @param lon1 [degE7] Longitude (WGS84).
 * @param alt1 [m] Altitude (MSL).
 * @param lat2 [degE7] Latitude (WGS84).
 * @param lon2 [degE7] Longitude (WGS84).
 * @param alt2 [m] Altitude (MSL).
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param type  0: takeoff, 1: return to launch.
 * @param mode  0: efficiency mode, 1: simple mode.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_auto_route_request_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t lat1,int32_t lon1,int32_t alt1,int32_t lat2,int32_t lon2,int32_t alt2,uint8_t target_system,uint8_t target_component,uint8_t type,uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_LEN];
    _mav_put_int32_t(buf, 0, lat1);
    _mav_put_int32_t(buf, 4, lon1);
    _mav_put_int32_t(buf, 8, alt1);
    _mav_put_int32_t(buf, 12, lat2);
    _mav_put_int32_t(buf, 16, lon2);
    _mav_put_int32_t(buf, 20, alt2);
    _mav_put_uint8_t(buf, 24, target_system);
    _mav_put_uint8_t(buf, 25, target_component);
    _mav_put_uint8_t(buf, 26, type);
    _mav_put_uint8_t(buf, 27, mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_LEN);
#else
    mavlink_auto_route_request_t packet;
    packet.lat1 = lat1;
    packet.lon1 = lon1;
    packet.alt1 = alt1;
    packet.lat2 = lat2;
    packet.lon2 = lon2;
    packet.alt2 = alt2;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.type = type;
    packet.mode = mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_MIN_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_CRC);
}

/**
 * @brief Encode a auto_route_request struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param auto_route_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_auto_route_request_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_auto_route_request_t* auto_route_request)
{
    return mavlink_msg_auto_route_request_pack(system_id, component_id, msg, auto_route_request->lat1, auto_route_request->lon1, auto_route_request->alt1, auto_route_request->lat2, auto_route_request->lon2, auto_route_request->alt2, auto_route_request->target_system, auto_route_request->target_component, auto_route_request->type, auto_route_request->mode);
}

/**
 * @brief Encode a auto_route_request struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param auto_route_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_auto_route_request_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_auto_route_request_t* auto_route_request)
{
    return mavlink_msg_auto_route_request_pack_chan(system_id, component_id, chan, msg, auto_route_request->lat1, auto_route_request->lon1, auto_route_request->alt1, auto_route_request->lat2, auto_route_request->lon2, auto_route_request->alt2, auto_route_request->target_system, auto_route_request->target_component, auto_route_request->type, auto_route_request->mode);
}

/**
 * @brief Encode a auto_route_request struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param auto_route_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_auto_route_request_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_auto_route_request_t* auto_route_request)
{
    return mavlink_msg_auto_route_request_pack_status(system_id, component_id, _status, msg,  auto_route_request->lat1, auto_route_request->lon1, auto_route_request->alt1, auto_route_request->lat2, auto_route_request->lon2, auto_route_request->alt2, auto_route_request->target_system, auto_route_request->target_component, auto_route_request->type, auto_route_request->mode);
}

/**
 * @brief Send a auto_route_request message
 * @param chan MAVLink channel to send the message
 *
 * @param lat1 [degE7] Latitude (WGS84).
 * @param lon1 [degE7] Longitude (WGS84).
 * @param alt1 [m] Altitude (MSL).
 * @param lat2 [degE7] Latitude (WGS84).
 * @param lon2 [degE7] Longitude (WGS84).
 * @param alt2 [m] Altitude (MSL).
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param type  0: takeoff, 1: return to launch.
 * @param mode  0: efficiency mode, 1: simple mode.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_auto_route_request_send(mavlink_channel_t chan, int32_t lat1, int32_t lon1, int32_t alt1, int32_t lat2, int32_t lon2, int32_t alt2, uint8_t target_system, uint8_t target_component, uint8_t type, uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_LEN];
    _mav_put_int32_t(buf, 0, lat1);
    _mav_put_int32_t(buf, 4, lon1);
    _mav_put_int32_t(buf, 8, alt1);
    _mav_put_int32_t(buf, 12, lat2);
    _mav_put_int32_t(buf, 16, lon2);
    _mav_put_int32_t(buf, 20, alt2);
    _mav_put_uint8_t(buf, 24, target_system);
    _mav_put_uint8_t(buf, 25, target_component);
    _mav_put_uint8_t(buf, 26, type);
    _mav_put_uint8_t(buf, 27, mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST, buf, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_MIN_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_CRC);
#else
    mavlink_auto_route_request_t packet;
    packet.lat1 = lat1;
    packet.lon1 = lon1;
    packet.alt1 = alt1;
    packet.lat2 = lat2;
    packet.lon2 = lon2;
    packet.alt2 = alt2;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.type = type;
    packet.mode = mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST, (const char *)&packet, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_MIN_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_CRC);
#endif
}

/**
 * @brief Send a auto_route_request message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_auto_route_request_send_struct(mavlink_channel_t chan, const mavlink_auto_route_request_t* auto_route_request)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_auto_route_request_send(chan, auto_route_request->lat1, auto_route_request->lon1, auto_route_request->alt1, auto_route_request->lat2, auto_route_request->lon2, auto_route_request->alt2, auto_route_request->target_system, auto_route_request->target_component, auto_route_request->type, auto_route_request->mode);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST, (const char *)auto_route_request, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_MIN_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_CRC);
#endif
}

#if MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_auto_route_request_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t lat1, int32_t lon1, int32_t alt1, int32_t lat2, int32_t lon2, int32_t alt2, uint8_t target_system, uint8_t target_component, uint8_t type, uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, lat1);
    _mav_put_int32_t(buf, 4, lon1);
    _mav_put_int32_t(buf, 8, alt1);
    _mav_put_int32_t(buf, 12, lat2);
    _mav_put_int32_t(buf, 16, lon2);
    _mav_put_int32_t(buf, 20, alt2);
    _mav_put_uint8_t(buf, 24, target_system);
    _mav_put_uint8_t(buf, 25, target_component);
    _mav_put_uint8_t(buf, 26, type);
    _mav_put_uint8_t(buf, 27, mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST, buf, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_MIN_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_CRC);
#else
    mavlink_auto_route_request_t *packet = (mavlink_auto_route_request_t *)msgbuf;
    packet->lat1 = lat1;
    packet->lon1 = lon1;
    packet->alt1 = alt1;
    packet->lat2 = lat2;
    packet->lon2 = lon2;
    packet->alt2 = alt2;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->type = type;
    packet->mode = mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST, (const char *)packet, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_MIN_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_LEN, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_CRC);
#endif
}
#endif

#endif

// MESSAGE AUTO_ROUTE_REQUEST UNPACKING


/**
 * @brief Get field lat1 from auto_route_request message
 *
 * @return [degE7] Latitude (WGS84).
 */
static inline int32_t mavlink_msg_auto_route_request_get_lat1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field lon1 from auto_route_request message
 *
 * @return [degE7] Longitude (WGS84).
 */
static inline int32_t mavlink_msg_auto_route_request_get_lon1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field alt1 from auto_route_request message
 *
 * @return [m] Altitude (MSL).
 */
static inline int32_t mavlink_msg_auto_route_request_get_alt1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field lat2 from auto_route_request message
 *
 * @return [degE7] Latitude (WGS84).
 */
static inline int32_t mavlink_msg_auto_route_request_get_lat2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field lon2 from auto_route_request message
 *
 * @return [degE7] Longitude (WGS84).
 */
static inline int32_t mavlink_msg_auto_route_request_get_lon2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field alt2 from auto_route_request message
 *
 * @return [m] Altitude (MSL).
 */
static inline int32_t mavlink_msg_auto_route_request_get_alt2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Get field target_system from auto_route_request message
 *
 * @return  System ID.
 */
static inline uint8_t mavlink_msg_auto_route_request_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field target_component from auto_route_request message
 *
 * @return  Component ID.
 */
static inline uint8_t mavlink_msg_auto_route_request_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field type from auto_route_request message
 *
 * @return  0: takeoff, 1: return to launch.
 */
static inline uint8_t mavlink_msg_auto_route_request_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field mode from auto_route_request message
 *
 * @return  0: efficiency mode, 1: simple mode.
 */
static inline uint8_t mavlink_msg_auto_route_request_get_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  27);
}

/**
 * @brief Decode a auto_route_request message into a struct
 *
 * @param msg The message to decode
 * @param auto_route_request C-struct to decode the message contents into
 */
static inline void mavlink_msg_auto_route_request_decode(const mavlink_message_t* msg, mavlink_auto_route_request_t* auto_route_request)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    auto_route_request->lat1 = mavlink_msg_auto_route_request_get_lat1(msg);
    auto_route_request->lon1 = mavlink_msg_auto_route_request_get_lon1(msg);
    auto_route_request->alt1 = mavlink_msg_auto_route_request_get_alt1(msg);
    auto_route_request->lat2 = mavlink_msg_auto_route_request_get_lat2(msg);
    auto_route_request->lon2 = mavlink_msg_auto_route_request_get_lon2(msg);
    auto_route_request->alt2 = mavlink_msg_auto_route_request_get_alt2(msg);
    auto_route_request->target_system = mavlink_msg_auto_route_request_get_target_system(msg);
    auto_route_request->target_component = mavlink_msg_auto_route_request_get_target_component(msg);
    auto_route_request->type = mavlink_msg_auto_route_request_get_type(msg);
    auto_route_request->mode = mavlink_msg_auto_route_request_get_mode(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_LEN? msg->len : MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_LEN;
        memset(auto_route_request, 0, MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_LEN);
    memcpy(auto_route_request, _MAV_PAYLOAD(msg), len);
#endif
}
