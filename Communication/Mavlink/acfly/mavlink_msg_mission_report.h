#pragma once
// MESSAGE MISSION_REPORT PACKING

#define MAVLINK_MSG_ID_MISSION_REPORT 210


typedef struct __mavlink_mission_report_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot).*/
 uint16_t seq; /*<  Mission sequence number.*/
 uint16_t cmd; /*<  Mission command identifier.*/
 uint16_t mission_count; /*<  Total number of mission items in the mission.*/
 uint8_t progress; /*<  Current mission item progress, 0 to 100.*/
 uint8_t result; /*<  Mission execution state.*/
} mavlink_mission_report_t;

#define MAVLINK_MSG_ID_MISSION_REPORT_LEN 16
#define MAVLINK_MSG_ID_MISSION_REPORT_MIN_LEN 16
#define MAVLINK_MSG_ID_210_LEN 16
#define MAVLINK_MSG_ID_210_MIN_LEN 16

#define MAVLINK_MSG_ID_MISSION_REPORT_CRC 224
#define MAVLINK_MSG_ID_210_CRC 224



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MISSION_REPORT { \
    210, \
    "MISSION_REPORT", \
    6, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_mission_report_t, time_usec) }, \
         { "seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_mission_report_t, seq) }, \
         { "cmd", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_mission_report_t, cmd) }, \
         { "mission_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_mission_report_t, mission_count) }, \
         { "progress", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_mission_report_t, progress) }, \
         { "result", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_mission_report_t, result) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MISSION_REPORT { \
    "MISSION_REPORT", \
    6, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_mission_report_t, time_usec) }, \
         { "seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_mission_report_t, seq) }, \
         { "cmd", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_mission_report_t, cmd) }, \
         { "mission_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_mission_report_t, mission_count) }, \
         { "progress", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_mission_report_t, progress) }, \
         { "result", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_mission_report_t, result) }, \
         } \
}
#endif

/**
 * @brief Pack a mission_report message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot).
 * @param seq  Mission sequence number.
 * @param cmd  Mission command identifier.
 * @param mission_count  Total number of mission items in the mission.
 * @param progress  Current mission item progress, 0 to 100.
 * @param result  Mission execution state.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_report_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint16_t seq, uint16_t cmd, uint16_t mission_count, uint8_t progress, uint8_t result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_REPORT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint16_t(buf, 8, seq);
    _mav_put_uint16_t(buf, 10, cmd);
    _mav_put_uint16_t(buf, 12, mission_count);
    _mav_put_uint8_t(buf, 14, progress);
    _mav_put_uint8_t(buf, 15, result);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_REPORT_LEN);
#else
    mavlink_mission_report_t packet;
    packet.time_usec = time_usec;
    packet.seq = seq;
    packet.cmd = cmd;
    packet.mission_count = mission_count;
    packet.progress = progress;
    packet.result = result;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_REPORT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MISSION_REPORT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MISSION_REPORT_MIN_LEN, MAVLINK_MSG_ID_MISSION_REPORT_LEN, MAVLINK_MSG_ID_MISSION_REPORT_CRC);
}

/**
 * @brief Pack a mission_report message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot).
 * @param seq  Mission sequence number.
 * @param cmd  Mission command identifier.
 * @param mission_count  Total number of mission items in the mission.
 * @param progress  Current mission item progress, 0 to 100.
 * @param result  Mission execution state.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_report_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t time_usec, uint16_t seq, uint16_t cmd, uint16_t mission_count, uint8_t progress, uint8_t result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_REPORT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint16_t(buf, 8, seq);
    _mav_put_uint16_t(buf, 10, cmd);
    _mav_put_uint16_t(buf, 12, mission_count);
    _mav_put_uint8_t(buf, 14, progress);
    _mav_put_uint8_t(buf, 15, result);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_REPORT_LEN);
#else
    mavlink_mission_report_t packet;
    packet.time_usec = time_usec;
    packet.seq = seq;
    packet.cmd = cmd;
    packet.mission_count = mission_count;
    packet.progress = progress;
    packet.result = result;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_REPORT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MISSION_REPORT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MISSION_REPORT_MIN_LEN, MAVLINK_MSG_ID_MISSION_REPORT_LEN, MAVLINK_MSG_ID_MISSION_REPORT_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MISSION_REPORT_MIN_LEN, MAVLINK_MSG_ID_MISSION_REPORT_LEN);
#endif
}

/**
 * @brief Pack a mission_report message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot).
 * @param seq  Mission sequence number.
 * @param cmd  Mission command identifier.
 * @param mission_count  Total number of mission items in the mission.
 * @param progress  Current mission item progress, 0 to 100.
 * @param result  Mission execution state.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_report_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint16_t seq,uint16_t cmd,uint16_t mission_count,uint8_t progress,uint8_t result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_REPORT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint16_t(buf, 8, seq);
    _mav_put_uint16_t(buf, 10, cmd);
    _mav_put_uint16_t(buf, 12, mission_count);
    _mav_put_uint8_t(buf, 14, progress);
    _mav_put_uint8_t(buf, 15, result);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_REPORT_LEN);
#else
    mavlink_mission_report_t packet;
    packet.time_usec = time_usec;
    packet.seq = seq;
    packet.cmd = cmd;
    packet.mission_count = mission_count;
    packet.progress = progress;
    packet.result = result;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_REPORT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MISSION_REPORT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MISSION_REPORT_MIN_LEN, MAVLINK_MSG_ID_MISSION_REPORT_LEN, MAVLINK_MSG_ID_MISSION_REPORT_CRC);
}

/**
 * @brief Encode a mission_report struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_report C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_report_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mission_report_t* mission_report)
{
    return mavlink_msg_mission_report_pack(system_id, component_id, msg, mission_report->time_usec, mission_report->seq, mission_report->cmd, mission_report->mission_count, mission_report->progress, mission_report->result);
}

/**
 * @brief Encode a mission_report struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_report C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_report_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mission_report_t* mission_report)
{
    return mavlink_msg_mission_report_pack_chan(system_id, component_id, chan, msg, mission_report->time_usec, mission_report->seq, mission_report->cmd, mission_report->mission_count, mission_report->progress, mission_report->result);
}

/**
 * @brief Encode a mission_report struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param mission_report C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_report_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_mission_report_t* mission_report)
{
    return mavlink_msg_mission_report_pack_status(system_id, component_id, _status, msg,  mission_report->time_usec, mission_report->seq, mission_report->cmd, mission_report->mission_count, mission_report->progress, mission_report->result);
}

/**
 * @brief Send a mission_report message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot).
 * @param seq  Mission sequence number.
 * @param cmd  Mission command identifier.
 * @param mission_count  Total number of mission items in the mission.
 * @param progress  Current mission item progress, 0 to 100.
 * @param result  Mission execution state.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mission_report_send(mavlink_channel_t chan, uint64_t time_usec, uint16_t seq, uint16_t cmd, uint16_t mission_count, uint8_t progress, uint8_t result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_REPORT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint16_t(buf, 8, seq);
    _mav_put_uint16_t(buf, 10, cmd);
    _mav_put_uint16_t(buf, 12, mission_count);
    _mav_put_uint8_t(buf, 14, progress);
    _mav_put_uint8_t(buf, 15, result);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_REPORT, buf, MAVLINK_MSG_ID_MISSION_REPORT_MIN_LEN, MAVLINK_MSG_ID_MISSION_REPORT_LEN, MAVLINK_MSG_ID_MISSION_REPORT_CRC);
#else
    mavlink_mission_report_t packet;
    packet.time_usec = time_usec;
    packet.seq = seq;
    packet.cmd = cmd;
    packet.mission_count = mission_count;
    packet.progress = progress;
    packet.result = result;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_REPORT, (const char *)&packet, MAVLINK_MSG_ID_MISSION_REPORT_MIN_LEN, MAVLINK_MSG_ID_MISSION_REPORT_LEN, MAVLINK_MSG_ID_MISSION_REPORT_CRC);
#endif
}

/**
 * @brief Send a mission_report message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mission_report_send_struct(mavlink_channel_t chan, const mavlink_mission_report_t* mission_report)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mission_report_send(chan, mission_report->time_usec, mission_report->seq, mission_report->cmd, mission_report->mission_count, mission_report->progress, mission_report->result);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_REPORT, (const char *)mission_report, MAVLINK_MSG_ID_MISSION_REPORT_MIN_LEN, MAVLINK_MSG_ID_MISSION_REPORT_LEN, MAVLINK_MSG_ID_MISSION_REPORT_CRC);
#endif
}

#if MAVLINK_MSG_ID_MISSION_REPORT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mission_report_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint16_t seq, uint16_t cmd, uint16_t mission_count, uint8_t progress, uint8_t result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint16_t(buf, 8, seq);
    _mav_put_uint16_t(buf, 10, cmd);
    _mav_put_uint16_t(buf, 12, mission_count);
    _mav_put_uint8_t(buf, 14, progress);
    _mav_put_uint8_t(buf, 15, result);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_REPORT, buf, MAVLINK_MSG_ID_MISSION_REPORT_MIN_LEN, MAVLINK_MSG_ID_MISSION_REPORT_LEN, MAVLINK_MSG_ID_MISSION_REPORT_CRC);
#else
    mavlink_mission_report_t *packet = (mavlink_mission_report_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->seq = seq;
    packet->cmd = cmd;
    packet->mission_count = mission_count;
    packet->progress = progress;
    packet->result = result;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_REPORT, (const char *)packet, MAVLINK_MSG_ID_MISSION_REPORT_MIN_LEN, MAVLINK_MSG_ID_MISSION_REPORT_LEN, MAVLINK_MSG_ID_MISSION_REPORT_CRC);
#endif
}
#endif

#endif

// MESSAGE MISSION_REPORT UNPACKING


/**
 * @brief Get field time_usec from mission_report message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot).
 */
static inline uint64_t mavlink_msg_mission_report_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field seq from mission_report message
 *
 * @return  Mission sequence number.
 */
static inline uint16_t mavlink_msg_mission_report_get_seq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field cmd from mission_report message
 *
 * @return  Mission command identifier.
 */
static inline uint16_t mavlink_msg_mission_report_get_cmd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field mission_count from mission_report message
 *
 * @return  Total number of mission items in the mission.
 */
static inline uint16_t mavlink_msg_mission_report_get_mission_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field progress from mission_report message
 *
 * @return  Current mission item progress, 0 to 100.
 */
static inline uint8_t mavlink_msg_mission_report_get_progress(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field result from mission_report message
 *
 * @return  Mission execution state.
 */
static inline uint8_t mavlink_msg_mission_report_get_result(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  15);
}

/**
 * @brief Decode a mission_report message into a struct
 *
 * @param msg The message to decode
 * @param mission_report C-struct to decode the message contents into
 */
static inline void mavlink_msg_mission_report_decode(const mavlink_message_t* msg, mavlink_mission_report_t* mission_report)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mission_report->time_usec = mavlink_msg_mission_report_get_time_usec(msg);
    mission_report->seq = mavlink_msg_mission_report_get_seq(msg);
    mission_report->cmd = mavlink_msg_mission_report_get_cmd(msg);
    mission_report->mission_count = mavlink_msg_mission_report_get_mission_count(msg);
    mission_report->progress = mavlink_msg_mission_report_get_progress(msg);
    mission_report->result = mavlink_msg_mission_report_get_result(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MISSION_REPORT_LEN? msg->len : MAVLINK_MSG_ID_MISSION_REPORT_LEN;
        memset(mission_report, 0, MAVLINK_MSG_ID_MISSION_REPORT_LEN);
    memcpy(mission_report, _MAV_PAYLOAD(msg), len);
#endif
}
