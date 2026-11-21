#pragma once
// MESSAGE THREAD_INFO PACKING

#define MAVLINK_MSG_ID_THREAD_INFO 211


typedef struct __mavlink_thread_info_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 uint16_t remaining_stack; /*<  Remaining free stack space for the thread in native CPU words (on a 32-bit MCU one word equals 4 bytes). No standard MAVLink unit for CPU words, so the unit is omitted.*/
 uint16_t stack_size; /*<  Total stack size allocated for the thread in native CPU words. No standard MAVLink unit for CPU words, so the unit is omitted. Stack usage ratio = (stack_size - remaining_stack) / stack_size (unit cancels out).*/
 uint16_t cpu_usage; /*< [c%] Thread CPU usage over the recent measurement window in centi-percent (0-10000). 10000 = 100.00%, so two decimal places are preserved without a float field.*/
 uint8_t index; /*<  Thread index (0-based, pairs with num_threads to iterate all threads).*/
 uint8_t num_threads; /*<  Total number of threads reported by the autopilot.*/
 uint8_t priority; /*<  Thread priority (RTOS native value; higher usually means higher priority).*/
 uint8_t state; /*<  Thread scheduling state (mirrors FreeRTOS eTaskState).*/
 char name[24]; /*<  Null-terminated thread name.*/
} mavlink_thread_info_t;

#define MAVLINK_MSG_ID_THREAD_INFO_LEN 38
#define MAVLINK_MSG_ID_THREAD_INFO_MIN_LEN 38
#define MAVLINK_MSG_ID_211_LEN 38
#define MAVLINK_MSG_ID_211_MIN_LEN 38

#define MAVLINK_MSG_ID_THREAD_INFO_CRC 114
#define MAVLINK_MSG_ID_211_CRC 114

#define MAVLINK_MSG_THREAD_INFO_FIELD_NAME_LEN 24

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_THREAD_INFO { \
    211, \
    "THREAD_INFO", \
    9, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_thread_info_t, time_boot_ms) }, \
         { "remaining_stack", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_thread_info_t, remaining_stack) }, \
         { "stack_size", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_thread_info_t, stack_size) }, \
         { "cpu_usage", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_thread_info_t, cpu_usage) }, \
         { "index", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_thread_info_t, index) }, \
         { "num_threads", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_thread_info_t, num_threads) }, \
         { "priority", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_thread_info_t, priority) }, \
         { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_thread_info_t, state) }, \
         { "name", NULL, MAVLINK_TYPE_CHAR, 24, 14, offsetof(mavlink_thread_info_t, name) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_THREAD_INFO { \
    "THREAD_INFO", \
    9, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_thread_info_t, time_boot_ms) }, \
         { "remaining_stack", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_thread_info_t, remaining_stack) }, \
         { "stack_size", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_thread_info_t, stack_size) }, \
         { "cpu_usage", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_thread_info_t, cpu_usage) }, \
         { "index", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_thread_info_t, index) }, \
         { "num_threads", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_thread_info_t, num_threads) }, \
         { "priority", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_thread_info_t, priority) }, \
         { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_thread_info_t, state) }, \
         { "name", NULL, MAVLINK_TYPE_CHAR, 24, 14, offsetof(mavlink_thread_info_t, name) }, \
         } \
}
#endif

/**
 * @brief Pack a thread_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param remaining_stack  Remaining free stack space for the thread in native CPU words (on a 32-bit MCU one word equals 4 bytes). No standard MAVLink unit for CPU words, so the unit is omitted.
 * @param stack_size  Total stack size allocated for the thread in native CPU words. No standard MAVLink unit for CPU words, so the unit is omitted. Stack usage ratio = (stack_size - remaining_stack) / stack_size (unit cancels out).
 * @param cpu_usage [c%] Thread CPU usage over the recent measurement window in centi-percent (0-10000). 10000 = 100.00%, so two decimal places are preserved without a float field.
 * @param index  Thread index (0-based, pairs with num_threads to iterate all threads).
 * @param num_threads  Total number of threads reported by the autopilot.
 * @param priority  Thread priority (RTOS native value; higher usually means higher priority).
 * @param state  Thread scheduling state (mirrors FreeRTOS eTaskState).
 * @param name  Null-terminated thread name.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_thread_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint16_t remaining_stack, uint16_t stack_size, uint16_t cpu_usage, uint8_t index, uint8_t num_threads, uint8_t priority, uint8_t state, const char *name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_THREAD_INFO_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint16_t(buf, 4, remaining_stack);
    _mav_put_uint16_t(buf, 6, stack_size);
    _mav_put_uint16_t(buf, 8, cpu_usage);
    _mav_put_uint8_t(buf, 10, index);
    _mav_put_uint8_t(buf, 11, num_threads);
    _mav_put_uint8_t(buf, 12, priority);
    _mav_put_uint8_t(buf, 13, state);
    _mav_put_char_array(buf, 14, name, 24);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_THREAD_INFO_LEN);
#else
    mavlink_thread_info_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.remaining_stack = remaining_stack;
    packet.stack_size = stack_size;
    packet.cpu_usage = cpu_usage;
    packet.index = index;
    packet.num_threads = num_threads;
    packet.priority = priority;
    packet.state = state;
    mav_array_assign_char(packet.name, name, 24);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_THREAD_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_THREAD_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_THREAD_INFO_MIN_LEN, MAVLINK_MSG_ID_THREAD_INFO_LEN, MAVLINK_MSG_ID_THREAD_INFO_CRC);
}

/**
 * @brief Pack a thread_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param remaining_stack  Remaining free stack space for the thread in native CPU words (on a 32-bit MCU one word equals 4 bytes). No standard MAVLink unit for CPU words, so the unit is omitted.
 * @param stack_size  Total stack size allocated for the thread in native CPU words. No standard MAVLink unit for CPU words, so the unit is omitted. Stack usage ratio = (stack_size - remaining_stack) / stack_size (unit cancels out).
 * @param cpu_usage [c%] Thread CPU usage over the recent measurement window in centi-percent (0-10000). 10000 = 100.00%, so two decimal places are preserved without a float field.
 * @param index  Thread index (0-based, pairs with num_threads to iterate all threads).
 * @param num_threads  Total number of threads reported by the autopilot.
 * @param priority  Thread priority (RTOS native value; higher usually means higher priority).
 * @param state  Thread scheduling state (mirrors FreeRTOS eTaskState).
 * @param name  Null-terminated thread name.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_thread_info_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint16_t remaining_stack, uint16_t stack_size, uint16_t cpu_usage, uint8_t index, uint8_t num_threads, uint8_t priority, uint8_t state, const char *name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_THREAD_INFO_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint16_t(buf, 4, remaining_stack);
    _mav_put_uint16_t(buf, 6, stack_size);
    _mav_put_uint16_t(buf, 8, cpu_usage);
    _mav_put_uint8_t(buf, 10, index);
    _mav_put_uint8_t(buf, 11, num_threads);
    _mav_put_uint8_t(buf, 12, priority);
    _mav_put_uint8_t(buf, 13, state);
    _mav_put_char_array(buf, 14, name, 24);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_THREAD_INFO_LEN);
#else
    mavlink_thread_info_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.remaining_stack = remaining_stack;
    packet.stack_size = stack_size;
    packet.cpu_usage = cpu_usage;
    packet.index = index;
    packet.num_threads = num_threads;
    packet.priority = priority;
    packet.state = state;
    mav_array_memcpy(packet.name, name, sizeof(char)*24);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_THREAD_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_THREAD_INFO;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_THREAD_INFO_MIN_LEN, MAVLINK_MSG_ID_THREAD_INFO_LEN, MAVLINK_MSG_ID_THREAD_INFO_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_THREAD_INFO_MIN_LEN, MAVLINK_MSG_ID_THREAD_INFO_LEN);
#endif
}

/**
 * @brief Pack a thread_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param remaining_stack  Remaining free stack space for the thread in native CPU words (on a 32-bit MCU one word equals 4 bytes). No standard MAVLink unit for CPU words, so the unit is omitted.
 * @param stack_size  Total stack size allocated for the thread in native CPU words. No standard MAVLink unit for CPU words, so the unit is omitted. Stack usage ratio = (stack_size - remaining_stack) / stack_size (unit cancels out).
 * @param cpu_usage [c%] Thread CPU usage over the recent measurement window in centi-percent (0-10000). 10000 = 100.00%, so two decimal places are preserved without a float field.
 * @param index  Thread index (0-based, pairs with num_threads to iterate all threads).
 * @param num_threads  Total number of threads reported by the autopilot.
 * @param priority  Thread priority (RTOS native value; higher usually means higher priority).
 * @param state  Thread scheduling state (mirrors FreeRTOS eTaskState).
 * @param name  Null-terminated thread name.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_thread_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint16_t remaining_stack,uint16_t stack_size,uint16_t cpu_usage,uint8_t index,uint8_t num_threads,uint8_t priority,uint8_t state,const char *name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_THREAD_INFO_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint16_t(buf, 4, remaining_stack);
    _mav_put_uint16_t(buf, 6, stack_size);
    _mav_put_uint16_t(buf, 8, cpu_usage);
    _mav_put_uint8_t(buf, 10, index);
    _mav_put_uint8_t(buf, 11, num_threads);
    _mav_put_uint8_t(buf, 12, priority);
    _mav_put_uint8_t(buf, 13, state);
    _mav_put_char_array(buf, 14, name, 24);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_THREAD_INFO_LEN);
#else
    mavlink_thread_info_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.remaining_stack = remaining_stack;
    packet.stack_size = stack_size;
    packet.cpu_usage = cpu_usage;
    packet.index = index;
    packet.num_threads = num_threads;
    packet.priority = priority;
    packet.state = state;
    mav_array_assign_char(packet.name, name, 24);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_THREAD_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_THREAD_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_THREAD_INFO_MIN_LEN, MAVLINK_MSG_ID_THREAD_INFO_LEN, MAVLINK_MSG_ID_THREAD_INFO_CRC);
}

/**
 * @brief Encode a thread_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param thread_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_thread_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_thread_info_t* thread_info)
{
    return mavlink_msg_thread_info_pack(system_id, component_id, msg, thread_info->time_boot_ms, thread_info->remaining_stack, thread_info->stack_size, thread_info->cpu_usage, thread_info->index, thread_info->num_threads, thread_info->priority, thread_info->state, thread_info->name);
}

/**
 * @brief Encode a thread_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param thread_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_thread_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_thread_info_t* thread_info)
{
    return mavlink_msg_thread_info_pack_chan(system_id, component_id, chan, msg, thread_info->time_boot_ms, thread_info->remaining_stack, thread_info->stack_size, thread_info->cpu_usage, thread_info->index, thread_info->num_threads, thread_info->priority, thread_info->state, thread_info->name);
}

/**
 * @brief Encode a thread_info struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param thread_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_thread_info_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_thread_info_t* thread_info)
{
    return mavlink_msg_thread_info_pack_status(system_id, component_id, _status, msg,  thread_info->time_boot_ms, thread_info->remaining_stack, thread_info->stack_size, thread_info->cpu_usage, thread_info->index, thread_info->num_threads, thread_info->priority, thread_info->state, thread_info->name);
}

/**
 * @brief Send a thread_info message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param remaining_stack  Remaining free stack space for the thread in native CPU words (on a 32-bit MCU one word equals 4 bytes). No standard MAVLink unit for CPU words, so the unit is omitted.
 * @param stack_size  Total stack size allocated for the thread in native CPU words. No standard MAVLink unit for CPU words, so the unit is omitted. Stack usage ratio = (stack_size - remaining_stack) / stack_size (unit cancels out).
 * @param cpu_usage [c%] Thread CPU usage over the recent measurement window in centi-percent (0-10000). 10000 = 100.00%, so two decimal places are preserved without a float field.
 * @param index  Thread index (0-based, pairs with num_threads to iterate all threads).
 * @param num_threads  Total number of threads reported by the autopilot.
 * @param priority  Thread priority (RTOS native value; higher usually means higher priority).
 * @param state  Thread scheduling state (mirrors FreeRTOS eTaskState).
 * @param name  Null-terminated thread name.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_thread_info_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint16_t remaining_stack, uint16_t stack_size, uint16_t cpu_usage, uint8_t index, uint8_t num_threads, uint8_t priority, uint8_t state, const char *name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_THREAD_INFO_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint16_t(buf, 4, remaining_stack);
    _mav_put_uint16_t(buf, 6, stack_size);
    _mav_put_uint16_t(buf, 8, cpu_usage);
    _mav_put_uint8_t(buf, 10, index);
    _mav_put_uint8_t(buf, 11, num_threads);
    _mav_put_uint8_t(buf, 12, priority);
    _mav_put_uint8_t(buf, 13, state);
    _mav_put_char_array(buf, 14, name, 24);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THREAD_INFO, buf, MAVLINK_MSG_ID_THREAD_INFO_MIN_LEN, MAVLINK_MSG_ID_THREAD_INFO_LEN, MAVLINK_MSG_ID_THREAD_INFO_CRC);
#else
    mavlink_thread_info_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.remaining_stack = remaining_stack;
    packet.stack_size = stack_size;
    packet.cpu_usage = cpu_usage;
    packet.index = index;
    packet.num_threads = num_threads;
    packet.priority = priority;
    packet.state = state;
    mav_array_assign_char(packet.name, name, 24);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THREAD_INFO, (const char *)&packet, MAVLINK_MSG_ID_THREAD_INFO_MIN_LEN, MAVLINK_MSG_ID_THREAD_INFO_LEN, MAVLINK_MSG_ID_THREAD_INFO_CRC);
#endif
}

/**
 * @brief Send a thread_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_thread_info_send_struct(mavlink_channel_t chan, const mavlink_thread_info_t* thread_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_thread_info_send(chan, thread_info->time_boot_ms, thread_info->remaining_stack, thread_info->stack_size, thread_info->cpu_usage, thread_info->index, thread_info->num_threads, thread_info->priority, thread_info->state, thread_info->name);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THREAD_INFO, (const char *)thread_info, MAVLINK_MSG_ID_THREAD_INFO_MIN_LEN, MAVLINK_MSG_ID_THREAD_INFO_LEN, MAVLINK_MSG_ID_THREAD_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_THREAD_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_thread_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint16_t remaining_stack, uint16_t stack_size, uint16_t cpu_usage, uint8_t index, uint8_t num_threads, uint8_t priority, uint8_t state, const char *name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint16_t(buf, 4, remaining_stack);
    _mav_put_uint16_t(buf, 6, stack_size);
    _mav_put_uint16_t(buf, 8, cpu_usage);
    _mav_put_uint8_t(buf, 10, index);
    _mav_put_uint8_t(buf, 11, num_threads);
    _mav_put_uint8_t(buf, 12, priority);
    _mav_put_uint8_t(buf, 13, state);
    _mav_put_char_array(buf, 14, name, 24);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THREAD_INFO, buf, MAVLINK_MSG_ID_THREAD_INFO_MIN_LEN, MAVLINK_MSG_ID_THREAD_INFO_LEN, MAVLINK_MSG_ID_THREAD_INFO_CRC);
#else
    mavlink_thread_info_t *packet = (mavlink_thread_info_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->remaining_stack = remaining_stack;
    packet->stack_size = stack_size;
    packet->cpu_usage = cpu_usage;
    packet->index = index;
    packet->num_threads = num_threads;
    packet->priority = priority;
    packet->state = state;
    mav_array_assign_char(packet->name, name, 24);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THREAD_INFO, (const char *)packet, MAVLINK_MSG_ID_THREAD_INFO_MIN_LEN, MAVLINK_MSG_ID_THREAD_INFO_LEN, MAVLINK_MSG_ID_THREAD_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE THREAD_INFO UNPACKING


/**
 * @brief Get field time_boot_ms from thread_info message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_thread_info_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field remaining_stack from thread_info message
 *
 * @return  Remaining free stack space for the thread in native CPU words (on a 32-bit MCU one word equals 4 bytes). No standard MAVLink unit for CPU words, so the unit is omitted.
 */
static inline uint16_t mavlink_msg_thread_info_get_remaining_stack(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field stack_size from thread_info message
 *
 * @return  Total stack size allocated for the thread in native CPU words. No standard MAVLink unit for CPU words, so the unit is omitted. Stack usage ratio = (stack_size - remaining_stack) / stack_size (unit cancels out).
 */
static inline uint16_t mavlink_msg_thread_info_get_stack_size(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field cpu_usage from thread_info message
 *
 * @return [c%] Thread CPU usage over the recent measurement window in centi-percent (0-10000). 10000 = 100.00%, so two decimal places are preserved without a float field.
 */
static inline uint16_t mavlink_msg_thread_info_get_cpu_usage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field index from thread_info message
 *
 * @return  Thread index (0-based, pairs with num_threads to iterate all threads).
 */
static inline uint8_t mavlink_msg_thread_info_get_index(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field num_threads from thread_info message
 *
 * @return  Total number of threads reported by the autopilot.
 */
static inline uint8_t mavlink_msg_thread_info_get_num_threads(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field priority from thread_info message
 *
 * @return  Thread priority (RTOS native value; higher usually means higher priority).
 */
static inline uint8_t mavlink_msg_thread_info_get_priority(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field state from thread_info message
 *
 * @return  Thread scheduling state (mirrors FreeRTOS eTaskState).
 */
static inline uint8_t mavlink_msg_thread_info_get_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field name from thread_info message
 *
 * @return  Null-terminated thread name.
 */
static inline uint16_t mavlink_msg_thread_info_get_name(const mavlink_message_t* msg, char *name)
{
    return _MAV_RETURN_char_array(msg, name, 24,  14);
}

/**
 * @brief Decode a thread_info message into a struct
 *
 * @param msg The message to decode
 * @param thread_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_thread_info_decode(const mavlink_message_t* msg, mavlink_thread_info_t* thread_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    thread_info->time_boot_ms = mavlink_msg_thread_info_get_time_boot_ms(msg);
    thread_info->remaining_stack = mavlink_msg_thread_info_get_remaining_stack(msg);
    thread_info->stack_size = mavlink_msg_thread_info_get_stack_size(msg);
    thread_info->cpu_usage = mavlink_msg_thread_info_get_cpu_usage(msg);
    thread_info->index = mavlink_msg_thread_info_get_index(msg);
    thread_info->num_threads = mavlink_msg_thread_info_get_num_threads(msg);
    thread_info->priority = mavlink_msg_thread_info_get_priority(msg);
    thread_info->state = mavlink_msg_thread_info_get_state(msg);
    mavlink_msg_thread_info_get_name(msg, thread_info->name);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_THREAD_INFO_LEN? msg->len : MAVLINK_MSG_ID_THREAD_INFO_LEN;
        memset(thread_info, 0, MAVLINK_MSG_ID_THREAD_INFO_LEN);
    memcpy(thread_info, _MAV_PAYLOAD(msg), len);
#endif
}
