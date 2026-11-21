#pragma once
// MESSAGE BATTERY_STATUS_ACFLY PACKING

#define MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY 602


typedef struct __mavlink_battery_status_acfly_t {
 uint32_t voltage; /*<  Battery voltage in 100 mV (e.g. 123 means 12.3 V). No standard MAVLink unit for 100 mV, so the unit is omitted.*/
 uint32_t capacity; /*< [mAh] */
 uint32_t sequence_num; /*<  */
 uint16_t fault_bitmask; /*<  */
 int16_t temperature; /*< [deg] */
 uint16_t cycle_count; /*<  */
 int16_t current; /*<  Battery current in 100 mA (e.g. 123 means 12.3 A). No standard MAVLink unit for 100 mA, so the unit is omitted.*/
 uint8_t id; /*<  */
 uint8_t health; /*< [%] */
 uint8_t remaining_percentage; /*< [%] */
 uint8_t cell_count; /*<  */
 uint8_t voltages[48]; /*<  Per-cell voltages in 100 mV (e.g. 123 means 12.3 V). No standard MAVLink unit for 100 mV, so the unit is omitted.*/
 char serial_number[30]; /*<  Battery serial number (null-terminated string if shorter than 30 bytes).*/
} mavlink_battery_status_acfly_t;

#define MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_LEN 102
#define MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_MIN_LEN 72
#define MAVLINK_MSG_ID_602_LEN 102
#define MAVLINK_MSG_ID_602_MIN_LEN 72

#define MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_CRC 159
#define MAVLINK_MSG_ID_602_CRC 159

#define MAVLINK_MSG_BATTERY_STATUS_ACFLY_FIELD_VOLTAGES_LEN 48
#define MAVLINK_MSG_BATTERY_STATUS_ACFLY_FIELD_SERIAL_NUMBER_LEN 30

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_BATTERY_STATUS_ACFLY { \
    602, \
    "BATTERY_STATUS_ACFLY", \
    13, \
    {  { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_battery_status_acfly_t, id) }, \
         { "health", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_battery_status_acfly_t, health) }, \
         { "remaining_percentage", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_battery_status_acfly_t, remaining_percentage) }, \
         { "cell_count", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_battery_status_acfly_t, cell_count) }, \
         { "fault_bitmask", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_battery_status_acfly_t, fault_bitmask) }, \
         { "temperature", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_battery_status_acfly_t, temperature) }, \
         { "cycle_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_battery_status_acfly_t, cycle_count) }, \
         { "current", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_battery_status_acfly_t, current) }, \
         { "voltage", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_battery_status_acfly_t, voltage) }, \
         { "capacity", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_battery_status_acfly_t, capacity) }, \
         { "sequence_num", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_battery_status_acfly_t, sequence_num) }, \
         { "voltages", NULL, MAVLINK_TYPE_UINT8_T, 48, 24, offsetof(mavlink_battery_status_acfly_t, voltages) }, \
         { "serial_number", NULL, MAVLINK_TYPE_CHAR, 30, 72, offsetof(mavlink_battery_status_acfly_t, serial_number) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_BATTERY_STATUS_ACFLY { \
    "BATTERY_STATUS_ACFLY", \
    13, \
    {  { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_battery_status_acfly_t, id) }, \
         { "health", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_battery_status_acfly_t, health) }, \
         { "remaining_percentage", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_battery_status_acfly_t, remaining_percentage) }, \
         { "cell_count", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_battery_status_acfly_t, cell_count) }, \
         { "fault_bitmask", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_battery_status_acfly_t, fault_bitmask) }, \
         { "temperature", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_battery_status_acfly_t, temperature) }, \
         { "cycle_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_battery_status_acfly_t, cycle_count) }, \
         { "current", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_battery_status_acfly_t, current) }, \
         { "voltage", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_battery_status_acfly_t, voltage) }, \
         { "capacity", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_battery_status_acfly_t, capacity) }, \
         { "sequence_num", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_battery_status_acfly_t, sequence_num) }, \
         { "voltages", NULL, MAVLINK_TYPE_UINT8_T, 48, 24, offsetof(mavlink_battery_status_acfly_t, voltages) }, \
         { "serial_number", NULL, MAVLINK_TYPE_CHAR, 30, 72, offsetof(mavlink_battery_status_acfly_t, serial_number) }, \
         } \
}
#endif

/**
 * @brief Pack a battery_status_acfly message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param id  
 * @param health [%] 
 * @param remaining_percentage [%] 
 * @param cell_count  
 * @param fault_bitmask  
 * @param temperature [deg] 
 * @param cycle_count  
 * @param current  Battery current in 100 mA (e.g. 123 means 12.3 A). No standard MAVLink unit for 100 mA, so the unit is omitted.
 * @param voltage  Battery voltage in 100 mV (e.g. 123 means 12.3 V). No standard MAVLink unit for 100 mV, so the unit is omitted.
 * @param capacity [mAh] 
 * @param sequence_num  
 * @param voltages  Per-cell voltages in 100 mV (e.g. 123 means 12.3 V). No standard MAVLink unit for 100 mV, so the unit is omitted.
 * @param serial_number  Battery serial number (null-terminated string if shorter than 30 bytes).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_battery_status_acfly_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t id, uint8_t health, uint8_t remaining_percentage, uint8_t cell_count, uint16_t fault_bitmask, int16_t temperature, uint16_t cycle_count, int16_t current, uint32_t voltage, uint32_t capacity, uint32_t sequence_num, const uint8_t *voltages, const char *serial_number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_LEN];
    _mav_put_uint32_t(buf, 0, voltage);
    _mav_put_uint32_t(buf, 4, capacity);
    _mav_put_uint32_t(buf, 8, sequence_num);
    _mav_put_uint16_t(buf, 12, fault_bitmask);
    _mav_put_int16_t(buf, 14, temperature);
    _mav_put_uint16_t(buf, 16, cycle_count);
    _mav_put_int16_t(buf, 18, current);
    _mav_put_uint8_t(buf, 20, id);
    _mav_put_uint8_t(buf, 21, health);
    _mav_put_uint8_t(buf, 22, remaining_percentage);
    _mav_put_uint8_t(buf, 23, cell_count);
    _mav_put_uint8_t_array(buf, 24, voltages, 48);
    _mav_put_char_array(buf, 72, serial_number, 30);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_LEN);
#else
    mavlink_battery_status_acfly_t packet;
    packet.voltage = voltage;
    packet.capacity = capacity;
    packet.sequence_num = sequence_num;
    packet.fault_bitmask = fault_bitmask;
    packet.temperature = temperature;
    packet.cycle_count = cycle_count;
    packet.current = current;
    packet.id = id;
    packet.health = health;
    packet.remaining_percentage = remaining_percentage;
    packet.cell_count = cell_count;
    mav_array_assign_uint8_t(packet.voltages, voltages, 48);
    mav_array_assign_char(packet.serial_number, serial_number, 30);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_MIN_LEN, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_LEN, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_CRC);
}

/**
 * @brief Pack a battery_status_acfly message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param id  
 * @param health [%] 
 * @param remaining_percentage [%] 
 * @param cell_count  
 * @param fault_bitmask  
 * @param temperature [deg] 
 * @param cycle_count  
 * @param current  Battery current in 100 mA (e.g. 123 means 12.3 A). No standard MAVLink unit for 100 mA, so the unit is omitted.
 * @param voltage  Battery voltage in 100 mV (e.g. 123 means 12.3 V). No standard MAVLink unit for 100 mV, so the unit is omitted.
 * @param capacity [mAh] 
 * @param sequence_num  
 * @param voltages  Per-cell voltages in 100 mV (e.g. 123 means 12.3 V). No standard MAVLink unit for 100 mV, so the unit is omitted.
 * @param serial_number  Battery serial number (null-terminated string if shorter than 30 bytes).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_battery_status_acfly_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t id, uint8_t health, uint8_t remaining_percentage, uint8_t cell_count, uint16_t fault_bitmask, int16_t temperature, uint16_t cycle_count, int16_t current, uint32_t voltage, uint32_t capacity, uint32_t sequence_num, const uint8_t *voltages, const char *serial_number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_LEN];
    _mav_put_uint32_t(buf, 0, voltage);
    _mav_put_uint32_t(buf, 4, capacity);
    _mav_put_uint32_t(buf, 8, sequence_num);
    _mav_put_uint16_t(buf, 12, fault_bitmask);
    _mav_put_int16_t(buf, 14, temperature);
    _mav_put_uint16_t(buf, 16, cycle_count);
    _mav_put_int16_t(buf, 18, current);
    _mav_put_uint8_t(buf, 20, id);
    _mav_put_uint8_t(buf, 21, health);
    _mav_put_uint8_t(buf, 22, remaining_percentage);
    _mav_put_uint8_t(buf, 23, cell_count);
    _mav_put_uint8_t_array(buf, 24, voltages, 48);
    _mav_put_char_array(buf, 72, serial_number, 30);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_LEN);
#else
    mavlink_battery_status_acfly_t packet;
    packet.voltage = voltage;
    packet.capacity = capacity;
    packet.sequence_num = sequence_num;
    packet.fault_bitmask = fault_bitmask;
    packet.temperature = temperature;
    packet.cycle_count = cycle_count;
    packet.current = current;
    packet.id = id;
    packet.health = health;
    packet.remaining_percentage = remaining_percentage;
    packet.cell_count = cell_count;
    mav_array_memcpy(packet.voltages, voltages, sizeof(uint8_t)*48);
    mav_array_memcpy(packet.serial_number, serial_number, sizeof(char)*30);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_MIN_LEN, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_LEN, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_MIN_LEN, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_LEN);
#endif
}

/**
 * @brief Pack a battery_status_acfly message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param id  
 * @param health [%] 
 * @param remaining_percentage [%] 
 * @param cell_count  
 * @param fault_bitmask  
 * @param temperature [deg] 
 * @param cycle_count  
 * @param current  Battery current in 100 mA (e.g. 123 means 12.3 A). No standard MAVLink unit for 100 mA, so the unit is omitted.
 * @param voltage  Battery voltage in 100 mV (e.g. 123 means 12.3 V). No standard MAVLink unit for 100 mV, so the unit is omitted.
 * @param capacity [mAh] 
 * @param sequence_num  
 * @param voltages  Per-cell voltages in 100 mV (e.g. 123 means 12.3 V). No standard MAVLink unit for 100 mV, so the unit is omitted.
 * @param serial_number  Battery serial number (null-terminated string if shorter than 30 bytes).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_battery_status_acfly_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t id,uint8_t health,uint8_t remaining_percentage,uint8_t cell_count,uint16_t fault_bitmask,int16_t temperature,uint16_t cycle_count,int16_t current,uint32_t voltage,uint32_t capacity,uint32_t sequence_num,const uint8_t *voltages,const char *serial_number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_LEN];
    _mav_put_uint32_t(buf, 0, voltage);
    _mav_put_uint32_t(buf, 4, capacity);
    _mav_put_uint32_t(buf, 8, sequence_num);
    _mav_put_uint16_t(buf, 12, fault_bitmask);
    _mav_put_int16_t(buf, 14, temperature);
    _mav_put_uint16_t(buf, 16, cycle_count);
    _mav_put_int16_t(buf, 18, current);
    _mav_put_uint8_t(buf, 20, id);
    _mav_put_uint8_t(buf, 21, health);
    _mav_put_uint8_t(buf, 22, remaining_percentage);
    _mav_put_uint8_t(buf, 23, cell_count);
    _mav_put_uint8_t_array(buf, 24, voltages, 48);
    _mav_put_char_array(buf, 72, serial_number, 30);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_LEN);
#else
    mavlink_battery_status_acfly_t packet;
    packet.voltage = voltage;
    packet.capacity = capacity;
    packet.sequence_num = sequence_num;
    packet.fault_bitmask = fault_bitmask;
    packet.temperature = temperature;
    packet.cycle_count = cycle_count;
    packet.current = current;
    packet.id = id;
    packet.health = health;
    packet.remaining_percentage = remaining_percentage;
    packet.cell_count = cell_count;
    mav_array_assign_uint8_t(packet.voltages, voltages, 48);
    mav_array_assign_char(packet.serial_number, serial_number, 30);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_MIN_LEN, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_LEN, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_CRC);
}

/**
 * @brief Encode a battery_status_acfly struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param battery_status_acfly C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_battery_status_acfly_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_battery_status_acfly_t* battery_status_acfly)
{
    return mavlink_msg_battery_status_acfly_pack(system_id, component_id, msg, battery_status_acfly->id, battery_status_acfly->health, battery_status_acfly->remaining_percentage, battery_status_acfly->cell_count, battery_status_acfly->fault_bitmask, battery_status_acfly->temperature, battery_status_acfly->cycle_count, battery_status_acfly->current, battery_status_acfly->voltage, battery_status_acfly->capacity, battery_status_acfly->sequence_num, battery_status_acfly->voltages, battery_status_acfly->serial_number);
}

/**
 * @brief Encode a battery_status_acfly struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param battery_status_acfly C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_battery_status_acfly_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_battery_status_acfly_t* battery_status_acfly)
{
    return mavlink_msg_battery_status_acfly_pack_chan(system_id, component_id, chan, msg, battery_status_acfly->id, battery_status_acfly->health, battery_status_acfly->remaining_percentage, battery_status_acfly->cell_count, battery_status_acfly->fault_bitmask, battery_status_acfly->temperature, battery_status_acfly->cycle_count, battery_status_acfly->current, battery_status_acfly->voltage, battery_status_acfly->capacity, battery_status_acfly->sequence_num, battery_status_acfly->voltages, battery_status_acfly->serial_number);
}

/**
 * @brief Encode a battery_status_acfly struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param battery_status_acfly C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_battery_status_acfly_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_battery_status_acfly_t* battery_status_acfly)
{
    return mavlink_msg_battery_status_acfly_pack_status(system_id, component_id, _status, msg,  battery_status_acfly->id, battery_status_acfly->health, battery_status_acfly->remaining_percentage, battery_status_acfly->cell_count, battery_status_acfly->fault_bitmask, battery_status_acfly->temperature, battery_status_acfly->cycle_count, battery_status_acfly->current, battery_status_acfly->voltage, battery_status_acfly->capacity, battery_status_acfly->sequence_num, battery_status_acfly->voltages, battery_status_acfly->serial_number);
}

/**
 * @brief Send a battery_status_acfly message
 * @param chan MAVLink channel to send the message
 *
 * @param id  
 * @param health [%] 
 * @param remaining_percentage [%] 
 * @param cell_count  
 * @param fault_bitmask  
 * @param temperature [deg] 
 * @param cycle_count  
 * @param current  Battery current in 100 mA (e.g. 123 means 12.3 A). No standard MAVLink unit for 100 mA, so the unit is omitted.
 * @param voltage  Battery voltage in 100 mV (e.g. 123 means 12.3 V). No standard MAVLink unit for 100 mV, so the unit is omitted.
 * @param capacity [mAh] 
 * @param sequence_num  
 * @param voltages  Per-cell voltages in 100 mV (e.g. 123 means 12.3 V). No standard MAVLink unit for 100 mV, so the unit is omitted.
 * @param serial_number  Battery serial number (null-terminated string if shorter than 30 bytes).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_battery_status_acfly_send(mavlink_channel_t chan, uint8_t id, uint8_t health, uint8_t remaining_percentage, uint8_t cell_count, uint16_t fault_bitmask, int16_t temperature, uint16_t cycle_count, int16_t current, uint32_t voltage, uint32_t capacity, uint32_t sequence_num, const uint8_t *voltages, const char *serial_number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_LEN];
    _mav_put_uint32_t(buf, 0, voltage);
    _mav_put_uint32_t(buf, 4, capacity);
    _mav_put_uint32_t(buf, 8, sequence_num);
    _mav_put_uint16_t(buf, 12, fault_bitmask);
    _mav_put_int16_t(buf, 14, temperature);
    _mav_put_uint16_t(buf, 16, cycle_count);
    _mav_put_int16_t(buf, 18, current);
    _mav_put_uint8_t(buf, 20, id);
    _mav_put_uint8_t(buf, 21, health);
    _mav_put_uint8_t(buf, 22, remaining_percentage);
    _mav_put_uint8_t(buf, 23, cell_count);
    _mav_put_uint8_t_array(buf, 24, voltages, 48);
    _mav_put_char_array(buf, 72, serial_number, 30);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY, buf, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_MIN_LEN, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_LEN, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_CRC);
#else
    mavlink_battery_status_acfly_t packet;
    packet.voltage = voltage;
    packet.capacity = capacity;
    packet.sequence_num = sequence_num;
    packet.fault_bitmask = fault_bitmask;
    packet.temperature = temperature;
    packet.cycle_count = cycle_count;
    packet.current = current;
    packet.id = id;
    packet.health = health;
    packet.remaining_percentage = remaining_percentage;
    packet.cell_count = cell_count;
    mav_array_assign_uint8_t(packet.voltages, voltages, 48);
    mav_array_assign_char(packet.serial_number, serial_number, 30);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY, (const char *)&packet, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_MIN_LEN, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_LEN, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_CRC);
#endif
}

/**
 * @brief Send a battery_status_acfly message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_battery_status_acfly_send_struct(mavlink_channel_t chan, const mavlink_battery_status_acfly_t* battery_status_acfly)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_battery_status_acfly_send(chan, battery_status_acfly->id, battery_status_acfly->health, battery_status_acfly->remaining_percentage, battery_status_acfly->cell_count, battery_status_acfly->fault_bitmask, battery_status_acfly->temperature, battery_status_acfly->cycle_count, battery_status_acfly->current, battery_status_acfly->voltage, battery_status_acfly->capacity, battery_status_acfly->sequence_num, battery_status_acfly->voltages, battery_status_acfly->serial_number);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY, (const char *)battery_status_acfly, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_MIN_LEN, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_LEN, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_CRC);
#endif
}

#if MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_battery_status_acfly_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t id, uint8_t health, uint8_t remaining_percentage, uint8_t cell_count, uint16_t fault_bitmask, int16_t temperature, uint16_t cycle_count, int16_t current, uint32_t voltage, uint32_t capacity, uint32_t sequence_num, const uint8_t *voltages, const char *serial_number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, voltage);
    _mav_put_uint32_t(buf, 4, capacity);
    _mav_put_uint32_t(buf, 8, sequence_num);
    _mav_put_uint16_t(buf, 12, fault_bitmask);
    _mav_put_int16_t(buf, 14, temperature);
    _mav_put_uint16_t(buf, 16, cycle_count);
    _mav_put_int16_t(buf, 18, current);
    _mav_put_uint8_t(buf, 20, id);
    _mav_put_uint8_t(buf, 21, health);
    _mav_put_uint8_t(buf, 22, remaining_percentage);
    _mav_put_uint8_t(buf, 23, cell_count);
    _mav_put_uint8_t_array(buf, 24, voltages, 48);
    _mav_put_char_array(buf, 72, serial_number, 30);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY, buf, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_MIN_LEN, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_LEN, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_CRC);
#else
    mavlink_battery_status_acfly_t *packet = (mavlink_battery_status_acfly_t *)msgbuf;
    packet->voltage = voltage;
    packet->capacity = capacity;
    packet->sequence_num = sequence_num;
    packet->fault_bitmask = fault_bitmask;
    packet->temperature = temperature;
    packet->cycle_count = cycle_count;
    packet->current = current;
    packet->id = id;
    packet->health = health;
    packet->remaining_percentage = remaining_percentage;
    packet->cell_count = cell_count;
    mav_array_assign_uint8_t(packet->voltages, voltages, 48);
    mav_array_assign_char(packet->serial_number, serial_number, 30);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY, (const char *)packet, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_MIN_LEN, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_LEN, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_CRC);
#endif
}
#endif

#endif

// MESSAGE BATTERY_STATUS_ACFLY UNPACKING


/**
 * @brief Get field id from battery_status_acfly message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_battery_status_acfly_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field health from battery_status_acfly message
 *
 * @return [%] 
 */
static inline uint8_t mavlink_msg_battery_status_acfly_get_health(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Get field remaining_percentage from battery_status_acfly message
 *
 * @return [%] 
 */
static inline uint8_t mavlink_msg_battery_status_acfly_get_remaining_percentage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Get field cell_count from battery_status_acfly message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_battery_status_acfly_get_cell_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  23);
}

/**
 * @brief Get field fault_bitmask from battery_status_acfly message
 *
 * @return  
 */
static inline uint16_t mavlink_msg_battery_status_acfly_get_fault_bitmask(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field temperature from battery_status_acfly message
 *
 * @return [deg] 
 */
static inline int16_t mavlink_msg_battery_status_acfly_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field cycle_count from battery_status_acfly message
 *
 * @return  
 */
static inline uint16_t mavlink_msg_battery_status_acfly_get_cycle_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field current from battery_status_acfly message
 *
 * @return  Battery current in 100 mA (e.g. 123 means 12.3 A). No standard MAVLink unit for 100 mA, so the unit is omitted.
 */
static inline int16_t mavlink_msg_battery_status_acfly_get_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  18);
}

/**
 * @brief Get field voltage from battery_status_acfly message
 *
 * @return  Battery voltage in 100 mV (e.g. 123 means 12.3 V). No standard MAVLink unit for 100 mV, so the unit is omitted.
 */
static inline uint32_t mavlink_msg_battery_status_acfly_get_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field capacity from battery_status_acfly message
 *
 * @return [mAh] 
 */
static inline uint32_t mavlink_msg_battery_status_acfly_get_capacity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field sequence_num from battery_status_acfly message
 *
 * @return  
 */
static inline uint32_t mavlink_msg_battery_status_acfly_get_sequence_num(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field voltages from battery_status_acfly message
 *
 * @return  Per-cell voltages in 100 mV (e.g. 123 means 12.3 V). No standard MAVLink unit for 100 mV, so the unit is omitted.
 */
static inline uint16_t mavlink_msg_battery_status_acfly_get_voltages(const mavlink_message_t* msg, uint8_t *voltages)
{
    return _MAV_RETURN_uint8_t_array(msg, voltages, 48,  24);
}

/**
 * @brief Get field serial_number from battery_status_acfly message
 *
 * @return  Battery serial number (null-terminated string if shorter than 30 bytes).
 */
static inline uint16_t mavlink_msg_battery_status_acfly_get_serial_number(const mavlink_message_t* msg, char *serial_number)
{
    return _MAV_RETURN_char_array(msg, serial_number, 30,  72);
}

/**
 * @brief Decode a battery_status_acfly message into a struct
 *
 * @param msg The message to decode
 * @param battery_status_acfly C-struct to decode the message contents into
 */
static inline void mavlink_msg_battery_status_acfly_decode(const mavlink_message_t* msg, mavlink_battery_status_acfly_t* battery_status_acfly)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    battery_status_acfly->voltage = mavlink_msg_battery_status_acfly_get_voltage(msg);
    battery_status_acfly->capacity = mavlink_msg_battery_status_acfly_get_capacity(msg);
    battery_status_acfly->sequence_num = mavlink_msg_battery_status_acfly_get_sequence_num(msg);
    battery_status_acfly->fault_bitmask = mavlink_msg_battery_status_acfly_get_fault_bitmask(msg);
    battery_status_acfly->temperature = mavlink_msg_battery_status_acfly_get_temperature(msg);
    battery_status_acfly->cycle_count = mavlink_msg_battery_status_acfly_get_cycle_count(msg);
    battery_status_acfly->current = mavlink_msg_battery_status_acfly_get_current(msg);
    battery_status_acfly->id = mavlink_msg_battery_status_acfly_get_id(msg);
    battery_status_acfly->health = mavlink_msg_battery_status_acfly_get_health(msg);
    battery_status_acfly->remaining_percentage = mavlink_msg_battery_status_acfly_get_remaining_percentage(msg);
    battery_status_acfly->cell_count = mavlink_msg_battery_status_acfly_get_cell_count(msg);
    mavlink_msg_battery_status_acfly_get_voltages(msg, battery_status_acfly->voltages);
    mavlink_msg_battery_status_acfly_get_serial_number(msg, battery_status_acfly->serial_number);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_LEN? msg->len : MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_LEN;
        memset(battery_status_acfly, 0, MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_LEN);
    memcpy(battery_status_acfly, _MAV_PAYLOAD(msg), len);
#endif
}
