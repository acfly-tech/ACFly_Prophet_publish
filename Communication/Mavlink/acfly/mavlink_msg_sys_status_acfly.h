#pragma once
// MESSAGE SYS_STATUS_ACFLY PACKING

#define MAVLINK_MSG_ID_SYS_STATUS_ACFLY 200


typedef struct __mavlink_sys_status_acfly_t {
 uint32_t remaining_memory; /*<  Remaining free OS memory in native CPU words (on a 32-bit MCU one word equals 4 bytes). No standard MAVLink unit for CPU words, so the unit is omitted.*/
 uint32_t sysStatusFlag; /*<  .*/
 uint32_t sysStatusFlag2; /*<  .*/
 uint32_t pos_cflags; /*<  */
 uint32_t pos_aflags; /*<  */
 uint16_t cpu_usage; /*< [d%] Maximum usage in percent of the mainloop time. Units: 0.1 % (e.g. 123 means 12.3 %). Values: [0-1000] - should always be below 1000.*/
 int16_t daoYaw[2]; /*<  Heading measured by direction-finding sensor for each array element, in 0.1 deg (e.g. 1234 means 123.4 deg). -1000 = no direction-finding sensor, 1000 = direction-finding sensor present but unavailable, 0-3600 = direction-finding sensor normal.*/
 uint16_t adc_voltage[2]; /*<  Raw ADC voltage readings in 0.1 V (e.g. 123 means 12.3 V). No standard MAVLink unit for 0.1 V, so the unit is omitted.*/
 uint16_t adc_current[2]; /*< [cA] Raw ADC current readings. Units: 0.01 A (e.g. 123 means 1.23 A). 0 = no current sensor on this channel.*/
 uint16_t sd_card_capacity; /*<  Total capacity of the SD card in 10 MB (e.g. 123 means 1230 MB). No standard MAVLink unit for 10 MB, so the unit is omitted. 0 = no SD card inserted or capacity unknown.*/
 uint16_t sd_card_remain; /*<  Remaining free capacity of the SD card in 10 MB (e.g. 123 means 1230 MB). No standard MAVLink unit for 10 MB, so the unit is omitted. 0 = no SD card inserted or free capacity unknown.*/
 uint16_t imu_cflags; /*<  */
 uint16_t imu_aflags; /*<  */
 uint16_t mode_code; /*<  Current flight state code (MODE_CODE). Values map one-to-one to the DJI Cloud API mode_code property. Extension field (MAVLink 2 only).*/
 uint16_t mode_code_reason; /*<  Reason for entering the current flight state (MODE_CODE_REASON). Values below 100 map one-to-one to the DJI Cloud API mode_code_reason property, values 100 and above are ACFly vendor-specific extensions. Extension field (MAVLink 2 only).*/
} mavlink_sys_status_acfly_t;

#define MAVLINK_MSG_ID_SYS_STATUS_ACFLY_LEN 46
#define MAVLINK_MSG_ID_SYS_STATUS_ACFLY_MIN_LEN 42
#define MAVLINK_MSG_ID_200_LEN 46
#define MAVLINK_MSG_ID_200_MIN_LEN 42

#define MAVLINK_MSG_ID_SYS_STATUS_ACFLY_CRC 127
#define MAVLINK_MSG_ID_200_CRC 127

#define MAVLINK_MSG_SYS_STATUS_ACFLY_FIELD_DAOYAW_LEN 2
#define MAVLINK_MSG_SYS_STATUS_ACFLY_FIELD_ADC_VOLTAGE_LEN 2
#define MAVLINK_MSG_SYS_STATUS_ACFLY_FIELD_ADC_CURRENT_LEN 2

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SYS_STATUS_ACFLY { \
    200, \
    "SYS_STATUS_ACFLY", \
    15, \
    {  { "cpu_usage", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_sys_status_acfly_t, cpu_usage) }, \
         { "remaining_memory", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_sys_status_acfly_t, remaining_memory) }, \
         { "daoYaw", NULL, MAVLINK_TYPE_INT16_T, 2, 22, offsetof(mavlink_sys_status_acfly_t, daoYaw) }, \
         { "adc_voltage", NULL, MAVLINK_TYPE_UINT16_T, 2, 26, offsetof(mavlink_sys_status_acfly_t, adc_voltage) }, \
         { "adc_current", NULL, MAVLINK_TYPE_UINT16_T, 2, 30, offsetof(mavlink_sys_status_acfly_t, adc_current) }, \
         { "sd_card_capacity", NULL, MAVLINK_TYPE_UINT16_T, 0, 34, offsetof(mavlink_sys_status_acfly_t, sd_card_capacity) }, \
         { "sd_card_remain", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_sys_status_acfly_t, sd_card_remain) }, \
         { "sysStatusFlag", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_sys_status_acfly_t, sysStatusFlag) }, \
         { "sysStatusFlag2", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_sys_status_acfly_t, sysStatusFlag2) }, \
         { "pos_cflags", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_sys_status_acfly_t, pos_cflags) }, \
         { "pos_aflags", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_sys_status_acfly_t, pos_aflags) }, \
         { "imu_cflags", NULL, MAVLINK_TYPE_UINT16_T, 0, 38, offsetof(mavlink_sys_status_acfly_t, imu_cflags) }, \
         { "imu_aflags", NULL, MAVLINK_TYPE_UINT16_T, 0, 40, offsetof(mavlink_sys_status_acfly_t, imu_aflags) }, \
         { "mode_code", NULL, MAVLINK_TYPE_UINT16_T, 0, 42, offsetof(mavlink_sys_status_acfly_t, mode_code) }, \
         { "mode_code_reason", NULL, MAVLINK_TYPE_UINT16_T, 0, 44, offsetof(mavlink_sys_status_acfly_t, mode_code_reason) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SYS_STATUS_ACFLY { \
    "SYS_STATUS_ACFLY", \
    15, \
    {  { "cpu_usage", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_sys_status_acfly_t, cpu_usage) }, \
         { "remaining_memory", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_sys_status_acfly_t, remaining_memory) }, \
         { "daoYaw", NULL, MAVLINK_TYPE_INT16_T, 2, 22, offsetof(mavlink_sys_status_acfly_t, daoYaw) }, \
         { "adc_voltage", NULL, MAVLINK_TYPE_UINT16_T, 2, 26, offsetof(mavlink_sys_status_acfly_t, adc_voltage) }, \
         { "adc_current", NULL, MAVLINK_TYPE_UINT16_T, 2, 30, offsetof(mavlink_sys_status_acfly_t, adc_current) }, \
         { "sd_card_capacity", NULL, MAVLINK_TYPE_UINT16_T, 0, 34, offsetof(mavlink_sys_status_acfly_t, sd_card_capacity) }, \
         { "sd_card_remain", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_sys_status_acfly_t, sd_card_remain) }, \
         { "sysStatusFlag", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_sys_status_acfly_t, sysStatusFlag) }, \
         { "sysStatusFlag2", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_sys_status_acfly_t, sysStatusFlag2) }, \
         { "pos_cflags", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_sys_status_acfly_t, pos_cflags) }, \
         { "pos_aflags", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_sys_status_acfly_t, pos_aflags) }, \
         { "imu_cflags", NULL, MAVLINK_TYPE_UINT16_T, 0, 38, offsetof(mavlink_sys_status_acfly_t, imu_cflags) }, \
         { "imu_aflags", NULL, MAVLINK_TYPE_UINT16_T, 0, 40, offsetof(mavlink_sys_status_acfly_t, imu_aflags) }, \
         { "mode_code", NULL, MAVLINK_TYPE_UINT16_T, 0, 42, offsetof(mavlink_sys_status_acfly_t, mode_code) }, \
         { "mode_code_reason", NULL, MAVLINK_TYPE_UINT16_T, 0, 44, offsetof(mavlink_sys_status_acfly_t, mode_code_reason) }, \
         } \
}
#endif

/**
 * @brief Pack a sys_status_acfly message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param cpu_usage [d%] Maximum usage in percent of the mainloop time. Units: 0.1 % (e.g. 123 means 12.3 %). Values: [0-1000] - should always be below 1000.
 * @param remaining_memory  Remaining free OS memory in native CPU words (on a 32-bit MCU one word equals 4 bytes). No standard MAVLink unit for CPU words, so the unit is omitted.
 * @param daoYaw  Heading measured by direction-finding sensor for each array element, in 0.1 deg (e.g. 1234 means 123.4 deg). -1000 = no direction-finding sensor, 1000 = direction-finding sensor present but unavailable, 0-3600 = direction-finding sensor normal.
 * @param adc_voltage  Raw ADC voltage readings in 0.1 V (e.g. 123 means 12.3 V). No standard MAVLink unit for 0.1 V, so the unit is omitted.
 * @param adc_current [cA] Raw ADC current readings. Units: 0.01 A (e.g. 123 means 1.23 A). 0 = no current sensor on this channel.
 * @param sd_card_capacity  Total capacity of the SD card in 10 MB (e.g. 123 means 1230 MB). No standard MAVLink unit for 10 MB, so the unit is omitted. 0 = no SD card inserted or capacity unknown.
 * @param sd_card_remain  Remaining free capacity of the SD card in 10 MB (e.g. 123 means 1230 MB). No standard MAVLink unit for 10 MB, so the unit is omitted. 0 = no SD card inserted or free capacity unknown.
 * @param sysStatusFlag  .
 * @param sysStatusFlag2  .
 * @param pos_cflags  
 * @param pos_aflags  
 * @param imu_cflags  
 * @param imu_aflags  
 * @param mode_code  Current flight state code (MODE_CODE). Values map one-to-one to the DJI Cloud API mode_code property. Extension field (MAVLink 2 only).
 * @param mode_code_reason  Reason for entering the current flight state (MODE_CODE_REASON). Values below 100 map one-to-one to the DJI Cloud API mode_code_reason property, values 100 and above are ACFly vendor-specific extensions. Extension field (MAVLink 2 only).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sys_status_acfly_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t cpu_usage, uint32_t remaining_memory, const int16_t *daoYaw, const uint16_t *adc_voltage, const uint16_t *adc_current, uint16_t sd_card_capacity, uint16_t sd_card_remain, uint32_t sysStatusFlag, uint32_t sysStatusFlag2, uint32_t pos_cflags, uint32_t pos_aflags, uint16_t imu_cflags, uint16_t imu_aflags, uint16_t mode_code, uint16_t mode_code_reason)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SYS_STATUS_ACFLY_LEN];
    _mav_put_uint32_t(buf, 0, remaining_memory);
    _mav_put_uint32_t(buf, 4, sysStatusFlag);
    _mav_put_uint32_t(buf, 8, sysStatusFlag2);
    _mav_put_uint32_t(buf, 12, pos_cflags);
    _mav_put_uint32_t(buf, 16, pos_aflags);
    _mav_put_uint16_t(buf, 20, cpu_usage);
    _mav_put_uint16_t(buf, 34, sd_card_capacity);
    _mav_put_uint16_t(buf, 36, sd_card_remain);
    _mav_put_uint16_t(buf, 38, imu_cflags);
    _mav_put_uint16_t(buf, 40, imu_aflags);
    _mav_put_uint16_t(buf, 42, mode_code);
    _mav_put_uint16_t(buf, 44, mode_code_reason);
    _mav_put_int16_t_array(buf, 22, daoYaw, 2);
    _mav_put_uint16_t_array(buf, 26, adc_voltage, 2);
    _mav_put_uint16_t_array(buf, 30, adc_current, 2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SYS_STATUS_ACFLY_LEN);
#else
    mavlink_sys_status_acfly_t packet;
    packet.remaining_memory = remaining_memory;
    packet.sysStatusFlag = sysStatusFlag;
    packet.sysStatusFlag2 = sysStatusFlag2;
    packet.pos_cflags = pos_cflags;
    packet.pos_aflags = pos_aflags;
    packet.cpu_usage = cpu_usage;
    packet.sd_card_capacity = sd_card_capacity;
    packet.sd_card_remain = sd_card_remain;
    packet.imu_cflags = imu_cflags;
    packet.imu_aflags = imu_aflags;
    packet.mode_code = mode_code;
    packet.mode_code_reason = mode_code_reason;
    mav_array_assign_int16_t(packet.daoYaw, daoYaw, 2);
    mav_array_assign_uint16_t(packet.adc_voltage, adc_voltage, 2);
    mav_array_assign_uint16_t(packet.adc_current, adc_current, 2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SYS_STATUS_ACFLY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SYS_STATUS_ACFLY;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SYS_STATUS_ACFLY_MIN_LEN, MAVLINK_MSG_ID_SYS_STATUS_ACFLY_LEN, MAVLINK_MSG_ID_SYS_STATUS_ACFLY_CRC);
}

/**
 * @brief Pack a sys_status_acfly message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param cpu_usage [d%] Maximum usage in percent of the mainloop time. Units: 0.1 % (e.g. 123 means 12.3 %). Values: [0-1000] - should always be below 1000.
 * @param remaining_memory  Remaining free OS memory in native CPU words (on a 32-bit MCU one word equals 4 bytes). No standard MAVLink unit for CPU words, so the unit is omitted.
 * @param daoYaw  Heading measured by direction-finding sensor for each array element, in 0.1 deg (e.g. 1234 means 123.4 deg). -1000 = no direction-finding sensor, 1000 = direction-finding sensor present but unavailable, 0-3600 = direction-finding sensor normal.
 * @param adc_voltage  Raw ADC voltage readings in 0.1 V (e.g. 123 means 12.3 V). No standard MAVLink unit for 0.1 V, so the unit is omitted.
 * @param adc_current [cA] Raw ADC current readings. Units: 0.01 A (e.g. 123 means 1.23 A). 0 = no current sensor on this channel.
 * @param sd_card_capacity  Total capacity of the SD card in 10 MB (e.g. 123 means 1230 MB). No standard MAVLink unit for 10 MB, so the unit is omitted. 0 = no SD card inserted or capacity unknown.
 * @param sd_card_remain  Remaining free capacity of the SD card in 10 MB (e.g. 123 means 1230 MB). No standard MAVLink unit for 10 MB, so the unit is omitted. 0 = no SD card inserted or free capacity unknown.
 * @param sysStatusFlag  .
 * @param sysStatusFlag2  .
 * @param pos_cflags  
 * @param pos_aflags  
 * @param imu_cflags  
 * @param imu_aflags  
 * @param mode_code  Current flight state code (MODE_CODE). Values map one-to-one to the DJI Cloud API mode_code property. Extension field (MAVLink 2 only).
 * @param mode_code_reason  Reason for entering the current flight state (MODE_CODE_REASON). Values below 100 map one-to-one to the DJI Cloud API mode_code_reason property, values 100 and above are ACFly vendor-specific extensions. Extension field (MAVLink 2 only).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sys_status_acfly_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint16_t cpu_usage, uint32_t remaining_memory, const int16_t *daoYaw, const uint16_t *adc_voltage, const uint16_t *adc_current, uint16_t sd_card_capacity, uint16_t sd_card_remain, uint32_t sysStatusFlag, uint32_t sysStatusFlag2, uint32_t pos_cflags, uint32_t pos_aflags, uint16_t imu_cflags, uint16_t imu_aflags, uint16_t mode_code, uint16_t mode_code_reason)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SYS_STATUS_ACFLY_LEN];
    _mav_put_uint32_t(buf, 0, remaining_memory);
    _mav_put_uint32_t(buf, 4, sysStatusFlag);
    _mav_put_uint32_t(buf, 8, sysStatusFlag2);
    _mav_put_uint32_t(buf, 12, pos_cflags);
    _mav_put_uint32_t(buf, 16, pos_aflags);
    _mav_put_uint16_t(buf, 20, cpu_usage);
    _mav_put_uint16_t(buf, 34, sd_card_capacity);
    _mav_put_uint16_t(buf, 36, sd_card_remain);
    _mav_put_uint16_t(buf, 38, imu_cflags);
    _mav_put_uint16_t(buf, 40, imu_aflags);
    _mav_put_uint16_t(buf, 42, mode_code);
    _mav_put_uint16_t(buf, 44, mode_code_reason);
    _mav_put_int16_t_array(buf, 22, daoYaw, 2);
    _mav_put_uint16_t_array(buf, 26, adc_voltage, 2);
    _mav_put_uint16_t_array(buf, 30, adc_current, 2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SYS_STATUS_ACFLY_LEN);
#else
    mavlink_sys_status_acfly_t packet;
    packet.remaining_memory = remaining_memory;
    packet.sysStatusFlag = sysStatusFlag;
    packet.sysStatusFlag2 = sysStatusFlag2;
    packet.pos_cflags = pos_cflags;
    packet.pos_aflags = pos_aflags;
    packet.cpu_usage = cpu_usage;
    packet.sd_card_capacity = sd_card_capacity;
    packet.sd_card_remain = sd_card_remain;
    packet.imu_cflags = imu_cflags;
    packet.imu_aflags = imu_aflags;
    packet.mode_code = mode_code;
    packet.mode_code_reason = mode_code_reason;
    mav_array_memcpy(packet.daoYaw, daoYaw, sizeof(int16_t)*2);
    mav_array_memcpy(packet.adc_voltage, adc_voltage, sizeof(uint16_t)*2);
    mav_array_memcpy(packet.adc_current, adc_current, sizeof(uint16_t)*2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SYS_STATUS_ACFLY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SYS_STATUS_ACFLY;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SYS_STATUS_ACFLY_MIN_LEN, MAVLINK_MSG_ID_SYS_STATUS_ACFLY_LEN, MAVLINK_MSG_ID_SYS_STATUS_ACFLY_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SYS_STATUS_ACFLY_MIN_LEN, MAVLINK_MSG_ID_SYS_STATUS_ACFLY_LEN);
#endif
}

/**
 * @brief Pack a sys_status_acfly message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param cpu_usage [d%] Maximum usage in percent of the mainloop time. Units: 0.1 % (e.g. 123 means 12.3 %). Values: [0-1000] - should always be below 1000.
 * @param remaining_memory  Remaining free OS memory in native CPU words (on a 32-bit MCU one word equals 4 bytes). No standard MAVLink unit for CPU words, so the unit is omitted.
 * @param daoYaw  Heading measured by direction-finding sensor for each array element, in 0.1 deg (e.g. 1234 means 123.4 deg). -1000 = no direction-finding sensor, 1000 = direction-finding sensor present but unavailable, 0-3600 = direction-finding sensor normal.
 * @param adc_voltage  Raw ADC voltage readings in 0.1 V (e.g. 123 means 12.3 V). No standard MAVLink unit for 0.1 V, so the unit is omitted.
 * @param adc_current [cA] Raw ADC current readings. Units: 0.01 A (e.g. 123 means 1.23 A). 0 = no current sensor on this channel.
 * @param sd_card_capacity  Total capacity of the SD card in 10 MB (e.g. 123 means 1230 MB). No standard MAVLink unit for 10 MB, so the unit is omitted. 0 = no SD card inserted or capacity unknown.
 * @param sd_card_remain  Remaining free capacity of the SD card in 10 MB (e.g. 123 means 1230 MB). No standard MAVLink unit for 10 MB, so the unit is omitted. 0 = no SD card inserted or free capacity unknown.
 * @param sysStatusFlag  .
 * @param sysStatusFlag2  .
 * @param pos_cflags  
 * @param pos_aflags  
 * @param imu_cflags  
 * @param imu_aflags  
 * @param mode_code  Current flight state code (MODE_CODE). Values map one-to-one to the DJI Cloud API mode_code property. Extension field (MAVLink 2 only).
 * @param mode_code_reason  Reason for entering the current flight state (MODE_CODE_REASON). Values below 100 map one-to-one to the DJI Cloud API mode_code_reason property, values 100 and above are ACFly vendor-specific extensions. Extension field (MAVLink 2 only).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sys_status_acfly_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t cpu_usage,uint32_t remaining_memory,const int16_t *daoYaw,const uint16_t *adc_voltage,const uint16_t *adc_current,uint16_t sd_card_capacity,uint16_t sd_card_remain,uint32_t sysStatusFlag,uint32_t sysStatusFlag2,uint32_t pos_cflags,uint32_t pos_aflags,uint16_t imu_cflags,uint16_t imu_aflags,uint16_t mode_code,uint16_t mode_code_reason)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SYS_STATUS_ACFLY_LEN];
    _mav_put_uint32_t(buf, 0, remaining_memory);
    _mav_put_uint32_t(buf, 4, sysStatusFlag);
    _mav_put_uint32_t(buf, 8, sysStatusFlag2);
    _mav_put_uint32_t(buf, 12, pos_cflags);
    _mav_put_uint32_t(buf, 16, pos_aflags);
    _mav_put_uint16_t(buf, 20, cpu_usage);
    _mav_put_uint16_t(buf, 34, sd_card_capacity);
    _mav_put_uint16_t(buf, 36, sd_card_remain);
    _mav_put_uint16_t(buf, 38, imu_cflags);
    _mav_put_uint16_t(buf, 40, imu_aflags);
    _mav_put_uint16_t(buf, 42, mode_code);
    _mav_put_uint16_t(buf, 44, mode_code_reason);
    _mav_put_int16_t_array(buf, 22, daoYaw, 2);
    _mav_put_uint16_t_array(buf, 26, adc_voltage, 2);
    _mav_put_uint16_t_array(buf, 30, adc_current, 2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SYS_STATUS_ACFLY_LEN);
#else
    mavlink_sys_status_acfly_t packet;
    packet.remaining_memory = remaining_memory;
    packet.sysStatusFlag = sysStatusFlag;
    packet.sysStatusFlag2 = sysStatusFlag2;
    packet.pos_cflags = pos_cflags;
    packet.pos_aflags = pos_aflags;
    packet.cpu_usage = cpu_usage;
    packet.sd_card_capacity = sd_card_capacity;
    packet.sd_card_remain = sd_card_remain;
    packet.imu_cflags = imu_cflags;
    packet.imu_aflags = imu_aflags;
    packet.mode_code = mode_code;
    packet.mode_code_reason = mode_code_reason;
    mav_array_assign_int16_t(packet.daoYaw, daoYaw, 2);
    mav_array_assign_uint16_t(packet.adc_voltage, adc_voltage, 2);
    mav_array_assign_uint16_t(packet.adc_current, adc_current, 2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SYS_STATUS_ACFLY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SYS_STATUS_ACFLY;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SYS_STATUS_ACFLY_MIN_LEN, MAVLINK_MSG_ID_SYS_STATUS_ACFLY_LEN, MAVLINK_MSG_ID_SYS_STATUS_ACFLY_CRC);
}

/**
 * @brief Encode a sys_status_acfly struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sys_status_acfly C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sys_status_acfly_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sys_status_acfly_t* sys_status_acfly)
{
    return mavlink_msg_sys_status_acfly_pack(system_id, component_id, msg, sys_status_acfly->cpu_usage, sys_status_acfly->remaining_memory, sys_status_acfly->daoYaw, sys_status_acfly->adc_voltage, sys_status_acfly->adc_current, sys_status_acfly->sd_card_capacity, sys_status_acfly->sd_card_remain, sys_status_acfly->sysStatusFlag, sys_status_acfly->sysStatusFlag2, sys_status_acfly->pos_cflags, sys_status_acfly->pos_aflags, sys_status_acfly->imu_cflags, sys_status_acfly->imu_aflags, sys_status_acfly->mode_code, sys_status_acfly->mode_code_reason);
}

/**
 * @brief Encode a sys_status_acfly struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sys_status_acfly C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sys_status_acfly_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sys_status_acfly_t* sys_status_acfly)
{
    return mavlink_msg_sys_status_acfly_pack_chan(system_id, component_id, chan, msg, sys_status_acfly->cpu_usage, sys_status_acfly->remaining_memory, sys_status_acfly->daoYaw, sys_status_acfly->adc_voltage, sys_status_acfly->adc_current, sys_status_acfly->sd_card_capacity, sys_status_acfly->sd_card_remain, sys_status_acfly->sysStatusFlag, sys_status_acfly->sysStatusFlag2, sys_status_acfly->pos_cflags, sys_status_acfly->pos_aflags, sys_status_acfly->imu_cflags, sys_status_acfly->imu_aflags, sys_status_acfly->mode_code, sys_status_acfly->mode_code_reason);
}

/**
 * @brief Encode a sys_status_acfly struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param sys_status_acfly C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sys_status_acfly_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_sys_status_acfly_t* sys_status_acfly)
{
    return mavlink_msg_sys_status_acfly_pack_status(system_id, component_id, _status, msg,  sys_status_acfly->cpu_usage, sys_status_acfly->remaining_memory, sys_status_acfly->daoYaw, sys_status_acfly->adc_voltage, sys_status_acfly->adc_current, sys_status_acfly->sd_card_capacity, sys_status_acfly->sd_card_remain, sys_status_acfly->sysStatusFlag, sys_status_acfly->sysStatusFlag2, sys_status_acfly->pos_cflags, sys_status_acfly->pos_aflags, sys_status_acfly->imu_cflags, sys_status_acfly->imu_aflags, sys_status_acfly->mode_code, sys_status_acfly->mode_code_reason);
}

/**
 * @brief Send a sys_status_acfly message
 * @param chan MAVLink channel to send the message
 *
 * @param cpu_usage [d%] Maximum usage in percent of the mainloop time. Units: 0.1 % (e.g. 123 means 12.3 %). Values: [0-1000] - should always be below 1000.
 * @param remaining_memory  Remaining free OS memory in native CPU words (on a 32-bit MCU one word equals 4 bytes). No standard MAVLink unit for CPU words, so the unit is omitted.
 * @param daoYaw  Heading measured by direction-finding sensor for each array element, in 0.1 deg (e.g. 1234 means 123.4 deg). -1000 = no direction-finding sensor, 1000 = direction-finding sensor present but unavailable, 0-3600 = direction-finding sensor normal.
 * @param adc_voltage  Raw ADC voltage readings in 0.1 V (e.g. 123 means 12.3 V). No standard MAVLink unit for 0.1 V, so the unit is omitted.
 * @param adc_current [cA] Raw ADC current readings. Units: 0.01 A (e.g. 123 means 1.23 A). 0 = no current sensor on this channel.
 * @param sd_card_capacity  Total capacity of the SD card in 10 MB (e.g. 123 means 1230 MB). No standard MAVLink unit for 10 MB, so the unit is omitted. 0 = no SD card inserted or capacity unknown.
 * @param sd_card_remain  Remaining free capacity of the SD card in 10 MB (e.g. 123 means 1230 MB). No standard MAVLink unit for 10 MB, so the unit is omitted. 0 = no SD card inserted or free capacity unknown.
 * @param sysStatusFlag  .
 * @param sysStatusFlag2  .
 * @param pos_cflags  
 * @param pos_aflags  
 * @param imu_cflags  
 * @param imu_aflags  
 * @param mode_code  Current flight state code (MODE_CODE). Values map one-to-one to the DJI Cloud API mode_code property. Extension field (MAVLink 2 only).
 * @param mode_code_reason  Reason for entering the current flight state (MODE_CODE_REASON). Values below 100 map one-to-one to the DJI Cloud API mode_code_reason property, values 100 and above are ACFly vendor-specific extensions. Extension field (MAVLink 2 only).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sys_status_acfly_send(mavlink_channel_t chan, uint16_t cpu_usage, uint32_t remaining_memory, const int16_t *daoYaw, const uint16_t *adc_voltage, const uint16_t *adc_current, uint16_t sd_card_capacity, uint16_t sd_card_remain, uint32_t sysStatusFlag, uint32_t sysStatusFlag2, uint32_t pos_cflags, uint32_t pos_aflags, uint16_t imu_cflags, uint16_t imu_aflags, uint16_t mode_code, uint16_t mode_code_reason)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SYS_STATUS_ACFLY_LEN];
    _mav_put_uint32_t(buf, 0, remaining_memory);
    _mav_put_uint32_t(buf, 4, sysStatusFlag);
    _mav_put_uint32_t(buf, 8, sysStatusFlag2);
    _mav_put_uint32_t(buf, 12, pos_cflags);
    _mav_put_uint32_t(buf, 16, pos_aflags);
    _mav_put_uint16_t(buf, 20, cpu_usage);
    _mav_put_uint16_t(buf, 34, sd_card_capacity);
    _mav_put_uint16_t(buf, 36, sd_card_remain);
    _mav_put_uint16_t(buf, 38, imu_cflags);
    _mav_put_uint16_t(buf, 40, imu_aflags);
    _mav_put_uint16_t(buf, 42, mode_code);
    _mav_put_uint16_t(buf, 44, mode_code_reason);
    _mav_put_int16_t_array(buf, 22, daoYaw, 2);
    _mav_put_uint16_t_array(buf, 26, adc_voltage, 2);
    _mav_put_uint16_t_array(buf, 30, adc_current, 2);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_STATUS_ACFLY, buf, MAVLINK_MSG_ID_SYS_STATUS_ACFLY_MIN_LEN, MAVLINK_MSG_ID_SYS_STATUS_ACFLY_LEN, MAVLINK_MSG_ID_SYS_STATUS_ACFLY_CRC);
#else
    mavlink_sys_status_acfly_t packet;
    packet.remaining_memory = remaining_memory;
    packet.sysStatusFlag = sysStatusFlag;
    packet.sysStatusFlag2 = sysStatusFlag2;
    packet.pos_cflags = pos_cflags;
    packet.pos_aflags = pos_aflags;
    packet.cpu_usage = cpu_usage;
    packet.sd_card_capacity = sd_card_capacity;
    packet.sd_card_remain = sd_card_remain;
    packet.imu_cflags = imu_cflags;
    packet.imu_aflags = imu_aflags;
    packet.mode_code = mode_code;
    packet.mode_code_reason = mode_code_reason;
    mav_array_assign_int16_t(packet.daoYaw, daoYaw, 2);
    mav_array_assign_uint16_t(packet.adc_voltage, adc_voltage, 2);
    mav_array_assign_uint16_t(packet.adc_current, adc_current, 2);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_STATUS_ACFLY, (const char *)&packet, MAVLINK_MSG_ID_SYS_STATUS_ACFLY_MIN_LEN, MAVLINK_MSG_ID_SYS_STATUS_ACFLY_LEN, MAVLINK_MSG_ID_SYS_STATUS_ACFLY_CRC);
#endif
}

/**
 * @brief Send a sys_status_acfly message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sys_status_acfly_send_struct(mavlink_channel_t chan, const mavlink_sys_status_acfly_t* sys_status_acfly)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sys_status_acfly_send(chan, sys_status_acfly->cpu_usage, sys_status_acfly->remaining_memory, sys_status_acfly->daoYaw, sys_status_acfly->adc_voltage, sys_status_acfly->adc_current, sys_status_acfly->sd_card_capacity, sys_status_acfly->sd_card_remain, sys_status_acfly->sysStatusFlag, sys_status_acfly->sysStatusFlag2, sys_status_acfly->pos_cflags, sys_status_acfly->pos_aflags, sys_status_acfly->imu_cflags, sys_status_acfly->imu_aflags, sys_status_acfly->mode_code, sys_status_acfly->mode_code_reason);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_STATUS_ACFLY, (const char *)sys_status_acfly, MAVLINK_MSG_ID_SYS_STATUS_ACFLY_MIN_LEN, MAVLINK_MSG_ID_SYS_STATUS_ACFLY_LEN, MAVLINK_MSG_ID_SYS_STATUS_ACFLY_CRC);
#endif
}

#if MAVLINK_MSG_ID_SYS_STATUS_ACFLY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sys_status_acfly_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t cpu_usage, uint32_t remaining_memory, const int16_t *daoYaw, const uint16_t *adc_voltage, const uint16_t *adc_current, uint16_t sd_card_capacity, uint16_t sd_card_remain, uint32_t sysStatusFlag, uint32_t sysStatusFlag2, uint32_t pos_cflags, uint32_t pos_aflags, uint16_t imu_cflags, uint16_t imu_aflags, uint16_t mode_code, uint16_t mode_code_reason)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, remaining_memory);
    _mav_put_uint32_t(buf, 4, sysStatusFlag);
    _mav_put_uint32_t(buf, 8, sysStatusFlag2);
    _mav_put_uint32_t(buf, 12, pos_cflags);
    _mav_put_uint32_t(buf, 16, pos_aflags);
    _mav_put_uint16_t(buf, 20, cpu_usage);
    _mav_put_uint16_t(buf, 34, sd_card_capacity);
    _mav_put_uint16_t(buf, 36, sd_card_remain);
    _mav_put_uint16_t(buf, 38, imu_cflags);
    _mav_put_uint16_t(buf, 40, imu_aflags);
    _mav_put_uint16_t(buf, 42, mode_code);
    _mav_put_uint16_t(buf, 44, mode_code_reason);
    _mav_put_int16_t_array(buf, 22, daoYaw, 2);
    _mav_put_uint16_t_array(buf, 26, adc_voltage, 2);
    _mav_put_uint16_t_array(buf, 30, adc_current, 2);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_STATUS_ACFLY, buf, MAVLINK_MSG_ID_SYS_STATUS_ACFLY_MIN_LEN, MAVLINK_MSG_ID_SYS_STATUS_ACFLY_LEN, MAVLINK_MSG_ID_SYS_STATUS_ACFLY_CRC);
#else
    mavlink_sys_status_acfly_t *packet = (mavlink_sys_status_acfly_t *)msgbuf;
    packet->remaining_memory = remaining_memory;
    packet->sysStatusFlag = sysStatusFlag;
    packet->sysStatusFlag2 = sysStatusFlag2;
    packet->pos_cflags = pos_cflags;
    packet->pos_aflags = pos_aflags;
    packet->cpu_usage = cpu_usage;
    packet->sd_card_capacity = sd_card_capacity;
    packet->sd_card_remain = sd_card_remain;
    packet->imu_cflags = imu_cflags;
    packet->imu_aflags = imu_aflags;
    packet->mode_code = mode_code;
    packet->mode_code_reason = mode_code_reason;
    mav_array_assign_int16_t(packet->daoYaw, daoYaw, 2);
    mav_array_assign_uint16_t(packet->adc_voltage, adc_voltage, 2);
    mav_array_assign_uint16_t(packet->adc_current, adc_current, 2);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_STATUS_ACFLY, (const char *)packet, MAVLINK_MSG_ID_SYS_STATUS_ACFLY_MIN_LEN, MAVLINK_MSG_ID_SYS_STATUS_ACFLY_LEN, MAVLINK_MSG_ID_SYS_STATUS_ACFLY_CRC);
#endif
}
#endif

#endif

// MESSAGE SYS_STATUS_ACFLY UNPACKING


/**
 * @brief Get field cpu_usage from sys_status_acfly message
 *
 * @return [d%] Maximum usage in percent of the mainloop time. Units: 0.1 % (e.g. 123 means 12.3 %). Values: [0-1000] - should always be below 1000.
 */
static inline uint16_t mavlink_msg_sys_status_acfly_get_cpu_usage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field remaining_memory from sys_status_acfly message
 *
 * @return  Remaining free OS memory in native CPU words (on a 32-bit MCU one word equals 4 bytes). No standard MAVLink unit for CPU words, so the unit is omitted.
 */
static inline uint32_t mavlink_msg_sys_status_acfly_get_remaining_memory(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field daoYaw from sys_status_acfly message
 *
 * @return  Heading measured by direction-finding sensor for each array element, in 0.1 deg (e.g. 1234 means 123.4 deg). -1000 = no direction-finding sensor, 1000 = direction-finding sensor present but unavailable, 0-3600 = direction-finding sensor normal.
 */
static inline uint16_t mavlink_msg_sys_status_acfly_get_daoYaw(const mavlink_message_t* msg, int16_t *daoYaw)
{
    return _MAV_RETURN_int16_t_array(msg, daoYaw, 2,  22);
}

/**
 * @brief Get field adc_voltage from sys_status_acfly message
 *
 * @return  Raw ADC voltage readings in 0.1 V (e.g. 123 means 12.3 V). No standard MAVLink unit for 0.1 V, so the unit is omitted.
 */
static inline uint16_t mavlink_msg_sys_status_acfly_get_adc_voltage(const mavlink_message_t* msg, uint16_t *adc_voltage)
{
    return _MAV_RETURN_uint16_t_array(msg, adc_voltage, 2,  26);
}

/**
 * @brief Get field adc_current from sys_status_acfly message
 *
 * @return [cA] Raw ADC current readings. Units: 0.01 A (e.g. 123 means 1.23 A). 0 = no current sensor on this channel.
 */
static inline uint16_t mavlink_msg_sys_status_acfly_get_adc_current(const mavlink_message_t* msg, uint16_t *adc_current)
{
    return _MAV_RETURN_uint16_t_array(msg, adc_current, 2,  30);
}

/**
 * @brief Get field sd_card_capacity from sys_status_acfly message
 *
 * @return  Total capacity of the SD card in 10 MB (e.g. 123 means 1230 MB). No standard MAVLink unit for 10 MB, so the unit is omitted. 0 = no SD card inserted or capacity unknown.
 */
static inline uint16_t mavlink_msg_sys_status_acfly_get_sd_card_capacity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  34);
}

/**
 * @brief Get field sd_card_remain from sys_status_acfly message
 *
 * @return  Remaining free capacity of the SD card in 10 MB (e.g. 123 means 1230 MB). No standard MAVLink unit for 10 MB, so the unit is omitted. 0 = no SD card inserted or free capacity unknown.
 */
static inline uint16_t mavlink_msg_sys_status_acfly_get_sd_card_remain(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  36);
}

/**
 * @brief Get field sysStatusFlag from sys_status_acfly message
 *
 * @return  .
 */
static inline uint32_t mavlink_msg_sys_status_acfly_get_sysStatusFlag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field sysStatusFlag2 from sys_status_acfly message
 *
 * @return  .
 */
static inline uint32_t mavlink_msg_sys_status_acfly_get_sysStatusFlag2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field pos_cflags from sys_status_acfly message
 *
 * @return  
 */
static inline uint32_t mavlink_msg_sys_status_acfly_get_pos_cflags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field pos_aflags from sys_status_acfly message
 *
 * @return  
 */
static inline uint32_t mavlink_msg_sys_status_acfly_get_pos_aflags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  16);
}

/**
 * @brief Get field imu_cflags from sys_status_acfly message
 *
 * @return  
 */
static inline uint16_t mavlink_msg_sys_status_acfly_get_imu_cflags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  38);
}

/**
 * @brief Get field imu_aflags from sys_status_acfly message
 *
 * @return  
 */
static inline uint16_t mavlink_msg_sys_status_acfly_get_imu_aflags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  40);
}

/**
 * @brief Get field mode_code from sys_status_acfly message
 *
 * @return  Current flight state code (MODE_CODE). Values map one-to-one to the DJI Cloud API mode_code property. Extension field (MAVLink 2 only).
 */
static inline uint16_t mavlink_msg_sys_status_acfly_get_mode_code(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  42);
}

/**
 * @brief Get field mode_code_reason from sys_status_acfly message
 *
 * @return  Reason for entering the current flight state (MODE_CODE_REASON). Values below 100 map one-to-one to the DJI Cloud API mode_code_reason property, values 100 and above are ACFly vendor-specific extensions. Extension field (MAVLink 2 only).
 */
static inline uint16_t mavlink_msg_sys_status_acfly_get_mode_code_reason(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  44);
}

/**
 * @brief Decode a sys_status_acfly message into a struct
 *
 * @param msg The message to decode
 * @param sys_status_acfly C-struct to decode the message contents into
 */
static inline void mavlink_msg_sys_status_acfly_decode(const mavlink_message_t* msg, mavlink_sys_status_acfly_t* sys_status_acfly)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sys_status_acfly->remaining_memory = mavlink_msg_sys_status_acfly_get_remaining_memory(msg);
    sys_status_acfly->sysStatusFlag = mavlink_msg_sys_status_acfly_get_sysStatusFlag(msg);
    sys_status_acfly->sysStatusFlag2 = mavlink_msg_sys_status_acfly_get_sysStatusFlag2(msg);
    sys_status_acfly->pos_cflags = mavlink_msg_sys_status_acfly_get_pos_cflags(msg);
    sys_status_acfly->pos_aflags = mavlink_msg_sys_status_acfly_get_pos_aflags(msg);
    sys_status_acfly->cpu_usage = mavlink_msg_sys_status_acfly_get_cpu_usage(msg);
    mavlink_msg_sys_status_acfly_get_daoYaw(msg, sys_status_acfly->daoYaw);
    mavlink_msg_sys_status_acfly_get_adc_voltage(msg, sys_status_acfly->adc_voltage);
    mavlink_msg_sys_status_acfly_get_adc_current(msg, sys_status_acfly->adc_current);
    sys_status_acfly->sd_card_capacity = mavlink_msg_sys_status_acfly_get_sd_card_capacity(msg);
    sys_status_acfly->sd_card_remain = mavlink_msg_sys_status_acfly_get_sd_card_remain(msg);
    sys_status_acfly->imu_cflags = mavlink_msg_sys_status_acfly_get_imu_cflags(msg);
    sys_status_acfly->imu_aflags = mavlink_msg_sys_status_acfly_get_imu_aflags(msg);
    sys_status_acfly->mode_code = mavlink_msg_sys_status_acfly_get_mode_code(msg);
    sys_status_acfly->mode_code_reason = mavlink_msg_sys_status_acfly_get_mode_code_reason(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SYS_STATUS_ACFLY_LEN? msg->len : MAVLINK_MSG_ID_SYS_STATUS_ACFLY_LEN;
        memset(sys_status_acfly, 0, MAVLINK_MSG_ID_SYS_STATUS_ACFLY_LEN);
    memcpy(sys_status_acfly, _MAV_PAYLOAD(msg), len);
#endif
}
