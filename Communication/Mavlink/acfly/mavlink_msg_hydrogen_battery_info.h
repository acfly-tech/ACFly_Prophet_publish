#pragma once
// MESSAGE HYDROGEN_BATTERY_INFO PACKING

#define MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO 227


typedef struct __mavlink_hydrogen_battery_info_t {
 float humidity; /*<  Humidity of environment.*/
 float temperature; /*<  Temperature of environment.*/
 float fan_speed; /*<  Fan speed.*/
 float hb_tank_pressure; /*<  Hydrogen tank pressure.*/
 float hb_gas_pressure; /*<  Hydrogen gas pressure.*/
 float hb_voltage; /*<  Hydrogen battery voltage.*/
 float hb_current; /*<  Hydrogen battery current.*/
 float hb_Temperature; /*<  Hydrogen battery temperature.*/
 float hb_power; /*<  Hydrogen battery power.*/
 float lP_voltage; /*<  LIPO battery voltage.*/
 float lP_current; /*<  LIPO battery current.*/
 float lP_Temperature; /*<  LIPO battery temperature.*/
 float lP_power; /*<  LIPO battery power.*/
 float toatal_power; /*<  Total power (legacy spelling preserved).*/
} mavlink_hydrogen_battery_info_t;

#define MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_LEN 56
#define MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_MIN_LEN 56
#define MAVLINK_MSG_ID_227_LEN 56
#define MAVLINK_MSG_ID_227_MIN_LEN 56

#define MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_CRC 216
#define MAVLINK_MSG_ID_227_CRC 216



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_HYDROGEN_BATTERY_INFO { \
    227, \
    "HYDROGEN_BATTERY_INFO", \
    14, \
    {  { "humidity", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_hydrogen_battery_info_t, humidity) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_hydrogen_battery_info_t, temperature) }, \
         { "fan_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_hydrogen_battery_info_t, fan_speed) }, \
         { "hb_tank_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_hydrogen_battery_info_t, hb_tank_pressure) }, \
         { "hb_gas_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_hydrogen_battery_info_t, hb_gas_pressure) }, \
         { "hb_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_hydrogen_battery_info_t, hb_voltage) }, \
         { "hb_current", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_hydrogen_battery_info_t, hb_current) }, \
         { "hb_Temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_hydrogen_battery_info_t, hb_Temperature) }, \
         { "hb_power", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_hydrogen_battery_info_t, hb_power) }, \
         { "lP_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_hydrogen_battery_info_t, lP_voltage) }, \
         { "lP_current", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_hydrogen_battery_info_t, lP_current) }, \
         { "lP_Temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_hydrogen_battery_info_t, lP_Temperature) }, \
         { "lP_power", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_hydrogen_battery_info_t, lP_power) }, \
         { "toatal_power", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_hydrogen_battery_info_t, toatal_power) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_HYDROGEN_BATTERY_INFO { \
    "HYDROGEN_BATTERY_INFO", \
    14, \
    {  { "humidity", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_hydrogen_battery_info_t, humidity) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_hydrogen_battery_info_t, temperature) }, \
         { "fan_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_hydrogen_battery_info_t, fan_speed) }, \
         { "hb_tank_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_hydrogen_battery_info_t, hb_tank_pressure) }, \
         { "hb_gas_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_hydrogen_battery_info_t, hb_gas_pressure) }, \
         { "hb_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_hydrogen_battery_info_t, hb_voltage) }, \
         { "hb_current", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_hydrogen_battery_info_t, hb_current) }, \
         { "hb_Temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_hydrogen_battery_info_t, hb_Temperature) }, \
         { "hb_power", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_hydrogen_battery_info_t, hb_power) }, \
         { "lP_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_hydrogen_battery_info_t, lP_voltage) }, \
         { "lP_current", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_hydrogen_battery_info_t, lP_current) }, \
         { "lP_Temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_hydrogen_battery_info_t, lP_Temperature) }, \
         { "lP_power", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_hydrogen_battery_info_t, lP_power) }, \
         { "toatal_power", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_hydrogen_battery_info_t, toatal_power) }, \
         } \
}
#endif

/**
 * @brief Pack a hydrogen_battery_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param humidity  Humidity of environment.
 * @param temperature  Temperature of environment.
 * @param fan_speed  Fan speed.
 * @param hb_tank_pressure  Hydrogen tank pressure.
 * @param hb_gas_pressure  Hydrogen gas pressure.
 * @param hb_voltage  Hydrogen battery voltage.
 * @param hb_current  Hydrogen battery current.
 * @param hb_Temperature  Hydrogen battery temperature.
 * @param hb_power  Hydrogen battery power.
 * @param lP_voltage  LIPO battery voltage.
 * @param lP_current  LIPO battery current.
 * @param lP_Temperature  LIPO battery temperature.
 * @param lP_power  LIPO battery power.
 * @param toatal_power  Total power (legacy spelling preserved).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hydrogen_battery_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float humidity, float temperature, float fan_speed, float hb_tank_pressure, float hb_gas_pressure, float hb_voltage, float hb_current, float hb_Temperature, float hb_power, float lP_voltage, float lP_current, float lP_Temperature, float lP_power, float toatal_power)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_LEN];
    _mav_put_float(buf, 0, humidity);
    _mav_put_float(buf, 4, temperature);
    _mav_put_float(buf, 8, fan_speed);
    _mav_put_float(buf, 12, hb_tank_pressure);
    _mav_put_float(buf, 16, hb_gas_pressure);
    _mav_put_float(buf, 20, hb_voltage);
    _mav_put_float(buf, 24, hb_current);
    _mav_put_float(buf, 28, hb_Temperature);
    _mav_put_float(buf, 32, hb_power);
    _mav_put_float(buf, 36, lP_voltage);
    _mav_put_float(buf, 40, lP_current);
    _mav_put_float(buf, 44, lP_Temperature);
    _mav_put_float(buf, 48, lP_power);
    _mav_put_float(buf, 52, toatal_power);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_LEN);
#else
    mavlink_hydrogen_battery_info_t packet;
    packet.humidity = humidity;
    packet.temperature = temperature;
    packet.fan_speed = fan_speed;
    packet.hb_tank_pressure = hb_tank_pressure;
    packet.hb_gas_pressure = hb_gas_pressure;
    packet.hb_voltage = hb_voltage;
    packet.hb_current = hb_current;
    packet.hb_Temperature = hb_Temperature;
    packet.hb_power = hb_power;
    packet.lP_voltage = lP_voltage;
    packet.lP_current = lP_current;
    packet.lP_Temperature = lP_Temperature;
    packet.lP_power = lP_power;
    packet.toatal_power = toatal_power;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_MIN_LEN, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_LEN, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_CRC);
}

/**
 * @brief Pack a hydrogen_battery_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param humidity  Humidity of environment.
 * @param temperature  Temperature of environment.
 * @param fan_speed  Fan speed.
 * @param hb_tank_pressure  Hydrogen tank pressure.
 * @param hb_gas_pressure  Hydrogen gas pressure.
 * @param hb_voltage  Hydrogen battery voltage.
 * @param hb_current  Hydrogen battery current.
 * @param hb_Temperature  Hydrogen battery temperature.
 * @param hb_power  Hydrogen battery power.
 * @param lP_voltage  LIPO battery voltage.
 * @param lP_current  LIPO battery current.
 * @param lP_Temperature  LIPO battery temperature.
 * @param lP_power  LIPO battery power.
 * @param toatal_power  Total power (legacy spelling preserved).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hydrogen_battery_info_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float humidity, float temperature, float fan_speed, float hb_tank_pressure, float hb_gas_pressure, float hb_voltage, float hb_current, float hb_Temperature, float hb_power, float lP_voltage, float lP_current, float lP_Temperature, float lP_power, float toatal_power)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_LEN];
    _mav_put_float(buf, 0, humidity);
    _mav_put_float(buf, 4, temperature);
    _mav_put_float(buf, 8, fan_speed);
    _mav_put_float(buf, 12, hb_tank_pressure);
    _mav_put_float(buf, 16, hb_gas_pressure);
    _mav_put_float(buf, 20, hb_voltage);
    _mav_put_float(buf, 24, hb_current);
    _mav_put_float(buf, 28, hb_Temperature);
    _mav_put_float(buf, 32, hb_power);
    _mav_put_float(buf, 36, lP_voltage);
    _mav_put_float(buf, 40, lP_current);
    _mav_put_float(buf, 44, lP_Temperature);
    _mav_put_float(buf, 48, lP_power);
    _mav_put_float(buf, 52, toatal_power);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_LEN);
#else
    mavlink_hydrogen_battery_info_t packet;
    packet.humidity = humidity;
    packet.temperature = temperature;
    packet.fan_speed = fan_speed;
    packet.hb_tank_pressure = hb_tank_pressure;
    packet.hb_gas_pressure = hb_gas_pressure;
    packet.hb_voltage = hb_voltage;
    packet.hb_current = hb_current;
    packet.hb_Temperature = hb_Temperature;
    packet.hb_power = hb_power;
    packet.lP_voltage = lP_voltage;
    packet.lP_current = lP_current;
    packet.lP_Temperature = lP_Temperature;
    packet.lP_power = lP_power;
    packet.toatal_power = toatal_power;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_MIN_LEN, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_LEN, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_MIN_LEN, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_LEN);
#endif
}

/**
 * @brief Pack a hydrogen_battery_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param humidity  Humidity of environment.
 * @param temperature  Temperature of environment.
 * @param fan_speed  Fan speed.
 * @param hb_tank_pressure  Hydrogen tank pressure.
 * @param hb_gas_pressure  Hydrogen gas pressure.
 * @param hb_voltage  Hydrogen battery voltage.
 * @param hb_current  Hydrogen battery current.
 * @param hb_Temperature  Hydrogen battery temperature.
 * @param hb_power  Hydrogen battery power.
 * @param lP_voltage  LIPO battery voltage.
 * @param lP_current  LIPO battery current.
 * @param lP_Temperature  LIPO battery temperature.
 * @param lP_power  LIPO battery power.
 * @param toatal_power  Total power (legacy spelling preserved).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hydrogen_battery_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float humidity,float temperature,float fan_speed,float hb_tank_pressure,float hb_gas_pressure,float hb_voltage,float hb_current,float hb_Temperature,float hb_power,float lP_voltage,float lP_current,float lP_Temperature,float lP_power,float toatal_power)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_LEN];
    _mav_put_float(buf, 0, humidity);
    _mav_put_float(buf, 4, temperature);
    _mav_put_float(buf, 8, fan_speed);
    _mav_put_float(buf, 12, hb_tank_pressure);
    _mav_put_float(buf, 16, hb_gas_pressure);
    _mav_put_float(buf, 20, hb_voltage);
    _mav_put_float(buf, 24, hb_current);
    _mav_put_float(buf, 28, hb_Temperature);
    _mav_put_float(buf, 32, hb_power);
    _mav_put_float(buf, 36, lP_voltage);
    _mav_put_float(buf, 40, lP_current);
    _mav_put_float(buf, 44, lP_Temperature);
    _mav_put_float(buf, 48, lP_power);
    _mav_put_float(buf, 52, toatal_power);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_LEN);
#else
    mavlink_hydrogen_battery_info_t packet;
    packet.humidity = humidity;
    packet.temperature = temperature;
    packet.fan_speed = fan_speed;
    packet.hb_tank_pressure = hb_tank_pressure;
    packet.hb_gas_pressure = hb_gas_pressure;
    packet.hb_voltage = hb_voltage;
    packet.hb_current = hb_current;
    packet.hb_Temperature = hb_Temperature;
    packet.hb_power = hb_power;
    packet.lP_voltage = lP_voltage;
    packet.lP_current = lP_current;
    packet.lP_Temperature = lP_Temperature;
    packet.lP_power = lP_power;
    packet.toatal_power = toatal_power;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_MIN_LEN, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_LEN, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_CRC);
}

/**
 * @brief Encode a hydrogen_battery_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param hydrogen_battery_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hydrogen_battery_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_hydrogen_battery_info_t* hydrogen_battery_info)
{
    return mavlink_msg_hydrogen_battery_info_pack(system_id, component_id, msg, hydrogen_battery_info->humidity, hydrogen_battery_info->temperature, hydrogen_battery_info->fan_speed, hydrogen_battery_info->hb_tank_pressure, hydrogen_battery_info->hb_gas_pressure, hydrogen_battery_info->hb_voltage, hydrogen_battery_info->hb_current, hydrogen_battery_info->hb_Temperature, hydrogen_battery_info->hb_power, hydrogen_battery_info->lP_voltage, hydrogen_battery_info->lP_current, hydrogen_battery_info->lP_Temperature, hydrogen_battery_info->lP_power, hydrogen_battery_info->toatal_power);
}

/**
 * @brief Encode a hydrogen_battery_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param hydrogen_battery_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hydrogen_battery_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_hydrogen_battery_info_t* hydrogen_battery_info)
{
    return mavlink_msg_hydrogen_battery_info_pack_chan(system_id, component_id, chan, msg, hydrogen_battery_info->humidity, hydrogen_battery_info->temperature, hydrogen_battery_info->fan_speed, hydrogen_battery_info->hb_tank_pressure, hydrogen_battery_info->hb_gas_pressure, hydrogen_battery_info->hb_voltage, hydrogen_battery_info->hb_current, hydrogen_battery_info->hb_Temperature, hydrogen_battery_info->hb_power, hydrogen_battery_info->lP_voltage, hydrogen_battery_info->lP_current, hydrogen_battery_info->lP_Temperature, hydrogen_battery_info->lP_power, hydrogen_battery_info->toatal_power);
}

/**
 * @brief Encode a hydrogen_battery_info struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param hydrogen_battery_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hydrogen_battery_info_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_hydrogen_battery_info_t* hydrogen_battery_info)
{
    return mavlink_msg_hydrogen_battery_info_pack_status(system_id, component_id, _status, msg,  hydrogen_battery_info->humidity, hydrogen_battery_info->temperature, hydrogen_battery_info->fan_speed, hydrogen_battery_info->hb_tank_pressure, hydrogen_battery_info->hb_gas_pressure, hydrogen_battery_info->hb_voltage, hydrogen_battery_info->hb_current, hydrogen_battery_info->hb_Temperature, hydrogen_battery_info->hb_power, hydrogen_battery_info->lP_voltage, hydrogen_battery_info->lP_current, hydrogen_battery_info->lP_Temperature, hydrogen_battery_info->lP_power, hydrogen_battery_info->toatal_power);
}

/**
 * @brief Send a hydrogen_battery_info message
 * @param chan MAVLink channel to send the message
 *
 * @param humidity  Humidity of environment.
 * @param temperature  Temperature of environment.
 * @param fan_speed  Fan speed.
 * @param hb_tank_pressure  Hydrogen tank pressure.
 * @param hb_gas_pressure  Hydrogen gas pressure.
 * @param hb_voltage  Hydrogen battery voltage.
 * @param hb_current  Hydrogen battery current.
 * @param hb_Temperature  Hydrogen battery temperature.
 * @param hb_power  Hydrogen battery power.
 * @param lP_voltage  LIPO battery voltage.
 * @param lP_current  LIPO battery current.
 * @param lP_Temperature  LIPO battery temperature.
 * @param lP_power  LIPO battery power.
 * @param toatal_power  Total power (legacy spelling preserved).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_hydrogen_battery_info_send(mavlink_channel_t chan, float humidity, float temperature, float fan_speed, float hb_tank_pressure, float hb_gas_pressure, float hb_voltage, float hb_current, float hb_Temperature, float hb_power, float lP_voltage, float lP_current, float lP_Temperature, float lP_power, float toatal_power)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_LEN];
    _mav_put_float(buf, 0, humidity);
    _mav_put_float(buf, 4, temperature);
    _mav_put_float(buf, 8, fan_speed);
    _mav_put_float(buf, 12, hb_tank_pressure);
    _mav_put_float(buf, 16, hb_gas_pressure);
    _mav_put_float(buf, 20, hb_voltage);
    _mav_put_float(buf, 24, hb_current);
    _mav_put_float(buf, 28, hb_Temperature);
    _mav_put_float(buf, 32, hb_power);
    _mav_put_float(buf, 36, lP_voltage);
    _mav_put_float(buf, 40, lP_current);
    _mav_put_float(buf, 44, lP_Temperature);
    _mav_put_float(buf, 48, lP_power);
    _mav_put_float(buf, 52, toatal_power);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO, buf, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_MIN_LEN, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_LEN, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_CRC);
#else
    mavlink_hydrogen_battery_info_t packet;
    packet.humidity = humidity;
    packet.temperature = temperature;
    packet.fan_speed = fan_speed;
    packet.hb_tank_pressure = hb_tank_pressure;
    packet.hb_gas_pressure = hb_gas_pressure;
    packet.hb_voltage = hb_voltage;
    packet.hb_current = hb_current;
    packet.hb_Temperature = hb_Temperature;
    packet.hb_power = hb_power;
    packet.lP_voltage = lP_voltage;
    packet.lP_current = lP_current;
    packet.lP_Temperature = lP_Temperature;
    packet.lP_power = lP_power;
    packet.toatal_power = toatal_power;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO, (const char *)&packet, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_MIN_LEN, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_LEN, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_CRC);
#endif
}

/**
 * @brief Send a hydrogen_battery_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_hydrogen_battery_info_send_struct(mavlink_channel_t chan, const mavlink_hydrogen_battery_info_t* hydrogen_battery_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_hydrogen_battery_info_send(chan, hydrogen_battery_info->humidity, hydrogen_battery_info->temperature, hydrogen_battery_info->fan_speed, hydrogen_battery_info->hb_tank_pressure, hydrogen_battery_info->hb_gas_pressure, hydrogen_battery_info->hb_voltage, hydrogen_battery_info->hb_current, hydrogen_battery_info->hb_Temperature, hydrogen_battery_info->hb_power, hydrogen_battery_info->lP_voltage, hydrogen_battery_info->lP_current, hydrogen_battery_info->lP_Temperature, hydrogen_battery_info->lP_power, hydrogen_battery_info->toatal_power);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO, (const char *)hydrogen_battery_info, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_MIN_LEN, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_LEN, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_hydrogen_battery_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float humidity, float temperature, float fan_speed, float hb_tank_pressure, float hb_gas_pressure, float hb_voltage, float hb_current, float hb_Temperature, float hb_power, float lP_voltage, float lP_current, float lP_Temperature, float lP_power, float toatal_power)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, humidity);
    _mav_put_float(buf, 4, temperature);
    _mav_put_float(buf, 8, fan_speed);
    _mav_put_float(buf, 12, hb_tank_pressure);
    _mav_put_float(buf, 16, hb_gas_pressure);
    _mav_put_float(buf, 20, hb_voltage);
    _mav_put_float(buf, 24, hb_current);
    _mav_put_float(buf, 28, hb_Temperature);
    _mav_put_float(buf, 32, hb_power);
    _mav_put_float(buf, 36, lP_voltage);
    _mav_put_float(buf, 40, lP_current);
    _mav_put_float(buf, 44, lP_Temperature);
    _mav_put_float(buf, 48, lP_power);
    _mav_put_float(buf, 52, toatal_power);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO, buf, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_MIN_LEN, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_LEN, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_CRC);
#else
    mavlink_hydrogen_battery_info_t *packet = (mavlink_hydrogen_battery_info_t *)msgbuf;
    packet->humidity = humidity;
    packet->temperature = temperature;
    packet->fan_speed = fan_speed;
    packet->hb_tank_pressure = hb_tank_pressure;
    packet->hb_gas_pressure = hb_gas_pressure;
    packet->hb_voltage = hb_voltage;
    packet->hb_current = hb_current;
    packet->hb_Temperature = hb_Temperature;
    packet->hb_power = hb_power;
    packet->lP_voltage = lP_voltage;
    packet->lP_current = lP_current;
    packet->lP_Temperature = lP_Temperature;
    packet->lP_power = lP_power;
    packet->toatal_power = toatal_power;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO, (const char *)packet, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_MIN_LEN, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_LEN, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE HYDROGEN_BATTERY_INFO UNPACKING


/**
 * @brief Get field humidity from hydrogen_battery_info message
 *
 * @return  Humidity of environment.
 */
static inline float mavlink_msg_hydrogen_battery_info_get_humidity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field temperature from hydrogen_battery_info message
 *
 * @return  Temperature of environment.
 */
static inline float mavlink_msg_hydrogen_battery_info_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field fan_speed from hydrogen_battery_info message
 *
 * @return  Fan speed.
 */
static inline float mavlink_msg_hydrogen_battery_info_get_fan_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field hb_tank_pressure from hydrogen_battery_info message
 *
 * @return  Hydrogen tank pressure.
 */
static inline float mavlink_msg_hydrogen_battery_info_get_hb_tank_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field hb_gas_pressure from hydrogen_battery_info message
 *
 * @return  Hydrogen gas pressure.
 */
static inline float mavlink_msg_hydrogen_battery_info_get_hb_gas_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field hb_voltage from hydrogen_battery_info message
 *
 * @return  Hydrogen battery voltage.
 */
static inline float mavlink_msg_hydrogen_battery_info_get_hb_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field hb_current from hydrogen_battery_info message
 *
 * @return  Hydrogen battery current.
 */
static inline float mavlink_msg_hydrogen_battery_info_get_hb_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field hb_Temperature from hydrogen_battery_info message
 *
 * @return  Hydrogen battery temperature.
 */
static inline float mavlink_msg_hydrogen_battery_info_get_hb_Temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field hb_power from hydrogen_battery_info message
 *
 * @return  Hydrogen battery power.
 */
static inline float mavlink_msg_hydrogen_battery_info_get_hb_power(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field lP_voltage from hydrogen_battery_info message
 *
 * @return  LIPO battery voltage.
 */
static inline float mavlink_msg_hydrogen_battery_info_get_lP_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field lP_current from hydrogen_battery_info message
 *
 * @return  LIPO battery current.
 */
static inline float mavlink_msg_hydrogen_battery_info_get_lP_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field lP_Temperature from hydrogen_battery_info message
 *
 * @return  LIPO battery temperature.
 */
static inline float mavlink_msg_hydrogen_battery_info_get_lP_Temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field lP_power from hydrogen_battery_info message
 *
 * @return  LIPO battery power.
 */
static inline float mavlink_msg_hydrogen_battery_info_get_lP_power(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field toatal_power from hydrogen_battery_info message
 *
 * @return  Total power (legacy spelling preserved).
 */
static inline float mavlink_msg_hydrogen_battery_info_get_toatal_power(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Decode a hydrogen_battery_info message into a struct
 *
 * @param msg The message to decode
 * @param hydrogen_battery_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_hydrogen_battery_info_decode(const mavlink_message_t* msg, mavlink_hydrogen_battery_info_t* hydrogen_battery_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    hydrogen_battery_info->humidity = mavlink_msg_hydrogen_battery_info_get_humidity(msg);
    hydrogen_battery_info->temperature = mavlink_msg_hydrogen_battery_info_get_temperature(msg);
    hydrogen_battery_info->fan_speed = mavlink_msg_hydrogen_battery_info_get_fan_speed(msg);
    hydrogen_battery_info->hb_tank_pressure = mavlink_msg_hydrogen_battery_info_get_hb_tank_pressure(msg);
    hydrogen_battery_info->hb_gas_pressure = mavlink_msg_hydrogen_battery_info_get_hb_gas_pressure(msg);
    hydrogen_battery_info->hb_voltage = mavlink_msg_hydrogen_battery_info_get_hb_voltage(msg);
    hydrogen_battery_info->hb_current = mavlink_msg_hydrogen_battery_info_get_hb_current(msg);
    hydrogen_battery_info->hb_Temperature = mavlink_msg_hydrogen_battery_info_get_hb_Temperature(msg);
    hydrogen_battery_info->hb_power = mavlink_msg_hydrogen_battery_info_get_hb_power(msg);
    hydrogen_battery_info->lP_voltage = mavlink_msg_hydrogen_battery_info_get_lP_voltage(msg);
    hydrogen_battery_info->lP_current = mavlink_msg_hydrogen_battery_info_get_lP_current(msg);
    hydrogen_battery_info->lP_Temperature = mavlink_msg_hydrogen_battery_info_get_lP_Temperature(msg);
    hydrogen_battery_info->lP_power = mavlink_msg_hydrogen_battery_info_get_lP_power(msg);
    hydrogen_battery_info->toatal_power = mavlink_msg_hydrogen_battery_info_get_toatal_power(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_LEN? msg->len : MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_LEN;
        memset(hydrogen_battery_info, 0, MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_LEN);
    memcpy(hydrogen_battery_info, _MAV_PAYLOAD(msg), len);
#endif
}
