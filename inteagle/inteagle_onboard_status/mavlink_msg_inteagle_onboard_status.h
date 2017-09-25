#pragma once
// MESSAGE INTEAGLE_ONBOARD_STATUS PACKING

#define MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS 157

MAVPACKED(
typedef struct __mavlink_inteagle_onboard_status_t {
 uint64_t timestamp; /*<  Time stamp [microseconds].*/
 float battery_voltage; /*< Battery voltage [V].*/
 float temperature_pwr; /*< PCB temperature [C].*/
 uint8_t commander_state; /*< Commander state [].*/
 uint8_t battery_state; /*< Battery state [].*/
 uint8_t control_mode; /*< Control mode [].*/
}) mavlink_inteagle_onboard_status_t;

#define MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_LEN 19
#define MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_MIN_LEN 19
#define MAVLINK_MSG_ID_157_LEN 19
#define MAVLINK_MSG_ID_157_MIN_LEN 19

#define MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_CRC 172
#define MAVLINK_MSG_ID_157_CRC 172



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_INTEAGLE_ONBOARD_STATUS { \
    157, \
    "INTEAGLE_ONBOARD_STATUS", \
    6, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_inteagle_onboard_status_t, timestamp) }, \
         { "commander_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_inteagle_onboard_status_t, commander_state) }, \
         { "battery_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_inteagle_onboard_status_t, battery_state) }, \
         { "control_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_inteagle_onboard_status_t, control_mode) }, \
         { "battery_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_inteagle_onboard_status_t, battery_voltage) }, \
         { "temperature_pwr", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_inteagle_onboard_status_t, temperature_pwr) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_INTEAGLE_ONBOARD_STATUS { \
    "INTEAGLE_ONBOARD_STATUS", \
    6, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_inteagle_onboard_status_t, timestamp) }, \
         { "commander_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_inteagle_onboard_status_t, commander_state) }, \
         { "battery_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_inteagle_onboard_status_t, battery_state) }, \
         { "control_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_inteagle_onboard_status_t, control_mode) }, \
         { "battery_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_inteagle_onboard_status_t, battery_voltage) }, \
         { "temperature_pwr", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_inteagle_onboard_status_t, temperature_pwr) }, \
         } \
}
#endif

/**
 * @brief Pack a inteagle_onboard_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  Time stamp [microseconds].
 * @param commander_state Commander state [].
 * @param battery_state Battery state [].
 * @param control_mode Control mode [].
 * @param battery_voltage Battery voltage [V].
 * @param temperature_pwr PCB temperature [C].
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_inteagle_onboard_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t commander_state, uint8_t battery_state, uint8_t control_mode, float battery_voltage, float temperature_pwr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, battery_voltage);
    _mav_put_float(buf, 12, temperature_pwr);
    _mav_put_uint8_t(buf, 16, commander_state);
    _mav_put_uint8_t(buf, 17, battery_state);
    _mav_put_uint8_t(buf, 18, control_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_LEN);
#else
    mavlink_inteagle_onboard_status_t packet;
    packet.timestamp = timestamp;
    packet.battery_voltage = battery_voltage;
    packet.temperature_pwr = temperature_pwr;
    packet.commander_state = commander_state;
    packet.battery_state = battery_state;
    packet.control_mode = control_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_MIN_LEN, MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_LEN, MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_CRC);
}

/**
 * @brief Pack a inteagle_onboard_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp  Time stamp [microseconds].
 * @param commander_state Commander state [].
 * @param battery_state Battery state [].
 * @param control_mode Control mode [].
 * @param battery_voltage Battery voltage [V].
 * @param temperature_pwr PCB temperature [C].
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_inteagle_onboard_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t commander_state,uint8_t battery_state,uint8_t control_mode,float battery_voltage,float temperature_pwr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, battery_voltage);
    _mav_put_float(buf, 12, temperature_pwr);
    _mav_put_uint8_t(buf, 16, commander_state);
    _mav_put_uint8_t(buf, 17, battery_state);
    _mav_put_uint8_t(buf, 18, control_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_LEN);
#else
    mavlink_inteagle_onboard_status_t packet;
    packet.timestamp = timestamp;
    packet.battery_voltage = battery_voltage;
    packet.temperature_pwr = temperature_pwr;
    packet.commander_state = commander_state;
    packet.battery_state = battery_state;
    packet.control_mode = control_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_MIN_LEN, MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_LEN, MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_CRC);
}

/**
 * @brief Encode a inteagle_onboard_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param inteagle_onboard_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_inteagle_onboard_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_inteagle_onboard_status_t* inteagle_onboard_status)
{
    return mavlink_msg_inteagle_onboard_status_pack(system_id, component_id, msg, inteagle_onboard_status->timestamp, inteagle_onboard_status->commander_state, inteagle_onboard_status->battery_state, inteagle_onboard_status->control_mode, inteagle_onboard_status->battery_voltage, inteagle_onboard_status->temperature_pwr);
}

/**
 * @brief Encode a inteagle_onboard_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param inteagle_onboard_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_inteagle_onboard_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_inteagle_onboard_status_t* inteagle_onboard_status)
{
    return mavlink_msg_inteagle_onboard_status_pack_chan(system_id, component_id, chan, msg, inteagle_onboard_status->timestamp, inteagle_onboard_status->commander_state, inteagle_onboard_status->battery_state, inteagle_onboard_status->control_mode, inteagle_onboard_status->battery_voltage, inteagle_onboard_status->temperature_pwr);
}

/**
 * @brief Send a inteagle_onboard_status message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp  Time stamp [microseconds].
 * @param commander_state Commander state [].
 * @param battery_state Battery state [].
 * @param control_mode Control mode [].
 * @param battery_voltage Battery voltage [V].
 * @param temperature_pwr PCB temperature [C].
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_inteagle_onboard_status_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t commander_state, uint8_t battery_state, uint8_t control_mode, float battery_voltage, float temperature_pwr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, battery_voltage);
    _mav_put_float(buf, 12, temperature_pwr);
    _mav_put_uint8_t(buf, 16, commander_state);
    _mav_put_uint8_t(buf, 17, battery_state);
    _mav_put_uint8_t(buf, 18, control_mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS, buf, MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_MIN_LEN, MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_LEN, MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_CRC);
#else
    mavlink_inteagle_onboard_status_t packet;
    packet.timestamp = timestamp;
    packet.battery_voltage = battery_voltage;
    packet.temperature_pwr = temperature_pwr;
    packet.commander_state = commander_state;
    packet.battery_state = battery_state;
    packet.control_mode = control_mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS, (const char *)&packet, MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_MIN_LEN, MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_LEN, MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_CRC);
#endif
}

/**
 * @brief Send a inteagle_onboard_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_inteagle_onboard_status_send_struct(mavlink_channel_t chan, const mavlink_inteagle_onboard_status_t* inteagle_onboard_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_inteagle_onboard_status_send(chan, inteagle_onboard_status->timestamp, inteagle_onboard_status->commander_state, inteagle_onboard_status->battery_state, inteagle_onboard_status->control_mode, inteagle_onboard_status->battery_voltage, inteagle_onboard_status->temperature_pwr);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS, (const char *)inteagle_onboard_status, MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_MIN_LEN, MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_LEN, MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_inteagle_onboard_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t commander_state, uint8_t battery_state, uint8_t control_mode, float battery_voltage, float temperature_pwr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, battery_voltage);
    _mav_put_float(buf, 12, temperature_pwr);
    _mav_put_uint8_t(buf, 16, commander_state);
    _mav_put_uint8_t(buf, 17, battery_state);
    _mav_put_uint8_t(buf, 18, control_mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS, buf, MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_MIN_LEN, MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_LEN, MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_CRC);
#else
    mavlink_inteagle_onboard_status_t *packet = (mavlink_inteagle_onboard_status_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->battery_voltage = battery_voltage;
    packet->temperature_pwr = temperature_pwr;
    packet->commander_state = commander_state;
    packet->battery_state = battery_state;
    packet->control_mode = control_mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS, (const char *)packet, MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_MIN_LEN, MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_LEN, MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE INTEAGLE_ONBOARD_STATUS UNPACKING


/**
 * @brief Get field timestamp from inteagle_onboard_status message
 *
 * @return  Time stamp [microseconds].
 */
static inline uint64_t mavlink_msg_inteagle_onboard_status_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field commander_state from inteagle_onboard_status message
 *
 * @return Commander state [].
 */
static inline uint8_t mavlink_msg_inteagle_onboard_status_get_commander_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field battery_state from inteagle_onboard_status message
 *
 * @return Battery state [].
 */
static inline uint8_t mavlink_msg_inteagle_onboard_status_get_battery_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field control_mode from inteagle_onboard_status message
 *
 * @return Control mode [].
 */
static inline uint8_t mavlink_msg_inteagle_onboard_status_get_control_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field battery_voltage from inteagle_onboard_status message
 *
 * @return Battery voltage [V].
 */
static inline float mavlink_msg_inteagle_onboard_status_get_battery_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field temperature_pwr from inteagle_onboard_status message
 *
 * @return PCB temperature [C].
 */
static inline float mavlink_msg_inteagle_onboard_status_get_temperature_pwr(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a inteagle_onboard_status message into a struct
 *
 * @param msg The message to decode
 * @param inteagle_onboard_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_inteagle_onboard_status_decode(const mavlink_message_t* msg, mavlink_inteagle_onboard_status_t* inteagle_onboard_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    inteagle_onboard_status->timestamp = mavlink_msg_inteagle_onboard_status_get_timestamp(msg);
    inteagle_onboard_status->battery_voltage = mavlink_msg_inteagle_onboard_status_get_battery_voltage(msg);
    inteagle_onboard_status->temperature_pwr = mavlink_msg_inteagle_onboard_status_get_temperature_pwr(msg);
    inteagle_onboard_status->commander_state = mavlink_msg_inteagle_onboard_status_get_commander_state(msg);
    inteagle_onboard_status->battery_state = mavlink_msg_inteagle_onboard_status_get_battery_state(msg);
    inteagle_onboard_status->control_mode = mavlink_msg_inteagle_onboard_status_get_control_mode(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_LEN? msg->len : MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_LEN;
        memset(inteagle_onboard_status, 0, MAVLINK_MSG_ID_INTEAGLE_ONBOARD_STATUS_LEN);
    memcpy(inteagle_onboard_status, _MAV_PAYLOAD(msg), len);
#endif
}
