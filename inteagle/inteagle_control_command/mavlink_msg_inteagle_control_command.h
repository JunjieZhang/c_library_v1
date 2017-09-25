#pragma once
// MESSAGE INTEAGLE_CONTROL_COMMAND PACKING

#define MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND 158

MAVPACKED(
typedef struct __mavlink_inteagle_control_command_t {
 uint64_t timestamp; /*<  Time stamp [microseconds].*/
 uint64_t execution_time; /*<  Execution time stamp [microseconds].*/
 float thrust_dir_x; /*< Thrust direction unit vector [-].*/
 float thrust_dir_y; /*< Thrust direction unit vector [-].*/
 float thrust_dir_z; /*< Thrust direction unit vector [-].*/
 float bodyrates_x; /*< Body rate [rad/s].*/
 float bodyrates_y; /*< Body rate [rad/s].*/
 float bodyrates_z; /*< Body rate [rad/s].*/
 float angular_acc_x; /*< Body angular acceleration [rad/s^2].*/
 float angular_acc_y; /*< Body angular acceleration [rad/s^2].*/
 float angular_acc_z; /*< Body angular acceleration [rad/s^2].*/
 float thrust; /*< Mass normalized thrust [m/s^2].*/
 uint8_t control_mode; /*< Control mode {NONE=0, ANGLE=1, ANGLERATE=2}.*/
 uint8_t off; /*< Control is OFF [on=0, off=1].*/
}) mavlink_inteagle_control_command_t;

#define MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_LEN 58
#define MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_MIN_LEN 58
#define MAVLINK_MSG_ID_158_LEN 58
#define MAVLINK_MSG_ID_158_MIN_LEN 58

#define MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_CRC 52
#define MAVLINK_MSG_ID_158_CRC 52



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_INTEAGLE_CONTROL_COMMAND { \
    158, \
    "INTEAGLE_CONTROL_COMMAND", \
    14, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_inteagle_control_command_t, timestamp) }, \
         { "execution_time", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_inteagle_control_command_t, execution_time) }, \
         { "control_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 56, offsetof(mavlink_inteagle_control_command_t, control_mode) }, \
         { "off", NULL, MAVLINK_TYPE_UINT8_T, 0, 57, offsetof(mavlink_inteagle_control_command_t, off) }, \
         { "thrust_dir_x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_inteagle_control_command_t, thrust_dir_x) }, \
         { "thrust_dir_y", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_inteagle_control_command_t, thrust_dir_y) }, \
         { "thrust_dir_z", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_inteagle_control_command_t, thrust_dir_z) }, \
         { "bodyrates_x", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_inteagle_control_command_t, bodyrates_x) }, \
         { "bodyrates_y", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_inteagle_control_command_t, bodyrates_y) }, \
         { "bodyrates_z", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_inteagle_control_command_t, bodyrates_z) }, \
         { "angular_acc_x", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_inteagle_control_command_t, angular_acc_x) }, \
         { "angular_acc_y", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_inteagle_control_command_t, angular_acc_y) }, \
         { "angular_acc_z", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_inteagle_control_command_t, angular_acc_z) }, \
         { "thrust", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_inteagle_control_command_t, thrust) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_INTEAGLE_CONTROL_COMMAND { \
    "INTEAGLE_CONTROL_COMMAND", \
    14, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_inteagle_control_command_t, timestamp) }, \
         { "execution_time", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_inteagle_control_command_t, execution_time) }, \
         { "control_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 56, offsetof(mavlink_inteagle_control_command_t, control_mode) }, \
         { "off", NULL, MAVLINK_TYPE_UINT8_T, 0, 57, offsetof(mavlink_inteagle_control_command_t, off) }, \
         { "thrust_dir_x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_inteagle_control_command_t, thrust_dir_x) }, \
         { "thrust_dir_y", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_inteagle_control_command_t, thrust_dir_y) }, \
         { "thrust_dir_z", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_inteagle_control_command_t, thrust_dir_z) }, \
         { "bodyrates_x", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_inteagle_control_command_t, bodyrates_x) }, \
         { "bodyrates_y", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_inteagle_control_command_t, bodyrates_y) }, \
         { "bodyrates_z", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_inteagle_control_command_t, bodyrates_z) }, \
         { "angular_acc_x", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_inteagle_control_command_t, angular_acc_x) }, \
         { "angular_acc_y", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_inteagle_control_command_t, angular_acc_y) }, \
         { "angular_acc_z", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_inteagle_control_command_t, angular_acc_z) }, \
         { "thrust", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_inteagle_control_command_t, thrust) }, \
         } \
}
#endif

/**
 * @brief Pack a inteagle_control_command message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  Time stamp [microseconds].
 * @param execution_time  Execution time stamp [microseconds].
 * @param control_mode Control mode {NONE=0, ANGLE=1, ANGLERATE=2}.
 * @param off Control is OFF [on=0, off=1].
 * @param thrust_dir_x Thrust direction unit vector [-].
 * @param thrust_dir_y Thrust direction unit vector [-].
 * @param thrust_dir_z Thrust direction unit vector [-].
 * @param bodyrates_x Body rate [rad/s].
 * @param bodyrates_y Body rate [rad/s].
 * @param bodyrates_z Body rate [rad/s].
 * @param angular_acc_x Body angular acceleration [rad/s^2].
 * @param angular_acc_y Body angular acceleration [rad/s^2].
 * @param angular_acc_z Body angular acceleration [rad/s^2].
 * @param thrust Mass normalized thrust [m/s^2].
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_inteagle_control_command_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint64_t execution_time, uint8_t control_mode, uint8_t off, float thrust_dir_x, float thrust_dir_y, float thrust_dir_z, float bodyrates_x, float bodyrates_y, float bodyrates_z, float angular_acc_x, float angular_acc_y, float angular_acc_z, float thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, execution_time);
    _mav_put_float(buf, 16, thrust_dir_x);
    _mav_put_float(buf, 20, thrust_dir_y);
    _mav_put_float(buf, 24, thrust_dir_z);
    _mav_put_float(buf, 28, bodyrates_x);
    _mav_put_float(buf, 32, bodyrates_y);
    _mav_put_float(buf, 36, bodyrates_z);
    _mav_put_float(buf, 40, angular_acc_x);
    _mav_put_float(buf, 44, angular_acc_y);
    _mav_put_float(buf, 48, angular_acc_z);
    _mav_put_float(buf, 52, thrust);
    _mav_put_uint8_t(buf, 56, control_mode);
    _mav_put_uint8_t(buf, 57, off);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_LEN);
#else
    mavlink_inteagle_control_command_t packet;
    packet.timestamp = timestamp;
    packet.execution_time = execution_time;
    packet.thrust_dir_x = thrust_dir_x;
    packet.thrust_dir_y = thrust_dir_y;
    packet.thrust_dir_z = thrust_dir_z;
    packet.bodyrates_x = bodyrates_x;
    packet.bodyrates_y = bodyrates_y;
    packet.bodyrates_z = bodyrates_z;
    packet.angular_acc_x = angular_acc_x;
    packet.angular_acc_y = angular_acc_y;
    packet.angular_acc_z = angular_acc_z;
    packet.thrust = thrust;
    packet.control_mode = control_mode;
    packet.off = off;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_MIN_LEN, MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_LEN, MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_CRC);
}

/**
 * @brief Pack a inteagle_control_command message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp  Time stamp [microseconds].
 * @param execution_time  Execution time stamp [microseconds].
 * @param control_mode Control mode {NONE=0, ANGLE=1, ANGLERATE=2}.
 * @param off Control is OFF [on=0, off=1].
 * @param thrust_dir_x Thrust direction unit vector [-].
 * @param thrust_dir_y Thrust direction unit vector [-].
 * @param thrust_dir_z Thrust direction unit vector [-].
 * @param bodyrates_x Body rate [rad/s].
 * @param bodyrates_y Body rate [rad/s].
 * @param bodyrates_z Body rate [rad/s].
 * @param angular_acc_x Body angular acceleration [rad/s^2].
 * @param angular_acc_y Body angular acceleration [rad/s^2].
 * @param angular_acc_z Body angular acceleration [rad/s^2].
 * @param thrust Mass normalized thrust [m/s^2].
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_inteagle_control_command_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint64_t execution_time,uint8_t control_mode,uint8_t off,float thrust_dir_x,float thrust_dir_y,float thrust_dir_z,float bodyrates_x,float bodyrates_y,float bodyrates_z,float angular_acc_x,float angular_acc_y,float angular_acc_z,float thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, execution_time);
    _mav_put_float(buf, 16, thrust_dir_x);
    _mav_put_float(buf, 20, thrust_dir_y);
    _mav_put_float(buf, 24, thrust_dir_z);
    _mav_put_float(buf, 28, bodyrates_x);
    _mav_put_float(buf, 32, bodyrates_y);
    _mav_put_float(buf, 36, bodyrates_z);
    _mav_put_float(buf, 40, angular_acc_x);
    _mav_put_float(buf, 44, angular_acc_y);
    _mav_put_float(buf, 48, angular_acc_z);
    _mav_put_float(buf, 52, thrust);
    _mav_put_uint8_t(buf, 56, control_mode);
    _mav_put_uint8_t(buf, 57, off);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_LEN);
#else
    mavlink_inteagle_control_command_t packet;
    packet.timestamp = timestamp;
    packet.execution_time = execution_time;
    packet.thrust_dir_x = thrust_dir_x;
    packet.thrust_dir_y = thrust_dir_y;
    packet.thrust_dir_z = thrust_dir_z;
    packet.bodyrates_x = bodyrates_x;
    packet.bodyrates_y = bodyrates_y;
    packet.bodyrates_z = bodyrates_z;
    packet.angular_acc_x = angular_acc_x;
    packet.angular_acc_y = angular_acc_y;
    packet.angular_acc_z = angular_acc_z;
    packet.thrust = thrust;
    packet.control_mode = control_mode;
    packet.off = off;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_MIN_LEN, MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_LEN, MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_CRC);
}

/**
 * @brief Encode a inteagle_control_command struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param inteagle_control_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_inteagle_control_command_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_inteagle_control_command_t* inteagle_control_command)
{
    return mavlink_msg_inteagle_control_command_pack(system_id, component_id, msg, inteagle_control_command->timestamp, inteagle_control_command->execution_time, inteagle_control_command->control_mode, inteagle_control_command->off, inteagle_control_command->thrust_dir_x, inteagle_control_command->thrust_dir_y, inteagle_control_command->thrust_dir_z, inteagle_control_command->bodyrates_x, inteagle_control_command->bodyrates_y, inteagle_control_command->bodyrates_z, inteagle_control_command->angular_acc_x, inteagle_control_command->angular_acc_y, inteagle_control_command->angular_acc_z, inteagle_control_command->thrust);
}

/**
 * @brief Encode a inteagle_control_command struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param inteagle_control_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_inteagle_control_command_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_inteagle_control_command_t* inteagle_control_command)
{
    return mavlink_msg_inteagle_control_command_pack_chan(system_id, component_id, chan, msg, inteagle_control_command->timestamp, inteagle_control_command->execution_time, inteagle_control_command->control_mode, inteagle_control_command->off, inteagle_control_command->thrust_dir_x, inteagle_control_command->thrust_dir_y, inteagle_control_command->thrust_dir_z, inteagle_control_command->bodyrates_x, inteagle_control_command->bodyrates_y, inteagle_control_command->bodyrates_z, inteagle_control_command->angular_acc_x, inteagle_control_command->angular_acc_y, inteagle_control_command->angular_acc_z, inteagle_control_command->thrust);
}

/**
 * @brief Send a inteagle_control_command message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp  Time stamp [microseconds].
 * @param execution_time  Execution time stamp [microseconds].
 * @param control_mode Control mode {NONE=0, ANGLE=1, ANGLERATE=2}.
 * @param off Control is OFF [on=0, off=1].
 * @param thrust_dir_x Thrust direction unit vector [-].
 * @param thrust_dir_y Thrust direction unit vector [-].
 * @param thrust_dir_z Thrust direction unit vector [-].
 * @param bodyrates_x Body rate [rad/s].
 * @param bodyrates_y Body rate [rad/s].
 * @param bodyrates_z Body rate [rad/s].
 * @param angular_acc_x Body angular acceleration [rad/s^2].
 * @param angular_acc_y Body angular acceleration [rad/s^2].
 * @param angular_acc_z Body angular acceleration [rad/s^2].
 * @param thrust Mass normalized thrust [m/s^2].
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_inteagle_control_command_send(mavlink_channel_t chan, uint64_t timestamp, uint64_t execution_time, uint8_t control_mode, uint8_t off, float thrust_dir_x, float thrust_dir_y, float thrust_dir_z, float bodyrates_x, float bodyrates_y, float bodyrates_z, float angular_acc_x, float angular_acc_y, float angular_acc_z, float thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, execution_time);
    _mav_put_float(buf, 16, thrust_dir_x);
    _mav_put_float(buf, 20, thrust_dir_y);
    _mav_put_float(buf, 24, thrust_dir_z);
    _mav_put_float(buf, 28, bodyrates_x);
    _mav_put_float(buf, 32, bodyrates_y);
    _mav_put_float(buf, 36, bodyrates_z);
    _mav_put_float(buf, 40, angular_acc_x);
    _mav_put_float(buf, 44, angular_acc_y);
    _mav_put_float(buf, 48, angular_acc_z);
    _mav_put_float(buf, 52, thrust);
    _mav_put_uint8_t(buf, 56, control_mode);
    _mav_put_uint8_t(buf, 57, off);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND, buf, MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_MIN_LEN, MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_LEN, MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_CRC);
#else
    mavlink_inteagle_control_command_t packet;
    packet.timestamp = timestamp;
    packet.execution_time = execution_time;
    packet.thrust_dir_x = thrust_dir_x;
    packet.thrust_dir_y = thrust_dir_y;
    packet.thrust_dir_z = thrust_dir_z;
    packet.bodyrates_x = bodyrates_x;
    packet.bodyrates_y = bodyrates_y;
    packet.bodyrates_z = bodyrates_z;
    packet.angular_acc_x = angular_acc_x;
    packet.angular_acc_y = angular_acc_y;
    packet.angular_acc_z = angular_acc_z;
    packet.thrust = thrust;
    packet.control_mode = control_mode;
    packet.off = off;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND, (const char *)&packet, MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_MIN_LEN, MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_LEN, MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_CRC);
#endif
}

/**
 * @brief Send a inteagle_control_command message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_inteagle_control_command_send_struct(mavlink_channel_t chan, const mavlink_inteagle_control_command_t* inteagle_control_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_inteagle_control_command_send(chan, inteagle_control_command->timestamp, inteagle_control_command->execution_time, inteagle_control_command->control_mode, inteagle_control_command->off, inteagle_control_command->thrust_dir_x, inteagle_control_command->thrust_dir_y, inteagle_control_command->thrust_dir_z, inteagle_control_command->bodyrates_x, inteagle_control_command->bodyrates_y, inteagle_control_command->bodyrates_z, inteagle_control_command->angular_acc_x, inteagle_control_command->angular_acc_y, inteagle_control_command->angular_acc_z, inteagle_control_command->thrust);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND, (const char *)inteagle_control_command, MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_MIN_LEN, MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_LEN, MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_CRC);
#endif
}

#if MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_inteagle_control_command_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint64_t execution_time, uint8_t control_mode, uint8_t off, float thrust_dir_x, float thrust_dir_y, float thrust_dir_z, float bodyrates_x, float bodyrates_y, float bodyrates_z, float angular_acc_x, float angular_acc_y, float angular_acc_z, float thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, execution_time);
    _mav_put_float(buf, 16, thrust_dir_x);
    _mav_put_float(buf, 20, thrust_dir_y);
    _mav_put_float(buf, 24, thrust_dir_z);
    _mav_put_float(buf, 28, bodyrates_x);
    _mav_put_float(buf, 32, bodyrates_y);
    _mav_put_float(buf, 36, bodyrates_z);
    _mav_put_float(buf, 40, angular_acc_x);
    _mav_put_float(buf, 44, angular_acc_y);
    _mav_put_float(buf, 48, angular_acc_z);
    _mav_put_float(buf, 52, thrust);
    _mav_put_uint8_t(buf, 56, control_mode);
    _mav_put_uint8_t(buf, 57, off);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND, buf, MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_MIN_LEN, MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_LEN, MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_CRC);
#else
    mavlink_inteagle_control_command_t *packet = (mavlink_inteagle_control_command_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->execution_time = execution_time;
    packet->thrust_dir_x = thrust_dir_x;
    packet->thrust_dir_y = thrust_dir_y;
    packet->thrust_dir_z = thrust_dir_z;
    packet->bodyrates_x = bodyrates_x;
    packet->bodyrates_y = bodyrates_y;
    packet->bodyrates_z = bodyrates_z;
    packet->angular_acc_x = angular_acc_x;
    packet->angular_acc_y = angular_acc_y;
    packet->angular_acc_z = angular_acc_z;
    packet->thrust = thrust;
    packet->control_mode = control_mode;
    packet->off = off;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND, (const char *)packet, MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_MIN_LEN, MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_LEN, MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_CRC);
#endif
}
#endif

#endif

// MESSAGE INTEAGLE_CONTROL_COMMAND UNPACKING


/**
 * @brief Get field timestamp from inteagle_control_command message
 *
 * @return  Time stamp [microseconds].
 */
static inline uint64_t mavlink_msg_inteagle_control_command_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field execution_time from inteagle_control_command message
 *
 * @return  Execution time stamp [microseconds].
 */
static inline uint64_t mavlink_msg_inteagle_control_command_get_execution_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field control_mode from inteagle_control_command message
 *
 * @return Control mode {NONE=0, ANGLE=1, ANGLERATE=2}.
 */
static inline uint8_t mavlink_msg_inteagle_control_command_get_control_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  56);
}

/**
 * @brief Get field off from inteagle_control_command message
 *
 * @return Control is OFF [on=0, off=1].
 */
static inline uint8_t mavlink_msg_inteagle_control_command_get_off(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  57);
}

/**
 * @brief Get field thrust_dir_x from inteagle_control_command message
 *
 * @return Thrust direction unit vector [-].
 */
static inline float mavlink_msg_inteagle_control_command_get_thrust_dir_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field thrust_dir_y from inteagle_control_command message
 *
 * @return Thrust direction unit vector [-].
 */
static inline float mavlink_msg_inteagle_control_command_get_thrust_dir_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field thrust_dir_z from inteagle_control_command message
 *
 * @return Thrust direction unit vector [-].
 */
static inline float mavlink_msg_inteagle_control_command_get_thrust_dir_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field bodyrates_x from inteagle_control_command message
 *
 * @return Body rate [rad/s].
 */
static inline float mavlink_msg_inteagle_control_command_get_bodyrates_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field bodyrates_y from inteagle_control_command message
 *
 * @return Body rate [rad/s].
 */
static inline float mavlink_msg_inteagle_control_command_get_bodyrates_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field bodyrates_z from inteagle_control_command message
 *
 * @return Body rate [rad/s].
 */
static inline float mavlink_msg_inteagle_control_command_get_bodyrates_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field angular_acc_x from inteagle_control_command message
 *
 * @return Body angular acceleration [rad/s^2].
 */
static inline float mavlink_msg_inteagle_control_command_get_angular_acc_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field angular_acc_y from inteagle_control_command message
 *
 * @return Body angular acceleration [rad/s^2].
 */
static inline float mavlink_msg_inteagle_control_command_get_angular_acc_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field angular_acc_z from inteagle_control_command message
 *
 * @return Body angular acceleration [rad/s^2].
 */
static inline float mavlink_msg_inteagle_control_command_get_angular_acc_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field thrust from inteagle_control_command message
 *
 * @return Mass normalized thrust [m/s^2].
 */
static inline float mavlink_msg_inteagle_control_command_get_thrust(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Decode a inteagle_control_command message into a struct
 *
 * @param msg The message to decode
 * @param inteagle_control_command C-struct to decode the message contents into
 */
static inline void mavlink_msg_inteagle_control_command_decode(const mavlink_message_t* msg, mavlink_inteagle_control_command_t* inteagle_control_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    inteagle_control_command->timestamp = mavlink_msg_inteagle_control_command_get_timestamp(msg);
    inteagle_control_command->execution_time = mavlink_msg_inteagle_control_command_get_execution_time(msg);
    inteagle_control_command->thrust_dir_x = mavlink_msg_inteagle_control_command_get_thrust_dir_x(msg);
    inteagle_control_command->thrust_dir_y = mavlink_msg_inteagle_control_command_get_thrust_dir_y(msg);
    inteagle_control_command->thrust_dir_z = mavlink_msg_inteagle_control_command_get_thrust_dir_z(msg);
    inteagle_control_command->bodyrates_x = mavlink_msg_inteagle_control_command_get_bodyrates_x(msg);
    inteagle_control_command->bodyrates_y = mavlink_msg_inteagle_control_command_get_bodyrates_y(msg);
    inteagle_control_command->bodyrates_z = mavlink_msg_inteagle_control_command_get_bodyrates_z(msg);
    inteagle_control_command->angular_acc_x = mavlink_msg_inteagle_control_command_get_angular_acc_x(msg);
    inteagle_control_command->angular_acc_y = mavlink_msg_inteagle_control_command_get_angular_acc_y(msg);
    inteagle_control_command->angular_acc_z = mavlink_msg_inteagle_control_command_get_angular_acc_z(msg);
    inteagle_control_command->thrust = mavlink_msg_inteagle_control_command_get_thrust(msg);
    inteagle_control_command->control_mode = mavlink_msg_inteagle_control_command_get_control_mode(msg);
    inteagle_control_command->off = mavlink_msg_inteagle_control_command_get_off(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_LEN? msg->len : MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_LEN;
        memset(inteagle_control_command, 0, MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_LEN);
    memcpy(inteagle_control_command, _MAV_PAYLOAD(msg), len);
#endif
}
