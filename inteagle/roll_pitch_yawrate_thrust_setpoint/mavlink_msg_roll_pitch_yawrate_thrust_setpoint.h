#pragma once
// MESSAGE ROLL_PITCH_YAWRATE_THRUST_SETPOINT PACKING

#define MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT 162

MAVPACKED(
typedef struct __mavlink_roll_pitch_yawrate_thrust_setpoint_t {
 float roll; /*< Desired roll angular speed in rad/s*/
 float pitch; /*< Desired roll angular speed in rad/s.*/
 float yaw_rate; /*< Desired roll angular speed in rad/s.*/
 float thrust; /*< Collective thrust, normalized to 0 .. 1.*/
}) mavlink_roll_pitch_yawrate_thrust_setpoint_t;

#define MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_LEN 16
#define MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_MIN_LEN 16
#define MAVLINK_MSG_ID_162_LEN 16
#define MAVLINK_MSG_ID_162_MIN_LEN 16

#define MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_CRC 77
#define MAVLINK_MSG_ID_162_CRC 77



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ROLL_PITCH_YAWRATE_THRUST_SETPOINT { \
    162, \
    "ROLL_PITCH_YAWRATE_THRUST_SETPOINT", \
    4, \
    {  { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_roll_pitch_yawrate_thrust_setpoint_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_roll_pitch_yawrate_thrust_setpoint_t, pitch) }, \
         { "yaw_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_roll_pitch_yawrate_thrust_setpoint_t, yaw_rate) }, \
         { "thrust", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_roll_pitch_yawrate_thrust_setpoint_t, thrust) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ROLL_PITCH_YAWRATE_THRUST_SETPOINT { \
    "ROLL_PITCH_YAWRATE_THRUST_SETPOINT", \
    4, \
    {  { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_roll_pitch_yawrate_thrust_setpoint_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_roll_pitch_yawrate_thrust_setpoint_t, pitch) }, \
         { "yaw_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_roll_pitch_yawrate_thrust_setpoint_t, yaw_rate) }, \
         { "thrust", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_roll_pitch_yawrate_thrust_setpoint_t, thrust) }, \
         } \
}
#endif

/**
 * @brief Pack a roll_pitch_yawrate_thrust_setpoint message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param roll Desired roll angular speed in rad/s
 * @param pitch Desired roll angular speed in rad/s.
 * @param yaw_rate Desired roll angular speed in rad/s.
 * @param thrust Collective thrust, normalized to 0 .. 1.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_roll_pitch_yawrate_thrust_setpoint_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float roll, float pitch, float yaw_rate, float thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_LEN];
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw_rate);
    _mav_put_float(buf, 12, thrust);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_LEN);
#else
    mavlink_roll_pitch_yawrate_thrust_setpoint_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw_rate = yaw_rate;
    packet.thrust = thrust;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_LEN, MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_CRC);
}

/**
 * @brief Pack a roll_pitch_yawrate_thrust_setpoint message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param roll Desired roll angular speed in rad/s
 * @param pitch Desired roll angular speed in rad/s.
 * @param yaw_rate Desired roll angular speed in rad/s.
 * @param thrust Collective thrust, normalized to 0 .. 1.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_roll_pitch_yawrate_thrust_setpoint_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float roll,float pitch,float yaw_rate,float thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_LEN];
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw_rate);
    _mav_put_float(buf, 12, thrust);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_LEN);
#else
    mavlink_roll_pitch_yawrate_thrust_setpoint_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw_rate = yaw_rate;
    packet.thrust = thrust;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_LEN, MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_CRC);
}

/**
 * @brief Encode a roll_pitch_yawrate_thrust_setpoint struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param roll_pitch_yawrate_thrust_setpoint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_roll_pitch_yawrate_thrust_setpoint_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_roll_pitch_yawrate_thrust_setpoint_t* roll_pitch_yawrate_thrust_setpoint)
{
    return mavlink_msg_roll_pitch_yawrate_thrust_setpoint_pack(system_id, component_id, msg, roll_pitch_yawrate_thrust_setpoint->roll, roll_pitch_yawrate_thrust_setpoint->pitch, roll_pitch_yawrate_thrust_setpoint->yaw_rate, roll_pitch_yawrate_thrust_setpoint->thrust);
}

/**
 * @brief Encode a roll_pitch_yawrate_thrust_setpoint struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param roll_pitch_yawrate_thrust_setpoint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_roll_pitch_yawrate_thrust_setpoint_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_roll_pitch_yawrate_thrust_setpoint_t* roll_pitch_yawrate_thrust_setpoint)
{
    return mavlink_msg_roll_pitch_yawrate_thrust_setpoint_pack_chan(system_id, component_id, chan, msg, roll_pitch_yawrate_thrust_setpoint->roll, roll_pitch_yawrate_thrust_setpoint->pitch, roll_pitch_yawrate_thrust_setpoint->yaw_rate, roll_pitch_yawrate_thrust_setpoint->thrust);
}

/**
 * @brief Send a roll_pitch_yawrate_thrust_setpoint message
 * @param chan MAVLink channel to send the message
 *
 * @param roll Desired roll angular speed in rad/s
 * @param pitch Desired roll angular speed in rad/s.
 * @param yaw_rate Desired roll angular speed in rad/s.
 * @param thrust Collective thrust, normalized to 0 .. 1.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_roll_pitch_yawrate_thrust_setpoint_send(mavlink_channel_t chan, float roll, float pitch, float yaw_rate, float thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_LEN];
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw_rate);
    _mav_put_float(buf, 12, thrust);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT, buf, MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_LEN, MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_CRC);
#else
    mavlink_roll_pitch_yawrate_thrust_setpoint_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw_rate = yaw_rate;
    packet.thrust = thrust;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT, (const char *)&packet, MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_LEN, MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_CRC);
#endif
}

/**
 * @brief Send a roll_pitch_yawrate_thrust_setpoint message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_roll_pitch_yawrate_thrust_setpoint_send_struct(mavlink_channel_t chan, const mavlink_roll_pitch_yawrate_thrust_setpoint_t* roll_pitch_yawrate_thrust_setpoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_roll_pitch_yawrate_thrust_setpoint_send(chan, roll_pitch_yawrate_thrust_setpoint->roll, roll_pitch_yawrate_thrust_setpoint->pitch, roll_pitch_yawrate_thrust_setpoint->yaw_rate, roll_pitch_yawrate_thrust_setpoint->thrust);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT, (const char *)roll_pitch_yawrate_thrust_setpoint, MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_LEN, MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_CRC);
#endif
}

#if MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_roll_pitch_yawrate_thrust_setpoint_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float roll, float pitch, float yaw_rate, float thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw_rate);
    _mav_put_float(buf, 12, thrust);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT, buf, MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_LEN, MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_CRC);
#else
    mavlink_roll_pitch_yawrate_thrust_setpoint_t *packet = (mavlink_roll_pitch_yawrate_thrust_setpoint_t *)msgbuf;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw_rate = yaw_rate;
    packet->thrust = thrust;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT, (const char *)packet, MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_LEN, MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_CRC);
#endif
}
#endif

#endif

// MESSAGE ROLL_PITCH_YAWRATE_THRUST_SETPOINT UNPACKING


/**
 * @brief Get field roll from roll_pitch_yawrate_thrust_setpoint message
 *
 * @return Desired roll angular speed in rad/s
 */
static inline float mavlink_msg_roll_pitch_yawrate_thrust_setpoint_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pitch from roll_pitch_yawrate_thrust_setpoint message
 *
 * @return Desired roll angular speed in rad/s.
 */
static inline float mavlink_msg_roll_pitch_yawrate_thrust_setpoint_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field yaw_rate from roll_pitch_yawrate_thrust_setpoint message
 *
 * @return Desired roll angular speed in rad/s.
 */
static inline float mavlink_msg_roll_pitch_yawrate_thrust_setpoint_get_yaw_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field thrust from roll_pitch_yawrate_thrust_setpoint message
 *
 * @return Collective thrust, normalized to 0 .. 1.
 */
static inline float mavlink_msg_roll_pitch_yawrate_thrust_setpoint_get_thrust(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a roll_pitch_yawrate_thrust_setpoint message into a struct
 *
 * @param msg The message to decode
 * @param roll_pitch_yawrate_thrust_setpoint C-struct to decode the message contents into
 */
static inline void mavlink_msg_roll_pitch_yawrate_thrust_setpoint_decode(const mavlink_message_t* msg, mavlink_roll_pitch_yawrate_thrust_setpoint_t* roll_pitch_yawrate_thrust_setpoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    roll_pitch_yawrate_thrust_setpoint->roll = mavlink_msg_roll_pitch_yawrate_thrust_setpoint_get_roll(msg);
    roll_pitch_yawrate_thrust_setpoint->pitch = mavlink_msg_roll_pitch_yawrate_thrust_setpoint_get_pitch(msg);
    roll_pitch_yawrate_thrust_setpoint->yaw_rate = mavlink_msg_roll_pitch_yawrate_thrust_setpoint_get_yaw_rate(msg);
    roll_pitch_yawrate_thrust_setpoint->thrust = mavlink_msg_roll_pitch_yawrate_thrust_setpoint_get_thrust(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_LEN? msg->len : MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_LEN;
        memset(roll_pitch_yawrate_thrust_setpoint, 0, MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_LEN);
    memcpy(roll_pitch_yawrate_thrust_setpoint, _MAV_PAYLOAD(msg), len);
#endif
}
