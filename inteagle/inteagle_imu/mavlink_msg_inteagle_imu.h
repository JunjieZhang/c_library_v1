#pragma once
// MESSAGE INTEAGLE_IMU PACKING

#define MAVLINK_MSG_ID_INTEAGLE_IMU 150

MAVPACKED(
typedef struct __mavlink_inteagle_imu_t {
 uint64_t timestamp; /*<  Time stamp [microseconds].*/
 float gyro_x; /*< Roll rate [rad/s].*/
 float gyro_y; /*< Pitch rate [rad/s].*/
 float gyro_z; /*< Yaw rate rate [rad/s].*/
 float acc_x; /*< X acceleration [m/s2].*/
 float acc_y; /*< Y acceleration [m/s2].*/
 float acc_z; /*< Z acceleration [m/s2].*/
}) mavlink_inteagle_imu_t;

#define MAVLINK_MSG_ID_INTEAGLE_IMU_LEN 32
#define MAVLINK_MSG_ID_INTEAGLE_IMU_MIN_LEN 32
#define MAVLINK_MSG_ID_150_LEN 32
#define MAVLINK_MSG_ID_150_MIN_LEN 32

#define MAVLINK_MSG_ID_INTEAGLE_IMU_CRC 5
#define MAVLINK_MSG_ID_150_CRC 5



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_INTEAGLE_IMU { \
    150, \
    "INTEAGLE_IMU", \
    7, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_inteagle_imu_t, timestamp) }, \
         { "gyro_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_inteagle_imu_t, gyro_x) }, \
         { "gyro_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_inteagle_imu_t, gyro_y) }, \
         { "gyro_z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_inteagle_imu_t, gyro_z) }, \
         { "acc_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_inteagle_imu_t, acc_x) }, \
         { "acc_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_inteagle_imu_t, acc_y) }, \
         { "acc_z", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_inteagle_imu_t, acc_z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_INTEAGLE_IMU { \
    "INTEAGLE_IMU", \
    7, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_inteagle_imu_t, timestamp) }, \
         { "gyro_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_inteagle_imu_t, gyro_x) }, \
         { "gyro_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_inteagle_imu_t, gyro_y) }, \
         { "gyro_z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_inteagle_imu_t, gyro_z) }, \
         { "acc_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_inteagle_imu_t, acc_x) }, \
         { "acc_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_inteagle_imu_t, acc_y) }, \
         { "acc_z", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_inteagle_imu_t, acc_z) }, \
         } \
}
#endif

/**
 * @brief Pack a inteagle_imu message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  Time stamp [microseconds].
 * @param gyro_x Roll rate [rad/s].
 * @param gyro_y Pitch rate [rad/s].
 * @param gyro_z Yaw rate rate [rad/s].
 * @param acc_x X acceleration [m/s2].
 * @param acc_y Y acceleration [m/s2].
 * @param acc_z Z acceleration [m/s2].
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_inteagle_imu_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float gyro_x, float gyro_y, float gyro_z, float acc_x, float acc_y, float acc_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INTEAGLE_IMU_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, gyro_x);
    _mav_put_float(buf, 12, gyro_y);
    _mav_put_float(buf, 16, gyro_z);
    _mav_put_float(buf, 20, acc_x);
    _mav_put_float(buf, 24, acc_y);
    _mav_put_float(buf, 28, acc_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_INTEAGLE_IMU_LEN);
#else
    mavlink_inteagle_imu_t packet;
    packet.timestamp = timestamp;
    packet.gyro_x = gyro_x;
    packet.gyro_y = gyro_y;
    packet.gyro_z = gyro_z;
    packet.acc_x = acc_x;
    packet.acc_y = acc_y;
    packet.acc_z = acc_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_INTEAGLE_IMU_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_INTEAGLE_IMU;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_INTEAGLE_IMU_MIN_LEN, MAVLINK_MSG_ID_INTEAGLE_IMU_LEN, MAVLINK_MSG_ID_INTEAGLE_IMU_CRC);
}

/**
 * @brief Pack a inteagle_imu message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp  Time stamp [microseconds].
 * @param gyro_x Roll rate [rad/s].
 * @param gyro_y Pitch rate [rad/s].
 * @param gyro_z Yaw rate rate [rad/s].
 * @param acc_x X acceleration [m/s2].
 * @param acc_y Y acceleration [m/s2].
 * @param acc_z Z acceleration [m/s2].
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_inteagle_imu_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float gyro_x,float gyro_y,float gyro_z,float acc_x,float acc_y,float acc_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INTEAGLE_IMU_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, gyro_x);
    _mav_put_float(buf, 12, gyro_y);
    _mav_put_float(buf, 16, gyro_z);
    _mav_put_float(buf, 20, acc_x);
    _mav_put_float(buf, 24, acc_y);
    _mav_put_float(buf, 28, acc_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_INTEAGLE_IMU_LEN);
#else
    mavlink_inteagle_imu_t packet;
    packet.timestamp = timestamp;
    packet.gyro_x = gyro_x;
    packet.gyro_y = gyro_y;
    packet.gyro_z = gyro_z;
    packet.acc_x = acc_x;
    packet.acc_y = acc_y;
    packet.acc_z = acc_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_INTEAGLE_IMU_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_INTEAGLE_IMU;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_INTEAGLE_IMU_MIN_LEN, MAVLINK_MSG_ID_INTEAGLE_IMU_LEN, MAVLINK_MSG_ID_INTEAGLE_IMU_CRC);
}

/**
 * @brief Encode a inteagle_imu struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param inteagle_imu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_inteagle_imu_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_inteagle_imu_t* inteagle_imu)
{
    return mavlink_msg_inteagle_imu_pack(system_id, component_id, msg, inteagle_imu->timestamp, inteagle_imu->gyro_x, inteagle_imu->gyro_y, inteagle_imu->gyro_z, inteagle_imu->acc_x, inteagle_imu->acc_y, inteagle_imu->acc_z);
}

/**
 * @brief Encode a inteagle_imu struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param inteagle_imu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_inteagle_imu_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_inteagle_imu_t* inteagle_imu)
{
    return mavlink_msg_inteagle_imu_pack_chan(system_id, component_id, chan, msg, inteagle_imu->timestamp, inteagle_imu->gyro_x, inteagle_imu->gyro_y, inteagle_imu->gyro_z, inteagle_imu->acc_x, inteagle_imu->acc_y, inteagle_imu->acc_z);
}

/**
 * @brief Send a inteagle_imu message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp  Time stamp [microseconds].
 * @param gyro_x Roll rate [rad/s].
 * @param gyro_y Pitch rate [rad/s].
 * @param gyro_z Yaw rate rate [rad/s].
 * @param acc_x X acceleration [m/s2].
 * @param acc_y Y acceleration [m/s2].
 * @param acc_z Z acceleration [m/s2].
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_inteagle_imu_send(mavlink_channel_t chan, uint64_t timestamp, float gyro_x, float gyro_y, float gyro_z, float acc_x, float acc_y, float acc_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INTEAGLE_IMU_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, gyro_x);
    _mav_put_float(buf, 12, gyro_y);
    _mav_put_float(buf, 16, gyro_z);
    _mav_put_float(buf, 20, acc_x);
    _mav_put_float(buf, 24, acc_y);
    _mav_put_float(buf, 28, acc_z);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INTEAGLE_IMU, buf, MAVLINK_MSG_ID_INTEAGLE_IMU_MIN_LEN, MAVLINK_MSG_ID_INTEAGLE_IMU_LEN, MAVLINK_MSG_ID_INTEAGLE_IMU_CRC);
#else
    mavlink_inteagle_imu_t packet;
    packet.timestamp = timestamp;
    packet.gyro_x = gyro_x;
    packet.gyro_y = gyro_y;
    packet.gyro_z = gyro_z;
    packet.acc_x = acc_x;
    packet.acc_y = acc_y;
    packet.acc_z = acc_z;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INTEAGLE_IMU, (const char *)&packet, MAVLINK_MSG_ID_INTEAGLE_IMU_MIN_LEN, MAVLINK_MSG_ID_INTEAGLE_IMU_LEN, MAVLINK_MSG_ID_INTEAGLE_IMU_CRC);
#endif
}

/**
 * @brief Send a inteagle_imu message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_inteagle_imu_send_struct(mavlink_channel_t chan, const mavlink_inteagle_imu_t* inteagle_imu)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_inteagle_imu_send(chan, inteagle_imu->timestamp, inteagle_imu->gyro_x, inteagle_imu->gyro_y, inteagle_imu->gyro_z, inteagle_imu->acc_x, inteagle_imu->acc_y, inteagle_imu->acc_z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INTEAGLE_IMU, (const char *)inteagle_imu, MAVLINK_MSG_ID_INTEAGLE_IMU_MIN_LEN, MAVLINK_MSG_ID_INTEAGLE_IMU_LEN, MAVLINK_MSG_ID_INTEAGLE_IMU_CRC);
#endif
}

#if MAVLINK_MSG_ID_INTEAGLE_IMU_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_inteagle_imu_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float gyro_x, float gyro_y, float gyro_z, float acc_x, float acc_y, float acc_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, gyro_x);
    _mav_put_float(buf, 12, gyro_y);
    _mav_put_float(buf, 16, gyro_z);
    _mav_put_float(buf, 20, acc_x);
    _mav_put_float(buf, 24, acc_y);
    _mav_put_float(buf, 28, acc_z);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INTEAGLE_IMU, buf, MAVLINK_MSG_ID_INTEAGLE_IMU_MIN_LEN, MAVLINK_MSG_ID_INTEAGLE_IMU_LEN, MAVLINK_MSG_ID_INTEAGLE_IMU_CRC);
#else
    mavlink_inteagle_imu_t *packet = (mavlink_inteagle_imu_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->gyro_x = gyro_x;
    packet->gyro_y = gyro_y;
    packet->gyro_z = gyro_z;
    packet->acc_x = acc_x;
    packet->acc_y = acc_y;
    packet->acc_z = acc_z;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INTEAGLE_IMU, (const char *)packet, MAVLINK_MSG_ID_INTEAGLE_IMU_MIN_LEN, MAVLINK_MSG_ID_INTEAGLE_IMU_LEN, MAVLINK_MSG_ID_INTEAGLE_IMU_CRC);
#endif
}
#endif

#endif

// MESSAGE INTEAGLE_IMU UNPACKING


/**
 * @brief Get field timestamp from inteagle_imu message
 *
 * @return  Time stamp [microseconds].
 */
static inline uint64_t mavlink_msg_inteagle_imu_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field gyro_x from inteagle_imu message
 *
 * @return Roll rate [rad/s].
 */
static inline float mavlink_msg_inteagle_imu_get_gyro_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field gyro_y from inteagle_imu message
 *
 * @return Pitch rate [rad/s].
 */
static inline float mavlink_msg_inteagle_imu_get_gyro_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field gyro_z from inteagle_imu message
 *
 * @return Yaw rate rate [rad/s].
 */
static inline float mavlink_msg_inteagle_imu_get_gyro_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field acc_x from inteagle_imu message
 *
 * @return X acceleration [m/s2].
 */
static inline float mavlink_msg_inteagle_imu_get_acc_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field acc_y from inteagle_imu message
 *
 * @return Y acceleration [m/s2].
 */
static inline float mavlink_msg_inteagle_imu_get_acc_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field acc_z from inteagle_imu message
 *
 * @return Z acceleration [m/s2].
 */
static inline float mavlink_msg_inteagle_imu_get_acc_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a inteagle_imu message into a struct
 *
 * @param msg The message to decode
 * @param inteagle_imu C-struct to decode the message contents into
 */
static inline void mavlink_msg_inteagle_imu_decode(const mavlink_message_t* msg, mavlink_inteagle_imu_t* inteagle_imu)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    inteagle_imu->timestamp = mavlink_msg_inteagle_imu_get_timestamp(msg);
    inteagle_imu->gyro_x = mavlink_msg_inteagle_imu_get_gyro_x(msg);
    inteagle_imu->gyro_y = mavlink_msg_inteagle_imu_get_gyro_y(msg);
    inteagle_imu->gyro_z = mavlink_msg_inteagle_imu_get_gyro_z(msg);
    inteagle_imu->acc_x = mavlink_msg_inteagle_imu_get_acc_x(msg);
    inteagle_imu->acc_y = mavlink_msg_inteagle_imu_get_acc_y(msg);
    inteagle_imu->acc_z = mavlink_msg_inteagle_imu_get_acc_z(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_INTEAGLE_IMU_LEN? msg->len : MAVLINK_MSG_ID_INTEAGLE_IMU_LEN;
        memset(inteagle_imu, 0, MAVLINK_MSG_ID_INTEAGLE_IMU_LEN);
    memcpy(inteagle_imu, _MAV_PAYLOAD(msg), len);
#endif
}
