/** @file
 *    @brief MAVLink comm protocol testsuite generated from roll_pitch_yawrate_thrust_setpoint.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef ROLL_PITCH_YAWRATE_THRUST_SETPOINT_TESTSUITE_H
#define ROLL_PITCH_YAWRATE_THRUST_SETPOINT_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_roll_pitch_yawrate_thrust_setpoint(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_roll_pitch_yawrate_thrust_setpoint(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_roll_pitch_yawrate_thrust_setpoint(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_roll_pitch_yawrate_thrust_setpoint_t packet_in = {
        17.0,45.0,73.0,101.0
    };
    mavlink_roll_pitch_yawrate_thrust_setpoint_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.roll = packet_in.roll;
        packet1.pitch = packet_in.pitch;
        packet1.yaw_rate = packet_in.yaw_rate;
        packet1.thrust = packet_in.thrust;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_roll_pitch_yawrate_thrust_setpoint_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_roll_pitch_yawrate_thrust_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_roll_pitch_yawrate_thrust_setpoint_pack(system_id, component_id, &msg , packet1.roll , packet1.pitch , packet1.yaw_rate , packet1.thrust );
    mavlink_msg_roll_pitch_yawrate_thrust_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_roll_pitch_yawrate_thrust_setpoint_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.roll , packet1.pitch , packet1.yaw_rate , packet1.thrust );
    mavlink_msg_roll_pitch_yawrate_thrust_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_roll_pitch_yawrate_thrust_setpoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_roll_pitch_yawrate_thrust_setpoint_send(MAVLINK_COMM_1 , packet1.roll , packet1.pitch , packet1.yaw_rate , packet1.thrust );
    mavlink_msg_roll_pitch_yawrate_thrust_setpoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_roll_pitch_yawrate_thrust_setpoint(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_roll_pitch_yawrate_thrust_setpoint(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // ROLL_PITCH_YAWRATE_THRUST_SETPOINT_TESTSUITE_H
