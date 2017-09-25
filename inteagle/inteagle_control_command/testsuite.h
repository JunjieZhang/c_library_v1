/** @file
 *    @brief MAVLink comm protocol testsuite generated from inteagle_control_command.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef INTEAGLE_CONTROL_COMMAND_TESTSUITE_H
#define INTEAGLE_CONTROL_COMMAND_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_inteagle_control_command(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_inteagle_control_command(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_inteagle_control_command(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_inteagle_control_command_t packet_in = {
        93372036854775807ULL,93372036854776311ULL,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0,173,240
    };
    mavlink_inteagle_control_command_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.execution_time = packet_in.execution_time;
        packet1.thrust_dir_x = packet_in.thrust_dir_x;
        packet1.thrust_dir_y = packet_in.thrust_dir_y;
        packet1.thrust_dir_z = packet_in.thrust_dir_z;
        packet1.bodyrates_x = packet_in.bodyrates_x;
        packet1.bodyrates_y = packet_in.bodyrates_y;
        packet1.bodyrates_z = packet_in.bodyrates_z;
        packet1.angular_acc_x = packet_in.angular_acc_x;
        packet1.angular_acc_y = packet_in.angular_acc_y;
        packet1.angular_acc_z = packet_in.angular_acc_z;
        packet1.thrust = packet_in.thrust;
        packet1.control_mode = packet_in.control_mode;
        packet1.off = packet_in.off;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_INTEAGLE_CONTROL_COMMAND_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_inteagle_control_command_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_inteagle_control_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_inteagle_control_command_pack(system_id, component_id, &msg , packet1.timestamp , packet1.execution_time , packet1.control_mode , packet1.off , packet1.thrust_dir_x , packet1.thrust_dir_y , packet1.thrust_dir_z , packet1.bodyrates_x , packet1.bodyrates_y , packet1.bodyrates_z , packet1.angular_acc_x , packet1.angular_acc_y , packet1.angular_acc_z , packet1.thrust );
    mavlink_msg_inteagle_control_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_inteagle_control_command_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.execution_time , packet1.control_mode , packet1.off , packet1.thrust_dir_x , packet1.thrust_dir_y , packet1.thrust_dir_z , packet1.bodyrates_x , packet1.bodyrates_y , packet1.bodyrates_z , packet1.angular_acc_x , packet1.angular_acc_y , packet1.angular_acc_z , packet1.thrust );
    mavlink_msg_inteagle_control_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_inteagle_control_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_inteagle_control_command_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.execution_time , packet1.control_mode , packet1.off , packet1.thrust_dir_x , packet1.thrust_dir_y , packet1.thrust_dir_z , packet1.bodyrates_x , packet1.bodyrates_y , packet1.bodyrates_z , packet1.angular_acc_x , packet1.angular_acc_y , packet1.angular_acc_z , packet1.thrust );
    mavlink_msg_inteagle_control_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_inteagle_control_command(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_inteagle_control_command(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // INTEAGLE_CONTROL_COMMAND_TESTSUITE_H
