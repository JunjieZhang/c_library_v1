/** @file
 *    @brief MAVLink comm protocol testsuite generated from inteagle_rc.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef INTEAGLE_RC_TESTSUITE_H
#define INTEAGLE_RC_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_inteagle_rc(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_inteagle_rc(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_inteagle_rc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_INTEAGLE_RC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_inteagle_rc_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,77,144,211,22
    };
    mavlink_inteagle_rc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.channel_1 = packet_in.channel_1;
        packet1.channel_2 = packet_in.channel_2;
        packet1.channel_3 = packet_in.channel_3;
        packet1.channel_4 = packet_in.channel_4;
        packet1.switch_1 = packet_in.switch_1;
        packet1.switch_2 = packet_in.switch_2;
        packet1.switch_3 = packet_in.switch_3;
        packet1.switch_4 = packet_in.switch_4;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_INTEAGLE_RC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_INTEAGLE_RC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_inteagle_rc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_inteagle_rc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_inteagle_rc_pack(system_id, component_id, &msg , packet1.timestamp , packet1.channel_1 , packet1.channel_2 , packet1.channel_3 , packet1.channel_4 , packet1.switch_1 , packet1.switch_2 , packet1.switch_3 , packet1.switch_4 );
    mavlink_msg_inteagle_rc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_inteagle_rc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.channel_1 , packet1.channel_2 , packet1.channel_3 , packet1.channel_4 , packet1.switch_1 , packet1.switch_2 , packet1.switch_3 , packet1.switch_4 );
    mavlink_msg_inteagle_rc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_inteagle_rc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_inteagle_rc_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.channel_1 , packet1.channel_2 , packet1.channel_3 , packet1.channel_4 , packet1.switch_1 , packet1.switch_2 , packet1.switch_3 , packet1.switch_4 );
    mavlink_msg_inteagle_rc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_inteagle_rc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_inteagle_rc(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // INTEAGLE_RC_TESTSUITE_H
