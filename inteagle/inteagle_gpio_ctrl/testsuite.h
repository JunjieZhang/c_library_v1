/** @file
 *    @brief MAVLink comm protocol testsuite generated from inteagle_gpio_ctrl.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef INTEAGLE_GPIO_CTRL_TESTSUITE_H
#define INTEAGLE_GPIO_CTRL_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_inteagle_gpio_ctrl(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_inteagle_gpio_ctrl(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_inteagle_gpio_ctrl(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_INTEAGLE_GPIO_CTRL >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_inteagle_gpio_ctrl_t packet_in = {
        93372036854775807ULL,73.0,41,108
    };
    mavlink_inteagle_gpio_ctrl_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.value = packet_in.value;
        packet1.device = packet_in.device;
        packet1.num_gpio = packet_in.num_gpio;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_INTEAGLE_GPIO_CTRL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_INTEAGLE_GPIO_CTRL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_inteagle_gpio_ctrl_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_inteagle_gpio_ctrl_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_inteagle_gpio_ctrl_pack(system_id, component_id, &msg , packet1.timestamp , packet1.device , packet1.num_gpio , packet1.value );
    mavlink_msg_inteagle_gpio_ctrl_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_inteagle_gpio_ctrl_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.device , packet1.num_gpio , packet1.value );
    mavlink_msg_inteagle_gpio_ctrl_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_inteagle_gpio_ctrl_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_inteagle_gpio_ctrl_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.device , packet1.num_gpio , packet1.value );
    mavlink_msg_inteagle_gpio_ctrl_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_inteagle_gpio_ctrl(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_inteagle_gpio_ctrl(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // INTEAGLE_GPIO_CTRL_TESTSUITE_H
