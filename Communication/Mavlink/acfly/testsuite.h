/** @file
 *    @brief MAVLink comm protocol testsuite generated from acfly.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef ACFLY_TESTSUITE_H
#define ACFLY_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_acfly(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_acfly(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_wind(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_WIND >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_wind_t packet_in = {
        17.0,45.0,73.0
    };
    mavlink_wind_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.direction = packet_in.direction;
        packet1.speed = packet_in.speed;
        packet1.speed_z = packet_in.speed_z;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_WIND_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_WIND_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wind_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_wind_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wind_pack(system_id, component_id, &msg , packet1.direction , packet1.speed , packet1.speed_z );
    mavlink_msg_wind_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wind_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.direction , packet1.speed , packet1.speed_z );
    mavlink_msg_wind_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_wind_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wind_send(MAVLINK_COMM_1 , packet1.direction , packet1.speed , packet1.speed_z );
    mavlink_msg_wind_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("WIND") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_WIND) != NULL);
#endif
}

static void mavlink_test_data64(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DATA64 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_data64_t packet_in = {
        5,72,{ 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202 }
    };
    mavlink_data64_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.type = packet_in.type;
        packet1.len = packet_in.len;
        
        mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*64);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DATA64_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DATA64_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_data64_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_data64_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_data64_pack(system_id, component_id, &msg , packet1.type , packet1.len , packet1.data );
    mavlink_msg_data64_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_data64_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.type , packet1.len , packet1.data );
    mavlink_msg_data64_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_data64_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_data64_send(MAVLINK_COMM_1 , packet1.type , packet1.len , packet1.data );
    mavlink_msg_data64_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("DATA64") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_DATA64) != NULL);
#endif
}

static void mavlink_test_rangefinder(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RANGEFINDER >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_rangefinder_t packet_in = {
        17.0,45.0
    };
    mavlink_rangefinder_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.distance = packet_in.distance;
        packet1.voltage = packet_in.voltage;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RANGEFINDER_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RANGEFINDER_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rangefinder_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_rangefinder_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rangefinder_pack(system_id, component_id, &msg , packet1.distance , packet1.voltage );
    mavlink_msg_rangefinder_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rangefinder_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.distance , packet1.voltage );
    mavlink_msg_rangefinder_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_rangefinder_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rangefinder_send(MAVLINK_COMM_1 , packet1.distance , packet1.voltage );
    mavlink_msg_rangefinder_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("RANGEFINDER") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_RANGEFINDER) != NULL);
#endif
}

static void mavlink_test_rally_point(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RALLY_POINT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_rally_point_t packet_in = {
        963497464,963497672,17651,17755,17859,175,242,53,120,187
    };
    mavlink_rally_point_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.lat = packet_in.lat;
        packet1.lng = packet_in.lng;
        packet1.alt = packet_in.alt;
        packet1.break_alt = packet_in.break_alt;
        packet1.land_dir = packet_in.land_dir;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.idx = packet_in.idx;
        packet1.count = packet_in.count;
        packet1.flags = packet_in.flags;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RALLY_POINT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RALLY_POINT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rally_point_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_rally_point_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rally_point_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.idx , packet1.count , packet1.lat , packet1.lng , packet1.alt , packet1.break_alt , packet1.land_dir , packet1.flags );
    mavlink_msg_rally_point_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rally_point_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.idx , packet1.count , packet1.lat , packet1.lng , packet1.alt , packet1.break_alt , packet1.land_dir , packet1.flags );
    mavlink_msg_rally_point_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_rally_point_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rally_point_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.idx , packet1.count , packet1.lat , packet1.lng , packet1.alt , packet1.break_alt , packet1.land_dir , packet1.flags );
    mavlink_msg_rally_point_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("RALLY_POINT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_RALLY_POINT) != NULL);
#endif
}

static void mavlink_test_rally_fetch_point(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RALLY_FETCH_POINT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_rally_fetch_point_t packet_in = {
        5,72,139
    };
    mavlink_rally_fetch_point_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.idx = packet_in.idx;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RALLY_FETCH_POINT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RALLY_FETCH_POINT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rally_fetch_point_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_rally_fetch_point_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rally_fetch_point_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.idx );
    mavlink_msg_rally_fetch_point_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rally_fetch_point_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.idx );
    mavlink_msg_rally_fetch_point_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_rally_fetch_point_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rally_fetch_point_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.idx );
    mavlink_msg_rally_fetch_point_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("RALLY_FETCH_POINT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_RALLY_FETCH_POINT) != NULL);
#endif
}

static void mavlink_test_compassmot_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_COMPASSMOT_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_compassmot_status_t packet_in = {
        17.0,45.0,73.0,101.0,18067,18171
    };
    mavlink_compassmot_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.current = packet_in.current;
        packet1.CompensationX = packet_in.CompensationX;
        packet1.CompensationY = packet_in.CompensationY;
        packet1.CompensationZ = packet_in.CompensationZ;
        packet1.throttle = packet_in.throttle;
        packet1.interference = packet_in.interference;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_COMPASSMOT_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_COMPASSMOT_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_compassmot_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_compassmot_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_compassmot_status_pack(system_id, component_id, &msg , packet1.throttle , packet1.current , packet1.interference , packet1.CompensationX , packet1.CompensationY , packet1.CompensationZ );
    mavlink_msg_compassmot_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_compassmot_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.throttle , packet1.current , packet1.interference , packet1.CompensationX , packet1.CompensationY , packet1.CompensationZ );
    mavlink_msg_compassmot_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_compassmot_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_compassmot_status_send(MAVLINK_COMM_1 , packet1.throttle , packet1.current , packet1.interference , packet1.CompensationX , packet1.CompensationY , packet1.CompensationZ );
    mavlink_msg_compassmot_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("COMPASSMOT_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_COMPASSMOT_STATUS) != NULL);
#endif
}

static void mavlink_test_ahrs2(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_AHRS2 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_ahrs2_t packet_in = {
        17.0,45.0,73.0,101.0,963498296,963498504
    };
    mavlink_ahrs2_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.roll = packet_in.roll;
        packet1.pitch = packet_in.pitch;
        packet1.yaw = packet_in.yaw;
        packet1.altitude = packet_in.altitude;
        packet1.lat = packet_in.lat;
        packet1.lng = packet_in.lng;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_AHRS2_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_AHRS2_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ahrs2_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_ahrs2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ahrs2_pack(system_id, component_id, &msg , packet1.roll , packet1.pitch , packet1.yaw , packet1.altitude , packet1.lat , packet1.lng );
    mavlink_msg_ahrs2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ahrs2_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.roll , packet1.pitch , packet1.yaw , packet1.altitude , packet1.lat , packet1.lng );
    mavlink_msg_ahrs2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_ahrs2_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ahrs2_send(MAVLINK_COMM_1 , packet1.roll , packet1.pitch , packet1.yaw , packet1.altitude , packet1.lat , packet1.lng );
    mavlink_msg_ahrs2_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("AHRS2") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_AHRS2) != NULL);
#endif
}

static void mavlink_test_camera_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CAMERA_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_camera_status_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,18483,211,22,89
    };
    mavlink_camera_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.p1 = packet_in.p1;
        packet1.p2 = packet_in.p2;
        packet1.p3 = packet_in.p3;
        packet1.p4 = packet_in.p4;
        packet1.img_idx = packet_in.img_idx;
        packet1.target_system = packet_in.target_system;
        packet1.cam_idx = packet_in.cam_idx;
        packet1.event_id = packet_in.event_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CAMERA_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CAMERA_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_camera_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_status_pack(system_id, component_id, &msg , packet1.time_usec , packet1.target_system , packet1.cam_idx , packet1.img_idx , packet1.event_id , packet1.p1 , packet1.p2 , packet1.p3 , packet1.p4 );
    mavlink_msg_camera_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.target_system , packet1.cam_idx , packet1.img_idx , packet1.event_id , packet1.p1 , packet1.p2 , packet1.p3 , packet1.p4 );
    mavlink_msg_camera_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_camera_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_status_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.target_system , packet1.cam_idx , packet1.img_idx , packet1.event_id , packet1.p1 , packet1.p2 , packet1.p3 , packet1.p4 );
    mavlink_msg_camera_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CAMERA_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CAMERA_STATUS) != NULL);
#endif
}

static void mavlink_test_camera_feedback(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CAMERA_FEEDBACK >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_camera_feedback_t packet_in = {
        93372036854775807ULL,963497880,963498088,129.0,157.0,185.0,213.0,241.0,269.0,19315,3,70,137,19575
    };
    mavlink_camera_feedback_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.lat = packet_in.lat;
        packet1.lng = packet_in.lng;
        packet1.alt_msl = packet_in.alt_msl;
        packet1.alt_rel = packet_in.alt_rel;
        packet1.roll = packet_in.roll;
        packet1.pitch = packet_in.pitch;
        packet1.yaw = packet_in.yaw;
        packet1.foc_len = packet_in.foc_len;
        packet1.img_idx = packet_in.img_idx;
        packet1.target_system = packet_in.target_system;
        packet1.cam_idx = packet_in.cam_idx;
        packet1.flags = packet_in.flags;
        packet1.completed_captures = packet_in.completed_captures;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CAMERA_FEEDBACK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CAMERA_FEEDBACK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_feedback_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_camera_feedback_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_feedback_pack(system_id, component_id, &msg , packet1.time_usec , packet1.target_system , packet1.cam_idx , packet1.img_idx , packet1.lat , packet1.lng , packet1.alt_msl , packet1.alt_rel , packet1.roll , packet1.pitch , packet1.yaw , packet1.foc_len , packet1.flags , packet1.completed_captures );
    mavlink_msg_camera_feedback_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_feedback_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.target_system , packet1.cam_idx , packet1.img_idx , packet1.lat , packet1.lng , packet1.alt_msl , packet1.alt_rel , packet1.roll , packet1.pitch , packet1.yaw , packet1.foc_len , packet1.flags , packet1.completed_captures );
    mavlink_msg_camera_feedback_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_camera_feedback_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_feedback_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.target_system , packet1.cam_idx , packet1.img_idx , packet1.lat , packet1.lng , packet1.alt_msl , packet1.alt_rel , packet1.roll , packet1.pitch , packet1.yaw , packet1.foc_len , packet1.flags , packet1.completed_captures );
    mavlink_msg_camera_feedback_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CAMERA_FEEDBACK") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CAMERA_FEEDBACK) != NULL);
#endif
}

static void mavlink_test_battery2(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_BATTERY2 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_battery2_t packet_in = {
        17235,17339
    };
    mavlink_battery2_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.voltage = packet_in.voltage;
        packet1.current_battery = packet_in.current_battery;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_BATTERY2_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_BATTERY2_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_battery2_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_battery2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_battery2_pack(system_id, component_id, &msg , packet1.voltage , packet1.current_battery );
    mavlink_msg_battery2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_battery2_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.voltage , packet1.current_battery );
    mavlink_msg_battery2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_battery2_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_battery2_send(MAVLINK_COMM_1 , packet1.voltage , packet1.current_battery );
    mavlink_msg_battery2_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("BATTERY2") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_BATTERY2) != NULL);
#endif
}

static void mavlink_test_ahrs3(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_AHRS3 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_ahrs3_t packet_in = {
        17.0,45.0,73.0,101.0,963498296,963498504,185.0,213.0,241.0,269.0
    };
    mavlink_ahrs3_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.roll = packet_in.roll;
        packet1.pitch = packet_in.pitch;
        packet1.yaw = packet_in.yaw;
        packet1.altitude = packet_in.altitude;
        packet1.lat = packet_in.lat;
        packet1.lng = packet_in.lng;
        packet1.v1 = packet_in.v1;
        packet1.v2 = packet_in.v2;
        packet1.v3 = packet_in.v3;
        packet1.v4 = packet_in.v4;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_AHRS3_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_AHRS3_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ahrs3_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_ahrs3_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ahrs3_pack(system_id, component_id, &msg , packet1.roll , packet1.pitch , packet1.yaw , packet1.altitude , packet1.lat , packet1.lng , packet1.v1 , packet1.v2 , packet1.v3 , packet1.v4 );
    mavlink_msg_ahrs3_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ahrs3_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.roll , packet1.pitch , packet1.yaw , packet1.altitude , packet1.lat , packet1.lng , packet1.v1 , packet1.v2 , packet1.v3 , packet1.v4 );
    mavlink_msg_ahrs3_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_ahrs3_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ahrs3_send(MAVLINK_COMM_1 , packet1.roll , packet1.pitch , packet1.yaw , packet1.altitude , packet1.lat , packet1.lng , packet1.v1 , packet1.v2 , packet1.v3 , packet1.v4 );
    mavlink_msg_ahrs3_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("AHRS3") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_AHRS3) != NULL);
#endif
}

static void mavlink_test_autopilot_version_request(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_autopilot_version_request_t packet_in = {
        5,72
    };
    mavlink_autopilot_version_request_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_autopilot_version_request_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_autopilot_version_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_autopilot_version_request_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component );
    mavlink_msg_autopilot_version_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_autopilot_version_request_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component );
    mavlink_msg_autopilot_version_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_autopilot_version_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_autopilot_version_request_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component );
    mavlink_msg_autopilot_version_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("AUTOPILOT_VERSION_REQUEST") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST) != NULL);
#endif
}

static void mavlink_test_remote_log_data_block(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_remote_log_data_block_t packet_in = {
        963497464,17,84,{ 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94 }
    };
    mavlink_remote_log_data_block_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.seqno = packet_in.seqno;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*200);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_remote_log_data_block_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_remote_log_data_block_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_remote_log_data_block_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.seqno , packet1.data );
    mavlink_msg_remote_log_data_block_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_remote_log_data_block_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.seqno , packet1.data );
    mavlink_msg_remote_log_data_block_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_remote_log_data_block_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_remote_log_data_block_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.seqno , packet1.data );
    mavlink_msg_remote_log_data_block_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("REMOTE_LOG_DATA_BLOCK") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK) != NULL);
#endif
}

static void mavlink_test_remote_log_block_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_remote_log_block_status_t packet_in = {
        963497464,17,84,151
    };
    mavlink_remote_log_block_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.seqno = packet_in.seqno;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.status = packet_in.status;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_remote_log_block_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_remote_log_block_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_remote_log_block_status_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.seqno , packet1.status );
    mavlink_msg_remote_log_block_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_remote_log_block_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.seqno , packet1.status );
    mavlink_msg_remote_log_block_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_remote_log_block_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_remote_log_block_status_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.seqno , packet1.status );
    mavlink_msg_remote_log_block_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("REMOTE_LOG_BLOCK_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS) != NULL);
#endif
}

static void mavlink_test_led_control(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LED_CONTROL >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_led_control_t packet_in = {
        5,72,139,206,17,{ 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107 }
    };
    mavlink_led_control_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.instance = packet_in.instance;
        packet1.pattern = packet_in.pattern;
        packet1.custom_len = packet_in.custom_len;
        
        mav_array_memcpy(packet1.custom_bytes, packet_in.custom_bytes, sizeof(uint8_t)*24);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LED_CONTROL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LED_CONTROL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_led_control_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_led_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_led_control_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.instance , packet1.pattern , packet1.custom_len , packet1.custom_bytes );
    mavlink_msg_led_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_led_control_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.instance , packet1.pattern , packet1.custom_len , packet1.custom_bytes );
    mavlink_msg_led_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_led_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_led_control_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.instance , packet1.pattern , packet1.custom_len , packet1.custom_bytes );
    mavlink_msg_led_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LED_CONTROL") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LED_CONTROL) != NULL);
#endif
}

static void mavlink_test_mag_cal_progress(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MAG_CAL_PROGRESS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mag_cal_progress_t packet_in = {
        17.0,45.0,73.0,41,108,175,242,53,{ 120, 121, 122, 123, 124, 125, 126, 127, 128, 129 }
    };
    mavlink_mag_cal_progress_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.direction_x = packet_in.direction_x;
        packet1.direction_y = packet_in.direction_y;
        packet1.direction_z = packet_in.direction_z;
        packet1.compass_id = packet_in.compass_id;
        packet1.cal_mask = packet_in.cal_mask;
        packet1.cal_status = packet_in.cal_status;
        packet1.attempt = packet_in.attempt;
        packet1.completion_pct = packet_in.completion_pct;
        
        mav_array_memcpy(packet1.completion_mask, packet_in.completion_mask, sizeof(uint8_t)*10);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MAG_CAL_PROGRESS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MAG_CAL_PROGRESS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mag_cal_progress_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mag_cal_progress_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mag_cal_progress_pack(system_id, component_id, &msg , packet1.compass_id , packet1.cal_mask , packet1.cal_status , packet1.attempt , packet1.completion_pct , packet1.completion_mask , packet1.direction_x , packet1.direction_y , packet1.direction_z );
    mavlink_msg_mag_cal_progress_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mag_cal_progress_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.compass_id , packet1.cal_mask , packet1.cal_status , packet1.attempt , packet1.completion_pct , packet1.completion_mask , packet1.direction_x , packet1.direction_y , packet1.direction_z );
    mavlink_msg_mag_cal_progress_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mag_cal_progress_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mag_cal_progress_send(MAVLINK_COMM_1 , packet1.compass_id , packet1.cal_mask , packet1.cal_status , packet1.attempt , packet1.completion_pct , packet1.completion_mask , packet1.direction_x , packet1.direction_y , packet1.direction_z );
    mavlink_msg_mag_cal_progress_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MAG_CAL_PROGRESS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MAG_CAL_PROGRESS) != NULL);
#endif
}

static void mavlink_test_sys_status_acfly(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SYS_STATUS_ACFLY >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_sys_status_acfly_t packet_in = {
        963497464,963497672,963497880,963498088,963498296,18275,{ 18379, 18380 },{ 18587, 18588 },{ 18795, 18796 },19003,19107,19211,19315,19419,19523
    };
    mavlink_sys_status_acfly_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.remaining_memory = packet_in.remaining_memory;
        packet1.sysStatusFlag = packet_in.sysStatusFlag;
        packet1.sysStatusFlag2 = packet_in.sysStatusFlag2;
        packet1.pos_cflags = packet_in.pos_cflags;
        packet1.pos_aflags = packet_in.pos_aflags;
        packet1.cpu_usage = packet_in.cpu_usage;
        packet1.sd_card_capacity = packet_in.sd_card_capacity;
        packet1.sd_card_remain = packet_in.sd_card_remain;
        packet1.imu_cflags = packet_in.imu_cflags;
        packet1.imu_aflags = packet_in.imu_aflags;
        packet1.mode_code = packet_in.mode_code;
        packet1.mode_code_reason = packet_in.mode_code_reason;
        
        mav_array_memcpy(packet1.daoYaw, packet_in.daoYaw, sizeof(int16_t)*2);
        mav_array_memcpy(packet1.adc_voltage, packet_in.adc_voltage, sizeof(uint16_t)*2);
        mav_array_memcpy(packet1.adc_current, packet_in.adc_current, sizeof(uint16_t)*2);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SYS_STATUS_ACFLY_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SYS_STATUS_ACFLY_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sys_status_acfly_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_sys_status_acfly_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sys_status_acfly_pack(system_id, component_id, &msg , packet1.cpu_usage , packet1.remaining_memory , packet1.daoYaw , packet1.adc_voltage , packet1.adc_current , packet1.sd_card_capacity , packet1.sd_card_remain , packet1.sysStatusFlag , packet1.sysStatusFlag2 , packet1.pos_cflags , packet1.pos_aflags , packet1.imu_cflags , packet1.imu_aflags , packet1.mode_code , packet1.mode_code_reason );
    mavlink_msg_sys_status_acfly_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sys_status_acfly_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.cpu_usage , packet1.remaining_memory , packet1.daoYaw , packet1.adc_voltage , packet1.adc_current , packet1.sd_card_capacity , packet1.sd_card_remain , packet1.sysStatusFlag , packet1.sysStatusFlag2 , packet1.pos_cflags , packet1.pos_aflags , packet1.imu_cflags , packet1.imu_aflags , packet1.mode_code , packet1.mode_code_reason );
    mavlink_msg_sys_status_acfly_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_sys_status_acfly_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sys_status_acfly_send(MAVLINK_COMM_1 , packet1.cpu_usage , packet1.remaining_memory , packet1.daoYaw , packet1.adc_voltage , packet1.adc_current , packet1.sd_card_capacity , packet1.sd_card_remain , packet1.sysStatusFlag , packet1.sysStatusFlag2 , packet1.pos_cflags , packet1.pos_aflags , packet1.imu_cflags , packet1.imu_aflags , packet1.mode_code , packet1.mode_code_reason );
    mavlink_msg_sys_status_acfly_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SYS_STATUS_ACFLY") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SYS_STATUS_ACFLY) != NULL);
#endif
}

static void mavlink_test_acflyhtl_pwmchans_fb(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ACFLYHTL_PWMCHANS_FB >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_acflyhtl_pwmchans_fb_t packet_in = {
        { 17235, 17236, 17237, 17238, 17239, 17240, 17241, 17242, 17243, 17244, 17245, 17246, 17247, 17248, 17249, 17250, 17251, 17252, 17253, 17254, 17255, 17256, 17257, 17258, 17259, 17260, 17261, 17262, 17263, 17264, 17265, 17266 }
    };
    mavlink_acflyhtl_pwmchans_fb_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mav_array_memcpy(packet1.pwms, packet_in.pwms, sizeof(uint16_t)*32);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ACFLYHTL_PWMCHANS_FB_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ACFLYHTL_PWMCHANS_FB_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_acflyhtl_pwmchans_fb_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_acflyhtl_pwmchans_fb_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_acflyhtl_pwmchans_fb_pack(system_id, component_id, &msg , packet1.pwms );
    mavlink_msg_acflyhtl_pwmchans_fb_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_acflyhtl_pwmchans_fb_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.pwms );
    mavlink_msg_acflyhtl_pwmchans_fb_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_acflyhtl_pwmchans_fb_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_acflyhtl_pwmchans_fb_send(MAVLINK_COMM_1 , packet1.pwms );
    mavlink_msg_acflyhtl_pwmchans_fb_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ACFLYHTL_PWMCHANS_FB") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ACFLYHTL_PWMCHANS_FB) != NULL);
#endif
}

static void mavlink_test_acflyhtl_imu_register(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ACFLYHTL_IMU_REGISTER >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_acflyhtl_imu_register_t packet_in = {
        123.0,17651,17755,41,108,"OPQRSTUVWXYZABC"
    };
    mavlink_acflyhtl_imu_register_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.sensitivity = packet_in.sensitivity;
        packet1.imu_type = packet_in.imu_type;
        packet1.freq = packet_in.freq;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        mav_array_memcpy(packet1.sensor_name, packet_in.sensor_name, sizeof(char)*16);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ACFLYHTL_IMU_REGISTER_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ACFLYHTL_IMU_REGISTER_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_acflyhtl_imu_register_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_acflyhtl_imu_register_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_acflyhtl_imu_register_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.imu_type , packet1.freq , packet1.sensor_name , packet1.sensitivity );
    mavlink_msg_acflyhtl_imu_register_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_acflyhtl_imu_register_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.imu_type , packet1.freq , packet1.sensor_name , packet1.sensitivity );
    mavlink_msg_acflyhtl_imu_register_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_acflyhtl_imu_register_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_acflyhtl_imu_register_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.imu_type , packet1.freq , packet1.sensor_name , packet1.sensitivity );
    mavlink_msg_acflyhtl_imu_register_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ACFLYHTL_IMU_REGISTER") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ACFLYHTL_IMU_REGISTER) != NULL);
#endif
}

static void mavlink_test_acflyhtl_imu_update(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ACFLYHTL_IMU_UPDATE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_acflyhtl_imu_update_t packet_in = {
        963497464,963497672,963497880,17859,175,242
    };
    mavlink_acflyhtl_imu_update_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.imu_type = packet_in.imu_type;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ACFLYHTL_IMU_UPDATE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ACFLYHTL_IMU_UPDATE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_acflyhtl_imu_update_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_acflyhtl_imu_update_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_acflyhtl_imu_update_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.imu_type , packet1.x , packet1.y , packet1.z );
    mavlink_msg_acflyhtl_imu_update_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_acflyhtl_imu_update_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.imu_type , packet1.x , packet1.y , packet1.z );
    mavlink_msg_acflyhtl_imu_update_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_acflyhtl_imu_update_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_acflyhtl_imu_update_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.imu_type , packet1.x , packet1.y , packet1.z );
    mavlink_msg_acflyhtl_imu_update_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ACFLYHTL_IMU_UPDATE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ACFLYHTL_IMU_UPDATE) != NULL);
#endif
}

static void mavlink_test_acfly_mission_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ACFLY_MISSION_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_acfly_mission_status_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0,18899,19003,19107
    };
    mavlink_acfly_mission_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.heading = packet_in.heading;
        packet1.vector_x = packet_in.vector_x;
        packet1.vector_y = packet_in.vector_y;
        packet1.vector_z = packet_in.vector_z;
        packet1.distance = packet_in.distance;
        packet1.trigger_interval = packet_in.trigger_interval;
        packet1.speed = packet_in.speed;
        packet1.radius = packet_in.radius;
        packet1.seq = packet_in.seq;
        packet1.resumeSeq = packet_in.resumeSeq;
        packet1.flag = packet_in.flag;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ACFLY_MISSION_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_acfly_mission_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_acfly_mission_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_acfly_mission_status_pack(system_id, component_id, &msg , packet1.seq , packet1.resumeSeq , packet1.flag , packet1.heading , packet1.vector_x , packet1.vector_y , packet1.vector_z , packet1.distance , packet1.trigger_interval , packet1.speed , packet1.radius );
    mavlink_msg_acfly_mission_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_acfly_mission_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.seq , packet1.resumeSeq , packet1.flag , packet1.heading , packet1.vector_x , packet1.vector_y , packet1.vector_z , packet1.distance , packet1.trigger_interval , packet1.speed , packet1.radius );
    mavlink_msg_acfly_mission_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_acfly_mission_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_acfly_mission_status_send(MAVLINK_COMM_1 , packet1.seq , packet1.resumeSeq , packet1.flag , packet1.heading , packet1.vector_x , packet1.vector_y , packet1.vector_z , packet1.distance , packet1.trigger_interval , packet1.speed , packet1.radius );
    mavlink_msg_acfly_mission_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ACFLY_MISSION_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ACFLY_MISSION_STATUS) != NULL);
#endif
}

static void mavlink_test_acflypossensor_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ACFlyPosSensor_INFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_acflypossensor_info_t packet_in = {
        93372036854775807ULL,179.0,235.0,291.0,241.0,269.0,297.0,963499752,963499960,381.0,409.0,437.0,"MNOPQRSTUVWXYZA",245,56,123,190,{ 1, 2, 3, 4 },{ 633.0, 634.0, 635.0, 636.0, 637.0, 638.0, 639.0, 640.0 }
    };
    mavlink_acflypossensor_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.posE = packet_in.posE;
        packet1.posN = packet_in.posN;
        packet1.posU = packet_in.posU;
        packet1.velE = packet_in.velE;
        packet1.velN = packet_in.velN;
        packet1.velU = packet_in.velU;
        packet1.Lat = packet_in.Lat;
        packet1.Lon = packet_in.Lon;
        packet1.delay = packet_in.delay;
        packet1.trustXY = packet_in.trustXY;
        packet1.trustZ = packet_in.trustZ;
        packet1.ind = packet_in.ind;
        packet1.type = packet_in.type;
        packet1.DataFrame = packet_in.DataFrame;
        packet1.DataType = packet_in.DataType;
        
        mav_array_memcpy(packet1.sensor_name, packet_in.sensor_name, sizeof(char)*16);
        mav_array_memcpy(packet1.additionInfo1, packet_in.additionInfo1, sizeof(uint8_t)*4);
        mav_array_memcpy(packet1.additionInfo2, packet_in.additionInfo2, sizeof(float)*8);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ACFlyPosSensor_INFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ACFlyPosSensor_INFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_acflypossensor_info_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_acflypossensor_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_acflypossensor_info_pack(system_id, component_id, &msg , packet1.time_usec , packet1.posE , packet1.posN , packet1.posU , packet1.velE , packet1.velN , packet1.velU , packet1.Lat , packet1.Lon , packet1.delay , packet1.trustXY , packet1.trustZ , packet1.sensor_name , packet1.ind , packet1.type , packet1.DataFrame , packet1.DataType , packet1.additionInfo1 , packet1.additionInfo2 );
    mavlink_msg_acflypossensor_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_acflypossensor_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.posE , packet1.posN , packet1.posU , packet1.velE , packet1.velN , packet1.velU , packet1.Lat , packet1.Lon , packet1.delay , packet1.trustXY , packet1.trustZ , packet1.sensor_name , packet1.ind , packet1.type , packet1.DataFrame , packet1.DataType , packet1.additionInfo1 , packet1.additionInfo2 );
    mavlink_msg_acflypossensor_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_acflypossensor_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_acflypossensor_info_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.posE , packet1.posN , packet1.posU , packet1.velE , packet1.velN , packet1.velU , packet1.Lat , packet1.Lon , packet1.delay , packet1.trustXY , packet1.trustZ , packet1.sensor_name , packet1.ind , packet1.type , packet1.DataFrame , packet1.DataType , packet1.additionInfo1 , packet1.additionInfo2 );
    mavlink_msg_acflypossensor_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ACFlyPosSensor_INFO") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ACFlyPosSensor_INFO) != NULL);
#endif
}

static void mavlink_test_follow_target_acfly(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_FOLLOW_TARGET_ACFLY >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_follow_target_acfly_t packet_in = {
        93372036854775807ULL,93372036854776311ULL,963498296,963498504,185.0,{ 213.0, 214.0, 215.0 },{ 297.0, 298.0, 299.0 },{ 381.0, 382.0, 383.0, 384.0 },{ 493.0, 494.0, 495.0 },{ 577.0, 578.0, 579.0 },661.0,37
    };
    mavlink_follow_target_acfly_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.custom_state = packet_in.custom_state;
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.alt = packet_in.alt;
        packet1.yaw = packet_in.yaw;
        packet1.est_capabilities = packet_in.est_capabilities;
        
        mav_array_memcpy(packet1.vel, packet_in.vel, sizeof(float)*3);
        mav_array_memcpy(packet1.acc, packet_in.acc, sizeof(float)*3);
        mav_array_memcpy(packet1.attitude_q, packet_in.attitude_q, sizeof(float)*4);
        mav_array_memcpy(packet1.rates, packet_in.rates, sizeof(float)*3);
        mav_array_memcpy(packet1.position_cov, packet_in.position_cov, sizeof(float)*3);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_FOLLOW_TARGET_ACFLY_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_FOLLOW_TARGET_ACFLY_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_follow_target_acfly_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_follow_target_acfly_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_follow_target_acfly_pack(system_id, component_id, &msg , packet1.timestamp , packet1.est_capabilities , packet1.lat , packet1.lon , packet1.alt , packet1.vel , packet1.acc , packet1.attitude_q , packet1.rates , packet1.position_cov , packet1.custom_state , packet1.yaw );
    mavlink_msg_follow_target_acfly_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_follow_target_acfly_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.est_capabilities , packet1.lat , packet1.lon , packet1.alt , packet1.vel , packet1.acc , packet1.attitude_q , packet1.rates , packet1.position_cov , packet1.custom_state , packet1.yaw );
    mavlink_msg_follow_target_acfly_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_follow_target_acfly_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_follow_target_acfly_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.est_capabilities , packet1.lat , packet1.lon , packet1.alt , packet1.vel , packet1.acc , packet1.attitude_q , packet1.rates , packet1.position_cov , packet1.custom_state , packet1.yaw );
    mavlink_msg_follow_target_acfly_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("FOLLOW_TARGET_ACFLY") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_FOLLOW_TARGET_ACFLY) != NULL);
#endif
}

static void mavlink_test_acfly_regeisterpossensor(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ACFLY_REGEISTERPOSSENSOR >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_acfly_regeisterpossensor_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,65,132,"WXYZABCDEFGHIJK",247,58,125,192
    };
    mavlink_acfly_regeisterpossensor_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.delay = packet_in.delay;
        packet1.trustXY = packet_in.trustXY;
        packet1.trustZ = packet_in.trustZ;
        packet1.LTTrustXY = packet_in.LTTrustXY;
        packet1.LTTrustZ = packet_in.LTTrustZ;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.ind = packet_in.ind;
        packet1.type = packet_in.type;
        packet1.DataFrame = packet_in.DataFrame;
        packet1.DataType = packet_in.DataType;
        
        mav_array_memcpy(packet1.sensor_name, packet_in.sensor_name, sizeof(char)*16);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ACFLY_REGEISTERPOSSENSOR_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ACFLY_REGEISTERPOSSENSOR_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_acfly_regeisterpossensor_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_acfly_regeisterpossensor_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_acfly_regeisterpossensor_pack(system_id, component_id, &msg , packet1.delay , packet1.trustXY , packet1.trustZ , packet1.LTTrustXY , packet1.LTTrustZ , packet1.target_system , packet1.target_component , packet1.sensor_name , packet1.ind , packet1.type , packet1.DataFrame , packet1.DataType );
    mavlink_msg_acfly_regeisterpossensor_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_acfly_regeisterpossensor_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.delay , packet1.trustXY , packet1.trustZ , packet1.LTTrustXY , packet1.LTTrustZ , packet1.target_system , packet1.target_component , packet1.sensor_name , packet1.ind , packet1.type , packet1.DataFrame , packet1.DataType );
    mavlink_msg_acfly_regeisterpossensor_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_acfly_regeisterpossensor_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_acfly_regeisterpossensor_send(MAVLINK_COMM_1 , packet1.delay , packet1.trustXY , packet1.trustZ , packet1.LTTrustXY , packet1.LTTrustZ , packet1.target_system , packet1.target_component , packet1.sensor_name , packet1.ind , packet1.type , packet1.DataFrame , packet1.DataType );
    mavlink_msg_acfly_regeisterpossensor_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ACFLY_REGEISTERPOSSENSOR") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ACFLY_REGEISTERPOSSENSOR) != NULL);
#endif
}

static void mavlink_test_acfly_updatepossensor(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ACFLY_UPDATEPOSSENSOR >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_acfly_updatepossensor_t packet_in = {
        123.0,179.0,235.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0,173,240,51,118,185,{ 444.0, 445.0, 446.0, 447.0 }
    };
    mavlink_acfly_updatepossensor_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.posX = packet_in.posX;
        packet1.posY = packet_in.posY;
        packet1.posZ = packet_in.posZ;
        packet1.velX = packet_in.velX;
        packet1.velY = packet_in.velY;
        packet1.velZ = packet_in.velZ;
        packet1.delay = packet_in.delay;
        packet1.trustXY = packet_in.trustXY;
        packet1.trustZ = packet_in.trustZ;
        packet1.LTTrustXY = packet_in.LTTrustXY;
        packet1.LTTrustZ = packet_in.LTTrustZ;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.ind = packet_in.ind;
        packet1.DataType = packet_in.DataType;
        packet1.reset = packet_in.reset;
        
        mav_array_memcpy(packet1.AttQuat, packet_in.AttQuat, sizeof(float)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ACFLY_UPDATEPOSSENSOR_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ACFLY_UPDATEPOSSENSOR_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_acfly_updatepossensor_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_acfly_updatepossensor_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_acfly_updatepossensor_pack(system_id, component_id, &msg , packet1.posX , packet1.posY , packet1.posZ , packet1.velX , packet1.velY , packet1.velZ , packet1.delay , packet1.trustXY , packet1.trustZ , packet1.LTTrustXY , packet1.LTTrustZ , packet1.target_system , packet1.target_component , packet1.ind , packet1.DataType , packet1.reset , packet1.AttQuat );
    mavlink_msg_acfly_updatepossensor_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_acfly_updatepossensor_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.posX , packet1.posY , packet1.posZ , packet1.velX , packet1.velY , packet1.velZ , packet1.delay , packet1.trustXY , packet1.trustZ , packet1.LTTrustXY , packet1.LTTrustZ , packet1.target_system , packet1.target_component , packet1.ind , packet1.DataType , packet1.reset , packet1.AttQuat );
    mavlink_msg_acfly_updatepossensor_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_acfly_updatepossensor_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_acfly_updatepossensor_send(MAVLINK_COMM_1 , packet1.posX , packet1.posY , packet1.posZ , packet1.velX , packet1.velY , packet1.velZ , packet1.delay , packet1.trustXY , packet1.trustZ , packet1.LTTrustXY , packet1.LTTrustZ , packet1.target_system , packet1.target_component , packet1.ind , packet1.DataType , packet1.reset , packet1.AttQuat );
    mavlink_msg_acfly_updatepossensor_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ACFLY_UPDATEPOSSENSOR") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ACFLY_UPDATEPOSSENSOR) != NULL);
#endif
}

static void mavlink_test_mission_report(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MISSION_REPORT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mission_report_t packet_in = {
        93372036854775807ULL,17651,17755,17859,175,242
    };
    mavlink_mission_report_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.seq = packet_in.seq;
        packet1.cmd = packet_in.cmd;
        packet1.mission_count = packet_in.mission_count;
        packet1.progress = packet_in.progress;
        packet1.result = packet_in.result;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MISSION_REPORT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MISSION_REPORT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_report_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mission_report_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_report_pack(system_id, component_id, &msg , packet1.time_usec , packet1.seq , packet1.cmd , packet1.mission_count , packet1.progress , packet1.result );
    mavlink_msg_mission_report_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_report_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.seq , packet1.cmd , packet1.mission_count , packet1.progress , packet1.result );
    mavlink_msg_mission_report_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mission_report_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_report_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.seq , packet1.cmd , packet1.mission_count , packet1.progress , packet1.result );
    mavlink_msg_mission_report_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MISSION_REPORT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MISSION_REPORT) != NULL);
#endif
}

static void mavlink_test_thread_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_THREAD_INFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_thread_info_t packet_in = {
        963497464,17443,17547,17651,163,230,41,108,"OPQRSTUVWXYZABCDEFGHIJK"
    };
    mavlink_thread_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.remaining_stack = packet_in.remaining_stack;
        packet1.stack_size = packet_in.stack_size;
        packet1.cpu_usage = packet_in.cpu_usage;
        packet1.index = packet_in.index;
        packet1.num_threads = packet_in.num_threads;
        packet1.priority = packet_in.priority;
        packet1.state = packet_in.state;
        
        mav_array_memcpy(packet1.name, packet_in.name, sizeof(char)*24);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_THREAD_INFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_THREAD_INFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_thread_info_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_thread_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_thread_info_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.remaining_stack , packet1.stack_size , packet1.cpu_usage , packet1.index , packet1.num_threads , packet1.priority , packet1.state , packet1.name );
    mavlink_msg_thread_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_thread_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.remaining_stack , packet1.stack_size , packet1.cpu_usage , packet1.index , packet1.num_threads , packet1.priority , packet1.state , packet1.name );
    mavlink_msg_thread_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_thread_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_thread_info_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.remaining_stack , packet1.stack_size , packet1.cpu_usage , packet1.index , packet1.num_threads , packet1.priority , packet1.state , packet1.name );
    mavlink_msg_thread_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("THREAD_INFO") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_THREAD_INFO) != NULL);
#endif
}

static void mavlink_test_water_quality_sensor(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_WATER_QUALITY_SENSOR >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_water_quality_sensor_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,65
    };
    mavlink_water_quality_sensor_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.temperature = packet_in.temperature;
        packet1.pH = packet_in.pH;
        packet1.dissolved_oxygen = packet_in.dissolved_oxygen;
        packet1.turbidity = packet_in.turbidity;
        packet1.conductivity = packet_in.conductivity;
        packet1.type = packet_in.type;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_WATER_QUALITY_SENSOR_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_WATER_QUALITY_SENSOR_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_water_quality_sensor_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_water_quality_sensor_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_water_quality_sensor_pack(system_id, component_id, &msg , packet1.temperature , packet1.pH , packet1.dissolved_oxygen , packet1.turbidity , packet1.conductivity , packet1.type );
    mavlink_msg_water_quality_sensor_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_water_quality_sensor_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.temperature , packet1.pH , packet1.dissolved_oxygen , packet1.turbidity , packet1.conductivity , packet1.type );
    mavlink_msg_water_quality_sensor_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_water_quality_sensor_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_water_quality_sensor_send(MAVLINK_COMM_1 , packet1.temperature , packet1.pH , packet1.dissolved_oxygen , packet1.turbidity , packet1.conductivity , packet1.type );
    mavlink_msg_water_quality_sensor_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("WATER_QUALITY_SENSOR") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_WATER_QUALITY_SENSOR) != NULL);
#endif
}

static void mavlink_test_component_extension43(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_COMPONENT_EXTENSION43 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_component_extension43_t packet_in = {
        17235,139,206,17,{ 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126 }
    };
    mavlink_component_extension43_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.message_type = packet_in.message_type;
        packet1.target_network = packet_in.target_network;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        mav_array_memcpy(packet1.payload, packet_in.payload, sizeof(uint8_t)*43);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_COMPONENT_EXTENSION43_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_COMPONENT_EXTENSION43_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_component_extension43_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_component_extension43_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_component_extension43_pack(system_id, component_id, &msg , packet1.target_network , packet1.target_system , packet1.target_component , packet1.message_type , packet1.payload );
    mavlink_msg_component_extension43_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_component_extension43_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_network , packet1.target_system , packet1.target_component , packet1.message_type , packet1.payload );
    mavlink_msg_component_extension43_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_component_extension43_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_component_extension43_send(MAVLINK_COMM_1 , packet1.target_network , packet1.target_system , packet1.target_component , packet1.message_type , packet1.payload );
    mavlink_msg_component_extension43_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("COMPONENT_EXTENSION43") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_COMPONENT_EXTENSION43) != NULL);
#endif
}

static void mavlink_test_battery_status_acfly(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_battery_status_acfly_t packet_in = {
        963497464,963497672,963497880,17859,17963,18067,18171,65,132,199,10,{ 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124 },"UVWXYZABCDEFGHIJKLMNOPQRSTUVW"
    };
    mavlink_battery_status_acfly_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.voltage = packet_in.voltage;
        packet1.capacity = packet_in.capacity;
        packet1.sequence_num = packet_in.sequence_num;
        packet1.fault_bitmask = packet_in.fault_bitmask;
        packet1.temperature = packet_in.temperature;
        packet1.cycle_count = packet_in.cycle_count;
        packet1.current = packet_in.current;
        packet1.id = packet_in.id;
        packet1.health = packet_in.health;
        packet1.remaining_percentage = packet_in.remaining_percentage;
        packet1.cell_count = packet_in.cell_count;
        
        mav_array_memcpy(packet1.voltages, packet_in.voltages, sizeof(uint8_t)*48);
        mav_array_memcpy(packet1.serial_number, packet_in.serial_number, sizeof(char)*30);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_battery_status_acfly_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_battery_status_acfly_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_battery_status_acfly_pack(system_id, component_id, &msg , packet1.id , packet1.health , packet1.remaining_percentage , packet1.cell_count , packet1.fault_bitmask , packet1.temperature , packet1.cycle_count , packet1.current , packet1.voltage , packet1.capacity , packet1.sequence_num , packet1.voltages , packet1.serial_number );
    mavlink_msg_battery_status_acfly_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_battery_status_acfly_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.id , packet1.health , packet1.remaining_percentage , packet1.cell_count , packet1.fault_bitmask , packet1.temperature , packet1.cycle_count , packet1.current , packet1.voltage , packet1.capacity , packet1.sequence_num , packet1.voltages , packet1.serial_number );
    mavlink_msg_battery_status_acfly_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_battery_status_acfly_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_battery_status_acfly_send(MAVLINK_COMM_1 , packet1.id , packet1.health , packet1.remaining_percentage , packet1.cell_count , packet1.fault_bitmask , packet1.temperature , packet1.cycle_count , packet1.current , packet1.voltage , packet1.capacity , packet1.sequence_num , packet1.voltages , packet1.serial_number );
    mavlink_msg_battery_status_acfly_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("BATTERY_STATUS_ACFLY") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_BATTERY_STATUS_ACFLY) != NULL);
#endif
}

static void mavlink_test_hydrogen_battery_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_hydrogen_battery_info_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0
    };
    mavlink_hydrogen_battery_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.humidity = packet_in.humidity;
        packet1.temperature = packet_in.temperature;
        packet1.fan_speed = packet_in.fan_speed;
        packet1.hb_tank_pressure = packet_in.hb_tank_pressure;
        packet1.hb_gas_pressure = packet_in.hb_gas_pressure;
        packet1.hb_voltage = packet_in.hb_voltage;
        packet1.hb_current = packet_in.hb_current;
        packet1.hb_Temperature = packet_in.hb_Temperature;
        packet1.hb_power = packet_in.hb_power;
        packet1.lP_voltage = packet_in.lP_voltage;
        packet1.lP_current = packet_in.lP_current;
        packet1.lP_Temperature = packet_in.lP_Temperature;
        packet1.lP_power = packet_in.lP_power;
        packet1.toatal_power = packet_in.toatal_power;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hydrogen_battery_info_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_hydrogen_battery_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hydrogen_battery_info_pack(system_id, component_id, &msg , packet1.humidity , packet1.temperature , packet1.fan_speed , packet1.hb_tank_pressure , packet1.hb_gas_pressure , packet1.hb_voltage , packet1.hb_current , packet1.hb_Temperature , packet1.hb_power , packet1.lP_voltage , packet1.lP_current , packet1.lP_Temperature , packet1.lP_power , packet1.toatal_power );
    mavlink_msg_hydrogen_battery_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hydrogen_battery_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.humidity , packet1.temperature , packet1.fan_speed , packet1.hb_tank_pressure , packet1.hb_gas_pressure , packet1.hb_voltage , packet1.hb_current , packet1.hb_Temperature , packet1.hb_power , packet1.lP_voltage , packet1.lP_current , packet1.lP_Temperature , packet1.lP_power , packet1.toatal_power );
    mavlink_msg_hydrogen_battery_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_hydrogen_battery_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hydrogen_battery_info_send(MAVLINK_COMM_1 , packet1.humidity , packet1.temperature , packet1.fan_speed , packet1.hb_tank_pressure , packet1.hb_gas_pressure , packet1.hb_voltage , packet1.hb_current , packet1.hb_Temperature , packet1.hb_power , packet1.lP_voltage , packet1.lP_current , packet1.lP_Temperature , packet1.lP_power , packet1.toatal_power );
    mavlink_msg_hydrogen_battery_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("HYDROGEN_BATTERY_INFO") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_HYDROGEN_BATTERY_INFO) != NULL);
#endif
}

static void mavlink_test_camera_gimbal_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_camera_gimbal_status_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,963499544,963499752,963499960,19939,20043,20147,20251,185,252,63,130,197,8
    };
    mavlink_camera_gimbal_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.gimBal_pitch = packet_in.gimBal_pitch;
        packet1.gimBal_yaw = packet_in.gimBal_yaw;
        packet1.hfov_now = packet_in.hfov_now;
        packet1.hfov_min = packet_in.hfov_min;
        packet1.hfov_max = packet_in.hfov_max;
        packet1.vfov_now = packet_in.vfov_now;
        packet1.vfov_min = packet_in.vfov_min;
        packet1.vfov_max = packet_in.vfov_max;
        packet1.distance_detected = packet_in.distance_detected;
        packet1.target_lat = packet_in.target_lat;
        packet1.target_lon = packet_in.target_lon;
        packet1.target_alt = packet_in.target_alt;
        packet1.target_xMin = packet_in.target_xMin;
        packet1.target_xMax = packet_in.target_xMax;
        packet1.target_yMin = packet_in.target_yMin;
        packet1.target_yMax = packet_in.target_yMax;
        packet1.currentZoomLevel = packet_in.currentZoomLevel;
        packet1.zoomLevel_min = packet_in.zoomLevel_min;
        packet1.zoomLevel_max = packet_in.zoomLevel_max;
        packet1.yaw_mode = packet_in.yaw_mode;
        packet1.track_mode = packet_in.track_mode;
        packet1.flag = packet_in.flag;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_gimbal_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_camera_gimbal_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_gimbal_status_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.gimBal_pitch , packet1.gimBal_yaw , packet1.hfov_now , packet1.hfov_min , packet1.hfov_max , packet1.vfov_now , packet1.vfov_min , packet1.vfov_max , packet1.distance_detected , packet1.target_lat , packet1.target_lon , packet1.target_alt , packet1.target_xMin , packet1.target_xMax , packet1.target_yMin , packet1.target_yMax , packet1.currentZoomLevel , packet1.zoomLevel_min , packet1.zoomLevel_max , packet1.yaw_mode , packet1.track_mode , packet1.flag );
    mavlink_msg_camera_gimbal_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_gimbal_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.gimBal_pitch , packet1.gimBal_yaw , packet1.hfov_now , packet1.hfov_min , packet1.hfov_max , packet1.vfov_now , packet1.vfov_min , packet1.vfov_max , packet1.distance_detected , packet1.target_lat , packet1.target_lon , packet1.target_alt , packet1.target_xMin , packet1.target_xMax , packet1.target_yMin , packet1.target_yMax , packet1.currentZoomLevel , packet1.zoomLevel_min , packet1.zoomLevel_max , packet1.yaw_mode , packet1.track_mode , packet1.flag );
    mavlink_msg_camera_gimbal_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_camera_gimbal_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_gimbal_status_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.gimBal_pitch , packet1.gimBal_yaw , packet1.hfov_now , packet1.hfov_min , packet1.hfov_max , packet1.vfov_now , packet1.vfov_min , packet1.vfov_max , packet1.distance_detected , packet1.target_lat , packet1.target_lon , packet1.target_alt , packet1.target_xMin , packet1.target_xMax , packet1.target_yMin , packet1.target_yMax , packet1.currentZoomLevel , packet1.zoomLevel_min , packet1.zoomLevel_max , packet1.yaw_mode , packet1.track_mode , packet1.flag );
    mavlink_msg_camera_gimbal_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CAMERA_GIMBAL_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS) != NULL);
#endif
}

static void mavlink_test_uavcan_raw(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_UAVCAN_RAW >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_uavcan_raw_t packet_in = {
        963497464,17,84,151,218,29,{ 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159 }
    };
    mavlink_uavcan_raw_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.can_id = packet_in.can_id;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.sequence = packet_in.sequence;
        packet1.can_port = packet_in.can_port;
        packet1.payloadLen = packet_in.payloadLen;
        
        mav_array_memcpy(packet1.payload, packet_in.payload, sizeof(uint8_t)*64);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_UAVCAN_RAW_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_UAVCAN_RAW_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uavcan_raw_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_uavcan_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uavcan_raw_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.sequence , packet1.can_port , packet1.payloadLen , packet1.can_id , packet1.payload );
    mavlink_msg_uavcan_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uavcan_raw_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.sequence , packet1.can_port , packet1.payloadLen , packet1.can_id , packet1.payload );
    mavlink_msg_uavcan_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_uavcan_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uavcan_raw_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.sequence , packet1.can_port , packet1.payloadLen , packet1.can_id , packet1.payload );
    mavlink_msg_uavcan_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("UAVCAN_RAW") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_UAVCAN_RAW) != NULL);
#endif
}

static void mavlink_test_auto_route_request(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_auto_route_request_t packet_in = {
        963497464,963497672,963497880,963498088,963498296,963498504,77,144,211,22
    };
    mavlink_auto_route_request_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.lat1 = packet_in.lat1;
        packet1.lon1 = packet_in.lon1;
        packet1.alt1 = packet_in.alt1;
        packet1.lat2 = packet_in.lat2;
        packet1.lon2 = packet_in.lon2;
        packet1.alt2 = packet_in.alt2;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.type = packet_in.type;
        packet1.mode = packet_in.mode;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_auto_route_request_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_auto_route_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_auto_route_request_pack(system_id, component_id, &msg , packet1.lat1 , packet1.lon1 , packet1.alt1 , packet1.lat2 , packet1.lon2 , packet1.alt2 , packet1.target_system , packet1.target_component , packet1.type , packet1.mode );
    mavlink_msg_auto_route_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_auto_route_request_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.lat1 , packet1.lon1 , packet1.alt1 , packet1.lat2 , packet1.lon2 , packet1.alt2 , packet1.target_system , packet1.target_component , packet1.type , packet1.mode );
    mavlink_msg_auto_route_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_auto_route_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_auto_route_request_send(MAVLINK_COMM_1 , packet1.lat1 , packet1.lon1 , packet1.alt1 , packet1.lat2 , packet1.lon2 , packet1.alt2 , packet1.target_system , packet1.target_component , packet1.type , packet1.mode );
    mavlink_msg_auto_route_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("AUTO_ROUTE_REQUEST") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST) != NULL);
#endif
}

static void mavlink_test_auto_route_request_ack(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_auto_route_request_ack_t packet_in = {
        5,72,139,206,17
    };
    mavlink_auto_route_request_ack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.type = packet_in.type;
        packet1.mode = packet_in.mode;
        packet1.progress = packet_in.progress;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_auto_route_request_ack_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_auto_route_request_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_auto_route_request_ack_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.type , packet1.mode , packet1.progress );
    mavlink_msg_auto_route_request_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_auto_route_request_ack_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.type , packet1.mode , packet1.progress );
    mavlink_msg_auto_route_request_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_auto_route_request_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_auto_route_request_ack_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.type , packet1.mode , packet1.progress );
    mavlink_msg_auto_route_request_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("AUTO_ROUTE_REQUEST_ACK") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_AUTO_ROUTE_REQUEST_ACK) != NULL);
#endif
}

static void mavlink_test_formation_command_int(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_FORMATION_COMMAND_INT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_formation_command_int_t packet_in = {
        93372036854775807ULL,93372036854776311ULL,{ 129.0, 130.0, 131.0, 132.0, 133.0, 134.0, 135.0 },963499752,963499960,381.0,20147,20251,20355,63,130
    };
    mavlink_formation_command_int_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.cmdExTime = packet_in.cmdExTime;
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.command = packet_in.command;
        packet1.frame = packet_in.frame;
        packet1.flag = packet_in.flag;
        
        mav_array_memcpy(packet1.params, packet_in.params, sizeof(float)*7);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_FORMATION_COMMAND_INT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_FORMATION_COMMAND_INT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_formation_command_int_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_formation_command_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_formation_command_int_pack(system_id, component_id, &msg , packet1.time_usec , packet1.cmdExTime , packet1.target_system , packet1.target_component , packet1.frame , packet1.flag , packet1.command , packet1.params , packet1.x , packet1.y , packet1.z );
    mavlink_msg_formation_command_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_formation_command_int_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.cmdExTime , packet1.target_system , packet1.target_component , packet1.frame , packet1.flag , packet1.command , packet1.params , packet1.x , packet1.y , packet1.z );
    mavlink_msg_formation_command_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_formation_command_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_formation_command_int_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.cmdExTime , packet1.target_system , packet1.target_component , packet1.frame , packet1.flag , packet1.command , packet1.params , packet1.x , packet1.y , packet1.z );
    mavlink_msg_formation_command_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("FORMATION_COMMAND_INT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_FORMATION_COMMAND_INT) != NULL);
#endif
}

static void mavlink_test_get_rc_channel_value_req(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_REQ >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_get_rc_channel_value_req_t packet_in = {
        5
    };
    mavlink_get_rc_channel_value_req_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.req = packet_in.req;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_REQ_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_REQ_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_get_rc_channel_value_req_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_get_rc_channel_value_req_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_get_rc_channel_value_req_pack(system_id, component_id, &msg , packet1.req );
    mavlink_msg_get_rc_channel_value_req_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_get_rc_channel_value_req_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.req );
    mavlink_msg_get_rc_channel_value_req_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_get_rc_channel_value_req_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_get_rc_channel_value_req_send(MAVLINK_COMM_1 , packet1.req );
    mavlink_msg_get_rc_channel_value_req_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GET_RC_CHANNEL_VALUE_REQ") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_REQ) != NULL);
#endif
}

static void mavlink_test_get_rc_channel_value_respond(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_get_rc_channel_value_respond_t packet_in = {
        { 17235, 17236, 17237, 17238, 17239, 17240, 17241, 17242, 17243, 17244, 17245, 17246, 17247, 17248, 17249, 17250 },{ 18899, 18900, 18901, 18902, 18903, 18904, 18905, 18906, 18907, 18908, 18909, 18910, 18911, 18912, 18913, 18914 }
    };
    mavlink_get_rc_channel_value_respond_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mav_array_memcpy(packet1.channel, packet_in.channel, sizeof(uint16_t)*16);
        mav_array_memcpy(packet1.channelExt, packet_in.channelExt, sizeof(uint16_t)*16);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_get_rc_channel_value_respond_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_get_rc_channel_value_respond_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_get_rc_channel_value_respond_pack(system_id, component_id, &msg , packet1.channel , packet1.channelExt );
    mavlink_msg_get_rc_channel_value_respond_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_get_rc_channel_value_respond_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.channel , packet1.channelExt );
    mavlink_msg_get_rc_channel_value_respond_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_get_rc_channel_value_respond_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_get_rc_channel_value_respond_send(MAVLINK_COMM_1 , packet1.channel , packet1.channelExt );
    mavlink_msg_get_rc_channel_value_respond_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GET_RC_CHANNEL_VALUE_RESPOND") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GET_RC_CHANNEL_VALUE_RESPOND) != NULL);
#endif
}

static void mavlink_test_acfly(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_wind(system_id, component_id, last_msg);
    mavlink_test_data64(system_id, component_id, last_msg);
    mavlink_test_rangefinder(system_id, component_id, last_msg);
    mavlink_test_rally_point(system_id, component_id, last_msg);
    mavlink_test_rally_fetch_point(system_id, component_id, last_msg);
    mavlink_test_compassmot_status(system_id, component_id, last_msg);
    mavlink_test_ahrs2(system_id, component_id, last_msg);
    mavlink_test_camera_status(system_id, component_id, last_msg);
    mavlink_test_camera_feedback(system_id, component_id, last_msg);
    mavlink_test_battery2(system_id, component_id, last_msg);
    mavlink_test_ahrs3(system_id, component_id, last_msg);
    mavlink_test_autopilot_version_request(system_id, component_id, last_msg);
    mavlink_test_remote_log_data_block(system_id, component_id, last_msg);
    mavlink_test_remote_log_block_status(system_id, component_id, last_msg);
    mavlink_test_led_control(system_id, component_id, last_msg);
    mavlink_test_mag_cal_progress(system_id, component_id, last_msg);
    mavlink_test_sys_status_acfly(system_id, component_id, last_msg);
    mavlink_test_acflyhtl_pwmchans_fb(system_id, component_id, last_msg);
    mavlink_test_acflyhtl_imu_register(system_id, component_id, last_msg);
    mavlink_test_acflyhtl_imu_update(system_id, component_id, last_msg);
    mavlink_test_acfly_mission_status(system_id, component_id, last_msg);
    mavlink_test_acflypossensor_info(system_id, component_id, last_msg);
    mavlink_test_follow_target_acfly(system_id, component_id, last_msg);
    mavlink_test_acfly_regeisterpossensor(system_id, component_id, last_msg);
    mavlink_test_acfly_updatepossensor(system_id, component_id, last_msg);
    mavlink_test_mission_report(system_id, component_id, last_msg);
    mavlink_test_thread_info(system_id, component_id, last_msg);
    mavlink_test_water_quality_sensor(system_id, component_id, last_msg);
    mavlink_test_component_extension43(system_id, component_id, last_msg);
    mavlink_test_battery_status_acfly(system_id, component_id, last_msg);
    mavlink_test_hydrogen_battery_info(system_id, component_id, last_msg);
    mavlink_test_camera_gimbal_status(system_id, component_id, last_msg);
    mavlink_test_uavcan_raw(system_id, component_id, last_msg);
    mavlink_test_auto_route_request(system_id, component_id, last_msg);
    mavlink_test_auto_route_request_ack(system_id, component_id, last_msg);
    mavlink_test_formation_command_int(system_id, component_id, last_msg);
    mavlink_test_get_rc_channel_value_req(system_id, component_id, last_msg);
    mavlink_test_get_rc_channel_value_respond(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // ACFLY_TESTSUITE_H
