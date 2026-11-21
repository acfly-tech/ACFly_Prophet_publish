#pragma once

/*
 * K9 application compatibility aliases for symbols that were removed or
 * renamed by upstream mavlink.
 *
 * This is the FC-side equivalent of GCSPRO/src/comm/AcflyMavlinkCompat.h and
 * is the ONLY file that should need updating when upstream mavlink renames or
 * removes symbols. It is injected at the end of the generated
 * acfly/mavlink.h by GCSPRO/script/mavlink/regen_acfly.py --target-fc and is
 * NOT overwritten by the generator.
 */

#include "px4_custom_mode.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include <stdint.h>
#include <stdbool.h>

/* MAV_FRAME aliases - old names removed from standard */
#define MAV_FRAME_BODY_FLU 13
#define MAV_FRAME_VISION_NED 16
#define MAV_FRAME_VISION_ENU 17

/* CAMERA_CAP_FLAGS aliases - ACFly private bits kept as ACFLY_CAMERA_CAP_* */
#define CAMERA_CAP_FLAGS_HAS_LASER ACFLY_CAMERA_CAP_HAS_LASER
#define CAMERA_CAP_FLAGS_HAS_THERMAL ACFLY_CAMERA_CAP_HAS_THERMAL
#define CAMERA_CAP_FLAGS_HAS_DOUBLE_VIDEO ACFLY_CAMERA_CAP_HAS_DOUBLE_VIDEO
#define CAMERA_CAP_FLAGS_HAS_GIMBAL_PITCH ACFLY_CAMERA_CAP_HAS_GIMBAL_PITCH
#define CAMERA_CAP_FLAGS_HAS_GIMBAL_YAW ACFLY_CAMERA_CAP_HAS_GIMBAL_YAW
#define CAMERA_CAP_FLAGS_HAS_GIMBAL_UP ACFLY_CAMERA_CAP_HAS_GIMBAL_UP
#define CAMERA_CAP_FLAGS_HAS_GIMBAL_DOWN ACFLY_CAMERA_CAP_HAS_GIMBAL_DOWN
#define CAMERA_CAP_FLAGS_HAS_GIMBAL_CENTER ACFLY_CAMERA_CAP_HAS_GIMBAL_CENTER
#define CAMERA_CAP_FLAGS_HAS_LED ACFLY_CAMERA_CAP_HAS_LED
#define CAMERA_CAP_FLAGS_HAS_FLIP ACFLY_CAMERA_CAP_HAS_FLIP
#define CAMERA_CAP_FLAGS_HAS_SWITCH ACFLY_CAMERA_CAP_HAS_SWITCH
#define CAMERA_CAP_FLAGS_HAS_CALIBRATION ACFLY_CAMERA_CAP_HAS_CALIBRATION
#define CAMERA_CAP_FLAGS_HAS_ACK ACFLY_CAMERA_CAP_HAS_ACK
#define CAMERA_CAP_FLAGS_SHOW_UI_BUTTON ACFLY_CAMERA_CAP_SHOW_UI_BUTTON
#define CAMERA_CAP_FLAGS_GIMBAL_CTRL_SEND_WHEN_CHANGED ACFLY_CAMERA_CAP_GIMBAL_CTRL_SEND_WHEN_CHANGED
#define CAMERA_CAP_FLAGS_TARGET_DETECTION ACFLY_CAMERA_CAP_TARGET_DETECTION
#define CAMERA_CAP_FLAGS_HAS_OSD ACFLY_CAMERA_CAP_HAS_OSD
#define CAMERA_CAP_FLAGS_SHOW_CAMERA_AREA ACFLY_CAMERA_CAP_SHOW_CAMERA_AREA
#define CAMERA_CAP_FLAGS_TARGET_SELECTION_RECTANGLE ACFLY_CAMERA_CAP_TARGET_SELECTION_RECTANGLE

/* Legacy MAV_TYPE aliases - map to current upstream names (same values) */
#define MAV_TYPE_VTOL_DUOROTOR MAV_TYPE_VTOL_TAILSITTER_DUOROTOR
#define MAV_TYPE_VTOL_QUADROTOR MAV_TYPE_VTOL_TAILSITTER_QUADROTOR
#define MAV_TYPE_VTOL_RESERVED2 MAV_TYPE_VTOL_FIXEDROTOR
#define MAV_TYPE_VTOL_RESERVED3 MAV_TYPE_VTOL_TAILSITTER
#define MAV_TYPE_VTOL_RESERVED4 MAV_TYPE_VTOL_TILTWING

/* Reposition flags - legacy ACFly bit 2 semantics */
#define MAV_DO_REPOSITION_FLAGS_ALTITUDE_MODE ACFLY_REPOSITION_FLAGS_ALTITUDE_MODE

/* COMP_METADATA_TYPE: upstream renamed VERSION(0) to GENERAL(0) */
#define COMP_METADATA_TYPE_VERSION COMP_METADATA_TYPE_GENERAL

/* Command ack alias */
#define mavlink_msg_command_ack_pack_chan_full mavlink_msg_command_ack_pack_chan

/* MANUAL_CONTROL extendedAxis - old fields renamed to aux */
#define extendedAxis1 aux1
#define extendedAxis2 aux2
#define extendedAxis3 aux3
#define extendedAxis4 aux4

/* Mission current flags - old ACFly extension */
#define IS_IN_MISSION_FLIGHT 1
#define IS_RESUME_WAYPOINT_EXIST 2
#define MAV_MISSION_CURRENT_IS_IN_MISSION_FLIGHT 1
#define MAV_MISSION_CURRENT_IS_RESUME_WAYPOINT_EXIST 2

/* Legacy ACFly component IDs (formerly the ACFLY_COMPONENT_ID_ALIASES enum) */
#define MAV_COMP_ID_ENGINE_XUANFU_7KW 25
#define MAV_COMP_ID_ENGINE_XUANFU_16KW 26
#define MAV_COMP_ID_ENGINE_ZONGSHEN 27
#define MAV_COMP_ID_CAMERA_SIYI_ZR10 100
#define MAV_COMP_ID_CAMERA_INYYO 101
#define MAV_COMP_ID_CAMERA_RYEOP 102
#define MAV_COMP_ID_CAMERA_AILF 103
#define MAV_COMP_ID_CAMERA_VIEWLING 104
#define MAV_COMP_ID_CAMERA_SKYDROID_2AXIS 106
#define MAV_COMP_ID_CAMERA_SKYDROID_3AXIS 107
#define MAV_COMP_ID_CAMERA_NEXTVISION 108
#define MAV_COMP_ID_CAMERA_AI3C70B 109
#define MAV_COMP_ID_CAMERA_SKYDROID_TCP 110
#define MAV_COMP_ID_CAMERA_SKYDROID_UDP 111
#define MAV_COMP_ID_CAMERA_TOPOTEK 112
#define MAV_COMP_ID_CAMERA_AI3C130A 113
#define MAV_COMP_ID_CAMERA_TS05LK 114
#define MAV_COMP_ID_CAMERA_AA32A2401 115
#define MAV_COMP_ID_CAMERA_AA342401 116
#define MAV_COMP_ID_CAMERA_AGA01 117
#define MAV_COMP_ID_CAMERA_LUXNOBA 118
#define MAV_COMP_ID_CAMERA_ZHUOYI 118
#define MAV_COMP_ID_CAMERA_HEQK8T 119
#define MAV_COMP_ID_CAMERA_SUMBOY 120
#define MAV_COMP_ID_CAMERA_HEQK40T 121
#define MAV_COMP_ID_HYDROGEN_BATTERY 139
#define MAV_COMP_ID_CAMERA_SkyDroid_2_Axis MAV_COMP_ID_CAMERA_SKYDROID_2AXIS
#define MAV_COMP_ID_CAMERA_SkyDroid_3_Axis MAV_COMP_ID_CAMERA_SKYDROID_3AXIS
#define MAV_COMP_ID_CAMERA_SkyDroid_TCP MAV_COMP_ID_CAMERA_SKYDROID_TCP
#define MAV_COMP_ID_CAMERA_SkyDroid_UDP MAV_COMP_ID_CAMERA_SKYDROID_UDP

/* FTP opcode aliases */
#define opCodeRspAck MAV_FTP_OPCODE_ACK
#define opCodeRspNak MAV_FTP_OPCODE_NAK
#define opCodeBurstReadFile MAV_FTP_OPCODE_BURSTREADFILE
#define opCodeOpenFileRO MAV_FTP_OPCODE_OPENFILERO
#define opCodeOpenFileWO MAV_FTP_OPCODE_OPENFILEWO
#define opCodeCreateFile MAV_FTP_OPCODE_CREATEFILE
#define opCodeWriteFile MAV_FTP_OPCODE_WRITEFILE
#define opCodeReadFile MAV_FTP_OPCODE_READFILE
#define opCodeListDirectory MAV_FTP_OPCODE_LISTDIRECTORY
#define opCodeCreateDirectory MAV_FTP_OPCODE_CREATEDIRECTORY
#define opCodeTruncateFile MAV_FTP_OPCODE_TRUNCATEFILE
#define opCodeTerminateSession MAV_FTP_OPCODE_TERMINATESESSION
#define opCodeResetSessions MAV_FTP_OPCODE_RESETSESSION
#define opCodeRename MAV_FTP_OPCODE_RENAME
#define opCodeCalcFileCRC32 MAV_FTP_OPCODE_CALCFILECRC
#define opCodeRemoveFile MAV_FTP_OPCODE_REMOVEFILE
#define opCodeErrNone MAV_FTP_ERR_NONE
#define opCodeErrFail MAV_FTP_ERR_FAIL
#define opCodeErrInvalidSession MAV_FTP_ERR_INVALIDSESSION
#define opCodeErrNoSessionsAvailable MAV_FTP_ERR_NOSESSIONSAVAILABLE
#define opCodeErrEOF MAV_FTP_ERR_EOF
#define opCodeErrUnknownCommand MAV_FTP_ERR_UNKNOWNCOMMAND
#define opCodeErrFileNotFound MAV_FTP_ERR_FILENOTFOUND
typedef uint8_t OpErrorCode;

/* FENCE_TYPE compatibility */
#define FENCE_TYPE_INCLUSION 1
#define FENCE_TYPE_EXCLUSION 2
#define FENCE_TYPE_ALL 3

/* Note: camera_gimbal_status (272), formation_command_int (601), uavcan_raw
   (312), auto_route (414/415), rc channel value (21168/21169) and
   hydrogen_battery_info (227) are generated from acfly.xml with frozen CRCs.
   Do not re-declare their structs or pack functions here. */

#ifdef __cplusplus
static inline uint16_t mavlink_msg_mission_ack_pack_chan(
    uint8_t system_id, uint8_t component_id, uint8_t chan,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component,
    uint8_t type, uint8_t mission_type)
{
    return mavlink_msg_mission_ack_pack_chan(
        system_id, component_id, chan, msg,
        target_system, target_component, type, mission_type, 0);
}

static inline uint16_t mavlink_msg_timesync_pack_chan(
    uint8_t system_id, uint8_t component_id, uint8_t chan,
    mavlink_message_t* msg,
    int64_t tc1, int64_t ts1)
{
    return mavlink_msg_timesync_pack_chan(
        system_id, component_id, chan, msg, tc1, ts1, 0, 0);
}
#endif

#ifdef __cplusplus
extern "C" {
#endif

void mavlink_init_chan( uint8_t chan );
bool mavlink_lock_chan( uint8_t chan, double TIMEOUT );
void mavlink_unlock_chan( uint8_t chan );

int8_t reg_custom_mavlink_chan();

#ifdef __cplusplus
}
#endif
