#pragma once

#include <AP_Logger/LogStructure.h>
#include <AC_CustomControl/AC_CustomControl_config.h>

#define LOG_DATA_FROM_CC \
    LOG_CC0_MSG, \
    LOG_CC1_MSG, \
    LOG_CC2_MSG, \
    LOG_CC3_MSG, \
    LOG_CC4_MSG

#if AP_CUSTOMCONTROL_ADAPTIVE_ENABLED

// @LoggerMessage: CC_MRAC
// @Description: Custom Control MRAC states
// @Field: Uroll: Control output for roll
// @Field: Upitch: Control output for pitch
// @Field: Uyaw: Control output for yaw
// @Field: Xmr: Model reference roll
// @Field: Xmp: Model reference pitch
// @Field: Xmy: Model reference yaw
// @Field: Dxmr: Model reference roll rate
// @Field: Dxmr: Model reference pitch rate
// @Field: Dxmr: Model reference yaw rate

struct PACKED log_CC0 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float u_roll;
    float u_pitch;
    float u_yaw;
    float xm_roll;
    float xm_pitch;
    float xm_yaw;
    float dxm_roll;
    float dxm_pitch;
    float dxm_yaw;
    float ddxm_roll;
    float ddxm_pitch;
    float ddxm_yaw;
};

struct PACKED log_CC1 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float ah_r1;
    float ah_r2;
    float ah_p1;
    float ah_p2;
    float ah_y1;
    float ah_y2;
    float roll_e;
    float pitch_e;
    float yaw_e;
};

struct PACKED log_CC2 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float dxr_roll_c;
    float dxr_pitch_c;
    float dxr_yaw_c;
    float ddxr_roll_c;
    float ddxr_pitch_c;
    float ddxr_yaw_c;
};

struct PACKED log_CC3 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float dxr_roll_a;
    float dxr_pitch_a;
    float dxr_yaw_a;
    float ddxr_roll_a;
    float ddxr_pitch_a;
    float ddxr_yaw_a;
};

struct PACKED log_CC4 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float s_roll_c;
    float s_pitch_c;
    float s_yaw_c;
    float s_roll_a;
    float s_pitch_a;
    float s_yaw_a;
};

#define LOG_STRUCTURE_FROM_CC \
    { LOG_CC0_MSG, sizeof(log_CC0), \
        "CCL0", "Qffffffffffff", "TimeUS,Ur,Up,Uy,Xmr,Xmp,Xmy,Dxmr,Dxmp,Dxmy,Ddxmr,Ddxmp,Ddxmy", "s------------", "F------------" , true}, \
    { LOG_CC1_MSG, sizeof(log_CC1), \
        "CCL1", "Qfffffffff", "TimeUS,ah_r1,ah_r2,ah_p1,ah_p2,ah_y1,ah_y2,roll_e,pitch_e,yaw_e", "s---------", "F---------", true}, \
    { LOG_CC2_MSG, sizeof(log_CC2), \
        "CCL2", "Qffffff", "TimeUS,dxr_roll,dxr_pitch,dxr_yaw,ddxr_roll,ddxr_pitch,ddxr_yaw", "s------", "F------", true}, \
    { LOG_CC3_MSG, sizeof(log_CC3), \
        "CCL3", "Qffffff", "TimeUS,dxr_roll,dxr_pitch,dxr_yaw,ddxr_roll,ddxr_pitch,ddxr_yaw", "s------", "F------", true}, \
    { LOG_CC4_MSG, sizeof(log_CC4), \
        "CCL4", "Qffffff", "TimeUS,s_roll_c,s_pitch_c,s_yaw_c,s_roll_a,s_pitch_a,s_yaw_a", "s------", "F------", true},
#else
#define LOG_STRUCTURE_FROM_CC
#endif