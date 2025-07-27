#pragma once

#include <AP_Logger/LogStructure.h>
#include <AC_CustomControl/AC_CustomControl_config.h>

#define LOG_DATA_FROM_CC \
    LOG_CC0_MSG, \
    LOG_CC1_MSG

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
    float u_roll;
    float u_pitch;
    float u_yaw;
    float xm_roll;
    float xm_pitch;
    float xm_yaw;
    float dxm_roll;
    float dxm_pitch;
    float dxm_yaw;
};

struct PACKED log_CC1 {
    LOG_PACKET_HEADER;
    float ah_r1;
    float ah_r2;
    float ah_p1;
    float ah_p2;
    float ah_y1;
    float ah_y2;
};

#define LOG_STRUCTURE_FROM_CC        \
    { LOG_CC0_MSG, sizeof(log_CC0), \
        "CCL0", "fffffffff", "Uroll,Upitch,Uyaw,Xmr,Xmp,Xmy,Dxmr,Dxmp,Dxmy", "---------", "---------" , true }, \
    { LOG_CC1_MSG, sizeof(log_CC1), \
        "CCL1", "ffffff", "ah_r1,ah_r2,ah_p1,ah_p2,ah_y1,ah_y2", "------", "------", true},
#else
#define LOG_STRUCTURE_FROM_CC
#endif