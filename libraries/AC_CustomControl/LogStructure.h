#pragma once

#include <AP_Logger/LogStructure.h>
#include <AC_CustomControl/AC_CustomControl_config.h>

#define LOG_DATA_FROM_CC \
    LOG_CC0_MSG, \
    LOG_CC1_MSG, \
    LOG_CC2_MSG, \
    LOG_CC3_MSG

#if AP_CUSTOMCONTROL_ADAPTIVE_ENABLED

// @LoggerMessage: CC0

struct PACKED log_CC0 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float u_roll;
    float u_pitch;
    float u_yaw;
    float dwm1;
    float dwm2;
    float dwm3;
    float err1;
    float err2;
    float err3;
    float dah1;
    float dah2;
    float dah3;
};

// @LoggerMessage: CC1

struct PACKED log_CC1 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float wr1;
    float wr2;
    float wr3;
    float dwr1;
    float dwr2;
    float dwr3;
    float w1;
    float w2;
    float w3;
    float ah1;
    float ah2;
    float ah3;
};

// @LoggerMessage: CC2

struct PACKED log_CC2 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float control_r;
    float control_p;
    float control_y;
    float adapt_r;
    float adapt_p;
    float adapt_y;
};

// @LoggerMessage: CC3

struct PACKED log_CC3 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float s_roll;
    float s_pitch;
    float s_yaw;
    float wd1;
    float wd2;
    float wd3;
    float ys1;
    float ys2;
    float ys3;
};

#define LOG_STRUCTURE_FROM_CC \
    { LOG_CC0_MSG, sizeof(log_CC0), \
        "CCL0", "Qffffffffffff", "TimeUS,U1,U2,U3,dwm1,dwm2,dwm3,e1,e2,e3,dah1,dah2,dah3", "s------------", "F------------" , true}, \
    { LOG_CC1_MSG, sizeof(log_CC1), \
        "CCL1", "Qffffffffffff", "TimeUS,wr1,wr2,wr3,dwr1,dwr2,dwr3,w1,w2,w3,ah1,ah2,ah3", "s------------", "F------------", true}, \
    { LOG_CC2_MSG, sizeof(log_CC2), \
        "CCL2", "Qffffff", "TimeUS,c_r,c_p,c_y,ad_r,ad_p,ad_y", "s------", "F------", true}, \
    { LOG_CC3_MSG, sizeof(log_CC3), \
        "CCL3", "Qfffffffff", "TimeUS,s1,s2,s3,wd1,wd2,wd3,ys1,ys2,ys3", "s---------", "F---------", true},
#else
#define LOG_STRUCTURE_FROM_CC
#endif