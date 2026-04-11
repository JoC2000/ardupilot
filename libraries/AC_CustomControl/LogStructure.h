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

// @LoggerMessage: CC0

struct PACKED log_CC0 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float u_roll;
    float u_pitch;
    float u_yaw;
    float control_r;
    float control_p;
    float control_y;
    float adapt_r;
    float adapt_p;
    float adapt_y;
    float err1;
    float err2;
    float err3;
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
    float wm1;
    float wm2;
    float wm3;
    float dwm1;
    float dwm2;
    float dwm3;
};

// @LoggerMessage: CC2

struct PACKED log_CC2 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float w1;
    float w2;
    float w3;
    float wd1;
    float wd2;
    float wd3;
    float s_roll;
    float s_pitch;
    float s_yaw;
    float ys1;
    float ys2;
    float ys3;
};

// @LoggerMessage: CC3

struct PACKED log_CC3 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float ah1;
    float ah2;
    float ah3;
    float dah1;
    float dah2;
    float dah3;
    float dh1;
    float dh2;
    float dh3;
    float ddh1;
    float ddh2;
    float ddh3;
};

struct PACKED log_CC4 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float bh1;
    float bh2;
    float bh3;
    float dbh1;
    float dbh2;
    float dbh3;
};

#define LOG_STRUCTURE_FROM_CC \
    { LOG_CC0_MSG, sizeof(log_CC0), \
        "CCL0", "Qffffffffffff", "TimeUS,U1,U2,U3,c_r,c_p,c_y,ad_r,ad_p,ad_y,e1,e2,e3", "s------------", "F------------" , true}, \
    { LOG_CC1_MSG, sizeof(log_CC1), \
        "CCL1", "Qffffffffffff", "TimeUS,wr1,wr2,wr3,dwr1,dwr2,dwr3,wm1,wm2,wm3,dwm1,dwm2,dwm3", "s------------", "F------------", true}, \
    { LOG_CC2_MSG, sizeof(log_CC2), \
        "CCL2", "Qffffffffffff", "TimeUS,w1,w2,w3,wd1,wd2,wd3,s1,s2,s3,ys1,ys2,ys3", "s------------", "F------------", true}, \
    { LOG_CC3_MSG, sizeof(log_CC3), \
        "CCL3", "Qffffffffffff", "TimeUS,ah1,ah2,ah3,dah1,dah2,dah3,dh1,dh2,dh3,ddh1,ddh2,ddh3", "s------------", "F------------", true}, \
    { LOG_CC4_MSG, sizeof(log_CC4), \
        "CCL4", "Qffffff", "TimeUS,bh1,bh2,bh3,dbh1,dbh2,dbh3", "s------", "F------", true}, 
#else
#define LOG_STRUCTURE_FROM_CC
#endif