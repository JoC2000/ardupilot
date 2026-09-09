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
    float u_r;
    float u_p;
    float u_y;
    float pid_r;
    float pid_p;
    float pid_y;
    float adapt_r;
    float adapt_p;
    float adapt_y;
    float err_r;
    float err_p;
    float err_y;
};

// @LoggerMessage: CC1

struct PACKED log_CC1 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float wr_r;
    float wr_p;
    float wr_y;
    float dwr_r;
    float dwr_p;
    float dwr_y;
};

// @LoggerMessage: CC2

struct PACKED log_CC2 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float s_r;
    float s_p;
    float s_y;
    float ah_r;
    float ah_p;
    float ah_y;
    float dah_r;
    float dah_p;
    float dah_y;
};

// @LoggerMessage: CC3

struct PACKED log_CC3 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float dh_r;
    float dh_p;
    float dh_y;
    float ddh_r;
    float ddh_p;
    float ddh_y;
};

struct PACKED log_CC4 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float bh_r;
    float bh_p;
    float bh_y;
    float dbh_r;
    float dbh_p;
    float dbh_y;
};

#define LOG_STRUCTURE_FROM_CC \
    { LOG_CC0_MSG, sizeof(log_CC0), \
        "CCL0", "Qffffffffffff", "TimeUS,U_r,U_p,U_y,pd_r,pd_p,pd_y,ad_r,ad_p,ad_y,e_r,e_p,e_y", "s------------", "F------------" , true}, \
    { LOG_CC1_MSG, sizeof(log_CC1), \
        "CCL1", "Qffffff", "TimeUS,wr_r,wr_p,wr_y,dwr_r,dwr_p,dwr_y", "s------", "F------", true}, \
    { LOG_CC2_MSG, sizeof(log_CC2), \
        "CCL2", "Qfffffffff", "TimeUS,s_r,s_p,s_y,ah_r,ah_p,ah_y,dah_r,dah_p,dah_y", "s---------", "F---------", true}, \
    { LOG_CC3_MSG, sizeof(log_CC3), \
        "CCL3", "Qffffff", "TimeUS,dh_r,dh_p,dh_y,ddh_r,ddh_p,ddh_y", "s------", "F------", true}, \
    { LOG_CC4_MSG, sizeof(log_CC4), \
        "CCL4", "Qffffff", "TimeUS,bh_r,bh_p,bh_y,dbh_r,dbh_p,dbh_y", "s------", "F------", true}, 
#else
#define LOG_STRUCTURE_FROM_CC
#endif