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
// @Description: Custom Controller Control Outputs and Reference Model States
// @Field: TimeUS: Time since system startup
// @Field: U_roll: Control output for roll
// @Field: U_pitch: Control output for pitch
// @Field: U_yaw: Control output for yaw
// @Field: Xm_roll: Reference Model roll
// @Field: Xm_pitch: Reference Model pitch
// @Field: Xm_yaw: Reference Model yaw
// @Field: Dxm_roll: Reference Model roll rate
// @Field: Dxm_pitch: Reference Model pitch rate
// @Field: Dxm_yaw: Reference Model yaw rate
// @Field: Ddxm_roll: Reference Model Roll Acceleration
// @Field: Ddxm_pitch: Reference Model Pitch Acceleration
// @Field: Ddxm_yaw: Reference Model Yaw Acceleration

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

// @LoggerMessage: CC1
// @Description: Custom Controller Adaptive Mechanism Outputs and Error
// @Field: TimeUS: Time since system startup
// @Field: ah_R1: Adaptive param for acceleration compensation for roll
// @Field: ah_R2: Adaptive param for decoupling effect for roll
// @Field: ah_P1: Adaptive param for acceleration compensation for pitch
// @Field: ah_P2: Adaptive param for decoupling effect for pitch
// @Field: ah_Y1: Adaptive param for acceleration compensation for yaw
// @Field: ah_Y1: Adaptive param for acceleration compensation for yaw
// @Field: roll_e: Roll error
// @Field: pitch_e: Pitch error
// @Field: yaw_e: Yaw error
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

// @LoggerMessage: CC2
// @Description: Custom Controller Augmented Reference Velocity and Acceleration for the controller
// @Field: TimeUS: Time since system startup
// @Field: dxr_roll_controller: Augmented Reference Velocity Roll
// @Field: dxr_pitch_controller: Augmented Reference Velocity Pitch
// @Field: dxr_yaw_controller: Augmented Reference Velocity Yaw
// @Field: ddxr_roll_controller: Augmented Reference Acceleration Roll
// @Field: ddxr_pitch_controller: Augmented Reference Acceleration Pitch
// @Field: ddxr_yaw_controller: Augmented Reference Acceleration Yaw
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

// @LoggerMessage: CC3
// @Description: Custom Controller Sliding Surface
// @Field: TimeUS: Time since system startup
// @Field: s_roll_controller: Sliding surface Roll
// @Field: s_pitch_controller: Sliding surface Pitch
// @Field: s_yaw_controller: Sliding surface Yaw
struct PACKED log_CC3 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float s_roll;
    float s_pitch;
    float s_yaw;
};

#define LOG_STRUCTURE_FROM_CC \
    { LOG_CC0_MSG, sizeof(log_CC0), \
        "CCL0", "Qffffffffffff", "TimeUS,Ur,Up,Uy,Xmr,Xmp,Xmy,Dxmr,Dxmp,Dxmy,Ddxmr,Ddxmp,Ddxmy", "s------------", "F------------" , true}, \
    { LOG_CC1_MSG, sizeof(log_CC1), \
        "CCL1", "Qfffffffff", "TimeUS,ah_r1,ah_r2,ah_p1,ah_p2,ah_y1,ah_y2,roll_e,pitch_e,yaw_e", "s---------", "F---------", true}, \
    { LOG_CC2_MSG, sizeof(log_CC2), \
        "CCL2", "Qffffff", "TimeUS,dxr_roll,dxr_pitch,dxr_yaw,ddxr_roll,ddxr_pitch,ddxr_yaw", "s------", "F------", true}, \
    { LOG_CC3_MSG, sizeof(log_CC3), \
        "CCL3", "Qfff", "TimeUS,s_roll,s_pitch,s_yaw", "s---", "F---", true},
#else
#define LOG_STRUCTURE_FROM_CC
#endif