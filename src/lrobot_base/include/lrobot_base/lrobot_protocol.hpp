#ifndef LROBOT_PROTOCOL_H
#define LROBOT_PROTOCOL_H
#include <stdint.h>

/// Message IDs
#define CMD_ID ((uint32_t)0x0602)

/// Index
#define CMD_MyCmdSPS ((uint32_t)0x511C)
#define CMD_SollVelProzentSPS ((uint32_t)0x511A)
#define CMD_SollWinkelGradSPS ((uint32_t)0x511A)

/// MyCmdSps Func
#define CMD_NOP ((uint8_t)0x00)
#define CMD_OFF ((uint8_t)0x01)
#define CMD_STOP ((uint8_t)0x02)
#define CMD_FLS_DRIVE ((uint8_t)0x04)
#define CMD_SET_HOMMING ((uint8_t)0x05)
#define CMD_POS_DEAD_WINDOW ((uint8_t)0x06)
#define CMD_FLS_MODULO_SHORT ((uint8_t)0x07)
#define CMD_QUICK_STOP ((uint8_t)0x08)
#define CMD_BREAKE_SELFCHECK ((uint8_t)0x09)

///
typedef struct{
    int16_t linear; 
    int16_t angular;
}MotionCommandMessage;

typedef struct{
    uint8_t cmd;
}MyCmdSPSMessage;

typedef enum
{
    LRobotMsgUnkonwn = 0x00,
    LRobotMsgCmdSps=0x01,
    LRobotMsgMotionLinearCommand=0x02,
    LRobotMsgMotionAngularCommand,
} MsgType;

typedef struct{
    MsgType type;
    union{
        MotionCommandMessage motion_command_msg;
        MyCmdSPSMessage mycmdsps_command_msg;
    }body;
}LRobotmessage;

#endif