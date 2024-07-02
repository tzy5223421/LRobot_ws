/*
 * lrobot_message.h
 *
 * Created on: Jun 10, 2024 11:47
 * Description:
 *  all values are using SI units (e.g. meter/second/radian)
 *
 * Copyright (c) 2024 Tzy
 */
#ifndef LROBOT_MESSAGE_H
#define LROBOT_MESSAGE_H
#ifdef __cplusplus
extern "C"
{
#endif
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
    typedef struct{
        uint8_t func;
        uint8_t index_low_byte;
        uint8_t index_high_byte;
        uint8_t subIndex;
        uint8_t cmd;
        uint8_t reserve_1;
        uint8_t reserve_2;
        uint8_t reserve_3;        
    }SpsCMDFrame;
    
    typedef struct{
        uint8_t func;
        uint8_t index_low_byte;
        uint8_t index_high_byte;
        uint8_t subIndex;
        uint8_t linear_velocity_low_byte;
        uint8_t linear_velocity_high_byte;
        uint8_t reserve_1;
        uint8_t reserve_2;
    }MotionCommandLinearFrame;

     typedef struct{
        uint8_t func;
        uint8_t index_low_byte;
        uint8_t index_high_byte;
        uint8_t subIndex;
        uint8_t angular_velocity_low_byte;
        uint8_t angular_velocity_high_byte;
        uint8_t reserve_1;
        uint8_t reserve_2;
    }MotionCommandAngularFrame;

#ifdef __cplusplus
}
#endif
#endif /* LROBOT_MESSAGE_H*/
