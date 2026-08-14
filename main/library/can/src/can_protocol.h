#pragma once

#include <stdint.h>

// =====================
// CAN ID
// =====================

// System
constexpr uint16_t CAN_ID_SYSTEM_STOP  = 0x001;
constexpr uint16_t CAN_ID_MODE_SETTING = 0x002;

// Drive
constexpr uint16_t CAN_ID_DRIVE_TARGET   = 0x100;
constexpr uint16_t CAN_ID_D_MOTOR1_PARAM = 0x101;
constexpr uint16_t CAN_ID_D_MOTOR2_PARAM = 0x102;
constexpr uint16_t CAN_ID_D_MOTOR3_PARAM = 0x103;
constexpr uint16_t CAN_ID_D_MOTOR4_PARAM = 0x104;

// Get
constexpr uint16_t CAN_ID_GET_TARGET       = 0x200;
constexpr uint16_t CAN_ID_GET_MOTOR1_PARAM = 0x201;
constexpr uint16_t CAN_ID_GET_MOTOR2_PARAM = 0x202;

constexpr uint16_t CAN_ID_GET_AIR_TARGET = 0x300;

// Shoot
constexpr uint16_t CAN_ID_SHOOT_TARGET       = 0x400;
constexpr uint16_t CAN_ID_SHOOT_MOTOR1_PARAM = 0x401;
constexpr uint16_t CAN_ID_SHOOT_MOTOR2_PARAM = 0x402;

// Lift
constexpr uint16_t CAN_ID_LIFT_TARGET       = 0x500;
constexpr uint16_t CAN_ID_LIFT_MOTOR1_PARAM = 0x501;

// Parameter back
constexpr uint16_t CAN_ID_DRIVE_PARAM_BACK_PID   = 0x601;
constexpr uint16_t CAN_ID_GET_PARAM_BACK_PID     = 0x602;
constexpr uint16_t CAN_ID_SHOOT_PARAM_BACK_PID   = 0x604;
constexpr uint16_t CAN_ID_LIFT_PARAM_BACK_PID    = 0x605;

constexpr uint16_t CAN_ID_GET_PARAM_BACK_TARGET  = 0x612;
constexpr uint16_t CAN_ID_LIFT_PARAM_BACK_TARGET = 0x615;


// =====================
// MODE_SETTING
// =====================

constexpr int16_t CAN_MODE_NORMAL = 0x001;
constexpr int16_t CAN_MODE_TEST   = 0x002;


// =====================
// PID parameter select
// =====================

constexpr int16_t CAN_GAIN_P = 0x001;
constexpr int16_t CAN_GAIN_I = 0x002;
constexpr int16_t CAN_GAIN_D = 0x003;