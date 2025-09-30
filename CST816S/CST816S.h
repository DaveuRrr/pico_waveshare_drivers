/*****************************************************************************
* | File      	:   CST816S.h
* | Author      :   Waveshare team, Modified by Dave uRrr
* | Function    :   Hardware underlying interface
* | Info        :	Used to shield the underlying layers of each master and enhance portability
*----------------
* |	This version:   V1.1
* | Date        :   2025-09-23
* | Info        :   Removed WS_Config.h dependency, requires user-defined pin macros
*
 ******************************************************************************/

#ifndef __CST816S_H
#define __CST816S_H

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/time.h"

// Include board configuration if available
#ifdef __has_include
    #if __has_include("board_resources.h")
        #include "board_resources.h"
    #endif
#endif

// PINs used in QMI8658 - Include board_resources.h to override these defaults
#ifndef SENSOR_I2C_PORT
#define SENSOR_I2C_PORT (i2c1)
#endif
#ifndef SENSOR_SDA_PIN
#define SENSOR_SDA_PIN  (6)
#endif
#ifndef SENSOR_SCL_PIN
#define SENSOR_SCL_PIN  (7)
#endif
#ifndef TOUCH_INT_PIN
#define TOUCH_INT_PIN   (21)
#endif
#ifndef TOUCH_RST_PIN
#define TOUCH_RST_PIN   (22)
#endif

// Rotation of the screen to find new X / Y Points
#define ROTATION_0      0  // Normal/Portrait
#define ROTATION_90     1  // 90° clockwise 
#define ROTATION_180    2  // 180° (upside down)
#define ROTATION_270    3  // 270° clockwise (or 90° counter-clockwise)

#define CST816_ADDR (0x15)

typedef enum
{
    CST816_GestureID = 0x01,
    CST816_FingerNum,
    CST816_XposH,
    CST816_XposL,
    CST816_YposH,
    CST816_YposL,

    CST816_ChipID = 0xA7,
    CST816_ProjID,
    CST816_FwVersion,
    CST816_MotionMask,

    CST816_BPC0H = 0xB0,
    CST816_BPC0L,
    CST816_BPC1H,
    CST816_BPC1L,

    CST816_IrqPluseWidth = 0xED,
    CST816_NorScanPer,
    CST816_MotionSlAngle,
    CST816_LpScanRaw1H =0XF0,
    CST816_LpScanRaw1L,
    CST816_LpScanRaw2H,
    CST816_LpScanRaw2L,
    CST816_LpAutoWakeTime,
    CST816_LpScanTH,
    CST816_LpScanWin,
    CST816_LpScanFreq,
    CST816_LpScanIdac,
    CST816_AutoSleepTime,
    CST816_IrqCtl,
    CST816_AutoReset,
    CST816_LongPressTime,
    CST816_IOCtl,
    CST816_DisAutoSleep
} CST816S_Register;

/**
 * Whether the graphic is filled
 **/
typedef enum
{
    CST816S_Point_Mode = 1,
    CST816S_Gesture_Mode,
    CST816S_ALL_Mode,
} CST816S_Mode;

typedef enum
{
    CST816S_Gesture_None = 0,
    CST816S_Gesture_Up,
    CST816S_Gesture_Down,
    CST816S_Gesture_Left,
    CST816S_Gesture_Right,
    CST816S_Gesture_Click,
    CST816S_Gesture_Double_Click = 0x0b,
    CST816S_Gesture_Long_Press=0x0c,
} CST816S_Gesture;

typedef struct
{
    uint16_t x_point;
    uint16_t y_point;
    CST816S_Mode mode;
} CST816S;

extern CST816S Touch_CTS816;

uint8_t CST816S_init(uint8_t mode);
CST816S CST816S_Get_Point();
uint8_t CST816S_Get_Gesture(void);
uint8_t CST816S_I2C_Read(uint8_t reg);
void CST816S_Set_Mode(uint8_t mode);

#endif
