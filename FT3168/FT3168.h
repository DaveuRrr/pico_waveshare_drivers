/*****************************************************************************
* | File      	:   FT3168.h
* | Author      :   Waveshare Team, Modified by Dave uRrr
* | Function    :   FT3168 Interface Functions
* | Info        :   Used to shield the underlying layers of each master and enhance portability
*----------------
* |	This version:   V1.1
* | Date        :   2026-02-25
* | Info        :   Removed DEV_Config.h dependency, requires user-defined pin macros
*
******************************************************************************/
#ifndef _FT3168_H_
#define _FT3168_H_

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

// PINs used in FT3168 - Include board_resources.h to override these defaults
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
#define TOUCH_INT_PIN   (4)
#endif
#ifndef TOUCH_RST_PIN
#define TOUCH_RST_PIN   (5)
#endif

// Rotation of the screen to find new X / Y Points
#define ROTATION_0      0  // Normal/Portrait
#define ROTATION_90     1  // 90° clockwise
#define ROTATION_180    2  // 180° (upside down)
#define ROTATION_270    3  // 270° clockwise (or 90° counter-clockwise)

#define FT3168_ADDR  (0x38)

typedef enum
{
    FT3168_FingerNum = 0x02,
    FT3168_XposH,
    FT3168_XposL,
    FT3168_YposH,
    FT3168_YposL,

    FT3168_MonitorMode = 0x86,
    FT3168_MonitorTime,

    FT3168_DeviceID = 0xA0,
    FT3168_IrqMode = 0xA4,
    FT3168_PowerMode,
    FT3168_FwVersion,
    FT3168_ChipID = 0xA8,

    FT3168_ProximitySensingMode = 0xB0,

    FT3168_GestureIDMode = 0xD0,
    FT3168_GestureID = 0xD3,
} FT3168_Register;

typedef enum {
    FT3168_DEVICE_ON,
    FT3168_DEVICE_OFF
} Device_State;

typedef enum {
    FT3168_POWER_ACTIVE,
    FT3168_POWER_MONITOR,
    FT3168_POWER_STANDBY,
    FT3168_POWER_HIBERNATE
} Device_Mode;

typedef enum
{
	FT3168_Point_Mode = 1,
	FT3168_Gesture_Mode,
} FT3168_Mode;

typedef enum
{
	FT3168_Gesture_None  = 0,
	FT3168_Gesture_Right = 0x21,
	FT3168_Gesture_Left  = 0x20,
	FT3168_Gesture_Down  = 0x23,
	FT3168_Gesture_Up    = 0x22,
	FT3168_Gesture_Click = 0x26,
	FT3168_Gesture_Double_Click = 0x24,
} FT3168_Gesture;

typedef struct
{
	uint16_t x_point;
	uint16_t y_point;
	FT3168_Mode mode;
} FT3168_ATTRIBUTES;

extern FT3168_ATTRIBUTES FT3168;

uint8_t FT3168_Init(uint8_t mode);
uint8_t FT3168_Get_Gesture(void);
uint8_t FT3168_I2C_Read(uint8_t reg);
FT3168_ATTRIBUTES FT3168_Get_Point(uint8_t rotation, uint16_t width, uint16_t height);

void FT3168_Reset();
void FT3168_Wake_Up();
void FT3168_Stop_Sleep();
void FT3168_Set_Mode(uint8_t mode);

#endif