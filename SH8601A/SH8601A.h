/*****************************************************************************
* | File      	:   SH8601A.h
* | Author      :   Waveshare Team, Modified by Dave uRrr
* | Function    :   AMOLED Interface Functions
* | Info        :   Used to shield the underlying layers of each master and enhance portability
*----------------
* |	This version:   V1.1
* | Date        :   2026-02-25
* | Info        :   Removed DEV_Config.h dependency, added rotation support and missing features
*
******************************************************************************/
#ifndef _SH8601A_H_
#define _SH8601A_H_

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "pico/time.h"
#include "QSPI_PIO.h"

// Include board configuration if available
#ifdef __has_include
    #if __has_include("board_resources.h")
        #include "board_resources.h"
    #endif
#endif

// Display Settings
#define SH8601A_WIDTH 368
#define SH8601A_HEIGHT 448

#define ROTATION_0      0  // Normal/Portrait
#define ROTATION_90     1  // 90° clockwise
#define ROTATION_180    2  // 180° (upside down)
#define ROTATION_270    3  // 270° clockwise (or 90° counter-clockwise)

#define WHITE         0xFFFF
#define BLACK		  0x0000
#define BLUE 		  0x001F
#define BRED 	      0XF81F
#define GRED 		  0XFFE0
#define GBLUE		  0X07FF
#define RED  		  0xF800
#define MAGENTA		  0xF81F
#define GREEN		  0x07E0
#define CYAN 		  0x7FFF
#define YELLOW		  0xFFE0
#define BROWN		  0XBC40
#define BRRED		  0XFC07
#define GRAY 	      0X8430
#define DARKBLUE	  0X01CF
#define LIGHTBLUE	  0X7D7C
#define GRAYBLUE      0X5458
#define LIGHTGREEN    0X841F
#define LGRAY 		  0XC618
#define LGRAYBLUE     0XA651
#define LBBLUE        0X2B12


typedef struct{
    uint16_t WIDTH;
    uint16_t HEIGHT;
    uint8_t ROTATION;
} SH8601A_ATTRIBUTES;

extern SH8601A_ATTRIBUTES SH8601A;

/********************************************************************************
function:	Macro definition variable name
********************************************************************************/
void SH8601A_Init(uint8_t rotation);
void SH8601A_Set_Brightness(uint8_t brightness);
void SH8601A_Set_Windows(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end);
void SH8601A_Clear(uint16_t color);
void SH8601A_Display(uint16_t *image);
void SH8601A_Display_Windows(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, uint16_t *image);
void SH8601A_Draw_Point(uint16_t x, uint16_t y, uint16_t color);

void SH8601A_Sleep(void);
void SH8601A_Awake(void);

#endif // !_SH8601A_H_