/*****************************************************************************
* | File      	:   GC9A01A.h
* | Author      :   Waveshare team, Modified by Dave uRrr
* | Function    :   Hardware underlying interface
* | Info        :   Used to shield the underlying layers of each master and enhance portability
*----------------
* |	This version:   V1.1
* | Date        :   2025-09-23
* | Info        :   Removed WS_Config.h dependency, requires user-defined pin macros
*
******************************************************************************/

#ifndef __GC9A01A_H
#define __GC9A01A_H	

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>


#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "pico/time.h"

// Include board configuration if available
#ifdef __has_include
    #if __has_include("board_resources.h")
        #include "board_resources.h"
    #endif
#endif


// PINs used in GC9A01A - Include board_resources.h to override these defaults
#ifndef SCREEN_SPI_PORT
#define SCREEN_SPI_PORT    (spi1)
#endif
#ifndef SCREEN_DC_PIN
#define SCREEN_DC_PIN      (8)
#endif
#ifndef SCREEN_CS_PIN
#define SCREEN_CS_PIN      (9)
#endif
#ifndef SCREEN_CLK_PIN
#define SCREEN_CLK_PIN     (10)
#endif
#ifndef SCREEN_MOSI_PIN
#define SCREEN_MOSI_PIN    (11)
#endif
#ifndef SCREEN_MISO_PIN
#define SCREEN_MISO_PIN    (12)
#endif
#ifndef SCREEN_RST_PIN
#define SCREEN_RST_PIN     (13)
#endif
#ifndef SCREEN_BL_PIN
#define SCREEN_BL_PIN      (25)
#endif

extern uint GC9A01A_DMA_TX;
extern dma_channel_config GC9A01A_DMA_CONFIG;

// Display Settings
#define GC9A01A_HEIGHT  240
#define GC9A01A_WIDTH   240

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
}GC9A01A_ATTRIBUTES;
extern GC9A01A_ATTRIBUTES GC9A01A;

/********************************************************************************
function:	Macro definition variable name
********************************************************************************/
void GC9A01A_Init(uint8_t scan_direction);
void GC9A01A_Clear(uint16_t color);
void GC9A01A_Display(uint16_t *image);
void GC9A01A_Display_Windows(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, uint16_t *image);
void GC9A01A_Set_Windows(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end);
void GC9A01A_Draw_Point(uint16_t x, uint16_t y, uint16_t color);

#endif