/*****************************************************************************
* | File      	:   ST7789V2.c
* | Author      :   Waveshare team, Modified by Dave uRrr
* | Function    :   Hardware underlying interface
* | Info        :	Used to shield the underlying layers of each master and enhance portability
*----------------
* |	This version:   V1.1
* | Date        :   2025-09-24
* | Info        :   Removed WS_Config.h dependency, requires user-defined pin macros
*
 *****************************************************************************/

#ifndef __ST7789V2_H
#define __ST7789V2_H   
    
#include <stdint.h>

#include <stdlib.h>
#include <stdio.h>

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

// PINs used in ST7789V2 - Include board_resources.h to override these defaults
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

extern uint ST7789V2_DMA_TX;
extern dma_channel_config ST7789V2_DMA_CONFIG;

#define ST7789V2_HEIGHT 280
#define ST7789V2_WIDTH 240

#define HORIZONTAL 0
#define VERTICAL   1

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
    uint8_t SCAN_DIR;
}ST7789V2_ATTRIBUTES;
extern ST7789V2_ATTRIBUTES ST7789V2;

/********************************************************************************
function:   Macro definition variable name
********************************************************************************/
void ST7789V2_Init(uint8_t scan_direction);
void ST7789V2_Clear(uint16_t color);
void ST7789V2_Display(uint16_t *image);
void ST7789V2_Display_Windows(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, uint16_t *image);
void ST7789V2_Set_Windows(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end);
void ST7789V2_Draw_Point(uint16_t x, uint16_t y, uint16_t color);

#endif
