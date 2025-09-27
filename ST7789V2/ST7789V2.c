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

#include "ST7789V2.h"

#include <stdlib.h> //itoa()
#include <stdio.h>

ST7789V2_ATTRIBUTES ST7789V2;

uint ST7789V2_SLICE_NUM;
uint ST7789V2_DMA_TX;
dma_channel_config ST7789V2_DMA_CONFIG;

/********************************************************************************
 * @brief           Initializes GPIO pin and sets direction
 * @param pin       GPIO pin number to initialize
 * @param direction GPIO direction (GPIO_IN or GPIO_OUT)
********************************************************************************/
static void ST7789V2_GPIO(uint16_t pin, uint16_t direction)
{
    gpio_init(pin);
    gpio_set_dir(pin, direction);
}

/********************************************************************************
 * @brief           Sets the PWM to ST7789V2 display
 * @param value		Set PWM to value 0-100
********************************************************************************/
void ST7789V2_SET_PWM(uint8_t value)
{
    if (value < 0 || value > 100) printf("WS_SET_PWM Error \r\n");
    else pwm_set_chan_level(ST7789V2_SLICE_NUM, PWM_CHAN_B, value);
}

/********************************************************************************
 * @brief           Performs hardware reset sequence for ST7789V2 display
********************************************************************************/
static void ST7789V2_Reset(void)
{
    gpio_put(SCREEN_RST_PIN, 1);
    sleep_ms(100);
    gpio_put(SCREEN_RST_PIN, 0);
    sleep_ms(100);
    gpio_put(SCREEN_RST_PIN, 1);
    sleep_ms(100);
}

/********************************************************************************
 * @brief           Sends a register command to ST7789V2 display
 * @param command   8-bit command byte to send
********************************************************************************/
static void ST7789V2_Send_Command(uint8_t command)
{
    gpio_put(SCREEN_DC_PIN, 0);
    gpio_put(SCREEN_CS_PIN, 0);
    spi_write_blocking(SCREEN_SPI_PORT, command, 1);
    gpio_put(SCREEN_CS_PIN, 1);
}

/********************************************************************************
 * @brief           Sends 8-bit data to ST7789V2 display
 * @param data      8-bit data byte to send
********************************************************************************/
static void ST7789V2_Send_Data_8Bit(uint8_t data)
{
    gpio_put(SCREEN_DC_PIN, 1);
    gpio_put(SCREEN_CS_PIN, 0);
    spi_write_blocking(SCREEN_SPI_PORT, data, 1);
    gpio_put(SCREEN_CS_PIN, 1);
}

/********************************************************************************
 * @brief           Sends 16-bit data to ST7789V2 display (big-endian format)
 * @param Data      16-bit data value to send
********************************************************************************/
static void ST7789V2_Send_Data_16Bit(uint16_t data)
{
    gpio_put(SCREEN_DC_PIN, 1);
    gpio_put(SCREEN_CS_PIN, 0);
    spi_write_blocking(SCREEN_SPI_PORT, data >> 8 & 0xFF, 1);
    spi_write_blocking(SCREEN_SPI_PORT, data & 0xFF, 1);
    gpio_put(SCREEN_CS_PIN, 1);
}

/********************************************************************************
 * @brief           Initializes all ST7789V2 display registers with default values
********************************************************************************/
static void ST7789V2_Init_Reg(void)
{
    ST7789V2_Send_Command(0x3A);
    ST7789V2_Send_Data_8Bit(0x05);

    ST7789V2_Send_Command(0xB2);
    ST7789V2_Send_Data_8Bit(0x0B);
    ST7789V2_Send_Data_8Bit(0x0B);
    ST7789V2_Send_Data_8Bit(0x00);
    ST7789V2_Send_Data_8Bit(0x33);
    ST7789V2_Send_Data_8Bit(0x35);

    ST7789V2_Send_Command(0xB7);
    ST7789V2_Send_Data_8Bit(0x11);

    ST7789V2_Send_Command(0xBB);
    ST7789V2_Send_Data_8Bit(0x35);

    ST7789V2_Send_Command(0xC0);
    ST7789V2_Send_Data_8Bit(0x2C);

    ST7789V2_Send_Command(0xC2);
    ST7789V2_Send_Data_8Bit(0x01);

    ST7789V2_Send_Command(0xC3);
    ST7789V2_Send_Data_8Bit(0x0D);

    ST7789V2_Send_Command(0xC4);
    ST7789V2_Send_Data_8Bit(0x20);

    ST7789V2_Send_Command(0xC6);
    ST7789V2_Send_Data_8Bit(0x13);

    ST7789V2_Send_Command(0xD0);
    ST7789V2_Send_Data_8Bit(0xA4);
    ST7789V2_Send_Data_8Bit(0xA1);

    ST7789V2_Send_Command(0xD6);
    ST7789V2_Send_Data_8Bit(0xA1);

    ST7789V2_Send_Command(0xE0);
    ST7789V2_Send_Data_8Bit(0xF0);
    ST7789V2_Send_Data_8Bit(0x06);
    ST7789V2_Send_Data_8Bit(0x0B);
    ST7789V2_Send_Data_8Bit(0x0A);
    ST7789V2_Send_Data_8Bit(0x09);
    ST7789V2_Send_Data_8Bit(0x26);
    ST7789V2_Send_Data_8Bit(0x29);
    ST7789V2_Send_Data_8Bit(0x33);
    ST7789V2_Send_Data_8Bit(0x41);
    ST7789V2_Send_Data_8Bit(0x18);
    ST7789V2_Send_Data_8Bit(0x16);
    ST7789V2_Send_Data_8Bit(0x15);
    ST7789V2_Send_Data_8Bit(0x29);
    ST7789V2_Send_Data_8Bit(0x2D);

    ST7789V2_Send_Command(0xE1);
    ST7789V2_Send_Data_8Bit(0xF0);
    ST7789V2_Send_Data_8Bit(0x04);
    ST7789V2_Send_Data_8Bit(0x08);
    ST7789V2_Send_Data_8Bit(0x08);
    ST7789V2_Send_Data_8Bit(0x07);
    ST7789V2_Send_Data_8Bit(0x03);
    ST7789V2_Send_Data_8Bit(0x28);
    ST7789V2_Send_Data_8Bit(0x32);
    ST7789V2_Send_Data_8Bit(0x40);
    ST7789V2_Send_Data_8Bit(0x3B);
    ST7789V2_Send_Data_8Bit(0x19);
    ST7789V2_Send_Data_8Bit(0x18);
    ST7789V2_Send_Data_8Bit(0x2A);
    ST7789V2_Send_Data_8Bit(0x2E);

    ST7789V2_Send_Command(0xE4);
    ST7789V2_Send_Data_8Bit(0x25);
    ST7789V2_Send_Data_8Bit(0x00);
    ST7789V2_Send_Data_8Bit(0x00);

    ST7789V2_Send_Command(0x21);

    ST7789V2_Send_Command(0x11);
    sleep_ms(120);
    ST7789V2_Send_Command(0x29);

}

/********************************************************************************
 * @brief           Sets display attributes and scan direction
 * @param Scan_dir  Display orientation (HORIZONTAL or VERTICAL)
********************************************************************************/
static void ST7789V2_Set_Attributes(uint8_t scan_direction)
{
    // Get the screen scan direction
    ST7789V2.SCAN_DIR = scan_direction;
    uint8_t MemoryAccessReg = 0x00;

    // Get GRAM and LCD width and height
    if (scan_direction == HORIZONTAL) 
    {
        ST7789V2.HEIGHT = ST7789V2_HEIGHT;
        ST7789V2.WIDTH = ST7789V2_WIDTH;
        MemoryAccessReg = 0XA0;
    }
    else 
    {
        ST7789V2.HEIGHT = ST7789V2_HEIGHT;
        ST7789V2.WIDTH = ST7789V2_WIDTH;      
        MemoryAccessReg = 0X00;
    }

    // Set the read / write scan direction of the frame memory
    ST7789V2_Send_Command(0x36); // MX, MY, RGB mode
    ST7789V2_Send_Data_8Bit(MemoryAccessReg); // 0x08 set RGB
}

/********************************************************************************
 * @brief           Initializes ST7789V2 display controller
 * @param scan_direction    Display orientation (HORIZONTAL or VERTICAL)
********************************************************************************/
void ST7789V2_Init(uint8_t scan_direction)
{
    // GPIO Config
    ST7789V2_GPIO(SCREEN_RST_PIN, 1);
    ST7789V2_GPIO(SCREEN_DC_PIN, 1);
    ST7789V2_GPIO(SCREEN_CS_PIN, 1);
    ST7789V2_GPIO(SCREEN_BL_PIN, 1);

    gpio_put(SCREEN_CS_PIN, 1);
    gpio_put(SCREEN_DC_PIN, 0);
    // gpio_put(SCREEN_BL_PIN, 1);

    // PWM Configuration
    gpio_set_function(SCREEN_BL_PIN, GPIO_FUNC_PWM);
    ST7789V2_SLICE_NUM = pwm_gpio_to_slice_num(SCREEN_BL_PIN);
    pwm_set_wrap(ST7789V2_SLICE_NUM, 100);
    pwm_set_chan_level(ST7789V2_SLICE_NUM, PWM_CHAN_B, 0);
    pwm_set_clkdiv(ST7789V2_SLICE_NUM, 50);
    pwm_set_enabled(ST7789V2_SLICE_NUM, true);

    // SPI Configuration
    spi_init(SCREEN_SPI_PORT, 270000 * 1000);
    gpio_set_function(SCREEN_CLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SCREEN_MOSI_PIN, GPIO_FUNC_SPI);

    // DMA Configuration
    ST7789V2_DMA_TX = dma_claim_unused_channel(true);
    ST7789V2_DMA_CONFIG = dma_channel_get_default_config(ST7789V2_DMA_TX);
    channel_config_set_transfer_data_size(&ST7789V2_DMA_CONFIG, DMA_SIZE_8); 
    channel_config_set_dreq(&ST7789V2_DMA_CONFIG, spi_get_dreq(SCREEN_SPI_PORT, true));

    // Hardware reset
    ST7789V2_Reset();

    // Set the resolution and scanning method of the screen
    ST7789V2_Set_Attributes(scan_direction);

    // Set the initialization register
    ST7789V2_Init_Reg();
}

/********************************************************************************
 * @brief           Sets the drawing window coordinates for display operations
 * @param x_start   X-axis start coordinate (0-239)
 * @param y_start   Y-axis start coordinate (0-279)
 * @param x_end     X-axis end coordinate (0-239)
 * @param y_end     Y-axis end coordinate (0-279)
********************************************************************************/
void ST7789V2_Set_Windows(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end)
{    
    if (ST7789V2.SCAN_DIR == VERTICAL) 
    { 
        // set the X coordinates
        ST7789V2_Send_Command(0x2A);
        ST7789V2_Send_Data_8Bit(x_start >> 8);
        ST7789V2_Send_Data_8Bit(x_start);
        ST7789V2_Send_Data_8Bit((x_end-1) >> 8);
        ST7789V2_Send_Data_8Bit(x_end-1);

        // set the Y coordinates
        ST7789V2_Send_Command(0x2B);
        ST7789V2_Send_Data_8Bit((y_start+20) >> 8);
        ST7789V2_Send_Data_8Bit(y_start+20);
        ST7789V2_Send_Data_8Bit((y_end+20-1) >> 8);
        ST7789V2_Send_Data_8Bit(y_end+20-1);
    }
    else { 
        // set the X coordinates
        ST7789V2_Send_Command(0x2A);
        ST7789V2_Send_Data_8Bit((x_start+20) >> 8);
        ST7789V2_Send_Data_8Bit(x_start+20);
        ST7789V2_Send_Data_8Bit((x_end+20-1) >> 8);
        ST7789V2_Send_Data_8Bit(x_end+20-1);

        // set the Y coordinates
        ST7789V2_Send_Command(0x2B);
        ST7789V2_Send_Data_8Bit(y_start >> 8);
        ST7789V2_Send_Data_8Bit(y_start);
        ST7789V2_Send_Data_8Bit((y_end-1) >> 8);
        ST7789V2_Send_Data_8Bit(y_end-1);
    }
    ST7789V2_Send_Command(0x2C);   
}

/********************************************************************************
 * @brief           Clears entire screen with specified color
 * @param color     16-bit RGB565 color value to fill screen
********************************************************************************/
void ST7789V2_Clear(uint16_t color)
{
    uint16_t j;
    uint16_t image[ST7789V2_WIDTH*ST7789V2_HEIGHT];

    color = ((color<<8)&0xff00)|(color>>8);

    for (j=0; j<ST7789V2_WIDTH; j++) 
    {
        image[j] = color;
    }

    ST7789V2_Set_Windows(0, 0, ST7789V2_WIDTH, ST7789V2_HEIGHT);
    gpio_put(SCREEN_DC_PIN, 1);
    gpio_put(SCREEN_CS_PIN, 0);
    for (j = 0; j < ST7789V2_HEIGHT; j++) 
    {
        spi_write_blocking(SCREEN_SPI_PORT, (uint8_t *)&image[j*ST7789V2_WIDTH], ST7789V2_WIDTH*2);
    }
    gpio_put(SCREEN_CS_PIN, 1);
}

/********************************************************************************
 * @brief           Displays a full-screen image buffer
 * @param image     Pointer to 16-bit RGB565 image data (240x280 pixels)
********************************************************************************/
void ST7789V2_Display(uint16_t *image)
{
    uint16_t j;
    
    ST7789V2_Set_Windows(0, 0, ST7789V2_WIDTH, ST7789V2_HEIGHT);
    gpio_put(SCREEN_DC_PIN, 1);
    gpio_put(SCREEN_CS_PIN, 0);
    for ( j= 0; j< ST7789V2_HEIGHT; j++) 
    {
        spi_write_blocking(SCREEN_SPI_PORT, (uint8_t *)&image[j*ST7789V2_WIDTH], ST7789V2_WIDTH*2);
    }
    gpio_put(SCREEN_CS_PIN, 1);
    // SCREEN_1IN47_SendCommand(0x29);
}

/********************************************************************************
 * @brief           Displays image data within specified window coordinates
 * @param x_start   X-axis start coordinate (0-239)
 * @param y_start   Y-axis start coordinate (0-279)
 * @param x_end     X-axis end coordinate (0-239)
 * @param y_end     Y-axis end coordinate (0-279)
 * @param image     Pointer to 16-bit RGB565 image data for the window
********************************************************************************/
void ST7789V2_DisplayWindows(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, uint16_t *image)
{
    uint16_t j, data;
    uint32_t address = 0;
    if(x_start > x_end) 
    {
        data = x_start;
        x_start = x_end;
        x_end = data;
    }
    if (y_start > y_end)
    {
        data = y_start;
        y_start = y_end;
        y_end = data;
    }

    x_start -= 10;x_end += 10;
    y_start -= 10;y_end += 10;
    x_start = (x_start < 300)? x_start : 0;
    y_start = (y_start < 300)? y_start : 0;

    x_end = (x_end < 240)? x_end : 240;
    y_end = (y_end < 280)? y_end : 280;

    ST7789V2_Set_Windows(x_start, y_start, x_end, y_end);
    gpio_put(SCREEN_DC_PIN, 1);
    gpio_put(SCREEN_CS_PIN, 0);
    for (j=y_start; j<y_end-1; j++) 
    {
        address = x_start + j * ST7789V2_WIDTH;
        spi_write_blocking(SCREEN_SPI_PORT, (uint8_t *)&image[address], (x_end-x_start)*2);
    }
    gpio_put(SCREEN_CS_PIN, 1);
}

/********************************************************************************
 * @brief           Displays a single pixel at specified coordinates
 * @param X         X-axis coordinate (0-239)
 * @param Y         Y-axis coordinate (0-279)
 * @param Color     16-bit RGB565 color value for the pixel
********************************************************************************/
void ST7789V2_Draw_Point(uint16_t x, uint16_t y, uint16_t color)
{
    ST7789V2_Set_Windows(x, y, x, y);
    ST7789V2_Send_Data_16Bit(color);
}