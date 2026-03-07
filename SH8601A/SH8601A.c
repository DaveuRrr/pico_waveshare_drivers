/*****************************************************************************
* | File      	:   SH8601A.c
* | Author      :   Waveshare Team, Modified by Dave uRrr
* | Function    :   AMOLED Interface Functions
* | Info        :   Used to shield the underlying layers of each master and enhance portability
*----------------
* | This version:   V1.1
* | Date        :   2026-02-25
* | Info        :   Removed DEV_Config.h dependency, added rotation support and missing features
*
******************************************************************************/

#include "SH8601A.h"

SH8601A_ATTRIBUTES SH8601A;

/********************************************************************************
 * @brief           Sends a register command to SH8601A display
 * @param command   8-bit command byte to send
********************************************************************************/
static void SH8601A_Send_Command(uint8_t command)
{
    QSPI_Select(qspi);
    QSPI_REGISTER_Write(qspi, command);
    QSPI_Deselect(qspi);
}

/********************************************************************************
 * @brief           Sends 8-bit data to SH8601A display
 * @param data      8-bit data byte to send
********************************************************************************/
static void SH8601A_Send_Data(uint8_t data)
{
    QSPI_Select(qspi);
    QSPI_DATA_Write(qspi, data);
    QSPI_Deselect(qspi);
}

/********************************************************************************
 * @brief           Sets the drawing window coordinates for display operations
 * @param x_start   X-axis start coordinate (0-367)
 * @param y_start   Y-axis start coordinate (0-447)
 * @param x_end     X-axis end coordinate (0-367)
 * @param y_end     Y-axis end coordinate (0-447)
********************************************************************************/
void SH8601A_Set_Windows(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end){
    QSPI_Select(qspi);
    QSPI_REGISTER_Write(qspi, 0x2a);
    QSPI_DATA_Write(qspi, x_start>>8);
    QSPI_DATA_Write(qspi, x_start&0xff);
    QSPI_DATA_Write(qspi, (x_end-1)>>8);
    QSPI_DATA_Write(qspi, (x_end-1)&0xff);
    QSPI_Deselect(qspi);

    QSPI_Select(qspi);
    QSPI_REGISTER_Write(qspi, 0x2b);
    QSPI_DATA_Write(qspi, y_start>>8);
    QSPI_DATA_Write(qspi, y_start&0xff);
    QSPI_DATA_Write(qspi, (y_end-1)>>8);
    QSPI_DATA_Write(qspi, (y_end-1)&0xff);
    QSPI_Deselect(qspi);

    QSPI_Select(qspi);
    QSPI_REGISTER_Write(qspi, 0x2c);
    QSPI_Deselect(qspi);
}

/********************************************************************************
 * @brief           Initializes all SH8601A display registers with default values
********************************************************************************/
static void SH8601A_Init_Registers(void){
    SH8601A_Send_Command(0x11);  // Sleep Out
    sleep_ms(120);

    QSPI_Select(qspi);
    QSPI_REGISTER_Write(qspi, 0x44);
    QSPI_DATA_Write(qspi, 0x01);
    QSPI_DATA_Write(qspi, 0xC5);
    QSPI_Deselect(qspi);

    QSPI_Select(qspi);
    QSPI_REGISTER_Write(qspi, 0x35);  // Tearing Effect Line ON
    QSPI_DATA_Write(qspi, 0x00);
    QSPI_Deselect(qspi);

    QSPI_Select(qspi);
    QSPI_REGISTER_Write(qspi, 0x3A);  // Interface Pixel Format
    QSPI_DATA_Write(qspi, 0x55);      // 16-bit/pixel
    QSPI_Deselect(qspi);

    QSPI_Select(qspi);
    QSPI_REGISTER_Write(qspi, 0xC4);
    QSPI_DATA_Write(qspi, 0x80);
    QSPI_Deselect(qspi);

    QSPI_Select(qspi);
    QSPI_REGISTER_Write(qspi, 0x53);  // Write CTRL Display
    QSPI_DATA_Write(qspi, 0x20);
    QSPI_Deselect(qspi);

    QSPI_Select(qspi);
    QSPI_REGISTER_Write(qspi, 0x51);  // Write Display Brightness
    QSPI_DATA_Write(qspi, 0xFF);
    QSPI_Deselect(qspi);

    SH8601A_Send_Command(0x29);  // Display ON

    sleep_ms(10);
}

/********************************************************************************
 * @brief           Performs hardware reset sequence for SH8601A display
********************************************************************************/
static void SH8601A_Reset(void){
    gpio_put(qspi.pin_rst, 1);
    sleep_ms(50);
    gpio_put(qspi.pin_rst, 0);
    sleep_ms(50);
    gpio_put(qspi.pin_rst, 1);
    sleep_ms(300);
}

/********************************************************************************
 * @brief           Sets display attributes and rotation
 * @param rotation  Display orientation 0, 90, 180, 270
********************************************************************************/
static void SH8601A_Set_Rotation(uint8_t rotation)
{
    SH8601A.ROTATION = rotation;
    uint8_t MemoryAccessReg = 0x00;

    switch(rotation)
    {
        case ROTATION_0:
            SH8601A.HEIGHT = SH8601A_HEIGHT;
            SH8601A.WIDTH = SH8601A_WIDTH;
            MemoryAccessReg = 0x00;  // Normal orientation
            break;

        case ROTATION_90:
            SH8601A.HEIGHT = SH8601A_WIDTH;
            SH8601A.WIDTH = SH8601A_HEIGHT;
            MemoryAccessReg = 0x60;  // MV=1, MX=1 (X-Y Exchange + X-Mirror)
            break;

        case ROTATION_180:
            SH8601A.HEIGHT = SH8601A_HEIGHT;
            SH8601A.WIDTH = SH8601A_WIDTH;
            MemoryAccessReg = 0xC0;  // MX=1, MY=1 (X-Mirror Y-Mirror)
            break;

        case ROTATION_270:
            SH8601A.HEIGHT = SH8601A_WIDTH;
            SH8601A.WIDTH = SH8601A_HEIGHT;
            MemoryAccessReg = 0xA0;  // MV=1, MY=1 (X-Y Exchange + Y-Mirror)
            break;
    }

    // Set the read / write scan direction of the frame memory
    QSPI_Select(qspi);
    QSPI_REGISTER_Write(qspi, 0x36);  // MADCTL: Memory Data Access Control
    QSPI_DATA_Write(qspi, MemoryAccessReg);
    QSPI_Deselect(qspi);
}

/********************************************************************************
 * @brief           Initializes SH8601A display controller
 * @param rotation  Display orientation 0, 90, 180, 270
********************************************************************************/
void SH8601A_Init(uint8_t rotation)
{
    // Hardware reset
    SH8601A_Reset();

    // Set the resolution and scanning method of the screen
    SH8601A_Set_Rotation(rotation);

    // Set the initialization register
    SH8601A_Init_Registers();
}

/********************************************************************************
 * @brief           Sets AMOLED display brightness
 * @param brightness Brightness level 0-100 (percentage)
********************************************************************************/
void SH8601A_Set_Brightness(uint8_t brightness){
    if(brightness > 100) brightness = 100;
    brightness = brightness * 255 / 100;

    QSPI_Select(qspi);
    QSPI_REGISTER_Write(qspi, 0x51);  // Write Display Brightness
    QSPI_DATA_Write(qspi, brightness);
    QSPI_Deselect(qspi);
}


/********************************************************************************
 * @brief           Clears entire screen with specified color
 * @param color     16-bit RGB565 color value to fill screen
********************************************************************************/
void SH8601A_Clear(uint16_t color) {
    // Color data
    uint16_t i;
    uint16_t image[SH8601A.HEIGHT];
    for(i=0; i<SH8601A.HEIGHT; i++){
        image[i] = color>>8 | (color&0xff)<<8;
    }
    uint8_t *partial_image = (uint8_t *)(image);

    // Send command in one-line mode
    SH8601A_Set_Windows(0, 0, SH8601A.WIDTH, SH8601A.HEIGHT);
    QSPI_Select(qspi);
    QSPI_Pixel_Write(qspi, 0x2c);

    // Four-wire mode sends RGB data
    channel_config_set_dreq(&c, pio_get_dreq(qspi.pio, qspi.sm, true));
    for (int i = 0; i < SH8601A.HEIGHT; i++) {
        dma_channel_configure(dma_tx,
                            &c,
                            &qspi.pio->txf[qspi.sm],  // Destination pointer (PIO TX FIFO)
                            partial_image,            // Source pointer (data buffer)
                            SH8601A.WIDTH*2,          // Data length (unit: number of transmissions)
                            true                      // Start transferring immediately
        );

        // Waiting for DMA transfer to complete
        while(dma_channel_is_busy(dma_tx));
    }

    QSPI_Deselect(qspi);
}


/********************************************************************************
 * @brief           Displays a full-screen image buffer
 * @param image     Pointer to 16-bit RGB565 image data (368x448 pixels)
********************************************************************************/
void SH8601A_Display(uint16_t *image)
{
    // Send command in one-line mode
    SH8601A_Set_Windows(0, 0, SH8601A.WIDTH, SH8601A.HEIGHT);
    QSPI_Select(qspi);
    QSPI_Pixel_Write(qspi, 0x2c);

    // Four-wire mode sends RGB data
    channel_config_set_dreq(&c, pio_get_dreq(qspi.pio, qspi.sm, true));
    dma_channel_configure(dma_tx,
                        &c,
                        &qspi.pio->txf[qspi.sm],         // Destination pointer (PIO TX FIFO)
                        (uint8_t *)image,                // Source pointer (data buffer)
                        SH8601A.WIDTH*SH8601A.HEIGHT*2,  // Data length (unit: number of transmissions)
                        true                             // Start transferring immediately
    );

    // Waiting for DMA transfer to complete
    while(dma_channel_is_busy(dma_tx));
    QSPI_Deselect(qspi);
}

/********************************************************************************
 * @brief           Displays image data within specified window coordinates
 * @param x_start   X-axis start coordinate (0-367)
 * @param y_start   Y-axis start coordinate (0-447)
 * @param x_end     X-axis end coordinate (0-367)
 * @param y_end     Y-axis end coordinate (0-447)
 * @param image     Pointer to 16-bit RGB565 image data for the window
********************************************************************************/
void SH8601A_Display_Windows(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, uint16_t *image) {
    // Send command in one-line mode
    SH8601A_Set_Windows(x_start, y_start, x_end, y_end);
    QSPI_Select(qspi);
    QSPI_Pixel_Write(qspi, 0x2c);

    // Four-wire mode sends RGB data
    channel_config_set_dreq(&c, pio_get_dreq(qspi.pio, qspi.sm, true));

    int i;
    uint32_t pixel_offset;
    uint8_t *partial_image;
    for (i = y_start; i < y_end - 1; i++) {
        pixel_offset = (i * SH8601A.WIDTH + x_start) * 2;
        partial_image = (uint8_t *)image + pixel_offset;
        dma_channel_configure(dma_tx,
                            &c,
                            &qspi.pio->txf[qspi.sm],  // Destination pointer (PIO TX FIFO)
                            partial_image,            // Source pointer (data buffer)
                            (x_end-x_start)*2,        // Data length (unit: number of transmissions)
                            true                      // Start transferring immediately
        );

        // Waiting for DMA transfer to complete
        while(dma_channel_is_busy(dma_tx));
    }

    QSPI_Deselect(qspi);
}

/********************************************************************************
 * @brief           Displays a single pixel at specified coordinates
 * @param x         X-axis coordinate (0-367)
 * @param y         Y-axis coordinate (0-447)
 * @param color     16-bit RGB565 color value for the pixel
********************************************************************************/
void SH8601A_Draw_Point(uint16_t x, uint16_t y, uint16_t color)
{
    SH8601A_Set_Windows(x, y, x+1, y+1);
    QSPI_Select(qspi);
    QSPI_Pixel_Write(qspi, 0x2c);

    uint8_t color_bytes[2];
    color_bytes[0] = color >> 8;
    color_bytes[1] = color & 0xFF;

    QSPI_DATA_Write(qspi, color_bytes[0]);
    QSPI_DATA_Write(qspi, color_bytes[1]);
    QSPI_Deselect(qspi);
}

/********************************************************************************
 * @brief           Put Display in a Sleep State
********************************************************************************/
void SH8601A_Sleep(void)
{
    SH8601A_Send_Command(0x10);  // Enter Sleep Mode
}

/********************************************************************************
 * @brief           Put Display in an Awake State
********************************************************************************/
void SH8601A_Awake(void)
{
    SH8601A_Send_Command(0x11);  // Sleep Out
    sleep_ms(120);  // Wait for display to wake up
}
