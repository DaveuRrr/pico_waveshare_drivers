/*****************************************************************************
* | File      	:   GC9A01A.c
* | Author      :   Waveshare team, Modified by Dave uRrr
* | Function    :   Hardware underlying interface
* | Info        :   Used to shield the underlying layers of each master and enhance portability
*----------------
* |	This version:   V1.1
* | Date        :   2025-09-23
* | Info        :   Removed WS_Config.h dependency, requires user-defined pin macros
*
******************************************************************************/

#include "GC9A01A.h"

GC9A01A_ATTRIBUTES GC9A01A;

uint GC9A01A_SLICE_NUM;
uint GC9A01A_DMA_TX;
dma_channel_config GC9A01A_DMA_CONFIG;

/********************************************************************************
 * @brief           Initializes GPIO pin and sets direction
 * @param pin       GPIO pin number to initialize
 * @param direction GPIO direction (GPIO_IN or GPIO_OUT)
********************************************************************************/
static void GC9A01A_GPIO(uint16_t pin, uint16_t direction)
{
    gpio_init(pin);
    gpio_set_dir(pin, direction);
}

/********************************************************************************
 * @brief           Sets the PWM to GC9A01A display
 * @param value		Set PWM to value 0-100
********************************************************************************/
void GC9A01A_SET_PWM(uint8_t value)
{
    if (value < 0 || value > 100) printf("WS_SET_PWM Error \r\n");
    else pwm_set_chan_level(GC9A01A_SLICE_NUM, PWM_CHAN_B, value);
}

/********************************************************************************
 * @brief           Performs hardware reset sequence for GC9A01A display
********************************************************************************/
static void GC9A01A_Reset(void)
{
    gpio_put(LCD_RST_PIN, 1);
    sleep_ms(100);
    gpio_put(LCD_RST_PIN, 0);
    sleep_ms(100);
    gpio_put(LCD_RST_PIN, 1);
    gpio_put(LCD_CS_PIN, 0);
    sleep_ms(100);
}

/********************************************************************************
 * @brief           Sends a register command to GC9A01A display
 * @param command   8-bit command byte to send
********************************************************************************/
static void GC9A01A_Send_Command(uint8_t command)
{
    gpio_put(LCD_DC_PIN, 0);
    spi_write_blocking(LCD_SPI_PORT, &command, 1);
}

/********************************************************************************
 * @brief           Sends 8-bit data to GC9A01A display
 * @param data      8-bit data byte to send
********************************************************************************/
static void GC9A01A_Send_Data_8Bit(uint8_t data)
{
    gpio_put(LCD_DC_PIN, 1);
    spi_write_blocking(LCD_SPI_PORT, &data, 1);
}

/********************************************************************************
 * @brief           Sends 16-bit data to GC9A01A display (big-endian format)
 * @param data      16-bit data value to send
********************************************************************************/
static void GC9A01A_Send_Data_16Bit(uint16_t data)
{
    gpio_put(LCD_DC_PIN, 1);
    uint8_t high_byte = data >> 8;
    uint8_t low_byte = data & 0xFF;
    spi_write_blocking(LCD_SPI_PORT, &high_byte, 1);
    spi_write_blocking(LCD_SPI_PORT, &low_byte, 1);	
}

/********************************************************************************
 * @brief           Initializes all GC9A01A display registers with default values
********************************************************************************/
static void GC9A01A_Init_Registers(void)
{
    GC9A01A_Send_Command(0xEF);
    GC9A01A_Send_Command(0xEB);
    GC9A01A_Send_Data_8Bit(0x14); 

    GC9A01A_Send_Command(0xFE);			 
    GC9A01A_Send_Command(0xEF); 

    GC9A01A_Send_Command(0xEB);	
    GC9A01A_Send_Data_8Bit(0x14); 

    GC9A01A_Send_Command(0x84);			
    GC9A01A_Send_Data_8Bit(0x40); 

    GC9A01A_Send_Command(0x85);			
    GC9A01A_Send_Data_8Bit(0xFF); 

    GC9A01A_Send_Command(0x86);			
    GC9A01A_Send_Data_8Bit(0xFF); 

    GC9A01A_Send_Command(0x87);			
    GC9A01A_Send_Data_8Bit(0xFF);

    GC9A01A_Send_Command(0x88);			
    GC9A01A_Send_Data_8Bit(0x0A);

    GC9A01A_Send_Command(0x89);			
    GC9A01A_Send_Data_8Bit(0x21); 

    GC9A01A_Send_Command(0x8A);			
    GC9A01A_Send_Data_8Bit(0x00); 

    GC9A01A_Send_Command(0x8B);			
    GC9A01A_Send_Data_8Bit(0x80); 

    GC9A01A_Send_Command(0x8C);			
    GC9A01A_Send_Data_8Bit(0x01); 

    GC9A01A_Send_Command(0x8D);			
    GC9A01A_Send_Data_8Bit(0x01); 

    GC9A01A_Send_Command(0x8E);			
    GC9A01A_Send_Data_8Bit(0xFF); 

    GC9A01A_Send_Command(0x8F);			
    GC9A01A_Send_Data_8Bit(0xFF); 


    GC9A01A_Send_Command(0xB6);
    GC9A01A_Send_Data_8Bit(0x00);
    GC9A01A_Send_Data_8Bit(0x20);

    GC9A01A_Send_Command(0x36);
    GC9A01A_Send_Data_8Bit(0x08);//Set as vertical screen

    GC9A01A_Send_Command(0x3A);			
    GC9A01A_Send_Data_8Bit(0x05); 


    GC9A01A_Send_Command(0x90);			
    GC9A01A_Send_Data_8Bit(0x08);
    GC9A01A_Send_Data_8Bit(0x08);
    GC9A01A_Send_Data_8Bit(0x08);
    GC9A01A_Send_Data_8Bit(0x08); 

    GC9A01A_Send_Command(0xBD);			
    GC9A01A_Send_Data_8Bit(0x06);

    GC9A01A_Send_Command(0xBC);			
    GC9A01A_Send_Data_8Bit(0x00);	

    GC9A01A_Send_Command(0xFF);			
    GC9A01A_Send_Data_8Bit(0x60);
    GC9A01A_Send_Data_8Bit(0x01);
    GC9A01A_Send_Data_8Bit(0x04);

    GC9A01A_Send_Command(0xC3);			
    GC9A01A_Send_Data_8Bit(0x13);
    GC9A01A_Send_Command(0xC4);			
    GC9A01A_Send_Data_8Bit(0x13);

    GC9A01A_Send_Command(0xC9);			
    GC9A01A_Send_Data_8Bit(0x22);

    GC9A01A_Send_Command(0xBE);			
    GC9A01A_Send_Data_8Bit(0x11); 

    GC9A01A_Send_Command(0xE1);			
    GC9A01A_Send_Data_8Bit(0x10);
    GC9A01A_Send_Data_8Bit(0x0E);

    GC9A01A_Send_Command(0xDF);			
    GC9A01A_Send_Data_8Bit(0x21);
    GC9A01A_Send_Data_8Bit(0x0c);
    GC9A01A_Send_Data_8Bit(0x02);

    GC9A01A_Send_Command(0xF0);   
    GC9A01A_Send_Data_8Bit(0x45);
    GC9A01A_Send_Data_8Bit(0x09);
    GC9A01A_Send_Data_8Bit(0x08);
    GC9A01A_Send_Data_8Bit(0x08);
    GC9A01A_Send_Data_8Bit(0x26);
    GC9A01A_Send_Data_8Bit(0x2A);

    GC9A01A_Send_Command(0xF1);    
    GC9A01A_Send_Data_8Bit(0x43);
    GC9A01A_Send_Data_8Bit(0x70);
    GC9A01A_Send_Data_8Bit(0x72);
    GC9A01A_Send_Data_8Bit(0x36);
    GC9A01A_Send_Data_8Bit(0x37);  
    GC9A01A_Send_Data_8Bit(0x6F);


    GC9A01A_Send_Command(0xF2);   
    GC9A01A_Send_Data_8Bit(0x45);
    GC9A01A_Send_Data_8Bit(0x09);
    GC9A01A_Send_Data_8Bit(0x08);
    GC9A01A_Send_Data_8Bit(0x08);
    GC9A01A_Send_Data_8Bit(0x26);
    GC9A01A_Send_Data_8Bit(0x2A);

    GC9A01A_Send_Command(0xF3);   
    GC9A01A_Send_Data_8Bit(0x43);
    GC9A01A_Send_Data_8Bit(0x70);
    GC9A01A_Send_Data_8Bit(0x72);
    GC9A01A_Send_Data_8Bit(0x36);
    GC9A01A_Send_Data_8Bit(0x37); 
    GC9A01A_Send_Data_8Bit(0x6F);

    GC9A01A_Send_Command(0xED);	
    GC9A01A_Send_Data_8Bit(0x1B); 
    GC9A01A_Send_Data_8Bit(0x0B); 

    GC9A01A_Send_Command(0xAE);			
    GC9A01A_Send_Data_8Bit(0x77);

    GC9A01A_Send_Command(0xCD);			
    GC9A01A_Send_Data_8Bit(0x63);		


    GC9A01A_Send_Command(0x70);			
    GC9A01A_Send_Data_8Bit(0x07);
    GC9A01A_Send_Data_8Bit(0x07);
    GC9A01A_Send_Data_8Bit(0x04);
    GC9A01A_Send_Data_8Bit(0x0E); 
    GC9A01A_Send_Data_8Bit(0x0F); 
    GC9A01A_Send_Data_8Bit(0x09);
    GC9A01A_Send_Data_8Bit(0x07);
    GC9A01A_Send_Data_8Bit(0x08);
    GC9A01A_Send_Data_8Bit(0x03);

    GC9A01A_Send_Command(0xE8);			
    GC9A01A_Send_Data_8Bit(0x34);

    GC9A01A_Send_Command(0x62);			
    GC9A01A_Send_Data_8Bit(0x18);
    GC9A01A_Send_Data_8Bit(0x0D);
    GC9A01A_Send_Data_8Bit(0x71);
    GC9A01A_Send_Data_8Bit(0xED);
    GC9A01A_Send_Data_8Bit(0x70); 
    GC9A01A_Send_Data_8Bit(0x70);
    GC9A01A_Send_Data_8Bit(0x18);
    GC9A01A_Send_Data_8Bit(0x0F);
    GC9A01A_Send_Data_8Bit(0x71);
    GC9A01A_Send_Data_8Bit(0xEF);
    GC9A01A_Send_Data_8Bit(0x70); 
    GC9A01A_Send_Data_8Bit(0x70);

    GC9A01A_Send_Command(0x63);			
    GC9A01A_Send_Data_8Bit(0x18);
    GC9A01A_Send_Data_8Bit(0x11);
    GC9A01A_Send_Data_8Bit(0x71);
    GC9A01A_Send_Data_8Bit(0xF1);
    GC9A01A_Send_Data_8Bit(0x70); 
    GC9A01A_Send_Data_8Bit(0x70);
    GC9A01A_Send_Data_8Bit(0x18);
    GC9A01A_Send_Data_8Bit(0x13);
    GC9A01A_Send_Data_8Bit(0x71);
    GC9A01A_Send_Data_8Bit(0xF3);
    GC9A01A_Send_Data_8Bit(0x70); 
    GC9A01A_Send_Data_8Bit(0x70);

    GC9A01A_Send_Command(0x64);			
    GC9A01A_Send_Data_8Bit(0x28);
    GC9A01A_Send_Data_8Bit(0x29);
    GC9A01A_Send_Data_8Bit(0xF1);
    GC9A01A_Send_Data_8Bit(0x01);
    GC9A01A_Send_Data_8Bit(0xF1);
    GC9A01A_Send_Data_8Bit(0x00);
    GC9A01A_Send_Data_8Bit(0x07);

    GC9A01A_Send_Command(0x66);			
    GC9A01A_Send_Data_8Bit(0x3C);
    GC9A01A_Send_Data_8Bit(0x00);
    GC9A01A_Send_Data_8Bit(0xCD);
    GC9A01A_Send_Data_8Bit(0x67);
    GC9A01A_Send_Data_8Bit(0x45);
    GC9A01A_Send_Data_8Bit(0x45);
    GC9A01A_Send_Data_8Bit(0x10);
    GC9A01A_Send_Data_8Bit(0x00);
    GC9A01A_Send_Data_8Bit(0x00);
    GC9A01A_Send_Data_8Bit(0x00);

    GC9A01A_Send_Command(0x67);			
    GC9A01A_Send_Data_8Bit(0x00);
    GC9A01A_Send_Data_8Bit(0x3C);
    GC9A01A_Send_Data_8Bit(0x00);
    GC9A01A_Send_Data_8Bit(0x00);
    GC9A01A_Send_Data_8Bit(0x00);
    GC9A01A_Send_Data_8Bit(0x01);
    GC9A01A_Send_Data_8Bit(0x54);
    GC9A01A_Send_Data_8Bit(0x10);
    GC9A01A_Send_Data_8Bit(0x32);
    GC9A01A_Send_Data_8Bit(0x98);

    GC9A01A_Send_Command(0x74);			
    GC9A01A_Send_Data_8Bit(0x10);	
    GC9A01A_Send_Data_8Bit(0x85);	
    GC9A01A_Send_Data_8Bit(0x80);
    GC9A01A_Send_Data_8Bit(0x00); 
    GC9A01A_Send_Data_8Bit(0x00); 
    GC9A01A_Send_Data_8Bit(0x4E);
    GC9A01A_Send_Data_8Bit(0x00);					

    GC9A01A_Send_Command(0x98);			
    GC9A01A_Send_Data_8Bit(0x3e);
    GC9A01A_Send_Data_8Bit(0x07);

    GC9A01A_Send_Command(0x35);	
    GC9A01A_Send_Command(0x21);

    GC9A01A_Send_Command(0x11);
    sleep_ms(120);
    GC9A01A_Send_Command(0x29);
    sleep_ms(20);
}

/********************************************************************************
 * @brief           Sets display attributes and scan direction
 * @param scan_direction    Display orientation (HORIZONTAL or VERTICAL)
********************************************************************************/
static void GC9A01A_Set_Attributes(uint8_t scan_direction)
{
    //Get the screen scan direction
    GC9A01A.SCAN_DIR = scan_direction;
    uint8_t MemoryAccessReg = 0x08;

    //Get GRAM and LCD width and height
    if(scan_direction == HORIZONTAL) 
    {
        GC9A01A.HEIGHT	= GC9A01A_HEIGHT;
        GC9A01A.WIDTH   = GC9A01A_WIDTH;
        MemoryAccessReg = 0Xc8;
    } else 
    {
        GC9A01A.HEIGHT	= GC9A01A_WIDTH;
        GC9A01A.WIDTH   = GC9A01A_HEIGHT;
        MemoryAccessReg = 0X68;
    }

    // Set the read / write scan direction of the frame memory
    GC9A01A_Send_Command(0x36); //MX, MY, RGB mode
    //GC9A01A_Send_Data_8Bit(MemoryAccessReg);	//0x08 set RGB
    GC9A01A_Send_Data_8Bit(MemoryAccessReg);	//0x08 set RGB
}

/********************************************************************************
 * @brief           Initializes GC9A01A display controller
 * @param scan_direction    Display orientation (HORIZONTAL or VERTICAL)
********************************************************************************/
void GC9A01A_Init(uint8_t scan_direction)
{
    // GPIO Initialize
    GC9A01A_GPIO(LCD_RST_PIN, 1);
    GC9A01A_GPIO(LCD_DC_PIN, 1);
    GC9A01A_GPIO(LCD_CS_PIN, 1);
    GC9A01A_GPIO(LCD_BL_PIN, 1);

    gpio_put(LCD_CS_PIN, 1);
    gpio_put(LCD_DC_PIN, 0);
    gpio_put(LCD_BL_PIN, 1);

    // PWM Configuration
    gpio_set_function(LCD_BL_PIN, GPIO_FUNC_PWM);
    GC9A01A_SLICE_NUM = pwm_gpio_to_slice_num(LCD_BL_PIN);
    pwm_set_wrap(GC9A01A_SLICE_NUM, 100);
    pwm_set_chan_level(GC9A01A_SLICE_NUM, PWM_CHAN_B, 0);
    pwm_set_clkdiv(GC9A01A_SLICE_NUM, 50);
    pwm_set_enabled(GC9A01A_SLICE_NUM, true);

    // SPI Configuration
    spi_init(LCD_SPI_PORT, 270000 * 1000);
    gpio_set_function(LCD_CLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(LCD_MOSI_PIN, GPIO_FUNC_SPI);

    // DMA Configuration
    GC9A01A_DMA_TX = dma_claim_unused_channel(true);
    GC9A01A_DMA_CONFIG = dma_channel_get_default_config(GC9A01A_DMA_TX);
    channel_config_set_transfer_data_size(&GC9A01A_DMA_CONFIG, DMA_SIZE_8); 
    channel_config_set_dreq(&GC9A01A_DMA_CONFIG, spi_get_dreq(LCD_SPI_PORT, true));

    // Hardware reset
    GC9A01A_Reset();

    // Set the resolution and scanning method of the screen
    GC9A01A_Set_Attributes(scan_direction);

    // Set the initialization register
    GC9A01A_Init_Registers();
}

/********************************************************************************
 * @brief           Sets the drawing window coordinates for display operations
 * @param x_start   X-axis start coordinate (0-239)
 * @param y_start   Y-axis start coordinate (0-239)
 * @param x_end     X-axis end coordinate (0-239)
 * @param y_end     Y-axis end coordinate (0-239)
********************************************************************************/
void GC9A01A_Set_Windows(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end)
{
    //set the X coordinates
    GC9A01A_Send_Command(0x2A);
    GC9A01A_Send_Data_8Bit(0x00);
    GC9A01A_Send_Data_8Bit(x_start);
    GC9A01A_Send_Data_8Bit((x_end)>>8);
    GC9A01A_Send_Data_8Bit(x_end);

    //set the Y coordinates
    GC9A01A_Send_Command(0x2B);
    GC9A01A_Send_Data_8Bit(0x00);
    GC9A01A_Send_Data_8Bit(y_start);
    GC9A01A_Send_Data_8Bit((x_end)>>8);
    GC9A01A_Send_Data_8Bit(y_end);

    GC9A01A_Send_Command(0X2C);
    gpio_put(LCD_DC_PIN, 1);
}

/********************************************************************************
 * @brief           Clears entire screen with specified color
 * @param color     16-bit RGB565 color value to fill screen
********************************************************************************/
void GC9A01A_Clear(uint16_t color)
{
    uint16_t j;
    uint16_t image[GC9A01A_WIDTH*GC9A01A_HEIGHT];

    color = ((color<<8)&0xff00)|(color>>8);

    for (j = 0; j < GC9A01A_HEIGHT*GC9A01A_WIDTH; j++) 
    {
        image[j] = color;
    }

    GC9A01A_Set_Windows(0, 0, GC9A01A_WIDTH, GC9A01A_HEIGHT);
    gpio_put(LCD_DC_PIN, 1);
    for(j = 0; j < GC9A01A_HEIGHT; j++)
    {
        spi_write_blocking(LCD_SPI_PORT, (uint8_t *)&image[j*GC9A01A_WIDTH], GC9A01A_WIDTH*2);
    }
}

/********************************************************************************
 * @brief           Displays a full-screen image buffer
 * @param image     Pointer to 16-bit RGB565 image data (240x240 pixels)
********************************************************************************/
void GC9A01A_Display(uint16_t *image)
{
    uint16_t j;
    GC9A01A_Set_Windows(0, 0, GC9A01A_WIDTH, GC9A01A_HEIGHT);
    gpio_put(LCD_DC_PIN, 1);
    for (j = 0; j < GC9A01A_HEIGHT; j++) 
    {
        spi_write_blocking(LCD_SPI_PORT, (uint8_t *)&image[j*GC9A01A_WIDTH], GC9A01A_WIDTH*2);
    }
}

/********************************************************************************
 * @brief           Displays image data within specified window coordinates
 * @param x_start   X-axis start coordinate (0-239)
 * @param y_start   Y-axis start coordinate (0-239)
 * @param x_end     X-axis end coordinate (0-239)
 * @param y_end     Y-axis end coordinate (0-239)
 * @param image     Pointer to 16-bit RGB565 image data for the window
********************************************************************************/
void GC9A01A_Display_Windows(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, uint16_t *image)
{
    uint32_t address = 0;

    uint16_t j;
    GC9A01A_Set_Windows(x_start, y_start, x_end , y_end);
    gpio_put(LCD_DC_PIN, 1);
    for (j = y_start; j < y_end; j++) 
    {
        address = x_start + j * GC9A01A_WIDTH ;
        spi_write_blocking(LCD_SPI_PORT, (uint8_t *)&image[address], (x_end-x_start)*2);
    }
}

/********************************************************************************
 * @brief           Displays a single pixel at specified coordinates
 * @param x         X-axis coordinate (0-239)
 * @param y         Y-axis coordinate (0-239)
 * @param color     16-bit RGB565 color value for the pixel
********************************************************************************/
void GC9A01A_Draw_Point(uint16_t x, uint16_t y, uint16_t color)
{
    GC9A01A_Set_Windows(x, y, x, y);
    GC9A01A_Send_Data_16Bit(color);
}

