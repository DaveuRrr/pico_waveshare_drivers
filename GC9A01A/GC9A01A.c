/*****************************************************************************
* | File      	:   GC9A01A.c
* | Author      :   Waveshare team
* | Function    :   Hardware underlying interface
* | Info        :
*                Used to shield the underlying layers of each master
*                and enhance portability
*----------------
* |	This version:   V1.0
* | Date        :   2020-12-16
* | Info        :   Basic version
*
******************************************************************************/
#include "GC9A01A.h"

#include <stdlib.h>		//itoa()
#include <stdio.h>

GC9A01A_ATTRIBUTES GC9A01A;


/******************************************************************************
function :	Hardware reset
parameter:
******************************************************************************/
static void GC9A01A_Reset(void)
{
    WS_Digital_Write(LCD_RST_PIN, 1);
    WS_Delay_ms(100);
    WS_Digital_Write(LCD_RST_PIN, 0);
    WS_Delay_ms(100);
    WS_Digital_Write(LCD_RST_PIN, 1);
	WS_Digital_Write(LCD_CS_PIN, 0);
    WS_Delay_ms(100);
}

/******************************************************************************
function :	send command
parameter:
     Reg : Command register
******************************************************************************/
static void GC9A01A_SendCommand(uint8_t Reg)
{
    WS_Digital_Write(LCD_DC_PIN, 0);
    //WS_Digital_Write(LCD_CS_PIN, 0);
    WS_SPI_WriteByte(LCD_SPI_PORT,Reg);
    //WS_Digital_Write(LCD_CS_PIN, 1);
}

/******************************************************************************
function :	send data
parameter:
    Data : Write data
******************************************************************************/
static void GC9A01A_SendData_8Bit(uint8_t Data)
{
    WS_Digital_Write(LCD_DC_PIN, 1);
    //WS_Digital_Write(LCD_CS_PIN, 0);
    WS_SPI_WriteByte(LCD_SPI_PORT,Data);
    //WS_Digital_Write(LCD_CS_PIN, 1);
}

/******************************************************************************
function :	send data
parameter:
    Data : Write data
******************************************************************************/
static void GC9A01A_SendData_16Bit(uint16_t Data)
{
    WS_Digital_Write(LCD_DC_PIN, 1);
    //WS_Digital_Write(LCD_CS_PIN, 0);
    WS_SPI_WriteByte(LCD_SPI_PORT,Data >> 8);
    WS_SPI_WriteByte(LCD_SPI_PORT,Data);
   // WS_Digital_Write(LCD_CS_PIN, 1);
	
}

/******************************************************************************
function :	Initialize the lcd register
parameter:
******************************************************************************/
static void GC9A01A_InitReg(void)
{
    GC9A01A_SendCommand(0xEF);
	GC9A01A_SendCommand(0xEB);
	GC9A01A_SendData_8Bit(0x14); 
	
    GC9A01A_SendCommand(0xFE);			 
	GC9A01A_SendCommand(0xEF); 

	GC9A01A_SendCommand(0xEB);	
	GC9A01A_SendData_8Bit(0x14); 

	GC9A01A_SendCommand(0x84);			
	GC9A01A_SendData_8Bit(0x40); 

	GC9A01A_SendCommand(0x85);			
	GC9A01A_SendData_8Bit(0xFF); 

	GC9A01A_SendCommand(0x86);			
	GC9A01A_SendData_8Bit(0xFF); 

	GC9A01A_SendCommand(0x87);			
	GC9A01A_SendData_8Bit(0xFF);

	GC9A01A_SendCommand(0x88);			
	GC9A01A_SendData_8Bit(0x0A);

	GC9A01A_SendCommand(0x89);			
	GC9A01A_SendData_8Bit(0x21); 

	GC9A01A_SendCommand(0x8A);			
	GC9A01A_SendData_8Bit(0x00); 

	GC9A01A_SendCommand(0x8B);			
	GC9A01A_SendData_8Bit(0x80); 

	GC9A01A_SendCommand(0x8C);			
	GC9A01A_SendData_8Bit(0x01); 

	GC9A01A_SendCommand(0x8D);			
	GC9A01A_SendData_8Bit(0x01); 

	GC9A01A_SendCommand(0x8E);			
	GC9A01A_SendData_8Bit(0xFF); 

	GC9A01A_SendCommand(0x8F);			
	GC9A01A_SendData_8Bit(0xFF); 


	GC9A01A_SendCommand(0xB6);
	GC9A01A_SendData_8Bit(0x00);
	GC9A01A_SendData_8Bit(0x20);

	GC9A01A_SendCommand(0x36);
	GC9A01A_SendData_8Bit(0x08);//Set as vertical screen

	GC9A01A_SendCommand(0x3A);			
	GC9A01A_SendData_8Bit(0x05); 


	GC9A01A_SendCommand(0x90);			
	GC9A01A_SendData_8Bit(0x08);
	GC9A01A_SendData_8Bit(0x08);
	GC9A01A_SendData_8Bit(0x08);
	GC9A01A_SendData_8Bit(0x08); 

	GC9A01A_SendCommand(0xBD);			
	GC9A01A_SendData_8Bit(0x06);
	
	GC9A01A_SendCommand(0xBC);			
	GC9A01A_SendData_8Bit(0x00);	

	GC9A01A_SendCommand(0xFF);			
	GC9A01A_SendData_8Bit(0x60);
	GC9A01A_SendData_8Bit(0x01);
	GC9A01A_SendData_8Bit(0x04);

	GC9A01A_SendCommand(0xC3);			
	GC9A01A_SendData_8Bit(0x13);
	GC9A01A_SendCommand(0xC4);			
	GC9A01A_SendData_8Bit(0x13);

	GC9A01A_SendCommand(0xC9);			
	GC9A01A_SendData_8Bit(0x22);

	GC9A01A_SendCommand(0xBE);			
	GC9A01A_SendData_8Bit(0x11); 

	GC9A01A_SendCommand(0xE1);			
	GC9A01A_SendData_8Bit(0x10);
	GC9A01A_SendData_8Bit(0x0E);

	GC9A01A_SendCommand(0xDF);			
	GC9A01A_SendData_8Bit(0x21);
	GC9A01A_SendData_8Bit(0x0c);
	GC9A01A_SendData_8Bit(0x02);

	GC9A01A_SendCommand(0xF0);   
	GC9A01A_SendData_8Bit(0x45);
	GC9A01A_SendData_8Bit(0x09);
	GC9A01A_SendData_8Bit(0x08);
	GC9A01A_SendData_8Bit(0x08);
	GC9A01A_SendData_8Bit(0x26);
 	GC9A01A_SendData_8Bit(0x2A);

 	GC9A01A_SendCommand(0xF1);    
 	GC9A01A_SendData_8Bit(0x43);
 	GC9A01A_SendData_8Bit(0x70);
 	GC9A01A_SendData_8Bit(0x72);
 	GC9A01A_SendData_8Bit(0x36);
 	GC9A01A_SendData_8Bit(0x37);  
 	GC9A01A_SendData_8Bit(0x6F);


 	GC9A01A_SendCommand(0xF2);   
 	GC9A01A_SendData_8Bit(0x45);
 	GC9A01A_SendData_8Bit(0x09);
 	GC9A01A_SendData_8Bit(0x08);
 	GC9A01A_SendData_8Bit(0x08);
 	GC9A01A_SendData_8Bit(0x26);
 	GC9A01A_SendData_8Bit(0x2A);

 	GC9A01A_SendCommand(0xF3);   
 	GC9A01A_SendData_8Bit(0x43);
 	GC9A01A_SendData_8Bit(0x70);
 	GC9A01A_SendData_8Bit(0x72);
 	GC9A01A_SendData_8Bit(0x36);
 	GC9A01A_SendData_8Bit(0x37); 
 	GC9A01A_SendData_8Bit(0x6F);

	GC9A01A_SendCommand(0xED);	
	GC9A01A_SendData_8Bit(0x1B); 
	GC9A01A_SendData_8Bit(0x0B); 

	GC9A01A_SendCommand(0xAE);			
	GC9A01A_SendData_8Bit(0x77);
	
	GC9A01A_SendCommand(0xCD);			
	GC9A01A_SendData_8Bit(0x63);		


	GC9A01A_SendCommand(0x70);			
	GC9A01A_SendData_8Bit(0x07);
	GC9A01A_SendData_8Bit(0x07);
	GC9A01A_SendData_8Bit(0x04);
	GC9A01A_SendData_8Bit(0x0E); 
	GC9A01A_SendData_8Bit(0x0F); 
	GC9A01A_SendData_8Bit(0x09);
	GC9A01A_SendData_8Bit(0x07);
	GC9A01A_SendData_8Bit(0x08);
	GC9A01A_SendData_8Bit(0x03);

	GC9A01A_SendCommand(0xE8);			
	GC9A01A_SendData_8Bit(0x34);

	GC9A01A_SendCommand(0x62);			
	GC9A01A_SendData_8Bit(0x18);
	GC9A01A_SendData_8Bit(0x0D);
	GC9A01A_SendData_8Bit(0x71);
	GC9A01A_SendData_8Bit(0xED);
	GC9A01A_SendData_8Bit(0x70); 
	GC9A01A_SendData_8Bit(0x70);
	GC9A01A_SendData_8Bit(0x18);
	GC9A01A_SendData_8Bit(0x0F);
	GC9A01A_SendData_8Bit(0x71);
	GC9A01A_SendData_8Bit(0xEF);
	GC9A01A_SendData_8Bit(0x70); 
	GC9A01A_SendData_8Bit(0x70);

	GC9A01A_SendCommand(0x63);			
	GC9A01A_SendData_8Bit(0x18);
	GC9A01A_SendData_8Bit(0x11);
	GC9A01A_SendData_8Bit(0x71);
	GC9A01A_SendData_8Bit(0xF1);
	GC9A01A_SendData_8Bit(0x70); 
	GC9A01A_SendData_8Bit(0x70);
	GC9A01A_SendData_8Bit(0x18);
	GC9A01A_SendData_8Bit(0x13);
	GC9A01A_SendData_8Bit(0x71);
	GC9A01A_SendData_8Bit(0xF3);
	GC9A01A_SendData_8Bit(0x70); 
	GC9A01A_SendData_8Bit(0x70);

	GC9A01A_SendCommand(0x64);			
	GC9A01A_SendData_8Bit(0x28);
	GC9A01A_SendData_8Bit(0x29);
	GC9A01A_SendData_8Bit(0xF1);
	GC9A01A_SendData_8Bit(0x01);
	GC9A01A_SendData_8Bit(0xF1);
	GC9A01A_SendData_8Bit(0x00);
	GC9A01A_SendData_8Bit(0x07);

	GC9A01A_SendCommand(0x66);			
	GC9A01A_SendData_8Bit(0x3C);
	GC9A01A_SendData_8Bit(0x00);
	GC9A01A_SendData_8Bit(0xCD);
	GC9A01A_SendData_8Bit(0x67);
	GC9A01A_SendData_8Bit(0x45);
	GC9A01A_SendData_8Bit(0x45);
	GC9A01A_SendData_8Bit(0x10);
	GC9A01A_SendData_8Bit(0x00);
	GC9A01A_SendData_8Bit(0x00);
	GC9A01A_SendData_8Bit(0x00);

	GC9A01A_SendCommand(0x67);			
	GC9A01A_SendData_8Bit(0x00);
	GC9A01A_SendData_8Bit(0x3C);
	GC9A01A_SendData_8Bit(0x00);
	GC9A01A_SendData_8Bit(0x00);
	GC9A01A_SendData_8Bit(0x00);
	GC9A01A_SendData_8Bit(0x01);
	GC9A01A_SendData_8Bit(0x54);
	GC9A01A_SendData_8Bit(0x10);
	GC9A01A_SendData_8Bit(0x32);
	GC9A01A_SendData_8Bit(0x98);

	GC9A01A_SendCommand(0x74);			
	GC9A01A_SendData_8Bit(0x10);	
	GC9A01A_SendData_8Bit(0x85);	
	GC9A01A_SendData_8Bit(0x80);
	GC9A01A_SendData_8Bit(0x00); 
	GC9A01A_SendData_8Bit(0x00); 
	GC9A01A_SendData_8Bit(0x4E);
	GC9A01A_SendData_8Bit(0x00);					
	
    GC9A01A_SendCommand(0x98);			
	GC9A01A_SendData_8Bit(0x3e);
	GC9A01A_SendData_8Bit(0x07);

	GC9A01A_SendCommand(0x35);	
	GC9A01A_SendCommand(0x21);

	GC9A01A_SendCommand(0x11);
	WS_Delay_ms(120);
	GC9A01A_SendCommand(0x29);
	WS_Delay_ms(20);
}

/********************************************************************************
function:	Set the resolution and scanning method of the screen
parameter:
		Scan_dir:   Scan direction
********************************************************************************/
static void GC9A01A_SetAttributes(uint8_t Scan_dir)
{
    //Get the screen scan direction
    GC9A01A.SCAN_DIR = Scan_dir;
    uint8_t MemoryAccessReg = 0x08;

    //Get GRAM and LCD width and height
    if(Scan_dir == HORIZONTAL) {
        GC9A01A.HEIGHT	= GC9A01A_HEIGHT;
        GC9A01A.WIDTH   = GC9A01A_WIDTH;
        MemoryAccessReg = 0Xc8;
    } else {
        GC9A01A.HEIGHT	= GC9A01A_WIDTH;
        GC9A01A.WIDTH   = GC9A01A_HEIGHT;
        MemoryAccessReg = 0X68;
    }

    // Set the read / write scan direction of the frame memory
    GC9A01A_SendCommand(0x36); //MX, MY, RGB mode
    //GC9A01A_SendData_8Bit(MemoryAccessReg);	//0x08 set RGB
	GC9A01A_SendData_8Bit(MemoryAccessReg);	//0x08 set RGB
}

/********************************************************************************
function :	Initialize the lcd
parameter:
********************************************************************************/
void GC9A01A_Init(uint8_t Scan_dir)
{
    //Turn on the backlight
    //WS_SET_PWM(100);
    //Hardware reset
    GC9A01A_Reset();

    //Set the resolution and scanning method of the screen
    GC9A01A_SetAttributes(Scan_dir);
    
    //Set the initialization register
    GC9A01A_InitReg();
}

/********************************************************************************
function:	Sets the start position and size of the display area
parameter:
		Xstart 	:   X direction Start coordinates
		Ystart  :   Y direction Start coordinates
		Xend    :   X direction end coordinates
		Yend    :   Y direction end coordinates
********************************************************************************/
void GC9A01A_SetWindows(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend)
{
    //set the X coordinates
    GC9A01A_SendCommand(0x2A);
    GC9A01A_SendData_8Bit(0x00);
    GC9A01A_SendData_8Bit(Xstart);
    GC9A01A_SendData_8Bit((Xend)>>8);
    GC9A01A_SendData_8Bit(Xend);

    //set the Y coordinates
    GC9A01A_SendCommand(0x2B);
    GC9A01A_SendData_8Bit(0x00);
    GC9A01A_SendData_8Bit(Ystart);
    GC9A01A_SendData_8Bit((Xend)>>8);
    GC9A01A_SendData_8Bit(Yend);

    GC9A01A_SendCommand(0X2C);
    WS_Digital_Write(LCD_DC_PIN, 1);;
}

/******************************************************************************
function :	Clear screen
parameter:
******************************************************************************/
void GC9A01A_Clear(uint16_t Color)
{
    uint16_t j;
    uint16_t Image[GC9A01A_WIDTH*GC9A01A_HEIGHT];
    
    Color = ((Color<<8)&0xff00)|(Color>>8);
   
    for (j = 0; j < GC9A01A_HEIGHT*GC9A01A_WIDTH; j++) {
        Image[j] = Color;
    }
    
    GC9A01A_SetWindows(0, 0, GC9A01A_WIDTH, GC9A01A_HEIGHT);
    WS_Digital_Write(LCD_DC_PIN, 1);;
    for(j = 0; j < GC9A01A_HEIGHT; j++){
        WS_SPI_Write_nByte(LCD_SPI_PORT,(uint8_t *)&Image[j*GC9A01A_WIDTH], GC9A01A_WIDTH*2);
    }
}

/******************************************************************************
function :	Sends the image buffer in RAM to displays
parameter:
******************************************************************************/
void GC9A01A_Display(uint16_t *Image)
{
    uint16_t j;
    GC9A01A_SetWindows(0, 0, GC9A01A_WIDTH, GC9A01A_HEIGHT);
    WS_Digital_Write(LCD_DC_PIN, 1);;
    for (j = 0; j < GC9A01A_HEIGHT; j++) {
        WS_SPI_Write_nByte(LCD_SPI_PORT,(uint8_t *)&Image[j*GC9A01A_WIDTH], GC9A01A_WIDTH*2);
    }
}

void GC9A01A_DisplayWindows(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend, uint16_t *Image)
{
    // display
    uint32_t Addr = 0;

    uint16_t j;
    GC9A01A_SetWindows(Xstart, Ystart, Xend , Yend);
    WS_Digital_Write(LCD_DC_PIN, 1);;
    for (j = Ystart; j < Yend; j++) {
        Addr = Xstart + j * GC9A01A_WIDTH ;
        WS_SPI_Write_nByte(LCD_SPI_PORT,(uint8_t *)&Image[Addr], (Xend-Xstart)*2);
    }
}


void GC9A01A_DisplayPoint(uint16_t X, uint16_t Y, uint16_t Color)
{
    GC9A01A_SetWindows(X,Y,X,Y);
    GC9A01A_SendData_16Bit(Color);
}

