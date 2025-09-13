/*****************************************************************************
* | File      	:   RP2350TouchLCD_config.h
* | Author      :
* | Function    :   Hardware underlying interface
* | Info        :
*----------------
* |	This version:   V1.0
* | Date        :   2021-03-16
* | Info        :
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of theex Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
******************************************************************************/
#ifndef _WAVESHARE_CONFIG_H_
#define _WAVESHARE_CONFIG_H_

#include "stdio.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/pll.h"
#include "hardware/clocks.h"

#define PLL_SYS_KHZ (270 * 1000)

#define LCD_SPI_PORT    (spi1)
#define SENSOR_I2C_PORT (i2c1)
/**
 * GPIOI config
 **/
#define WS_SDA_PIN      (6)
#define WS_SCL_PIN      (7)

#define LCD_DC_PIN      (8)
#define LCD_CS_PIN      (9)
#define LCD_CLK_PIN     (10)
#define LCD_MOSI_PIN    (11)
#ifndef LCD_RST_PIN
#define LCD_MISO_PIN    (12)
#define LCD_RST_PIN     (13)
#endif
#define LCD_BL_PIN      (25)

#define TOUCH_INT_PIN   (21)
#define TOUCH_RST_PIN   (22)

#define IMU_INT1        (23)
#define IMU_INT2        (24)

#define BAT_ADC_PIN     (29)
#define BAR_CHANNEL     (3)

/*------------------------------------------------------------------------------------------------------*/

extern uint dma_tx;
extern dma_channel_config c;

void WS_Delay_ms(uint32_t xms);
void WS_Delay_us(uint32_t xus);

void WS_Digital_Write(uint16_t Pin, uint8_t Value);
uint8_t WS_Digital_Read(uint16_t Pin);

void WS_GPIO_Mode(uint16_t Pin, uint16_t Mode);
void WS_KEY_Config(uint16_t Pin);
void WS_Digital_Write(uint16_t Pin, uint8_t Value);
uint8_t WS_Digital_Read(uint16_t Pin);

uint16_t DEC_ADC_Read(void);

void WS_SPI_WriteByte(spi_inst_t *SPI_PORT,uint8_t Value);
void WS_SPI_Write_nByte(spi_inst_t *SPI_PORT,uint8_t *pData, uint32_t Len);


void WS_I2C_Write_Byte(i2c_inst_t *I2C_PORT,uint8_t addr, uint8_t reg, uint8_t Value);
void WS_I2C_Write_nByte(i2c_inst_t *I2C_PORT,uint8_t addr, uint8_t *pData, uint32_t Len);
uint8_t WS_I2C_Read_Byte(i2c_inst_t *I2C_PORT,uint8_t addr, uint8_t reg);
void WS_I2C_Read_nByte(i2c_inst_t *I2C_PORT,uint8_t addr,uint8_t reg, uint8_t *pData, uint32_t Len);

void WS_IRQ_SET(uint gpio, uint32_t events, gpio_irq_callback_t callback);

void WS_SET_PWM(uint8_t Value);

uint8_t WS_Module_Init(void);
void WS_Module_Exit(void);

#endif
