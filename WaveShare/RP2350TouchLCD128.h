#ifndef _WAVESHARE_RP2350TOUCHLCD128_CONFIG_H_
#define _WAVESHARE_RP2350TOUCHLCD128_CONFIG_H_

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

#define SENSOR_SDA_PIN  (6)
#define SENSOR_SCL_PIN  (7)

#define LCD_DC_PIN      (8)
#define LCD_CS_PIN      (9)
#define LCD_CLK_PIN     (10)
#define LCD_MOSI_PIN    (11)
#define LCD_MISO_PIN    (12)
#define LCD_RST_PIN     (13)
#define LCD_BL_PIN      (25)

#define TOUCH_INT_PIN   (21)
#define TOUCH_RST_PIN   (22)

#define DOF_INT1        (23)
#define DOF_INT2        (24)

#define BAT_ADC_PIN     (29)
#define BAR_CHANNEL     (3)

extern uint dma_tx;
extern dma_channel_config c;

#endif /* _WAVESHARE_RP2350TOUCHLCD128_CONFIG_H_ */