#ifndef _WAVESHARE_RP2350LCD128_CONFIG_H_
#define _WAVESHARE_RP2350LCD128_CONFIG_H_

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

#define SCREEN_SPI_PORT     (spi1)
#define SENSOR_I2C_PORT     (i2c1)

#define SENSOR_SDA_PIN      (6)
#define SENSOR_SCL_PIN      (7)

#define SCREEN_DC_PIN       (8)
#define SCREEN_CS_PIN       (9)
#define SCREEN_CLK_PIN      (10)
#define SCREEN_MOSI_PIN     (11)
#define SCREEN_RST_PIN      (12)
#define SCREEN_BL_PIN       (25)

#define TOUCH_INT_PIN       (21)
#define TOUCH_RST_PIN       (22)

#define DOF_INT1            (23)
#define DOF_INT2            (24)

#define BAT_ADC_PIN         (29)
#define BAT_CHANNEL         (3)

extern uint dma_tx;
extern dma_channel_config c;

void Device_Module_Init();

#endif /* _WAVESHARE_RP2350LCD128_CONFIG_H_ */