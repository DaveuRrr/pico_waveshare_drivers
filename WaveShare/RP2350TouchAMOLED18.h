#ifndef _WAVESHARE_RP2350TOUCHAMOLED18_CONFIG_H_
#define _WAVESHARE_RP2350TOUCHAMOLED18_CONFIG_H_

#include "stdio.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/pll.h"
#include "hardware/clocks.h"

#define PLL_SYS_KHZ (270 * 1000)

#define SENSOR_I2C_PORT (i2c1)

#define SENSOR_SDA_PIN  (6)
#define SENSOR_SCL_PIN  (7)

#define TP_SDA_PIN      (6)
#define TP_SCL_PIN      (7)
#define TP_INT_PIN      (21)
#define TP_RST_PIN      (22)

#define TOUCH_INT_PIN   (21)
#define TOUCH_RST_PIN   (22)

#define QMI_SDA_PIN     (6)
#define QMI_SCL_PIN     (7)
#define QMI_INT1        (23)
#define QMI_INT2        (8)

#define DOF_INT1        (23)
#define DOF_INT2        (8)

#define RTC_SDA_PIN     (6)
#define RTC_SCL_PIN     (7)
#define RTC_INT_PIN     (18)

#define LCD_CS_PIN      (9)
#define LCD_RESET_PIN   (13)
#define LCD_TE_PIN      (17)
#define DSI_PWR_EN_PIN  (16)

#define QSPI_SCL_PIN    (10)
#define QSPI_SIO0_PIN   (11)
#define QSPI_SI1_PIN    (12)
#define QSPI_SI2_PIN    (14)
#define QSPI_SI3_PIN    (15)

#define I2S_SCLK_PIN    (20)
#define I2S_LRCK_PIN    (24)
#define I2S_DSDIN_PIN   (22)
#define I2S_ASDOUT_PIN  (7)
#define I2S_MCLK_PIN    (6)

#define CODEC_SDA_PIN   (6)
#define CODEC_SCL_PIN   (7)
#define CODEC_CE_PIN    (21)

#define PA_CTRL_PIN     (19)

#define SDCS_PIN        (25)
#define SD_MOSI_PIN     (26)
#define SD_MISO_PIN     (27)
#define SD_SCK_PIN      (28)

#define BAT_ADC_PIN     (29)
#define BAT_CHANNEL     (3)

#define PWR_KEY_PIN     (18)

#define AXP_IRQ_PIN     (2)
#define AXP_SDA_PIN     (6)
#define AXP_SCL_PIN     (7)

extern uint dma_tx;
extern dma_channel_config c;

void Device_Module_Init();

#endif /* _WAVESHARE_RP2350TOUCHAMOLED18_CONFIG_H_ */
