#include "RP2350TouchLCD169.h"

uint bl_slice_num;
uint beep_slice_num;
uint dma_tx;
dma_channel_config c;

int main() 
{
    // Clock Config
    set_sys_clock_khz(PLL_SYS_KHZ, true);
    clock_configure(
        clk_peri,
        0,                                                
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, 
        PLL_SYS_KHZ * 1000,                               
        PLL_SYS_KHZ * 1000                              
    );

    stdio_init_all();

    // GPIO Config
    ST7789V2_GPIO(LCD_RST_PIN, 1);
	ST7789V2_GPIO(LCD_DC_PIN, 1);
    ST7789V2_GPIO(LCD_CS_PIN, 1);
    ST7789V2_GPIO(LCD_BL_PIN, 1);

    gpio_put(LCD_CS_PIN, 1);
	gpio_put(LCD_DC_PIN, 0);
    // gpio_put(LCD_BL_PIN, 1);

    CST816S_GPIO(TOUCH_RST_PIN, 1);
    DEV_GPIO(BAT_PWR_PIN, 1);
    DEV_GPIO(BEEP_PIN, 1);

    gpio_put(TOUCH_RST_PIN, 1);
    gpio_put(LCD_DC_PIN, 0);
    gpio_put(BAT_PWR_PIN, 1);

    // ADC
    adc_init();
    adc_gpio_init(BAT_ADC_PIN);
    adc_select_input(BAT_CHANNEL);

    // PWM Config -- LED_BL
    gpio_set_function(LCD_BL_PIN, GPIO_FUNC_PWM);
    bl_slice_num = pwm_gpio_to_slice_num(LCD_BL_PIN);
    pwm_set_wrap(bl_slice_num, 100);
    pwm_set_chan_level(bl_slice_num, PWM_CHAN_B, 60);
    pwm_set_clkdiv(bl_slice_num, 50);
    pwm_set_enabled(bl_slice_num, true);

    // PWM Config -- BEEP
    gpio_set_function(BEEP_PIN, GPIO_FUNC_PWM);
    beep_slice_num = pwm_gpio_to_slice_num(BEEP_PIN);
    pwm_set_wrap(beep_slice_num, 2000);
    pwm_set_chan_level(beep_slice_num, PWM_CHAN_A, 1000);
    pwm_set_clkdiv(beep_slice_num, 200);
    pwm_set_enabled(beep_slice_num, false);

    // SPI Config
    spi_init(LCD_SPI_PORT, 200 * 1000 * 1000);
    gpio_set_function(LCD_CLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(LCD_MOSI_PIN, GPIO_FUNC_SPI);

    // DMA Config
    dma_tx = dma_claim_unused_channel(true);
    c = dma_channel_get_default_config(dma_tx);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8); 
    channel_config_set_dreq(&c, spi_get_dreq(LCD_SPI_PORT, true));

    // I2C Config
    i2c_init(SENSOR_I2C_PORT, 400 * 1000);
    gpio_set_function(DEV_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(DEV_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(DEV_SDA_PIN);
    gpio_pull_up(DEV_SCL_PIN);

    // IRQ Config
    gpio_init(TOUCH_INT_PIN);
    gpio_pull_up(TOUCH_INT_PIN);
    gpio_set_dir(TOUCH_INT_PIN, GPIO_IN);

    gpio_init(PWR_KEY_PIN);
    gpio_pull_up(PWR_KEY_PIN);
    gpio_set_dir(PWR_KEY_PIN, GPIO_IN);

    gpio_init(RTC_INT_PIN);
    gpio_pull_up(RTC_INT_PIN);
    gpio_set_dir(RTC_INT_PIN, GPIO_IN);
}