#include "RP2350TouchLCD128.h"


uint slice_num;
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
	GC9A01A_GPIO(LCD_RST_PIN, 1);
	GC9A01A_GPIO(LCD_DC_PIN, 1);
    GC9A01A_GPIO(LCD_CS_PIN, 1);
    GC9A01A_GPIO(LCD_BL_PIN, 1);

	gpio_put(LCD_CS_PIN, 1);
	gpio_put(LCD_DC_PIN, 0);
	gpio_put(LCD_BL_PIN, 1);

    // ADC Config
    adc_init();
    adc_gpio_init(BAT_ADC_PIN);
    adc_select_input(BAR_CHANNEL);

    // PWM Config
    gpio_set_function(LCD_BL_PIN, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(LCD_BL_PIN);
    pwm_set_wrap(slice_num, 100);
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
    pwm_set_clkdiv(slice_num, 50);
    pwm_set_enabled(slice_num, true);

    // SPI Config
    spi_init(LCD_SPI_PORT, 270000 * 1000);
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
}