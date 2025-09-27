#include "RP2350LCD128.h"


uint slice_num;
uint dma_tx;
dma_channel_config c;

void Device_Module_Init()
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

    // GPIO Config (GPIO_OUT 1, GPIO_IN 0)
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
    
    // PWM Configuration
    gpio_set_function(LCD_BL_PIN, GPIO_FUNC_PWM);
    GC9A01A_SLICE_NUM = pwm_gpio_to_slice_num(LCD_BL_PIN);
    pwm_set_wrap(GC9A01A_SLICE_NUM, 100);
    pwm_set_chan_level(GC9A01A_SLICE_NUM, PWM_CHAN_B, 0);
    pwm_set_clkdiv(GC9A01A_SLICE_NUM, 50);
    pwm_set_enabled(GC9A01A_SLICE_NUM, true);
    
    // SPI Config
    spi_init(LCD_SPI_PORT, 270000 * 1000);
    gpio_set_function(LCD_CLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(LCD_MOSI_PIN, GPIO_FUNC_SPI);
    
    // DMA Configuration
    GC9A01A_DMA_TX = dma_claim_unused_channel(true);
    GC9A01A_DMA_CONFIG = dma_channel_get_default_config(GC9A01A_DMA_TX);
    channel_config_set_transfer_data_size(&GC9A01A_DMA_CONFIG, DMA_SIZE_8); 
    channel_config_set_dreq(&GC9A01A_DMA_CONFIG, spi_get_dreq(LCD_SPI_PORT, true));
    
    // I2C Config
    i2c_init(SENSOR_I2C_PORT, 400 * 1000);
    gpio_set_function(SENSOR_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SENSOR_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SENSOR_SDA_PIN);
    gpio_pull_up(SENSOR_SCL_PIN);
    
    // IRQ Config
    gpio_init(TOUCH_INT_PIN);
    gpio_pull_up(TOUCH_INT_PIN);
    gpio_set_dir(TOUCH_INT_PIN, GPIO_IN);
}