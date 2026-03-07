#include "RP2350TouchAMOLED18.h"

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
    gpio_init(LCD_RESET_PIN);
    gpio_set_dir(LCD_RESET_PIN, GPIO_OUT);
    gpio_init(LCD_CS_PIN);
    gpio_set_dir(LCD_CS_PIN, GPIO_OUT);
    gpio_init(DSI_PWR_EN_PIN);
    gpio_set_dir(DSI_PWR_EN_PIN, GPIO_OUT);
    gpio_init(LCD_TE_PIN);
    gpio_set_dir(LCD_TE_PIN, GPIO_IN);

    gpio_init(TOUCH_RST_PIN);
    gpio_set_dir(TOUCH_RST_PIN, GPIO_OUT);

    gpio_init(CODEC_CE_PIN);
    gpio_set_dir(CODEC_CE_PIN, GPIO_OUT);

    gpio_init(PA_CTRL_PIN);
    gpio_set_dir(PA_CTRL_PIN, GPIO_OUT);

    gpio_put(LCD_CS_PIN, 1);
    gpio_put(LCD_RESET_PIN, 1);
    gpio_put(DSI_PWR_EN_PIN, 1);
    gpio_put(TOUCH_RST_PIN, 1);
    gpio_put(CODEC_CE_PIN, 1);
    gpio_put(PA_CTRL_PIN, 0);

    // ADC Config
    adc_init();
    adc_gpio_init(BAT_ADC_PIN);
    adc_select_input(BAT_CHANNEL);

    // I2C Config (shared by QMI8658, Touch, RTC, Codec, AXP2101)
    i2c_init(SENSOR_I2C_PORT, 400 * 1000);
    gpio_set_function(SENSOR_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SENSOR_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SENSOR_SDA_PIN);
    gpio_pull_up(SENSOR_SCL_PIN);

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

    gpio_init(DOF_INT1);
    gpio_pull_up(DOF_INT1);
    gpio_set_dir(DOF_INT1, GPIO_IN);

    gpio_init(DOF_INT2);
    gpio_pull_up(DOF_INT2);
    gpio_set_dir(DOF_INT2, GPIO_IN);

    gpio_init(AXP_IRQ_PIN);
    gpio_pull_up(AXP_IRQ_PIN);
    gpio_set_dir(AXP_IRQ_PIN, GPIO_IN);
}
