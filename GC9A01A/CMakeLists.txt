# Search for all source files in the current directory
# and save the names to the DIR_LCD_SRCS variable
aux_source_directory(. DIR_GC9A01A_SRCS)

# include_directories(../WaveShare)

# Generate link library
add_library(GC9A01A ${DIR_GC9A01A_SRCS})
target_link_libraries(GC9A01A PUBLIC pico_stdlib hardware_gpio hardware_spi hardware_pwm hardware_dma hardware_adc hardware_i2c hardware_pll hardware_clocks)