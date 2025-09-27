# Search for all source files in the current directory
# and save the names to the DIR_examples_SRCS variable
aux_source_directory(. DIR_PCF85063A_SRCS)

# Add header file directory
# include_directories(../WaveShare)

# Generate link library
add_library(PCF85063A ${DIR_PCF85063A_SRCS})
target_link_libraries(PCF85063A PUBLIC pico_stdlib hardware_gpio hardware_spi hardware_pwm hardware_dma hardware_adc hardware_i2c hardware_pll hardware_clocks)