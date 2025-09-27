# Search for all source files in the current directory
# and save the names to the DIR_LCD_SRCS variable
aux_source_directory(. DIR_ST7789V2_SRCS)

# Add header file directory
# include_directories(../Config)

# Generate link library
add_library(ST7789V2 ${DIR_ST7789V2_SRCS})
target_link_libraries(ST7789V2 PUBLIC pico_stdlib hardware_spi hardware_gpio)