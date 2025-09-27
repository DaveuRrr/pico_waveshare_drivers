# Search for all source files in the current directory
# and save the names to the DIR_Touch_SRCS variable
aux_source_directory(. DIR_CST816S_SRCS)

#include_directories(../WaveShare)

# Generate link library
add_library(CST816S ${DIR_CST816S_SRCS})
target_link_libraries(CST816S PUBLIC pico_stdlib hardware_i2c hardware_gpio)