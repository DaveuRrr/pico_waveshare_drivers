# Search for all source files in the current directory
# and save the names to the DIR_QMI8658_SRCS variable
aux_source_directory(. DIR_QMI8658_SRCS)

# include_directories(../WaveShare)

# Generate link library
add_library(QMI8658 ${DIR_QMI8658_SRCS})
target_link_libraries(QMI8658 PUBLIC pico_stdlib hardware_i2c hardware_gpio)