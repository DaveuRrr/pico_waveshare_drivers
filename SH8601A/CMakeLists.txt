# Search for all source files in the current directory
# and save the names to the DIR_SH8601A_SRCS variable
aux_source_directory(. DIR_SH8601A_SRCS)

include_directories(../WaveShare)
include_directories(../QSPI_PIO)

# Adds a library target 
add_library(SH8601A ${DIR_SH8601A_SRCS})
target_link_libraries(SH8601A PUBLIC QSPI_PIO WaveShare pico_stdlib hardware_gpio)