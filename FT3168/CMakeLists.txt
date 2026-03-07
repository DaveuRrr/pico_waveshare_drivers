# Search for all source files in the current directory
# and save the names to the DIR_Touch_SRCS variable
aux_source_directory(. DIR_FT3168_SRCS)

include_directories(../WaveShare)

# Generate link library
add_library(FT3168 ${DIR_FT3168_SRCS})
target_link_libraries(FT3168 PUBLIC WaveShare)