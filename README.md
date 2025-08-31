# Waveshare Drivers Library
Copy of Waveshare Code to be used as a library...

#### Waveshare
You can find information on the [RP2350-Touch-LCD-1.28 here](https://www.waveshare.com/wiki/RP2350-Touch-LCD-1.28). I grabbed the [LVGL](https://files.waveshare.com/wiki/RP2350-Touch-LCD-1.28/RP2350-Touch-LCD-1.28-LVGL.zip) version since I will be using LVGL with my projects. None of the LVGL code is here, you would need to grab it from [LVGL](https://github.com/lvgl/lvgl)

#### CMAKE Notes
if you are going to include this to your project via `git` you chould include the following in your cmake.
```cmake
# Add compilation subdirectory
add_subdirectory(./lib/WaveShare)
add_subdirectory(./lib/GC9A01A)
add_subdirectory(./lib/CST816S)
add_subdirectory(./lib/QMI8658)

include_directories(./lib/WaveShare)
include_directories(./lib/GC9A01A)
include_directories(./lib/CST816S)
include_directories(./lib/QMI8658)

target_link_libraries(WaveshareDriversLibrary
                      WaveShare
                      GC9A01A
                      CST816S
                      QMI8658
                      pico_stdlib
                      hardware_spi
                      hardware_i2c
                      hardware_dma
                      )
```
