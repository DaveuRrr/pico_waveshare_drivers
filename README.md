# Waveshare Drivers Library
Copy of Waveshare Code to be used as a library...

#### Waveshare
You can find information on the [RP2350-Touch-LCD-1.28 here](https://www.waveshare.com/wiki/RP2350-Touch-LCD-1.28). I grabbed the [LVGL](https://files.waveshare.com/wiki/RP2350-Touch-LCD-1.28/RP2350-Touch-LCD-1.28-LVGL.zip) version since I will be using LVGL with my projects. None of the LVGL code is here, you would need to grab it from [LVGL](https://github.com/lvgl/lvgl)

#### CMAKE Notes
if you are going to include this to your project via `git` you chould include the following in your cmake.
```cmake
# Add compilation subdirectory
add_subdirectory(./lib/config)
add_subdirectory(./lib/gc9a01a)
add_subdirectory(./lib/touch)
add_subdirectory(./lib/qmi8658)

include_directories(./lib/config)
include_directories(./lib/gc9a01a)
include_directories(./lib/touch)
include_directories(./lib/qmi8658)

target_link_libraries(waveshare_drivers_library
                      config
                      gc9a01a 
                      touch
                      qmi8658
                      pico_stdlib
                      hardware_spi
                      hardware_i2c
                      hardware_dma
                      )
```
