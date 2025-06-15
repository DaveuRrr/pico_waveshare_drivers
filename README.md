# RP2350TouchLCD128
Copy of Waveshare Code to be used as a library...

#### Waveshare
You can find information on the [RP2350-Touch-LCD-1.28 here](https://www.waveshare.com/wiki/RP2350-Touch-LCD-1.28). I grabbed the [LVGL](https://files.waveshare.com/wiki/RP2350-Touch-LCD-1.28/RP2350-Touch-LCD-1.28-LVGL.zip) version since I will be using LVGL with my projects. None of the LVGL code is here, you would need to grab it from [LVGL](https://github.com/lvgl/lvgl)

#### CMAKE Notes
if you are going to include this to your project via `git` you chould include the following in your cmake.
```cmake
# Add compilation subdirectory
add_subdirectory(./lib/Config)
add_subdirectory(./lib/LCD)
add_subdirectory(./lib/Touch)
add_subdirectory(./lib/QMI8658)

target_link_libraries(RP2350-Touch-LCD-1.28  
                      LCD 
                      Touch
                      QMI8658
                      Config
                      pico_stdlib
                      hardware_spi
                      hardware_i2c
                      hardware_dma
                      )
```