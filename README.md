# Pico WaveShare Drivers
A copy of WaveShare Code for their components to be used as a library. Original code contains an MIT license

#### Waveshare
You can find information on the [RP2350-Touch-LCD-1.28 here](https://www.waveshare.com/wiki/RP2350-Touch-LCD-1.28). I grabbed the [LVGL](https://files.waveshare.com/wiki/RP2350-Touch-LCD-1.28/RP2350-Touch-LCD-1.28-LVGL.zip) version since I will be using LVGL with my projects. None of the LVGL code is here, you would need to grab it from [LVGL](https://github.com/lvgl/lvgl)

#### CMAKE Notes
if you are going to include this to your project via `git` you chould include the following in your cmake.
```cmake
  # Compilation subdirectory for Waveshare
  add_subdirectory(../../lib/pico_waveshare_drivers/GC9A01A GC9A01A)
  add_subdirectory(../../lib/pico_waveshare_drivers/CST816S CST816S)
  add_subdirectory(../../lib/pico_waveshare_drivers/QMI8658 QMI8658)

  # Board Configurations ...
  include_directories(${CMAKE_CURRENT_LIST_DIR}) # board_resources.h
  
  # Make board_resources.h available to all libraries
  target_include_directories(GC9A01A PRIVATE ${CMAKE_CURRENT_LIST_DIR})
  target_include_directories(CST816S PRIVATE ${CMAKE_CURRENT_LIST_DIR})
  target_include_directories(QMI8658 PRIVATE ${CMAKE_CURRENT_LIST_DIR})
  include_directories(../../lib/pico_waveshare_drivers/GC9A01A)
  include_directories(../../lib/pico_waveshare_drivers/CST816S)
  include_directories(../../lib/pico_waveshare_drivers/QMI8658)
```
