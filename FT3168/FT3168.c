/*****************************************************************************
* | File      	:   FT3168.c
* | Author      :   Waveshare Team, Modified by Dave uRrr
* | Function    :   FT3168 Interface Functions
* | Info        :   Used to shield the underlying layers of each master and enhance portability
*----------------
* | This version:   V1.1
* | Date        :   2026-02-25
* | Info        :   Removed DEV_Config.h dependency, requires user-defined pin macros
*
******************************************************************************/

#include "FT3168.h"

FT3168_ATTRIBUTES FT3168;

/********************************************************************************
 * @brief           Writes a value to FT3168 I2C register
 * @param address   I2C register address to write to
 * @param value     Value to write to the register
********************************************************************************/
void FT3168_I2C_Write(uint8_t address, uint8_t value)
{
    uint8_t data[2] = {address, value};
    i2c_write_blocking(SENSOR_I2C_PORT, FT3168_ADDR, data, 2, false);
}

/********************************************************************************
 * @brief           Reads a value from FT3168 I2C register
 * @param address   I2C register address to read from
 * @return          Value read from the register
********************************************************************************/
uint8_t FT3168_I2C_Read(uint8_t address)
{
    uint8_t result;
    i2c_write_blocking(SENSOR_I2C_PORT, FT3168_ADDR, &address, 1, true);
    i2c_read_blocking(SENSOR_I2C_PORT, FT3168_ADDR, &result, 1, false);
    return result;
}

/********************************************************************************
 * @brief           Reads buffer from FT3168 I2C register
 * @param address   I2C register address to read from
 * @param buffer    Buffer to store read data
 * @param len       Number of bytes to read
********************************************************************************/
void FT3168_I2C_Read_Buffer(uint8_t address, uint8_t *buffer, uint32_t len)
{
    i2c_write_blocking(SENSOR_I2C_PORT, FT3168_ADDR, &address, 1, true);
    i2c_read_blocking(SENSOR_I2C_PORT, FT3168_ADDR, buffer, len, false);
}

/********************************************************************************
 * @brief           Hardware reset of FT3168 touch controller
********************************************************************************/
void FT3168_Reset()
{
    gpio_put(TOUCH_RST_PIN, 0);
    sleep_ms(100);
    gpio_put(TOUCH_RST_PIN, 1);
    sleep_ms(100);
}

/********************************************************************************
 * @brief           Initializes FT3168 touch controller
 * @param mode      Operating mode to set after initialization
 * @return          true if initialization successful, false otherwise
********************************************************************************/
uint8_t FT3168_Init(uint8_t mode)
{
    // IRQ Config
    gpio_init(TOUCH_INT_PIN);
    gpio_pull_up(TOUCH_INT_PIN);
    gpio_set_dir(TOUCH_INT_PIN, GPIO_IN);

    uint8_t bRet, Rev;
    FT3168_Reset();
    bRet = FT3168_WhoAmI();
    if (bRet)
    {
        printf("Success: Detected FT3168.\r\n");
        Rev = FT3168_Read_Revision();
        printf("FT3168 Firmware Revision = %d\r\n", Rev);
        FT3168_Stop_Sleep();
    }
    else
    {
        printf("Error: Not Detected FT3168.\r\n");
        return false;
    }

    FT3168_Set_Mode(mode);
    FT3168.x_point = 0;
    FT3168.y_point = 0;
    FT3168.mode = mode;

    printf("FT3168 initialized successfully\n");
    return true;
}

/********************************************************************************
 * @brief           Checks if FT3168 chip is detected by reading chip ID
 * @return          true if chip detected, false otherwise
********************************************************************************/
uint8_t FT3168_WhoAmI()
{
    if (FT3168_I2C_Read(FT3168_ChipID) == 0x03) return true;
    else return false;
}

/********************************************************************************
 * @brief           Reads the firmware version/revision of FT3168
 * @return          Firmware version number
********************************************************************************/
uint8_t FT3168_Read_Revision()
{
    return FT3168_I2C_Read(FT3168_FwVersion);
}

/********************************************************************************
 * @brief           Wakes up FT3168 from sleep mode
********************************************************************************/
void FT3168_Wake_Up()
{
    gpio_put(TOUCH_RST_PIN, 0);
    sleep_ms(10);
    gpio_put(TOUCH_RST_PIN, 1);
    sleep_ms(50);
    FT3168_I2C_Write(FT3168_PowerMode, FT3168_POWER_ACTIVE);
}

/********************************************************************************
 * @brief           Disables auto-sleep mode on FT3168
********************************************************************************/
void FT3168_Stop_Sleep()
{
    FT3168_I2C_Write(FT3168_PowerMode, FT3168_POWER_ACTIVE);
}

/********************************************************************************
 * @brief           Sets the operating mode of FT3168
 * @param mode      Operating mode (Point or Gesture modes)
********************************************************************************/
void FT3168_Set_Mode(uint8_t mode)
{
    if (mode == FT3168_Point_Mode)
    {
        FT3168_I2C_Write(FT3168_GestureIDMode, 0x00);
        FT3168_I2C_Write(FT3168_IrqMode, 0x01);
        FT3168.x_point = 0;
        FT3168.y_point = 0;
        FT3168.mode = FT3168_Point_Mode;
    }
    else if (mode == FT3168_Gesture_Mode)
    {
        FT3168_I2C_Write(FT3168_GestureIDMode, 0x01);
        FT3168_I2C_Write(FT3168_IrqMode, 0x01);
        FT3168.x_point = 0;
        FT3168.y_point = 0;
        FT3168.mode = FT3168_Gesture_Mode;
    }
}

/********************************************************************************
 * @brief           Reads current touch point coordinates
 * @param rotation  Screen rotation (ROTATION_0, 90, 180, 270)
 * @param width     Screen width in pixels
 * @param height    Screen height in pixels
 * @return          FT3168_Struct containing x,y coordinates and mode
********************************************************************************/
FT3168_ATTRIBUTES FT3168_Get_Point(uint8_t rotation, uint16_t width, uint16_t height)
{
    uint8_t x_point_h = FT3168_I2C_Read(FT3168_XposH);
    uint8_t x_point_l = FT3168_I2C_Read(FT3168_XposL);
    uint8_t y_point_h = FT3168_I2C_Read(FT3168_YposH);
    uint8_t y_point_l = FT3168_I2C_Read(FT3168_YposL);

    uint16_t x = ((x_point_h & 0x0f) << 8) + x_point_l;
    uint16_t y = ((y_point_h & 0x0f) << 8) + y_point_l;

    uint16_t x_point, y_point;

    switch(rotation)
    {
        case ROTATION_0:    // (x, y) => (x, y)
            x_point = x;
            y_point = y;
            break;

        case ROTATION_90:   // (x, y) => (-y, x)
            x_point = y;
            y_point = width - x - 1;
            break;

        case ROTATION_180:  // (x, y) => (-x, -y)
            x_point = width - x - 1;
            y_point = height - y - 1;
            break;

        case ROTATION_270:  // (x, y) => (y, -x)
            x_point = height - y - 1;
            y_point = x;
            break;
    }

    FT3168.x_point = x_point;
    FT3168.y_point = y_point;

    return FT3168;
}

/********************************************************************************
 * @brief           Reads current gesture from FT3168
 * @return          Gesture ID (Up, Down, Left, Right, Click, etc.)
********************************************************************************/
uint8_t FT3168_Get_Gesture(void)
{
    uint8_t gesture;
    gesture = FT3168_I2C_Read(FT3168_GestureID);
    return gesture;
}