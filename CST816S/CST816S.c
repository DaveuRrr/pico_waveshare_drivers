/*****************************************************************************
* | File      	:   CST816S.c
* | Author      :   Waveshare team, Modified by Dave uRrr
* | Function    :   Hardware underlying interface
* | Info        :   Used to shield the underlying layers of each master and enhance portability
*----------------
* | This version:   V1.1
* | Date        :   2025-09-23
* | Info        :   Removed WS_Config.h dependency, requires user-defined pin macros
*
 ******************************************************************************/

#include "CST816S.h"

CST816S Touch_CTS816;

/********************************************************************************
 * @brief           Writes a value to CST816S I2C register
 * @param address   I2C register address to write to
 * @param value     Value to write to the register
********************************************************************************/
void CST816S_I2C_Write(uint8_t address, uint8_t value)
{
    uint8_t data[2] = {address, value};
    i2c_write_blocking(SENSOR_I2C_PORT, CST816_ADDR, data, 2, false);
}

/********************************************************************************
 * @brief           Reads a value from CST816S I2C register
 * @param address   I2C register address to read from
 * @return          Value read from the register
********************************************************************************/
uint8_t CST816S_I2C_Read(uint8_t address)
{
    uint8_t result;
    i2c_write_blocking(SENSOR_I2C_PORT, CST816_ADDR, &address, 1, true);
    i2c_read_blocking(SENSOR_I2C_PORT, CST816_ADDR, &result, 1, false);
    return result;
}

/********************************************************************************
 * @brief           Reads buffer from CST816S I2C register
 * @param address   I2C register address to read from
********************************************************************************/
void CST816S_I2C_Read_Buffer(uint8_t address, uint8_t *buffer, uint32_t len)
{
    i2c_write_blocking(SENSOR_I2C_PORT, CST816_ADDR, &address, 1, true);
    i2c_read_blocking(SENSOR_I2C_PORT, CST816_ADDR, buffer, len, false);
}

/********************************************************************************
 * @brief           Checks if CST816S chip is detected by reading chip ID
 * @return          true if chip detected, false otherwise
********************************************************************************/
uint8_t CST816S_WhoAmI()
{
    if (CST816S_I2C_Read(CST816_ChipID) == 0xB5) return true;
    else return false;
}

/********************************************************************************
 * @brief           Hardware reset of CST816S touch controller
********************************************************************************/
void CST816S_Reset()
{
    gpio_put(TOUCH_RST_PIN, 0);
    sleep_ms(100);
    gpio_put(TOUCH_RST_PIN, 1);
    sleep_ms(100);
}

/********************************************************************************
 * @brief           Reads the firmware version/revision of CST816S
 * @return          Firmware version number
********************************************************************************/
uint8_t CST816S_Read_Revision()
{
    return CST816S_I2C_Read(CST816_FwVersion);
}

/********************************************************************************
 * @brief           Wakes up CST816S from sleep mode
********************************************************************************/
void CST816S_Wake_up()
{
    gpio_put(TOUCH_RST_PIN, 0);
    sleep_ms(10);
    gpio_put(TOUCH_RST_PIN, 1);
    sleep_ms(50);
    CST816S_I2C_Write(CST816_DisAutoSleep, 0x01);
}

/********************************************************************************
 * @brief           Disables auto-sleep mode on CST816S
********************************************************************************/
void CST816S_Stop_Sleep()
{
    CST816S_I2C_Write(CST816_DisAutoSleep, 0x01);
}

/********************************************************************************
 * @brief           Sets the operating mode of CST816S
 * @param mode      Operating mode (Point, Gesture, or All modes)
********************************************************************************/
void CST816S_Set_Mode(uint8_t mode)
{
    if (mode == CST816S_Point_Mode)
    {
        // 
        CST816S_I2C_Write(CST816_IrqCtl, 0x41);    
        Touch_CTS816.x_point = 0;
        Touch_CTS816.y_point = 0;
        Touch_CTS816.mode = mode;
    }
    else if (mode == CST816S_Gesture_Mode)
    {
        CST816S_I2C_Write(CST816_IrqCtl, 0X11);
        CST816S_I2C_Write(CST816_MotionMask, 0x01);
        Touch_CTS816.x_point = 0;
        Touch_CTS816.y_point = 0;
        Touch_CTS816.mode = mode;
    }
    else
    {
        CST816S_I2C_Write(CST816_IrqCtl, 0X71);
    }
        
}

/********************************************************************************
 * @brief           Initializes CST816S touch controller
 * @param mode      Operating mode to set after initialization
 * @return          true if initialization successful, false otherwise
********************************************************************************/
uint8_t CST816S_init(uint8_t mode)
{
    // Checks if I2C is up
    if (i2c_get_baudrate(SENSOR_I2C_PORT) == 0)
    {
        // I2C Config
        i2c_init(SENSOR_I2C_PORT, 400 * 1000);
        gpio_set_function(SENSOR_SDA_PIN, GPIO_FUNC_I2C);
        gpio_set_function(SENSOR_SCL_PIN, GPIO_FUNC_I2C);
        gpio_pull_up(SENSOR_SDA_PIN);
        gpio_pull_up(SENSOR_SCL_PIN);
    }

    // IRQ Config
    gpio_init(TOUCH_INT_PIN);
    gpio_pull_up(TOUCH_INT_PIN);
    gpio_set_dir(TOUCH_INT_PIN, GPIO_IN);

    uint8_t bRet, Rev;
    CST816S_Reset();
    bRet = CST816S_WhoAmI();
    if (bRet)
    {
        printf("Success:Detected CST816T.\r\n");
        Rev = CST816S_Read_Revision();
        printf("CST816T Revision = %d\r\n", Rev);
        CST816S_Stop_Sleep();
    }
    else
    {
        printf("Error: Not Detected CST816T.\r\n");
        return false;
    }

    CST816S_Set_Mode(mode);
    Touch_CTS816.x_point = 0;
    Touch_CTS816.y_point = 0;
    CST816S_I2C_Write(CST816_IrqPluseWidth, 0x01);
    CST816S_I2C_Write(CST816_NorScanPer, 0x01);
    
    Touch_CTS816.mode = mode;

    return true;
}

/********************************************************************************
 * @brief           Reads current touch point coordinates
 * @return          CST816S struct containing x,y coordinates and mode
********************************************************************************/
CST816S CST816S_Get_Point()
{
    uint8_t x_point_h, x_point_l, y_point_h, y_point_l;
    // CST816S_Wake_up();

    x_point_h = CST816S_I2C_Read(CST816_XposH);
    x_point_l = CST816S_I2C_Read(CST816_XposL);
    y_point_h = CST816S_I2C_Read(CST816_YposH);
    y_point_l = CST816S_I2C_Read(CST816_YposL);

    Touch_CTS816.x_point = ((x_point_h & 0x0f) << 8) + x_point_l;
    Touch_CTS816.y_point = ((y_point_h & 0x0f) << 8) + y_point_l;

    return Touch_CTS816;
}

/********************************************************************************
 * @brief           Reads current gesture from CST816S
 * @return          Gesture ID (Up, Down, Left, Right, Click, etc.)
********************************************************************************/
uint8_t CST816S_Get_Gesture(void)
{
    uint8_t gesture;
    gesture = CST816S_I2C_Read(CST816_GestureID);
    return gesture;
}
