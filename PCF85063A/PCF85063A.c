/*****************************************************************************
* | File      	:   PCF85063A.c
* | Author      :   Zildjhan, Modified by Dave uRrr
* | Function    :   Hardware underlying interface
* | Info        :   Used to shield the underlying layers of each master and enhance portability
*----------------
* |	This version:   V1.1
* | Date        :   2025-09-23
* | Info        :   Removed WS_Config.h dependency, requires user-defined pin macros
*
******************************************************************************/

#include "PCF85063A.h"

static uint8_t Decimal_To_Binary(int value);
static int Binary_To_Decimal(uint8_t value);

const unsigned char MonthStr[12][4] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov","Dec"};

/********************************************************************************
 * @brief           Converts decimal number to Binary Coded Decimal (BCD)
 * @param value     Decimal value to convert (0-99)
 * @return          BCD representation of the decimal value
********************************************************************************/
static uint8_t Decimal_To_Binary(int value)
{
	return (uint8_t)((value / 10 * 16) + (value % 10));
}

/********************************************************************************
 * @brief           Converts Binary Coded Decimal (BCD) to decimal number
 * @param value     BCD value to convert
 * @return          Decimal representation of the BCD value
********************************************************************************/
static int Binary_To_Decimal(uint8_t value)
{
    return (int)((value / 16 * 10) + (value % 16));
}

/********************************************************************************
 * @brief           Writes a value to PCF85063A I2C register
 * @param address   I2C register address to write to
 * @param value     Value to write to the register
********************************************************************************/
void PCF85063A_I2C_Write(uint8_t address, uint8_t value)
{
    uint8_t data[2] = {address, value};
    i2c_write_blocking(SENSOR_I2C_PORT, PCF85063A_ADDRESS, data, 2, false);
}

/********************************************************************************
 * @brief           Reads a value from PCF85063A I2C register
 * @param address   I2C register address to read from
 * @return          Value read from the register
********************************************************************************/
uint8_t PCF85063A_I2C_Read(uint8_t address)
{
    uint8_t result;
    i2c_write_blocking(SENSOR_I2C_PORT, PCF85063A_ADDRESS, &address, 1, true);
    i2c_read_blocking(SENSOR_I2C_PORT, PCF85063A_ADDRESS, &result, 1, false);
    return result;
}

/********************************************************************************
 * @brief           Reads buffer from PCF85063A I2C register
 * @param address   I2C register address to read from
********************************************************************************/
void PCF85063A_I2C_Read_Buffer(uint8_t address, uint8_t *buffer, uint32_t len)
{
    i2c_write_blocking(SENSOR_I2C_PORT, PCF85063A_ADDRESS, &address, 1, true);
    i2c_read_blocking(SENSOR_I2C_PORT, PCF85063A_ADDRESS, buffer, len, false);
}

/********************************************************************************
 * @brief           Initializes PCF85063A RTC in normal mode with 24-hour format
********************************************************************************/
void PCF85063A_Init()
{
    // IRQ Config
    gpio_init(RTC_INT_PIN);
    gpio_pull_up(RTC_INT_PIN);
    gpio_set_dir(RTC_INT_PIN, GPIO_IN);

    // Initiate Normal Mode, RTC Run, NO reset, No correction , 24hr format, Internal load capacitane 12.5pf
    uint8_t value = RTC_CTRL_1_DEFAULT|RTC_CTRL_1_CAP_SEL;
    PCF85063A_I2C_Write(RTC_CTRL_1_ADDR, value);
}

/********************************************************************************
 * @brief           Performs software reset of PCF85063A RTC
********************************************************************************/
void PCF85063A_Reset()
{
    uint8_t value = RTC_CTRL_1_DEFAULT|RTC_CTRL_1_CAP_SEL|RTC_CTRL_1_SR;
    PCF85063A_I2C_Write(RTC_CTRL_1_ADDR, value);
}

/********************************************************************************
 * @brief           Sets the current time on PCF85063A RTC
 * @param time      datetime_t structure containing hour, minute, second
********************************************************************************/
void PCF85063A_Set_Time(datetime_t time)
{
    uint8_t buffer[4] = {RTC_SECOND_ADDR,
                        Decimal_To_Binary(time.sec),
                        Decimal_To_Binary(time.min),
                        Decimal_To_Binary(time.hour)};
    i2c_write_blocking(SENSOR_I2C_PORT, PCF85063A_ADDRESS, buffer, 4, false);
}

/********************************************************************************
 * @brief           Sets the current date on PCF85063A RTC
 * @param time      datetime_t structure containing day, day-of-week, month, year
********************************************************************************/
void PCF85063A_Set_Date(datetime_t time)
{
    uint8_t buffer[5] = {RTC_DAY_ADDR,
                        Decimal_To_Binary(time.day),
                        Decimal_To_Binary(time.dotw),
                        Decimal_To_Binary(time.month),
                        Decimal_To_Binary(time.year - YEAR_OFFSET)};
    i2c_write_blocking(SENSOR_I2C_PORT, PCF85063A_ADDRESS, buffer, 5, false);
}

/********************************************************************************
 * @brief           Sets both date and time on PCF85063A RTC in one operation
 * @param time      datetime_t structure containing all date/time fields
********************************************************************************/
void PCF85063A_Set_All(datetime_t time)
{
    uint8_t buffer[8] = {RTC_SECOND_ADDR,
                        Decimal_To_Binary(time.sec),
                        Decimal_To_Binary(time.min),
                        Decimal_To_Binary(time.hour),
                        Decimal_To_Binary(time.day),
                        Decimal_To_Binary(time.dotw),
                        Decimal_To_Binary(time.month),
                        Decimal_To_Binary(time.year - YEAR_OFFSET)};
    i2c_write_blocking(SENSOR_I2C_PORT, PCF85063A_ADDRESS, buffer, 8, false);
}

/********************************************************************************
 * @brief           Reads current date and time from PCF85063A RTC
 * @param time      Pointer to datetime_t structure to store read values
********************************************************************************/
void PCF85063A_Read_Now(datetime_t *time)
{
    uint8_t bufss[7] = {0};

    uint8_t reg_addr = RTC_SECOND_ADDR;
    i2c_write_blocking(SENSOR_I2C_PORT, PCF85063A_ADDRESS, &reg_addr, 1, true);
    i2c_read_blocking(SENSOR_I2C_PORT, PCF85063A_ADDRESS, bufss, 7, false);

    time->sec = Binary_To_Decimal(bufss[0] & 0x7F);
    time->min = Binary_To_Decimal(bufss[1] & 0x7F);
    time->hour = Binary_To_Decimal(bufss[2] & 0x3F);
    time->day = Binary_To_Decimal(bufss[3] & 0x3F);
    time->dotw = Binary_To_Decimal(bufss[4] & 0x07);
    time->month = Binary_To_Decimal(bufss[5] & 0x1F);
    time->year = Binary_To_Decimal(bufss[6])+YEAR_OFFSET;
}

/********************************************************************************
 * @brief           Enables alarm interrupt and clears alarm flag
********************************************************************************/
void PCF85063A_Enable_Alarm()
{
    uint8_t value = RTC_CTRL_2_DEFAULT | RTC_CTRL_2_AIE;
    value &= ~RTC_CTRL_2_AF;
    PCF85063A_I2C_Write(RTC_CTRL_2_ADDR, value);
}

/********************************************************************************
 * @brief           Gets the current alarm flag status
 * @return          Non-zero if alarm flag is set, zero otherwise
********************************************************************************/
uint8_t PCF85063A_Get_Alarm_Flag()
{
    uint8_t value = PCF85063A_I2C_Read(RTC_CTRL_2_ADDR);
    value &= RTC_CTRL_2_AF;
    return value;
}

/********************************************************************************
 * @brief           Clears the alarm flag
********************************************************************************/
void PCF85063A_Clean_Alarm_Flag()
{
    uint8_t value = PCF85063A_I2C_Read(RTC_CTRL_2_ADDR);
    value &= (~RTC_CTRL_2_AF);
    PCF85063A_I2C_Write(RTC_CTRL_2_ADDR, value);
}

/********************************************************************************
 * @brief           Sets alarm time (seconds, minutes, hours only)
 * @param time      datetime_t structure containing alarm time values
********************************************************************************/
void PCF85063A_Set_Alarm(datetime_t time)
{
    uint8_t buffer[6] ={RTC_SECOND_ALARM,
                        Decimal_To_Binary(time.sec)&(~RTC_ALARM),
                        Decimal_To_Binary(time.min)&(~RTC_ALARM),
                        Decimal_To_Binary(time.hour)&(~RTC_ALARM),
                        //Decimal_To_Binary(time.day)&(~RTC_ALARM),
                        //Decimal_To_Binary(time.dotw)&(~RTC_ALARM)
                        RTC_ALARM, 	//disalbe day
                        RTC_ALARM	//disalbe weekday
                        };
    i2c_write_blocking(SENSOR_I2C_PORT, PCF85063A_ADDRESS, buffer, 6, false);
}

/********************************************************************************
 * @brief           Reads current alarm settings from PCF85063A RTC
 * @param time      Pointer to datetime_t structure to store alarm values
********************************************************************************/
void PCF85063A_Read_Alarm(datetime_t *time)
{
    uint8_t bufss[5] = {0};

    uint8_t alarm_reg_addr = RTC_SECOND_ALARM;
    i2c_write_blocking(SENSOR_I2C_PORT, PCF85063A_ADDRESS, &alarm_reg_addr, 1, true);
    i2c_read_blocking(SENSOR_I2C_PORT, PCF85063A_ADDRESS, bufss, 5, false);

    time->sec = Binary_To_Decimal(bufss[0] & 0x7F);
    time->min = Binary_To_Decimal(bufss[1] & 0x7F);
    time->hour = Binary_To_Decimal(bufss[2] & 0x3F);
    time->day = Binary_To_Decimal(bufss[3] & 0x3F);
    time->dotw = Binary_To_Decimal(bufss[4] & 0x07);
}