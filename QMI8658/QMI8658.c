/*****************************************************************************
* | File      	:   QMI8658.c
* | Author      :   Waveshare team, Modified by Dave uRrr
* | Function    :   Hardware underlying interface
* | Info        :	Used to shield the underlying layers of each master and enhance portability
*----------------
* |	This version:   V1.1
* | Date        :   2025-09-23
* | Info        :   Removed WS_Config.h dependency, requires user-defined pin macros
*
 *****************************************************************************/

// #include "stdafx.h"
#include "QMI8658.h"

#define QMI8658_SLAVE_ADDR_L 0x6a
#define QMI8658_SLAVE_ADDR_H 0x6b
#define QMI8658_printf printf

#define QMI8658_UINT_MG_DPS

enum
{
    AXIS_X = 0,
    AXIS_Y = 1,
    AXIS_Z = 2,

    AXIS_TOTAL
};

typedef struct
{
    short sign[AXIS_TOTAL];
    unsigned short map[AXIS_TOTAL];
} qst_imu_layout;

static unsigned short acc_lsb_div = 0;
static unsigned short gyro_lsb_div = 0;
static unsigned short ae_q_lsb_div = (1 << 14);
static unsigned short ae_v_lsb_div = (1 << 10);
static unsigned int imu_timestamp = 0;
static struct QMI8658_Config qmi8658_config;
static unsigned char QMI8658_SLAVE_ADDR = QMI8658_SLAVE_ADDR_L;

/********************************************************************************
 * @brief           Writes a value to QMI8658 I2C register
 * @param address   I2C register address to write to
 * @param value     Value to write to the register
********************************************************************************/
void QMI8658_I2C_Write(uint8_t address, uint8_t value)
{
    uint8_t data[2] = {address, value};
    i2c_write_blocking(SENSOR_I2C_PORT, QMI8658_SLAVE_ADDR, data, 2, false);
}

/********************************************************************************
 * @brief           Reads a value from QMI8658 I2C register
 * @param address   I2C register address to read from
 * @return          Value read from the register
********************************************************************************/
uint8_t QMI8658_I2C_Read(uint8_t address)
{
    uint8_t result;
    i2c_write_blocking(SENSOR_I2C_PORT, QMI8658_SLAVE_ADDR, &address, 1, true);
    i2c_read_blocking(SENSOR_I2C_PORT, QMI8658_SLAVE_ADDR, &result, 1, false);
    return result;
}

/********************************************************************************
 * @brief           Reads buffer from QMI8658 I2C register
 * @param address   I2C register address to read from
 * @param buffer    Buffer to store read data
 * @param len       Number of bytes to read
********************************************************************************/
void QMI8658_I2C_Read_Buffer(uint8_t address, uint8_t *buffer, uint32_t len)
{
    i2c_write_blocking(SENSOR_I2C_PORT, QMI8658_SLAVE_ADDR, &address, 1, true);
    i2c_read_blocking(SENSOR_I2C_PORT, QMI8658_SLAVE_ADDR, buffer, len, false);
}

#if 0
static qst_imu_layout imu_map;

void QMI8658_set_layout(short layout)
{
	if(layout == 0)
	{
		imu_map.sign[AXIS_X] = 1;
		imu_map.sign[AXIS_Y] = 1;
		imu_map.sign[AXIS_Z] = 1;
		imu_map.map[AXIS_X] = AXIS_X;
		imu_map.map[AXIS_Y] = AXIS_Y;
		imu_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 1)
	{
		imu_map.sign[AXIS_X] = -1;
		imu_map.sign[AXIS_Y] = 1;
		imu_map.sign[AXIS_Z] = 1;
		imu_map.map[AXIS_X] = AXIS_Y;
		imu_map.map[AXIS_Y] = AXIS_X;
		imu_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 2)
	{
		imu_map.sign[AXIS_X] = -1;
		imu_map.sign[AXIS_Y] = -1;
		imu_map.sign[AXIS_Z] = 1;
		imu_map.map[AXIS_X] = AXIS_X;
		imu_map.map[AXIS_Y] = AXIS_Y;
		imu_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 3)
	{
		imu_map.sign[AXIS_X] = 1;
		imu_map.sign[AXIS_Y] = -1;
		imu_map.sign[AXIS_Z] = 1;
		imu_map.map[AXIS_X] = AXIS_Y;
		imu_map.map[AXIS_Y] = AXIS_X;
		imu_map.map[AXIS_Z] = AXIS_Z;
	}	
	else if(layout == 4)
	{
		imu_map.sign[AXIS_X] = -1;
		imu_map.sign[AXIS_Y] = 1;
		imu_map.sign[AXIS_Z] = -1;
		imu_map.map[AXIS_X] = AXIS_X;
		imu_map.map[AXIS_Y] = AXIS_Y;
		imu_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 5)
	{
		imu_map.sign[AXIS_X] = 1;
		imu_map.sign[AXIS_Y] = 1;
		imu_map.sign[AXIS_Z] = -1;
		imu_map.map[AXIS_X] = AXIS_Y;
		imu_map.map[AXIS_Y] = AXIS_X;
		imu_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 6)
	{
		imu_map.sign[AXIS_X] = 1;
		imu_map.sign[AXIS_Y] = -1;
		imu_map.sign[AXIS_Z] = -1;
		imu_map.map[AXIS_X] = AXIS_X;
		imu_map.map[AXIS_Y] = AXIS_Y;
		imu_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 7)
	{
		imu_map.sign[AXIS_X] = -1;
		imu_map.sign[AXIS_Y] = -1;
		imu_map.sign[AXIS_Z] = -1;
		imu_map.map[AXIS_X] = AXIS_Y;
		imu_map.map[AXIS_Y] = AXIS_X;
		imu_map.map[AXIS_Z] = AXIS_Z;
	}
	else		
	{
		imu_map.sign[AXIS_X] = 1;
		imu_map.sign[AXIS_Y] = 1;
		imu_map.sign[AXIS_Z] = 1;
		imu_map.map[AXIS_X] = AXIS_X;
		imu_map.map[AXIS_Y] = AXIS_Y;
		imu_map.map[AXIS_Z] = AXIS_Z;
	}
}
#endif

/********************************************************************************
 * @brief           Configures accelerometer settings
 * @param range     Accelerometer measurement range
 * @param odr       Output data rate
 * @param lpfEnable Low-pass filter enable/disable
 * @param stEnable  Self-test enable/disable
********************************************************************************/
void QMI8658_Config_Acc(enum QMI8658_AccRange range, enum QMI8658_AccOdr odr, enum QMI8658_Lpf_Config lpfEnable, enum QMI8658_St_Config stEnable)
{
    unsigned char ctl_dada;

    switch (range)
    {
    case QMI8658_AccRange_2g:
        acc_lsb_div = (1 << 14);
        break;
    case QMI8658_AccRange_4g:
        acc_lsb_div = (1 << 13);
        break;
    case QMI8658_AccRange_8g:
        acc_lsb_div = (1 << 12);
        break;
    case QMI8658_AccRange_16g:
        acc_lsb_div = (1 << 11);
        break;
    default:
        range = QMI8658_AccRange_8g;
        acc_lsb_div = (1 << 12);
    }
    if (stEnable == QMI8658_St_Enable)
        ctl_dada = (unsigned char)range | (unsigned char)odr | 0x80;
    else
        ctl_dada = (unsigned char)range | (unsigned char)odr;

    QMI8658_I2C_Write(QMI8658_Register_Ctrl2, ctl_dada);
    // set LPF & HPF
    QMI8658_I2C_Read_Buffer(QMI8658_Register_Ctrl5, &ctl_dada, 1);
    ctl_dada &= 0xf0;
    if (lpfEnable == QMI8658_Lpf_Enable)
    {
        ctl_dada |= A_LSP_MODE_3;
        ctl_dada |= 0x01;
    }
    else
    {
        ctl_dada &= ~0x01;
    }
    ctl_dada = 0x00;
    QMI8658_I2C_Write(QMI8658_Register_Ctrl5, ctl_dada);
    // set LPF & HPF
}

/********************************************************************************
 * @brief           Configures gyroscope settings
 * @param range     Gyroscope measurement range
 * @param odr       Output data rate
 * @param lpfEnable Low-pass filter enable/disable
 * @param stEnable  Self-test enable/disable
********************************************************************************/
void QMI8658_Config_Gyro(enum QMI8658_GyrRange range, enum QMI8658_GyrOdr odr, enum QMI8658_Lpf_Config lpfEnable, enum QMI8658_St_Config stEnable)
{
    // Set the CTRL3 register to configure dynamic range and ODR
    unsigned char ctl_dada;

    // Store the scale factor for use when processing raw data
    switch (range)
    {
    case QMI8658_GyrRange_32dps:
        gyro_lsb_div = 1024;
        break;
    case QMI8658_GyrRange_64dps:
        gyro_lsb_div = 512;
        break;
    case QMI8658_GyrRange_128dps:
        gyro_lsb_div = 256;
        break;
    case QMI8658_GyrRange_256dps:
        gyro_lsb_div = 128;
        break;
    case QMI8658_GyrRange_512dps:
        gyro_lsb_div = 64;
        break;
    case QMI8658_GyrRange_1024dps:
        gyro_lsb_div = 32;
        break;
    case QMI8658_GyrRange_2048dps:
        gyro_lsb_div = 16;
        break;
    case QMI8658_GyrRange_4096dps:
        gyro_lsb_div = 8;
        break;
    default:
        range = QMI8658_GyrRange_512dps;
        gyro_lsb_div = 64;
        break;
    }

    if (stEnable == QMI8658_St_Enable)
        ctl_dada = (unsigned char)range | (unsigned char)odr | 0x80;
    else
        ctl_dada = (unsigned char)range | (unsigned char)odr;
    QMI8658_I2C_Write(QMI8658_Register_Ctrl3, ctl_dada);

    // Conversion from degrees/s to rad/s if necessary
    // set LPF & HPF
    QMI8658_I2C_Read_Buffer(QMI8658_Register_Ctrl5, &ctl_dada, 1);
    ctl_dada &= 0x0f;
    if (lpfEnable == QMI8658_Lpf_Enable)
    {
        ctl_dada |= G_LSP_MODE_3;
        ctl_dada |= 0x10;
    }
    else
    {
        ctl_dada &= ~0x10;
    }
    ctl_dada = 0x00;
    QMI8658_I2C_Write(QMI8658_Register_Ctrl5, ctl_dada);
    // set LPF & HPF
}

/********************************************************************************
 * @brief           Configures magnetometer settings
 * @param device    Magnetometer device type
 * @param odr       Output data rate
********************************************************************************/
void QMI8658_Config_Mag(enum QMI8658_MagDev device, enum QMI8658_MagOdr odr)
{
    QMI8658_I2C_Write(QMI8658_Register_Ctrl4, device | odr);
}

/********************************************************************************
 * @brief           Configures attitude engine settings
 * @param odr       Output data rate
********************************************************************************/
void QMI8658_Config_AE(enum QMI8658_AeOdr odr)
{
    // QMI8658_Config_Acc(QMI8658_AccRange_8g, AccOdr_1000Hz, Lpf_Enable, St_Enable);
    // QMI8658_Config_Gyro(QMI8658_GyrRange_2048dps, GyrOdr_1000Hz, Lpf_Enable, St_Enable);
    QMI8658_Config_Acc(qmi8658_config.accRange, qmi8658_config.accOdr, QMI8658_Lpf_Enable, QMI8658_St_Disable);
    QMI8658_Config_Gyro(qmi8658_config.gyrRange, qmi8658_config.gyrOdr, QMI8658_Lpf_Enable, QMI8658_St_Disable);
    QMI8658_Config_Mag(qmi8658_config.magDev, qmi8658_config.magOdr);
    QMI8658_I2C_Write(QMI8658_Register_Ctrl6, odr);
}

/********************************************************************************
 * @brief           Reads Status0 register
 * @return          Status0 register value
********************************************************************************/
uint8_t QMI8658_Read_Status0(void)
{
    unsigned char status[2];

    QMI8658_I2C_Read_Buffer(QMI8658_Register_Status0, status, sizeof(status));
    // printf("status[0x%x	0x%x]\n",status[0],status[1]);

    return status[0];
}
/*!
 * \brief Blocking read of data status register 1 (::QMI8658_Register_Status1).
 * \returns Status byte \see STATUS1 for flag definitions.
 */
/********************************************************************************
 * @brief           Reads Status1 register
 * @return          Status1 register value
********************************************************************************/
uint8_t QMI8658_Read_Status1(void)
{
    unsigned char status;

    QMI8658_I2C_Read_Buffer(QMI8658_Register_Status1, &status, sizeof(status));

    return status;
}

/********************************************************************************
 * @brief           Reads temperature from sensor
 * @return          Temperature value in degrees Celsius
********************************************************************************/
float QMI8658_Read_Temp(void)
{
    unsigned char buf[2];
    short temp = 0;
    float temp_f = 0;

    QMI8658_I2C_Read_Buffer(QMI8658_Register_Tempearture_L, buf, 2);
    temp = ((short)buf[1] << 8) | buf[0];
    temp_f = (float)temp / 256.0f;

    return temp_f;
}

/********************************************************************************
 * @brief           Reads accelerometer XYZ data
 * @param acc_xyz   Array to store accelerometer data [X, Y, Z]
********************************************************************************/
void QMI8658_Read_Acc_XYZ(float acc_xyz[3])
{
    unsigned char buf_reg[6];
    short raw_acc_xyz[3];

    QMI8658_I2C_Read_Buffer(QMI8658_Register_Ax_L, buf_reg, 6); // 0x19, 25
    raw_acc_xyz[0] = (short)((unsigned short)(buf_reg[1] << 8) | (buf_reg[0]));
    raw_acc_xyz[1] = (short)((unsigned short)(buf_reg[3] << 8) | (buf_reg[2]));
    raw_acc_xyz[2] = (short)((unsigned short)(buf_reg[5] << 8) | (buf_reg[4]));

    acc_xyz[0] = (raw_acc_xyz[0] * ONE_G) / acc_lsb_div;
    acc_xyz[1] = (raw_acc_xyz[1] * ONE_G) / acc_lsb_div;
    acc_xyz[2] = (raw_acc_xyz[2] * ONE_G) / acc_lsb_div;

    // QMI8658_printf("fis210x acc:	%f	%f	%f\n", acc_xyz[0], acc_xyz[1], acc_xyz[2]);
}

/********************************************************************************
 * @brief           Reads gyroscope XYZ data
 * @param gyro_xyz  Array to store gyroscope data [X, Y, Z]
********************************************************************************/
void QMI8658_Read_Gyro_XYZ(float gyro_xyz[3])
{
    unsigned char buf_reg[6];
    short raw_gyro_xyz[3];

    QMI8658_I2C_Read_Buffer(QMI8658_Register_Gx_L, buf_reg, 6); // 0x1f, 31
    raw_gyro_xyz[0] = (short)((unsigned short)(buf_reg[1] << 8) | (buf_reg[0]));
    raw_gyro_xyz[1] = (short)((unsigned short)(buf_reg[3] << 8) | (buf_reg[2]));
    raw_gyro_xyz[2] = (short)((unsigned short)(buf_reg[5] << 8) | (buf_reg[4]));

    gyro_xyz[0] = (raw_gyro_xyz[0] * 1.0f) / gyro_lsb_div;
    gyro_xyz[1] = (raw_gyro_xyz[1] * 1.0f) / gyro_lsb_div;
    gyro_xyz[2] = (raw_gyro_xyz[2] * 1.0f) / gyro_lsb_div;

    // QMI8658_printf("fis210x gyro:	%f	%f	%f\n", gyro_xyz[0], gyro_xyz[1], gyro_xyz[2]);
}

/********************************************************************************
 * @brief           Reads both accelerometer and gyroscope XYZ data with timestamp
 * @param acc       Array to store accelerometer data [X, Y, Z]
 * @param gyro      Array to store gyroscope data [X, Y, Z]
 * @param tim_count Pointer to store timestamp value
********************************************************************************/
void QMI8658_Read_XYZ(float acc[3], float gyro[3], unsigned int *tim_count)
{
    unsigned char buf_reg[12];
    short raw_acc_xyz[3];
    short raw_gyro_xyz[3];
    //	float acc_t[3];
    //	float gyro_t[3];

    if (tim_count)
    {
        unsigned char buf[3];
        unsigned int timestamp;
        QMI8658_I2C_Read_Buffer(QMI8658_Register_Timestamp_L, buf, 3); // 0x18	24
        timestamp = (unsigned int)(((unsigned int)buf[2] << 16) | ((unsigned int)buf[1] << 8) | buf[0]);
        if (timestamp > imu_timestamp)
            imu_timestamp = timestamp;
        else
            imu_timestamp = (timestamp + 0x1000000 - imu_timestamp);

        *tim_count = imu_timestamp;
    }

    QMI8658_I2C_Read_Buffer(QMI8658_Register_Ax_L, buf_reg, 12); // 0x19, 25
    raw_acc_xyz[0] = (short)((unsigned short)(buf_reg[1] << 8) | (buf_reg[0]));
    raw_acc_xyz[1] = (short)((unsigned short)(buf_reg[3] << 8) | (buf_reg[2]));
    raw_acc_xyz[2] = (short)((unsigned short)(buf_reg[5] << 8) | (buf_reg[4]));

    raw_gyro_xyz[0] = (short)((unsigned short)(buf_reg[7] << 8) | (buf_reg[6]));
    raw_gyro_xyz[1] = (short)((unsigned short)(buf_reg[9] << 8) | (buf_reg[8]));
    raw_gyro_xyz[2] = (short)((unsigned short)(buf_reg[11] << 8) | (buf_reg[10]));

#if defined(QMI8658_UINT_MG_DPS)
    // mg
    acc[AXIS_X] = (float)(raw_acc_xyz[AXIS_X] * 1000.0f) / acc_lsb_div;
    acc[AXIS_Y] = (float)(raw_acc_xyz[AXIS_Y] * 1000.0f) / acc_lsb_div;
    acc[AXIS_Z] = (float)(raw_acc_xyz[AXIS_Z] * 1000.0f) / acc_lsb_div;
#else
    // m/s2
    acc[AXIS_X] = (float)(raw_acc_xyz[AXIS_X] * ONE_G) / acc_lsb_div;
    acc[AXIS_Y] = (float)(raw_acc_xyz[AXIS_Y] * ONE_G) / acc_lsb_div;
    acc[AXIS_Z] = (float)(raw_acc_xyz[AXIS_Z] * ONE_G) / acc_lsb_div;
#endif
    //	acc[AXIS_X] = imu_map.sign[AXIS_X]*acc_t[imu_map.map[AXIS_X]];
    //	acc[AXIS_Y] = imu_map.sign[AXIS_Y]*acc_t[imu_map.map[AXIS_Y]];
    //	acc[AXIS_Z] = imu_map.sign[AXIS_Z]*acc_t[imu_map.map[AXIS_Z]];

#if defined(QMI8658_UINT_MG_DPS)
    // dps
    gyro[0] = (float)(raw_gyro_xyz[0] * 1.0f) / gyro_lsb_div;
    gyro[1] = (float)(raw_gyro_xyz[1] * 1.0f) / gyro_lsb_div;
    gyro[2] = (float)(raw_gyro_xyz[2] * 1.0f) / gyro_lsb_div;
#else
    // rad/s
    gyro[AXIS_X] = (float)(raw_gyro_xyz[AXIS_X] * 0.01745f) / gyro_lsb_div; // *pi/180
    gyro[AXIS_Y] = (float)(raw_gyro_xyz[AXIS_Y] * 0.01745f) / gyro_lsb_div;
    gyro[AXIS_Z] = (float)(raw_gyro_xyz[AXIS_Z] * 0.01745f) / gyro_lsb_div;
#endif
    //	gyro[AXIS_X] = imu_map.sign[AXIS_X]*gyro_t[imu_map.map[AXIS_X]];
    //	gyro[AXIS_Y] = imu_map.sign[AXIS_Y]*gyro_t[imu_map.map[AXIS_Y]];
    //	gyro[AXIS_Z] = imu_map.sign[AXIS_Z]*gyro_t[imu_map.map[AXIS_Z]];
}

/********************************************************************************
 * @brief           Reads raw accelerometer and gyroscope XYZ data with timestamp
 * @param raw_acc_xyz   Array to store raw accelerometer data [X, Y, Z]
 * @param raw_gyro_xyz  Array to store raw gyroscope data [X, Y, Z]
 * @param tim_count     Pointer to store timestamp value
********************************************************************************/
void QMI8658_Read_XYZ_Raw(short raw_acc_xyz[3], short raw_gyro_xyz[3], unsigned int *tim_count)
{
    unsigned char buf_reg[12];

    if (tim_count)
    {
        unsigned char buf[3];
        unsigned int timestamp;
        QMI8658_I2C_Read_Buffer(QMI8658_Register_Timestamp_L, buf, 3); // 0x18	24
        timestamp = (unsigned int)(((unsigned int)buf[2] << 16) | ((unsigned int)buf[1] << 8) | buf[0]);
        if (timestamp > imu_timestamp)
            imu_timestamp = timestamp;
        else
            imu_timestamp = (timestamp + 0x1000000 - imu_timestamp);

        *tim_count = imu_timestamp;
    }
    QMI8658_I2C_Read_Buffer(QMI8658_Register_Ax_L, buf_reg, 12); // 0x19, 25

    raw_acc_xyz[0] = (short)((unsigned short)(buf_reg[1] << 8) | (buf_reg[0]));
    raw_acc_xyz[1] = (short)((unsigned short)(buf_reg[3] << 8) | (buf_reg[2]));
    raw_acc_xyz[2] = (short)((unsigned short)(buf_reg[5] << 8) | (buf_reg[4]));

    raw_gyro_xyz[0] = (short)((unsigned short)(buf_reg[7] << 8) | (buf_reg[6]));
    raw_gyro_xyz[1] = (short)((unsigned short)(buf_reg[9] << 8) | (buf_reg[8]));
    raw_gyro_xyz[2] = (short)((unsigned short)(buf_reg[11] << 8) | (buf_reg[10]));
}

/********************************************************************************
 * @brief           Reads attitude engine quaternion and velocity data
 * @param quat      Array to store quaternion data [q1, q2, q3, q4]
 * @param velocity  Array to store velocity data [X, Y, Z]
********************************************************************************/
void QMI8658_Read_AE(float quat[4], float velocity[3])
{
    unsigned char buf_reg[14];
    short raw_q_xyz[4];
    short raw_v_xyz[3];

    QMI8658_I2C_Read_Buffer(QMI8658_Register_Q1_L, buf_reg, 14);
    raw_q_xyz[0] = (short)((unsigned short)(buf_reg[1] << 8) | (buf_reg[0]));
    raw_q_xyz[1] = (short)((unsigned short)(buf_reg[3] << 8) | (buf_reg[2]));
    raw_q_xyz[2] = (short)((unsigned short)(buf_reg[5] << 8) | (buf_reg[4]));
    raw_q_xyz[3] = (short)((unsigned short)(buf_reg[7] << 8) | (buf_reg[6]));

    raw_v_xyz[1] = (short)((unsigned short)(buf_reg[9] << 8) | (buf_reg[8]));
    raw_v_xyz[2] = (short)((unsigned short)(buf_reg[11] << 8) | (buf_reg[10]));
    raw_v_xyz[2] = (short)((unsigned short)(buf_reg[13] << 8) | (buf_reg[12]));

    quat[0] = (float)(raw_q_xyz[0] * 1.0f) / ae_q_lsb_div;
    quat[1] = (float)(raw_q_xyz[1] * 1.0f) / ae_q_lsb_div;
    quat[2] = (float)(raw_q_xyz[2] * 1.0f) / ae_q_lsb_div;
    quat[3] = (float)(raw_q_xyz[3] * 1.0f) / ae_q_lsb_div;

    velocity[0] = (float)(raw_v_xyz[0] * 1.0f) / ae_v_lsb_div;
    velocity[1] = (float)(raw_v_xyz[1] * 1.0f) / ae_v_lsb_div;
    velocity[2] = (float)(raw_v_xyz[2] * 1.0f) / ae_v_lsb_div;
}

/********************************************************************************
 * @brief           Executes a CTRL9 command with proper handshake protocol
 * @param cmd       CTRL9 command to execute
********************************************************************************/
void QMI8658_doCtrl9Command(enum QMI8658_Ctrl9_Command cmd)
{
    uint8_t statusint_val;
    int timeout_count = 0;
    const int MAX_TIMEOUT = 100; // 1 second timeout (10ms * 100)

    // Step 1: Write command to CTRL9 register
    QMI8658_I2C_Write(QMI8658_Register_Ctrl9, cmd);
    printf("[Debug] Sent CTRL9 command: 0x%02x\n", cmd);

    // Step 2: Wait for STATUSINT.bit7 to be set (command completion)
    while (timeout_count < MAX_TIMEOUT)
    {
        QMI8658_I2C_Read_Buffer(QMI8658_Register_StatusInt, &statusint_val, 1);
        printf("[Debug] StatusInt: 0x%02x, timeout: %d\n", statusint_val, timeout_count);
        
        // Check if bit 7 is set (0x80 mask)
        if (statusint_val & 0x80)
        {
            printf("[Debug] Command completed, sending ACK\n");
            break;
        }
        
        sleep_ms(10);
        timeout_count++;
    }

    if (timeout_count >= MAX_TIMEOUT)
    {
        printf("[Error] CTRL9 command timeout - no completion signal\n");
        return;
    }

    // Step 3: Send acknowledgment by writing 0x00 to CTRL9
    QMI8658_I2C_Write(QMI8658_Register_Ctrl9, 0x00); // CTRL_CMD_ACK
    printf("[Debug] Sent CTRL9 ACK (0x00)\n");

    // Step 4: Wait for STATUSINT.bit7 to be cleared (ACK processed)
    timeout_count = 0;
    while (timeout_count < MAX_TIMEOUT)
    {
        QMI8658_I2C_Read_Buffer(QMI8658_Register_StatusInt, &statusint_val, 1);
        printf("[Debug] StatusInt after ACK: 0x%02x\n", statusint_val);
        
        // Check if bit 7 is cleared (command fully processed)
        if (!(statusint_val & 0x80))
        {
            printf("[Debug] CTRL9 command handshake complete\n");
            return;
        }
        
        sleep_ms(10);
        timeout_count++;
    }

    printf("[Warning] CTRL9 ACK timeout - bit7 not cleared\n");
}
/********************************************************************************
 * @brief           Enables wake-on-motion functionality
********************************************************************************/
void QMI8658_Enable_Wake_On_Motion(void)
{
    unsigned char womCmd[3];
    enum QMI8658_Interrupt interrupt = QMI8658_Int1;
    enum QMI8658_InterruptState initialState = QMI8658_State_low;
    enum QMI8658_WakeOnMotionThreshold threshold = QMI8658_WomThreshold_low;
    unsigned char blankingTime = 0x00;
    const unsigned char blankingTimeMask = 0x3F;

    QMI8658_Enable_Sensors(QMI8658_CTRL7_DISABLE_ALL);
    QMI8658_Config_Acc(QMI8658_AccRange_2g, QMI8658_AccOdr_LowPower_21Hz, QMI8658_Lpf_Disable, QMI8658_St_Disable);

    womCmd[0] = QMI8658_Register_Cal1_L; // WoM Threshold: absolute value in mg (with 1mg/LSB resolution)
    womCmd[1] = threshold;
    womCmd[2] = (unsigned char)interrupt | (unsigned char)initialState | (blankingTime & blankingTimeMask);
    QMI8658_I2C_Write(QMI8658_Register_Cal1_L, womCmd[1]);
    QMI8658_I2C_Write(QMI8658_Register_Cal1_H, womCmd[2]);



    QMI8658_doCtrl9Command(QMI8658_Ctrl9_Cmd_WoM_Setting);
    QMI8658_Enable_Sensors(QMI8658_CTRL7_ACC_ENABLE);

}

/********************************************************************************
 * @brief           Disables wake-on-motion functionality
********************************************************************************/
void QMI8658_Disable_Wake_On_Motion(void)
{
    QMI8658_Enable_Sensors(QMI8658_CTRL7_DISABLE_ALL);
    QMI8658_I2C_Write(QMI8658_Register_Cal1_L, 0);
    // QMI8658_doCtrl9Command(QMI8658_Ctrl9_Cmd_WoM_Setting);
}

/********************************************************************************
 * @brief           Enables specified sensors
 * @param enable_flags  Sensor enable flags (accelerometer, gyroscope, etc.)
********************************************************************************/
void QMI8658_Enable_Sensors(unsigned char enable_flags)
{
    if (enable_flags & QMI8658_CONFIG_AE_ENABLE)
    {
        enable_flags |= QMI8658_CTRL7_ACC_ENABLE | QMI8658_CTRL7_GYR_ENABLE;
    }

    QMI8658_I2C_Write(QMI8658_Register_Ctrl7, enable_flags & QMI8658_CTRL7_ENABLE_MASK);
}

/********************************************************************************
 * @brief           Applies complete configuration to QMI8658 sensor
 * @param config    Pointer to configuration structure
********************************************************************************/
void QMI8658_Config_Apply(struct QMI8658_Config const *config)
{
    // Configure pedometer FIRST if needed (before accelerometer)
    if (config->enablePedometer)
    {
        QMI8658_Config_Pedometer(&config->pedoConfig);
        QMI8658_Enable_Pedometer();
    }
    // Fusion / Inertia Sensors
    if (config->inputSelection & QMI8658_CONFIG_AE_ENABLE)
    {
        QMI8658_Config_AE(config->aeOdr);
    }
    else
    {
        if (config->inputSelection & QMI8658_CONFIG_ACC_ENABLE)
        {
            QMI8658_Config_Acc(config->accRange, config->accOdr, QMI8658_Lpf_Enable, QMI8658_St_Disable);
        }
        if (config->inputSelection & QMI8658_CONFIG_GYR_ENABLE)
        {
            QMI8658_Config_Gyro(config->gyrRange, config->gyrOdr, QMI8658_Lpf_Enable, QMI8658_St_Disable);
        }
    }

    if (config->inputSelection & QMI8658_CONFIG_MAG_ENABLE)
    {
        QMI8658_Config_Mag(config->magDev, config->magOdr);
    }

    QMI8658_Enable_Sensors(config->inputSelection);
}

/********************************************************************************
 * @brief           Initializes QMI8658 sensor with specified configuration
 * @param configuration  Configuration structure for sensor setup
 * @return          1 if initialization successful, 0 if failed
********************************************************************************/
uint8_t QMI8658_init(struct QMI8658_Config configuration)
{
    unsigned char QMI8658_chip_id = 0x00;
    unsigned char QMI8658_revision_id = 0x00;
    unsigned char QMI8658_slave[2] = {QMI8658_SLAVE_ADDR_L, QMI8658_SLAVE_ADDR_H};
    unsigned char iCount = 0;
    int retry = 0;

    while (iCount < 2)
    {
        QMI8658_SLAVE_ADDR = QMI8658_slave[iCount];
        retry = 0;

        while ((QMI8658_chip_id != 0x05) && (retry++ < 5))
        {

            QMI8658_I2C_Read_Buffer(QMI8658_Register_WhoAmI, &QMI8658_chip_id, 1);
            QMI8658_printf("QMI8658_Register_WhoAmI = 0x%x\n", QMI8658_chip_id);
        }
        if (QMI8658_chip_id == 0x05)
        {
            break;
        }
        iCount++;
    }
    QMI8658_I2C_Read_Buffer(QMI8658_Register_Revision, &QMI8658_revision_id, 1);
    if (QMI8658_chip_id == 0x05)
    {
        QMI8658_printf("QMI8658_init slave=0x%x  \r\nQMI8658_Register_WhoAmI=0x%x 0x%x\n", QMI8658_SLAVE_ADDR, QMI8658_chip_id, QMI8658_revision_id);
        QMI8658_I2C_Write(QMI8658_Register_Ctrl1, 0x60);
        //qmi8658_config.inputSelection = QMI8658_CONFIG_ACCGYR_ENABLE; // QMI8658_CONFIG_ACCGYR_ENABLE;
        //qmi8658_config.accRange = QMI8658_AccRange_4g;
        //qmi8658_config.accOdr = QMI8658_AccOdr_125Hz;
        //qmi8658_config.gyrRange = QMI8658_GyrRange_512dps; // QMI8658_GyrRange_2048dps   QMI8658_GyrRange_1024dps
        //qmi8658_config.gyrOdr = QMI8658_GyrOdr_1000Hz;
        //qmi8658_config.magOdr = QMI8658_MagOdr_125Hz;
        //qmi8658_config.magDev = QMI8658_MagDev_AKM09918;
        //qmi8658_config.aeOdr = QMI8658_AeOdr_128Hz;

        qmi8658_config = configuration;
        QMI8658_Config_Apply(&qmi8658_config);
        if (1)
        {
            unsigned char read_data = 0x00;
            QMI8658_I2C_Read_Buffer(QMI8658_Register_Ctrl1, &read_data, 1);
            QMI8658_printf("QMI8658_Register_Ctrl1=0x%x \n", read_data);
            QMI8658_I2C_Read_Buffer(QMI8658_Register_Ctrl2, &read_data, 1);
            QMI8658_printf("QMI8658_Register_Ctrl2=0x%x \n", read_data);
            QMI8658_I2C_Read_Buffer(QMI8658_Register_Ctrl3, &read_data, 1);
            QMI8658_printf("QMI8658_Register_Ctrl3=0x%x \n", read_data);
            QMI8658_I2C_Read_Buffer(QMI8658_Register_Ctrl4, &read_data, 1);
            QMI8658_printf("QMI8658_Register_Ctrl4=0x%x \n", read_data);
            QMI8658_I2C_Read_Buffer(QMI8658_Register_Ctrl5, &read_data, 1);
            QMI8658_printf("QMI8658_Register_Ctrl5=0x%x \n", read_data);
            QMI8658_I2C_Read_Buffer(QMI8658_Register_Ctrl6, &read_data, 1);
            QMI8658_printf("QMI8658_Register_Ctrl6=0x%x \n", read_data);
            QMI8658_I2C_Read_Buffer(QMI8658_Register_Ctrl7, &read_data, 1);
            QMI8658_printf("QMI8658_Register_Ctrl7=0x%x \n", read_data);
            QMI8658_I2C_Read_Buffer(QMI8658_Register_Ctrl8, &read_data, 1);
            QMI8658_printf("QMI8658_Register_Ctrl8=0x%x \n", read_data);
        }
        //		QMI8658_set_layout(2);
        return 1;
    }
    else
    {
        QMI8658_printf("QMI8658_init fail\n");
        QMI8658_chip_id = 0;
        return 0;
    }
    // return QMI8658_chip_id;
}

/********************************************************************************
 * @brief           Configures pedometer settings
 * @param config    Pointer to pedometer configuration structure
********************************************************************************/
void QMI8658_Config_Pedometer(struct QMI8658_PedoConfig const *config)
{
    // First phase: Send first set of parameters
    QMI8658_I2C_Write(QMI8658_Register_Cal1_L, config->sample_count);
    QMI8658_I2C_Write(QMI8658_Register_Cal1_H, config->sample_count);
    QMI8658_I2C_Write(QMI8658_Register_Cal2_L, config->fix_peak2peak);
    QMI8658_I2C_Write(QMI8658_Register_Cal2_H, config->fix_peak2peak);
    QMI8658_I2C_Write(QMI8658_Register_Cal3_L, config->fix_peak);
    QMI8658_I2C_Write(QMI8658_Register_Cal3_H, config->fix_peak);
    QMI8658_I2C_Write(QMI8658_Register_Cal4_H, 0x01);  // First phase marker

    // Trigger first CTRL9 command
    QMI8658_doCtrl9Command(QMI8658_Ctrl9_Cmd_ConfigurePedometer);

    // Second phase: Send pedometer control configuration
    QMI8658_I2C_Write(QMI8658_Register_Cal1_L, config->time_up);
    QMI8658_I2C_Write(QMI8658_Register_Cal1_H, config->time_up);
    QMI8658_I2C_Write(QMI8658_Register_Cal2_L, config->time_low);
    QMI8658_I2C_Write(QMI8658_Register_Cal2_H, config->time_count_entry);
    QMI8658_I2C_Write(QMI8658_Register_Cal3_L, config->fix_precision);
    QMI8658_I2C_Write(QMI8658_Register_Cal3_H, config->signal_count);
    QMI8658_I2C_Write(QMI8658_Register_Cal4_H, 0x02);  // Second phase marker

    // Trigger second CTRL9 command
    QMI8658_doCtrl9Command(QMI8658_Ctrl9_Cmd_ConfigurePedometer);
}

/********************************************************************************
 * @brief           Enables pedometer functionality
********************************************************************************/
void QMI8658_Enable_Pedometer(void)
{
    unsigned char ctrl_data;

    QMI8658_I2C_Read_Buffer(QMI8658_Register_Ctrl8, &ctrl_data, 1);
    ctrl_data |= QMI8658_PedoMode_Enable;
    QMI8658_I2C_Write(QMI8658_Register_Ctrl8, ctrl_data);

    //QMI8658_Config_Acc(QMI8658_AccRange_4g, QMI8658_AccOdr_125Hz, QMI8658_Lpf_Enable, QMI8658_St_Disable);
    //QMI8658_Enable_Sensors(QMI8658_CONFIG_ACC_ENABLE);
}

/********************************************************************************
 * @brief           Disables pedometer functionality
********************************************************************************/
void QMI8658_Disable_Pedometer(void)
{
    unsigned char ctrl_data;

    QMI8658_I2C_Read_Buffer(QMI8658_Register_Ctrl8, &ctrl_data, 1);
    ctrl_data &= ~QMI8658_PedoMode_Enable;
    QMI8658_I2C_Write(QMI8658_Register_Ctrl8, ctrl_data);
}

/********************************************************************************
 * @brief           Reads current step count from pedometer
 * @param stepCount Pointer to store step count value
********************************************************************************/
void QMI8658_Read_Step_Count(unsigned int *stepCount)
{
    unsigned char buf[3];

    if (stepCount)
    {
        QMI8658_I2C_Read_Buffer(QMI8658_Register_STEP_CNT_LOW, buf, 3);
        *stepCount = (unsigned int)buf[0] | 
                    ((unsigned int)buf[1] << 8) | 
                    ((unsigned int)buf[2] << 16);
    }
}

/********************************************************************************
 * @brief           Resets pedometer step count to zero
********************************************************************************/
void QMI8658_Reset_Step_Count(void)
{
    QMI8658_I2C_Write(QMI8658_Register_STEP_CNT_LOW, 0x00);
    QMI8658_I2C_Write(QMI8658_Register_STEP_CNT_MIDL, 0x00);
    QMI8658_I2C_Write(QMI8658_Register_STEP_CNT_HIGH, 0x00);
}

/********************************************************************************
 * @brief           Configures pedometer interrupt settings
********************************************************************************/
void QMI8658_Config_Pedometer_interrupt(void)
{
    unsigned char ctrl1_data, ctrl8_data;

    // Read current CTRL1 register
    QMI8658_I2C_Read_Buffer(QMI8658_Register_Ctrl1, &ctrl1_data, 1);

    // Enable INT1 pin (set bit 3)
    ctrl1_data |= 0x08;  // CTRL1.bit3 = 1 for INT1 enable
    QMI8658_I2C_Write(QMI8658_Register_Ctrl1, ctrl1_data);

    // Read current CTRL8 register  
    QMI8658_I2C_Read_Buffer(QMI8658_Register_Ctrl8, &ctrl8_data, 1);

    // Configure pedometer interrupt routing:
    // - Set bit 6 = 1 to route pedometer interrupt to INT1 
    // - Set bit 7 = 0 to enable INT1 interrupt output
    ctrl8_data |= 0x40;   // CTRL8.bit6 = 1 (route to INT1)
    ctrl8_data &= ~0x80;  // CTRL8.bit7 = 0 (enable INT1 output)
    QMI8658_I2C_Write(QMI8658_Register_Ctrl8, ctrl8_data);

    printf("[Debug] Pedometer interrupt configured - CTRL1: 0x%02x, CTRL8: 0x%02x\n", 
            ctrl1_data, ctrl8_data);
}

/********************************************************************************
 * @brief           Checks pedometer interrupt status
********************************************************************************/
void QMI8658_Check_Pedometer_Interrupt(void)
{
    unsigned char status1_data;

    // Read STATUS1 register
    QMI8658_I2C_Read_Buffer(QMI8658_Register_Status1, &status1_data, 1);

    // Check if pedometer interrupt bit (bit 4) is set
    if (status1_data & 0x10) {
        printf("[Debug] Pedometer interrupt detected - STATUS1: 0x%02x\n", status1_data);
    }
}
