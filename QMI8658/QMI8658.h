#ifndef QMI8658_H
#define QMI8658_H

#include "WS_Config.h"
#include <stdint.h>
#include <stdlib.h> //itoa()
#include <stdio.h>

#ifndef M_PI
#define M_PI (3.14159265358979323846f)
#endif
#ifndef ONE_G
#define ONE_G (9.807f)
#endif

#define QMI8658_CTRL7_DISABLE_ALL (0x0)
#define QMI8658_CTRL7_ACC_ENABLE (0x1)
#define QMI8658_CTRL7_GYR_ENABLE (0x2)
#define QMI8658_CTRL7_MAG_ENABLE (0x4)
#define QMI8658_CTRL7_AE_ENABLE (0x8)
#define QMI8658_CTRL7_GYR_SNOOZE_ENABLE (0x10)
#define QMI8658_CTRL7_ENABLE_MASK (0xF)

#define QMI8658_CONFIG_ACC_ENABLE QMI8658_CTRL7_ACC_ENABLE
#define QMI8658_CONFIG_GYR_ENABLE QMI8658_CTRL7_GYR_ENABLE
#define QMI8658_CONFIG_MAG_ENABLE QMI8658_CTRL7_MAG_ENABLE
#define QMI8658_CONFIG_AE_ENABLE QMI8658_CTRL7_AE_ENABLE

#define QMI8658_CONFIG_ACCGYR_ENABLE (QMI8658_CONFIG_ACC_ENABLE | QMI8658_CONFIG_GYR_ENABLE)
#define QMI8658_CONFIG_ACCGYRMAG_ENABLE (QMI8658_CONFIG_ACC_ENABLE | QMI8658_CONFIG_GYR_ENABLE | QMI8658_CONFIG_MAG_ENABLE)
#define QMI8658_CONFIG_AEMAG_ENABLE (QMI8658_CONFIG_AE_ENABLE | QMI8658_CONFIG_MAG_ENABLE)

#define QMI8658_STATUS1_CMD_DONE (0x01)
#define QMI8658_STATUS1_WAKEUP_EVENT (0x04)

enum QMI8658_Register
{
    QMI8658_Register_WhoAmI = 0,        /*! \brief FIS device identifier register. */
    QMI8658_Register_Revision,          /*! \brief FIS hardware revision register. */
    QMI8658_Register_Ctrl1,             /*! \brief General and power management modes. */
    QMI8658_Register_Ctrl2,             /*! \brief Accelerometer control. */
    QMI8658_Register_Ctrl3,             /*! \brief Gyroscope control. */
    QMI8658_Register_Ctrl4,             /*! \brief Magnetometer control. */
    QMI8658_Register_Ctrl5,             /*! \brief Data processing settings. */
    QMI8658_Register_Ctrl6,             /*! \brief AttitudeEngine control. */
    QMI8658_Register_Ctrl7,             /*! \brief Sensor enabled status. */
    QMI8658_Register_Ctrl8,             /*! \brief Motion Detection Control. */
    QMI8658_Register_Ctrl9,             /*! \brief Host command register. */
    QMI8658_Register_Cal1_L = 11,       /*! \brief Calibration register 1 most significant byte. */
    QMI8658_Register_Cal1_H,            /*! \brief Calibration register 1 least significant byte. */
    QMI8658_Register_Cal2_L,            /*! \brief Calibration register 2 most significant byte. */
    QMI8658_Register_Cal2_H,            /*! \brief Calibration register 2 least significant byte. */
    QMI8658_Register_Cal3_L,            /*! \brief Calibration register 3 most significant byte. */
    QMI8658_Register_Cal3_H,            /*! \brief Calibration register 3 least significant byte. */
    QMI8658_Register_Cal4_L,            /*! \brief Calibration register 4 most significant byte. */
    QMI8658_Register_Cal4_H,            /*! \brief Calibration register 4 least significant byte. */
    QMI8658_Register_FifoCtrl = 19,     /*! \brief FIFO control register. */
    QMI8658_Register_FifoData,          /*! \brief FIFO data register. */
    QMI8658_Register_FifoStatus,        /*! \brief FIFO status register. */
    QMI8658_Register_StatusInt = 45,    /*! \brief Output data overrun and availability. */
    QMI8658_Register_Status0,           /*! \brief Output data overrun and availability. */
    QMI8658_Register_Status1,           /*! \brief Miscellaneous status register. */
    QMI8658_Register_Timestamp_L = 48,  /*! \brief timestamp low. */
    QMI8658_Register_Timestamp_M,       /*! \brief timestamp mid. */
    QMI8658_Register_Timestamp_H,       /*! \brief timestamp high. */
    QMI8658_Register_Tempearture_L = 51, /*! \brief temperature low. */
    QMI8658_Register_Tempearture_H,     /*! \brief temperature high. */
    QMI8658_Register_Ax_L = 53,         /*! \brief Accelerometer X axis least significant byte. */
    QMI8658_Register_Ax_H,              /*! \brief Accelerometer X axis most significant byte. */
    QMI8658_Register_Ay_L,              /*! \brief Accelerometer Y axis least significant byte. */
    QMI8658_Register_Ay_H,              /*! \brief Accelerometer Y axis most significant byte. */
    QMI8658_Register_Az_L,              /*! \brief Accelerometer Z axis least significant byte. */
    QMI8658_Register_Az_H,              /*! \brief Accelerometer Z axis most significant byte. */
    QMI8658_Register_Gx_L = 59,         /*! \brief Gyroscope X axis least significant byte. */
    QMI8658_Register_Gx_H,              /*! \brief Gyroscope X axis most significant byte. */
    QMI8658_Register_Gy_L,              /*! \brief Gyroscope Y axis least significant byte. */
    QMI8658_Register_Gy_H,              /*! \brief Gyroscope Y axis most significant byte. */
    QMI8658_Register_Gz_L,              /*! \brief Gyroscope Z axis least significant byte. */
    QMI8658_Register_Gz_H,              /*! \brief Gyroscope Z axis most significant byte. */
    QMI8658_Register_Mx_L = 65,         /*! \brief Magnetometer X axis least significant byte. */
    QMI8658_Register_Mx_H,              /*! \brief Magnetometer X axis most significant byte. */
    QMI8658_Register_My_L,              /*! \brief Magnetometer Y axis least significant byte. */
    QMI8658_Register_My_H,              /*! \brief Magnetometer Y axis most significant byte. */
    QMI8658_Register_Mz_L,              /*! \brief Magnetometer Z axis least significant byte. */
    QMI8658_Register_Mz_H,              /*! \brief Magnetometer Z axis most significant byte. */
    QMI8658_Register_Q1_L = 73,         /*! \brief Quaternion increment W least significant byte. */
    QMI8658_Register_Q1_H,              /*! \brief Quaternion increment W most significant byte. */
    QMI8658_Register_Q2_L,              /*! \brief Quaternion increment X least significant byte. */
    QMI8658_Register_Q2_H,              /*! \brief Quaternion increment X most significant byte. */
    QMI8658_Register_Q3_L,              /*! \brief Quaternion increment Y least significant byte. */
    QMI8658_Register_Q3_H,              /*! \brief Quaternion increment Y most significant byte. */
    QMI8658_Register_Q4_L,              /*! \brief Quaternion increment Z least significant byte. */
    QMI8658_Register_Q4_H,              /*! \brief Quaternion increment Z most significant byte. */
    QMI8658_Register_Dvx_L = 81,        /*! \brief Velocity increment X least significant byte. */
    QMI8658_Register_Dvx_H,             /*! \brief Velocity increment X most significant byte. */
    QMI8658_Register_Dvy_L,             /*! \brief Velocity increment Y least significant byte. */
    QMI8658_Register_Dvy_H,             /*! \brief Velocity increment Y most significant byte. */
    QMI8658_Register_Dvz_L,             /*! \brief Velocity increment Z least significant byte. */
    QMI8658_Register_Dvz_H,             /*! \brief Velocity increment Z most significant byte. */
    QMI8658_Register_AeReg1 = 87,       /*! \brief AttitudeEngine reg1. */
    QMI8658_Register_AeOverflow,        /*! \brief AttitudeEngine overflow flags. */
    QMI8658_Register_TAP_STATUS,        /*! \brief Activity Detection Tap Status */
    QMI8658_Register_STEP_CNT_LOW,      /*! \brief Activity Detection Low Byte of Step */
    QMI8658_Register_STEP_CNT_MIDL,     /*! \brief Activity Detection Middle Byte of Step */
    QMI8658_Register_STEP_CNT_HIGH,     /*! \brief Activity Detection High Byte of Step */
    QMI8658_Register_I2CM_STATUS = 110
};

enum QMI8658_Ois_Register
{
    /*-----------------------------*/
    /* Setup and Control Registers */
    /*-----------------------------*/
    QMI8658_OIS_Reg_Ctrl1 = 0x02,       /*! \brief SPI Endian Selection, and SPI 3/4 Wire */
    QMI8658_OIS_Reg_Ctrl2,              /*! \brief Accelerometer control: ODR, Full Scale, Self Test */
    QMI8658_OIS_Reg_Ctrl3,              /*! \brief Gyroscope control: ODR, Full Scale, Self Test */
    QMI8658_OIS_Reg_Ctrl5 = 0x06,       /*! \brief Sensor Data Processing Settings */
    QMI8658_OIS_Reg_Ctrl7 = 0x08,       /*! \brief Sensor enabled status: Enable Sensors */
    /*-------------------*/
    /* Status Registers  */
    /*-------------------*/
    QMI8658_OIS_Reg_StatusInt = 0x2D,   /*! \brief Sensor Data Availability and Lock Register */
    QMI8658_OIS_Reg_Status0 = 0x2E,     /*! \brief Output data overrun and availability */

    /*-----------------------------------------------------*/
    /* OIS Sensor Data Output Registers. 16-bit 2's complement */
    /*-----------------------------------------------------*/
    QMI8658_OIS_Reg_Ax_L = 0x33,        /*! \brief Accelerometer X axis least significant byte */
    QMI8658_OIS_Reg_Ax_H,               /*! \brief Accelerometer X axis most significant byte */
    QMI8658_OIS_Reg_Ay_L,               /*! \brief Accelerometer Y axis least significant byte */
    QMI8658_OIS_Reg_Ay_H,               /*! \brief Accelerometer Y axis most significant byte */
    QMI8658_OIS_Reg_Az_L,               /*! \brief Accelerometer Z axis least significant byte */
    QMI8658_OIS_Reg_Az_H,               /*! \brief Accelerometer Z axis most significant byte */

    QMI8658_OIS_Reg_Gx_L = 0x3B,        /*! \brief Gyroscope X axis least significant byte */
    QMI8658_OIS_Reg_Gx_H,               /*! \brief Gyroscope X axis most significant byte */
    QMI8658_OIS_Reg_Gy_L,               /*! \brief Gyroscope Y axis least significant byte */
    QMI8658_OIS_Reg_Gy_H,               /*! \brief Gyroscope Y axis most significant byte */
    QMI8658_OIS_Reg_Gz_L,               /*! \brief Gyroscope Z axis least significant byte */
    QMI8658_OIS_Reg_Gz_H                /*! \brief Gyroscope Z axis most significant byte */
};

enum QMI8658_Ctrl9_Command
{
    QMI8658_Ctrl9_Cmd_NOP = 0X00,
    QMI8658_Ctrl9_Cmd_GyroBias = 0X01,
    QMI8658_Ctrl9_Cmd_Rqst_Sdi_Mod = 0X03,
    QMI8658_Ctrl9_Cmd_WoM_Setting = 0x08,
    QMI8658_Ctrl9_Cmd_AccelHostDeltaOffset = 0x09,
    QMI8658_Ctrl9_Cmd_GyroHostDeltaOffset = 0x0A,
    QMI8658_Ctrl9_Cmd_ConfigurePedometer = 0x0D,
    QMI8658_Ctrl9_Cmd_Dbg_WoM_Data_Enable = 0xF8,

};

enum QMI8658_Lpf_Config
{
    QMI8658_Lpf_Disable, /*! \brief Disable low pass filter. */
    QMI8658_Lpf_Enable   /*! \brief Enable low pass filter. */
};

enum QMI8658_Hpf_Config
{
    QMI8658_Hpf_Disable, /*! \brief Disable high pass filter. */
    QMI8658_Hpf_Enable   /*! \brief Enable high pass filter. */
};

enum QMI8658_St_Config
{
    QMI8658_St_Disable, /*! \brief Disable high pass filter. */
    QMI8658_St_Enable   /*! \brief Enable high pass filter. */
};

enum QMI8658_Lpf_Mode
{
    A_LSP_MODE_0 = 0x00 << 1,
    A_LSP_MODE_1 = 0x01 << 1,
    A_LSP_MODE_2 = 0x02 << 1,
    A_LSP_MODE_3 = 0x03 << 1,

    G_LSP_MODE_0 = 0x00 << 5,
    G_LSP_MODE_1 = 0x01 << 5,
    G_LSP_MODE_2 = 0x02 << 5,
    G_LSP_MODE_3 = 0x03 << 5
};

enum QMI8658_AccRange
{
    QMI8658_AccRange_2g = 0x00 << 4, /*! \brief +/- 2g range */
    QMI8658_AccRange_4g = 0x01 << 4, /*! \brief +/- 4g range */
    QMI8658_AccRange_8g = 0x02 << 4, /*! \brief +/- 8g range */
    QMI8658_AccRange_16g = 0x03 << 4 /*! \brief +/- 16g range */
};

enum QMI8658_AccOdr
{
    QMI8658_AccOdr_8000Hz = 0x00,         /*! \brief High resolution 8000Hz output rate. */
    QMI8658_AccOdr_4000Hz = 0x01,         /*! \brief High resolution 4000Hz output rate. */
    QMI8658_AccOdr_2000Hz = 0x02,         /*! \brief High resolution 2000Hz output rate. */
    QMI8658_AccOdr_1000Hz = 0x03,         /*! \brief High resolution 1000Hz output rate. */
    QMI8658_AccOdr_500Hz = 0x04,          /*! \brief High resolution 500Hz output rate. */
    QMI8658_AccOdr_250Hz = 0x05,          /*! \brief High resolution 250Hz output rate. */
    QMI8658_AccOdr_125Hz = 0x06,          /*! \brief High resolution 125Hz output rate. */
    QMI8658_AccOdr_62_5Hz = 0x07,         /*! \brief High resolution 62.5Hz output rate. */
    QMI8658_AccOdr_31_25Hz = 0x08,        /*! \brief High resolution 31.25Hz output rate. */
    QMI8658_AccOdr_LowPower_128Hz = 0x0c, /*! \brief Low power 128Hz output rate. */
    QMI8658_AccOdr_LowPower_21Hz = 0x0d,  /*! \brief Low power 21Hz output rate. */
    QMI8658_AccOdr_LowPower_11Hz = 0x0e,  /*! \brief Low power 11Hz output rate. */
    QMI8658_AccOdr_LowPower_3Hz = 0x0f    /*! \brief Low power 3Hz output rate. */
};

enum QMI8658_GyrRange
{
    QMI8658_GyrRange_32dps = 0 << 4,   /*! \brief +-32 degrees per second. */
    QMI8658_GyrRange_64dps = 1 << 4,   /*! \brief +-64 degrees per second. */
    QMI8658_GyrRange_128dps = 2 << 4,  /*! \brief +-128 degrees per second. */
    QMI8658_GyrRange_256dps = 3 << 4,  /*! \brief +-256 degrees per second. */
    QMI8658_GyrRange_512dps = 4 << 4,  /*! \brief +-512 degrees per second. */
    QMI8658_GyrRange_1024dps = 5 << 4, /*! \brief +-1024 degrees per second. */
    QMI8658_GyrRange_2048dps = 6 << 4, /*! \brief +-2048 degrees per second. */
    QMI8658_GyrRange_4096dps = 7 << 4  /*! \brief +-2560 degrees per second. */
};

/*!
 * \brief Gyroscope output rate configuration.
 */
enum QMI8658_GyrOdr
{
    QMI8658_GyrOdr_8000Hz = 0x00, /*! \brief High resolution 8000Hz output rate. */
    QMI8658_GyrOdr_4000Hz = 0x01, /*! \brief High resolution 4000Hz output rate. */
    QMI8658_GyrOdr_2000Hz = 0x02, /*! \brief High resolution 2000Hz output rate. */
    QMI8658_GyrOdr_1000Hz = 0x03, /*! \brief High resolution 1000Hz output rate. */
    QMI8658_GyrOdr_500Hz = 0x04,  /*! \brief High resolution 500Hz output rate. */
    QMI8658_GyrOdr_250Hz = 0x05,  /*! \brief High resolution 250Hz output rate. */
    QMI8658_GyrOdr_125Hz = 0x06,  /*! \brief High resolution 125Hz output rate. */
    QMI8658_GyrOdr_62_5Hz = 0x07, /*! \brief High resolution 62.5Hz output rate. */
    QMI8658_GyrOdr_31_25Hz = 0x08 /*! \brief High resolution 31.25Hz output rate. */
};

enum QMI8658_AeOdr
{
    QMI8658_AeOdr_1Hz = 0x00,   /*! \brief 1Hz output rate. */
    QMI8658_AeOdr_2Hz = 0x01,   /*! \brief 2Hz output rate. */
    QMI8658_AeOdr_4Hz = 0x02,   /*! \brief 4Hz output rate. */
    QMI8658_AeOdr_8Hz = 0x03,   /*! \brief 8Hz output rate. */
    QMI8658_AeOdr_16Hz = 0x04,  /*! \brief 16Hz output rate. */
    QMI8658_AeOdr_32Hz = 0x05,  /*! \brief 32Hz output rate. */
    QMI8658_AeOdr_64Hz = 0x06,  /*! \brief 64Hz output rate. */
    QMI8658_AeOdr_128Hz = 0x07, /*! \brief 128Hz output rate. */
    /*!
     * \brief Motion on demand mode.
     *
     * In motion on demand mode the application can trigger AttitudeEngine
     * output samples as necessary. This allows the AttitudeEngine to be
     * synchronized with external data sources.
     *
     * When in Motion on Demand mode the application should request new data
     * by calling the QMI8658_requestAttitudeEngineData() function. The
     * AttitudeEngine will respond with a data ready event (INT2) when the
     * data is available to be read.
     */
    QMI8658_AeOdr_motionOnDemand = 128
};

enum QMI8658_MagOdr
{
    QMI8658_MagOdr_1000Hz = 0x00, /*! \brief 1000Hz output rate. */
    QMI8658_MagOdr_500Hz = 0x01,  /*! \brief 500Hz output rate. */
    QMI8658_MagOdr_250Hz = 0x02,  /*! \brief 250Hz output rate. */
    QMI8658_MagOdr_125Hz = 0x03,  /*! \brief 125Hz output rate. */
    QMI8658_MagOdr_62_5Hz = 0x04, /*! \brief 62.5Hz output rate. */
    QMI8658_MagOdr_31_25Hz = 0x05 /*! \brief 31.25Hz output rate. */
};

enum QMI8658_MagDev
{
    QMI8658_MagDev_AKM09918 = (0 << 3), /*! \brief AKM09918. */
};

enum QMI8658_AccUnit
{
    QMI8658_AccUnit_g,  /*! \brief Accelerometer output in terms of g (9.81m/s^2). */
    QMI8658_AccUnit_ms2 /*! \brief Accelerometer output in terms of m/s^2. */
};

enum QMI8658_GyrUnit
{
    QMI8658_GyrUnit_dps, /*! \brief Gyroscope output in degrees/s. */
    QMI8658_GyrUnit_rads /*! \brief Gyroscope output in rad/s. */
};

// Forward declaration or move PedoConfig struct here
struct QMI8658_PedoConfig
{
    unsigned char sample_count;                 /*! \brief Indicates the count of sample batch/window for calculation (0-255) */
    unsigned char fix_peak2peak;                /*! \brief Indicates the threshold of the valid peak-to-peak detection (0-255) */
    unsigned char fix_peak;                     /*! \brief Indicates the threshold of the peak detection comparing to average (0-255) */
    unsigned char time_up;                      /*! \brief Indicates the maximum duration (timeout window) for a step. (0-255) */
    unsigned char time_low;                     /*! \brief Indicates the minimum duration for a step. (0-255) */
    unsigned char time_count_entry;             /*! \brief Indicates the minimum continuous steps to start the valid step counting. (0-255) */
    unsigned char fix_precision;                /*! \brief 0 is recommended (0-255) */
    unsigned char signal_count;                 /*! \brief The amount of steps when to update the pedometer output registers. (0-255) */
};

struct QMI8658_Config
{
    unsigned char inputSelection;           /*! \brief Sensor fusion input selection. */
    enum QMI8658_AccRange accRange;         /*! \brief Accelerometer dynamic range configuration. */
    enum QMI8658_AccOdr accOdr;             /*! \brief Accelerometer output rate. */
    enum QMI8658_GyrRange gyrRange;         /*! \brief Gyroscope dynamic range configuration. */
    enum QMI8658_GyrOdr gyrOdr;             /*! \brief Gyroscope output rate. */
    enum QMI8658_AeOdr aeOdr;               /*! \brief AttitudeEngine output rate. */
    enum QMI8658_MagOdr magOdr;             /*! \brief Magnetometer output data rate. */
    enum QMI8658_MagDev magDev;             /*! \brief Magnetometer device to use. */
    unsigned char enablePedometer;          /*! \brief Enable pedometer (1=enable, 0=disable). */
    struct QMI8658_PedoConfig pedoConfig;   /*! \brief Pedometer configuration. */
};

#define QMI8658_SAMPLE_SIZE (3 * sizeof(short))
#define QMI8658_AE_SAMPLE_SIZE ((4 + 3 + 1) * sizeof(short) + sizeof(unsigned char))
struct FisImuRawSample
{
    /*! \brief The sample counter of the sample. */
    unsigned char timestamp[3];
    /*!
     * \brief Pointer to accelerometer data in the sample buffer.
     *
     * \c NULL if no accelerometer data is available in the buffer.
     */
    unsigned char const *accelerometerData;
    /*!
     * \brief Pointer to gyroscope data in the sample buffer.
     *
     * \c NULL if no gyroscope data is available in the buffer.
     */
    unsigned char const *gyroscopeData;
    /*!
     * \brief Pointer to magnetometer data in the sample buffer.
     *
     * \c NULL if no magnetometer data is available in the buffer.
     */
    unsigned char const *magnetometerData;
    /*!
     * \brief Pointer to AttitudeEngine data in the sample buffer.
     *
     * \c NULL if no AttitudeEngine data is available in the buffer.
     */
    unsigned char const *attitudeEngineData;
    /*! \brief Raw sample buffer. */
    unsigned char sampleBuffer[QMI8658_SAMPLE_SIZE + QMI8658_AE_SAMPLE_SIZE];
    /*! \brief Contents of the FIS status 1 register. */
    unsigned char status1;
    // unsigned char status0;
    // unsigned int durT;
};

struct QMI8658_offsetCalibration
{
    enum QMI8658_AccUnit accUnit;
    float accOffset[3];
    enum QMI8658_GyrUnit gyrUnit;
    float gyrOffset[3];
};

struct QMI8658_sensitivityCalibration
{
    float accSensitivity[3];
    float gyrSensitivity[3];
};

enum QMI8658_Interrupt
{
    /*! \brief FIS INT1 line. */
    QMI8658_Int1 = (0 << 6),
    /*! \brief FIS INT2 line. */
    QMI8658_Int2 = (1 << 6)
};

enum QMI8658_InterruptState
{
    QMI8658_State_high = (1 << 7), /*! \brief Interrupt high. */
    QMI8658_State_low = (0 << 7)   /*! \brief Interrupt low. */
};

enum QMI8658_WakeOnMotionThreshold
{
    QMI8658_WomThreshold_high = 128, /*! \brief High threshold - large motion needed to wake. */
    QMI8658_WomThreshold_low = 32    /*! \brief Low threshold - small motion needed to wake. */
};

enum QMI8658_PedoMode
{
    QMI8658_PedoMode_Disable = 0x00,  /*! \brief Disable pedometer. */
    QMI8658_PedoMode_Enable = 0x01    /*! \brief Enable pedometer. */
};

enum QMI8658_PedoFixPeakMode
{
    QMI8658_PedoFixPeak_Disable = 0x00, /*! \brief Disable fix peak detection. */
    QMI8658_PedoFixPeak_Enable = 0x02   /*! \brief Enable fix peak detection. */
};

enum QMI8658_PedoFixPrecision
{
    QMI8658_PedoFixPrecision_Disable = 0x00, /*! \brief Disable fix precision mode. */
    QMI8658_PedoFixPrecision_Enable = 0x04   /*! \brief Enable fix precision mode. */
};

enum QMI8658_PedoTimeMode
{
    QMI8658_PedoTimeMode_Disable = 0x00, /*! \brief Disable time mode. */
    QMI8658_PedoTimeMode_Enable = 0x08   /*! \brief Enable time mode. */
};

enum QMI8658_PedoStepSizeMode
{
    QMI8658_PedoStepSize_Normal = 0x00,   /*! \brief Normal step size mode. */
    QMI8658_PedoStepSize_Sensitivity = 0x10 /*! \brief High sensitivity step size mode. */
};

extern unsigned char QMI8658_write_reg(unsigned char reg, unsigned char value);
extern unsigned char QMI8658_read_reg(unsigned char reg, unsigned char *buf, unsigned short len);
extern unsigned char QMI8658_init(struct QMI8658_Config configuration);
extern void QMI8658_config_apply(struct QMI8658_Config const *config);
extern void QMI8658_enable_sensors(unsigned char enableFlags);
extern void QMI8658_read_acc_xyz(float acc_xyz[3]);
extern void QMI8658_read_gyro_xyz(float gyro_xyz[3]);
extern void QMI8658_read_xyz(float acc[3], float gyro[3], unsigned int *tim_count);
extern void QMI8658_read_xyz_raw(short raw_acc_xyz[3], short raw_gyro_xyz[3], unsigned int *tim_count);
extern void QMI8658_read_ae(float quat[4], float velocity[3]);
extern unsigned char QMI8658_read_status0(void);
extern unsigned char QMI8658_read_status1(void);
extern float QMI8658_read_temp(void);
extern void QMI8658_enable_wake_on_motion(void);
extern void QMI8658_disable_wake_on_motion(void);
extern void QMI8658_config_pedometer(struct QMI8658_PedoConfig const *config);
extern void QMI8658_enable_pedometer(void);
extern void QMI8658_disable_pedometer(void);
extern void QMI8658_read_step_count(unsigned int *stepCount);
extern void QMI8658_reset_step_count(void);

#endif
