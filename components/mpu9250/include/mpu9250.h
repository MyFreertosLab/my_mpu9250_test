#ifndef _MPU9250_H_
#define _MPU9250_H_

#include <esp_err.h>
#include <driver/spi_master.h>
#include "freertos/task.h"

#define MPU9250_ID 0x70
#define MPU9250_ID_1 0x71
#define X_POS 0
#define Y_POS 1
#define Z_POS 2

enum mpu9250_register {
	MPU9250_SELF_TEST_X_GYRO =  0x00,
	MPU9250_SELF_TEST_Y_GYRO =  0x01,
	MPU9250_SELF_TEST_Z_GYRO =  0x02,
	MPU9250_SELF_TEST_X_ACCEL = 0x0D,
	MPU9250_SELF_TEST_Y_ACCEL = 0x0E,
	MPU9250_SELF_TEST_Z_ACCEL = 0x0F,
	MPU9250_XG_OFFSET_H =       0x13,
	MPU9250_XG_OFFSET_L =       0x14,
	MPU9250_YG_OFFSET_H =       0x15,
	MPU9250_YG_OFFSET_L =       0x16,
	MPU9250_ZG_OFFSET_H =       0x17,
	MPU9250_ZG_OFFSET_L =       0x18,
	MPU9250_SMPLRT_DIV =        0x19,
	MPU9250_CONFIG =            0x1A,
	MPU9250_GYRO_CONFIG =       0x1B,
	MPU9250_ACCEL_CONFIG =      0x1C,
	MPU9250_ACCEL_CONFIG_2 =    0x1D,
	MPU9250_LP_ACCEL_ODR =      0x1E,
	MPU9250_WOM_THR =           0x1F,
	MPU9250_FIFO_EN =           0x23,
	MPU9250_I2C_MST_CTRL =      0x24,
	MPU9250_I2C_SLV0_ADDR =     0x25,
	MPU9250_I2C_SLV0_REG =      0x26,
	MPU9250_I2C_SLV0_CTRL =     0x27,
	MPU9250_I2C_SLV1_ADDR =     0x28,
	MPU9250_I2C_SLV1_REG =      0x29,
	MPU9250_I2C_SLV1_CTRL =     0x2A,
	MPU9250_I2C_SLV2_ADDR =     0x2B,
	MPU9250_I2C_SLV2_REG =      0x2C,
	MPU9250_I2C_SLV2_CTRL =     0x2D,
	MPU9250_I2C_SLV3_ADDR =     0x2E,
	MPU9250_I2C_SLV3_REG =      0x2F,
	MPU9250_I2C_SLV3_CTRL =     0x30,
	MPU9250_I2C_SLV4_ADDR =     0x31,
	MPU9250_I2C_SLV4_REG =      0x32,
	MPU9250_I2C_SLV4_DO =       0x33,
	MPU9250_I2C_SLV4_CTRL =     0x34,
	MPU9250_I2C_SLV4_DI =       0x35,
	MPU9250_I2C_MST_STATUS =    0x36,
	MPU9250_INT_PIN_CFG =       0x37,
	MPU9250_INT_ENABLE =        0x38,
	MPU9250_INT_STATUS =        0x3A,
	MPU9250_ACCEL_XOUT_H =      0x3B,
	MPU9250_ACCEL_XOUT_L =      0x3C,
	MPU9250_ACCEL_YOUT_H =      0x3D,
	MPU9250_ACCEL_YOUT_L =      0x3E,
	MPU9250_ACCEL_ZOUT_H =      0x3F,
	MPU9250_ACCEL_ZOUT_L =      0x40,
	MPU9250_TEMP_OUT_H =        0x41,
	MPU9250_TEMP_OUT_L =        0x42,
	MPU9250_GYRO_XOUT_H =       0x43,
	MPU9250_GYRO_XOUT_L =       0x44,
	MPU9250_GYRO_YOUT_H =       0x45,
	MPU9250_GYRO_YOUT_L =       0x46,
	MPU9250_GYRO_ZOUT_H =       0x47,
	MPU9250_GYRO_ZOUT_L =       0x48,
	MPU9250_EXT_SENS_DATA_00 =  0x49,
	MPU9250_EXT_SENS_DATA_01 =  0x4A,
	MPU9250_EXT_SENS_DATA_02 =  0x4B,
	MPU9250_EXT_SENS_DATA_03 =  0x4C,
	MPU9250_EXT_SENS_DATA_04 =  0x4D,
	MPU9250_EXT_SENS_DATA_05 =  0x4E,
	MPU9250_EXT_SENS_DATA_06 =  0x4F,
	MPU9250_EXT_SENS_DATA_07 =  0x50,
	MPU9250_EXT_SENS_DATA_08 =  0x51,
	MPU9250_EXT_SENS_DATA_09 =  0x52,
	MPU9250_EXT_SENS_DATA_10 =  0x53,
	MPU9250_EXT_SENS_DATA_11 =  0x54,
	MPU9250_EXT_SENS_DATA_12 =  0x55,
	MPU9250_EXT_SENS_DATA_13 =  0x56,
	MPU9250_EXT_SENS_DATA_14 =  0x57,
	MPU9250_EXT_SENS_DATA_15 =  0x58,
	MPU9250_EXT_SENS_DATA_16 =  0x59,
	MPU9250_EXT_SENS_DATA_17 =  0x5A,
	MPU9250_EXT_SENS_DATA_18 =  0x5B,
	MPU9250_EXT_SENS_DATA_19 =  0x5C,
	MPU9250_EXT_SENS_DATA_20 =  0x5D,
	MPU9250_EXT_SENS_DATA_21 =  0x5E,
	MPU9250_EXT_SENS_DATA_22 =  0x5F,
	MPU9250_EXT_SENS_DATA_23 =  0x60,
	MPU9250_I2C_SLV0_DO =       0x63,
	MPU9250_I2C_SLV1_DO =       0x64,
	MPU9250_I2C_SLV2_DO =       0x65,
	MPU9250_I2C_SLV3_DO =       0x66,
	MPU9250_I2C_MST_DELAY_CTRL =0x67,
	MPU9250_SIGNAL_PATH_RESET = 0x68,
	MPU9250_MOT_DETECT_CTRL =   0x69,
	MPU9250_USER_CTRL =         0x6A,
	MPU9250_PWR_MGMT_1 =        0x6B,
	MPU9250_PWR_MGMT_2 =        0x6C,
	MPU9250_FIFO_COUNTH =       0x72,
	MPU9250_FIFO_COUNTL =       0x73,
	MPU9250_FIFO_R_W =          0x74,
	MPU9250_WHO_AM_I =          0x75,
	MPU9250_XA_OFFSET_H =       0x77,
	MPU9250_XA_OFFSET_L =       0x78,
	MPU9250_YA_OFFSET_H =       0x7A,
	MPU9250_YA_OFFSET_L =       0x7B,
	MPU9250_ZA_OFFSET_H =       0x7D,
	MPU9250_ZA_OFFSET_L =       0x7E
};
#define MPU9250_WHO_AM_I_RESULT 0x70

/*********************************
*********** Gyroscope ************
*********************************/
enum gyro_config_bits {
	GYRO_CONFIG_FCHOICE_B = 0,
	GYRO_CONFIG_GYRO_FS_SEL = 3,
	GYRO_CONFIG_ZGYRO_CTEN = 5,
	GYRO_CONFIG_YGYRO_CTEN = 6,
	GYRO_CONFIG_XGYRO_CTEN = 7,
};

/* Gyro Full scale ranges. */
enum gyro_fsr_e {
    INV_FSR_250DPS = 0,
    INV_FSR_500DPS,
    INV_FSR_1000DPS,
    INV_FSR_2000DPS,
    NUM_GYRO_FSR
};

#define MPU9250_GYRO_FSR_MASK     0xE7
#define MPU9250_GYRO_FSR_LBIT     0x03
#define MPU9250_GYRO_FCHOICE_MASK 0x03

/*********************************
********* Accelerometer **********
*********************************/
enum accel_config_bit {
	ACCEL_CONFIG_ACCEL_FS_SEL = 3,
	ACCEL_CONFIG_AZ_ST_EN = 5,
	ACCEL_CONFIG_AY_ST_EN = 6,
	ACCEL_CONFIG_AX_ST_EN = 7,
};
#define MPU9250_ACC_FSR_MASK    0xE7
#define MPU9250_ACC_FSR_LBIT    0x03


enum accel_config_2_bits {
	ACCEL_CONFIG_2_A_DLPFCFG = 0,
	ACCEL_CONFIG_2_ACCEL_FCHOICE_B = 3,
};

/* Accel Full scale ranges. */
enum accel_fsr_e {
    INV_FSR_2G = 0,
    INV_FSR_4G,
    INV_FSR_8G,
    INV_FSR_16G,
    NUM_ACCEL_FSR
};
/* Low-power accel wakeup rates. */
enum lp_accel_rate_e {
    INV_LPA_0_3125HZ,
    INV_LPA_0_625HZ,
    INV_LPA_1_25HZ,
    INV_LPA_2_5HZ,
    INV_LPA_5HZ,
    INV_LPA_10HZ,
    INV_LPA_20HZ,
    INV_LPA_40HZ,
    INV_LPA_80HZ,
    INV_LPA_160HZ,
    INV_LPA_320HZ,
    INV_LPA_640HZ
};

/*********************************mpu9250_handle
********* Magnetometer ***********
*********************************/
typedef enum  {
    INV_MAG_PRECISION_14_BITS = 0,
    INV_MAG_PRECISION_16_BITS
} mag_precision_e;


/*********************************
******* Power Management *********
*********************************/
enum pwr_mgmt_1_bits {
	PWR_MGMT_1_CLKSEL = 0,
	PWR_MGMT_1_PD_PTAT = 3,
	PWR_MGMT_1_GYRO_STANDBY = 4,
	PWR_MGMT_1_CYCLE = 5,
	PWR_MGMT_1_SLEEP = 6,
	PWR_MGMT_1_H_RESET = 7
};

enum pwr_mgmt_2_bits {
	PWR_MGMT_2_DISABLE_ZG = 0,
	PWR_MGMT_2_DISABLE_YG = 1,
	PWR_MGMT_2_DISABLE_XG = 2,
	PWR_MGMT_2_DISABLE_ZA = 3,
	PWR_MGMT_2_DISABLE_YA = 4,
	PWR_MGMT_2_DISABLE_XA = 5,
};

/*********************************
*********** Interrupts ***********
*********************************/
enum interrupt_status_bits {
	INT_STATUS_RAW_DATA_RDY_INT = 0,
	INT_STATUS_RAW_DMP_INT = 1,
	INT_STATUS_FSYNC_INT = 3,
	INT_STATUS_FIFO_OVERFLOW_INT = 4,
	INT_STATUS_WOM_INT = 6,
};

enum int_enable_bits {
	INT_ENABLE_RAW_RDY_EN = 0,
	INT_ENABLE_FSYNC_INT_EN = 3,
	INT_ENABLE_FIFO_OVERFLOW_EN = 4,
	INT_ENABLE_WOM_EN = 6,
};

enum int_pin_cfg_bits {
	INT_PIN_CFG_BYPASS_EN = 1,
	INT_PIN_CFG_FSYNC_INT_MODE_EN = 2,
	INT_PIN_CFG_ACTL_FSYNC = 3,
	INT_PIN_CFG_INT_ANYRD_2CLEAR = 4,
	INT_PIN_CFG_LATCH_INT_EN = 5,
	INT_PIN_CFG_OPEN = 6,
	INT_PIN_CFG_ACTL = 7,
};
#define INT_PIN_CFG_INT_MASK 0xF0

/*********************************
************* Compass ************
*********************************/
enum ak8963_register {
	AK8963_WIA = 0x0,
	AK8963_INFO = 0x1,
	AK8963_ST1 = 0x2,
	AK8963_HXL = 0x3,
	AK8963_HXH = 0x4,
	AK8963_HYL = 0x5,
	AK8963_HYH = 0x6,
	AK8963_HZL = 0x7,
	AK8963_HZH = 0x8,
	AK8963_ST2 = 0x9,
	AK8963_CNTL1 = 0x0A,
	AK8963_CNTL2 = 0x0B,
	AK8963_ASTC = 0xC,
	AK8963_TS1 = 0xD,
	AK8963_TS2 = 0xE,
	AK8963_I2CDIS = 0xF,
	AK8963_ASAX = 0x10,
	AK8963_ASAY = 0x11,
	AK8963_ASAZ = 0x12,
};

#define MAG_CTRL_OP_MODE_MASK 0xF
#define AK8963_ST1_DRDY_BIT 0
#define AK8963_WHO_AM_I_RESULT 0x48
#define AK8963_ADDRESS 0x0C
#define AK8963_PWR_DOWN 0x00
#define I2C_SLV_EN 0x80
#define MPU9250_I2C_READ_FLAG 0x80
#define MPU9250_RESET 0x80
#define AK8963_RESET 0x01
#define AK8963_SINGLE_MEASUREMENT 0x01
#define I2C_MST_EN 0x20
#define I2C_MST_CLK 0x0D
#define H_RESET 0x80
#define CLKSEL_PLL 0x01
#define AK8963_FUSE_ROM 0x0F
#define AK8963_PRECISION_MASK 0x10
#define AK8963_MODE_2 0x06

/*********************************
********* Low Pass Filter ********
*********************************/
enum lpf_e {
    INV_FILTER_256HZ_NOLPF2 = 0,
    INV_FILTER_188HZ,
    INV_FILTER_98HZ,
    INV_FILTER_42HZ,
    INV_FILTER_20HZ,
    INV_FILTER_10HZ,
    INV_FILTER_5HZ,
    INV_FILTER_2100HZ_NOLPF,
    NUM_FILTER
};


/*********************************
********* Clock Sources **********
*********************************/
enum clock_sel_e {
    INV_CLK_INTERNAL = 0,
    INV_CLK_PLL,
    NUM_CLK
};
/*********************************
*********** Utilities ************
*********************************/
typedef union {
   int8_t array[3];
   struct {
     int8_t x;
     int8_t y;
     int8_t z;
   } xyz;
} mpu9250_int8_3d_t;

typedef union {
   uint8_t array[3];
   struct {
     uint8_t x;
     uint8_t y;
     uint8_t z;
   } xyz;
} mpu9250_uint8_3d_t;

typedef union {
   int16_t array[3];
   struct {
     int16_t x;
     int16_t y;
     int16_t z;
   } xyz;
} mpu9250_int_3d_t;

typedef union {
   int32_t array[3];
   struct {
     int32_t x;
     int32_t y;
     int32_t z;
   } xyz;
} mpu9250_int32_3d_t;

typedef union {
   int64_t array[3];
   struct {
     int64_t x;
     int64_t y;
     int64_t z;
   } xyz;
} mpu9250_int64_3d_t;

typedef union {
   uint16_t array[3];
   struct {
     uint16_t x;
     uint16_t y;
     uint16_t z;
   } xyz;
} mpu9250_uint_3d_t;

typedef union {
   uint32_t array[3];
   struct {
     uint32_t x;
     uint32_t y;
     uint32_t z;
   } xyz;
} mpu9250_uint32_3d_t;

typedef union {
   uint64_t array[3];
   struct {
     uint64_t x;
     uint64_t y;
     uint64_t z;
   } xyz;
} mpu9250_uint64_3d_t;

typedef union {
   double array[3];
   struct {
     double x;
     double y;
     double z;
   } xyz;
} mpu9250_double_3d_t;

typedef union {
   float array[3];
   struct {
     float x;
     float y;
     float z;
   } xyz;
} mpu9250_float_3d_t;

/* Se necessario usare
 * float fixed point
 * 22bit integer, 10bit decimal
 * precision: 1/(2^10-1) = 0,000977517
 *
 *
 */

/*
 * we assume this cycle:
 * Calc Predition:
 *   X(k)=A*X(k-1)+B*u(k-1)
 *   P(k)=A*P(k-1)*A'+Q
 * Calc Update:
 *   K(k)=P(k)H'(H*P(k)*H'+R)^(-1)
 *   X(k)=X(k)-K(k)(Sample(k)-H*X(k))
 *   P(k)=(I-K(k)*H)*P(k)
 * with:
 *   A=H=1,
 *   B=Q=0,
 *   R=variance of state,
 * then:
 *   initialization:
 *     X(0)=fixed expected response
 *     P(0)=1
 *     Q=1.5
 *   cycle for each sample
 *     X(k)=X(k-1)
 *     P(k)=P(k-1)+Q
 *     K(k)=P(k)/(P(k)+R)
 *     X(k)=X(k)+K(k)*(Sample(k)-X(k))
 *     P(k)=(1-K(k))*P(k)
 */
typedef struct {
	uint8_t initialized;
	int16_t X,sample;
	uint16_t R;
	float P,Q,K;
} mpu9250_kalman_t;

/* Offsets */
typedef mpu9250_int_3d_t mpu9250_offset_t;
typedef mpu9250_offset_t* mpu9250_offset_buff_t;

/* Media */
typedef mpu9250_int_3d_t mpu9250_means_t;
typedef mpu9250_means_t* mpu9250_means_buff_t;

/* Varianza */
typedef mpu9250_uint_3d_t mpu9250_var_t;
typedef mpu9250_var_t* mpu9250_var_buff_t;

/* Scarto quadratico medio */
typedef mpu9250_int_3d_t mpu9250_sqm_t;
typedef mpu9250_sqm_t* mpu9250_sqm_buff_t;

typedef mpu9250_float_3d_t mpu9250_scale_factors_t;
typedef mpu9250_float_3d_t mpu9250_scaled_offset_t;


/* RPY */
typedef mpu9250_double_3d_t mpu9250_rpy_t;

typedef struct {
    mpu9250_offset_t offset;
    mpu9250_means_t means[4];
    mpu9250_var_t var[4];
    mpu9250_sqm_t sqm[4];
    mpu9250_kalman_t kalman[3];
} mpu9250_cal_data_t;

typedef struct {
    mpu9250_uint8_3d_t asa;
	mpu9250_scale_factors_t scale_factors; // factory factors
    mpu9250_offset_t offset[2];
    mpu9250_means_t means[4];
    mpu9250_var_t var[4];
    mpu9250_sqm_t sqm[4];
    mpu9250_kalman_t kalman[3];
	mpu9250_scale_factors_t scale_factors2[2]; // correction of factory factors
    mpu9250_scaled_offset_t scaled_offset[2];
} mpu9250_mag_cal_data_t;

typedef struct {
	uint8_t initialized;
	uint32_t X,sample;
	uint32_t R;
	uint32_t variance;
	uint32_t sqm;
	uint32_t means;
	double P,Q,K;
} mpu9250_kalman_u32_t;

typedef struct {
	uint8_t initialized;
	float X,sample;
	float R;
	float variance;
	float P,Q,K;
} mpu9250_kalman_float_t;

typedef struct {
    uint16_t par_t1;
    uint16_t par_t2;
    int8_t par_t3;
    int16_t par_p1;
    int16_t par_p2;
    int8_t par_p3;
    int8_t par_p4;
    uint16_t par_p5;
    uint16_t par_p6;
    int8_t par_p7;
    int8_t par_p8;
    int16_t par_p9;
    int8_t par_p10;
    int8_t par_p11;
    double q_par_t1;
    double q_par_t2;
    double q_par_t3;
    double q_par_p1;
    double q_par_p2;
    double q_par_p3;
    double q_par_p4;
    double q_par_p5;
    double q_par_p6;
    double q_par_p7;
    double q_par_p8;
    double q_par_p9;
    double q_par_p10;
    double q_par_p11;
    mpu9250_kalman_u32_t kalman_pressure;
    mpu9250_kalman_u32_t kalman_temperature;
    mpu9250_kalman_float_t kalman_vspeed;
} mpu9250_baro_cal_data_t;

/* Circular Buffer */
#define CIRCULAR_BUFFER_SIZE 5
typedef struct {
	int16_t data[CIRCULAR_BUFFER_SIZE];
	uint8_t cursor;
} mpu9250_cb_t;
typedef mpu9250_cb_t* mpu9250_cb_handle_t;

/* Circular Buffer */
typedef struct {
	float data[CIRCULAR_BUFFER_SIZE];
	uint8_t cursor;
} mpu9250_cb_float_t;
typedef mpu9250_cb_float_t* mpu9250_cb_float_handle_t;


#define PI_2 6.283185307f
#define PI 3.141592654f
#define PI_HALF 1.570796327f

/********************************************************************************************************************
********** MPU9250 API **********************************************************************************************
********************************************************************************************************************/
typedef union {
   struct {
     int16_t accel[3];
     int16_t temp;
     int16_t gyro[3];
     int16_t mag[3];
     uint32_t pressure;
     uint32_t temperature;
     int16_t ext[5];
   } data_s_vector;
   struct {
     int16_t accel_data_x;
     int16_t accel_data_y;
     int16_t accel_data_z;
     int16_t temp_data;
     int16_t gyro_data_x;
     int16_t gyro_data_y;
     int16_t gyro_data_z;
     int16_t mag_data_x;
     int16_t mag_data_y;
     int16_t mag_data_z;
     uint32_t pressure;
     uint32_t temperature;
     int16_t ext_data[5];
   } data_s_xyz;
} mpu9250_raw_data_t;
typedef mpu9250_raw_data_t* mpu9250_raw_data_buff_t;

/*********************************
********* ACCELEROMETER **********
*********************************/
typedef struct mpu9250_accel_s {
    mpu9250_cal_data_t cal; // calibration data
	uint8_t fsr;
    uint16_t lsb;
	mpu9250_cb_t cb[3]; // circular buffer
	mpu9250_rpy_t rpy;
} mpu9250_accel_t;

/*********************************
*********** GYROSCOPE ************
*********************************/
typedef struct mpu9250_gyro_s {
    mpu9250_cal_data_t cal; // calibration data
	uint8_t fsr;
    float lsb;
	mpu9250_cb_t cb[3]; // circular buffer
	mpu9250_rpy_t rpy;
} mpu9250_gyro_t;

typedef uint8_t mpu9250_int_status_t;

/*********************************
********* MAGNETOMETER ***********
*********************************/
typedef struct mpu9250_mag_s {
	mpu9250_mag_cal_data_t cal; // mag calibration data
	mag_precision_e precision;
    float precision_factor;
	mpu9250_rpy_t rpy;
    mpu9250_float_3d_t inertial_frame_data; // toIntertialFrame(body_frame_data)
    mpu9250_float_3d_t body_frame_data; // toBodyFrame(inertial_data)
    float module;
    uint8_t drdy;
} mpu9250_mag_t;

/*********************************
********* BAROMETER ***********
*********************************/
typedef struct mpu9250_baro_s {
	mpu9250_baro_cal_data_t cal;
	double pressure;
	double temperature;
	float altitude;
	uint8_t present;
    uint8_t drdy;
	mpu9250_cb_float_t cb_vspeed; // circular buffer
} mpu9250_baro_t;

/*********************************
******** MPU9250 HANDLE **********
*********************************/
typedef struct mpu9250_init_s {
	spi_bus_config_t buscfg;
	spi_device_interface_config_t devcfg;
	spi_device_handle_t device_handle;
	TaskHandle_t data_ready_task_handle;

	// int config and status
	int int_pin;
    uint8_t int_status;

    // MPU9250 id
    uint8_t whoami;

    mpu9250_raw_data_t raw_data;

    mpu9250_accel_t accel;
    mpu9250_gyro_t gyro;
    mpu9250_mag_t mag;
    mpu9250_baro_t baro;

	double attitude[3];

	float cy; // cos(Yaw)
	float cp; // cos(Pitch)
	float cr; // cos(Roll)
	float sy; // sin(Yaw)
	float sp; // sin(Pitch)
	float sr; // sin(Roll)


} mpu9250_init_t;

typedef mpu9250_init_t* mpu9250_handle_t;

#define MPU9250_READ_FLAG 0x80

/* Private Methods */
void mpu9250_cb_add(mpu9250_cb_handle_t cb, int16_t val);
void mpu9250_cb_means(mpu9250_cb_handle_t cb, int16_t* mean);
void mpu9250_cb_last(mpu9250_cb_handle_t cb, int16_t* val);
void mpu9250_cb_float_add(mpu9250_cb_float_handle_t cb, float val);
void mpu9250_cb_float_means(mpu9250_cb_float_handle_t cb, float* mean);
void mpu9250_cb_float_last(mpu9250_cb_float_handle_t cb, float* val);

/* Public Methids */
/* Set up APIs */
esp_err_t mpu9250_init(mpu9250_handle_t mpu9250_handle);
esp_err_t mpu9250_test_connection(mpu9250_handle_t mpu9250_handle);
esp_err_t mpu9250_load_whoami(mpu9250_handle_t mpu9250_handle);
esp_err_t mpu9250_load_int_status(mpu9250_handle_t mpu9250_handle);
esp_err_t mpu9250_load_raw_data(mpu9250_handle_t mpu9250_handle);
esp_err_t mpu9250_load_data(mpu9250_handle_t mpu9250_handle);

#endif // _MPU9250_H_
