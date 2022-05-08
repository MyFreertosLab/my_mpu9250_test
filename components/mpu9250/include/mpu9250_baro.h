/*
 * mpu9250_baro.h
 *
 *  Created on: 30 apr 2022
 *      Author: andrea
 */

#ifndef COMPONENTS_DRIVERS_MPU9250_INCLUDE_MPU9250_BARO_H_
#define COMPONENTS_DRIVERS_MPU9250_INCLUDE_MPU9250_BARO_H_

#include <mpu9250.h>

#define BMP388_ADDRESS   0x76
#define BMP388_CHIP_ID   0x50

#define BMP388_REG_CHIP_ID 0x00
#define BMP388_REG_CHIP_ID                        0x00
#define BMP388_REG_ERR                            0x02
#define BMP388_REG_SENS_STATUS                    0x03
#define BMP388_REG_DATA                           0x04
#define BMP388_REG_EVENT                          0x10
#define BMP388_REG_INT_STATUS                     0x11
#define BMP388_REG_FIFO_LENGTH                    0x12
#define BMP388_REG_FIFO_DATA                      0x14
#define BMP388_REG_FIFO_WM                        0x15
#define BMP388_REG_FIFO_CONFIG_1                  0x17
#define BMP388_REG_FIFO_CONFIG_2                  0x18
#define BMP388_REG_INT_CTRL                       0x19
#define BMP388_REG_IF_CONF                        0x1A
#define BMP388_REG_PWR_CTRL                       0x1B
#define BMP388_REG_OSR                            0X1C
#define BMP388_REG_ODR                            0x1D
#define BMP388_REG_CONFIG                         0x1F
#define BMP388_REG_CALIB_DATA                     0x31
#define BMP388_REG_CMD                            0x7E
#define BMP388_NUM_BYTES_CALIB_DATA               0x15

#define BMP388_NO_OVERSAMPLING                    0x00
#define BMP388_OVERSAMPLING_2X                    0x01
#define BMP388_OVERSAMPLING_4X                    0x02
#define BMP388_OVERSAMPLING_8X                    0x03
#define BMP388_OVERSAMPLING_16X                   0x04
#define BMP388_OVERSAMPLING_32X                   0x05

#define BMP388_IIR_FILTER_DISABLE                 0x00
#define BMP388_IIR_FILTER_COEFF_1                 0x01
#define BMP388_IIR_FILTER_COEFF_3                 0x02
#define BMP388_IIR_FILTER_COEFF_7                 0x03
#define BMP388_IIR_FILTER_COEFF_15                0x04
#define BMP388_IIR_FILTER_COEFF_31                0x05
#define BMP388_IIR_FILTER_COEFF_63                0x06
#define BMP388_IIR_FILTER_COEFF_127               0x07

#define BMP388_ODR_200_HZ                         0x00
#define BMP388_ODR_100_HZ                         0x01
#define BMP388_ODR_50_HZ                          0x02
#define BMP388_ODR_25_HZ                          0x03
#define BMP388_ODR_12_5_HZ                        0x04
#define BMP388_ODR_6_25_HZ                        0x05
#define BMP388_ODR_3_1_HZ                         0x06
#define BMP388_ODR_1_5_HZ                         0x07
#define BMP388_ODR_0_78_HZ                        0x08
#define BMP388_ODR_0_39_HZ                        0x09
#define BMP388_ODR_0_2_HZ                         0x0A
#define BMP388_ODR_0_1_HZ                         0x0B
#define BMP388_ODR_0_05_HZ                        0x0C
#define BMP388_ODR_0_02_HZ                        0x0D
#define BMP388_ODR_0_01_HZ                        0x0E
#define BMP388_ODR_0_006_HZ                       0x0F
#define BMP388_ODR_0_003_HZ                       0x10
#define BMP388_ODR_0_001_HZ                       0x11

#define BMP388_SOFT_RESET                         0xB6
#define BMP388_PWR_CTRL_PRESSURE_ENABLED          0x01
#define BMP388_PWR_CTRL_TEMPERATURE_ENABLED       0x02
#define BMP388_PWR_CTRL_NORMAL_MODE               0x30
#define BMP388_SEA_LEVEL_PRESSURE_HPA             1013.25f

esp_err_t mpu9250_baro_test(mpu9250_handle_t mpu9250_handle);
esp_err_t mpu9250_baro_init(mpu9250_handle_t mpu9250_handle);
esp_err_t mpu9250_baro_load_coefficients(mpu9250_handle_t mpu9250_handle);
esp_err_t mpu9250_baro_set_oversampling(mpu9250_handle_t mpu9250_handle, uint8_t osr_t, uint8_t osr_p);
esp_err_t mpu9250_baro_get_oversampling(mpu9250_handle_t mpu9250_handle, uint8_t* osr);
esp_err_t mpu9250_baro_set_IIR_coefficient(mpu9250_handle_t mpu9250_handle, uint8_t iir_coeff);
esp_err_t mpu9250_baro_set_output_data_rate(mpu9250_handle_t mpu9250_handle, uint8_t odr_hz);
esp_err_t mpu9250_baro_set_pwr_ctrl(mpu9250_handle_t mpu9250_handle, uint8_t value);
esp_err_t mpu9250_baro_set_continuous_reading(mpu9250_handle_t mpu9250_handle);
esp_err_t mpu9250_baro_compensate(mpu9250_handle_t mpu9250_handle);

#endif /* COMPONENTS_DRIVERS_MPU9250_INCLUDE_MPU9250_BARO_H_ */
