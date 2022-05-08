/*
 * mpu9250_baro_calibrator.h
 *
 *  Created on: 26 set 2021
 *      Author: andrea
 */

#ifndef COMPONENTS_MPU9250_INCLUDE_MPU9250_BARO_CALIBRATOR_H_
#define COMPONENTS_MPU9250_INCLUDE_MPU9250_BARO_CALIBRATOR_H_

#define MPU9250_BARO_CAL_MAX_KSAMPLE_CYCLES 10

#include <mpu9250_baro.h>

esp_err_t mpu9250_baro_calibrate(mpu9250_handle_t mpu9250_handle);
esp_err_t mpu9250_baro_save_calibration_data(mpu9250_handle_t mpu9250_handle);


#endif /* COMPONENTS_MPU9250_INCLUDE_MPU9250_BARO_CALIBRATOR_H_ */
