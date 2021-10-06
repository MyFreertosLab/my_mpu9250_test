/*
 * mpu9250_mag_calibrator.h
 *
 *  Created on: 26 set 2021
 *      Author: andrea
 */

#ifndef COMPONENTS_MPU9250_INCLUDE_MPU9250_MAG_CALIBRATOR_H_
#define COMPONENTS_MPU9250_INCLUDE_MPU9250_MAG_CALIBRATOR_H_

#define MPU9250_MAG_CAL_MAX_KSAMPLE_CYCLES 10

#include <mpu9250_mag.h>

esp_err_t mpu9250_mag_calibrate(mpu9250_handle_t mpu9250_handle);
esp_err_t mpu9250_mag_save_calibration_data(mpu9250_handle_t mpu9250_handle);


#endif /* COMPONENTS_MPU9250_INCLUDE_MPU9250_MAG_CALIBRATOR_H_ */
