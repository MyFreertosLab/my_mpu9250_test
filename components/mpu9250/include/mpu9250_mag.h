/*
 * mpu9250_mag.h
 *
 *  Created on: 25 set 2021
 *      Author: andrea
 */

#ifndef COMPONENTS_MPU9250_INCLUDE_MPU9250_MAG_H_
#define COMPONENTS_MPU9250_INCLUDE_MPU9250_MAG_H_

#include <mpu9250.h>

// public methods
esp_err_t mpu9250_mag_init(mpu9250_handle_t mpu9250_handle);
esp_err_t mpu9250_mag_load_precision(mpu9250_handle_t mpu9250_handle);
esp_err_t mpu9250_mag_set_precision(mpu9250_handle_t mpu9250_handle, uint8_t fsr);
esp_err_t mpu9250_mag_update_state(mpu9250_handle_t mpu9250_handle);
esp_err_t mpu9250_mag_save_calibration_data (mpu9250_handle_t mpu9250_handle);
esp_err_t mpu9250_mag_set_mode2_with_precision(mpu9250_handle_t mpu9250_handle, uint8_t precision);
esp_err_t mpu9250_mag_set_continuous_reading(mpu9250_handle_t mpu9250_handle);
esp_err_t mpu9250_mag_scale_data_in_body_frame(mpu9250_handle_t mpu9250_handle);
#endif /* COMPONENTS_MPU9250_INCLUDE_MPU9250_MAG_H_ */
