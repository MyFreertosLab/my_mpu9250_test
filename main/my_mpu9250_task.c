/*
 * my_mp9250_task.c
 *
 *  Created on: 29 gen 2021
 *      Author: andrea
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <my_mpu9250_task.h>
#include <mpu9250_accel.h>
#include <mpu9250_gyro.h>
#include <mpu9250_mag.h>
#include <mpu9250_baro.h>
#include <mpu9250_calibrator.h>

esp_err_t my_mpu9250_init_flash(){
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
	return err;
}


void my_mpu9250_task_init(mpu9250_handle_t mpu9250) {
	ESP_ERROR_CHECK(my_mpu9250_init_flash());
}

void my_mpu9250_temperature_read_data_cycle(mpu9250_handle_t mpu9250_handle) {
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 500 );
	uint32_t counter = 0;

	while (true) {
		counter++;
		if( ulTaskNotifyTake( pdTRUE,xMaxBlockTime ) == 1) {
			ESP_ERROR_CHECK(mpu9250_load_raw_data(mpu9250_handle));
			// TODO: probabilmente offset non è 0. Verificare
			if(counter%100 == 0) {
				float temp_deg = mpu9250_handle->raw_data.data_s_xyz.temp_data/333.87 + 21;

				printf("Temperature: [%2.3f][%d]\n", temp_deg, mpu9250_handle->raw_data.data_s_xyz.temp_data);
			}
	    } else {
	    	ESP_ERROR_CHECK(mpu9250_test_connection(mpu9250_handle));
			if(counter%100 == 0) {
		    	printf("SORRY!! Interrupt LOST!\n");
			}
	    }
	}
}

static esp_err_t mpu9250_discard_messages(mpu9250_handle_t mpu9250_handle, uint16_t num_msgs) {
	printf("Discarding %d Samples ... \n", num_msgs);
	for(uint16_t i = 0; i < num_msgs; i++) {
		ulTaskNotifyTake( pdTRUE,pdMS_TO_TICKS( 500 ) );
	}
	return ESP_OK;
}

void my_mpu9250_read_data_cycle(mpu9250_handle_t mpu9250_handle) {
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 500 );
	uint32_t counter = 0;
	ESP_ERROR_CHECK(mpu9250_gyro_set_fsr(mpu9250_handle, INV_FSR_1000DPS));
	ESP_ERROR_CHECK(mpu9250_acc_set_fsr(mpu9250_handle, INV_FSR_8G));
    ESP_ERROR_CHECK(mpu9250_mag_set_mode2_with_precision(mpu9250_handle, INV_MAG_PRECISION_16_BITS));
	ESP_ERROR_CHECK(mpu9250_mag_set_continuous_reading(mpu9250_handle));
	ESP_ERROR_CHECK(mpu9250_baro_init_calibration_data(mpu9250_handle));
	ESP_ERROR_CHECK(mpu9250_baro_set_continuous_reading(mpu9250_handle));
	ESP_ERROR_CHECK(mpu9250_discard_messages(mpu9250_handle, 10000));

	while (true) {
		counter++;
		if( ulTaskNotifyTake( pdTRUE,xMaxBlockTime ) == 1) {
			ESP_ERROR_CHECK(mpu9250_load_data(mpu9250_handle));
			if(counter%100 == 0) {
				printf("Gyro.: S[%d][%d][%d] X[%d][%d][%d]\n", mpu9250_handle->gyro.cal.kalman[X_POS].sample, mpu9250_handle->gyro.cal.kalman[Y_POS].sample, mpu9250_handle->gyro.cal.kalman[Z_POS].sample, mpu9250_handle->gyro.cal.kalman[X_POS].X , mpu9250_handle->gyro.cal.kalman[Y_POS].X, mpu9250_handle->gyro.cal.kalman[Z_POS].X);
				printf("Accel: S[%d][%d][%d] X[%d][%d][%d]\n", mpu9250_handle->accel.cal.kalman[X_POS].sample, mpu9250_handle->accel.cal.kalman[Y_POS].sample, mpu9250_handle->accel.cal.kalman[Z_POS].sample, mpu9250_handle->accel.cal.kalman[X_POS].X , mpu9250_handle->accel.cal.kalman[Y_POS].X, mpu9250_handle->accel.cal.kalman[Z_POS].X);
				printf("Mag: S[%d][%d][%d] X[%2.3f][%2.3f][%2.3f] [%3.3f]\n", mpu9250_handle->mag.cal.kalman[X_POS].sample, mpu9250_handle->mag.cal.kalman[Y_POS].sample, mpu9250_handle->mag.cal.kalman[Z_POS].sample, mpu9250_handle->mag.body_frame_data.array[X_POS], mpu9250_handle->mag.body_frame_data.array[Y_POS], mpu9250_handle->mag.body_frame_data.array[Z_POS], mpu9250_handle->mag.module);
				printf("Baro: P[%8.8f],T[%8.8f],A[%3.3f], RP[%d]\n", mpu9250_handle->baro.pressure, mpu9250_handle->baro.temperature, mpu9250_handle->baro.altitude, mpu9250_handle->raw_data.data_s_xyz.pressure);
				printf("AG-RPY[%2.2f][%2.2f][%2.2f]\n", mpu9250_handle->gyro.rpy.xyz.x*(double)360.0f/(double)6.283185307f, mpu9250_handle->gyro.rpy.xyz.y*(double)360.0f/(double)6.283185307f, mpu9250_handle->gyro.rpy.xyz.z*(double)360.0f/(double)6.283185307f);
				printf("AA-RPY[%2.2f][%2.2f][%2.2f]\n", mpu9250_handle->accel.rpy.xyz.x*(double)360.0f/(double)6.283185307f, mpu9250_handle->accel.rpy.xyz.y*(double)360.0f/(double)6.283185307f, mpu9250_handle->accel.rpy.xyz.z*(double)360.0f/(double)6.283185307f);
				printf("AM-RPY[%2.2f][%2.2f][%2.2f]\n", mpu9250_handle->mag.rpy.xyz.x*(double)360.0f/(double)6.283185307f, mpu9250_handle->mag.rpy.xyz.y*(double)360.0f/(double)6.283185307f, mpu9250_handle->mag.rpy.xyz.z*(double)360.0f/(double)6.283185307f);
//				printf("KG[%1.5f][%1.5f][%1.5f]\n", mpu9250_handle->gyro.cal.kalman[X_POS].K, mpu9250_handle->gyro.cal.kalman[Y_POS].K, mpu9250_handle->gyro.cal.kalman[Z_POS].K );
			}
	    } else {
	    	ESP_ERROR_CHECK(mpu9250_test_connection(mpu9250_handle));
			if(counter%100 == 0) {
		    	printf("SORRY!! Interrupt LOST!\n");
			}
	    }
	}
}

void my_mpu9250_task(void *arg) {
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 500 );
	// MPU9250 Handle
	mpu9250_init_t mpu9250;
	mpu9250_handle_t mpu9250_handle = &mpu9250;
	my_mpu9250_task_init(mpu9250_handle);

	mpu9250_cal_t calibrator;
	calibrator.mpu9250_handle = mpu9250_handle;
	mpu9250_cal_handle_t mpu9250_cal_handle = &calibrator;

	// Init MPU9250
	ESP_ERROR_CHECK(mpu9250_init(mpu9250_handle));

	// Calibration
	// calibration offset and biases
	ESP_ERROR_CHECK(mpu9250_calibrate(mpu9250_cal_handle));
	ESP_ERROR_CHECK(mpu9250_save_calibration_data(mpu9250_cal_handle));

	// load circular buffer
	for(uint8_t i = 0; i < CIRCULAR_BUFFER_SIZE; i++) {
		if( ulTaskNotifyTake( pdTRUE,xMaxBlockTime ) == 1) {
			ESP_ERROR_CHECK(mpu9250_load_raw_data(mpu9250_handle));
		}
	}
	my_mpu9250_read_data_cycle(mpu9250_handle);
}
