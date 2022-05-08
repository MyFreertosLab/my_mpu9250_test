/*
 * mpu9250_baro_calibrator.c
 *
 *  Created on: 26 set 2021
 *      Author: andrea
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <mpu9250_baro_calibrator.h>
#include <math.h>


/************************************************************************
 ************** P R I V A T E  I M P L E M E N T A T I O N **************
 ************************************************************************/

static esp_err_t mpu9250_baro_load_raw_data(mpu9250_handle_t mpu9250_handle) {
	uint16_t max_retry = 100;
	do {
		max_retry--;
		ulTaskNotifyTake( pdTRUE,pdMS_TO_TICKS( 500 ) );
		ESP_ERROR_CHECK(mpu9250_load_raw_data(mpu9250_handle));
	} while(!(mpu9250_handle->mag.drdy) && (max_retry != 0));

	return (max_retry == 0 ? ESP_FAIL : ESP_OK);
}
static esp_err_t mpu9250_baro_cal_init(mpu9250_handle_t mpu9250_handle) {
//	esp_error_check(mpu9250_baro_set_mode2_with_precision(mpu9250_handle, precision));
//    esp_error_check(mpu9250_baro_set_continuous_reading(mpu9250_handle));
	return ESP_OK;
}
static esp_err_t mpu9250_baro_cal_discard_messages(mpu9250_handle_t mpu9250_handle, uint16_t samples) {
	printf("Discarding %d Mag Samples ... \n", samples);
	for(uint16_t i = 0; i < samples; i++) {
	    ESP_ERROR_CHECK(mpu9250_baro_load_raw_data(mpu9250_handle));
	}
	return ESP_OK;
}

static esp_err_t mpu9250_baro_cal_calc_means(mpu9250_handle_t mpu9250_handle, uint8_t cycles) {
	int64_t mag_sum[3] = {0,0,0};

	for(uint8_t i = 0; i < 3; i++) {
		mpu9250_handle->mag.cal.means[mpu9250_handle->mag.precision].array[i] = 0;
	}

	printf("Calculating Mag Means with %d000 samples (wait for %d seconds)... \n", cycles, cycles);
	for(int j = 0; j < cycles*1000; j++) {
	    ESP_ERROR_CHECK(mpu9250_baro_load_raw_data(mpu9250_handle));

		for(uint8_t i = 0; i < 3; i++) {
			mag_sum[i] += mpu9250_handle->raw_data.data_s_vector.mag[i];
		}
	}
	for(uint8_t i = 0; i < 3; i++) {
		mpu9250_handle->mag.cal.means[mpu9250_handle->mag.precision].array[i] = mag_sum[i]/(cycles*1000);
	}
	return ESP_OK;
}

static esp_err_t mpu9250_baro_cal_calc_var(mpu9250_handle_t mpu9250_handle, uint8_t cycles) {
	uint64_t mag_sum[3] = {0,0,0};

	printf("Calculating Var with %d000 samples (wait for %d seconds)... \n", cycles, cycles);
	for(int j = 0; j < cycles*1000; j++) {
	    ESP_ERROR_CHECK(mpu9250_baro_load_raw_data(mpu9250_handle));
		for(uint8_t i = 0; i < 3; i++) {
			mag_sum[i] += (mpu9250_handle->raw_data.data_s_vector.mag[i] - mpu9250_handle->mag.cal.means[mpu9250_handle->mag.precision].array[i])*
					      (mpu9250_handle->raw_data.data_s_vector.mag[i] - mpu9250_handle->mag.cal.means[mpu9250_handle->mag.precision].array[i]);
		}
	}

	for(uint8_t i = 0; i < 3; i++) {
		mpu9250_handle->mag.cal.var[mpu9250_handle->mag.precision].array[i] = (uint16_t)(mag_sum[i]/(cycles*1000));
	}
	printf("Mag_var: [%d][%d][%d]\n",
			mpu9250_handle->mag.cal.var[mpu9250_handle->mag.precision].array[0],
			mpu9250_handle->mag.cal.var[mpu9250_handle->mag.precision].array[1],
			mpu9250_handle->mag.cal.var[mpu9250_handle->mag.precision].array[2]);

	return ESP_OK;
}

static esp_err_t mpu9250_baro_cal_calc_sqm(mpu9250_handle_t mpu9250_handle) {
	for(uint8_t i = 0; i < 3; i++) {
		mpu9250_handle->mag.cal.sqm[mpu9250_handle->mag.precision].array[i] = sqrt(mpu9250_handle->mag.cal.var[mpu9250_handle->mag.precision].array[i]);
	}
	printf("Mag_sqm: [%d][%d][%d]\n",
			mpu9250_handle->mag.cal.sqm[mpu9250_handle->mag.precision].array[0],
			mpu9250_handle->mag.cal.sqm[mpu9250_handle->mag.precision].array[1],
			mpu9250_handle->mag.cal.sqm[mpu9250_handle->mag.precision].array[2]);
	return ESP_OK;
}

static esp_err_t mpu9250_baro_cal_calc_bias(mpu9250_handle_t mpu9250_handle) {
	// Calc Bias
	{
		printf("Calculating Mag Bias ... \n");

		memset(mpu9250_handle->mag.cal.var[mpu9250_handle->mag.precision].array, 0, sizeof(mpu9250_handle->mag.cal.var[mpu9250_handle->mag.precision].array));       //Zero out var
		memset(mpu9250_handle->mag.cal.sqm[mpu9250_handle->mag.precision].array, 0, sizeof(mpu9250_handle->mag.cal.sqm[mpu9250_handle->mag.precision].array));       //Zero out sqm

		ESP_ERROR_CHECK(mpu9250_baro_cal_calc_var(mpu9250_handle, MPU9250_BARO_CAL_MAX_KSAMPLE_CYCLES));
		ESP_ERROR_CHECK(mpu9250_baro_cal_calc_sqm(mpu9250_handle));

	}
	return ESP_OK;
}

/************************************************************************
 ****************** A P I  I M P L E M E N T A T I O N ******************
 ************************************************************************/
esp_err_t mpu9250_baro_calibrate(mpu9250_handle_t mpu9250_handle) {
	ESP_ERROR_CHECK(mpu9250_baro_cal_init(mpu9250_handle));
	ESP_ERROR_CHECK(mpu9250_baro_cal_discard_messages(mpu9250_handle, 2000));
	ESP_ERROR_CHECK(mpu9250_baro_cal_calc_means(mpu9250_handle, MPU9250_BARO_CAL_MAX_KSAMPLE_CYCLES));
	ESP_ERROR_CHECK(mpu9250_baro_cal_calc_bias(mpu9250_handle));
	return ESP_OK;
}
