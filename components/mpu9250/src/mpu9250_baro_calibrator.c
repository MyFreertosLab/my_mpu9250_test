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
	} while(!(mpu9250_handle->baro.drdy) && (max_retry != 0));

	return (max_retry == 0 ? ESP_FAIL : ESP_OK);
}
static esp_err_t mpu9250_baro_cal_init(mpu9250_handle_t mpu9250_handle) {
	mpu9250_handle->baro.cal.kalman_pressure.variance = 0;
	mpu9250_handle->baro.cal.kalman_temperature.variance = 0;
	mpu9250_handle->baro.cal.kalman_pressure.sqm = 0;
	mpu9250_handle->baro.cal.kalman_temperature.sqm = 0;
	mpu9250_handle->baro.cal.kalman_pressure.means = 0;
	mpu9250_handle->baro.cal.kalman_temperature.means = 0;
	return ESP_OK;
}

static esp_err_t mpu9250_baro_cal_discard_messages(mpu9250_handle_t mpu9250_handle, uint16_t samples) {
	printf("Discarding %d Baro Samples ... \n", samples);
	for(uint16_t i = 0; i < samples; i++) {
	    ESP_ERROR_CHECK(mpu9250_baro_load_raw_data(mpu9250_handle));
	}
	return ESP_OK;
}

static esp_err_t mpu9250_baro_cal_calc_means(mpu9250_handle_t mpu9250_handle, uint8_t cycles) {
	uint64_t baro_sum_pressure = 0;
	uint64_t baro_sum_temperature = 0;

	mpu9250_handle->baro.cal.kalman_pressure.means = 0;
	mpu9250_handle->baro.cal.kalman_temperature.means = 0;

	printf(
			"Calculating BMP388 Means with %d000 samples (wait for %d seconds)... \n",
			cycles, cycles);
	for(uint8_t i = 0; i < cycles; i++) {
		uint64_t baro_sum_pressure_j = 0;
		uint64_t baro_sum_temperature_j = 0;
		for (uint8_t j = 0; j < 10; j++) {
			uint64_t baro_sum_pressure_k = 0;
			uint64_t baro_sum_temperature_k = 0;
			for(uint8_t k = 0; k < 10; k++) {
				uint64_t baro_sum_pressure_s = 0;
				uint64_t baro_sum_temperature_s = 0;
				for(uint8_t s = 0; s < 10; s++) {
					ESP_ERROR_CHECK(mpu9250_baro_load_raw_data(mpu9250_handle));
					baro_sum_pressure_s += mpu9250_handle->raw_data.data_s_vector.pressure;
					baro_sum_temperature_s += mpu9250_handle->raw_data.data_s_vector.temperature;
				}
				baro_sum_pressure_k += (baro_sum_pressure_s/10);
				baro_sum_temperature_k += (baro_sum_temperature_s/10);
			}
			baro_sum_pressure_j += (baro_sum_pressure_k/10);
			baro_sum_temperature_j += (baro_sum_temperature_k/10);
		}
		baro_sum_pressure += (baro_sum_pressure_j/10);
		baro_sum_temperature += (baro_sum_temperature_j/10);
	}
	mpu9250_handle->baro.cal.kalman_pressure.means =
			(baro_sum_pressure / (cycles));
	mpu9250_handle->baro.cal.kalman_temperature.means =
			(baro_sum_temperature / (cycles));
	printf("BMP388::mpu9250_baro_cal_calc_means P[%d] T[%d]\n", mpu9250_handle->baro.cal.kalman_pressure.means, mpu9250_handle->baro.cal.kalman_temperature.means);
	return ESP_OK;
}

static esp_err_t mpu9250_baro_cal_calc_var(mpu9250_handle_t mpu9250_handle, uint8_t cycles) {
	uint64_t baro_sum_pressure = 0;
	uint64_t baro_sum_temperature = 0;

	printf("Calculating Var with %d000 samples (wait for %d seconds)... \n", cycles, cycles);
	for(uint8_t i = 0; i < cycles; i++) {
		uint64_t baro_sum_pressure_j = 0;
		uint64_t baro_sum_temperature_j = 0;
		for(uint8_t j = 0; j < 10; j++) {
			uint64_t baro_sum_pressure_k = 0;
			uint64_t baro_sum_temperature_k = 0;
			for(uint8_t k = 0; k < 10; k++) {
				uint64_t baro_sum_pressure_s = 0;
				uint64_t baro_sum_temperature_s = 0;
				for(uint8_t s = 0; s < 10; s++) {
				    ESP_ERROR_CHECK(mpu9250_baro_load_raw_data(mpu9250_handle));
					baro_sum_pressure_s += (mpu9250_handle->raw_data.data_s_vector.pressure - mpu9250_handle->baro.cal.kalman_pressure.means)*
								      (mpu9250_handle->raw_data.data_s_vector.pressure - mpu9250_handle->baro.cal.kalman_pressure.means);
					baro_sum_temperature_s += (mpu9250_handle->raw_data.data_s_vector.temperature - mpu9250_handle->baro.cal.kalman_temperature.means)*
								      (mpu9250_handle->raw_data.data_s_vector.temperature - mpu9250_handle->baro.cal.kalman_temperature.means);
				}
				baro_sum_pressure_k += (baro_sum_pressure_s/10);
				baro_sum_temperature_k += (baro_sum_temperature_s/10);
			}
			baro_sum_pressure_j += (baro_sum_pressure_k/10);
			baro_sum_temperature_j += (baro_sum_temperature_k/10);
		}
		baro_sum_pressure += (baro_sum_pressure_j/10);
		baro_sum_temperature += (baro_sum_temperature_j/10);
		printf("BMP388::mpu9250_baro_cal_calc_var SUM P[%d] T[%d]\n", (uint32_t)(baro_sum_pressure/cycles), (uint32_t)(baro_sum_temperature/cycles));
	}

	mpu9250_handle->baro.cal.kalman_pressure.variance = baro_sum_pressure/(cycles);
	mpu9250_handle->baro.cal.kalman_temperature.variance = baro_sum_temperature/(cycles);
	printf("BMP388 variance: P[%d] T[%d]\n",
			mpu9250_handle->baro.cal.kalman_pressure.variance,
			mpu9250_handle->baro.cal.kalman_temperature.variance);
	return ESP_OK;
}

static esp_err_t mpu9250_baro_cal_calc_sqm(mpu9250_handle_t mpu9250_handle) {
	mpu9250_handle->baro.cal.kalman_pressure.sqm = sqrt(mpu9250_handle->baro.cal.kalman_pressure.variance);
	mpu9250_handle->baro.cal.kalman_temperature.sqm = sqrt(mpu9250_handle->baro.cal.kalman_temperature.variance);
	printf("BMP388 sqm: P[%d] T[%d]\n",
			mpu9250_handle->baro.cal.kalman_pressure.sqm,
			mpu9250_handle->baro.cal.kalman_temperature.sqm);
	return ESP_OK;
}

static esp_err_t mpu9250_baro_cal_calc_bias(mpu9250_handle_t mpu9250_handle) {
	// Calc Bias
	{
		printf("Calculating Baro Bias ... \n");
		mpu9250_handle->baro.cal.kalman_pressure.variance = 0;
		mpu9250_handle->baro.cal.kalman_temperature.variance = 0;
		mpu9250_handle->baro.cal.kalman_pressure.sqm = 0;
		mpu9250_handle->baro.cal.kalman_temperature.sqm = 0;

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
