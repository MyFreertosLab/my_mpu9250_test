/*
 * mpu9250_mag_calibrator.c
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
#include <mpu9250_mag_calibrator.h>
#include <math.h>


/************************************************************************
 ************** P R I V A T E  I M P L E M E N T A T I O N **************
 ************************************************************************/

static esp_err_t mpu9250_mag_load_raw_data(mpu9250_handle_t mpu9250_handle) {
	uint16_t max_retry = 100;
	do {
		max_retry--;
		ulTaskNotifyTake( pdTRUE,pdMS_TO_TICKS( 500 ) );
		ESP_ERROR_CHECK(mpu9250_load_raw_data(mpu9250_handle));
	} while(!(mpu9250_handle->mag.drdy) && (max_retry != 0));

	return (max_retry == 0 ? ESP_FAIL : ESP_OK);
}
static esp_err_t mpu9250_mag_cal_init(mpu9250_handle_t mpu9250_handle, uint8_t precision) {
	ESP_ERROR_CHECK(mpu9250_mag_set_mode2_with_precision(mpu9250_handle, precision));
    ESP_ERROR_CHECK(mpu9250_mag_set_continuous_reading(mpu9250_handle));
	return ESP_OK;
}
static esp_err_t mpu9250_mag_cal_discard_messages(mpu9250_handle_t mpu9250_handle, uint16_t samples) {
	printf("Discarding %d Mag Samples ... \n", samples);
	for(uint16_t i = 0; i < samples; i++) {
	    ESP_ERROR_CHECK(mpu9250_mag_load_raw_data(mpu9250_handle));
	}
	return ESP_OK;
}

static esp_err_t mpu9250_mag_cal_calc_means(mpu9250_handle_t mpu9250_handle, uint8_t cycles) {
	int64_t mag_sum[3] = {0,0,0};

	for(uint8_t i = 0; i < 3; i++) {
		mpu9250_handle->mag.cal.means[mpu9250_handle->mag.precision].array[i] = 0;
	}

	printf("Calculating Mag Means with %d000 samples (wait for %d seconds)... \n", cycles, cycles);
	for(int j = 0; j < cycles*1000; j++) {
	    ESP_ERROR_CHECK(mpu9250_mag_load_raw_data(mpu9250_handle));

		for(uint8_t i = 0; i < 3; i++) {
			mag_sum[i] += mpu9250_handle->raw_data.data_s_vector.mag[i];
		}
	}
	for(uint8_t i = 0; i < 3; i++) {
		mpu9250_handle->mag.cal.means[mpu9250_handle->mag.precision].array[i] = mag_sum[i]/(cycles*1000);
	}
	return ESP_OK;
}

static esp_err_t mpu9250_mag_cal_calc_var(mpu9250_handle_t mpu9250_handle, uint8_t cycles) {
	uint64_t mag_sum[3] = {0,0,0};

	printf("Calculating Var with %d000 samples (wait for %d seconds)... \n", cycles, cycles);
	for(int j = 0; j < cycles*1000; j++) {
	    ESP_ERROR_CHECK(mpu9250_mag_load_raw_data(mpu9250_handle));
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

static esp_err_t mpu9250_mag_cal_calc_sqm(mpu9250_handle_t mpu9250_handle) {
	for(uint8_t i = 0; i < 3; i++) {
		mpu9250_handle->mag.cal.sqm[mpu9250_handle->mag.precision].array[i] = sqrt(mpu9250_handle->mag.cal.var[mpu9250_handle->mag.precision].array[i]);
	}
	printf("Mag_sqm: [%d][%d][%d]\n",
			mpu9250_handle->mag.cal.sqm[mpu9250_handle->mag.precision].array[0],
			mpu9250_handle->mag.cal.sqm[mpu9250_handle->mag.precision].array[1],
			mpu9250_handle->mag.cal.sqm[mpu9250_handle->mag.precision].array[2]);
	return ESP_OK;
}

static esp_err_t mpu9250_mag_cal_calc_bias(mpu9250_handle_t mpu9250_handle) {
	// Calc Bias
	{
		printf("Calculating Mag Bias ... \n");

		memset(mpu9250_handle->mag.cal.var[mpu9250_handle->mag.precision].array, 0, sizeof(mpu9250_handle->mag.cal.var[mpu9250_handle->mag.precision].array));       //Zero out var
		memset(mpu9250_handle->mag.cal.sqm[mpu9250_handle->mag.precision].array, 0, sizeof(mpu9250_handle->mag.cal.sqm[mpu9250_handle->mag.precision].array));       //Zero out sqm

		ESP_ERROR_CHECK(mpu9250_mag_cal_calc_var(mpu9250_handle, MPU9250_MAG_CAL_MAX_KSAMPLE_CYCLES));
		ESP_ERROR_CHECK(mpu9250_mag_cal_calc_sqm(mpu9250_handle));

	}
	return ESP_OK;
}

//Mag axis offsets [193,210,191]
static esp_err_t mpu9250_mag_cal_calc_max_min(mpu9250_handle_t mpu9250_handle) {
	float max[3] = {-4912, -4912, -4912};
	float min[3] = {4912, 4912, 4912};
	for(uint32_t i = 0; i < 18000; i++ ) {
		if((i % 1000) == 0) {
			printf("cycle n. [%d]\n", i);
		}
		ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS( 500 ));
		ESP_ERROR_CHECK(mpu9250_mag_load_raw_data(mpu9250_handle));
		ESP_ERROR_CHECK(mpu9250_mag_update_state(mpu9250_handle));

		for(uint8_t i = 0; i < 3; i++) {
			if(max[i] < mpu9250_handle->mag.body_frame_data.array[i]) {
				max[i] = mpu9250_handle->mag.body_frame_data.array[i];
			}
			if(min[i] > mpu9250_handle->mag.body_frame_data.array[i]) {
				min[i] = mpu9250_handle->mag.body_frame_data.array[i];
			}
		}
	}

	float range[3] = {0.0f, 0.0f, 0.0f};
	for(uint8_t i = 0; i < 3; i++) {
		range[i] = (max[i] - min[i]);
	}
	float range_mean = (range[X_POS] + range[Y_POS] + range[Z_POS])/3.0f;
	for(uint8_t i = 0; i < 3; i++) {
		mpu9250_handle->mag.cal.scale_factors2[mpu9250_handle->mag.precision].array[i] = range[i]/range_mean;
		max[i] *= mpu9250_handle->mag.cal.scale_factors2[mpu9250_handle->mag.precision].array[i];
		min[i] *= mpu9250_handle->mag.cal.scale_factors2[mpu9250_handle->mag.precision].array[i];
		mpu9250_handle->mag.cal.scaled_offset[mpu9250_handle->mag.precision].array[i] = (max[i] + min[i])/2.0f;
	}
	printf("Mag soff: [%2.5f,%2.5f,%2.5f]\n",
			mpu9250_handle->mag.cal.scaled_offset[mpu9250_handle->mag.precision].array[X_POS],
			mpu9250_handle->mag.cal.scaled_offset[mpu9250_handle->mag.precision].array[Y_POS],
			mpu9250_handle->mag.cal.scaled_offset[mpu9250_handle->mag.precision].array[Z_POS]
		  );
	printf("Mag sf2: [%2.5f,%2.5f,%2.5f]\n",
			mpu9250_handle->mag.cal.scale_factors2[mpu9250_handle->mag.precision].array[X_POS],
			mpu9250_handle->mag.cal.scale_factors2[mpu9250_handle->mag.precision].array[Y_POS],
			mpu9250_handle->mag.cal.scale_factors2[mpu9250_handle->mag.precision].array[Z_POS]
		  );
	return ESP_OK;
}

/************************************************************************
 ****************** A P I  I M P L E M E N T A T I O N ******************
 ************************************************************************/
esp_err_t mpu9250_mag_calibrate(mpu9250_handle_t mpu9250_handle) {
	ESP_ERROR_CHECK(mpu9250_mag_cal_init(mpu9250_handle, INV_MAG_PRECISION_14_BITS));
	ESP_ERROR_CHECK(mpu9250_mag_cal_discard_messages(mpu9250_handle, 2000));
	ESP_ERROR_CHECK(mpu9250_mag_cal_calc_means(mpu9250_handle, MPU9250_MAG_CAL_MAX_KSAMPLE_CYCLES));
	ESP_ERROR_CHECK(mpu9250_mag_cal_calc_bias(mpu9250_handle));

	printf("********************************************************************************************************\n");
	printf("ROTATE SENSOR IN ALL DIRECTIONS\n");
	printf("********************************************************************************************************\n");
	ESP_ERROR_CHECK(mpu9250_mag_cal_calc_max_min(mpu9250_handle));
	printf("Mag offset calculated at 0.6 precision\n");
	printf("********************************************************************************************************\n");

	ESP_ERROR_CHECK(mpu9250_mag_cal_init(mpu9250_handle, INV_MAG_PRECISION_16_BITS));
	ESP_ERROR_CHECK(mpu9250_mag_cal_discard_messages(mpu9250_handle, 2000));
	ESP_ERROR_CHECK(mpu9250_mag_cal_calc_means(mpu9250_handle, MPU9250_MAG_CAL_MAX_KSAMPLE_CYCLES));
	ESP_ERROR_CHECK(mpu9250_mag_cal_calc_bias(mpu9250_handle));

	printf("********************************************************************************************************\n");
	printf("ROTATE SENSOR IN ALL DIRECTIONS\n");
	printf("********************************************************************************************************\n");
	ESP_ERROR_CHECK(mpu9250_mag_cal_calc_max_min(mpu9250_handle));
	printf("Mag offset calculated at 0.15 precision\n");
	printf("********************************************************************************************************\n");

	return ESP_OK;
}
