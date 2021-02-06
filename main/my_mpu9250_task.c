/*
 * my_mp9250_task.c
 *
 *  Created on: 29 gen 2021
 *      Author: andrea
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <my_mpu9250_task.h>
#include <mpu9250_accel.h>

void my_mpu9250_task_init(mpu9250_handle_t mpu9250) {
}

void my_mpu9250_task(void *arg) {
	// MPU9250 Handle
	mpu9250_init_t mpu9250;
	mpu9250_handle_t mpu9250_handle = &mpu9250;

	// Init MPU9250
	ESP_ERROR_CHECK(mpu9250_init(mpu9250_handle));

	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 500 );

	// calibration offset and biases
	ESP_ERROR_CHECK(mpu9250_acc_calibrate(mpu9250_handle));


	// set accel full scale range = 4G
	ESP_ERROR_CHECK(mpu9250_acc_set_fsr(mpu9250_handle, INV_FSR_4G));
	ESP_ERROR_CHECK(mpu9250_discard_messages(mpu9250_handle, 1000));

	uint32_t counter = 0;
	while (true) {
		counter++;
		if( ulTaskNotifyTake( pdTRUE,xMaxBlockTime ) == 1) {
			ESP_ERROR_CHECK(mpu9250_load_raw_data(mpu9250_handle));

			// TODO: Definire struttura dati stimati in mu9250.h
			// TODO: Assegnare i dati stimati alla struttura dati

			if(counter%100 == 0) {
				// TODO: fare funzione conversione RAW -> G
				int8_t sx = (mpu9250_handle->raw_data.data_s_xyz.accel_data_x < 0 ? -1 : 1);
				int8_t sy = (mpu9250_handle->raw_data.data_s_xyz.accel_data_y < 0 ? -1 : 1);
				int8_t sz = (mpu9250_handle->raw_data.data_s_xyz.accel_data_z < 0 ? -1 : 1);

				// correggo con lo scarto quadratico medio
				mpu9250_handle->raw_data.data_s_xyz.accel_data_x -= sx*(mpu9250_handle->acc_sqm[mpu9250_handle->acc_fsr].xyz.x);
				mpu9250_handle->raw_data.data_s_xyz.accel_data_y -= sy*(mpu9250_handle->acc_sqm[mpu9250_handle->acc_fsr].xyz.y);
				mpu9250_handle->raw_data.data_s_xyz.accel_data_z -= sz*(mpu9250_handle->acc_sqm[mpu9250_handle->acc_fsr].xyz.z);

				// esprimo in g
				int16_t xg = (mpu9250_handle->raw_data.data_s_xyz.accel_data_x*1000/mpu9250_handle->acc_lsb);
				int16_t yg = (mpu9250_handle->raw_data.data_s_xyz.accel_data_y*1000/mpu9250_handle->acc_lsb);
				int16_t zg = (mpu9250_handle->raw_data.data_s_xyz.accel_data_z*1000/mpu9250_handle->acc_lsb);

				printf("Acc_X_H/L/V [%d][%d]\n", xg, mpu9250_handle->raw_data.data_s_xyz.accel_data_x);
				printf("Acc_Y_H/L/V [%d][%d]\n", yg, mpu9250_handle->raw_data.data_s_xyz.accel_data_y);
				printf("Acc_Z_H/L/V [%d][%d]\n", zg, mpu9250_handle->raw_data.data_s_xyz.accel_data_z);
			}

			if(counter <= 20000) {
				if(mpu9250_handle->acc_fsr != INV_FSR_4G) {
					ESP_ERROR_CHECK(mpu9250_acc_set_fsr(mpu9250_handle, INV_FSR_4G));
				}
			} else if(counter > 20000 && counter <= 40000) {
				if(mpu9250_handle->acc_fsr != INV_FSR_8G) {
					ESP_ERROR_CHECK(mpu9250_acc_set_fsr(mpu9250_handle, INV_FSR_8G));
				}
			} else if(counter > 40000 && counter <= 60000) {
				if(mpu9250_handle->acc_fsr != INV_FSR_2G) {
					ESP_ERROR_CHECK(mpu9250_acc_set_fsr(mpu9250_handle, INV_FSR_2G));
				}
			} else if(counter > 60000 && counter <= 80000) {
				if(mpu9250_handle->acc_fsr != INV_FSR_16G) {
					ESP_ERROR_CHECK(mpu9250_acc_set_fsr(mpu9250_handle, INV_FSR_16G));
				}
			} else if(counter > 80000) {
			    counter = 0;
			}
	    } else {
	    	ESP_ERROR_CHECK(mpu9250_test_connection(mpu9250_handle));
			if(counter%100 == 0) {
		    	printf("SORRY!! Interrupt LOST!\n");
			}
	    }
	}


}
