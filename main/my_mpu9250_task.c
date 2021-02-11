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
#include <mpu9250_gyro.h>

void my_mpu9250_task_init(mpu9250_handle_t mpu9250) {
}
void my_mpu9250_acc_static_calibration(mpu9250_handle_t mpu9250_handle) {
	// calibration offset and biases
	ESP_ERROR_CHECK(mpu9250_acc_calibrate(mpu9250_handle));

}


typedef struct {
	int16_t data[20];
	uint8_t cursor;
} my_mpu9250_circular_buffer_t;
typedef my_mpu9250_circular_buffer_t* my_mpu9250_circular_buffer_p_t;
void my_mpu9250_cb_init(my_mpu9250_circular_buffer_p_t cb) {
	cb->cursor = -1;
	memset(cb, 0, sizeof(my_mpu9250_circular_buffer_t));
}

void my_mpu9250_cb_add(my_mpu9250_circular_buffer_p_t cb, int16_t val) {
	cb->cursor++;
	cb->cursor %= 20;
	cb->data[cb->cursor] = val;
}
void my_mpu9250_cb_means(my_mpu9250_circular_buffer_p_t cb, int16_t* mean) {
	int64_t sum = 0;
	for(uint8_t i = 0; i < 20; i++) {
		sum += cb->data[i];
	}
	*mean = sum/20;
}

/*
 * we assume this cycle:
 * Calc Predition:
 *   X(k)=A*X(k-1)+B*u(k-1)
 *   P(k)=A*P(k-1)*A'+Q
 * Calc Update:
 *   K(k)=P(k)H'(H*P(k)*H'+R)^(-1)
 *   X(k)=X(k)+K(k)(Sample(k)-H*X(k))
 *   P(k)=(I-K(k)*H)*P(k)
 * with:
 *   A=H=1,
 *   B=0,
 *   Q=1.5
 *   R=variance of state,
 * then:
 *   initialization:
 *     X(0)=fixed expected response
 *     P(0)=1
 *     Q=1.5
 *   cycle for each sample
 *     X(k)=X(k-1)
 *     P(k)=P(k-1) + Q
 *     K(k)=P(k)/(P(k)+R)
 *     X(k)=X(k)-K(k)*(Sample(k)-X(k))
 *     P(k)=(1-K(k))*P(k)
 */
void my_mpu9250_acc_read_data_cycle(mpu9250_handle_t mpu9250_handle) {
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 500 );

	mpu9250_kalman_t acc_z_kalman;
	acc_z_kalman.X = mpu9250_handle->acc_lsb;
	acc_z_kalman.sample=0;
	acc_z_kalman.P=1.0f;
	acc_z_kalman.Q=1.5;
	acc_z_kalman.K=0.0f;
	acc_z_kalman.R=mpu9250_handle->acc_var[mpu9250_handle->acc_fsr].xyz.z;

	// prepare circular buffer
	my_mpu9250_circular_buffer_t cbt;
	my_mpu9250_circular_buffer_p_t cb = &cbt;
	my_mpu9250_cb_init(cb);
	for(uint8_t i = 0; i < 20; i++) {
		if( ulTaskNotifyTake( pdTRUE,xMaxBlockTime ) == 1) {
			ESP_ERROR_CHECK(mpu9250_load_raw_data(mpu9250_handle));
			my_mpu9250_cb_add(cb, mpu9250_handle->raw_data.data_s_xyz.accel_data_z);
		}
	}

	// read cycle
	uint32_t counter = 0;
	int64_t k_err_means_sum = 0;
	int64_t err_means_sum = 0;
	while (true) {
		counter++;
		if( ulTaskNotifyTake( pdTRUE,xMaxBlockTime ) == 1) {
			ESP_ERROR_CHECK(mpu9250_load_raw_data(mpu9250_handle));
			my_mpu9250_cb_add(cb, mpu9250_handle->raw_data.data_s_xyz.accel_data_z);

			/*
			 *     X(k)=X(k-1)
			 *     P(k)=P(k-1)+Q
			 *     K(k)=P(k)/(P(k)+R)
			 *     X(k)=X(k)+K(k)*(Sample(k)-X(k))
			 *     P(k)=(1-K(k))*P(k)
			 */

			my_mpu9250_cb_means(cb, &acc_z_kalman.sample);
			acc_z_kalman.P=acc_z_kalman.P+acc_z_kalman.Q;
			acc_z_kalman.K = acc_z_kalman.P/(acc_z_kalman.P+acc_z_kalman.R);
			acc_z_kalman.X = acc_z_kalman.X + acc_z_kalman.K*(acc_z_kalman.sample - acc_z_kalman.X);
			acc_z_kalman.P=(1-acc_z_kalman.K)*acc_z_kalman.P;
			k_err_means_sum += (acc_z_kalman.X - mpu9250_handle->acc_lsb);
			err_means_sum += (acc_z_kalman.sample - mpu9250_handle->acc_lsb);
			if(counter%100 == 0) {
				float knoise = 1.0f*(k_err_means_sum/counter);
				float noise = 1.0f*(err_means_sum/counter);
				// esprimo in g
				int16_t xg = (mpu9250_handle->raw_data.data_s_xyz.accel_data_x*1000/mpu9250_handle->acc_lsb);
				int16_t yg = (mpu9250_handle->raw_data.data_s_xyz.accel_data_y*1000/mpu9250_handle->acc_lsb);
				int16_t zg = (mpu9250_handle->raw_data.data_s_xyz.accel_data_z*1000/mpu9250_handle->acc_lsb);
                int16_t zgc = acc_z_kalman.X*1000/mpu9250_handle->acc_lsb;

//				printf("Acc_X_H/L/V [%d][%d]\n", xg, mpu9250_handle->raw_data.data_s_xyz.accel_data_x);
//				printf("Acc_Y_H/L/V [%d][%d]\n", yg, mpu9250_handle->raw_data.data_s_xyz.accel_data_y);
//				printf("Acc_Z_H/L/V [%d][%d]\n", zg, mpu9250_handle->raw_data.data_s_xyz.accel_data_z);
				printf("Kalman KE[%3.5f],E[%3.5f],X[%d],P[%3.5f],K[%3.8f],Z[%d],zgc[%d],zg[%d]\n",
						knoise, noise, acc_z_kalman.X, acc_z_kalman.P, acc_z_kalman.K, acc_z_kalman.sample,zgc, zg);

			}

//			if(counter <= 20000) {
//				if(mpu9250_handle->acc_fsr != INV_FSR_4G) {
//					ESP_ERROR_CHECK(mpu9250_acc_set_fsr(mpu9250_handle, INV_FSR_4G));
//				}
//			} else if(counter > 20000 && counter <= 40000) {
//				if(mpu9250_handle->acc_fsr != INV_FSR_8G) {
//					ESP_ERROR_CHECK(mpu9250_acc_set_fsr(mpu9250_handle, INV_FSR_8G));
//				}
//			} else if(counter > 40000 && counter <= 60000) {
//				if(mpu9250_handle->acc_fsr != INV_FSR_2G) {
//					ESP_ERROR_CHECK(mpu9250_acc_set_fsr(mpu9250_handle, INV_FSR_2G));
//				}
//			} else if(counter > 60000 && counter <= 80000) {
//				if(mpu9250_handle->acc_fsr != INV_FSR_16G) {
//					ESP_ERROR_CHECK(mpu9250_acc_set_fsr(mpu9250_handle, INV_FSR_16G));
//				}
//			} else if(counter > 80000) {
//			    counter = 0;
//			}
	    } else {
	    	ESP_ERROR_CHECK(mpu9250_test_connection(mpu9250_handle));
			if(counter%100 == 0) {
		    	printf("SORRY!! Interrupt LOST!\n");
			}
	    }
	}
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

void my_mpu9250_gyro_static_calibration(mpu9250_handle_t mpu9250_handle) {
	// calibration offset and biases
	ESP_ERROR_CHECK(mpu9250_gyro_calibrate(mpu9250_handle));


	// set gyroel full scale range = 4G
	ESP_ERROR_CHECK(mpu9250_gyro_set_fsr(mpu9250_handle, INV_FSR_2000DPS));
	ESP_ERROR_CHECK(mpu9250_discard_messages(mpu9250_handle, 10000));

}
void my_mpu9250_gyro_read_data_cycle(mpu9250_handle_t mpu9250_handle) {
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 500 );
	uint32_t counter = 0;

	while (true) {
		counter++;
		if( ulTaskNotifyTake( pdTRUE,xMaxBlockTime ) == 1) {
			ESP_ERROR_CHECK(mpu9250_load_raw_data(mpu9250_handle));

			if(counter%100 == 0) {
				// esprimo in g
				int16_t mxrs = (mpu9250_handle->raw_data.data_s_xyz.gyro_data_x*1000/mpu9250_handle->gyro_lsb);
				int16_t myrs = (mpu9250_handle->raw_data.data_s_xyz.gyro_data_y*1000/mpu9250_handle->gyro_lsb);
				int16_t mzrs = (mpu9250_handle->raw_data.data_s_xyz.gyro_data_z*1000/mpu9250_handle->gyro_lsb);

				printf("Gyro_X_H/L/V [%d][%d]\n", mxrs, mpu9250_handle->raw_data.data_s_xyz.gyro_data_x);
				printf("Gyro_Y_H/L/V [%d][%d]\n", myrs, mpu9250_handle->raw_data.data_s_xyz.gyro_data_y);
				printf("Gyro_Z_H/L/V [%d][%d]\n", mzrs, mpu9250_handle->raw_data.data_s_xyz.gyro_data_z);
			}

			if(counter <= 20000) {
				if(mpu9250_handle->gyro_fsr != INV_FSR_250DPS) {
					ESP_ERROR_CHECK(mpu9250_gyro_set_fsr(mpu9250_handle, INV_FSR_250DPS));
				}
			} else if(counter > 20000 && counter <= 40000) {
				if(mpu9250_handle->gyro_fsr != INV_FSR_500DPS) {
					ESP_ERROR_CHECK(mpu9250_gyro_set_fsr(mpu9250_handle, INV_FSR_500DPS));
				}
			} else if(counter > 40000 && counter <= 60000) {
				if(mpu9250_handle->gyro_fsr != INV_FSR_1000DPS) {
					ESP_ERROR_CHECK(mpu9250_gyro_set_fsr(mpu9250_handle, INV_FSR_1000DPS));
				}
			} else if(counter > 60000 && counter <= 80000) {
				if(mpu9250_handle->gyro_fsr != INV_FSR_2000DPS) {
					ESP_ERROR_CHECK(mpu9250_gyro_set_fsr(mpu9250_handle, INV_FSR_2000DPS));
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

void my_mpu9250_task(void *arg) {
	// MPU9250 Handle
	mpu9250_init_t mpu9250;
	mpu9250_handle_t mpu9250_handle = &mpu9250;

	// Init MPU9250
	ESP_ERROR_CHECK(mpu9250_init(mpu9250_handle));

//	// Gyro
//	my_mpu9250_gyro_static_calibration(mpu9250_handle);
//	my_mpu9250_gyro_read_data_cycle(mpu9250_handle);

	// Accelerometer
	my_mpu9250_acc_static_calibration(mpu9250_handle);
	my_mpu9250_acc_read_data_cycle(mpu9250_handle);
}
