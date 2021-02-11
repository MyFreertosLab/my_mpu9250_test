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

#define CIRCULAR_BUFFER_SIZE 30

void my_mpu9250_task_init(mpu9250_handle_t mpu9250) {
}
void my_mpu9250_acc_static_calibration(mpu9250_handle_t mpu9250_handle) {
	// calibration offset and biases
	ESP_ERROR_CHECK(mpu9250_acc_calibrate(mpu9250_handle));

}


typedef struct {
	int16_t data[CIRCULAR_BUFFER_SIZE];
	uint8_t cursor;
} my_mpu9250_circular_buffer_t;
typedef my_mpu9250_circular_buffer_t* my_mpu9250_circular_buffer_p_t;
void my_mpu9250_cb_init(my_mpu9250_circular_buffer_p_t cb) {
	cb->cursor = -1;
	memset(cb, 0, sizeof(my_mpu9250_circular_buffer_t));
}

void my_mpu9250_cb_add(my_mpu9250_circular_buffer_p_t cb, int16_t val) {
	cb->cursor++;
	cb->cursor %= CIRCULAR_BUFFER_SIZE;
	cb->data[cb->cursor] = val;
}
void my_mpu9250_cb_means(my_mpu9250_circular_buffer_p_t cb, int16_t* mean) {
	int64_t sum = 0;
	for(uint8_t i = 0; i < CIRCULAR_BUFFER_SIZE; i++) {
		sum += cb->data[i];
	}
	*mean = sum/CIRCULAR_BUFFER_SIZE;
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

	// define and init kalman filter
	mpu9250_kalman_t acc_kalman[3];
	for(uint8_t i = X_POS; i <= Z_POS; i++) {
		acc_kalman[i].X = mpu9250_handle->acc_lsb;
		acc_kalman[i].sample=0;
		acc_kalman[i].P=1.0f;
		acc_kalman[i].Q=1.5;
		acc_kalman[i].K=0.0f;
		acc_kalman[i].R=mpu9250_handle->acc_var[mpu9250_handle->acc_fsr].array[i];
	}

	// prepare circular buffer
	my_mpu9250_circular_buffer_t cbt[3];
	my_mpu9250_circular_buffer_p_t cb[3];
	for(uint8_t i = X_POS; i <= Z_POS; i++) {
		cb[i] = &(cbt[i]);
		my_mpu9250_cb_init(cb[i]);
	}
	for(uint8_t i = 0; i < CIRCULAR_BUFFER_SIZE; i++) {
		if( ulTaskNotifyTake( pdTRUE,xMaxBlockTime ) == 1) {
			ESP_ERROR_CHECK(mpu9250_load_raw_data(mpu9250_handle));
			my_mpu9250_cb_add(cb[X_POS], mpu9250_handle->raw_data.data_s_xyz.accel_data_x);
			my_mpu9250_cb_add(cb[Y_POS], mpu9250_handle->raw_data.data_s_xyz.accel_data_y);
			my_mpu9250_cb_add(cb[Z_POS], mpu9250_handle->raw_data.data_s_xyz.accel_data_z);
		}
	}

	// read cycle
	int32_t counter = 0;
	while (true) {
		counter++;
		if( ulTaskNotifyTake( pdTRUE,xMaxBlockTime ) == 1) {
			ESP_ERROR_CHECK(mpu9250_load_raw_data(mpu9250_handle));
			for(uint8_t i = X_POS; i <= Z_POS; i++) {
				my_mpu9250_cb_add(cb[i], mpu9250_handle->raw_data.data_s_vector.accel[i]);
				my_mpu9250_cb_means(cb[i], &acc_kalman[i].sample);
				acc_kalman[i].P=acc_kalman[i].P+acc_kalman[i].Q;
				acc_kalman[i].K = acc_kalman[i].P/(acc_kalman[i].P+acc_kalman[i].R);
				acc_kalman[i].X = acc_kalman[i].X + acc_kalman[i].K*(acc_kalman[i].sample - acc_kalman[i].X);
				acc_kalman[i].P=(1-acc_kalman[i].K)*acc_kalman[i].P;
			}

			/*
			 *     X(k)=X(k-1)
			 *     P(k)=P(k-1)+Q
			 *     K(k)=P(k)/(P(k)+R)
			 *     X(k)=X(k)+K(k)*(Sample(k)-X(k))
			 *     P(k)=(1-K(k))*P(k)
			 */

			if(counter%100 == 0) {
				int16_t mg[3] = {0,0,0};
				for(uint8_t i = X_POS; i <= Z_POS; i++) {
					mg[i] = acc_kalman[i].X*1000/mpu9250_handle->acc_lsb;
					printf("%d: (X[%d],K[%3.8f]),(lsb[%d],xgc[%d])\n",
							i, acc_kalman[i].X, acc_kalman[i].K, acc_kalman[i].sample,mg[i]);
				}

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
