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
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <my_mpu9250_task.h>
#include <mpu9250_accel.h>
#include <mpu9250_gyro.h>

void my_mpu9250_task_init(mpu9250_handle_t mpu9250) {
}
void my_mpu9250_acc_static_calibration(mpu9250_handle_t mpu9250_handle) {
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 500 );

	// calibration offset and biases
	ESP_ERROR_CHECK(mpu9250_acc_calibrate(mpu9250_handle));

	for(uint8_t i = 0; i < CIRCULAR_BUFFER_SIZE; i++) {
		if( ulTaskNotifyTake( pdTRUE,xMaxBlockTime ) == 1) {
			ESP_ERROR_CHECK(mpu9250_load_raw_data(mpu9250_handle));
		}
	}

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

	// read cycle
	int32_t counter = 0;
	while (true) {
		counter++;
		if( ulTaskNotifyTake( pdTRUE,xMaxBlockTime ) == 1) {
			ESP_ERROR_CHECK(mpu9250_load_data(mpu9250_handle));

			if(counter%100 == 0) {
				int16_t mg[3] = {0,0,0};
				float angles[3] = {0.0f,0.0f,0.0f};
				for(uint8_t i = X_POS; i <= Z_POS; i++) {
					mg[i] = mpu9250_handle->accel.kalman[i].X*1000/mpu9250_handle->accel.lsb;
				}
				int32_t module = mg[X_POS]*mg[X_POS]+mg[Y_POS]*mg[Y_POS]+mg[Z_POS]*mg[Z_POS];
				for(uint8_t i = X_POS; i <= Z_POS; i++) {
					angles[i] = acos(mg[i]/sqrt(module))/6.283185307*360.0f;
					printf("%d: (X[%d],K[%3.8f]),(lsb[%d],xgc[%d]),(a[%3.3f])\n",
							i, mpu9250_handle->accel.kalman[i].X, mpu9250_handle->accel.kalman[i].K, mpu9250_handle->accel.kalman[i].sample,mg[i], angles[i]);
				}

			}

// TODO: Verificare se è necessario applicare un filtro kalman diverso per ogni FSR
//			if(counter <= 20000) {
//				if(mpu9250_handle->accel.fsr != INV_FSR_4G) {
//					ESP_ERROR_CHECK(mpu9250_acc_set_fsr(mpu9250_handle, INV_FSR_4G));
//				}
//			} else if(counter > 20000 && counter <= 40000) {
//				if(mpu9250_handle->accel.fsr != INV_FSR_8G) {
//					ESP_ERROR_CHECK(mpu9250_acc_set_fsr(mpu9250_handle, INV_FSR_8G));
//				}
//			} else if(counter > 40000 && counter <= 60000) {
//				if(mpu9250_handle->accel.fsr != INV_FSR_2G) {
//					ESP_ERROR_CHECK(mpu9250_acc_set_fsr(mpu9250_handle, INV_FSR_2G));
//				}
//			} else if(counter > 60000 && counter <= 80000) {
//				if(mpu9250_handle->accel.fsr != INV_FSR_16G) {
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
				int16_t mxrs = (mpu9250_handle->raw_data.data_s_xyz.gyro_data_x*1000/mpu9250_handle->gyro.lsb);
				int16_t myrs = (mpu9250_handle->raw_data.data_s_xyz.gyro_data_y*1000/mpu9250_handle->gyro.lsb);
				int16_t mzrs = (mpu9250_handle->raw_data.data_s_xyz.gyro_data_z*1000/mpu9250_handle->gyro.lsb);

				printf("Gyro_X_H/L/V [%d][%d]\n", mxrs, mpu9250_handle->raw_data.data_s_xyz.gyro_data_x);
				printf("Gyro_Y_H/L/V [%d][%d]\n", myrs, mpu9250_handle->raw_data.data_s_xyz.gyro_data_y);
				printf("Gyro_Z_H/L/V [%d][%d]\n", mzrs, mpu9250_handle->raw_data.data_s_xyz.gyro_data_z);
			}

			if(counter <= 20000) {
				if(mpu9250_handle->gyro.fsr != INV_FSR_250DPS) {
					ESP_ERROR_CHECK(mpu9250_gyro_set_fsr(mpu9250_handle, INV_FSR_250DPS));
				}
			} else if(counter > 20000 && counter <= 40000) {
				if(mpu9250_handle->gyro.fsr != INV_FSR_500DPS) {
					ESP_ERROR_CHECK(mpu9250_gyro_set_fsr(mpu9250_handle, INV_FSR_500DPS));
				}
			} else if(counter > 40000 && counter <= 60000) {
				if(mpu9250_handle->gyro.fsr != INV_FSR_1000DPS) {
					ESP_ERROR_CHECK(mpu9250_gyro_set_fsr(mpu9250_handle, INV_FSR_1000DPS));
				}
			} else if(counter > 60000 && counter <= 80000) {
				if(mpu9250_handle->gyro.fsr != INV_FSR_2000DPS) {
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
