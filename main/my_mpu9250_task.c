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
#include <mpu9250.h>

#ifdef CONFIG_IDF_TARGET_ESP32
#define MY_SPI_MPU9250_HOST HSPI_HOST
#define PIN_NUM_MISO GPIO_NUM_12
#define PIN_NUM_MOSI GPIO_NUM_13
#define PIN_NUM_CLK  GPIO_NUM_14
#define PIN_NUM_CS   GPIO_NUM_15
#define PIN_NUM_INT  GPIO_NUM_2

#elif defined CONFIG_IDF_TARGET_ESP32S2
#define MY_SPI_MPU9250_HOST SPI2_HOST
#define PIN_NUM_MISO GPIO_NUM_37
#define PIN_NUM_MOSI GPIO_NUM_35
#define PIN_NUM_CLK  GPIO_NUM_36
#define PIN_NUM_CS   GPIO_NUM_34
#define PIN_NUM_INT  GPIO_NUM_2
#endif


void my_mpu9250_task_init(mpu9250_handle_t mpu9250) {
    memset(mpu9250, 0, sizeof(mpu9250_init_t));
    memset(&mpu9250->buscfg, 0, sizeof(spi_bus_config_t));
    memset(&mpu9250->devcfg, 0, sizeof(spi_device_interface_config_t));

	mpu9250->buscfg.miso_io_num = PIN_NUM_MISO;
	mpu9250->buscfg.mosi_io_num = PIN_NUM_MOSI;
	mpu9250->buscfg.sclk_io_num = PIN_NUM_CLK;
	mpu9250->buscfg.quadwp_io_num = -1;
	mpu9250->buscfg.quadhd_io_num = -1;
	mpu9250->buscfg.max_transfer_sz = 256;

	mpu9250->devcfg.spics_io_num = PIN_NUM_CS;
	mpu9250->devcfg.clock_speed_hz = SPI_MASTER_FREQ_20M; //Clock out at 20 MHz
	mpu9250->devcfg.address_bits = 8;
	mpu9250->devcfg.mode = 3; //SPI mode 3
	mpu9250->devcfg.queue_size = 7;  //We want to be able to queue 7 transactions at a time

	mpu9250->int_pin=PIN_NUM_INT;

	printf("SIZEOF MPU9250 [%d]\n", sizeof(*mpu9250));
	printf("SIZEOF BUSCFG [%d]\n", sizeof(mpu9250->buscfg));
	printf("SIZEOF INTR FLAGS [%d]\n", mpu9250->buscfg.intr_flags);
	printf("SIZEOF DEVCFG [%d]\n", sizeof(mpu9250->devcfg));
	printf("MISO: [%d]\n", mpu9250->buscfg.miso_io_num);
	printf("MOSI: [%d]\n", mpu9250->buscfg.mosi_io_num);
	printf("CS: [%d]\n", mpu9250->devcfg.spics_io_num);
	printf("FREQ: [%d]\n", mpu9250->devcfg.clock_speed_hz);
	printf("FLAGS: [%d]\n", mpu9250->devcfg.flags);
	printf("PRETRANS: [%d]\n", mpu9250->devcfg.cs_ena_pretrans);
	printf("POSTTRANS: [%d]\n", mpu9250->devcfg.cs_ena_posttrans);

	//Initialize the SPI bus
	ESP_ERROR_CHECK(spi_bus_initialize(MY_SPI_MPU9250_HOST, &mpu9250->buscfg, 0));
	//Attach the MPU9250 to the SPI bus
	printf("SIZEOF DEVCFG: [%d] addr: [%d]\n", sizeof(mpu9250->devcfg), (uint32_t)&(mpu9250->devcfg));
	ESP_ERROR_CHECK(spi_bus_add_device(MY_SPI_MPU9250_HOST, &mpu9250->devcfg, &(mpu9250->device_handle)));

	printf("INIT MPU9250\n");
	mpu9250_init(mpu9250);
	printf("INITIALIZED MPU9250\n");

}

void my_mpu9250_task(void *arg) {
	// MPU9250 Handle
	mpu9250_init_t mpu9250;
	mpu9250_handle_t mpu9250_handle = &mpu9250;

	// Init MPU9250
	my_mpu9250_task_init(mpu9250_handle);
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 500 );

	ESP_ERROR_CHECK(mpu9250_calc_acc_offset(mpu9250_handle));
	ESP_ERROR_CHECK(mpu9250_calc_acc_biases(mpu9250_handle));

	mpu9250_handle->acc_fsr=INV_FSR_4G;
	ESP_ERROR_CHECK(mpu9250_save_acc_fsr(mpu9250_handle));

	uint32_t counter = 0;
	while (true) {
		counter++;
		if( ulTaskNotifyTake( pdTRUE,xMaxBlockTime ) == 1) {
			ESP_ERROR_CHECK(mpu9250_load_raw_data(mpu9250_handle));
			if(counter%100 == 0) {
				// TODO: fare funzione conversione RAW -> G
				printf("Acc_X_H/L/V [%d][%d]\n", mpu9250_handle->raw_data.data_s_xyz.accel_data_x*100/mpu9250_handle->acc_lsb, mpu9250_handle->raw_data.data_s_xyz.accel_data_x);
				printf("Acc_Y_H/L/V [%d][%d]\n", mpu9250_handle->raw_data.data_s_xyz.accel_data_y*100/mpu9250_handle->acc_lsb, mpu9250_handle->raw_data.data_s_xyz.accel_data_y);
				printf("Acc_Z_H/L/V [%d][%d]\n", mpu9250_handle->raw_data.data_s_xyz.accel_data_z*100/mpu9250_handle->acc_lsb, mpu9250_handle->raw_data.data_s_xyz.accel_data_z);
			}
	    } else {
	    	ESP_ERROR_CHECK(mpu9250_test_connection(mpu9250_handle));
			if(counter%100 == 0) {
		    	printf("SORRY!! Interrupt LOST!\n");
			}
	    }
	}


}
