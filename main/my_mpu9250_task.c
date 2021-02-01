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

	//Initialize the SPI bus
	ESP_ERROR_CHECK(spi_bus_initialize(MY_SPI_MPU9250_HOST, &(mpu9250->buscfg), 0));

	//Attach the MPU9250 to the SPI bus
	ESP_ERROR_CHECK(spi_bus_add_device(MY_SPI_MPU9250_HOST, &(mpu9250->devcfg), &(mpu9250->device_handle)));

	printf("INIT MPU9250\n");
	mpu9250_init(mpu9250);
	printf("INITIALIZED MPU9250\n");

}

void my_mpu9250_task(void *arg) {
	mpu9250_init_t mpu9250;
	mpu9250_handle_t mpu9250_handle = &mpu9250;

	my_mpu9250_task_init(mpu9250_handle);

	while (true) {
	    if(xSemaphoreTake(((mpu9250_handle_t)mpu9250_handle)->spi_mutex, portMAX_DELAY) == pdTRUE) {
			mpu9250_get_int_status(mpu9250_handle);
			mpu9250_whoami(mpu9250_handle);
	    }
	}
}
