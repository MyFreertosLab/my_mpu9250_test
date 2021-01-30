/*
 * my_mp9250_task.c
 *
 *  Created on: 29 gen 2021
 *      Author: andrea
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "my_mpu9250_task.h"

#ifdef CONFIG_IDF_TARGET_ESP32
#define MY_SPI_MPU9250_HOST HSPI_HOST
#define PIN_NUM_MISO GPIO_NUM_12
#define PIN_NUM_MOSI GPIO_NUM_13
#define PIN_NUM_CLK  GPIO_NUM_14
#define PIN_NUM_CS   GPIO_NUM_15
#define PIN_NUM_INT  GPIO_NUM_2

#elif defined CONFIG_IDF_TARGET_ESP32S2
#define MY_SPI_MPU9250_HOST SPI2_HOST
#define PIN_NUM_MISO 37
#define PIN_NUM_MOSI 35
#define PIN_NUM_CLK  36
#define PIN_NUM_CS   34
#define PIN_NUM_INT  GPIO_NUM_2
#endif

// MPU6500 registers
#define MPU_6500_SPI_WHOAMI_REGISTER              117
#define MPU_6500_SPI_ACC_OFFSET_REGISTER          119
#define MPU_6500_SPI_GYRO_OFFSET_REGISTER_X_H      19
#define MPU_6500_SPI_GYRO_OFFSET_REGISTER_X_L      20
#define MPU_6500_SPI_GYRO_OFFSET_REGISTER_Y_H      21
#define MPU_6500_SPI_GYRO_OFFSET_REGISTER_Y_L      22
#define MPU_6500_SPI_GYRO_OFFSET_REGISTER_Z_H      23
#define MPU_6500_SPI_GYRO_OFFSET_REGISTER_Z_L      24
#define MPU_6500_SPI_ACC_OFFSET_REGISTER_X_H      119
#define MPU_6500_SPI_ACC_OFFSET_REGISTER_X_L      120
#define MPU_6500_SPI_ACC_OFFSET_REGISTER_Y_H      122
#define MPU_6500_SPI_ACC_OFFSET_REGISTER_Y_L      123
#define MPU_6500_SPI_ACC_OFFSET_REGISTER_Z_H      125
#define MPU_6500_SPI_ACC_OFFSET_REGISTER_Z_L      126
#define MPU_6500_SPI_ACCEL_RAW_REGISTER            59
#define MPU_6500_SPI_TEMP_RAW_REGISTER             65
#define MPU_6500_SPI_GYRO_RAW_REGISTER             67
#define MPU_6500_SPI_CONF_REGISTER                 26
#define MPU_6500_SPI_GYRO_CONF_REGISTER            27
#define MPU_6500_SPI_ACC_CONF1_REGISTER            28
#define MPU_6500_SPI_ACC_CONF2_REGISTER            29
#define MPU_6500_SPI_PWR_MGMT1_REGISTER           107
#define MPU_6500_SPI_PWR_MGMT2_REGISTER           108
#define MPU_6500_SPI_INT_ENABLE_REGISTER           56
#define MPU_6500_SPI_INT_STATUS_REGISTER           58

#define READ_FLAG 0x80

void my_mpu9250_task_init(spi_device_handle_t *spi) {
	spi_bus_config_t buscfg = {
			.miso_io_num = PIN_NUM_MISO,
			.mosi_io_num = PIN_NUM_MOSI,
			.sclk_io_num = PIN_NUM_CLK,
			.quadwp_io_num = -1,
			.quadhd_io_num = -1,
			.max_transfer_sz = 256
	};
	spi_device_interface_config_t devcfg = {
			.spics_io_num = PIN_NUM_CS,
			.clock_speed_hz = SPI_MASTER_FREQ_20M, //Clock out at 20 MHz
			.address_bits = 8,
			.mode = 3, //SPI mode 3
			.queue_size = 7  //We want to be able to queue 7 transactions at a time
			};

	//Initialize the SPI bus
	ESP_ERROR_CHECK(spi_bus_initialize(MY_SPI_MPU9250_HOST, &buscfg, 0));

	//Attach the MPU9250 to the SPI bus
	ESP_ERROR_CHECK(spi_bus_add_device(MY_SPI_MPU9250_HOST, &devcfg, spi));
}

esp_err_t my_mpu9250_task_keep_alive(spi_device_handle_t spi) {
	esp_err_t ret;
	spi_transaction_t t;
	printf("Keep Alive ...\n");
	memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.length = 8;                    //Transaction length is in bits.
	t.addr = (READ_FLAG | MPU_6500_SPI_WHOAMI_REGISTER);
	t.flags = SPI_TRANS_USE_RXDATA;
	ret = spi_device_polling_transmit(spi, &t);  //Transmit!
	if (ret == ESP_OK) {
		printf("MPU9250 ID: [%d][bits: %d]\n", (t.rx_data[0]), t.rxlength);
	} else {
		printf("ERROR: [%d]\n", ret);
	}
	return ret;
}

void my_mpu9250_task(void *arg) {
	spi_device_handle_t spi;
	my_mpu9250_task_init(&spi);

	while (true) {
		my_mpu9250_task_keep_alive(spi);
		vTaskDelay(20);
	}
}
