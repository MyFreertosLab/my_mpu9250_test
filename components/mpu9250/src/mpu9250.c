/**
 *  @addtogroup  DRIVERS Sensor Driver Layer
 *  @brief       Hardware drivers for mp9250
 *
 *  @{
 *      @file       mpu9250.c
 *      @brief      An SPI-based driver for mpu9250
 *      @details    This driver currently works for the following devices:
 *                  MPU6500
 *                  MPU9250 (or MPU6500 w/ AK8963 on the auxiliary bus)
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <mpu9250.h>
#include <driver/gpio.h>

// TODO: definire in Kdconfig
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



/************************************************************************
 ************** P R I V A T E  I M P L E M E N T A T I O N **************
 ************************************************************************/

//This function is called with every interrupt
static void IRAM_ATTR mpu9250_isr(void* mpu9250_handle)
{
	static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(((mpu9250_handle_t)mpu9250_handle)->data_ready_task_handle, &xHigherPriorityTaskWoken);
	if( xHigherPriorityTaskWoken) {
		portYIELD_FROM_ISR(); // this wakes up sample_timer_task immediately
	}
}

static esp_err_t mpu9250_read8(mpu9250_handle_t mpu9250_handle, uint8_t reg, uint8_t* val) {
	esp_err_t ret;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.length = 8;                    //Transaction length is in bits.
	t.addr = (MPU9250_READ_FLAG | reg);
	t.flags = SPI_TRANS_USE_RXDATA;
	ret = spi_device_polling_transmit(((mpu9250_handle_t)mpu9250_handle)->device_handle, &t);  //Transmit!
	if (ret == ESP_OK) {
	  	  *val=t.rx_data[0];
	}
	return ret;
}

static esp_err_t mpu9250_write8(mpu9250_handle_t mpu9250_handle, uint8_t reg, uint8_t val) {
	esp_err_t ret;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.length = 8;                    //Transaction length is in bits.
	t.addr = reg;
	t.tx_data[0] = val;
	t.flags = SPI_TRANS_USE_TXDATA;
	ret = spi_device_polling_transmit(((mpu9250_handle_t)mpu9250_handle)->device_handle, &t);  //Transmit!
	return ret;
}

static esp_err_t mpu9250_read_buff(mpu9250_handle_t mpu9250_handle, uint8_t reg, void* buff, uint8_t length) {
	esp_err_t ret;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.length = length;                    //Transaction length is in bits.
	t.addr = (MPU9250_READ_FLAG | reg);
	t.rx_buffer=buff;
	ret = spi_device_polling_transmit(((mpu9250_handle_t)mpu9250_handle)->device_handle, &t);  //Transmit!
	return ret;
}

static esp_err_t mpu9250_write_buff(mpu9250_handle_t mpu9250_handle, uint8_t reg, void* buff, uint8_t length) {
	esp_err_t ret;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.length = length;              //Transaction length is in bits.
	t.addr = reg;
	t.tx_buffer=buff;
	ret = spi_device_polling_transmit(((mpu9250_handle_t)mpu9250_handle)->device_handle, &t);  //Transmit!
	return ret;
}

static esp_err_t mpu9250_spi_init(mpu9250_handle_t mpu9250_handle) {
	printf("MPU9250: Initializing SPI ... \n");
    memset(mpu9250_handle, 0, sizeof(mpu9250_init_t));

    mpu9250_handle->buscfg.miso_io_num = PIN_NUM_MISO;
    mpu9250_handle->buscfg.mosi_io_num = PIN_NUM_MOSI;
    mpu9250_handle->buscfg.sclk_io_num = PIN_NUM_CLK;
    mpu9250_handle->buscfg.quadwp_io_num = -1;
    mpu9250_handle->buscfg.quadhd_io_num = -1;
    mpu9250_handle->buscfg.max_transfer_sz = 256;

    mpu9250_handle->devcfg.spics_io_num = PIN_NUM_CS;
    mpu9250_handle->devcfg.clock_speed_hz = SPI_MASTER_FREQ_20M; //Clock out at 20 MHz
    mpu9250_handle->devcfg.address_bits = 8;
    mpu9250_handle->devcfg.mode = 3; //SPI mode 3
    mpu9250_handle->devcfg.queue_size = 7;  //We want to be able to queue 7 transactions at a time

    mpu9250_handle->int_pin=PIN_NUM_INT;

	printf("MPU9250: miso [%d]\n", mpu9250_handle->buscfg.miso_io_num);
	printf("MPU9250: mosi [%d]\n", mpu9250_handle->buscfg.mosi_io_num);
	printf("MPU9250: cs [%d]\n", mpu9250_handle->devcfg.spics_io_num);
	printf("MPU9250: freq [%d]\n", mpu9250_handle->devcfg.clock_speed_hz);
	printf("MPU9250: flags [%d]\n", mpu9250_handle->devcfg.flags);

	//Initialize the SPI bus
	ESP_ERROR_CHECK(spi_bus_initialize(MY_SPI_MPU9250_HOST, &mpu9250_handle->buscfg, 0));
	//Attach the MPU9250 to the SPI bus
	ESP_ERROR_CHECK(spi_bus_add_device(MY_SPI_MPU9250_HOST, &mpu9250_handle->devcfg, &(mpu9250_handle->device_handle)));
	printf("MPU9250: SPI initialized\n");
	return ESP_OK;
}
static esp_err_t mpu9250_save_acc_fsr(mpu9250_handle_t mpu9250_handle) {
	uint8_t acc_conf = 0;
	uint8_t acc_conf_req = mpu9250_handle->acc_fsr;
    ESP_ERROR_CHECK(mpu9250_read8(mpu9250_handle, MPU9250_ACCEL_CONFIG, &acc_conf));
    uint8_t toWrite = (acc_conf & MPU9250_ACC_FSR_MASK) | ((mpu9250_handle->acc_fsr << MPU9250_ACC_FSR_LBIT)& (~MPU9250_ACC_FSR_MASK));
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_ACCEL_CONFIG, toWrite));
    ESP_ERROR_CHECK(mpu9250_load_acc_fsr(mpu9250_handle));

	switch(mpu9250_handle->acc_fsr) {
	case INV_FSR_2G: {
		printf("MPU9250: AccFSR 2g\n");
		break;
	}
	case INV_FSR_4G: {
		printf("MPU9250: AccFSR 4g\n");
		break;
	}
	case INV_FSR_8G: {
		printf("MPU9250: AccFSR 8g\n");
		break;
	}
	case INV_FSR_16G: {
		printf("MPU9250: AccFSR 16g\n");
		break;
	}
	default: {
		printf("MPU9250: AccFSR UNKNOWN [%d]\n", mpu9250_handle->acc_fsr);
		break;
	}
	}
    return (mpu9250_handle->acc_fsr == acc_conf_req ? ESP_OK : ESP_FAIL);
}

static esp_err_t mpu9250_calc_acc_means(mpu9250_handle_t mpu9250_handle, int32_t* acc_means, uint8_t cycles) {
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 500 );
	printf("Calculating Acc Means with %d000 samples (wait for %d seconds)... \n", cycles, cycles);
	acc_means[0] = 0;
	acc_means[1] = 0;
	acc_means[2] = 0;

	for(int j = 0; j < cycles; j++) {
		uint16_t max_samples = 1000;
		int32_t acc_sum[3] = {0,0,0};
		for(int i = 0; i < max_samples; i++) {
			ulTaskNotifyTake( pdTRUE,xMaxBlockTime );
			ESP_ERROR_CHECK(mpu9250_load_raw_data(mpu9250_handle));
			acc_sum[0] += mpu9250_handle->raw_data.data_s_xyz.accel_data_x;
			acc_sum[1] += mpu9250_handle->raw_data.data_s_xyz.accel_data_y;
			acc_sum[2] += mpu9250_handle->raw_data.data_s_xyz.accel_data_z;
		}

		// offsets respect vertical attitude
		acc_means[0] += acc_sum[0]/max_samples;
		acc_means[1] += acc_sum[1]/max_samples;
		acc_means[2] += acc_sum[2]/max_samples;
	}
	acc_means[0] = acc_means[0]/cycles;
	acc_means[1] = acc_means[1]/cycles;
	acc_means[2] = acc_means[2]/cycles;
	return ESP_OK;
}

static esp_err_t mpu9250_calc_acc_sqm(mpu9250_handle_t mpu9250_handle, int32_t* acc_means, float* acc_sqm, uint8_t cycles) {
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 500 );

	printf("Calculating Acc sqm with %d000 samples (wait for %d seconds)... \n", cycles, cycles);
	acc_sqm[0] = 0.0f;
	acc_sqm[1] = 0.0f;
	acc_sqm[2] = 0.0f;

	int32_t acc_sum[3] = {0,0,0};
	int16_t max_cycles = cycles*1000;
	for(int j = 0; j < max_cycles; j++) {
		ulTaskNotifyTake( pdTRUE,xMaxBlockTime );
		ESP_ERROR_CHECK(mpu9250_load_raw_data(mpu9250_handle));
		acc_sum[0] += (mpu9250_handle->raw_data.data_s_xyz.accel_data_x - acc_means[0])*(mpu9250_handle->raw_data.data_s_xyz.accel_data_x - acc_means[0]);
		acc_sum[1] += (mpu9250_handle->raw_data.data_s_xyz.accel_data_y - acc_means[1])*(mpu9250_handle->raw_data.data_s_xyz.accel_data_y - acc_means[1]);
		acc_sum[2] += (mpu9250_handle->raw_data.data_s_xyz.accel_data_z - acc_means[2])*(mpu9250_handle->raw_data.data_s_xyz.accel_data_z - acc_means[2]);
	}
	acc_sqm[0] = sqrt(acc_means[0]/(max_cycles));
	acc_sqm[1] = sqrt(acc_means[1]/(max_cycles));
	acc_sqm[2] = sqrt(acc_means[2]/(max_cycles));
	return ESP_OK;
}

static esp_err_t mpu9250_calc_acc_cdv(mpu9250_handle_t mpu9250_handle, int32_t* acc_means, float* acc_sqm, float* acc_cdv, uint8_t cycles) {
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 500 );

	printf("Calculating Acc cdv with %d000 samples (wait for %d seconds)... \n", cycles, cycles);
	acc_cdv[0] = acc_sqm[0]/acc_means[0];
	acc_cdv[1] = acc_sqm[1]/acc_means[1];
	acc_cdv[2] = acc_sqm[2]/acc_means[2];

	return ESP_OK;
}

/************************************************************************
 ****************** A P I  I M P L E M E N T A T I O N ******************
 ************************************************************************/

esp_err_t mpu9250_init(mpu9250_handle_t mpu9250_handle) {
	ESP_ERROR_CHECK(mpu9250_spi_init(mpu9250_handle));
	mpu9250_handle->data_ready_task_handle=xTaskGetCurrentTaskHandle();

    // set Configuration Register
	printf("MPU9250: Gyro bandwidth 184Hz\n");
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_CONFIG, 0x01)); // gyro bandwidth 184Hz

    // set Gyro Configuration Register
	printf("MPU9250: Gyro +-250deg/sec\n");
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_GYRO_CONFIG, 0x0)); // +-250deg/sec

    // set Acc Conf1 Register
	mpu9250_handle->acc_fsr=INV_FSR_16G;
	ESP_ERROR_CHECK(mpu9250_save_acc_fsr(mpu9250_handle));

    // set Acc Conf2 Register
	printf("MPU9250: Accel bandwidth 92Hz\n");
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_ACCEL_CONFIG_2, 0x02)); // bandwidth 92Hz

    // set PwrMgmt2 Register
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_PWR_MGMT_2, 0x00));

    // set PwrMgmt1 Register
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_PWR_MGMT_1, 0x01)); // clock, no cycle

    // set Int Status Register
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_INT_STATUS, 0x00)); // reset all interrupts?

    // set Int Enable Register
	printf("MPU9250: Enable Interrupts\n");
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_INT_ENABLE, 0x01)); // data ready int

	// prepare GPIO Interrupt
	printf("MPU9250: Gpio interrupt pin [%d]\n", mpu9250_handle->int_pin);
    gpio_config_t io_conf={
        .intr_type=GPIO_PIN_INTR_POSEDGE,
        .mode=GPIO_MODE_INPUT,
        .pull_down_en=1,
        .pin_bit_mask=(1<<mpu9250_handle->int_pin)
    };
    gpio_config(&io_conf);
	printf("MPU9250: Gpio interrupt pin configured\n");

	printf("MPU9250: Configuring gpio interrupts ....\n");
	ESP_ERROR_CHECK(gpio_set_intr_type(mpu9250_handle->int_pin, GPIO_PIN_INTR_POSEDGE));
	ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
	ESP_ERROR_CHECK(gpio_isr_handler_add(mpu9250_handle->int_pin, mpu9250_isr, (void*)mpu9250_handle));
	printf("MPU9250: Gpio interrupts configured ..\n");

    return ESP_OK;
}

esp_err_t mpu9250_load_int_status(mpu9250_handle_t mpu9250_handle) {
	return mpu9250_read8(mpu9250_handle, MPU9250_INT_STATUS, &((mpu9250_handle_t)mpu9250_handle)->int_status);
}

esp_err_t mpu9250_load_whoami(mpu9250_handle_t mpu9250_handle) {
	esp_err_t ret = ESP_FAIL;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));        //Zero out the transaction
	t.length = 8;                    //Transaction length is in bits.
	t.addr = (MPU9250_READ_FLAG | MPU9250_WHO_AM_I);
	t.flags = SPI_TRANS_USE_RXDATA;
	ret = spi_device_polling_transmit(mpu9250_handle->device_handle, &t);  //Transmit!
	mpu9250_handle->whoami = (ret == ESP_OK ? t.rx_data[0] : 0);
	return ret;
}
esp_err_t mpu9250_load_acc_fsr(mpu9250_handle_t mpu9250_handle) {
	uint8_t acc_conf = 0;
    ESP_ERROR_CHECK(mpu9250_read8(mpu9250_handle, MPU9250_ACCEL_CONFIG, &acc_conf));
    mpu9250_handle->acc_fsr = (acc_conf & (~MPU9250_ACC_FSR_MASK)) >> MPU9250_ACC_FSR_LBIT;
    ESP_ERROR_CHECK(mpu9250_calc_acc_lsb(mpu9250_handle));
	return ESP_OK;
}

esp_err_t mpu9250_set_acc_fsr(mpu9250_handle_t mpu9250_handle, uint8_t fsr) {
	mpu9250_handle->acc_fsr=fsr;
	ESP_ERROR_CHECK(mpu9250_save_acc_fsr(mpu9250_handle));
	return ESP_OK;
}

esp_err_t mpu9250_test_connection(mpu9250_handle_t mpu9250_handle) {
	mpu9250_load_whoami(mpu9250_handle);
	return (mpu9250_handle->whoami == MPU9250_ID ? ESP_OK : ESP_FAIL);

}

esp_err_t mpu9250_load_raw_data(mpu9250_handle_t mpu9250_handle) {
	uint8_t buff[26];
	esp_err_t ret = mpu9250_read_buff(mpu9250_handle, MPU9250_ACCEL_XOUT_H, buff, 26*8);
	mpu9250_handle->raw_data.data_s_xyz.accel_data_x = ((buff[0] << 8) + buff[1]) - mpu9250_handle->acc_bias[mpu9250_handle->acc_fsr].xyz.x;
	mpu9250_handle->raw_data.data_s_xyz.accel_data_y = ((buff[2] << 8) + buff[3]) - mpu9250_handle->acc_bias[mpu9250_handle->acc_fsr].xyz.y;
	mpu9250_handle->raw_data.data_s_xyz.accel_data_z = ((buff[4] << 8) + buff[5]) - mpu9250_handle->acc_bias[mpu9250_handle->acc_fsr].xyz.z;
	return ret;
}
esp_err_t mpu9250_load_acc_offset(mpu9250_handle_t mpu9250_handle) {
	uint8_t buff[8];
	esp_err_t ret = mpu9250_read_buff(mpu9250_handle, MPU9250_XA_OFFSET_H, buff, 8*8);
	mpu9250_handle->acc_offset.xyz.x = (buff[0] << 8) + buff[1];
	mpu9250_handle->acc_offset.xyz.y = (buff[3] << 8) + buff[4];
	mpu9250_handle->acc_offset.xyz.z = (buff[6] << 8) + buff[7];
	return ret;
}
esp_err_t mpu9250_save_acc_offset(mpu9250_handle_t mpu9250_handle) {
	uint8_t buff[8];
	buff[0] = mpu9250_handle->acc_offset.xyz.x >> 8;
	buff[1] = mpu9250_handle->acc_offset.xyz.x & 0x00FF;
	buff[2] = 0;
	buff[3] = mpu9250_handle->acc_offset.xyz.y >> 8;
	buff[4] = mpu9250_handle->acc_offset.xyz.y & 0x00FF;
	buff[5] = 0;
	buff[6] = mpu9250_handle->acc_offset.xyz.z >> 8;
	buff[7] = mpu9250_handle->acc_offset.xyz.z & 0x00FF;

	esp_err_t ret = mpu9250_write_buff(mpu9250_handle, MPU9250_XA_OFFSET_H, buff, 8*8);
	return ret;
}

esp_err_t mpu9250_calc_acc_bias(mpu9250_handle_t mpu9250_handle) {
	// Calc Bias
	{
		// init values
		mpu9250_handle->acc_bias[mpu9250_handle->acc_fsr].xyz.x = 0;
		mpu9250_handle->acc_bias[mpu9250_handle->acc_fsr].xyz.y = 0;
		mpu9250_handle->acc_bias[mpu9250_handle->acc_fsr].xyz.z = 0;

		printf("Calculating Acc Bias ... \n");
		int32_t acc_means[3] = {0,0,0};
		ESP_ERROR_CHECK(mpu9250_calc_acc_means(mpu9250_handle, acc_means, 30));

		// offsets respect vertical attitude
		mpu9250_handle->acc_bias[mpu9250_handle->acc_fsr].xyz.x = acc_means[0];
		mpu9250_handle->acc_bias[mpu9250_handle->acc_fsr].xyz.y = acc_means[1];
		mpu9250_handle->acc_bias[mpu9250_handle->acc_fsr].xyz.z = (acc_means[2] - mpu9250_handle->acc_lsb);

		// TODO: Calcolare Scarto Quadratico Medio rispetto al Bias
		// TODO: Calcolare Coefficiente di variazione rispetto al Bias (sqm/bias)
		// TODO: Assegnare fattore K=(1-cdv) ad una struttura ad hoc di mpu9250_handle
		printf("Acc bias: [%d][%d][%d]\n", mpu9250_handle->acc_bias[mpu9250_handle->acc_fsr].xyz.x, mpu9250_handle->acc_bias[mpu9250_handle->acc_fsr].xyz.y,mpu9250_handle->acc_bias[mpu9250_handle->acc_fsr].xyz.z);
	}
	return ESP_OK;
}

esp_err_t mpu9250_calc_acc_biases(mpu9250_handle_t mpu9250_handle) {
	ESP_ERROR_CHECK(mpu9250_set_acc_fsr(mpu9250_handle, INV_FSR_16G));
	ESP_ERROR_CHECK(mpu9250_discard_messages(mpu9250_handle, 10));
	ESP_ERROR_CHECK(mpu9250_calc_acc_bias(mpu9250_handle));

	ESP_ERROR_CHECK(mpu9250_set_acc_fsr(mpu9250_handle, INV_FSR_8G));
	ESP_ERROR_CHECK(mpu9250_discard_messages(mpu9250_handle, 10));
	ESP_ERROR_CHECK(mpu9250_calc_acc_bias(mpu9250_handle));

	ESP_ERROR_CHECK(mpu9250_set_acc_fsr(mpu9250_handle, INV_FSR_4G));
	ESP_ERROR_CHECK(mpu9250_discard_messages(mpu9250_handle, 10));
	ESP_ERROR_CHECK(mpu9250_calc_acc_bias(mpu9250_handle));

	ESP_ERROR_CHECK(mpu9250_set_acc_fsr(mpu9250_handle, INV_FSR_2G));
	ESP_ERROR_CHECK(mpu9250_discard_messages(mpu9250_handle, 10));
	ESP_ERROR_CHECK(mpu9250_calc_acc_bias(mpu9250_handle));

	return ESP_OK;
}

esp_err_t mpu9250_calc_acc_lsb(mpu9250_handle_t mpu9250_handle) {
	mpu9250_handle->acc_lsb = (32768 >> (mpu9250_handle->acc_fsr + 1));
	return ESP_OK;
}

esp_err_t mpu9250_calc_acc_offset(mpu9250_handle_t mpu9250_handle) {
	if(mpu9250_handle->acc_fsr != INV_FSR_16G) {
		mpu9250_handle->acc_fsr=INV_FSR_16G;
		ESP_ERROR_CHECK(mpu9250_save_acc_fsr(mpu9250_handle));
	}


	// Original Acc offsets: [6684][-5436][9684]
	ESP_ERROR_CHECK(mpu9250_load_acc_offset(mpu9250_handle));
	printf("Acc offsets: [%d][%d][%d]\n", mpu9250_handle->acc_offset.xyz.x, mpu9250_handle->acc_offset.xyz.y,mpu9250_handle->acc_offset.xyz.z);
	mpu9250_acc_offset_t original_offsets = {
			.xyz.x = mpu9250_handle->acc_offset.xyz.x,
			.xyz.y = mpu9250_handle->acc_offset.xyz.y,
			.xyz.z = mpu9250_handle->acc_offset.xyz.z
	};
	mpu9250_handle->acc_bias[mpu9250_handle->acc_fsr].xyz.x = 0;
	mpu9250_handle->acc_bias[mpu9250_handle->acc_fsr].xyz.y = 0;
	mpu9250_handle->acc_bias[mpu9250_handle->acc_fsr].xyz.z = 0;

	// dicard 10000 samples
	ESP_ERROR_CHECK(mpu9250_discard_messages(mpu9250_handle, 10000));

	printf("Calculating Acc Offset ... \n");
	uint16_t max_means = 60;
	int32_t acc_means[3] = {0,0,0};
	ESP_ERROR_CHECK(mpu9250_calc_acc_means(mpu9250_handle, acc_means, max_means));

	mpu9250_handle->acc_offset.xyz.x -= acc_means[0];
	mpu9250_handle->acc_offset.xyz.y -= acc_means[1];
	mpu9250_handle->acc_offset.xyz.z -= (acc_means[2] - mpu9250_handle->acc_lsb);

	printf("Acc offsets: [%d][%d][%d]\n", mpu9250_handle->acc_offset.xyz.x, mpu9250_handle->acc_offset.xyz.y,mpu9250_handle->acc_offset.xyz.z);
	printf("Acc means: [%d][%d][%d]\n", acc_means[0], acc_means[1],acc_means[2]);

	ESP_ERROR_CHECK(mpu9250_save_acc_offset(mpu9250_handle));
	ESP_ERROR_CHECK(mpu9250_discard_messages(mpu9250_handle, 10));
	ESP_ERROR_CHECK(mpu9250_calc_acc_biases(mpu9250_handle));

	// TODO: Invocare mpu9250_calc_acc_sqm
	// TODO: Invocare mpu9250_calc_acc_cdv
	// TODO: Assegnare fattore K=(1-cdv)

	return ESP_OK;
}

esp_err_t mpu9250_discard_messages(mpu9250_handle_t mpu9250_handle, uint16_t num_msgs) {
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 500 );
	printf("Discarding %d Samples ... \n", num_msgs);
	for(uint16_t i = 0; i < num_msgs; i++) {
		ulTaskNotifyTake( pdTRUE,xMaxBlockTime );
	}
	return ESP_OK;
}
