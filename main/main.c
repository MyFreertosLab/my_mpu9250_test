#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "my_mpu9250_task.h"

void app_main(void)
{
    xTaskCreate(my_mpu9250_task, "my_mpu9250_task", 4096, NULL, 5, NULL);
}

