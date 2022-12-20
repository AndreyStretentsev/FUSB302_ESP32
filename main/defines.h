#pragma once

#include "freertos/FreeRTOS.h"


#define DELAY(__attr__)         vTaskDelay(__attr__ / portTICK_PERIOD_MS);

#define I2C_SCL     22
#define I2C_SDA     21
#define FUSB_INT    25
#define FUSB_ADDR   0x23
