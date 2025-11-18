#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "esp_err.h"

esp_err_t esp_eth_console_create(RingbufHandle_t* rx, RingbufHandle_t* tx);