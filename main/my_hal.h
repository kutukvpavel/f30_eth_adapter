#pragma once

#include <inttypes.h>
#include <stdlib.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <esp_netif.h>

#define FIRMWARE_VERSION_STR "f30_eth_adapter-v0.1"

#ifdef __cplusplus

namespace my_hal
{
    enum sr_types
    {
        SR_INPUT
    };
    enum hardware_rev_types
    {
        pcbV1
    };

    esp_err_t init(void (*read_interrupt_handler)(void* arg));

    esp_netif_t* get_netif();
    bool get_btn_pressed();

    void sr_read(sr_types t, uint8_t* contents);

    void set_sr_reading_in_progress(bool b);
    void set_trigger(bool b);
}

#endif