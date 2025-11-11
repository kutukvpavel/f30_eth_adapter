/**
 * @file main.cpp
 * @author Paul Kutukov
 * @brief F30 multimeter ethernet interface adapter
 * @version 0.1
 * @date 2025-11-11
 *  
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "params.h"
#include "dbg_console.h"
#include "my_hal.h"
#include "modbus.h"
#include "f30.h"

#define BUTTON_DEBOUNCE_DELAY 10 //x[main loop delay]

static const char *TAG = "main";
static volatile bool autotrigger = false;

bool data_read_callback(const f30::reg_file_t* data)
{
    uint32_t raw_value = data->DEC1;
    raw_value += data->DEC2 * 10u;
    raw_value += data->DEC3 * 100u;
    raw_value += data->DEC4 * 1000u;
    raw_value += data->DEC5 * 10000u;
    float ranged_value = static_cast<float>(raw_value) * (1.0f / 1E5f);
    switch (data->NPD_UNITS)
    {
    case f30::units_t::NPD_U:
        switch (data->RANGE)
        {
        case f30::range_t::RANGE_10uA_10k_10mV:
            ranged_value *= 0.01f;
            break;
        case f30::range_t::RANGE_100uA_100k_100mV:
            ranged_value *= 0.1f;
            break;
        case f30::range_t::RANGE_1mA_1M_1V:
            break;
        case f30::range_t::RANGE_10mA_10V:
            ranged_value *= 10;
            break;
        case f30::range_t::RANGE_100V:
            ranged_value *= 100;
            break;
        case f30::range_t::RANGE_350V:
            ranged_value *= 1000;
            break;
        default:
            ESP_LOGE(TAG, "Unknown range for U: %0" PRIX8, data->RANGE);
            break;
        }
        break;
    case f30::units_t::NPD_I:
        switch (data->RANGE)
        {
        case f30::range_t::RANGE_1uA_1k:
            ranged_value *= 1.0f / 1E6f;
            break;
        case f30::range_t::RANGE_10uA_10k_10mV:
            ranged_value *= 1.0f / 1E5f;
            break;
        case f30::range_t::RANGE_100uA_100k_100mV:
            ranged_value *= 1.0f / 1E4f;
            break;
        case f30::range_t::RANGE_1mA_1M_1V:
            ranged_value *= 1.0f / 1E3f;
            break;
        case f30::range_t::RANGE_10mA_10V:
            ranged_value *= 1.0f / 1E2f;
            break;
        default:
            ESP_LOGE(TAG, "Unknown range for I: %0" PRIX8, data->RANGE);
            break;
        }
        break;
    case f30::units_t::NPD_R:
        switch (data->RANGE)
        {
        case f30::range_t::RANGE_1uA_1k:
            ranged_value *= 1E3;
            break;
        case f30::range_t::RANGE_10uA_10k_10mV:
            ranged_value *= 1E4;
            break;
        case f30::range_t::RANGE_100uA_100k_100mV:
            ranged_value *= 1E5;
            break;
        case f30::range_t::RANGE_1mA_1M_1V:
            ranged_value *= 1E6;
            break;
        default:
            ESP_LOGE(TAG, "Unknown range for R: %0" PRIX8, data->RANGE);
            break;
        }
        break;
    default:
        ESP_LOGE(TAG, "Unknown unit code: %0" PRIX8, data->NPD_UNITS);
        break;
    }
    if (data->NPD_MINUS) ranged_value = -ranged_value;
    modbus::set_values(ranged_value, data->NPD_UNITS, data->RANGE);
    return autotrigger;
}

_BEGIN_STD_C
void app_main(void)
{
    static esp_err_t ret;
    static QueueHandle_t dbg_queue; //Interop commands from debug console (for example, calibrations)
    static bool init_ok = true;

    vTaskDelay(pdMS_TO_TICKS(1000));

    //Init NVS
    ret = my_params::init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Init failed: params, err: %s", esp_err_to_name(ret));
        init_ok = false;
    }
    //Init HAL
    if (my_hal::init() != ESP_OK)
    {
        ESP_LOGE(TAG, "Init failed: hal");
        init_ok = false;
    }
    //Modbus Slave
    modbus::init(my_hal::get_netif());
    //Init F30 logic
    f30::init(&data_read_callback, my_params::get_autotrigger_interval());
    //Debug console
    dbg_queue = xQueueCreate(4, sizeof(dbg_console::interop_cmd_t));
    dbg_console::init(dbg_queue);

    //Initialization complete
    if (!init_ok)
    {
        ESP_LOGE(TAG, "Init failed. Remote operation is prohibited.");
    }
    else
    {
        modbus::set_init_ok(*my_params::get_autotrigger_interval());
    }

    //Main loop
    static dbg_console::interop_cmd_t dbg_cmd;
    while (1)
    {
        if (init_ok)
        {
            bool set_autotrigger = false;
            bool do_initial_trigger = false;
            if (modbus::get_remote_enabled())
            {
                if (*my_params::get_autotrigger_interval() != modbus::get_auto_trigger_interval())
                {
                    my_params::set_autotrigger_interval(modbus::get_auto_trigger_interval());
                    my_params::save();
                }
                set_autotrigger = modbus::get_auto_trigger_enabled();
                do_initial_trigger = !autotrigger && (set_autotrigger || modbus::get_single_shot_requested());
            }
            autotrigger = set_autotrigger;
            if (do_initial_trigger) f30::trigger();
        }

        if (xQueueReceive(dbg_queue, &dbg_cmd, 0) == pdTRUE)
        {
            ESP_LOGI(TAG, "Processing debug interop command #%u...", dbg_cmd.cmd);
            switch (dbg_cmd.cmd) // Blocks
            {
            default:
                ESP_LOGW(TAG, "Unknown debug interop command: %i", dbg_cmd.cmd);
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(30));
    }    
}
_END_STD_C