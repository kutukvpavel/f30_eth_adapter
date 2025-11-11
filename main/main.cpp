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
static volatile TickType_t last_conversion_completed = configINITIAL_TICK_COUNT;

bool data_read_callback(const f30::reg_file_t* data, float ranged_value)
{
    last_conversion_completed = xTaskGetTickCount();
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
            bool set_autotrigger = my_params::get_autotrigger_locally();
            bool do_initial_trigger = false;
            uint32_t interval = *my_params::get_autotrigger_interval();
            TickType_t last = last_conversion_completed;
            if (modbus::get_remote_enabled())
            {
                if (interval != modbus::get_auto_trigger_interval())
                {
                    my_params::set_autotrigger_interval(modbus::get_auto_trigger_interval());
                    my_params::save();
                }
                set_autotrigger = modbus::get_auto_trigger_enabled();
                do_initial_trigger = !autotrigger && (set_autotrigger || modbus::get_single_shot_requested());
            }
            autotrigger = set_autotrigger;
            if (do_initial_trigger || (((xTaskGetTickCount() - last) > (interval * 2)) && set_autotrigger))
                f30::trigger();
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

        vTaskDelay(pdMS_TO_TICKS(50));
    }    
}
_END_STD_C