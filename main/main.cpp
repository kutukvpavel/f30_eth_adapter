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
#include "eth_mdns_init.h"
#include "meter_web_server.h"

static const char *TAG = "main";
static volatile bool autotrigger = false;
static volatile TickType_t last_conversion_completed = configINITIAL_TICK_COUNT;

bool data_read_callback(const f30::reg_file_t* data, float ranged_value)
{
    last_conversion_completed = xTaskGetTickCount();
    modbus::set_values(ranged_value, data->NPD_UNITS, data->RANGE);
    meter_web_server::set_data(ranged_value, f30::get_unit_string(data->NPD_UNITS));
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
    if (my_hal::init(f30::read_interrupt_handler) != ESP_OK)
    {
        ESP_LOGE(TAG, "Init failed: hal");
        init_ok = false;
    }
    //Init mDNS
    mdns_start_service(my_params::get_hostname(), FIRMWARE_VERSION_STR);
    mdns_register_modbus(CONFIG_FMB_TCP_PORT_DEFAULT, CONFIG_FMB_CONTROLLER_SLAVE_ID);
    mdns_register_console(CONFIG_CONSOLE_PORT);
    mdns_register_echo(CONFIG_ECHO_PORT);
    //Modbus Slave
    modbus::init(my_hal::get_netif());
    //Web Server
    ESP_ERROR_CHECK_WITHOUT_ABORT(meter_web_server::init());
    //Init F30 logic
    f30::init(&data_read_callback, my_params::get_autotrigger_interval());
    //Debug console
    dbg_queue = xQueueCreate(4, sizeof(dbg_console::interop_cmd_t));
    dbg_console::init(dbg_queue);

    //Initialization complete
    my_hal::set_led_state(my_hal::status_led_states::off, 0);
    if (!init_ok)
    {
        ESP_LOGE(TAG, "Init failed. Remote operation is prohibited.");
        my_hal::set_led_state(my_hal::status_led_states::pulsed_fast, 1500);
    }
    else
    {
        modbus::set_init_ok(*my_params::get_autotrigger_interval());
        my_hal::set_led_state(my_hal::status_led_states::on, 1500);
        my_hal::set_led_state(my_hal::status_led_states::off, 1000);
        vTaskDelay(pdMS_TO_TICKS(1500));
    }

    //Main loop
    static dbg_console::interop_cmd_t dbg_cmd;
    while (1)
    {
        if (init_ok)
        {
            //Business logic
            bool set_autotrigger = my_params::get_autotrigger_locally();
            bool do_initial_trigger = false;
            uint32_t interval = *my_params::get_autotrigger_interval();
            TickType_t last = last_conversion_completed;
            bool remote = modbus::get_remote_enabled();
            bool response_timeout = (xTaskGetTickCount() - last) > (interval * 2);
            if (remote)
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
            if (do_initial_trigger || (response_timeout && set_autotrigger))
                f30::trigger();

            //Status LED
            if (remote)
            {
                if (set_autotrigger && !response_timeout) my_hal::set_led_state(my_hal::status_led_states::pulsed_fast, 0);
                else if (response_timeout) 
                {
                    my_hal::set_led_state(my_hal::status_led_states::off, 1000);
                    last_conversion_completed = xTaskGetTickCount();
                }
                else my_hal::set_led_state(my_hal::status_led_states::on, 0);
            }
            else
            {
                if (set_autotrigger && !response_timeout) my_hal::set_led_state(my_hal::status_led_states::pulsed_slow, 0);
                else my_hal::set_led_state(my_hal::status_led_states::off, 0);
            }
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