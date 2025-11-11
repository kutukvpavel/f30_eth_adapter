#include "f30.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>

#include "my_hal.h"

namespace f30
{
    static const char TAG[] = "F30";
    static TaskHandle_t read_task_handle = NULL;
    static reg_file_t register_file = {};
    static bool initialized = false;
    static bool (*callback)(const reg_file_t* data, float ranged_value) = NULL;
    static const volatile uint32_t* auto_trigger_interval = NULL;

    void IRAM_ATTR read_interrupt_handler()
    {
        static BaseType_t task_woken;

        if (!initialized) return;
        my_hal::set_sr_reading_in_progress(true); //Will be unset by my_hal::sr_read
        vTaskNotifyGiveFromISR(read_task_handle, &task_woken);
        portYIELD_FROM_ISR(task_woken);
    }

    void read_task(void* arg)
    {
        static TickType_t last_woken = configINITIAL_TICK_COUNT;
        while (1)
        {
            while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) == 0);
            //Read the data once ready
            my_hal::sr_read(my_hal::sr_types::SR_INPUT, reinterpret_cast<uint8_t*>(&register_file));
            //Parse the data
            uint32_t raw_value = register_file.DEC1;
            raw_value += register_file.DEC2 * 10u;
            raw_value += register_file.DEC3 * 100u;
            raw_value += register_file.DEC4 * 1000u;
            raw_value += register_file.DEC5 * 10000u;
            float ranged_value = static_cast<float>(raw_value) * (1.0f / 1E5f);
            switch (register_file.NPD_UNITS)
            {
            case f30::units_t::NPD_U:
                switch (register_file.RANGE)
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
                    ESP_LOGE(TAG, "Unknown range for U: %0" PRIX8, register_file.RANGE);
                    break;
                }
                break;
            case f30::units_t::NPD_I:
                switch (register_file.RANGE)
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
                    ESP_LOGE(TAG, "Unknown range for I: %0" PRIX8, register_file.RANGE);
                    break;
                }
                break;
            case f30::units_t::NPD_R:
                switch (register_file.RANGE)
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
                    ESP_LOGE(TAG, "Unknown range for R: %0" PRIX8, register_file.RANGE);
                    break;
                }
                break;
            default:
                ESP_LOGE(TAG, "Unknown unit code: %0" PRIX8, register_file.NPD_UNITS);
                break;
            }
            if (register_file.NPD_MINUS) ranged_value = -ranged_value;
            //Execute callback
            bool do_trigger = false;
            if (callback) do_trigger = callback(&register_file, ranged_value);
            //Trigger next measurement
            if (!do_trigger) continue;
            vTaskDelayUntil(&last_woken, pdMS_TO_TICKS(*auto_trigger_interval));
            trigger();
        }
    }

    void trigger()
    {
        my_hal::set_trigger(true);
        vTaskDelay(pdMS_TO_TICKS(8));
        my_hal::set_trigger(false);
    }
    void set_interval(uint32_t i)
    {

    }
    void init(bool (*data_read_callback)(const reg_file_t* data, float ranged_value), const volatile uint32_t* interval_ptr)
    {
        xTaskCreate(read_task, "f30_read", 4096, NULL, 1, &read_task_handle);
        assert(read_task_handle);
        callback = data_read_callback;
        auto_trigger_interval = interval_ptr;
        assert(auto_trigger_interval);
        initialized = true;
    }
} // namespace f30
