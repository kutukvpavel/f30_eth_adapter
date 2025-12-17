#include "f30.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>

#include "macros.h"

#include "my_hal.h"

namespace f30
{
    static const char TAG[] = "F30";
    static TaskHandle_t read_task_handle = NULL;
    static reg_file_t register_file = {};
    static volatile bool initialized = false;
    static bool (*callback)(const reg_file_t* data, float ranged_value) = NULL;
    static const volatile uint32_t* auto_trigger_interval = NULL;
    static volatile uint32_t trigger_stats = 0;
    static volatile uint32_t data_read_stats = 0;
    static SemaphoreHandle_t register_file_mutex = NULL;

    void IRAM_ATTR read_interrupt_handler(void* arg)
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
            float ranged_value = static_cast<float>(raw_value);
            switch (register_file.NPD_QUANTITY)
            {
            case f30::quantity_t::NPD_U:
                switch (register_file.RANGE)
                {
                case f30::range_t::RANGE_10uA_10k_10mV:
                    ranged_value *= 0.000001f;
                    break;
                case f30::range_t::RANGE_100uA_100k_100mV:
                    ranged_value *= 0.00001f;
                    break;
                case f30::range_t::RANGE_1mA_1M_1V:
                    ranged_value *= 0.0001;
                    break;
                case f30::range_t::RANGE_10mA_10V:
                    ranged_value *= 0.001;
                    break;
                case f30::range_t::RANGE_100V:
                    ranged_value *= 0.01;
                    break;
                case f30::range_t::RANGE_350V:
                    ranged_value *= 0.1;
                    break;
                default:
                    ESP_LOGE(TAG, "Unknown range for U: %0" PRIX8, register_file.RANGE);
                    break;
                }
                break;
            case f30::quantity_t::NPD_I:
                switch (register_file.RANGE)
                {
                case f30::range_t::RANGE_1uA_1k:
                    ranged_value *= 1.0f / 1E10f;
                    break;
                case f30::range_t::RANGE_10uA_10k_10mV:
                    ranged_value *= 1.0f / 1E9f;
                    break;
                case f30::range_t::RANGE_100uA_100k_100mV:
                    ranged_value *= 1.0f / 1E8f;
                    break;
                case f30::range_t::RANGE_1mA_1M_1V:
                    ranged_value *= 1.0f / 1E7f;
                    break;
                case f30::range_t::RANGE_10mA_10V:
                    ranged_value *= 1.0f / 1E6f;
                    break;
                default:
                    ESP_LOGE(TAG, "Unknown range for I: %0" PRIX8, register_file.RANGE);
                    break;
                }
                break;
            case f30::quantity_t::NPD_R:
                switch (register_file.RANGE)
                {
                case f30::range_t::RANGE_1uA_1k:
                    ranged_value *= 0.1f;
                    break;
                case f30::range_t::RANGE_10uA_10k_10mV:
                    break;
                case f30::range_t::RANGE_100uA_100k_100mV:
                    ranged_value *= 10;
                    break;
                case f30::range_t::RANGE_1mA_1M_1V:
                    ranged_value *= 1E2;
                    break;
                default:
                    ESP_LOGE(TAG, "Unknown range for R: %0" PRIX8, register_file.RANGE);
                    break;
                }
                break;
            default:
                ESP_LOGE(TAG, "Unknown unit code: %0" PRIX8, register_file.NPD_QUANTITY);
                break;
            }
            if (register_file.NPD_MINUS) ranged_value = -ranged_value;
            //Execute callback
            bool do_trigger = false;
            if (callback) do_trigger = callback(&register_file, ranged_value);
            data_read_stats++;
            //Trigger next measurement
            if (!do_trigger) continue;
            uint32_t interval = *auto_trigger_interval;
            vTaskDelayUntil(&last_woken, pdMS_TO_TICKS(interval));
            trigger();
        }
    }

    const char* get_quantity_string(quantity_t q)
    {
        switch (q)
        {
        case quantity_t::NPD_I:
            return "A";
        case quantity_t::NPD_R:
            return "Ohm";
        case quantity_t::NPD_U:
            return "V";
        default:
            break;
        }
        return "";
    }
    const char* get_range_string(range_t r)
    {
        switch (r)
        {
        case range_t::RANGE_1uA_1k:
            return "1uA/1k";
        case range_t::RANGE_10uA_10k_10mV:
            return "10mV/10uA/10k";
        case range_t::RANGE_100uA_100k_100mV:
            return "100mV/100uA/100k";
        case range_t::RANGE_1mA_1M_1V:
            return "1V/1mA/1M";
        case range_t::RANGE_10mA_10V:
            return "10V/10mA";
        case range_t::RANGE_100V:
            return "100V";
        case range_t::RANGE_350V:
            return "350V";
        default: return "";
        }
    }
    void trigger()
    {
        my_hal::set_trigger(true);
        vTaskDelay(pdMS_TO_TICKS(8));
        my_hal::set_trigger(false);
        trigger_stats++;
    }
    void init(bool (*data_read_callback)(const reg_file_t* data, float ranged_value), const volatile uint32_t* interval_ptr)
    {
        xTaskCreate(read_task, "f30_read", 4096, NULL, 1, &read_task_handle);
        assert(read_task_handle);
        register_file_mutex = xSemaphoreCreateMutex();
        assert(register_file_mutex);
        callback = data_read_callback;
        auto_trigger_interval = interval_ptr;
        assert(auto_trigger_interval);
        initialized = true;
    }

    void dbg_print()
    {
        uint32_t read = data_read_stats;
        uint32_t trigger = trigger_stats;
        printf("F30 status:\n"
            "\tTotal data read events = %" PRIu32 "\n"
            "\tTotal trigger events = %" PRIu32 "\n"
            "\tRegister file (LSB->MSB):",
            read,
            trigger
        );
        xSemaphoreTake(register_file_mutex, portMAX_DELAY);
        for (size_t i = 0; i < sizeof(register_file); i++)
        {
            putchar(' ');
            uint8_t p = reinterpret_cast<uint8_t*>(&register_file)[i];
            for (size_t j = 0; j < CHAR_BIT; j++)
            {
                if (p & _BV(j)) putchar('1');
                else putchar('0');
            }
        }
        range_t r = register_file.RANGE;
        quantity_t q = register_file.NPD_QUANTITY;
        xSemaphoreGive(register_file_mutex);
        printf("\n\tMeasurement quantity: %s\n"
            "\tRange: %s\n",
            get_quantity_string(q),
            get_range_string(r)
        );
    }
} // namespace f30
