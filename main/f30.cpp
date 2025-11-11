#include "f30.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "my_hal.h"

namespace f30
{
    static TaskHandle_t read_task_handle = NULL;
    static reg_file_t register_file = {};
    static bool initialized = false;
    static bool (*callback)(const reg_file_t* data) = NULL;
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
            bool do_trigger = false;
            if (callback) do_trigger = callback(&register_file);
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
    void init(bool (*data_read_callback)(const reg_file_t* data), const volatile uint32_t* interval_ptr)
    {
        xTaskCreate(read_task, "f30_read", 4096, NULL, 1, &read_task_handle);
        assert(read_task_handle);
        callback = data_read_callback;
        auto_trigger_interval = interval_ptr;
        assert(auto_trigger_interval);
        initialized = true;
    }
} // namespace f30
