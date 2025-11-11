#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "rom/crc.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <inttypes.h>

#define INFO_STR_MAX_LEN 31

/// @brief Pointers to device info strings
struct my_dev_info_t
{
    const char* name;
    const char* manufacturer; ///< Device manufacturer (IRZ)
    const char* model; ///< Device model, should be used to differentiate portable and desktop variants
    const char* sn; ///< Serial number, should be unique and should be noted in the device documents
    const char* pcb_rev; ///< PCB hardware revision
};

/// @brief Public API related to configuration and data storages (NVS, SPIFFS, raw partitions)
namespace my_params 
{
    esp_err_t init();
    esp_err_t save();
    const volatile uint8_t* get_nvs_dump(size_t* len);
    esp_err_t factory_reset();
    esp_err_t reset();

    const volatile uint32_t* get_autotrigger_interval();
    void set_autotrigger_interval(uint32_t i);
    bool get_autotrigger_locally();
    void set_autotrigger_locally(bool b);
}

/// @brief Private params helper API
namespace my_params_helpers
{
    void report_spiffs_error(const char* msg, const char* path);
    size_t read_str(FILE* f, char* buf, size_t offset, size_t max_len);
    size_t write_str(FILE* f, const char* buf, size_t offset);
    esp_err_t open_helper(nvs_handle_t* handle, nvs_open_mode_t mode);
    uint8_t read_nvs_ver(nvs_handle_t handle, const char* id);
    esp_err_t reset_nvs(const char* ver_id, const char* id);

    template<typename T> esp_err_t save_helper(nvs_handle_t handle, const char* ver_id, uint8_t ver, const char* id, T* val)
    {
        nvs_set_u8(handle, ver_id, ver);
        return nvs_set_blob(handle, id, val, sizeof(T));
    }
    template<typename T> esp_err_t init_nvs(const char* ver_id, uint8_t ver, const char* id, T* val)
    {
        esp_err_t err;
        nvs_handle_t handle;
        err = open_helper(&handle, NVS_READONLY);
        if (err != ESP_OK) return err;
        // Read the size of memory space required for blob
        T tmp;
        size_t required_size = sizeof(tmp); // value will default to sizeof(tmp), if not set yet in NVS
        err = nvs_get_blob(handle, id, &tmp, &required_size);
        if (err == ESP_ERR_NVS_NOT_FOUND || err == ESP_ERR_NVS_INVALID_LENGTH) 
        {
            nvs_close(handle);
            open_helper(&handle, nvs_open_mode::NVS_READWRITE);
            save_helper(handle, ver_id, ver, id, val); //If not found, write defaults
            ESP_LOGW("PARAMS", "NVS storage not found. Reset to defaults.");
        }
        else if (err == ESP_ERR_NVS_INVALID_STATE)
        {
            my_params::factory_reset();
            ESP_LOGW("PARAMS", "NVS state not consistent. Reset to defaults.");
            vTaskDelay(pdMS_TO_TICKS(1000));
            abort();
        }
        else if (err != ESP_OK) {
            ESP_LOGE("PARAMS", "Error reading NVS: %s", esp_err_to_name(err));
            return err;
        }
        else
        {
            *val = tmp;
        }
        uint8_t v = read_nvs_ver(handle, ver_id);
        nvs_close(handle);
        if (v != ver)
        {
            ESP_LOGW("PARAMS", "NVS version not consistent. Reset to defaults.");
            ESP_ERROR_CHECK(reset_nvs(ver_id, id));
            vTaskDelay(pdMS_TO_TICKS(500));
            abort();
        }
        return err;
    }
}

namespace my_params
{
    void test_crc_dbg();
    void reset_dev_info_dbg();
    uint8_t get_nvs_version();

    my_dev_info_t* get_dev_info();
    void set_serial_number(const char* val);
    void set_pcb_revision(const char* val);
}
