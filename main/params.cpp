/**
 * @file params_common.cpp
 * @author MSU
 * @brief NVS: Common parameter storage. 
 * SPIFFS: temperature profile, device info, measurements history, sensor calibration curve (left for debug purposes, legacy)
 * Memory-mapped partition: ML model
 * @date 2024-11-26
 * 
 */

#include "params.h"

#include "nvs.h"
#include "nvs_handle.hpp"
#include "rom/crc.h"
#include "macros.h"
#include "esp_check.h"
#include "esp_spiffs.h"

#include <string.h>

using namespace my_params_helpers;

/// @brief This is the main parameter blob
/// Do not modify! If you do, OTA update will be impossible (breaking change) due to NVS layout change.
/// If you need to add a new parameter, create a new NVS entry with a new ID (see several of those below).
struct my_param_storage_common_t
{
    volatile uint32_t autotrigger_interval_ms;
};
/// @brief NVS RAM cache, initialized with defaults
static my_param_storage_common_t storage =
{
    .autotrigger_interval_ms = 100
};
/// @brief SPIFFS configuration
static esp_vfs_spiffs_conf_t flash_conf = 
{
    .base_path = "/spiffs",
    .partition_label = NULL,
    .max_files = 4,
    .format_if_mount_failed = true
};
/// @brief Debug console log tag
static const char TAG[] = "PARAMS";
/*** NVS storage constants */
static const uint8_t storage_ver = 3;
static const char storage_ver_id[] = "storage_ver";
static const char storage_val_id[] = "storage";
static const char my_nvs_namespace[] = "my";
/*** SPIFFS storage constants */
static const char flash_info_path[] = "/spiffs/i.bin"; //Device info, strings at constant offsets (32*6 = 192 --> 256B)

namespace my_params
{
    /// @brief Gets device info strings. If none have been written to the SPIFFS file, default strings will be used as required.
    /// @return Pointer to a static my_dev_info_t buffer inside this function.
    my_dev_info_t* get_dev_info()
    {
        const size_t len = 4;
        static const char na[] = "N/A";
        static char buffer[len][INFO_STR_MAX_LEN + 1];
        static my_dev_info_t res;

        res = { "MDC", "SensorBurner", na, na };
        FILE* f = fopen(flash_info_path, "rb");
        if (f == NULL) 
        {
            report_spiffs_error("SPIFFS: file does not exist, reset to defaults", flash_info_path);
            ESP_LOGD(TAG, "Errno: %i", errno);
            //Pad the file with null-characters manually, since SPIFFS doesn't support seeking past EOF
            f = fopen(flash_info_path, "w+b");
            if (f == NULL)
            {
                report_spiffs_error("SPIFFS: failed to create file", flash_info_path);
                return &res;
            }
            const char pad_chr = '\0';
            const size_t pad_len = (INFO_STR_MAX_LEN + 1) * len + 1;
            if (fwrite(&pad_chr, sizeof(pad_chr), pad_len, f) < pad_len)
            {
                report_spiffs_error("SPIFFS: failed to pad file", flash_info_path);
                return &res;
            }
            const char nl = '\n';
            if (fwrite(&nl, sizeof(nl), 1, f) < 1)
            {
                report_spiffs_error("SPIFFS: failed to finalize file padding", flash_info_path);
                return &res;
            }
            ESP_LOGI(TAG, "Padded i.bin with %u null-bytes + \\n", pad_len);
            for (size_t i = 0; i < len; i++)
            {
                size_t w = 0;
                size_t o = i * (INFO_STR_MAX_LEN + 1);
                if ( (w = write_str(f, reinterpret_cast<const char**>(&res)[i], o)) )
                {
                    ESP_LOGI(TAG, "Written %u bytes into string #%u @ %u in i.bin", w, i, o);
                }
                else
                {
                    ESP_LOGE(TAG, "Failed to write into string #%u @ %u in i.bin", i, o);
                }
            }
            fclose(f);
            return &res;
        }
        
        auto p = reinterpret_cast<const char**>(&res);
        for (size_t i = 0; i < len; i++)
        {
            size_t o = (INFO_STR_MAX_LEN + 1) * i;
            if (read_str(f, buffer[i], o, INFO_STR_MAX_LEN)) p[i] = buffer[i];
            else
            {
                ESP_LOGW(TAG, "Failed to read devinfo string #%u @ %u", i, o);
            }
        }

        fclose(f);
        return &res;
    }

    /// @brief Set serial number string
    /// @param val Can't be longer than INFO_STR_MAX_LEN characters (usually 31)
    void set_serial_number(const char* val)
    {
        FILE* f = fopen(flash_info_path, "r+b");
        if (f == NULL) 
        {
            report_spiffs_error("Failed to open file", flash_info_path);
            unlink(flash_info_path);
            return;
        }
        const size_t offset = (INFO_STR_MAX_LEN + 1) * 2;
        ESP_LOGI(TAG, "Writing %u bytes at %u offset (sn) into i.bin", strnlen(val, INFO_STR_MAX_LEN), offset);
        write_str(f, val, offset);
        fclose(f);
    }
    /// @brief Set PCB revision string
    /// @param val Can't be longer than INFO_STR_MAX_LEN characters (usually 31)
    void set_pcb_revision(const char* val)
    {
        FILE* f = fopen(flash_info_path, "r+b");
        if (f == NULL) 
        {
            report_spiffs_error("Failed to open file", flash_info_path);
            unlink(flash_info_path);
            return;
        }
        const size_t offset = (INFO_STR_MAX_LEN + 1) * 3;
        ESP_LOGI(TAG, "Writing %u bytes at %u offset (pcb rev) into i.bin", strnlen(val, INFO_STR_MAX_LEN), offset);
        write_str(f, val, offset);
        fclose(f);
    }

    /// @brief Initialize all of the common storage spaces
    /// @return ESP_OK if succeeded, ESP_ERR_NOT_FOUND if calibration curve was not found, see also nvs_flash_init, esp_vfs_spiffs_register,
    /// my_params_helpers::map_model, my_params_helpers::open_model_file, esp_spiffs_info
    esp_err_t init()
    {
        // Initialize NVS
        ESP_LOGI(TAG, "Init...");
        esp_err_t err = nvs_flash_init();
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            ESP_LOGW(TAG, "NVS had been truncated and had to be erased! Retrying...");
            ESP_ERROR_CHECK(nvs_flash_erase());
            err = nvs_flash_init();
        }
        else if (err == ESP_ERR_NVS_NOT_FOUND)
        {
            err = nvs_flash_init();
        }
        ESP_ERROR_CHECK(err);
        my_params_helpers::init_nvs(storage_ver_id, storage_ver, storage_val_id, &storage); //Reads main storage blob
        //Read other (non-monolithic and non-essential) NVS keys
        /*nvs_handle_t nvs_handle;
        err = open_helper(&nvs_handle, NVS_READONLY);
        if (err == ESP_OK)
        {
            
            nvs_close(nvs_handle);
        }*/

        // Initialize SPIFFS
        ESP_LOGI(TAG, "SPIFFS init...");
        err = esp_vfs_spiffs_register(&flash_conf);
        if (err != ESP_OK)
        {
            if (err == ESP_FAIL)
            {
                ESP_LOGE(TAG, "Failed to mount or format SPIFFS");
            }
            else if (err == ESP_ERR_NOT_FOUND)
            {
                ESP_LOGE(TAG, "Failed to find SPIFFS partition");
            }
            else
            {
                ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(err));
            }
            return err;
        }
        err = esp_spiffs_check(flash_conf.partition_label);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "SPIFFS check failed (%s)", esp_err_to_name(err));
            return err;
        }
        else
        {
            ESP_LOGI(TAG, "SPIFFS check successful");
        }
        size_t total = 0, used = 0;
        err = esp_spiffs_info(flash_conf.partition_label, &total, &used);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s). Formatting...", esp_err_to_name(err));
            esp_spiffs_format(flash_conf.partition_label);
            return err;
        }
        else
        {
            ESP_LOGI(TAG, "SPIFFS: Partition size: total: %d, used: %d", total, used);
        }

        return ESP_OK;
    }
    /// @brief Save common NVS
    /// @return ESP_OK if succeeded, else see open_helper, save_helper
    esp_err_t save()
    {
        nvs_handle_t handle;
        esp_err_t err = open_helper(&handle, NVS_READWRITE);
        if (err != ESP_OK) return err;
        /*ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_set_u32(handle, key_last_set_pwr, *reinterpret_cast<uint32_t*>(&last_set_pwr)));
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_set_u32(handle, key_last_set_vlim, *reinterpret_cast<uint32_t*>(&last_set_vlim)));*/
        return save_helper(handle, storage_ver_id, storage_ver, storage_val_id, &storage);
    }
    /// @brief Bytewise NVS dump
    /// @param len NVS size (output)
    /// @return Pointer to the NVS RAM cache as a byte array.
    const volatile uint8_t* get_nvs_dump(size_t* len)
    {
        *len = sizeof(storage);
        return reinterpret_cast<const volatile uint8_t*>(&storage);
    }
    /// @brief Erases all NVS entries, including variant ones.
    /// @return 
    esp_err_t factory_reset()
    {
        return nvs_flash_erase();
    }
    /// @brief Reset only common NVS storage
    /// @return 
    esp_err_t reset()
    {
        return my_params_helpers::reset_nvs(storage_ver_id, storage_val_id);
    }
    uint8_t get_nvs_version()
    {
        nvs_handle_t h;
        my_params_helpers::open_helper(&h, nvs_open_mode::NVS_READONLY);
        auto ret = my_params_helpers::read_nvs_ver(h, storage_ver_id);
        nvs_close(h);
        return ret;
    }

    void test_crc_dbg()
    {
        nvs_handle_t handle;
        if (open_helper(&handle, nvs_open_mode::NVS_READWRITE) != ESP_OK)
        {
            ESP_LOGW(TAG, "Failed to open NVS for r/w.");
            return;
        }
        nvs_set_u32(handle, storage_ver_id, 0);
        nvs_close(handle);
    }
    /// @brief Unlink device info strings file in SPIFFS
    void reset_dev_info_dbg()
    {
        unlink(flash_info_path);
    }

    const volatile uint32_t* get_autotrigger_interval()
    {
        //Pass as a pointer to volatile only works since 32-bit types are atomic on ESP32!
        return &(storage.autotrigger_interval_ms);
    }
    void set_autotrigger_interval(uint32_t i)
    {
        storage.autotrigger_interval_ms = i;
    }
}

namespace my_params_helpers
{
    /// @brief Helper function to report SPIFFS operation error. Raises my_error_codes::spiffs_failure for the USB-CDC interface (my_uart).
    /// @param msg Error message
    /// @param path Related file path
    void report_spiffs_error(const char *msg, const char *path)
    {
        ESP_LOGE(TAG, "%s. From %s", msg, path);
    }
    /// @brief Helper function to read a variable-length device info C-string from a SPIFFS file at specified offset.
    /// @param f The file
    /// @param buf The buffer for the string to be read into
    /// @param offset File offset to start at
    /// @param max_len Max string length
    /// @return Number of characters actually read
    size_t read_str(FILE *f, char *buf, size_t offset, size_t max_len)
    {
        const char *start = buf;
        fseek(f, offset, SEEK_SET);
        int t;
        //Read until we encounter a null-terminator, copy everything including the null-terminator
        do
        {
            t = fgetc(f);
            //In case we've reached the end of the file and no terminator was found, terminate the string manually
            if (t == EOF || ((buf - start) == (max_len - 1)))
                *buf = '\0';
            else
                *buf = t;
        } while (*buf++ != '\0');
        return buf - start - 1;
    }
    /// @brief Helper function to write a variable-length device info C-string into a SPIFFS file at specified offset.
    /// @param f The file
    /// @param buf The string
    /// @param offset File offset to start at
    /// @return Number of characters written
    size_t write_str(FILE *f, const char *buf, size_t offset)
    {
        int ret;
        if ((ret = fseek(f, offset, SEEK_SET)))
        {
            report_spiffs_error("Failed to seek to specified offset. Reset the file.", flash_info_path);
            ESP_LOGD(TAG, "fseek errno: %i", ret);
            return 0;
        }
        size_t len = strnlen(buf, INFO_STR_MAX_LEN) + 1; // null-term
        size_t written = fwrite(buf, sizeof(*buf), len, f);
        if (written < len)
            report_spiffs_error("Failed to write string", flash_info_path);
        else
            ESP_LOGI(TAG, "SPIFFS: Written %u characters to %s", written, flash_info_path);
        return written;
    }
    /// @brief Helper-function to open a new NVS handle and log any possible errors
    /// @param handle Handle (output)
    /// @param mode NVS open mode (R or R/W)
    /// @return ESP_ERR_NVS_NOT_FOUND if NVS namespace was not found, otherwise see nvs_open
    esp_err_t open_helper(nvs_handle_t* handle, nvs_open_mode_t mode)
    {
        esp_err_t err = nvs_open(my_nvs_namespace, mode, handle);
        if (err == ESP_ERR_NVS_NOT_FOUND)
        {
            ESP_LOGW(TAG, "NVS namespace doesn't exist and will be created (first run?)");
            err = nvs_open(my_nvs_namespace, NVS_READWRITE, handle); // retry with write permissions
        }
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        }
        else
        {
            ESP_LOGI(TAG, "NVS handle opening SUCCESS.");
        }
        return err;
    }
    /// @brief Read common NVS storage version.
    /// @param handle An open NVS handle, provided by open_helper
    /// @param id NVS version key ID
    /// @return 0 if failed, NVS version otherwise
    uint8_t read_nvs_ver(nvs_handle_t handle, const char* id)
    {
        uint8_t res;
        esp_err_t err = nvs_get_u8(handle, id, &res);
        if (err != ESP_OK) 
        {
            ESP_LOGW(TAG, "Failed to read NVS storage version, error %s. Returning 0.", esp_err_to_name(err));
            return 0;
        }
        ESP_LOGI(TAG, "Read storage key version: 0x%X", res);
        return res;
    }
    /// @brief Helper function to erase NVS storage and storage version keys.
    /// @param ver_id Storage version NVS entry ID
    /// @param id Storage NVS entry ID
    /// @return See open_helper and nvs_erase_key
    esp_err_t reset_nvs(const char* ver_id, const char* id)
    {
        nvs_handle_t handle;
        esp_err_t ret;
        if ((ret = open_helper(&handle, nvs_open_mode::NVS_READWRITE)) == ESP_OK)
        {
            nvs_erase_key(handle, ver_id);
            return nvs_erase_key(handle, id);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to reset common NVS storge. A factory reset is required to fix this.");
            return ret;
        }
    }
}