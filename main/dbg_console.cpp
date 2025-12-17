#include "dbg_console.h"

#include "macros.h"
#include "params.h"
#include "my_hal.h"
#include "eth_console_vfs.h"
#include "modbus.h"
#include "f30.h"
#include "eth_mdns_init.h"

#include "esp_linenoise.h"
#include "linenoise/linenoise.h"
#include "esp_linenoise_shim.h"
#include "argtable3/argtable3.h"
#include "driver/uart.h"
#include "esp_chip_info.h"
#include "esp_log.h"
#include "esp_flash.h"
#include "driver/uart_vfs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <cstring>
#include <string.h>
#include <sys/fcntl.h>

#define PROMPT_STR CONFIG_IDF_TARGET
#define PROMPT_MAX_LEN 32
#define MAX_CMDLINE_LENGTH 256

using namespace my_dbg_helpers;

enum console_instances : size_t {
    CONSOLE_INST_UART = 0,
    CONSOLE_INST_ETH,

    CONSOLE_TOTAL_INST
};
struct console_instance_t
{
    console_instances type;
    esp_linenoise_handle_t linenoise_handle;
    int stdout_fd;
};

static const char* TAG = "DBG_MENU";
static const char interactive_prompt[] = LOG_COLOR_I PROMPT_STR "> " LOG_RESET_COLOR;
static const char dumb_prompt[] = PROMPT_STR "> ";
QueueHandle_t interop_queue_handle;
static dbg_console::interop_cmd_t interop_cmd;
static console_instance_t consoles[CONSOLE_TOTAL_INST] = { { .type = CONSOLE_INST_UART }, { .type = CONSOLE_INST_ETH } };
static SemaphoreHandle_t esp_console_mutex = NULL;
static vprintf_like_t default_vprintf = NULL;

static void initialize_console();
static void probe_terminal(esp_linenoise_handle_t h);

namespace my_dbg_commands {
    static console_instance_t* console_context = NULL;

    int dump_nvs(int argc, char** argv)
    {
        printf(
            "\tAutotrigger interval = %" PRIu32 "\n"
            "\tAutotrigger locally = %i\n"
            "\tmDNS hostname = %s\n",
            *my_params::get_autotrigger_interval(),
            my_params::get_autotrigger_locally() ? 1 : 0,
            my_params::get_hostname()
        );
        return 0;
    }
    int hw_report(int argc, char** argv)
    {
        f30::dbg_print();
        modbus::dbg_print();
        return 0;
    }
    /* 'version' command */
    static int get_version(int argc, char** argv)
    {
        const char* model;
        esp_chip_info_t info;
        uint32_t flash_size;
        esp_chip_info(&info);

        switch (info.model) {
        case CHIP_ESP32:
            model = "ESP32";
            break;
        case CHIP_ESP32S2:
            model = "ESP32-S2";
            break;
        case CHIP_ESP32S3:
            model = "ESP32-S3";
            break;
        case CHIP_ESP32C3:
            model = "ESP32-C3";
            break;
        case CHIP_ESP32H2:
            model = "ESP32-H2";
            break;
        default:
            model = "Unknown";
            break;
        }
        if (esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
            printf("Get flash size failed");
            return 1;
        }
        printf("IDF Version:%s\r\n", esp_get_idf_version());
        printf("Chip info:\r\n");
        printf("\tmodel:%s\r\n", model);
        printf("\tcores:%d\r\n", info.cores);
        printf("\tfeature:%s%s%s%s%" PRIu32 "%s\r\n",
            info.features & CHIP_FEATURE_WIFI_BGN ? "/802.11bgn" : "",
            info.features & CHIP_FEATURE_BLE ? "/BLE" : "",
            info.features & CHIP_FEATURE_BT ? "/BT" : "",
            info.features & CHIP_FEATURE_EMB_FLASH ? "/Embedded-Flash:" : "/External-Flash:",
            flash_size / (1024 * 1024), " MB");
        printf("\trevision number:%d\r\n", info.revision);
        printf("FW ver = %s\r\n", FIRMWARE_VERSION_STR);
        return 0;
    }
    static int reboot(int argc, char** argv)
    {
        esp_restart();
    }
    static int reset_nvs(int argc, char** argv)
    {
        return my_params::factory_reset();
    }
    static int save_nvs(int argc, char** argv)
    {
        return my_params::save();
    }
    static int set_sn(int argc, char** argv)
    {
        if (argc < 2) return 1;
        if (strnlen(argv[1], INFO_STR_MAX_LEN + 1) > INFO_STR_MAX_LEN) return 2;
        my_params::set_serial_number(argv[1]);
        return 0;
    }
    static int set_pcb(int argc, char** argv)
    {
        if (argc < 2) return 1;
        if (strnlen(argv[1], INFO_STR_MAX_LEN + 1) > INFO_STR_MAX_LEN) return 2;
        my_params::set_pcb_revision(argv[1]);
        return 0;
    }
    static int test_nvs_crc(int argc, char** argv)
    {
        my_params::test_crc_dbg();
        return 0;
    }
    static int reset_dev_info(int argc, char** argv)
    {
        my_params::reset_dev_info_dbg();
        return 0;
    }
    static int override_error(int argc, char** argv)
    {
        my_dbg_helpers::interop_enqueue(dbg_console::interop_cmds::override_errors, NULL);
        return 0;
    }
    static int log_set_debug(int argc, char** argv)
    {
        esp_log_level_set("*", esp_log_level_t::ESP_LOG_DEBUG);
        return 0;
    }
    static int get_reset_reason(int argc, char** argv)
    {
        return esp_reset_reason();
    }
    static int get_free_heap(int argc, char** argv)
    {
        printf("%u\n", xPortGetFreeHeapSize());
        return 0;
    }
    static int set_hostname(int argc, char** argv)
    {
        if (argc < 2) return 1;
        if (strnlen(argv[1], MDNS_MAX_HOSTNAME_LEN + 1) > MDNS_MAX_HOSTNAME_LEN) return 2;
        my_params::set_hostname(argv[1]);
        return 0;
    }
    static int probe(int argc, char** argv)
    {
        assert(console_context);
        probe_terminal(console_context->linenoise_handle);
        return 0;
    }
    static int set_auto_trigger_delay(int argc, char** argv)
    {
        if (argc < 2) return 1;
        uint32_t delay; //ms
        if (sscanf(argv[1], "%" SCNu32, &delay) != 1) return 2;
        if (delay < (1000u / 20u)) return 3; //20 Hz max
        my_params::set_autotrigger_interval(delay);
        return 0;
    }
    static int set_auto_trigger_locally(int argc, char** argv)
    {
        if (argc < 2) return 1;
        if (argv[1][0] == '1')
        {
            my_params::set_autotrigger_locally(true);
        }
        else if (argv[1][0] == '0')
        {
            my_params::set_autotrigger_locally(false);
        }
        else return 2;
        return 0;
    }
}

static const esp_console_cmd_t commands[] = {
    { .command = "dump_nvs",
        .help = "Dump NVS data",
        .hint = NULL,
        .func = &my_dbg_commands::dump_nvs },
    { .command = "hw_report",
        .help = "Report hardware state",
        .hint = NULL,
        .func = &my_dbg_commands::hw_report },
    { .command = "version",
        .help = "Get version of chip and SDK",
        .hint = NULL,
        .func = &my_dbg_commands::get_version },
    { .command = "reboot",
        .help = "Software reset",
        .hint = NULL,
        .func = &my_dbg_commands::reboot },
    { .command = "reset_nvs",
        .help = "Erase NVS storage section (reset required to load defaults)",
        .hint = NULL,
        .func = &my_dbg_commands::reset_nvs },
    { .command = "save_nvs",
        .help = "Save configuration to NVS",
        .hint = NULL,
        .func = &my_dbg_commands::save_nvs },
    { .command = "set_sn",
        .help = "Set device S/N (string up to 31 characters long)",
        .hint = NULL,
        .func = &my_dbg_commands::set_sn },
    { .command = "set_pcb",
        .help = "Set pcb rev (string up to 31 characters long)",
        .hint = NULL,
        .func = &my_dbg_commands::set_pcb },
    { .command = "test_nvs_crc",
        .help = "Set CRC to 0",
        .hint = NULL,
        .func = &my_dbg_commands::test_nvs_crc },
    { .command = "reset_dev_info",
        .help = "Reset device info SPIFFS file",
        .hint = NULL,
        .func = &my_dbg_commands::reset_dev_info },
    { .command = "override_error",
        .help = "Override any startup error",
        .hint = NULL,
        .func = &my_dbg_commands::override_error },
    { .command = "log_set_debug",
        .help = "Set log level to DEBUG. This action can be undone only by a reset.",
        .hint = NULL,
        .func = &my_dbg_commands::log_set_debug },
    { .command = "get_reset_reason",
        .help = "Returns reset reason code",
        .hint = NULL,
        .func = &my_dbg_commands::get_reset_reason },
    { .command = "get_free_heap",
        .help = "Prints free heap memory according to FreeRTOS",
        .hint = NULL,
        .func = &my_dbg_commands::get_free_heap },
    { .command = "set_hostname",
        .help = "Set mDNS hostname",
        .hint = NULL,
        .func = &my_dbg_commands::set_hostname },
    { .command = "probe",
        .help = "Re-probe the terminal capabilities",
        .hint = NULL,
        .func = &my_dbg_commands::probe },
    { .command = "set_trig_interval",
        .help = "Set auto trigger interval (integer, mS)",
        .hint = NULL,
        .func = &my_dbg_commands::set_auto_trigger_delay },
    { .command = "set_trig_local",
        .help = "Enable (1) or disable (0) local auto trigger",
        .hint = NULL,
        .func = &my_dbg_commands::set_auto_trigger_locally }
};

/// @brief Figure out if the terminal supports escape sequences
static void probe_terminal(esp_linenoise_handle_t h)
{
    ESP_LOGI(TAG, "Will now probe...");
    int probe_status = _esp_linenoise_probe(h);
    if (probe_status) { /* zero indicates success */
        printf("\n"
               "Your terminal application does not support escape sequences.\n"
               "Line editing and history features are disabled.\n"
               "On Windows, try using Putty instead. Status: %d\n", probe_status);
        esp_linenoise_set_dumb_mode(h, true);
#if CONFIG_LOG_COLORS
        /* Since the terminal doesn't support escape sequences,
         * don't use color codes in the prompt.
         */
        _esp_linenoise_set_prompt(h, dumb_prompt);
#endif // CONFIG_LOG_COLORS
    }
    else
    {
        printf("\n"
           "Type 'help' to get the list of commands.\n"
           "Use UP/DOWN arrows to navigate through command history.\n"
           "Press TAB when typing command name to auto-complete.\n");
        esp_linenoise_set_dumb_mode(h, false);
#if CONFIG_LOG_COLORS
        _esp_linenoise_set_prompt(h, interactive_prompt);
#endif // CONFIG_LOG_COLORS
    }
}
void esp_console_get_completion_wrapper(const char *str, void *cb_ctx, esp_linenoise_completion_cb_t cb)
{
    while (xSemaphoreTakeRecursive(esp_console_mutex, portMAX_DELAY) != pdTRUE);

    linenoiseCompletions lc;
    esp_console_get_completion(str, &lc);
    for (size_t i = 0; i < lc.len; i++)
    {
        cb(cb_ctx, lc.cvec[i]);
    }

    xSemaphoreGiveRecursive(esp_console_mutex);
}
char* esp_console_get_hint_wrapper(const char *str, int *color, int *bold)
{   
    while (xSemaphoreTakeRecursive(esp_console_mutex, portMAX_DELAY) != pdTRUE);

    char* ret = const_cast<char*>(esp_console_get_hint(str, color, bold));

    xSemaphoreGiveRecursive(esp_console_mutex);
    return ret;
}
int local_vprintf(const char *fmt, va_list args)
{
    if (!default_vprintf) return -1;
    int ret1 = default_vprintf(fmt, args);
    if (consoles[CONSOLE_INST_ETH].stdout_fd != fileno(stdout))
    {
        int ret2 = eth_console_vfs::vprintf(fmt, args);
        return ret2 < ret1 ? ret2 : ret1;
    }
    return ret1;
}
/// @brief Initialize esp console, lineNoise library and install uart VFS drivers, redirecting stdout into the console.
static void initialize_console()
{
    setvbuf(stdin, NULL, _IONBF, 0);
    ESP_ERROR_CHECK(uart_driver_install((uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM,
        256, 0, 0, NULL, 0));
    uart_vfs_dev_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CR);
    uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);

    ESP_ERROR_CHECK_WITHOUT_ABORT(eth_console_vfs::init_console());
    eth_console_vfs::set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    default_vprintf = esp_log_set_vprintf(local_vprintf);
    ESP_LOGI(TAG, "Vprintf redirected successfully");

    /* Initialize the console 
    *   The esp_console is a singleton as has to be protected with a mutex from multiple access by linenoise instances
    */
    esp_console_config_t console_config = {
        .max_cmdline_length = MAX_CMDLINE_LENGTH,
        .max_cmdline_args = 8,
#if CONFIG_LOG_COLORS
        .hint_color = atoi(LOG_COLOR_CYAN)
#endif
    };
    ESP_ERROR_CHECK(esp_console_init(&console_config));

    for (size_t i = 0; i < CONSOLE_TOTAL_INST; i++)
    {
        esp_linenoise_config_t config;
        esp_linenoise_get_instance_config_default(&config);
        config.completion_cb = &esp_console_get_completion_wrapper;
        config.hints_cb = &esp_console_get_hint_wrapper;
        config.allow_multi_line = true;
        config.history_max_length = 32;
        config.max_cmd_line_length = console_config.max_cmdline_length;
        config.allow_empty_line = false;
        config.allow_dumb_mode = true;
        config.prompt =
#if CONFIG_LOG_COLORS
            interactive_prompt
#else
            dumb_prompt
#endif
        ;
        if (i == static_cast<size_t>(CONSOLE_INST_ETH))
        {
            FILE *eth_rx, *eth_tx;
            eth_console_vfs::get_streams(&eth_rx, &eth_tx);
            config.in_fd = fileno(eth_rx);
            config.out_fd = fileno(eth_tx);
        }
        consoles[i].stdout_fd = config.out_fd;
        ESP_ERROR_CHECK(esp_linenoise_create_instance(&config, &(consoles[i].linenoise_handle)));
        ESP_LOGI(TAG, "Console %i initialized!", i);
    }

    /* Register commands */
    esp_console_register_help_command();
    my_dbg_helpers::register_cmds(commands, ARRAY_SIZE(commands));
}
/// @brief Console input parser task body function.
/// @param arg esp_linenoise_handle_t
static void parser_task(void* arg)
{
    char line[MAX_CMDLINE_LENGTH];
    console_instance_t* con = reinterpret_cast<console_instance_t*>(arg);
    switch (con->type)
    {
    case console_instances::CONSOLE_INST_ETH:
        ESP_ERROR_CHECK_WITHOUT_ABORT(eth_console_vfs::redirect_std_streams());
        break;
    default:
        break;
    }
    probe_terminal(con->linenoise_handle);
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(20));
        /* Get a line using linenoise.
         * The line is returned when ENTER is pressed.
         */
        esp_err_t res = esp_linenoise_get_line(con->linenoise_handle, line, sizeof(line));
        if (res != ESP_OK) { /* Break on EOF or error */
            continue;
        }
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_linenoise_history_add(con->linenoise_handle, line));

        /* Try to run the command */
        int ret;
        while (xSemaphoreTakeRecursive(esp_console_mutex, portMAX_DELAY) != pdTRUE);
        my_dbg_commands::console_context = con;
        esp_err_t err = esp_console_run(line, &ret);
        my_dbg_commands::console_context = NULL;
        xSemaphoreGiveRecursive(esp_console_mutex);
        if (err == ESP_ERR_NOT_FOUND) {
            ESP_LOGW(TAG, "Unrecognized command: '%s'\n", line);
        } else if (err == ESP_ERR_INVALID_ARG) {
            // command was empty
        } else if (err == ESP_OK && ret != ESP_OK) {
            ESP_LOGW(TAG, "Command returned non-zero error code: 0x%x (%s)\n", ret, esp_err_to_name(ret));
        } else if (err != ESP_OK) {
            ESP_LOGE(TAG, "Internal error: %s\n", esp_err_to_name(err));
        }
    }
}

namespace my_dbg_helpers
{
    /// @brief Boolean argument parser helper function
    /// @param argc from console
    /// @param argv from console
    /// @return True if argument was '1' or if there was no argument. False otherwise.
    bool bool_arg_helper(int argc, char** argv)
    {
        if (argc > 1) {
            char* a = argv[1];
            return *a == '1';
        } else {
            return true;
        }
    }
    /// @brief A helper function to register an array of console commands
    /// @param arr array
    /// @param len array length
    void register_cmds(const esp_console_cmd_t* arr, size_t len)
    {
        for (size_t i = 0; i < len; i++)
        {
            esp_console_cmd_register(&(arr[i]));
        }
    }
    /// @brief Try to enqueue a new interoperability command.
    /// @param cmd Interop command
    /// @param arg Interop arguments
    /// @return True if succeeded, False otherwise.
    bool interop_enqueue(dbg_console::interop_cmds cmd, void* arg)
    {
        assert(interop_queue_handle);

        interop_cmd.cmd = cmd;
        interop_cmd.args = arg;
        if (xQueueSend(interop_queue_handle, &interop_cmd, 0) != pdTRUE)
        {
            printf("Failed to enqueue a new debug interoperation. Please wait for previous ones to finish.\n");
            return false;
        }
        else
        {
            ESP_LOGD(TAG, "Enqued interop message");
        }
        return true;
    }
}


namespace dbg_console {
    /// @brief Initialize common debug console and lineNoise, install uart VFS driver. Creates debug console task.
    /// @param interop_queue Interoperablity queue used to execute debug commands like calibrations and error overrides
    void init(QueueHandle_t interop_queue)
    {
        ESP_LOGI(TAG, "Initializing...");
        assert(interop_queue);
        esp_console_mutex = xSemaphoreCreateRecursiveMutex();
        assert(esp_console_mutex);

        interop_queue_handle = interop_queue;
        initialize_console();
        assert(xTaskCreate(parser_task, "uart_console_parser", 10000, &(consoles[console_instances::CONSOLE_INST_UART]), 1, NULL) == pdPASS);
        assert(xTaskCreate(parser_task, "eth_console_parser", 10000, &(consoles[console_instances::CONSOLE_INST_ETH]), 1, NULL) == pdPASS);
    }
}