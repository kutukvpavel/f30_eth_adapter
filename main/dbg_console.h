#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_console.h"

/// @brief Debug console public API
namespace dbg_console
{
    /// @brief Main-loop interoperability queue commands that can be issued by the debug console
    enum interop_cmds
    {
        override_errors
    };
    
    /// @brief Main-loop interoperability queue command structure for the debug console
    struct interop_cmd_t
    {
        interop_cmds cmd; ///< Command, see interop_cmds
        void* args; ///< Arguments (actual structure depends on the command)
    };

    void init(QueueHandle_t interop_queue);
}

/// @brief Debug console helper functions (private API)
namespace my_dbg_helpers
{
    bool bool_arg_helper(int argc, char** argv);
    void register_cmds(const esp_console_cmd_t* arr, size_t len);
    bool interop_enqueue(dbg_console::interop_cmds cmd, void* arg);
}