#pragma once

#include <esp_err.h>

namespace meter_web_server
{
    esp_err_t init();

    void set_data(float reading, const char* units);
} // namespace meter_web_server
