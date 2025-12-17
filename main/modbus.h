#pragma once

#include <esp_err.h>
#include <esp_netif.h>

namespace modbus
{
    void init(esp_netif_t* netif_ptr);

    bool get_remote_enabled();
    bool get_single_shot_requested();
    bool get_auto_trigger_enabled();
    uint16_t get_auto_trigger_interval();

    void set_init_ok(uint16_t initial_interval);
    void set_values(float measured_value, uint16_t quantity_code, uint16_t range_code);
    void disable_remote();

    void dbg_print();
} // namespace modbus
