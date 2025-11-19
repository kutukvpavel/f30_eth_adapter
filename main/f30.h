#pragma once

#include <esp_attr.h>

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus

namespace f30
{
    enum units_t : uint8_t
    {
        NPD_U = 0,
        NPD_I = 1u << 1u,
        NPD_R = 1u << 2u
    };
    enum range_t : uint8_t
    {
        RANGE_1uA_1k = 0,
        RANGE_10uA_10k_10mV = 1u << 1u,
        RANGE_100uA_100k_100mV = 1u << 2u,
        RANGE_1mA_1M_1V = 1u << 3u,
        RANGE_10mA_10V = 1u << 4u,
        RANGE_100V = 1u << 5u,
        RANGE_350V = 1u << 6u,
    };

    struct __packed reg_file_t
    {
        uint8_t padding_zero : 4;
        uint8_t DEC1 : 4;
        uint8_t DEC2 : 4;
        uint8_t DEC3 : 4;
        uint8_t DEC4 : 4;
        uint8_t DEC5 : 1;
        uint8_t NPD_PLUS : 1;
        uint8_t NPD_MINUS : 1;
        units_t NPD_UNITS : 3;
        range_t RANGE : 7;
    };

    void read_interrupt_handler(void* arg);

    void trigger();
    void init(bool (*data_read_callback)(const reg_file_t* data, float ranged_value), const volatile uint32_t* interval_ms);

    void dbg_print();
} // namespace f30

#endif