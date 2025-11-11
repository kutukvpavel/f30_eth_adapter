/*
 * SPDX-FileCopyrightText: 2016-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*=====================================================================================
 * Description:
 *   The Modbus parameter structures used to define Modbus instances that
 *   can be addressed by Modbus protocol. Define these structures per your needs in
 *   your application. Below is just an example of possible parameters.
 *====================================================================================*/
#ifndef _DEVICE_PARAMS
#define _DEVICE_PARAMS

#include <stdint.h>

#define MAX_REGISTERS 255

#ifdef __cplusplus
extern "C" {
#endif

// This file defines structure of modbus parameters which reflect correspond modbus address space
// for each modbus register type (coils, discreet inputs, holding registers, input registers)
#pragma pack(push, 1)
typedef struct
{
    uint8_t init_ok:1;
    uint8_t discrete_input1:1;
    uint8_t discrete_input2:1;
    uint8_t discrete_input3:1;
    uint8_t discrete_input4:1;
    uint8_t discrete_input5:1;
    uint8_t discrete_input6:1;
    uint8_t discrete_input7:1;
} discrete_reg_params_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
    uint8_t enable_remote:1;
    uint8_t enable_auto_trigger:1;
    uint8_t single_shot:1;
    uint8_t coil_3:1;
    uint8_t coil_4:1;
    uint8_t coil_5:1;
    uint8_t coil_6:1;
    uint8_t coil_7:1;
} coil_reg_params_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
    float measured_value;
    uint16_t unit_code;
    uint16_t range_code;
    uint16_t test_regs[MAX_REGISTERS - 1 * 2 - 2 * 1];
} input_reg_params_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
    uint16_t autotrigger_interval; //ms
    uint16_t test_regs[MAX_REGISTERS - 1 * 1];
} holding_reg_params_t;
#pragma pack(pop)

extern holding_reg_params_t holding_reg_params;
extern input_reg_params_t input_reg_params;
extern coil_reg_params_t coil_reg_params;
extern discrete_reg_params_t discrete_reg_params;

#ifdef __cplusplus
}
#endif

#endif // !defined(_DEVICE_PARAMS)
