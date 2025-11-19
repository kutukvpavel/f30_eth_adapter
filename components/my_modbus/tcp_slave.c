/*
 * SPDX-FileCopyrightText: 2016-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// FreeModbus Slave Example ESP32

#include "tcp_slave.h"

#include <stdio.h>
#include "esp_err.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"

// Defines below are used to define register start address for each type of Modbus registers
#define HOLD_OFFSET(field) ((uint16_t)(offsetof(holding_reg_params_t, field) >> 1))
#define INPUT_OFFSET(field) ((uint16_t)(offsetof(input_reg_params_t, field) >> 1))
#define MB_REG_DISCRETE_INPUT_START         (0x0000)
#define MB_REG_COILS_START                  (0x0000)
#define MB_REG_INPUT_START_AREA0            (INPUT_OFFSET(measured_value)) // register offset input area 0
#define MB_REG_HOLDING_START_AREA0          (HOLD_OFFSET(test_regs))

#define MB_PAR_INFO_GET_TOUT                (10) // Timeout for get parameter info

static const char *TAG = "mb_tcp_slave";
static void (*mb_event_handler_func)(const mb_param_info_t* reg_info) = NULL;
static void *slave_handle = NULL;

void slave_operation_func(void *arg)
{
    static mb_param_info_t reg_info; // keeps the Modbus registers access information

    ESP_LOGI(TAG, "Modbus task started.");
    for(;;) {
        // Check for read/write events of Modbus master for certain events
        (void)mbc_slave_check_event(slave_handle, MB_READ_WRITE_MASK);
        ESP_ERROR_CHECK_WITHOUT_ABORT(mbc_slave_get_param_info(slave_handle, &reg_info, MB_PAR_INFO_GET_TOUT));
        if ((reg_info.type != MB_EVENT_NO_EVENTS) && (mb_event_handler_func)) mb_event_handler_func(&reg_info);
    }
}

// Modbus slave initialization
esp_err_t slave_init(mb_communication_info_t* comm_info, void (*event_handler_func)(const mb_param_info_t*), void** handle)
{
    mb_register_area_descriptor_t reg_area; // Modbus register area descriptor structure

    mb_event_handler_func = event_handler_func;

    // Initialization of Modbus controller
    esp_err_t err = mbc_slave_create_tcp(comm_info, &slave_handle);
    MB_RETURN_ON_FALSE((err == ESP_OK && slave_handle != NULL), ESP_ERR_INVALID_STATE,
                                TAG,
                                "mb controller initialization fail.");
    *handle = slave_handle;

    // The code below initializes Modbus register area descriptors
    // for Modbus Holding Registers, Input Registers, Coils and Discrete Inputs
    // Initialization should be done for each supported Modbus register area according to register map.
    // When external master trying to access the register in the area that is not initialized
    // by mbc_slave_set_descriptor() API call then Modbus stack
    // will send exception response for this register area.
    reg_area.type = MB_PARAM_HOLDING; // Set type of register area
    reg_area.start_offset = MB_REG_HOLDING_START_AREA0; // Offset of register area in Modbus protocol
    reg_area.address = (void*)&(holding_reg_params.autotrigger_interval); // Set pointer to storage instance
    reg_area.size = sizeof(holding_reg_params); // Set the size of register storage instance
    err = mbc_slave_set_descriptor(slave_handle, reg_area);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                    TAG,
                                    "mbc_slave_set_descriptor fail, returns(0x%x).",
                                    (int)err);

    // Initialization of Input Registers area
    reg_area.type = MB_PARAM_INPUT;
    reg_area.start_offset = MB_REG_INPUT_START_AREA0;
    reg_area.address = (void*)&(input_reg_params.measured_value);
    reg_area.size = sizeof(input_reg_params);
    err = mbc_slave_set_descriptor(slave_handle, reg_area);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                        TAG,
                                        "mbc_slave_set_descriptor fail, returns(0x%x).",
                                        (int)err);

    // Initialization of Coils register area
    reg_area.type = MB_PARAM_COIL;
    reg_area.start_offset = MB_REG_COILS_START;
    reg_area.address = (void*)&coil_reg_params;
    reg_area.size = sizeof(coil_reg_params);
    err = mbc_slave_set_descriptor(slave_handle, reg_area);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                    TAG,
                                    "mbc_slave_set_descriptor fail, returns(0x%x).",
                                    (int)err);

    // Initialization of Discrete Inputs register area
    reg_area.type = MB_PARAM_DISCRETE;
    reg_area.start_offset = MB_REG_DISCRETE_INPUT_START;
    reg_area.address = (void*)&discrete_reg_params;
    reg_area.size = sizeof(discrete_reg_params);
    err = mbc_slave_set_descriptor(slave_handle, reg_area);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                    TAG,
                                    "mbc_slave_set_descriptor fail, returns(0x%x).",
                                    (int)err);

    // Starts of modbus controller and stack
    err = mbc_slave_start(slave_handle);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                        TAG,
                                        "mbc_slave_start fail, returns(0x%x).",
                                        (int)err);
    vTaskDelay(5);
    return err;
}

esp_err_t slave_destroy(void)
{
    mb_event_handler_func = NULL;
    esp_err_t err = mbc_slave_delete(slave_handle);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                TAG,
                                "mbc_slave_destroy fail, returns(0x%x).",
                                (int)err);
    return err;
}
