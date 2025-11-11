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
#include "mdns.h"
#include "esp_netif.h"
#include "esp_mac.h"

// Defines below are used to define register start address for each type of Modbus registers
#define HOLD_OFFSET(field) ((uint16_t)(offsetof(holding_reg_params_t, field) >> 1))
#define INPUT_OFFSET(field) ((uint16_t)(offsetof(input_reg_params_t, field) >> 1))
#define MB_REG_DISCRETE_INPUT_START         (0x0000)
#define MB_REG_COILS_START                  (0x0000)
#define MB_REG_INPUT_START_AREA0            (INPUT_OFFSET(measured_value)) // register offset input area 0
#define MB_REG_HOLDING_START_AREA0          (HOLD_OFFSET(test_regs))

#define MB_PAR_INFO_GET_TOUT                (10) // Timeout for get parameter info

static const char *TAG = "mb_tcp_slave";

#if CONFIG_MB_MDNS_IP_RESOLVER

#define MB_ID_BYTE0(id) ((uint8_t)(id))
#define MB_ID_BYTE1(id) ((uint8_t)(((uint16_t)(id) >> 8) & 0xFF))
#define MB_ID_BYTE2(id) ((uint8_t)(((uint32_t)(id) >> 16) & 0xFF))
#define MB_ID_BYTE3(id) ((uint8_t)(((uint32_t)(id) >> 24) & 0xFF))

#define MB_ID2STR(id) MB_ID_BYTE0(id), MB_ID_BYTE1(id), MB_ID_BYTE2(id), MB_ID_BYTE3(id)

#if CONFIG_FMB_CONTROLLER_SLAVE_ID_SUPPORT
#define MB_DEVICE_ID (uint32_t)CONFIG_FMB_CONTROLLER_SLAVE_ID
#endif

#define MB_MDNS_INSTANCE(pref) pref"mb_slave_tcp"

static void (*mb_event_handler_func)(const mb_param_info_t* reg_info) = NULL;
static void *slave_handle = NULL;

// convert mac from binary format to string
static inline char* gen_mac_str(const uint8_t* mac, char* pref, char* mac_str)
{
    sprintf(mac_str, "%s%02X%02X%02X%02X%02X%02X", pref, MAC2STR(mac));
    return mac_str;
}

static inline char* gen_id_str(char* service_name, char* slave_id_str)
{
    sprintf(slave_id_str, "%s%02X%02X%02X%02X", service_name, MB_ID2STR(MB_DEVICE_ID));
    return slave_id_str;
}

static inline char* gen_host_name_str(const char* service_name, char* name)
{
    sprintf(name, "%s_%02X", service_name, MB_SLAVE_ADDR);
    return name;
}

static void start_mdns_service(const char* hostname_pref)
{
    char temp_str[32] = {0};
    uint8_t sta_mac[6] = {0};
    ESP_ERROR_CHECK(esp_read_mac(sta_mac, ESP_MAC_WIFI_STA));
    char* hostname = gen_host_name_str(hostname_pref, temp_str);
    //initialize mDNS
    ESP_ERROR_CHECK(mdns_init());
    //set mDNS hostname (required if you want to advertise services)
    ESP_ERROR_CHECK(mdns_hostname_set(hostname));
    ESP_LOGI(TAG, "mdns hostname set to: [%s]", hostname);

    //set default mDNS instance name
    ESP_ERROR_CHECK(mdns_instance_name_set(MB_MDNS_INSTANCE("esp32_")));

    //structure with TXT records
    mdns_txt_item_t serviceTxtData[] = {
        {"device",""}
    };

    //initialize service
    ESP_ERROR_CHECK(mdns_service_add(hostname, "_modbus", "_tcp", MB_MDNS_PORT, serviceTxtData, 1));
    //add mac key string text item
    ESP_ERROR_CHECK(mdns_service_txt_item_set("_modbus", "_tcp", "mac", gen_mac_str(sta_mac, "\0", temp_str)));
    //add slave id key txt item
    ESP_ERROR_CHECK( mdns_service_txt_item_set("_modbus", "_tcp", "mb_id", gen_id_str("\0", temp_str)));
}

static void stop_mdns_service(void)
{
    mdns_free();
}

#endif

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

esp_err_t init_services(void)
{
#if CONFIG_MB_MDNS_IP_RESOLVER
    // Start mdns service and register device
    start_mdns_service(CONFIG_MB_MDNS_NAME);
#endif
    return ESP_OK;
}

esp_err_t destroy_services(void)
{
#if CONFIG_MB_MDNS_IP_RESOLVER
    stop_mdns_service();
#endif
    return ESP_OK;
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

static esp_err_t slave_destroy(void)
{
    mb_event_handler_func = NULL;
    esp_err_t err = mbc_slave_delete(slave_handle);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                TAG,
                                "mbc_slave_destroy fail, returns(0x%x).",
                                (int)err);
    return err;
}
