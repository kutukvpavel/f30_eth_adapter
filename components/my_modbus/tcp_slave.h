#pragma once

#include <esp_err.h>
#include "mbcontroller.h"       // for mbcontroller defines and api
#include "modbus_params.h"      // for modbus parameters structures

#define MB_TCP_PORT_NUMBER      (CONFIG_FMB_TCP_PORT_DEFAULT)
#define MB_MDNS_PORT            (502)
#define MB_SLAVE_ADDR (CONFIG_MB_SLAVE_ADDR)

#define MB_READ_MASK                        (MB_EVENT_INPUT_REG_RD \
                                                | MB_EVENT_HOLDING_REG_RD \
                                                | MB_EVENT_DISCRETE_RD \
                                                | MB_EVENT_COILS_RD)
#define MB_WRITE_MASK                       (MB_EVENT_HOLDING_REG_WR \
                                                | MB_EVENT_COILS_WR)
#define MB_READ_WRITE_MASK                  (MB_READ_MASK | MB_WRITE_MASK)

#ifdef __cplusplus
extern "C" {
#endif

void slave_operation_func(void *arg);

esp_err_t init_services(void);
esp_err_t destroy_services(void);
esp_err_t slave_init(mb_communication_info_t* comm_info, void (*event_handler_func)(const mb_param_info_t*), void** handle);

#ifdef __cplusplus
}
#endif