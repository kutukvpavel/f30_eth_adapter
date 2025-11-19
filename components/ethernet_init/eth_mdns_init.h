#pragma once

#include <inttypes.h>
#include <stddef.h>

#define MDNS_MAX_HOSTNAME_LEN 32

_BEGIN_STD_C

void mdns_start_service(const char* hostname, const char* default_instance_name);

void mdns_stop_service(void);

void mdns_register_modbus(int port, uint32_t slave_id);

void mdns_register_console(int port);

void mdns_register_echo(int port);

_END_STD_C