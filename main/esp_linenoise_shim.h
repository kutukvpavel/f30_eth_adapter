#pragma once

#include "esp_linenoise.h"

#ifdef __cplusplus
extern "C" {
#endif

int _esp_linenoise_probe(esp_linenoise_handle_t instance);
void _esp_linenoise_set_prompt(esp_linenoise_handle_t instance, const char* prompt);

#ifdef __cplusplus
}
#endif