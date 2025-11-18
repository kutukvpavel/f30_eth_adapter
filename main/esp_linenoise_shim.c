#include "esp_linenoise_shim.h"

#include "private_include/esp_linenoise_private.h"

int _esp_linenoise_probe(esp_linenoise_handle_t instance)
{
    return esp_linenoise_probe(instance);
}

void _esp_linenoise_set_prompt(esp_linenoise_handle_t instance, const char* prompt)
{
    instance->config.prompt = prompt;
}