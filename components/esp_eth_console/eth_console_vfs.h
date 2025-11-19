#pragma once

#include "esp_err.h"
#include "esp_vfs_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"

#ifdef __cplusplus

namespace eth_console_vfs
{
    /**
     * @brief Redirect output to the USB serial
     * @param cdc_intf - interface number of TinyUSB's CDC
     *
     * @return esp_err_t - ESP_OK, ESP_FAIL or an error code
     */
    esp_err_t init_console();

    /**
     * @brief Switch log to the default output
     * @param cdc_intf - interface number of TinyUSB's CDC
     *
     * @return esp_err_t
     */
    esp_err_t deinit_console();

    /**
     * @brief Set the line endings to sent
     *
     * This specifies the conversion between newlines ('\n', LF) on stdout and line
     * endings sent:
     *
     * - ESP_LINE_ENDINGS_CRLF: convert LF to CRLF
     * - ESP_LINE_ENDINGS_CR: convert LF to CR
     * - ESP_LINE_ENDINGS_LF: no modification
     *
     * @param[in] mode line endings to send
     */
    void set_tx_line_endings(esp_line_endings_t mode);

    /**
     * @brief Set the line endings expected to be received
     *
     * This specifies the conversion between line endings received and
     * newlines ('\n', LF) passed into stdin:
     *
     * - ESP_LINE_ENDINGS_CRLF: convert CRLF to LF
     * - ESP_LINE_ENDINGS_CR: convert CR to LF
     * - ESP_LINE_ENDINGS_LF: no modification
     *
     * @param[in] mode line endings expected
     */
    void set_rx_line_endings(esp_line_endings_t mode);

    void discard_input_buffer();

    int vprintf(const char* fmt, va_list args);

    esp_err_t redirect_std_streams();
}

#endif