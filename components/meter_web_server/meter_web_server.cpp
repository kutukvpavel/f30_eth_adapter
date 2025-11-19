#include "meter_web_server.h"

#include <esp_http_server.h>
#include <esp_err.h>
#include <esp_check.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#define SSE_MAX_LINE_LEN 128

extern const char http_root_start[] asm("_binary_root_html_start");
extern const char http_root_end[] asm("_binary_root_html_end");

namespace meter_web_server
{
    struct sse_data_t
    {
        float reading;
        const char* units;
        TickType_t timestamp;
    };

    static const char TAG[] = "web_srv";
    static httpd_handle_t server = NULL;
    static volatile sse_data_t sse_data = {};
    static SemaphoreHandle_t sse_mutex = NULL;

    static esp_err_t root_get_handler(httpd_req_t *req)
    {
        const uint32_t root_len = http_root_end - http_root_start;

        ESP_LOGD(TAG, "Serve root");
        httpd_resp_set_hdr(req, "Connection", "keep-alive");
        httpd_resp_set_type(req, "text/html");
        httpd_resp_send(req, http_root_start, root_len);

        return ESP_OK;
    }

    static esp_err_t sse_handler(httpd_req_t *req)
    {
        char sse_data_buffer[SSE_MAX_LINE_LEN];
        sse_data_t thread_local_sse_data = { .timestamp = configINITIAL_TICK_COUNT };

        httpd_resp_set_type(req, "text/event-stream");
        httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
        httpd_resp_set_hdr(req, "Connection", "keep-alive");

        while (1) {
            bool update = false;
            while (xSemaphoreTake(sse_mutex, portMAX_DELAY) != pdTRUE);
            if (thread_local_sse_data.timestamp != sse_data.timestamp)
            {
                update = true;
                thread_local_sse_data.reading = sse_data.reading;
                thread_local_sse_data.units = sse_data.units;
                thread_local_sse_data.timestamp = sse_data.timestamp;
            }
            xSemaphoreGive(sse_mutex);

            int len;
            if (update)
            {
                len = snprintf(sse_data_buffer, sizeof(sse_data_buffer),
                    "event: reading\n"
                    "data: %6f\n\n"
                    "event: units\n"
                    "data: %s\n\n",
                    thread_local_sse_data.reading,
                    thread_local_sse_data.units
                );
            }
            else
            {
                len = snprintf(sse_data_buffer, sizeof(sse_data_buffer),
                    ": keep-alive\n\n"
                );
            }

            esp_err_t err;
            if ((err = httpd_resp_send_chunk(req, sse_data_buffer, len)) != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send sse data (returned %02X)", err);
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        httpd_resp_send_chunk(req, NULL, 0); // End response
        return ESP_OK;
    }

    static const httpd_uri_t root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler
    };
    static const httpd_uri_t sse = {
        .uri = "/sse",
        .method = HTTP_GET,
        .handler = sse_handler
    };

    static httpd_handle_t start_webserver()
    {
        httpd_handle_t server = NULL;
        httpd_config_t config = HTTPD_DEFAULT_CONFIG();
#if CONFIG_IDF_TARGET_LINUX
        // Setting port as 8001 when building for Linux. Port 80 can be used only by a privileged user in linux.
        // So when a unprivileged user tries to run the application, it throws bind error and the server is not started.
        // Port 8001 can be used by an unprivileged user as well. So the application will not throw bind error and the
        // server will be started.
        config.server_port = 8001;
#endif // !CONFIG_IDF_TARGET_LINUX
        config.lru_purge_enable = true;
        config.stack_size = 8128;

        // Start the httpd server
        ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
        if (httpd_start(&server, &config) == ESP_OK)
        {
            // Set URI handlers
            ESP_LOGI(TAG, "Registering URI handlers");
            httpd_register_uri_handler(server, &root);
            httpd_register_uri_handler(server, &sse); // Register SSE handler
            return server;
        }

        ESP_LOGI(TAG, "Error starting server!");
        return NULL;
    }

    void set_data(float reading, const char* units)
    {
        while (xSemaphoreTake(sse_mutex, portMAX_DELAY) != pdTRUE);
        sse_data.reading = reading;
        sse_data.units = units;
        sse_data.timestamp = xTaskGetTickCount();
        xSemaphoreGive(sse_mutex);
    }

    esp_err_t init()
    {
        sse_mutex = xSemaphoreCreateMutex();
        if (!sse_mutex) return ESP_ERR_NO_MEM;
        server = start_webserver();
        if (!server) 
        {
            vSemaphoreDelete(sse_mutex);
            return ESP_FAIL;
        }
        return ESP_OK;
    }
} // namespace meter_web_server
