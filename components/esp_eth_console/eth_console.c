/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "eth_console.h"

#include <string.h>
#include <sys/param.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_check.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include <stdbool.h>

#define KEEPALIVE_IDLE              CONFIG_EXAMPLE_KEEPALIVE_IDLE
#define KEEPALIVE_INTERVAL          CONFIG_EXAMPLE_KEEPALIVE_INTERVAL
#define KEEPALIVE_COUNT             CONFIG_EXAMPLE_KEEPALIVE_COUNT

static const char *TAG = "eth_serial";
RingbufHandle_t eth_console_ringbuffer_rx = NULL;
RingbufHandle_t eth_console_ringbuffer_tx = NULL;

struct server_port;
typedef void (*sock_handler_t)(int, struct server_port*);

#define BUFF_SZ 1024

struct server_port {
    uint16_t       port;
    sock_handler_t handler;
    char           buff[BUFF_SZ];
};

static void do_echo(int sock, struct server_port* srv)
{
    for (;;)
    {
        int len = recv(sock, srv->buff, BUFF_SZ, 0);
        if (len > 0) {
            // send() can return less bytes than supplied length.
            // Walk-around for robust implementation.
            char* ptr = srv->buff;
            while (len) {
                int const written = send(sock, ptr, len, 0);
                if (written < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    // Failed to retransmit, giving up
                    break;
                }
                len -= written;
                ptr += written;
            }
        }
    }
}

static void do_console(int sock, struct server_port* srv)
{
    ESP_ERROR_CHECK(fcntl(sock, F_SETFL, fcntl(sock, F_GETFL, 0) | O_NONBLOCK));
    for (;;) {
        bool idle = true;
        // Read Console
        for (;;) {
            size_t size = 0;
            const uint8_t* data = (uint8_t*)xRingbufferReceive(eth_console_ringbuffer_tx, &size, idle ? 0 : 1);
            if ((size <= 0) || (!data))
                break;
            ESP_LOGD(TAG, "Console -> Eth  %d bytes", size);
            char* ptr = srv->buff;
            memcpy(ptr, data, size);
            vRingbufferReturnItem(eth_console_ringbuffer_tx, data);
            while (size > 0) {
                int const written = send(sock, ptr, size, 0);
                if (written < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
                size -= written;
                ptr  += written;
            }
            idle = false;
        }
        // Read Eth
        int const rx_len = recv(sock, srv->buff, BUFF_SZ, 0);
        if (rx_len > 0) {
            ESP_LOGD(TAG, "Eth  -> Console %d bytes", rx_len);
            xRingbufferSend(eth_console_ringbuffer_rx, srv->buff, rx_len, portMAX_DELAY);
            idle = false;
        }
        if (idle)
            vTaskDelay(1);
    }
}

static void tcp_server_task(void *pvParameters)
{
    char addr_str[128];
    struct server_port* srv = pvParameters;
    int ip_protocol = 0;
    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;
    struct sockaddr_storage dest_addr;

    struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
    dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr_ip4->sin_family = AF_INET;
    dest_addr_ip4->sin_port = htons(srv->port);
    ip_protocol = IPPROTO_IP;

    int listen_sock = socket(AF_INET, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", srv->port);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    while (1) {

        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        // Set tcp keepalive option
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
        // Convert ip address to string
        if (source_addr.ss_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }
        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

        (srv->handler)(sock, srv);

        shutdown(sock, 0);
        close(sock);
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}

static struct server_port echo_server   = {.port = CONFIG_ECHO_PORT,   .handler = do_echo};
static struct server_port console_server = {.port = CONFIG_CONSOLE_PORT, .handler = do_console};

esp_err_t esp_eth_console_create(RingbufHandle_t* rx, RingbufHandle_t* tx)
{
    static bool initialized = false;

    if (initialized) return ESP_ERR_INVALID_STATE;

    eth_console_ringbuffer_rx = xRingbufferCreate(BUFF_SZ, RINGBUF_TYPE_NOSPLIT);
    if (!eth_console_ringbuffer_rx) return ESP_ERR_NO_MEM;
    eth_console_ringbuffer_tx = xRingbufferCreate(BUFF_SZ, RINGBUF_TYPE_NOSPLIT);
    if (!eth_console_ringbuffer_tx) return ESP_ERR_NO_MEM;
    if (xTaskCreate(tcp_server_task, "echo_server",   4096, (void*)&echo_server,   5, NULL) != pdTRUE) return ESP_ERR_NO_MEM;
    if (xTaskCreate(tcp_server_task, "console_server", 4096, (void*)&console_server, 5, NULL) != pdTRUE) return ESP_ERR_NO_MEM;
    *rx = eth_console_ringbuffer_rx;
    *tx = eth_console_ringbuffer_tx;

    initialized = true;
    return ESP_OK;
}