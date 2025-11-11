/**
 * @file my_hal.cpp
 * @author MSU
 * @brief Hardware Abstraction Layer: GPIO pin layout, hardware timers and PWM (buzzer, LED and analog PSU soft-start),
 * ADC channels, entering and recovering from sleep mode, Shift-Register IO, CPU power management
 * @date 2022-10-21
 * 
 */

#include "my_hal.h"

#include "params.h"
#include "macros.h"

#include <esp_log.h>
#include <esp_check.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <rom/ets_sys.h>
#include <driver/ledc.h>
#include <soc/rtc.h>
#include <soc/rtc_cntl_reg.h>
#include <driver/rtc_io.h>
#include <rom/gpio.h>
#include <esp_eth.h>
#include <esp_event.h>

#include "ethernet_init.h"

static const char TAG[] = "HAL";

/**
 * Configuration Section: Pin Numbers
 * 
 */

const gpio_num_t pin_reading = GPIO_NUM_15;
const gpio_num_t pin_trigger = GPIO_NUM_14;
const gpio_num_t pin_read = GPIO_NUM_36;
const gpio_num_t pin_p1 = GPIO_NUM_17;
const gpio_num_t pin_p2 = GPIO_NUM_33;
const gpio_num_t pin_led = GPIO_NUM_5;

const gpio_num_t input_gpio[] =
{
    pin_read,
    pin_p1,
    pin_p2
};
const gpio_num_t output_gpio[] =
{
    pin_led,
    pin_trigger,
    pin_reading
};

/**
 * @brief Shift Registers
 * 
 */
struct my_sr
{
    gpio_num_t d; //Data input
    gpio_num_t clk;
    gpio_num_t latch; //SH/LD
    bool msb_first; //Bit order
    size_t len; //Bytes
};
const my_sr regs[] = 
{
    { GPIO_NUM_35, GPIO_NUM_4, pin_reading, true, 4 }, // Input register file
};
static SemaphoreHandle_t sr_mutex_handle = NULL;

// Ethernet
static uint8_t eth_port_cnt = 0;
static esp_eth_handle_t *eth_handles;
esp_netif_t **eth_netifs;
/** Event handler for Ethernet events */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    /* we can get the ethernet driver handle from event data */
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "Ethernet Link Up");
        ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Down");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}
/** Event handler for IP_EVENT_ETH_GOT_IP */
static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "~~~~~~~~~~~");
    ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(TAG, "~~~~~~~~~~~");
}

/**
 * PUBLIC routines
 * 
 */

namespace my_hal
{
    /// @brief Initialize HAL and the peripherals it contorls.
    /// @param buzzer_freq Buzzer frequency (driving square wave frequency, harmonics will be present)
    /// @param shdn_pump_freq Driving frequency for the charge pump that drives the gate of the MOSFET that enables analog PSU
    /// @return ESK_OK, or panics otherwise
    esp_err_t init()
    {
        const uint32_t zero = 0;
        const uint8_t* const zero_ptr = reinterpret_cast<const uint8_t*>(&zero);
        ESP_LOGI(TAG, "HAL initialization");

        ESP_LOGI(TAG, "Init GPIO direction...");
        //Output pins
        for (auto &&i : output_gpio)
        {
            gpio_pad_select_gpio(i);
            ESP_ERROR_CHECK(gpio_set_direction(i, GPIO_MODE_OUTPUT));
            ESP_ERROR_CHECK(gpio_set_level(i, 0));
        }
        for (auto &&i : input_gpio)
        {
            gpio_pad_select_gpio(i);
            ESP_ERROR_CHECK(gpio_set_direction(i, GPIO_MODE_INPUT));
            ESP_ERROR_CHECK(gpio_set_pull_mode(i, GPIO_FLOATING));
        }

        ESP_LOGI(TAG, "Init SRs...");
        sr_mutex_handle = xSemaphoreCreateMutex();
        assert(sr_mutex_handle);
        //Set shift register pins as outputs and load all zeros
        for (size_t i = 0; i < ARRAY_SIZE(regs); i++)
        {
            gpio_pad_select_gpio(regs[i].d);
            gpio_pad_select_gpio(regs[i].clk);
            gpio_pad_select_gpio(regs[i].latch);
            gpio_set_direction(regs[i].d, GPIO_MODE_INPUT);
            gpio_set_direction(regs[i].clk, GPIO_MODE_OUTPUT);
            gpio_set_direction(regs[i].latch, GPIO_MODE_OUTPUT);
            assert(regs[i].len <= sizeof(zero));
        }

        // Initialize Ethernet driver
        ESP_LOGI(TAG, "Init ethernet...");
        ESP_ERROR_CHECK(example_eth_init(&eth_handles, &eth_port_cnt));
        eth_netifs = new esp_netif_t* [eth_port_cnt];
        // Initialize TCP/IP network interface aka the esp-netif (should be called only once in application)
        ESP_ERROR_CHECK(esp_netif_init());
        // Create default event loop that running in background
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        esp_eth_netif_glue_handle_t eth_netif_glues[eth_port_cnt];
        // Create instance(s) of esp-netif for Ethernet(s)
        if (eth_port_cnt == 1)
        {
            // Use ESP_NETIF_DEFAULT_ETH when just one Ethernet interface is used and you don't need to modify
            // default esp-netif configuration parameters.
            esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
            eth_netifs[0] = esp_netif_new(&cfg);
            eth_netif_glues[0] = esp_eth_new_netif_glue(eth_handles[0]);
            // Attach Ethernet driver to TCP/IP stack
            ESP_ERROR_CHECK(esp_netif_attach(eth_netifs[0], eth_netif_glues[0]));
        }
        else
        {
            // Use ESP_NETIF_INHERENT_DEFAULT_ETH when multiple Ethernet interfaces are used and so you need to modify
            // esp-netif configuration parameters for each interface (name, priority, etc.).
            esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_ETH();
            esp_netif_config_t cfg_spi = {
                .base = &esp_netif_config,
                .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH};
            char if_key_str[10];
            char if_desc_str[10];
            char num_str[3];
            for (int i = 0; i < eth_port_cnt; i++)
            {
                itoa(i, num_str, 10);
                strcat(strcpy(if_key_str, "ETH_"), num_str);
                strcat(strcpy(if_desc_str, "eth"), num_str);
                esp_netif_config.if_key = if_key_str;
                esp_netif_config.if_desc = if_desc_str;
                esp_netif_config.route_prio -= i * 5;
                eth_netifs[i] = esp_netif_new(&cfg_spi);
                eth_netif_glues[i] = esp_eth_new_netif_glue(eth_handles[i]);
                // Attach Ethernet driver to TCP/IP stack
                ESP_ERROR_CHECK(esp_netif_attach(eth_netifs[i], eth_netif_glues[i]));
            }
        }
        // Register user defined event handlers
        ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));
        // Start Ethernet driver state machine
        for (int i = 0; i < eth_port_cnt; i++)
        {
            ESP_ERROR_CHECK(esp_eth_start(eth_handles[i]));
        }

        ESP_LOGI(TAG, "HAL init finished");
        return ESP_OK;
    }

    esp_netif_t* get_netif()
    {
        return eth_netifs[0];
    }

    void sr_read(sr_types t, uint8_t* contents)
    {
        const size_t byte_len = 8;
        assert(t < ARRAY_SIZE(regs));
        assert(sr_mutex_handle);
        
        while (xSemaphoreTake(sr_mutex_handle, portMAX_DELAY) != pdTRUE);

        auto sr = regs[t];
        // Shift the data out
        for (size_t i = 0; i < sr.len; i++)
        {
            for (size_t j = 0; j < byte_len; j++)
            {
                uint32_t mask = 1u << (sr.msb_first ? (byte_len - 1 - j) : j);

                //Read bit
                if (gpio_get_level(sr.d)) contents[sr.msb_first ? (sr.len - 1 - i) : i] |= mask;
                else contents[sr.msb_first ? (sr.len - 1 - i) : i] &= ~mask;
                //Emit clock pulse
                ESP_ERROR_CHECK(gpio_set_level(sr.clk, 1));
                ets_delay_us(1);
                ESP_ERROR_CHECK(gpio_set_level(sr.clk, 0));
                ets_delay_us(1);
            }
        }
        // Let go of SH/LD
        ESP_ERROR_CHECK(gpio_set_level(sr.latch, 0));

        xSemaphoreGive(sr_mutex_handle);
    }
    void set_sr_reading_in_progress(bool b)
    {
        gpio_set_level(pin_reading, b ? 1 : 0);
    }
    void set_trigger(bool b)
    {
        gpio_set_level(pin_trigger, b ? 1 : 0);
    }
}
