#include "eth_mdns_init.h"

#include "sdkconfig.h"
#include "mdns.h"
#include "esp_mac.h"
#include "esp_log.h"

#include "macros.h"

#define MB_ID_BYTE0(id) ((uint8_t)(id))
#define MB_ID_BYTE1(id) ((uint8_t)(((uint16_t)(id) >> 8) & 0xFF))
#define MB_ID_BYTE2(id) ((uint8_t)(((uint32_t)(id) >> 16) & 0xFF))
#define MB_ID_BYTE3(id) ((uint8_t)(((uint32_t)(id) >> 24) & 0xFF))

#define MB_ID2STR(id) MB_ID_BYTE0(id), MB_ID_BYTE1(id), MB_ID_BYTE2(id), MB_ID_BYTE3(id)

static const char *TAG = "mdns";
static const char* mdns_hostname = NULL;

// convert mac from binary format to string
static inline char* gen_mac_str(const uint8_t* mac, char* pref, char* mac_str)
{
    sprintf(mac_str, "%s%02X%02X%02X%02X%02X%02X", pref, MAC2STR(mac));
    return mac_str;
}

static inline char* gen_id_str(char* service_name, char* slave_id_str, uint32_t id)
{
    sprintf(slave_id_str, "%s%02X%02X%02X%02X", service_name, MB_ID2STR(id));
    return slave_id_str;
}

void mdns_start_service(const char* hostname, const char* default_instance_name)
{
    //initialize mDNS
    ESP_ERROR_CHECK(mdns_init());
    //set mDNS hostname (required if you want to advertise services)
    ESP_ERROR_CHECK(mdns_hostname_set(hostname));
    mdns_hostname = hostname;
    ESP_LOGI(TAG, "mdns hostname set to: [%s]", hostname);
    //set default mDNS instance name
    ESP_ERROR_CHECK(mdns_instance_name_set(default_instance_name));
}

void mdns_stop_service(void)
{
    mdns_free();
}

void mdns_register_modbus(int port, uint32_t slave_id)
{
    char mac_str[32];
    char mb_id_str[32];
    uint8_t eth_mac[6] = {0};
    ESP_ERROR_CHECK(esp_read_mac(eth_mac, ESP_MAC_ETH));
    gen_mac_str(eth_mac, "\0", mac_str);
    gen_id_str("\0", mb_id_str, slave_id);
    mdns_txt_item_t serviceTxtData[] = {
        { "device", "" },
        { "mac", mac_str },
        { "mb_id", mb_id_str }
    };
    //initialize service
    ESP_ERROR_CHECK(mdns_service_add(mdns_hostname, "_modbus", "_tcp", port, serviceTxtData, ARRAY_SIZE(serviceTxtData)));
}

void mdns_register_console(int port)
{
    mdns_txt_item_t serviceTxtData[] = {
        { "tty", "linenoise" }
    };
    //initialize service
    ESP_ERROR_CHECK(mdns_service_add(mdns_hostname, "_tty", "_tcp", port, serviceTxtData, ARRAY_SIZE(serviceTxtData)));
}

void mdns_register_echo(int port)
{
    mdns_txt_item_t serviceTxtData[] = {
        { "tty", "echo" }
    };
    //initialize service
    ESP_ERROR_CHECK(mdns_service_add(mdns_hostname, "_echo", "_tcp", port, serviceTxtData, ARRAY_SIZE(serviceTxtData)));
}