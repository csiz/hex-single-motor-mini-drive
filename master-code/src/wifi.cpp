#include "wifi.hpp"


#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "wifi_provisioning/manager.h"

#include "wifi_provisioning/scheme_ble.h"
#include "wifi_provisioning/scheme_softap.h"

#include "qrcode.h"

static const char *TAG = "app";

#include "esp_srp.h"

// The provisioning system uses a username and password, for which we
// need to generate a salt and verifier.
const char *username = "example_user";
const char *password = "example_password";

char * salt = nullptr;
char * verifier = nullptr;
int verifier_len = 0;
int salt_len = 16; // Standard salt length


/* Signal Wi-Fi events on this event-group */
void (*on_connected)() = nullptr;

#define PROV_QR_VERSION         "v1"
#define PROV_TRANSPORT_SOFTAP   "softap"
#define PROV_TRANSPORT_BLE      "ble"
#define QRCODE_BASE_URL         "https://espressif.github.io/esp-jumpstart/qrcode.html"

static void wifi_prov_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data){
    switch (event_id) {
        case WIFI_PROV_START:
            ESP_LOGI(TAG, "Provisioning started");
            break;
        case WIFI_PROV_CRED_RECV: {
            wifi_sta_config_t *wifi_sta_cfg = (wifi_sta_config_t *)event_data;
            ESP_LOGI(TAG, "Received Wi-Fi credentials"
                        "\n\tSSID     : %s\n\tPassword : %s",
                        (const char *) wifi_sta_cfg->ssid,
                        (const char *) wifi_sta_cfg->password);
            break;
        }
        case WIFI_PROV_CRED_FAIL: {
            wifi_prov_sta_fail_reason_t *reason = (wifi_prov_sta_fail_reason_t *)event_data;
            ESP_LOGE(TAG, "Provisioning failed!\n\tReason : %s"
                        "\n\tPlease reset to factory and retry provisioning",
                        (*reason == WIFI_PROV_STA_AUTH_ERROR) ?
                        "Wi-Fi station authentication failed" : "Wi-Fi access-point not found");

            // Reset the state machine on provisioning failure.
            wifi_prov_mgr_reset_sm_state_on_failure();
            break;
        }
        case WIFI_PROV_CRED_SUCCESS:
            ESP_LOGI(TAG, "Provisioning successful");
            break;
        case WIFI_PROV_END:
            /* De-initialize manager once provisioning is finished */
            wifi_prov_mgr_deinit();

            // IMPORTANT: Free the memory after use.
            free(salt);
            free(verifier);
            break;
        default:
            break;
    }
}

static void wifi_status_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data){
    switch (event_id) {
        case WIFI_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGI(TAG, "Disconnected. Connecting to the AP again...");
            esp_wifi_connect();
            break;
        case WIFI_EVENT_AP_STACONNECTED:
            ESP_LOGI(TAG, "SoftAP transport: Connected!");
            break;
        case WIFI_EVENT_AP_STADISCONNECTED:
            ESP_LOGI(TAG, "SoftAP transport: Disconnected!");
            break;
        default:
            break;
    }
}

static void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data){
    if (event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Connected with IP Address:" IPSTR, IP2STR(&event->ip_info.ip));
        /* Signal main application to continue execution */
        if (on_connected) on_connected();
    }
}

static void protocomm_ble_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data){
    switch (event_id) {
        case PROTOCOMM_TRANSPORT_BLE_CONNECTED:
            ESP_LOGI(TAG, "BLE transport: Connected!");
            break;
        case PROTOCOMM_TRANSPORT_BLE_DISCONNECTED:
            ESP_LOGI(TAG, "BLE transport: Disconnected!");
            break;
        default:
            break;
    }
}

/* Event handler for catching system events */
static void protocomm_sec_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data){
    switch (event_id) {
        case PROTOCOMM_SECURITY_SESSION_SETUP_OK:
            ESP_LOGI(TAG, "Secured session established!");
            break;
        case PROTOCOMM_SECURITY_SESSION_INVALID_SECURITY_PARAMS:
            ESP_LOGE(TAG, "Received invalid security parameters for establishing secure session!");
            break;
        case PROTOCOMM_SECURITY_SESSION_CREDENTIALS_MISMATCH:
            ESP_LOGE(TAG, "Received incorrect username and/or PoP for establishing secure session!");
            break;
        default:
            break;
    }
}


static void wifi_init_sta(void)
{
    /* Start Wi-Fi in station mode */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void get_device_service_name(char *service_name, size_t max)
{
    uint8_t eth_mac[6];
    const char *ssid_prefix = "EXAMPLE_";
    esp_wifi_get_mac(WIFI_IF_STA, eth_mac);
    snprintf(service_name, max, "%s%02X%02X%02X",
             ssid_prefix, eth_mac[3], eth_mac[4], eth_mac[5]);
}


static void wifi_prov_print_qr(const char *name, const char *username, const char *pop, const char *transport)
{
    if (!name || !transport) {
        ESP_LOGW(TAG, "Cannot generate QR code payload. Data missing.");
        return;
    }
    char payload[150] = {0};
    if (pop) {
        snprintf(payload, sizeof(payload), "{\"ver\":\"%s\",\"name\":\"%s\"" \
                    ",\"username\":\"%s\",\"pop\":\"%s\",\"transport\":\"%s\"}",
                    PROV_QR_VERSION, name, username, pop, transport);
    } else {
        snprintf(payload, sizeof(payload), "{\"ver\":\"%s\",\"name\":\"%s\"" \
                    ",\"transport\":\"%s\"}",
                    PROV_QR_VERSION, name, transport);
    }

    ESP_LOGI(TAG, "Scan this QR code from the provisioning application for Provisioning.");
    esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();
    esp_qrcode_generate(&cfg, payload);
    ESP_LOGI(TAG, "If QR code is not visible, copy paste the below URL in a browser.\n%s?data=%s", QRCODE_BASE_URL, payload);
}


void start_wifi_provisioning(void (*on_connected_callback)()) {
    on_connected = on_connected_callback;

    // Initialize TCP/IP
    ESP_ERROR_CHECK(esp_netif_init());

    // Initialize the event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Register our event handler for Wi-Fi, IP and Provisioning related events
    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &wifi_prov_event_handler, NULL));

    ESP_ERROR_CHECK(esp_event_handler_register(
        PROTOCOMM_TRANSPORT_BLE_EVENT, ESP_EVENT_ANY_ID, &protocomm_ble_event_handler, NULL));

    ESP_ERROR_CHECK(esp_event_handler_register(
        PROTOCOMM_SECURITY_SESSION_EVENT, ESP_EVENT_ANY_ID, &protocomm_sec_event_handler, NULL));

    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));

    // Initialize Wi-Fi including netif with default config
    esp_netif_create_default_wifi_sta();

    // TODO: Uncomment for softAP mode?
    // esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Configuration for the provisioning manager
    wifi_prov_mgr_config_t config = {
        .scheme = wifi_prov_scheme_ble,
        .scheme_event_handler = WIFI_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM,
        .app_event_handler = WIFI_PROV_EVENT_HANDLER_NONE,
        .wifi_prov_conn_cfg = {
            .wifi_conn_attempts =  3,
        },
    };

    // Initialize WiFi provisioning manager so we can log onto the local wifi network
    // when the user activates it via an app.
    ESP_ERROR_CHECK(wifi_prov_mgr_init(config));

    bool provisioned = false;

    // Let's find out if the device is provisioned
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

    // If device is not yet provisioned start provisioning service
    if (!provisioned) {
        ESP_LOGI(TAG, "Starting provisioning");

        // What is the Device Service Name that we want
        // This translates to :
        //     - Wi-Fi SSID when scheme is wifi_prov_scheme_softap
        //     - device name when scheme is wifi_prov_scheme_ble
        char service_name[16];
        get_device_service_name(service_name, sizeof(service_name));

        wifi_prov_security_t security = WIFI_PROV_SECURITY_2;

        // This pop field represents the password that will be used to generate salt and verifier.
        // The field is present here in order to generate the QR code containing password.
        // In production this password field shall not be stored on the device
        const char *pop = password;

        ESP_ERROR_CHECK(esp_srp_gen_salt_verifier(
            username, strlen(username), 
            password, strlen(password), 
            &salt, salt_len, 
            &verifier, &verifier_len
        ));

        // This is the structure for passing security parameters
        // for the protocomm security 2.
        // If dynamically allocated, sec2_params pointer and its content
        // must be valid till WIFI_PROV_END event is triggered.
        wifi_prov_security2_params_t sec2_params = {
            .salt = salt,
            .salt_len = static_cast<uint16_t>(salt_len),
            .verifier = verifier,
            .verifier_len = static_cast<uint16_t>(verifier_len),
        };


        wifi_prov_security2_params_t *sec_params = &sec2_params;

        // What is the service key (could be NULL)
        // This translates to :
        //     - Wi-Fi password when scheme is wifi_prov_scheme_softap
        //          (Minimum expected length: 8, maximum 64 for WPA2-PSK)
        //     - simply ignored when scheme is wifi_prov_scheme_ble
        const char *service_key = NULL;


        // This step is only useful when scheme is wifi_prov_scheme_ble. This will
        // set a custom 128 bit UUID which will be included in the BLE advertisement
        // and will correspond to the primary GATT service that provides provisioning
        // endpoints as GATT characteristics. Each GATT characteristic will be
        // formed using the primary service UUID as base, with different auto assigned
        // 12th and 13th bytes (assume counting starts from 0th byte). The client side
        // applications must identify the endpoints by reading the User Characteristic
        // Description descriptor (0x2901) for each characteristic, which contains the
        // endpoint name of the characteristic */
        uint8_t custom_service_uuid[] = {
            /* LSB <---------------------------------------
             * ---------------------------------------> MSB */
            0xb4, 0xdf, 0x5a, 0x1c, 0x3f, 0x6b, 0xf4, 0xbf,
            0xea, 0x4a, 0x82, 0x03, 0x04, 0x90, 0x1a, 0x02,
        };


        wifi_prov_scheme_ble_set_service_uuid(custom_service_uuid);

        /* Start provisioning service */
        ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(security, (const void *) sec_params, service_name, service_key));

        /* Print QR code for provisioning */
        wifi_prov_print_qr(service_name, username, pop, PROV_TRANSPORT_BLE);

    } else {
        ESP_LOGI(TAG, "Already provisioned, starting Wi-Fi STA");

        /* We don't need the manager as device is already provisioned,
         * so let's release it's resources */
        wifi_prov_mgr_deinit();

        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_status_event_handler, NULL));

        /* Start Wi-Fi station */
        wifi_init_sta();
    }

    // Do not wait for connection so we do not block the watchdog.
}