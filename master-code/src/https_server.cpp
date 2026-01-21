#include "https_server.hpp"

#include "esp_http_server.h"
#include "mdns.h"
#include "esp_https_server.h"
#include "esp_log.h"

static const char* TAG = "https_server";

// Self-signed certificate (you'll need to generate this)
extern const uint8_t server_cert_start[] asm("_binary_cacert_pem_start");
extern const uint8_t server_cert_end[] asm("_binary_cacert_pem_end");
extern const uint8_t server_key_start[] asm("_binary_prvtkey_pem_start");
extern const uint8_t server_key_end[] asm("_binary_prvtkey_pem_end");

// Simple HTML response
const char* HTML_RESPONSE = R"(
<!DOCTYPE html>
<html>
<body>Hi</body>
</html>
)";

// Handler for GET /
static esp_err_t root_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, HTML_RESPONSE, strlen(HTML_RESPONSE));
    return ESP_OK;
}

static const httpd_uri_t root = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_get_handler,
    .user_ctx  = NULL
};

void setup_http_server() {
    httpd_config_t conf = HTTPD_DEFAULT_CONFIG();
    conf.server_port = 80;
    
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &conf) == ESP_OK) {
        httpd_register_uri_handler(server, &root);
        ESP_LOGI(TAG, "HTTP server started on port 80");
    } else {
        ESP_LOGE(TAG, "Failed to start HTTP server");
    }
}

void setup_https_server() {
    httpd_ssl_config_t conf = HTTPD_SSL_CONFIG_DEFAULT();
    conf.servercert = server_cert_start;
    conf.servercert_len = server_cert_end - server_cert_start;
    conf.prvtkey_pem = server_key_start;
    conf.prvtkey_len = server_key_end - server_key_start;
    
    httpd_handle_t server = NULL;
    if (httpd_ssl_start(&server, &conf) == ESP_OK) {
        httpd_register_uri_handler(server, &root);
        ESP_LOGI(TAG, "HTTPS server started on port 443");
    } else {
        ESP_LOGE(TAG, "Failed to start HTTPS server");
    }
}

void setup_mdns() {
    mdns_init();
    mdns_hostname_set("hex-mini-drive");
    mdns_instance_name_set("Hex Mini Drive");
    
    mdns_service_add(NULL, "_https", "_tcp", 443, NULL, 0);
    
    ESP_LOGI(TAG, "mDNS configured: hex-mini-drive.local");
}
