#include "esp_http_server.h"
#include "mdns.h"
#include "https_server.hpp"
#include "esp_log.h"

static const char* TAG = "https_server";

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

void setup_https_server() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &root);
        ESP_LOGI(TAG, "HTTP server started on port 80");
    }
}

void setup_mdns() {
    mdns_init();
    mdns_hostname_set("hex-mini-drive");
    mdns_instance_name_set("Hex Mini Drive");
    
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    
    ESP_LOGI(TAG, "mDNS configured: hex-mini-drive.local");
}
