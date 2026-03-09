#include "https_server.hpp"

#include "esp_http_server.h"
#include "mdns.h"
#include "esp_https_server.h"

#include "esp_log.h"
#include <vector>

static const char* TAG = "https_server";

// WebSocket state
static httpd_handle_t http_server = NULL;
static httpd_handle_t https_server = NULL;
static ws_receive_callback_t ws_receive_callback = nullptr;

struct WebSocketClient {
    int fd;
    httpd_handle_t server;
};

static std::vector<WebSocketClient> active_ws_clients;

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


// WebSocket handler
static esp_err_t ws_handler(httpd_req_t *req) {
    // Get the server that made the request.
    httpd_handle_t server = req->handle;
    
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "WebSocket handshake from fd %d", httpd_req_to_sockfd(req));
        
        // Track this connection
        int fd = httpd_req_to_sockfd(req);

        bool found = false;
        for (const auto& client : active_ws_clients) {
            if (client.fd == fd and client.server == server) {
                found = true;
                break;
            }
        }
        if (!found) {
            active_ws_clients.push_back({fd, server});
        }
        
        return ESP_OK;
    }
    
    // Handle WebSocket frame
    httpd_ws_frame_t ws_pkt = {0};
    ws_pkt.type = HTTPD_WS_TYPE_BINARY;
    
    // Get frame info
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
        return ret;
    }
    
    if (ws_pkt.len) {
        // Allocate buffer for payload
        uint8_t* buf = (uint8_t*)calloc(1, ws_pkt.len + 1);
        if (buf == NULL) {
            ESP_LOGE(TAG, "Failed to allocate memory for ws payload");
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;
        
        // Receive frame payload
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
            free(buf);
            return ret;
        }
        
        // Handle different frame types
        if (ws_pkt.type == HTTPD_WS_TYPE_BINARY) {
            ESP_LOGI(TAG, "Received binary frame, len=%d", ws_pkt.len);
            if (ws_receive_callback != nullptr) {
                ws_receive_callback(buf, ws_pkt.len);
            }
        } else if (ws_pkt.type == HTTPD_WS_TYPE_TEXT) {
            ESP_LOGI(TAG, "Received text frame, len=%d", ws_pkt.len);
            if (ws_receive_callback != nullptr) {
                ws_receive_callback(buf, ws_pkt.len);
            }
        } else if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE) {
            ESP_LOGI(TAG, "WebSocket connection closed from fd %d", httpd_req_to_sockfd(req));
            // Remove from active connections
            int fd = httpd_req_to_sockfd(req);
            httpd_handle_t server = req->handle;
            for (auto it = active_ws_clients.begin(); it != active_ws_clients.end(); ++it) {
                if (it->fd == fd and it->server == server) {
                    active_ws_clients.erase(it);
                    break;
                }
            }
        }
        
        free(buf);
    }
    
    return ESP_OK;
}

static const httpd_uri_t ws = {
    .uri       = "/ws",
    .method    = HTTP_GET,
    .handler   = ws_handler,
    .user_ctx  = NULL,
    .is_websocket = true
};

int ws_send_binary(const uint8_t* buffer, size_t size) {
    if (http_server == NULL or https_server == NULL) {
        ESP_LOGE(TAG, "WebSocket server not initialized");
        return -1;
    }
    
    if (active_ws_clients.empty()) {
        ESP_LOGW(TAG, "No active WebSocket connections");
        return 0;
    }
    
    httpd_ws_frame_t ws_pkt = {0};
    ws_pkt.payload = (uint8_t*)buffer;
    ws_pkt.len = size;
    ws_pkt.type = HTTPD_WS_TYPE_BINARY;
    
    int sent_count = 0;
    std::vector<WebSocketClient> disconnected_clients;
    
    for (const auto& client : active_ws_clients) {
        esp_err_t ret = httpd_ws_send_frame_async(client.server, client.fd, &ws_pkt);
        if (ret == ESP_OK) {
            sent_count++;
        } else {
            ESP_LOGW(TAG, "Failed to send to fd %d, error: %d", client.fd, ret);
            disconnected_clients.push_back(client);
        }
    }
    
    // Remove disconnected clients
    for (const auto& disc_client : disconnected_clients) {
        for (auto it = active_ws_clients.begin(); it != active_ws_clients.end(); ++it) {
            if (it->fd == disc_client.fd and it->server == disc_client.server) {
                active_ws_clients.erase(it);
                break;
            }
        }
    }
    
    return sent_count;
}

void setup_mdns() {
    mdns_init();
    mdns_hostname_set("hex-mini-drive");
    mdns_instance_name_set("Hex Mini Drive");
    
    mdns_service_add(NULL, "_https", "_tcp", 443, NULL, 0);
    
    ESP_LOGI(TAG, "mDNS configured: hex-mini-drive.local");
}


void setup_server(ws_receive_callback_t receive_callback) {
    ws_receive_callback = receive_callback;

    {
        httpd_config_t conf = HTTPD_DEFAULT_CONFIG();
        conf.server_port = 80;
        
        if (httpd_start(&http_server, &conf) == ESP_OK) {
            httpd_register_uri_handler(http_server, &root);
            httpd_register_uri_handler(http_server, &ws);

            ESP_LOGI(TAG, "HTTP server started on port 80");
        } else {
            ESP_LOGE(TAG, "Failed to start HTTP server");
        }
    }

    {
        httpd_ssl_config_t conf = HTTPD_SSL_CONFIG_DEFAULT();
        conf.servercert = server_cert_start;
        conf.servercert_len = server_cert_end - server_cert_start;
        conf.prvtkey_pem = server_key_start;
        conf.prvtkey_len = server_key_end - server_key_start;
        
        if (httpd_ssl_start(&https_server, &conf) == ESP_OK) {
            httpd_register_uri_handler(https_server, &root);
            ESP_LOGI(TAG, "HTTPS server started on port 443");
            
            httpd_register_uri_handler(https_server, &ws);
            ESP_LOGI(TAG, "Secure WebSocket server started on port 443, endpoint: wss://hex-mini-drive.local/ws");
        
        } else {
            ESP_LOGE(TAG, "Failed to start HTTPS server");
        }
    }

    setup_mdns();
}
