#include "https_server.hpp"
#include "utils.hpp"

#include <esp_http_server.h>
#include <mdns.h>
#include <esp_https_server.h>
#include <esp_log.h>

#include <vector>

static const char* TAG = "https_server";

static const int64_t ACQUIRED_TIMEOUT_MS = 5000;

static const size_t MAX_BUFFER_SIZE = 16384;

static uint8_t receive_buffer[MAX_BUFFER_SIZE] = {0};

static uint8_t send_buffer[2][MAX_BUFFER_SIZE] = {0};

static size_t active_send_buffer_index = 0;

static size_t active_send_size = 0;

static inline size_t next_buffer_index() {
  return (active_send_buffer_index + 1) % 2;
}

// WebSocket state
static httpd_handle_t http_server = nullptr;
static httpd_handle_t https_server = nullptr;
static ws_receive_callback_t ws_receive_callback = nullptr;

struct WebSocketClient {
  int fd;
  httpd_handle_t server;
};

static WebSocketClient active_ws_client = {-1, nullptr};
static int64_t last_ws_message_time = 0;

static volatile bool sending_done = true;
static volatile bool sending_error = false;

// Callback for async send completion
static void ws_send_complete_callback(esp_err_t err, int socket, void *arg) {
  sending_done = true;
  if (err != ESP_OK) {
    sending_error = true;
    ESP_LOGW(TAG, "Async send failed for fd %d, error: %d", socket, err);
  }
}


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
  const httpd_handle_t server = req->handle;
  
  const auto current_time_ms = get_ms_time();

  // Track this connection
  int fd = httpd_req_to_sockfd(req);

  const bool same_client = (fd == active_ws_client.fd && server == active_ws_client.server);

  //  Get request means a new connection.
  if (req->method == HTTP_GET) {
    ESP_LOGI(TAG, "WebSocket handshake from fd %d", httpd_req_to_sockfd(req));
    
    // If we have an active client, check if it's within the timeout window.
    if (active_ws_client.fd != -1 and not same_client) {
      if (current_time_ms - last_ws_message_time < ACQUIRED_TIMEOUT_MS) {
        ESP_LOGW(TAG, "Rejecting new WebSocket connection from fd %d, active client fd %d within timeout window", fd, active_ws_client.fd);
        return ESP_ERR_HTTPD_RESP_SEND;
      } else {
        ESP_LOGI(TAG, "Previous WebSocket client fd %d timed out, accepting new connection from fd %d", active_ws_client.fd, fd);
      }
    }
    
    // Accept the connection and set as active client.
    active_ws_client.fd = fd;
    active_ws_client.server = server;
    last_ws_message_time = current_time_ms;

    return ESP_OK;
  }

  if (not same_client) {
    ESP_LOGW(TAG, "Received WebSocket message from fd %d but active client is fd %d, ignoring message", fd, active_ws_client.fd);
    return ESP_OK; // Ignore messages from non-active clients
  }
  
  // Handle WebSocket frame
  httpd_ws_frame_t ws_pkt = {0};
  
  ws_pkt.payload = receive_buffer;
  
  // Receive frame payload
  const auto ret = httpd_ws_recv_frame(req, &ws_pkt, MAX_BUFFER_SIZE);
  if (ret == ESP_ERR_INVALID_SIZE) {
    return ret; // Frame too large, ignore
  } else if (ret != ESP_OK) {
    ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
    return ret;
  }

  if (ws_pkt.len == 0) {
    ESP_LOGW(TAG, "Received empty WebSocket frame from fd %d", fd);
    return ESP_OK; // Ignore empty frames
  }

  last_ws_message_time = current_time_ms;
  
  
  // Handle different frame types
  if (ws_pkt.type == HTTPD_WS_TYPE_BINARY) {
    ESP_LOGI(TAG, "Received binary frame, len=%d", ws_pkt.len);
    if (ws_receive_callback != nullptr) {
      ws_receive_callback(ws_pkt.payload, ws_pkt.len);
    }
  } else if (ws_pkt.type == HTTPD_WS_TYPE_TEXT) {
    ESP_LOGI(TAG, "Received text frame, len=%d", ws_pkt.len);
    if (ws_receive_callback != nullptr) {
      ws_receive_callback(ws_pkt.payload, ws_pkt.len);
    }
  } else if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE) {
    ESP_LOGI(TAG, "WebSocket connection closed from fd %d", httpd_req_to_sockfd(req));
    // Remove from active connections
    active_ws_client.fd = -1;
    active_ws_client.server = nullptr;
  } else {
    ESP_LOGW(TAG, "Received unsupported WebSocket frame type %d from fd %d", ws_pkt.type, fd);
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

bool update_server(int64_t current_time_ms, std::function<size_t(uint8_t * buffer, size_t max_size)> write_func) {
  if (http_server == NULL or https_server == NULL) {
    ESP_LOGW(TAG, "WebSocket server not initialized");
    return false;
  }
  
  if (sending_error or active_ws_client.fd == -1 or active_ws_client.server == nullptr) {
    active_ws_client.fd = -1;
    active_ws_client.server = nullptr;
    sending_error = false;
    return false;
  }

  const size_t data_written = write_func(send_buffer[active_send_buffer_index] + active_send_size, MAX_BUFFER_SIZE - active_send_size);
  active_send_size += data_written;

  if (not sending_done) {
    // Previous send is still in progress, but we've queued up new data.
    return true;
  }

  // Now the previous send is done, we can send the active buffer and swap to write to the other buffer.

  if (active_send_size == 0) {
    // Nothing to send
    return true;
  }

  httpd_ws_frame_t ws_pkt = {0};
  ws_pkt.payload = send_buffer[active_send_buffer_index];
  ws_pkt.len = active_send_size;
  ws_pkt.type = HTTPD_WS_TYPE_BINARY;

  active_send_buffer_index = next_buffer_index();
  active_send_size = 0;
  sending_done = false;

  esp_err_t ret = httpd_ws_send_data_async(active_ws_client.server, active_ws_client.fd, &ws_pkt, ws_send_complete_callback, nullptr);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to queue async send to fd %d, error: %d", active_ws_client.fd, ret);
    active_ws_client.fd = -1; // Mark client as inactive on failure
    active_ws_client.server = nullptr;
    sending_done = true; // Reset sending state
    sending_error = false; // Reset error state
    return false;
  }

  return true;
}

void setup_mdns() {
  mdns_init();
  mdns_hostname_set("hex-mini-drive");
  mdns_instance_name_set("Hex Mini Drive");
  
  mdns_service_add(NULL, "_https", "_tcp", 443, NULL, 0);
  
  ESP_LOGI(TAG, "mDNS configured: hex-mini-drive.local");
}


void setup_server(int core_id, ws_receive_callback_t receive_callback) {
  ws_receive_callback = receive_callback;

  {
    httpd_config_t conf = HTTPD_DEFAULT_CONFIG();
    conf.server_port = 80;
    conf.core_id = core_id;
    
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
    conf.httpd.core_id = core_id;
    
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
