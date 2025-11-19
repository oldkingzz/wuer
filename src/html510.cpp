/**
 * @file html510.cpp
 * @brief ESP-IDF Web Server Implementation
 *
 * Converted from Arduino WiFi to ESP-IDF HTTP Server
 * Based on MEAM510 web interface by Mark Yim, UPenn
 *
 * ESP-IDF adaptation for lab4.2
 */

#include <string.h>
#include <stdlib.h>
#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "lwip/ip_addr.h"

#include "include/web.h"
#include "include/user_input.h"

static const char *TAG = "WEB_SERVER";

// WiFi credentials for Station mode (connect to router)
#define WIFI_SSID      "Verizon_VP9P6J"
#define WIFI_PASS      "nap9-poem-essay"

// Static IP configuration for Station mode
#define STATIC_IP_ADDR "192.168.0.189"
#define GATEWAY_ADDR   "192.168.0.1"
#define NETMASK_ADDR   "255.255.255.0"

// AP mode configuration (ESP32 as hotspot)
#define AP_SSID        "ESP32_Chassis"
#define AP_PASS        "12345678"      // At least 8 characters
#define AP_CHANNEL     1
#define AP_MAX_CONN    4

// AP mode IP configuration
#define AP_IP_ADDR     "192.168.4.1"
#define AP_GATEWAY     "192.168.4.1"
#define AP_NETMASK     "255.255.255.0"

// WiFi mode selection
#define USE_AP_MODE    true  // Set to true to use AP mode, false for Station mode

// Global chassis velocity values
static float g_chassis_linear_velocity = 0.0f;   // m/s
static float g_chassis_angular_velocity = 0.0f;  // rad/s

// HTTP server handle
static httpd_handle_t server = NULL;

// WiFi event group
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;
#define MAX_RETRY 10

/**
 * @brief WiFi event handler for Station mode
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry connecting to WiFi...");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "Failed to connect to WiFi");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/**
 * @brief Initialize WiFi in AP mode (ESP32 as hotspot)
 */
static esp_err_t wifi_init_ap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *netif = esp_netif_create_default_wifi_ap();

    // Configure AP IP address
    esp_netif_ip_info_t ip_info;
    memset(&ip_info, 0, sizeof(esp_netif_ip_info_t));

    IP4_ADDR(&ip_info.ip, 192, 168, 4, 1);
    IP4_ADDR(&ip_info.gw, 192, 168, 4, 1);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);

    esp_netif_dhcps_stop(netif);
    esp_netif_set_ip_info(netif, &ip_info);
    esp_netif_dhcps_start(netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {};
    memcpy(wifi_config.ap.ssid, AP_SSID, strlen(AP_SSID));
    wifi_config.ap.ssid_len = strlen(AP_SSID);
    memcpy(wifi_config.ap.password, AP_PASS, strlen(AP_PASS));
    wifi_config.ap.channel = AP_CHANNEL;
    wifi_config.ap.max_connection = AP_MAX_CONN;
    wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;

    if (strlen(AP_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    Serial.println();
    Serial.println("========================================");
    Serial.println("WiFi AP Mode Started!");
    Serial.print("SSID: ");
    Serial.println(AP_SSID);
    Serial.print("Password: ");
    Serial.println(AP_PASS);
    Serial.print("IP Address: http://");
    Serial.println(AP_IP_ADDR);
    Serial.println("========================================");
    Serial.println();

    ESP_LOGI(TAG, "WiFi AP started. SSID: %s, IP: %s", AP_SSID, AP_IP_ADDR);

    return ESP_OK;
}

/**
 * @brief Initialize WiFi with static IP (matching lab4.1.5.ino)
 */
static esp_err_t wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *netif = esp_netif_create_default_wifi_sta();

    // Configure static IP (matching lab4.1.5.ino IP assignment)
    esp_netif_dhcpc_stop(netif);

    esp_netif_ip_info_t ip_info;
    memset(&ip_info, 0, sizeof(esp_netif_ip_info_t));

    // Use ESP-IDF IP4 address macro
    IP4_ADDR(&ip_info.ip, 192, 168, 0, 189);
    IP4_ADDR(&ip_info.gw, 192, 168, 0, 1);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);

    esp_netif_set_ip_info(netif, &ip_info);

    // Set DNS servers (matching lab4.1.5.ino)
    esp_netif_dns_info_t dns_info;
    IP4_ADDR(&dns_info.ip.u_addr.ip4, 8, 8, 8, 8);
    dns_info.ip.type = ESP_IPADDR_TYPE_V4;
    esp_netif_set_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns_info);

    ESP_LOGI(TAG, "Configuring static IP: %s", STATIC_IP_ADDR);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {};
    memcpy(wifi_config.sta.ssid, WIFI_SSID, strlen(WIFI_SSID));
    memcpy(wifi_config.sta.password, WIFI_PASS, strlen(WIFI_PASS));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to %s...", WIFI_SSID);

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi connected!");
        ESP_LOGI(TAG, "ESP32 IP Address: http://%s", STATIC_IP_ADDR);
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to WiFi");
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "Unexpected WiFi event");
        return ESP_FAIL;
    }
}

/**
 * @brief HTTP GET handler for root page
 */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, WEB_PAGE_HTML, strlen(WEB_PAGE_HTML));
    return ESP_OK;
}

/**
 * @brief HTTP GET handler for setChassisVelocity endpoint
 */
static esp_err_t set_chassis_velocity_handler(httpd_req_t *req)
{
    char buf[200];
    size_t buf_len;

    // Get URL query string
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            char linear_param[32];
            char angular_param[32];

            if (httpd_query_key_value(buf, "linear", linear_param, sizeof(linear_param)) == ESP_OK &&
                httpd_query_key_value(buf, "angular", angular_param, sizeof(angular_param)) == ESP_OK) {

                g_chassis_linear_velocity = atof(linear_param);
                g_chassis_angular_velocity = atof(angular_param);

                ESP_LOGI(TAG, "Chassis velocity: linear=%.3f m/s, angular=%.3f rad/s",
                         g_chassis_linear_velocity, g_chassis_angular_velocity);
            }
        }
    }

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

/**
 * @brief HTTP GET handler for stopChassis endpoint
 */
static esp_err_t stop_chassis_handler(httpd_req_t *req)
{
    g_chassis_linear_velocity = 0.0f;
    g_chassis_angular_velocity = 0.0f;

    ESP_LOGI(TAG, "Emergency stop triggered!");

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

/**
 * @brief URI handlers
 */
static const httpd_uri_t root = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_get_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t set_chassis_velocity = {
    .uri       = "/setChassisVelocity",
    .method    = HTTP_GET,
    .handler   = set_chassis_velocity_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t stop_chassis = {
    .uri       = "/stopChassis",
    .method    = HTTP_GET,
    .handler   = stop_chassis_handler,
    .user_ctx  = NULL
};

/**
 * @brief Start HTTP server
 */
static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Starting HTTP server on port %d", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &root);
        httpd_register_uri_handler(server, &set_chassis_velocity);
        httpd_register_uri_handler(server, &stop_chassis);
        return server;
    }

    ESP_LOGE(TAG, "Failed to start HTTP server");
    return NULL;
}

/**
 * @brief Initialize web server (called from main.c)
 */
esp_err_t web_server_init(void)
{
    Serial.println("Initializing web server...");
    ESP_LOGI(TAG, "Initializing web server...");

    // Initialize NVS (required for WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize WiFi based on mode selection
    if (USE_AP_MODE) {
        Serial.println("Starting WiFi in AP mode (Hotspot)...");
        ret = wifi_init_ap();
    } else {
        Serial.println("Starting WiFi in Station mode...");
        ret = wifi_init_sta();
    }

    if (ret != ESP_OK) {
        Serial.println("ERROR: WiFi initialization failed!");
        return ret;
    }

    // Start HTTP server
    Serial.println("Starting HTTP server...");
    server = start_webserver();
    if (server == NULL) {
        Serial.println("ERROR: Failed to start HTTP server!");
        return ESP_FAIL;
    }

    Serial.println("Web server started successfully!");
    ESP_LOGI(TAG, "Web server started successfully!");
    return ESP_OK;
}

/**
 * @brief Get chassis linear velocity
 */
float web_server_get_linear_velocity(void)
{
    return g_chassis_linear_velocity;
}

/**
 * @brief Get chassis angular velocity
 */
float web_server_get_angular_velocity(void)
{
    return g_chassis_angular_velocity;
}
