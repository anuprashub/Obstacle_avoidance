#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "esp_http_server.h"

#include "driver/gpio.h"
#include "esp_rom_sys.h"

#include "esp_camera.h"
#include "lwip/sockets.h"

#include "motors.h"

static const char *TAG = "ROBOT_CAM";

/*==================== WIFI ====================*/

#define WIFI_SSID      "username" // WIFI NAME
#define WIFI_PASS      "password" // WIFI PASSWORD

static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int s_retry_num = 0;

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 5) {
            esp_wifi_connect();
            s_retry_num++;
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any;
    esp_event_handler_instance_t instance_got_ip;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID,
        &wifi_event_handler, NULL, &instance_any));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP,
        &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(
        wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, pdFALSE,
        portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to WiFi");
    } else {
        ESP_LOGE(TAG, "Failed to connect to WiFi");
    }
}

/*==================== CAMERA ====================*/

#define CAM_PIN_PWDN   -1
#define CAM_PIN_RESET  -1
#define CAM_PIN_XCLK   14
#define CAM_PIN_SIOD   4
#define CAM_PIN_SIOC   5

#define CAM_PIN_D7     16
#define CAM_PIN_D6     17
#define CAM_PIN_D5     18
#define CAM_PIN_D4     12
#define CAM_PIN_D3     10
#define CAM_PIN_D2     8
#define CAM_PIN_D1     9
#define CAM_PIN_D0     11

#define CAM_PIN_VSYNC  6
#define CAM_PIN_HREF   7
#define CAM_PIN_PCLK   13

static void camera_init(void)
{
    camera_config_t config = {
        .pin_pwdn  = CAM_PIN_PWDN,
        .pin_reset = CAM_PIN_RESET,
        .pin_xclk  = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,

        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,

        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href  = CAM_PIN_HREF,
        .pin_pclk  = CAM_PIN_PCLK,

        .xclk_freq_hz = 20000000,
        .ledc_channel = LEDC_CHANNEL_0,
        .ledc_timer   = LEDC_TIMER_0,

        .pixel_format = PIXFORMAT_JPEG,
        .frame_size   = FRAMESIZE_QVGA,
        .jpeg_quality = 12,
        .fb_count     = 2
    };

    ESP_ERROR_CHECK(esp_camera_init(&config));
    ESP_LOGI(TAG, "Camera initialized");
}

/*==================== HTTP STREAM ====================*/

httpd_handle_t stream_httpd = NULL;

static esp_err_t stream_handler(httpd_req_t *req)
{
    esp_err_t res = ESP_OK;
    camera_fb_t *fb = NULL;

    static const char* boundary = "--frame";
    static const char* part = "Content-Type: image/jpeg\r\n\r\n";

    httpd_resp_set_type(req, "multipart/x-mixed-replace; boundary=frame");

    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGW(TAG, "Camera capture failed");
            continue;
        }

        httpd_resp_send_chunk(req, boundary, strlen(boundary));
        httpd_resp_send_chunk(req, "\r\n", 2);
        httpd_resp_send_chunk(req, part, strlen(part));
        httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
        httpd_resp_send_chunk(req, "\r\n", 2);

        esp_camera_fb_return(fb);
    }
    return res;
}

static void start_stream_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 81;

    httpd_uri_t uri_stream = {
        .uri     = "/stream",
        .method  = HTTP_GET,
        .handler = stream_handler
    };

    httpd_start(&stream_httpd, &config);
    httpd_register_uri_handler(stream_httpd, &uri_stream);

    ESP_LOGI(TAG, "Camera stream ready at http://<ESP_IP>:81/stream");
}

/*==================== ULTRASONIC SENSORS ====================*/
/* Pins must match your wiring */

#define TRIG_FL  19
#define ECHO_FL  20
#define TRIG_FR  21
#define ECHO_FR  45

// Simple blocking distance read with timeout
static float read_distance_cm(int trig, int echo)
{
    const int timeout_us = 30000; // 30ms

    // Ensure trig is low
    gpio_set_level(trig, 0);
    esp_rom_delay_us(2);

    // 10us high pulse
    gpio_set_level(trig, 1);
    esp_rom_delay_us(10);
    gpio_set_level(trig, 0);

    // wait for echo HIGH (with timeout)
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(echo) == 0) {
        if (esp_timer_get_time() - start > timeout_us) {
            return -1.0f; // timeout
        }
    }

    int64_t echo_start = esp_timer_get_time();

    // wait for echo LOW
    while (gpio_get_level(echo) == 1) {
        if (esp_timer_get_time() - echo_start > timeout_us) {
            return -1.0f;
        }
    }

    int64_t echo_end = esp_timer_get_time();

    float time_us = (float)(echo_end - echo_start);
    // sound speed ~343 m/s => 0.0343 cm/us; divide by 2 for round trip
    float dist = (time_us * 0.0343f) / 2.0f;
    return dist;
}

/*==================== UDP COMMAND RECEIVER ====================*/

typedef enum {
    REGION_NONE = 0,
    REGION_LEFT,
    REGION_CENTER,
    REGION_RIGHT
} obstacle_region_t;

static volatile obstacle_region_t g_region = REGION_NONE;

#define UDP_PORT 5005

static void udp_server_task(void *pv)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create UDP socket");
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "Socket bind failed");
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UDP server listening on port %d", UDP_PORT);

    char rx[64];

    while (1) {
        int len = recv(sock, rx, sizeof(rx)-1, 0);
        if (len <= 0) continue;

        rx[len] = 0;  // null terminate

        // Parse simple text commands from PC
        if (strstr(rx, "LEFT")) {
            g_region = REGION_LEFT;
        } else if (strstr(rx, "RIGHT")) {
            g_region = REGION_RIGHT;
        } else if (strstr(rx, "CENTER")) {
            g_region = REGION_CENTER;
        } else if (strstr(rx, "NONE")) {
            g_region = REGION_NONE;
        }

        ESP_LOGI(TAG, "Received: %s -> region=%d", rx, (int)g_region);
    }
}

/*==================== CONTROL TASK ====================*/

static void control_task(void *pv)
{
    motor_set_speed(180);

    // Configure ultrasonic pins
    gpio_set_direction(TRIG_FL, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_FL, GPIO_MODE_INPUT);
    gpio_set_direction(TRIG_FR, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_FR, GPIO_MODE_INPUT);

    while (1) {
        float d_fl = read_distance_cm(TRIG_FL, ECHO_FL);
        float d_fr = read_distance_cm(TRIG_FR, ECHO_FR);

        ESP_LOGI(TAG, "Distances: FL=%.1f cm  FR=%.1f cm", d_fl, d_fr);

        const float SAFE_DIST = 25.0f;

        obstacle_region_t region = g_region;

        if (region == REGION_NONE) {
            // No obstacle detected by camera; rely on ultrasonics
            if ((d_fl < 0 || d_fl > SAFE_DIST) &&
                (d_fr < 0 || d_fr > SAFE_DIST)) {
                motor_forward();
            } else if (d_fl < d_fr) {
                // left side closer -> steer right
                motor_right();
            } else {
                motor_left();
            }
        }
        else if (region == REGION_LEFT) {
            // Obstacle on left -> steer right
            if (d_fr > SAFE_DIST) {
                motor_right();
            } else {
                motor_stop();
            }
        }
        else if (region == REGION_RIGHT) {
            // Obstacle on right -> steer left
            if (d_fl > SAFE_DIST) {
                motor_left();
            } else {
                motor_stop();
            }
        }
        else if (region == REGION_CENTER) {
            // Obstacle in the center; choose side with more free space
            if (d_fl < 0 && d_fr < 0) {
                motor_stop();
            } else if (d_fl > d_fr) {
                motor_left();
            } else {
                motor_right();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/*==================== app_main ====================*/

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    wifi_init_sta();

    camera_init();
    start_stream_server();

    motors_init();
    motor_stop();

    xTaskCreate(udp_server_task, "udp_server", 4096, NULL, 5, NULL);
    xTaskCreate(control_task,   "control",   4096, NULL, 5, NULL);
}
