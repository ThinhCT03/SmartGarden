#include <stdio.h>
#include <string.h>

// Thư viện FreeRTOS để tạo task, delay v.v.
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Thư viện UART
#include "driver/uart.h"

// Thư viện log để in thông tin ra serial
#include "esp_log.h"

// Thư viện WiFi và HTTP Client
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_http_client.h"

// Cấu hình UART
#define UART_NUM UART_NUM_1
#define BUF_SIZE 1024
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)

// Cấu hình WiFi và ThingSpeak
#define WIFI_SSID "Thinhct03"
#define WIFI_PASS "01012003"
#define THINGSPEAK_WRITE_KEY "Z2E9EB359WTDC1L8"
#define THINGSPEAK_CHANNEL_ID "2803741"
#define THINGSPEAK_READ_KEY "QV1TOFUY6NUS91MX"

static const char *TAG = "UART_WIFI";  // Tag để log

// Bộ đệm nhận UART
uint8_t uart_rx_buffer[BUF_SIZE];

// Biến lưu giá trị cảm biến từ STM32 gửi sang
uint8_t humidity, temperature;
uint8_t pump, led, soil, rain;

// Khai báo các hàm
void wifi_init_sta(void);
void parse_uart_data(uint8_t *data);
void send_data_to_thingspeak(void);
void check_control_from_thingspeak(void);

// Hàm chính của chương trình
void app_main(void)
{
    // Khởi tạo bộ nhớ flash NVS (non-volatile storage)
    nvs_flash_init();

    // Kết nối WiFi
    wifi_init_sta();

    // Cấu hình UART
    const uart_config_t uart_config = {
        .baud_rate = 11520,                     // Baudrate giống STM32
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // Cài đặt và gán chân cho UART
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Vòng lặp chính
    while (1) {
        // Đọc dữ liệu từ STM32 qua UART
        int len = uart_read_bytes(UART_NUM, uart_rx_buffer, 4, pdMS_TO_TICKS(2000));

        // Nếu đủ 4 byte và byte đầu là 0x02 (dấu hiệu bắt đầu frame)
        if (len == 4 && uart_rx_buffer[0] == 0x02) {
            parse_uart_data(uart_rx_buffer);       // Phân tích dữ liệu
            send_data_to_thingspeak();             // Gửi dữ liệu lên ThingSpeak
            check_control_from_thingspeak();       // Kiểm tra có lệnh điều khiển nào không
        }

        // Delay 10 giây rồi tiếp tục
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

// Hàm phân tích dữ liệu nhận từ STM32
void parse_uart_data(uint8_t *data) {
    humidity = data[1];
    temperature = data[2];

    // Tách các bit từ byte thứ 4
    pump = (data[3] >> 3) & 0x01;
    led  = (data[3] >> 2) & 0x01;
    soil = (data[3] >> 1) & 0x01;
    rain = data[3] & 0x01;

    ESP_LOGI(TAG, "Parsed -> Temp: %d, Humi: %d, Pump: %d, LED: %d, Soil: %d, Rain: %d",
             temperature, humidity, pump, led, soil, rain);
}

// Hàm gửi dữ liệu lên ThingSpeak
void send_data_to_thingspeak(void) {
    char url[256];

    // Tạo URL để gửi GET request lên ThingSpeak
    sprintf(url,
        "http://api.thingspeak.com/update?api_key=%s&field1=%d&field2=%d&field3=%d&field4=%d&field5=%d&field6=%d",
        THINGSPEAK_WRITE_KEY, temperature, humidity, pump, led, soil, rain);

    // Cấu hình client gửi HTTP GET
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    // Gửi request
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Data sent to ThingSpeak");
    } else {
        ESP_LOGE(TAG, "HTTP request failed: %s", esp_err_to_name(err));
    }

    // Dọn dẹp
    esp_http_client_cleanup(client);
}

// Hàm kiểm tra dữ liệu điều khiển từ ThingSpeak và gửi về STM32
void check_control_from_thingspeak(void) {
    char url[256];

    // Gửi request GET để đọc field 7 từ kênh
    sprintf(url,
        "http://api.thingspeak.com/channels/%s/fields/7/last.txt?api_key=%s",
        THINGSPEAK_CHANNEL_ID, THINGSPEAK_READ_KEY);

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_timeout_ms(client, 5000); // Timeout nếu mạng yếu

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        char buffer[16];
        int len = esp_http_client_read(client, buffer, sizeof(buffer) - 1);
        if (len > 0) {
            buffer[len] = '\0'; // Thêm null-terminator
            int relay_state = atoi(buffer); // Lấy giá trị relay

            ESP_LOGI(TAG, "Relay state from TS: %d", relay_state);

            // Gửi lệnh điều khiển về STM32
            uint8_t cmd_frame[2] = {0xA1, (relay_state & 0x03)}; // bit 0: relay1, bit 1: relay2
            uart_write_bytes(UART_NUM, (const char *)cmd_frame, 2);
        }
    } else {
        ESP_LOGE(TAG, "Failed to get control: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
}

// Hàm khởi tạo và kết nối WiFi
void wifi_init_sta(void)
{
    // Khởi tạo stack mạng
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta(); // Thiết lập là WiFi Station (STA mode)

    // Cấu hình mặc định
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Thiết lập SSID và Password
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    // Cài đặt và kết nối
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi started, connecting...");
    ESP_ERROR_CHECK(esp_wifi_connect());
}
