#include <stdio.h>
#include <string.h>

#include <stdlib.h>
#include <math.h>
#include <esp_wifi.h>
#include <esp_event_loop.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>

#include <esp_http_server.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#define GPIO_PWM0A_OUT 32   //Set GPIO 15 as PWM0A Left side
#define GPIO_PWM0B_OUT 25   //Set GPIO 16 as PWM0B Right side

#define BUF_SIZE (1024)

#define GPIO_R1 21
#define GPIO_R2 12
#define GPIO_L1 15 
#define GPIO_L2 14

#define ECHO_TEST_TXD   17
#define ECHO_TEST_RXD   16
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)

#define BEACON_TEST_TXD   17 //TX pin
#define BEACON_TEST_RXD   16 //RX pin

#define BUF_SIZE (1024)

#define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
#define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD

volatile double rightduty = 100.0;
volatile double leftduty = 100.0;

static const char *TAG="APP";

static void ir_init()
{
    uart_config_t uart_config = {
        .baud_rate = 1200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, BEACON_TEST_TXD, BEACON_TEST_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_2, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_set_line_inverse(UART_NUM_2, UART_INVERSE_RXD);
}

static void mcpwm_initialize()
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, GPIO_PWM0B_OUT);

    gpio_pad_select_gpio(GPIO_R1);
    gpio_pad_select_gpio(GPIO_R2);
    gpio_pad_select_gpio(GPIO_L1);
    gpio_pad_select_gpio(GPIO_L2);

    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_R1, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_R2, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_L1, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_L2, GPIO_MODE_OUTPUT);

    gpio_set_level(GPIO_R1, 0);
    gpio_set_level(GPIO_R2, 0);
    gpio_set_level(GPIO_L1, 0);
    gpio_set_level(GPIO_L2, 0);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 500;    //frequency = 500Hz,
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);
}

static void motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, float duty_cycle)
{
    gpio_set_level(GPIO_R1, 0);
    gpio_set_level(GPIO_R2, 1);
    gpio_set_level(GPIO_L1, 1);
    gpio_set_level(GPIO_L2, 0);

    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state   
}

static void motor_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, float duty_cycle)
{
    gpio_set_level(GPIO_R1, 1);
    gpio_set_level(GPIO_R2, 0);
    gpio_set_level(GPIO_L1, 0);
    gpio_set_level(GPIO_L2, 1);

    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

static void motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
}


char start = 0x1B;
volatile int rxID = 0;
volatile char rxFragment[11] = {};

void uart_receive()
{

    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
   
    while(1){
        int length = 0;

        uart_get_buffered_data_len(UART_NUM_2,(size_t*)&length);
        int len = uart_read_bytes(UART_NUM_2, data, BUF_SIZE, 20 / portTICK_RATE_MS);

        if (len>0) 
        {
            for (int i=0; i < 24; i++) {
                if (data[i] == start) {
                    rxID = (int)data[i+1];
                    for (int j = 0; j < 11; j++){
                        rxFragment[j] = (char) data[i+j+1];
                    }
                printf("Received from device ID 0x%02X fragment: %s\n", rxID, rxFragment);
                break;
                }
            }
        }
        else{
        // printf("Nothing received.\n");
        }       
        vTaskDelay(200/portTICK_RATE_MS);
    }
}

/* An HTTP GET handler */
esp_err_t uart_get_handler(httpd_req_t *req)
{

    /* Set some custom headers */
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char* buf = malloc(100);
    sprintf(buf,"%d,%s", rxID, rxFragment);
    /* Send response with custom headers and body set as the
     * string passed in user context*/
    const char* resp_str = (const char*) buf;
    httpd_resp_send(req, resp_str, strlen(resp_str));

    /* After sending the HTTP response the old HTTP request
     * headers are lost. Check if HTTP request headers can be read now. */
    if (httpd_req_get_hdr_value_len(req, "Host") == 0) {
        ESP_LOGI(TAG, "Request headers lost");
    }
    free(buf);
    return ESP_OK;
}

httpd_uri_t uart = {
    .uri       = "/uart",
    .method    = HTTP_GET,
    .handler   = uart_get_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx  = NULL
};

/* An HTTP POST handler */
esp_err_t forward_post_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char buf[9];
    int ret, remaining = req->content_len;
    while (remaining > 0) {
        /* Read the data for the request */
        if ((ret = httpd_req_recv(req, buf,
                        MIN(remaining, sizeof(buf)))) <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                /* Retry receiving if timeout occurred */
                continue;
            }
            return ESP_FAIL;
        }
        httpd_resp_send_chunk(req, buf, ret);
        remaining -= ret;
    }

    motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, leftduty);
    motor_forward(MCPWM_UNIT_1, MCPWM_TIMER_1, rightduty);

    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

httpd_uri_t forward = {
    .uri       = "/forward",
    .method    = HTTP_POST,
    .handler   = forward_post_handler,
    .user_ctx  = NULL
};

/* An HTTP GET handler */
esp_err_t backward_post_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char buf[9];
    int ret, remaining = req->content_len;
    while (remaining > 0) {
        /* Read the data for the request */
        if ((ret = httpd_req_recv(req, buf,
                        MIN(remaining, sizeof(buf)))) <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                /* Retry receiving if timeout occurred */
                continue;
            }
            return ESP_FAIL;
        }
        httpd_resp_send_chunk(req, buf, ret);
        remaining -= ret;
    }

    motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, leftduty);
    motor_backward(MCPWM_UNIT_1, MCPWM_TIMER_1, rightduty);

    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

httpd_uri_t backward = {
    .uri       = "/backward",
    .method    = HTTP_POST,
    .handler   = backward_post_handler,
    .user_ctx  = NULL
};

esp_err_t left_post_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char buf[2];
    int ret, remaining = req->content_len;

    while (remaining > 0) {
        if ((ret = httpd_req_recv(req, buf,
                        MIN(remaining, sizeof(buf)))) <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            return ESP_FAIL;
        }

        httpd_resp_send_chunk(req, buf, ret);
        remaining -= ret;
    }

    motor_forward(MCPWM_UNIT_1, MCPWM_TIMER_1, rightduty);
    motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);

    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

httpd_uri_t left = {
    .uri       = "/left",
    .method    = HTTP_POST,
    .handler   = left_post_handler,
    .user_ctx  = NULL
};

esp_err_t right_post_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char buf[2];
    int ret, remaining = req->content_len;

    while (remaining > 0) {
        if ((ret = httpd_req_recv(req, buf,
                        MIN(remaining, sizeof(buf)))) <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            return ESP_FAIL;
        }

        httpd_resp_send_chunk(req, buf, ret);
        remaining -= ret;
    }

    motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, leftduty);
    motor_stop(MCPWM_UNIT_1, MCPWM_TIMER_1);

    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

httpd_uri_t right = {
    .uri       = "/right",
    .method    = HTTP_POST,
    .handler   = right_post_handler,
    .user_ctx  = NULL
};

esp_err_t stop_post_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char buf[2];
    int ret, remaining = req->content_len;

    while (remaining > 0) {
        /* Read the data for the request */
        if ((ret = httpd_req_recv(req, buf,
                        MIN(remaining, sizeof(buf)))) <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                /* Retry receiving if timeout occurred */
                continue;
            }
            return ESP_FAIL;
        }

        httpd_resp_send_chunk(req, buf, ret);
        remaining -= ret;
    }

    motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    motor_stop(MCPWM_UNIT_1, MCPWM_TIMER_1);

    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

httpd_uri_t stop = {
    .uri       = "/stop",
    .method    = HTTP_POST,
    .handler   = stop_post_handler,
    .user_ctx  = NULL
};

httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &forward);
        httpd_register_uri_handler(server, &backward);
        httpd_register_uri_handler(server, &left);
        httpd_register_uri_handler(server, &right);
        httpd_register_uri_handler(server, &stop);
        httpd_register_uri_handler(server, &uart);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

void stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    httpd_stop(server);
}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    httpd_handle_t *server = (httpd_handle_t *) ctx;

    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_START");
        ESP_ERROR_CHECK(esp_wifi_connect());
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP");
        ESP_LOGI(TAG, "Got IP: '%s'",
                ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));

        /* Start the web server */
        if (*server == NULL) {
            *server = start_webserver();
        }
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_DISCONNECTED");
        ESP_ERROR_CHECK(esp_wifi_connect());

        /* Stop the web server */
        if (*server) {
            stop_webserver(*server);
            *server = NULL;
        }
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void initialise_wifi(void *arg)
{
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, arg));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
        },
    };
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void app_main()
{
    static httpd_handle_t server = NULL;
    mcpwm_initialize();
    ir_init();
    ESP_ERROR_CHECK(nvs_flash_init());
    initialise_wifi(&server);
    xTaskCreate(uart_receive, "uart_receive", 4096, NULL, 4, NULL);
}
