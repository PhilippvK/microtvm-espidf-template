#include "transport_wifi.h"

#include <string.h>
#include <unistd.h>

// #include "esp_event.h"
#include <tvm/runtime/crt/logging.h>
#include <tvm/runtime/crt/microtvm_rpc_server.h>
#include <tvm/runtime/crt/page_allocator.h>
#include <tvm/runtime/crt/graph_executor_module.h>

#include "sdkconfig.h"
#include "transport_state.h"
#include "common.h"
#include "esp_log.h"

#include "esp_netif.h"

#include "esp_wifi.h"

#include "freertos/FreeRTOS.h"

#include "freertos/event_groups.h"
#include "freertos/ringbuf.h"

#include <lwip/sockets.h>

#include <lwip/netdb.h>

static const char* TAG = "microtvm_wifi";

static EventGroupHandle_t wifi_event_group;

#define WIFI_CONNECTED_BIT BIT0

static int listen_socket = -1;

static int client_socket = -1;

static void wifi_event_handler(

    void* arg,

    esp_event_base_t event_base,

    int32_t event_id,

    void* event_data)

{

    if (event_base == WIFI_EVENT &&

        event_id == WIFI_EVENT_STA_START)

    {

        esp_wifi_connect();

    }

    else if (event_base == WIFI_EVENT &&

             event_id == WIFI_EVENT_STA_DISCONNECTED)

    {

        esp_wifi_connect();

        xEventGroupClearBits(

            wifi_event_group,

            WIFI_CONNECTED_BIT);

    }

    else if (event_base == IP_EVENT &&

             event_id == IP_EVENT_STA_GOT_IP)

    {

        xEventGroupSetBits(

            wifi_event_group,

            WIFI_CONNECTED_BIT);

    }

}

static void wifi_init_sta(void)

{

    wifi_event_group =

        xEventGroupCreate();

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg =

        WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(

        esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(

        esp_event_handler_register(

            WIFI_EVENT,

            ESP_EVENT_ANY_ID,

            wifi_event_handler,

            NULL));

    ESP_ERROR_CHECK(

        esp_event_handler_register(

            IP_EVENT,

            IP_EVENT_STA_GOT_IP,

            wifi_event_handler,

            NULL));

    wifi_config_t wifi_config = {};

    strcpy(

        (char*)wifi_config.sta.ssid,

        CONFIG_MICROTVM_WIFI_SSID);

    strcpy(

        (char*)wifi_config.sta.password,

        CONFIG_MICROTVM_WIFI_PASSWORD);

    ESP_ERROR_CHECK(

        esp_wifi_set_mode(WIFI_MODE_STA));

    ESP_ERROR_CHECK(

        esp_wifi_set_config(

            WIFI_IF_STA,

            &wifi_config));

    ESP_ERROR_CHECK(

        esp_wifi_start());

    xEventGroupWaitBits(

        wifi_event_group,

        WIFI_CONNECTED_BIT,

        pdFALSE,

        pdTRUE,

        portMAX_DELAY);

}

static void wifi_init_ap(void)
{

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg =

        WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(

        esp_wifi_init(&cfg));

    wifi_config_t ap_config = {};

    strcpy(

        (char*)ap_config.ap.ssid,

        CONFIG_MICROTVM_WIFI_SSID);

    strcpy(

        (char*)ap_config.ap.password,

        CONFIG_MICROTVM_WIFI_PASSWORD);

    ap_config.ap.max_connection = 1;

    ap_config.ap.authmode =

        WIFI_AUTH_WPA2_PSK;

    if (strlen(

        CONFIG_MICROTVM_WIFI_PASSWORD) == 0)

    {

        ap_config.ap.authmode =

            WIFI_AUTH_OPEN;

    }

    ESP_ERROR_CHECK(

        esp_wifi_set_mode(WIFI_MODE_AP));

    ESP_ERROR_CHECK(

        esp_wifi_set_config(

            WIFI_IF_AP,

            &ap_config));

    ESP_ERROR_CHECK(

        esp_wifi_start());

}

static void socket_server_start(void)

{

    struct sockaddr_in addr = {};

    addr.sin_family = AF_INET;

    addr.sin_port =

        htons(CONFIG_MICROTVM_WIFI_PORT);

    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    listen_socket =

        socket(AF_INET, SOCK_STREAM, IPPROTO_IP);

    ESP_ERROR_CHECK(

        listen_socket < 0 ? ESP_FAIL : ESP_OK);

    int opt = 1;

    setsockopt(

        listen_socket,

        SOL_SOCKET,

        SO_REUSEADDR,

        &opt,

        sizeof(opt));

    bind(

        listen_socket,

        (struct sockaddr*)&addr,

        sizeof(addr));

    listen(

        listen_socket,

        1);

    ESP_LOGI(

        TAG,

        "Waiting for MicroTVM connection...");

}

static void wait_for_client(void)

{

    struct sockaddr_storage source_addr;

    socklen_t addr_len =

        sizeof(source_addr);

    client_socket =

        accept(

            listen_socket,

            (struct sockaddr*)&source_addr,

            &addr_len);

    ESP_LOGI(

        TAG,

        "Client connected");

}

static RingbufHandle_t s_ringbuf = NULL;

static void wifi_rx_task(void* pvParameters)

{

    uint8_t temp_buffer[1460];
    bool client_connected = false;

    for (;;)

    {
        wait_for_client();
        client_connected = true;

        while (client_connected)
        {
            int n = recv(client_socket, temp_buffer, sizeof(temp_buffer), 0);
            if (n < 0)
            {
                TVMLogf("socket recv error");
                break;
            }
            if (n == 0)
            {
                TVMLogf("socket disconnected");
                client_connected = false;
                break;
            }
            uint8_t* data;
            BaseType_t res = xRingbufferSendAcquire(s_ringbuf, (void**)&data, n, pdMS_TO_TICKS(10000));
            if (res != pdTRUE)
            {
                TVMLogf("Failed to acquire memory");
                continue;
            }
            memcpy(data, temp_buffer, n);
            g_num_bytes_in_rx_buffer += n;
            if (g_num_bytes_in_rx_buffer > RING_BUF_SIZE_BYTES)
            {
                TVMPlatformAbort(
                    (tvm_crt_error_t)0xbeef3);
            }
            res = xRingbufferSendComplete(s_ringbuf, data);
            if (res != pdTRUE)
            {
                TVMLogf("Failed to send item");
            }
        }
    }

    vTaskDelete(NULL);

}


void transport_wifi_init(RingbufHandle_t ringbuf)
{

    s_ringbuf = ringbuf;
    printf("A\n");

    ESP_ERROR_CHECK(
        esp_netif_init());
    printf("B\n");

    ESP_ERROR_CHECK(
        esp_event_loop_create_default());
    printf("C\n");

#ifdef CONFIG_MICROTVM_WIFI_STA
    wifi_init_sta();
#endif

#ifdef CONFIG_MICROTVM_WIFI_AP
    wifi_init_ap();
#endif
    printf("D\n");

    socket_server_start();

    printf("E\n");

    // wait_for_client();
    xTaskCreate(wifi_rx_task, "wifi_rx_task", 2048, NULL, 12, NULL);

}

size_t transport_wifi_read(

    uint8_t* buffer,

    size_t max_len)

{

    if (client_socket < 0)

        return 0;

    int n =

        recv(

            client_socket,

            buffer,

            max_len,

            MSG_DONTWAIT);

    if (n <= 0)

        return 0;

    return (size_t)n;

}


ssize_t transport_wifi_write(

    const uint8_t* data,

    size_t len)

{

    if (client_socket < 0)

        return -1;

    return send(

        client_socket,

        data,

        len,

        0);

}
