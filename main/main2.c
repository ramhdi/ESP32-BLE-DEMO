#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#define SERVICE_1_UUID 0xFFF1
#define CHAR_1_UUID 0xFFF2
#define SERVICE_2_UUID 0xFFF3
#define CHAR_2_UUID 0xFFF4

static uint16_t char_1_handle;
static uint16_t char_2_handle;

// Global advertising parameters
static struct ble_gap_adv_params adv_params;

// Function to initialize the advertising parameters
static void configure_adv_params(void) {
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;  // Undirected advertising
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;  // General discoverability
}

void uart_init(void) {
    uart_config_t uart_config = {.baud_rate = 115200,
                                 .data_bits = UART_DATA_8_BITS,
                                 .parity = UART_PARITY_DISABLE,
                                 .stop_bits = UART_STOP_BITS_1,
                                 .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
}

static int ble_event_callback(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            ESP_LOGI("BLE", "Connection %s; status=%d",
                     event->connect.status == 0 ? "established" : "failed",
                     event->connect.status);
            if (event->connect.status != 0) {
                ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                                  &adv_params, ble_event_callback, NULL);
            }
            break;
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI("BLE", "Disconnected; reason=%d",
                     event->disconnect.reason);
            ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                              &adv_params, ble_event_callback, NULL);
            break;
        default:
            break;
    }
    return 0;
}

static int ble_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg) {
    uint8_t *data;
    size_t len;
    int rc;

    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            if (attr_handle == char_1_handle) {
                // Example: Send a static string or last received UART data
                data = (uint8_t *)"Static data for Service 1";
                len = strlen((char *)data);
                rc = os_mbuf_append(ctxt->om, data, len);
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            } else if (attr_handle == char_2_handle) {
                // Sending incremented integer in "Hello, world! <integer>"
                static uint32_t count = 0;
                char buf[64];
                sprintf(buf, "Hello, world! %ld", count++);
                data = (uint8_t *)buf;
                len = strlen(buf);
                rc = os_mbuf_append(ctxt->om, data, len);
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            break;

        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            if (attr_handle == char_1_handle) {
                // Assume the incoming data is a string, log it
                data = ctxt->om->om_data;
                len = ctxt->om->om_len;
                ESP_LOGI("BLE", "Received on Service 1: %.*s", len,
                         (char *)data);
                // Optionally, save or process data here
            }
            return 0;

        default:
            // Unsupported operation
            return BLE_ATT_ERR_UNLIKELY;
    }

    // Default case for operations that do not match or are not handled
    return BLE_ATT_ERR_UNLIKELY;
}

static void ble_services_init(void) {
    ble_svc_gap_init();
    ble_svc_gatt_init();

    const struct ble_gatt_svc_def svcs[] = {
        {
            .type = BLE_GATT_SVC_TYPE_PRIMARY,
            .uuid = BLE_UUID16_DECLARE(SERVICE_1_UUID),
            .characteristics =
                (struct ble_gatt_chr_def[]){
                    {
                        .uuid = BLE_UUID16_DECLARE(CHAR_1_UUID),
                        .access_cb =
                            ble_chr_access_cb,  // Implement this function
                        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                    },
                    {
                        0,  // end
                    },
                },
        },
        {
            .type = BLE_GATT_SVC_TYPE_PRIMARY,
            .uuid = BLE_UUID16_DECLARE(SERVICE_2_UUID),
            .characteristics =
                (struct ble_gatt_chr_def[]){
                    {
                        .uuid = BLE_UUID16_DECLARE(CHAR_2_UUID),
                        .access_cb =
                            ble_chr_access_cb,  // Implement this function
                        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                    },
                    {
                        0,  // end
                    },
                },
        },
        {
            0,  // No more services
        },
    };
    ble_gatts_count_cfg(svcs);
    ble_gatts_add_svcs(svcs);
}

static void ble_host_task(void *param) {
    // This function call initializes the NimBLE host.
    nimble_port_run();  // This will run forever unless nimble_port_stop() is
                        // called.

    // Clean up resources if nimble_port_stop() is called and the task ends.
    nimble_port_freertos_deinit();
}

void periodic_send_task(void *param) {
    uint32_t count = 0;
    char msg[64];

    while (1) {
        sprintf(msg, "Hello, world! %ld", count++);
        // Send this message over BLE using char_2_handle
        // Function to send data needs to be implemented

        vTaskDelay(pdMS_TO_TICKS(10000));  // Wait for 10 seconds
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        MODLOG_DFLT(ERROR, "Failed to init nimble %d \n", ret);
        return;
    }

    // Initialize BLE services and advertising parameters
    configure_adv_params();
    ble_hs_cfg.sync_cb = ble_services_init;

    nimble_port_freertos_init(ble_host_task);
    uart_init();  // Initialize UART for incoming serial data

    // Implement periodic sending in a separate task
    xTaskCreate(periodic_send_task, "send_hello_world", 2048, NULL, 10, NULL);
}
