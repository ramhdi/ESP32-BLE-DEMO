#include "ble_spp_server.h"
#include "console/console.h"
#include "esp_log.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

// Function prototypes
static int ble_spp_server_gap_event(struct ble_gap_event *event, void *arg);
int gatt_svr_register(void);

// Global variables
static uint8_t own_addr_type;
static uint16_t ble_spp_svc_gatt_read_val_handle;
static bool conn_handle_subs[CONFIG_BT_NIMBLE_MAX_CONNECTIONS + 1];

// Initializes the BLE storage configuration
void ble_store_config_init(void);

/**
 * Create a memory buffer from a string
 *
 * @param str Input string
 */
struct os_mbuf *create_mbuf_from_string(const char *str) {
    struct os_mbuf *om;
    size_t len;

    len = strlen(str);
    om = ble_hs_mbuf_from_flat(str, len);

    return om;
}

/**
 * Prints detailed information about a BLE connection.
 *
 * @param desc Pointer to BLE GAP connection descriptor
 */
static void ble_spp_server_print_conn_desc(struct ble_gap_conn_desc *desc) {
    MODLOG_DFLT(INFO, "handle=%d our_ota_addr_type=%d our_ota_addr=",
                desc->conn_handle, desc->our_ota_addr.type);
    print_addr(desc->our_ota_addr.val);
    MODLOG_DFLT(INFO,
                " our_id_addr_type=%d our_id_addr=", desc->our_id_addr.type);
    print_addr(desc->our_id_addr.val);
    MODLOG_DFLT(INFO, " peer_ota_addr_type=%d peer_ota_addr=",
                desc->peer_ota_addr.type);
    print_addr(desc->peer_ota_addr.val);
    MODLOG_DFLT(INFO,
                " peer_id_addr_type=%d peer_id_addr=", desc->peer_id_addr.type);
    print_addr(desc->peer_id_addr.val);
    MODLOG_DFLT(INFO,
                " conn_itvl=%d conn_latency=%d supervision_timeout=%d "
                "encrypted=%d authenticated=%d bonded=%d\n",
                desc->conn_itvl, desc->conn_latency, desc->supervision_timeout,
                desc->sec_state.encrypted, desc->sec_state.authenticated,
                desc->sec_state.bonded);
}

/**
 * Configures and starts BLE advertising.
 */
static void ble_spp_server_advertise(void) {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    // Set up advertisement data
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;  // General discoverability, BR/EDR
                                              // unsupported
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;  // Auto TX power level
    fields.name = (uint8_t *)ble_svc_gap_device_name();
    fields.name_len = strlen((char *)fields.name);
    fields.name_is_complete = 1;
    fields.uuids16 = (ble_uuid16_t[]){BLE_UUID16_INIT(BLE_SVC_SPP_UUID16)};
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    // Set advertisement data
    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error setting advertisement data; rc=%d\n", rc);
        return;
    }

    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode =
        BLE_GAP_CONN_MODE_UND;  // Undirected connectable mode
    adv_params.disc_mode =
        BLE_GAP_DISC_MODE_GEN;  // General discoverability mode
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params,
                           ble_spp_server_gap_event, NULL);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", rc);
    }
}

/**
 * Callback function for handling GAP events in a NimBLE-based SPP server.
 * This function processes various GAP events and takes appropriate actions
 * like starting advertising or updating connection parameters.
 *
 * @param event The GAP event.
 * @param arg   Pointer to user-defined data or NULL. Unused in this
 * application.
 * @return      0 on success, non-zero error code on failure.
 */
static int ble_spp_server_gap_event(struct ble_gap_event *event, void *arg) {
    struct ble_gap_conn_desc desc;  // Connection descriptor
    int rc;                         // Return code

    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            // Log the status of the connection attempt
            MODLOG_DFLT(INFO, "Connection %s; status=%d ",
                        (event->connect.status == 0) ? "established" : "failed",
                        event->connect.status);

            // If connection was successful, print connection details
            if (event->connect.status == 0) {
                rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
                assert(rc == 0);  // Ensure no errors finding the connection
                ble_spp_server_print_conn_desc(&desc);
            }
            MODLOG_DFLT(INFO, "\n");

            // Resume advertising if connection failed or multiple connections
            // are supported
            if (event->connect.status != 0 ||
                CONFIG_BT_NIMBLE_MAX_CONNECTIONS > 1) {
                ble_spp_server_advertise();
            }
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
            // Log the disconnection reason
            MODLOG_DFLT(INFO, "Disconnect; reason=%d ",
                        event->disconnect.reason);
            ble_spp_server_print_conn_desc(&event->disconnect.conn);
            MODLOG_DFLT(INFO, "\n");

            // Mark connection handle as unsubscribed
            conn_handle_subs[event->disconnect.conn.conn_handle] = false;

            // Resume advertising after disconnection
            ble_spp_server_advertise();
            return 0;

        case BLE_GAP_EVENT_CONN_UPDATE:
            // Connection parameters have been updated
            MODLOG_DFLT(INFO, "Connection updated; status=%d ",
                        event->conn_update.status);
            rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
            assert(rc == 0);
            ble_spp_server_print_conn_desc(&desc);
            MODLOG_DFLT(INFO, "\n");
            return 0;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            // Advertising has completed
            MODLOG_DFLT(INFO, "Advertise complete; reason=%d",
                        event->adv_complete.reason);
            ble_spp_server_advertise();
            return 0;

        case BLE_GAP_EVENT_MTU:
            // MTU size has been updated
            MODLOG_DFLT(INFO,
                        "MTU update event; conn_handle=%d cid=%d mtu=%d\n",
                        event->mtu.conn_handle, event->mtu.channel_id,
                        event->mtu.value);
            return 0;

        case BLE_GAP_EVENT_SUBSCRIBE:
            // A new subscription has been made
            MODLOG_DFLT(
                INFO,
                "Subscribe event; conn_handle=%d attr_handle=%d reason=%d "
                "prevn=%d curn=%d previ=%d curi=%d\n",
                event->subscribe.conn_handle, event->subscribe.attr_handle,
                event->subscribe.reason, event->subscribe.prev_notify,
                event->subscribe.cur_notify, event->subscribe.prev_indicate,
                event->subscribe.cur_indicate);
            // Mark the connection handle as subscribed
            conn_handle_subs[event->subscribe.conn_handle] = true;
            return 0;

        default:
            return 0;  // Unhandled events return 0
    }
}

/**
 * Handles reset events in the BLE host task.
 *
 * @param reason The reason code for the reset.
 */
static void ble_spp_server_on_reset(int reason) {
    MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

/**
 * Callback function when BLE stack is synchronized.
 * Initializes the device's advertising setup.
 */
static void ble_spp_server_on_sync(void) {
    int rc;
    uint8_t addr_val[6] = {0};

    // Ensure the BLE address is set correctly
    rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

    // Determine own address type (public or random)
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error determining address type; rc=%d\n", rc);
        return;
    }

    // Copy and print the BLE device address
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);
    MODLOG_DFLT(INFO, "Device Address: ");
    print_addr(addr_val);
    MODLOG_DFLT(INFO, "\n");

    // Begin advertising
    ble_spp_server_advertise();
}

/**
 * Task function for the BLE host, executed in a FreeRTOS task.
 *
 * @param param User-defined parameter, not used here.
 */
void ble_spp_server_host_task(void *param) {
    MODLOG_DFLT(INFO, "BLE Host Task Started");
    nimble_port_run();  // Runs the NimBLE host stack

    // Deinitialize NimBLE port when the host task stops
    nimble_port_freertos_deinit();
}

/**
 * GATT service callback to handle read/write operations on characteristics.
 *
 * @param conn_handle Connection handle
 * @param attr_handle Attribute handle
 * @param ctxt Context providing access to GATT operations and data
 * @param arg User-defined argument, unused here
 * @return 0 on successful handling, error code otherwise
 */
static int ble_svc_gatt_handler(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg) {
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            MODLOG_DFLT(INFO, "Callback for read");
            break;

        case BLE_GATT_ACCESS_OP_WRITE_CHR: {
            // Log the basic event info
            MODLOG_DFLT(
                INFO,
                "Data received in write event, conn_handle=%x, attr_handle=%x",
                conn_handle, attr_handle);

            // Access the data written to the characteristic
            uint8_t *data = ctxt->om->om_data;  // Pointer to the data buffer
            uint16_t len = ctxt->om->om_len;    // Length of the data

            // Assuming the data is UTF-8 encoded, null-terminate it for safe
            // printing. Make sure there is enough space or reduce length to
            // avoid buffer overflow.
            char buf[len + 1];
            memcpy(buf, data, len);
            buf[len] = '\0';  // Null-terminate the string

            // Log the received string
            MODLOG_DFLT(INFO, "Received data: %s", buf);
        } break;

        default:
            MODLOG_DFLT(INFO, "Default Callback");
            break;
    }
    return 0;
}

/**
 * Define the custom GATT service and its characteristics.
 */
static const struct ble_gatt_svc_def new_ble_svc_gatt_defs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_SPP_UUID16),
        .characteristics =
            (struct ble_gatt_chr_def[]){
                {
                    .uuid = BLE_UUID16_DECLARE(BLE_SVC_SPP_CHR_UUID16),
                    .access_cb = ble_svc_gatt_handler,
                    .val_handle = &ble_spp_svc_gatt_read_val_handle,
                    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                             BLE_GATT_CHR_F_NOTIFY,
                },
                {0},  // Terminating entry
            },
    },
    {0},  // Terminating entry for services
};

/**
 * Callback for GATT registration events, logs the details of registered
 * components.
 *
 * @param ctxt Context containing registration details
 * @param arg User-defined argument, unused here
 */
static void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt,
                                 void *arg) {
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
        case BLE_GATT_REGISTER_OP_SVC:
            MODLOG_DFLT(DEBUG, "registered service %s with handle=%d",
                        ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                        ctxt->svc.handle);
            break;
        case BLE_GATT_REGISTER_OP_CHR:
            MODLOG_DFLT(DEBUG,
                        "registering characteristic %s with def_handle=%d "
                        "val_handle=%d",
                        ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                        ctxt->chr.def_handle, ctxt->chr.val_handle);
            break;
        case BLE_GATT_REGISTER_OP_DSC:
            MODLOG_DFLT(DEBUG, "registering descriptor %s with handle=%d",
                        ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                        ctxt->dsc.handle);
            break;
        default:
            assert(0);
            break;
    }
}

/**
 * Initializes the GATT server with custom service definitions.
 *
 * @return int 0 if successful, error code otherwise.
 */
int gatt_svr_init(void) {
    int rc = 0;

    // Initialize GAP and GATT services
    ble_svc_gap_init();
    ble_svc_gatt_init();

    // Count and configure the GATT services
    rc = ble_gatts_count_cfg(new_ble_svc_gatt_defs);
    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(new_ble_svc_gatt_defs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}

/**
 * Task for handling UART events and sending notifications over BLE.
 *
 * @param pvParameters not used
 */
void ble_server_hello_task(void *pvParameters) {
    static uint32_t count = 0;
    char buf[64];

    MODLOG_DFLT(INFO, "BLE server Hello_task started\n");

    for (;;) {
        // Format the string with the increasing integer
        snprintf(buf, sizeof(buf), "Hello, world! %ld\n", count++);

        // Iterate over possible connections to send notifications
        for (int i = 0; i <= CONFIG_BT_NIMBLE_MAX_CONNECTIONS; i++) {
            if (conn_handle_subs[i]) {
                struct os_mbuf *txom =
                    create_mbuf_from_string((const char *)(buf));
                if (txom == NULL) {
                    MODLOG_DFLT(ERROR, "Failed to create mbuf from string\n");
                    continue;
                }

                int rc = ble_gatts_notify_custom(
                    i, ble_spp_svc_gatt_read_val_handle, txom);
                if (rc == 0) {
                    MODLOG_DFLT(INFO, "Notification sent successfully");
                } else {
                    MODLOG_DFLT(INFO, "Error in sending notification rc = %d",
                                rc);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10000));  // Wait for 10 seconds
    }

    vTaskDelete(NULL);
}

/**
 * Main application entry point. Initializes NVS, BLE stack, UART, and starts
 * the main BLE host task.
 */
void app_main(void) {
    int rc;

    // Initialize NVS for storing PHY calibration data
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // Erase if new NVS version is found or no free pages
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize NimBLE port
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        MODLOG_DFLT(ERROR, "Failed to init nimble %d \n", ret);
        return;
    }

    // Initialize connection handles array to false for all possible connections
    for (int i = 0; i <= CONFIG_BT_NIMBLE_MAX_CONNECTIONS; i++) {
        conn_handle_subs[i] = false;
    }

    // Create a FreeRTOS task for the Hello task
    xTaskCreate(ble_server_hello_task, "uTask", 4096, NULL, 8, NULL);

    // Configure BLE host stack callbacks
    ble_hs_cfg.reset_cb = ble_spp_server_on_reset;  // Callback for stack reset
    ble_hs_cfg.sync_cb = ble_spp_server_on_sync;    // Sync callback
    ble_hs_cfg.gatts_register_cb =
        gatt_svr_register_cb;  // GATT registration callback
    ble_hs_cfg.store_status_cb =
        ble_store_util_status_rr;  // Storage status callback

    // Configure security manager (SM) settings based on project configuration
    ble_hs_cfg.sm_io_cap = CONFIG_EXAMPLE_IO_TYPE;  // I/O capabilities
#ifdef CONFIG_EXAMPLE_BONDING
    ble_hs_cfg.sm_bonding = 1;  // Enable bonding
#endif
#ifdef CONFIG_EXAMPLE_MITM
    ble_hs_cfg.sm_mitm = 1;  // Enable Man-In-The-Middle protection
#endif
#ifdef CONFIG_EXAMPLE_USE_SC
    ble_hs_cfg.sm_sc = 1;  // Enable Secure Connections
#else
    ble_hs_cfg.sm_sc = 0;  // Disable Secure Connections
#endif
#ifdef CONFIG_EXAMPLE_BONDING
    ble_hs_cfg.sm_our_key_dist = 1;  // Key distribution
    ble_hs_cfg.sm_their_key_dist = 1;
#endif

    // Register custom GATT services
    rc = gatt_svr_init();
    assert(rc == 0);  // Ensure GATT server initialized successfully

    // Set the default BLE device name
    rc = ble_svc_gap_device_name_set("nimble-ble-spp-svr");
    assert(rc == 0);  // Ensure the device name was set successfully

    // Initialize BLE store configuration
    ble_store_config_init();

    // Start the NimBLE port with the main BLE server task
    nimble_port_freertos_init(ble_spp_server_host_task);
}
