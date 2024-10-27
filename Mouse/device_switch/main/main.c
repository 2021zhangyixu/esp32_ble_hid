
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"

#include "hid_dev.h"
#include "driver/uart.h"

#define HID_DEMO_TAG                "HID_DEMO"
#define HIDD_DEVICE_NAME            "HID"         // HID device name
#define EX_UART_NUM                 UART_NUM_0    // UART number
#define BUF_SIZE                    (128)         // UART buffer size
#define RD_BUF_SIZE                 (128)         // UART read buffer size


static QueueHandle_t uart0_queue;                 // This queue is used to receive UART events
static uint16_t hid_conn_id[2] = {0};             // HID connection ID
static uint8_t remote_addr_num = 0;               // Number of stored remote devices
static uint8_t remote_conn_id = 0;                // Connection ID of the remote device to operate
static bool sec_conn = false, adv_status = false; // Secure connection flag, Advertising status flag

static uint8_t hidd_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};

static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,                            // Not a scan response packet
    .include_name = true,                             // Include device name
    .include_txpower = true,                          // Include transmit power
    .min_interval = 0x0006,                           // Slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010,                           // Slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = ESP_BLE_APPEARANCE_HID_MOUSE,       // HID MOUSE appearance
    .manufacturer_len = 0,                            // No manufacturer data
    .p_manufacturer_data = NULL,
    .service_data_len = 0,                            // No service data
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128), // Service UUID length
    .p_service_uuid = hidd_service_uuid128,
    .flag = 0x6                                       // BLE flags
};

static esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min        = 0x20,                              // Minimum advertising interval = 0x20 * 0.625ms = 20ms
    .adv_int_max        = 0x30,                              // Maximum advertising interval = 0x30 * 0.625ms = 30ms
    .adv_type           = ADV_TYPE_IND,                      // General advertising
    .own_addr_type      = BLE_ADDR_TYPE_RANDOM,              // Random static address
    .channel_map        = ADV_CHNL_ALL,                      // Use channels 37, 38, and 39
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY  // Allow scan and connection
};

static void hid_control(uint8_t *value)
{
    if (adv_status) {
        switch (*value) {
            case '8': { // Press the button 8 to start broadcasting
                ESP_LOGI(HID_DEMO_TAG,"esp_ble_gap_start_advertising ADV_TYPE_IND");
                esp_ble_gap_start_advertising(&hidd_adv_params);
                break;
            }
            case '9': { // Press the button 9 Stop the broadcast and switch the device you want to control
                esp_ble_gap_stop_advertising();
                if (remote_conn_id < 1) {
                    remote_conn_id ++;
                } else {
                    remote_conn_id = 0;
                }
                ESP_LOGI(HID_DEMO_TAG,"remote_conn_id : %d",remote_conn_id);
                break;
            }
        }
    }
}

static void send_mouse_value(uint32_t *value)
{
    // Key values for arrow keys on the keyboard
    #define KEY_UP                  0x415B1B   // Up Arrow Key
    #define KEY_DOWN                0x425B1B   // Down Arrow Key
    #define KEY_LEFT                0x445B1B   // Left Arrow Key
    #define KEY_RIGHT               0x435B1B   // Right Arrow Key

    uint8_t mouse_button = 0;  // Mouse button state, set to 0 as default (no button pressed)
    char x = 0, y = 0;         // X and Y values for mouse movement

    ESP_LOGI(HID_DEMO_TAG, "send_mouse_value hid_conn_id[%d] : %d", remote_conn_id, hid_conn_id[remote_conn_id]);

    // Check if the connection is secure
    if (sec_conn) {
        switch (*value) {
            case KEY_RIGHT: x = 5;  goto send_value;  // Move right (positive X)
            case KEY_LEFT:  x = -5; goto send_value;  // Move left (negative X)
            case KEY_UP:    y = -5; goto send_value;  // Move up (negative Y)
            case KEY_DOWN:  y = 5;  goto send_value;  // Move down (positive Y)
send_value:
            // Send the mouse value with updated coordinates
            hidd_send_mouse_value(hid_conn_id[remote_conn_id], mouse_button, x, y);
            break;

            default: break;
        }
    }
}


static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    // Allocate RD_BUF_SIZE bytes of memory from the heap for storing received UART data
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    
    while (1) {
        vTaskDelay(20 / portTICK_PERIOD_MS);
        // Wait for UART events
        if (xQueueReceive(uart0_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE); // Clear the dtmp buffer
            
            switch (event.type) {
                case UART_DATA: // Triggered on receiving new data. Indicates that data is available in the UART receive buffer.
                    uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                    ESP_LOGI(HID_DEMO_TAG, "[UART DATA]: %d [DATA EVT]: %c", event.size, *dtmp);
                    if (event.size == 3) {
                        // Mouse control
                        send_mouse_value(dtmp);
                    } else if (event.size == 1) {
                        hid_control(dtmp);
                    } else {
                        ESP_LOGI(HID_DEMO_TAG, "Input error");
                    }
                    break;
                default: // Other events
                    ESP_LOGI(HID_DEMO_TAG, "UART event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}


static void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,                     // Baud rate: 115200
        .data_bits = UART_DATA_8_BITS,           // 8 data bits
        .parity = UART_PARITY_DISABLE,           // No parity
        .stop_bits = UART_STOP_BITS_1,           // 1 stop bit
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,   // No hardware flow control
        .source_clk = UART_SCLK_APB,             // Main clock source
    };

    uart_param_config(EX_UART_NUM, &uart_config);
    
    // Install the UART driver with RX and TX buffer sizes of 2 * BUF_SIZE,
    // event queue size of 1 byte. uart0_queue receives UART events without interrupts.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 1, &uart0_queue, 0);

    // Set UART log level
    esp_log_level_set(HID_DEMO_TAG, ESP_LOG_INFO);
    
    // Set UART pins (using default pins for UART0, no changes made)
    uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    
    // Create the UART event task with 3 KB stack size and priority 12
    xTaskCreate(uart_event_task, "uart_event_task", 3 * 1024, NULL, 12, NULL);
}


static void hidd_event_callback(hidd_cb_event_t event, hidd_cb_param_t *param)
{
    switch (event) {
        case HIDD_EVENT_REG_FINISH: {  // HID GATT service registration complete event
            if (param->init_finish.state == ESP_GATT_OK) {
                // Assign a random Bluetooth address for the BLE device
                esp_bd_addr_t rand_addr = {0x04, 0x11, 0x11, 0x11, 0x11, 0x05};
                esp_err_t ret = esp_ble_gap_set_rand_addr(rand_addr);
                if (ret == ESP_OK) {
                    ESP_LOGI("BT_ADDR", "Random address set successfully.");
                } else {
                    ESP_LOGE("BT_ADDR", "Failed to set random address, error: %s", esp_err_to_name(ret));
                }
                esp_ble_gap_set_device_name(HIDD_DEVICE_NAME);
                esp_ble_gap_config_adv_data(&hidd_adv_data);
            }
            break;
        }
        case HIDD_EVENT_BLE_CONNECT: { // HID GATT service connection event
            ESP_LOGI(HID_DEMO_TAG, "HIDD_EVENT_BLE_CONNECT");
            if (remote_addr_num == 2) {
                ESP_LOGI(HID_DEMO_TAG, "remote_addr_num exceeds 2");
            } else {
                if ((remote_addr_num == 0) || (hid_conn_id[remote_addr_num - 1] != param->connect.conn_id)) {
                    hid_conn_id[remote_addr_num] = param->connect.conn_id;
                    ESP_LOGI(HID_DEMO_TAG, "hid_conn_id[%d] = %d", remote_addr_num, hid_conn_id[remote_addr_num]);
                    remote_addr_num++;
                }
            }
            break;
        }
        case HIDD_EVENT_BLE_DISCONNECT: { // HID GATT service disconnection event
            sec_conn = false;
            ESP_LOGI(HID_DEMO_TAG, "HIDD_EVENT_BLE_DISCONNECT");
            break;
        }
        default:
            break;
    }
    return;
}


static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    ESP_LOGI(HID_DEMO_TAG, "====>ESP_GAP_BLE_EVT %d          ", event);
    
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT: { // Event indicating that GAP advertising data has been set
            ESP_LOGI(HID_DEMO_TAG, "ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT");
            adv_status = true;
            break;
        }
        case ESP_GAP_BLE_SEC_REQ_EVT: { // GAP security request event; some devices require encrypted connections for BLE HID
            for (int i = 0; i < ESP_BD_ADDR_LEN; i++) {
                ESP_LOGD(HID_DEMO_TAG, "%x:", param->ble_security.ble_req.bd_addr[i]);
            }
            esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
            break;
        }
        case ESP_GAP_BLE_AUTH_CMPL_EVT: { // Authentication completion indication
            sec_conn = true;
            ESP_LOGD(HID_DEMO_TAG, "pair status = %s", param->ble_security.auth_cmpl.success ? "success" : "fail");
            // If authentication fails, print the reason for failure
            if (!param->ble_security.auth_cmpl.success) {
                ESP_LOGE(HID_DEMO_TAG, "fail reason = 0x%x", param->ble_security.auth_cmpl.fail_reason);
            }
            break;
        }
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT: { // Connection parameters update event
            break;
        }
        default: {
            break;
        }
    }
}

void app_main(void)
{
    esp_err_t ret;

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Release memory allocated to Classic Bluetooth
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    // Initialize the Bluetooth Controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s initialize controller failed\n", __func__);
        return;
    }

    // Enable the Bluetooth Controller
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s enable controller failed\n", __func__);
        return;
    }

    // Initialize Bluedroid
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed\n", __func__);
        return;
    }

    // Enable Bluedroid
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s enable bluedroid failed\n", __func__);
        return;
    }

    // Register GAP callback function
    esp_ble_gap_register_callback(gap_event_handler);

    // Register GATT callback function
    hidd_register_callbacks(hidd_event_callback);

#if 1
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_NO_BOND;             // Bond with the remote device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;                      // Set IO capability as no output, no input
    uint8_t key_size = 16;                                         // Key size range: 7–16 bytes, set to 16 bytes here
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK; // Init key: encryption and identity keys for master
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;  // Response key: encryption and identity keys for slave

    // Set security parameters
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));

    /* For BLE device as slave: init_key represents keys expected from the master, 
       rsp_key represents keys slave provides to the master;
       For BLE device as master: rsp_key represents keys expected from slave,
       init_key represents keys master provides to slave. 
    */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
#endif

    uart_init();
}

