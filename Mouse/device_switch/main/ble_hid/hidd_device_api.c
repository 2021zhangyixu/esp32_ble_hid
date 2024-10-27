
#include "hidd_device_api.h"
#include "hid_device_le_prf.h"
#include "hid_dev.h"
#include "esp_log.h"

// HID mouse input report length
#define HID_MOUSE_IN_RPT_LEN        5

esp_err_t hidd_register_callbacks(esp_hidd_event_cb_t callbacks)
{
    esp_err_t hidd_status;

    if (callbacks != NULL) {
        hidd_le_env.hidd_cb = callbacks; // Assign the HID event callback function pointer to hidd_le_env.hidd_cb
    } else {
        return ESP_FAIL;
    }

    // Register the GATT callback function
    if ((hidd_status = esp_ble_gatts_register_callback(gatts_event_handler)) != ESP_OK) {
        return hidd_status;
    }

    // Register the GATT HID application, which will trigger the ESP_GATTS_REG_EVT event
    if ((hidd_status = esp_ble_gatts_app_register(HIDD_APP_ID)) != ESP_OK) {
        return hidd_status;
    }

    return hidd_status;
}

void hidd_send_mouse_value(uint16_t conn_id, uint8_t mouse_button, int8_t mickeys_x, int8_t mickeys_y)
{
    uint8_t buffer[HID_MOUSE_IN_RPT_LEN];

    buffer[0] = mouse_button;   // Buttons
    buffer[1] = mickeys_x;      // X
    buffer[2] = mickeys_y;      // Y
    buffer[3] = 0;              // Wheel
    buffer[4] = 0;              // AC Pan

    hid_dev_send_report(hidd_le_env.gatt_if, conn_id,
                        HID_RPT_ID_MOUSE_IN, HID_REPORT_TYPE_INPUT, HID_MOUSE_IN_RPT_LEN, buffer);
    return;
}

