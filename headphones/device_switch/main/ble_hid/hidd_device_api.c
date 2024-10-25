
#include "hidd_device_api.h"
#include "hid_device_le_prf.h"
#include "hid_dev.h"
#include "esp_log.h"


esp_err_t esp_hidd_register_callbacks(esp_hidd_event_cb_t callbacks)
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

void hid_headphones_control(uint16_t conn_id, uint8_t key_cmd)
{
    hid_dev_send_report(hidd_le_env.gatt_if, conn_id,
                        HID_RPT_ID_HEADPHONES_IN, HID_REPORT_TYPE_INPUT, sizeof(key_cmd), &key_cmd);
    return;
}


