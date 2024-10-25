
#include "hidd_device_api.h"
#include "hid_device_le_prf.h"
#include "hid_dev.h"
#include "esp_log.h"


esp_err_t esp_hidd_register_callbacks(esp_hidd_event_cb_t callbacks)
{
    esp_err_t hidd_status;

    if(callbacks != NULL) {
   	    hidd_le_env.hidd_cb = callbacks; // 将 HID 相关处理函数指针赋值给 hidd_le_env.hidd_cb
    } else {
        return ESP_FAIL;
    }
    // 注册 GATT 回调函数，里面就只是调用了 esp_ble_gatts_register_callback 函数
    if((hidd_status = hidd_register_cb()) != ESP_OK) {
        return hidd_status;
    }
    // 注册 GATT 电池应用程序，该函数将会触发 ESP_GATTS_REG_EVT 事件
    esp_ble_gatts_app_register(BATTRAY_APP_ID);
    // 注册 GATT HID 应用程序，该函数将会触发 ESP_GATTS_REG_EVT 事件
    if((hidd_status = esp_ble_gatts_app_register(HIDD_APP_ID)) != ESP_OK) {
        return hidd_status;
    }
    return hidd_status;
}

// 进行多媒体控制
void hid_headphones_control(uint16_t conn_id, uint8_t key_cmd)
{
    hid_dev_send_report(hidd_le_env.gatt_if, conn_id,
                        HID_RPT_ID_HEADPHONES_IN, HID_REPORT_TYPE_INPUT, sizeof(key_cmd), &key_cmd);
    return;

}


