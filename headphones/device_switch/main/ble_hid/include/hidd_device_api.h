

#ifndef __ESP_HIDD_API_H__
#define __ESP_HIDD_API_H__

#include "esp_bt_defs.h"
#include "esp_gatt_defs.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ESP_HIDD_EVENT_REG_FINISH = 0,   // HID GATT service registration complete event
    ESP_BAT_EVENT_REG,               // Battery level GATT service registration complete event
    ESP_HIDD_EVENT_BLE_CONNECT,      // HID GATT service connection event
    ESP_HIDD_EVENT_BLE_DISCONNECT,   // HID GATT service disconnection event
} esp_hidd_cb_event_t;

/**
 * @brief HIDD callback parameters union
 */
typedef union {
    /**
	 * @brief ESP_HIDD_EVENT_INIT_FINISH
	 */
    struct hidd_init_finish_evt_param {
        esp_gatt_status_t state;				/*!< Initial status */
        esp_gatt_if_t gatts_if;
    } init_finish;							    /*!< HID callback param of ESP_HIDD_EVENT_INIT_FINISH */

    /**
     * @brief ESP_HIDD_EVENT_CONNECT
	 */
    struct hidd_connect_evt_param {
        uint16_t conn_id;
        esp_bd_addr_t remote_bda;                   /*!< HID Remote bluetooth connection index */
        esp_ble_addr_type_t ble_addr_type;          /*!< Remote BLE device address type */
    } connect;									    /*!< HID callback param of ESP_HIDD_EVENT_CONNECT */
} esp_hidd_cb_param_t;

/**
 * @brief HID device event callback function type
 * @param event : Event type
 * @param param : Point to callback parameter, currently is union type
 */
typedef void (*esp_hidd_event_cb_t) (esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

/**
 *
 * @brief           This function is called to receive hid device callback event
 *
 * @param[in]    callbacks: callback functions
 *
 * @return         ESP_OK - success, other - failed
 *
 */
esp_err_t esp_hidd_register_callbacks(esp_hidd_event_cb_t callbacks);

/**
 * @brief           This function is called to send HID report to host
 * 
 * @param[in]    conn_id: connection id
 * @param[in]    report_id: report id
 * 
 */
void hid_headphones_control(uint16_t conn_id, uint8_t key_cmd);

#ifdef __cplusplus
}
#endif

#endif /* __ESP_HIDD_API_H__ */
