

#ifndef __ESP_HIDD_API_H__
#define __ESP_HIDD_API_H__

#include "esp_bt_defs.h"
#include "esp_gatt_defs.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HIDD_AUDIO_PAUSE_PLAY              (1 << 0)  // Pause and play button
#define HIDD_AUDIO_BRIGHTNESS_INCREASE     (1 << 1)  // Increase brightness
#define HIDD_AUDIO_BRIGHTNESS_DECREASE     (1 << 2)  // Decrease brightness
#define HIDD_AUDIO_NEXT_TRACK              (1 << 3)  // Next track
#define HIDD_AUDIO_PREVIOUS_TRACK          (1 << 4)  // Previous track
#define HIDD_AUDIO_INCREASE_VOLUME         (1 << 5)  // Increase volume
#define HIDD_AUDIO_DECREASE_VOLUME         (1 << 6)  // Decrease volume
#define HIDD_AUDIO_RESERVED                (1 << 7)  // Reserved


typedef enum {
    HIDD_EVENT_REG_FINISH = 0,   // HID GATT service registration complete event
    HIDD_EVENT_BLE_CONNECT,      // HID GATT service connection event
    HIDD_EVENT_BLE_DISCONNECT,   // HID GATT service disconnection event
} hidd_cb_event_t;

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
} hidd_cb_param_t;

/**
 * @brief HID device event callback function type
 * @param event : Event type
 * @param param : Point to callback parameter, currently is union type
 */
typedef void (*esp_hidd_event_cb_t) (hidd_cb_event_t event, hidd_cb_param_t *param);

/**
 *
 * @brief        This function is called to receive hid device callback event
 *
 * @param[in]    callbacks: callback functions
 *
 * @return       ESP_OK - success, other - failed
 *
 */
esp_err_t hidd_register_callbacks(esp_hidd_event_cb_t callbacks);

void hidd_send_mouse_value(uint16_t conn_id, uint8_t mouse_button, int8_t mickeys_x, int8_t mickeys_y);

#ifdef __cplusplus
}
#endif

#endif /* __ESP_HIDD_API_H__ */
