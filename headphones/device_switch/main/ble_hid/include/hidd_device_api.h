// Copyright 2017-2018 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef __ESP_HIDD_API_H__
#define __ESP_HIDD_API_H__

#include "esp_bt_defs.h"
#include "esp_gatt_defs.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ESP_HIDD_EVENT_REG_FINISH = 0,  // HID GATT 服务创建完成事件
    ESP_BAT_EVENT_REG,              // 电池电量 GATT 服务创建完成事件
    ESP_HIDD_EVENT_BLE_CONNECT,     // HID GATT 服务连接事件
    ESP_HIDD_EVENT_BLE_DISCONNECT,  // HID GATT 服务断开连接事件
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

void hid_headphones_control(uint16_t conn_id, uint8_t key_cmd);

#ifdef __cplusplus
}
#endif

#endif /* __ESP_HIDD_API_H__ */
