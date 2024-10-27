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



#ifndef __HID_DEVICE_LE_PRF__
#define __HID_DEVICE_LE_PRF__
#include <stdbool.h>
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "hidd_device_api.h"
#include "esp_gap_ble_api.h"
#include "hid_dev.h"


#define HID_LE_PRF_TAG                   "HID_LE_PRF" // HID BLE profile log tag
#define HID_NUM_REPORTS                  9            // Number of HID reports defined in the service
#define HID_RPT_ID_MOUSE_IN              1            // Mouse input report ID
#define HIDD_APP_ID                      0x1812       // HID service APP ID
#define ATT_SVC_HID                      0x1812
#define HIDD_LE_REPORT_MAX_LEN           (255)        // Maximum length of Report Characteristic Value
#define HIDD_LE_REPORT_MAP_MAX_LEN       (512)        // Maximum length of Report Map Characteristic Value

/* HID protocol mode values */
#define HID_PROTOCOL_MODE_BOOT                 0x00   // Boot Protocol Mode
#define HID_PROTOCOL_MODE_REPORT               0x01   // Report Protocol Mode

/* Attribute value lengths */
#define HID_PROTOCOL_MODE_LEN                  1      // HID Protocol Mode
#define HID_INFORMATION_LEN                    4      // HID Information
#define HID_REPORT_REF_LEN                     2      // HID Report Reference Descriptor
#define HID_EXT_REPORT_REF_LEN                 2      // External Report Reference Descriptor

/* HID Report types */
#define HID_REPORT_TYPE_INPUT                  1      // Input data to the client, such as from a mouse or keyboard
#define HID_REPORT_TYPE_OUTPUT                 2      // Output data sent from the client to the server, e.g., changing LED status or controlling vibration feedback
#define HID_REPORT_TYPE_FEATURE                3      // Provides additional device feature information, typically used to configure device behavior or state

// HID Service Attributes Indexes
enum {
    HIDD_LE_IDX_SVC,  // Service

    // Included Service
    HIDD_LE_IDX_INCL_SVC,

    // Report Map
    HIDD_LE_IDX_REPORT_MAP_CHAR,         // Report Map characteristic declaration (UUID 0x2A4B)
    HIDD_LE_IDX_REPORT_MAP_VAL,          // Report Map characteristic value
    HIDD_LE_IDX_REPORT_MAP_EXT_REP_REF,  // External Report Reference Descriptor

    /****** Mouse Control Service Attribute Indexes ******/
    HIDD_LE_IDX_REPORT_MOUSE_IN_CHAR,
    HIDD_LE_IDX_REPORT_MOUSE_IN_VAL,
    HIDD_LE_IDX_REPORT_MOUSE_IN_CCC,
    HIDD_LE_IDX_REPORT_MOUSE_REP_REF,
    /**********************************************************/

    HIDD_LE_IDX_NB,  // Total number of indexes
};


/// HID device information structure
typedef struct
{
    /// bcdHID
    uint16_t bcdHID;
    /// bCountryCode
    uint8_t bCountryCode;
    /// Flags
    uint8_t flags;
}hids_hid_info_t;


/* service engine control block */
typedef struct {
    esp_gatt_if_t          gatt_if;                  // Identifier for the service assigned by the protocol stack
    uint16_t               att_tbl[HIDD_LE_IDX_NB];  // List of attribute handles for HID
    esp_hidd_event_cb_t    hidd_cb;                  // Callback function for HID events
} hidd_le_env_t;

extern hidd_le_env_t hidd_le_env;
extern uint8_t hidProtocolMode;

void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
									esp_ble_gatts_cb_param_t *param);


#endif  ///__HID_DEVICE_LE_PRF__
