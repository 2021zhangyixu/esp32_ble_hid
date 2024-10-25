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

//HID BLE profile log tag
#define HID_LE_PRF_TAG                        "HID_LE_PRF"

/// Maximal number of HIDS that can be added in the DB
#ifndef USE_ONE_HIDS_INSTANCE
#define HIDD_LE_NB_HIDS_INST_MAX              (2)
#else
#define HIDD_LE_NB_HIDS_INST_MAX              (1)
#endif

// #define HIDD_GREAT_VER   0x01  //Version + Subversion
// #define HIDD_SUB_VER     0x00  //Version + Subversion
// #define HIDD_VERSION     ((HIDD_GREAT_VER<<8)|HIDD_SUB_VER)  //Version + Subversion

#define HID_MAX_APPS                 1

// Number of HID reports defined in the service
#define HID_NUM_REPORTS          9

#define	HID_RPT_ID_HEADPHONES_IN		5	//多媒体控制报告ID

#define HIDD_APP_ID			 0x1812   // HID service APP ID
#define BATTRAY_APP_ID       0x180f   // Battery service APP ID
#define ATT_SVC_HID          0x1812

/// Maximal number of Report Char. that can be added in the DB for one HIDS - Up to 11
#define HIDD_LE_NB_REPORT_INST_MAX            (5)

/// Maximal length of Report Char. Value
#define HIDD_LE_REPORT_MAX_LEN                (255)
/// Maximal length of Report Map Char. Value
#define HIDD_LE_REPORT_MAP_MAX_LEN            (512)

/// Length of Boot Report Char. Value Maximal Length
#define HIDD_LE_BOOT_REPORT_MAX_LEN           (8)

/* HID information flags */
#define HID_FLAGS_REMOTE_WAKE           0x01      // RemoteWake
#define HID_FLAGS_NORMALLY_CONNECTABLE  0x02      // NormallyConnectable

/* HID protocol mode values */
#define HID_PROTOCOL_MODE_BOOT          0x00      // Boot Protocol Mode
#define HID_PROTOCOL_MODE_REPORT        0x01      // Report Protocol Mode

/* Attribute value lengths */
#define HID_PROTOCOL_MODE_LEN           1         // HID Protocol Mode
#define HID_INFORMATION_LEN             4         // HID Information
#define HID_REPORT_REF_LEN              2         // HID Report Reference Descriptor
#define HID_EXT_REPORT_REF_LEN          2         // External Report Reference Descriptor

// HID feature flags
#define HID_KBD_FLAGS             HID_FLAGS_REMOTE_WAKE

/* HID Report type */
#define HID_REPORT_TYPE_INPUT       1 // 向客户端输入数据，如鼠标或键盘
#define HID_REPORT_TYPE_OUTPUT      2 // 客户端向服务端发送输出数据，如改变 LED 状态、控制振动反馈
#define HID_REPORT_TYPE_FEATURE     3 // 用于提供附加的设备特性信息，通常用于配置设备的行为或状态

// HID Service Attributes Indexes
enum {
    HIDD_LE_IDX_SVC,

    // 包含服务
    HIDD_LE_IDX_INCL_SVC,

    // Report Map
    HIDD_LE_IDX_REPORT_MAP_CHAR,         // Report Map 特征申明 (UUID 0x2A4B)
    HIDD_LE_IDX_REPORT_MAP_VAL,          // Report Map 特征值
    HIDD_LE_IDX_REPORT_MAP_EXT_REP_REF,  // External Report Reference Descriptor

/*****************多媒体控制服务属性表下标*******************/
	HIDD_LE_IDX_REPORT_HEADPHONES_IN_CHAR,
    HIDD_LE_IDX_REPORT_HEADPHONES_IN_VAL,
    HIDD_LE_IDX_REPORT_HEADPHONES_IN_CCC,
    HIDD_LE_IDX_REPORT_HEADPHONES_IN_REP_REF,
/********************************************************/
    HIDD_LE_IDX_NB,
};

/// HID 设备信息结构体
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
    esp_gatt_if_t          gatt_if;                  // 协议栈分配的，用于指定服务的标识符
    uint16_t               att_tbl[HIDD_LE_IDX_NB];  // HID 的属性句柄列表
    esp_hidd_event_cb_t    hidd_cb;                  // HID 回调函数
} hidd_le_env_t;

extern hidd_le_env_t hidd_le_env;
extern uint8_t hidProtocolMode;

esp_err_t hidd_register_cb(void);


#endif  ///__HID_DEVICE_LE_PRF__
