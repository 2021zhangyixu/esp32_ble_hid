
#include "hid_device_le_prf.h"
#include <string.h>
#include "esp_log.h"

// HID report mapping table
static hid_report_map_t hid_rpt_map[HID_NUM_REPORTS];

// HID Report Map characteristic value
// Keyboard report descriptor (using format for Boot interface descriptor)
static const uint8_t hidReportMap[] = {
/*****************多媒体控制报告描述符*********************/
	0x05, 0x0c,       // USAGE_PAGE (Consumer Devices)
    0x09, 0x01,       // Usage (Consumer Control)
    0xa1, 0x01,       // Collection (Application)
    0x85, 0x05,       //   Report Id (5)
    0x15, 0x00,       //   Logical minimum  (0)
    0x25, 0x01,       //   Logical maximum (1)
    0x75, 0x01,       //   Report Size (1)
    0x95, 0x01,       //   Report Count (1)
    0x09, 0xCD,       //   Usage (Play/Pause)						//播放/暂停
    0x81, 0x06,       //   Input (Data,Var,Rel)
    0x09, 0x6F,       //   Usage (Display Brightness Increment)		//亮度增加
    0x81, 0x06,       //   Input (Data,Var,Rel)
    0x09, 0x70,       //   Usage (Display Brightness Decrement)		//亮度降低
    0x81, 0x06,       //   Input (Data,Var,Rel)
    0x09, 0xb5,       //   Usage (Scan Next Track)					//下一曲
    0x81, 0x06,       //   Input (Data,Var,Rel)
	
	
    0x09, 0xb6,       //   Usage (Scan Previous Track)				//上一曲
    0x81, 0x06,       //   Input (Data,Var,Rel)
    0x09, 0xe9,       //   Usage (Volume Up)						//音量增大
    0x81, 0x06,       //   Input (Data,Var,Rel)
    0x09, 0xea,       //   Usage (Volume Down)						//音量减小
    0x81, 0x06,       //   Input (Data,Var,Rel)
	0x95, 0x01, 	  //   Report Count (1)
    0x75, 0x08,  	  //   Report Size (1)
    0x81, 0x01,  	  //   Input: (Constant)						//保留
    0xc0              // End Collection

/********************************************************/
};

#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)

hidd_le_env_t hidd_le_env;
uint8_t hidProtocolMode = HID_PROTOCOL_MODE_REPORT;

// HID External Report Reference Descriptor
static uint16_t hidExtReportRefDesc = ESP_GATT_UUID_BATTERY_LEVEL;

//多媒体控制的报告ID和类型
static uint8_t hidReportRefDmtIn[HID_REPORT_REF_LEN] =
             { HID_RPT_ID_HEADPHONES_IN, HID_REPORT_TYPE_INPUT };

/// hid Service uuid
static uint16_t hid_le_svc = ATT_SVC_HID;

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))
///the uuid definition
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint16_t hid_report_map_uuid    = ESP_GATT_UUID_HID_REPORT_MAP;
static const uint16_t hid_report_uuid = ESP_GATT_UUID_HID_REPORT;
static const uint16_t hid_repot_map_ext_desc_uuid = ESP_GATT_UUID_EXT_RPT_REF_DESCR;
static const uint16_t hid_report_ref_descr_uuid = ESP_GATT_UUID_RPT_REF_DESCR;
///the propoty definition
static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_NOTIFY;

// 完整HID设备数据库描述 - 用于向数据库中添加属性
static esp_gatts_attr_db_t hidd_le_gatt_db[HIDD_LE_IDX_NB] =
{
    // HID 服务声明
    [HIDD_LE_IDX_SVC]               = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid,
                                                             ESP_GATT_PERM_READ_ENCRYPTED, sizeof(uint16_t), sizeof(hid_le_svc),
                                                            (uint8_t *)&hid_le_svc}},
    // HID Report Map 特征申明 (UUID 0x2A4B, 上报给主机的数据描述) ，权限为仅可读
    [HIDD_LE_IDX_REPORT_MAP_CHAR]   = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
                                                              ESP_GATT_PERM_READ,
                                                              CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
                                                              (uint8_t *)&char_prop_read}},
    // 特征值，相关数据存储在 hidReportMap 变量中
    [HIDD_LE_IDX_REPORT_MAP_VAL]    = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_map_uuid,
                                                              ESP_GATT_PERM_READ,
                                                              HIDD_LE_REPORT_MAP_MAX_LEN, sizeof(hidReportMap),
                                                              (uint8_t *)&hidReportMap}},
    // Report Map 特征描述符 - External Report Reference Descriptor
    [HIDD_LE_IDX_REPORT_MAP_EXT_REP_REF]  = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_repot_map_ext_desc_uuid,
                                                                        ESP_GATT_PERM_READ,
                                                                        sizeof(uint16_t), sizeof(uint16_t),
                                                                        (uint8_t *)&hidExtReportRefDesc}},
/********************多媒体控制相关特征***********************/
	// 特征声明
    [HIDD_LE_IDX_REPORT_HEADPHONES_IN_CHAR]  	= {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
                                                                         ESP_GATT_PERM_READ,
                                                                         CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
                                                                         (uint8_t *)&char_prop_read_notify}},
    // 特征值
    [HIDD_LE_IDX_REPORT_HEADPHONES_IN_VAL]  	= {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_uuid,
                                                                       ESP_GATT_PERM_READ,
                                                                       HIDD_LE_REPORT_MAX_LEN, 0,
                                                                       NULL}},
    // 客户端特征配置，UUID 2902
    [HIDD_LE_IDX_REPORT_HEADPHONES_IN_CCC]  	= {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
                                                                      (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE_ENCRYPTED),
                                                                      sizeof(uint16_t), 0,
                                                                      NULL}},
     // 报告参考，UUID 2908,说明当前特征使用的是 MAP 5 ，报告类型为输入
    [HIDD_LE_IDX_REPORT_HEADPHONES_IN_REP_REF] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_ref_descr_uuid,
                                                                       ESP_GATT_PERM_READ,
                                                                       sizeof(hidReportRefDmtIn), sizeof(hidReportRefDmtIn),
                                                                       hidReportRefDmtIn}},
/********************************************************/
};

// 存储 HID 报告 ID 映射表
static void hid_add_id_tbl(void)
{
    //多媒体控制映射表封装
    hid_rpt_map[0].id = hidReportRefDmtIn[0];
    hid_rpt_map[0].type = hidReportRefDmtIn[1];
    hid_rpt_map[0].handle = hidd_le_env.att_tbl[HIDD_LE_IDX_REPORT_HEADPHONES_IN_VAL];
    hid_rpt_map[0].cccdHandle = hidd_le_env.att_tbl[HIDD_LE_IDX_REPORT_HEADPHONES_IN_CCC];
    hid_rpt_map[0].mode = HID_PROTOCOL_MODE_REPORT;
    ESP_LOGI(HID_LE_PRF_TAG,"headphones headle =	%d,Report ID = %d,type = %d ",hid_rpt_map[8].handle,hid_rpt_map[8].id,hid_rpt_map[8].type);

    // 存储支持的 report ID map
    hid_dev_register_reports(HID_NUM_REPORTS, hid_rpt_map);
}

// GATTS 回调事件
void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
									esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(HID_LE_PRF_TAG, "GATTS event = %d",event);
    switch(event) {
        case ESP_GATTS_REG_EVT: { // GATT 服务注册事件
            esp_hidd_cb_param_t hidd_param;
            hidd_param.init_finish.state = param->reg.status; // 存储 GATT 服务注册状态
            // 如果是 HID 服务注册
            if(param->reg.app_id == HIDD_APP_ID) {
                // 存储 gatt 服务接口，用于表示该 gatt 服务属于哪个 gatt 服务接口
                hidd_le_env.gatt_if = gatts_if;
                if(hidd_le_env.hidd_cb != NULL) {
                    // 向 HID 处理函数中传递 HID 服务注册完成事件
                    (hidd_le_env.hidd_cb)(ESP_HIDD_EVENT_REG_FINISH, &hidd_param);
                    // 注册 BLE HID 服务列表
                    esp_ble_gatts_create_attr_tab(hidd_le_gatt_db, gatts_if, HIDD_LE_IDX_NB, 0);
                }
            }
            break;
        }
        case ESP_GATTS_CONNECT_EVT: { // 连接建立事件
            esp_hidd_cb_param_t cb_param = {0};
            // 打印连接 ID
			ESP_LOGI(HID_LE_PRF_TAG, "HID connection establish, conn_id = %x",param->connect.conn_id);
            // 复制对端设备地址
			memcpy(cb_param.connect.remote_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            // 复制连接 ID
            cb_param.connect.conn_id = param->connect.conn_id;
            cb_param.connect.ble_addr_type = param->connect.ble_addr_type;
            // 设置 BLE 连接加密密钥，接中启用加密，但不要求 MITM
            esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_NO_MITM);
            if(hidd_le_env.hidd_cb != NULL) {
                (hidd_le_env.hidd_cb)(ESP_HIDD_EVENT_BLE_CONNECT, &cb_param);
            }
            break;
        }
        case ESP_GATTS_DISCONNECT_EVT: { // 断连事件
			 if(hidd_le_env.hidd_cb != NULL) {
                    (hidd_le_env.hidd_cb)(ESP_HIDD_EVENT_BLE_DISCONNECT, NULL);
             }
            break;
        }
        case ESP_GATTS_CREAT_ATTR_TAB_EVT: { // 创建 ATT 属性表事件
            /* 如果是创建的 HID ATT 属性表数量为 HIDD_LE_IDX_NB
             * 服务 UUID 为 ESP_GATT_UUID_HID_SVC
             * 状态为 ESP_GATT_OK
             */
            if (param->add_attr_tab.num_handle == HIDD_LE_IDX_NB &&
                param->add_attr_tab.svc_uuid.uuid.uuid16 == ESP_GATT_UUID_HID_SVC &&
                param->add_attr_tab.status == ESP_GATT_OK) {
                // 将 HID ATT 属性句柄列表存储在 hidd_le_env.att_tbl 中，根据这个列表，我们可以控制对应的特征
                memcpy(hidd_le_env.att_tbl, param->add_attr_tab.handles,HIDD_LE_IDX_NB*sizeof(uint16_t));
                // 打印 HID ATT 属性句柄列表起始句柄值
                ESP_LOGI(HID_LE_PRF_TAG, "hid svc handle = 0x%x",hidd_le_env.att_tbl[HIDD_LE_IDX_SVC]);
                // 记录 HID MAP
                hid_add_id_tbl();
                //  启动 HID GATT 服务
		        esp_ble_gatts_start_service(hidd_le_env.att_tbl[HIDD_LE_IDX_SVC]);
            }
            break;
         }
        case ESP_GATTS_START_EVT: // GATT 服务启动事件
            ESP_LOGI(HID_LE_PRF_TAG, "SERVICE_START_EVT, status %d, service_handle %d",param->start.status, param->start.service_handle);
            break;
        default:
            break;
    }
}
// 注册 HID 回调函数
esp_err_t hidd_register_cb(void)
{
	esp_err_t status;
    // 注册 GATT 回调函数
	status = esp_ble_gatts_register_callback(gatts_event_handler);
	return status;
}

