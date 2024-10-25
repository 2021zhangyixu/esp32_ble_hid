
#include "hid_device_le_prf.h"
#include <string.h>
#include "esp_log.h"

// HID report mapping table
static hid_report_map_t hid_rpt_map[HID_NUM_REPORTS];

// HID Report Map characteristic value
// Keyboard report descriptor (using format for Boot interface descriptor)
static const uint8_t hidReportMap[] = {
/*****************Multimedia control report descriptor*****************/
	0x05, 0x0c,       // USAGE_PAGE (Consumer Devices)
    0x09, 0x01,       // Usage (Consumer Control)
    0xa1, 0x01,       // Collection (Application)
    0x85, 0x05,       //   Report Id (5)
    0x15, 0x00,       //   Logical minimum  (0)
    0x25, 0x01,       //   Logical maximum (1)
    0x75, 0x01,       //   Report Size (1)
    0x95, 0x01,       //   Report Count (1)
    0x09, 0xCD,       //   Usage (Play/Pause)						
    0x81, 0x06,       //   Input (Data,Var,Rel)
    0x09, 0x6F,       //   Usage (Display Brightness Increment)		
    0x81, 0x06,       //   Input (Data,Var,Rel)
    0x09, 0x70,       //   Usage (Display Brightness Decrement)		
    0x81, 0x06,       //   Input (Data,Var,Rel)
    0x09, 0xb5,       //   Usage (Scan Next Track)					
    0x81, 0x06,       //   Input (Data,Var,Rel)
	
    0x09, 0xb6,       //   Usage (Scan Previous Track)				
    0x81, 0x06,       //   Input (Data,Var,Rel)
    0x09, 0xe9,       //   Usage (Volume Up)						
    0x81, 0x06,       //   Input (Data,Var,Rel)
    0x09, 0xea,       //   Usage (Volume Down)						
    0x81, 0x06,       //   Input (Data,Var,Rel)
	0x95, 0x01, 	  //   Report Count (1)
    0x75, 0x08,  	  //   Report Size (1)
    0x81, 0x01,  	  //   Input: (Constant)						
    0xc0              // End Collection

/**********************************************************************/
};

#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)

hidd_le_env_t hidd_le_env;
uint8_t hidProtocolMode = HID_PROTOCOL_MODE_REPORT;

// HID External Report Reference Descriptor
static uint16_t hidExtReportRefDesc = ESP_GATT_UUID_BATTERY_LEVEL;

// Report ID and type of headphones control
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

// Complete HID device database description - used to add attributes to the database
static esp_gatts_attr_db_t hidd_le_gatt_db[HIDD_LE_IDX_NB] =
{
    // HID Service Declaration
    [HIDD_LE_IDX_SVC]               = {{ESP_GATT_AUTO_RSP}, 
                                      {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid,
                                       ESP_GATT_PERM_READ_ENCRYPTED, sizeof(uint16_t), 
                                       sizeof(hid_le_svc), (uint8_t *)&hid_le_svc}},
    
    // HID Report Map Characteristic Declaration (UUID 0x2A4B, description of data reported to the host)
    // Permissions: Read only
    [HIDD_LE_IDX_REPORT_MAP_CHAR]   = {{ESP_GATT_AUTO_RSP}, 
                                      {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
                                       ESP_GATT_PERM_READ,
                                       CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
                                       (uint8_t *)&char_prop_read}},
    
    // Characteristic Value, related data stored in the hidReportMap variable
    [HIDD_LE_IDX_REPORT_MAP_VAL]     = {{ESP_GATT_AUTO_RSP}, 
                                      {ESP_UUID_LEN_16, (uint8_t *)&hid_report_map_uuid,
                                       ESP_GATT_PERM_READ,
                                       HIDD_LE_REPORT_MAP_MAX_LEN, sizeof(hidReportMap),
                                       (uint8_t *)&hidReportMap}},
    
    // Report Map Characteristic Descriptor - External Report Reference Descriptor
    [HIDD_LE_IDX_REPORT_MAP_EXT_REP_REF] = {{ESP_GATT_AUTO_RSP}, 
                                            {ESP_UUID_LEN_16, (uint8_t *)&hid_repot_map_ext_desc_uuid,
                                             ESP_GATT_PERM_READ,
                                             sizeof(uint16_t), sizeof(uint16_t),
                                             (uint8_t *)&hidExtReportRefDesc}},
    
    /******** Multimedia Control Related Characteristics ********/
    // Characteristic Declaration
    [HIDD_LE_IDX_REPORT_HEADPHONES_IN_CHAR]   = {{ESP_GATT_AUTO_RSP}, 
                                                 {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
                                                  ESP_GATT_PERM_READ,
                                                  CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
                                                  (uint8_t *)&char_prop_read_notify}},
    
    // Characteristic Value
    [HIDD_LE_IDX_REPORT_HEADPHONES_IN_VAL]    = {{ESP_GATT_AUTO_RSP}, 
                                                 {ESP_UUID_LEN_16, (uint8_t *)&hid_report_uuid,
                                                  ESP_GATT_PERM_READ,
                                                  HIDD_LE_REPORT_MAX_LEN, 0,
                                                  NULL}},
    
    // Client Characteristic Configuration, UUID 2902
    [HIDD_LE_IDX_REPORT_HEADPHONES_IN_CCC]    = {{ESP_GATT_AUTO_RSP}, 
                                                 {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
                                                  (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE_ENCRYPTED),
                                                  sizeof(uint16_t), 0,
                                                  NULL}},
    
    // Report Reference, UUID 2908, indicates that the current characteristic uses MAP 5, report type is input
    [HIDD_LE_IDX_REPORT_HEADPHONES_IN_REP_REF] = {{ESP_GATT_AUTO_RSP}, 
                                                  {ESP_UUID_LEN_16, (uint8_t *)&hid_report_ref_descr_uuid,
                                                   ESP_GATT_PERM_READ,
                                                   sizeof(hidReportRefDmtIn), sizeof(hidReportRefDmtIn),
                                                   hidReportRefDmtIn}},
    /************************************************************/
};

/**
 * @brief Store HID Report ID Mapping Table
 */
static void hid_add_id_tbl(void)
{
    // Multimedia control mapping table encapsulation
    hid_rpt_map[0].id = hidReportRefDmtIn[0];
    hid_rpt_map[0].type = hidReportRefDmtIn[1];
    hid_rpt_map[0].handle = hidd_le_env.att_tbl[HIDD_LE_IDX_REPORT_HEADPHONES_IN_VAL];
    hid_rpt_map[0].cccdHandle = hidd_le_env.att_tbl[HIDD_LE_IDX_REPORT_HEADPHONES_IN_CCC];
    hid_rpt_map[0].mode = HID_PROTOCOL_MODE_REPORT;
    
    ESP_LOGI(HID_LE_PRF_TAG, "Headphones handle = %d, Report ID = %d, Type = %d", 
             hid_rpt_map[0].handle, hid_rpt_map[0].id, hid_rpt_map[0].type);

    // Store the supported report ID map
    hid_dev_register_reports(HID_NUM_REPORTS, hid_rpt_map);
}

void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                         esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(HID_LE_PRF_TAG, "GATTS event = %d", event);
    
    switch (event) {
        case ESP_GATTS_REG_EVT: { // GATT service registration event
            hidd_cb_param_t hidd_param;
            hidd_param.init_finish.state = param->reg.status; // Store the GATT service registration status
            
            // If this is the HID service registration
            if (param->reg.app_id == HIDD_APP_ID) {
                // Store the GATT interface to identify the GATT service
                hidd_le_env.gatt_if = gatts_if;
                
                if (hidd_le_env.hidd_cb != NULL) {
                    // Pass the HID service registration complete event to the HID handler
                    (hidd_le_env.hidd_cb)(HIDD_EVENT_REG_FINISH, &hidd_param);
                    // Register the BLE HID service attribute table
                    esp_ble_gatts_create_attr_tab(hidd_le_gatt_db, gatts_if, HIDD_LE_IDX_NB, 0);
                }
            }
            break;
        }

        case ESP_GATTS_CONNECT_EVT: { // Connection established event
            hidd_cb_param_t cb_param = {0};
            // Log the connection ID
            ESP_LOGI(HID_LE_PRF_TAG, "HID connection established, conn_id = %x", param->connect.conn_id);
            // Copy the remote device address
            memcpy(cb_param.connect.remote_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            // Copy the connection ID
            cb_param.connect.conn_id = param->connect.conn_id;
            cb_param.connect.ble_addr_type = param->connect.ble_addr_type;
            // Set BLE connection encryption key; enable encryption but do not require MITM
            esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_NO_MITM);
            
            if (hidd_le_env.hidd_cb != NULL) {
                (hidd_le_env.hidd_cb)(HIDD_EVENT_BLE_CONNECT, &cb_param);
            }
            break;
        }

        case ESP_GATTS_DISCONNECT_EVT: { // Disconnection event
            if (hidd_le_env.hidd_cb != NULL) {
                (hidd_le_env.hidd_cb)(HIDD_EVENT_BLE_DISCONNECT, NULL);
            }
            break;
        }

        case ESP_GATTS_CREAT_ATTR_TAB_EVT: { // Create ATT attribute table event
            /* Check if the created HID ATT attribute table has the expected number of handles,
               if the service UUID is ESP_GATT_UUID_HID_SVC, and if the status is ESP_GATT_OK.
            */
            if (param->add_attr_tab.num_handle == HIDD_LE_IDX_NB &&
                param->add_attr_tab.svc_uuid.uuid.uuid16 == ESP_GATT_UUID_HID_SVC &&
                param->add_attr_tab.status == ESP_GATT_OK) {
                
                // Store the HID ATT attribute handle list in hidd_le_env.att_tbl
                memcpy(hidd_le_env.att_tbl, param->add_attr_tab.handles, HIDD_LE_IDX_NB * sizeof(uint16_t));
                // Log the starting handle value of the HID ATT attribute
                ESP_LOGI(HID_LE_PRF_TAG, "hid svc handle = 0x%x", hidd_le_env.att_tbl[HIDD_LE_IDX_SVC]);
                // Record HID map
                hid_add_id_tbl();
                // Start the HID GATT service
                esp_ble_gatts_start_service(hidd_le_env.att_tbl[HIDD_LE_IDX_SVC]);
            }
            break;
        }

        case ESP_GATTS_START_EVT: {// GATT service start event
            ESP_LOGI(HID_LE_PRF_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
            break;
        }
        default:
            break;
    }
}



