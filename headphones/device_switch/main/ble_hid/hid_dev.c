
#include "hid_dev.h"
#include "esp_log.h"

static hid_report_map_t *hid_dev_rpt_tbl; // Array to store the HID report map
static uint8_t hid_dev_rpt_tbl_len;       // Length of the HID report map array

/**
 * @brief Find HID map by ID and type
 * 
 * @param id ID of the HID map
 * @param type Type of the HID map
 * 
 * @return hid_report_map_t* Pointer to the HID map
 */
static hid_report_map_t *hid_dev_rpt_by_id(uint8_t id, uint8_t type)
{
    hid_report_map_t *rpt = hid_dev_rpt_tbl;

    for (uint8_t i = hid_dev_rpt_tbl_len; i > 0; i--, rpt++) {
        if (rpt->id == id && rpt->type == type && rpt->mode == hidProtocolMode) {
            return rpt;
        }
    }

    return NULL;
}

void hid_dev_register_reports(uint8_t num_reports, hid_report_map_t *p_report)
{
    hid_dev_rpt_tbl = p_report;
    hid_dev_rpt_tbl_len = num_reports;
    return;
}

void hid_dev_send_report(esp_gatt_if_t gatts_if, uint16_t conn_id,
                         uint8_t id, uint8_t type, uint8_t length, uint8_t *data)
{
    hid_report_map_t *p_rpt;

    // Get attribute handle for the report
    if ((p_rpt = hid_dev_rpt_by_id(id, type)) != NULL) {
        // If notifications are enabled
        ESP_LOGD(HID_LE_PRF_TAG, "%s(), sending the report, data = %d", __func__, *data);
        
        /* 
         * gatts_if : Indicates the handle of the GATT server, returned by esp_ble_gatts_register_callback()
         * conn_id : Connection ID
         * p_rpt->handle : Handle of the characteristic to notify or indicate
         * length : Length of the data to be sent, in bytes
         * data : Pointer to the data to be sent, pointing to the actual value sent to the client
         * false : Indicates that this is a notification, which does not require confirmation from the client
         */
        esp_ble_gatts_send_indicate(gatts_if, conn_id, p_rpt->handle, length, data, false);
    }

    return;
}

