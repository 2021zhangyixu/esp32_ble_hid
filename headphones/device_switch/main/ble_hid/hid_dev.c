
#include "hid_dev.h"
#include "esp_log.h"

static hid_report_map_t *hid_dev_rpt_tbl; // 记录 HID MAP 的数组
static uint8_t hid_dev_rpt_tbl_Len; // HID MAP 数组的长度

// 找到指定的特征句柄
static hid_report_map_t *hid_dev_rpt_by_id(uint8_t id, uint8_t type)
{
    hid_report_map_t *rpt = hid_dev_rpt_tbl;

    for (uint8_t i = hid_dev_rpt_tbl_Len; i > 0; i--, rpt++) {
        if (rpt->id == id && rpt->type == type && rpt->mode == hidProtocolMode) {
            return rpt;
        }
    }

    return NULL;
}

// 记录 HID MAP 的数组和数量
void hid_dev_register_reports(uint8_t num_reports, hid_report_map_t *p_report)
{
    hid_dev_rpt_tbl = p_report;
    hid_dev_rpt_tbl_Len = num_reports;
    return;
}

// 发送 HID 数据
void hid_dev_send_report(esp_gatt_if_t gatts_if, uint16_t conn_id,
                                    uint8_t id, uint8_t type, uint8_t length, uint8_t *data)
{
    hid_report_map_t *p_rpt;

    // get att handle for report
    if ((p_rpt = hid_dev_rpt_by_id(id, type)) != NULL) {
        // if notifications are enabled
        ESP_LOGD(HID_LE_PRF_TAG, "%s(), send the report, data = %d", __func__, *data);
        /* gatts_if : 用于指示 GATT server 的句柄，由 esp_ble_gatts_register_callback() 返回
         * conn_id : 连接 ID
         * p_rpt->handle : 表示要通知或指示的特性（Characteristic）的句柄
         * length : 要发送的数据长度，以字节为单位
         * data : 指向发送数据的指针，指向实际要发送给客户端的值（数据）
         * false : 表示这是一个通知（Notification），不需要客户端确认接收到消息
         */
        esp_ble_gatts_send_indicate(gatts_if, conn_id, p_rpt->handle, length, data, false);
    }

    return;
}
