
#ifndef HID_DEV_H__
#define HID_DEV_H__

#include "hid_device_le_prf.h"


#ifdef __cplusplus
extern "C" {
#endif

// HID report mapping table
typedef struct
{
  uint16_t    handle;           // Handle of report characteristic
  uint16_t    cccdHandle;       // Handle of CCCD for report characteristic
  uint8_t     id;               // Report ID
  uint8_t     type;             // Report type
  uint8_t     mode;             // Protocol mode (report or boot)
} hid_report_map_t;

/**
 * @brief Register HID reports
 * 
 * @param[in]  num_reports Number of reports
 * @param[in]  p_report Report mapping table
 */
void hid_dev_register_reports(uint8_t num_reports, hid_report_map_t *p_report);

/**
 * @brief Send HID report
 * 
 * @param[in]  gatts_if GATT interface
 * @param[in]  conn_id Connection ID
 * @param[in]  id Report ID
 * @param[in]  type Report type
 * @param[in]  length Report length
 * @param[in]  data Report data
*/
void hid_dev_send_report(esp_gatt_if_t gatts_if, uint16_t conn_id,
                                    uint8_t id, uint8_t type, uint8_t length, uint8_t *data);

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* HID_DEV_H__ */
