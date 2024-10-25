
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"

#include "hid_dev.h"
#include "driver/uart.h"

#define HID_DEMO_TAG "HID_DEMO"

static uint16_t hid_conn_id[2] = {0}; // HID 连接 ID
static uint8_t remote_addr_num = 0;   // 当前存储的远端设备数量
static uint8_t remote_conn_id = 0;    // 要操作的远端设备连接 ID
static bool sec_conn = false,adv_status = false; // 安全连接标志位，广播状态标志位

#define HIDD_DEVICE_NAME            "HID"
static uint8_t hidd_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};

static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,      // 不是扫描回应包
    .include_name = true,       // 包含设备名称
    .include_txpower = true,    // 包含发射功率
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = ESP_BLE_APPEARANCE_GENERIC_HID,       // HID Generic,	ESP_HID_APPEARANCE_KEYBOARD
    .manufacturer_len = 0,         // 无厂商数据
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,         // 无服务数据
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),  // 服务 UUID 长度
    .p_service_uuid = hidd_service_uuid128,
    .flag = 0x6,  // BLE 标志位
};

static esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x30,
    .adv_type           = ADV_TYPE_IND,          // 通用广播
    .own_addr_type      = BLE_ADDR_TYPE_RANDOM,  // 随机静态地址
    .channel_map        = ADV_CHNL_ALL,          // 37，38，39 三个通道都使用
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY, //允许扫描和连接
};

#define EX_UART_NUM UART_NUM_0
#define BUF_SIZE (128)
#define RD_BUF_SIZE (128)

static QueueHandle_t key_flag;    // 改消息队列用于记录 PC 端按键按下的数字

void send_headphones_value(uint8_t *value)
{
	static uint32_t tmp = 0xFFFFFFFF;
    ESP_LOGI(HID_DEMO_TAG, "send_headphones_value: %c",*value);
	switch(*value)
	{
		case '0':	tmp = 0;	break;
		case '1':	tmp = 1;	break;
		case '2':	tmp = 2;	break;
		case '3':	tmp = 3;	break;
		case '4':	tmp = 4;	break;
		case '5':	tmp = 5;	break;
		case '6':	tmp = 6;	break;
		case '7':	tmp = 7;	break;
		case '8':	tmp = 8;	break;
		case '9':	tmp = 9;    break;
		case 'a':	tmp = 10;	break;
		case 'b':	tmp = 11;	break;
		case 'c':	tmp = 12;	break;
		case 'd':	tmp = 13;	break;
		case 'e':	tmp = 14;	break;
		case 'f':	tmp = 15;	break;
		default : break;
	}
	if(tmp != 0xFFFFFFFF)
	{
		xQueueSend(key_flag,&tmp,100/portTICK_PERIOD_MS);
	}
}
// 消息队列
static QueueHandle_t uart0_queue; // 改消息队列用于记录 UART0 接收到的数据

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    // 接收数据缓冲区
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    while (1) {
        vTaskDelay(20 / portTICK_PERIOD_MS);
        // 等待串口事件
        if(xQueueReceive(uart0_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);// 清空 dtmp 缓冲区
            switch(event.type) {
                case UART_DATA: // 接收到新数据时触发。通过这个事件可以知道 UART 接收缓冲区中有数据可读。
                    // 将收到的数据存入缓冲区
                    uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                    ESP_LOGI(HID_DEMO_TAG, "[UART DATA]: %d [DATA EVT]: %c", event.size,*dtmp);
					if (event.size == 1) {
						//多媒体控制测试
						send_headphones_value(dtmp);
					} else {
						ESP_LOGI(HID_DEMO_TAG,"input error");
                    }
                    break;
                default: // 其他事件
                    ESP_LOGI(HID_DEMO_TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

// HID 测试任务
void hid_demo_task(void *pvParameters)
{
	uint8_t gpio_num = 0; // 按下的按键

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    while(1)
	{
        if(xQueueReceive(key_flag, &gpio_num, portMAX_DELAY))
        {
            if (adv_status)
            {   
                // 8. 进入通用广播模式,存储对端设备地址
                // 9. 切换连接设备
                // a. 切换连接设备     b. 清空连接设备列表
                switch (gpio_num)
                {
                case 8:
                    ESP_LOGI(HID_DEMO_TAG,"esp_ble_gap_start_advertising ADV_TYPE_IND");
                    esp_ble_gap_start_advertising(&hidd_adv_params); // 启动广播
                    break;
                case 9:
                    esp_ble_gap_stop_advertising(); // 停止广播
                    if (remote_conn_id < 1) // 数据存储满了
                    {
                        remote_conn_id ++;
                    } else {
                        remote_conn_id = 0;
                    }
                    ESP_LOGI(HID_DEMO_TAG,"remote_conn_id : %d",remote_conn_id);
                    break;
                case 10:
                    break;
                case 11:
                    break;
                default:
                    break;
                }
            }
            if (sec_conn) 
            {
                //多媒体控制				
                if(gpio_num < 8)
                {
                    //0.暂停和播放键
                    //1.亮度增大    2.亮度减弱
                    //3.下一曲      4.上一曲
                    //5.音量增大    6.音量减小
                    //7.保留
                    ESP_LOGI(HID_DEMO_TAG,"esp_hidd_send_duomeiti_value hid_conn_id[%d] : %d",remote_conn_id,hid_conn_id[remote_conn_id]);
                    hid_headphones_control(hid_conn_id[remote_conn_id],(1 << gpio_num));
                }
            }
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);	
	}
}

void uart_init(void)
{
	uart_config_t uart_config = {
			.baud_rate = 115200,
			.data_bits = UART_DATA_8_BITS,
			.parity = UART_PARITY_DISABLE, // 无校验位
			.stop_bits = UART_STOP_BITS_1, // 1 停止位
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // 无硬件流控
			.source_clk = UART_SCLK_APB, // 主时钟
		};
    // 注册 UART0 驱动，UART RX 环形缓冲区大小为 2 KB，UART 事件队列大小/深度为20字节，并且获得队列
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);

    // 设置 UART 日志等级
    esp_log_level_set(HID_DEMO_TAG, ESP_LOG_INFO);
    // 设置 UART 引脚 (使用 UART0 默认引脚，不进行改变)
    uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	// 创建一个队列来处理 PC 端按键按下的数字
	key_flag = xQueueCreate(1, sizeof(key_flag));
    if (key_flag == NULL) {
        ESP_LOGI(HID_DEMO_TAG, "Failed to create key_flag queue");
        return;
    }
	xTaskCreate(uart_event_task, "uart_event_task", 3*1024, NULL, 12, NULL);
    xTaskCreate(&hid_demo_task, "hid_task", 1024 * 3, NULL, 5, NULL);
}

// HID 回调函数
static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch(event) {
        case ESP_HIDD_EVENT_REG_FINISH: {  // HID GATT 服务创建完成事件
            if (param->init_finish.state == ESP_GATT_OK) {
                // 为 BLE 设备指定一个随机的蓝牙地址
                esp_bd_addr_t rand_addr = {0x04,0x11,0x11,0x11,0x11,0x05};
				esp_err_t ret = esp_ble_gap_set_rand_addr(rand_addr);
                if (ret == ESP_OK) {
                    ESP_LOGI("BT_ADDR", "Random address set successfully.");
                } else {
                    ESP_LOGE("BT_ADDR", "Failed to set random address, error: %s", esp_err_to_name(ret));
                }
                esp_ble_gap_set_device_name(HIDD_DEVICE_NAME); // 设置广播名称
                esp_ble_gap_config_adv_data(&hidd_adv_data); // 设置 GAP 广播数据
            }
            break;
        }
        case ESP_BAT_EVENT_REG: {  // 电池电量 GATT 服务创建完成事件
            ESP_LOGI(HID_DEMO_TAG, "ESP_BAT_EVENT_REG");
            break;
        }
		case ESP_HIDD_EVENT_BLE_CONNECT: { // HID GATT 服务连接事件
            ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_CONNECT");
            if (remote_addr_num == 2) {
                ESP_LOGI(HID_DEMO_TAG, "remote_addr_num more than 2");
            } else {
                if ((remote_addr_num == 0) || (hid_conn_id[remote_addr_num-1] != param->connect.conn_id))
                {
                    hid_conn_id[remote_addr_num] = param->connect.conn_id;
                    ESP_LOGI(HID_DEMO_TAG, "hid_conn_id[%d] = %d", remote_addr_num, hid_conn_id[remote_addr_num]);
                    remote_addr_num++;
                }
            }
            break;
        }
        case ESP_HIDD_EVENT_BLE_DISCONNECT: { // HID GATT 服务断开连接事件
            sec_conn = false;
            ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_DISCONNECT");
            break;
        }
        default:
            break;
    }
    return;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    ESP_LOGI(HID_DEMO_TAG, "====>ESP_GAP_BLE_EVT %d          ", event);
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT: // 设置 GAP 广播数据完成事件
        ESP_LOGI(HID_DEMO_TAG, "ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT");
        adv_status = true;
        break;
     case ESP_GAP_BLE_SEC_REQ_EVT: // GAP 安全请求事件，部分设备 BLE HID 强制要求加密连接
        for(int i = 0; i < ESP_BD_ADDR_LEN; i++) {
             ESP_LOGD(HID_DEMO_TAG, "%x:",param->ble_security.ble_req.bd_addr[i]);
        }
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
	 break;
     case ESP_GAP_BLE_AUTH_CMPL_EVT: // 认证完成指示
        sec_conn = true;
        ESP_LOGD(HID_DEMO_TAG, "pair status = %s",param->ble_security.auth_cmpl.success ? "success" : "fail");
        if(!param->ble_security.auth_cmpl.success) { // 如果认证失败，打印失败原因
            ESP_LOGE(HID_DEMO_TAG, "fail reason = 0x%x",param->ble_security.auth_cmpl.fail_reason);
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT: //连接参数更新事件
        break;
    default:
        break;
    }
}


void app_main(void)
{
    esp_err_t ret;

    // NVS 初始化
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    // 释放经典蓝牙内存
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    // 初始化 Contorl 层
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s initialize controller failed\n", __func__);
        return;
    }
    // 启动 Contorl 层
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s enable controller failed\n", __func__);
        return;
    }
    // 初始化 Bluedroid
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed\n", __func__);
        return;
    }
    // 启动 Bluedroid
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed\n", __func__);
        return;
    }
    // 注册 GAP 回调函数
    esp_ble_gap_register_callback(gap_event_handler);
    // 注册 GATT 回调函数
    esp_hidd_register_callbacks(hidd_event_callback);

#if 1
    /* 将安全iocap & auth_req &密钥大小& init密钥响应密钥参数设置到堆栈 */
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_NO_BOND;  // 认证后与对端设备绑定
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;           // 设置 IO 能力为无输出无输入
    uint8_t key_size = 16;      // 密钥大小为7~16字节，这里设置为16字节
    // 设置初始化密钥为加密密钥和身份验证密钥，表示期望的主设备分配给从设备的密钥类型
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    // 设置响应密钥为加密密钥和身份验证密钥，表示从设备分配给主设备的密钥类型
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    // 设置安全参数
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    // 设置 IO 能力为无输出无输入
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    // 设置密钥大小为16字节
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    /* 如果您的BLE设备作为从设备，则init_key表示您希望主设备分配给您的密钥类型，
     * 而响应密钥表示您可以分配给主设备的密钥；
     * 如果您的BLE设备作为主设备，则响应密钥表示您希望从设备分配给您的密钥类型，
     * 而init_key表示您可以分配给从设备的密钥。 
    */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
#endif
	uart_init();
}

