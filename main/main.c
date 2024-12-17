/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "esp_log.h"
#include "esp_hidd_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_bt.h"
#include "esp_err.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_gap_bt_api.h"
#include <string.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"


static const char *TAG_ = "BUTTON_PRESS_HANDLER";


// Press types
#define PRESS_TYPE_SINGLE    1
#define PRESS_TYPE_DOUBLE    2
#define PRESS_TYPE_TRIPLE    3
#define PRESS_TYPE_QUADRUPLE 4


#define btn1 18
#define btn2 19
#define btn3 21
#define btn4 22
#define btn5 23

uint8_t  button_array[5] = {
    btn1,
    btn2,
    btn3,
    btn4,
    btn5
};

    uint64_t press_start_time[5] = {0};
    bool button_was_pressed[5] = {false};

#define BUTTON_DEBOUNCE_DELAY_MS 50      // Debounce delay
#define BUTTON_PRESS_TIMEOUT_MS 500      // Max time to detect multiple presses
// GPIO setup
static void configure_buttons(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = ((1ULL << btn1) |
                         (1ULL << btn2) |
                         (1ULL << btn3) |
                         (1ULL << btn4) |
                         (1ULL << btn5)),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);
    //ESP_LOGI(TAG, "GPIO buttons configured.");
}

// Structure to track button presses
typedef struct {
    uint8_t which_button;
    uint8_t enable;
    uint8_t press_count;       // Number of presses
    uint64_t last_press_time;  // Time of the last button press
} button_press_t;

  button_press_t button_press[5];




#define REPORT_SIZE 2

typedef struct {
    esp_hidd_app_param_t app_param;
    esp_hidd_qos_param_t both_qos;
    uint8_t protocol_mode;
    SemaphoreHandle_t multiMedia_mutex;
    TaskHandle_t multiMedia_task_hdl;
    uint8_t buffer[REPORT_SIZE];

} local_param_t;

static local_param_t s_local_param = {0};


uint8_t hid_media_descriptor[] = {
    0x05, 0x0C,       // Usage Page (Consumer Devices)
    0x09, 0x01,       // Usage (Consumer Control)
    0xA1, 0x01,       // Collection (Application)

    // Report ID 1: Play/Pause
    0x85, 0x01,       // Report ID (1)
    0x09, 0xCD,       // Usage (Play/Pause)
    0x15, 0x00,       // Logical Minimum (0)
    0x25, 0x01,       // Logical Maximum (1)
    0x75, 0x01,       // Report Size (1)
    0x95, 0x01,       // Report Count (1)
    0x81, 0x02,       // Input (Data,Var,Abs)
    0x95, 0x07,       // Report Count (7 padding bits)
    0x75, 0x01,       // Report Size (1)
    0x81, 0x03,       // Input (Cnst,Var,Abs)

    // Report ID 2: Next Track, Previous Track, Track from Beginning
    0x85, 0x02,       // Report ID (2)
    0x09, 0xB5,       // Usage (Next Track)
    0x09, 0xB6,       // Usage (Previous Track)
    0x09, 0xB7,       // Usage (Track from Beginning)
    0x15, 0x00,       // Logical Minimum (0)
    0x25, 0x01,       // Logical Maximum (1)
    0x75, 0x01,       // Report Size (1)
    0x95, 0x03,       // Report Count (3)
    0x81, 0x02,       // Input (Data,Var,Abs)
    0x95, 0x05,       // Report Count (5 padding bits)
    0x75, 0x01,       // Report Size (1)
    0x81, 0x03,       // Input (Cnst,Var,Abs)

    // Report ID 3: Volume Control
    0x85, 0x03,       // Report ID (3)
    0x09, 0xE9,       // Usage (Volume Up)
    0x09, 0xEA,       // Usage (Volume Down)
    0x09, 0xE2,       // Usage (Mute/Unmute)
    0x15, 0x00,       // Logical Minimum (0)
    0x25, 0x01,       // Logical Maximum (1)
    0x75, 0x01,       // Report Size (1)
    0x95, 0x03,       // Report Count (3)
    0x81, 0x02,       // Input (Data,Var,Abs)
    0x95, 0x05,       // Report Count (5 padding bits)
    0x75, 0x01,       // Report Size (1)
    0x81, 0x03,       // Input (Cnst,Var,Abs)

    // Report ID 4: Noise Cancellation
    0x85, 0x04,       // Report ID (4)
    0x09, 0xC0,       // Usage (Noise Cancellation Toggle)
    0x15, 0x00,       // Logical Minimum (0)
    0x25, 0x01,       // Logical Maximum (1)
    0x75, 0x01,       // Report Size (1)
    0x95, 0x01,       // Report Count (1)
    0x81, 0x02,       // Input (Data,Var,Abs)
    0x95, 0x07,       // Report Count (7 padding bits)
    0x75, 0x01,       // Report Size (1)
    0x81, 0x03,       // Input (Cnst,Var,Abs)


    0xC0              // End Collection
};
const int hid_media_descriptor_len = sizeof(hid_media_descriptor);


void send_hid_report(uint8_t report_id,uint8_t key,uint32_t delay) {
        xSemaphoreTake(s_local_param.multiMedia_mutex, portMAX_DELAY);

  s_local_param.buffer[0] = key;
    s_local_param.buffer[1] = 0;
    esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, report_id, REPORT_SIZE, s_local_param.buffer);
    vTaskDelay(pdMS_TO_TICKS(delay)); // Simulate key press duration

    s_local_param.buffer[0]= 0; // Key release
    esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, report_id, REPORT_SIZE, s_local_param.buffer);
        xSemaphoreGive(s_local_param.multiMedia_mutex);
}

void play_pause_media() {
   send_hid_report(1, 0xCD,100); // Report ID 5, Play/Pause
}

void next_track(void) {
    send_hid_report(2, 0xB5,100); // Next Track
}

void previous_track(void) {
    send_hid_report(2, 0xB6,100); // Previous Track
}

void track_from_beginning(void) {
    send_hid_report(2, 0xB7,500); // Track from Beginning
}


void volume_up(void) {
    send_hid_report(3, 0xE9,50); // Volume Up
}

void volume_down(void) {
    send_hid_report(3, 0xEA,50); // Volume Down
}

void mute() {

    send_hid_report(3, 0xE2,2000); // Mute/Unmute


}


void toggle_noise_cancellation(void) {
    send_hid_report(4, 0xC0,100); // Noise Cancellation Toggle
}


void activate_voice_assistant(void) {
    send_hid_report(1, 0xCD,500); // Voice Assistant Activation
}
/**
 * @brief Integrity check of the report ID and report type for GET_REPORT request from HID host.
 *        Boot Protocol Mode requires report ID. For Report Protocol Mode, when the report descriptor
 *        does not declare report ID Global ITEMS, the report ID does not exist in the GET_REPORT request,
 *        and a value of 0 for report_id will occur in ESP_HIDD_GET_REPORT_EVT callback parameter.
 */
bool check_report_id_type(uint8_t report_id, uint8_t report_type)
{
    bool ret = false;
    xSemaphoreTake(s_local_param.multiMedia_mutex, portMAX_DELAY);
    do {
        if (report_type != ESP_HIDD_REPORT_TYPE_INPUT) {
            break;
        }
        if (s_local_param.protocol_mode == ESP_HIDD_BOOT_MODE) {
            if (report_id == ESP_HIDD_BOOT_REPORT_ID_KEYBOARD) {
                ret = true;
                break;
            }
        } else {
            if (report_id == 0) {
                ret = true;
                break;
            }
        }
    } while (0);

    if (!ret) {
        if (s_local_param.protocol_mode == ESP_HIDD_BOOT_MODE) {
            esp_bt_hid_device_report_error(ESP_HID_PAR_HANDSHAKE_RSP_ERR_INVALID_REP_ID);
        } else {
            esp_bt_hid_device_report_error(ESP_HID_PAR_HANDSHAKE_RSP_ERR_INVALID_REP_ID);
        }
    }
    xSemaphoreGive(s_local_param.multiMedia_mutex);
    return ret;
}



// Function to handle button presses based on the press count
static void handle_button_press(button_press_t button_data) {
    switch (button_data.press_count) {
        case PRESS_TYPE_SINGLE:
            ESP_LOGI(TAG_, "Single press detected");
            switch(button_data.which_button)
            {
                case btn1:
                {
                    play_pause_media();
                    break;
                }
                case btn2:
                {
                    next_track();
                    break;
                }
                case btn3:
                {
                    track_from_beginning();
                    break;
                }
                case btn4:
                {
                    volume_up();
                    break;
                }
                case btn5:
                {
                    volume_down();
                    break;
                }

            }
            
            break;
        case PRESS_TYPE_DOUBLE:
            ESP_LOGI(TAG_, "Double press detected");
              switch(button_data.which_button)
            {
                case btn1:
                {
                    mute();
                    break;
                }
                case btn3:
                {
                    previous_track();
                    break;
                }
                case btn4:
                case btn2:
                case btn5:
                default:
                {
                     ESP_LOGI(TAG_, "Nothing to do"); 
                }

            }
            
            break;
        case PRESS_TYPE_TRIPLE:
            ESP_LOGI(TAG_, "Triple press detected");
                          switch(button_data.which_button)
            {
                case btn1:
                {
                    toggle_noise_cancellation();
                    break;
                }
                case btn3:
                case btn4:
                case btn2:
                case btn5:
                default:
                {
                     ESP_LOGI(TAG_, "Nothing to do"); 
                }

            }
            break;
        case PRESS_TYPE_QUADRUPLE:
            ESP_LOGI(TAG_, "Quadruple press detected");
                                      switch(button_data.which_button)
            {
                case btn1:
                {
                    activate_voice_assistant();
                    break;
                }
                case btn3:
                case btn4:
                case btn2:
                case btn5:
                default:
                {
                     ESP_LOGI(TAG_, "Nothing to do"); 
                }

            }
            break;
        default:
            ESP_LOGW(TAG_,"Unhandled button presses");
            break;
    }
}

// Check if a button is pressed (LOW logic level)
static bool is_button_pressed(gpio_num_t gpio) {
    return (gpio_get_level(gpio) == 0);
}


static void button_loop(uint8_t index)
{
  if (is_button_pressed(button_array[index])) {
        if (!button_was_pressed[index]) {
            button_was_pressed[index] = true;
            press_start_time[index] = xTaskGetTickCount(); // Get current tick count
             button_press[index].last_press_time = xTaskGetTickCount();
            button_press[index].press_count++;
        }
    } else {
        if (button_was_pressed[index]) {
          
            uint64_t current_time = xTaskGetTickCount(); // Get current tick count
            button_press[index].enable = 1;
            button_press[index].last_press_time = current_time;
              button_was_pressed[index] = false;
            // // Check if debounce delay has passed
            // if ((current_time - press_start_time[index]) * portTICK_PERIOD_MS > BUTTON_DEBOUNCE_DELAY_MS) {
 
            // }
        }
    }

    // Timeout to determine the press count
    uint64_t current_time = xTaskGetTickCount();
    if (button_press[index].enable &&
        ((current_time - button_press[index].last_press_time) * portTICK_PERIOD_MS > BUTTON_PRESS_TIMEOUT_MS)) {
          button_press[index].enable = 0;
        
        // Process press count
        button_press[index].which_button = button_array[index];
        handle_button_press(button_press[index]);
        ESP_LOGI(TAG_, "Button GPIO %d: %d press detected.", button_array[index], button_press[index].press_count);

        // Reset press count
        button_press[index].press_count = 0;
    }
}



void multiMedia_task(void *pvParameters)
{
    const char *TAG = "multiMedia_task";

    ESP_LOGI(TAG, "starting");

    configure_buttons();
    for (;;) {



for(int i = 0; i < 5 ; i++)
    {
           button_loop(i);
    }
        vTaskDelay(pdMS_TO_TICKS(100)); // Polling interval

    }
}

static void print_bt_address(void)
{
    const char *TAG = "bt_address";
    const uint8_t *bd_addr;

    bd_addr = esp_bt_dev_get_address();
    ESP_LOGI(TAG, "my bluetooth address is %02X:%02X:%02X:%02X:%02X:%02X",
             bd_addr[0], bd_addr[1], bd_addr[2], bd_addr[3], bd_addr[4], bd_addr[5]);
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    const char *TAG = "esp_bt_gap_cb";
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT: {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT: {
        ESP_LOGI(TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
#endif
    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d", param->mode_chg.mode);
        break;
    default:
        ESP_LOGI(TAG, "event: %d", event);
        break;
    }
    return;
}

void bt_app_task_start_up(void)
{
    s_local_param.multiMedia_mutex = xSemaphoreCreateMutex();
        memset(s_local_param.buffer, 0, REPORT_SIZE);
    xTaskCreate(multiMedia_task, "multiMedia_task", 2 * 1024, NULL, configMAX_PRIORITIES - 3, &s_local_param.multiMedia_task_hdl);
    return;
}

void bt_app_task_shut_down(void)
{
    if (s_local_param.multiMedia_task_hdl) {
        vTaskDelete(s_local_param.multiMedia_task_hdl);
        s_local_param.multiMedia_task_hdl = NULL;
    }

    if (s_local_param.multiMedia_mutex) {
        vSemaphoreDelete(s_local_param.multiMedia_mutex);
        s_local_param.multiMedia_mutex = NULL;
    }
    return;
}

void esp_bt_hidd_cb(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    static const char *TAG = "esp_bt_hidd_cb";
    switch (event) {
    case ESP_HIDD_INIT_EVT:
        if (param->init.status == ESP_HIDD_SUCCESS) {
            ESP_LOGI(TAG, "setting hid parameters");
            esp_bt_hid_device_register_app(&s_local_param.app_param, &s_local_param.both_qos, &s_local_param.both_qos);
        } else {
            ESP_LOGE(TAG, "init hidd failed!");
        }
        break;
    case ESP_HIDD_DEINIT_EVT:
        break;
    case ESP_HIDD_REGISTER_APP_EVT:
        if (param->register_app.status == ESP_HIDD_SUCCESS) {
            ESP_LOGI(TAG, "setting hid parameters success!");
            ESP_LOGI(TAG, "setting to connectable, discoverable");
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            if (param->register_app.in_use && param->register_app.bd_addr != NULL) {
                ESP_LOGI(TAG, "start virtual cable plug!");
                esp_bt_hid_device_connect(param->register_app.bd_addr);
            }
        } else {
            ESP_LOGE(TAG, "setting hid parameters failed!");
        }
        break;
    case ESP_HIDD_UNREGISTER_APP_EVT:
        if (param->unregister_app.status == ESP_HIDD_SUCCESS) {
            ESP_LOGI(TAG, "unregister app success!");
        } else {
            ESP_LOGE(TAG, "unregister app failed!");
        }
        break;
    case ESP_HIDD_OPEN_EVT:
        if (param->open.status == ESP_HIDD_SUCCESS) {
            if (param->open.conn_status == ESP_HIDD_CONN_STATE_CONNECTING) {
                ESP_LOGI(TAG, "connecting...");
            } else if (param->open.conn_status == ESP_HIDD_CONN_STATE_CONNECTED) {
                ESP_LOGI(TAG, "connected to %02x:%02x:%02x:%02x:%02x:%02x", param->open.bd_addr[0],
                         param->open.bd_addr[1], param->open.bd_addr[2], param->open.bd_addr[3], param->open.bd_addr[4],
                         param->open.bd_addr[5]);
                bt_app_task_start_up();
                ESP_LOGI(TAG, "making self non-discoverable and non-connectable.");
                esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            } else {
                ESP_LOGE(TAG, "unknown connection status");
            }
        } else {
            ESP_LOGE(TAG, "open failed!");
        }
        break;
    case ESP_HIDD_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_CLOSE_EVT");
        if (param->close.status == ESP_HIDD_SUCCESS) {
            if (param->close.conn_status == ESP_HIDD_CONN_STATE_DISCONNECTING) {
                ESP_LOGI(TAG, "disconnecting...");
            } else if (param->close.conn_status == ESP_HIDD_CONN_STATE_DISCONNECTED) {
                ESP_LOGI(TAG, "disconnected!");
                bt_app_task_shut_down();
                ESP_LOGI(TAG, "making self discoverable and connectable again.");
                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            } else {
                ESP_LOGE(TAG, "unknown connection status");
            }
        } else {
            ESP_LOGE(TAG, "close failed!");
        }
        break;
    case ESP_HIDD_SEND_REPORT_EVT:
        if (param->send_report.status == ESP_HIDD_SUCCESS) {
            ESP_LOGI(TAG, "ESP_HIDD_SEND_REPORT_EVT id:0x%02x, type:%d", param->send_report.report_id,
                     param->send_report.report_type);
        } else {
            ESP_LOGE(TAG, "ESP_HIDD_SEND_REPORT_EVT id:0x%02x, type:%d, status:%d, reason:%d",
                     param->send_report.report_id, param->send_report.report_type, param->send_report.status,
                     param->send_report.reason);
        }
        break;
    case ESP_HIDD_REPORT_ERR_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_REPORT_ERR_EVT");
        break;
    case ESP_HIDD_GET_REPORT_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_GET_REPORT_EVT id:0x%02x, type:%d, size:%d", param->get_report.report_id,
                 param->get_report.report_type, param->get_report.buffer_size);
        if (check_report_id_type(param->get_report.report_id, param->get_report.report_type)) {
            uint8_t report_id;
            uint16_t report_len;
            if (s_local_param.protocol_mode == ESP_HIDD_REPORT_MODE) {
                report_id = 0;
                report_len = REPORT_SIZE;
            } else {
                // Boot Mode
                report_id = ESP_HIDD_BOOT_REPORT_ID_KEYBOARD;
                report_len = ESP_HIDD_BOOT_REPORT_SIZE_KEYBOARD - 1;
            }
            xSemaphoreTake(s_local_param.multiMedia_mutex, portMAX_DELAY);
            esp_bt_hid_device_send_report(param->get_report.report_type, report_id, report_len, s_local_param.buffer);
            xSemaphoreGive(s_local_param.multiMedia_mutex);
        } else {
            ESP_LOGE(TAG, "check_report_id failed!");
        }
        break;
    case ESP_HIDD_SET_REPORT_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_SET_REPORT_EVT");
        break;
    case ESP_HIDD_SET_PROTOCOL_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_SET_PROTOCOL_EVT");
        if (param->set_protocol.protocol_mode == ESP_HIDD_BOOT_MODE) {
            ESP_LOGI(TAG, "  - boot protocol");
        } else if (param->set_protocol.protocol_mode == ESP_HIDD_REPORT_MODE) {
            ESP_LOGI(TAG, "  - report protocol");
        }
        xSemaphoreTake(s_local_param.multiMedia_mutex, portMAX_DELAY);
        s_local_param.protocol_mode = param->set_protocol.protocol_mode;
        xSemaphoreGive(s_local_param.multiMedia_mutex);
        break;
    case ESP_HIDD_INTR_DATA_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_INTR_DATA_EVT");
        break;
    case ESP_HIDD_VC_UNPLUG_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_VC_UNPLUG_EVT");
        if (param->vc_unplug.status == ESP_HIDD_SUCCESS) {
            if (param->close.conn_status == ESP_HIDD_CONN_STATE_DISCONNECTED) {
                ESP_LOGI(TAG, "disconnected!");
                bt_app_task_shut_down();
                ESP_LOGI(TAG, "making self discoverable and connectable again.");
                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            } else {
                ESP_LOGE(TAG, "unknown connection status");
            }
        } else {
            ESP_LOGE(TAG, "close failed!");
        }
        break;
    default:
        break;
    }
}

void app_main(void)
{
    const char *TAG = "app_main";
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "initialize controller failed: %s\n", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(TAG, "enable controller failed: %s\n", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(TAG, "initialize bluedroid failed: %s\n", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(TAG, "enable bluedroid failed: %s\n", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(TAG, "gap register failed: %s\n", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "setting device name");
    esp_bt_dev_set_device_name("MediaControl_Device");

    ESP_LOGI(TAG, "setting cod major, peripheral");
    esp_bt_cod_t cod;
    cod.major = ESP_BT_COD_MAJOR_DEV_PERIPHERAL;
    esp_bt_gap_set_cod(cod, ESP_BT_SET_COD_MAJOR_MINOR);

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // Initialize HID SDP information and L2CAP parameters.
    // to be used in the call of `esp_bt_hid_device_register_app` after profile initialization finishes
    do {
        s_local_param.app_param.name = "Keyboard";
        s_local_param.app_param.description = "Keyboard_Working";
        s_local_param.app_param.provider = "ESP32";
        s_local_param.app_param.subclass = ESP_HID_CLASS_KBD;
        s_local_param.app_param.desc_list = hid_media_descriptor;
        s_local_param.app_param.desc_list_len = hid_media_descriptor_len;

        memset(&s_local_param.both_qos, 0, sizeof(esp_hidd_qos_param_t)); // don't set the qos parameters
    } while (0);

    // Report Protocol Mode is the default mode, according to Bluetooth HID specification
    s_local_param.protocol_mode = ESP_HIDD_REPORT_MODE;

    ESP_LOGI(TAG, "register hid device callback");
    esp_bt_hid_device_register_callback(esp_bt_hidd_cb);

    ESP_LOGI(TAG, "starting hid device");
    esp_bt_hid_device_init();

#if (CONFIG_BT_SSP_ENABLED == true)
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    print_bt_address();
    ESP_LOGI(TAG, "exiting");
}
