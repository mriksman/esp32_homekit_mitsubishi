#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"              // For EventGroupHandle_t
#include "driver/gpio.h"
#include <sys/param.h>                          // MIN MAX
#include <string.h>                             // strcmp

#include "esp_event.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "esp_wifi.h"
#include "wifi.h"
#ifdef CONFIG_IDF_TARGET_ESP32
    #include "esp_netif.h"                      // Must be included before esp_wifi_default.h
    #include "esp_wifi_default.h"               // For esp_netif_create_default_wifi_sta
#endif

#include "button.h"
ESP_EVENT_DEFINE_BASE(BUTTON_EVENT);            // Convert button events into esp event system      

#include "led_status.h"

#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#ifdef CONFIG_IDF_TARGET_ESP8266
    #include "mdns.h"                           // ESP8266 RTOS SDK mDNS needs legacy STATUS_EVENT to be sent to it
#endif
ESP_EVENT_DEFINE_BASE(HOMEKIT_EVENT);           // Convert esp-homekit events into esp event system      

#include "esp_log.h"
static const char *TAG = "main";


#include "driver/uart.h"
#include "mitsubishi.h"

// Have set lwip sockets from 10 to 16 (maximum allowed)
//   5 for httpd (down from default of 7)
//   12 for HomeKit (up from 8)


// setting a value less than 10 causes 
//     timers.c:795 (prvProcessReceivedCommands)- assert failed!
// on ESP-IDF
static led_status_pattern_t ap_mode = LED_STATUS_PATTERN({1000, -1000});
static led_status_pattern_t not_paired = LED_STATUS_PATTERN({100, -100});
static led_status_pattern_t normal_mode = LED_STATUS_PATTERN({10, -9990});
static led_status_pattern_t identify = LED_STATUS_PATTERN({100, -100, 100, -350, 100, -100, 100, -350, 100, -100, 100, -350});
static led_status_t led_status;

static bool paired = false;


static homekit_accessory_t *accessories[2];

uint8_t calc_checksum(uint8_t* data, int len) {
    uint8_t sum = 0;
    for (int i = 0; i < len; i++) {
        sum += data[i];
    }
    return (0xfc - sum) & 0xff;
}

#define UART_BUF_SIZE (128)

uint8_t get_data(uint8_t *send_data, uint8_t *recv_data) 
{
    send_data[0]  = 0xfc;                    // header 0
    send_data[2]  = 0x01;                    // header 2
    send_data[3]  = 0x30;                    // header 3

    // packet length is data length + header (5) + checksum (1)
    uint8_t packet_len = send_data[4] + 6;      
    // checksum based on total packet length (less checksum position)
    uint8_t checksum_pos = packet_len - 1;
    send_data[checksum_pos] = calc_checksum(send_data, packet_len - 1); 

    ESP_LOG_BUFFER_HEXDUMP(TAG, send_data, packet_len, ESP_LOG_INFO);

    // clean out rx buffers
    uart_flush(UART_NUM_2);
    bzero(recv_data, UART_BUF_SIZE);

    uart_write_bytes(UART_NUM_2, (const char *) send_data, packet_len);

    // all uart_read_bytes have a 1 second timeout. apparently mitsubishi don't like 
    //  requests faster than this. this works well with the polling method.
    uint8_t len = uart_read_bytes(UART_NUM_2, recv_data, UART_BUF_SIZE, pdMS_TO_TICKS(5000));

    ESP_LOG_BUFFER_HEXDUMP(TAG, recv_data, len, ESP_LOG_INFO);

    return len;

}

static TaskHandle_t mitsubishi_poll_task_handle = NULL;

static void mitsubishi_poll_task(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 2400,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_driver_install(UART_NUM_2, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uint8_t *send_data = (uint8_t *) malloc(UART_BUF_SIZE);
    uint8_t *recv_data = (uint8_t *) malloc(UART_BUF_SIZE);

    // if there is any error in transmission, retry connection packet
    bool connected = false;

    // map to homekit addresses
    homekit_accessory_t *accessory              = accessories[0];
    homekit_service_t *thermostat_service       = homekit_service_by_type(accessory, HOMEKIT_SERVICE_THERMOSTAT);
    homekit_characteristic_t *current_temp      = homekit_service_characteristic_by_type(thermostat_service, HOMEKIT_CHARACTERISTIC_CURRENT_TEMPERATURE);
    homekit_characteristic_t *target_temp       = homekit_service_characteristic_by_type(thermostat_service, HOMEKIT_CHARACTERISTIC_TARGET_TEMPERATURE);
    homekit_characteristic_t *current_state     = homekit_service_characteristic_by_type(thermostat_service, HOMEKIT_CHARACTERISTIC_CURRENT_HEATING_COOLING_STATE);
    homekit_characteristic_t *target_state      = homekit_service_characteristic_by_type(thermostat_service, HOMEKIT_CHARACTERISTIC_TARGET_HEATING_COOLING_STATE);

    homekit_service_t *fan_service              = homekit_service_by_type(accessory, HOMEKIT_SERVICE_FAN2);
    homekit_characteristic_t *active            = homekit_service_characteristic_by_type(fan_service, HOMEKIT_CHARACTERISTIC_ACTIVE);
    homekit_characteristic_t *rotation_speed    = homekit_service_characteristic_by_type(fan_service, HOMEKIT_CHARACTERISTIC_ROTATION_SPEED);
    homekit_characteristic_t *fan_mode          = homekit_service_characteristic_by_type(fan_service, HOMEKIT_CHARACTERISTIC_TARGET_FAN_STATE);
    homekit_characteristic_t *swing_mode        = homekit_service_characteristic_by_type(fan_service, HOMEKIT_CHARACTERISTIC_SWING_MODE);

    // save retrieved settings from previous request, and compare to current wanted settings
    uint8_t power_map = 0x00; 
    uint8_t mode_map  = 0x00; 
    uint8_t temp_map  = 0x00; 
    uint8_t fan_map   = 0x00; 
    uint8_t vane_map  = 0x00;   

    // each time the vane is NOT in SWING mode, save the current value for use when disabling SWING mode
    uint8_t last_vane_setting  = 0x00; 


    while(1) {
        int len;

        // ************** Connect *****************

        if (!connected) {

ESP_LOGW(TAG, "connect attempt");

            send_data[1]  = 0x5a;                    // connect request
            send_data[4]  = CONNECT_PACKET_LEN - 6;  // data length (not including header (5) and checksum (1))    
            // zero data section
            bzero(&send_data[5], send_data[4]);
            // add data (always the same)
            send_data[5]  = 0xca;                    // 0x02 for current settings, 0x03 for room temps
            send_data[6]  = 0x01;                    // 0x02 for current settings, 0x03 for room temps

            len = get_data(send_data, recv_data);

            if (recv_data[1] == 0x7a) {
                // maybe checksum and header checks?
                connected = true;
            }
            else {
                // wait 10 seconds and try again
                ESP_LOGE(TAG, "failed to connect");
                vTaskDelay(pdMS_TO_TICKS(10000));
                continue;
            }
        }



        // ************** Set Changed Settings *****************

        // check if there is a requested change of settings, and clear the incremented value
        if (ulTaskNotifyTake(pdTRUE, 0)) {

ESP_LOGW(TAG, "set changed settings");

            send_data[1]  = 0x41;                    // info request
            send_data[4]  = INFO_PACKET_LEN - 6;     // data length (not including header (5) and checksum (1))    
            // zero data section
            bzero(&send_data[5], send_data[4]);
            send_data[5]  = 0x01;                    // 0x01 is 'set settings'


            // *** power recv_data[8]  and  operating_state recv_data[9]***
            uint8_t check_mode = AUTO_MODE;
            uint8_t check_power = POWER_ON;

            if (target_state->value.int_value == 0 && active->value.int_value == 1) {
                check_mode  = FAN_MODE;
            }
            else if (target_state->value.int_value == 1) {
                check_mode  = HEAT_MODE;
            }
            else if (target_state->value.int_value == 2) {
                check_mode  = COOL_MODE;
            }
            else if (target_state->value.int_value == 3) {
                check_mode  = AUTO_MODE;
            }
            else {
                check_power = POWER_OFF;
            }

            if (check_power != power_map) {
                send_data[6] |= POWER_CONTROL;
                send_data[8] = check_power;
            }
            if (check_power == POWER_ON &&
                check_mode != mode_map) {
                send_data[6] |= MODE_CONTROL;
                send_data[9] = check_mode;   
            }
            

            // *** target temperature recv_data[10] ***
            uint8_t check_temp = 31 - (uint8_t)target_temp->value.float_value;
            if (check_temp != temp_map) {
                send_data[6] |= TEMP_CONTROL;
                send_data[10] = check_temp;
            }


            // *** fan recv_data[11] ***
            uint8_t check_fan = FAN_AUTO;

            if (fan_mode->value.int_value == 1) { 
                check_fan = FAN_AUTO;
            }
            else if (rotation_speed->value.float_value == 20.0)  { 
                check_fan = FAN_QUIET;
            }
            else if (rotation_speed->value.float_value == 40.0) {  
                check_fan = FAN_SPEED1;
            }
            else if (rotation_speed->value.float_value == 60.0) { 
                check_fan = FAN_SPEED2;
            }
            else if (rotation_speed->value.float_value == 80.0) { 
                check_fan = FAN_SPEED3;
            }
            else if (rotation_speed->value.float_value == 100.0) { 
                check_fan = FAN_SPEED4;
            }

            if (check_fan != fan_map) {
                send_data[6] |= FAN_CONTROL;
                send_data[11] = check_fan;
            }


            // *** vane recv_data[12] ***
            uint8_t check_vane;

            if (swing_mode->value.int_value == 1) { 
                check_vane = VANE_SWING;
            }
            else { 
                check_vane = last_vane_setting;
            }

            if (check_vane != vane_map) {
                send_data[6] |= VANE_CONTROL;
                send_data[12] = check_vane;
            }


            // settings need updating
            if (send_data[6] != 0x00) {
                len = get_data(send_data, recv_data);

                if (recv_data[1] == 0x61) {
                    // settings successfully set

                }
                else {
                    connected = false;
                    continue;
                }
            }
        }


        // ************** Get Current Settings *****************

ESP_LOGW(TAG, "get current settings");

        send_data[1]  = 0x42;                    // info request
        send_data[4]  = INFO_PACKET_LEN - 6;     // data length (not including header (5) and checksum (1))    
        // zero data section
        bzero(&send_data[5], send_data[4]);
        send_data[5]  = 0x02;                    // 0x02 for current settings, 0x03 for room temp, 0x06 compressor status

        len = get_data(send_data, recv_data);

        // check if there are pending settings (but do not alter notification value). 
        //  if there are, restart loop and handle settings
        if (xTaskNotifyWait(0, 0, NULL, 0)) {
            continue;
        }

        if (len == INFO_PACKET_LEN) {
            // check header and checksum (recv_data[21]) is valid
            // check response recv_data[1] is 0x62
            // check response type recv_data[5] is 0x02 as well


            // *** power recv_data[8]  and  operating_state recv_data[9]***
            power_map = recv_data[8];
            mode_map = recv_data[9];

            uint8_t target_state_val = 0;
            bool target_state_valid = true;

            // treat DRY like COLD
            if (mode_map == DRY_MODE) {
                mode_map = COOL_MODE;
            }

            if (power_map == POWER_OFF || mode_map == FAN_MODE) {       // OFF or in FAN MODE
                target_state_val = 0;
            }
            else if (mode_map == HEAT_MODE) {                           // HEAT
                target_state_val = 1;
            }
            else if (mode_map == COOL_MODE) {                           // COOL or DRY
                target_state_val = 2;
            }
            else if (mode_map == AUTO_MODE) {                           // AUTO
                target_state_val = 3;
            }
            else {
                target_state_valid = false;
                ESP_LOGE(TAG, "unknown mode %d", mode_map);
            }
            
            if (target_state_valid && target_state->value.int_value != target_state_val) {
                target_state->value = HOMEKIT_UINT8(target_state_val);
                homekit_characteristic_notify(target_state, target_state->value);
            }

            // if it was FAN mode, we need to set FAN as ACTIVE (it would have turned off
            //  when the THERMOSTAT TARGET_HEATING_COOLING_STATE was switched off)
            if (power_map == POWER_ON && mode_map == FAN_MODE) { 
                if (active->value.int_value != 1) {
                    active->value = HOMEKIT_UINT8(1);
                    homekit_characteristic_notify(active, active->value);
                }
            }


            // *** target temperature recv_data[10] ***
            temp_map = recv_data[10];
            uint8_t convert_temp_map = (temp_map <= 0x0f) ? (31 - temp_map) : 0;
            if (target_temp->value.float_value != convert_temp_map) {
                target_temp->value = HOMEKIT_FLOAT(convert_temp_map);
                homekit_characteristic_notify(target_temp, target_temp->value);
            }


            // *** fan recv_data[11] ***
            float rotation_val = 0.0;
            bool rotation_val_valid = true;

            fan_map = recv_data[11];
            if (fan_map == FAN_AUTO) {                         // AUTO
                rotation_val_valid = false;
                if (fan_mode->value.int_value != 1) {
                    fan_mode->value = HOMEKIT_UINT8(1);
                    homekit_characteristic_notify(fan_mode, fan_mode->value);
                }
            }
            else if (fan_map == FAN_QUIET)  {                    // QUIET
                rotation_val = 20.0;
            }
            else if (fan_map == FAN_SPEED1) {                    // 1
                rotation_val = 40.0;
            }
            else if (fan_map == FAN_SPEED2) {                    // 2
                rotation_val = 60.0;
            }
            else if (fan_map == FAN_SPEED3) {                    // 3
                rotation_val = 80.0;
            }
            else if (fan_map == FAN_SPEED4) {                    // 4
                rotation_val = 100.0;
             }
            else {
                rotation_val_valid = false;
                ESP_LOGE(TAG, "unknown fan mode %d", fan_map);
            }   

            // setting the ROTATION SPEED will automatically set TARGET_FAN_STATE to disabled
            if (rotation_val_valid && rotation_speed->value.float_value != rotation_val) {
                rotation_speed->value = HOMEKIT_FLOAT(rotation_val);
                homekit_characteristic_notify(rotation_speed, rotation_speed->value);
            }


            // *** vane recv_data[12] ***
            uint8_t vane_state_val = 0;
            bool vane_state_valid = true;

            vane_map = recv_data[12];
            if (vane_map == VANE_SWING) { 
                vane_state_val = 1;
            }
            else if (vane_map <= 0x05) {
                vane_state_val = 0;
                last_vane_setting = vane_map;
            }   
            else {
                vane_state_valid = false;
                ESP_LOGE(TAG, "unknown vane mode %d", vane_map);
            }

            if (vane_state_valid && swing_mode->value.int_value != vane_state_val) {
                swing_mode->value = HOMEKIT_UINT8(vane_state_val);
                homekit_characteristic_notify(swing_mode, swing_mode->value);
            }


        }
        else {
            connected = false;
            continue;

        }
 

        // ************** Get Room Temperature *****************

ESP_LOGW(TAG, "get room temp");

        send_data[1]  = 0x42;                    // info request
        send_data[4]  = INFO_PACKET_LEN - 6;     // data length (not including header (5) and checksum (1))    
        // zero data section
        bzero(&send_data[5], send_data[4]);
        send_data[5]  = 0x03;                    // 0x02 for current settings, 0x03 for room temp, 0x06 compressor status

        len = get_data(send_data, recv_data);

        // check if there are pending settings (but do not alter notification value). 
        //  if there are, restart loop and handle settings
        if (xTaskNotifyWait(0, 0, NULL, 0)) {
            continue;
        }

        if (len == INFO_PACKET_LEN) {
 
            // if RAW temp value is available, use that (provides 0.5 resolution).
            float room_temp;
            if(recv_data[11] != 0x00) {
                room_temp = recv_data[11];
                room_temp -= 128;
                room_temp = room_temp / 2.0f;
            } else {
                uint8_t temp_val = recv_data[8];
                temp_val = temp_val <= 0x1f ? (temp_val + 10) : 0;
                room_temp = temp_val;
            }

            if (current_temp->value.float_value != room_temp) {
                current_temp->value = HOMEKIT_FLOAT(room_temp);
                homekit_characteristic_notify(current_temp, current_temp->value);
            }


        }
        else {
            connected = false;
            continue;
        }


        // ************** Get Compressor Status *****************

ESP_LOGW(TAG, "get compressor stat");

        send_data[1]  = 0x42;                    // info request
        send_data[4]  = INFO_PACKET_LEN - 6;     // data length (not including header (5) and checksum (1))    
        // zero data section
        bzero(&send_data[5], send_data[4]);
        send_data[5]  = 0x06;                    // 0x02 for current settings, 0x03 for room temp, 0x06 compressor status

        len = get_data(send_data, recv_data);

        // check if there are pending settings (but do not alter notification value). 
        //  if there are, restart loop and handle settings
        if (xTaskNotifyWait(0, 0, NULL, 0)) {
            continue;
        }

        if (len == INFO_PACKET_LEN) {
 

            uint8_t compressor_map = recv_data[9];

            uint8_t current_mode_val = 0;

            if (compressor_map == 0x00) {                   // IDLE/OFF
                current_mode_val = 0;
            }
            else if (target_temp->value.float_value > current_temp->value.float_value &&
                    (target_state->value.int_value == 1 || target_state->value.int_value == 3) ) {
                 current_mode_val = 1;
            }   
            else if (target_temp->value.float_value < current_temp->value.float_value &&
                    (target_state->value.int_value == 2 || target_state->value.int_value == 3) ) {
                current_mode_val = 2;
            }  
            else {
                current_mode_val = 0;
            }

            if (current_state->value.int_value != current_mode_val) {
                current_state->value = HOMEKIT_UINT8(current_mode_val);
                homekit_characteristic_notify(current_state, current_state->value);
            }

        }
        else {
            connected = false;
            continue;
        }

    }
}




void status_led_identify(homekit_value_t _value) {
    led_status_signal(led_status, &identify);
}


void state_change_on_callback(homekit_characteristic_t *_ch, homekit_value_t value, void *context) {

    ESP_LOGW(TAG, "%s", _ch->description);

    homekit_service_t *fan_service            = homekit_service_by_type(_ch->service->accessory, HOMEKIT_SERVICE_FAN2);
    homekit_characteristic_t *active          = homekit_service_characteristic_by_type(fan_service, HOMEKIT_CHARACTERISTIC_ACTIVE);
    homekit_characteristic_t *rotation_speed  = homekit_service_characteristic_by_type(fan_service, HOMEKIT_CHARACTERISTIC_ROTATION_SPEED);
    homekit_characteristic_t *fan_mode        = homekit_service_characteristic_by_type(fan_service, HOMEKIT_CHARACTERISTIC_TARGET_FAN_STATE);

    homekit_service_t *thermostat_service     = homekit_service_by_type(_ch->service->accessory, HOMEKIT_SERVICE_THERMOSTAT);
    homekit_characteristic_t *target_state    = homekit_service_characteristic_by_type(thermostat_service, HOMEKIT_CHARACTERISTIC_TARGET_HEATING_COOLING_STATE);


    // ******* Set states based on changes ******* //

    // See 'Air Purifier' in 'HAP Specification Non Commercial Version'
    // Fan state must track AC state
    if (strcmp(_ch->type, HOMEKIT_CHARACTERISTIC_TARGET_HEATING_COOLING_STATE) == 0) {
        if (target_state->value.int_value == 0 && active->value.int_value != 0) {
            active->value = HOMEKIT_UINT8(0);
            homekit_characteristic_notify(active, active->value);
        }  
        else if (target_state->value.int_value != 0 && active->value.int_value == 0) {
            active->value = HOMEKIT_UINT8(1);
            homekit_characteristic_notify(active, active->value);
        }  
    }
    // However, AC state only tracks fan when it is turned off, not when fan is turned on separately
    if (strcmp(_ch->type, HOMEKIT_CHARACTERISTIC_ACTIVE) == 0) {
        if (active->value.int_value == 0 && target_state->value.int_value != 0) {
            target_state->value = HOMEKIT_UINT8(0);
            homekit_characteristic_notify(target_state, target_state->value);
        }  
    }

    // if Fan turns on, if it's ROTATION SPEED is less than 20.0% ('QUIET'), set it to this minimum
    //  this happens when you stop the fan by using ROTATION SPEED and setting it to 0.0%
    if (strcmp(_ch->type, HOMEKIT_CHARACTERISTIC_ACTIVE) == 0) {
        if (active->value.int_value == 1 && rotation_speed->value.float_value < 20.0) {
            rotation_speed->value = HOMEKIT_FLOAT(20.0);
            homekit_characteristic_notify(rotation_speed, rotation_speed->value);
        }  
    }

    // If ROTATION SPEED is manually adjusted, then disable AUTO mode
    if (strcmp(_ch->type, HOMEKIT_CHARACTERISTIC_ROTATION_SPEED) == 0) {
        if (fan_mode->value.int_value == 1) {
            fan_mode->value = HOMEKIT_UINT8(0);
            homekit_characteristic_notify(fan_mode, fan_mode->value);
        }  
    }

}


// using .setter_ex, so update characteristic value and notify clients as per usual. but also send
//  task notification that settings have changed
void update_settings_callback(homekit_characteristic_t *_ch, homekit_value_t value) {

    ESP_LOGW(TAG, "update settings");

    _ch->value = value;
    homekit_characteristic_notify(_ch, value);

    // notify the polling task that there is a change of settings
    xTaskNotifyGive(mitsubishi_poll_task_handle);

}


// Need to call this function from a task different to the button_callback (executing in Tmr Svc)
// Have had occurrences when, if called from button_callback directly, the scheduler seems
// to lock up. 
static void start_ap_task(void * arg)
{
    ESP_LOGI(TAG, "Start AP task");
    start_ap_prov();
    vTaskDelete(NULL);
}


/* Event handler for Events */
static void main_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_AP_START || event_id == WIFI_EVENT_STA_DISCONNECTED) {
            led_status_set(led_status, &ap_mode);
        } else if (event_id == WIFI_EVENT_AP_STOP) {
            led_status_set(led_status, paired ? &normal_mode : &not_paired);
        } 
    } else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            wifi_mode_t wifi_mode;
            esp_wifi_get_mode(&wifi_mode);
            if (wifi_mode == WIFI_MODE_STA) {
                led_status_set(led_status, paired ? &normal_mode : &not_paired);
            } else {
                led_status_set(led_status, &ap_mode);
            }
        }
    } else if (event_base == HOMEKIT_EVENT) {
        if (event_id == HOMEKIT_EVENT_CLIENT_CONNECTED) {
            ESP_LOGI(TAG, "HOMEKIT_EVENT_CLIENT_CONNECTED");
        }
        else if (event_id == HOMEKIT_EVENT_CLIENT_DISCONNECTED) {
            ESP_LOGI(TAG, "HOMEKIT_EVENT_CLIENT_DISCONNECTED");
        }
        else if (event_id == HOMEKIT_EVENT_PAIRING_ADDED || event_id == HOMEKIT_EVENT_PAIRING_REMOVED) {
            ESP_LOGI(TAG, "HOMEKIT_EVENT_PAIRING_ADDED or HOMEKIT_EVENT_PAIRING_REMOVED");
            paired = homekit_is_paired();
            led_status_set(led_status, paired ? &normal_mode : &not_paired);
        }
    } else if (event_base == BUTTON_EVENT) {

        if (event_id == BUTTON_EVENT_UP) {

        }
        else if (event_id == BUTTON_EVENT_DOWN) {

        }

        else if (event_id == BUTTON_EVENT_DOWN_HOLD) {

        }
        else if (event_id == BUTTON_EVENT_UP_HOLD) {

        }

        else if (event_id == BUTTON_EVENT_LONG_PRESS) {
            ESP_LOGI(TAG, "button long press event. start AP");  
            //start_ap_prov();        
            xTaskCreate(&start_ap_task, "Start AP", 4096, NULL, tskIDLE_PRIORITY, NULL);
        }
        else {

            if (event_id == 1) {

            } 

            else if (event_id == 4) {
                ESP_LOGW(TAG, "HEAP %d",  heap_caps_get_free_size(MALLOC_CAP_8BIT));

                char buffer[600];
                vTaskList(buffer);
                ESP_LOGI(TAG, "\n%s", buffer);
            } 
        }
    }
}

void homekit_on_event(homekit_event_t event) {
    esp_event_post(HOMEKIT_EVENT, event, NULL, sizeof(NULL), 10);
}
void button_callback(button_event_t event, void* context) {
    // esp_event_post sends a pointer to a COPY of the data.
    esp_event_post(BUTTON_EVENT, event, context, sizeof(uint8_t), 10);
}


homekit_server_config_t config = {
    .accessories = accessories,
    .password = "111-11-111",
    .on_event = homekit_on_event,
};

void init_accessory() {
    uint8_t macaddr[6];
    esp_read_mac(macaddr, ESP_MAC_WIFI_SOFTAP);
    int name_len = snprintf( NULL, 0, "esp-%02x%02x%02x", macaddr[3], macaddr[4], macaddr[5] );
    char *name_value = malloc(name_len + 1);
    snprintf( name_value, name_len + 1, "esp-%02x%02x%02x", macaddr[3], macaddr[4], macaddr[5] ); 

    // ACCESSORY_INFORMATION, THERMOSTAT, FAN, and NULL
    homekit_service_t* services[4]; 
    homekit_service_t** s = services;

    *(s++) = NEW_HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
        NEW_HOMEKIT_CHARACTERISTIC(NAME, name_value),
        NEW_HOMEKIT_CHARACTERISTIC(MANUFACTURER, "MikeKit"),
        NEW_HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "037A3BBBF29E"),
        NEW_HOMEKIT_CHARACTERISTIC(MODEL, "MitsubishiAC"),
        NEW_HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.1"),
        NEW_HOMEKIT_CHARACTERISTIC(IDENTIFY, status_led_identify),
        NULL
    });


    *(s++) = NEW_HOMEKIT_SERVICE(THERMOSTAT, .primary=true, .characteristics=(homekit_characteristic_t*[]) {
        NEW_HOMEKIT_CHARACTERISTIC(NAME, "MitsubishiAC"),
        NEW_HOMEKIT_CHARACTERISTIC(
            CURRENT_HEATING_COOLING_STATE, 0
        ),
        NEW_HOMEKIT_CHARACTERISTIC(
            TARGET_HEATING_COOLING_STATE, 0,
            .setter_ex = update_settings_callback,
            .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(state_change_on_callback)
        ),
        NEW_HOMEKIT_CHARACTERISTIC(
            CURRENT_TEMPERATURE, 20
        ),
        NEW_HOMEKIT_CHARACTERISTIC(
            TARGET_TEMPERATURE, 20.0,
            .min_value = (float[]) {16},
            .max_value = (float[]) {31},
            .min_step = (float[]) {1.0},
            .setter_ex = update_settings_callback,
        ),
        NEW_HOMEKIT_CHARACTERISTIC(
            TEMPERATURE_DISPLAY_UNITS, 0
        ),
        NULL
    });

    *(s++) = NEW_HOMEKIT_SERVICE(FAN2, .characteristics=(homekit_characteristic_t*[]) {
        NEW_HOMEKIT_CHARACTERISTIC(NAME, "Fan Control"),
        NEW_HOMEKIT_CHARACTERISTIC(
            ACTIVE, false,
            .setter_ex = update_settings_callback,
            .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(state_change_on_callback)
        ),
        NEW_HOMEKIT_CHARACTERISTIC(
            TARGET_FAN_STATE, 1,                //	0 Manual	1 Auto
            .setter_ex = update_settings_callback,
            .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(state_change_on_callback)
        ),
        NEW_HOMEKIT_CHARACTERISTIC(
            ROTATION_SPEED, 20.0,               // Map this to {"QUIET", "1", "2", "3", "4"}
            .value = HOMEKIT_FLOAT_(20.0),      // going to 0.0 will cause ACTIVE = OFF
            .min_step = (float[]) {20.0},       // Map this to { 20.0, 40.0, 60.0, 80.0, 100.0}
            .setter_ex = update_settings_callback,
            .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(state_change_on_callback)
        ),
        NEW_HOMEKIT_CHARACTERISTIC(
            SWING_MODE, 0,                      // 0 Swing disabled       1 Swing enabled
            .setter_ex = update_settings_callback,
        ),
        NULL
    });

    *(s++) = NULL;

    accessories[0] = NEW_HOMEKIT_ACCESSORY(.category=homekit_accessory_category_thermostat, .services=services);
    accessories[1] = NULL;

}




void app_main(void)
{
    esp_err_t err;

    esp_log_level_set("*", ESP_LOG_DEBUG);      
    esp_log_level_set("httpd", ESP_LOG_INFO); 
    esp_log_level_set("httpd_uri", ESP_LOG_INFO);    
    esp_log_level_set("httpd_txrx", ESP_LOG_INFO);     
    esp_log_level_set("httpd_sess", ESP_LOG_INFO);
    esp_log_level_set("httpd_parse", ESP_LOG_INFO);  
    esp_log_level_set("vfs", ESP_LOG_INFO);     
    esp_log_level_set("esp_timer", ESP_LOG_INFO);  
    esp_log_level_set("esp_netif_lwip", ESP_LOG_INFO);     
 
    // Initialize NVS. 
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {     // can happen if truncated/partition size changed
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(esp_event_loop_create_default());

   // esp_event_handler_register is being deprecated
    #ifdef CONFIG_IDF_TARGET_ESP32
        ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, main_event_handler, NULL, NULL));
        ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, main_event_handler, NULL, NULL));
        ESP_ERROR_CHECK(esp_event_handler_instance_register(HOMEKIT_EVENT, ESP_EVENT_ANY_ID, main_event_handler, NULL, NULL));
        ESP_ERROR_CHECK(esp_event_handler_instance_register(BUTTON_EVENT, ESP_EVENT_ANY_ID, main_event_handler, NULL, NULL));
    #elif CONFIG_IDF_TARGET_ESP8266
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, main_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, main_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(HOMEKIT_EVENT, ESP_EVENT_ANY_ID, main_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(BUTTON_EVENT, ESP_EVENT_ANY_ID, main_event_handler, NULL));
    #endif

    led_status = led_status_init(2, true);

    wifi_init();

    // button configuration
    button_config_t button_config = {
        .active_level = BUTTON_ACTIVE_LOW,
        .repeat_press_timeout = 300,
        .long_press_time = 10000,
    };
    button_create(0, button_config, button_callback, NULL);


    init_accessory();
    homekit_server_init(&config);
    paired = homekit_is_paired();


    xTaskCreate(mitsubishi_poll_task, "mits_task", 4096, NULL, 10, &mitsubishi_poll_task_handle);

}
