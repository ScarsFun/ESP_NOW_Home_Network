/**
 * @defgroup   ESPNOW_NODE ESPNOW NODE
 *
 * @brief      This file implements ESPNOW sender/receiver.
 *
 * @author     FART ELECTRONICA
 * @date       2023
 */

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "driver/gpio.h"
#include "esp_crc.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/idf_additions.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_chip_info.h"
#include "esp_app_desc.h"
#include "esp_timer.h"
#include "my_espnow.h"

#include "ESPNOW_ota.h"


char *l_my_node_id; //  ID of the node stored in NVS
char *l_board_type; //  Hardware module type stored in NVS
uint8_t gw_MAC[ESP_NOW_ETH_ALEN] = {};
static const uint8_t broadcast_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

#define ESPNOW_QUEUE_SIZE           10
#define WIFI_MAX_CHANNEL            13


/*led pin definition and pin mask*/
#define GPIO_OUTPUT_LED CONFIG_LED_GPIO
#define GPIO_OUTPUT_PIN_SEL (1ULL << GPIO_OUTPUT_LED)
#define GPIO_INPUT_SWBUTTON CONFIG_BUTTON_GPIO
#define GPIO_INPUT_SWBUTTON_SEL (1ULL << GPIO_INPUT_SWBUTTON)

espnow_send_param_t *send_param;

uint8_t espnow_send_errors = 0;

/*Queue handle*/
QueueHandle_t espnow_queue, xQueueOtaData;
SemaphoreHandle_t xSemaphoreData, xSemaphoreEspnow;
EventGroupHandle_t scan_ch_group;
EventGroupHandle_t ack_flag;
EventGroupHandle_t ota_status_group;
TaskHandle_t esp_now_task_handle = NULL;
TaskHandle_t button_task_handle = NULL;
TaskHandle_t scan_gw_channel_handle = NULL;
TaskHandle_t dummy_task_handle = NULL;
TaskHandle_t ota_firmware_store_task_handle = NULL;
//UBaseType_t uxHighWaterMark;

void send_temperature_task(void *);
void sw_button_task(void *);
void scan_gw_channel(void *);
bool pubblish_mqqt_message(char *);

#define MAX_ESPNOW_SEND_ERRORS 4

#define BROADCAST_CONNECTED_BIT BIT0
#define ACK_RECEIVED_BIT BIT0

/**
 * @brief  initialize gpio pin
 * @param  None
 * @retval None
 */
void gpio_init(void)
{
    gpio_config_t pin_conf;

    /*configure output led pin*/
    pin_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    pin_conf.mode = GPIO_MODE_OUTPUT;
    pin_conf.pull_up_en = false;
    pin_conf.pull_down_en = false;
    pin_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&pin_conf);

    /*configure input switch button*/
    pin_conf.pin_bit_mask = GPIO_INPUT_SWBUTTON_SEL;
    pin_conf.mode = GPIO_MODE_INPUT;
    pin_conf.pull_up_en = true;
    pin_conf.pull_down_en = false;
    pin_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&pin_conf);
}



/**
 * @brief      Pubblish MQTT message
 * @param      message : Pointer to message
 * @return     true if received ack from gateway
 */
bool pubblish_mqqt_message(char *message)
{
    ESP_LOGI(pcTaskGetName(0), "mqtt pubblish");
    EventBits_t ack_bits;
    espnow_mqtt_data_t *buf = (espnow_mqtt_data_t *)send_param->buffer;
    assert(send_param->len = sizeof(espnow_mqtt_data_t));
    buf->type = ESPNOW_TYPE_MQTT_PUBBLISH;
    strcpy(buf->topic, ESP_MQTT_CLIENT_PUBBLISH);
    strcpy(buf->payload, message);
    strcpy(buf->node_id, l_my_node_id);

    if (esp_now_send(send_param->dest_mac, send_param->buffer,
                     send_param->len) != ESP_OK)
    {
        ESP_LOGE(pcTaskGetName(0), "espnow Send error");
    }
    ack_bits = xEventGroupWaitBits(ack_flag, ACK_RECEIVED_BIT, pdTRUE,
                                   pdFALSE, 700 / portTICK_PERIOD_MS);
    if ((ack_bits & ACK_RECEIVED_BIT))
    {
        espnow_send_errors = 0;
        ESP_LOGI(pcTaskGetName(0), "ack bit cleared");
        return true;
    }
    else
    {
        // increment errors variable until MAX_ESPNOW_SEND_ERRORS then retry connect to gateway
        espnow_send_errors++;
        ESP_LOGE(pcTaskGetName(0), "espnow ACK error %d", espnow_send_errors);
        if (espnow_send_errors >= MAX_ESPNOW_SEND_ERRORS)
        {
            xEventGroupClearBits(scan_ch_group, BROADCAST_CONNECTED_BIT);
            xTaskCreate(scan_gw_channel, "ESPNOW_scan_CH", 1024 * 8, NULL, 2, &scan_gw_channel_handle);
        }
    }
    return false;
}

/**
 * @brief      pubblish node status to node-red
 * @param      node_status  The node status
 */
void pubblish_mqtt_node_status(char *node_status)
{
    //vTaskDelay(200 / portTICK_PERIOD_MS);
    char status [40];
    sprintf(status, "{\"status\":\"%s\"}", node_status);
    pubblish_mqqt_message(status);

}

/**
 * @brief      scan MQTT string for received command and execute action
 * @param      data_to_process : pointer to data string
 * @return     true: if command correctly processed
 */
bool process_rcv_mqtt_data(char *data_to_process)
{
    char delimiter[] = ",";
    char *ptr = strtok(data_to_process, delimiter);

    // LED switch message
    if (!strcmp(ptr, "LED1"))
    {
        ptr = strtok(NULL, delimiter);
        if (!strcmp(ptr, "true"))
        {
            ESP_ERROR_CHECK(gpio_set_level(GPIO_OUTPUT_LED, 1));
            ESP_LOGI(pcTaskGetName(0), "LED1 is ON");
            return true;
        }
        else if (!strcmp(ptr, "false"))
        {
            ESP_ERROR_CHECK(gpio_set_level(GPIO_OUTPUT_LED, 0));
            ESP_LOGI(pcTaskGetName(0), "LED1 is OFF");
            return true;
        }
    }
    // send node alive with timestamp
    if(!strcmp(ptr, "check"))
    {   
        char buffer[25]; 
        uint32_t now = (uint32_t) esp_timer_get_time() / 1000000;   
        uint32_t days    = now / 86400;
        uint32_t hours   = (now % 86400) / 3600;
        uint32_t minutes = (now % 3600) / 60;
        uint32_t seconds = now % 60;

        sprintf(buffer, "idle %lu g %02lu:%02lu:%02lu",
            days, hours, minutes, seconds); 
        pubblish_mqtt_node_status( buffer);
        return true;   
    }
    // running firmware info request
    if(!strcmp(ptr, "get_fw_version"))
    {
        char msg[180];
        const esp_app_desc_t *description = esp_app_get_description();
        //esp_chip_info(esp_chip_info_t *out_info);
        sprintf(msg, "{\"version\":\"%s\",\"fw_name\":\"%s\",\"date\":\"%s\",\"time\":\"%s\",\"board\":\"%s\"}",
                description->version, description->project_name, description->date, description->time, l_board_type);
        ESP_LOGI(pcTaskGetName(0), "test %s", msg);
        pubblish_mqqt_message(msg);
        return true;
    }
    // received Firmware update request from NODE-RED.
    // ask download to the gateway
    if (!strcmp(ptr, "fw_update"))
    {
        ptr = strtok(NULL, delimiter);
        espnow_firmware_t buf;
        buf.type = ESPNOW_TYPE_OTA_FIRMWARE_REQUEST;
        strcpy(buf.node_id, l_my_node_id);
        strcpy(buf.firmware_url, ptr);
        printf("get fw : %s\n", buf.firmware_url);
        send_param->buffer = (uint8_t *)&buf;
        send_param->len = sizeof(espnow_firmware_t);
        if (esp_now_send(send_param->dest_mac, send_param->buffer,
                         send_param->len) != ESP_OK)
        {
            ESP_LOGE(pcTaskGetName(0), "espnow_firmware_request error");
            return false;
        }
        else  ESP_LOGI(pcTaskGetName(0), "sent firmware request");
        return true;
    }
    ESP_LOGE(pcTaskGetName(0), "received invalid command");
    return false;
}


/**
 * @brief  sending callback of ESPNOW
 * @param  mac address of sending device,  transmission status
 * @retval None
 */
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
static void espnow_send_cb(const esp_now_send_info_t *tx_info,
                           esp_now_send_status_t status)
{
    espnow_event_t evt;
    espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (tx_info == NULL)
    {
        ESP_LOGE(pcTaskGetName(0), "Send cb arg error");
        return;
    }
    evt.id = ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, tx_info->des_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    ESP_LOGI(pcTaskGetName(0), "Send to espnow queue");
    if (xQueueSend(espnow_queue, &evt, portMAX_DELAY) != pdTRUE)
    {
        ESP_LOGW(pcTaskGetName(0), "Send queue fail");
    }

}

#else
static void espnow_send_cb(const uint8_t *mac_addr,
                           esp_now_send_status_t status)
{
    espnow_event_t evt;
    espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (mac_addr == NULL)
    {
        ESP_LOGE(pcTaskGetName(0), "Send cb arg error");
        return;
    }
    evt.id = ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(espnow_queue, &evt, portMAX_DELAY) != pdTRUE)
    {
        ESP_LOGW(pcTaskGetName(0), "Send queue fail");
    }

}
#endif
/**
 * @brief  receiving callback of ESPNOW
 * @param  mac address of received device, data received and length of received
 *         data
 * @retval None
 */
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 1)
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info,
                           const uint8_t *data, int len)
#else
static void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data,
                           int len)
#endif
{
    espnow_event_t evt;
    espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 1)
    uint8_t *mac_addr = recv_info->src_addr;
#endif
    if (mac_addr == NULL || data == NULL || len <= 0)
    {
        ESP_LOGE(pcTaskGetName(0), "Receive cb arg error");
        return;
    }
    evt.id = ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    xSemaphoreTake(xSemaphoreData, portMAX_DELAY);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL)
    {
        ESP_LOGE(pcTaskGetName(0), "malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(espnow_queue, &evt, 200) != pdTRUE)
    {
        ESP_LOGW(pcTaskGetName(0), "Send receive queue fail");
        free(recv_cb->data);
    }
    xSemaphoreGive(xSemaphoreData);
    ESP_LOGI(pcTaskGetName(0), "espnow_recv_cb Finish");
}


/**
 * @brief  ESPNOW main task
 * @param  task parameters
 * @retval None
 * @note   this task sends data and waits to receive ack,checks if ack data is
 *          correct and then enters transmit mode again...
 */
void esp_now_task(void *pvParameters)
{
    espnow_event_t evt;

    /* Initialize sending parameters. */
    static espnow_send_param_t send_param_data;
    send_param = &send_param_data;
    //espnow_info_t recv_data;
    ESP_LOGI(pcTaskGetName(0), "ESPNOW Start...");
    for (;;)
    {
        //printf("esp_now_task stack free %d", uxTaskGetStackHighWaterMark(NULL));
        while (xQueueReceive(espnow_queue, &evt, portMAX_DELAY) == pdTRUE)
        {
            switch (evt.id)
            {
            case ESPNOW_SEND_CB:
            {
                ESP_LOGI(pcTaskGetName(0), "send cb task complete");
            }
            break;
            case ESPNOW_RECV_CB:
            {
                xSemaphoreTake(xSemaphoreData, portMAX_DELAY);
                espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
                ESP_LOGI(pcTaskGetName(0), "received cb task. lenght %d", evt.info.recv_cb.data_len);
                uint8_t data_type = ((uint8_t *)recv_cb->data)[0];
                switch (data_type)
                {
                case ESPNOW_TYPE_PAIRING_ACK :
                {
                    free(recv_cb->data);
                    xSemaphoreGive(xSemaphoreData);
                    memcpy(gw_MAC, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                    xEventGroupSetBits(scan_ch_group, BROADCAST_CONNECTED_BIT);
                    ESP_LOGI(pcTaskGetName(0), "ACK PAIRING OK");
                }
                break ;
                case ESPNOW_TYPE_ACK:
                {
                    ESP_LOGI(pcTaskGetName(0), "ACK OK");
                    free(recv_cb->data);
                    xSemaphoreGive(xSemaphoreData);
                    xEventGroupSetBits(ack_flag, ACK_RECEIVED_BIT);
                }
                break;
                case ESPNOW_TYPE_MQTT_SUBSCRIBE:
                {
                    espnow_mqtt_data_t recv_data;
                    memcpy((char *)&recv_data, (char *)recv_cb->data, recv_cb->data_len);
                    free(recv_cb->data);
                    xSemaphoreGive(xSemaphoreData);
                    if (!memcmp(&recv_data.node_id, l_my_node_id, strlen(l_my_node_id)))
                    {
                        // received MQTT frame do something
                        ESP_LOGW(pcTaskGetName(0), "MQTT Data: %s", recv_data.payload);
                        if (!process_rcv_mqtt_data(recv_data.payload))
                            ESP_LOGE(pcTaskGetName(0), "invalid MQTT data received");
                    }
                }
                break;
                case ESPNOW_TYPE_OTA_INFO:
                {
                    espnow_info_t recv_data;
                    memcpy((char *)&recv_data, (char *)recv_cb->data, recv_cb->data_len);
                    free(recv_cb->data);
                    xSemaphoreGive(xSemaphoreData);
                    ESP_LOGI(pcTaskGetName(0), "received OTA INFO");
                    ESP_LOGI(pcTaskGetName(0), "status %d", recv_data.status);
                    switch (recv_data.status)
                    {
                    case ESPNOW_STATUS_OTA_START_FIRMWARE_DOWNLOAD:
                    {
                        ESP_LOGI(pcTaskGetName(0), "starting ota store...");
                        size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
                        ESP_LOGI(pcTaskGetName(0), " free heap: %zu\n", free_heap);
                        xTaskCreate(ota_firmware_store_task, "ota_store_task", 1024 * 10, NULL, 4, NULL);
                    }
                    break;// start ota firmware task
                    case ESPNOW_STATUS_OTA_END_FIRMWARE_DOWNLOAD:
                    {
                        xEventGroupSetBits(ota_status_group, OTA_END_FW_BIT);
                        ESP_LOGI(pcTaskGetName(0), "ota store END");
                    }
                    break; // end ota firm task
                    case ESPNOW_STATUS_OTA_ABORT_FIRMWARE_DOWNLOAD:
                    {
                        xEventGroupSetBits(ota_status_group, OTA_ABORT_FW_BIT);
                    }
                    break; // abort
                    }

                }
                break;
                case ESPNOW_TYPE_OTA_DATA:
                {
                    ESP_LOGI(pcTaskGetName(0), "ota data received");
                    espnow_ota_data_t recv_ota_data;
                    memcpy((char *)&recv_ota_data, (char *)recv_cb->data, recv_cb->data_len);
                    free(recv_cb->data);
                    xEventGroupSetBits(ota_status_group, OTA_GET_FW_BIT);
                    if (xQueueSend(xQueueOtaData, &recv_ota_data, 200) != pdTRUE)
                    {
                        ESP_LOGW(pcTaskGetName(0), "Send send queue fail");
                    }

                    xSemaphoreGive(xSemaphoreData);
                }
                break;
                default:
                {
                    free(recv_cb->data);
                    xSemaphoreGive(xSemaphoreData);
                    ESP_LOGE(pcTaskGetName(0), "UNRECOGNIZED ESPNOW PACKET");
                    __asm__ __volatile__("nop;nop;nop;nop;nop;nop;nop;");
                }
                }   // end of switch (data_type)

            }
            break;
            default:
            {
                ESP_LOGE(pcTaskGetName(0), "Callback type error: %d", evt.id);
            }
            break;
            } // end switch (evt.id)
        } // end while
    }
    vTaskDelete(NULL);
}

/**
 * @brief      Scans for gateway WiFi channel.
 *
 * @param      pvParameters  The pv parameters
 */
void scan_gw_channel(void *pvParameters)
{
    xSemaphoreTake(xSemaphoreEspnow, portMAX_DELAY);
    esp_now_peer_num_t num_peers;

    /*add broadcast address to peer list*/
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL)
    {
        ESP_LOGE(pcTaskGetName(0), "malloc peer information fail");
        esp_now_deinit();
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    if (esp_now_get_peer(broadcast_mac, peer) == ESP_ERR_ESPNOW_NOT_FOUND)
    {
        peer->channel = 0;
        peer->ifidx = ESP_IF_WIFI_STA;
        peer->encrypt = false;
        memcpy(peer->peer_addr, broadcast_mac, sizeof(broadcast_mac));
        ESP_LOGI(pcTaskGetName(0), "storing peer " MACSTR "",
                 MAC2STR(peer->peer_addr));
        ESP_ERROR_CHECK(esp_now_add_peer(peer));
    }
    /*get number of peers and peer data from stored list*/
    ESP_ERROR_CHECK(esp_now_get_peer_num(&num_peers));
    ESP_LOGI(pcTaskGetName(0), "no of peers in peers list:%d",
             num_peers.total_num);

    for (int num_peer = 0; num_peer < num_peers.total_num; num_peer++)
    {
        esp_now_get_peer(broadcast_mac, peer);
        ESP_LOGI(pcTaskGetName(0), "channel:%d", peer->channel);
        ESP_LOGI(pcTaskGetName(0), "peer address " MACSTR "",
                 MAC2STR(peer->peer_addr));
    }
    send_param->len = sizeof(espnow_info_t);
    send_param->buffer = malloc(send_param->len);
    espnow_info_t *buf = (espnow_info_t *)send_param->buffer;
    assert(send_param->len >= sizeof(espnow_info_t));
    buf->type = ESPNOW_TYPE_PAIRING_REQUEST;
    strcpy(buf->node_id, l_my_node_id);

    EventBits_t bits;
    for (uint8_t i = 1; i <= WIFI_MAX_CHANNEL; i++)
    {
        ESP_LOGI(pcTaskGetName(0), "scan CH %d ...", i);
        ESP_ERROR_CHECK(esp_wifi_set_channel(i, WIFI_SECOND_CHAN_NONE));
        peer->channel = i;
        ESP_ERROR_CHECK(esp_now_mod_peer(peer));
        if (esp_now_send(broadcast_mac, send_param->buffer, send_param->len) !=      ESP_OK)
            ESP_LOGE(pcTaskGetName(0), "Send to Broadcast error");
        bits = xEventGroupWaitBits(scan_ch_group, BROADCAST_CONNECTED_BIT, pdFALSE,
                                   pdFALSE, 500 / portTICK_PERIOD_MS);
        if (bits & BROADCAST_CONNECTED_BIT)
        {
            ESP_LOGI(pcTaskGetName(0), "received on CH %d ...", i);
            peer->channel = i;
            memcpy(peer->peer_addr, gw_MAC, sizeof(gw_MAC));
            ESP_ERROR_CHECK(esp_now_add_peer(peer));
            ESP_ERROR_CHECK(esp_now_del_peer(broadcast_mac));
            memcpy(send_param->dest_mac, peer->peer_addr, ESP_NOW_ETH_ALEN);
            break;
        };
        vTaskDelay(100 / portTICK_PERIOD_MS);
    };
    if (!(bits & BROADCAST_CONNECTED_BIT))
    {
        ESP_LOGE(pcTaskGetName(0), "channel scan failed. reboot in 5 sec...");
        free(peer);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        esp_restart();
        vTaskDelete(NULL);
    };
    esp_now_get_peer(peer->peer_addr, peer);
    ESP_LOGI(pcTaskGetName(0), "channel:%d", peer->channel);
    ESP_LOGI(pcTaskGetName(0), "gateway address " MACSTR "",
             MAC2STR(peer->peer_addr));
    free(peer);
    xSemaphoreGive(xSemaphoreEspnow);
    xQueueReset(espnow_queue);
    vTaskDelete(NULL);
}


/**
 * @brief  Wifi mode init
 * @param  None
 * @retval None
 * @note   WiFi should start before using ESPNOW. also long range is enabled so
 *         TX power is high and bandwidth low
 */
static void wifi_init(void)
{
    ESP_LOGI(pcTaskGetName(0), "WIFI init... ");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(11, 0));

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK(esp_wifi_set_protocol(
                        ESP_IF_WIFI_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G |
                        WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
#endif
}

/**
 * @brief  ESPNOW init
 * @param  None
 * @retval None
 * @note   initialize queue size, network params, peer list update and
 *          generating sending data
 */
void espnow_init(void)
{
    ESP_LOGI(pcTaskGetName(0), "ESPNOW init... ");
    scan_ch_group = xEventGroupCreate();
    xEventGroupClearBits(scan_ch_group, BROADCAST_CONNECTED_BIT);

    ack_flag = xEventGroupCreate();
    xEventGroupClearBits(scan_ch_group, ACK_RECEIVED_BIT);

    ota_status_group = xEventGroupCreate();
    xEventGroupClearBits(ota_status_group, OTA_GET_FW_BIT | OTA_ABORT_FW_BIT | OTA_END_FW_BIT);

    /*create queue for application*/
    espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
    if (espnow_queue == NULL)
    {
        ESP_LOGE(pcTaskGetName(0), "Create espnow_queue fail");
    }

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    /*set primary key*/
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)CONFIG_ESP_NOW_PMK));
    /*create tasks*/
    //xTaskCreate(dummy_task, "dummy_task", 1024 * 8, NULL, 3, &dummy_task_handle);
    xTaskCreate(esp_now_task, "ESPNOW_Task", 1024 * 10, NULL, 3, &esp_now_task_handle);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    xTaskCreate(scan_gw_channel, "ESPNOW_scan_CH", 1024 * 8, NULL, 2, &scan_gw_channel_handle);
    xTaskCreate(sw_button_task, "SWBUTTON_Task", 1024 * 6, NULL, 2, &button_task_handle);

}

/**
 * @brief      check switch button status
 *
 * @param      pvParameters  The pv parameters
 */
void sw_button_task(void *pvParameters)
{
    static uint16_t btcounter = 0;
    char msg [20];
    ESP_LOGI(pcTaskGetName(0), "SW button task started...");
    for (;;)
    {
        if (gpio_get_level(GPIO_INPUT_SWBUTTON) == 0)
        {
            btcounter ++;
            xSemaphoreTake(xSemaphoreEspnow, portMAX_DELAY);
            sprintf(msg, "{\"button\":%d}", btcounter);
            pubblish_mqqt_message(msg);
            xSemaphoreGive(xSemaphoreEspnow);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief  main application
 * @param  None
 * @retval None
 */
void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
            ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    xSemaphoreEspnow = xSemaphoreCreateBinary();
    configASSERT(xSemaphoreEspnow);
    xSemaphoreGive(xSemaphoreEspnow);

    // get NODE name from NVS
    nvs_handle_t my_nvs_handle;
    ret = nvs_open("storage", NVS_READONLY, &my_nvs_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(pcTaskGetName(0), "Error (%s) opening NVS handle!\n", esp_err_to_name(ret));
    }
    else
    {
        size_t required_size;
        nvs_get_str(my_nvs_handle, "my_node_id", NULL, &required_size);
        l_my_node_id = malloc(required_size);
        nvs_get_str(my_nvs_handle, "my_node_id", l_my_node_id, &required_size);
        ESP_LOGI(pcTaskGetName(0), "node id %s, size %d", l_my_node_id, required_size);
        nvs_get_str(my_nvs_handle, "board_type", NULL, &required_size);
        l_board_type = malloc(required_size);
        nvs_get_str(my_nvs_handle, "board_type", l_board_type, &required_size);
        ESP_LOGI(pcTaskGetName(0), "board type %s, size %d", l_board_type, required_size);
        nvs_close(my_nvs_handle);
    }
    // Create Semaphore
    xSemaphoreData = xSemaphoreCreateBinary();
    configASSERT(xSemaphoreData);
    xSemaphoreGive(xSemaphoreData);
    esp_log_level_set("*", ESP_LOG_DEBUG);

    gpio_init();
    wifi_init();
    espnow_init();

}
