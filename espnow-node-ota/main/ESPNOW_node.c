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
#include "my_espnow.h"


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
/*Queue handle*/
QueueHandle_t espnow_queue, xQueueOtaData;
SemaphoreHandle_t xSemaphoreData;
EventGroupHandle_t scan_ch_group;
EventGroupHandle_t ack_flag;
EventGroupHandle_t ota_status_group;
TaskHandle_t esp_now_task_handle = NULL;
TaskHandle_t button_task_handle = NULL;
TaskHandle_t scan_gw_channel_handle = NULL;
TaskHandle_t dummy_task_handle = NULL;
TaskHandle_t pubblish_mqtt_message_task_handle = NULL;
TaskHandle_t ota_firmware_store_task_handle = NULL;
UBaseType_t uxHighWaterMark;

void sw_button_task(void *);
void scan_gw_channel(void *);
bool pubblish_mqqt_message(char *);


uint8_t espnow_send_errors = 0;
#define MAX_ESPNOW_SEND_ERRORS 4

#define BROADCAST_CONNECTED_BIT BIT0
#define ACK_RECEIVED_BIT BIT0

#define OTA_GET_FW_BIT BIT0
#define OTA_END_FW_BIT BIT1
#define OTA_ABORT_FW_BIT BIT2

StaticTask_t * pxTaskBuffer;
StackType_t * pxStack;



/**
 * @brief  initialize gpio pin
 * @param  None
 * @retval None
 */
void gpio_init(void) {
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
  EventBits_t ack_bits;
  espnow_mqtt_data_t *buf = (espnow_mqtt_data_t *)send_param->buffer;
  assert(send_param->len = sizeof(espnow_mqtt_data_t));
  buf->type = ESPNOW_TYPE_MQTT_PUBBLISH;
  strcpy(buf->topic, ESP_MQTT_CLIENT_PUBBLISH);
  strcpy(buf->payload, message);
  strcpy(buf->node_id, l_my_node_id);

  if (esp_now_send(send_param->dest_mac, send_param->buffer,
                   send_param->len) != ESP_OK) {
    ESP_LOGE(pcTaskGetName(0), "espnow Send error");
  }
  ESP_LOGW(pcTaskGetName(0),"%d\n", uxTaskGetStackHighWaterMark(NULL));
  ack_bits = xEventGroupWaitBits(ack_flag, ACK_RECEIVED_BIT, pdTRUE,
                                 pdFALSE, 700 / portTICK_PERIOD_MS);
  if ((ack_bits & ACK_RECEIVED_BIT)) {
    espnow_send_errors = 0;
    ESP_LOGI(pcTaskGetName(0), "ack bit cleared");
    return true;
  }
  else {
    // increment errors variable until MAX_ESPNOW_SEND_ERRORS then retry connect to gateway
    espnow_send_errors++;
    ESP_LOGE(pcTaskGetName(0), "espnow ACK error %d", espnow_send_errors);
    if (espnow_send_errors >= MAX_ESPNOW_SEND_ERRORS) {
      xEventGroupClearBits(scan_ch_group, BROADCAST_CONNECTED_BIT);
      xTaskCreate(scan_gw_channel, "ESPNOW_scan_CH", 1024 * 8, NULL, 2, &scan_gw_channel_handle);
    }
  }
  return false;
}

/**
 * @brief      pubblish node status to node-red
 *
 * @param      node_status  The node status
 */
void pubblish_mqtt_node_status(char *node_status)
{
  vTaskDelay(200 / portTICK_PERIOD_MS);
  char status [40];
  sprintf(status, "{\"status\":\"%s\"}", node_status);
  pubblish_mqqt_message(status);

}

/**
 * @brief      scan MQTT string for received command and execute action
 * @param      data_to_process : pointer to data string
 * @return     true: if command correctly processed
 */
bool process_rcv_mqtt_data(char * data_to_process)
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
  // running firmware info request
  if(!strcmp(ptr,"get_fw_version"))
  {
    char msg[180];
    const esp_app_desc_t* description = esp_app_get_description();
    //esp_chip_info(esp_chip_info_t *out_info);
    sprintf(msg, "{\"version\":\"%s\",\"fw_name\":\"%s\",\"date\":\"%s\",\"time\":\"%s\",\"board\":\"%s\"}", 
      description->version, description->project_name, description->date, description->time, l_board_type);
    ESP_LOGI(pcTaskGetName(0),"test %s" ,msg);
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
    send_param->len) != ESP_OK) {
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
static void espnow_send_cb(const uint8_t *mac_addr,
                           esp_now_send_status_t status) {
  espnow_event_t evt;
  espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

  if (mac_addr == NULL) {
    ESP_LOGE(pcTaskGetName(0), "Send cb arg error");
    return;
  }
  evt.id = ESPNOW_SEND_CB;
  memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
  send_cb->status = status;
  if (xQueueSend(espnow_queue, &evt, portMAX_DELAY) != pdTRUE) {
    ESP_LOGW(pcTaskGetName(0), "Send queue fail");
  }
  
}

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
  if (mac_addr == NULL || data == NULL || len <= 0) {
    ESP_LOGE(pcTaskGetName(0), "Receive cb arg error");
    return;
  }
  evt.id = ESPNOW_RECV_CB;
  memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
  xSemaphoreTake(xSemaphoreData, portMAX_DELAY);
  recv_cb->data = malloc(len);
  if (recv_cb->data == NULL) {
    ESP_LOGE(pcTaskGetName(0), "malloc receive data fail");
    return;
  }
  memcpy(recv_cb->data, data, len);
  recv_cb->data_len = len;
  if (xQueueSend(espnow_queue, &evt, 200) != pdTRUE) {
    ESP_LOGW(pcTaskGetName(0), "Send receive queue fail");
    free(recv_cb->data);
  }
  xSemaphoreGive(xSemaphoreData);
  ESP_LOGI(pcTaskGetName(0), "espnow_recv_cb Finish");
}

/**
 * @brief      deinit ESPNOW
 */
static void espnow_deinit(void) {
  free(send_param->buffer);
  free(send_param);
  vSemaphoreDelete(espnow_queue);
  esp_now_deinit();
}

/**
 * @brief      send OTA infos to gateway
 *
 * @param[in]  message_type  The message type
 * @param[in]  status        The status
 * @param[in]  chunk         The chunk
 */
static void espnow_send_ota_info(espnow_ota_type_t message_type, espnow_status_id_t status, uint16_t chunk)
{
  espnow_info_t *buf = (espnow_info_t *)send_param->buffer;
  assert(send_param->len = sizeof(espnow_info_t));
  buf->type = message_type;
  buf->status = status;
  buf->ota_chunk = chunk;
  if (esp_now_send(send_param->dest_mac, send_param->buffer,
                   send_param->len) != ESP_OK) {
    ESP_LOGE(pcTaskGetName(0), "espnow_send_ota_info error");
  }
  else  ESP_LOGI(pcTaskGetName(0), "sent info ack");
}

void dummy_task(void *pvParameters)
{
  ESP_LOGI(pcTaskGetName(0), "starting DUMMY TASK.... ");
  char *data = malloc(100);
  strcpy ((char*)data, "prova");
  printf("dummy_task stack free %d\n", uxTaskGetStackHighWaterMark(NULL));
  for(int i=0;i<6;i++)
  {
    ESP_LOGI(pcTaskGetName(0), "DUMMY TASK %s",data);
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
  free(data);
  vTaskDelete(NULL);
}
/**
 * @brief      Process data for OTA firmware upgrade and store new firmware
 *              in available partition
 * @param      pvParameters  The pv parameters
 */

void ota_firmware_store_task(void *pvParameters)
{
#define BUFFSIZE 920
  //ESP_ERROR_CHECK( heap_trace_start(HEAP_TRACE_LEAKS) );
  esp_err_t err;
  espnow_ota_data_t recv_ota_data;
  // update handle : set by esp_ota_begin(), must be freed via esp_ota_end() 
  esp_ota_handle_t update_handle = 0 ;
  const esp_partition_t *update_partition = NULL;
  uint8_t * ota_write_data = malloc(sizeof(uint8_t) * BUFFSIZE + 1);
  if (ota_write_data == NULL)
    ESP_LOGE(pcTaskGetName(0), "malloc error in OTA task");

  ESP_LOGI(pcTaskGetName(0), "Starting OTA task");
  size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  ESP_LOGI(pcTaskGetName(0), " free heap: %zu", free_heap);
  const esp_partition_t *configured = esp_ota_get_boot_partition();
  const esp_partition_t *running = esp_ota_get_running_partition();
  // check ota partition
  if (configured != running) {
    ESP_LOGW(pcTaskGetName(0), "Configured OTA boot partition at offset 0x%08"PRIx32", but running from offset 0x%08"PRIx32,
             configured->address, running->address);
    ESP_LOGW(pcTaskGetName(0), "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
  }
  ESP_LOGI(pcTaskGetName(0), "Running partition type %d subtype %d (offset 0x%08"PRIx32")",
           running->type, running->subtype, running->address);
  //if ( heap_caps_check_integrity(MALLOC_CAP_INTERNAL, true))
  //ESP_LOGE(pcTaskGetName(0), "heap ok");
  update_partition = esp_ota_get_next_update_partition(NULL);
  assert(update_partition != NULL);
  ESP_LOGI(pcTaskGetName(0), "Writing to partition subtype %d at offset 0x%"PRIx32,
           update_partition->subtype, update_partition->address);
  int binary_file_length = 0;
  uint16_t ota_write_data_size = 0;
  uint8_t ota_write_data_sequence = 0;
  BaseType_t ret_Queue;
  bool image_header_was_checked = false;

  //deal with all receive packet
  xQueueOtaData = xQueueCreate(5, sizeof(espnow_ota_data_t));
  if (xQueueOtaData == NULL) {
    ESP_LOGE(pcTaskGetName(0), "Create xQueueOtaData fail");
  }
  while (1) {
    EventBits_t uxBits = xEventGroupWaitBits(ota_status_group,
                         OTA_GET_FW_BIT | OTA_ABORT_FW_BIT | OTA_END_FW_BIT,
                         pdTRUE, pdFALSE, 2500 / portTICK_PERIOD_MS);
    //ESP_LOGI(pcTaskGetName(0), "uxBits %lu", uxBits);
    
    if ( ( uxBits & OTA_ABORT_FW_BIT ) != 0 )
    {
      ESP_LOGE(pcTaskGetName(0), "OTA aborted by server");
      // pubblish_mqtt_node_status("OTA aborted");
      goto EndTask;
    }
    // get ota chunks while receiving OTA_END_FW_BIT (last chunks)
    else if ( (( uxBits & OTA_GET_FW_BIT ) != 0) || (( uxBits & OTA_END_FW_BIT ) != 0))
    {
      ret_Queue = xQueueReceive(xQueueOtaData, &recv_ota_data, 400 / portTICK_PERIOD_MS);
      uint8_t calc_crc = esp_crc8_le(0, (uint8_t const*)&recv_ota_data.ota_chunk, recv_ota_data.ota_chunk_size);
      
      //ESP_LOGI(pcTaskGetName(0), "chunk %d", recv_ota_data.sequence);
      //ESP_LOGW(pcTaskGetName(0), "Recv data: %.*s", recv_ota_data.ota_chunk_size, recv_ota_data.ota_chunk);
      if (calc_crc != recv_ota_data.ota_chunk_crc)
      {
        ESP_LOGE(pcTaskGetName(0), "Wrong CRC");
        espnow_send_ota_info(ESPNOW_TYPE_OTA_INFO, ESPNOW_STATUS_OTA_ACK_ERROR, recv_ota_data.sequence);
        goto EndTask;
      }
      // copy 4 chunks in ota_write_data buffer
      if (ota_write_data_sequence <= 3)
      {
        //ESP_LOGI(pcTaskGetName(0), "ota_write_data_sequence %d", ota_write_data_sequence);
        memcpy (ota_write_data + ota_write_data_size, recv_ota_data.ota_chunk,  recv_ota_data.ota_chunk_size);
         printf("ota store stack free %d\n", uxTaskGetStackHighWaterMark(NULL));
         ESP_LOGI(pcTaskGetName(0), " OTA free heap: %zu", free_heap);
        ota_write_data_size += recv_ota_data.ota_chunk_size;
        ota_write_data_sequence ++;
        //ESP_LOGI(pcTaskGetName(0), "ota data partial size = %d", ota_write_data_size);

        espnow_send_ota_info(ESPNOW_TYPE_OTA_INFO, ESPNOW_STATUS_OTA_ACK_OK, recv_ota_data.sequence);
      }
      // write buffered data in OTA partition if buffer full or timeout receiving chunk or flush last chunks (OTA_END_FW_BIT)
      if ((ota_write_data_sequence == 4) || (ret_Queue == pdFALSE) || (( uxBits & OTA_END_FW_BIT ) != 0))
      {
        //ESP_LOGI(pcTaskGetName(0), "ota data size = %d", ota_write_data_size);
        if (image_header_was_checked == false) {
          esp_app_desc_t new_app_info;
          if (ota_write_data_size > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t)) {
            // check current version with downloading
            memcpy(&new_app_info, &ota_write_data[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t));
            ESP_LOGI(pcTaskGetName(0), "New firmware version: %s", new_app_info.version);

            esp_app_desc_t running_app_info;
            if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
              ESP_LOGI(pcTaskGetName(0), "Running firmware version: %s", running_app_info.version);
            }

            const esp_partition_t* last_invalid_app = esp_ota_get_last_invalid_partition();
            esp_app_desc_t invalid_app_info;
            if (esp_ota_get_partition_description(last_invalid_app, &invalid_app_info) == ESP_OK) {
              ESP_LOGI(pcTaskGetName(0), "Last invalid firmware version: %s", invalid_app_info.version);
            }

            // check current version with last invalid partition
            if (last_invalid_app != NULL) {
              if (memcmp(invalid_app_info.version, new_app_info.version, sizeof(new_app_info.version)) == 0) {
                ESP_LOGW(pcTaskGetName(0), "New version is the same as invalid version.");
                ESP_LOGW(pcTaskGetName(0), "Previously, there was an attempt to launch the firmware with %s version, but it failed.", invalid_app_info.version);
                ESP_LOGW(pcTaskGetName(0), "The firmware has been rolled back to the previous version.");
                goto EndTask;
              }
            }
            image_header_was_checked = true;
            
            //err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
            err = ESP_OK;
            if (err != ESP_OK) {
              ESP_LOGE(pcTaskGetName(0), "esp_ota_begin failed (%s)", esp_err_to_name(err));
              esp_ota_abort(update_handle);
              goto EndTask;
            }
            ESP_LOGI(pcTaskGetName(0), "esp_ota_begin succeeded");
          } else {
            ESP_LOGE(pcTaskGetName(0), "received package not fit len");
            esp_ota_abort(update_handle);
            goto EndTask;
          }
        }
        //err = esp_ota_write( update_handle, (const void *)ota_write_data, ota_write_data_size);
        //printf("ota stack free %d", uxTaskGetStackHighWaterMark(NULL));
        if (err != ESP_OK) {
          esp_ota_abort(update_handle);
          goto EndTask;
        }
        //vTaskDelay(10 / portTICK_PERIOD_MS);
        binary_file_length += ota_write_data_size;
        ESP_LOGW(pcTaskGetName(0), "Written image length %d", binary_file_length);
        ota_write_data_size = 0;
        ota_write_data_sequence = 0;
        //ESP_ERROR_CHECK( heap_trace_stop() );
        //heap_trace_dump();
      }
      // received end of OTA data from gateway. Finish to write OTA image check and reboot.
      if (( uxBits & OTA_END_FW_BIT ) != 0)
      {
        ESP_LOGI(pcTaskGetName(0), "ota firmware download END ");
        free (ota_write_data);
        //err = esp_ota_end(update_handle);
        if (err != ESP_OK) {
          if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
            ESP_LOGE(pcTaskGetName(0), "Image validation failed, image is corrupted");
          } else {
            ESP_LOGE(pcTaskGetName(0), "esp_ota_end failed (%s)!", esp_err_to_name(err));
          }
          goto EndTask;
        } 
       // err = esp_ota_set_boot_partition(update_partition);
        if (err != ESP_OK) {
          ESP_LOGE(pcTaskGetName(0), "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
          goto EndTask;
        }
        ESP_LOGI(pcTaskGetName(0), "Firmware download completed.");
        ESP_LOGW(pcTaskGetName(0), "Prepare to restart system!");
        vQueueDelete( xQueueOtaData );
        //pubblish_mqtt_node_status ("OTA ok rebooting..");
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
        esp_restart(); // system restart with new firmware
        vTaskDelete(NULL);
      }
    }
    // timeout
    else
    {
      ESP_LOGE(pcTaskGetName(0), "TIMEOUT. Abort OTA download");
      espnow_send_ota_info(ESPNOW_TYPE_OTA_INFO, ESPNOW_STATUS_OTA_ABORT_FIRMWARE_DOWNLOAD, recv_ota_data.sequence);
      esp_ota_abort(update_handle);
      goto EndTask;
    }
  }
  EndTask:
  vQueueDelete( xQueueOtaData );
  free (ota_write_data);
  xEventGroupClearBits(ota_status_group, OTA_GET_FW_BIT);
  //vTaskResume(pubblish_mqtt_message_task_handle);
  vTaskDelete(NULL);
}

/*
void ota_firmware_store_task(void *pvParameters)
{
#define BUFFSIZE 920
  esp_err_t err;
  espnow_ota_data_t recv_ota_data;
  // update handle : set by esp_ota_begin(), must be freed via esp_ota_end() 
  esp_ota_handle_t update_handle = 0 ;
  const esp_partition_t *update_partition = NULL;
  //uint8_t * ota_write_data = malloc(sizeof(uint8_t) * BUFFSIZE + 1);
  uint8_t * ota_write_data = heap_caps_malloc(sizeof(uint8_t) * BUFFSIZE + 1, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (ota_write_data == NULL)
    ESP_LOGE(pcTaskGetName(0), "malloc error in OTA task");

  ESP_LOGI(pcTaskGetName(0), "Starting OTA task");
  size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  ESP_LOGI(pcTaskGetName(0), " free heap: %zu", free_heap);
  const esp_partition_t *configured = esp_ota_get_boot_partition();
  const esp_partition_t *running = esp_ota_get_running_partition();
  // check ota partition
  if (configured != running) {
    ESP_LOGW(pcTaskGetName(0), "Configured OTA boot partition at offset 0x%08"PRIx32", but running from offset 0x%08"PRIx32,
             configured->address, running->address);
    ESP_LOGW(pcTaskGetName(0), "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
  }
  ESP_LOGI(pcTaskGetName(0), "Running partition type %d subtype %d (offset 0x%08"PRIx32")",
           running->type, running->subtype, running->address);
  //if ( heap_caps_check_integrity(MALLOC_CAP_INTERNAL, true))
  //ESP_LOGE(pcTaskGetName(0), "heap ok");
  update_partition = esp_ota_get_next_update_partition(NULL);
  assert(update_partition != NULL);
  ESP_LOGI(pcTaskGetName(0), "Writing to partition subtype %d at offset 0x%"PRIx32,
           update_partition->subtype, update_partition->address);
  int binary_file_length = 0;
  uint16_t ota_write_data_size = 0;
  uint8_t ota_write_data_sequence = 0;
  BaseType_t ret_Queue;
  bool image_header_was_checked = false;

  //deal with all receive packet
  xQueueOtaData = xQueueCreate(10, sizeof(espnow_ota_data_t));
  if (xQueueOtaData == NULL) {
    ESP_LOGE(pcTaskGetName(0), "Create xQueueOtaData fail");
  }
  while (1) {
    EventBits_t uxBits = xEventGroupWaitBits(ota_status_group,
                         OTA_GET_FW_BIT | OTA_ABORT_FW_BIT | OTA_END_FW_BIT,
                         pdTRUE, pdFALSE, 4500 / portTICK_PERIOD_MS);
    //ESP_LOGI(pcTaskGetName(0), "uxBits %lu", uxBits);
    //printf("ota store stack free %d\n", uxTaskGetStackHighWaterMark(NULL));
    if ( ( uxBits & OTA_ABORT_FW_BIT ) != 0 )
    {
      ESP_LOGE(pcTaskGetName(0), "OTA aborted by server");
      free (ota_write_data);
      vTaskDelete(NULL);
    }
    // get ota chunks while receiving OTA_END_FW_BIT (last chunks)
    else if ( (( uxBits & OTA_GET_FW_BIT ) != 0) || (( uxBits & OTA_END_FW_BIT ) != 0))
    {
      ret_Queue = xQueueReceive(xQueueOtaData, &recv_ota_data, 500 / portTICK_PERIOD_MS);
      uint8_t calc_crc = esp_crc8_le(0, (uint8_t const*)&recv_ota_data.ota_chunk, recv_ota_data.ota_chunk_size);
      //ESP_LOGI(pcTaskGetName(0), "chunk %d", recv_ota_data.sequence);
      size_t free_iram = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
      ESP_LOGW(pcTaskGetName(0), " free iram: %zu", free_iram);
      size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
      ESP_LOGW(pcTaskGetName(0), " free psram: %zu", free_psram);

      ESP_LOGW(pcTaskGetName(0),"%d\n", uxTaskGetStackHighWaterMark(NULL));
      //ESP_LOGW(pcTaskGetName(0), "Recv data: %.*s", recv_ota_data.ota_chunk_size, recv_ota_data.ota_chunk);
      if (calc_crc != recv_ota_data.ota_chunk_crc)
      {
        ESP_LOGE(pcTaskGetName(0), "Wrong CRC");
        espnow_send_ota_info(ESPNOW_TYPE_OTA_INFO, ESPNOW_STATUS_OTA_ACK_ERROR, recv_ota_data.sequence);
        free (ota_write_data);
        vTaskDelete(NULL);
      }
      // copy 4 chunks in ota_write_data buffer
      if (ota_write_data_sequence <= 3)
      {
        //ESP_LOGI(pcTaskGetName(0), "ota_write_data_sequence %d", ota_write_data_sequence);
        memcpy (ota_write_data + ota_write_data_size, recv_ota_data.ota_chunk,  recv_ota_data.ota_chunk_size);
        ota_write_data_size += recv_ota_data.ota_chunk_size;
        ota_write_data_sequence ++;
        //ESP_LOGI(pcTaskGetName(0), "ota data partial size = %d", ota_write_data_size);

        espnow_send_ota_info(ESPNOW_TYPE_OTA_INFO, ESPNOW_STATUS_OTA_ACK_OK, recv_ota_data.sequence);
      }
      // write buffered data in OTA partition if buffer full or timeout receiving chunk or flush last chunks (OTA_END_FW_BIT)
      if ((ota_write_data_sequence == 4) || (ret_Queue == pdFALSE) || (( uxBits & OTA_END_FW_BIT ) != 0))
      {
        //ESP_LOGI(pcTaskGetName(0), "ota data size = %d", ota_write_data_size);
        if (image_header_was_checked == false) {
          esp_app_desc_t new_app_info;
          if (ota_write_data_size > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t)) {
            // check current version with downloading
            memcpy(&new_app_info, &ota_write_data[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t));
            ESP_LOGI(pcTaskGetName(0), "New firmware version: %s", new_app_info.version);

            esp_app_desc_t running_app_info;
            if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
              ESP_LOGI(pcTaskGetName(0), "Running firmware version: %s", running_app_info.version);
            }

            const esp_partition_t* last_invalid_app = esp_ota_get_last_invalid_partition();
            esp_app_desc_t invalid_app_info;
            if (esp_ota_get_partition_description(last_invalid_app, &invalid_app_info) == ESP_OK) {
              ESP_LOGI(pcTaskGetName(0), "Last invalid firmware version: %s", invalid_app_info.version);
            }

            // check current version with last invalid partition
            if (last_invalid_app != NULL) {
              if (memcmp(invalid_app_info.version, new_app_info.version, sizeof(new_app_info.version)) == 0) {
                ESP_LOGW(pcTaskGetName(0), "New version is the same as invalid version.");
                ESP_LOGW(pcTaskGetName(0), "Previously, there was an attempt to launch the firmware with %s version, but it failed.", invalid_app_info.version);
                ESP_LOGW(pcTaskGetName(0), "The firmware has been rolled back to the previous version.");
                free (ota_write_data);
                vTaskDelete(NULL);
              }
            }
            image_header_was_checked = true;

            err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
            if (err != ESP_OK) {
              ESP_LOGE(pcTaskGetName(0), "esp_ota_begin failed (%s)", esp_err_to_name(err));
              esp_ota_abort(update_handle);
              free (ota_write_data);
              vTaskDelete(NULL);
            }
            ESP_LOGI(pcTaskGetName(0), "esp_ota_begin succeeded");
          } else {
            ESP_LOGE(pcTaskGetName(0), "received package is not fit len");
            esp_ota_abort(update_handle);
            free (ota_write_data);
            vTaskDelete(NULL);
          }
        }
        err = esp_ota_write( update_handle, (const void *)ota_write_data, ota_write_data_size);
        //printf("ota stack free %d", uxTaskGetStackHighWaterMark(NULL));
        if (err != ESP_OK) {
          esp_ota_abort(update_handle);
          free (ota_write_data);
          vTaskDelete(NULL);
        }
        binary_file_length += ota_write_data_size;
        ESP_LOGI(pcTaskGetName(0), "Written image length %d", binary_file_length);
        ota_write_data_size = 0;
        ota_write_data_sequence = 0;
      }
      // received end of OTA data from gateway. Finish to write OTA image check and reboot.
      if (( uxBits & OTA_END_FW_BIT ) != 0)
      {
        ESP_LOGI(pcTaskGetName(0), "ota firmware download END ");
        free (ota_write_data);
        err = esp_ota_end(update_handle);
        if (err != ESP_OK) {
          if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
            ESP_LOGE(pcTaskGetName(0), "Image validation failed, image is corrupted");
          } else {
            ESP_LOGE(pcTaskGetName(0), "esp_ota_end failed (%s)!", esp_err_to_name(err));
          }
          vTaskDelete(NULL);
        }

        err = esp_ota_set_boot_partition(update_partition);
        if (err != ESP_OK) {
          ESP_LOGE(pcTaskGetName(0), "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
          vTaskDelete(NULL);
        }
        ESP_LOGI(pcTaskGetName(0), "Firmware download completed.");
        ESP_LOGW(pcTaskGetName(0), "Prepare to restart system!");
        vQueueDelete( xQueueOtaData );
        esp_restart(); // system restart with new firmware
        vTaskDelete(NULL);
      }
    }

    // timeout
    
    else
    {
      ESP_LOGE(pcTaskGetName(0), "TIMEOUT. Abort OTA download");
      espnow_send_ota_info(ESPNOW_TYPE_OTA_INFO, ESPNOW_STATUS_OTA_ABORT_FIRMWARE_DOWNLOAD, recv_ota_data.sequence);
      esp_ota_abort(update_handle);
      free (ota_write_data);
      vQueueDelete( xQueueOtaData );
      vTaskDelete(NULL);
    }
    
  }
  vTaskDelete(NULL);
}
*/

/**
 * @brief  ESPNOW main task
 * @param  task parameters
 * @retval None
 * @note   this task sends data and waits to receive ack,checks if ack data is
 *          correct and then enters transmit mode again...
 */
void esp_now_task(void *pvParameters) {
  espnow_event_t evt;
  /* Initialize sending parameters.
  send_param = malloc(sizeof(espnow_send_param_t));
  if (send_param == NULL) {
    ESP_LOGE(pcTaskGetName(0), "malloc send parameter fail");
  }*/
   /* Initialize sending parameters. */
  static espnow_send_param_t send_param_data;
  send_param = &send_param_data;
  //espnow_info_t recv_data;
  ESP_LOGI(pcTaskGetName(0), "ESPNOW Start...");
  for (;;) {
    //printf("esp_now_task stack free %d", uxTaskGetStackHighWaterMark(NULL));
    while (xQueueReceive(espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
      switch (evt.id) {
      case ESPNOW_SEND_CB: {
        ESP_LOGI(pcTaskGetName(0), "send cb task complete");
      } break;
      case ESPNOW_RECV_CB: {
        //ESP_LOGI(pcTaskGetName(0), "ESPNOW_RECV_CB");
        ESP_LOGW(pcTaskGetName(0),"%d\n", uxTaskGetStackHighWaterMark(NULL));
        //char *my_node_id = CONFIG_NODE_ID;
        xSemaphoreTake(xSemaphoreData, portMAX_DELAY);
        espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
        ESP_LOGI(pcTaskGetName(0), "received cb task. lenght %d", evt.info.recv_cb.data_len);
        uint8_t data_type = ((uint8_t *)recv_cb->data)[0];
        switch (data_type) {
        case ESPNOW_TYPE_PAIRING_ACK :
        {
          free(recv_cb->data);
          xSemaphoreGive(xSemaphoreData);
          memcpy(gw_MAC, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
          xEventGroupSetBits(scan_ch_group, BROADCAST_CONNECTED_BIT);
          ESP_LOGI(pcTaskGetName(0), "ACK PAIRING OK");
          //printf("esp_now_task stack free %d", uxTaskGetStackHighWaterMark(NULL));
        } break ;
        case ESPNOW_TYPE_ACK:
        {
          ESP_LOGI(pcTaskGetName(0), "ACK OK");
          free(recv_cb->data);
          xSemaphoreGive(xSemaphoreData);
          xEventGroupSetBits(ack_flag, ACK_RECEIVED_BIT);
        } break;
        case ESPNOW_TYPE_MQTT_SUBSCRIBE:
        {
          espnow_mqtt_data_t recv_data;
          memcpy((char *)&recv_data, (char *)recv_cb->data, recv_cb->data_len);
          free(recv_cb->data);
          xSemaphoreGive(xSemaphoreData);
          if (!memcmp(&recv_data.node_id, l_my_node_id, strlen(l_my_node_id))) {
            // received MQTT frame do something
            ESP_LOGW(pcTaskGetName(0), "MQTT Data: %s", recv_data.payload);
            if (!process_rcv_mqtt_data(recv_data.payload))
              ESP_LOGE(pcTaskGetName(0), "invalid MQTT data received");
          }
        } break;
        case ESPNOW_TYPE_OTA_INFO:
        {
          espnow_info_t recv_data;
          memcpy((char *)&recv_data, (char *)recv_cb->data, recv_cb->data_len);
          free(recv_cb->data);
          xSemaphoreGive(xSemaphoreData);
          ESP_LOGI(pcTaskGetName(0), "received OTA INFO");
          ESP_LOGI(pcTaskGetName(0), "status %d", recv_data.status);
          switch (recv_data.status) {
          case ESPNOW_STATUS_OTA_START_FIRMWARE_DOWNLOAD:
          {
            ESP_LOGI(pcTaskGetName(0), "starting ota store...");
            
            //printf("esp_now_task stack free %d\n", uxTaskGetStackHighWaterMark(NULL));
            //printf("button_task stack free %d\n", uxTaskGetStackHighWaterMark(button_task_handle));
            //printf("scan_ch stack free %d\n", uxTaskGetStackHighWaterMark(scan_gw_channel_handle));
            size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
            ESP_LOGI(pcTaskGetName(0), " free heap: %zu\n", free_heap);
            //xTaskCreate(ota_firmware_store_task, "ota_store_task", 1024 * 10, NULL, 3, NULL);
            // TCBs should always be in internal RAM
            pxTaskBuffer = heap_caps_malloc(sizeof(StaticTask_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            // Allocate the stack in PSRAM
            pxStack = heap_caps_malloc(1024*10, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            xTaskCreateStatic(ota_firmware_store_task, "ota_store_task", 1024 * 10,NULL, 3, pxStack, pxTaskBuffer);
          } break;// start ota firmware task
          case ESPNOW_STATUS_OTA_END_FIRMWARE_DOWNLOAD:
          {
            xEventGroupSetBits(ota_status_group, OTA_END_FW_BIT);
            ESP_LOGI(pcTaskGetName(0), "ota store END");
          } break; // end ota firm task
          case ESPNOW_STATUS_OTA_ABORT_FIRMWARE_DOWNLOAD:
          {
            xEventGroupSetBits(ota_status_group, OTA_ABORT_FW_BIT);
          } break; // abort
          }

        } break;
        case ESPNOW_TYPE_OTA_DATA:
        {
          ESP_LOGI(pcTaskGetName(0), "ota data received");
          espnow_ota_data_t recv_ota_data;
          memcpy((char *)&recv_ota_data, (char *)recv_cb->data, recv_cb->data_len);
          free(recv_cb->data);
          xEventGroupSetBits(ota_status_group, OTA_GET_FW_BIT);
          if (xQueueSend(xQueueOtaData, &recv_ota_data, 200) != pdTRUE) {
            ESP_LOGW(pcTaskGetName(0), "Send send queue fail");
          }

          xSemaphoreGive(xSemaphoreData);
        } break;
        default: {
          free(recv_cb->data);
          xSemaphoreGive(xSemaphoreData);
          ESP_LOGE(pcTaskGetName(0), "UNRECOGNIZED ESPNOW PACKET");
          __asm__ __volatile__("nop;nop;nop;nop;nop;nop;nop;");
        }
        }   // end of switch (data_type)

      } break;
      default: {
        ESP_LOGE(pcTaskGetName(0), "Callback type error: %d", evt.id);
      } break;
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
void scan_gw_channel(void *pvParameters) {
  esp_now_peer_num_t num_peers;

  /*add broadcast address to peer list*/
  esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
  if (peer == NULL) {
    ESP_LOGE(pcTaskGetName(0), "malloc peer information fail");
    esp_now_deinit();
  }
  memset(peer, 0, sizeof(esp_now_peer_info_t));
  if (esp_now_get_peer(broadcast_mac, peer) == ESP_ERR_ESPNOW_NOT_FOUND) {
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

  for (int num_peer = 0; num_peer < num_peers.total_num; num_peer++) {
    esp_now_get_peer(broadcast_mac, peer);
    ESP_LOGI(pcTaskGetName(0), "channel:%d", peer->channel);
    ESP_LOGI(pcTaskGetName(0), "peer address " MACSTR "",
             MAC2STR(peer->peer_addr));
  }
  
  //memset(send_param, 0, sizeof(espnow_send_param_t));
  send_param->len = sizeof(espnow_info_t);
  send_param->buffer = malloc(send_param->len);
  espnow_info_t *buf = (espnow_info_t *)send_param->buffer;
  assert(send_param->len >= sizeof(espnow_info_t));
  buf->type = ESPNOW_TYPE_PAIRING_REQUEST;
  strcpy(buf->node_id, l_my_node_id);
  
  EventBits_t bits;
  for (uint8_t i = 1; i <= WIFI_MAX_CHANNEL; i++) {
    ESP_LOGI(pcTaskGetName(0), "scan CH %d ...", i);
    ESP_ERROR_CHECK(esp_wifi_set_channel(i, WIFI_SECOND_CHAN_NONE));
    peer->channel = i;
    ESP_ERROR_CHECK(esp_now_mod_peer(peer));
    // ESP_LOGI(pcTaskGetName(0),"peer address "MACSTR"",
    // MAC2STR(peer->peer_addr)); ESP_LOGI(pcTaskGetName(0),"peer CH %d
    // ",peer->channel);

    if (esp_now_send(broadcast_mac, send_param->buffer, send_param->len) !=      ESP_OK)
      ESP_LOGE(pcTaskGetName(0), "Send to Broadcast error");
    bits = xEventGroupWaitBits(scan_ch_group, BROADCAST_CONNECTED_BIT, pdFALSE,
                               pdFALSE, 500 / portTICK_PERIOD_MS);
    if (bits & BROADCAST_CONNECTED_BIT) {
      ESP_LOGI(pcTaskGetName(0), "received on CH %d ...", i);
      peer->channel = i;
      memcpy(peer->peer_addr, gw_MAC, sizeof(gw_MAC));
      ESP_ERROR_CHECK(esp_now_add_peer(peer));
      ESP_ERROR_CHECK(esp_now_del_peer(broadcast_mac));
      memcpy(send_param->dest_mac, peer->peer_addr, ESP_NOW_ETH_ALEN);
      //printf("scan ch stack free %d", uxTaskGetStackHighWaterMark(NULL));
      break;
    };
    vTaskDelay(100 / portTICK_PERIOD_MS);
  };
  if (!(bits & BROADCAST_CONNECTED_BIT)) {
    ESP_LOGE(pcTaskGetName(0), "channel scan failed. reboot in 60 sec...");
    espnow_deinit();
    free(peer);
    vTaskDelay(60000 / portTICK_PERIOD_MS);
    esp_restart();
    vTaskDelete(NULL);
  };

  ESP_ERROR_CHECK(esp_now_get_peer_num(&num_peers));
  for (int num_peer = 0; num_peer < num_peers.total_num; num_peer++) {
    esp_now_get_peer(broadcast_mac, peer);
    ESP_LOGI(pcTaskGetName(0), "channel:%d", peer->channel);
    ESP_LOGI(pcTaskGetName(0), "gateway address " MACSTR "",
             MAC2STR(peer->peer_addr));
  }
  free(peer);
  vTaskDelete(NULL);
}

/**
 * @brief  Wifi mode init
 * @param  None
 * @retval None
 * @note   WiFi should start before using ESPNOW. also long range is enabled so
 *         TX power is high and bandwidth low
 */
static void wifi_init(void) {
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
void espnow_init(void) {
  scan_ch_group = xEventGroupCreate();
  xEventGroupClearBits(scan_ch_group, BROADCAST_CONNECTED_BIT);

  ack_flag = xEventGroupCreate();
  xEventGroupClearBits(scan_ch_group, ACK_RECEIVED_BIT);

  ota_status_group = xEventGroupCreate();
  xEventGroupClearBits(ota_status_group, OTA_GET_FW_BIT | OTA_ABORT_FW_BIT | OTA_END_FW_BIT);

  /*create queue for application*/
  espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
  if (espnow_queue == NULL) {
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
  xTaskCreate(scan_gw_channel, "ESPNOW_scan_CH", 1024 * 8, NULL, 2,&scan_gw_channel_handle);
  xTaskCreate(sw_button_task, "SWBUTTON_Task", 1024 * 6, NULL, 2, &button_task_handle);
}

/**
 * @brief      check switch button status
 *
 * @param      pvParameters  The pv parameters
 */
void sw_button_task(void *pvParameters) {
  static uint16_t counter = 0;
  char msg [20];
  ESP_LOGI(pcTaskGetName(0), "SW button task started...");
  for (;;) {
    if (gpio_get_level(GPIO_INPUT_SWBUTTON) == 0) {
      counter ++;
      sprintf(msg, "{\"button\":%d}", counter);
      pubblish_mqqt_message(msg);
    }
    //printf("button stack free %d", uxTaskGetStackHighWaterMark(NULL));
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

/**
 * @brief  main application
 * @param  None
 * @retval None
 */
void app_main(void) {
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
 vTaskDelay(6000 / portTICK_PERIOD_MS);
 // get NODE name from NVS 
 nvs_handle_t my_nvs_handle;
    ret = nvs_open("storage", NVS_READONLY, &my_nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(pcTaskGetName(0),"Error (%s) opening NVS handle!\n", esp_err_to_name(ret));
    } else {
        size_t required_size;
        nvs_get_str(my_nvs_handle, "my_node_id", NULL, &required_size);
        l_my_node_id = malloc(required_size);
        nvs_get_str(my_nvs_handle, "my_node_id", l_my_node_id, &required_size);
        ESP_LOGI(pcTaskGetName(0),"node id %s, size %d",l_my_node_id, required_size);
        nvs_get_str(my_nvs_handle, "board_type", NULL, &required_size);
        l_board_type = malloc(required_size);
        nvs_get_str(my_nvs_handle, "board_type", l_board_type, &required_size);
        ESP_LOGI(pcTaskGetName(0),"board type %s, size %d",l_board_type, required_size);
       nvs_close(my_nvs_handle);
        }
  // Create Semaphore
  xSemaphoreData = xSemaphoreCreateBinary();
  configASSERT(xSemaphoreData);
  xSemaphoreGive(xSemaphoreData);
  esp_log_level_set("*",ESP_LOG_WARN);

  gpio_init();
  wifi_init();
  espnow_init();
  
}
