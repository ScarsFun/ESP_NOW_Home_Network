/*@file   ESPNOW_sender.c
  @brief  implementation of ESPNOW protocol (sender end code)
  @author bheesma-10
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
#include "nvs_flash.h"
#include "espnow.h"
#include "esp_task_wdt.h"

/*receiver and sender ESP32 mac addresses(change these as per your device)*/
// uint8_t receiver_MAC[ESP_NOW_ETH_ALEN] = {0X10, 0x91, 0xa8, 0x36, 0x67,
// 0x28}; uint8_t sender_MAC []  = {0x7c,0xdf,0xa1,0xaf,0xd0,0x30};
uint8_t gw_MAC[ESP_NOW_ETH_ALEN] = {};
static const uint8_t broadcast_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


/*led pin definition and pin mask*/
#define GPIO_OUTPUT_LED CONFIG_LED_GPIO
#define GPIO_OUTPUT_PIN_SEL (1ULL << GPIO_OUTPUT_LED)
#define GPIO_INPUT_BUTTON CONFIG_BUTTON_GPIO
#define GPIO_INPUT_BUTTON_SEL (1ULL << GPIO_INPUT_BUTTON)

espnow_send_param_t *send_param;
/*Queue handle*/
xQueueHandle espnow_queue;
SemaphoreHandle_t xSemaphoreData;
EventGroupHandle_t scan_ch_group;
EventGroupHandle_t ack_flag;
TaskHandle_t esp_now_task_handle = NULL;
void button_task(void *pvParameters);


uint8_t espnow_send_errors = 0;
#define MAX_ESPNOW_SEND_ERRORS 4

#define BROADCAST_CONNECTED_BIT BIT0
#define ACK_RECEIVED_BIT BIT0

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

  /*configure output led pin*/
  pin_conf.pin_bit_mask = GPIO_INPUT_BUTTON_SEL;
  pin_conf.mode = GPIO_MODE_INPUT;
  pin_conf.pull_up_en = true;
  pin_conf.pull_down_en = false;
  pin_conf.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&pin_conf);
}

bool process_rcv_data(char * data_to_process)
{
  if (!strcmp(data_to_process, "LED_ON"))
  {
    ESP_ERROR_CHECK(gpio_set_level(GPIO_OUTPUT_LED, 1));
    ESP_LOGI(pcTaskGetName(0), "LED is ON");
    return true;
  }

  else if (!strcmp(data_to_process, "LED_OFF"))
  {
    ESP_ERROR_CHECK(gpio_set_level(GPIO_OUTPUT_LED, 0));
    ESP_LOGI(pcTaskGetName(0), "LED is OFF");
    return true;
  }
  else
    return false;
}



/**
 * @brief  sending callback of ESPNOW
 * @param  mac address of sending device, status of the transmission
 * @retval None
 */
static void espnow_send_cb(const uint8_t *mac_addr,
                           esp_now_send_status_t status) {
  espnow_event_t evt;
  espnow_event_send_cb_t send_cb;

  if (mac_addr == NULL) {
    ESP_LOGE(pcTaskGetName(0), "Send cb arg error");
    return;
  }
  memcpy(send_cb.mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
  send_cb.status = status;

  evt.id = ESPNOW_SEND_CB;
  evt.info.send_cb = send_cb;

  if (xQueueSend(espnow_queue, &evt, 200) != pdTRUE) {
    ESP_LOGW(pcTaskGetName(0), "Send send queue fail");
  }
}

/**
 * @brief  receiving callback of ESPNOW
 * @param  mac address of received device, data received and length of received
 * data
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
    ESP_LOGE(pcTaskGetName(0), "Malloc receive data fail");
    return;
  }
  memcpy(recv_cb->data, data, len);
  recv_cb->data_len = len;
  if (xQueueSend(espnow_queue, &evt, 200) != pdTRUE) {
    ESP_LOGW(pcTaskGetName(0), "Send receive queue fail");
    free(recv_cb->data);
  }
  xSemaphoreGive(xSemaphoreData);
  ESP_LOGD(pcTaskGetName(0), "espnow_recv_cb Finish");
}

/**
 * @brief  receiving callback of ESPNOW
 * @param  mac address of received device, data received and length of received
 * data
 * @retval None
 */
static void espnow_deinit(void) {
  free(send_param->buffer);
  free(send_param);
  vSemaphoreDelete(espnow_queue);
  esp_now_deinit();
}

/**
 * @brief  ESPNOW main task
 * @param  task parameters
 * @retval None
 * @note   this task sends data and waits to receive ack,checks if ack data is
 * correct and then enters transmit mode again...
 */
void esp_now_task(void *pvParameters) {
  espnow_event_t evt;
  espnow_data_t recv_data;
  ESP_LOGI(pcTaskGetName(0), "Start...");
  for (;;) {
    while (xQueueReceive(espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
      switch (evt.id) {
      case ESPNOW_SEND_CB: {
        ESP_LOGI(pcTaskGetName(0), "send cb task complete");

      } break;
      case ESPNOW_RECV_CB: {
        char *ack = "#ACK#";
        char *ack_broadcast = "#ACKBROADCAST#";
        char *my_node_id = CONFIG_NODE_ID;
        xSemaphoreTake(xSemaphoreData, portMAX_DELAY);
        espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
        memcpy((char *)&recv_data, (char *)recv_cb->data, recv_cb->data_len);
        free(recv_cb->data);
        xSemaphoreGive(xSemaphoreData);
        ESP_LOGI(pcTaskGetName(0), "receive cb task");
        ESP_LOGI(pcTaskGetName(0), "%d", evt.info.recv_cb.data_len);
        //espnow_data_t *rcv_data  = (espnow_data_t *)recv_cb->data;
        if (!memcmp(&recv_data, ack, strlen(ack))) {
          ESP_LOGI(pcTaskGetName(0), "ACK OK");
          xEventGroupSetBits(ack_flag, ACK_RECEIVED_BIT);
          //xTaskCreate(led_task, "LED_TASK", 2500, (void *)500, 5, NULL);
        } else if (!memcmp(&recv_data, ack_broadcast,
                           strlen(ack_broadcast))) {
          memcpy(gw_MAC, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
          xEventGroupSetBits(scan_ch_group, BROADCAST_CONNECTED_BIT);
          ESP_LOGI(pcTaskGetName(0), "ACK BROADCAST OK");

        } else if (!memcmp(&recv_data.node_id, my_node_id, strlen(my_node_id))) {
          // received MQTT frame do something
          ESP_LOGW(pcTaskGetName(0), "MQTT Data: %s", recv_data.payload);
          if(!process_rcv_data(recv_data.payload))
            ESP_LOGE(pcTaskGetName(0), "invalid data received");
        } else {
          __asm__ __volatile__("nop;nop;nop;nop;nop;nop;nop;");
        }
      }

      break;
      default: {
        ESP_LOGE(pcTaskGetName(0), "Callback type error: %d", evt.id);
      } break;
      
      }
    }
  }
  vTaskDelete(NULL);
}

void scan_gw_channel(void *pvParameters) {
  // ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
  // ESP_ERROR_CHECK(esp_task_wdt_status(NULL));
  esp_now_peer_num_t num_peers;

  /*add broadcast address to peer list*/
  esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
  if (peer == NULL) {
    ESP_LOGE(pcTaskGetName(0), "Malloc peer information fail");
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
  /* Initialize sending parameters. */
  send_param = malloc(sizeof(espnow_send_param_t));
  if (send_param == NULL) {
    ESP_LOGE(pcTaskGetName(0), "Malloc send parameter fail");
  }
  memset(send_param, 0, sizeof(espnow_send_param_t));
  send_param->len = sizeof(espnow_data_t);
  send_param->buffer = malloc(send_param->len);
  espnow_data_t *buf = (espnow_data_t *)send_param->buffer;
  assert(send_param->len >= sizeof(espnow_data_t));
  strcpy(buf->topic, "#PAIRING#");
  strcpy(buf->payload, "");
  strcpy(buf->node_id, CONFIG_NODE_ID);
  xTaskCreate(esp_now_task, "ESPNOW_Task", 5000, NULL, 2, &esp_now_task_handle);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  EventBits_t bits;
  for (uint8_t i = 1; i < WIFI_MAX_CHANNEL; i++) {
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
                               pdFALSE, 1000 / portTICK_PERIOD_MS);
    if (bits & BROADCAST_CONNECTED_BIT) {
      ESP_LOGI(pcTaskGetName(0), "received on CH %d ...", i);
      peer->channel = i;
      memcpy(peer->peer_addr, gw_MAC, sizeof(gw_MAC));
      ESP_ERROR_CHECK(esp_now_add_peer(peer));
      ESP_ERROR_CHECK(esp_now_del_peer(broadcast_mac));
      memcpy(send_param->dest_mac, peer->peer_addr, ESP_NOW_ETH_ALEN);
      xTaskCreate(button_task, "BUTTON_Task", 1024 * 4, NULL, 2, NULL);
      break;
    };
    //esp_task_wdt_reset();
    vTaskDelay(200 / portTICK_PERIOD_MS);
  };
  if (!(bits & BROADCAST_CONNECTED_BIT)) {
    ESP_LOGE(pcTaskGetName(0), "channel scan failed");
    espnow_deinit();
    free(peer);
    // ESP_ERROR_CHECK(esp_task_wdt_delete(NULL));
    vTaskDelete(NULL);
  };

  ESP_ERROR_CHECK(esp_now_get_peer_num(&num_peers));
  for (int num_peer = 0; num_peer < num_peers.total_num; num_peer++) {
    esp_now_get_peer(broadcast_mac, peer);
    ESP_LOGI(pcTaskGetName(0), "channel:%d", peer->channel);
    ESP_LOGI(pcTaskGetName(0), "peer address " MACSTR "",
             MAC2STR(peer->peer_addr));
  }
  free(peer);
// ESP_ERROR_CHECK(esp_task_wdt_delete(NULL));
  vTaskDelete(NULL);
}

/**
 * @brief  Wifi mode init
 * @param  None
 * @retval None
 * @note   WiFi should start before using ESPNOW. also long range is enabled so
 * TX power is high and bandwidth low
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
 * generating sending data
 */
void espnow_init(void) {
  scan_ch_group = xEventGroupCreate();
  xEventGroupClearBits(scan_ch_group, BROADCAST_CONNECTED_BIT);

  ack_flag = xEventGroupCreate();
  xEventGroupClearBits(scan_ch_group, ACK_RECEIVED_BIT);

  /*create queue for application*/
  espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
  if (espnow_queue == NULL) {
    ESP_LOGE(pcTaskGetName(0), "Create mutex fail");
  }

  ESP_ERROR_CHECK(esp_now_init());
  ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
  ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
  /*set primary key*/
  ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)CONFIG_ESP_NOW_PMK));
  /*create task*/
  xTaskCreate(scan_gw_channel, "ESPNOW_scan_CH", 1024 * 4, NULL, 2, NULL);
}

void button_task(void *pvParameters) {
  EventBits_t ack_bits;
  for (;;) {
    if (gpio_get_level(GPIO_INPUT_BUTTON) == 0) {
      espnow_data_t *buf = (espnow_data_t *)send_param->buffer;
      assert(send_param->len >= sizeof(espnow_data_t));
      ESP_LOGW(pcTaskGetName(0), "button pressed");
      strcpy(buf->topic, CONFIG_ESP_MQTT_CLIENT_PUBBLISH);
      strcpy(buf->payload, CONFIG_MESSAGE_PAYLOAD);
      strcpy(buf->node_id, CONFIG_NODE_ID);

      if (esp_now_send(send_param->dest_mac, send_param->buffer,
                       send_param->len) != ESP_OK) {
        ESP_LOGE(pcTaskGetName(0), "espnow Send error");
      }
      ack_bits = xEventGroupWaitBits(ack_flag, ACK_RECEIVED_BIT, pdTRUE,
                                     pdFALSE, 1500 / portTICK_PERIOD_MS);
      if ((ack_bits & ACK_RECEIVED_BIT))
        espnow_send_errors = 0;
      else {
        espnow_send_errors++;
        ESP_LOGE(pcTaskGetName(0), "espnow ACK error %d", espnow_send_errors);
      }
    }
    vTaskDelay(1500 / portTICK_PERIOD_MS);

    if (espnow_send_errors > MAX_ESPNOW_SEND_ERRORS) {
      // rescan for GW. channel may be changed
      espnow_send_errors = 0;
      esp_now_del_peer(send_param->dest_mac);
      vTaskDelete(esp_now_task_handle);
      xEventGroupClearBits(scan_ch_group, BROADCAST_CONNECTED_BIT);
      xTaskCreate(scan_gw_channel, "ESPNOW_scan_CH", 1024 * 4, NULL, 2, NULL);
      vTaskDelete(NULL);
    }
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

  // Create Semaphore
  xSemaphoreData = xSemaphoreCreateBinary();
  configASSERT(xSemaphoreData);
  xSemaphoreGive(xSemaphoreData);

  gpio_init();
  wifi_init();
  espnow_init();
}
