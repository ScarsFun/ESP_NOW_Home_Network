
/**
 * @defgroup   ESPNOW MQTT biderectional Gateway
 *
 * @brief      This file implements ESPNOW MQTT Gateway
 * This code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
 *
 * @author     FART ELECTRONICA
 * @date       2023
 */


#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "my_espnow.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// OTA
#include "esp_http_client.h"
//#include "esp_ota_ops.h"



#ifdef CONFIG_SNTP_TIME_SYNC
#include "esp_netif_sntp.h"
#include "esp_sntp.h"
#endif


#include "mdns.h"
#include "mqtt.h"

#include "led_mon.h"


QueueHandle_t xQueuePublish, xQueueSubscribe;
QueueHandle_t s_espnow_queue;
EventGroupHandle_t s_wifi_event_group;
SemaphoreHandle_t xSemaphoreData;
SemaphoreHandle_t xSemaphoreEspnowSend;
espnow_send_param_t *send_param;
espnow_peers_table_t peers_table[20];
led_param_t led_param;

/* The event group allows multiple bits for each event, but we only care about
 * two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static int s_retry_num = 0;

typedef struct Data_t
{
    uint8_t ota_destination_mac[ESP_NOW_ETH_ALEN];
    char url[100];
} OtaData_t;


/**
 * @brief      event handler for AP connection
 *
 * @param      arg         The argument
 * @param[in]  event_base  The event base
 * @param[in]  event_id    The event identifier
 * @param      event_data  The event data
 */
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    if (s_retry_num < CONFIG_STA_MAXIMUM_RETRY) {
      esp_wifi_connect();
      s_retry_num++;
      ESP_LOGI(pcTaskGetName(0), "retry to connect to the AP");
    } else {
      xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    }
    ESP_LOGI(pcTaskGetName(0), "connect to the AP fail");
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(pcTaskGetName(0), "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    s_retry_num = 0;
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}


/**
 * @brief      WiFi should start before using ESPNOW
 */
static void wifi_init(void) {
  esp_log_level_set("wifi", ESP_LOG_WARN);
  static bool initialized = false;
  if (initialized) {
    return;
  }
  s_wifi_event_group = xEventGroupCreate();
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
  assert(ap_netif);
  esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
  assert(sta_netif);
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_event_handler_register(
                    WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                  &event_handler, NULL));

  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  // ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_NULL) );
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
  ESP_ERROR_CHECK(esp_wifi_start());

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
  ESP_ERROR_CHECK(esp_wifi_set_protocol(
                    ESP_IF_WIFI_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G |
                    WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
#endif

  initialized = true;
}
/**
 * @brief      configure station mode and connect to AP
 *
 * @return     { description_of_the_return_value }
 */
static bool wifi_sta(void) {
  wifi_config_t sta_config = {0};
  strcpy((char *)sta_config.sta.ssid, CONFIG_STA_WIFI_SSID);
  strcpy((char *)sta_config.sta.password, CONFIG_STA_WIFI_PASS);

  // ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_APSTA) );
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &sta_config));

  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_LOGI(pcTaskGetName(0), "WIFI_MODE_STA started.");

  ESP_ERROR_CHECK(esp_wifi_connect());

  /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or
   * connection failed for the maximum number of re-tries (WIFI_FAIL_BIT). The
   * bits are set by event_handler() (see above) */
  esp_err_t ret = ESP_OK;
  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                         pdFALSE, pdFALSE, portMAX_DELAY);

  /* xEventGroupWaitBits() returns the bits before the call returned, hence we
   * can test which event actually happened. */
  if (bits & WIFI_CONNECTED_BIT) {

    ESP_LOGI(pcTaskGetName(0), "connected to ap SSID:%s password:%s", CONFIG_STA_WIFI_SSID,
             CONFIG_STA_WIFI_PASS);
    uint8_t sta_mac[6] = {0};
    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
    ESP_LOGI(pcTaskGetName(0), "sta_mac:" MACSTR, MAC2STR(sta_mac));
    uint8_t chPrimary;
    wifi_second_chan_t chSecond;
    ESP_ERROR_CHECK(esp_wifi_get_channel(&chPrimary, &chSecond));
    ESP_LOGI(pcTaskGetName(0), "channel: %d", chPrimary);
    //led_param_t *led_param = malloc(sizeof(led_param_t));
    led_param.delay = 50;
    led_param.rip = 3;
    led_param.led_id = 0;
    led_param.R = 0;
    led_param.G = 60;
    led_param.B = 0;
    xTaskCreate(led_task, "led_task", 512, &led_param, 4, NULL);

  } else if (bits & WIFI_FAIL_BIT) {
    ESP_LOGI(pcTaskGetName(0), "Failed to connect to SSID:%s, password:%s",
             CONFIG_STA_WIFI_SSID, CONFIG_STA_WIFI_PASS);
    ret = ESP_FAIL;
  } else {
    ESP_LOGE(pcTaskGetName(0), "UNEXPECTED EVENT");
    ret = ESP_FAIL;
  }
  vEventGroupDelete(s_wifi_event_group);
  return ret;
}



/**
 * @brief      dummy task only for testing ESPNOW queue
 *             (not used)
 *
 * @param      pvParameters  The pv parameters
 */
void test_task(void *pvParameters) {
  espnow_event_t evt;
  uint32_t delay = (uint32_t)pvParameters;

  evt.id = ESPNOW_TEST_TASK;

  xQueueReset(s_espnow_queue);
  if (xQueueSend(s_espnow_queue, &evt, 200) != pdPASS) {
    ESP_LOGW(pcTaskGetName(0), "send queue fail");
  }

  vTaskDelay(delay / portTICK_PERIOD_MS);
  vTaskDelete(NULL);
}



/**
 * @brief      ESPNOW sending or receiving callback function is called in WiFi task.
 *             Users should not do lengthy operations from this task. Instead, post
 *             necessary data to a queue and handle it from a lower priority task.
 *
 * @param[in]  mac_addr  The mac address
 * @param[in]  status    The status
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
  if (xQueueSend(s_espnow_queue, &evt, portMAX_DELAY) != pdTRUE) {
    ESP_LOGW(pcTaskGetName(0), "Send send queue fail");
  }
  vTaskDelay(500 / portTICK_PERIOD_MS);
  xSemaphoreGive(xSemaphoreEspnowSend);
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 1)
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info,
                           const uint8_t *data, int len)
#else
static void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data,
                           int len)
#endif
{
  ESP_LOGD(__FUNCTION__, "len=%d", len);
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
  if (xQueueSend(s_espnow_queue, &evt, portMAX_DELAY) != pdTRUE) {
    ESP_LOGW(pcTaskGetName(0), "Send receive queue fail");
    free(recv_cb->data);
  }
  xSemaphoreGive(xSemaphoreData);
  ESP_LOGD(pcTaskGetName(0), "espnow_recv_cb Finish");
}

/**
 * @brief      Adds a peer to list.
 *
 * @param      rcv_nodeid  The receive nodeid
 * @param      peer_mac    The peer mac
 * @param      lst_pointer structure list pointer
 *
 * @return     return true on succeed
 */

bool add_peer_to_list(char *rcv_nodeid, uint8_t *peer_mac, espnow_peers_table_t **lst_pointer) {
  static uint8_t idx = 0;
  espnow_peers_table_t *tb = &peers_table[0];
  if (idx > 19) return false;
  memcpy((char *)(tb + idx)->node_id, (char *)rcv_nodeid, 16);
  memcpy((uint8_t *)(tb + idx)->mac_addr, (uint8_t *)peer_mac, ESP_NOW_ETH_ALEN);
  *lst_pointer = (tb + idx);
  ESP_LOGI(pcTaskGetName(0), "list pointer %p", *lst_pointer);
  ESP_LOGI(pcTaskGetName(0), "added node: %s to list", (tb + idx)->node_id);
  ESP_LOGI(pcTaskGetName(0), "node mac:" MACSTR " ", MAC2STR((tb + idx)->mac_addr));
  idx++;
  return true;
}

/**
 * @brief      { function_description }
 *
 * @param      pvParameter  The pv parameter
 */
static void espnow_task(void *pvParameter) {
  ESP_LOGI(pcTaskGetName(0), "ESPNOW Start...");
  espnow_event_t evt;
  esp_now_peer_num_t num_peers;

  uint8_t chPrimary;
  wifi_second_chan_t chSecond;
  ESP_ERROR_CHECK(esp_wifi_get_channel(&chPrimary, &chSecond));
  // ESP_LOGI(pcTaskGetName(0), "channel: %d", chPrimary);

  send_param = malloc(sizeof(espnow_send_param_t));
  memset(send_param, 0, sizeof(espnow_send_param_t));
  if (send_param == NULL) {
    ESP_LOGE(pcTaskGetName(0), "Malloc send parameter fail");
  }

  MQTT_t mqttBuf;
  while (xQueueReceive(s_espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
    switch (evt.id) {
    case ESPNOW_SEND_CB: {
      ESP_LOGI(pcTaskGetName(0), "send cb task complete");

    } break;
    case ESPNOW_RECV_CB: {
      xSemaphoreTake(xSemaphoreData, portMAX_DELAY);
      espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
      ESP_LOGI(pcTaskGetName(0), "espnow frame received");
      uint8_t data_type = ((uint8_t *)recv_cb->data)[0];
      switch (data_type) {
      case ESPNOW_TYPE_PAIRING_REQUEST:
      {
        espnow_info_t recv_data;
        memcpy((char *)&recv_data, (char *)recv_cb->data, recv_cb->data_len);
        free(recv_cb->data);
        xSemaphoreGive(xSemaphoreData);

        // Add peer to list
        ESP_LOGI(pcTaskGetName(0), "received PAIRING REQUEST ");
        esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
        if (peer == NULL) ESP_LOGE(pcTaskGetName(0), "Malloc peer fail");
        memset(peer, 0, sizeof(esp_now_peer_info_t));
        if (esp_now_get_peer(recv_cb->mac_addr, peer) ==
            ESP_ERR_ESPNOW_NOT_FOUND) {
          peer->channel = chPrimary;
          peer->ifidx = ESP_IF_WIFI_STA;
          peer->encrypt = false;
          espnow_peers_table_t *list_pointer = NULL;
          add_peer_to_list((char *)&recv_data.node_id, recv_cb->mac_addr, &list_pointer);
          peer->priv = (void*)list_pointer; // store pointer to peer list item in private peer strucrure
          ESP_LOGI(pcTaskGetName(0), "peer->priv: %p", peer->priv);
          //
          memcpy(peer->peer_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
          ESP_LOGI(pcTaskGetName(0), "peer->mac:" MACSTR "",
                   MAC2STR(peer->peer_addr));
          ESP_ERROR_CHECK(esp_now_add_peer(peer));
          free(peer);
          ESP_LOGI(pcTaskGetName(0), "ADDED PEER TO LIST");
          ESP_ERROR_CHECK(esp_now_get_peer_num(&num_peers));
          ESP_LOGI(pcTaskGetName(0), "no of peers in peers list:%d", num_peers.total_num);
        }
        
          //char ack_broadcast[] = "#ACKBROADCAST#";
          send_param->len = sizeof(ESPNOW_TYPE_PAIRING_ACK);
          send_param->buffer = malloc(send_param->len);
          //ESP_LOGI(pcTaskGetName(0), "OK MALLOC");
          send_param->buffer[0] = (uint8_t) ESPNOW_TYPE_PAIRING_ACK;
          
      } break;
      case ESPNOW_TYPE_MQTT_PUBBLISH:  // PUBBLISH to MQTT server
      {
        espnow_mqtt_data_t recv_data;
        memcpy((char *)&recv_data, (char *)recv_cb->data, recv_cb->data_len);
        free(recv_cb->data);
        xSemaphoreGive(xSemaphoreData);

        ESP_LOGW(pcTaskGetName(0), "received MQTT from ESPNOW");
        ESP_LOGI(pcTaskGetName(0), "recv_data.topic=[%s]", recv_data.topic);
        ESP_LOGI(pcTaskGetName(0), "recv_data.node_id=[%s]", recv_data.node_id);
        ESP_LOGI(pcTaskGetName(0), "recv_data.payload=[%s]", recv_data.payload);

        mqttBuf.topic_type = PUBLISH;
        sprintf(mqttBuf.topic, "%s%s", recv_data.topic, recv_data.node_id);
        ESP_LOGI(pcTaskGetName(0), "MQTT topic %s", mqttBuf.topic);
        mqttBuf.topic_len = strlen(mqttBuf.topic);
        strcpy(mqttBuf.data, recv_data.payload);
        mqttBuf.data_len = strlen(mqttBuf.data);
        ESP_LOGI(pcTaskGetName(0), "PUBBLISH to MQTT server");
        xQueueSend(xQueuePublish, &mqttBuf, 0); 
        send_param->len = sizeof(ESPNOW_TYPE_ACK);
        send_param->buffer = malloc(send_param->len);
        send_param->buffer[0] = (uint8_t) ESPNOW_TYPE_ACK;
      } break;
      }
      // send  ACK to sender
      memcpy(send_param->dest_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);

      ESP_LOGI(pcTaskGetName(0), "send ACK to:" MACSTR,
               MAC2STR(send_param->dest_mac));
      if (esp_now_send(send_param->dest_mac, send_param->buffer,
                       send_param->len) != ESP_OK)
        ESP_LOGE(pcTaskGetName(0), "Send ACK error ");

    } break;
    case ESPNOW_TEST_TASK: {
    } break;
    default: {
      ESP_LOGE(pcTaskGetName(0), "Callback type error: %d", evt.id);
    } break;
    }
  }  // end while
  vTaskDelete(NULL);
}


/**
 * @brief      parse from MQTT topic
 *
 * @param      parsestr  The topic received from MQTT server
 * @param      parsed    The parsed node name
 *
 * @return     true on succeed
 */
bool parse_nodename(char *parsestr, char *parsed) {
  char *token, *str, *tofree;

  tofree = str = strdup(parsestr);
  while ((token = strsep(&str, "/"))) {
    if (!strcmp(token, CONFIG_PARSE_TOPIC_KEY)) {
      token = strsep(&str, "/");
      strcpy(parsed, token);
      free(tofree);
      return true;
    }
  }
  free(tofree);
  ESP_LOGE(pcTaskGetName(0), "cannot parse node_id");
  return false;
}


/**
 * @brief      Fetches a peer from peers_table
 *
 * @param      node     The node name
 * @param      macaddr  The macaddr
 *
 * @return     true on succeed
 */
bool fetch_peer(char *node, uint8_t *macaddr) {
  esp_now_peer_num_t num_peers;
  espnow_peers_table_t *tb = &peers_table[0];
  ESP_ERROR_CHECK(esp_now_get_peer_num(&num_peers));
  ESP_LOGI(pcTaskGetName(0), "num_peers %d", num_peers.total_num);
  for (uint8_t num_peer = 0; num_peer < num_peers.total_num; num_peer++) {
    if (!strcmp(node, (tb + num_peer)->node_id))
    {
      ESP_LOGI(pcTaskGetName(0), "fetched node %s", (char *)(tb + num_peer)->node_id);
      memcpy(macaddr, (tb + num_peer)->mac_addr, ESP_NOW_ETH_ALEN);
      return true;
    }
  }
  ESP_LOGE(pcTaskGetName(0), "node not in list");
  return false;
}

static void test_send (void *mac_addr){

  const uint8_t *node_mac_addr = malloc(ESP_NOW_ETH_ALEN);
  memcpy(node_mac_addr,mac_addr,ESP_NOW_ETH_ALEN);

  esp_err_t ret       = ESP_OK;

  espnow_ota_data_t *buf = (espnow_ota_data_t *)send_param->buffer;
  send_param->len = sizeof(espnow_ota_data_t);
  buf->type = ESPNOW_TYPE_OTA_DATA;
  buf->sequence = 0;
  memcpy(send_param->dest_mac, node_mac_addr, ESP_NOW_ETH_ALEN);
  //vTaskDelay(15000 / portTICK_PERIOD_MS);
  strcpy((char *)buf->ota_chunk,"454914934404585090055358134009287453692074567160473529335377493133432283509931663883034958156696508097090");
  for(int i=0; i<15; i++){
  //  vTaskDelay(200 / portTICK_PERIOD_MS);
  xSemaphoreTake(xSemaphoreEspnowSend, portMAX_DELAY);
  ret = esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len);
  if (ret != ESP_OK) 
    ESP_LOGE(pcTaskGetName(0), "ERROR Send Data");
  buf->sequence++;
}
//free(send_param->buffer);
vTaskDelay(100 / portTICK_PERIOD_MS);
vTaskDelete(NULL);
}

//static void ota_firmware_task(void *Data)
static size_t app_firmware_download(const char *url, const uint8_t *node_mac_addr)
{
#define OTA_DATA_PAYLOAD_LEN 200
  static const char *TAG = "OTA";
  esp_err_t ret       = ESP_OK;
  //esp_ota_handle_t ota_handle = 0;
  uint8_t *data       = malloc(OTA_DATA_PAYLOAD_LEN);
  size_t total_size   = 0;
 // uint32_t start_time = xTaskGetTickCount();

  //OtaData_t sample_ota;
  //strcpy(abc.char1,(const char*)(( struct1*)p)->char1);
  //strcpy(sample_ota.url, (const char*)(( OtaData_t*)Data)->url);
  //memcpy(sample_ota.ota_destination_mac, (const uint8_t*)(( OtaData_t*)Data)->ota_destination_mac, ESP_NOW_ETH_ALEN);

  esp_http_client_config_t config = {
    .url            = url,
    .transport_type = HTTP_TRANSPORT_UNKNOWN,
  };

  //ESP_LOGI(pcTaskGetName(0), "peer address " MACSTR "", MAC2STR((uint8_t *) sample_ota.ota_destination_mac));
  //ESP_LOGI(pcTaskGetName(0), "Open HTTP connection: %s", (char*)sample_ota.url);

  /**
   * @brief 1. Connect to the server
   */
  esp_http_client_handle_t client = esp_http_client_init(&config);
  if (!client)
  {
    ESP_LOGE(TAG, "Init HTTP connection: %s", url);
    free(data);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    return 0;
  }

  ESP_LOGI(TAG, "Open HTTP connection: %s", url);

  /**
   * @brief First, the firmware is obtained from the http server and stored
   */
  do {
    ret = esp_http_client_open(client, 0);

    if (ret != ESP_OK) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      ESP_LOGW(TAG, "<%s> Connection service failed", esp_err_to_name(ret));
    }
  } while (ret != ESP_OK);

  total_size = esp_http_client_fetch_headers(client);

  if (total_size <= 0) {
    ESP_LOGW(TAG, "Please check the address of the server");
    ret = esp_http_client_read(client, (char *)data, OTA_DATA_PAYLOAD_LEN);
    if (ret < 0) {
      ESP_LOGE(TAG, "<%s> Read data from http stream", esp_err_to_name(ret));
      free(data);
      esp_http_client_close(client);
      esp_http_client_cleanup(client);
      return 0;
    }

    ESP_LOGW(TAG, "Recv data: %.*s", ret, data);
    free(data);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    return 0;
  }

  /**
   * @brief 2. Read firmware from the server and write it to the flash of the root node
   */
  espnow_ota_data_t *buf = (espnow_ota_data_t *)send_param->buffer;
  send_param->len = sizeof(espnow_ota_data_t);
  buf->type = ESPNOW_TYPE_OTA_DATA;
  buf->sequence = 0;

  memcpy(send_param->dest_mac, node_mac_addr, ESP_NOW_ETH_ALEN);

  for (ssize_t size = 0, recv_size = 0; recv_size < total_size; recv_size += size)
  {
    xSemaphoreTake(xSemaphoreEspnowSend, portMAX_DELAY);
    size = esp_http_client_read(client, (char *)data, OTA_DATA_PAYLOAD_LEN);
    if (size < 0) {
      ESP_LOGE(TAG, "<%s> Read data from http stream", esp_err_to_name(ret));
      free(data);
      esp_http_client_close(client);
      esp_http_client_cleanup(client);
      return 0;
    }

    if (size > 0) {
      memcpy((char *)buf->ota_chunk, (char *)data, size);
      //free(data);
      ret = esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len);
      //free(send_param->buffer);
      //ESP_LOGI(TAG, "Send firmware node , size: %u",size);
      if (ret != ESP_OK) {
        ESP_LOGE(TAG, "<%s> Send firmware node , size: %u, data: %.*s",
                 esp_err_to_name(ret), size, size, data);
        free(data);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return 0;
      }
      //free(data);
      //ESP_LOGI(TAG, "sent chunk nr: %d", buf->sequence);
      buf->sequence++;

    } else {
      ESP_LOGW(TAG, "<%s> esp_http_client_read", esp_err_to_name(ret));
      free(data);
      esp_http_client_close(client);
      esp_http_client_cleanup(client);
      return 0;
    }
  }

//  ESP_LOGI(TAG, "The service download firmware is complete, Spend time: %" PRIu32 "s",
//           (xTaskGetTickCount() - start_time) * portTICK_PERIOD_MS / 1000);

  free(data);
  esp_http_client_close(client);
  esp_http_client_cleanup(client);
  return total_size;

}
/*
void ota_task(void *pvParameters) {
vTaskDelay(15000 / portTICK_PERIOD_MS);
ESP_LOGI(pcTaskGetName(0), "OTA start");
uint8_t ota_destination_mac[ESP_NOW_ETH_ALEN];
fetch_peer("NOD001", ota_destination_mac);
app_firmware_download("http://192.168.1.5:8070/LICENSE", ota_destination_mac);

vTaskDelete(NULL);

}*/

/**
 * @brief      publish MQTT data from Gateway to the Server. defined in mqtt.c
 *
 * @param      pvParameters  The pv parameters
 */
void mqtt_pub(void *pvParameters);



/**
 * @brief      Gets MQTT data from Server and submits to specific node
 *
 * @param      pvParameters  The pv parameters
 */
void mqtt_sub(void *pvParameters) {
  ESP_LOGI(pcTaskGetName(0), "Start receive MQTT...");
  MQTT_t mqttBuf_rx;
  while (1) {
    xQueueReceive(xQueueSubscribe, &mqttBuf_rx, portMAX_DELAY);
    if (mqttBuf_rx.topic_type == SUBSCRIBE) {
      char nodename[8];
      uint8_t destination_mac[ESP_NOW_ETH_ALEN];
      espnow_mqtt_data_t rcv_data;

      ESP_LOGI(pcTaskGetName(0), "mqtt_rx topic=[%s] topic_len=%d",
               mqttBuf_rx.topic, mqttBuf_rx.topic_len);
      ESP_LOGI(pcTaskGetName(0), "MQTT received data %s", mqttBuf_rx.data);
      if (parse_nodename(mqttBuf_rx.topic, nodename) &&
          fetch_peer(nodename, destination_mac)) {
        ESP_LOGI(pcTaskGetName(0), "Sending data to ESPNOW node : %s",
                 nodename);
        ESP_LOGI(pcTaskGetName(0), "peer address " MACSTR "",
                 MAC2STR(destination_mac));
        rcv_data.type = ESPNOW_TYPE_MQTT_SUBSCRIBE;
        strcpy(rcv_data.node_id, nodename);
        strcpy(rcv_data.payload, mqttBuf_rx.data);
        memcpy(send_param->dest_mac, destination_mac, ESP_NOW_ETH_ALEN);
        send_param->buffer = (uint8_t *)&rcv_data;
        send_param->len = sizeof(espnow_mqtt_data_t);
        if (esp_now_send(send_param->dest_mac, send_param->buffer,
                         send_param->len) != ESP_OK)
          ESP_LOGE(pcTaskGetName(0), "ESPNOW Send error ");
      } else
        ESP_LOGE(pcTaskGetName(0), "peer not in list. MQTT data not sent");
    }
  }
}

#ifdef CONFIG_SNTP_TIME_SYNC
void time_sync_notification_cb(struct timeval *tv)
{
  ESP_LOGI(pcTaskGetName(0), "Notification of a time synchronization event");
}

void SNTP_init (void)
{
  esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
  config.sync_cb = time_sync_notification_cb;
  esp_netif_sntp_init(&config);

  // wait for time to be set
  time_t now = 0;
  struct tm timeinfo = { 0 };
  int retry = 0;
  const int retry_count = 15;
  while (esp_netif_sntp_sync_wait(2000 / portTICK_PERIOD_MS) == ESP_ERR_TIMEOUT && ++retry < retry_count) {
    ESP_LOGI(pcTaskGetName(0), "Waiting for system time to be set... (%d/%d)", retry, retry_count);
  }
  time(&now);
  char strftime_buf[64];
  // Rome TimeZone
  setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
  tzset();
  localtime_r(&now, &timeinfo);
  strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
  ESP_LOGI(pcTaskGetName(0), "The current date/time in Rome is: %s", strftime_buf);
}
#endif

void app_main(void) {
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  configure_led();

  // Initialize WiFi
  wifi_init();

  // Start STA mode
  if (wifi_sta() != ESP_OK) {
    while (1) {
      vTaskDelay(10);
    }
  }

#ifdef CONFIG_SNTP_TIME_SYNC
  // initialize SNTP time sync
  SNTP_init ();
#endif
  // Initialize mDNS
  ESP_ERROR_CHECK(mdns_init());

  // Create Queue
  s_espnow_queue = xQueueCreate(10, sizeof(espnow_event_t));
  configASSERT(s_espnow_queue);
  xQueuePublish = xQueueCreate(10, sizeof(MQTT_t));
  configASSERT(xQueuePublish);

  xQueueSubscribe = xQueueCreate(10, sizeof(MQTT_t));
  configASSERT(xQueueSubscribe);

  // Create Semaphore
  xSemaphoreData = xSemaphoreCreateBinary();
  configASSERT(xSemaphoreData);
  xSemaphoreGive(xSemaphoreData);

  xSemaphoreEspnowSend = xSemaphoreCreateBinary();
  configASSERT(xSemaphoreEspnowSend);
  xSemaphoreGive(xSemaphoreEspnowSend);



  /* Initialize ESPNOW and register sending and receiving callback function. */
  ESP_ERROR_CHECK(esp_now_init());
  // send callback not use
  ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
  ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
  /*set primary key*/
  ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)CONFIG_ESP_NOW_PMK));


  
  xTaskCreate(espnow_task, "ESPNOW", 1024 * 4, NULL, 3, NULL);
  //xTaskCreate(test_task, "TEST", 1024 * 4, NULL, 4, NULL);
  //mqtt_init();
  //xTaskCreate(mqtt_pub, "MQTT_PUB", 1024 * 4, NULL, 2, NULL);
  //xTaskCreate(mqtt_sub, "MQTT_SUB", 1024 * 4, NULL, 2, NULL);
  
vTaskDelay(15000 / portTICK_PERIOD_MS);
uint8_t destination_mac[ESP_NOW_ETH_ALEN] ;
fetch_peer("NOD001", destination_mac);
xTaskCreate(test_send, "OTA_TASK", 1024 * 4, &destination_mac, 3, NULL);
//test_send (destination_mac);
//app_firmware_download("http://192.168.1.5:8070/LICENSE", destination_mac);


}
