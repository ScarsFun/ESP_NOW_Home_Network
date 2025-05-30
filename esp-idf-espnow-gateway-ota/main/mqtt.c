/*
   This code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <inttypes.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "mqtt_client.h"
#include "netdb.h" // gethostbyname
#include "mdns.h"
#include "esp_netif_ip_addr.h"
#include "driver/twai.h"
#include "led_mon.h"
#include "mqtt.h"

//static const char *SUB = "SUB";
//static const char *PUB = "PUB";
//static const char *TAG = "MQTT";

static EventGroupHandle_t s_mqtt_event_group;
static esp_mqtt_client_handle_t mqtt_client;

#define MQTT_CONNECTED_BIT BIT0

extern QueueHandle_t xQueuePublish;
extern QueueHandle_t xQueueSubscribe;
extern led_param_t led_param;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
#else
static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
#endif
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
	esp_mqtt_event_handle_t event = event_data;
#endif

	MQTT_t mqttBuf;
	switch (event->event_id) {
		case MQTT_EVENT_CONNECTED:
			ESP_LOGI(pcTaskGetName(0), "MQTT_EVENT_CONNECTED");
			xEventGroupSetBits(s_mqtt_event_group, MQTT_CONNECTED_BIT);
			//esp_mqtt_client_subscribe(mqtt_client, CONFIG_SUB_TOPIC, 0);
			break;
		case MQTT_EVENT_DISCONNECTED:
			ESP_LOGW(pcTaskGetName(0), "MQTT_EVENT_DISCONNECTED");
			xEventGroupClearBits(s_mqtt_event_group, MQTT_CONNECTED_BIT);
			break;
		case MQTT_EVENT_SUBSCRIBED:
			ESP_LOGI(pcTaskGetName(0), "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
			break;
		case MQTT_EVENT_UNSUBSCRIBED:
			ESP_LOGI(pcTaskGetName(0), "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
			break;
		case MQTT_EVENT_PUBLISHED:
			ESP_LOGI(pcTaskGetName(0), "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
			break;
		case MQTT_EVENT_DATA:
			ESP_LOGI(pcTaskGetName(0), "MQTT_EVENT_DATA");
			ESP_LOGI(pcTaskGetName(0), "TOPIC=%.*s\r", event->topic_len, event->topic);
			ESP_LOGI(pcTaskGetName(0), "DATA=%.*s\r", event->data_len, event->data);
			
			mqttBuf.topic_type = SUBSCRIBE;
			mqttBuf.topic_len = event->topic_len;
			for(int i=0;i<event->topic_len;i++) {
				mqttBuf.topic[i] = event->topic[i];
				mqttBuf.topic[i+1] = 0;
			}
			mqttBuf.data_len = event->data_len;
			for(int i=0;i<event->data_len;i++) {
				mqttBuf.data[i] = event->data[i];
				mqttBuf.data[i+1] = 0;
			}
			//xQueueSend(xQueueSubscribe, &mqttBuf, 0);
			xQueueSend(xQueueSubscribe, &mqttBuf, 0);
			break;
		case MQTT_EVENT_ERROR:
			ESP_LOGI(pcTaskGetName(0), "MQTT_EVENT_ERROR");
		led_param.delay = 100;
    led_param.rip =2;
    led_param.led_id =0;
    led_param.R = 255;
    led_param.G = 164;
    led_param.B = 8;
    xTaskCreate(led_task, "led_task", 512, &led_param, 4, NULL);
			break;
		default:
			ESP_LOGI(pcTaskGetName(0), "Other event id:%d", event->event_id);
			break;
	}
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
	return ESP_OK;
#endif
}

esp_err_t query_mdns_host(const char * host_name, char *ip)
{
	ESP_LOGD(__FUNCTION__, "Query A: [%s]", host_name);

	struct esp_ip4_addr addr;
	addr.addr = 0;

	esp_err_t err = mdns_query_a(host_name, 10000,	&addr);
	if(err){
		if(err == ESP_ERR_NOT_FOUND){
		 ESP_LOGW(__FUNCTION__, "%s: Host was not found!", esp_err_to_name(err));
		 return ESP_FAIL;
		}
		ESP_LOGE(__FUNCTION__, "Query Failed: %s", esp_err_to_name(err));
		return ESP_FAIL;
	}

	ESP_LOGD(__FUNCTION__, "Query A: %s.local resolved to: " IPSTR, host_name, IP2STR(&addr));
	sprintf(ip, IPSTR, IP2STR(&addr));
	return ESP_OK;
}

void convert_mdns_host(char * from, char * to)
{
	ESP_LOGI(__FUNCTION__, "from=[%s]",from);
	strcpy(to, from);
	char *sp;
	sp = strstr(from, ".local");
	if (sp == NULL) return;

	int _len = sp - from;
	ESP_LOGD(__FUNCTION__, "_len=%d", _len);
	char _from[128];
	strcpy(_from, from);
	_from[_len] = 0;
	ESP_LOGI(__FUNCTION__, "_from=[%s]", _from);

	char _ip[128];
	esp_err_t ret = query_mdns_host(_from, _ip);
	ESP_LOGI(__FUNCTION__, "query_mdns_host=%d _ip=[%s]", ret, _ip);
	if (ret != ESP_OK){ 
		return;
	}
	strcpy(to, _ip);
	ESP_LOGI(__FUNCTION__, "to=[%s]", to);
}


void mqtt_init() {
  ESP_LOGI(pcTaskGetName(0), "Start MQTT");

  uint8_t mac[8];
  ESP_ERROR_CHECK(esp_base_mac_addr_get(mac));
  char *mac64 = malloc(64);
  sprintf(mac64, "%02x %02x %02x %02x %02x %02x %02x %02x", mac[0], mac[1], mac[2],
          mac[3], mac[4], mac[5], mac[6], mac[7]);
  ESP_LOGI(pcTaskGetName(0), "mac EUI-64: %s", mac64);
  free (mac64);
   
  char client_id[64];
  sprintf(client_id, "esp32-%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2],
          mac[3], mac[4], mac[5]);
  ESP_LOGI(pcTaskGetName(0), "client_id=[%s]", client_id);

  // Resolve mDNS host name
  char ip[128];
  ESP_LOGI(pcTaskGetName(0), "CONFIG_MQTT_BROKER=[%s]", CONFIG_MQTT_BROKER);
  //convert_mdns_host(CONFIG_MQTT_BROKER, ip);
  convert_mdns_host("192.168.1.9", ip);
  ESP_LOGI(pcTaskGetName(0), "ip=[%s]", ip);
  char uri[138];
  sprintf(uri, "mqtt://%s", ip);
  ESP_LOGI(pcTaskGetName(0), "uri=[%s]", uri);

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  esp_mqtt_client_config_t mqtt_cfg = {.broker.address.uri = uri,
                                       .credentials.client_id = client_id};
#else
  esp_mqtt_client_config_t mqtt_cfg = {
      .uri = uri,
      .event_handle = mqtt_event_handler,
      .client_id = "subscribe",
  };
#endif

  

  s_mqtt_event_group = xEventGroupCreate();
  mqtt_client = esp_mqtt_client_init(&mqtt_cfg);

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID,
                                 mqtt_event_handler, NULL);
#endif

  xEventGroupClearBits(s_mqtt_event_group, MQTT_CONNECTED_BIT);
  esp_mqtt_client_start(mqtt_client);
  xEventGroupWaitBits(s_mqtt_event_group, MQTT_CONNECTED_BIT, false, true,
                      portMAX_DELAY);
  ESP_LOGI(pcTaskGetName(0), "Connect to MQTT Server");

  // Subscribe topic
  if (esp_mqtt_client_subscribe(mqtt_client, CONFIG_ESP_MQTT_CLIENT_SUBSCRIBE, 0) == -1)
  	{
  	/*	ESP_LOGE(pcTaskGetName(0), "CANNOT Connect to MQTT Server");
    led_param.delay = 100;
    led_param.rip =255;
    led_param.led_id =0;
    led_param.R = 100;
    led_param.G = 0;
    led_param.B = 0;
    xTaskCreate(led_task, "led_task", 256, &led_param, 4, NULL);*/
  	}; 
 
}

/**
 * @brief      publish MQTT data from Gateway to the Server
 *
 * @param      pvParameters  The pv parameters
 */
void mqtt_pub(void *pvParameters)
{
	MQTT_t mqttBuf3;
   while (1) {
  	
    ESP_LOGI(pcTaskGetName(0), "Wait for ESP mqtt topic");
     xQueueReceive(xQueuePublish, &mqttBuf3, portMAX_DELAY);
     //xQueueReset( xQueuePublish );
      if (mqttBuf3.topic_type == PUBLISH) {
        // ESP_LOGI(pcTaskGetName(0), "TOPIC=%.*s\r", mqttBuf.topic_len, mqttBuf.topic);
        ESP_LOGI(pcTaskGetName(0), "topic=[%s] topic_len=%d", mqttBuf3.topic,
                 mqttBuf3.topic_len);
        ESP_LOG_BUFFER_HEXDUMP(pcTaskGetName(0), mqttBuf3.data, mqttBuf3.data_len,
                               ESP_LOG_INFO);
        EventBits_t EventBits = xEventGroupGetBits(s_mqtt_event_group);
        ESP_LOGI(pcTaskGetName(0), "EventBits=0x%" PRIx32, EventBits);
        if (EventBits & MQTT_CONNECTED_BIT) {
          esp_mqtt_client_publish(mqtt_client, mqttBuf3.topic, mqttBuf3.data,
                                  mqttBuf3.data_len, 1, 0);
        } else {
          ESP_LOGE(pcTaskGetName(0), "mqtt broker not connect");
        }
      }  // end if
   }
}