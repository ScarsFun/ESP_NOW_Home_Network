/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef ESPNOW_H
#define ESPNOW_H

#define ESPNOW_PAYLOAD_LEN                  (230)
#define MQTT_PAYLOAD_LEN                    (180)
#define ESP_MQTT_CLIENT_PUBBLISH            "/ESP32/TX/"

/**
 * @brief Type of packet
 */
typedef enum {
    ESPNOW_TYPE_PAIRING_REQUEST,
    ESPNOW_TYPE_PAIRING_ACK,
    ESPNOW_TYPE_ACK,
    ESPNOW_TYPE_MQTT_PUBBLISH,
    ESPNOW_TYPE_MQTT_SUBSCRIBE,
    ESPNOW_TYPE_OTA_DATA,
    ESPNOW_TYPE_OTA_INFO,
    ESPNOW_TYPE_OTA_FIRMWARE_REQUEST,
} espnow_ota_type_t;

typedef enum {
    ESPNOW_STATUS_OTA_ACK_OK,
    ESPNOW_STATUS_OTA_ACK_ERROR,
    ESPNOW_STATUS_OTA_START_FIRMWARE_DOWNLOAD,
    ESPNOW_STATUS_OTA_END_FIRMWARE_DOWNLOAD,
    ESPNOW_STATUS_OTA_ABORT_FIRMWARE_DOWNLOAD
} espnow_status_id_t;

typedef enum {
    ESPNOW_SEND_CB,
    ESPNOW_RECV_CB,
    ESPNOW_TEST_TASK,
} espnow_event_id_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} espnow_event_send_cb_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int data_len;
} espnow_event_recv_cb_t;

typedef union {
    espnow_event_send_cb_t send_cb;
    espnow_event_recv_cb_t recv_cb;
} espnow_event_info_t;

/* When ESPNOW sending or receiving callback function is called, post event to ESPNOW task. */
typedef struct {
    espnow_event_id_t id;
    espnow_event_info_t info;
} espnow_event_t;

typedef struct{
    uint8_t type;                       // < type of packet, ESPNOW_TYPE_OTA_DATA
    char node_id[8]; 
    uint16_t sequence;
    uint8_t ota_chunk[230];
    uint8_t ota_chunk_size;
    uint8_t ota_chunk_crc;
} __attribute__((packed)) espnow_ota_data_t;

/* User defined field of ESPNOW MQTT data in this example. */
typedef struct {
    uint8_t type;                       // < type of packet, ESPNOW_TYPE_MQTT_PUBBLISH 
  char node_id[8];  
  char topic[40];
  char payload[180];
} __attribute__((packed)) espnow_mqtt_data_t;

typedef struct{
    uint8_t type;                       // < type of packet, ESPNOW_TYPE_OTA_INFO
    char node_id[8]; 
    uint8_t status;
    uint16_t ota_chunk;
} __attribute__((packed)) espnow_info_t;

typedef struct{
    uint8_t type;                       // < type of packet, ESPNOW_TYPE_OTA_FIRMWARE_REQUEST
    char node_id[8]; 
    char firmware_url[100];
} __attribute__((packed)) espnow_firmware_t;

/* Parameters of sending ESPNOW data. */
typedef struct {
    uint16_t len;                              //Length of ESPNOW data to be sent, unit: byte.
    uint8_t *buffer;                      //Buffer pointing to ESPNOW data.
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];   //MAC address of destination device.
}espnow_send_param_t;

/* peers table */
typedef struct 
{
    char node_id [16];
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
}espnow_peers_table_t;
#endif
