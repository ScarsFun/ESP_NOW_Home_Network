#ifndef ESPNOW_OTA_H
#define ESPNOW_OTA_H

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


#define OTA_GET_FW_BIT BIT0
#define OTA_END_FW_BIT BIT1
#define OTA_ABORT_FW_BIT BIT2



void espnow_send_ota_info(espnow_ota_type_t , espnow_status_id_t , uint16_t);
void ota_firmware_store_task(void *);
#endif