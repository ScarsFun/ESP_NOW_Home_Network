#include "ESPNOW_ota.h"

extern espnow_send_param_t *send_param;
extern QueueHandle_t xQueueOtaData;
extern EventGroupHandle_t ota_status_group;


/**
 * @brief      send OTA infos to gateway
 *
 * @param[in]  message_type  The message type
 * @param[in]  status        The status
 * @param[in]  chunk         The chunk
 */
void espnow_send_ota_info(espnow_ota_type_t message_type, espnow_status_id_t status, uint16_t chunk)
{
    espnow_info_t *buf = (espnow_info_t *)send_param->buffer;
    assert(send_param->len = sizeof(espnow_info_t));
    buf->type = message_type;
    buf->status = status;
    buf->ota_chunk = chunk;
    if (esp_now_send(send_param->dest_mac, send_param->buffer,
                     send_param->len) != ESP_OK)
    {
        ESP_LOGE(pcTaskGetName(0), "espnow_send_ota_info error");
    }
    else  ESP_LOGI(pcTaskGetName(0), "sent info ack");
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
    EventBits_t uxBits;
    esp_err_t err;
    espnow_ota_data_t recv_ota_data;
    // update handle : set by esp_ota_begin(), must be freed via esp_ota_end()
    esp_ota_handle_t update_handle = 0 ;
    const esp_partition_t *update_partition = NULL;
    uint8_t *ota_write_data = malloc(sizeof(uint8_t) * BUFFSIZE + 1);
    if (ota_write_data == NULL)
        ESP_LOGE(pcTaskGetName(0), "malloc error in OTA task");

    ESP_LOGI(pcTaskGetName(0), "Starting OTA task");
    //size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    //ESP_LOGI(pcTaskGetName(0), " free heap: %zu", free_heap);
    const esp_partition_t *configured = esp_ota_get_boot_partition();
    const esp_partition_t *running = esp_ota_get_running_partition();
    // check ota partition
    if (configured != running)
    {
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
    if (xQueueOtaData == NULL)
    {
        ESP_LOGE(pcTaskGetName(0), "Create xQueueOtaData fail");
    }
    
    while (1)
    {
        uxBits = xEventGroupWaitBits(ota_status_group,
                             OTA_GET_FW_BIT | OTA_ABORT_FW_BIT | OTA_END_FW_BIT,
                             pdTRUE, pdFALSE, 2500 / portTICK_PERIOD_MS);
        //ESP_LOGI(pcTaskGetName(0), "uxBits %lu", uxBits);

        if ( ( uxBits & OTA_ABORT_FW_BIT ) != 0 )
        {
            ESP_LOGE(pcTaskGetName(0), "OTA aborted by server");
            goto EndTask;
        }
        // get ota chunks while receiving OTA_END_FW_BIT (last chunks)
        else if ( (( uxBits & OTA_GET_FW_BIT ) != 0) || (( uxBits & OTA_END_FW_BIT ) != 0))
        {
            ret_Queue = xQueueReceive(xQueueOtaData, &recv_ota_data, 400 / portTICK_PERIOD_MS);
            uint8_t calc_crc = esp_crc8_le(0, (uint8_t const *)&recv_ota_data.ota_chunk, recv_ota_data.ota_chunk_size);

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
                //printf("ota store stack free %d\n", uxTaskGetStackHighWaterMark(NULL));
                //ESP_LOGI(pcTaskGetName(0), " OTA free heap: %zu", free_heap);
                ota_write_data_size += recv_ota_data.ota_chunk_size;
                ota_write_data_sequence ++;
                //ESP_LOGI(pcTaskGetName(0), "ota data partial size = %d", ota_write_data_size);

                espnow_send_ota_info(ESPNOW_TYPE_OTA_INFO, ESPNOW_STATUS_OTA_ACK_OK, recv_ota_data.sequence);
            }
            // write buffered data in OTA partition if buffer full or timeout receiving chunk or flush last chunks (OTA_END_FW_BIT)
            if ((ota_write_data_sequence == 4) || (ret_Queue == pdFALSE) || (( uxBits & OTA_END_FW_BIT ) != 0))
            {
                //ESP_LOGI(pcTaskGetName(0), "ota data size = %d", ota_write_data_size);
                if (image_header_was_checked == false)
                {
                    esp_app_desc_t new_app_info;
                    if (ota_write_data_size > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t))
                    {
                        // check current version with downloading
                        memcpy(&new_app_info, &ota_write_data[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t));
                        ESP_LOGI(pcTaskGetName(0), "New firmware version: %s", new_app_info.version);

                        esp_app_desc_t running_app_info;
                        if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK)
                        {
                            ESP_LOGI(pcTaskGetName(0), "Running firmware version: %s", running_app_info.version);
                        }

                        const esp_partition_t *last_invalid_app = esp_ota_get_last_invalid_partition();
                        esp_app_desc_t invalid_app_info;
                        if (esp_ota_get_partition_description(last_invalid_app, &invalid_app_info) == ESP_OK)
                        {
                            ESP_LOGI(pcTaskGetName(0), "Last invalid firmware version: %s", invalid_app_info.version);
                        }

                        // check current version with last invalid partition
                        if (last_invalid_app != NULL)
                        {
                            if (memcmp(invalid_app_info.version, new_app_info.version, sizeof(new_app_info.version)) == 0)
                            {
                                ESP_LOGW(pcTaskGetName(0), "New version is the same as invalid version.");
                                ESP_LOGW(pcTaskGetName(0), "Previously, there was an attempt to launch the firmware with %s version, but it failed.", invalid_app_info.version);
                                ESP_LOGW(pcTaskGetName(0), "The firmware has been rolled back to the previous version.");
                                goto EndTask;
                            }
                        }
                        image_header_was_checked = true;

                        err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
                        err = ESP_OK;
                        if (err != ESP_OK)
                        {
                            ESP_LOGE(pcTaskGetName(0), "esp_ota_begin failed (%s)", esp_err_to_name(err));
                            esp_ota_abort(update_handle);
                            goto EndTask;
                        }
                        ESP_LOGI(pcTaskGetName(0), "esp_ota_begin succeeded");
                    }
                    else
                    {
                        ESP_LOGE(pcTaskGetName(0), "received package not fit len");
                        esp_ota_abort(update_handle);
                        goto EndTask;
                    }
                }
                err = esp_ota_write( update_handle, (const void *)ota_write_data, ota_write_data_size);
                //printf("ota stack free %d", uxTaskGetStackHighWaterMark(NULL));
                if (err != ESP_OK)
                {
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
                err = esp_ota_end(update_handle);
                if (err != ESP_OK)
                {
                    if (err == ESP_ERR_OTA_VALIDATE_FAILED)
                    {
                        ESP_LOGE(pcTaskGetName(0), "Image validation failed, image is corrupted");
                    }
                    else
                    {
                        ESP_LOGE(pcTaskGetName(0), "esp_ota_end failed (%s)!", esp_err_to_name(err));
                    }
                    goto EndTask;
                }
                err = esp_ota_set_boot_partition(update_partition);
                if (err != ESP_OK)
                {
                    ESP_LOGE(pcTaskGetName(0), "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
                    goto EndTask;
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
            goto EndTask;
        }
    }
EndTask:
    vQueueDelete( xQueueOtaData );
    free (ota_write_data);
    xEventGroupClearBits(ota_status_group, OTA_GET_FW_BIT);
    ESP_LOGI(pcTaskGetName(0), "OTA End");
    vTaskDelete(NULL);
}
