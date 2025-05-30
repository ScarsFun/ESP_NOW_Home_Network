/**
 * @defgroup   LED_MON led monitor
 *
 * @brief      This file implements simple RGB addressable led monitoring.
 *
 * @author     Max
 * @date       2023
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "led_mon.h"

static led_strip_handle_t led_strip;

/**
 * @brief      configure addressable LED strip
 */
void configure_led(void)
{
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = CONFIG_BLINK_GPIO,      // The GPIO that connected to the LED strip's data line  
        .max_leds = 1,                            // at least one LED on board
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
        .led_model = LED_MODEL_WS2812,            // LED strip model
        .flags.invert_out = false,                // whether to invert the output signal
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, 
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false, 
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}

/**
 * @brief      LED monitor task 
 *
 * @param      pvParameter : wants led_param_t in led_mon.h
 */
void led_task(void *pvParameter)
{
    led_param_t *led_param = (led_param_t *)pvParameter;
    for (uint16_t idx = 0; idx < led_param->rip; idx++)
    {
        led_strip_set_pixel(led_strip, led_param->led_id, led_param->R, led_param->G, led_param->B);
        led_strip_refresh(led_strip);
        vTaskDelay( led_param->delay/ portTICK_PERIOD_MS);
        led_strip_clear(led_strip);
        vTaskDelay( led_param->delay/ portTICK_PERIOD_MS);
        // rip = 255 as infinite loop
        if(led_param->rip == 255)
            idx=4;
    }
    vTaskDelete(NULL);
}