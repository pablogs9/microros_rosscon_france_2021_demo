#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "led_strip.h"

#include <uros_network_interfaces.h>

led_strip_t *strip;
led_strip_config_t strip_config;

void micro_ros_task(void *);
void ledstrip_task(void *);

void app_main(void)
{
    // Init network
    ESP_ERROR_CHECK(uros_network_interface_initialize());

    // Init LED
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(CONFIG_EXAMPLE_RMT_TX_GPIO, RMT_CHANNEL_0);
    // set counter clock to 40MHz
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    strip_config = (led_strip_config_t) LED_STRIP_DEFAULT_CONFIG(CONFIG_EXAMPLE_STRIP_LED_NUMBER, (led_strip_dev_t)config.channel);

    // install ws2812 driver
    strip = led_strip_new_rmt_ws2812(&strip_config);

    xTaskCreate(micro_ros_task,
        "microros_task",
        16000,
        NULL,
        5,
        NULL);
}
