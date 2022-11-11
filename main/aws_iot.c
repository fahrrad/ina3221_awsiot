/* Read INA3221 and publish to  AWS IoT example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "mqtt_client.h"

#include <ina3221.h>

#define I2C_PORT 0


static const char *TAG = "MQTTS_EXAMPLE";

extern const uint8_t client_cert_pem_start[] asm("_binary_client_crt_start");
extern const uint8_t client_cert_pem_end[] asm("_binary_client_crt_end");
extern const uint8_t client_key_pem_start[] asm("_binary_client_key_start");
extern const uint8_t client_key_pem_end[] asm("_binary_client_key_end");
extern const uint8_t server_cert_pem_start[] asm("_binary_AmazonRootCA1_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_AmazonRootCA1_pem_end");

// Client object for MQTT connection
esp_mqtt_client_handle_t client;
TaskHandle_t xHandle = NULL;

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

void task(void *pvParameters)
{
    ina3221_t dev = {
            /* shunt values are 100 mOhm for each channel */
            .shunt = {
                CONFIG_EXAMPLE_SHUNT_RESISTOR_MILLI_OHM,
                CONFIG_EXAMPLE_SHUNT_RESISTOR_MILLI_OHM,
                CONFIG_EXAMPLE_SHUNT_RESISTOR_MILLI_OHM
            },
            .config.config_register = INA3221_DEFAULT_CONFIG,
            .mask.mask_register = INA3221_DEFAULT_MASK
    };
    memset(&dev.i2c_dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(i2cdev_init());
    ESP_ERROR_CHECK(ina3221_init_desc(&dev, CONFIG_EXAMPLE_I2C_ADDR, I2C_PORT, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

    ESP_ERROR_CHECK(ina3221_set_options(&dev, true, true, true));    // Mode selection, bus and shunt activated
    ESP_ERROR_CHECK(ina3221_enable_channel(&dev, true, true, true)); // Enable all channels
    ESP_ERROR_CHECK(ina3221_set_average(&dev, INA3221_AVG_64));      // 64 samples average
    ESP_ERROR_CHECK(ina3221_set_bus_conversion_time(&dev, INA3221_CT_2116));   // 2ms by channel
    ESP_ERROR_CHECK(ina3221_set_shunt_conversion_time(&dev, INA3221_CT_2116)); // 2ms by channel

    float bus_voltage;
    float shunt_voltage;
    float shunt_current;
    char json_msg[128];

    while (1)
    {
        int channel = 0;
        // Get voltage in volts
        ESP_ERROR_CHECK(ina3221_get_bus_voltage(&dev, channel, &bus_voltage));
        // Get voltage in millivolts and current in milliamperes
        ESP_ERROR_CHECK(ina3221_get_shunt_value(&dev, channel, &shunt_voltage, &shunt_current));

        printf("C%u:Bus voltage: %.02f V\n", channel + 1, bus_voltage);
        printf("C%u:Shunt voltage: %.02f mV\n", channel + 1, shunt_voltage);
        printf("C%u:Shunt current: %.02f mA\n\n", channel + 1, shunt_current);
  
        sprintf(json_msg, "{\"bus_voltage\": %.2f, \"shunt_voltage\": %.2f, \"shunt_current\": %.2f}", bus_voltage, shunt_voltage, shunt_current);
        esp_mqtt_client_publish(client, "sensor/ina3221", json_msg, 0, 0, 0);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%lu", base, event_id);
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        // Only start this task once (so not on reconnect)
        if (xHandle == NULL){
            xTaskCreate(task, "ina3221_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, &xHandle);
            ESP_LOGI(TAG, "Task created!");
        }else{
            ESP_LOGI(TAG, "Task was already created, probably a reconnect");
        }
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start(void)
{
  const esp_mqtt_client_config_t mqtt_cfg = {
    .broker.address.uri = CONFIG_EXAMPLE_AWS_IOT_URI,
    .broker.verification.certificate = (const char *)server_cert_pem_start,
    .credentials = {
      .authentication = {
        .certificate = (const char *)client_cert_pem_start,
        .key = (const char *)client_key_pem_start,
      },
    }
  };

    ESP_LOGI(TAG, "[APP] Free memory: %lu bytes", esp_get_free_heap_size());
    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    mqtt_app_start();
}
