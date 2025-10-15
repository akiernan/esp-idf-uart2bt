/*	ESP-BLE SPP Server using ble_conn_mgr

	This example code is in the Public Domain (or CC0 licensed, at your option.)

	Unless required by applicable law or agreed to in writing, this
	software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
	CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_event.h"

#include "esp_ble_conn_mgr.h"

#include "spp.h"

static esp_err_t esp_spp_chr_cb(const uint8_t *inbuf, uint16_t inlen, uint8_t **outbuf,
				uint16_t *outlen, void *priv_data);

#define DEVICE_NAME "ESP_BLE_SERVER" //The Device Name Characteristics in GAP

// 16 Bit SPP Service UUID
#define BLE_SVC_SPP_UUID16 0xABF0

// 16 Bit SPP Service Characteristic UUID
#define BLE_SVC_SPP_CHR_UUID16 0xABF1

static const char *TAG = "SPP";

extern QueueHandle_t xQueueSpp;
extern QueueHandle_t uart_tx_queue;

#define RX_BUF_SIZE 128

#ifdef CONFIG_BT_NIMBLE_ENABLED
#define ATTRIBUTE_MAX_CONNECTIONS CONFIG_BT_NIMBLE_MAX_CONNECTIONS
#else
#define ATTRIBUTE_MAX_CONNECTIONS CONFIG_BT_ACL_CONNECTIONS
#endif

uint16_t connection_handle[ATTRIBUTE_MAX_CONNECTIONS];

static esp_err_t esp_spp_chr_cb(const uint8_t *inbuf, uint16_t inlen, uint8_t **outbuf,
				uint16_t *outlen, void *priv_data)
{
	if (!outbuf || !outlen) {
		return ESP_ERR_INVALID_ARG;
	}

	if (!inbuf) {
		ESP_LOGI(TAG, "Callback for read");
		*outlen = strlen("SPP_CHR");
		*outbuf = (uint8_t *)strndup("SPP_CHR", *outlen);
	} else {
		ESP_LOGI(TAG, "Callback for write");
		*outbuf = calloc(1, inlen);
		memcpy(*outbuf, inbuf, inlen);
		*outlen = inlen;

		// Send data to UART TX queue
		CMD_t cmdBuf;
		cmdBuf.length = inlen;
		memcpy(cmdBuf.payload, inbuf, inlen);
		BaseType_t err = xQueueSend(uart_tx_queue, &cmdBuf, portMAX_DELAY);
		if (err != pdTRUE) {
			ESP_LOGE(TAG, "xQueueSend Fail");
		}
	}

	return ESP_OK;
}

static void app_ble_conn_event_handler(void *handler_args, esp_event_base_t base, int32_t id,
				       void *event_data)
{
	if (base != BLE_CONN_MGR_EVENTS) {
		return;
	}

	switch (id) {
	case ESP_BLE_CONN_EVENT_CONNECTED:
		ESP_LOGI(TAG, "ESP_BLE_CONN_EVENT_CONNECTED\n");
		connection_handle[0] = 1;
		break;
	case ESP_BLE_CONN_EVENT_DISCONNECTED:
		ESP_LOGI(TAG, "ESP_BLE_CONN_EVENT_DISCONNECTED\n");
		connection_handle[0] = 0;
		break;
	default:
		break;
	}
}

static const esp_ble_conn_character_t spp_nu_lookup_table[] = {
	{ "spp_chr",
	  BLE_CONN_UUID_TYPE_16,
	  BLE_CONN_GATT_CHR_READ | BLE_CONN_GATT_CHR_WRITE | BLE_CONN_GATT_CHR_NOTIFY |
		  BLE_CONN_GATT_CHR_INDICATE,
	  { BLE_SVC_SPP_CHR_UUID16 },
	  esp_spp_chr_cb },
};

static const esp_ble_conn_svc_t spp_svc = {
    .type = BLE_CONN_UUID_TYPE_16,
    .uuid = {
        .uuid16 = BLE_SVC_SPP_UUID16,
    },
    .nu_lookup_count = sizeof(spp_nu_lookup_table) / sizeof(spp_nu_lookup_table[0]),
    .nu_lookup = (esp_ble_conn_character_t *)spp_nu_lookup_table
};

int gatt_svr_register(void)
{
	return esp_ble_conn_add_svc(&spp_svc);
}

void nimble_spp_task(void *pvParameters)
{
	ESP_LOGI(TAG, "BLE SPP Task Started");

	esp_ble_conn_config_t config = {
		.device_name = DEVICE_NAME,
		.include_service_uuid = 1,
		.adv_uuid16 = BLE_SVC_SPP_UUID16,
	};

	esp_err_t ret;

	/* Initialize connection_handle array */
	for (int i = 0; i < ATTRIBUTE_MAX_CONNECTIONS; i++) {
		connection_handle[i] = 0;
	}

	esp_event_handler_register(BLE_CONN_MGR_EVENTS, ESP_EVENT_ANY_ID,
				   app_ble_conn_event_handler, NULL);

	esp_ble_conn_init(&config);

	ret = gatt_svr_register();
	assert(ret == 0);

	if (esp_ble_conn_start() != ESP_OK) {
		esp_ble_conn_stop();
		esp_ble_conn_deinit();
		esp_event_handler_unregister(BLE_CONN_MGR_EVENTS, ESP_EVENT_ANY_ID,
					     app_ble_conn_event_handler);
		return;
	}

	CMD_t cmdBuf;
	while (1) {
		xQueueReceive(xQueueSpp, &cmdBuf, portMAX_DELAY);
		ESP_LOGD(TAG, "cmdBuf.spp_event_id=%d", cmdBuf.spp_event_id);
		if (cmdBuf.spp_event_id == BLE_UART_EVT) {
			for (int i = 0; i < ATTRIBUTE_MAX_CONNECTIONS; i++) {
				if (connection_handle[i] != 0) {
					esp_ble_conn_data_t inbuff = {
						.type = BLE_CONN_UUID_TYPE_16,
						.uuid = {
							.uuid16 = BLE_SVC_SPP_CHR_UUID16,
						},
						.data = cmdBuf.payload,
						.data_len = cmdBuf.length,
					};
					int rc = esp_ble_conn_notify(&inbuff);
					if (rc == 0) {
						ESP_LOGD(TAG, "Notification sent successfully");
					} else {
						ESP_LOGI(TAG,
							 "Error in sending notification rc = %d",
							 rc);
					}
				}
			}
		}
	} // end while

	// never reach here
	vTaskDelete(NULL);
}
