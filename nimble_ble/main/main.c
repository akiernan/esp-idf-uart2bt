/*	ESP-NimBLE SPP Server Example

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
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_event.h"
#include "driver/uart.h"

#include "argtable3/argtable3.h"

#include "mesh/state_binding.h"

#include "console_simple_init.h"
#include "cmd_nvs.h"
#include "cmd_system.h"

#include "spp.h"

static const char *TAG = "MAIN";

QueueHandle_t xQueueSpp;
QueueHandle_t xQueueUart;

static struct {
	struct arg_int *actual;
	struct arg_end *end;
} bt_mesh_convert_lightness_actual_to_linear_args;

static struct {
	struct arg_int *linear;
	struct arg_end *end;
} bt_mesh_convert_lightness_linear_to_actual_args;

static int do_bt_mesh_convert_lightness_actual_to_linear(int argc, char **argv)
{
	int nerrors = arg_parse(argc, argv, (void **)&bt_mesh_convert_lightness_actual_to_linear_args);

	if (nerrors != 0) {
		arg_print_errors(stderr, bt_mesh_convert_lightness_actual_to_linear_args.end, argv[0]);
		return 0;
	}

	uint16_t lightness_actual = bt_mesh_convert_lightness_actual_to_linear_args.actual->ival[0];
	printf("%d\n", bt_mesh_convert_lightness_actual_to_linear(lightness_actual));
	return 0;
}

static int do_bt_mesh_convert_lightness_linear_to_actual(int argc, char **argv)
{
	int nerrors = arg_parse(argc, argv, (void **)&bt_mesh_convert_lightness_linear_to_actual_args);

	if (nerrors != 0) {
		arg_print_errors(stderr, bt_mesh_convert_lightness_linear_to_actual_args.end, argv[0]);
		return 0;
	}

	uint16_t lightness_linear = bt_mesh_convert_lightness_linear_to_actual_args.linear->ival[0];
	printf("%d\n", bt_mesh_convert_lightness_linear_to_actual(lightness_linear));
	return 0;
}

void register_commands(void)
{
	bt_mesh_convert_lightness_actual_to_linear_args.actual = arg_int1(NULL, NULL, "<actual>", "Light lightness actual");
	bt_mesh_convert_lightness_actual_to_linear_args.end = arg_end(1);
	const esp_console_cmd_t bt_mesh_convert_lightness_actual_to_linear_args_cmd = {
		.command = "bt_mesh_convert_lightness_actual_to_linear",
		.help = "Convert lightness actual to linear",
		.hint = NULL,
		.func = do_bt_mesh_convert_lightness_actual_to_linear,
		.argtable = &bt_mesh_convert_lightness_actual_to_linear_args
	};
	ESP_ERROR_CHECK(esp_console_cmd_register(&bt_mesh_convert_lightness_actual_to_linear_args_cmd));

	bt_mesh_convert_lightness_linear_to_actual_args.linear = arg_int1(NULL, NULL, "<linear>", "Light lightness linear");
	bt_mesh_convert_lightness_linear_to_actual_args.end = arg_end(1);
	const esp_console_cmd_t bt_mesh_convert_lightness_linear_to_actual_args_cmd = {
		.command = "bt_mesh_convert_lightness_linear_to_actual",
		.help = "Convert lightness linear to actual",
		.hint = NULL,
		.func = do_bt_mesh_convert_lightness_linear_to_actual,
		.argtable = &bt_mesh_convert_lightness_linear_to_actual_args
	};
	ESP_ERROR_CHECK(esp_console_cmd_register(&bt_mesh_convert_lightness_linear_to_actual_args_cmd));
}

static void uart_init(void)
{
	const uart_config_t uart_config = {
		//.baud_rate = 115200,
		.baud_rate = CONFIG_UART_BAUD_RATE,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
		.source_clk = UART_SCLK_DEFAULT,
#else
		.source_clk = UART_SCLK_APB,
#endif
	};
	// We won't use a buffer for sending data.
	uart_driver_install(CONFIG_UART_NUM, UART_HW_FIFO_LEN(CONFIG_UART_NUM) * 2, 0, 0, NULL, 0);
	uart_param_config(CONFIG_UART_NUM, &uart_config);
	uart_set_pin(CONFIG_UART_NUM, CONFIG_UART_TX_GPIO, CONFIG_UART_RX_GPIO, UART_PIN_NO_CHANGE,
		     UART_PIN_NO_CHANGE);
}

static void uart_tx_task(void *pvParameters)
{
	ESP_LOGI(pcTaskGetName(NULL), "Start using UART%d(GPIO%d)", CONFIG_UART_NUM,
		 CONFIG_UART_TX_GPIO);
	CMD_t cmdBuf;
	while (1) {
		xQueueReceive(xQueueUart, &cmdBuf, portMAX_DELAY);
		ESP_LOGD(pcTaskGetName(NULL), "cmdBuf.length=%d", cmdBuf.length);
		ESP_LOG_BUFFER_HEXDUMP(pcTaskGetName(NULL), cmdBuf.payload, cmdBuf.length,
				       ESP_LOG_DEBUG);
		int txBytes = uart_write_bytes(CONFIG_UART_NUM, cmdBuf.payload, cmdBuf.length);
		if (txBytes != cmdBuf.length) {
			ESP_LOGE(pcTaskGetName(NULL),
				 "uart_write_bytes Fail. txBytes=%d cmdBuf.length=%d", txBytes,
				 cmdBuf.length);
		}
	} // end while

	// Never reach here
	vTaskDelete(NULL);
}

static void uart_rx_task(void *pvParameters)
{
	ESP_LOGI(pcTaskGetName(NULL), "Start using UART%d(GPIO%d)", CONFIG_UART_NUM,
		 CONFIG_UART_RX_GPIO);
	CMD_t cmdBuf;
	cmdBuf.spp_event_id = BLE_UART_EVT;
	while (1) {
		cmdBuf.length = uart_read_bytes(CONFIG_UART_NUM, cmdBuf.payload, PAYLOAD_SIZE,
						10 / portTICK_PERIOD_MS);
		// There is some rxBuf in rx buffer
		if (cmdBuf.length > 0) {
			ESP_LOGD(pcTaskGetName(NULL), "cmdBuf.length=%d", cmdBuf.length);
			ESP_LOG_BUFFER_HEXDUMP(pcTaskGetName(NULL), cmdBuf.payload, cmdBuf.length,
					       ESP_LOG_DEBUG);
			BaseType_t err = xQueueSend(xQueueSpp, &cmdBuf, portMAX_DELAY);
			if (err != pdTRUE) {
				ESP_LOGE(pcTaskGetName(NULL), "xQueueSend Fail");
			}
		} else {
			// There is no data in rx buufer
			//ESP_LOGI(pcTaskGetName(NULL), "Read %d", rxBytes);
		}
	} // end while

	// Never reach here
	vTaskDelete(NULL);
}

void nimble_spp_task(void *pvParameters);

void initialize_nvs(void)
{
	esp_err_t ret = nvs_flash_init();

	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);
}

void app_main(void)
{
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	initialize_nvs();

	// Create Queue
	xQueueSpp = xQueueCreate(10, sizeof(CMD_t));
	configASSERT(xQueueSpp);
	xQueueUart = xQueueCreate(10, sizeof(CMD_t));
	configASSERT(xQueueUart);

	// Initialize UART
	uart_init();

	// Start tasks
	xTaskCreate(uart_tx_task, "UART-TX", 1024 * 4, NULL, 2, NULL);
	xTaskCreate(uart_rx_task, "UART-RX", 1024 * 4, NULL, 2, NULL);
	xTaskCreate(nimble_spp_task, "NIMBLE_SPP", 1024 * 4, NULL, 2, NULL);

	ESP_ERROR_CHECK(console_cmd_init()); // Initialize console
	register_system();
	register_nvs();
	register_commands();

	ESP_ERROR_CHECK(console_cmd_start()); // Start console
}
