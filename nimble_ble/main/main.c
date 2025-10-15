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
#include "zcp.h"

static const char *TAG = "MAIN";

QueueHandle_t xQueueSpp;
QueueHandle_t uart_tx_queue;

/*
    get-power              Get power - true / false
    get-power-on-mode      Read LED power-on behaviour mode
    get-power-on-val       Read LED power-on value
    get-state              Read current LED state
    get-user-conf-state    Read current LED state from User Config
    help                   Print this message or the help of the given subcommand(s)
    set-power              Set power - true / false
    set-power-on-mode      Write LED power-on behaviour mode
    set-power-on-preset    Write LED power-on preset
    set-raw                Set raw LED channels
    set-state              Set LED state
*/

static struct {
	struct arg_int *mode;
	struct arg_int *warm;
	struct arg_int *cool;
	struct arg_end *end;
} led_set_raw_args;

void appmcu_led_write_ring_state(uint8_t mode, uint16_t warm, uint16_t cool)
{
	CMD_t cmd_buf;

	cmd_buf.length = snprintf((char *)cmd_buf.payload, sizeof cmd_buf.payload,
				  ZCP(ZCP_COMMAND_LED, ZCP_OPERATION_LED_RING_TRANSITION,
				      "%02" PRIX8 "%04" PRIX16 "%04" PRIX16),
				  mode, warm, cool);
	printf("%s\n", cmd_buf.payload);
	xQueueSend(uart_tx_queue, &cmd_buf, portMAX_DELAY);
}

void zcp_sys_reset(void)
{
	static const char zcp_sys_reset[] = ZCP(ZCP_COMMAND_SYS, ZCP_OPERATION_SYS_RESET);

	uart_write_bytes(CONFIG_UART_NUM, zcp_sys_reset, sizeof zcp_sys_reset);
	// read back line
}

void zcp_sys_factory_reset(void)
{
	static const char zcp_sys_factory_reset[] = ZCP(ZCP_COMMAND_SYS, ZCP_OPERATION_SYS_FACTORY_RESET);

	uart_write_bytes(CONFIG_UART_NUM, zcp_sys_factory_reset, sizeof zcp_sys_factory_reset - 1);
	// read back line
}

static int do_led_set_raw(int argc, char **argv)
{
	int nerrors = arg_parse(argc, argv, (void **)&led_set_raw_args);

	if (nerrors != 0) {
		arg_print_errors(stderr, led_set_raw_args.end, argv[0]);
		return 0;
	}

	uint8_t mode = led_set_raw_args.mode->ival[0];
	uint16_t warm = led_set_raw_args.warm->ival[0];
	uint16_t cool = led_set_raw_args.cool->ival[0];
	appmcu_led_write_ring_state(mode, warm, cool);
	return 0;
}

static int do_zcp_sys_reset(int argc, char **argv)
{
	zcp_sys_reset();
	return 0;
}

static int do_zcp_sys_factory_reset(int argc, char **argv)
{
	zcp_sys_factory_reset();
	return 0;
}

// Calculate mixing ratio for dual LED controller
// Returns ratio of warm (3000K) LED intensity (0.0 to 1.0)
float calculate_led_ratio(float target_temp)
{
	const float warm_temp = 3000.0f;
	const float cool_temp = 5000.0f;

	if (target_temp < warm_temp) {
		return 1.0f;
	}
	if (target_temp > cool_temp) {
		return 0.0f;
	}

	float warm_ratio = (cool_temp - target_temp) / (cool_temp - warm_temp);
	return warm_ratio;
}

// Convert lightness (0-100) to luminance using Bluetooth Mesh square root method
float lightness_to_luminance(float lightness)
{
	// BLE Mesh uses: luminance = (lightness / 100)^2
	float normalized = lightness / 100.0f;
	return normalized * normalized;
}

// Calculate LED drive values (0-65535) accounting for efficacy and avoiding deadzone
// warm_efficacy and cool_efficacy are relative values (e.g., 1.0 for reference)
// Returns 0 on success, -1 if output cannot be achieved
int calculate_led_drive(float target_temp, float lightness, float warm_efficacy,
			float cool_efficacy, uint16_t *warm_drive, uint16_t *cool_drive)
{
	const float max_drive = 65535.0f;
	const float deadzone_max = 256.0f; // Deadzone: drive values [0-256]

	// Get color mixing ratio
	float warm_ratio = calculate_led_ratio(target_temp);
	float cool_ratio = 1.0f - warm_ratio;

	// Handle off state
	if (lightness == 0.0f) {
		*warm_drive = 0;
		*cool_drive = 0;
		return 0;
	}

	// Map lightness to luminance, skipping the deadzone
	// Deadzone is 0-256 out of 65535, so 0.39% of range
	// Effective usable range: 256-65535 (65279 steps)
	float deadzone_fraction = deadzone_max / max_drive; // ~0.0039

	// Remap lightness (0-100) to skip deadzone:
	// 0% → drive 0 (off)
	// Small % → drive 257+ (above deadzone)
	// 100% → drive 65535
	float luminance;
	if (lightness <= deadzone_fraction * 100.0f) {
		// Map lightness linearly into the usable range starting at deadzone_max
		luminance = (lightness / (deadzone_fraction * 100.0f)) * (deadzone_max / max_drive);
	} else {
		// Map the remaining lightness range (deadzone_fraction% to 100%) to (deadzone_max to max_drive)
		float adjusted_lightness = (lightness - deadzone_fraction * 100.0f) /
					   (100.0f - deadzone_fraction * 100.0f);
		luminance = (deadzone_max + adjusted_lightness * (max_drive - deadzone_max)) /
			    max_drive;
	}

	// Calculate uncorrected drive values (accounting for efficacy differences)
	float combined_efficacy = warm_ratio * warm_efficacy + cool_ratio * cool_efficacy;

	if (combined_efficacy == 0.0f) {
		return -1;
	}

	float warm_uncorrected =
		(warm_ratio * warm_efficacy / combined_efficacy) * luminance * max_drive;
	float cool_uncorrected =
		(cool_ratio * cool_efficacy / combined_efficacy) * luminance * max_drive;

	// Ensure both values are at least deadzone_max if they're non-zero
	if (warm_uncorrected > 0.0f && warm_uncorrected < deadzone_max) {
		warm_uncorrected = deadzone_max;
	}
	if (cool_uncorrected > 0.0f && cool_uncorrected < deadzone_max) {
		cool_uncorrected = deadzone_max;
	}

	// Convert to uint16_t, rounding
	*warm_drive = (uint16_t)(warm_uncorrected + 0.5f);
	*cool_drive = (uint16_t)(cool_uncorrected + 0.5f);

	return 0;
}

static struct {
	struct arg_int *mode;
	struct arg_dbl *lightness;
	struct arg_dbl *temperature;
	struct arg_end *end;
} led_set_state_args;

static int do_led_set_state(int argc, char **argv)
{
	int nerrors = arg_parse(argc, argv, (void **)&led_set_state_args);

	if (nerrors != 0) {
		arg_print_errors(stderr, led_set_state_args.end, argv[0]);
		return 0;
	}

	uint8_t mode = led_set_state_args.mode->ival[0];
	float lightness = led_set_state_args.lightness->dval[0];
	float temperature = led_set_state_args.temperature->dval[0];

	// Assume cool LED (5000K) is reference with efficacy 1.0
	// Warm LED (3000K) typically ~5-10% more efficient
	const float warm_efficacy = 1.08f;
	const float cool_efficacy = 1.0f;

	uint16_t warm_drive, cool_drive;

	calculate_led_drive(temperature, lightness, warm_efficacy, cool_efficacy, &warm_drive,
			    &cool_drive);
	printf("warm_drive = %" PRIX16 ", cool_drive = %" PRIX16 "\n", warm_drive, cool_drive);
	appmcu_led_write_ring_state(mode, warm_drive, cool_drive);
	return 0;
}

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
	int nerrors =
		arg_parse(argc, argv, (void **)&bt_mesh_convert_lightness_actual_to_linear_args);

	if (nerrors != 0) {
		arg_print_errors(stderr, bt_mesh_convert_lightness_actual_to_linear_args.end,
				 argv[0]);
		return 0;
	}

	uint16_t lightness_actual = bt_mesh_convert_lightness_actual_to_linear_args.actual->ival[0];
	printf("%d\n", bt_mesh_convert_lightness_actual_to_linear(lightness_actual));
	return 0;
}

static int do_bt_mesh_convert_lightness_linear_to_actual(int argc, char **argv)
{
	int nerrors =
		arg_parse(argc, argv, (void **)&bt_mesh_convert_lightness_linear_to_actual_args);

	if (nerrors != 0) {
		arg_print_errors(stderr, bt_mesh_convert_lightness_linear_to_actual_args.end,
				 argv[0]);
		return 0;
	}

	uint16_t lightness_linear = bt_mesh_convert_lightness_linear_to_actual_args.linear->ival[0];
	printf("%d\n", bt_mesh_convert_lightness_linear_to_actual(lightness_linear));
	return 0;
}

void register_commands_debug(void)
{
	bt_mesh_convert_lightness_actual_to_linear_args.actual =
		arg_int1(NULL, NULL, "<actual>", "Light lightness actual");
	bt_mesh_convert_lightness_actual_to_linear_args.end = arg_end(1);
	const esp_console_cmd_t bt_mesh_convert_lightness_actual_to_linear_args_cmd = {
		.command = "bt_mesh_convert_lightness_actual_to_linear",
		.help = "Convert lightness actual to linear",
		.hint = NULL,
		.func = do_bt_mesh_convert_lightness_actual_to_linear,
		.argtable = &bt_mesh_convert_lightness_actual_to_linear_args
	};
	ESP_ERROR_CHECK(
		esp_console_cmd_register(&bt_mesh_convert_lightness_actual_to_linear_args_cmd));

	bt_mesh_convert_lightness_linear_to_actual_args.linear =
		arg_int1(NULL, NULL, "<linear>", "Light lightness linear");
	bt_mesh_convert_lightness_linear_to_actual_args.end = arg_end(1);
	const esp_console_cmd_t bt_mesh_convert_lightness_linear_to_actual_args_cmd = {
		.command = "bt_mesh_convert_lightness_linear_to_actual",
		.help = "Convert lightness linear to actual",
		.hint = NULL,
		.func = do_bt_mesh_convert_lightness_linear_to_actual,
		.argtable = &bt_mesh_convert_lightness_linear_to_actual_args
	};
	ESP_ERROR_CHECK(
		esp_console_cmd_register(&bt_mesh_convert_lightness_linear_to_actual_args_cmd));

	led_set_raw_args.mode = arg_int1(NULL, NULL, "<mode>", "Transition mode");
	led_set_raw_args.warm = arg_int1(NULL, NULL, "<warm>", "Warm linear");
	led_set_raw_args.cool = arg_int1(NULL, NULL, "<cool>", "Cool linear");
	led_set_raw_args.end = arg_end(1);
	const esp_console_cmd_t led_set_raw_cmd = { .command = "led_set_raw",
						    .help = "Set raw LED ring state",
						    .hint = NULL,
						    .func = do_led_set_raw,
						    .argtable = &led_set_raw_args };
	ESP_ERROR_CHECK(esp_console_cmd_register(&led_set_raw_cmd));

	led_set_state_args.mode = arg_int1(NULL, NULL, "<mode>", "Transition mode");
	led_set_state_args.lightness = arg_dbl1(NULL, NULL, "<lightness>", "Light lightness");
	led_set_state_args.temperature =
		arg_dbl1(NULL, NULL, "<temperature>", "Target temperature");
	led_set_state_args.end = arg_end(1);
	const esp_console_cmd_t led_set_state_cmd = { .command = "led_set_state",
						      .help = "Set LED ring state",
						      .hint = NULL,
						      .func = do_led_set_state,
						      .argtable = &led_set_state_args };
	ESP_ERROR_CHECK(esp_console_cmd_register(&led_set_state_cmd));

	ESP_ERROR_CHECK(console_cmd_user_register("zcp_sys_reset", do_zcp_sys_reset));
	ESP_ERROR_CHECK(console_cmd_user_register("zcp_sys_factory_reset", do_zcp_sys_factory_reset));
}

static void uart_tx_task(void *pvParameters)
{
	ESP_LOGI(pcTaskGetName(NULL), "Start using UART%d(GPIO%d)", CONFIG_UART_NUM,
		 CONFIG_UART_TX_GPIO);
	CMD_t cmdBuf;
	while (1) {
		xQueueReceive(uart_tx_queue, &cmdBuf, portMAX_DELAY);
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

static void appmcu_rx(const uart_event_t *event)
{
	CMD_t cmdBuf;
	cmdBuf.spp_event_id = BLE_UART_EVT;

	switch (event->type) {
	case UART_DATA:
		cmdBuf.length = uart_read_bytes(CONFIG_UART_NUM, cmdBuf.payload, event->size,
						portMAX_DELAY);
		if (cmdBuf.length > 0) {
			ESP_LOGD(pcTaskGetName(NULL), "cmdBuf.length=%d", cmdBuf.length);
			ESP_LOG_BUFFER_HEXDUMP(pcTaskGetName(NULL), cmdBuf.payload, cmdBuf.length,
					       ESP_LOG_DEBUG);
			BaseType_t err = xQueueSend(xQueueSpp, &cmdBuf, portMAX_DELAY);
			if (err != pdTRUE) {
				ESP_LOGE(pcTaskGetName(NULL), "xQueueSend Fail");
			}
		}
		break;
	default:
		ESP_LOGI(pcTaskGetName(NULL), "UART event type: %d", event->type);
		break;
	}
}

static void uart_rx_task(void *pvParameters)
{
	uart_event_t event;
	QueueHandle_t xQueueUartEvent = pvParameters;

	ESP_LOGI(pcTaskGetName(NULL), "Start using UART%d(GPIO%d)", CONFIG_UART_NUM,
		 CONFIG_UART_RX_GPIO);
	for (;;) {
		if (xQueueReceive(xQueueUartEvent, (void *)&event, (TickType_t)portMAX_DELAY))
			appmcu_rx(&event);
	}

	__builtin_unreachable();
	vTaskDelete(NULL);
}

static void uart_init(void)
{
	const uart_config_t uart_config = {
		//.baud_rate = 115200,
		.baud_rate = CONFIG_UART_BAUD_RATE,    .data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,	       .stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
		.source_clk = UART_SCLK_DEFAULT,
#else
		.source_clk = UART_SCLK_APB,
#endif
	};
	QueueHandle_t xQueueUartEvent;

	// We won't use a buffer for sending data.
	uart_driver_install(CONFIG_UART_NUM, UART_HW_FIFO_LEN(CONFIG_UART_NUM) * 2, 0, 20,
			    &xQueueUartEvent, 0);
	uart_param_config(CONFIG_UART_NUM, &uart_config);
	uart_set_pin(CONFIG_UART_NUM, CONFIG_UART_TX_GPIO, CONFIG_UART_RX_GPIO, UART_PIN_NO_CHANGE,
		     UART_PIN_NO_CHANGE);

	uart_tx_queue = xQueueCreate(10, sizeof(CMD_t));
	configASSERT(uart_tx_queue);

	// Force the AppMCU to synchronise
	uart_write_bytes(CONFIG_UART_NUM, ZCP_FRAMING_END, sizeof(ZCP_FRAMING_END) - 1);
	uart_write_bytes(CONFIG_UART_NUM, ZCP_SYS_SET_ACK_ON, sizeof(ZCP_SYS_SET_ACK_ON) - 1);

	// Start tasks
	xTaskCreate(uart_tx_task, "UART-TX", 1024 * 4, NULL, 2, NULL);
	xTaskCreate(uart_rx_task, "UART-RX", 1024 * 4, xQueueUartEvent, 2, NULL);
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

	// Initialize UART
	uart_init();

	xTaskCreate(nimble_spp_task, "NIMBLE_SPP", 1024 * 4, NULL, 2, NULL);

	ESP_ERROR_CHECK(console_cmd_init()); // Initialize console
	register_system();
	register_nvs();
	register_commands_debug();

	// Register any other plugin command added to your project
	ESP_ERROR_CHECK(console_cmd_all_register());

	ESP_ERROR_CHECK(console_cmd_start()); // Start console
}
