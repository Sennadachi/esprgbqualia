// Example: initialize an RGB666 (18-bit) parallel panel using the esp_lcd rgb-panel driver.
//
// Notes / assumptions:
// - This example targets ESP-IDF's `esp_lcd` rgb panel driver (API names like
//   `esp_lcd_rgb_panel_config_t` / `esp_lcd_new_rgb_panel`). Adjust include paths
//   or symbols if your SDK version differs.
// - Pin numbers, resolution and timing values below are placeholders. Replace
//   them with the actual values for your board / panel (Adafruit Qualia S3 pins
//   if you know them).
// - This example turns on a simple GPIO backlight pin after initialization.

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

// esp_lcd headers (may vary by ESP-IDF version)
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"

static const char *TAG = "rgb666_init";

// --- User-configurable pins (change to match your board) ---
#define PIN_PCLK    1
#define PIN_HSYNC   41
#define PIN_VSYNC   42
#define PIN_DE      2
// If your board has DISP / BL pins, set them here. Leave -1 to disable.
#define PIN_DISP    45
#define PIN_BL      44

// Data pins: We'll configure 16 data lines (RGB565) and map them from your
// available pins. Original hardware had R1..R5, G0..G5, B1..B5 — for RGB565
// we map R0..R4 <- R1..R5 and B0..B4 <- B1..B5 so your numeric pins remain
// unchanged.
static const int rgb_pins[16] = {
	/* R0..R4 */ 11, 10, 9, 46, 3,
	/* G0..G5 */ 48, 47, 21, 14, 13, 12,
	/* B0..B4 */ 40, 39, 38, 0, 45
};

// --- Display timing / resolution (placeholder values) ---
#define DISP_H_RES  720
#define DISP_V_RES  720
#define PCLK_HZ     (385 * 100 * 1000) // 9 MHz pixel clock (example)

void app_main(void)
{
	ESP_LOGI(TAG, "Starting RGB666 panel init example");

	// Configure backlight pin if defined
	if (PIN_BL >= 0) {
		gpio_reset_pin(PIN_BL);
		gpio_set_direction(PIN_BL, GPIO_MODE_OUTPUT);
		gpio_set_level(PIN_BL, 0);
	}

	// Fill the rgb panel configuration structure.
	// The exact fields below follow the esp_lcd RGB panel API. If your SDK
	// exposes slightly different names, adapt accordingly.
	esp_lcd_rgb_panel_config_t rgb_config = {
		.data_width = 16,
		.num_fbs = 2,
		.clk_src = 0,
		.disp_gpio_num = PIN_DISP,
		.pclk_gpio_num = PIN_PCLK,
		.vsync_gpio_num = PIN_VSYNC,
		.hsync_gpio_num = PIN_HSYNC,
		.de_gpio_num = PIN_DE,
		.data_gpio_nums = {0},
		.timings = {
			.pclk_hz = PCLK_HZ,
			.h_res = DISP_H_RES,
			.v_res = DISP_V_RES,
			/*
			 * The following porch/pulse widths are examples. Replace with the
			 * panel-specific values from the datasheet.
			 */
			.hsync_pulse_width = 2,
			.hsync_back_porch = 44,
			.hsync_front_porch = 46,
			.vsync_pulse_width = 5,
			.vsync_back_porch = 16,
			.vsync_front_porch = 50,
		},
		.flags = {
			.fb_in_psram = true,
		},
	};

	// Copy data pin numbers into the config (API expects an array).
	for (int i = 0; i < rgb_config.data_width; ++i) {
		rgb_config.data_gpio_nums[i] = rgb_pins[i];
	}

	// Create the panel handle
	esp_lcd_panel_handle_t panel_handle = NULL;
	esp_err_t ret = esp_lcd_new_rgb_panel(&rgb_config, &panel_handle);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "esp_lcd_new_rgb_panel failed: %s", esp_err_to_name(ret));
		return;
	}

	ESP_LOGI(TAG, "RGB panel driver created");

	// Some drivers require an explicit reset + init sequence. If your panel
	// driver provides `esp_lcd_panel_reset()` / `esp_lcd_panel_init()` call
	// them here. (APIs vary between driver versions.) Example:
	// esp_lcd_panel_reset(panel_handle);
	// esp_lcd_panel_init(panel_handle);

	// Turn on backlight if defined
	if (PIN_BL >= 0) {
		gpio_set_level(PIN_BL, 1);
	}

	ESP_LOGI(TAG, "RGB666 panel initialization complete (pins/timings need verification)");

	// Keep the task alive so logs remain visible; in a real app you would
	// continue with frame rendering or other logic.
	while (1) {
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}