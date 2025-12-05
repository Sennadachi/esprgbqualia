// Example: initialise Qualia RGB666 panel (NV3052C), load register table via SPI,
// create double framebuffers in PSRAM, and show red/green/blue sequentially.
//
// Assumptions:
// - ESP-IDF (esp_lcd rgb-panel driver available).
// - PSRAM present and enabled in sdkconfig.
// - Uses your defined pin numbers (adjust if different).
// - NV3052C register sequence taken from user-provided list.
//
// Notes:
// - The NV3052C SPI protocol: CSX low enables serial interface, SDI sampled on SCL rising edge.
//   After SWRESET/SLPOUT delays are required; see datasheet. 

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"

static const char *TAG = "qualia_rgb_init";

/* --- Pins (from your example) --- */
#define PIN_PANEL_SPI_MOSI  7
#define PIN_PANEL_SPI_MISO  6
#define PIN_PANEL_SPI_SCLK  5
#define PIN_PANEL_SPI_CS    15

#define PIN_PCLK    1
#define PIN_HSYNC   41
#define PIN_VSYNC   42
#define PIN_DE      2
#define PIN_DISP    -1
#define PIN_BL      44

// Data pins: 18 entries for RGB666; R0 and B0 are -1 per your note.
static const int rgb_pins[18] = {
    /* R0..R5 */ -1, 11, 10, 9, 46, 3,
    /* G0..G5 */ 48, 47, 21, 14, 13, 12,
    /* B0..B5 */ -1, 40, 39, 38, 0, 45
};

/* --- Display timings / resolution (from your snippets) --- */
#define DISP_H_RES  720
#define DISP_V_RES  720
#define PCLK_HZ     (16 * 1000 * 1000) // 9 MHz pixel clock

// porch/pulse values from your message / datasheet snippet
#define HSPW 2
#define HBPD 44
#define HFPD 46
#define VSPW 5
#define VBPD 16
#define VFPD 50

/* --- Globals --- */
static esp_lcd_panel_handle_t panel_handle = NULL;
static spi_device_handle_t spi_dev = NULL;

/* --- Helper: check esp_err and log+abort on failure --- */
#define CHECK_ESP_OK(x, msg) do { esp_err_t __r = (x); if (__r != ESP_OK) { ESP_LOGE(TAG, "%s: %s", (msg), esp_err_to_name(__r)); return; } } while(0)

/* --- NV3052C SPI write helper
   Protocol note: send command byte(s) as the first byte(s) then data bytes.
   The user sequence uses single-byte register writes: Wrt_Reg_3052(cmd, val).
*/
static esp_err_t nv3052c_spi_write_byte(uint8_t b)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8; // bits
    t.tx_buffer = &b;
    // CS is handled by the host driver automatically for each transaction
    return spi_device_transmit(spi_dev, &t);
}

static esp_err_t nv3052c_write_reg(uint8_t reg, uint8_t val)
{
    // NV3052C expects: CS low, send register (command) byte with R/W bit = 0 (write),
    // then data byte(s). Many simple modules treat first byte as command code.
    // We'll send two bytes in one transaction (command, data).
    uint8_t tx[2] = { reg, val };
    spi_transaction_t t = { 0 };
    t.length = 16; // bits
    t.tx_buffer = tx;
    return spi_device_transmit(spi_dev, &t);
}

/* --- Load the register table you gave (converted to pairs) --- */
static void nv3052c_load_user_table(void)
{
    // Example subset: I include the long sequence the user provided.
    // Each row is {reg, val} (the original list in user message).
    const uint8_t reg_table[][2] = {
        {0xFF,0x30},{0xFF,0x52},{0xFF,0x01},
        {0xE3,0x00},{0x0A,0x11},
        {0x23,0xA0},{0x24,0x32},{0x25,0x12},{0x26,0x2E},{0x27,0x2E},
        {0x29,0x02},{0x2A,0xCF},{0x32,0x34},{0x38,0x9C},{0x39,0xA7},{0x3A,0x27},
        {0x3B,0x94},{0x42,0x6D},{0x43,0x83},{0x81,0x00},{0x91,0x67},{0x92,0x67},
        {0xA0,0x52},{0xA1,0x50},{0xA4,0x9C},{0xA7,0x02},{0xA8,0x02},{0xA9,0x02},
        {0xAA,0xA8},{0xAB,0x28},{0xAE,0xD2},{0xAF,0x02},{0xB0,0xD2},{0xB2,0x26},
        {0xB3,0x26},{0xFF,0x30},{0xFF,0x52},{0xFF,0x02},{0xB1,0x0A},{0xD1,0x0E},
        {0xB4,0x2F},{0xD4,0x2D},{0xB2,0x0C},{0xD2,0x0C},{0xB3,0x30},{0xD3,0x2A},
        {0xB6,0x1E},{0xD6,0x16},{0xB7,0x3B},{0xD7,0x35},{0xC1,0x08},{0xE1,0x08},
        {0xB8,0x0D},{0xD8,0x0D},{0xB9,0x05},{0xD9,0x05},{0xBD,0x15},{0xDD,0x15},
        {0xBC,0x13},{0xDC,0x13},{0xBB,0x12},{0xDB,0x10},{0xBA,0x11},{0xDA,0x11},
        {0xBE,0x17},{0xDE,0x17},{0xBF,0x0F},{0xDF,0x0F},{0xC0,0x16},{0xE0,0x16},
        {0xB5,0x2E},{0xD5,0x3F},{0xB0,0x03},{0xD0,0x02},{0xFF,0x30},{0xFF,0x52},
        {0xFF,0x03},{0x08,0x09},{0x09,0x0A},{0x0A,0x0B},{0x0B,0x0C},{0x28,0x22},
        {0x2A,0xE9},{0x2B,0xE9},{0x34,0x51},{0x35,0x01},{0x36,0x26},{0x37,0x13},
        {0x40,0x07},{0x41,0x08},{0x42,0x09},{0x43,0x0A},{0x44,0x22},{0x45,0xDB},
        {0x46,0xDC},{0x47,0x22},{0x48,0xDD},{0x49,0xDE},{0x50,0x0B},{0x51,0x0C},
        {0x52,0x0D},{0x53,0x0E},{0x54,0x22},{0x55,0xDF},{0x56,0xE0},{0x57,0x22},
        {0x58,0xE1},{0x59,0xE2},{0x80,0x1E},{0x81,0x1E},{0x82,0x1F},{0x83,0x1F},
        {0x84,0x05},{0x85,0x0A},{0x86,0x0A},{0x87,0x0C},{0x88,0x0C},{0x89,0x0E},
        {0x8A,0x0E},{0x8B,0x10},{0x8C,0x10},{0x8D,0x00},{0x8E,0x00},{0x8F,0x1F},
        {0x90,0x1F},{0x91,0x1E},{0x92,0x1E},{0x93,0x02},{0x94,0x04},{0x96,0x1E},
        {0x97,0x1E},{0x98,0x1F},{0x99,0x1F},{0x9A,0x05},{0x9B,0x09},{0x9C,0x09},
        {0x9D,0x0B},{0x9E,0x0B},{0x9F,0x0D},{0xA0,0x0D},{0xA1,0x0F},{0xA2,0x0F},
        {0xA3,0x00},{0xA4,0x00},{0xA5,0x1F},{0xA6,0x1F},{0xA7,0x1E},{0xA8,0x1E},
        {0xA9,0x01},{0xAA,0x03},
        {0xFF,0x30},{0xFF,0x52},{0xFF,0x00},{0x36,0x0A},{0x11,0x00},
        // Delay_ms(200) in original list
        {0x29,0x00},
        // Delay_ms(100) at end
    };

    const size_t entries = sizeof(reg_table) / sizeof(reg_table[0]);
    for (size_t i = 0; i < entries; ++i) {
        const uint8_t reg = reg_table[i][0];
        const uint8_t val = reg_table[i][1];

        esp_err_t r = nv3052c_write_reg(reg, val);
        if (r != ESP_OK) {
            ESP_LOGW(TAG, "nv3052 write %02X=%02X failed: %s", reg, val, esp_err_to_name(r));
        }
        // add specific delays where original sequence requested them
        if (reg == 0x11) { // SLPOUT (0x11) — datasheet suggests wait after this
            vTaskDelay(pdMS_TO_TICKS(200)); // original had 200ms somewhere
        }
        if (reg == 0x29) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        // short inter-command delay
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    ESP_LOGI(TAG, "NV3052C register table loaded");
}

/* --- Initialise SPI for NV3052C command writes --- */
static void spi_for_nv3052c_init(void)
{
    esp_err_t ret;

    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_PANEL_SPI_MOSI,
        .miso_io_num = PIN_PANEL_SPI_MISO,
        .sclk_io_num = PIN_PANEL_SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64, // small transfers for regs
    };
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000, // 10MHz clock for command SPI safe for NV3052C
        .mode = 0, // CPOL=0 CPHA=0 (sample on rising edge)
        .spics_io_num = PIN_PANEL_SPI_CS,
        .queue_size = 1,
    };

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "SPI for NV3052C initialised (CS=%d, SCLK=%d, MOSI=%d)", PIN_PANEL_SPI_CS, PIN_PANEL_SPI_SCLK, PIN_PANEL_SPI_MOSI);
}

/* --- Main application --- */
void app_main(void)
{
    ESP_LOGI(TAG, "Starting Qualia RGB initialisation example");

    // backlight pin
    if (PIN_BL >= 0) {
        gpio_reset_pin(PIN_BL);
        gpio_set_direction(PIN_BL, GPIO_MODE_OUTPUT);
        gpio_set_level(PIN_BL, 0);
    }

    // init SPI for NV3052C register writes
    spi_for_nv3052c_init();

    // Configure the RGB panel struct
    esp_lcd_rgb_panel_config_t panel_config = {
        .data_width = 18,
        .num_fbs = 2,
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .disp_gpio_num = PIN_DISP,
        .pclk_gpio_num = PIN_PCLK,
        .vsync_gpio_num = PIN_VSYNC,
        .hsync_gpio_num = PIN_HSYNC,
        .de_gpio_num = PIN_DE,
        .data_gpio_nums = { 0 }, // we'll fill below
        .timings = {
            .pclk_hz = PCLK_HZ,
            .h_res = DISP_H_RES,
            .v_res = DISP_V_RES,
            .hsync_pulse_width = HSPW,
            .hsync_back_porch = HBPD,
            .hsync_front_porch = HFPD,
            .vsync_pulse_width = VSPW,
            .vsync_back_porch = VBPD,
            .vsync_front_porch = VFPD,
        },
        .flags = {
            .fb_in_psram = true,
        },
    };

    // copy data pins (must match data_width)
    for (int i = 0; i < panel_config.data_width; ++i) {
        panel_config.data_gpio_nums[i] = rgb_pins[i];
    }

    // create panel
    esp_err_t ret = esp_lcd_new_rgb_panel(&panel_config, &panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_lcd_new_rgb_panel failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "RGB panel driver created");

    // Hardware reset if the driver exposes reset pin handling; otherwise toggle RESX if you wired it.
    ret = esp_lcd_panel_reset(panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "panel reset returned %s", esp_err_to_name(ret));
    }

    // Initialize panel internal logic (driver). Many panel drivers require esp_lcd_panel_init.
    ret = esp_lcd_panel_init(panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "panel init failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Panel init done, now loading NV3052C register table via SPI");

    // software reset the NV3052C (via SPI) before loading registers
    nv3052c_write_reg(0x01, 0x00); // SWRESET (01h), no param per datasheet
    vTaskDelay(pdMS_TO_TICKS(5));  // required after SWRESET. :contentReference[oaicite:4]{index=4}

    // load the long register list you had
    nv3052c_load_user_table();

    // Sleep Out (SLPOUT) command (0x11) then wait (datasheet: a little delay)
    nv3052c_write_reg(0x11, 0x00);
    vTaskDelay(pdMS_TO_TICKS(200)); // conservative — original sequence used delays. :contentReference[oaicite:5]{index=5}

    // Display On (DISP ON 0x29)
    nv3052c_write_reg(0x29, 0x00);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Turn on backlight
    if (PIN_BL >= 0) {
        gpio_set_level(PIN_BL, 1);
    }

    ESP_LOGI(TAG, "NV3052C initialisation complete; starting RGB test.");

    // --- Simple RGB test using 16-bit RGB565 frame buffer (driver usually accepts 16-bit frames) ---
    size_t fb_pixels = DISP_H_RES * DISP_V_RES;
    size_t fb_size_bytes = fb_pixels * sizeof(uint16_t);

    // Allocate in PSRAM explicitly to be safe (heap_caps)
    uint16_t *fb = (uint16_t *) heap_caps_malloc(fb_size_bytes, MALLOC_CAP_SPIRAM);
    if (!fb) {
        ESP_LOGE(TAG, "Failed to allocate framebuffer in PSRAM");
        return;
    }

    for (;;) {
        // Red full screen (RGB565 0xF800)
        memset(fb, 0x00, fb_size_bytes); // not strictly necessary
        for (size_t i = 0; i < fb_pixels; ++i) fb[i] = 0xF800;
        esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, DISP_H_RES, DISP_V_RES, fb);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Green (0x07E0)
        for (size_t i = 0; i < fb_pixels; ++i) fb[i] = 0x07E0;
        esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, DISP_H_RES, DISP_V_RES, fb);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Blue (0x001F)
        for (size_t i = 0; i < fb_pixels; ++i) fb[i] = 0x001F;
        esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, DISP_H_RES, DISP_V_RES, fb);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // never reached
    heap_caps_free(fb);
}
