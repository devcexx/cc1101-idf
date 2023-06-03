#include "esp_attr.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "hal/gpio_types.h"
#include "hal/spi_types.h"
#include "cc1101.h"
#include "sdkconfig.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <stdint.h>

#define MISO_GPIO GPIO_NUM_37
#define MOSI_GPIO GPIO_NUM_35
#define SCLK_GPIO GPIO_NUM_36
#define CSN_GPIO GPIO_NUM_38

// In this mode, this pin will be used as input. The CC1101 will be
// generate a clock at a frequency equal to the data rate (in this
// case, 16 kHz).
#define GDO2_GPIO GPIO_NUM_40

// Used for data transmission. The CC1101 will sample the signal of
// the GPIO on each clock raise.
#define GDO0_GPIO GPIO_NUM_17

#define TAG "transmit-sync-mode"

/*
 * Hardcoded value of the registers for the CC1101 since the library
 * still lacking of functions to properly set it up. These values can
 * be obtained by using the TI SmartRF studio software.
 *
 * https://www.ti.com/tool/SMARTRFTM-STUDIO
 *
 * This example configuration is mainly setting:
 * - 433.92 MHz frequency (check your local legislation!)
 * - 10dBm TX power.
 * - Data rate at 16 kbps.
 * - Modulation ASK/OOK.
 * - Infinte packet length (for doing a continuous transmission).
 */
uint8_t registers[] = {
    0x0b, 0x2e, 0x3f, 0x07, 0xd3, 0x91, 0xff, 0x04, 0x12, 0x00, 0x00, 0x0f,
    0x00, 0x10, 0xb0, 0x4b, 0x89, 0x42, 0x30, 0x22, 0xf8, 0x47, 0x07, 0x30,
    0x18, 0x14, 0x6c, 0x03, 0x40, 0x91, 0x87, 0x6b, 0xf8, 0x56, 0x11, 0xaa,
    0x2a, 0x17, 0x0d, 0x41, 0x00, 0x59, 0x7f, 0x3f, 0x88, 0x31, 0x0b};


uint8_t patable[8] = {0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

volatile int64_t counter = 0;
volatile bool prev_level = false;

/*
 * Callback that will be called on each clock pulse when signal raises.
 */
static void IRAM_ATTR sync_mode_clk_cb(const cc1101_device_t* device, void* user) {
  counter++;
  prev_level = !prev_level;
  cc1101_trans_continuous_write(device, prev_level);
}

void app_main(void) {
  // Install GPIO ISR service so that GPIO can handle
  // interruptions. The library will add the proper ISR handlers for
  // calling your callback, but won't do this for you.
  ESP_ERROR_CHECK(gpio_install_isr_service(0));

  spi_bus_config_t spi_bus_cfg = {
    .miso_io_num = MISO_GPIO,
    .mosi_io_num = MOSI_GPIO,
    .sclk_io_num = SCLK_GPIO,
  };

  cc1101_device_cfg_t cfg = {
    .spi_host = SPI2_HOST,
    .gdo0_io_num = 17,
    .gdo2_io_num = 40,
    .cs_io_num = CSN_GPIO,
    .miso_io_num = MISO_GPIO
  };

  cc1101_device_t* cc;

  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &spi_bus_cfg, SPI_DMA_CH_AUTO));
  ESP_ERROR_CHECK(cc1101_init(&cfg, &cc));

  ESP_LOGI(TAG, "Hard resetting device...");
  ESP_ERROR_CHECK(cc1101_hard_reset(cc));

  ESP_LOGI(TAG, "Configuring...");
  ESP_ERROR_CHECK(cc1101_write_burst(cc, CC1101_FIRST_CFG_REG, registers, sizeof(registers)));
  ESP_ERROR_CHECK(cc1101_write_patable(cc, patable));

  cc1101_sync_mode_cfg_t sync_cfg = {
    .clk_cb = sync_mode_clk_cb,
    .user = NULL
  };

  ESP_ERROR_CHECK(cc1101_configure_sync_mode(cc, &sync_cfg));
  ESP_ERROR_CHECK(cc1101_enable_tx(cc, CC1101_TRANS_MODE_SYNCHRONOUS));

  vTaskDelay(1); // Wait for auto calibration to finish.
  int64_t begin_time = esp_timer_get_time();
  while (true) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Counter: %lld; Freq: %.4f Hz", counter, (double)counter / ((esp_timer_get_time() - begin_time) / 1000000.0) );
  }
}
