#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "driver/spi_master.h"
#include "cc1101.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <stdint.h>

#define MISO_GPIO GPIO_NUM_37
#define MOSI_GPIO GPIO_NUM_35
#define SCLK_GPIO GPIO_NUM_36
#define CSN_GPIO GPIO_NUM_38

#define TAG "sanity-check"

void app_main(void) {
  spi_bus_config_t spi_bus_cfg = {
    .miso_io_num = MISO_GPIO,
    .mosi_io_num = MOSI_GPIO,
    .sclk_io_num = SCLK_GPIO,
  };

  cc1101_device_cfg_t cfg = {
    // GDO pins are not needed for this example.
    .gdo0_io_num = -1,
    .gdo2_io_num = -1,
    .cs_io_num = CSN_GPIO,
    .miso_io_num = MISO_GPIO,
    .spi_host = SPI2_HOST
  };

  cc1101_device_t* cc;

  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &spi_bus_cfg, SPI_DMA_CH_AUTO));
  ESP_ERROR_CHECK(cc1101_init(&cfg, &cc));

  bool sanity_result = true;
  ESP_LOGI(TAG, "Hard resetting device...");
  ESP_ERROR_CHECK(cc1101_hard_reset(cc));

  ESP_LOGI(TAG, "Device internal state after reset:");
  ESP_LOGI(TAG, "");

  ESP_ERROR_CHECK(cc1101_debug_print_regs(cc));
  ESP_LOGI(TAG, "Checking version and part number...");
  uint8_t version;
  uint8_t partnum;

  ESP_ERROR_CHECK(cc1101_read_status_reg(cc, CC1101_REG_STATUS_PARTNUM, &partnum));
  ESP_ERROR_CHECK(cc1101_read_status_reg(cc, CC1101_REG_STATUS_VERSION, &version));

  if (partnum == 0 && version == 0x14) {
    ESP_LOGI(TAG, "Version and part number seems to be fine! (partnum= 0x%02x; version= 0x%02x)", partnum, version);
  } else {
    sanity_result = false;
    ESP_LOGE(TAG, "Version and part number doesn't match expected values. There might be something wrong! (partnum= 0x%02x; version= 0x%02x)",
	     partnum, version);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }

  ESP_LOGI(TAG, "Trying to write values into all the registers...");
  uint8_t regs_expected[CC1101_CONFIG_REG_COUNT];
  uint8_t regs_got[CC1101_CONFIG_REG_COUNT];

  for (int i = 0; i < CC1101_CONFIG_REG_COUNT; i++) {
    if (CC1101_FIRST_CFG_REG + i == CC1101_REG_CFG_AGCTEST) {
      regs_expected[i] = 0xff;
    } else {
      regs_expected[i] = i;
    }
  }

  ESP_ERROR_CHECK(cc1101_write_burst(cc, CC1101_FIRST_CFG_REG, regs_expected, CC1101_CONFIG_REG_COUNT));
  ESP_LOGI(TAG, "Success! This is new internal state:");
  ESP_LOGI(TAG, "");
  ESP_ERROR_CHECK(cc1101_debug_print_regs(cc));

  ESP_LOGI(TAG, "Reading all the registers...");
  ESP_ERROR_CHECK(cc1101_read_burst(cc, CC1101_FIRST_CFG_REG, regs_got, CC1101_CONFIG_REG_COUNT));

  for (int i = 0; i < CC1101_CONFIG_REG_COUNT; i++) {
    if (regs_expected[i] != regs_got[i]) {
      sanity_result = false;
      ESP_LOGE(TAG, "Unexpected value at register 0x%02x (%s); Expected value to be 0x%02x but 0x%02x got.",
	       CC1101_FIRST_CFG_REG + i, cc1101_config_reg_name(CC1101_FIRST_CFG_REG + i), regs_expected[i], regs_got[i]);
    }
  }

  if (sanity_result) {
    ESP_LOGI(TAG, "Device PASSED sanity check!");
  } else {
    ESP_LOGE(TAG, "Device FAILED sanity check! Check logs for more info");
  }
  ESP_ERROR_CHECK(cc1101_hard_reset(cc));

}
