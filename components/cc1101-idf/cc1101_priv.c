#include "include/cc1101.h"
#include "include_priv/cc1101_priv.h"
#include "esp_err.h"

cc1101_device_priv_t cc1101_devices[CONFIG_MAX_CC1101_DEVICES] = { 0 };

esp_err_t cc1101_allocate_device(const cc1101_device_cfg_t* cfg, cc1101_device_priv_t** ptr) {
  size_t free_slot;

  for (free_slot = 0; free_slot < CONFIG_MAX_CC1101_DEVICES; free_slot++) {
    if (cc1101_devices[free_slot].spi_device == NULL) {
      *ptr = &cc1101_devices[free_slot];
      break;
    }
  }

  if (free_slot >= CONFIG_MAX_CC1101_DEVICES) {
    return ESP_ERR_NO_MEM;
  }

  return ESP_OK;
}

cc1101_device_priv_t* cc1101_get_device(const cc1101_device_t* dev) {
  return (cc1101_device_priv_t*) dev;
}
