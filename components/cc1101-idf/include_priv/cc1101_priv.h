#ifndef CC1101_PRIV_H
#define CC1101_PRIV_H

#include "../include/cc1101.h"
#include "esp_err.h"

typedef enum {
  CC1101_PRIV_STATE_IDLE = 0,
  CC1101_PRIV_STATE_TX = 1,
  CC1101_PRIV_STATE_RX = 2,
} cc1101_priv_state_t;

typedef struct {
  spi_device_handle_t spi_device;

  gpio_num_t miso_io_num;
  gpio_num_t gdo0_io_num;
  gpio_num_t gdo2_io_num;
  gpio_num_t cs_io_num;

  cc1101_priv_state_t last_state;
  cc1101_trans_mode_t last_trans_mode;
  bool sync_mode_configured;
  cc1101_sync_mode_cfg_t sync_mode_cfg;
} cc1101_device_priv_t;

#endif // CC1101_PRIV_H

// TODO Device access synchronization!
extern cc1101_device_priv_t cc1101_devices[CONFIG_MAX_CC1101_DEVICES];

esp_err_t cc1101_allocate_device(const cc1101_device_cfg_t* cfg, cc1101_device_priv_t** ptr);
cc1101_device_priv_t* cc1101_get_device(const cc1101_device_t* dev);
