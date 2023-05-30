#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "include/cc1101.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "hal/gpio_types.h"
#include "hal/spi_types.h"
#include <stdint.h>
#include <string.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_check.h"

#define TAG "cc1101"

#define ERR_MSG_SPI_BUS_LOCK "Unable to acquire SPI bus lock"
#define ERR_MSG_SPI_TX_BEGIN "Unable to begin transaction"
#define ERR_MSG_SPI_TX "Unable to perform SPI TX"

#ifdef CONFIG_ENABLE_DEBUG_SYMBOLS
const char *cc1101_config_reg_names[] = {
    "IOCFG2",  "IOCFG1",   "IOCFG0",   "FIFOTHR",  "SYNC1",    "SYNC0",
    "PKTLEN",  "PKTCTRL1", "PKTCTRL0", "ADDR",     "CHANNR",   "FSCTRL1",
    "FSCTRL0", "FREQ2",    "FREQ1",    "FREQ0",    "MDMCFG4",  "MDMCFG3",
    "MDMCFG2", "MDMCFG1",  "MDMCFG0",  "DEVIATN",  "MCSM2",    "MCSM1",
    "MCSM0",   "FOCCFG",   "BSCFG",    "AGCCTRL2", "AGCCTRL1", "AGCCTRL0",
    "WOREVT1", "WOREVT0",  "WORCTRL",  "FREND1",   "FREND0",   "FSCAL3",
    "FSCAL2",  "FSCAL1",   "FSCAL0",   "RCCTRL1",  "RCCTRL0",  "FSTEST",
    "PTEST",   "AGCTEST",  "TEST2",    "TEST1",    "TEST0"};

const char *cc1101_status_reg_names[] = {
    "PARTNUM",  "VERSION",  "FREQEST",   "LQI", "RSSI",    "MARCSTATE",
    "WORTIME1", "WORTIME0", "PKTSTATUS", "VCO", "TXBYTES", "RXBYTES",
};
#endif

static void active_wait_us(int micros) {
  int64_t begin = esp_timer_get_time();
  while (esp_timer_get_time() - begin < micros);
}

static inline esp_err_t cc1101_chip_select(const cc1101_device_t *device) {
  return gpio_set_level(device->cs_io_num, 0);
}

static inline esp_err_t cc1101_chip_deselect(const cc1101_device_t *device) {
  return gpio_set_level(device->cs_io_num, 1);
}

esp_err_t cc1101_init(spi_host_device_t spi_host, cc1101_device_t *device) {
  esp_err_t err;

  gpio_reset_pin(device->cs_io_num);
  gpio_set_direction(device->cs_io_num, GPIO_MODE_OUTPUT);
  cc1101_chip_deselect(device);

  spi_device_interface_config_t devcfg = {
    .clock_speed_hz=CC1101_SCLK_HZ,
    .mode=0,
    // Datasheet specifies that once the CS is pulled low, MCU must
    // wait until MISO goes low before start TX. For that reason we're
    // manually controlling CS.
    .spics_io_num=-1,
    .queue_size=CC1101_SPI_QUEUE_SIZE,
    // The most common way of communicating with the CC1101 is using a
    // header byte prior to sending data, so we're setting this as
    // default and opting-out when we need it.
    .command_bits = 8
  };

  if ((err = spi_bus_add_device(spi_host, &devcfg, &device->spi_device)) != ESP_OK) {
    return err;
  }

  return ESP_OK;
}

static void spi_fill_tx(spi_transaction_t* trans, uint8_t cmd, const uint8_t* in, uint8_t* out, size_t len) {
  trans->length = len * 8;
  trans->tx_buffer = in;
  trans->rx_buffer = out;
  trans->cmd = cmd;
}

static esp_err_t spi_tx(spi_device_handle_t handle, uint8_t cmd, const uint8_t* in, uint8_t* out, size_t len) {
  spi_transaction_t trans = { 0 };
  spi_fill_tx(&trans, cmd, in, out, len);
  return spi_device_transmit(handle, &trans);
}

static esp_err_t spi_tx_no_cmd(spi_device_handle_t handle, const uint8_t* in, uint8_t* out, size_t len) {
  spi_transaction_ext_t trans = { 0 };
  spi_fill_tx(&trans.base, 0, in, out, len);
  trans.base.flags |= SPI_TRANS_VARIABLE_CMD;
  trans.command_bits = 0;
  return spi_device_transmit(handle, (spi_transaction_t*) &trans);
}

static esp_err_t spi_begin_transaction(const cc1101_device_t *device) {
  int64_t begin_time = esp_timer_get_time();
  cc1101_chip_select(device);

  // TODO Improve this? using CPU cycles or something? Pulling down
  // MISO takes nothing, sometimes not even going into this while, so
  // using esp_timer_get_time() is too much.
  while(gpio_get_level(device->miso_io_num) && (esp_timer_get_time() - begin_time) < CC1101_SPI_BEGIN_TIMEOUT_US);
  if (gpio_get_level(device->miso_io_num)) {
    cc1101_chip_deselect(device);
    ESP_LOGE(TAG, "SPI transaction initialization timed-out");
    return ESP_ERR_TIMEOUT;
  }

  return ESP_OK;
}

static void spi_end_transaction(const cc1101_device_t *device) {
  active_wait_us(1);
  gpio_set_level(device->cs_io_num, 1);
}

static esp_err_t cc1101_spi_tx_no_cmd(const cc1101_device_t* device, const uint8_t* in,
			uint8_t* out, size_t len) {
  esp_err_t ret = ESP_OK;

  ESP_RETURN_ON_ERROR(spi_device_acquire_bus(device->spi_device, portMAX_DELAY), TAG, ERR_MSG_SPI_BUS_LOCK);
  ESP_GOTO_ON_ERROR(spi_begin_transaction(device), end_bus, TAG, ERR_MSG_SPI_TX_BEGIN);
  ESP_GOTO_ON_ERROR(spi_tx_no_cmd(device->spi_device, in, out, len), end_tx, TAG, ERR_MSG_SPI_TX);

 end_tx:
  spi_end_transaction(device);

 end_bus:
  spi_device_release_bus(device->spi_device);
  return ret;
 }


esp_err_t cc1101_spi_tx(const cc1101_device_t* device, uint8_t cmd, const uint8_t* in,
			uint8_t* out, size_t len) {
  esp_err_t ret = ESP_OK;

  ESP_RETURN_ON_ERROR(spi_device_acquire_bus(device->spi_device, portMAX_DELAY), TAG, ERR_MSG_SPI_BUS_LOCK);
  ESP_GOTO_ON_ERROR(spi_begin_transaction(device), end_bus, TAG, ERR_MSG_SPI_TX_BEGIN);
  ESP_GOTO_ON_ERROR(spi_tx(device->spi_device, cmd, in, out, len), end_tx, TAG, ERR_MSG_SPI_TX);

 end_tx:
  spi_end_transaction(device);

 end_bus:
  spi_device_release_bus(device->spi_device);
  return ret;
 }

esp_err_t cc1101_spi_tx_u8(const cc1101_device_t* device, uint8_t cmd, uint8_t in,
			   uint8_t* out) {
  return cc1101_spi_tx(device, cmd, &in, out, 1);
}

#ifdef CONFIG_ENABLE_DEBUG_SYMBOLS
#define REG_FORMAT "%02x:%-9s = %02x"
#define REG_SEPARATOR "  |  "
#define REG_FORMAT_PARAMS(name, addr, value) addr, name, value

const char* cc1101_device_state_str(cc1101_device_state_t state) {
  switch (state) {
  case CC1101_DEVICE_STATE_IDLE:
    return "IDLE";
  case CC1101_DEVICE_STATE_RX:
    return "RX";
  case CC1101_DEVICE_STATE_TX:
    return "TX";
  case CC1101_DEVICE_STATE_FSTXON:
    return "FSTXON";
  case CC1101_DEVICE_STATE_CALIBRATE:
    return "CALIBRATE";
  case CC1101_DEVICE_STATE_SETTLING:
    return "SETTLING";
  case CC1101_DEVICE_STATE_RXFIFO_OVERFLOW:
    return "RXFIFO_OVERFLOW";
  case CC1101_DEVICE_STATE_TXFIFO_UNDERFLOW:
    return "TXFIFO_UNDERFLOW";
  default:
    return "INVALID";
  }
}

const char *cc1101_config_reg_name(cc1101_config_reg_t reg) {
  return cc1101_config_reg_names[reg - CC1101_FIRST_CFG_REG];
}
const char *cc1101_status_reg_name(cc1101_status_reg_t reg) {
  return cc1101_status_reg_names[reg - CC1101_FIRST_STATUS_REG];
}

esp_err_t cc1101_debug_print_regs(const cc1101_device_t *device) {
  size_t num_conf_regs = CC1101_CONFIG_REG_COUNT;
  size_t num_status_regs = CC1101_STATUS_REG_COUNT;
  uint8_t config_regs[num_conf_regs];
  uint8_t status_regs[num_status_regs];
  uint8_t patable[CC1101_PATABLE_SIZE] = { 0 };
  cc1101_chip_status_t status;

  ESP_RETURN_ON_ERROR(cc1101_chip_status(device, &status),
		      TAG, "Couldn't read chip status");

  ESP_RETURN_ON_ERROR(cc1101_read_burst(device, CC1101_FIRST_CFG_REG, config_regs, num_conf_regs),
		      TAG, "Couldn't read config regs");

  for (int i = 0; i < num_status_regs; i++) {
    ESP_RETURN_ON_ERROR(cc1101_read_status_reg(device, CC1101_FIRST_STATUS_REG + i, &status_regs[i]),
			TAG, "Couldn't read status regs");
  }

  ESP_RETURN_ON_ERROR(cc1101_read_patable(device, patable),
		      TAG, "Couldn't read PATABLE");

  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "Chip status: !ready= %d; state= %s; fifo usage data= %d",
	   status.fields.ready,
	   cc1101_device_state_str(status.fields.state),
	   status.fields.fifo_bytes_available);


  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "Config registers values:");
  ESP_LOGI(TAG, "");
  for (int i = 0; i < num_conf_regs; i += 3) {
    size_t remaining = num_conf_regs - i;
    uint8_t reg_addr = CC1101_FIRST_CFG_REG + i;
    if (remaining >= 3) {
      ESP_LOGI(TAG, REG_FORMAT REG_SEPARATOR REG_FORMAT REG_SEPARATOR REG_FORMAT,
	       REG_FORMAT_PARAMS(cc1101_config_reg_name(reg_addr), reg_addr, config_regs[i]),
	       REG_FORMAT_PARAMS(cc1101_config_reg_name(reg_addr + 1), reg_addr + 1, config_regs[i + 1]),
	       REG_FORMAT_PARAMS(cc1101_config_reg_name(reg_addr + 2), reg_addr + 2, config_regs[i + 2]));
    } else if (remaining >= 2) {
      ESP_LOGI(TAG, REG_FORMAT REG_SEPARATOR REG_FORMAT REG_SEPARATOR,
	       REG_FORMAT_PARAMS(cc1101_config_reg_name(reg_addr), reg_addr, config_regs[i]),
	       REG_FORMAT_PARAMS(cc1101_config_reg_name(reg_addr + 1), reg_addr + 1, config_regs[i + 1]));
    } else {
      ESP_LOGI(TAG, REG_FORMAT REG_SEPARATOR,
	       REG_FORMAT_PARAMS(cc1101_config_reg_name(reg_addr), reg_addr, config_regs[i]));
    }
  }

  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "Status registers values:");
  ESP_LOGI(TAG, "");
  for (int i = 0; i < num_status_regs; i += 3) {
    size_t remaining = num_status_regs - i;
    uint8_t reg_addr = CC1101_FIRST_STATUS_REG + i;
    if (remaining >= 3) {
      ESP_LOGI(TAG, REG_FORMAT REG_SEPARATOR REG_FORMAT REG_SEPARATOR REG_FORMAT,
	       REG_FORMAT_PARAMS(cc1101_status_reg_name(reg_addr), reg_addr, status_regs[i]),
	       REG_FORMAT_PARAMS(cc1101_status_reg_name(reg_addr + 1), reg_addr + 1, status_regs[i + 1]),
	       REG_FORMAT_PARAMS(cc1101_status_reg_name(reg_addr + 2), reg_addr + 2, status_regs[i + 2]));
    } else if (remaining >= 2) {
      ESP_LOGI(TAG, REG_FORMAT REG_SEPARATOR REG_FORMAT REG_SEPARATOR,
	       REG_FORMAT_PARAMS(cc1101_status_reg_name(reg_addr), reg_addr, status_regs[i]),
	       REG_FORMAT_PARAMS(cc1101_status_reg_name(reg_addr + 1), reg_addr + 1, status_regs[i + 1]));
    } else {
      ESP_LOGI(TAG, REG_FORMAT REG_SEPARATOR,
	       REG_FORMAT_PARAMS(cc1101_status_reg_name(reg_addr), reg_addr, status_regs[i]));
    }
  }

  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "PATABLE:");
  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "%02x %02x %02x %02x %02x %02x %02x %02x",
	   patable[0], patable[1], patable[2],
	   patable[3], patable[4], patable[5],
	   patable[6], patable[7]);

  return ESP_OK;
}

#endif

esp_err_t cc1101_strobe(const cc1101_device_t *device, cc1101_strobe_t strobe, cc1101_chip_status_t* chip_status) {
  return cc1101_spi_tx_no_cmd(device, &strobe, &chip_status->byte, 1);
}

esp_err_t cc1101_chip_status(const cc1101_device_t *device,
                             cc1101_chip_status_t *chip_status) {
  // Chip status can be obtained from the CC1101 as the RX data coming
  // during a SPI transaction when the header byte is being sent, so
  // that basically means it can be obtained virtually anyway. Just
  // picking this one because I like it.
  return cc1101_strobe(device, CC1101_STROBE_NOP, chip_status);
}

esp_err_t cc1101_write_config_reg(const cc1101_device_t* device, cc1101_config_reg_t reg, uint8_t value) {
  return cc1101_spi_tx_u8(device, reg, value, NULL);
}

esp_err_t cc1101_read_config_reg(const cc1101_device_t* device, cc1101_config_reg_t reg, uint8_t *value) {
  return cc1101_spi_tx_u8(device, REGREAD(reg), 0, value);
}

esp_err_t cc1101_read_status_reg(const cc1101_device_t* device, cc1101_status_reg_t reg, uint8_t *value) {
  return cc1101_spi_tx_u8(device, STREAD(reg), 0, value);
}

esp_err_t cc1101_write_burst(const cc1101_device_t* device, cc1101_config_reg_t reg_begin, const uint8_t* values, size_t len) {
  return cc1101_spi_tx(device, BURSTWRITE(reg_begin), values, NULL, len);
}

esp_err_t cc1101_read_burst(const cc1101_device_t* device, cc1101_config_reg_t reg_begin, uint8_t* output, size_t len) {
  uint8_t txbuf[len];
  memset(txbuf, 0, len);
  return cc1101_spi_tx(device, BURSTREAD(reg_begin), txbuf, output, len);
}

esp_err_t cc1101_write_patable(const cc1101_device_t *device, const uint8_t data[CC1101_PATABLE_SIZE]) {
  return cc1101_spi_tx(device, BURSTWRITE(CC1101_HEADER_PATABLE), data, NULL, CC1101_PATABLE_SIZE);
}

esp_err_t cc1101_read_patable(const cc1101_device_t *device, uint8_t data[CC1101_PATABLE_SIZE]) {
  uint8_t txbuf[CC1101_PATABLE_SIZE] = { 0 };
  return cc1101_spi_tx(device, BURSTREAD(CC1101_HEADER_PATABLE), txbuf, data, CC1101_PATABLE_SIZE);
}

esp_err_t cc1101_hard_reset(const cc1101_device_t *device) {
  cc1101_chip_select(device);
  active_wait_us(5);
  cc1101_chip_deselect(device);
  active_wait_us(40);
  return cc1101_strobe(device, CC1101_STROBE_RES, NULL);
}
