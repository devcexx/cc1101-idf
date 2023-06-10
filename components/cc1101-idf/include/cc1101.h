#ifndef CC1101_H
#define CC1101_H

#include "driver/spi_master.h"
#include "hal/gpio_types.h"
#include "esp_err.h"
#include <stdint.h>
#include "cc1101_regs.h"

#define CC1101_SCLK_HZ 5000000
#define CC1101_SPI_QUEUE_SIZE 10
#define CC1101_SPI_BEGIN_TIMEOUT_US 100

#define CC1101_PIN_UNUSED -1

#define CC1101_CRYSTAL_26MHZ 26000000
#define CC1101_CRYSTAL_27MHZ 27000000

typedef enum __attribute__((packed)) {
  CC1101_STROBE_RES = 0x30,     // Reset chip.
  CC1101_STROBE_FSTXON = 0x31,  // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
                                // If in RX/TX: Go to a wait state where only the synthesizer is
                                // running (for quick RX / TX turnaround).
  CC1101_STROBE_XOFF = 0x32,    // Turn off crystal oscillator.
  CC1101_STROBE_CAL = 0x33,     // Calibrate frequency synthesizer and turn it off
                                // (enables quick start).
  CC1101_STROBE_RX = 0x34,      // Enable RX. Perform calibration first if coming from IDLE and
                                // MCSM0.FS_AUTOCAL=1.
  CC1101_STROBE_TX = 0x35,      // In IDLE state: Enable TX. Perform calibration first if
                                // MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
                                // Only go to TX if channel is clear.
  CC1101_STROBE_IDLE = 0x36,    // Exit RX / TX, turn off frequency synthesizer and exit
                                // Wake-On-Radio mode if applicable.
  CC1101_STROBE_AFC = 0x37,     // Perform AFC adjustment of the frequency synthesizer
  CC1101_STROBE_WOR = 0x38,     // Start automatic RX polling sequence (Wake-on-Radio)
  CC1101_STROBE_PWD = 0x39,     // Enter power down mode when CSn goes high.
  CC1101_STROBE_FRX = 0x3A,     // Flush the RX FIFO buffer.
  CC1101_STROBE_FTX = 0x3B,     // Flush the TX FIFO buffer.
  CC1101_STROBE_WORRST = 0x3C,  // Reset real time clock.
  CC1101_STROBE_NOP = 0x3D,     // No operation. May be used to pad strobe commands to two
                                // INT8Us for simpler software.

} cc1101_strobe_t;

#define CC1101_HEADER_PATABLE 0x3E
#define CC1101_PATABLE_SIZE 8

// Header building macros

/*
 * How header bytes works for SPI transmissions:  Every SPI
 * transaction (read registers, write registers, strobe, etc) is
 * initiated by a header byte, followed by zero, one, or more data
 * bytes, depending on the operation.
 *
 * The header byte has the following format:
 *
 * Header byte: x|x|xxxxxx
 *              R B addr
 *
 * where:
 * addr: the address of the register or strobe to access / execute.
 * R: Read/Write bit
 *    - If the addr is 0x3F, it is used to determine whether the TX
 *      or the RX FIFO is accessed. TX FIFO is write-only; and RX FIFO is read-only:
 *      - If set, RX is accessed.
 *	- If unset, TX is accessed.
 *    - Otherwise:
        - If set, the operation is a read operation.
	- If unset, the operation is a write operation.
 * B: Burst bit.
 *    - If accessing addr between 0x30 and 0x3D:
 *      - If set, it access registers (status registers)
 *	- If unset, it is treated as a command strobe.
 *    - If accessing config registers, it is used to determine
        if we're going to initiate a burst operation:
	- If set, it means we're gonna initiate a burst operation.
	- If unset, it means we're gonna initiate a normal operation.
 */

/**
 * Returns the header byte for starting a burst read beginning on the
 * given config register.
 */
#define BURSTREAD(reg) ((reg) | 0xC0)

/**
 * Returns the header byte for starting a burst write beginning on the
 * given config register.
 */
#define BURSTWRITE(reg) ((reg) | 0x40)

/**
 * Returns the header byte for writing a single given config register.
 */
#define REGREAD(reg) ((reg) | 0x80)

/**
 * Returns the header byte for writing a single given config register.
 */
#define REGWRITE(reg) (reg)

/**
 * Returns the header byte for reading a status register.
 */
#define STREAD(reg) BURSTREAD(reg)

typedef struct {
  // Keep the same header as cc1101_device_t so a ptr to
  // cc1101_device_priv_t can be trated as a ptr to cc1101_device_t.
  spi_device_handle_t spi_device;
} cc1101_device_t;

typedef struct {
  spi_host_device_t spi_host;
  gpio_num_t miso_io_num;
  gpio_num_t gdo0_io_num;
  gpio_num_t gdo2_io_num;
  gpio_num_t cs_io_num;
  uint32_t crystal_freq;
} cc1101_device_cfg_t;

typedef enum __attribute__((packed)) {
  CC1101_DEVICE_STATE_IDLE = 0,
  CC1101_DEVICE_STATE_RX = 1,
  CC1101_DEVICE_STATE_TX = 2,
  CC1101_DEVICE_STATE_FSTXON = 3,
  CC1101_DEVICE_STATE_CALIBRATE = 4,
  CC1101_DEVICE_STATE_SETTLING = 5,
  CC1101_DEVICE_STATE_RXFIFO_OVERFLOW = 6,
  CC1101_DEVICE_STATE_TXFIFO_UNDERFLOW = 7,
} cc1101_device_state_t;

typedef enum __attribute__((packed)) {
  /**
   * Indicates the Normal TX/RX mode. In this mode data is
   * sent/received through SPI using the device FIFO queues.
   */
  CC1101_TRANS_MODE_NORMAL = 0, // TODO Unsupported

  /**
   * Indicates the synchronous TX/RX mode. In this mode, the CC1101
   * sends a clock signal through one of the GPOx pins (In this
   * library, it is hardcoded to be GPO2). This signal is used to
   * synchronize data exchange between the MCU and the CC1101. In this
   * mode, the MCU should pull high or low the GPO0 pin when the clock
   * signal raises for transmitting data. In RX mode, MCU should
   * sample the value of any of the GPOx pins on each clock rise (on
   * this library, RX pin is also GPO0). The frequency of the clock is
   * equal to the configured data rate, except if the Manchester
   * encoding, in which the clock rate is half the data rate.
   *
   * More info on AN095 document:
   * https://www.ti.com/lit/an/swra359a/swra359a.pdf
   */
  CC1101_TRANS_MODE_SYNCHRONOUS = 1,

  /**
   * This mode is only used for testing purposes. In this mode, in TX
   * mode, the CC1101 will send random data. In RX mode, it will act
   * as the normal mode.
   */
  CC1101_TRANS_MODE_TEST_RANDOM = 2,

  /**
   * Indicates the asynchronous TX/RX mode. This mode works like the
   * synchronous mode, but without clock, which makes impossible to
   * get a synchronization between the MCU and the CC1101. This is a
   * legacy transmission mode and it is not recommended for
   * use. Normal or synchronous TX/RX mode are preferred.
   */
  CC1101_TRANS_MODE_ASYNCHRONOUS = 3,
} cc1101_trans_mode_t;

typedef union {
  uint8_t byte;
  struct {
    uint8_t fifo_bytes_available:4;
    cc1101_device_state_t state:3;
    bool ready:1;
  } fields;
} cc1101_chip_status_t;

typedef void (*cc1101_sync_mode_clk_cb)(const cc1101_device_t* device, void* user);
typedef struct {
  cc1101_sync_mode_clk_cb clk_cb;
  void* user;
} cc1101_sync_mode_cfg_t;

esp_err_t cc1101_init(const cc1101_device_cfg_t *cfg, cc1101_device_t** device);

esp_err_t cc1101_spi_tx(const cc1101_device_t *device, uint8_t cmd,
                        const uint8_t *in, uint8_t *out, size_t len);

esp_err_t cc1101_spi_tx_u8(const cc1101_device_t *device, uint8_t cmd,
                           uint8_t in, uint8_t *out);

esp_err_t cc1101_read_config_reg(const cc1101_device_t *device,
                                 cc1101_config_reg_t reg, uint8_t *value);

esp_err_t cc1101_read_status_reg(const cc1101_device_t *device,
                                 cc1101_status_reg_t reg, uint8_t *value);

esp_err_t cc1101_write_config_reg(const cc1101_device_t *device,
                                  cc1101_config_reg_t reg, uint8_t value);

esp_err_t cc1101_strobe(const cc1101_device_t *device, cc1101_strobe_t strobe,
                        cc1101_chip_status_t *chip_status);
esp_err_t cc1101_chip_status(const cc1101_device_t *device,
                             cc1101_chip_status_t *chip_status);

esp_err_t cc1101_hard_reset(const cc1101_device_t *device);

esp_err_t cc1101_write_burst(const cc1101_device_t *device,
                             cc1101_config_reg_t reg_begin,
                             const uint8_t *values, size_t len);
esp_err_t cc1101_read_burst(const cc1101_device_t *device,
                            cc1101_config_reg_t reg_begin, uint8_t *output,
                            size_t len);

esp_err_t cc1101_write_patable(const cc1101_device_t *device, const uint8_t data[CC1101_PATABLE_SIZE]);
esp_err_t cc1101_read_patable(const cc1101_device_t *device,
                              uint8_t data[CC1101_PATABLE_SIZE]);


esp_err_t cc1101_configure_sync_mode(const cc1101_device_t *device, const cc1101_sync_mode_cfg_t* cfg);
esp_err_t cc1101_trans_continuous_write(const cc1101_device_t *device, uint32_t level);
esp_err_t cc1101_trans_continuous_read(const cc1101_device_t *device, uint32_t* level);

esp_err_t cc1101_trans_mode_set(const cc1101_device_t *device);

esp_err_t cc1101_enable_tx(const cc1101_device_t *device, cc1101_trans_mode_t mode);
esp_err_t cc1101_enable_rx(const cc1101_device_t *device, cc1101_trans_mode_t mode);
esp_err_t cc1101_set_idle(const cc1101_device_t *device);
esp_err_t cc1101_calibrate(const cc1101_device_t *device);

esp_err_t cc1101_set_data_rate(const cc1101_device_t* device, uint32_t baud_rate);



#ifdef CONFIG_ENABLE_DEBUG_SYMBOLS
/**
 * Table that holds the names of the config registers
 */
extern const char* cc1101_config_reg_names[];
extern const char *cc1101_status_reg_names[];

esp_err_t cc1101_debug_print_regs(const cc1101_device_t *device);
const char *cc1101_device_state_str(cc1101_device_state_t state);
const char *cc1101_config_reg_name(cc1101_config_reg_t reg);
const char *cc1101_status_reg_name(cc1101_status_reg_t reg);
#endif // CONFIG_ENABLE_DEBUG_SYMBOLS
#endif // CC1101_H
