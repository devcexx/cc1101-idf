#include "driver/spi_master.h"
#include "hal/gpio_types.h"
#include "esp_err.h"
#include <stdint.h>

#define CC1101_SCLK_HZ 100000
#define CC1101_SPI_QUEUE_SIZE 10
#define CC1101_SPI_BEGIN_TIMEOUT_US 100

typedef enum __attribute__((packed)) {
  CC1101_REG_CFG_IOCFG2 = 0x00,   // GDO2 output pin configuration
  CC1101_REG_CFG_IOCFG1 = 0x01,   // GDO1 output pin configuration
  CC1101_REG_CFG_IOCFG0 = 0x02,   // GDO0 output pin configuration
  CC1101_REG_CFG_FIFOTHR = 0x03,  // RX FIFO and TX FIFO thresholds
  CC1101_REG_CFG_SYNC1 = 0x04,    // Sync word, high INT8U
  CC1101_REG_CFG_SYNC0 = 0x05,    // Sync word, low INT8U
  CC1101_REG_CFG_PKTLEN = 0x06,   // Packet length
  CC1101_REG_CFG_PKTCTRL1 = 0x07, // Packet automation control
  CC1101_REG_CFG_PKTCTRL0 = 0x08, // Packet automation control
  CC1101_REG_CFG_ADDR = 0x09,     // Device address
  CC1101_REG_CFG_CHANNR = 0x0A,   // Channel number
  CC1101_REG_CFG_FSCTRL1 = 0x0B,  // Frequency synthesizer control
  CC1101_REG_CFG_FSCTRL0 = 0x0C,  // Frequency synthesizer control
  CC1101_REG_CFG_FREQ2 = 0x0D,    // Frequency control word, high INT8U
  CC1101_REG_CFG_FREQ1 = 0x0E,    // Frequency control word, middle INT8U
  CC1101_REG_CFG_FREQ0 = 0x0F,    // Frequency control word, low INT8U
  CC1101_REG_CFG_MDMCFG4 = 0x10,  // Modem configuration
  CC1101_REG_CFG_MDMCFG3 = 0x11,  // Modem configuration
  CC1101_REG_CFG_MDMCFG2 = 0x12,  // Modem configuration
  CC1101_REG_CFG_MDMCFG1 = 0x13,  // Modem configuration
  CC1101_REG_CFG_MDMCFG0 = 0x14,  // Modem configuration
  CC1101_REG_CFG_DEVIATN = 0x15,  // Modem deviation setting
  CC1101_REG_CFG_MCSM2 = 0x16,    // Main Radio Control State Machine configuration
  CC1101_REG_CFG_MCSM1 = 0x17,    // Main Radio Control State Machine configuration
  CC1101_REG_CFG_MCSM0 = 0x18,    // Main Radio Control State Machine configuration
  CC1101_REG_CFG_FOCCFG = 0x19,   // Frequency Offset Compensation configuration
  CC1101_REG_CFG_BSCFG = 0x1A,    // Bit Synchronization configuration
  CC1101_REG_CFG_AGCCTRL2 = 0x1B, // AGC control
  CC1101_REG_CFG_AGCCTRL1 = 0x1C, // AGC control
  CC1101_REG_CFG_AGCCTRL0 = 0x1D, // AGC control
  CC1101_REG_CFG_WOREVT1 = 0x1E,  // High INT8U Event 0 timeout
  CC1101_REG_CFG_WOREVT0 = 0x1F,  // Low INT8U Event 0 timeout
  CC1101_REG_CFG_WORCTRL = 0x20,  // Wake On Radio control
  CC1101_REG_CFG_FREND1 = 0x21,   // Front end RX configuration
  CC1101_REG_CFG_FREND0 = 0x22,   // Front end TX configuration
  CC1101_REG_CFG_FSCAL3 = 0x23,   // Frequency synthesizer calibration
  CC1101_REG_CFG_FSCAL2 = 0x24,   // Frequency synthesizer calibration
  CC1101_REG_CFG_FSCAL1 = 0x25,   // Frequency synthesizer calibration
  CC1101_REG_CFG_FSCAL0 = 0x26,   // Frequency synthesizer calibration
  CC1101_REG_CFG_RCCTRL1 = 0x27,  // RC oscillator configuration
  CC1101_REG_CFG_RCCTRL0 = 0x28,  // RC oscillator configuration
  CC1101_REG_CFG_FSTEST = 0x29,   // Frequency synthesizer calibration control
  CC1101_REG_CFG_PTEST = 0x2A,    // Production test
  CC1101_REG_CFG_AGCTEST = 0x2B,  // AGC test
  CC1101_REG_CFG_TEST2 = 0x2C,    // Various test settings
  CC1101_REG_CFG_TEST1 = 0x2D,    // Various test settings
  CC1101_REG_CFG_TEST0 = 0x2E     // Various test settings
} cc1101_config_reg_t;

#define CC1101_FIRST_CFG_REG CC1101_REG_CFG_IOCFG2
#define CC1101_LAST_CFG_REG CC1101_REG_CFG_TEST0
#define CC1101_CONFIG_REG_COUNT (CC1101_LAST_CFG_REG - CC1101_FIRST_CFG_REG + 1)

typedef enum __attribute__((packed)) {
  CC1101_REG_STATUS_PARTNUM = 0x30,
  CC1101_REG_STATUS_VERSION = 0x31,
  CC1101_REG_STATUS_FREQEST = 0x32,
  CC1101_REG_STATUS_LQI = 0x33,
  CC1101_REG_STATUS_RSSI = 0x34,
  CC1101_REG_STATUS_MARCSTATE = 0x35,
  CC1101_REG_STATUS_WORTIME1 = 0x36,
  CC1101_REG_STATUS_WORTIME0 = 0x37,
  CC1101_REG_STATUS_PKTSTATUS = 0x38,
  CC1101_REG_STATUS_VCO_VC_DAC = 0x39,
  CC1101_REG_STATUS_TXBYTES = 0x3A,
  CC1101_REG_STATUS_RXBYTES = 0x3B
} cc1101_status_reg_t;

#define CC1101_FIRST_STATUS_REG CC1101_REG_STATUS_PARTNUM
#define CC1101_LAST_STATUS_REG CC1101_REG_STATUS_RXBYTES
#define CC1101_STATUS_REG_COUNT (CC1101_LAST_STATUS_REG - CC1101_FIRST_STATUS_REG + 1)

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
  spi_device_handle_t spi_device;

  gpio_num_t miso_io_num;
  gpio_num_t gdo0_io_num;
  gpio_num_t gdo2_io_num;
  gpio_num_t cs_io_num;
} cc1101_device_t;

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

typedef union {
  uint8_t byte;
  struct {
    uint8_t fifo_bytes_available:4;
    cc1101_device_state_t state:3;
    bool ready:1;
  } fields;
} cc1101_chip_status_t;

esp_err_t cc1101_init(spi_host_device_t spi_host, cc1101_device_t *device);

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
