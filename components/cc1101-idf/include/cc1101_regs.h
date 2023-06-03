#ifndef CC1101_REGS_H
#define CC1101_REGS_H

#include <stdint.h>
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
  // TODO This list is incomplete. Just added a few entries for now
  CC1101_GDOX_CFG_SERIAL_CLOCK = 0x0b,
  CC1101_GDOX_CFG_SERIAL_SYNCHRONOUS_DATA_OUTPUT = 0x0c,
  CC1101_GDOX_CFG_SERIAL_DATA_OUTPUT = 0x0d,
  CC1101_GDOX_CFG_CARRIER_SENSE = 0x0e,
  CC1101_GDOX_CFG_CRC_OK = 0x0f,
} cc1101_gdox_cfg_t;

typedef enum __attribute__((packed)) {
  CC1101_PKT_FORMAT_NORMAL = 0,
  CC1101_PKT_FORMAT_SYNCHRONOUS = 1,
  CC1101_PKT_FORMAT_RANDOM_TX_MODE = 2,
  CC1101_PKT_FORMAT_ASYNCHRONOUS = 3,
} cc1101_pkt_format_t;

typedef enum __attribute__((packed)) {
  CC1101_LENGTH_CONFIG_FIXED = 0,
  CC1101_LENGTH_CONFIG_VARIABLE = 1,
  CC1101_LENGTH_CONFIG_INFINITE = 2,
  CC1101_LENGTH_CONFIG_RESERVED = 3,
} cc1101_length_config_t;

typedef union __attribute__((packed)) {
  uint8_t value;
  struct {
    cc1101_gdox_cfg_t gdo2_cfg:6;
    uint8_t gdo2_inv:1;
    uint8_t unused:1;
  } fields;
} cc1101_reg_iocfg2_t;

typedef union __attribute__((packed)) {
  uint8_t value;
  struct {
    cc1101_gdox_cfg_t gdo1_cfg:6;
    uint8_t gdo1_inv:1;
    uint8_t unused:1;
  } fields;
} cc1101_reg_iocfg1_t;

typedef union __attribute__((packed)) {
  uint8_t value;
  struct {
    cc1101_gdox_cfg_t gdo0_cfg:6;
    uint8_t gdo0_inv:1;
    uint8_t temp_sensor_enable:1;
  } fields;
} cc1101_reg_iocfg0_t;

typedef union __attribute__((packed)) {
  uint8_t value;
  struct {
    cc1101_length_config_t length_config:2;
    uint8_t crc_en:1;
    uint8_t unused_1:1;
    cc1101_pkt_format_t pkt_format:2;
    uint8_t white_data:1;
    uint8_t unused_2:1;
  } fields;
} cc1101_reg_pktctrl0_t;
#endif // CC1101_REGS_H
