/****************************************************************************
 * boards/arm/imxrt/frdm-imxrt1186/src/imxrt_flexspi_nor_flash.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __BOARDS_ARM_IMXRT_FRDM_IMXRT1186_SRC_IMXRT_FLEXSPI_NOR_FLASH_H
#define __BOARDS_ARM_IMXRT_FRDM_IMXRT1186_SRC_IMXRT_FLEXSPI_NOR_FLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FLEXSPI_CFG_BLK_TAG          0x42464346
#define FLEXSPI_CFG_BLK_VERSION      0x56010400

#define CMD_SDR                      0x01
#define RADDR_SDR                    0x02
#define WRITE_SDR                    0x08
#define READ_SDR                     0x09
#define DUMMY_SDR                    0x0c
#define STOP                         0

#define FLEXSPI_1PAD                 0
#define FLEXSPI_4PAD                 2

#define FLEXSPI_LUT_OPERAND0(x)      ((uint32_t)(x))
#define FLEXSPI_LUT_NUM_PADS0(x)     ((uint32_t)(x) << 8)
#define FLEXSPI_LUT_OPCODE0(x)       ((uint32_t)(x) << 10)
#define FLEXSPI_LUT_OPERAND1(x)      ((uint32_t)(x) << 16)
#define FLEXSPI_LUT_NUM_PADS1(x)     ((uint32_t)(x) << 24)
#define FLEXSPI_LUT_OPCODE1(x)       ((uint32_t)(x) << 26)

#define FLEXSPI_LUT_SEQ(cmd0, pad0, op0, cmd1, pad1, op1) \
  (FLEXSPI_LUT_OPERAND0(op0) | FLEXSPI_LUT_NUM_PADS0(pad0) | \
   FLEXSPI_LUT_OPCODE0(cmd0) | FLEXSPI_LUT_OPERAND1(op1) | \
   FLEXSPI_LUT_NUM_PADS1(pad1) | FLEXSPI_LUT_OPCODE1(cmd1))

#define FLASH_READ_SAMPLE_CLK_DQSPAD 1
#define FLEXSPI_DEVICE_SERIAL_NOR    1
#define SERIAL_FLASH_4PADS           4
#define FLEXSPI_SERIAL_CLK_100MHZ    5
#define DEVICE_CONFIG_CMD_GENERIC    0

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct flexspi_lut_sequence_s
{
  uint8_t sequence_count;
  uint8_t sequence_index;
  uint16_t reserved;
};

struct flexspi_memory_config_s
{
  uint32_t tag;
  uint32_t version;
  uint32_t reserved0;
  uint8_t read_sample_clock_source;
  uint8_t chip_select_hold_time;
  uint8_t chip_select_setup_time;
  uint8_t column_address_width;
  uint8_t device_mode_config_enable;
  uint8_t device_mode_type;
  uint16_t wait_time_config_commands;
  struct flexspi_lut_sequence_s device_mode_sequence;
  uint32_t device_mode_argument;
  uint8_t config_command_enable;
  uint8_t config_mode_type[3];
  struct flexspi_lut_sequence_s config_command_sequences[3];
  uint32_t reserved1;
  uint32_t config_command_arguments[3];
  uint32_t reserved2;
  uint32_t controller_misc_option;
  uint8_t device_type;
  uint8_t serial_flash_pad_type;
  uint8_t serial_clock_frequency;
  uint8_t custom_lut_enable;
  uint32_t reserved3[2];
  uint32_t serial_flash_a1_size;
  uint32_t serial_flash_a2_size;
  uint32_t serial_flash_b1_size;
  uint32_t serial_flash_b2_size;
  uint32_t chip_select_pad_override;
  uint32_t clock_pad_override;
  uint32_t data_pad_override;
  uint32_t dqs_pad_override;
  uint32_t timeout_ms;
  uint32_t command_interval;
  uint16_t data_valid_time[2];
  uint16_t busy_offset;
  uint16_t busy_bit_polarity;
  uint32_t lookup_table[64];
  struct flexspi_lut_sequence_s custom_lut_sequences[12];
  uint32_t reserved4[4];
};

struct flexspi_nor_config_s
{
  struct flexspi_memory_config_s memory_config;
  uint32_t page_size;
  uint32_t sector_size;
  uint8_t ip_command_serial_clock_frequency;
  uint8_t uniform_block_size;
  uint8_t data_order_swapped;
  uint8_t reserved0[5];
  uint32_t block_size;
  uint32_t flash_state_context;
  uint32_t reserved1[10];
};

#endif /* __BOARDS_ARM_IMXRT_FRDM_IMXRT1186_SRC_IMXRT_FLEXSPI_NOR_FLASH_H */
