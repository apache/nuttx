/****************************************************************************
 * arch/arm/include/imxrt/imxrt1189_romapi.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_INCLUDE_IMXRT_IMXRT1189_ROMAPI_H
#define __ARCH_ARM_INCLUDE_IMXRT_IMXRT1189_ROMAPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stdint.h>

typedef int32_t status_t;

#ifndef kStatus_Success
#  define kStatus_Success ((status_t)0)
#endif

#ifndef kStatus_InvalidArgument
#  define kStatus_InvalidArgument ((status_t)4)
#endif

#define FSL_ROM_HAS_FLEXSPINOR_API 1
#define FSL_ROM_HAS_RUNBOOTLOADER_API 1
#define FSL_ROM_FLEXSPINOR_API_HAS_FEATURE_GET_CONFIG 1
#define FSL_ROM_FLEXSPINOR_API_HAS_FEATURE_FLASH_INIT 1
#define FSL_ROM_FLEXSPINOR_API_HAS_FEATURE_ERASE 1
#define FSL_ROM_FLEXSPINOR_API_HAS_FEATURE_ERASE_SECTOR 1
#define FSL_ROM_FLEXSPINOR_API_HAS_FEATURE_ERASE_BLOCK 1
#define FSL_ROM_FLEXSPINOR_API_HAS_FEATURE_ERASE_ALL 1
#define FSL_ROM_FLEXSPINOR_API_HAS_FEATURE_READ 1
#define FSL_ROM_FLEXSPINOR_API_HAS_FEATURE_UPDATE_LUT 1
#define FSL_ROM_FLEXSPINOR_API_HAS_FEATURE_CMD_XFER 1

#define FSL_ROM_ROMAPI_VERSION              0x00010103u
#define FSL_ROM_FLEXSPINOR_DRIVER_VERSION  0x00010700u

#define FLEXSPI_CFG_BLK_TAG     0x42464346UL
#define FLEXSPI_CFG_BLK_VERSION 0x56010400UL

typedef struct
{
  union
  {
    struct
    {
      uint32_t max_freq : 4;
      uint32_t misc_mode : 4;
      uint32_t quad_mode_setting : 4;
      uint32_t cmd_pads : 4;
      uint32_t query_pads : 4;
      uint32_t device_type : 4;
      uint32_t option_size : 4;
      uint32_t tag : 4;
    } B;
    uint32_t U;
  } option0;

  union
  {
    struct
    {
      uint32_t dummy_cycles : 8;
      uint32_t status_override : 8;
      uint32_t pinmux_group : 4;
      uint32_t dqs_pinmux_group : 4;
      uint32_t drive_strength : 4;
      uint32_t flash_connection : 4;
    } B;
    uint32_t U;
  } option1;
} serial_nor_config_option_t;

typedef struct
{
  uint8_t seq_num;
  uint8_t seq_id;
  uint16_t reserved;
} flexspi_lut_seq_t;

typedef struct
{
  uint8_t time_100ps;
  uint8_t delay_cells;
} flexspi_dll_time_t;

typedef struct
{
  uint32_t tag;
  uint32_t version;
  uint32_t reserved0;
  uint8_t read_sample_clk_src;
  uint8_t cs_hold_time;
  uint8_t cs_setup_time;
  uint8_t column_address_width;
  uint8_t device_mode_cfg_enable;
  uint8_t device_mode_type;
  uint16_t wait_time_cfg_commands;
  flexspi_lut_seq_t device_mode_seq;
  uint32_t device_mode_arg;
  uint8_t config_cmd_enable;
  uint8_t config_mode_type[3];
  flexspi_lut_seq_t config_cmd_seqs[3];
  uint32_t reserved1;
  uint32_t config_cmd_args[3];
  uint32_t reserved2;
  uint32_t controller_misc_option;
  uint8_t device_type;
  uint8_t sflash_pad_type;
  uint8_t serial_clk_freq;
  uint8_t lut_custom_seq_enable;
  uint32_t reserved3[2];
  uint32_t sflash_a1_size;
  uint32_t sflash_a2_size;
  uint32_t sflash_b1_size;
  uint32_t sflash_b2_size;
  uint32_t cs_pad_setting_override;
  uint32_t sclk_pad_setting_override;
  uint32_t data_pad_setting_override;
  uint32_t dqs_pad_setting_override;
  uint32_t timeout_in_ms;
  uint32_t command_interval;
  flexspi_dll_time_t data_valid_time[2];
  uint16_t busy_offset;
  uint16_t busy_bit_polarity;
  uint32_t lookup_table[64];
  flexspi_lut_seq_t lut_custom_seq[12];
  uint32_t reserved4[4];
} flexspi_mem_config_t;

typedef struct
{
  flexspi_mem_config_t mem_config;
  uint32_t page_size;
  uint32_t sector_size;
  uint8_t ipcmd_serial_clk_freq;
  uint8_t is_uniform_block_size;
  uint8_t is_data_order_swapped;
  uint8_t reserved0;
  uint8_t serial_nor_type;
  uint8_t need_exit_no_cmd_mode;
  uint8_t half_clk_for_non_read_cmd;
  uint8_t need_restore_no_cmd_mode;
  uint32_t block_size;
  uint32_t reserve2[11];
} flexspi_nor_config_t;

typedef enum
{
  flexspi_operation_command,
  flexspi_operation_config,
  flexspi_operation_write,
  flexspi_operation_read
} flexspi_operation_t;

typedef struct
{
  flexspi_operation_t operation;
  uint32_t base_address;
  uint32_t seq_id;
  uint32_t seq_num;
  bool is_parallel_mode_enable;
  uint32_t *tx_buffer;
  uint32_t tx_size;
  uint32_t *rx_buffer;
  uint32_t rx_size;
} flexspi_xfer_t;

/* ROM API entry points.  The RT1189 adapter uses the function table
 * directly, but these declarations keep the public ROM interface
 * available to NuttX.
 */

status_t rom_flexspi_nor_flash_get_config(uint32_t instance,
                                          flexspi_nor_config_t *config,
                                          serial_nor_config_option_t
                                          *option);
status_t rom_flexspi_nor_flash_init(uint32_t instance,
                                    flexspi_nor_config_t *config);
status_t rom_flexspi_nor_flash_program_page(uint32_t instance,
                                            flexspi_nor_config_t *config,
                                            uint32_t address,
                                            const uint32_t *src);
status_t rom_flexspi_nor_flash_read(uint32_t instance,
                                    flexspi_nor_config_t *config,
                                    uint32_t *dst, uint32_t address,
                                    uint32_t size);
status_t rom_flexspi_nor_flash_erase(uint32_t instance,
                                     flexspi_nor_config_t *config,
                                     uint32_t address, uint32_t size);
status_t rom_flexspi_nor_flash_erase_sector(uint32_t instance,
                                            flexspi_nor_config_t *config,
                                            uint32_t address);
status_t rom_flexspi_nor_flash_erase_block(uint32_t instance,
                                           flexspi_nor_config_t *config,
                                           uint32_t address);
status_t rom_flexspi_nor_flash_erase_all(uint32_t instance,
                                         flexspi_nor_config_t *config);
status_t rom_flexspi_nor_flash_command_xfer(uint32_t instance,
                                             flexspi_xfer_t *xfer);

#endif /* __ARCH_ARM_INCLUDE_IMXRT_IMXRT1189_ROMAPI_H */
