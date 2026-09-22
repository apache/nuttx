/****************************************************************************
 * boards/arm/imxrt/fmu-v6xrt/src/imxrt_flexspi_nor_flash.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http:
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "imxrt_flexspi_nor_flash.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

locate_data(".boot_hdr.conf")
const struct flexspi_nor_config_s g_flash_config =
{
  .mem_config =
  {
#if !defined(CONFIG_BOARD_BOOTLOADER_INVALID_FCB)
    .tag                 = FLEXSPI_CFG_BLK_TAG,
#else
    .tag                 = 0xffffffffl,
#endif
    .version             = FLEXSPI_CFG_BLK_VERSION,
    .read_sample_clksrc    = FLASH_READ_SAMPLE_CLK_LOOPBACK_INTERNELLY,
    .cs_hold_time          = 1,
    .cs_setup_time         = 1,
    .device_mode_cfg_enable = 1,
    .device_mode_type      = DEVICE_CONFIG_CMD_TYPE_GENERIC,
    .wait_time_cfg_commands = 1,
    .controller_misc_option =
    (1u << FLEXSPIMISC_OFFSET_SAFECONFIG_FREQ_EN),
    .device_type    = FLEXSPI_DEVICE_TYPE_SERIAL_NOR,
    .sflash_pad_type = SERIAL_FLASH_1PAD,
    .serial_clk_freq = FLEXSPI_SERIAL_CLKFREQ_30MHz,
    .sflash_a1size  = 64ul * 1024u * 1024u,
    .data_valid_time =
    {
      [0] = 0,
    },
    .busy_offset      = 0u,
    .busybit_polarity = 0u,
    .lookup_table =
    {
      /* Read Dedicated 3Byte Address Read(0x03), 24bit address */

      [0 + 0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x03,
          RADDR_SDR, FLEXSPI_1PAD, 0x18),
      [0 + 1] = FLEXSPI_LUT_SEQ(READ_SDR, FLEXSPI_1PAD, 0x04,
          STOP, FLEXSPI_1PAD, 0),
    },
  },
  .page_size           = 256u,
  .sector_size         = 4u * 1024u,
  .blocksize          = 64u * 1024u,
  .is_uniform_blocksize = false,
  .ipcmd_serial_clkfreq = 1,
  .serial_nor_type = 2,
  .reserve2[0] = 0x7008200,
};

const struct flexspi_nor_config_s g_flash_fast_config =
{
  .mem_config =
  {
    .tag                 = FLEXSPI_CFG_BLK_TAG,
    .version             = FLEXSPI_CFG_BLK_VERSION,
    .read_sample_clksrc    = FLASH_READ_SAMPLE_CLK_EXT_INPUT_FROM_DQSPAD,
    .cs_hold_time          = 1,
    .cs_setup_time         = 1,
    .device_mode_cfg_enable = 1,
    .device_mode_type      = DEVICE_CONFIG_CMD_TYPE_SPI2XPI,
    .wait_time_cfg_commands = 1,
    .device_mode_seq =
    {
      .seq_num   = 1,
      .seq_id    = 6, /* See Lookup table for more details */
      .reserved = 0,
    },
    .device_mode_arg = 2, /* Enable OPI DDR mode */
    .controller_misc_option =
    (1u << FLEXSPIMISC_OFFSET_SAFECONFIG_FREQ_EN) |
    (1u << FLEXSPIMISC_OFFSET_DDR_MODE_EN),
    .device_type    = FLEXSPI_DEVICE_TYPE_SERIAL_NOR,
    .sflash_pad_type = SERIAL_FLASH_8PADS,
    .serial_clk_freq = FLEXSPI_SERIAL_CLKFREQ_200MHz,
    .sflash_a1size  = 64ul * 1024u * 1024u,
    .data_valid_time =
    {
      [0] = 0,
    },
    .busy_offset      = 0u,
    .busybit_polarity = 0u,
    .lookup_table =
    {
      /* Read. Macronix wants 20 dummy cycles at 200MHz, and the
       * operand is 2N in DDR mode, hence 0x28.
       */

      [0 + 0] = FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0xee,
          CMD_DDR, FLEXSPI_8PAD, 0x11),
      [0 + 1] = FLEXSPI_LUT_SEQ(RADDR_DDR, FLEXSPI_8PAD, 0x20,
          DUMMY_DDR, FLEXSPI_8PAD, 0x28),
      [0 + 2] = FLEXSPI_LUT_SEQ(READ_DDR, FLEXSPI_8PAD, 0x04,
          STOP, FLEXSPI_1PAD, 0x00),

      /* Read status */

      [4 * 2 + 0] = FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x05,
          CMD_DDR, FLEXSPI_8PAD, 0xfa),
      [4 * 2 + 1] = FLEXSPI_LUT_SEQ(RADDR_DDR, FLEXSPI_8PAD, 0x20,
          DUMMY_DDR, FLEXSPI_8PAD, 0x04),
      [4 * 2 + 2] = FLEXSPI_LUT_SEQ(READ_DDR, FLEXSPI_8PAD, 0x04,
          STOP, FLEXSPI_1PAD, 0x00),

      /* Write enable SPI */

      [4 * 3 + 0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x06,
          STOP, FLEXSPI_1PAD, 0x00),

      /* Write enable OPI SPI */

      [4 * 4 + 0] = FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x06,
          CMD_DDR, FLEXSPI_8PAD, 0xf9),

      /* Erase sector */

      [4 * 5 + 0] = FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x21,
          CMD_DDR, FLEXSPI_8PAD, 0xde),
      [4 * 5 + 1] = FLEXSPI_LUT_SEQ(RADDR_DDR, FLEXSPI_8PAD, 0x20,
          STOP, FLEXSPI_1PAD, 0x00),

      /* Write Configuration Register 2 =01, Enable OPI DDR mode */

      [4 * 6 + 0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x72,
          CMD_SDR, FLEXSPI_1PAD, 0x00),
      [4 * 6 + 1] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x00,
          CMD_SDR, FLEXSPI_1PAD, 0x00),
      [4 * 6 + 2] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x00,
          WRITE_SDR, FLEXSPI_1PAD, 0x01),

      /* Page program */

      [4 * 9 + 0] = FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x12,
          CMD_DDR, FLEXSPI_8PAD, 0xed),
      [4 * 9 + 1] = FLEXSPI_LUT_SEQ(RADDR_DDR, FLEXSPI_8PAD, 0x20,
          WRITE_DDR, FLEXSPI_8PAD, 0x04),
    },
  },
  .page_size           = 256u,
  .sector_size         = 4u * 1024u,
  .blocksize          = 64u * 1024u,
  .is_uniform_blocksize = false,
  .ipcmd_serial_clkfreq = 1,
  .serial_nor_type = 2,
  .reserve2[0] = 0x7008200,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/
