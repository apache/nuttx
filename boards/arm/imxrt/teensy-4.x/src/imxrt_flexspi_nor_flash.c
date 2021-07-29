/****************************************************************************
 * boards/arm/imxrt/teensy-4.x/src/imxrt_flexspi_nor_flash.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "imxrt_flexspi_nor_flash.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

#if defined (CONFIG_TEENSY4_QSPI_FLASH)
locate_data(".boot_hdr.conf")
const struct flexspi_nor_config_s g_flash_config =
{
  .mem_config =
  {
    .tag = FLEXSPI_CFG_BLK_TAG,
    .version = FLEXSPI_CFG_BLK_VERSION,
    .read_sample_clksrc =  FLASH_READ_SAMPLE_CLK_LOOPBACK_FROM_SCKPAD,
    .cs_hold_time = 3u,
    .cs_setup_time = 3u,
    .column_address_width = 0u,
    .device_type = FLEXSPI_DEVICE_TYPE_SERIAL_NOR,
    .sflash_pad_type = SERIAL_FLASH_4PADS,
    .serial_clk_freq = FLEXSPI_SERIAL_CLKFREQ_60MHz,
    #ifdef CONFIG_TEENSY_40
    .sflash_a1size = 2u * 1024u * 1024u,
    #endif
    #ifdef CONFIG_TEENSY_41
    .sflash_a1size = 8u * 1024u * 1024u,
    #endif
    .data_valid_time =
    {
      0u, 0u
    },
    .lookup_table =
    {
      /* LUTs */

      FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0xeb,
                     RADDR_SDR, FLEXSPI_4PAD, 0x18),
      FLEXSPI_LUT_SEQ(DUMMY_SDR, FLEXSPI_4PAD, 0x06,
                      READ_SDR, FLEXSPI_4PAD, 0x04),
      0x00000000,
      0x00000000,

      FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x05,
                      READ_SDR, FLEXSPI_1PAD, 0x04),
      0x00000000,
      0x00000000,
      0x00000000,

      0x00000000,
      0x00000000,
      0x00000000,
      0x00000000,

      FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x01, STOP, FLEXSPI_1PAD, 0x0),
      0x00000000,
      0x00000000,
      0x00000000,

      0x00000000,
      0x00000000,
      0x00000000,
      0x00000000,

      FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x20,
                      RADDR_SDR, FLEXSPI_1PAD, 0x18),
      0x00000000,
      0x00000000,
      0x00000000,

      0x00000000,
      0x00000000,
      0x00000000,
      0x00000000,

      0x00000000,
      0x00000000,
      0x00000000,
      0x00000000,

      FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0xd8,
                      RADDR_SDR, FLEXSPI_1PAD, 0x18),
      0x00000000,
      0x00000000,
      0x00000000,

      FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x02,
                      RADDR_SDR, FLEXSPI_1PAD, 0x18),
      FLEXSPI_LUT_SEQ(WRITE_SDR, FLEXSPI_1PAD, 0x04,
                      STOP, FLEXSPI_1PAD, 0x0),
      0x00000000,
      0x00000000,

      0x00000000,
      0x00000000,
      0x00000000,
      0x00000000,

      FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x60, STOP, FLEXSPI_1PAD, 0x0),
    },
  },

  .page_size = 256u,
  .sector_size = 4u * 1024u,
  .blocksize = 64u * 1024u,
  .is_uniform_blocksize = false,
};
#else
#  error Boot Flash type not chosen!
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/