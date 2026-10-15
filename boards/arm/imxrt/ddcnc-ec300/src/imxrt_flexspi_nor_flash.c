/****************************************************************************
 * boards/arm/imxrt/ddcnc-ec300/src/imxrt_flexspi_nor_flash.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "imxrt_flexspi_nor_flash.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* FlexSPI NOR configuration block used by the i.MX RT ROM bootloader.
 *
 * The EC300 boots from a 4MiB Quad SPI NOR flash (Winbond W25Q32 style
 * command set) connected to FlexSPI port A1.  The LUT sequence indices
 * follow the layout expected by the ROM:
 *
 *   0: Quad I/O Fast Read (0xeb)
 *   1: Read Status Register (0x05)
 *   3: Write Enable (0x06)
 *   5: Sector Erase 4KiB (0x20)
 *   8: Block Erase 64KiB (0xd8)
 *   9: Page Program (0x02)
 *  11: Chip Erase (0x60)
 */

locate_data(".boot_hdr.conf")
const struct flexspi_nor_config_s g_flash_config =
{
  .mem_config =
  {
    .tag                    = FLEXSPI_CFG_BLK_TAG,
    .version                = FLEXSPI_CFG_BLK_VERSION,
    .read_sample_clksrc     = FLASH_READ_SAMPLE_CLK_LOOPBACK_FROM_DQSPAD,
    .cs_hold_time           = 3u,
    .cs_setup_time          = 3u,
    .controller_misc_option =
                      (1u << FLEXSPIMISC_OFFSET_SAFECONFIG_FREQ_EN),
    .device_type            = FLEXSPI_DEVICE_TYPE_SERIAL_NOR,
    .sflash_pad_type        = SERIAL_FLASH_4PADS,
    .serial_clk_freq        = FLEXSPI_SERIAL_CLKFREQ_133MHz,
    .sflash_a1size          = 4u * 1024u * 1024u,
    .lookup_table           =
    {
      /* 0 Quad I/O Fast Read */

      FLEXSPI_LUT_SEQ(CMD_SDR,
                      FLEXSPI_1PAD, 0xeb, RADDR_SDR, FLEXSPI_4PAD, 0x18),
      FLEXSPI_LUT_SEQ(DUMMY_SDR,
                      FLEXSPI_4PAD, 0x06, READ_SDR, FLEXSPI_4PAD, 0x04),
      0x00000000,
      0x00000000,

      /* 1 Read Status */

      FLEXSPI_LUT_SEQ(CMD_SDR,
                      FLEXSPI_1PAD, 0x05, READ_SDR, FLEXSPI_1PAD, 0x04),
      0x00000000,
      0x00000000,
      0x00000000,

      /* 2 */

      0x00000000,
      0x00000000,
      0x00000000,
      0x00000000,

      /* 3 Write Enable */

      FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x06, STOP, FLEXSPI_1PAD, 0x0),
      0x00000000,
      0x00000000,
      0x00000000,

      /* 4 */

      0x00000000,
      0x00000000,
      0x00000000,
      0x00000000,

      /* 5 Erase Sector (4KiB) */

      FLEXSPI_LUT_SEQ(CMD_SDR,
                      FLEXSPI_1PAD, 0x20, RADDR_SDR, FLEXSPI_1PAD, 0x18),
      0x00000000,
      0x00000000,
      0x00000000,

      /* 6 */

      0x00000000,
      0x00000000,
      0x00000000,
      0x00000000,

      /* 7 */

      0x00000000,
      0x00000000,
      0x00000000,
      0x00000000,

      /* 8 Erase Block (64KiB) */

      FLEXSPI_LUT_SEQ(CMD_SDR,
                      FLEXSPI_1PAD, 0xd8, RADDR_SDR, FLEXSPI_1PAD, 0x18),
      0x00000000,
      0x00000000,
      0x00000000,

      /* 9 Page Program */

      FLEXSPI_LUT_SEQ(CMD_SDR,
                      FLEXSPI_1PAD, 0x02, RADDR_SDR, FLEXSPI_1PAD, 0x18),
      FLEXSPI_LUT_SEQ(WRITE_SDR,
                      FLEXSPI_1PAD, 0x04, STOP, FLEXSPI_1PAD, 0x0),
      0x00000000,
      0x00000000,

      /* 10 */

      0x00000000,
      0x00000000,
      0x00000000,
      0x00000000,

      /* 11 Chip Erase */

      FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x60, STOP, FLEXSPI_1PAD, 0x0),
      0x00000000,
      0x00000000,
      0x00000000,
    },
  },

  .page_size                = 256u,
  .sector_size              = 4u * 1024u,
  .ipcmd_serial_clkfreq     = 1u,
  .is_uniform_blocksize     = false,
  .blocksize                = 64u * 1024u,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/
