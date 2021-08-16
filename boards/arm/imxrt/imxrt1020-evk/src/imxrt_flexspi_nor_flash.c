/****************************************************************************
 * boards/arm/imxrt/imxrt1020-evk/src/imxrt_flexspi_nor_flash.c
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

/* This configuration is for an IS25LP064A.
 * There will be slight differences for other chips but it's not as painful
 * or scary as it looks. Just get the flash data sheet and work through it
 * slowly to make sure the codes match. You will get something minimal up
 * just using an 0x03 read opcode, and you can optimize for the actual flash
 * you've got from there.
 *
 * It's best to not use the QPI because then you lose the ability to
 * communicate directly with the chip *except* in QPI mode until you power
 * cycle, and the overhead for block reads is minimal (reduces 32784 clock
 * ticks to 32778 for a 4K block read)
 */

#if defined (CONFIG_IMXRT1020_EVK_HYPER_FLASH)
locate_data(".boot_hdr.conf")
const struct flexspi_nor_config_s g_flash_config =
{
  .mem_config                =
  {
    .tag                     = FLEXSPI_CFG_BLK_TAG,
    .version                 = FLEXSPI_CFG_BLK_VERSION,
    .read_sample_clksrc      = FLASH_READ_SAMPLE_CLK_EXT_INPUT_FROM_DQSPAD,
    .cs_hold_time            = 3u,
    .cs_setup_time           = 3u,
    .column_address_width    = 3u,

  /* Enable DDR mode, Word addassable,
   * Safe configuration, Differential clock
   */

    .controller_misc_option  = (1u << FLEXSPIMISC_OFFSET_DDR_MODE_EN) |
                        (1u << FLEXSPIMISC_OFFSET_WORD_ADDRESSABLE_EN) |
                        (1u << FLEXSPIMISC_OFFSET_SAFECONFIG_FREQ_EN) |
                        (1u << FLEXSPIMISC_OFFSET_DIFFCLKEN),
    .sflash_pad_type         = SERIAL_FLASH_8PADS,
    .serial_clk_freq         = FLEXSPI_SERIAL_CLKFREQ_133MHz,
    .sflash_a1size           = 64u * 1024u * 1024u,
    .data_valid_time         =
    {
      16u, 16u
    },
    .lookup_table            =
    {
      /* Read LUTs */

      FLEXSPI_LUT_SEQ(CMD_DDR,
                      FLEXSPI_8PAD,
                      0xa0,
                      RADDR_DDR,
                      FLEXSPI_8PAD,
                      0x18),
      FLEXSPI_LUT_SEQ(CADDR_DDR,
                      FLEXSPI_8PAD,
                      0x10,
                      DUMMY_DDR,
                      FLEXSPI_8PAD,
                      0x06),
      FLEXSPI_LUT_SEQ(READ_DDR,
                      FLEXSPI_8PAD,
                      0x04,
                      STOP,
                      FLEXSPI_1PAD,
                      0x0),
    },
  },
  .page_size                 = 512u,
  .sector_size               = 256u * 1024u,
  .blocksize                 = 256u * 1024u,
  .is_uniform_blocksize      = 1,
};
#elif defined (CONFIG_IMXRT1020_EVK_QSPI_FLASH)
locate_data(".boot_hdr.conf")
const struct flexspi_nor_config_s g_flash_config =
{
  .mem_config =
  {
    .tag                     = FLEXSPI_CFG_BLK_TAG,
    .version                 = FLEXSPI_CFG_BLK_VERSION,
    .read_sample_clksrc      = FLASH_READ_SAMPLE_CLK_LOOPBACK_INTERNELLY,
    .cs_hold_time            = 3u,
    .cs_setup_time           = 3u,
    .device_mode_cfg_enable  = true,
    .device_mode_seq.seq_num = 1,
    .device_mode_seq.seq_id  = 4,     /* These commands set the Quad bit */
    .device_mode_arg         = 0x40,  /* on the flash to drive 4 pins.   */
    .device_type             = FLEXSPI_DEVICE_TYPE_SERIAL_NOR,
    .sflash_pad_type         = SERIAL_FLASH_4PADS,
    .serial_clk_freq         = FLEXSPI_SERIAL_CLKFREQ_100MHz,
    .sflash_a1size           = 8u * 1024u * 1024u,
    .data_valid_time         =
    {
      16u, 16u
    },
    .lookup_table            =
    {
      /* 0 - Quad Input/output read sequence - with optimised XIP support */

      [0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0xeb, RADDR_SDR,
                            FLEXSPI_4PAD, 0x18),
      [1] = FLEXSPI_LUT_SEQ(MODE8_SDR, FLEXSPI_4PAD, 0xa0, DUMMY_SDR,
                            FLEXSPI_4PAD, 0x04),
      [2] = FLEXSPI_LUT_SEQ(READ_SDR, FLEXSPI_4PAD, 0x04, JMP_ON_CS, 0, 1),

      /* 1 - Read Status */

      [1 * 4] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x05, READ_SDR,
                                FLEXSPI_1PAD, 0x01),

      /* 3 - Write Enable */

      [3 * 4] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x06, STOP, 0, 0),

      /* 4 - Write status */

      [4 * 4] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x01, WRITE_SDR,
                                FLEXSPI_1PAD, 0x1),

      /* 5 - Erase Sector */

      [5 * 4] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0xd7, RADDR_SDR,
                                FLEXSPI_1PAD, 0x18),

      /* 9 - Page Program */

      [9 * 4] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x02, RADDR_SDR,
                                FLEXSPI_1PAD, 0x18),
      [9 * 4 + 1] = FLEXSPI_LUT_SEQ(WRITE_SDR, FLEXSPI_1PAD, 0x8, STOP,
                                    FLEXSPI_1PAD, 0x0),

      /* 11 - Chip Erase */

      [11 * 4] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0xc7, STOP,
                                 FLEXSPI_1PAD, 0x0),
    },
  },

  .page_size                 = 256u,
  .sector_size               = 4u * 1024u,
  .blocksize                 = 32u * 1024u,
  .is_uniform_blocksize      = false,
};
#else
# error Boot Flash type not chosen!
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/
