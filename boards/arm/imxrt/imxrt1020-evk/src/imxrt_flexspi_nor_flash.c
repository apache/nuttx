/****************************************************************************
 * boards/arm/imxrt/imxrt1020-evk/src/imxrt_flexspi_nor_flash.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Ivan Ucherdzhiev <ivanucherdjiev@gmail.com>
 *           Dave Marples <dave@marples.net>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*******************************************************************************
 * Included Files
 ******************************************************************************/

#include "imxrt_flexspi_nor_flash.h"

/*******************************************************************************
 * Public Data
 ******************************************************************************/

/* This configuration is for an IS25LP064A. There will be slight differences for
 * other chips but it's not as painful or scary as it looks. Just get the flash
 * data sheet and work through it slowly to make sure the codes match. You will
 * get something minimal up just using an 0x03 read opcode, and you can optimize
 * for the actual flash you've got from there.
 *
 * It's best to not use the QPI because then you lose the ability to communicate
 * directly with the chip *except* in QPI mode until you power cycle, and the
 * overhead for block reads is minimal (reduces 32784 clock ticks to 32778 for
 * a 4K block read)
 */

#if defined (CONFIG_IMXRT1020_EVK_HYPER_FLASH)
__attribute__((section(".boot_hdr.conf")))
const struct flexspi_nor_config_s g_flash_config =
{
  .mem_config                =
  {
    .tag                     = FLEXSPI_CFG_BLK_TAG,
    .version                 = FLEXSPI_CFG_BLK_VERSION,
    .read_sample_clksrc      = FLASH_READ_SAMPLE_CLK_EXTERNALINPUT_FROM_DQSPAD,
    .cs_hold_time            = 3u,
    .cs_setup_time           = 3u,
    .column_address_width    = 3u,

    /* Enable DDR mode, Word addassable, Safe configuration, Differential clock */

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

      FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0xa0, RADDR_DDR, FLEXSPI_8PAD, 0x18),
      FLEXSPI_LUT_SEQ(CADDR_DDR, FLEXSPI_8PAD, 0x10, DUMMY_DDR, FLEXSPI_8PAD, 0x06),
      FLEXSPI_LUT_SEQ(READ_DDR, FLEXSPI_8PAD, 0x04, STOP, FLEXSPI_1PAD, 0x0),
    },
  },
  .page_size                 = 512u,
  .sector_size               = 256u * 1024u,
  .blocksize                 = 256u * 1024u,
  .is_uniform_blocksize      = 1,
};
#elif defined (CONFIG_IMXRT1020_EVK_QSPI_FLASH)
__attribute__((section(".boot_hdr.conf")))
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
