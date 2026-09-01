/****************************************************************************
 * boards/arm/imxrt/imxrt1180-evk/src/imxrt_flash_headers.c
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

/* Flash headers baked into the M33 boot image at the offsets the RT1180
 * ROM expects (IMXRT1180RM Ch. 12 Fig. 50):
 *
 *   0x0000_0400  Serial NOR Flash Configuration Block (FCB, 512 B)
 *
 * The AHAB container is NOT baked into this object file.  It is added
 * later by tools/imxrt1180/build_flash_image.sh, which invokes NXP's
 * SPSDK "nxpimage" tool to assemble:
 *
 *   0x0000_?000  Application AHAB container (this M33 image)
 *
 * The Cortex-M7 image is flashed separately at offset 0x0008_0000 and is
 * a raw XIP payload with no flash headers of its own.
 *
 * The M7 payload is not wrapped in an AHAB container: the M33 bootloader
 * ("bl" configuration) points the M7 at raw flash offset 0x80000 (XIP
 * address 0x28080000) and releases it by clearing M7_CFG.WAIT.
 *
 * The board fits a Winbond W25Q128JWSIQ (16 MB, 1.8 V QSPI) on
 * FlexSPI1 Port A (verified in the MIMXRT1180-EVK schematic, ref U12).
 */

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* FlexSPI Configuration Block (FCB) */

#define FLEXSPI_LUT_OPCODE0(x)    ((uint32_t)((x) & 0x3f) << 10)
#define FLEXSPI_LUT_NUM_PADS0(x)  ((uint32_t)((x) & 0x03) << 8)
#define FLEXSPI_LUT_OPERAND0(x)   ((uint32_t)((x) & 0xff))
#define FLEXSPI_LUT_OPCODE1(x)    ((uint32_t)((x) & 0x3f) << 26)
#define FLEXSPI_LUT_NUM_PADS1(x)  ((uint32_t)((x) & 0x03) << 24)
#define FLEXSPI_LUT_OPERAND1(x)   ((uint32_t)((x) & 0xff) << 16)

#define FLEXSPI_LUT_SEQ(c0, p0, o0, c1, p1, o1) \
    (FLEXSPI_LUT_OPERAND0(o0) | FLEXSPI_LUT_NUM_PADS0(p0) | \
     FLEXSPI_LUT_OPCODE0(c0)  | FLEXSPI_LUT_OPERAND1(o1) | \
     FLEXSPI_LUT_NUM_PADS1(p1) | FLEXSPI_LUT_OPCODE1(c1))

#define CMD_SDR    0x01
#define RADDR_SDR  0x02
#define DUMMY_SDR  0x0c
#define READ_SDR   0x09
#define WRITE_SDR  0x08
#define STOP       0x00

#define FLEXSPI_1PAD 0
#define FLEXSPI_4PAD 2

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Layout matches IMXRT1180RM Ch. 12 Table 62/65 (identical to the RT1170
 * FCB layout so we can reuse the flexspi_nor_config_s / flexspi_mem_config_s
 * struct definitions from the imxrt1170-evk board without change).
 */

struct flexspi_lut_seq_s
{
  uint8_t seq_num;
  uint8_t seq_id;
  uint16_t reserved;
};

struct flexspi_mem_config_s
{
  uint32_t tag;
  uint32_t version;
  uint32_t reserved0;
  uint8_t  read_sample_clk_src;
  uint8_t  cs_hold_time;
  uint8_t  cs_setup_time;
  uint8_t  column_address_width;
  uint8_t  device_mode_cfg_enable;
  uint8_t  device_mode_type;
  uint16_t wait_time_cfg_commands;
  struct flexspi_lut_seq_s device_mode_seq;
  uint32_t device_mode_arg;
  uint8_t  config_cmd_enable;
  uint8_t  config_mode_type[3];
  struct flexspi_lut_seq_s config_cmd_seqs[3];
  uint32_t reserved1;
  uint32_t config_cmd_args[3];
  uint32_t reserved2;
  uint32_t controller_misc_option;
  uint8_t  device_type;
  uint8_t  sflash_pad_type;
  uint8_t  serial_clk_freq;
  uint8_t  lut_custom_seq_enable;
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
  uint16_t data_valid_time[2];
  uint16_t busy_offset;
  uint16_t busy_bit_polarity;
  uint32_t lookup_table[64];
  struct flexspi_lut_seq_s lut_custom_seq[12];
  uint32_t reserved4[4];
};

struct flexspi_nor_config_s
{
  struct flexspi_mem_config_s mem_config;
  uint32_t page_size;
  uint32_t sector_size;
  uint8_t  ipcmd_serial_clk_freq;
  uint8_t  is_uniform_block_size;
  uint8_t  reserved0[2];
  uint32_t block_size;
  uint32_t reserved1[11];
};

#define FCB_TAG        0x42464346u  /* 'FCFB' little endian */
#define FCB_VERSION    0x56010400u  /* v1.4.0                  */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Winbond W25Q128JW: 16 MB, 4 KB sector, 64 KB block, 256 B page.
 * QSPI @ 133 MHz, DQS loopback, LUT provides 0xEB Quad I/O Read (24-bit
 * address, 6 dummy cycles) — matches the MCUXpresso SDK reference config
 * for evkmimxrt1180 (evkmimxrt1180_flexspi_nor_config.c).
 */

__attribute__((section(".boot_hdr.conf"), used))
const struct flexspi_nor_config_s g_flash_config =
{
  .mem_config =
  {
    .tag                    = FCB_TAG,
    .version                = FCB_VERSION,
    .read_sample_clk_src    = 1,   /* Loopback from DQS pad */
    .cs_hold_time           = 3,
    .cs_setup_time          = 3,
    .controller_misc_option = 0x10,
    .device_type            = 1,   /* Serial NOR                */
    .sflash_pad_type        = 4,   /* 4 pads (QSPI)             */
    .serial_clk_freq        = 7,   /* 133 MHz                   */
    .sflash_a1_size         = 16u * 1024u * 1024u,
    .lookup_table =
    {
      [0]      = FLEXSPI_LUT_SEQ(CMD_SDR,   FLEXSPI_1PAD, 0xeb,
                                 RADDR_SDR, FLEXSPI_4PAD, 0x18),
      [1]      = FLEXSPI_LUT_SEQ(DUMMY_SDR, FLEXSPI_4PAD, 0x06,
                                 READ_SDR,  FLEXSPI_4PAD, 0x04),
      [4 * 1]  = FLEXSPI_LUT_SEQ(CMD_SDR,   FLEXSPI_1PAD, 0x05,
                                 READ_SDR,  FLEXSPI_1PAD, 0x04),
      [4 * 3]  = FLEXSPI_LUT_SEQ(CMD_SDR,   FLEXSPI_1PAD, 0x06,
                                 STOP,      FLEXSPI_1PAD, 0x00),
      [4 * 5]  = FLEXSPI_LUT_SEQ(CMD_SDR,   FLEXSPI_1PAD, 0x20,
                                 RADDR_SDR, FLEXSPI_1PAD, 0x18),
      [4 * 8]  = FLEXSPI_LUT_SEQ(CMD_SDR,   FLEXSPI_1PAD, 0xd8,
                                 RADDR_SDR, FLEXSPI_1PAD, 0x18),
      [4 * 9]  = FLEXSPI_LUT_SEQ(CMD_SDR,   FLEXSPI_1PAD, 0x02,
                                 RADDR_SDR, FLEXSPI_1PAD, 0x18),
      [4 * 9 + 1] = FLEXSPI_LUT_SEQ(WRITE_SDR, FLEXSPI_1PAD, 0x04,
                                    STOP,      FLEXSPI_1PAD, 0x00),
      [4 * 11] = FLEXSPI_LUT_SEQ(CMD_SDR,   FLEXSPI_1PAD, 0x60,
                                 STOP,      FLEXSPI_1PAD, 0x00),
    },
  },
  .page_size            = 256u,
  .sector_size          = 4u * 1024u,
  .ipcmd_serial_clk_freq = 1,
  .is_uniform_block_size = 0,
  .block_size           = 64u * 1024u,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* No functions in this file — boot header data only.  The AHAB container
 * that references this M33 image is generated separately by
 * tools/imxrt1180/build_flash_image.sh using NXP SPSDK "nxpimage".
 * The M7 NuttX payload is not part of any AHAB container: it is flashed
 * raw at flash offset 0x80000 (XIP address 0x28080000) and started by the
 * M33 bootloader with WAIT cleared.
 */
