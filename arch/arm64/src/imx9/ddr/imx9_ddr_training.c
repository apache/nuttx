/****************************************************************************
 * arch/arm64/src/imx9/ddr/imx9_ddr_training.c
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

#include <nuttx/config.h>
#include <stdint.h>
#include <assert.h>
#include <debug.h>
#include <arch/chip/chip.h>

#include "arm64_arch.h"
#include "arm64_internal.h"
#include "imx9_ccm.h"
#include "imx9_clockconfig.h"
#include "hardware/imx93/imx93_memorymap.h"
#include "hardware/imx9_ddr_training.h"
#include <arch/board/imx9_ddr_training.h>

#include "imx9_trdc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define imx9_ddrphy_write(data, addr) \
  putreg32(data, ddrphy_to_axi_addr_remap(addr))
#define imx9_ddrphy_read(addr) \
  getreg32(ddrphy_to_axi_addr_remap(addr))

#define MAX(a, b) (((a) > (b)) ? (a) : (b))

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* DDR training binaries via incbin */

__asm__ (
  "   .section .rodata                   \n"
  "   .balign  64                        \n"
  "   .globl   g_dmem1d_begin            \n"
  "g_dmem1d_begin:                       \n"
  "   .incbin " STRINGIFY(FILE_DMEM_1D) "\n"
  "   .globl   g_dmem1d_end              \n"
  "g_dmem1d_end:                         \n"
  "    .balign 64                        \n"
  "    .globl  g_dmem2d_begin            \n"
  "g_dmem2d_begin:                       \n"
  "   .incbin " STRINGIFY(FILE_DMEM_2D) "\n"
  "   .globl   g_dmem2d_end              \n"
  "g_dmem2d_end:                         \n"
  "    .balign 64                        \n"
  "    .globl  g_imem1d_begin            \n"
  "g_imem1d_begin:                       \n"
  "   .incbin " STRINGIFY(FILE_IMEM_1D) "\n"
  "   .globl   g_imem1d_end              \n"
  "g_imem1d_end:                         \n"
  "    .balign 64                        \n"
  "    .globl  g_imem2d_begin            \n"
  "g_imem2d_begin:                       \n"
  "   .incbin " STRINGIFY(FILE_IMEM_2D) "\n"
  "   .globl   g_imem2d_end              \n"
  "g_imem2d_end:                         \n"
);

extern const uint8_t g_dmem1d_begin[];
extern const uint8_t g_dmem1d_end[];
extern const uint8_t g_dmem2d_begin[];
extern const uint8_t g_dmem2d_end[];
extern const uint8_t g_imem1d_begin[];
extern const uint8_t g_imem1d_end[];
extern const uint8_t g_imem2d_begin[];
extern const uint8_t g_imem2d_end[];

static uint32_t g_ccd_rr_max;
static uint32_t g_ccd_rw_max;
static uint32_t g_ccd_wr_max;
static uint32_t g_ccd_ww_max;

static struct dram_fsp_msg ddr_dram_fsp_msg[] =
{
  {
    /* P0 3733 1D */

    .drate = 3733,
    .fw_type = FW_1D_IMAGE,
    .fsp_cfg = ddr_fsp0_cfg,
    .fsp_cfg_num = nitems(ddr_fsp0_cfg),
  },
  {
    /* P0 3733 2D */

    .drate = 3733,
    .fw_type = FW_2D_IMAGE,
    .fsp_cfg = ddr_fsp0_2d_cfg,
    .fsp_cfg_num = nitems(ddr_fsp0_2d_cfg),
  },
};

/* DDR timing config params */

struct dram_timing_info dram_timing_default =
{
  .ddrc_cfg = ddr_ddrc_cfg,
  .ddrc_cfg_num = nitems(ddr_ddrc_cfg),
  .ddrphy_cfg = ddr_ddrphy_cfg,
  .ddrphy_cfg_num = nitems(ddr_ddrphy_cfg),
  .fsp_msg = ddr_dram_fsp_msg,
  .fsp_msg_num = nitems(ddr_dram_fsp_msg),
  .ddrphy_pie = ddr_phy_pie,
  .ddrphy_pie_num = nitems(ddr_phy_pie),
  .fsp_cfg = ddr_dram_fsp_cfg,
  .fsp_cfg_num = nitems(ddr_dram_fsp_cfg),
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ddrphy_to_axi_addr_remap
 *
 * Description:
 *   This performs ddr phy address remapping.
 *
 * Input Parameters:
 *   paddr_apb_from_ctlr   -  Address to be remapped
 *
 * Returned Value:
 *   Remapped address
 *
 ****************************************************************************/

static uint32_t ddrphy_to_axi_addr_remap(uint32_t ddr_phy_addr)
{
  uint32_t x1;
  uint32_t x2;
  uint32_t y1;
  uint32_t y2;

  x1 = (ddr_phy_addr >> 12) & 0x3ff;
  x2 = ddr_phy_addr & 0xfff;

  switch (x1)
    {
      case 0x000:
        y1 = 0x00;
        break;
      case 0x001:
        y1 = 0x01;
        break;
      case 0x002:
        y1 = 0x02;
        break;
      case 0x003:
        y1 = 0x03;
        break;
      case 0x004:
        y1 = 0x04;
        break;
      case 0x005:
        y1 = 0x05;
        break;
      case 0x006:
        y1 = 0x06;
        break;
      case 0x007:
        y1 = 0x07;
        break;
      case 0x008:
        y1 = 0x08;
        break;
      case 0x009:
        y1 = 0x09;
        break;
      case 0x00a:
        y1 = 0x0a;
        break;
      case 0x00b:
        y1 = 0x0b;
        break;
      case 0x100:
        y1 = 0x0c;
        break;
      case 0x101:
        y1 = 0x0d;
        break;
      case 0x102:
        y1 = 0x0e;
        break;
      case 0x103:
        y1 = 0x0f;
        break;
      case 0x104:
        y1 = 0x10;
        break;
      case 0x105:
        y1 = 0x11;
        break;
      case 0x106:
        y1 = 0x12;
        break;
      case 0x107:
        y1 = 0x13;
        break;
      case 0x108:
        y1 = 0x14;
        break;
      case 0x109:
        y1 = 0x15;
        break;
      case 0x10a:
        y1 = 0x16;
        break;
      case 0x10b:
        y1 = 0x17;
        break;
      case 0x200:
        y1 = 0x18;
        break;
      case 0x201:
        y1 = 0x19;
        break;
      case 0x202:
        y1 = 0x1a;
        break;
      case 0x203:
        y1 = 0x1b;
        break;
      case 0x204:
        y1 = 0x1c;
        break;
      case 0x205:
        y1 = 0x1d;
        break;
      case 0x206:
        y1 = 0x1e;
        break;
      case 0x207:
        y1 = 0x1f;
        break;
      case 0x208:
        y1 = 0x20;
        break;
      case 0x209:
        y1 = 0x21;
        break;
      case 0x20a:
        y1 = 0x22;
        break;
      case 0x20b:
        y1 = 0x23;
        break;
      case 0x300:
        y1 = 0x24;
        break;
      case 0x301:
        y1 = 0x25;
        break;
      case 0x302:
        y1 = 0x26;
        break;
      case 0x303:
        y1 = 0x27;
        break;
      case 0x304:
        y1 = 0x28;
        break;
      case 0x305:
        y1 = 0x29;
        break;
      case 0x306:
        y1 = 0x2a;
        break;
      case 0x307:
        y1 = 0x2b;
        break;
      case 0x308:
        y1 = 0x2c;
        break;
      case 0x309:
        y1 = 0x2d;
        break;
      case 0x30a:
        y1 = 0x2e;
        break;
      case 0x30b:
        y1 = 0x2f;
        break;
      case 0x010:
        y1 = 0x30;
        break;
      case 0x011:
        y1 = 0x31;
        break;
      case 0x012:
        y1 = 0x32;
        break;
      case 0x013:
        y1 = 0x33;
        break;
      case 0x014:
        y1 = 0x34;
        break;
      case 0x015:
        y1 = 0x35;
        break;
      case 0x016:
        y1 = 0x36;
        break;
      case 0x017:
        y1 = 0x37;
        break;
      case 0x018:
        y1 = 0x38;
        break;
      case 0x019:
        y1 = 0x39;
        break;
      case 0x110:
        y1 = 0x3a;
        break;
      case 0x111:
        y1 = 0x3b;
        break;
      case 0x112:
        y1 = 0x3c;
        break;
      case 0x113:
        y1 = 0x3d;
        break;
      case 0x114:
        y1 = 0x3e;
        break;
      case 0x115:
        y1 = 0x3f;
        break;
      case 0x116:
        y1 = 0x40;
        break;
      case 0x117:
        y1 = 0x41;
        break;
      case 0x118:
        y1 = 0x42;
        break;
      case 0x119:
        y1 = 0x43;
        break;
      case 0x210:
        y1 = 0x44;
        break;
      case 0x211:
        y1 = 0x45;
        break;
      case 0x212:
        y1 = 0x46;
        break;
      case 0x213:
        y1 = 0x47;
        break;
      case 0x214:
        y1 = 0x48;
        break;
      case 0x215:
        y1 = 0x49;
        break;
      case 0x216:
        y1 = 0x4a;
        break;
      case 0x217:
        y1 = 0x4b;
        break;
      case 0x218:
        y1 = 0x4c;
        break;
      case 0x219:
        y1 = 0x4d;
        break;
      case 0x310:
        y1 = 0x4e;
        break;
      case 0x311:
        y1 = 0x4f;
        break;
      case 0x312:
        y1 = 0x50;
        break;
      case 0x313:
        y1 = 0x51;
        break;
      case 0x314:
        y1 = 0x52;
        break;
      case 0x315:
        y1 = 0x53;
        break;
      case 0x316:
        y1 = 0x54;
        break;
      case 0x317:
        y1 = 0x55;
        break;
      case 0x318:
        y1 = 0x56;
        break;
      case 0x319:
        y1 = 0x57;
        break;
      case 0x020:
        y1 = 0x58;
        break;
      case 0x120:
        y1 = 0x59;
        break;
      case 0x220:
        y1 = 0x5a;
        break;
      case 0x320:
        y1 = 0x5b;
        break;
      case 0x040:
        y1 = 0x5c;
        break;
      case 0x140:
        y1 = 0x5d;
        break;
      case 0x240:
        y1 = 0x5e;
        break;
      case 0x340:
        y1 = 0x5f;
        break;
      case 0x050:
        y1 = 0x60;
        break;
      case 0x051:
        y1 = 0x61;
        break;
      case 0x052:
        y1 = 0x62;
        break;
      case 0x053:
        y1 = 0x63;
        break;
      case 0x054:
        y1 = 0x64;
        break;
      case 0x055:
        y1 = 0x65;
        break;
      case 0x056:
        y1 = 0x66;
        break;
      case 0x057:
        y1 = 0x67;
        break;
      case 0x070:
        y1 = 0x68;
        break;
      case 0x090:
        y1 = 0x69;
        break;
      case 0x190:
        y1 = 0x6a;
        break;
      case 0x290:
        y1 = 0x6b;
        break;
      case 0x390:
        y1 = 0x6c;
        break;
      case 0x0c0:
        y1 = 0x6d;
        break;
      case 0x0d0:
        y1 = 0x6e;
        break;
      default:
        y1 = 0x00;
        break;
  }

  y2 = ((y1 << 13) | (x2 << 1)) << 1;
  y2 += IMX9_DDR_PHY_BASE;

  return y2;
}

/****************************************************************************
 * Name: ddr_firmware_upload
 *
 * Description:
 *   This copies a firmware from memory into ddr phy.
 *
 * Input Parameters:
 *   dst   -  Destination address of the firmware
 *   src   -  Source address of the firmware
 *   len   -  Size of the firmware
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ddr_firmware_upload(uint32_t dst, uint32_t src, uint32_t len)
{
  uint32_t tmp;
  int i;

  for (i = src; i < (src + len); i += 4)
    {
      tmp = getreg32(i);
      imx9_ddrphy_write(tmp & 0xffff, dst);
      dst += 1;
      imx9_ddrphy_write((tmp >> 16), dst);
      dst += 1;
    }
}

/****************************************************************************
 * Name: ddr_firmware_verify
 *
 * Description:
 *   This verifies that the uploaded firmware matches the on sent. This way
 *   some errors are detected in the early phase.
 *
 * Input Parameters:
 *   dst   -  Destination address of the firmware
 *   src   -  Source address of the firmware
 *   len   -  Size of the firmware
 *
 * Returned Value:
 *   Number of errors detected, zero if none
 *
 ****************************************************************************/

static uint32_t ddr_firmware_verify(uint32_t dst, uint32_t src, uint32_t len)
{
  uint32_t errors = 0;
  uint32_t tmp;
  uint32_t i;

  for (i = src; i < (src + len); i += 4)
    {
      tmp = imx9_ddrphy_read(dst) & 0xffff;
      dst += 1;
      tmp += imx9_ddrphy_read(dst) << 16;

      if (tmp != getreg32(i))
        {
          errors++;
        }

      dst += 1;
    }

  return errors;
}

/****************************************************************************
 * Name: ddr_load_training_firmware
 *
 * Description:
 *   Loads the DDR training binaries from memory and checks they're properly
 *   uploaded. DDR training binaries reside in binary files.
 *
 * Input Parameters:
 *   type   -  Firmware type (FW_1D_IMAGE or FW_2D_IMAGE)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ddr_load_training_firmware(enum fw_type type)
{
  uint64_t imem_start;
  uint64_t dmem_start;
  uint64_t imem_len;
  uint64_t dmem_len;
  uint32_t errors = 0;

  if (type == FW_1D_IMAGE)
    {
      imem_start = (uint64_t)g_imem1d_begin;
      imem_len   = (uint64_t)(g_imem1d_end - g_imem1d_begin);
      dmem_start = (uint64_t)g_dmem1d_begin;
      dmem_len   = (uint64_t)(g_dmem1d_end - g_dmem1d_begin);
    }
  else
    {
      imem_start = (uint64_t)g_imem2d_begin;
      imem_len   = (uint64_t)(g_imem2d_end - g_imem2d_begin);
      dmem_start = (uint64_t)g_dmem2d_begin;
      dmem_len   = (uint64_t)(g_dmem2d_end - g_dmem2d_begin);
    }

  /* Upload */

  ddr_firmware_upload(IMX9_IMEM_OFFSET_ADDR, imem_start, imem_len);
  ddr_firmware_upload(IMX9_DMEM_OFFSET_ADDR, dmem_start, dmem_len);

  /* Verify */

  errors = ddr_firmware_verify(IMX9_IMEM_OFFSET_ADDR, imem_start, imem_len);
  assert(errors == 0);
  errors = ddr_firmware_verify(IMX9_DMEM_OFFSET_ADDR, dmem_start, dmem_len);
  assert(errors == 0);
}

/****************************************************************************
 * Name: ddr_pmu_poll_msg_ready
 *
 * Description:
 *   Poll until the message is ready / available from the DDR Power
 *   Management Unit (PMU).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void ddr_pmu_poll_msg_ready(void)
{
  uint32_t reg;

  do
    {
      reg = imx9_ddrphy_read(0xd0004);
    }
  while (reg & 0x1);
}

/****************************************************************************
 * Name: ddr_pmu_ack_msg_receive
 *
 * Description:
 *   Ack the message from the DDR Power Management Unit.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void ddr_pmu_ack_msg_receive(void)
{
  uint32_t reg;

  imx9_ddrphy_write(0, 0xd0031);

  do
    {
      reg = imx9_ddrphy_read(0xd0004);
    }
  while (!(reg & 0x1));

  imx9_ddrphy_write(0x1, 0xd0031);
}

/****************************************************************************
 * Name: ddr_get_mail
 *
 * Description:
 *   Get the mail from the PMU mailbox.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The message
 *
 ****************************************************************************/

static inline uint32_t ddr_get_mail(void)
{
  uint32_t reg;

  ddr_pmu_poll_msg_ready();

  reg = imx9_ddrphy_read(0xd0032);

  ddr_pmu_ack_msg_receive();

  return reg;
}

/****************************************************************************
 * Name: ddr_get_stream_msg
 *
 * Description:
 *   Get the streaming message.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The message
 *
 ****************************************************************************/

static inline uint32_t ddr_get_stream_msg(void)
{
  uint32_t reg;
  uint32_t reg2;

  ddr_pmu_poll_msg_ready();

  reg = imx9_ddrphy_read(0xd0032);

  reg2 = imx9_ddrphy_read(0xd0034);

  reg2 = (reg2 << 16) | reg;

  ddr_pmu_ack_msg_receive();

  return reg2;
}

/****************************************************************************
 * Name: ddr_decode_streaming_msg
 *
 * Description:
 *   Decode the expected streaming message into useful format.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void ddr_decode_streaming_msg(void)
{
  uint32_t string_index;
  int i = 0;

  string_index = ddr_get_stream_msg();

  while (i < (string_index & 0xffff))
    {
      ddr_get_stream_msg();
      i++;
    }
}

/****************************************************************************
 * Name: ddrphy_wait_training_complete
 *
 * Description:
 *   Wait for the phy training to complete.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int ddrphy_wait_training_complete(void)
{
  uint32_t mail;

  while (1)
    {
      mail = ddr_get_mail();

      if (mail == 0x08)
        {
          ddr_decode_streaming_msg();
        }
      else if (mail == 0x07)
        {
          return 0;
        }
      else if (mail == 0xff)
        {
          return -EIO;
        }
    }
}

/****************************************************************************
 * Name: array_max
 *
 * Description:
 *   Get the maximum value out of and array, bounded by the addr_start and
 *   addr_end limiters
 *
 * Input Parameters:
 *   data         -  Array of data variables
 *   addr_start   -  Address start marker
 *   addr_end     -  Address end marker
 *
 * Returned Value:
 *   Maximum value of the input array bound by the limiters
 *
 ****************************************************************************/

static uint32_t array_max(uint32_t data[],
                          uint32_t addr_start,
                          uint32_t addr_end)
{
  uint32_t imax = 0;
  uint32_t i;

  for (i = addr_start; i <= addr_end; i++)
    {
      if (((data[i] >> 7) == 0) && (data[i] > imax))
        {
          imax = data[i];
        }
    }

  return imax;
}

/****************************************************************************
 * Name: imx9_save_trained_ccd
 *
 * Description:
 *   Get and save the trained column to column delay for the requested
 *   frequency configuration for later use.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void imx9_save_trained_ccd(void)
{
  uint32_t ccd_channel_a[12];
  uint32_t ccd_channel_b[12];
  uint32_t *ccd_ptr;
  uint32_t ccd_ch_r_max[4];
  uint32_t ccd_ch_w_max[4];
  uint32_t tmp;
  uint32_t idx;
  uint32_t i;

  for (i = 0; i < 7; i++)
    {
      idx = (i << 1);

      if (i < 6)
        {
          tmp = imx9_ddrphy_read(0x54013 + i);
          ccd_channel_a[idx] = tmp & 0xff;
          ccd_channel_a[idx + 1] = (tmp >> 8) & 0xff;
        }

      tmp = imx9_ddrphy_read(0x5402c + i);

      if (i > 0)
        {
          ccd_channel_b[idx - 1] = tmp & 0xff;
        }

      if (i < 6)
        {
          ccd_channel_b[idx] = (tmp >> 8) & 0xff;
        }
    }

  for (i = 0; i < 4; i++)
    {
      if (i & 1)
        {
          ccd_ptr = ccd_channel_b;
        }
      else
        {
          ccd_ptr = ccd_channel_a;
        }

      if (i < 2)
        {
          ccd_ch_r_max[i] = array_max(ccd_ptr, 0, 1);
          ccd_ch_w_max[i] = array_max(ccd_ptr, 6, 9);
        }
      else
        {
          ccd_ch_r_max[i] = array_max(ccd_ptr, 2, 5);
          ccd_ch_w_max[i] = array_max(ccd_ptr, 10, 11);
        }
    }

  g_ccd_rr_max = ccd_ch_r_max[0] > ccd_ch_r_max[1] ?
                 ccd_ch_r_max[0] : ccd_ch_r_max[1];
  g_ccd_rw_max = ccd_ch_r_max[2] > ccd_ch_r_max[3] ?
                 ccd_ch_r_max[2] : ccd_ch_r_max[3];

  g_ccd_wr_max = ccd_ch_w_max[0] > ccd_ch_w_max[1] ?
                 ccd_ch_w_max[0] : ccd_ch_w_max[1];
  g_ccd_ww_max = ccd_ch_w_max[2] > ccd_ch_w_max[3] ?
                 ccd_ch_w_max[2] : ccd_ch_w_max[3];
}

/****************************************************************************
 * Name: imx9_ddr_cfg_phy
 *
 * Description:
 *   Configure the ddr phy with the appropriate timing structure.
 *
 * Input Parameters:
 *   dram_timing    -  DRAM timing data structure
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int imx9_ddr_cfg_phy(struct dram_timing_info *dram_timing)
{
  struct dram_cfg_param *dram_cfg;
  struct dram_fsp_msg *fsp_msg;
  uint32_t num;
  int ret;
  int i;
  int j;

  /* Prepare PHY configuration */

  dram_cfg = dram_timing->ddrphy_cfg;
  num  = dram_timing->ddrphy_cfg_num;
  for (i = 0; i < num; i++)
    {
      /* Configure phy regs */

      imx9_ddrphy_write(dram_cfg->val, dram_cfg->reg);
      dram_cfg++;
    }

  fsp_msg = dram_timing->fsp_msg;
  for (i = 0; i < dram_timing->fsp_msg_num; i++)
    {
      /* Load the dram training firmware image */

      imx9_ddrphy_write(0x0, 0xd0000);
      ddr_load_training_firmware(fsp_msg->fw_type);

      /* Load the frequency set point message block parameter */

      dram_cfg = fsp_msg->fsp_cfg;
      num = fsp_msg->fsp_cfg_num;
      for (j = 0; j < num; j++)
        {
          imx9_ddrphy_write(dram_cfg->val, dram_cfg->reg);
          dram_cfg++;
        }

      /* Execute the firmware */

      imx9_ddrphy_write(0x1, 0xd0000);
      imx9_ddrphy_write(0x9, 0xd0099);
      imx9_ddrphy_write(0x1, 0xd0099);
      imx9_ddrphy_write(0x0, 0xd0099);

      /* Wait for the training firmware to complete */

      ret = ddrphy_wait_training_complete();
      if (ret)
        {
          return ret;
        }

      /* Halt the microcontroller */

      imx9_ddrphy_write(0x1, 0xd0099);
      imx9_ddrphy_write(0x0, 0xd0000);

      if (fsp_msg->fw_type != FW_2D_IMAGE)
        imx9_save_trained_ccd();

      imx9_ddrphy_write(0x1, 0xd0000);

      fsp_msg++;
    }

  /* Load PHY Init Engine Image */

  dram_cfg = dram_timing->ddrphy_pie;
  num = dram_timing->ddrphy_pie_num;
  for (i = 0; i < num; i++)
    {
      imx9_ddrphy_write(dram_cfg->val, dram_cfg->reg);
      dram_cfg++;
    }

  return 0;
}

/****************************************************************************
 * Name: imx9_ddrphy_coldreset
 *
 * Description:
 *   Perform a cold reset on the DDR phy.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void imx9_ddrphy_coldreset(void)
{
  /* dramphy_apb_n default 1 , assert -> 0, de_assert -> 1 */

  /* dramphy_reset_n default 0 , assert -> 0, de_assert -> 1 */

  /* dramphy_PwrOKIn default 0 , assert -> 1, de_assert -> 0 */

  /* src_gen_dphy_apb_sw_rst_de_assert */

  modifyreg32(IMX9_REG_SRC_DPHY_SW_CTRL, (1 << 0), 0);

  /* src_gen_dphy_sw_rst_de_assert */

  modifyreg32(IMX9_REG_SRC_DPHY_SINGLE_RESET_SW_CTRL, (1 << 2), 0);

  /* src_gen_dphy_PwrOKIn_sw_rst_de_assert() */

  modifyreg32(IMX9_REG_SRC_DPHY_SINGLE_RESET_SW_CTRL, 0, (1 << 0));

  /* mdelay(10); */

  up_udelay(10 * 1000);

  /* src_gen_dphy_apb_sw_rst_assert */

  modifyreg32(IMX9_REG_SRC_DPHY_SW_CTRL, 0, (1 << 0));

  /* src_gen_dphy_sw_rst_assert */

  modifyreg32(IMX9_REG_SRC_DPHY_SINGLE_RESET_SW_CTRL, 0, (1 << 2));

  /* mdelay(10); */

  up_udelay(10 * 1000);

  /* src_gen_dphy_PwrOKIn_sw_rst_assert */

  modifyreg32(IMX9_REG_SRC_DPHY_SINGLE_RESET_SW_CTRL, (1 << 0), 0);

  /* mdelay(10); */

  up_udelay(10 * 1000);

  /* src_gen_dphy_apb_sw_rst_de_assert */

  modifyreg32(IMX9_REG_SRC_DPHY_SW_CTRL, (1 << 0), 0);

  /* src_gen_dphy_sw_rst_de_assert() */

  modifyreg32(IMX9_REG_SRC_DPHY_SINGLE_RESET_SW_CTRL, (1 << 2), 0);
}

/****************************************************************************
 * Name: imx9_check_ddrc_busy
 *
 * Description:
 *   Wait until the ddr becomes idle from busy state. Bit 31 zero means it's
 *   busy, when it becomes 1, it's no longer busy.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void imx9_check_ddrc_busy(void)
{
  uint32_t regval;

  do
    {
      /* DDR SDRAM Debug Status 2 (DDRDSR_2) */

      regval = getreg32(IMX9_DDRDSR_2);
    }
  while (!(regval & (1u << 31)));
}

/****************************************************************************
 * Name: imx9_check_dfi_init_complete
 *
 * Description:
 *   Check whether the DDR phy training interface (DFI) initialization has
 *   completed. Wait until done.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void imx9_check_dfi_init_complete(void)
{
  uint32_t regval;

  do
    {
      regval = getreg32(IMX9_DDRDSR_2);
    }
  while (!(regval & (1 << 2)));

  modifyreg32(IMX9_DDRDSR_2, 0, (1 << 2));
}

/****************************************************************************
 * Name: ddrc_config
 *
 * Description:
 *   Configure the DDR according to the proposed values.
 *
 * Input Parameters:
 *   dram_timing    -  DRAM timing parameters
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void imx9_ddrc_config(struct dram_timing_info *dram_timing)
{
  uint32_t num = dram_timing->ddrc_cfg_num;
  struct dram_cfg_param *ddrc_config;
  uint32_t i;

  ddrc_config = dram_timing->ddrc_cfg;
  for (i = 0; i < num; i++)
    {
      putreg32(ddrc_config->val, ddrc_config->reg);
      ddrc_config++;
    }

  if (dram_timing->fsp_cfg != NULL)
    {
      ddrc_config = dram_timing->fsp_cfg[0].ddrc_cfg;
      while (ddrc_config->reg != 0)
        {
          putreg32(ddrc_config->val, ddrc_config->reg);
          ddrc_config++;
        }
    }
}

/****************************************************************************
 * Name: ddrc_get_fsp_reg
 *
 * Description:
 *   Get register 'reg' value from known configurations. This returns the
 *   first available value, not expecting duplicates.
 *
 * Input Parameters:
 *   ddrc_cfg    -  DRAM configuration
 *   cfg_num     -  Number of configs
 *   reg         -  Register
 *
 * Returned Value:
 *   Register value or 0 if not found
 *
 ****************************************************************************/

static uint32_t ddrc_get_fsp_reg(struct dram_cfg_param *ddrc_cfg,
                                 uint32_t cfg_num, uint32_t reg)
{
  uint32_t i;

  for (i = 0; i < cfg_num; i++)
    {
      if (reg == ddrc_cfg[i].reg)
        {
          return ddrc_cfg[i].val;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: ddrc_update_fsp_reg
 *
 * Description:
 *   Update register 'reg' vith an updated value 'val' for all known
 *   configurations 'cfg_num'.
 *
 * Input Parameters:
 *   ddrc_cfg    -  DRAM configuration
 *   cfg_num     -  Number of configs
 *   reg         -  Register
 *   val         -  Register value
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ddrc_update_fsp_reg(struct dram_cfg_param *ddrc_cfg,
                                int cfg_num, uint32_t reg,
                                uint32_t val)
{
  uint32_t i;

  for (i = 0; i < cfg_num; i++)
    {
      if (reg == ddrc_cfg[i].reg)
        {
          ddrc_cfg[i].val = val;
          return;
        }
    }
}

/****************************************************************************
 * Name: imx9_update_rank_space
 *
 * Description:
 *   Update a range of values
 *
 * Input Parameters:
 *   dram_timing    -  DRAM timing info
 *   num_cfgs       -  Number of configs
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void imx9_update_rank_space(struct dram_timing_info *dram_timing,
                                   uint32_t num_cfgs)
{
  uint32_t extra_wwt;
  uint32_t extra_rrt;
  uint32_t extra_wrt;
  uint32_t extra_rwt;
  uint32_t max_wwt;
  uint32_t max_rrt;
  uint32_t max_wrt;
  uint32_t max_rwt;
  uint32_t tmp_0;
  uint32_t tmp_4;
  uint32_t wwt;
  uint32_t rrt;
  uint32_t wrt;
  uint32_t rwt;
  uint32_t i;

  assert(dram_timing->fsp_cfg_num > 0);

  for (i = 0; i < num_cfgs; i++)
    {
      /* Read turnaround cycles from timing_cfg_0 */

      tmp_0 = ddrc_get_fsp_reg(dram_timing->fsp_cfg[i].ddrc_cfg,
                               nitems(dram_timing->fsp_cfg[i].ddrc_cfg),
                               IMX9_DDRC_TIMING_CFG_0);

      /* Clock cycles for different turnarounds */

      wwt = (tmp_0 >> 24) & 0x3; /* Write-to-write turnaround */
      rrt = (tmp_0 >> 26) & 0x3; /* Read-to-read turnaround */
      wrt = (tmp_0 >> 28) & 0x3; /* Write-to-read turnaround */
      rwt = (tmp_0 >> 30) & 0x3; /* Read-to-write turnaround */

      /* Read extra turnaround cycles from timing_cfg_4 */

      tmp_4 = ddrc_get_fsp_reg(dram_timing->fsp_cfg[i].ddrc_cfg,
                               nitems(dram_timing->fsp_cfg[i].ddrc_cfg),
                               IMX9_DDRC_TIMING_CFG_4);
      extra_wwt = (tmp_4 >> 8)  & 0x3; /* Write-to-write extra clock cycles */
      extra_rrt = (tmp_4 >> 10) & 0x3; /* Read-to-read extra clock cycles */
      extra_wrt = (tmp_4 >> 12) & 0x3; /* Write-to-read extra clock cycles */
      extra_rwt = (tmp_4 >> 14) & 0x3; /* Read-to-write extra clock cycles */

      /* Total cycles: cycles + extra cycles */

      wwt = (extra_wwt << 2) | wwt;
      rrt = (extra_rrt << 2) | rrt;
      wrt = (extra_wrt << 2) | wrt;
      rwt = (extra_rwt << 2) | rwt;

      /* Compare to column to column delay: pick the maximum */

      max_wwt = MAX(g_ccd_ww_max, wwt);
      max_rrt = MAX(g_ccd_rr_max, rrt);
      max_wrt = MAX(g_ccd_wr_max, wrt);
      max_rwt = MAX(g_ccd_rw_max, rwt);

      /* Truncate to max 15 */

      if (max_wwt > 15)
        max_wwt = 15;
      if (max_rrt > 15)
        max_rrt = 15;
      if (max_wrt > 15)
        max_wrt = 15;
      if (max_rwt > 15)
        max_rwt = 15;

      /* Pick the max timings for controller registers */

      wwt = max_wwt & 0x3;
      rrt = max_rrt & 0x3;
      wrt = max_wrt & 0x3;
      rwt = max_rwt & 0x3;

      extra_wwt = (max_wwt >> 2);
      extra_rrt = (max_rrt >> 2);
      extra_wrt = (max_wrt >> 2);
      extra_rwt = (max_rwt >> 2);

      /* Refresh timing_cfg_0 and timing_cfg_4 */

      tmp_0 &= 0x00ffffff;
      tmp_0 |= (rwt << 30) | (wrt << 28) | (rrt << 26) | (wwt << 24);

      tmp_4 &= 0xffff00ff;
      tmp_4 |= (extra_rwt << 14) | (extra_wrt << 12) | (extra_rrt << 10) |
               (extra_wwt << 8);

      ddrc_update_fsp_reg(dram_timing->fsp_cfg[i].ddrc_cfg,
                          nitems(dram_timing->fsp_cfg[i].ddrc_cfg),
                          IMX9_DDRC_TIMING_CFG_0, tmp_0);
      ddrc_update_fsp_reg(dram_timing->fsp_cfg[i].ddrc_cfg,
                          nitems(dram_timing->fsp_cfg[i].ddrc_cfg),
                          IMX9_DDRC_TIMING_CFG_4, tmp_4);
    }
}

/****************************************************************************
 * Name: imx9_ddr_init
 *
 * Description:
 *   Initializes the ddr. This contains all necessary DDR training sequences
 *   required for the system to set up the memory.
 *
 * Input Parameters:
 *   dram_timing    -  DRAM timing structure
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int imx9_ddr_init(struct dram_timing_info *dram_timing)
{
  struct dram_cfg_param *ddrc_cfg;
  uint32_t ddrc_cfg_num;
  int ret;
  int i;

  /* Reset ddrphy */

  imx9_ddrphy_coldreset();

  ret = imx9_ddr_cfg_phy(dram_timing);

  if (ret)
    {
      return ret;
    }

  /* No support for zero fsp_msg_num at the moment */

  assert((dram_timing->fsp_msg_num - 1) >= 0);

  imx9_update_rank_space(dram_timing, dram_timing->fsp_msg_num - 1);

  /* Program the ddrc registers */

  imx9_ddrc_config(dram_timing);

  imx9_check_dfi_init_complete();

  /* DDRC enable */

  modifyreg32(IMX9_DDRC_SDRAM_CFG, 0, IMX9_DDRC_MEM_EN);

  /* Wait while busy */

  imx9_check_ddrc_busy();

  /* If DRAM Data INIT set, wait it be completed */

  ddrc_cfg = dram_timing->ddrc_cfg;
  ddrc_cfg_num = dram_timing->ddrc_cfg_num;
  for (i = 0; i < ddrc_cfg_num; i++)
    {
      if (ddrc_cfg->reg == IMX9_DDRC_SDRAM_CFG2)
        {
          if (ddrc_cfg->val & IMX9_DDRC_CFG2_D_INIT)
            {
              while (getreg32(IMX9_DDRC_SDRAM_CFG2) & IMX9_DDRC_CFG2_D_INIT);
            }
          break;
        }

        ddrc_cfg++;
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_dram_init
 *
 * Description:
 *   Performs DDR training
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_dram_init(void)
{
  struct dram_timing_info *ptiming = &dram_timing_default;

  return imx9_ddr_init(ptiming);
}
