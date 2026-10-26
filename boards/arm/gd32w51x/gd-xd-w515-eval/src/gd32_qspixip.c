/****************************************************************************
 * boards/arm/gd32w51x/gd-xd-w515-eval/src/gd32_qspixip.c
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

/* Code and constant data used in place from the QSPI flash.
 *
 * The .qspixip section is linked at 0x10f00000, which the instruction cache
 * remaps to the QSPI window (0x90f00000, the last MiB of the flash), and
 * stored after the image in the internal flash.  At boot it is programmed
 * to the QSPI flash when it differs, then the window is remapped.
 *
 * Code run from the window itself hangs the bus as soon as it also reads
 * the window (its literals, its data); through the remap, code and data
 * both work.  Nothing may access the window directly while code runs from
 * the QSPI flash.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <syslog.h>
#include <sys/param.h>

#include <nuttx/compiler.h>
#include <nuttx/crc32.h>
#include <nuttx/mtd/mtd.h>

#include "arm_internal.h"
#include "chip.h"
#include "gd-xd-w515-eval.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Instruction cache: region 0 remaps 16 MiB at 0x10000000 to the QSPI
 * window at 0x90000000, through the master port of the external memories
 * (MSEL), with incrementing bursts (OBT).
 */

#define ICACHE_CTL              (GD32W51X_ICACHE_BASE + 0x0000)
#define ICACHE_STAT             (GD32W51X_ICACHE_BASE + 0x0004)
#define ICACHE_FC               (GD32W51X_ICACHE_BASE + 0x000c)
#define ICACHE_CFG0             (GD32W51X_ICACHE_BASE + 0x0020)

#define ICACHE_CTL_EN           (1 << 0)
#define ICACHE_CTL_INVAL        (1 << 1)
#define ICACHE_STAT_BUSY        (1 << 0)
#define ICACHE_STAT_END         (1 << 1)
#define ICACHE_FC_ENDC          (1 << 1)

#define ICACHE_CFG_BADDR(a)     (((a) >> 21) & 0xff)
#define ICACHE_CFG_SIZE_16M     (4 << 9)
#define ICACHE_CFG_EN           (1 << 15)
#define ICACHE_CFG_RADDR(a)     ((((a) >> 21) & 0x7ff) << 16)
#define ICACHE_CFG_MSEL         (1 << 28)
#define ICACHE_CFG_OBT          (1 << 31)

#define QSPIXIP_ALIAS           0x10000000
#define QSPIXIP_REMAP           (ICACHE_CFG_OBT | ICACHE_CFG_MSEL | \
                                 ICACHE_CFG_RADDR(GD32W51X_QSPIMEM_BASE) | \
                                 ICACHE_CFG_EN | ICACHE_CFG_SIZE_16M | \
                                 ICACHE_CFG_BADDR(QSPIXIP_ALIAS))

/* The section lives in the last MiB of the 16 MiB flash */

#define QSPIXIP_SIZE            (1024 * 1024)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Linker symbols: the section where it is used, and its copy */

extern const uint8_t _sqspixip[];
extern const uint8_t _eqspixip[];
extern const uint8_t _lqspixip[];

/* Data of the self-test, read in place */

locate_data(".qspixip.rodata")
static const char g_qspixip_text[] =
  "NuttX: code and data run in place from the GD25Q128E QSPI NOR flash "
  "of the GD-xD-W515-EVAL, through the instruction cache remap.";

static uint8_t g_qspixip_block[256];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qspixip_crc32
 *
 * Description:
 *   Run from the QSPI flash: the CRC-32 of crc32(), without its table.
 *
 ****************************************************************************/

locate_code(".qspixip.text") noinline_function
static uint32_t qspixip_crc32(const uint8_t *src, size_t len)
{
  uint32_t crc = 0;
  int i;

  while (len-- > 0)
    {
      crc ^= *src++;
      for (i = 0; i < 8; i++)
        {
          crc = (crc >> 1) ^ (0xedb88320 & -(crc & 1));
        }
    }

  return crc;
}

/****************************************************************************
 * Name: qspixip_cache
 *
 * Description:
 *   With the instruction cache disabled, set the remap region up (when
 *   'remap'), invalidate the cache, then enable it again.
 *
 ****************************************************************************/

static void qspixip_cache(bool remap)
{
  modifyreg32(ICACHE_CTL, ICACHE_CTL_EN, 0);
  while ((getreg32(ICACHE_STAT) & ICACHE_STAT_BUSY) != 0)
    {
    }

  if (remap)
    {
      putreg32(QSPIXIP_REMAP, ICACHE_CFG0);
    }

  putreg32(ICACHE_FC_ENDC, ICACHE_FC);
  modifyreg32(ICACHE_CTL, 0, ICACHE_CTL_INVAL);
  while ((getreg32(ICACHE_STAT) & ICACHE_STAT_END) == 0)
    {
    }

  modifyreg32(ICACHE_CTL, 0, ICACHE_CTL_EN);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_qspixip_install
 *
 * Description:
 *   Program the .qspixip section to the last MiB of the QSPI flash, from
 *   its copy in the internal flash, when it differs.  Called before the
 *   flash is mapped in memory.
 *
 ****************************************************************************/

int gd32_qspixip_install(struct mtd_dev_s *mtd)
{
  struct mtd_geometry_s geo;
  size_t size = _eqspixip - _sqspixip;
  off_t first;
  size_t nblocks;
  size_t i;
  size_t n;
  bool same = true;
  int ret;

  ret = MTD_IOCTL(mtd, MTDIOC_GEOMETRY, (unsigned long)&geo);
  if (ret < 0 || geo.blocksize > sizeof(g_qspixip_block) ||
      size > QSPIXIP_SIZE)
    {
      return ret < 0 ? ret : -EINVAL;
    }

  first   = ((off_t)geo.neraseblocks * geo.erasesize - QSPIXIP_SIZE) /
            geo.blocksize;
  nblocks = (size + geo.blocksize - 1) / geo.blocksize;

  /* Compare, one read block at a time */

  for (i = 0; i < nblocks && same; i++)
    {
      n = MIN(geo.blocksize, size - i * geo.blocksize);
      ret = MTD_BREAD(mtd, first + i, 1, g_qspixip_block);
      if (ret < 0)
        {
          return ret;
        }

      same = memcmp(g_qspixip_block, &_lqspixip[i * geo.blocksize], n) == 0;
    }

  if (same)
    {
      finfo("QSPI XIP section up to date (%zu bytes)\n", size);
      return OK;
    }

  ret = MTD_ERASE(mtd, first * geo.blocksize / geo.erasesize,
                  (size + geo.erasesize - 1) / geo.erasesize);
  if (ret < 0)
    {
      return ret;
    }

  for (i = 0; i < nblocks; i++)
    {
      n = MIN(geo.blocksize, size - i * geo.blocksize);
      memset(g_qspixip_block, 0xff, geo.blocksize);
      memcpy(g_qspixip_block, &_lqspixip[i * geo.blocksize], n);

      ret = MTD_BWRITE(mtd, first + i, 1, g_qspixip_block);
      if (ret < 0)
        {
          return ret;
        }
    }

  syslog(LOG_INFO, "QSPI XIP section programmed (%zu bytes)\n", size);
  return OK;
}

/****************************************************************************
 * Name: gd32_qspixip_initialize
 *
 * Description:
 *   Once the flash is mapped in memory, remap the QSPI window in the
 *   instruction cache, where the .qspixip section is linked, and check
 *   that code runs from it.
 *
 ****************************************************************************/

int gd32_qspixip_initialize(void)
{
  const uint8_t *copy = &_lqspixip[(const uint8_t *)g_qspixip_text -
                                   _sqspixip];
  uint32_t expected;
  uint32_t crc;

  qspixip_cache(true);

  expected = crc32(copy, sizeof(g_qspixip_text));
  crc      = qspixip_crc32((const uint8_t *)g_qspixip_text,
                           sizeof(g_qspixip_text));

  syslog(LOG_INFO, "QSPI XIP: code at %p, data at %p: crc %08" PRIx32
         " %s\n", qspixip_crc32, g_qspixip_text, crc,
         crc == expected ? "OK" : "FAILED");

  return crc == expected ? OK : -EIO;
}
