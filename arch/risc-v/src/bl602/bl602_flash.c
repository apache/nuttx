/****************************************************************************
 * arch/risc-v/src/bl602/bl602_flash.c
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
#include <syslog.h>
#include <debug.h>

#include <nuttx/irq.h>
#include "bl602_romapi.h"

#ifdef CONFIG_BL602_SPIFLASH

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define bl602_romapi_sflash_erase \
  ((void (*)(uint8_t *, uint32_t, int))BL602_ROMAPI_SFLASH_EREASE_NEEDLOCK)

#define bl602_romapi_sflash_write \
  ((void (*)(uint8_t *, uint32_t, const uint8_t *, int)) \
     BL602_ROMAPI_SFLASH_WRITE_NEEDLOCK)

#define bl602_romapi_sflash_read \
  ((void (*)( \
    uint8_t *, uint32_t, uint8_t *, int))BL602_ROMAPI_SFLASH_READ_NEEDLOCK)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct bl602_romflash_cfg_desc g_bl602_romflash_cfg;

struct bl602_romflash_cfg_desc
{
  uint32_t magic;
  uint8_t  cfg[84];
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int bl602_flash_erase(uint32_t addr, int len)
{
  irqstate_t flags;

  finfo("addr = %08lx, len = %d\n", addr, len);

  flags = up_irq_save();
  bl602_romapi_sflash_erase(g_bl602_romflash_cfg.cfg, addr, addr + len - 1);
  up_irq_restore(flags);

  return 0;
}

int bl602_flash_write(uint32_t addr, const uint8_t *src, int len)
{
  irqstate_t flags;

  finfo("addr = %08lx, len = %d\n", addr, len);

  flags = up_irq_save();
  bl602_romapi_sflash_write(g_bl602_romflash_cfg.cfg, addr, src, len);
  up_irq_restore(flags);

  return 0;
}

int bl602_flash_read(uint32_t addr, uint8_t *dst, int len)
{
  irqstate_t flags;

  finfo("addr = %08lx, len = %d\n", addr, len);

  flags = up_irq_save();
  bl602_romapi_sflash_read(g_bl602_romflash_cfg.cfg, addr, dst, len);
  up_irq_restore(flags);

  return 0;
}
#endif /* CONFIG_BL602_SPIFLASH */

