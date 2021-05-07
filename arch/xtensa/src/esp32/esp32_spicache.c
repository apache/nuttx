/****************************************************************************
 * arch/xtensa/src/esp32/esp32_spicache.c
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

#if defined(CONFIG_ESP32_SPIRAM) || defined(CONFIG_ESP32_SPIFLASH)

#include <stdint.h>
#include <debug.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>

#include "xtensa.h"
#include "xtensa_attr.h"

#include "hardware/esp32_soc.h"
#include "hardware/esp32_spi.h"
#include "hardware/esp32_dport.h"

#ifdef CONFIG_ESP32_SPIRAM
#include "esp32_spiram.h"
#endif

#include "esp32_spicache.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spiflash_disable_cache
 ****************************************************************************/

void IRAM_ATTR spi_disable_cache(int cpu, uint32_t *state)
{
  const uint32_t cache_mask = 0x3f; /* Caches' bits in CTRL1_REG */
  uint32_t regval;
  uint32_t ret = 0;

  if (cpu == 0)
    {
      ret |= (getreg32(DPORT_PRO_CACHE_CTRL1_REG) & cache_mask);
      while (((getreg32(DPORT_PRO_DCACHE_DBUG0_REG) >>
                      DPORT_PRO_CACHE_STATE_S) & DPORT_PRO_CACHE_STATE) != 1)
        {
          ;
        }

      regval  = getreg32(DPORT_PRO_CACHE_CTRL_REG);
      regval &= ~DPORT_PRO_CACHE_ENABLE_M;
      putreg32(regval, DPORT_PRO_CACHE_CTRL_REG);
    }
#ifdef CONFIG_SMP
  else
    {
      ret |= (getreg32(DPORT_APP_CACHE_CTRL1_REG) & cache_mask);
      while (((getreg32(DPORT_APP_DCACHE_DBUG0_REG) >>
                      DPORT_APP_CACHE_STATE_S) & DPORT_APP_CACHE_STATE) != 1)
        {
          ;
        }

      regval  = getreg32(DPORT_APP_CACHE_CTRL_REG);
      regval &= ~DPORT_APP_CACHE_ENABLE_M;
      putreg32(regval, DPORT_APP_CACHE_CTRL_REG);
    }

#endif
  *state = ret;
}

/****************************************************************************
 * Name: spiflash_enable_cache
 ****************************************************************************/

void IRAM_ATTR spi_enable_cache(int cpu, uint32_t state)
{
  const uint32_t cache_mask = 0x3f;  /* Caches' bits in CTRL1_REG */
  uint32_t regval;
  uint32_t ctrlreg;
  uint32_t ctrl1reg;
  uint32_t ctrlmask;

  if (cpu == 0)
    {
      ctrlreg = DPORT_PRO_CACHE_CTRL_REG;
      ctrl1reg = DPORT_PRO_CACHE_CTRL1_REG;
      ctrlmask = DPORT_PRO_CACHE_ENABLE_M;
    }
#ifdef CONFIG_SMP
  else
    {
      ctrlreg = DPORT_APP_CACHE_CTRL_REG;
      ctrl1reg = DPORT_APP_CACHE_CTRL1_REG;
      ctrlmask = DPORT_APP_CACHE_ENABLE_M;
    }
#endif

  regval  = getreg32(ctrlreg);
  regval |= ctrlmask;
  putreg32(regval, ctrlreg);

  regval  = getreg32(ctrl1reg);
  regval &= ~cache_mask;
  regval |= state;
  putreg32(regval, ctrl1reg);
}

#endif /* CONFIG_ESP32_SPICACHE */
