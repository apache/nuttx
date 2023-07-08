/****************************************************************************
 * arch/arm/src/nrf91/nrf91_errata.c
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

#include "arm_internal.h"
#include "barriers.h"

#include "hardware/nrf91_memorymap.h"
#include "hardware/nrf91_regulators.h"
#include "hardware/nrf91_uicr.h"
#include "hardware/nrf91_ficr.h"
#include "hardware/nrf91_nvmc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MEMORY_SYNC() ARM_ISB(); ARM_DSB()

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_NRF91_UICR_HFXO_WORKAROUND
/****************************************************************************
 * Name: nrf91_uicr_recover
 *
 * Description:
 *   Recover HFXOCNT and HFXOSRC registers after erasing all FLASH.
 *   Without this workaround, the modem core never boots properly.
 *
 ****************************************************************************/

void nrf91_uicr_recover(void)
{
  bool hfxosrc_recover = false;
  bool hfxocnt_recover = false;

  if (getreg32(NRF91_UICR_HFXOCNT) == 0xffffffff)
    {
      hfxocnt_recover = true;
    }

  if (getreg32(NRF91_UICR_HFXOSRC) == 0xffffffff)
    {
      hfxosrc_recover = true;
    }

  if (!hfxocnt_recover && !hfxosrc_recover)
    {
      return;
    }

  /* Wait for flash */

  while (!(getreg32(NRF91_NVMC_READY) & NVMC_READY_READY))
    {
    }

  /* Enable write */

  putreg32(NVMC_CONFIG_WEN, NRF91_NVMC_CONFIG);

  /* Memory sync */

  MEMORY_SYNC();

  /* Write HFXOSRC */

  if (hfxosrc_recover)
    {
      putreg32(0x0 , NRF91_UICR_HFXOSRC);
    }

  /* Wait for flash */

  while (!(getreg32(NRF91_NVMC_READY) & NVMC_READY_READY))
    {
    }

  /* Memory sync */

  MEMORY_SYNC();

  /* Write HFXOCNT */

  if (hfxocnt_recover)
    {
      putreg32(0x20, NRF91_UICR_HFXOCNT);
    }

  /* Wait for flash */

  while (!(getreg32(NRF91_NVMC_READY) & NVMC_READY_READY))
    {
    }

  /* Memory sync */

  MEMORY_SYNC();

  /* Read only access */

  putreg32(NVMC_CONFIG_REN, NRF91_NVMC_CONFIG);

  /* System reset */

  up_systemreset();
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifndef CONFIG_ARCH_TRUSTZONE_NONSECURE
/****************************************************************************
 * Name: nrf91_errata_secure
 *
 * Description:
 *   Errata that must be applied in asecure environment
 *
 ****************************************************************************/

void nrf91_errata_secure(void)
{
#ifdef NRF91_DCDC_ERRATA15_WORKAROUND
  /* Workaround for 3.7 [15] REGULATORS: Supply regulators default to
   * LDO mode after reset
   */

  putreg32(REGULATORS_DCDCEN_ENABLE, NRF91_REGULATORS_DCDCEN);
#endif

#ifdef NRF91_LFXO_ERRATA31_WORKAROUND
  /* Workaround for 3.15 [31] LFXO: LFXO startup fails */

  *((volatile uint32_t *)0x5000470c) = 0x0;
  *((volatile uint32_t *)0x50004710) = 0x1;
#endif

#ifdef CONFIG_NRF91_UICR_HFXO_WORKAROUND
  nrf91_uicr_recover();
#endif
}

/****************************************************************************
 * Name: nrf91_ficr_ram_copy
 *
 * Description:
 *   Copy FICR to a fixed RAM region.
 *
 ****************************************************************************/

void nrf91_ficr_ram_copy(void)
{
  const uint32_t *src;
  uint32_t *dest;

  for (src = (const uint32_t *)NRF91_FICR_BASE,
         dest = (uint32_t *)(NRF91_NONSECURE_RAM_FICR);
       dest < (uint32_t *)(NRF91_SRAM_BASE +
                           CONFIG_NRF91_CPUAPP_MEM_RAM_SIZE);
    )
    {
      *dest++ = *src++;
    }

  /* TODO: make RAM FICR read-only */
}
#endif
