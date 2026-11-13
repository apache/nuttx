/****************************************************************************
 * arch/tricore/src/tc3xx/tc3xx_start.c
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

#include <stdbool.h>
#include <stdint.h>

#include <arch/arch.h>

#include "tricore_internal.h"
#include "tricore_start.h"
#include "hardware/tc3xx_cpu.h"
#include "hardware/tc3xx_scu.h"
#include "tc3xx_wdt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The BIV of the single interrupt vector at INTTAB + 0x1fe0 */

#define TC3XX_BIV_VECTOR           0x1fe0

/****************************************************************************
 * External Function Prototypes
 ****************************************************************************/

void core0_main(void);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Provided by the linker script */

extern uintptr_t __CSA0[];
extern uintptr_t __CSA0_END[];
extern uintptr_t __TRAPTAB_CPU0[];
extern uintptr_t __INTTAB_CPU0[];
extern uintptr_t __ISTACK0[];
extern uintptr_t __START1[];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tc3xx_application_reset
 ****************************************************************************/

static bool tc3xx_application_reset(void)
{
  uint32_t rststat = getreg32(TC3XX_SCU_RSTSTAT);
  uint32_t bit;

  if ((rststat & TC3XX_RSTSTAT_POWERON) != 0)
    {
      return false;
    }

  if ((rststat & TC3XX_RSTSTAT_RESET) != 0)
    {
      rststat &= TC3XX_RSTSTAT_RESET;
      for (bit = 31; (rststat & (1u << bit)) == 0; bit--);

      return ((getreg32(TC3XX_SCU_RSTCON) >> (bit << 1)) & 3) ==
             TC3XX_RSTCON_APPLICATION;
    }

  if ((rststat & TC3XX_RSTSTAT_CB3) != 0)
    {
      return true;
    }

  return (getreg32(TC3XX_CPU0_KRST0) & TC3XX_KRST0_RSTSTAT_MASK) != 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_start
 *
 * Description:
 *   C entry of the startup, jumped to from __start in tricore_head with the
 *   stack and the small data pointers set.  It does what the iLLD startup
 *   code did for CPU0: link the CSAs, set up the clocks after a power-on
 *   reset, initialize the C runtime with the watchdogs disabled, start
 *   CPU1, enable the caches, set BTV, BIV and ISP and enter core0_main.
 *
 ****************************************************************************/

void tricore_start(void)
{
  tricore_csa_init(__CSA0, __CSA0_END);
  UP_ISB();

  if (!tc3xx_application_reset())
    {
      tc3xx_wdt_service();
      if (tricore_clockconfig() != 0)
        {
          __asm__ volatile ("debug");
        }
    }

  tc3xx_wdt_set_disabled(true);
  tricore_data_init();
  tc3xx_wdt_set_disabled(false);

  putreg32((getreg32(TC3XX_CPU1_PC) & 1) | ((uintptr_t)__START1 & ~1),
           TC3XX_CPU1_PC);
  if ((getreg32(TC3XX_CPU1_SYSCON) & TC3XX_SYSCON_BHALT) != 0)
    {
      putreg32(getreg32(TC3XX_CPU1_SYSCON) & ~TC3XX_SYSCON_BHALT,
               TC3XX_CPU1_SYSCON);
    }

  /* Caches on, trap and interrupt vectors, interrupt stack */

  tc3xx_wdt_cpu_endinit(false);
  tricore_mtcr(TRICORE_CPU_PCON0, 0);
  UP_ISB();
  tricore_mtcr(TRICORE_CPU_DCON0, 0);
  UP_ISB();
  tricore_mtcr(TRICORE_CPU_BTV, (uintptr_t)__TRAPTAB_CPU0);
  tricore_mtcr(TRICORE_CPU_BIV, (uintptr_t)__INTTAB_CPU0 | TC3XX_BIV_VECTOR);
  tricore_mtcr(TRICORE_CPU_ISP, (uintptr_t)__ISTACK0);
  tc3xx_wdt_cpu_endinit(true);

  core0_main();

  for (; ; );
}
