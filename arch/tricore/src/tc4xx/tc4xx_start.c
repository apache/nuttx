/****************************************************************************
 * arch/tricore/src/tc4xx/tc4xx_start.c
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
#include "hardware/tc4xx_clock.h"
#include "hardware/tc4xx_cpu.h"
#include "hardware/tc4xx_pms.h"
#include "hardware/tc4xx_smm.h"
#include "hardware/tc4xx_vmt.h"
#include "tc4xx_wdt.h"
#include "tricore_stm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Watchdog reload values of the startup and of the application */

#define TC4XX_WDT_RELOAD_STARTUP   0xffc2
#define TC4XX_WDT_RELOAD_APP       0xfffc

/* STM of CPU0 */

#define TC4XX_STM0_ABS  (TRICORE_STM0_BASE + TRICORE_STM_ABS_OFFSET)

/* The BIV of the single interrupt vector at INTTAB + 0x1fe0 */

#define TC4XX_BIV_VECTOR           0x1fe0

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
extern uintptr_t __INTTAB_CPU0[];
extern uintptr_t __ISTACK0[];
extern uintptr_t __START1[];
extern uintptr_t __START2[];
extern uintptr_t __START3[];
extern uintptr_t __START4[];
extern uintptr_t __START5[];

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* MBIST selectors of the LMU and MCAN RAMs that are cleared */

static const uint8_t g_tc4xx_sram[] =
{
  0x17, 0x18, 0x20, 0x21, 0x19, 0x1a, 0x1b, 0x22, 0x23, 0x24, /* LMU0-9 */
  0xa2, 0xa3, 0xa4, 0xa5, 0xa6                                /* MCAN0-4 */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tc4xx_sram_clear
 *
 * Description:
 *   Clear an SRAM with its MBIST controller, as IfxVmt_clearSram does.
 *
 ****************************************************************************/

static void tc4xx_sram_clear(uint32_t sel)
{
  uintptr_t vmt = TC4XX_VMT_BASE + (sel >> 5) * TC4XX_VMT_SIZE;
  uintptr_t mc = vmt + TC4XX_VMT_MC(sel % 32);
  uint32_t mask = 1 << (sel % 32);

  if ((getreg32(vmt + TC4XX_VMT_CLC) & TC4XX_VMT_CLC_DISS) != 0)
    {
      putreg32(0, vmt + TC4XX_VMT_CLC);
    }

  /* Enable the MBIST shell and start the clear */

  putreg32(getreg32(mc + TC4XX_VMT_MC_ALMSRCS) & ~TC4XX_ALMSRCS_ALARMS,
           mc + TC4XX_VMT_MC_ALMSRCS);
  putreg32(getreg32(vmt + TC4XX_VMT_MEMTEST) | mask,
           vmt + TC4XX_VMT_MEMTEST);
  while ((getreg32(vmt + TC4XX_VMT_MEMSTAT) & mask) != 0);

  putreg32((1 << 15) | (1 << 0), mc + TC4XX_VMT_MC_MCONTROL);
  putreg32(getreg32(mc + TC4XX_VMT_MC_MCONTROL) & ~(1 << 0),
           mc + TC4XX_VMT_MC_MCONTROL);
  while ((getreg32(vmt + TC4XX_VMT_MEMDONE) & mask) == 0);

  /* Read address 0 once, then disable the MBIST shell again */

  putreg32(1 << 3, mc + TC4XX_VMT_MC_MCONTROL);
  putreg32((1 << 12) | (1 << 0), mc + TC4XX_VMT_MC_CONFIG0);
  putreg32(0, mc + TC4XX_VMT_MC_CONFIG1);
  putreg32(0, mc + TC4XX_VMT_MC_RANGE);
  putreg32((1 << 3) | (1 << 0), mc + TC4XX_VMT_MC_MCONTROL);
  putreg32(1 << 3, mc + TC4XX_VMT_MC_MCONTROL);
  while ((getreg32(vmt + TC4XX_VMT_MEMDONE) & mask) == 0);

  putreg32(getreg32(vmt + TC4XX_VMT_MEMTEST) & ~mask,
           vmt + TC4XX_VMT_MEMTEST);
  while ((getreg32(vmt + TC4XX_VMT_MEMSTAT) & mask) != 0);
  putreg32(getreg32(mc + TC4XX_VMT_MC_ALMSRCS) | TC4XX_ALMSRCS_ALARMS,
           mc + TC4XX_VMT_MC_ALMSRCS);
}

/****************************************************************************
 * Name: tc4xx_stm_frequency
 *
 * Description:
 *   STM frequency as the iLLD startup code computes it for its delays.
 *
 ****************************************************************************/

static uint32_t tc4xx_stm_frequency(void)
{
  uint32_t insel = (getreg32(TC4XX_CLOCK_OSCCON) &
                    TC4XX_OSCCON_INSEL_MASK) >> TC4XX_OSCCON_INSEL_SHIFT;
  uint32_t clksels = getreg32(TC4XX_CLOCK_CCUSTAT) & TC4XX_CCU_CLKSELS_MASK;
  uint32_t con0 = getreg32(TC4XX_CLOCK_SYSPLLCON0);
  uint32_t stat = getreg32(TC4XX_CLOCK_RAMPSTAT);
  uint32_t fstat = (stat & TC4XX_RAMPSTAT_FSTAT_MASK) >>
                   TC4XX_RAMPSTAT_FSTAT_SHIFT;
  uint32_t osc;
  uint32_t freq;

  osc = insel == 2 ? 20000000 : insel == 1 ? 25000000 : 100000000;
  if (clksels == TC4XX_CCU_CLKSELS_PLL)
    {
      uint32_t ndiv = (con0 & TC4XX_PLLCON0_NDIV_MASK) >>
                      TC4XX_PLLCON0_NDIV_SHIFT;
      uint32_t pdiv = (con0 & TC4XX_PLLCON0_PDIV_MASK) >>
                      TC4XX_PLLCON0_PDIV_SHIFT;
      uint32_t k2div = getreg32(TC4XX_CLOCK_SYSPLLCON1) &
                       TC4XX_PLLCON1_K2DIV_MASK;

      freq = (uint32_t)((float)osc * (ndiv + 1) /
                        ((pdiv + 1) * (k2div + 1)) + 0.5f);
    }
  else if (clksels == TC4XX_CCU_CLKSELS_RAMP)
    {
      freq = 0;
      if ((stat & TC4XX_RAMPSTAT_SSTAT_MASK) == 0)
        {
          if (fstat == 2)
            {
              freq = (getreg32(TC4XX_CLOCK_RAMPCON0) &
                      TC4XX_RAMPCON0_UFL_MASK) * 1000000;
            }
          else if (fstat == 1)
            {
              freq = 100000000;
            }
        }
    }
  else
    {
      freq = 100000000;
    }

  return freq / ((getreg32(TC4XX_CLOCK_SYSCCUCON0) &
                  TC4XX_SYSCCUCON0_STMDIV_MASK) >>
                 TC4XX_SYSCCUCON0_STMDIV_SHIFT);
}

/****************************************************************************
 * Name: tc4xx_delay
 *
 * Description:
 *   Wait 100 us on the STM between the starts of the cores.
 *
 ****************************************************************************/

static void tc4xx_delay(void)
{
  uint32_t ticks = tc4xx_stm_frequency() / 10000;
  uint32_t start = getreg32(TC4XX_STM0_ABS);

  while (getreg32(TC4XX_STM0_ABS) - start < ticks);
}

/****************************************************************************
 * Name: tc4xx_start_core
 ****************************************************************************/

static void tc4xx_start_core(int cpu, uintptr_t pc)
{
  putreg32((getreg32(TC4XX_CPU_HRA_PC(cpu)) & 1) | (pc & ~1),
           TC4XX_CPU_HRA_PC(cpu));
  if ((getreg32(TC4XX_CPU_HRA_BOOTCON(cpu)) & TC4XX_BOOTCON_BHALT) != 0)
    {
      putreg32(getreg32(TC4XX_CPU_HRA_BOOTCON(cpu)) & ~TC4XX_BOOTCON_BHALT,
               TC4XX_CPU_HRA_BOOTCON(cpu));
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_start
 *
 * Description:
 *   C entry of the startup, jumped to from __start in tricore_head after it
 *   has disabled the virtualization and set the stack and the small data
 *   pointers.  It does what the iLLD startup code did for CPU0: link the
 *   CSAs, and after a power-on reset clear the LMU and MCAN RAMs and set
 *   up the crystal and the clocks; initialize the C runtime, start the
 *   other cores, enable the caches, set BIV and
 *   ISP and enter core0_main.  An application reset with the key-off
 *   marker set stops, as the iLLD key-off path does without its tests.
 *
 ****************************************************************************/

void tricore_start(void)
{
  int i;

  tricore_csa_init(__CSA0, __CSA0_END);
  UP_ISB();

  tc4xx_wdt_service();

  if ((getreg32(TC4XX_SMM_RSTSTATH) & TC4XX_RSTSTATH_APPRST) == 0)
    {
      tc4xx_wdt_service();
      tc4xx_wdt_reload(TC4XX_WDT_RELOAD_STARTUP);

      for (i = 0; i < sizeof(g_tc4xx_sram); i++)
        {
          tc4xx_sram_clear(g_tc4xx_sram[i]);
        }

      tricore_data_init();

      tc4xx_wdt_service();
      if (tricore_clockconfig() != 0)
        {
          __asm__ __volatile__ ("debug");
        }
    }
  else if ((getreg32(TC4XX_PMS_CLDPORSTUSRINFO) & 0xff) != 0)
    {
      tc4xx_wdt_reload(TC4XX_WDT_RELOAD_STARTUP);
      putreg32(0, TC4XX_PMS_CLDPORSTUSRINFO);
      for (; ; );
    }
  else
    {
      tricore_data_init();
    }

  tc4xx_wdt_service();

  tc4xx_start_core(1, (uintptr_t)__START1);
  tc4xx_delay();
  tc4xx_start_core(2, (uintptr_t)__START2);
  tc4xx_delay();
  tc4xx_start_core(3, (uintptr_t)__START3);
  tc4xx_delay();
  tc4xx_start_core(4, (uintptr_t)__START4);
  tc4xx_delay();
  tc4xx_start_core(5, (uintptr_t)__START5);

  /* Caches on, interrupt vector, interrupt stack */

  tricore_mtcr(TRICORE_CPU_PCON0, 0);
  UP_ISB();
  tricore_mtcr(TRICORE_CPU_DCON0, 0);
  UP_ISB();
  tricore_mtcr(TRICORE_CPU_BIV, (uintptr_t)__INTTAB_CPU0 | TC4XX_BIV_VECTOR);
  UP_ISB();
  tricore_mtcr(TRICORE_CPU_ISP, (uintptr_t)__ISTACK0);
  UP_ISB();

  tc4xx_wdt_reload(TC4XX_WDT_RELOAD_APP);

  core0_main();

  for (; ; );
}
