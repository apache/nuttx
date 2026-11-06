/****************************************************************************
 * arch/tricore/src/common/tricore_main.c
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

#include <nuttx/init.h>

#include <arch/arch.h>

#include "tricore_internal.h"
#include "tricore_stm.h"
#include "chip.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_core_sync
 *
 * Description:
 *   Set the bit of this core in the sync word, atomically for the other
 *   cores, and wait until the bits of TRICORE_CORE_SYNC_MASK are set or the
 *   timeout on the STM of CPU0 has passed.
 *
 ****************************************************************************/

static void tricore_core_sync(volatile uint32_t *sync)
{
  uint32_t core = tricore_core_id();
#if TRICORE_CORE_SYNC_TIMEOUT > 0
  uint32_t start = getreg32(TRICORE_STM0_BASE + TRICORE_STM_TIM0_OFFSET);
#endif

#ifdef CONFIG_TRICORE_TOOLCHAIN_TASKING
  __imaskldmst((unsigned int *)sync, 1, core, 1);
#else
  long long tmp;

  __asm__ volatile ("imask %A0, %1, %2, 1\n\t"
                    "ldmst [%3]0, %A0"
                    : "=&d"(tmp) : "d"(1), "d"(core), "a"(sync) : "memory");
#endif

  while ((*sync & TRICORE_CORE_SYNC_MASK) != TRICORE_CORE_SYNC_MASK)
    {
#if TRICORE_CORE_SYNC_TIMEOUT > 0
      if (getreg32(TRICORE_STM0_BASE + TRICORE_STM_TIM0_OFFSET) - start >=
          TRICORE_CORE_SYNC_TIMEOUT)
        {
          break;
        }
#endif
    }
}

static void core_main(void)
{
  static volatile uint32_t g_sync_event;

#ifdef CONFIG_ARCH_HAVE_FPU
  tricore_fpuinit();
#endif

  /* !!WATCHDOG0 AND SAFETY WATCHDOG ARE DISABLED HERE!!
   * Enable the watchdogs and service them periodically if it is required
   */

  /* Wait for CPU sync event */

  tricore_core_sync(&g_sync_event);

  if (tricore_core_index() == 0)
    {
      tricore_earlyserialinit();
      nx_start();
    }

  while (1);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void core0_main(void)
{
  /* All WDTs except WDTCPU0 and system watchdog WDTSYS are in
   * disabled mode after Boot-FW execution. Disable the watchdog
   * to ensure the normal startup of the system.
   */

  tricore_wdt_disable();

  core_main();
}

void core1_main(void)
{
  core_main();
}

void core2_main(void)
{
  core_main();
}

void core3_main(void)
{
  core_main();
}

void core4_main(void)
{
  core_main();
}

void core5_main(void)
{
  core_main();
}

void core6_main(void)
{
  core_main();
}
