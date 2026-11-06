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
 * Public Data
 ****************************************************************************/

/* Entries of the other cores, provided by the linker script */

extern uintptr_t __start1[];
extern uintptr_t __start2[];
extern uintptr_t __start3[];
extern uintptr_t __start4[];
extern uintptr_t __start5[];

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uintptr_t * const g_tricore_entry[] =
{
  __start1, __start2, __start3, __start4, __start5
};

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

  __asm__ __volatile__ ("imask %A0, %1, %2, 1\n\t"
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_main
 *
 * Description:
 *   Entry into NuttX, called by tricore_start on CPU0: initialize the FPU
 *   and the MPU, wait for the other cores and start NuttX.
 *
 ****************************************************************************/

void tricore_main(void)
{
  static volatile uint32_t g_sync_event;

#ifdef CONFIG_ARCH_HAVE_FPU
  tricore_fpuinit();
#endif

#ifdef CONFIG_ARCH_USE_MPU
  tricore_mpuinit();
#endif

  /* !!WATCHDOG0 AND SAFETY WATCHDOG ARE DISABLED HERE!!
   * Enable the watchdogs and service them periodically if it is required
   */

  /* Wait for CPU sync event */

  tricore_core_sync(&g_sync_event);

  if (tricore_core_index() == 0)
    {
      tricore_earlyserialinit();
#ifdef CONFIG_BUILD_PROTECTED
      tricore_userspace();
#endif
      nx_start();
    }

  while (1);
}

/****************************************************************************
 * Name: tricore_start
 *
 * Description:
 *   C entry of the startup, jumped to from tricore_head on CPU0 once the
 *   core registers, the CSA list, the watchdogs, the RAMs, the C runtime
 *   and the clocks are set up.  Release CPU1 to TRICORE_START_CORES at
 *   __start1 and on, the start of their own images, as the iLLD startup
 *   code does, and enter NuttX.
 *
 ****************************************************************************/

void tricore_start(void)
{
  int cpu;

  for (cpu = 1; cpu <= TRICORE_START_CORES; cpu++)
    {
      uintptr_t pc = (uintptr_t)g_tricore_entry[cpu - 1];

      putreg32((getreg32(TRICORE_START_PC(cpu)) & 1) | (pc & ~1),
               TRICORE_START_PC(cpu));
      if ((getreg32(TRICORE_START_HALT(cpu)) & TRICORE_START_BHALT) != 0)
        {
          putreg32(getreg32(TRICORE_START_HALT(cpu)) & ~TRICORE_START_BHALT,
                   TRICORE_START_HALT(cpu));
        }
    }

  tricore_main();
}
