/****************************************************************************
 * arch/tricore/src/common/tricore_start.c
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

#include "tricore_internal.h"
#include "chip.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Entries of the other cores, provided by the linker script.  With SMP the
 * other cores run this image instead, up_cpu_start() starts them.
 */

#ifndef CONFIG_SMP
extern uintptr_t __start1[];
extern uintptr_t __start2[];
extern uintptr_t __start3[];
extern uintptr_t __start4[];
extern uintptr_t __start5[];
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef CONFIG_SMP
static uintptr_t * const g_tricore_entry[] =
{
  __start1, __start2, __start3, __start4, __start5
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
#ifndef CONFIG_SMP
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
#endif

  tricore_main();
}
