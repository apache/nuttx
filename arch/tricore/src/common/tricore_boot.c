/****************************************************************************
 * arch/tricore/src/common/tricore_boot.c
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with
 * the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/init.h>

#include <arch/arch.h>
#include <arch/chip/chip.h>

#include "tricore_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Kick another core by setting its entry address and clear halt flag */

#define tricore_start_core(base, start)                                \
  do                                                                   \
    {                                                                  \
      putreg32((uint32_t)(uintptr_t)(start),                           \
               (base) + CPU_SFR_PC_OFFSET);                            \
      UP_DSB();                                                        \
      modreg32(0, CPU_SFR_HALT_BHALT, (base) + CPU_SFR_HALT_OFFSET);   \
      UP_DSB();                                                        \
      up_udelay(100);                                                  \
    }                                                                  \
  while (0)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Common entry of all cpus. */

void __start(void);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_ARCH_RELEASE_CORES

/* CPU module SFR base address of each core */

static const uint32_t g_cpu_module[] = TRICORE_CPU_MODULE_BASES;
#endif

/****************************************************************************
 * Name: tricore_start_cores
 ****************************************************************************/

#ifdef CONFIG_ARCH_RELEASE_CORES
static void tricore_start_cores(void)
{
  int i;

  if (up_cpu_index() != 0)
    {
      return;
    }

  for (i = 1; i < CONFIG_SMP_NCPUS; i++)
    {
      tricore_start_core(g_cpu_module[i], __start);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_boot_primary
 *
 * Description:
 *   C entry of the primary core, called from tricore_head.S once the
 *   C environment is up.  Never returns.
 *
 ****************************************************************************/

void tricore_boot_primary(void)
{
  tricore_boot();

#ifdef CONFIG_ARCH_HAVE_FPU
  tricore_fpuinit();
#endif

#ifdef CONFIG_ARCH_USE_MPU
  tricore_mpuinit();
#endif

#ifdef CONFIG_ARCH_PERF_EVENTS
  up_perf_init((void *)CONFIG_CPU_CLOCK_FREQUENCY);
#endif

#ifdef USE_EARLYSERIALINIT
  tricore_earlyserialinit();
#endif

#ifdef CONFIG_BUILD_PROTECTED
  tricore_userspace();
#endif

  tricore_mtcr(CPU_PSW, TRICORE_PSW_DEFAULT);
  UP_ISB();

  /* Release other cores */

#ifdef CONFIG_ARCH_RELEASE_CORES
  tricore_start_cores();
#endif

  nx_start();
}

/****************************************************************************
 * Name: tricore_boot_secondary
 *
 * Description:
 *   C entry of the secondary cores, called from tricore_head.S once the
 *   C environment is up.  Never returns.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
void tricore_boot_secondary(void)
{
#ifdef CONFIG_ARCH_HAVE_FPU
  tricore_fpuinit();
#endif

#ifdef CONFIG_ARCH_USE_MPU
  tricore_mpuinit();
#endif

#ifdef CONFIG_ARCH_PERF_EVENTS
  up_perf_init((void *)CONFIG_CPU_CLOCK_FREQUENCY);
#endif

  tricore_cpu_boot(up_cpu_index());

#ifdef CONFIG_BUILD_PROTECTED
  tricore_userspace();
#endif

  tricore_mtcr(CPU_PSW, TRICORE_PSW_DEFAULT);
  UP_ISB();

  nx_start();
}
#endif
