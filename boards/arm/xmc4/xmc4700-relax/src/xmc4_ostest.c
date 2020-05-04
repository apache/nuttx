/****************************************************************************
 * boards/arm/xmc4/xmc4700-relax/src/xmc4_ostest.c
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
#include <stdbool.h>
#include <string.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "arm_arch.h"
#include "arm_internal.h"
#include "xmc4700-relax.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#undef HAVE_FPU
#if defined(CONFIG_ARCH_FPU) && !defined(CONFIG_TESTING_OSTEST_FPUTESTDISABLE) && \
    defined(CONFIG_TESTING_OSTEST_FPUSIZE) && defined(CONFIG_SCHED_WAITPID)
#    define HAVE_FPU 1
#endif

#ifdef HAVE_FPU

#if CONFIG_TESTING_OSTEST_FPUSIZE != (4*SW_FPU_REGS)
#  error "CONFIG_TESTING_OSTEST_FPUSIZE has the wrong size"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_saveregs[XCPTCONTEXT_REGS];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Given an array of size CONFIG_TESTING_OSTEST_FPUSIZE, this function will
 * return the current FPU registers.
 */

void arch_getfpu(FAR uint32_t *fpusave)
{
  irqstate_t flags;

  /* Take a snapshot of the thread context right now */

  flags = enter_critical_section();
  arm_saveusercontext(g_saveregs);

  /* Return only the floating register values */

  memcpy(fpusave, &g_saveregs[REG_S0], (4*SW_FPU_REGS));
  leave_critical_section(flags);
}

/* Given two arrays of size CONFIG_TESTING_OSTEST_FPUSIZE this function
 * will compare them and return true if they are identical.
 */

bool arch_cmpfpu(FAR const uint32_t *fpusave1, FAR const uint32_t *fpusave2)
{
  return memcmp(fpusave1, fpusave2, (4*SW_FPU_REGS)) == 0;
}

#endif /* HAVE_FPU */
