/****************************************************************************
 * arch/risc-v/src/common/riscv_mtimer.c
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
#include <nuttx/arch.h>

#include <arch/barriers.h>

#include <stdint.h>

#include "riscv_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define is_lt_u64(a, b)     ((int64_t)((a) - (b)) < 0)
#define is_gtoreq_u64(a, b) !is_lt_u64(a, b)

#ifdef CONFIG_ARCH_RV32
#  warning "Missing logic... Access to 64-bit registers needs fixing"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Memory mapped register base addresses */

static volatile uintptr_t g_mtimecmp_base;
static volatile uintptr_t g_mtime;

/* Amount of M-mode ticks for one OS tick */

static volatile uint64_t  g_tick;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void riscv_init_mtimer(uintptr_t mcmpbase, uintptr_t mtime, uint64_t tick)
{
  /* Set the memory mapped register bases */

  g_mtimecmp_base = mcmpbase;
  g_mtime         = mtime;
  g_tick          = tick;
}

void riscv_reload_mtimecmp(void)
{
  uintptr_t hartid   = READ_CSR(mhartid);
  uintptr_t mtimecmp = (uintptr_t)((uintptr_t *)g_mtimecmp_base + hartid);
  uint64_t  value    = getreg64(mtimecmp);
  uint64_t  time     = getreg64(g_mtime);
  uint64_t  tick     = g_tick;

  /* Reload time and make sure the time is in the future. This is ESSENTIAL
   * as the timer will keep interrupting as long as time >= mtimecmp, thus
   * if the time is very far in the future, the system will be starved as
   * we will be servicing this ISR over and over.
   */

  if (is_lt_u64(time + tick, value))
    {
      /* Normal case when booting, mtimecmp can point far into the future */

      value = time + tick;
    }
  else if (is_lt_u64(value + tick, time))
    {
      /* Handle case where 1 (or more) ticks is missed */

      value = time + tick;
    }
  else
    {
      /* Keep ticking. To avoid bias, use previous tick time as base */

      value = value + tick;
    }

  /* Write the value back */

  putreg64(value, mtimecmp);

  /* Make sure it sticks */

  __DMB();
}
