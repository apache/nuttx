/****************************************************************************
 * arch/risc-v/src/nuttsbi/sbi_mtimer.c
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
#include <nuttx/compiler.h>
#include <nuttx/irq.h>

#include <arch/barriers.h>

#include <stdint.h>

#include "riscv_internal.h"

#include "sbi_internal.h"
#include "sbi_mcall.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Memory mapped register base addresses */

static volatile uintptr_t g_mtimecmp locate_data(".noinit");
static volatile uintptr_t g_mtime    locate_data(".noinit");

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void sbi_init_mtimer(uintptr_t mtime, uintptr_t mtimecmp)
{
  /* Set the memory mapped register bases */

  g_mtimecmp = mtimecmp;
  g_mtime    = mtime;
}

uint64_t sbi_get_mtime(void)
{
#ifdef CONFIG_ARCH_RV64
  return getreg64(g_mtime);
#else
  uint32_t hi;
  uint32_t lo;

  do
    {
      hi = getreg32(g_mtime + 4);
      lo = getreg32(g_mtime);
    }
  while (getreg32(g_mtime + 4) != hi);

  return ((uint64_t)hi << 32) | lo;
#endif
}

void sbi_set_mtimecmp(uint64_t value)
{
  uintptr_t mtimecmp = g_mtimecmp + READ_CSR(mhartid) * sizeof(uintptr_t);
#ifdef CONFIG_ARCH_RV64
  putreg64(value, mtimecmp);
#else
  putreg32(UINT32_MAX, mtimecmp + 4);
  putreg32(value, mtimecmp);
  putreg32(value >> 32, mtimecmp + 4);
#endif

  /* Make sure it sticks */

  __MB();
}
