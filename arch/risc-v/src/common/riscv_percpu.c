/****************************************************************************
 * arch/risc-v/src/common/riscv_percpu.c
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
#include <nuttx/irq.h>

#include <arch/barriers.h>
#include <arch/mode.h>

#include <assert.h>
#include <stdint.h>

#include "riscv_internal.h"
#include "riscv_percpu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HART_CNT    (CONFIG_ARCH_CPU_COUNT)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct riscv_percpu_s g_percpu[HART_CNT];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_percpu_init
 *
 * Description:
 *   Initialize the per CPU structures, should only be done on the boot
 *   hart.
 *
 ****************************************************************************/

void riscv_percpu_init(void)
{
  uintptr_t i;

  for (i = 0; i < HART_CNT; i++)
    {
      g_percpu[i].hartid = i;
    }
}

/****************************************************************************
 * Name: riscv_percpu_get_addr
 *
 * Description:
 *   Get add a hart to the per CPU area
 *
 * Input Parameters:
 *   hartid - Hart number
 *
 ****************************************************************************/

void riscv_percpu_add_hart(uintptr_t hartid)
{
  /* Hart IDs go from 0...4 */

  DEBUGASSERT(hartid < HART_CNT);

  /* Set the scratch register value to point to the scratch area */

  WRITE_CSR(CSR_SCRATCH, &g_percpu[hartid]);

  /* Make sure it sticks */

  __DMB();
}

/****************************************************************************
 * Name: riscv_percpu_get_hartid
 *
 * Description:
 *   Get harts own hartid by reading it from the per CPU area. This is safe
 *   to use from lower privilege modes (than M-mode).
 *
 * Returned Value:
 *   Hart id
 *
 ****************************************************************************/

uintptr_t riscv_percpu_get_hartid(void)
{
  uintptr_t scratch = READ_CSR(CSR_SCRATCH);

  DEBUGASSERT(scratch >= (uintptr_t) &g_percpu &&
              scratch < (uintptr_t) &g_percpu + sizeof(g_percpu));

  return ((struct riscv_percpu_s *)scratch)->hartid;
}
