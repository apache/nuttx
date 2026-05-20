/****************************************************************************
 * arch/tricore/src/common/tricore_fpu.c
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

#include <string.h>

#include "tricore_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_fpuinit
 ****************************************************************************/

void tricore_fpuinit(void)
{
  /* FPU zero-divide trap enable */

  __mtcr(FPU_SYNC_TRAP_REG,
         __mfcr(FPU_SYNC_TRAP_REG) | (1U << FPU_TRAP_FZE_SHIFT));
}

/****************************************************************************
 * Name: up_fpucmp
 ****************************************************************************/

bool up_fpucmp(const void *saveregs1, const void *saveregs2)
{
  const uintptr_t *regs1 = (const uintptr_t *)saveregs1 + TC_CONTEXT_REGS;
  const uintptr_t *regs2 = (const uintptr_t *)saveregs2 + TC_CONTEXT_REGS;

  /* TriCore uses D8-D15 in upper CSA as FPU data registers.
   * Skip A12-A15 (offsets 8-11) which naturally differ between saves.
   */

  if (memcmp(&regs1[REG_D8], &regs2[REG_D8],
             4 * sizeof(uintptr_t)) != 0)
    {
      return false;
    }

  return memcmp(&regs1[REG_D12], &regs2[REG_D12],
                4 * sizeof(uintptr_t)) == 0;
}
