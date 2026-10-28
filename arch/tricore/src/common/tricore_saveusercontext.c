/****************************************************************************
 * arch/tricore/src/common/tricore_saveusercontext.c
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

#include <arch/arch.h>

#include "tricore_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_saveusercontext
 *
 * Description:
 *   Save the current thread context
 *
 ****************************************************************************/

int up_saveusercontext(void *saveregs)
{
  uintptr_t *regs;
  uintptr_t  pcxi;
  uintptr_t  lpcxi;
  int csa_size = TC_CONTEXT_REGS * sizeof(uintptr_t);

  TRICORE_MFCR(TRICORE_CPU_PCXI, pcxi);
  regs = tricore_csa2addr(pcxi);
  memcpy((char *)saveregs + csa_size, regs, csa_size);

  __asm__ __volatile__("svlcx" ::: "memory");

  TRICORE_MFCR(TRICORE_CPU_PCXI, lpcxi);
  regs = tricore_csa2addr(lpcxi);
  memcpy(saveregs, regs, csa_size);

  /* Lower CSA link normally points to the upper CSA; here both halves sit
   * contiguously in the buffer, so repurpose it to carry the upper PCXI.
   */

  ((uintptr_t *)saveregs)[REG_LPCXI] = pcxi & (~PCXI_UL);

  __asm__ __volatile__("rslcx" ::: "memory");
  return 0;
}
