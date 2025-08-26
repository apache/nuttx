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

#include "tricore_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
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

int up_saveusercontext(void *saveregs_)
{
  uintptr_t *saveregs = (uintptr_t *)saveregs_;
  uintptr_t *regs;
  uintptr_t pcxi;

  /* Save and copy the current lower csa */

  __asm("svlcx");

  pcxi = tricore_mfcr(TRICORE_CPU_PCXI);
  regs = tricore_csa2addr(pcxi);
  memcpy(saveregs, regs, TC_CONTEXT_SIZE);
  saveregs[REG_LPCXI] = tricore_addr2csa(saveregs + TC_CONTEXT_REGS)
                        | (pcxi & ~FCX_FREE);

  /* Copy the current upper csa */

  regs = tricore_csa2addr(regs[REG_LPCXI]);
  memcpy(saveregs + TC_CONTEXT_REGS, regs, TC_CONTEXT_SIZE);

  __asm("rslcx");

  return 0;
}
