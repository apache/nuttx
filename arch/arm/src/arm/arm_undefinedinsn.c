/****************************************************************************
 * arch/arm/src/arm/arm_undefinedinsn.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <sched/sched.h>

#include "arm_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_undefinedinsn
 ****************************************************************************/

void arm_undefinedinsn(uint32_t *regs)
{
  struct tcb_s *tcb = this_task();

  tcb->xcp.regs = regs;
  up_set_interrupt_context(true);

  if (regs[REG_PC] >= (uint32_t)_stext && regs[REG_PC] < (uint32_t)_etext)
    {
      _alert("Undefined instruction at 0x%" PRIx32 ": 0x%" PRIx32 "\n",
             regs[REG_PC], *(uint32_t *)regs[REG_PC]);
    }
  else
    {
      _alert("Undefined instruction at 0x%" PRIx32 "\n", regs[REG_PC]);
    }

  PANIC_WITH_REGS("panic", regs);
}
