/****************************************************************************
 * arch/tricore/src/common/tricore_trapcall.c
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
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <syscall.h>

#include <arch/irq.h>
#include <sched/sched.h>
#include <nuttx/sched.h>

#include "tricore_internal.h"

#include "IfxCpu_Trap.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_trapcall
 *
 * Description:
 *   This is Trap exception handler
 *
 ****************************************************************************/

void tricore_trapcall(volatile void *trap)
{
  uintptr_t *regs;

  regs = tricore_csa2addr(__mfcr(CPU_PCXI));

  up_set_current_regs(regs);

  up_irq_save();
  PANIC_WITH_REGS("Trap", up_current_regs());
}
