/****************************************************************************
 * arch/arm/src/armv7-r/arm_prefetchabort.c
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
#include <debug.h>

#include <nuttx/irq.h>

#include "sched/sched.h"
#include "arm_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_prefetchabort
 *
 * Description:
 *   This is the prefetch abort exception handler. The ARM prefetch abort
 *   exception occurs when a memory fault is detected during an an
 *   instruction fetch.
 *
 ****************************************************************************/

uint32_t *arm_prefetchabort(uint32_t *regs, uint32_t ifar, uint32_t ifsr)
{
  /* Save the saved processor context in CURRENT_REGS where it can be
   * accessed for register dumps and possibly context switching.
   */

  CURRENT_REGS = regs;

  /* Crash -- possibly showing diagnostic debug information. */

  _alert("Prefetch abort. PC: %08x IFAR: %08x IFSR: %08x\n",
        regs[REG_PC], ifar, ifsr);
  PANIC();
  return regs; /* To keep the compiler happy */
}
