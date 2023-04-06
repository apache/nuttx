/****************************************************************************
 * arch/ceva/src/common/ceva_registerdump.c
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

#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "ceva_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_getusrsp
 ****************************************************************************/

uintptr_t up_getusrsp(void *regs)
{
  uint32_t *ptr = regs;
  return ptr[REG_SP];
}

/****************************************************************************
 * Name: up_dump_register
 ****************************************************************************/

void up_dump_register(void *dumpregs)
{
  volatile uint32_t *regs = dumpregs ? dumpregs : CURRENT_REGS;
  int rx;

  /* Dump the interrupt registers */

  for (rx = 0; rx < XCPTCONTEXT_REGS; rx += 8)
    {
      _alert("R%03d: %08x %08x %08x %08x %08x %08x %08x %08x\n",
             rx, regs[rx], regs[rx + 1], regs[rx + 2], regs[rx + 3],
             regs[rx + 4], regs[rx + 5], regs[rx + 6], regs[rx + 7]);
    }
}
