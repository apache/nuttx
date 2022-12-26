/****************************************************************************
 * arch/avr/src/avr32/avr_registerdump.c
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
#include <stdlib.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "avr_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_getusrsp
 ****************************************************************************/

uintptr_t up_getusrsp(void)
{
  return g_current_regs[REG_R13];
}

/****************************************************************************
 * Name: up_dump_register
 ****************************************************************************/

void up_dump_register(void *dumpregs)
{
  volatile uint32_t *regs = dumpregs ? dumpregs : g_current_regs;

  /* Are user registers available from interrupt processing? */

  if (regs)
    {
      _alert("R%d: %08x %08x %08x %08x %08x %08x %08x %08x\n",
             0,
             regs[REG_R0], regs[REG_R1],
             regs[REG_R2], regs[REG_R3],
             regs[REG_R4], regs[REG_R5],
             regs[REG_R6], regs[REG_R7]);

      _alert("R%d: %08x %08x %08x %08x %08x %08x %08x %08x\n",
             8,
             regs[REG_R8],  regs[REG_R9],
             regs[REG_R10], regs[REG_R11],
             regs[REG_R12], regs[REG_R13],
             regs[REG_R14], regs[REG_R15]);

      _alert("SR: %08x\n", regs[REG_SR]);
    }
}
