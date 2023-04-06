/****************************************************************************
 * arch/sparc/src/sparc_v8/sparc_v8_registerdump.c
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
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "sparc_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_getusrsp
 ****************************************************************************/

uintptr_t up_getusrsp(void *regs)
{
  uint32_t *ptr = regs;
  return ptr[REG_I6];
}

/****************************************************************************
 * Name: up_dump_register
 ****************************************************************************/

void up_dump_register(void *dumpregs)
{
  volatile uint32_t *regs = dumpregs ? dumpregs : CURRENT_REGS;

  /* Are user registers available from interrupt processing? */

  if (regs)
    {
      _alert("R%d: %08x %08x %08x %08x %08x %08x %08x %08x\n",
            0,
            regs[REG_R16], regs[REG_R17],
            regs[REG_R18], regs[REG_R19],
            regs[REG_R20], regs[REG_R21],
            regs[REG_R22], regs[REG_R23]);

      _alert("R%d: %08x %08x %08x %08x %08x %08x %08x %08x\n",
            8,
            regs[REG_R24], regs[REG_R25],
            regs[REG_R26], regs[REG_R27],
            regs[REG_R28], regs[REG_R29],
            regs[REG_R30], regs[REG_R31]);

      _alert("SR: %08x\n", regs[REG_R14]);
    }
}
