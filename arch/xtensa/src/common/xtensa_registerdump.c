/****************************************************************************
 * arch/xtensa/src/common/xtensa_registerdump.c
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

#include "xtensa.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_getusrsp
 ****************************************************************************/

uintptr_t up_getusrsp(void)
{
  return CURRENT_REGS[REG_A1];
}

/****************************************************************************
 * Name: up_dump_register
 ****************************************************************************/

void up_dump_register(void *dumpregs)
{
  volatile uintptr_t *regs = dumpregs ? dumpregs :
                            (uintptr_t *)CURRENT_REGS;

  _alert("   PC: %08lx    PS: %08lx\n",
         (unsigned long)regs[REG_PC], (unsigned long)regs[REG_PS]);
  _alert("   A0: %08lx    A1: %08lx    A2: %08lx    A3: %08lx\n",
         (unsigned long)regs[REG_A0], (unsigned long)regs[REG_A1],
         (unsigned long)regs[REG_A2], (unsigned long)regs[REG_A3]);
  _alert("   A4: %08lx    A5: %08lx    A6: %08lx    A7: %08lx\n",
         (unsigned long)regs[REG_A4], (unsigned long)regs[REG_A5],
         (unsigned long)regs[REG_A6], (unsigned long)regs[REG_A7]);
  _alert("   A8: %08lx    A9: %08lx   A10: %08lx   A11: %08lx\n",
         (unsigned long)regs[REG_A8], (unsigned long)regs[REG_A9],
         (unsigned long)regs[REG_A10], (unsigned long)regs[REG_A11]);
  _alert("  A12: %08lx   A13: %08lx   A14: %08lx   A15: %08lx\n",
         (unsigned long)regs[REG_A12], (unsigned long)regs[REG_A13],
         (unsigned long)regs[REG_A14], (unsigned long)regs[REG_A15]);
  _alert("  SAR: %08lx CAUSE: %08lx VADDR: %08lx\n",
         (unsigned long)regs[REG_SAR], (unsigned long)regs[REG_EXCCAUSE],
         (unsigned long)regs[REG_EXCVADDR]);
#if XCHAL_HAVE_LOOPS != 0
  _alert(" LBEG: %08lx  LEND: %08lx  LCNT: %08lx\n",
         (unsigned long)regs[REG_LBEG], (unsigned long)regs[REG_LEND],
         (unsigned long)regs[REG_LCOUNT]);
#endif
}
