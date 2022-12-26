/****************************************************************************
 * arch/misoc/src/minerva/minerva_registerdump.c
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

#include "minerva.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_getusrsp
 ****************************************************************************/

uintptr_t up_getusrsp(void)
{
  return g_current_regs[REG_X2];
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
      _alert("EPC:%08x\n", regs[REG_CSR_MEPC]);
      _alert(" X0:%08x  A0:%08x  A1:%08x  A2:%08x "
             " A3:%08x  A4:%08x  A5:%08x  A6:%08x\n",
             regs[REG_X0_NDX], regs[REG_X1_NDX],
             regs[REG_X2_NDX], regs[REG_X3_NDX],
             regs[REG_X4_NDX], regs[REG_X5_NDX],
             regs[REG_X6_NDX], regs[REG_X7_NDX]);
      _alert(" A7:%08x  X9:%08x X10:%08x X11:%08x "
             "X12:%08x X13:%08x X14:%08x X15:%08x\n",
             regs[REG_X8_NDX], regs[REG_X9_NDX],
             regs[REG_X10_NDX], regs[REG_X11_NDX],
             regs[REG_X12_NDX], regs[REG_X13_NDX],
             regs[REG_X14_NDX], regs[REG_X15_NDX]);
      _alert("X16:%08x X17:%08x X18:%08x X19:%08x"
             "X20:%08x X21:%08x X22:%08x X23:%08x\n",
             regs[REG_X16_NDX], regs[REG_X17_NDX],
             regs[REG_X18_NDX], regs[REG_X19_NDX],
             regs[REG_X20_NDX], regs[REG_X21_NDX],
             regs[REG_X22_NDX], regs[REG_X23_NDX]);
      _alert("X24:%08x X25:%08x  GP:%08x  FP:%08x "
             " SP:%08x  RA:%08x  EA:%08x  BA:%08x\n",
             regs[REG_X24_NDX], regs[REG_X25_NDX],
             regs[REG_X26_NDX], regs[REG_X27_NDX],
             regs[REG_X28_NDX], regs[REG_X29_NDX],
             regs[REG_X30_NDX], regs[REG_X31_NDX]);
      _alert(" IE:%08x\n", regs[REG_CSR_MSTATUS]);
    }
}
