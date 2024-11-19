/****************************************************************************
 * arch/tricore/src/common/tricore_registerdump.c
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

#include <stdio.h>
#include <stdint.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "tricore_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_dump_register
 ****************************************************************************/

void up_dump_register(void *dumpregs)
{
  volatile uint32_t *regs = dumpregs;

  _alert("PCXI:%08x  PSW:%08x  SP:%08x  PC:%08x\n",
         regs[REG_UPCXI], regs[REG_PSW], regs[REG_A10], regs[REG_UA11]);
  _alert("D8:%08x    D9:%08x   D10:%08x D11:%08x\n",
         regs[REG_D8], regs[REG_D9], regs[REG_D10], regs[REG_D11]);
  _alert("A12:%08x   A13:%08x  A14:%08x A15:%08x\n",
        regs[REG_A12], regs[REG_A13], regs[REG_A14], regs[REG_A15]);
  _alert("D12:%08x   D13:%08x  D14:%08x D15:%08x\n\n",
         regs[REG_D12], regs[REG_D13], regs[REG_D14], regs[REG_D15]);
}
