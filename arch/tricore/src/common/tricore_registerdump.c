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
#include <arch/arch.h>

#include "tricore_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_dump_register
 *
 * Description:
 *   Dump the contents of all general-purpose registers from the saved
 *   context. The supplied pointer points at the lower CSA captured by
 *   svlcx in the trap entry path. The upper CSA (containing a10-a15,
 *   d8-d15, PSW) is reached through the PCXI chain field of the lower
 *   CSA.
 *
 ****************************************************************************/

void up_dump_register(void *dumpregs)
{
  uintptr_t *lower = dumpregs;
  uintptr_t *upper;

  if (lower == NULL)
    {
      return;
    }

  upper = tricore_csa2addr(lower[REG_LPCXI]);

  lowsyslog("A2:  %08lx A3:  %08lx A4:  %08lx A5:  %08lx\n",
         (unsigned long)lower[REG_A2], (unsigned long)lower[REG_A3],
         (unsigned long)lower[REG_A4], (unsigned long)lower[REG_A5]);
  lowsyslog("A6:  %08lx A7:  %08lx A10: %08lx A11: %08lx\n",
         (unsigned long)lower[REG_A6], (unsigned long)lower[REG_A7],
         (unsigned long)upper[REG_A10], (unsigned long)upper[REG_UA11]);
  lowsyslog("A12: %08lx A13: %08lx A14: %08lx A15: %08lx\n",
         (unsigned long)upper[REG_A12], (unsigned long)upper[REG_A13],
         (unsigned long)upper[REG_A14], (unsigned long)upper[REG_A15]);
  lowsyslog("D0:  %08lx D1:  %08lx D2:  %08lx D3:  %08lx\n",
         (unsigned long)lower[REG_D0], (unsigned long)lower[REG_D1],
         (unsigned long)lower[REG_D2], (unsigned long)lower[REG_D3]);
  lowsyslog("D4:  %08lx D5:  %08lx D6:  %08lx D7:  %08lx\n",
         (unsigned long)lower[REG_D4], (unsigned long)lower[REG_D5],
         (unsigned long)lower[REG_D6], (unsigned long)lower[REG_D7]);
  lowsyslog("D8:  %08lx D9:  %08lx D10: %08lx D11: %08lx\n",
         (unsigned long)upper[REG_D8], (unsigned long)upper[REG_D9],
         (unsigned long)upper[REG_D10], (unsigned long)upper[REG_D11]);
  lowsyslog("D12: %08lx D13: %08lx D14: %08lx D15: %08lx\n",
         (unsigned long)upper[REG_D12], (unsigned long)upper[REG_D13],
         (unsigned long)upper[REG_D14], (unsigned long)upper[REG_D15]);
  lowsyslog("PC:  %08lx SP:  %08lx PSW: %08lx PCXI:%08lx\n",
         (unsigned long)upper[REG_UPC], (unsigned long)upper[REG_SP],
         (unsigned long)upper[REG_PSW], (unsigned long)upper[REG_UPCXI]);
}
