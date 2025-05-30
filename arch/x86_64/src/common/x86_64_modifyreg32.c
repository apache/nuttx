/****************************************************************************
 * arch/x86_64/src/common/x86_64_modifyreg32.c
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
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>

#include "x86_64_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static spinlock_t g_modifyreg_lock = SP_UNLOCKED;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: modifyreg32
 *
 * Description:
 *   Atomically modify the specified bits in a memory mapped register
 *
 ****************************************************************************/

void modifyreg32(unsigned int addr, uint32_t clearbits, uint32_t setbits)
{
  irqstate_t flags;
  uint32_t   regval;

  flags   = spin_lock_irqsave(&g_modifyreg_lock);
  regval  = getreg32((uint16_t)addr);
  regval &= ~clearbits;
  regval |= setbits;
  putreg32(regval, (uint16_t)addr);
  spin_unlock_irqrestore(&g_modifyreg_lock, flags);
}
