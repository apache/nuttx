/****************************************************************************
 * arch/arm/src/armv7-a/arm_dofiq.c
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
#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "gic.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_dofiq
 *
 * Description:
 *   Receives the decoded GIC interrupt information and dispatches control
 *   to the attached fiq handler. It is not allowed to call OS functions
 *   within a FIQ handler.
 *
 ****************************************************************************/

uint32_t *arm_dofiq(int fiq, uint32_t *regs)
{
  board_autoled_on(LED_INIRQ);

#ifdef CONFIG_SUPPRESS_INTERRUPTS
  PANIC();
#else
  irq_dispatch(fiq, regs);
#endif

  board_autoled_off(LED_INIRQ);
  return regs;
}
