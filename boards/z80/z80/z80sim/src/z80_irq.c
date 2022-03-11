/****************************************************************************
 * boards/z80/z80/z80sim/src/z80_irq.c
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

#include <nuttx/irq.h>

#include "z80_internal.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int z80sim_timerisr(int irq, FAR chipreg_t *regs);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Attach the timer interrupt -- There is not special timer interrupt
   * enable in the simulation so it must be enabled here before interrupts
   * are enabled.
   *
   * NOTE:  Normally, there are separate enables for "global" interrupts
   * and specific device interrupts.  In such a "normal" case, the timer
   * interrupt should be attached and enabled in the function
   * up_timer_initialize()
   */

  irq_attach(Z80_IRQ_SYSTIMER, (xcpt_t)z80sim_timerisr, NULL);

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* And finally, enable interrupts (including the timer) */

  up_irq_restore(Z80_C_FLAG);
#endif
}
