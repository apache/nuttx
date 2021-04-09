/****************************************************************************
 * arch/z80/src/z180/z180_irq.c
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

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <arch/io.h>

#include "switch.h"
#include "z180_iomap.h"
#include "z80_internal.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This holds a references to the current interrupt level register storage
 * structure.  If is non-NULL only during interrupt processing.
 */

volatile chipreg_t *g_current_regs;

/* This holds the value of the MMU's CBR register.  This value is set to the
 * interrupted tasks's CBR on interrupt entry, changed to the new task's CBR
 * if an interrupt level context switch occurs, and restored on interrupt
 * exit.  In this way, the CBR is always correct on interrupt exit.
 */

uint8_t current_cbr;

/* The interrupt vector table is exported by z180_vectors.asm or
 * z180_romvectors.asm with the name up_vectors:
 */

extern uintptr_t up_vectors[16];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z180_seti
 *
 * Description:
 *   Input byte from port p
 *
 ****************************************************************************/

static void z180_seti(uint8_t value) __naked
{
  __asm
  ld a, 4(ix) ; value
  ld l, a
  __endasm;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irq_save
 *
 * Description:
 *   Disable all interrupts; return previous interrupt state
 *
 ****************************************************************************/

irqstate_t up_irq_save(void) __naked
{
  __asm
  ld a, i ; AF parity bit holds interrupt state
  di ; interrupts are disabled
  push af; return AF in HL
  pop hl ;
  ret ;
  __endasm;
}

/****************************************************************************
 * Name: up_irq_restore
 *
 * Description:
 *   Restore previous interrupt state
 *
 ****************************************************************************/

void up_irq_restore(irqstate_t flags) __naked
{
  __asm
  di ; assume disabled
  pop hl ; HL = return address
  pop af ; AF parity bit holds interrupt state
  jp po, statedisable
  ei
statedisable:
  push af ; restore stack
  push hl ;
  ret ; and return
  __endasm;
}

/****************************************************************************
 * Name: up_irq_enable
 *
 * Description:
 *   Enable all interrupts; return previous interrupt state
 *
 ****************************************************************************/

irqstate_t up_irq_enable(void) __naked
{
  __asm
  ld a, i ; AF parity bit holds interrupt state
  ei ; interrupts are enabled
  push af ; return AF in HL
  pop hl ;
  ret ;
  __endasm;
}

/****************************************************************************
 * Name: up_irqinitialize
 *
 * Description:
 *   Initialize and enable interrupts
 *
 ****************************************************************************/

void up_irqinitialize(void)
{
  uint16_t vectaddr = (uint16_t)up_vectors;
  uint8_t regval;

  /* Initialize the I and IL registers so that the interrupt vector table
   * is used.
   */

  regval = (uint8_t)(vectaddr >> 8);
  z180_seti(regval);

  regval = (uint8_t)(vectaddr & IL_MASK);
  outp(Z180_INT_IL, regval);

  /* Disable external interrupts */

  outp(Z180_INT_ITC, 0);

  /* And finally, enable interrupts (including the timer) */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  up_irq_restore(Z180_C_FLAG);
#endif
}
