/****************************************************************************
 * boards/z80/ez80/ez80f910200zco/src/ez80_buttons.c
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

#include <nuttx/irq.h>
#include <nuttx/board.h>

#include "chip.h"
#include "z80_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_PB/1/2interrupt
 *
 * Description:
 *   These could be extended to provide interrupt driven button input
 *
 ****************************************************************************/

#if 0
void ez80_pbinterrupt(void)
{
  uint8_t regval;

  regval = inp(EZ80_PB_DR); /* Clear interrupt flag for eZ80F91 date codes before 0611 */
  regval |= 7;
  outp(EZ80_PB_DR, regval);

  regval = inp(EZ80_PB_ALT0); /* Clear interrupt flag for eZ80F91 date codes 0611 and after */
  regval |= 1;
  outp(EZ80_PB_ALT0, regval);
}

void ez80_pb1interrupt(void)
{
  uint8_t regval;

  regval = inp(EZ80_PB_DR); /* Clear interrupt flag for eZ80F91 date codes before 0611 */
  regval |= 7;
  outp(EZ80_PB_DR, regval);

  regval = inp(EZ80_PB_ALT0); /* Clear interrupt flag for eZ80F91 date codes 0611 and after */
  regval |= 2;
  outp(EZ80_PB_ALT0, regval);
}

void ez80_pb2interrupt(void)
{
  uint8_t regval;

  regval = inp(EZ80_PB_DR); /* Clear interrupt flag for eZ80F91 date codes
                             * before 0611 */
  regval |= 7;
  outp(EZ80_PB_DR, regval);

  regval = inp(EZ80_PB_ALT0);
  regval |= 4;
  outp(EZ80_PB_ALT0, regval); /* Clear interrupt flag for eZ80F91 date codes
                               * 0611 and after */
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_initialize
 ****************************************************************************/

#ifdef CONFIG_ARCH_BUTTONS
uint32_t board_button_initialize(void)
{
  uint8_t regval;

#if 0 /* Interrupts are not used */

  /* Attach GIO interrupts */

  irq_attach(EZ80_PB_IRQ, ez80_pbinterrupt, NULL);
  irq_attach(EZ80_PB1_IRQ, ez80_pb1interrupt, NULL);
  irq_attach(EZ80_PB2_IRQ, ez80_pb2interrupt, NULL);

  /* Configure PB0,1,2 as interrupt, rising edge */

  regval = inp(EZ80_PB_DR);
  regval |= 7;
  outp(EZ80_PB_DR, regval);

  regval = inp(EZ80_PB_DDR);
  regval |= 7;
  outp(EZ80_PB_DDR, regval);

  regval = inp(EZ80_PB_ALT1);
  regval |= 7;
  outp(EZ80_PB_ALT1, regval);

  regval = inp(EZ80_PB_ALT2);
  regval |= 7;
  outp(EZ80_PB_ALT2, regval);
#else
  /* Configure PB0,1,2 as inputs */

  regval = inp(EZ80_PB_DDR);
  regval |= 7;
  outp(EZ80_PB_DDR, regval);

  regval = inp(EZ80_PB_ALT1);
  regval &= ~7;
  outp(EZ80_PB_ALT1, regval);

  regval = inp(EZ80_PB_ALT2);
  regval &= ~7;
  outp(EZ80_PB_ALT2, regval);
#endif

  return 3;
}

/****************************************************************************
 * Name: board_buttons
 ****************************************************************************/

uint32_t board_buttons(void)
{
  return inp(EZ80_PB_DDR) & 7;
}
#endif /* CONFIG_ARCH_BUTTONS */
