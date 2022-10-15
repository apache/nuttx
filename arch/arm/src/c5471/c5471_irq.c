/****************************************************************************
 * arch/arm/src/c5471/c5471_irq.c
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

#include "arm.h"
#include "chip.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ILR_EDGESENSITIVE 0x00000020
#define ILR_PRIORITY      0x0000001E

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The value of _svectors is defined in ld.script.  It could be hard-coded
 * because we know that correct IRAM area is 0xffc00000.
 */

extern up_vector_t _svectors[];

/* The C5471 has FLASH at the low end of memory.  The rrload bootloaer will
 * catch all interrupts and re-vector them to vectors stored in IRAM.  The
 * following table is used to initialize those vectors.
 */

static up_vector_t g_vectorinittab[] =
{
  NULL,
  arm_vectorundefinsn,
  arm_vectorsvc,
  arm_vectorprefetch,
  arm_vectordata,
  arm_vectoraddrexcptn,
  arm_vectorirq,
  arm_vectorfiq
};
#define NVECTORS ((sizeof(g_vectorinittab)) / sizeof(up_vector_t))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_ackirq
 *
 * Description:
 *   Acknowlede the IRQ.Bit 0 of the Interrupt Control
 *   Register ==  New IRQ agreement (NEW_IRQ_AGR). Reset IRQ
 *   output. Clear source IRQ register. Enables a new IRQ
 *   generation. Reset by internal logic.
 *
 ****************************************************************************/

static inline void up_ackirq(unsigned int irq)
{
  uint32_t reg;
  reg = getreg32(SRC_IRQ_REG);              /* Insure appropriate IT_REG bit clears */
  putreg32(reg | 0x00000001, INT_CTRL_REG); /* write the NEW_IRQ_AGR bit. */
}

/****************************************************************************
 * Name: up_ackfiq
 *
 * Description:
 *   Acknowledge the FIQ.  Bit 1 of the Interrupt Control
 *   Register ==  New FIQ agreement (NEW_FIQ_AGR). Reset FIQ
 *   output. Clear source FIQ register. Enables a new FIQ
 *   generation. Reset by internal logic.
 *
 ****************************************************************************/

static inline void up_ackfiq(unsigned int irq)
{
  uint32_t reg;
  reg = getreg32(SRC_FIQ_REG);              /* Insure appropriate IT_REG bit clears */
  putreg32(reg | 0x00000002, INT_CTRL_REG); /* write the NEW_FIQ_AGR bit. */
}

/****************************************************************************
 * Name: up_vectorinitialize
 ****************************************************************************/

static inline void up_vectorinitialize(void)
{
  up_vector_t *src  = g_vectorinittab;
  up_vector_t *dest = _svectors;
  int i;

  for (i = 0; i < NVECTORS; i++)
    {
      *dest++ = *src++;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Disable all interrupts. */

  putreg32(0x0000ffff, MASK_IT_REG);

  /* Clear any pending interrupts */

  up_ackirq(0);
  up_ackfiq(0);
  putreg32(0x00000000, IT_REG);

  /* Override hardware defaults */

  putreg32(ILR_EDGESENSITIVE | ILR_PRIORITY, ILR_IRQ2_REG);
  putreg32(ILR_EDGESENSITIVE | ILR_PRIORITY, ILR_IRQ4_REG);
  putreg32(ILR_PRIORITY,                     ILR_IRQ6_REG);
  putreg32(ILR_EDGESENSITIVE | ILR_PRIORITY, ILR_IRQ15_REG);

  /* Initialize hardware interrupt vectors */

  up_vectorinitialize();

  /* And finally, enable interrupts */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  up_irq_restore(PSR_MODE_SYS | PSR_F_BIT);
#endif
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  if ((unsigned)irq < NR_IRQS)
    {
      uint32_t reg = getreg32(MASK_IT_REG);
      putreg32(reg | (1 << irq), MASK_IT_REG);
    }
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  if ((unsigned)irq < NR_IRQS)
    {
      uint32_t reg = getreg32(MASK_IT_REG);
      putreg32(reg & ~(1 << irq), MASK_IT_REG);
    }
}

/****************************************************************************
 * Name: arm_ack_irq
 *
 * Description:
 *   Acknowledge the interrupt
 *
 ****************************************************************************/

void arm_ack_irq(int irq)
{
  uint32_t reg;

  /* Set the NEW_IRQ_AGR bit.  This clears the IRQ src register
   * enables generation of a new IRQ.
   */

  reg = getreg32(INT_CTRL_REG);
  putreg32(reg | 0x00000001, INT_CTRL_REG); /* write the NEW_IRQ_AGR bit. */
}
