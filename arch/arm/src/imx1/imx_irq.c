/****************************************************************************
 * arch/arm/src/imx1/imx_irq.c
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
#include <arch/irq.h>

#include "chip.h"
#include "arm_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Clear, disable and configure all interrupts. */

  putreg32(0, IMX_AITC_INTENABLEH);
  putreg32(0, IMX_AITC_INTENABLEL);

  /* Set masking of normal interrupts by priority.  Writing all ones
   * (or -1) to the NIMASK register sets the normal interrupt mask to
   * -1 and does not disable any normal interrupt priority levels.
   */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  putreg32(-1, IMX_AITC_NIMASK); /* -1: No priority levels masked */

  /* Initialize FIQs */

#ifdef CONFIG_ARCH_FIQ
  up_fiqinitialize();
#endif

  /* And finally, enable interrupts */

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
  putreg32(irq, IMX_AITC_INTDISNUM);
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
  putreg32(irq, IMX_AITC_INTENNUM);
}
