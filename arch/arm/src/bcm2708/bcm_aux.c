/****************************************************************************
 * arch/arm/src/bcm2708/bcm_aux.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <nuttx/arch.h>
#include <arch/irq.h>

#include "up_arch.h"
#include "bcm_config.h"
#include "chip/bcm2708_aux.h"
#include "bcm_aux.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR void *g_aux_arg[3];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcm_aux_interrupt
 *
 * Description:
 *   AUX interrupt handler
 *
 * Input Parameters:
 *   Standard interrupt input parameters
 *
 * Returned Value:
 *   Always returns OK
 *
 * Assumptions:
 *   Interrupts ar disabled
 *
 ****************************************************************************/

static int bcm_aux_interrupt(int irq, FAR void *context, FAR void *arg)
{
  uint32_t auxirq;

  /* Read the pending AUX interrupts */

  auxirq = getreg32(BCM_AUX_IRQ);

#ifdef CONFIG_BCM2708_MINI_UART
  if ((auxirq & BCM_AUX_IRQ_MU) != 0)
    {
      (void)bcm_mu_interrupt(irq, context, g_aux_arg[(int)BCM_AUX_MINI_UART]);
    }
#endif

#ifdef CONFIG_BCM2708_SPI1
  if ((auxirq & BCM_AUX_IRQ_SPI1) != 0)
    {
      (void)bcm_spi1_interrupt(irq, context, g_aux_arg[(int)BCM_AUX_MINI_SPI1]);
    }
#endif

#ifdef CONFIG_BCM2708_SPI2
  if ((auxirq & BCM_AUX_IRQ_SPI2) != 0)
    {
      (void)bcm_spi2_interrupt(irq, context, g_aux_arg[(int)BCM_AUX_MINI_SPI2]);
    }
#endif

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcm_aux_irqinitialize
 *
 * Description:
 *   Called during IRQ initialize to initialize the shard AUX interrupt
 *   logic.
 *
 ****************************************************************************/

void bcm_aux_irqinitialize(void)
{
  /* Disable all AUX interrupt sources (Also disables all peripheral
   * register accesses).
   *
   * Keep the Mini-UART enabled if we are using it for the system console
   * (since it was initialized earlier in the boot-up sequence).
   */

#ifdef CONFIG_BCM2708_MINI_UART_SERIAL_CONSOLE
  putreg32(BCM_AUX_ENB_MU, BCM_AUX_ENB);
#else
  putreg32(0, BCM_AUX_ENB);
#endif

  /* Attach and enable the AUX interrupt */

  (void)irq_attach(BCM_IRQ_AUX, bcm_aux_interrupt, NULL);
  up_enable_irq(BCM_IRQ_AUX);
}

/****************************************************************************
 * Name: bcm_aux_enable
 *
 * Description:
 *   Enable the specified AUX interrupt (also enables access to peripheral
 *   registers).
 *
 ****************************************************************************/

void bcm_aux_enable(enum bcm_aux_peripheral_e periph, FAR void *arg)
{
  uint32_t setbits;

  /* Read the AUX perpheral enable */

#ifdef CONFIG_BCM2708_MINI_UART
  if (periph == BCM_AUX_MINI_UART)
    {
      setbits = BCM_AUX_ENB_MU;
    }
  else
#endif

#ifdef CONFIG_BCM2708_SPI1
  if (periph == BCM_AUX_MINI_UART)
    {
      setbits = BCM_AUX_ENB_SPI1;
    }
  else
#endif

#ifdef CONFIG_BCM2708_SPI2
  if (periph == BCM_AUX_MINI_UART)
    {
      setbits = BCM_AUX_ENB_SPI1;
    }
  else
#endif
    {
      return;
    }

  g_aux_arg[(int)periph] = arg;
  modifyreg32(BCM_AUX_ENB, 0, setbits);
}

/****************************************************************************
 * Name: bcm_aux_disable
 *
 * Description:
 *   Disable the specified AUX interrupt (also disables access to peripheral
 *   registers).
 *
 ****************************************************************************/

void bcm_aux_disable(enum bcm_aux_peripheral_e periph)
{
  uint32_t clrbits;

  /* Read the AUX perpheral enable */

#ifdef CONFIG_BCM2708_MINI_UART
  if (periph == BCM_AUX_MINI_UART)
    {
      clrbits = BCM_AUX_ENB_MU;
    }
  else
#endif

#ifdef CONFIG_BCM2708_SPI1
  if (periph == BCM_AUX_MINI_UART)
    {
      clrbits = BCM_AUX_ENB_SPI1;
    }
  else
#endif

#ifdef CONFIG_BCM2708_SPI2
  if (periph == BCM_AUX_MINI_UART)
    {
      clrbits = BCM_AUX_ENB_SPI1;
    }
  else
#endif
    {
      return;
    }

  g_aux_arg[(int)periph] = NULL;
  modifyreg32(BCM_AUX_ENB, clrbits, 0);
}
