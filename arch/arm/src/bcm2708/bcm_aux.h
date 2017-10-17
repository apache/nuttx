/****************************************************************************
 * arch/arm/src/bcm2708/bcm_aux.h
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

#ifndef __ARCH_ARM_SRC_BCM2708_BCM_AUX_H
#define __ARCH_ARM_SRC_BCM2708_BCM_AUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum bcm_aux_peripheral_e
{
  BCM_AUX_MINI_UART = 0,  /* Mini UART peripheral */
  BCM_AUX_MINI_SPI1,      /* SPI1 peripheral */
  BCM_AUX_MINI_SPI2,      /* SPI2 peripheral */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: bcm_aux_irqinitialize
 *
 * Description:
 *   Called during IRQ initialize to initialize the shard AUX interrupt
 *   logic.
 *
 ****************************************************************************/

void bcm_aux_irqinitialize(void);

/****************************************************************************
 * Name: bcm_aux_enable
 *
 * Description:
 *   Enable the specified AUX interrupt (also enables access to peripheral
 *   registers).
 *
 ****************************************************************************/

void bcm_aux_enable(enum bcm_aux_peripheral_e periph, FAR void *arg);

/****************************************************************************
 * Name: bcm_aux_disable
 *
 * Description:
 *   Disable the specified AUX interrupt (also disables access to peripheral
 *   registers).
 *
 ****************************************************************************/

void bcm_aux_disable(enum bcm_aux_peripheral_e periph);

/****************************************************************************
 * Name: bcm_[mu|spi1|spi2]_interupt
 *
 * Description:
 *   These callbacks must be provided by Mini-UART, SPI1, and SPI2 logic
 *   when those peripherals are configured.  These callbacks will be invoked
 *   from interrupt level processing when an interrupt for one of those
 *   peripherals is received (with interrupts disabled)
 *
 ****************************************************************************/

#ifdef CONFIG_BCM2708_MINI_UART
int bcm_mu_interrupt(int irq, FAR void *context, FAR void *arg);
#endif
#ifdef CONFIG_BCM2708_SPI1
int bcm_spi1_interrupt(int irq, FAR void *context, FAR void *arg);
#endif
#ifdef CONFIG_BCM2708_SPI2
int bcm_spi2_interrupt(int irq, FAR void *context, FAR void *arg);
#endif

#endif /* __ARCH_ARM_SRC_BCM2708_BCM_AUX_H */
