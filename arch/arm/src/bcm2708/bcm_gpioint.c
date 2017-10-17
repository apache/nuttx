/****************************************************************************
 * arch/arm/src/bcm2708/bcm_gpioint.c
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

#include "bcm_config.h"
#include "bcm_gpio.h"

#ifdef CONFIG_BCM2708_GPIO_IRQ

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Name: bcm_gpio_irqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for GPIO pins.
 *
 ************************************************************************************/

void bcm_gpio_irqinitialize(void)
{
#warning Missing Logic
}

/************************************************************************************
 * Name: bcm_gpio_irqconfig
 *
 * Description:
 *   Configure an interrupt for the specified GPIO pin.
 *
 ************************************************************************************/

void bcm_gpio_irqconfig(gpio_pinset_t pinset)
{
#warning Missing Logic
}

/************************************************************************************
 * Name: bcm_gpio_irqenable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

void bcm_gpio_irqenable(int irq)
{
#warning Missing Logic
}

/************************************************************************************
 * Name: bcm_gpio_irqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

void bcm_gpio_irqdisable(int irq)
{
#warning Missing Logic
}

#endif /* CONFIG_BCM2708_GPIO_IRQ */
