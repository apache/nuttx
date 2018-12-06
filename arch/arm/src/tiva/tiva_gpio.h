/****************************************************************************
 * arch/arm/src/tiva/tiva_gpio.h
 *
 *   Copyright (C) 2009-2010, 2013-2015, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * With modifications from Calvin Maguranis <calvin.maguranis@trd2inc.com>
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

#ifndef __ARCH_ARM_SRC_TIVA_TIVA_GPIO_H
#define __ARCH_ARM_SRC_TIVA_TIVA_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/irq.h>

#include "up_internal.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Include chip specific definitions */

#if defined(CONFIG_ARCH_CHIP_LM3S)
#  include "lm/lm3s_gpio.h"
#elif defined(CONFIG_ARCH_CHIP_LM4F)
#  include "lm/lm4f_gpio.h"
#elif defined(CONFIG_ARCH_CHIP_TM4C)
#  include "tm4c/tm4c_gpio.h"
#elif defined(CONFIG_ARCH_CHIP_CC13X0) || defined(CONFIG_ARCH_CHIP_CC13X2)
#  include "cc13xx/cc13xx_gpio.h"
#else
#  error "Unsupported Tiva/Stellaris system GPIO"
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Name: tiva_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int tiva_configgpio(uint32_t pinset);

/****************************************************************************
 * Name: tiva_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void tiva_gpiowrite(uint32_t pinset, bool value);

/****************************************************************************
 * Name: tiva_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool tiva_gpioread(uint32_t pinset);

/****************************************************************************
 * Function:  tiva_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the provided base address
 *
 ****************************************************************************/

int tiva_dumpgpio(uint32_t pinset, const char *msg);

/****************************************************************************
 * Name: tiva_gpio_lockport
 *
 * Description:
 *   Certain pins require to be unlocked from the NMI to use for normal GPIO
 *   use. See table 10-10 in datasheet for pins with special considerations.
 *
 ****************************************************************************/

void tiva_gpio_lockport(uint32_t pinset, bool lock);

/****************************************************************************
 * Function:  tiva_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the provided base address
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
void tiva_gpio_dumpconfig(uint32_t pinset);
#else
# define tiva_gpio_dumpconfig(p)
#endif

#ifdef CONFIG_TIVA_GPIO_IRQS
/****************************************************************************
 * Name: gpio_irqinitialize
 *
 * Description:
 *   Initialize all vectors to the unexpected interrupt handler
 *
 ****************************************************************************/

int weak_function tiva_gpioirqinitialize(void);

/****************************************************************************
 * Name: tiva_gpioirqattach
 *
 * Description:
 *   Attach in GPIO interrupt to the provided 'isr'. If isr==NULL, then the
 *   irq_unexpected_isr handler is assigned and the pin's interrupt mask is
 *   disabled to stop further interrupts. Otherwise, the new isr is linked
 *   and the pin's interrupt mask is set.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   return to indicate the nature of the failure.
 *
 ****************************************************************************/

int tiva_gpioirqattach(uint32_t pinset, xcpt_t isr, void *arg);
#  define tiva_gpioirqdetach(p) tiva_gpioirqattach((p),NULL,NULL)

/****************************************************************************
 * Name: tiva_gpioportirqattach
 *
 * Description:
 *   Attach 'isr' to the GPIO port. Only use this if you want to handle
 *   the entire ports interrupts explicitly.
 *
 ****************************************************************************/

void tiva_gpioportirqattach(uint8_t port, xcpt_t isr);
#  define tiva_gpioportirqdetach(port) tiva_gpioportirqattach(port, NULL)

/****************************************************************************
 * Name: tiva_gpioirqenable
 *
 * Description:
 *   Enable the GPIO port IRQ
 *
 ****************************************************************************/

void tiva_gpioirqenable(uint8_t port, uint8_t pin);

/****************************************************************************
 * Name: tiva_gpioirqdisable
 *
 * Description:
 *   Disable the GPIO port IRQ
 *
 ****************************************************************************/

void tiva_gpioirqdisable(uint8_t port, uint8_t pin);

/****************************************************************************
 * Name: tiva_gpioirqstatus
 *
 * Description:
 *   Returns raw or masked interrupt status.
 *
 ****************************************************************************/

uint32_t tiva_gpioirqstatus(uint8_t port, bool masked);

/****************************************************************************
 * Name: tiva_gpioirqclear
 *
 * Description:
 *   Clears the interrupt status of the input base
 *
 ****************************************************************************/

void tiva_gpioirqclear(uint8_t port, uint32_t pinmask);

#endif /* CONFIG_TIVA_GPIO_IRQS */

#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_TIVA_TIVA_GPIO_H */
