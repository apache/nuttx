/************************************************************************************
 * arch/arm/src/lpc11xx/lpc11_gpio.h
 *
 *   Copyright (C) 2010, 2013 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC11XX_LPC11_GPIO_H
#define __ARCH_ARM_SRC_LPC11XX_LPC11_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

#include <arch/lpc11xx/chip.h>

#include "chip/lpc11_gpio.h"
#include "chip/lpc11_pinconfig.h"

/* Include the GPIO definitions for the selected LPC17xx family. */

#if defined(LPC111x)
#  include "lpc111x_gpio.h"
#elif defined(LPC11C)
#  include "lpc11c_gpio.h"
#else
#  error "Unrecognized LPC11xx family"
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__
#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* These tables have global scope only because they are shared between lpc11_gpio.c,
 * lpc11_gpioint.c, and lpc11_gpiodbg.c
 */

#ifdef CONFIG_LPC11_GPIOIRQ
EXTERN uint64_t g_intedge0;
EXTERN uint64_t g_intedge2;
#endif

EXTERN const uint32_t g_fiobase[GPIO_NPORTS];
EXTERN const uint32_t g_intbase[GPIO_NPORTS];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Name: lpc11_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for GPIO pins.
 *
 ************************************************************************************/

#ifdef CONFIG_LPC11_GPIOIRQ
void lpc11_gpioirqinitialize(void);
#else
#  define lpc11_gpioirqinitialize()
#endif

/************************************************************************************
 * Name: lpc11_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ************************************************************************************/

int lpc11_configgpio(lpc11_pinset_t cfgset);

/************************************************************************************
 * Name: lpc11_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ************************************************************************************/

void lpc11_gpiowrite(lpc11_pinset_t pinset, bool value);

/************************************************************************************
 * Name: lpc11_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ************************************************************************************/

bool lpc11_gpioread(lpc11_pinset_t pinset);

/************************************************************************************
 * Name: lpc11_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_LPC11_GPIOIRQ
void lpc11_gpioirqenable(int irq);
#else
#  define lpc11_gpioirqenable(irq)
#endif

/************************************************************************************
 * Name: lpc11_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_LPC11_GPIOIRQ
void lpc11_gpioirqdisable(int irq);
#else
#  define lpc11_gpioirqdisable(irq)
#endif

/************************************************************************************
 * Function:  lpc11_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the base address of the provided pinset.
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
int lpc11_dumpgpio(lpc11_pinset_t pinset, const char *msg);
#else
#  define lpc11_dumpgpio(p,m)
#endif

#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_LPC11XX_LPC11_GPIO_H */
