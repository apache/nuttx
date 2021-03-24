/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc17_40_gpio.h
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

#ifndef __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_GPIO_H
#define __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

#include <arch/lpc17xx_40xx/chip.h>

#include "hardware/lpc17_40_gpio.h"
#include "hardware/lpc17_40_pinconn.h"
#include "hardware/lpc17_40_pinconfig.h"

/* Include the GPIO definitions for the selected LPC17xx/LPC40xx family. */

#if defined(LPC176x)
#  include "lpc176x_gpio.h"
#elif defined(LPC178x_40xx)
#  include "lpc178x_40xx_gpio.h"
#else
#  error "Unrecognized LPC17xx/LPC40xx family"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* These tables have global scope only because they are shared between
 * lpc17_40_gpio.c, lpc17_40_gpioint.c, and lpc17_40_gpiodbg.c
 */

#ifdef CONFIG_LPC17_40_GPIOIRQ
EXTERN uint64_t g_intedge0;
EXTERN uint64_t g_intedge2;
#endif

EXTERN const uint32_t g_fiobase[GPIO_NPORTS];
EXTERN const uint32_t g_intbase[GPIO_NPORTS];

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_40_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
   * GPIO pins.
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_40_GPIOIRQ
void lpc17_40_gpioirqinitialize(void);
#else
#  define lpc17_40_gpioirqinitialize()
#endif

/****************************************************************************
 * Name: lpc17_40_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int lpc17_40_configgpio(lpc17_40_pinset_t cfgset);

/****************************************************************************
 * Name: lpc17_40_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void lpc17_40_gpiowrite(lpc17_40_pinset_t pinset, bool value);

/****************************************************************************
 * Name: lpc17_40_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool lpc17_40_gpioread(lpc17_40_pinset_t pinset);

/****************************************************************************
 * Name: lpc17_40_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_40_GPIOIRQ
void lpc17_40_gpioirqenable(int irq);
#else
#  define lpc17_40_gpioirqenable(irq)
#endif

/****************************************************************************
 * Name: lpc17_40_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_40_GPIOIRQ
void lpc17_40_gpioirqdisable(int irq);
#else
#  define lpc17_40_gpioirqdisable(irq)
#endif

/****************************************************************************
 * Function:  lpc17_40_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the base address of the provided
 *   pinset.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
int lpc17_40_dumpgpio(lpc17_40_pinset_t pinset, const char *msg);
#else
#  define lpc17_40_dumpgpio(p,m)
#endif

#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_GPIO_H */
