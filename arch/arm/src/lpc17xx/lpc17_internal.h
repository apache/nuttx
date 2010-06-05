/************************************************************************************
 * arch/arm/src/lpc17xx/lpc17_internal.h
 *
 *   Copyright (C) 2009-2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __ARCH_ARM_SRC_LPC17XX_LPC17_INTERNAL_H
#define __ARCH_ARM_SRC_LPC17XX_LPC17_INTERNAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include "up_internal.h"
#include "chip.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/

/* Bit-encoded input to lpc17_configgpio() ******************************************/

/* GPIO pin definitions *************************************************************/

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Data
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: lpc17_clockconfig
 *
 * Description:
 *   Called to initialize the LPC17XX.  This does whatever setup is needed to put the
 *   MCU in a usable state.  This includes the initialization of clocking using the
 *   settings in board.h.
 *
 ************************************************************************************/

EXTERN void lpc17_clockconfig(void);

/************************************************************************************
 * Name: lpc17_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level initialization
 *   including setup of the console UART.  This UART done early so that the serial
 *   console is available for debugging very early in the boot sequence.
 *
 ************************************************************************************/

EXTERN void lpc17_lowsetup(void);

/************************************************************************************
 * Name: lpc17_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for GPIO pins.
 *
 ************************************************************************************/

#ifdef CONFIG_GPIO_IRQ
EXTERN void lpc17_gpioirqinitialize(void);
#else
#  define lpc17_gpioirqinitialize()
#endif

/************************************************************************************
 * Name: lpc17_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ************************************************************************************/

EXTERN int lpc17_configgpio(uint16_t cfgset);

/************************************************************************************
 * Name: lpc17_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ************************************************************************************/

EXTERN void lpc17_gpiowrite(uint16_t pinset, bool value);

/************************************************************************************
 * Name: lpc17_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ************************************************************************************/

EXTERN bool lpc17_gpioread(uint16_t pinset);

/************************************************************************************
 * Name: lpc17_gpioirq
 *
 * Description:
 *   Configure an interrupt for the specified GPIO pin.
 *
 ************************************************************************************/

#ifdef CONFIG_GPIO_IRQ
EXTERN void lpc17_gpioirq(uint16_t pinset);
#else
#  define lpc17_gpioirq(pinset)
#endif

/************************************************************************************
 * Name: lpc17_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_GPIO_IRQ
EXTERN void lpc17_gpioirqenable(int irq);
#else
#  define lpc17_gpioirqenable(irq)
#endif

/************************************************************************************
 * Name: lpc17_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_GPIO_IRQ
EXTERN void lpc17_gpioirqdisable(int irq);
#else
#  define lpc17_gpioirqdisable(irq)
#endif

/************************************************************************************
 * Function:  lpc17_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the base address of the provided pinset.
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_GPIO
EXTERN int lpc17_dumpgpio(uint32_t pinset, const char *msg);
#else
#  define lpc17_dumpgpio(p,m)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_LPC17XX_LPC17_INTERNAL_H */
