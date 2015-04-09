/************************************************************************************
 * arch/arm/src/samv7/sam_gpio.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAMV7_SAM_GPIO_H
#define __ARCH_ARM_SRC_SAMV7_SAM_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include <arch/samv7/chip.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Bit-encoded input to sam_configgpio() ********************************************/

/* 32-bit Encoding:
 *
 *   .... .... MMMC CCCC  IIIV D... PPPB BBBB
 */

/* Input/Output mode:
 *
 *   .... .... MMM. .... .... .... .... ....
 */

#define GPIO_MODE_SHIFT            (21)        /* Bits 21-23: GPIO mode */
#define GPIO_MODE_MASK             (7 << GPIO_MODE_SHIFT)
#  define GPIO_ALTERNATE           (0 << GPIO_MODE_SHIFT) /* PIO alternate function */
#  define GPIO_INPUT               (1 << GPIO_MODE_SHIFT) /* PIO Input */
#  define GPIO_OUTPUT              (2 << GPIO_MODE_SHIFT) /* PIO Output */
#  define GPIO_PERIPHA             (3 << GPIO_MODE_SHIFT) /* Controlled by periph A signal */
#  define GPIO_PERIPHB             (4 << GPIO_MODE_SHIFT) /* Controlled by periph B signal */
#  define GPIO_PERIPHC             (5 << GPIO_MODE_SHIFT) /* Controlled by periph C signal */
#  define GPIO_PERIPHD             (6 << GPIO_MODE_SHIFT) /* Controlled by periph D signal */

/* These bits set the configuration of the pin:
 * NOTE: No definitions for parallel capture mode
 *
 *   .... .... ...C CCCC .... .... .... ....
 */

#define GPIO_CFG_SHIFT             (16)        /* Bits 16-20: GPIO configuration bits */
#define GPIO_CFG_MASK              (31 << GPIO_CFG_SHIFT)
#  define GPIO_CFG_DEFAULT         (0  << GPIO_CFG_SHIFT) /* Default, no attribute */
#  define GPIO_CFG_PULLUP          (1  << GPIO_CFG_SHIFT) /* Bit 16: Internal pull-up */
#  define GPIO_CFG_PULLDOWN        (2  << GPIO_CFG_SHIFT) /* Bit 17: Internal pull-down */
#  define GPIO_CFG_DEGLITCH        (4  << GPIO_CFG_SHIFT) /* Bit 18: Internal glitch filter */
#  define GPIO_CFG_OPENDRAIN       (8  << GPIO_CFG_SHIFT) /* Bit 19: Open drain */
#  define GPIO_CFG_SCHMITT         (16 << GPIO_CFG_SHIFT) /* Bit 20: Schmitt trigger */

/* Additional interrupt modes:
 *
 *   .... .... .... .... III. .... .... ....
 */

#define GPIO_INT_SHIFT             (13)        /* Bits 13-15: GPIO interrupt bits */
#define GPIO_INT_MASK              (7 << GPIO_INT_SHIFT)
#  define _GIO_INT_AIM             (1 << 15)   /* Bit 15: Additional Interrupt modes */
#  define _GPIO_INT_LEVEL          (1 << 14)   /* Bit 14: Level detection interrupt */
#  define _GPIO_INT_EDGE           (0)         /*         (vs. Edge detection interrupt) */
#  define _GPIO_INT_RH             (1 << 13)   /* Bit 13: Rising edge/High level detection interrupt */
#  define _GPIO_INT_FL             (0)         /*         (vs. Falling edge/Low level detection interrupt) */

#  define GPIO_INT_HIGHLEVEL       (_GIO_INT_AIM | _GPIO_INT_LEVEL | _GPIO_INT_RH)
#  define GPIO_INT_LOWLEVEL        (_GIO_INT_AIM | _GPIO_INT_LEVEL | _GPIO_INT_FL)
#  define GPIO_INT_RISING          (_GIO_INT_AIM | _GPIO_INT_EDGE  | _GPIO_INT_RH)
#  define GPIO_INT_FALLING         (_GIO_INT_AIM | _GPIO_INT_EDGE  | _GPIO_INT_FL)
#  define GPIO_INT_BOTHEDGES       (0)

/* If the pin is an GPIO output, then this identifies the initial output value:
 *
 *   .... .... .... .... ...V .... .... ....
 */

#define GPIO_OUTPUT_SET            (1 << 12)   /* Bit 12: Initial value of output */
#define GPIO_OUTPUT_CLEAR          (0)

/* If the pin is an GPIO output, then this identifies the output drive strength:
 *
 *   .... .... .... .... .... D... .... ....
 */

#define GPIO_OUTPUT_DRIVE          (1 << 11)   /* Bit 11: Initial value of output */
#  define GPIO_OUTPUT_HIGH_DRIVE   (1 << 11)
  #define GPIO_OUTPUT_LOW_DRIVE    (0)

/* This identifies the GPIO port:
 *
 *   .... .... .... .... .... PPP. ....
 */

#define GPIO_PORT_SHIFT            (5)         /* Bit 5-7:  Port number */
#define GPIO_PORT_MASK             (7 << GPIO_PORT_SHIFT)
#  define GPIO_PORT_PIOA           (0 << GPIO_PORT_SHIFT)
#  define GPIO_PORT_PIOB           (1 << GPIO_PORT_SHIFT)
#  define GPIO_PORT_PIOC           (2 << GPIO_PORT_SHIFT)
#  define GPIO_PORT_PIOD           (3 << GPIO_PORT_SHIFT)
#  define GPIO_PORT_PIOE           (4 << GPIO_PORT_SHIFT)

/* This identifies the bit in the port:
 *
 *   ..... .... ... .... .... ...B BBBB
 */

#define GPIO_PIN_SHIFT             (0)         /* Bits 0-4: GPIO number: 0-31 */
#define GPIO_PIN_MASK              (31 << GPIO_PIN_SHIFT)
#define GPIO_PIN0                  (0  << GPIO_PIN_SHIFT)
#define GPIO_PIN1                  (1  << GPIO_PIN_SHIFT)
#define GPIO_PIN2                  (2  << GPIO_PIN_SHIFT)
#define GPIO_PIN3                  (3  << GPIO_PIN_SHIFT)
#define GPIO_PIN4                  (4  << GPIO_PIN_SHIFT)
#define GPIO_PIN5                  (5  << GPIO_PIN_SHIFT)
#define GPIO_PIN6                  (6  << GPIO_PIN_SHIFT)
#define GPIO_PIN7                  (7  << GPIO_PIN_SHIFT)
#define GPIO_PIN8                  (8  << GPIO_PIN_SHIFT)
#define GPIO_PIN9                  (9  << GPIO_PIN_SHIFT)
#define GPIO_PIN10                 (10 << GPIO_PIN_SHIFT)
#define GPIO_PIN11                 (11 << GPIO_PIN_SHIFT)
#define GPIO_PIN12                 (12 << GPIO_PIN_SHIFT)
#define GPIO_PIN13                 (13 << GPIO_PIN_SHIFT)
#define GPIO_PIN14                 (14 << GPIO_PIN_SHIFT)
#define GPIO_PIN15                 (15 << GPIO_PIN_SHIFT)
#define GPIO_PIN16                 (16 << GPIO_PIN_SHIFT)
#define GPIO_PIN17                 (17 << GPIO_PIN_SHIFT)
#define GPIO_PIN18                 (18 << GPIO_PIN_SHIFT)
#define GPIO_PIN19                 (19 << GPIO_PIN_SHIFT)
#define GPIO_PIN20                 (20 << GPIO_PIN_SHIFT)
#define GPIO_PIN21                 (21 << GPIO_PIN_SHIFT)
#define GPIO_PIN22                 (22 << GPIO_PIN_SHIFT)
#define GPIO_PIN23                 (23 << GPIO_PIN_SHIFT)
#define GPIO_PIN24                 (24 << GPIO_PIN_SHIFT)
#define GPIO_PIN25                 (25 << GPIO_PIN_SHIFT)
#define GPIO_PIN26                 (26 << GPIO_PIN_SHIFT)
#define GPIO_PIN27                 (27 << GPIO_PIN_SHIFT)
#define GPIO_PIN28                 (28 << GPIO_PIN_SHIFT)
#define GPIO_PIN29                 (29 << GPIO_PIN_SHIFT)
#define GPIO_PIN30                 (30 << GPIO_PIN_SHIFT)
#define GPIO_PIN31                 (31 << GPIO_PIN_SHIFT)

/************************************************************************************
 * Public Types
 ************************************************************************************/

#ifndef __ASSEMBLY__

/* Must be big enough to hold the 32-bit encoding */

typedef uint32_t gpio_pinset_t;

/************************************************************************************
 * Public Data
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

EXTERN const uintptr_t g_portbase[SAMV7_NPIO];

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

/****************************************************************************
 * Name: sam_gpio_base
 *
 * Description:
 *   Return the base address of the GPIO register set
 *
 ****************************************************************************/

static inline uintptr_t sam_gpio_base(gpio_pinset_t cfgset)
{
  int port = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  DEBUGASSERT(port <SAMV7_NPIO);
  return g_portbase[port];
}

/****************************************************************************
 * Name: sam_gpio_port
 *
 * Description:
 *   Return the PIO port number
 *
 ****************************************************************************/

static inline int sam_gpio_port(gpio_pinset_t cfgset)
{
  return (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
}

/****************************************************************************
 * Name: sam_gpio_pin
 *
 * Description:
 *   Return the PIO pin number
 *
 ****************************************************************************/

static inline int sam_gpio_pin(gpio_pinset_t cfgset)
{
  return (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
}

/****************************************************************************
 * Name: sam_gpio_pinmask
 *
 * Description:
 *   Return the PIO pin bit maskt
 *
 ****************************************************************************/

static inline int sam_gpio_pinmask(gpio_pinset_t cfgset)
{
  return 1 << ((cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT);
}

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: sam_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for GPIO pins.
 *
 ************************************************************************************/

#ifdef CONFIG_SAMV7_GPIO_IRQ
void sam_gpioirqinitialize(void);
#else
#  define sam_gpioirqinitialize()
#endif

/************************************************************************************
 * Name: sam_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ************************************************************************************/

int sam_configgpio(gpio_pinset_t cfgset);

/************************************************************************************
 * Name: sam_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ************************************************************************************/

void sam_gpiowrite(gpio_pinset_t pinset, bool value);

/************************************************************************************
 * Name: sam_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ************************************************************************************/

bool sam_gpioread(gpio_pinset_t pinset);

/************************************************************************************
 * Name: sam_gpioirq
 *
 * Description:
 *   Configure an interrupt for the specified GPIO pin.
 *
 ************************************************************************************/

#ifdef CONFIG_SAMV7_GPIO_IRQ
void sam_gpioirq(gpio_pinset_t pinset);
#else
#  define sam_gpioirq(pinset)
#endif

/************************************************************************************
 * Name: sam_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_SAMV7_GPIO_IRQ
void sam_gpioirqenable(int irq);
#else
#  define sam_gpioirqenable(irq)
#endif

/************************************************************************************
 * Name: sam_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_SAMV7_GPIO_IRQ
void sam_gpioirqdisable(int irq);
#else
#  define sam_gpioirqdisable(irq)
#endif

/************************************************************************************
 * Function:  sam_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the base address of the provided pinset.
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_GPIO
int sam_dumpgpio(uint32_t pinset, const char *msg);
#else
#  define sam_dumpgpio(p,m)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_SAMV7_SAM_GPIO_H */
