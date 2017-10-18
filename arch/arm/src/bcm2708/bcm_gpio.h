/************************************************************************************
 * arch/arm/src/bcm2708/bcm_gpio.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_BCM2708_BCM_GPIO_H
#define __ARCH_ARM_SRC_BCM2708_BCM_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Bit-encoded input to bcm_gpio_config() ********************************************/

/* 32-bit Encoding:
 *
 *   .... .... .... .MMM PPII IIII V.BB BBBB
 */

/* Input/Output mode:
 *
 *   .... .... .... .MMM .... .... .... ....
 */

#define GPIO_MODE_SHIFT            (16)        /* Bits 16-18: GPIO mode */
#define GPIO_MODE_MASK             (7 << GPIO_MODE_SHIFT)
#  define GPIO_INPUT               (0 << GPIO_MODE_SHIFT) /* PIO Input */
#  define GPIO_OUTPUT              (1 << GPIO_MODE_SHIFT) /* PIO Output */
#  define GPIO_ALT0                (4 << GPIO_MODE_SHIFT) /* Alternate function 0 */
#  define GPIO_ALT1                (5 << GPIO_MODE_SHIFT) /* Alternate function 1 */
#  define GPIO_ALT2                (6 << GPIO_MODE_SHIFT) /* Alternate function 2 */
#  define GPIO_ALT3                (7 << GPIO_MODE_SHIFT) /* Alternate function 3 */
#  define GPIO_ALT4                (3 << GPIO_MODE_SHIFT) /* Alternate function 4 */
#  define GPIO_ALT5                (2 << GPIO_MODE_SHIFT) /* Alternate function 5 */

/* These bits set the pull up/down configuration of the pin:
 *
 *   .... .... .... .... PP.. .... .... ....
 *
 * NOTE: The shifted values match the values of the GPPUD egister.
 */

#define GPIO_PUD_SHIFT             (14)        /* Bits 14-16: GPIO configuration bits */
#define GPIO_PUD_MASK              (3 << GPIO_PUD_SHIFT)
#  define GPIO_PUD_NONE            (0  << GPIO_PUD_SHIFT) /* Default, no pull */
#  define GPIO_PUD_PULLDOWN        (1  << GPIO_PUD_SHIFT) /* Enable pull down */
#  define GPIO_PUD_PULLUP          (2  << GPIO_PUD_SHIFT) /* Enable pull up */

/* Interrupt detection modes:
 *
 *   .... .... .... .... ..II IIII .... ....
 */

#define GPIO_INT_SHIFT             (8)         /* Bits 8-13: GPIO interrupt bits */
#define GPIO_INT_MASK              (0x3f << GPIO_INT_SHIFT)
#  define GPIO_INT_NONE            (0 << GPIO_INT_SHIFT)
#  define GPIO_INT_RISING          (1 << GPIO_INT_SHIFT)
#  define GPIO_INT_FALLING         (2 << GPIO_INT_SHIFT)
#  define GPIO_INT_BOTHEDGES       (GPIO_INT_RISING | GPIO_INT_FALLING)

#  define GPIO_INT_ASYNCHRISING    (4 << GPIO_INT_SHIFT)
#  define GPIO_INT_ASYNCHFALLING   (8 << GPIO_INT_SHIFT)
#  define GPIO_INT_ASYNCHBOTH      (GPIO_INT_ASYNCHRISING | GPIO_INT_ASYNCHFALLING)

#  define GPIO_INT_HIGHLEVEL       (16 << GPIO_INT_SHIFT)
#  define GPIO_INT_LOWLEVEL        (32 << GPIO_INT_SHIFT)
#  define GPIO_INT_LEVELBOTH       (GPIO_INT_HIGHLEVEL | GPIO_INT_LOWLEVEL)

/* If the pin is an GPIO output, then this identifies the initial output value:
 *
 *   .... .... .... .... .... .... V... ....
 */

#define GPIO_OUTPUT_MASK           (1 << 7)    /* Bit 7: Initial value of output */
#  define GPIO_OUTPUT_CLEAR        (0)
#  define GPIO_OUTPUT_SET          (1 << 7)

/* This identifies the GPIO pin:
 *
 *   ..... .... ... .... .... ..BB BBBB
 */

#define GPIO_PIN_SHIFT             (0)         /* Bits 0-5: GPIO number: 0-31 */
#define GPIO_PIN_MASK              (63 << GPIO_PIN_SHIFT)
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
#define GPIO_PIN32                 (32 << GPIO_PIN_SHIFT)
#define GPIO_PIN33                 (33 << GPIO_PIN_SHIFT)
#define GPIO_PIN34                 (34 << GPIO_PIN_SHIFT)
#define GPIO_PIN35                 (35 << GPIO_PIN_SHIFT)
#define GPIO_PIN36                 (36 << GPIO_PIN_SHIFT)
#define GPIO_PIN37                 (37 << GPIO_PIN_SHIFT)
#define GPIO_PIN38                 (38 << GPIO_PIN_SHIFT)
#define GPIO_PIN39                 (39 << GPIO_PIN_SHIFT)
#define GPIO_PIN40                 (40 << GPIO_PIN_SHIFT)
#define GPIO_PIN41                 (41 << GPIO_PIN_SHIFT)
#define GPIO_PIN42                 (42 << GPIO_PIN_SHIFT)
#define GPIO_PIN43                 (43 << GPIO_PIN_SHIFT)
#define GPIO_PIN44                 (44 << GPIO_PIN_SHIFT)
#define GPIO_PIN45                 (45 << GPIO_PIN_SHIFT)
#define GPIO_PIN46                 (46 << GPIO_PIN_SHIFT)
#define GPIO_PIN47                 (47 << GPIO_PIN_SHIFT)
#define GPIO_PIN48                 (48 << GPIO_PIN_SHIFT)
#define GPIO_PIN49                 (49 << GPIO_PIN_SHIFT)
#define GPIO_PIN50                 (50 << GPIO_PIN_SHIFT)
#define GPIO_PIN51                 (51 << GPIO_PIN_SHIFT)
#define GPIO_PIN52                 (52 << GPIO_PIN_SHIFT)
#define GPIO_PIN53                 (53 << GPIO_PIN_SHIFT)

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

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Function:  bcm_gpio_initialize
 *
 * Description:
 *   Based on configuration within the .config file, it does:
 *    - Remaps positions of alternative functions for GPIO.
 *
 *   Typically called from bcm_start().
 *
 ************************************************************************************/

void bcm_gpio_initialize(void);

/************************************************************************************
 * Name: bcm_gpio_irqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for GPIO pins.
 *
 ************************************************************************************/

#ifdef CONFIG_BCM2708_GPIO_IRQ
void bcm_gpio_irqinitialize(void);
#else
#  define bcm_gpio_irqinitialize()
#endif

/************************************************************************************
 * Name: bcm_gpio_config
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ************************************************************************************/

int bcm_gpio_config(gpio_pinset_t pinset);

/************************************************************************************
 * Name: bcm_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ************************************************************************************/

void bcm_gpio_write(gpio_pinset_t pinset, bool value);

/************************************************************************************
 * Name: bcm_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ************************************************************************************/

bool bcm_gpio_read(gpio_pinset_t pinset);

/****************************************************************************
 * Name: bcm_gpio_irqenable
 *
 * Description:
 *   Configure interrupt event detection for the specified GPIO pin.  This
 *   effective enables the pin interrupts.
 *
 ****************************************************************************/

#ifdef CONFIG_BCM2708_GPIO_IRQ
void bcm_gpio_irqenable(gpio_pinset_t pinset);
#else
#  define bcm_gpio_irqenable(p)
#endif

/************************************************************************************
 * Name: bcm_gpio_irqdisable
 *
 * Description:
 *   Reset interrupt event detection for the specified GPIO pin.  This
 *   effective disables the pin interrupts.
 *
 ************************************************************************************/

#ifdef CONFIG_BCM2708_GPIO_IRQ
void bcm_gpio_irqdisable(gpio_pinset_t pinset);
#else
#  define bcm_gpio_irqdisable(p)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_BCM2708_BCM_GPIO_H */
