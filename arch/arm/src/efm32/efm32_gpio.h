/************************************************************************************
 * arch/arm/src/efm32/efm32_gpio.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_EFM32_EFM32_GPIO_H
#define __ARCH_ARM_SRC_EFM32_EFM32_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_GPIO
#endif

#define EFM32_NGPIO                5 /* (5) GPIOA-F */

/* Bit-encoded input to efm32_configgpio() *******************************************/

/* 16-bit Encoding:
 *
 *   Input:  MMMM OII. .PPP BBBB
 *   Output: MMMM VDD. .PPP BBBB
 */

/* Port mode:
 *
 *   MMMM O... .... ....
 */

#define GPIO_MODE_SHIFT            (12)        /* Bits 12-15: Port mode */
#define GPIO_MODE_MASK             (15 << GPIO_MODE_SHIFT)

#define GPIO_MODE_DOUT_SHIFT       (11)        /* Bit 11: Input mode modifer */
#define GPIO_MODE_DOUT_MASK        (1 << GPIO_MODE_DOUT_SHIFT)
#  define GPIO_MODE_DOUT           GPIO_MODE_DOUT_MASK

/* Input modes */

#  define _GPIO_DISABLE            (0 << GPIO_MODE_SHIFT)
#  define _GPIO_INPUT              (1 << GPIO_MODE_SHIFT)
#  define _GPIO_INPUT_PULL         (2 << GPIO_MODE_SHIFT)
#  define _GPIO_INPUT_PULL_FILTER  (3 << GPIO_MODE_SHIFT)

#  define GPIO_PULLUP \
   (_GPIO_DISABLE | GPIO_MODE_DOUT)
#  define GPIO_INPUT \
   _GPIO_INPUT
#  define GPIO_INPUT_FILTER \
   (_GPIO_INPUT | GPIO_MODE_DOUT)
#  define GPIO_INPUT_PULLDOWN \
   _GPIO_INPUT_PULL
#  define GPIO_INPUT_PULLUP \
   (_GPIO_INPUT_PULL | GPIO_MODE_DOUT)
#  define GPIO_INPUT_PULLDOWN_FILTER \
   _GPIO_INPUT_PULL_FILTER
#  define GPIO_INPUT_PULLUP_FILTER \
   (_GPIO_INPUT_PULL_FILTER | GPIO_MODE_DOUT)

/* Output modes */

#  define GPIO_OUTPUT_PUSHPULL \
   (4 << GPIO_MODE_SHIFT)
#  define GPIO_OUTPUT_PUSHPULL_DRIVE \
   (5 << GPIO_MODE_SHIFT)
#  define GPIO_OUTPUT_WIREDOR \
   (6 << GPIO_MODE_SHIFT)
#  define GPIO_OUTPUT_WIREDOR_PULLDOWN \
   (7 << GPIO_MODE_SHIFT)
#  define GPIO_OUTPUT_WIREDAND \
   (8 << GPIO_MODE_SHIFT)
#  define GPIO_OUTPUT_WIREDAND_FILTER \
   (9 << GPIO_MODE_SHIFT)
#  define GPIO_OUTPUT_WIREDAND_PULLUP \
   (10 << GPIO_MODE_SHIFT)
#  define GPIO_OUTPUT_WIREDAND_PULLUP_FILTER \
   (11 << GPIO_MODE_SHIFT)
#  define GPIO_OUTPUT_WIREDAND_DRIVE \
   (12 << GPIO_MODE_SHIFT)
#  define GPIO_OUTPUT_WIREDAND_DRIVE_FILTER \
   (13 << GPIO_MODE_SHIFT)
#  define GPIO_OUTPUT_WIREDAND_DRIVE_PULLUP \
   (14 << GPIO_MODE_SHIFT)
#  define GPIO_OUTPUT_WIREDAND_DRIVE_PULLUP_FILTER  \
   (15 << GPIO_MODE_SHIFT)

/* If the pin is an PIO output, then this identifies the initial output value:
 *
 *  .... V... .... ....
 */

#define GPIO_OUTPUT_SHIFT          (11)        /* Bit 11: Initial value of output */
#define GPIO_OUTPUT_MASK           (1 << GPIO_OUTPUT_SHIFT)
#  define GPIO_OUTPUT_SET          GPIO_OUTPUT_MASK
#  define GPIO_OUTPUT_CLEAR        (0)

/* Output drive:
 *
 *   .... .DD. .... ....
 */

#define GPIO_DRIVE_SHIFT           (9)         /* Bits 9-10: Output drive strength */
#define GPIO_DRIVE_MASK            (3 << GPIO_MODE_SHIFT)
#  define _GPIO_DRIVE_STANDARD     (0)         /* 6 mA drive current */
#  define _GPIO_DRIVE_LOWEST       (1)         /* 0.5 mA drive current */
#  define _GPIO_DRIVE_HIGH         (2)         /* 20 mA drive current */
#  define _GPIO_DRIVE_LOW          (3)         /* 2 mA drive current */
#  define GPIO_DRIVE_STANDARD      (_GPIO_DRIVE_STANDARD << GPIO_MODE_SHIFT)
#  define GPIO_DRIVE_LOWEST        (_GPIO_DRIVE_LOWEST << GPIO_MODE_SHIFT)
#  define GPIO_DRIVE_HIGH          (_GPIO_DRIVE_HIGH << GPIO_MODE_SHIFT)
#  define GPIO_DRIVE_LOW           (_GPIO_DRIVE_LOW << GPIO_MODE_SHIFT)

/* Interrupt Mode (Input only):
 *
 *   .... .II. .... ....
 */

#define GPIO_INT_SHIFT             (9)        /* Bits 9-10: Interrupt mode */
#define GPIO_INT_MASK              (3 << GPIO_INT_SHIFT)
#  define GPIO_INT_NONE            (0)
#  define GPIO_INT_RISING          (1 << GPIO_INT_SHIFT)
#  define GPIO_INT_FALLING         (2 << GPIO_INT_SHIFT)
#  define GPIO_INT_BOTH            (GPIO_INT_RISING | GPIO_INT_FALLING)

/* This identifies the PIO port:
 *
 *   .... .... .PPP ....
 */

#define GPIO_PORT_SHIFT            (4)         /* Bit 4-6:  Port number */
#define GPIO_PORT_MASK             (7 << GPIO_PORT_SHIFT)
#  define GPIO_PORTA               (0 << GPIO_PORT_SHIFT)
#  define GPIO_PORTB               (1 << GPIO_PORT_SHIFT)
#  define GPIO_PORTC               (2 << GPIO_PORT_SHIFT)
#  define GPIO_PORTD               (3 << GPIO_PORT_SHIFT)
#  define GPIO_PORTE               (4 << GPIO_PORT_SHIFT)
#  define GPIO_PORTF               (5 << GPIO_PORT_SHIFT)

/* This identifies the pin in the port:
 *
 *   .... .... .... BBBB
 */

#define GPIO_PIN_SHIFT             (0)         /* Bits 0-3: Pin number: 0-15 */
#define GPIO_PIN_MASK              (15 << GPIO_PIN_SHIFT)
#  define GPIO_PIN0                (0  << GPIO_PIN_SHIFT)
#  define GPIO_PIN1                (1  << GPIO_PIN_SHIFT)
#  define GPIO_PIN2                (2  << GPIO_PIN_SHIFT)
#  define GPIO_PIN3                (3  << GPIO_PIN_SHIFT)
#  define GPIO_PIN4                (4  << GPIO_PIN_SHIFT)
#  define GPIO_PIN5                (5  << GPIO_PIN_SHIFT)
#  define GPIO_PIN6                (6  << GPIO_PIN_SHIFT)
#  define GPIO_PIN7                (7  << GPIO_PIN_SHIFT)
#  define GPIO_PIN8                (8  << GPIO_PIN_SHIFT)
#  define GPIO_PIN9                (9  << GPIO_PIN_SHIFT)
#  define GPIO_PIN10               (10 << GPIO_PIN_SHIFT)
#  define GPIO_PIN11               (11 << GPIO_PIN_SHIFT)
#  define GPIO_PIN12               (12 << GPIO_PIN_SHIFT)
#  define GPIO_PIN13               (13 << GPIO_PIN_SHIFT)
#  define GPIO_PIN14               (14 << GPIO_PIN_SHIFT)
#  define GPIO_PIN15               (15 << GPIO_PIN_SHIFT)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/* Must be big enough to hold the 16-bit encoding */

typedef uint16_t gpio_pinset_t;

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
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: efm32_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for PIO pins.
 *
 ************************************************************************************/

#ifdef CONFIG_EFM32_GPIO_IRQ
void efm32_gpioirqinitialize(void);
#else
#  define efm32_gpioirqinitialize()
#endif

/************************************************************************************
 * Name: efm32_configgpio
 *
 * Description:
 *   Configure a PIO pin based on bit-encoded description of the pin.
 *
 ************************************************************************************/

int efm32_configgpio(gpio_pinset_t cfgset);

/************************************************************************************
 * Name: efm32_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected PIO pin
 *
 ************************************************************************************/

void efm32_gpiowrite(gpio_pinset_t pinset, bool value);

/************************************************************************************
 * Name: efm32_gpioread
 *
 * Description:
 *   Read one or zero from the selected PIO pin
 *
 ************************************************************************************/

bool efm32_gpioread(gpio_pinset_t pinset);

/************************************************************************************
 * Name: efm32_gpioirq
 *
 * Description:
 *   Configure an interrupt for the specified PIO pin.
 *
 ************************************************************************************/

#ifdef CONFIG_EFM32_GPIO_IRQ
void efm32_gpioirq(gpio_pinset_t pinset);
#else
#  define efm32_gpioirq(pinset)
#endif

/************************************************************************************
 * Name: efm32_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified PIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_EFM32_GPIO_IRQ
void efm32_gpioirqenable(int irq);
#else
#  define efm32_gpioirqenable(irq)
#endif

/************************************************************************************
 * Name: efm32_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified PIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_EFM32_GPIO_IRQ
void efm32_gpioirqdisable(int irq);
#else
#  define efm32_gpioirqdisable(irq)
#endif

/************************************************************************************
 * Function:  efm32_dumpgpio
 *
 * Description:
 *   Dump all PIO registers associated with the base address of the provided pinset.
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_GPIO
int efm32_dumpgpio(uint32_t pinset, const char *msg);
#else
#  define efm32_dumpgpio(p,m)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_EFM32_EFM32_GPIO_H */
