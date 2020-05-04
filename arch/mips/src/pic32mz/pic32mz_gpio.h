/************************************************************************************
 * arch/mips/src/pic32mz/pic32mz_gpio.h
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

#ifndef __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_GPIO_H
#define __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

#include <nuttx/irq.h>
#include <arch/pic32mz/irq.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* GPIO settings used in the configport, readport, writeport, etc.
 *
 * General encoding:
 * xxxx xxxx xxxx xSSM MAVI IIII RRRR PPPP
 *
 * x = Not implemented
 * S = Slew Rate
 * M = Mode (input/output)
 * I = Interrupt
 * R = Register
 * P = Pin
 */

/* Slew Rate Control.
 *
 * .... .... .... .SS. .... .... .... ....
 */

#define GPIO_SR_SHIFT        (17) /* Bits 17-18: Slew Rate Control */
#define GPIO_SR_MASK         (3 << GPIO_SR_SHIFT)
#  define GPIO_FASTEST       (0 << GPIO_SR_SHIFT) /* 00: Disabled and set to fastest */
#  define GPIO_MEDIUM        (1 << GPIO_SR_SHIFT) /* 01: Enabled and set to medium   */
#  define GPIO_SLOW          (2 << GPIO_SR_SHIFT) /* 10: Enabled and set to slow     */
#  define GPIO_SLOWEST       (3 << GPIO_SR_SHIFT) /* 11: Enabled and set to slowest  */
#define GPIO_SR_CON0_SHIFT   (0) /* Bit 0: SRCON0x */
#define GPIO_SR_CON0_MASK    (1 << GPIO_SR_CON0_SHIFT)
#define GPIO_SR_CON1_SHIFT   (1) /* Bit 1: SRCON1x */
#define GPIO_SR_CON1_MASK    (1 << GPIO_SR_CON1_SHIFT)

/* Input/Output mode:
 *
 * .... .... .... ...M M... .... .... ....
 */

#define GPIO_MODE_SHIFT      (15)      /* Bits 15-16: I/O mode */
#define GPIO_MODE_MASK       (3 << GPIO_MODE_SHIFT)
#  define GPIO_INPUT         (0 << GPIO_MODE_SHIFT) /* 00 Normal input */
#  define GPIO_OUTPUT        (2 << GPIO_MODE_SHIFT) /* 10 Normal output */
#  define GPIO_OPENDRAIN     (3 << GPIO_MODE_SHIFT) /* 11 Open drain output */

/* Analog mode.
 *
 * .... .... .... .... .A.. .... .... ....
 */

#define GPIO_ANALOG_SHIFT    (14) /* Bit 14: Analog */
#define GPIO_ANALOG_MASK     (1 << GPIO_ANALOG_SHIFT)
#  define GPIO_ANALOG        (1 << GPIO_ANALOG_SHIFT)
#  define GPIO_DIGITAL       (0)

/* Pin Value.
 *
 * .... .... .... .... ..V. .... .... ....
 */

#define GPIO_VALUE_SHIFT     (13) /* Bit 13: Initializetial output value */
#define GPIO_VALUE_MASK      (1 << GPIO_VALUE_SHIFT)
#  define GPIO_VALUE_ONE     (1 << GPIO_VALUE_SHIFT)
#  define GPIO_VALUE_ZERO    (0)

/* Change Notification config.
 *
 * .... .... .... .... ...I IIII .... ....
 */

#define GPIO_CN_SHIFT        (8) /* Bit 8 - 12: Change notification bits */
#define GPIO_CN_MASK         (31 << GPIO_CN_SHIFT)
#  define GPIO_INTERRUPT     (1  << GPIO_CN_SHIFT)  /* Bit 8:  Change notification enable bit */
#  define GPIO_PULLUP        (2  << GPIO_CN_SHIFT)  /* Bit 9:  Change notification pull-up */
#  define GPIO_PULLDOWN      (4  << GPIO_CN_SHIFT)  /* Bit 10: Change notification pull-down */
#  define GPIO_EDGE_DETECT   (8  << GPIO_CN_SHIFT)  /* Bit 11: Change notification interrupt mode */
#  define GPIO_MISMATCH      (0)
#  define GPIO_EDGE_RISING   (12 << GPIO_CN_SHIFT)  /* Bit 12: Change notification edge type */
#  define GPIO_EDGE_FALLING  (0)

/* GPIO Port.
 *
 * .... .... .... .... .... .... RRRR ....
 */

#define GPIO_PORT_SHIFT      (4)       /* Bits 4-7: Port number */
#define GPIO_PORT_MASK       (15 << GPIO_PORT_SHIFT)
#  define GPIO_PORTA         (0 << GPIO_PORT_SHIFT)
#  define GPIO_PORTB         (1 << GPIO_PORT_SHIFT)
#  define GPIO_PORTC         (2 << GPIO_PORT_SHIFT)
#  define GPIO_PORTD         (3 << GPIO_PORT_SHIFT)
#  define GPIO_PORTE         (4 << GPIO_PORT_SHIFT)
#  define GPIO_PORTF         (5 << GPIO_PORT_SHIFT)
#  define GPIO_PORTG         (6 << GPIO_PORT_SHIFT)
#  define GPIO_PORTH         (7 << GPIO_PORT_SHIFT)
#  define GPIO_PORTJ         (8 << GPIO_PORT_SHIFT)
#  define GPIO_PORTK         (9 << GPIO_PORT_SHIFT)

/* GPIO Pin.
 *
 * .... .... .... .... .... .... .... PPPP
 */

#define GPIO_PIN_SHIFT       0        /* Bits 0-3: GPIO number: 0-15 */
#define GPIO_PIN_MASK        (15 << GPIO_PIN_SHIFT)
#define GPIO_PIN0            (0  << GPIO_PIN_SHIFT)
#define GPIO_PIN1            (1  << GPIO_PIN_SHIFT)
#define GPIO_PIN2            (2  << GPIO_PIN_SHIFT)
#define GPIO_PIN3            (3  << GPIO_PIN_SHIFT)
#define GPIO_PIN4            (4  << GPIO_PIN_SHIFT)
#define GPIO_PIN5            (5  << GPIO_PIN_SHIFT)
#define GPIO_PIN6            (6  << GPIO_PIN_SHIFT)
#define GPIO_PIN7            (7  << GPIO_PIN_SHIFT)
#define GPIO_PIN8            (8  << GPIO_PIN_SHIFT)
#define GPIO_PIN9            (9  << GPIO_PIN_SHIFT)
#define GPIO_PIN10           (10 << GPIO_PIN_SHIFT)
#define GPIO_PIN11           (11 << GPIO_PIN_SHIFT)
#define GPIO_PIN12           (12 << GPIO_PIN_SHIFT)
#define GPIO_PIN13           (13 << GPIO_PIN_SHIFT)
#define GPIO_PIN14           (14 << GPIO_PIN_SHIFT)
#define GPIO_PIN15           (15 << GPIO_PIN_SHIFT)

/************************************************************************************
 * Public Types
 ************************************************************************************/

#ifndef __ASSEMBLY__

typedef uint32_t pinset_t;

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

/* This table can be used to map a port number to a IOPORT base address.  For
 * example, an index of zero would correspond to IOPORTA, one with IOPORTB,
 * etc.
 */

EXTERN const uintptr_t g_gpiobase[CHIP_NPORTS];

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: pic32mz_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin (the interrupt
 *   will be configured when pic32mz_attach() is called).
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ************************************************************************************/

int pic32mz_configgpio(pinset_t cfgset);

/************************************************************************************
 * Name: pic32mz_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ************************************************************************************/

void pic32mz_gpiowrite(pinset_t pinset, bool value);

/************************************************************************************
 * Name: pic32mz_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ************************************************************************************/

bool pic32mz_gpioread(pinset_t pinset);

/************************************************************************************
 * Name: pic32mz_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a GPIO change notification interrupts.  This
 *   function is called internally by the system on power up and should not be
 *   called again.
 *
 ************************************************************************************/

#ifdef CONFIG_PIC32MZ_GPIOIRQ
void pic32mz_gpioirqinitialize(void);
#else
#  define pic32mz_gpioirqinitialize()
#endif

/************************************************************************************
 * Name: pic32mz_gpioattach
 *
 * Description:
 *   Attach an interrupt service routine to a GPIO interrupt.  This will also
 *   reconfigure the pin as an interrupting input.  The change notification number is
 *   associated with all interrupt-capable GPIO pins.  The association could,
 *   however, differ from part to part and must be  provided by the caller.
 *
 *   When an interrupt occurs, it is due to a change on the GPIO input pin.  In that
 *   case, all attached handlers will be called.  Each handler must maintain state
 *   and determine if the underlying GPIO input value changed.
 *
 *   pinset  - GPIO pin configuration
 *   handler - Interrupt handler (may be NULL to detach)
 *   arg     - The argument that accompanies the interrupt
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated error value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_PIC32MZ_GPIOIRQ
int pic32mz_gpioattach(pinset_t pinset, xcpt_t handler, void *arg);
#else
#  define pic32mz_gpioattach(p,h,a) (0)
#endif

/************************************************************************************
 * Name: pic32mz_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_PIC32MZ_GPIOIRQ
void pic32mz_gpioirqenable(pinset_t pinset);
#else
#  define pic32mz_gpioirqenable(irq)
#endif

/************************************************************************************
 * Name: pic32mz_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_PIC32MZ_GPIOIRQ
void pic32mz_gpioirqdisable(pinset_t pinset);
#else
#  define pic32mz_gpioirqdisable(irq)
#endif

/************************************************************************************
 * Function:  pic32mz_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the provided base address
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
void pic32mz_dumpgpio(pinset_t pinset, const char *msg);
#else
#  define pic32mz_dumpgpio(p,m)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_GPIO_H */
