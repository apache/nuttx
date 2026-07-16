/*****************************************************************************
 * arch/mips/src/jz4780/jz4780_gpio.h
 *
 * SPDX-License-Identifier: Apache-2.0
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
 *****************************************************************************/

#ifndef __ARCH_MIPS_SRC_JZ4780_JZ4780_GPIO_H
#define __ARCH_MIPS_SRC_JZ4780_JZ4780_GPIO_H

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/irq.h>
#include <arch/jz4780/irq.h>

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

#define JZ4780_BASE_GPIOA          0xB0010000U
#define JZ4780_BASE_GPIOB          0xB0010100U
#define JZ4780_BASE_GPIOC          0xB0010200U
#define JZ4780_BASE_GPIOD          0xB0010300U
#define JZ4780_BASE_GPIOE          0xB0010400U
#define JZ4780_BASE_GPIOF          0xB0010500U

#define GPIO_PXPIN 0x00

#define GPIO_INT   0x10
#define GPIO_INTS  0x14
#define GPIO_INTC  0x18

#define GPIO_MSK   0x20
#define GPIO_MSKS  0x24
#define GPIO_MSKC  0x28

#define GPIO_PXT1  0x30
#define GPIO_PXT1S 0x34
#define GPIO_PXT1C 0x38

#define GPIO_PXT0  0x40
#define GPIO_PXT0S 0x44
#define GPIO_PXT0C 0x48

#define GPIO_FLG   0x50
#define GPIO_FLGC  0x58

#define GPIO_PEN   0x70
#define GPIO_PENS  0x74
#define GPIO_PENC  0x78

/* GPIO settings */

#define MODE_BIT_PXT0   (1 << GPIO_MODE_SHIFT)
#define MODE_BIT_PXT1   (2 << GPIO_MODE_SHIFT)
#define MODE_BIT_MSK    (4 << GPIO_MODE_SHIFT)
#define MODE_BIT_INT    (8 << GPIO_MODE_SHIFT)

#define GPIO_MODE_SHIFT         (9)                   /* Bits 9-12: I/O mode */
#define GPIO_MODE_MASK          (0xF << GPIO_MODE_SHIFT)

#  define GPIO_MODE_DEVICE0     (0x0 << GPIO_MODE_SHIFT) /* 0000 Device 0 */
#  define GPIO_MODE_DEVICE1     (0x1 << GPIO_MODE_SHIFT) /* 0001 Device 1 */
#  define GPIO_MODE_DEVICE2     (0x2 << GPIO_MODE_SHIFT) /* 0010 Device 2 */
#  define GPIO_MODE_DEVICE3     (0x3 << GPIO_MODE_SHIFT) /* 0011 Device 3 */

#  define GPIO_MODE_OUTPUT0     (0x4 << GPIO_MODE_SHIFT) /* 0100 GPIO out 0  */
#  define GPIO_MODE_OUTPUT1     (0x5 << GPIO_MODE_SHIFT) /* 0101 GPIO out 1  */
#  define GPIO_MODE_INPUT       (0x6 << GPIO_MODE_SHIFT) /* 0110 GPIO in     */
#  define GPIO_MODE_INPUT_TOO   (0x7 << GPIO_MODE_SHIFT) /* 0111 GPIO in     */

#  define GPIO_MODE_INTR_LOW    (0x8 << GPIO_MODE_SHIFT) /* 1000 low level interrupt  */
#  define GPIO_MODE_INTR_HIGH   (0x9 << GPIO_MODE_SHIFT) /* 1001 high level interrupt */
#  define GPIO_MODE_INTR_FALL   (0xA << GPIO_MODE_SHIFT) /* 1010 falling edge intr.   */
#  define GPIO_MODE_INTR_RISE   (0xB << GPIO_MODE_SHIFT) /* 1011 rising edge intr     */

/* Interrupt masked and flag is record (*IMF) */

#  define GPIO_MODE_IMF_LOW     (0xC << GPIO_MODE_SHIFT) /* 1100 low level interrupt  */
#  define GPIO_MODE_IMF_HIGH    (0xD << GPIO_MODE_SHIFT) /* 1101 high level interrupt */
#  define GPIO_MODE_IMF_FALL    (0xE << GPIO_MODE_SHIFT) /* 1110 falling edge intr.   */
#  define GPIO_MODE_IMF_RISE    (0xF << GPIO_MODE_SHIFT) /* 1111 rising edge intr     */

#define GPIO_PULL_EN            (1 << 8)  /* Bit 8:  pull-up/pull-down */

#define GPIO_PORT_SHIFT         (5)       /* Bits 5-7: Port number */
#define GPIO_PORT_MASK          (7 << GPIO_PORT_SHIFT)
#  define GPIO_PORTA            (0 << GPIO_PORT_SHIFT)
#  define GPIO_PORTB            (1 << GPIO_PORT_SHIFT)
#  define GPIO_PORTC            (2 << GPIO_PORT_SHIFT)
#  define GPIO_PORTD            (3 << GPIO_PORT_SHIFT)
#  define GPIO_PORTE            (4 << GPIO_PORT_SHIFT)
#  define GPIO_PORTF            (5 << GPIO_PORT_SHIFT)

#define GPIO_PIN_SHIFT          0        /* Bits 0-4: GPIO number: 0-31 */
#define GPIO_PIN_MASK           (31 << GPIO_PIN_SHIFT)

#  define GPIO_PIN0             (0  << GPIO_PIN_SHIFT)
#  define GPIO_PIN1             (1  << GPIO_PIN_SHIFT)
#  define GPIO_PIN2             (2  << GPIO_PIN_SHIFT)
#  define GPIO_PIN3             (3  << GPIO_PIN_SHIFT)
#  define GPIO_PIN4             (4  << GPIO_PIN_SHIFT)
#  define GPIO_PIN5             (5  << GPIO_PIN_SHIFT)
#  define GPIO_PIN6             (6  << GPIO_PIN_SHIFT)
#  define GPIO_PIN7             (7  << GPIO_PIN_SHIFT)
#  define GPIO_PIN8             (8  << GPIO_PIN_SHIFT)
#  define GPIO_PIN9             (9  << GPIO_PIN_SHIFT)
#  define GPIO_PIN10            (10 << GPIO_PIN_SHIFT)
#  define GPIO_PIN11            (11 << GPIO_PIN_SHIFT)
#  define GPIO_PIN12            (12 << GPIO_PIN_SHIFT)
#  define GPIO_PIN13            (13 << GPIO_PIN_SHIFT)
#  define GPIO_PIN14            (14 << GPIO_PIN_SHIFT)
#  define GPIO_PIN15            (15 << GPIO_PIN_SHIFT)
#  define GPIO_PIN16            (16 << GPIO_PIN_SHIFT)
#  define GPIO_PIN17            (17 << GPIO_PIN_SHIFT)
#  define GPIO_PIN18            (18 << GPIO_PIN_SHIFT)
#  define GPIO_PIN19            (19 << GPIO_PIN_SHIFT)
#  define GPIO_PIN20            (20 << GPIO_PIN_SHIFT)
#  define GPIO_PIN21            (21 << GPIO_PIN_SHIFT)
#  define GPIO_PIN22            (22 << GPIO_PIN_SHIFT)
#  define GPIO_PIN23            (23 << GPIO_PIN_SHIFT)
#  define GPIO_PIN24            (24 << GPIO_PIN_SHIFT)
#  define GPIO_PIN25            (25 << GPIO_PIN_SHIFT)
#  define GPIO_PIN26            (26 << GPIO_PIN_SHIFT)
#  define GPIO_PIN27            (27 << GPIO_PIN_SHIFT)
#  define GPIO_PIN28            (28 << GPIO_PIN_SHIFT)
#  define GPIO_PIN29            (29 << GPIO_PIN_SHIFT)
#  define GPIO_PIN30            (30 << GPIO_PIN_SHIFT)
#  define GPIO_PIN31            (31 << GPIO_PIN_SHIFT)

/*****************************************************************************
 * Public Types
 *****************************************************************************/

#ifndef __ASSEMBLY__

typedef uint32_t pinset_t;

/*****************************************************************************
 * Public Data
 *****************************************************************************/

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

/*****************************************************************************
 * Public Function Prototypes
 *****************************************************************************/

/*****************************************************************************
 * Name: jz4780_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin (the
 *   interrupt will be configured when jz4780_attach() is called).
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 *****************************************************************************/

int jz4780_configgpio(pinset_t cfgset);

/*****************************************************************************
 * Name: jz4780_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 *****************************************************************************/

void jz4780_gpiowrite(pinset_t pinset, bool value);

/*****************************************************************************
 * Name: jz4780_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 *****************************************************************************/

bool jz4780_gpioread(pinset_t pinset);

/*****************************************************************************
 * Name: jz4780_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a GPIO change notification interrupts.  This
 *   function is called internally by the system on power up and should not be
 *   called again.
 *
 *****************************************************************************/

#ifdef CONFIG_JZ4780_GPIOIRQ
void jz4780_gpioirqinitialize(void);
#else
#  define jz4780_gpioirqinitialize()
#endif

/*****************************************************************************
 * Name: jz4780_gpioattach
 *
 * Description:
 *   Attach an interrupt service routine to a GPIO interrupt.  This will also
 *   reconfigure the pin as an interrupting input.  The change notification
 *   number is associated with all interrupt-capable GPIO pins.  The
 *   association could, however, differ from part to part and must be provided
 *   by the caller.
 *
 *   When an interrupt occurs, it is due to a change on the GPIO input pin.
 *   In that case, all attached handlers will be called.  Each handler must
 *   maintain state and determine if the underlying GPIO input value changed.
 *
 *   pinset  - GPIO pin configuration
 *   handler - Interrupt handler (may be NULL to detach)
 *   arg     - The argument that accompanies the interrupt
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated error value is returned on
 *   any failure to indicate the nature of the failure.
 *
 *****************************************************************************/

#ifdef CONFIG_JZ4780_GPIOIRQ
int jz4780_gpioattach(uint32_t pinset, xcpt_t handler, void *arg);
#else
#  define jz4780_gpioattach(p,h,a) (0)
#endif

/*****************************************************************************
 * Name: jz4780_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 *****************************************************************************/

#ifdef CONFIG_JZ4780_GPIOIRQ
void jz4780_gpioirqenable(pinset_t pinset);
#else
#  define jz4780_gpioirqenable(irq)
#endif

/*****************************************************************************
 * Name: jz4780_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 *****************************************************************************/

#ifdef CONFIG_JZ4780_GPIOIRQ
void jz4780_gpioirqdisable(pinset_t pinset);
#else
#  define jz4780_gpioirqdisable(irq)
#endif

/*****************************************************************************
 * Function:  jz4780_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the provided base address
 *
 *****************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
void jz4780_dumpgpio(uint32_t pinset, const char *msg);
#else
#  define jz4780_dumpgpio(p,m)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_JZ4780_JZ4780_GPIO_H */
