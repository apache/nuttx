/****************************************************************************
 * arch/risc-v/src/rv32m1/rv32m1_gpio.h
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

#ifndef __ARCH_RISCV_SRC_RV32M1_RV32M1_GPIO_H
#define __ARCH_RISCV_SRC_RV32M1_RV32M1_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#include "riscv_internal.h"
#include "chip.h"
#include "hardware/rv32m1_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Each port bit of the general-purpose I/O (GPIO) ports can be
 * individually configured by software in several modes:
 *
 *  - Input floating
 *  - Input pull-up
 *  - Input-pull-down
 *  - Output open-drain with pull-up or pull-down capability
 *  - Output push-pull with pull-up or pull-down capability
 *  - Alternate function push-pull with pull-up or pull-down capability
 *  - Alternate function open-drain with pull-up or pull-down capability
 *  - Analog
 *
 * 20-bit Encoding:       1111 1111 1100 0000 0000
 *                        9876 5432 1098 7654 3210
 *                        ---- ---- ---- ---- ----
 * Inputs:                MMUU IIII FS.. PPPB BBBB
 * Outputs:               MMUU .... FSOV PPPB BBBB
 * Alternate Functions:   MMUU AAAA FSO. PPPB BBBB
 * Analog:                MM.. .... FS.. PPPB BBBB
 */

/* Mode:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * MM.. .... .... .... ....
 */

#define GPIO_MODE_SHIFT               (18)                       /* Bits 18-19: GPIO port mode */
#define GPIO_MODE_MASK                (3 << GPIO_MODE_SHIFT)
#  define GPIO_INPUT                  (0 << GPIO_MODE_SHIFT)     /* Input mode */
#  define GPIO_OUTPUT                 (1 << GPIO_MODE_SHIFT)     /* General purpose output mode */
#  define GPIO_ALT                    (2 << GPIO_MODE_SHIFT)     /* Alternate function mode */

/* Input/output pull-ups/downs (not used with analog):
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * ..UU .... .... .... ....
 */

#define GPIO_PUPD_SHIFT               (16)                       /* Bits 16-17: Pull-up/pull down */
#define GPIO_PUPD_MASK                (3 << GPIO_PUPD_SHIFT)
#  define GPIO_FLOAT                  (0 << GPIO_PUPD_SHIFT)     /* No pull-up, pull-down */
#  define GPIO_PULLUP                 (1 << GPIO_PUPD_SHIFT)     /* Pull-up */
#  define GPIO_PULLDOWN               (2 << GPIO_PUPD_SHIFT)     /* Pull-down */

/* Alternate Functions:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... AAAA .... .... ....
 */

#define GPIO_AF_SHIFT                 (12)                       /* Bits 12-15: Alternate function */
#define GPIO_AF_MASK                  (7 << GPIO_AF_SHIFT)
#  define GPIO_AF(n)                  ((n) << GPIO_AF_SHIFT)
#  define GPIO_AF0                    (0 << GPIO_AF_SHIFT)       /* Analog */
#  define GPIO_AF1                    (1 << GPIO_AF_SHIFT)       /* GPIO */
#  define GPIO_AF2                    (2 << GPIO_AF_SHIFT)
#  define GPIO_AF3                    (3 << GPIO_AF_SHIFT)
#  define GPIO_AF4                    (4 << GPIO_AF_SHIFT)
#  define GPIO_AF5                    (5 << GPIO_AF_SHIFT)
#  define GPIO_AF6                    (6 << GPIO_AF_SHIFT)
#  define GPIO_AF7                    (7 << GPIO_AF_SHIFT)

/* Interrupt for Input pins only:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... IIII .... .... ....
 */

#define GPIO_INT_SHIFT               (12)
#define GPIO_INT_MASK                (0xf << GPIO_INT_SHIFT)
#define GPIO_INT_NONE                (0   << GPIO_INT_SHIFT)
#define GPIO_INT_LOW                 (8   << GPIO_INT_SHIFT)
#define GPIO_INT_RISE                (9   << GPIO_INT_SHIFT)
#define GPIO_INT_FALL                (10  << GPIO_INT_SHIFT)
#define GPIO_INT_EDGE                (11  << GPIO_INT_SHIFT)
#define GPIO_INT_HIGH                (12  << GPIO_INT_SHIFT)

/* Passive Filter Enable:
 *
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... F... .... ....
 */

#define GPIO_FILTER                  (1 << 11)                  /* Bit11: 1=Passive Filter Enable */

/* Slow Slew Rate Enable (Default Fast Slew Rate):
 *
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... .S.. .... ....
 */

#define GPIO_SSR                     (1 << 10)                  /* Bit10: 1=Slow Slew rate, 0=Fast Slew Rate */

/* Output/Alt function type selection:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... ..O. .... ....
 */

#define GPIO_OPENDRAIN                (1 << 9)                  /* Bit9: 1=Open-drain output */

/* If the pin is a GPIO digital output, then this identifies the initial
 * output value.  If the pin is an input, this bit is overloaded to
 * provide the qualifier to distinguish input pull-up and -down:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... ...V .... ....
 */

#define GPIO_OUTPUT_SET               (1 << 8)                  /* Bit 8: If output, initial value of output */
#define GPIO_OUTPUT_CLEAR             (0)

/* This identifies the GPIO port:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... .... PPP. ....
 */

#define GPIO_PORT_SHIFT               (5)                       /* Bit 5-7:  Port number */
#define GPIO_PORT_MASK                (7 << GPIO_PORT_SHIFT)
#  define GPIO_PORTA                  (0 << GPIO_PORT_SHIFT)    /*   GPIOA */
#  define GPIO_PORTB                  (1 << GPIO_PORT_SHIFT)    /*   GPIOB */
#  define GPIO_PORTC                  (2 << GPIO_PORT_SHIFT)    /*   GPIOC */
#  define GPIO_PORTD                  (3 << GPIO_PORT_SHIFT)    /*   GPIOD */
#  define GPIO_PORTE                  (4 << GPIO_PORT_SHIFT)    /*   GPIOE */

/* This identifies the bit in the port:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... .... ...B BBBB
 */

#define GPIO_PIN_SHIFT                (0)                       /* Bits 0-4: GPIO number: 0-31 */
#define GPIO_PIN_MASK                 (31 << GPIO_PIN_SHIFT)
#  define GPIO_PIN0                   (0  << GPIO_PIN_SHIFT)
#  define GPIO_PIN1                   (1  << GPIO_PIN_SHIFT)
#  define GPIO_PIN2                   (2  << GPIO_PIN_SHIFT)
#  define GPIO_PIN3                   (3  << GPIO_PIN_SHIFT)
#  define GPIO_PIN4                   (4  << GPIO_PIN_SHIFT)
#  define GPIO_PIN5                   (5  << GPIO_PIN_SHIFT)
#  define GPIO_PIN6                   (6  << GPIO_PIN_SHIFT)
#  define GPIO_PIN7                   (7  << GPIO_PIN_SHIFT)
#  define GPIO_PIN8                   (8  << GPIO_PIN_SHIFT)
#  define GPIO_PIN9                   (9  << GPIO_PIN_SHIFT)
#  define GPIO_PIN10                  (10 << GPIO_PIN_SHIFT)
#  define GPIO_PIN11                  (11 << GPIO_PIN_SHIFT)
#  define GPIO_PIN12                  (12 << GPIO_PIN_SHIFT)
#  define GPIO_PIN13                  (13 << GPIO_PIN_SHIFT)
#  define GPIO_PIN14                  (14 << GPIO_PIN_SHIFT)
#  define GPIO_PIN15                  (15 << GPIO_PIN_SHIFT)
#  define GPIO_PIN16                  (16 << GPIO_PIN_SHIFT)
#  define GPIO_PIN17                  (17 << GPIO_PIN_SHIFT)
#  define GPIO_PIN18                  (18 << GPIO_PIN_SHIFT)
#  define GPIO_PIN19                  (19 << GPIO_PIN_SHIFT)
#  define GPIO_PIN20                  (20 << GPIO_PIN_SHIFT)
#  define GPIO_PIN21                  (21 << GPIO_PIN_SHIFT)
#  define GPIO_PIN22                  (22 << GPIO_PIN_SHIFT)
#  define GPIO_PIN23                  (23 << GPIO_PIN_SHIFT)
#  define GPIO_PIN24                  (24 << GPIO_PIN_SHIFT)
#  define GPIO_PIN25                  (25 << GPIO_PIN_SHIFT)
#  define GPIO_PIN26                  (26 << GPIO_PIN_SHIFT)
#  define GPIO_PIN27                  (27 << GPIO_PIN_SHIFT)
#  define GPIO_PIN28                  (28 << GPIO_PIN_SHIFT)
#  define GPIO_PIN29                  (29 << GPIO_PIN_SHIFT)
#  define GPIO_PIN30                  (30 << GPIO_PIN_SHIFT)
#  define GPIO_PIN31                  (31 << GPIO_PIN_SHIFT)

/****************************************************************************
 * Public Functions Prototypes
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

/****************************************************************************
 * Name: rv32m1_gpio_config
 ****************************************************************************/

EXTERN int rv32m1_gpio_config(uint32_t cfgset);

/****************************************************************************
 * Name: rv32m1_gpio_write
 ****************************************************************************/

EXTERN void rv32m1_gpio_write(uint32_t cfgset, bool value);

/****************************************************************************
 * Name: rv32m1_gpio_toggle
 ****************************************************************************/

EXTERN void rv32m1_gpio_toggle(uint32_t cfgset);

/****************************************************************************
 * Name: rv32m1_gpio_read
 ****************************************************************************/

EXTERN bool rv32m1_gpio_read(uint32_t cfgset);

/****************************************************************************
 * Name: rv32m1_gpio_irqenable
 ****************************************************************************/

EXTERN void rv32m1_gpio_irqenable(uint32_t cfgset);

/****************************************************************************
 * Name: rv32m1_gpio_irqdisable
 ****************************************************************************/

EXTERN void rv32m1_gpio_irqdisable(uint32_t cfgset);

/****************************************************************************
 * Name: rv32m1_gpio_irqattach
 ****************************************************************************/

EXTERN int rv32m1_gpio_irqattach(uint32_t cfgset, xcpt_t isr, void *arg);

/****************************************************************************
 * Name: rv32m1_gpio_irqdetach
 ****************************************************************************/

EXTERN int rv32m1_gpio_irqdetach(uint32_t cfgset, xcpt_t isr, void *arg);

/****************************************************************************
 * Name: rv32m1_gpio_clearpending
 ****************************************************************************/

EXTERN void rv32m1_gpio_clearpending(uint32_t cfgset);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_RV32M1_RV32M1_GPIO_H */
