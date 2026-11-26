/****************************************************************************
 * arch/arm/src/efr32/hardware/efr32_gpio.h
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_EFR32_HARDWARE_EFR32_GPIO_H
#define __ARCH_ARM_SRC_EFR32_HARDWARE_EFR32_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/efr32_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

/* Port registers.  Port n (A = 0, B = 1, ...) starts at 0x30 + 0x30 * n. */

#define EFR32_GPIO_PORT_OFFSET(n)        (0x0030 + 0x0030 * (n))

#define EFR32_GPIO_CTRL_OFFSET           0x0000  /* Port control */
#define EFR32_GPIO_MODEL_OFFSET          0x0004  /* Mode, pins 0-7 */
#define EFR32_GPIO_MODEH_OFFSET          0x000c  /* Mode, pins 8-15 */
#define EFR32_GPIO_DOUT_OFFSET           0x0010  /* Data out */
#define EFR32_GPIO_DIN_OFFSET            0x0014  /* Data in */

#define EFR32_GPIO_LOCK_OFFSET           0x0300  /* Configuration lock */

/* External interrupt registers */

#define EFR32_GPIO_EXTIPSELL_OFFSET      0x0400  /* Port select, EXTI0-7 */
#define EFR32_GPIO_EXTIPSELH_OFFSET      0x0404  /* Port select, EXTI8-11 */
#define EFR32_GPIO_EXTIPINSELL_OFFSET    0x0408  /* Pin select, EXTI0-7 */
#define EFR32_GPIO_EXTIPINSELH_OFFSET    0x040c  /* Pin select, EXTI8-11 */
#define EFR32_GPIO_EXTIRISE_OFFSET       0x0410  /* Rising edge enable */
#define EFR32_GPIO_EXTIFALL_OFFSET       0x0414  /* Falling edge enable */
#define EFR32_GPIO_IF_OFFSET             0x0420  /* Interrupt flags */
#define EFR32_GPIO_IEN_OFFSET            0x0424  /* Interrupt enables */

/* Peripheral route registers (USART0) */

#define EFR32_GPIO_USART0_ROUTEEN_OFFSET  0x0720 /* Pin enables */
#define EFR32_GPIO_USART0_CSROUTE_OFFSET  0x0724 /* CS pin */
#define EFR32_GPIO_USART0_CTSROUTE_OFFSET 0x0728 /* CTS pin */
#define EFR32_GPIO_USART0_RTSROUTE_OFFSET 0x072c /* RTS pin */
#define EFR32_GPIO_USART0_RXROUTE_OFFSET  0x0730 /* RX pin */
#define EFR32_GPIO_USART0_CLKROUTE_OFFSET 0x0734 /* CLK pin */
#define EFR32_GPIO_USART0_TXROUTE_OFFSET  0x0738 /* TX pin */

/* Register addresses *******************************************************/

#define EFR32_GPIO_PORT_BASE(n)  (EFR32_GPIO_BASE + EFR32_GPIO_PORT_OFFSET(n))

#define EFR32_GPIO_PORT_CTRL(n)  (EFR32_GPIO_PORT_BASE(n) + EFR32_GPIO_CTRL_OFFSET)
#define EFR32_GPIO_PORT_MODEL(n) (EFR32_GPIO_PORT_BASE(n) + EFR32_GPIO_MODEL_OFFSET)
#define EFR32_GPIO_PORT_MODEH(n) (EFR32_GPIO_PORT_BASE(n) + EFR32_GPIO_MODEH_OFFSET)
#define EFR32_GPIO_PORT_DOUT(n)  (EFR32_GPIO_PORT_BASE(n) + EFR32_GPIO_DOUT_OFFSET)
#define EFR32_GPIO_PORT_DIN(n)   (EFR32_GPIO_PORT_BASE(n) + EFR32_GPIO_DIN_OFFSET)

#define EFR32_GPIO_LOCK          (EFR32_GPIO_BASE + EFR32_GPIO_LOCK_OFFSET)

#define EFR32_GPIO_EXTIPSELL     (EFR32_GPIO_BASE + EFR32_GPIO_EXTIPSELL_OFFSET)
#define EFR32_GPIO_EXTIPSELH     (EFR32_GPIO_BASE + EFR32_GPIO_EXTIPSELH_OFFSET)
#define EFR32_GPIO_EXTIPINSELL   (EFR32_GPIO_BASE + EFR32_GPIO_EXTIPINSELL_OFFSET)
#define EFR32_GPIO_EXTIPINSELH   (EFR32_GPIO_BASE + EFR32_GPIO_EXTIPINSELH_OFFSET)
#define EFR32_GPIO_EXTIRISE      (EFR32_GPIO_BASE + EFR32_GPIO_EXTIRISE_OFFSET)
#define EFR32_GPIO_EXTIFALL      (EFR32_GPIO_BASE + EFR32_GPIO_EXTIFALL_OFFSET)
#define EFR32_GPIO_IF            (EFR32_GPIO_BASE + EFR32_GPIO_IF_OFFSET)
#define EFR32_GPIO_IEN           (EFR32_GPIO_BASE + EFR32_GPIO_IEN_OFFSET)

#define EFR32_GPIO_USART0_ROUTEEN  (EFR32_GPIO_BASE + EFR32_GPIO_USART0_ROUTEEN_OFFSET)
#define EFR32_GPIO_USART0_CTSROUTE (EFR32_GPIO_BASE + EFR32_GPIO_USART0_CTSROUTE_OFFSET)
#define EFR32_GPIO_USART0_RTSROUTE (EFR32_GPIO_BASE + EFR32_GPIO_USART0_RTSROUTE_OFFSET)
#define EFR32_GPIO_USART0_RXROUTE  (EFR32_GPIO_BASE + EFR32_GPIO_USART0_RXROUTE_OFFSET)
#define EFR32_GPIO_USART0_TXROUTE  (EFR32_GPIO_BASE + EFR32_GPIO_USART0_TXROUTE_OFFSET)

/* Register bit definitions *************************************************/

/* Port mode: 4 bits per pin, pins 0-7 in MODEL and 8-15 in MODEH */

#define GPIO_MODE_PIN_SHIFT(pin)          (((pin) & 7) << 2)
#define GPIO_MODE_PIN_MASK(pin)           (0xf << GPIO_MODE_PIN_SHIFT(pin))

#define GPIO_MODE_DISABLED               0   /* Input disabled */
#define GPIO_MODE_INPUT                  1   /* Input */
#define GPIO_MODE_INPUTPULL              2   /* Input, pull set by DOUT */
#define GPIO_MODE_INPUTPULLFILTER        3   /* As above with glitch filter */
#define GPIO_MODE_PUSHPULL               4   /* Push-pull output */
#define GPIO_MODE_PUSHPULLALT            5   /* Push-pull, alternate ctrl */
#define GPIO_MODE_WIREDOR                6   /* Open source */
#define GPIO_MODE_WIREDORPULLDOWN        7   /* Open source with pull-down */
#define GPIO_MODE_WIREDAND               8   /* Open drain */
#define GPIO_MODE_WIREDANDFILTER         9   /* Open drain with filter */
#define GPIO_MODE_WIREDANDPULLUP         10  /* Open drain with pull-up */
#define GPIO_MODE_WIREDANDPULLUPFILTER   11  /* Open drain, pull-up, filter */

/* External interrupts.  EXTIn selects a port in EXTIPSELL/H and one of
 * the pins 4 * (n / 4) + 0..3 in EXTIPINSELL/H (4 bits per line, 2 used).
 */

#define GPIO_NEXTI                       12
#define GPIO_EXTI_SHIFT(n)               (((n) & 7) << 2)
#define GPIO_EXTI_MASK(n)                (0x3 << GPIO_EXTI_SHIFT(n))
#define GPIO_EXTI_ALL                    0x00000fff
#define GPIO_EXTI_EVEN                   0x00000555
#define GPIO_EXTI_ODD                    0x00000aaa

/* Configuration lock */

#define GPIO_LOCK_LOCKKEY_UNLOCK         0xa534

/* Route enable (USART0) */

#define GPIO_USART_ROUTEEN_CSPEN         (1 << 0)
#define GPIO_USART_ROUTEEN_RTSPEN        (1 << 1)
#define GPIO_USART_ROUTEEN_RXPEN         (1 << 2)
#define GPIO_USART_ROUTEEN_CLKPEN        (1 << 3)
#define GPIO_USART_ROUTEEN_TXPEN         (1 << 4)

/* Pin route: port in bits 1:0, pin in bits 19:16 */

#define GPIO_ROUTE_PORT_SHIFT            (0)
#define GPIO_ROUTE_PORT_MASK             (0x3 << GPIO_ROUTE_PORT_SHIFT)
#define GPIO_ROUTE_PIN_SHIFT             (16)
#define GPIO_ROUTE_PIN_MASK              (0xf << GPIO_ROUTE_PIN_SHIFT)
#define GPIO_ROUTE(port, pin) \
  (((uint32_t)(port) << GPIO_ROUTE_PORT_SHIFT) | \
   ((uint32_t)(pin) << GPIO_ROUTE_PIN_SHIFT))

#endif /* __ARCH_ARM_SRC_EFR32_HARDWARE_EFR32_GPIO_H */
