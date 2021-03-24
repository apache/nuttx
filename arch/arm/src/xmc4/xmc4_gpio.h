/****************************************************************************
 * arch/arm/src/xmc4/xmc4_gpio.h
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "hardware/xmc4_ports.h"

/****************************************************************************
 * Preprocessor Definitions
 ****************************************************************************/

/* 32-bit GIO encoding:
 *
 *   TTTT TMPD DDCC VO...  .... .... PPPP BBBB
 */

/* This identifies the GPIO pint type:
 *
 *   TTTT T... .... ....  .... .... .... ....
 */

#define GPIO_PINTYPE_SHIFT         (27)       /* Bits 27-31: Pin type */
#define GPIO_PINTYPE_MASK          (31                  << GPIO_PINTYPE_SHIFT)

/* See chip/xmc4_ports.h for the IOCR definitions */

/* Direct input */

#  define GPIO_INPUT               (IOCR_INPUT_NOPULL   << GPIO_PINTYPE_SHIFT)
#  define GPIO_INPUT_PULLDOWN      (IOCR_INPUT_PULLDOWN << GPIO_PINTYPE_SHIFT)
#  define GPIO_INPUT_PULLUP        (IOCR_INPUT_PULLUP   << GPIO_PINTYPE_SHIFT)
#  define GPIO_INPUT_CONT          (IOCR_INPUT_CONT     << GPIO_PINTYPE_SHIFT)

/* Push-pull Output (direct input) */

#  define GPIO_OUTPUT              (IOCR_OUTPUT         << GPIO_PINTYPE_SHIFT)
#  define GPIO_OUTPUT_ALT1         (IOCR_OUTPUT_ALT1    << GPIO_PINTYPE_SHIFT)
#  define GPIO_OUTPUT_ALT2         (IOCR_OUTPUT_ALT2    << GPIO_PINTYPE_SHIFT)
#  define GPIO_OUTPUT_ALT3         (IOCR_OUTPUT_ALT3    << GPIO_PINTYPE_SHIFT)
#  define GPIO_OUTPUT_ALT4         (IOCR_OUTPUT_ALT4    << GPIO_PINTYPE_SHIFT)

#  define _GPIO_OUTPUT_BIT         (16                      << GPIO_PINTYPE_SHIFT)
#  define GPIO_ISINPUT(p)          (((p) & _GPIO_OUTPUT_BIT) == 0)
#  define GPIO_ISOUTPUT(p)         (((p) & _GPIO_OUTPUT_BIT) != 0)

/* Pin type modifier:
 *
 *   .... .M.. .... .O..  .... .... .... ....
 */

#define GPIO_INPUT_INVERT          (1 << 26) /* Bit 26: Inverted direct input modifier */

#define GPIO_OUTPUT_OPENDRAIN      (1 << 18) /* Bit 18: Output drain output modifier */
#define GPIO_OUTPUT_PUSHPULL       (0)       /*         Push-pull output is the default */

/* Disable PAD:
 *
 *   .... ..P. .... .....  .... .... .... ....
 *
 * For P0-P6, the PDISC register is ready only.
 * For P14-P15, the bit setting also selects Analog+Digital or Analog only
 */

#define GPIO_PAD_DISABLE           (1 << 25) /* Bit 25: Disable Pad (P7-P9) */
#define GPIO_PAD_ANALOG            (1 << 25) /* Bit 25: Analog only (P14-P15) */

/* Pad driver strength:
 *
 *   .... ...D DD.. .....  .... ......... ....
 */

#define GPIO_PADTYPE_SHIFT         (22)       /* Bits 22-24: Pad driver strength */
#define GPIO_PADTYPE_MASK          (7                      << GPIO_PADTYPE_SHIFT)

/* See chip/xmc4_ports.h for the PDR definitions */

/* Pad class A1: */

#  define GPIO_PADA1_MEDIUM        (PDR_PADA1_MEDIUM       << GPIO_PADTYPE_SHIFT)
#  define GPIO_PADA1_WEAK          (PDR_PADA1_WEAK         << GPIO_PADTYPE_SHIFT)

/* Pad class A1+: */

#  define GPIO_PADA1P_STRONGSOFT   (PDR_PADA1P_STRONGSOFT  << GPIO_PADTYPE_SHIFT)
#  define GPIO_PADA1P_STRONGSLOW   (PDR_PADA1P_STRONGSLOW  << GPIO_PADTYPE_SHIFT)
#  define GPIO_PADA1P_MEDIUM       (PDR_PADA1P_MEDIUM      << GPIO_PADTYPE_SHIFT)
#  define GPIO_PADA1P_WEAK         (PDR_PADA1P_WEAK        << GPIO_PADTYPE_SHIFT)

/* Pad class A2: */

#  define GPIO_PADA2_STRONGSHARP   (PDR_PADA2_STRONGSHARP  << GPIO_PADTYPE_SHIFT)
#  define GPIO_PADA2_STRONGMEDIUM  (PDR_PADA2_STRONGMEDIUM << GPIO_PADTYPE_SHIFT)
#  define GPIO_PADA2_STRONGSOFT    (PDR_PADA2_STRONGSOFT   << GPIO_PADTYPE_SHIFT)
#  define GPIO_PADA2_MEDIUM        (PDR_PADA2_MEDIUM       << GPIO_PADTYPE_SHIFT)
#  define GPIO_PADA2_WEAK          (PDR_PADA2_WEAK         << GPIO_PADTYPE_SHIFT)

/* Pin control:
 *
 *   .... .... ..CC .....  .... .... .... ....
 */

#define GPIO_PINCTRL_SHIFT         (20)       /* Bits 20-21: Pad driver strength */
#define GPIO_PINCTRL_MASK          (3                      << GPIO_PINCTRL_SHIFT)

/* See chip/xmc4_ports.h for the PDR definitions */

#  define GPIO_PINCTRL_SOFTWARE    (HWSEL_SW               << GPIO_PINCTRL_SHIFT)
#  define GPIO_PINCTRL_HW0         (HWSEL_HW0              << GPIO_PINCTRL_SHIFT)
#  define GPIO_PINCTRL_HW1         (HWSEL_HW1              << GPIO_PINCTRL_SHIFT)

/* If the pin is an GPIO output, then this identifies the initial output
 * value:
 *
 *   .... .... .... V....  .... .... PPPP BBBB
 */

#define GPIO_OUTPUT_SET            (1 << 19)   /* Bit 19: Initial value of output */
#define GPIO_OUTPUT_CLEAR          (0)

/* This identifies the GPIO port:
 *
 *   .... .... .... ....  .... .... PPPP ....
 */

#define GPIO_PORT_SHIFT            (4)         /* Bit 4-7:  Port number */
#define GPIO_PORT_MASK             (15 << GPIO_PORT_SHIFT)
#  define GPIO_PORT0               (0  << GPIO_PORT_SHIFT)
#  define GPIO_PORT1               (1  << GPIO_PORT_SHIFT)
#  define GPIO_PORT2               (2  << GPIO_PORT_SHIFT)
#  define GPIO_PORT3               (3  << GPIO_PORT_SHIFT)
#  define GPIO_PORT4               (4  << GPIO_PORT_SHIFT)
#  define GPIO_PORT5               (5  << GPIO_PORT_SHIFT)
#  define GPIO_PORT6               (6  << GPIO_PORT_SHIFT)
#  define GPIO_PORT7               (7  << GPIO_PORT_SHIFT)
#  define GPIO_PORT8               (8  << GPIO_PORT_SHIFT)
#  define GPIO_PORT9               (9  << GPIO_PORT_SHIFT)
#  define GPIO_PORT14              (14 << GPIO_PORT_SHIFT)
#  define GPIO_PORT15              (15 << GPIO_PORT_SHIFT)

/* This identifies the bit in the port:
 *
 *   ... ..... .... ....  .... .... .... BBBB
 */

#define GPIO_PIN_SHIFT             (0)         /* Bits 0-3: GPIO pin: 0-15 */
#define GPIO_PIN_MASK              (15 << GPIO_PIN_SHIFT)
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

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This is a type large enough to hold all pin configuration bits. */

typedef uint32_t gpioconfig_t;

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: xmc4_gpio_config
 *
 * Description:
 *   Configure a PIN based on bit-encoded description of the pin,
 *   'pincconfig'.
 *
 ****************************************************************************/

int xmc4_gpio_config(gpioconfig_t pinconfig);

/****************************************************************************
 * Name: xmc4_gpio_write
 *
 * Description:
 *   Write one or zero to the PORT pin selected by 'pinconfig'
 *
 ****************************************************************************/

void xmc4_gpio_write(gpioconfig_t pinconfig, bool value);

/****************************************************************************
 * Name: xmc4_gpio_read
 *
 * Description:
 *   Read one or zero from the PORT pin selected by 'pinconfig'
 *
 ****************************************************************************/

bool xmc4_gpio_read(gpioconfig_t pinconfig);
