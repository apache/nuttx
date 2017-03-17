/****************************************************************************
 *  arch/arm/src/xmc4/xmc4_gpio.h
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "chip/xmc4_ports.h"

/****************************************************************************
 * Preprocessor Definitions
 ****************************************************************************/

/* 32-bit GIO encoding:
 *
 *   .... TTTT TMDD DCC.  .... .... PPPP BBBB
 */


/* This identifies the GPIO pint type:
 *
 *   .... TTTT T... ....  .... .... .... ....
 */

#define GPIO_PINTYPE_SHIFT         (23)       /* Bits 23-27: Pin type */
#define GPIO_PINTYPE_MASK          (31                  << GPIO_PINTYPE_SHIFT)

/* See chip/xmc4_ports.h for the IOCR definitions */
/* Direct input */

#  define GPIO_INPUT_NOPULL        (IOCR_INPUT_NOPULL       << GPIO_PINTYPE_SHIFT)
#  define GPIO_INPUT_PULLDOWN      (IOCR_INPUT_PULLDOWN     << GPIO_PINTYPE_SHIFT)
#  define GPIO_INPUT_PULLUP        (IOCR_INPUT_PULLUP       << GPIO_PINTYPE_SHIFT)
#  define GPIO_INPUT_CONT          (IOCR_INPUT_CONT         << GPIO_PINTYPE_SHIFT)

/* Push-pull Output (direct input) */

#  define GPIO_OUTPUT              (IOCR_OUTPUT             << GPIO_PINTYPE_SHIFT)
#  define GPIO_OUTPUT_ALT1         (IOCR_OUTPUT_ALT1        << GPIO_PINTYPE_SHIFT)
#  define GPIO_OUTPUT_ALT2         (IOCR_OUTPUT_ALT2        << GPIO_PINTYPE_SHIFT)
#  define GPIO_OUTPUT_ALT3         (IOCR_OUTPUT_ALT3        << GPIO_PINTYPE_SHIFT)
#  define GPIO_OUTPUT_ALT4         (IOCR_OUTPUT_ALT4        << GPIO_PINTYPE_SHIFT)

#  define _GPIO_OUTPUT_BIT         (16                      << GPIO_PINTYPE_SHIFT)
#  define GPIO_ISINPUT(p)          (((p) & _GPIO_OUTPUT_BIT) != 0)
#  define GPIO_ISOUTPUT(p)         (((p) & _GPIO_OUTPUT_BIT) == 0)

/* Pin type modifier:
 *
 *   .... .... .M.. ....  .... .... .... ....
 */

#define GPIO_INPUT_INVERT           (1 << 22) /* Inverted input modifier */
#define GPIOS_OUTPUT_OPENDRAIN       (1 << 22) /* Output drain output modifier */

/* Pad driver strength:
 *
 *   .... .... ..DD D...  .... .... .... ....
 */

#define GPIO_PADTYPE_SHIFT          (19)       /* Bits 19-21: Pad driver strength */
#define GPIO_PADTYPE_MASK           (7                      << GPIO_PADTYPE_SHIFT)

/* See chip/xmc4_ports.h for the PDR definitions */
/* Pad class A1: */

#  define GPIO_PADA1_MEDIUM         (PDR_PADA1_MEDIUM       << GPIO_PADTYPE_SHIFT)
#  define GPIO_PADA1_WEAK           (PDR_PADA1_WEAK         << GPIO_PADTYPE_SHIFT)

/* Pad class A1+: */

#  define GPIO_PADA1P_STRONGSOFT    (PDR_PADA1P_STRONGSOFT  << GPIO_PADTYPE_SHIFT)
#  define GPIO_PADA1P_STRONGSLOW    (PDR_PADA1P_STRONGSLOW  << GPIO_PADTYPE_SHIFT)
#  define GPIO_PADA1P_MEDIUM        (PDR_PADA1P_MEDIUM      << GPIO_PADTYPE_SHIFT)
#  define GPIO_PADA1P_WEAK          (PDR_PADA1P_WEAK        << GPIO_PADTYPE_SHIFT)

/* Pad class A2: */

#  define GPIO_PADA2_STRONGSHARP    (PDR_PADA2_STRONGSHARP  << GPIO_PADTYPE_SHIFT)
#  define GPIO_PADA2_STRONGMEDIUM   (PDR_PADA2_STRONGMEDIUM << GPIO_PADTYPE_SHIFT)
#  define GPIO_PADA2_STRONGSOFT     (PDR_PADA2_STRONGSOFT   << GPIO_PADTYPE_SHIFT)
#  define GPIO_PADA2_MEDIUM         (PDR_PADA2_MEDIUM       << GPIO_PADTYPE_SHIFT)
#  define GPIO_PADA2_WEAK           (PDR_PADA2_WEAK         << GPIO_PADTYPE_SHIFT)

/* Pin control:
 *
 *   .... .... .... .CC.  .... .... .... ....
 */

#define GPIO_PINCTRL_SHIFT          (17)       /* Bits 17-18: Pad driver strength */
#define GPIO_PINCTRL_MASK           (3                      << GPIO_PINCTRL_SHIFT)

/* See chip/xmc4_ports.h for the PDR definitions */

#  define GPIO_PINCTRL_SOFTWARE     (HWSEL_SOFTWARE         << GPIO_PINCTRL_SHIFT)
#  define GPIO_PINCTRL_OVERRIDE0    (HWSEL_OVERRIDE0        << GPIO_PINCTRL_SHIFT)
#  define GPIO_PINCTRL_OVERRIDE1    (HWSEL_OVERRIDE1        << GPIO_PINCTRL_SHIFT)

/* This identifies the GPIO port:
 *
 *   .... ... .... ....  .... .... PPPP ....
 */

#define GPIO_PORT_SHIFT            (4)         /* Bit 4-7:  Port number */
#define GPIO_PORT_MASK             (7 << GPIO_PORT_SHIFT)
#  define GPIO_PORT0               (0 << GPIO_PORT_SHIFT)
#  define GPIO_PORT1               (1 << GPIO_PORT_SHIFT)
#  define GPIO_PORT2               (2 << GPIO_PORT_SHIFT)
#  define GPIO_PORT3               (3 << GPIO_PORT_SHIFT)
#  define GPIO_PORT4               (4 << GPIO_PORT_SHIFT)
#  define GPIO_PORT5               (5 << GPIO_PORT_SHIFT)
#  define GPIO_PORT6               (6 << GPIO_PORT_SHIFT)
#  define GPIO_PORT7               (7 << GPIO_PORT_SHIFT)
#  define GPIO_PORT8               (8 << GPIO_PORT_SHIFT)
#  define GPIO_PORT9               (9 << GPIO_PORT_SHIFT)
#  define GPIO_PORT14              (14 << GPIO_PORT_SHIFT)
#  define GPIO_PORT15              (15 << GPIO_PORT_SHIFT)

/* This identifies the bit in the port:
 *
 *   ... ..... .... ....  .... .... .... BBBB
 */

#define GPIO_PIN_SHIFT             (0)         /* Bits 0-3: GPIO pin: 0-15 */
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


/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This is a type large enought to hold all pin configuration bits. */

typedef uint32_t gpioconfig_t;

/****************************************************************************
 * Public Functions
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
