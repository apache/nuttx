/************************************************************************************
 * arch/arm/src/sam34/sam4l_gpio.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAM34_SAM4L_GPIO_H
#define __ARCH_ARM_SRC_SAM34_SAM4L_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Bit-encoded input to sam_configgpio() ********************************************/

/* 24-bit Encoding.  This could be compacted into 16-bits by making the bit usage
 * mode specific.  However, by giving each bit field a unique position, we handle
 * bad combinations of properties safely.
 *
 *   MODE         BITFIELDS
 *   ------------ -----------------------------
 *                2222 1111 1111 1100 0000 0000
 *                3210 9876 5432 1098 7654 3210
 *   ------------ -----------------------------
 *   GPIO Input:  MMRR .... .... IIGT .PPB BBBB
 *   GPIO Output: MM.. .... DDSV .... .PPB BBBB
 *   Peripheral:  MM.. FFFE .... IIG. .PPB BBBB
 *   ------------ -----------------------------
 *                MMRR FFFE DDSV IIGT .PPB BBBB
 */

/* Input/output/peripheral mode:
 *
 *   MODE         BITFIELDS
 *   ------------ -----------------------------
 *                2222 1111 1111 1100 0000 0000
 *                3210 9876 5432 1098 7654 3210
 *   ------------ -----------------------------
 *   GPIO Input:  MM.. .... .... .... .... ....
 *   GPIO Output: MM.. .... .... .... .... ....
 *   Peripheral:  MM.. .... .... .... .... ....
 */

#define GPIO_MODE_SHIFT            (22)        /* Bits 22-23: GPIO mode */
#define GPIO_MODE_MASK             (3 << GPIO_MODE_SHIFT)
#  define GPIO_INPUT               (0 << GPIO_MODE_SHIFT) /* GPIO Input */
#  define GPIO_OUTPUT              (1 << GPIO_MODE_SHIFT) /* GPIO Output */
#  define GPIO_PERIPHERAL          (2 << GPIO_MODE_SHIFT) /* Controlled by peripheral */
#  define GPIO_INTERRUPT           (3 << GPIO_MODE_SHIFT) /* Interrupting input */

/* Pull-up/down resistor control for inputs
 *
 *   MODE         BITFIELDS
 *   ------------ -----------------------------
 *                2222 1111 1111 1100 0000 0000
 *                3210 9876 5432 1098 7654 3210
 *   ------------ -----------------------------
 *   GPIO Input:  ..RR .... .... .... .... ....
 *   GPIO Output: .... .... .... .... .... ....
 *   Peripheral:  .... .... .... .... .... ....
 */

#define GPIO_PULL_SHIFT            (20)       /* Bits 20-21: Pull-up/down resistor control */
#define GPIO_PULL_MASK             (3 << GPIO_PULL_SHIFT)
#  define GPIO_PULL_NONE           (0 << GPIO_PULL_SHIFT)
#  define GPIO_PULL_UP             (1 << GPIO_PULL_SHIFT)
#  define GPIO_PULL_DOWN           (2 << GPIO_PULL_SHIFT)
#  define GPIO_PULL_BUSKEEPER      (3 << GPIO_PULL_SHIFT)

/* Peripheral Function
 *
 *   MODE         BITFIELDS
 *   ------------ -----------------------------
 *                2222 1111 1111 1100 0000 0000
 *                3210 9876 5432 1098 7654 3210
 *   ------------ -----------------------------
 *   GPIO Input:  .... .... .... .... .... ....
 *   GPIO Output: .... .... .... .... .... ....
 *   Peripheral:  .... FFF. .... .... .... ....
 */

#define GPIO_FUNC_SHIFT            (17)       /* Bits 17-19: Peripheral function */
#define GPIO_FUNC_MASK             (7 << GPIO_FUNC_SHIFT)
#  define _GPIO_FUNCA              (0 << GPIO_FUNC_SHIFT) /* Function A */
#  define _GPIO_FUNCB              (1 << GPIO_FUNC_SHIFT) /* Function B */
#  define _GPIO_FUNCC              (2 << GPIO_FUNC_SHIFT) /* Function C */
#  define _GPIO_FUNCD              (3 << GPIO_FUNC_SHIFT) /* Function D */
#  define _GPIO_FUNCE              (4 << GPIO_FUNC_SHIFT) /* Function E */
#  define _GPIO_FUNCF              (5 << GPIO_FUNC_SHIFT) /* Function F */
#  define _GPIO_FUNCG              (6 << GPIO_FUNC_SHIFT) /* Function G */
#  define _GPIO_FUNCH              (7 << GPIO_FUNC_SHIFT) /* Function H */

/* Extended input/output/peripheral mode:
 *
 *   MODE         BITFIELDS
 *   ------------ -----------------------------
 *                2222 1111 1111 1100 0000 0000
 *                3210 9876 5432 1098 7654 3210
 *   ------------ -----------------------------
 *   GPIO Input:  .... .... .... .... .... ....
 *   GPIO Output: .... .... .... .... .... ....
 *   Peripheral:  MM.. FFF. .... .... .... ....
 */

#define GPIO_FUNCA                 (GPIO_PERIPHERAL | _GPIO_FUNCA) /* Function A */
#define GPIO_FUNCB                 (GPIO_PERIPHERAL | _GPIO_FUNCB) /* Function B */
#define GPIO_FUNCC                 (GPIO_PERIPHERAL | _GPIO_FUNCC) /* Function C */
#define GPIO_FUNCD                 (GPIO_PERIPHERAL | _GPIO_FUNCD) /* Function D */
#define GPIO_FUNCE                 (GPIO_PERIPHERAL | _GPIO_FUNCE) /* Function E */
#define GPIO_FUNCF                 (GPIO_PERIPHERAL | _GPIO_FUNCF) /* Function F */
#define GPIO_FUNCG                 (GPIO_PERIPHERAL | _GPIO_FUNCG) /* Function G */
#define GPIO_FUNCH                 (GPIO_PERIPHERAL | _GPIO_FUNCH) /* Function H */

/* Peripheral event control
 *
 *   MODE         BITFIELDS
 *   ------------ -----------------------------
 *                2222 1111 1111 1100 0000 0000
 *                3210 9876 5432 1098 7654 3210
 *   ------------ -----------------------------
 *   GPIO Input:  .... .... .... .... .... ....
 *   GPIO Output: .... .... .... .... .... ....
 *   Peripheral:  .... ...E .... .... .... ....
 */

#define GPIO_PERIPH_EVENTS        (1 << 16)  /* Bit 16: Enable peripheral events */

/* Output drive control
 *
 *   MODE         BITFIELDS
 *   ------------ -----------------------------
 *                2222 1111 1111 1100 0000 0000
 *                3210 9876 5432 1098 7654 3210
 *   ------------ -----------------------------
 *   GPIO Input:  .... .... .... .... .... ....
 *   GPIO Output: .... .... DD.. .... .... ....
 *   Peripheral:  .... .... .... .... .... ....
 */

#define GPIO_DRIVE_SHIFT           (14)       /* Bits 14-15: Interrupting input control */
#define GPIO_DRIVE_MASK            (3 << GPIO_INT_SHIFT) /* Lowest drive strength*/
#  define GPIO_DRIVE_LOW           (0 << GPIO_INT_SHIFT)
#  define GPIO_DRIVE_MEDLOW        (1 << GPIO_INT_SHIFT)
#  define GPIO_DRIVE_MEDHIGH       (2 << GPIO_INT_SHIFT)
#  define GPIO_DRIVE_HIGH          (3 << GPIO_INT_SHIFT) /* Highest drive strength */

/* Output slew rate control
 *
 *   MODE         BITFIELDS
 *   ------------ -----------------------------
 *                2222 1111 1111 1100 0000 0000
 *                3210 9876 5432 1098 7654 3210
 *   ------------ -----------------------------
 *   GPIO Input:  .... .... .... .... .... ....
 *   GPIO Output: .... .... ..S. .... .... ....
 *   Peripheral:  .... .... .... .... .... ....
 */

#define GPIO_SLEW                  (1 << 13)  /* Bit 13: Enable output slew control */

/* If the pin is an GPIO output, then this identifies the initial output value:
 *
 *   MODE         BITFIELDS
 *   ------------ -----------------------------
 *                2222 1111 1111 1100 0000 0000
 *                3210 9876 5432 1098 7654 3210
 *   ------------ -----------------------------
 *   GPIO Input:  .... .... .... .... .... ....
 *   GPIO Output: .... .... ...V .... .... ....
 *   Peripheral:  .... .... .... .... .... ....
 */

#define GPIO_OUTPUT_SET            (1 << 12)    /* Bit 12: Inital value of output */
#define GPIO_OUTPUT_CLEAR          (0)

/* Selections for an interrupting input and peripheral events:
 *
 *   MODE         BITFIELDS
 *   ------------ -----------------------------
 *                2222 1111 1111 1100 0000 0000
 *                3210 9876 5432 1098 7654 3210
 *   ------------ -----------------------------
 *   GPIO Input:  .... .... .... II.. .... ....
 *   GPIO Output: .... .... .... .... .... ....
 *   Peripheral:  .... .... .... II.. .... ....
 */

#define GPIO_INT_SHIFT             (10)       /* Bits 10-11: Interrupting input control */
#define GPIO_INT_MASK              (3 << GPIO_INT_SHIFT)
#  define GPIO_INT_CHANGE          (0 << GPIO_INT_SHIFT) /* Pin change */
#  define GPIO_INT_RISING          (1 << GPIO_INT_SHIFT) /* Rising edge */
#  define GPIO_INT_FALLING         (2 << GPIO_INT_SHIFT) /* Falling edge */

/* These combinations control events.  These help to clean up pin definitions. */

#define GPIO_EVENT_CHANGE          (GPIO_PERIPH_EVENTS | GPIO_INT_CHANGE) /* Pin change */
#define GPIO_EVENT_RISING          (GPIO_PERIPH_EVENTS | GPIO_INT_RISING) /* Rising edge */
#define GPIO_EVENT_FALLING         (GPIO_PERIPH_EVENTS | GPIO_INT_FALLING) /* Falling edge */

/* Enable input/periphal glitch filter
 *
 *   MODE         BITFIELDS
 *   ------------ -----------------------------
 *                2222 1111 1111 1100 0000 0000
 *                3210 9876 5432 1098 7654 3210
 *   ------------ -----------------------------
 *   GPIO Input:  .... .... .... ..G. .... ....
 *   GPIO Output: .... .... .... .... .... ....
 *   Peripheral:  .... .... .... ..G. .... ....
 */

#define GPIO_GLITCH_FILTER         (1 << 9)   /* Bit 9: Enable input/peripheral glitch filter */

/* Input Schmitt trigger
 *
 *   MODE         BITFIELDS
 *   ------------ -----------------------------
 *                2222 1111 1111 1100 0000 0000
 *                3210 9876 5432 1098 7654 3210
 *   ------------ -----------------------------
 *   GPIO Input:  .... .... .... ...T .... ....
 *   GPIO Output: .... .... .... .... .... ....
 *   Peripheral:  .... .... .... .... .... ....
 */

#define GPIO_SCHMITT_TRIGGER       (1 << 8)   /* Bit 8: Enable Input Schmitt trigger */

/* This identifies the GPIO port:
 *
 *   MODE         BITFIELDS
 *   ------------ -----------------------------
 *                2222 1111 1111 1100 0000 0000
 *                3210 9876 5432 1098 7654 3210
 *   ------------ -----------------------------
 *   GPIO Input:  .... .... .... .... .PP. ....
 *   GPIO Output: .... .... .... .... .PP. ....
 *   Peripheral:  .... .... .... .... .PP. ....
 */

#define GPIO_PORT_SHIFT            (5)         /* Bit 5-6:  Port number */
#define GPIO_PORT_MASK             (3 << GPIO_PORT_SHIFT)
#  define GPIO_PORTA               (0 << GPIO_PORT_SHIFT)
#  define GPIO_PORTB               (1 << GPIO_PORT_SHIFT)
#  define GPIO_PORTC               (2 << GPIO_PORT_SHIFT)

/* This identifies the bit in the port:
 *
 *   MODE         BITFIELDS
 *   ------------ -----------------------------
 *                2222 1111 1111 1100 0000 0000
 *                3210 9876 5432 1098 7654 3210
 *   ------------ -----------------------------
 *   GPIO Input:  .... .... .... .... ...B BBBB
 *   GPIO Output: .... .... .... .... ...B BBBB
 *   Peripheral:  .... .... .... .... ...B BBBB
 */

#define GPIO_PIN_SHIFT             0        /* Bits 0-4: GPIO number: 0-31 */
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

/* Must be big enough to hold the 24-bit encoding */

typedef uint32_t gpio_pinset_t;

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

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_SAM34_SAM4L_GPIO_H */
