/************************************************************************************
 * arch/arm/src/max326xx/max32660/max32660_gpio.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_MAX326XX_MAX32660_MAX32660_GPIO_H
#define __ARCH_ARM_SRC_MAX326XX_MAX32660_MAX32660_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Bit-encoded input to max326_gpio_config() ******************************************/

/* 16-Bit Encoding: FFFF WRRV HSDD NNNN
 *
 *   Pin Function:          FFFF
 *   Wakeup:                W
 *   Pin Pull Up/Down:      RR
 *   Initial Value:         V (output pins)
 *   Input Hysteresis:      H
 *   Slew Rate:             S
 *   Drive Strength:        DD
 *   Port Number:           (implicitly 0)
 *   Pin Number:            NNNN (0-13)
 */

/* Pin Function bits:
 * Only meaningful when the GPIO function is GPIO_PIN
 *
 * FFFF .... .... ....
 */

#define GPIO_FUNC_SHIFT     (12)                      /* Bits 12-15: Pin Function */
#define GPIO_FUNC_MASK      (15 << GPIO_FUNC_SHIFT)
#  define GPIO_INPUT        (0 << GPIO_FUNC_SHIFT)    /* 0000 GPIO input pin */
#  define GPIO_OUTPUT       (1 << GPIO_FUNC_SHIFT)    /* 0001 GPIO output pin */
#  define GPIO_ALT1         (5 << GPIO_FUNC_SHIFT)    /* 0100 GPIO Alternate function 1 */
#  define GPIO_ALT2         (6 << GPIO_FUNC_SHIFT)    /* 0101 GPIO Alternate function 2 */
#  define GPIO_ALT3         (7 << GPIO_FUNC_SHIFT)    /* 0110 GPIO Alternate function 3 */
#  define GPIO_INTFE        (9 << GPIO_FUNC_SHIFT)    /* 1001 GPIO interrupt falling edge */
#  define GPIO_INTRE        (10 << GPIO_FUNC_SHIFT)   /* 1010 GPIO interrupt rising edge */
#  define GPIO_INTBOTH      (11 << GPIO_FUNC_SHIFT)   /* 1011 GPIO interrupt both edges */
#  define GPIO_INTLOW       (13 << GPIO_FUNC_SHIFT)   /* 1101 GPIO interrupt low level */
#  define GPIO_INTHIGH      (14 << GPIO_FUNC_SHIFT)   /* 1110 GPIO interrupt high level */

/* Bit encoding */

#define GPIO_GPIO_MASK      (0xc << GPIO_FUNC_SHIFT)  /* 11xx */
#define GPIO_GPIO_CODE      (0x0 << GPIO_FUNC_SHIFT)  /* 00xx */
#define GPIO_ALT_MASK       (0xc << GPIO_FUNC_SHIFT)  /* 11xx */
#define GPIO_ALT_CODE       (0x4 << GPIO_FUNC_SHIFT)  /* 01xx */
#define GPIO_INTR_MASK      (0x8 << GPIO_FUNC_SHIFT)  /* 1xxx */
#define GPIO_INTR_CODE      (0x8 << GPIO_FUNC_SHIFT)  /* 1xxx */
#define GPIO_INTEDGE_MASK   (0xc << GPIO_FUNC_SHIFT)  /* 11xx */
#define GPIO_INTEDGE_CODE   (0x8 << GPIO_FUNC_SHIFT)  /* 10xx */
#define GPIO_INTLVL_MASK    (0xc << GPIO_FUNC_SHIFT)  /* 11xx */
#define GPIO_INTLVL_CODE    (0xc << GPIO_FUNC_SHIFT)  /* 11xx */

#define GPIO_IS_GPIO(ps)    (((uint16_t)(ps) & GPIO_GPIO_MASK) == GPIO_GPIO_CODE)
#define GPIO_IS_ALT(ps)     (((uint16_t)(ps) & GPIO_ALT_MASK) == GPIO_ALT_CODE)
#define GPIO_IS_INTR(ps)    (((uint16_t)(ps) & GPIO_ALT_MASK) == GPIO_INTR_CODE)
#define GPIO_IS_INTEDGE(ps) (((uint16_t)(ps) & GPIO_INTEDGE_MASK) == GPIO_INTEDGE_CODE)
#define GPIO_IS_INTLVL(ps)  (((uint16_t)(ps) & GPIO_INTLVL_MASK) == GPIO_INTLVL_CODE)

/* Wake-UP:
 *
 * .... W... .... ....
 */

#define GPIO_WAKEUP         (1 << 11)                 /* Bit 11: Wakeup Enable */

/* Pin Pull Up/Down: PP
 *
 * .... .RR. .... ....
 */

#define GPIO_MODE_SHIFT     (9)                       /* Bits 9-10: Pin pull up/down mode */
#define GPIO_MODE_MASK      (3 << GPIO_MODE_SHIFT)
#  define GPIO_FLOAT        (0 << GPIO_MODE_SHIFT)    /* Neither pull-up nor -down */
#  define GPIO_PULLDOWN     (1 << GPIO_MODE_SHIFT)    /* Pull-down resistor enabled */
#  define GPIO_PULLUP       (2 << GPIO_MODE_SHIFT)    /* Pull-up resistor enabled */

/* Initial value: V
 *
 * .... ...V .... ....
 */

#define GPIO_VALUE          (1 << 8)                  /* Bit 8: Initial GPIO output value */
#  define GPIO_VALUE_ONE    GPIO_VALUE
#  define GPIO_VALUE_ZERO   (0)

/* Input Hysteresis:
 *
 * .... .... H... ....
 */

#define GPIO_HYSTERESIS     (1 << 7)                  /* Bit 7: Input hysteresis */

/* Slew Rate:
 *
 * .... .... .S.. ....
 */

#define GPIO_SLEW           (1 << 6)                  /* Bit 7: Slew rate mode */

/* Drive Strength:
 *
 * .... .... ..DD ....
 */

#define GPIO_DRIVE_SHIFT    (4)                       /* Bits 4-5: Drive strength */
#define GPIO_DRIVE_MASK     (3 << GPIO_MODE_SHIFT)
#  define GPIO_DRIVE_LO     (0 << GPIO_MODE_SHIFT)    /* Low drive strength */
#  define GPIO_DRIVE_MEDLO  (1 << GPIO_MODE_SHIFT)    /* Low or medium-low drive */
#  define GPIO_DRIVE_MEDHI  (2 << GPIO_MODE_SHIFT)    /* High or midium-high driver */
#  define GPIO_DRIVE_HI     (3 << GPIO_MODE_SHIFT)    /* High drive strength */

/* Port number:  There is only one port, GPIO0
 *
 * .... .... .... ....
 */

#define GPIO_PORT0          (0)

/* Pin number: NNNN (0-13)
 *
 * .... .... .... NNNN
 */

#define GPIO_PIN_SHIFT      0                         /* Bits 0-3: GPIO number: 0-14 */
#define GPIO_PIN_MASK       (15 << GPIO_PIN_SHIFT)
#  define GPIO_PIN0         (0  << GPIO_PIN_SHIFT)
#  define GPIO_PIN1         (1  << GPIO_PIN_SHIFT)
#  define GPIO_PIN2         (2  << GPIO_PIN_SHIFT)
#  define GPIO_PIN3         (3  << GPIO_PIN_SHIFT)
#  define GPIO_PIN4         (4  << GPIO_PIN_SHIFT)
#  define GPIO_PIN5         (5  << GPIO_PIN_SHIFT)
#  define GPIO_PIN6         (6  << GPIO_PIN_SHIFT)
#  define GPIO_PIN7         (7  << GPIO_PIN_SHIFT)
#  define GPIO_PIN8         (8  << GPIO_PIN_SHIFT)
#  define GPIO_PIN9         (9  << GPIO_PIN_SHIFT)
#  define GPIO_PIN10        (10 << GPIO_PIN_SHIFT)
#  define GPIO_PIN11        (11 << GPIO_PIN_SHIFT)
#  define GPIO_PIN12        (12 << GPIO_PIN_SHIFT)
#  define GPIO_PIN13        (13 << GPIO_PIN_SHIFT)

#  define GPIO_PINMIN       0
#  define GPIO_PINMAX       13

/************************************************************************************
 * Public Types
 ************************************************************************************/

typedef uint16_t max326_pinset_t;

#endif /* __ARCH_ARM_SRC_MAX326XX_MAX32660_MAX32660_GPIO_H */
