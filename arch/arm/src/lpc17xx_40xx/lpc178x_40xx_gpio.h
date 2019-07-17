/************************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc178x_40xx_gpio.h
 *
 *   Copyright (C) 2010, 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC17XX_40XX_LPC178X_GPIO_H
#define __ARCH_ARM_SRC_LPC17XX_40XX_LPC178X_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Bit-encoded input to lpc17_40_configgpio() ******************************************/

/* Encoding: TTTT TTTT FFFF MMOV PPPN NNNN
 *
 *   Special Pin Functions: TTTT TTTT
 *   Pin Function:           FFFF
 *   Pin Mode bits:          MM
 *   Open drain:             O (output pins)
 *   Initial value:          V (output pins)
 *   Port number:            PPP (0-4)
 *   Pin number:             NNNNN (0-31)
 */

/* Special Pin Functions
 * For pins that have ADC/DAC, USB, I2C
 */

#define GPIO_INBUFF_SHIFT      (16)      /* Bit 16: HYSTERESIS: 0-Disable, 1-Enabled */
#define GPIO_INBUFF_MASK       (1 << GPIO_INBUFF_SHIFT)
#  define GPIO_HYSTERESIS      (1 << GPIO_INBUFF_SHIFT)

#define GPIO_INVERT            (1 << 17) /* Bit 17: Input: 0-Not Inverted, 1-Inverted */

#define GPIO_SLEW_SHIFT        (18)      /* Bit 18: Rate Control: 0-Standard mode, 1-Fast mode */
#define GPIO_SLEW_MASK         (1 << GPIO_SLEW_SHIFT)
#  define GPIO_SLEW_NORMAL     (0 << GPIO_SLEW_SHIFT)
#  define GPIO_SLEW_FAST       (1 << GPIO_SLEW_SHIFT)

#define GPIO_ADMODE_SHIFT      (19)      /* Bit 19: A/D Modes: 0-Analog, 1-Digital  */
#define GPIO_ADMODE_MASK       (1 << GPIO_ADMODE_SHIFT)
#  define GPIO_MODE_DIGITAL    (0 << GPIO_ADMODE_SHIFT)
#  define GPIO_MODE_ANALOG     (1 << GPIO_ADMODE_SHIFT)

#define GPIO_FILTER_SHIFT      (20)      /* Bit 20: Filter: 0-Off, 1-ON */
#define GPIO_FILTER_MASK       (1 << GPIO_FILTER_SHIFT)
#  define GPIO_FILTER_OFF      (0 << GPIO_FILTER_SHIFT)
#  define GPIO_FILTER_ON       (1 << GPIO_FILTER_SHIFT)

#define GPIO_DACEN             (1 << 21) /* Bit 21: DAC: 0-Disabled, 1-Enabled, P0:26 only */

#define GPIO_I2CMODE_SHIFT     (22)      /* Bits 22-23: I2C mode */
#define GPIO_I2CMODE_MASK      (3 << GPIO_I2CMODE_SHIFT)
#  define GPIO_I2CHS           (1 << 22) /* Bit 22: Filter and Rate Control: 0-Enabled, 1-Disabled */
#  define GPIO_HIDRIVE         (1 << 23) /* Bit 23: Current Sink: 0-4mA, 1-20mA  P5:2 and P5:3 only,*/

/* Pin Function bits: FFFF
 * Only meaningful when the GPIO function is GPIO_PIN
 */

#define GPIO_FUNC_SHIFT        (12)    /* Bits 12-15: GPIO mode */
#define GPIO_FUNC_MASK         (15 << GPIO_FUNC_SHIFT)
#  define GPIO_INPUT           (0 << GPIO_FUNC_SHIFT) /* 0000 GPIO input pin */
#  define GPIO_INTFE           (1 << GPIO_FUNC_SHIFT) /* 0001 GPIO interrupt falling edge */
#  define GPIO_INTRE           (2 << GPIO_FUNC_SHIFT) /* 0010 GPIO interrupt rising edge */
#  define GPIO_INTBOTH         (3 << GPIO_FUNC_SHIFT) /* 0011 GPIO interrupt both edges */
#  define GPIO_OUTPUT          (4 << GPIO_FUNC_SHIFT) /* 0100 GPIO outpout pin */
#  define GPIO_ALT1            (5 << GPIO_FUNC_SHIFT) /* 0101 Alternate function 1 */
#  define GPIO_ALT2            (6 << GPIO_FUNC_SHIFT) /* 0110 Alternate function 2 */
#  define GPIO_ALT3            (7 << GPIO_FUNC_SHIFT) /* 0111 Alternate function 3 */
#  define GPIO_ALT4            (8 << GPIO_FUNC_SHIFT) /* 1000 Alternate function 4 */
#  define GPIO_ALT5            (9 << GPIO_FUNC_SHIFT) /* 1001 Alternate function 5 */
#  define GPIO_ALT6            (10 << GPIO_FUNC_SHIFT) /* 1010 Alternate function 6 */
#  define GPIO_ALT7            (11 << GPIO_FUNC_SHIFT) /* 1011 Alternate function 7 */

#define GPIO_EDGE_SHIFT        (12)      /* Bits 12-13: Interrupt edge bits */
#define GPIO_EDGE_MASK         (3 << GPIO_EDGE_SHIFT)

#define GPIO_INOUT_MASK        GPIO_OUTPUT
#define GPIO_FE_MASK           GPIO_INTFE
#define GPIO_RE_MASK           GPIO_INTRE

#define GPIO_ISGPIO(ps)        ((uint16_t(ps) & GPIO_FUNC_MASK) <= GPIO_OUTPUT)
#define GPIO_ISALT(ps)         ((uint16_t(ps) & GPIO_FUNC_MASK) > GPIO_OUTPUT)
#define GPIO_ISINPUT(ps)       (((ps) & GPIO_FUNC_MASK) == GPIO_INPUT)
#define GPIO_ISOUTPUT(ps)      (((ps) & GPIO_FUNC_MASK) == GPIO_OUTPUT)
#define GPIO_ISINORINT(ps)     (((ps) & GPIO_INOUT_MASK) == 0)
#define GPIO_ISOUTORALT(ps)    (((ps) & GPIO_INOUT_MASK) != 0)
#define GPIO_ISINTERRUPT(ps)   (GPIO_ISOUTPUT(ps) && !GPIO_ISINPUT(ps))
#define GPIO_ISFE(ps)          (((ps) & GPIO_FE_MASK) != 0)
#define GPIO_ISRE(ps)          (((ps) & GPIO_RE_MASK) != 0)

/* Pin Mode: MM */

#define GPIO_PUMODE_SHIFT      (10)      /* Bits 10-11: Pin pull-up mode */
#define GPIO_PUMODE_MASK       (3 << GPIO_PUMODE_SHIFT)
#  define GPIO_FLOAT           (0 << GPIO_PUMODE_SHIFT) /* Neither pull-up nor -down */
#  define GPIO_PULLDN          (1 << GPIO_PUMODE_SHIFT) /* Pull-down resistor enabled */
#  define GPIO_PULLUP          (2 << GPIO_PUMODE_SHIFT) /* Pull-up resistor enabled */
#  define GPIO_REPEATER        (3 << GPIO_PUMODE_SHIFT) /* Repeater mode enabled */

/* Open drain: O */

#define GPIO_OPEN_DRAIN        (1 << 9)  /* Bit 9:  Open drain mode */

/* Initial value: V */

#define GPIO_VALUE             (1 << 8)  /* Bit 8: Initial GPIO output value */
#  define GPIO_VALUE_ONE       GPIO_VALUE
#  define GPIO_VALUE_ZERO      (0)

/* Port number: PPP (0-5) */

#define GPIO_PORT_SHIFT        (5)       /* Bit 5-7:  Port number */
#define GPIO_PORT_MASK         (7 << GPIO_PORT_SHIFT)
#  define GPIO_PORT0           (0 << GPIO_PORT_SHIFT)
#  define GPIO_PORT1           (1 << GPIO_PORT_SHIFT)
#  define GPIO_PORT2           (2 << GPIO_PORT_SHIFT)
#  define GPIO_PORT3           (3 << GPIO_PORT_SHIFT)
#  define GPIO_PORT4           (4 << GPIO_PORT_SHIFT)
#  define GPIO_PORT5           (5 << GPIO_PORT_SHIFT)

#define GPIO_NPORTS            6

/* Pin number: NNNNN (0-31) */

#define GPIO_PIN_SHIFT         0         /* Bits 0-4: GPIO number: 0-31 */
#define GPIO_PIN_MASK          (31 << GPIO_PIN_SHIFT)
#  define GPIO_PIN0            (0  << GPIO_PIN_SHIFT)
#  define GPIO_PIN1            (1  << GPIO_PIN_SHIFT)
#  define GPIO_PIN2            (2  << GPIO_PIN_SHIFT)
#  define GPIO_PIN3            (3  << GPIO_PIN_SHIFT)
#  define GPIO_PIN4            (4  << GPIO_PIN_SHIFT)
#  define GPIO_PIN5            (5  << GPIO_PIN_SHIFT)
#  define GPIO_PIN6            (6  << GPIO_PIN_SHIFT)
#  define GPIO_PIN7            (7  << GPIO_PIN_SHIFT)
#  define GPIO_PIN8            (8  << GPIO_PIN_SHIFT)
#  define GPIO_PIN9            (9  << GPIO_PIN_SHIFT)
#  define GPIO_PIN10           (10 << GPIO_PIN_SHIFT)
#  define GPIO_PIN11           (11 << GPIO_PIN_SHIFT)
#  define GPIO_PIN12           (12 << GPIO_PIN_SHIFT)
#  define GPIO_PIN13           (13 << GPIO_PIN_SHIFT)
#  define GPIO_PIN14           (14 << GPIO_PIN_SHIFT)
#  define GPIO_PIN15           (15 << GPIO_PIN_SHIFT)
#  define GPIO_PIN16           (16 << GPIO_PIN_SHIFT)
#  define GPIO_PIN17           (17 << GPIO_PIN_SHIFT)
#  define GPIO_PIN18           (18 << GPIO_PIN_SHIFT)
#  define GPIO_PIN19           (19 << GPIO_PIN_SHIFT)
#  define GPIO_PIN20           (20 << GPIO_PIN_SHIFT)
#  define GPIO_PIN21           (21 << GPIO_PIN_SHIFT)
#  define GPIO_PIN22           (22 << GPIO_PIN_SHIFT)
#  define GPIO_PIN23           (23 << GPIO_PIN_SHIFT)
#  define GPIO_PIN24           (24 << GPIO_PIN_SHIFT)
#  define GPIO_PIN25           (25 << GPIO_PIN_SHIFT)
#  define GPIO_PIN26           (26 << GPIO_PIN_SHIFT)
#  define GPIO_PIN27           (27 << GPIO_PIN_SHIFT)
#  define GPIO_PIN28           (28 << GPIO_PIN_SHIFT)
#  define GPIO_PIN29           (29 << GPIO_PIN_SHIFT)
#  define GPIO_PIN30           (30 << GPIO_PIN_SHIFT)
#  define GPIO_PIN31           (31 << GPIO_PIN_SHIFT)

/************************************************************************************
 * Public Types
 ************************************************************************************/

typedef uint32_t lpc17_40_pinset_t;

/************************************************************************************
 * Public Data
 ************************************************************************************/

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
 * Public Functions
 ****************************************************************************/

#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_LPC17XX_40XX_LPC178X_GPIO_H */
