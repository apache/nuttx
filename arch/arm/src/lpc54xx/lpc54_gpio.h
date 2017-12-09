/************************************************************************************
 * arch/arm/src/lpc54xx/lpc54_gpio.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC54XX_LPC54_GPIO_H
#define __ARCH_ARM_SRC_LPC54XX_LPC54_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

#include <arch/lpc54xx/chip.h>

#include "chip/lpc54_gpio.h"
#include "chip/lpc54_iocon.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Bit-encoded input to lpc54_gpio_config() ******************************************/

/* 32-Bit Encoding: .... .... TTTT TTTT  FFFF ...V PPPN NNNN
 *
 *   Special Pin Functions: TTTT TTTT
 *   Pin Function:          FFFF
 *   Pin Mode bits:         MM
 *   Open drain:            O (output pins)
 *   Initial value:         V (output pins)
 *   Port number:           PPP (0-5)
 *   Pin number:            NNNNN (0-31)
 */

/* Special Pin Functions:
 * For pins that have ADC/DAC, USB, I2C
 *
 * .... .... TTTT TTTT  .... .... .... ....
 */

#define GPIO_I2CSLEW_SHIFT       (16)      /* Bit 16:  Controls slew rate of I2C pad */
#define GPIO_I2CSLEW_MASK        (1 << GPIO_I2CSLEW_SHIFT)
#  define GPIO_I2CSLEW_I2C       (0)
#  define GPIO_I2CSLEW_GPIO      GPIO_I2CSLEW_MASK

#define GPIO_INVERT_SHIFT        (17)      /* Bit 17: Input polarity */
#define GPIO_INVERT_MASK         (1 << GPIO_INVERT_SHIFT)
#  define GPIO_INVERT            GPIO_INVERT_MASK

#define GPIO_DIGIMODE_SHIFT      (18)      /* Bit 18: Select Analog/Digital mode */
#define GPIO_DIGIMODE_MASK       (1 << GPIO_DIGIMODE_SHIFT)
#  define GPIO_MODE_ANALOG       (0)
#  define GPIO_MODE_DIGITAL      GPIO_DIGIMODE_MASK

#define GPIO_FILTEROFF_SHIFT     (19)      /* Bit 19: Controls input glitch filter */
#define GPIO_FILTEROFF_MASK      (1 << GPIO_FILTEROFF_SHIFT)
#  define GPIO_FILTER_ON         (0)
#  define GPIO_FILTER_OFF        GPIO_FILTEROFF_MASK

#define GPIO_SLEW_SHIFT          (20)      /* Bit 20: Driver slew rate */
#define GPIO_SLEW_MASK           (1 << GPIO_SLEW_SHIFT)
#  define GPIO_SLEW_STANDARD     (0)
#  define GPIO_SLEW_FAST         GPIO_SLEW_MASK

#define GPIO_I2CDRIVE_SHIFT      (21)      /* Bit 21: Driver slew rate */
#define GPIO_I2CDRIVE_MASK       (1 << GPIO_I2CDRIVE_SHIFT)
#  define GPIO_I2CDRIVE_LOW      (0)
#  define GPIO_I2CDRIVE_HIGH     GPIO_I2CDRIVE_MASK

#define GPIO_OD_SHIFT            (22)      /* Bit 22: Controls open-drain mode */
#define GPIO_OD_MASK             (1 << GPIO_OD_SHIFT)
#  define GPIO_PUSHPULL          (0)
#  define GPIO_OPENDRAIN         GPIO_OD_MASK

#define GPIO_I2CFILTEROFF_SHIFT  (23)      /* Bit 23: Configures I2C glitch filter */
#define GPIO_I2CFILTEROFF_MASK   (1 << GPIO_I2CFILTEROFF_SHIFT)
#  define GPIO_I2C_FILTER_ON     (0)
#  define GPIO_I2C_FILTER_OFF    GPIO_I2CFILTEROFF_MASK

/* Pin Function bits:
 * Only meaningful when the GPIO function is GPIO_PIN
 *
 * .... .... .... ....  FFFF .... .... ....
 */

#define GPIO_FUNC_SHIFT         (12)    /* Bits 12-15: GPIO mode */
#define GPIO_FUNC_MASK          (15 << GPIO_FUNC_SHIFT)
#  define GPIO_INPUT            (0 << GPIO_FUNC_SHIFT)  /* 0000 GPIO input pin */
#  define GPIO_INTFE            (1 << GPIO_FUNC_SHIFT)  /* 0001 GPIO interrupt falling edge */
#  define GPIO_INTRE            (2 << GPIO_FUNC_SHIFT)  /* 0010 GPIO interrupt rising edge */
#  define GPIO_INTBOTH          (3 << GPIO_FUNC_SHIFT)  /* 0011 GPIO interrupt both edges */
#  define GPIO_OUTPUT           (4 << GPIO_FUNC_SHIFT)  /* 0100 GPIO outpout pin */
#  define GPIO_ALT1             (5 << GPIO_FUNC_SHIFT)  /* 0101 Alternate function 1 */
#  define GPIO_ALT2             (6 << GPIO_FUNC_SHIFT)  /* 0110 Alternate function 2 */
#  define GPIO_ALT3             (7 << GPIO_FUNC_SHIFT)  /* 0111 Alternate function 3 */
#  define GPIO_ALT4             (8 << GPIO_FUNC_SHIFT)  /* 1000 Alternate function 4 */
#  define GPIO_ALT5             (9 << GPIO_FUNC_SHIFT)  /* 1001 Alternate function 5 */
#  define GPIO_ALT6             (10 << GPIO_FUNC_SHIFT) /* 1010 Alternate function 6 */
#  define GPIO_ALT7             (11 << GPIO_FUNC_SHIFT) /* 1011 Alternate function 7 */

#define GPIO_EDGE_SHIFT         (12)      /* Bits 12-13: Interrupt edge bits */
#define GPIO_EDGE_MASK          (3 << GPIO_EDGE_SHIFT)

#define GPIO_INOUT_MASK         GPIO_OUTPUT
#define GPIO_FE_MASK            GPIO_INTFE
#define GPIO_RE_MASK            GPIO_INTRE

#define GPIO_ISGPIO(ps)         ((uint16_t(ps) & GPIO_FUNC_MASK) <= GPIO_OUTPUT)
#define GPIO_ISALT(ps)          ((uint16_t(ps) & GPIO_FUNC_MASK) > GPIO_OUTPUT)
#define GPIO_ISINPUT(ps)        (((ps) & GPIO_FUNC_MASK) == GPIO_INPUT)
#define GPIO_ISOUTPUT(ps)       (((ps) & GPIO_FUNC_MASK) == GPIO_OUTPUT)
#define GPIO_ISINORINT(ps)      (((ps) & GPIO_INOUT_MASK) == 0)
#define GPIO_ISOUTORALT(ps)     (((ps) & GPIO_INOUT_MASK) != 0)
#define GPIO_ISINTERRUPT(ps)    (GPIO_ISOUTPUT(ps) && !GPIO_ISINPUT(ps))
#define GPIO_ISFE(ps)           (((ps) & GPIO_FE_MASK) != 0)
#define GPIO_ISRE(ps)           (((ps) & GPIO_RE_MASK) != 0)

/* Pin Mode: MM
 *
 * .... .... .... ....  .... MM.. .... ....
 */

#define GPIO_MODE_SHIFT         (10)      /* Bits 10-11: Pin pull-up mode */
#define GPIO_MODE_MASK          (3 << GPIO_MODE_SHIFT)
#  define GPIO_FLOAT            (IOCON_MODE_FLOAT    << GPIO_MODE_SHIFT) /* Neither pull-up nor -down */
#  define GPIO_PULLDOWN         (IOCON_MODE_PULLDOWN << GPIO_MODE_SHIFT) /* Pull-down resistor enabled */
#  define GPIO_PULLUP           (IOCON_MODE_PULLUP   << GPIO_MODE_SHIFT) /* Pull-up resistor enabled */
#  define GPIO_REPEATER         (IOCON_MODE_REPEATER << GPIO_MODE_SHIFT) /* Repeater mode enabled */

/* Initial value: V
 */

#define GPIO_VALUE              (1 << 8)  /* Bit 8: Initial GPIO output value */
#  define GPIO_VALUE_ONE        GPIO_VALUE
#  define GPIO_VALUE_ZERO       (0)

/* Port number: PPP (0-5) */

#define GPIO_PORT_SHIFT         (5)       /* Bit 5-7:  Port number */
#define GPIO_PORT_MASK          (7 << GPIO_PORT_SHIFT)
#  define GPIO_PORT0            (0 << GPIO_PORT_SHIFT)
#  define GPIO_PORT1            (1 << GPIO_PORT_SHIFT)
#  define GPIO_PORT2            (2 << GPIO_PORT_SHIFT)
#  define GPIO_PORT3            (3 << GPIO_PORT_SHIFT)
#  define GPIO_PORT4            (4 << GPIO_PORT_SHIFT)
#  define GPIO_PORT5            (5 << GPIO_PORT_SHIFT)

/* Pin number: NNNNN (0-31) */

#define GPIO_PIN_SHIFT          0         /* Bits 0-4: GPIO number: 0-31 */
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

/************************************************************************************
 * Public Types
 ************************************************************************************/

typedef uint32_t lpc54_pinset_t;

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

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: lpc54_gpio_irqinitialize
 *
 * Description:
 *   Initialize logic to support interrupting GPIO pins.  This function is called by
 *   the OS inialization logic and is not a user interface.
 *
 ************************************************************************************/

#ifdef CONFIG_LPC54_GPIOIRQ
void lpc54_gpio_irqinitialize(void);
#else
#  define lpc54_gpio_irqinitialize()
#endif

/************************************************************************************
 * Name: lpc54_gpio_config
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ************************************************************************************/

int lpc54_gpio_config(lpc54_pinset_t cfgset);

/************************************************************************************
 * Name: lpc54_gpio_interrupt
 *
 * Description:
 *   Configure a GPIO interrupt pin based on bit-encoded description of the pin.
 *   This function is called by lpc54_gpio_config to setup interrupting pins.  It is
 *   not a user interface.
 *
 ************************************************************************************/

#ifdef CONFIG_LPC54_GPIOIRQ
void lpc54_gpio_interrupt(lpc54_pinset_t pinset);
#endif

/************************************************************************************
 * Name: lpc54_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ************************************************************************************/

void lpc54_gpio_write(lpc54_pinset_t pinset, bool value);

/************************************************************************************
 * Name: lpc54_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ************************************************************************************/

bool lpc54_gpio_read(lpc54_pinset_t pinset);

/************************************************************************************
 * Function:  lpc54_gpio_dump
 *
 * Description:
 *   Dump all GPIO registers associated with the base address of the provided pinset.
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
int lpc54_gpio_dump(lpc54_pinset_t pinset, const char *msg);
#else
#  define lpc54_gpio_dump(p,m)
#endif

#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_LPC54XX_LPC54_GPIO_H */
