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

#include "hardware/lpc54_gpio.h"
#include "hardware/lpc54_iocon.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Bit-encoded input to lpc54_gpio_config() ******************************************/

/* 32-Bit Encoding: .... .... TTTT TTTT  FFFF FMMV PPPN NNNN
 *
 *   Special Pin Functions: TTTT TTTT
 *   Pin Function:          FFFF F
 *   Pin Mode bits:         MM
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
 * .... .... .... ....  FFFF F... .... ....
 */

#define GPIO_FUNC_SHIFT         (11)    /* Bits 11-15: GPIO mode */
#define GPIO_FUNC_MASK          (0x1f << GPIO_FUNC_SHIFT)
#  define GPIO_INPUT            (0x00 << GPIO_FUNC_SHIFT)  /* 00000 GPIO input pin */
#  define GPIO_OUTPUT           (0x01 << GPIO_FUNC_SHIFT)  /* 00001 GPIO output pin */

#  define GPIO_INTFE            (0x09 << GPIO_FUNC_SHIFT)  /* 01001 GPIO interrupt falling edge */
#  define GPIO_INTRE            (0x0a << GPIO_FUNC_SHIFT)  /* 01010 GPIO interrupt rising edge */
#  define GPIO_INTBOTH          (0x0b << GPIO_FUNC_SHIFT)  /* 01011 GPIO interrupt both edges */
#  define GPIO_INTLOW           (0x0d << GPIO_FUNC_SHIFT)  /* 01101 GPIO interrupt low level */
#  define GPIO_INTHIGH          (0x0e << GPIO_FUNC_SHIFT)  /* 01110 GPIO interrupt high level */

#  define GPIO_ALT1             (0x11 << GPIO_FUNC_SHIFT)  /* 10001 Alternate function 1 */
#  define GPIO_ALT2             (0x12 << GPIO_FUNC_SHIFT)  /* 10010 Alternate function 2 */
#  define GPIO_ALT3             (0x13 << GPIO_FUNC_SHIFT)  /* 10011 Alternate function 3 */
#  define GPIO_ALT4             (0x14 << GPIO_FUNC_SHIFT)  /* 10100 Alternate function 4 */
#  define GPIO_ALT5             (0x15 << GPIO_FUNC_SHIFT)  /* 10101 Alternate function 5 */
#  define GPIO_ALT6             (0x16 << GPIO_FUNC_SHIFT)  /* 10110 Alternate function 6 */
#  define GPIO_ALT7             (0x17 << GPIO_FUNC_SHIFT)  /* 10111 Alternate function 7 */

#define GPIO_GPIO_MASK          (0x1e << GPIO_FUNC_SHIFT)  /* 1111x */
#define GPIO_GPIO_CODE          (0x00 << GPIO_FUNC_SHIFT)  /* 0000x */
#define GPIO_IS_GPIO(ps)        (((uint32_t)(ps) & GPIO_GPIO_MASK) == GPIO_GPIO_CODE)
#define GPIO_IS_GPIOINPUT(ps)   ((uint32_t)(ps) == GPIO_INPUT)
#define GPIO_IS_GPIOOUTPUT(ps)  ((uint32_t)(ps) == GPIO_OUTPUT)

#define GPIO_INTR_MASK          (0x18 << GPIO_FUNC_SHIFT)  /* 11xxx */
#define GPIO_INTR_CODE          (0x08 << GPIO_FUNC_SHIFT)  /* 01xxx */
#define GPIO_IS_INTR(ps)        (((uint32_t)(ps) & GPIO_INTR_MASK) == GPIO_INTR_CODE)

#define GPIO_TRIG_MASK          (0x18 << GPIO_FUNC_SHIFT)  /* 111xx */
#define GPIO_TRIG_EDGE_CODE     (0x08 << GPIO_FUNC_SHIFT)  /* 010xx */
#define GPIO_TRIG_LEVEL_CODE    (0x0c << GPIO_FUNC_SHIFT)  /* 011xx */
#define GPIO_IS_INTEDGE(ps)     (((uint32_t)(ps) & GPIO_TRIG_MASK) == GPIO_TRIG_EDGE_CODE)
#define GPIO_IS_INTLEVEL(ps)    (((uint32_t)(ps) & GPIO_TRIG_MASK) == GPIO_TRIG_LEVEL_CODE)

#define GPIO_ALT_MASK           (0x18 << GPIO_FUNC_SHIFT)  /* 11xxx */
#define GPIO_ALT_CODE           (0x10 << GPIO_FUNC_SHIFT)  /* 10xxx */
#define GPIO_IS_ALT(ps)         (((uint32_t)(ps) & GPIO_ALT_MASK) == GPIO_ALT_CODE)

/* Pin Mode: MM
 *
 * .... .... .... ....  .... .MM. .... ....
 */

#define GPIO_MODE_SHIFT         (9)      /* Bits 9-10: Pin pull-up mode */
#define GPIO_MODE_MASK          (3 << GPIO_MODE_SHIFT)
#  define GPIO_FLOAT            (IOCON_MODE_FLOAT    << GPIO_MODE_SHIFT) /* Neither pull-up nor -down */
#  define GPIO_PULLDOWN         (IOCON_MODE_PULLDOWN << GPIO_MODE_SHIFT) /* Pull-down resistor enabled */
#  define GPIO_PULLUP           (IOCON_MODE_PULLUP   << GPIO_MODE_SHIFT) /* Pull-up resistor enabled */
#  define GPIO_REPEATER         (IOCON_MODE_REPEATER << GPIO_MODE_SHIFT) /* Repeater mode enabled */

/* Initial value: V
 *
 * .... .... .... ....  .... ...V .... ....
 */

#define GPIO_VALUE              (1 << 8)  /* Bit 8: Initial GPIO output value */
#  define GPIO_VALUE_ONE        GPIO_VALUE
#  define GPIO_VALUE_ZERO       (0)

/* Port number: PPP (0-5)
 *
 * .... .... .... ....  .... .... PPP. ....
 */

#define GPIO_PORT_SHIFT         (5)       /* Bit 5-7:  Port number */
#define GPIO_PORT_MASK          (7 << GPIO_PORT_SHIFT)
#  define GPIO_PORT0            (0 << GPIO_PORT_SHIFT)
#  define GPIO_PORT1            (1 << GPIO_PORT_SHIFT)
#  define GPIO_PORT2            (2 << GPIO_PORT_SHIFT)
#  define GPIO_PORT3            (3 << GPIO_PORT_SHIFT)
#  define GPIO_PORT4            (4 << GPIO_PORT_SHIFT)
#  define GPIO_PORT5            (5 << GPIO_PORT_SHIFT)

/* Pin number: NNNNN (0-31)
 *
 * .... .... .... ....  .... .... ...N NNNN
 */

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
int lpc54_gpio_interrupt(lpc54_pinset_t pinset);
#endif

/************************************************************************************
 * Name: lpc54_gpio_irqno
 *
 * Description:
 *   Returns the IRQ number that was associated with an interrupt pin after it was
 *   configured.
 *
 ************************************************************************************/

#ifdef CONFIG_LPC54_GPIOIRQ
int lpc54_gpio_irqno(lpc54_pinset_t pinset);
#endif

/************************************************************************************
 * Name: lpc54_gpio_ackedge
 *
 * Description:
 *   Acknowledge edge interrupts by clearing the associated bits in the rising and
 *   falling registers.  This acknowledgemment is, of course, not needed for level
 *   interrupts.
 *
 ************************************************************************************/

#ifdef CONFIG_LPC54_GPIOIRQ
int lpc54_gpio_ackedge(int irq);
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
