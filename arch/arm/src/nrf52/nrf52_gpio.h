/************************************************************************************
 * arch/arm/src/nrf52/nrf52_gpio.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author:  Janne Rosberg <janne@offcode.fi>
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

#ifndef __ARCH_ARM_SRC_NRF52_NRF52_GPIO_H
#define __ARCH_ARM_SRC_NRF52_NRF52_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

#include <arch/nrf52/chip.h>
#include "chip/nrf52_gpio.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Bit-encoded input to nrf52_gpio_config() *****************************************/

/* 32-Bit Encoding: .... .... .... ....  FFSS DDDM MVPN NNNN
 *
 *   Pin Function:          FF
 *   Pin Sense:             SS
 *   Pin Drive:             DDD
 *   Pin Mode bits:         MM
 *   Initial value:         V (output pins)
 *   Port number:           P (0-1)
 *   Pin number:            NNNNN (0-31)
 */

/* Pin Function bits:
 * Only meaningful when the GPIO function is GPIO_PIN
 *
 * .... .... .... ....  FF.. .... .... ....
 */

#define GPIO_FUNC_SHIFT         (14)    /* Bits 14-15: GPIO mode */
#define GPIO_FUNC_MASK          (0x03 << GPIO_FUNC_SHIFT)
#  define GPIO_INPUT            (0x00 << GPIO_FUNC_SHIFT)  /* 00000 GPIO input pin */
#  define GPIO_OUTPUT           (0x01 << GPIO_FUNC_SHIFT)  /* 00001 GPIO output pin */

/* Pin Sense bits:
 *
 * .... .... .... ....  ..SS .... .... ....
 */

#define GPIO_SENSE_SHIFT        (12)     /* Bits 12-13: Pin Sense mode */
#define GPIO_SENSE_MASK         (0x3 << GPIO_SENSE_SHIFT)
#  define GPIO_SENSE_NONE       (0 << GPIO_SENSE_SHIFT)
#  define GPIO_SENSE_HIGH       (2 << GPIO_SENSE_SHIFT)
#  define GPIO_SENSE_LOW        (3 << GPIO_SENSE_SHIFT)

/* Pin Drive bits:
 *
 * .... .... .... ....  .... DDD. .... ....
 */

#define GPIO_DRIVE_SHIFT        (9)      /* Bits 9-11: Pin pull-up mode */
#define GPIO_DRIVE_MASK         (0x3 << GPIO_DRIVE_SHIFT)
#  define GPIO_DRIVE_S0S1       (0 << GPIO_DRIVE_SHIFT) /* Standard '0', standard '1' */
#  define GPIO_DRIVE_H0S1       (1 << GPIO_DRIVE_SHIFT) /* High drive '0', standard '1' */
#  define GPIO_DRIVE_S0H1       (2 << GPIO_DRIVE_SHIFT) /* */
#  define GPIO_DRIVE_H0H1       (3 << GPIO_DRIVE_SHIFT) /* */
#  define GPIO_DRIVE_D0S1       (4 << GPIO_DRIVE_SHIFT) /* */
#  define GPIO_DRIVE_D0H1       (5 << GPIO_DRIVE_SHIFT) /* */
#  define GPIO_DRIVE_S0D1       (6 << GPIO_DRIVE_SHIFT) /* */
#  define GPIO_DRIVE_H0D1       (7 << GPIO_DRIVE_SHIFT) /* */

/* Pin Mode: MM
 *
 * .... .... .... ....  .... ...M M... ....
 */

#define GPIO_MODE_SHIFT         (7)      /* Bits 7-8: Pin pull-up mode */
#define GPIO_MODE_MASK          (0x3 << GPIO_MODE_SHIFT)
#  define GPIO_FLOAT            (0 << GPIO_MODE_SHIFT) /* Neither pull-up nor -down */
#  define GPIO_PULLDOWN         (1 << GPIO_MODE_SHIFT) /* Pull-down resistor enabled */
#  define GPIO_PULLUP           (2 << GPIO_MODE_SHIFT) /* Pull-up resistor enabled */

/* Initial value: V
 *
 * .... .... .... ....  .... .... .V.. ....
 */

#define GPIO_VALUE              (1 << 6)  /* Bit 6: Initial GPIO output value */
#  define GPIO_VALUE_ONE        GPIO_VALUE
#  define GPIO_VALUE_ZERO       (0)

/* Port number: PPP (0-5)
 *
 * .... .... .... ....  .... .... ..P. ....
 */

#define GPIO_PORT_SHIFT         (5)       /* Bit 5:  Port number */
#define GPIO_PORT_MASK          (0x1 << GPIO_PORT_SHIFT)
#  define GPIO_PORT0            (0 << GPIO_PORT_SHIFT)
#  define GPIO_PORT1            (1 << GPIO_PORT_SHIFT)

/* Pin number: NNNNN (0-31)
 *
 * .... .... .... ....  .... .... ...N NNNN
 */

#define GPIO_PIN_SHIFT          0         /* Bits 0-4: GPIO number: 0-31 */
#define GPIO_PIN_MASK           (0x1f << GPIO_PIN_SHIFT)
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

typedef uint32_t nrf52_pinset_t;

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
 * Name: nrf52_gpio_irqinitialize
 *
 * Description:
 *   Initialize logic to support interrupting GPIO pins.  This function is called by
 *   the OS inialization logic and is not a user interface.
 *
 ************************************************************************************/

#ifdef CONFIG_NRF52_GPIOIRQ
void nrf52_gpio_irqinitialize(void);
#else
#  define nrf52_gpio_irqinitialize()
#endif

/************************************************************************************
 * Name: nrf52_gpio_config
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ************************************************************************************/

int nrf52_gpio_config(nrf52_pinset_t cfgset);

/************************************************************************************
 * Name: nrf52_gpio_interrupt
 *
 * Description:
 *   Configure a GPIO interrupt pin based on bit-encoded description of the pin.
 *   This function is called by nrf52_gpio_config to setup interrupting pins.  It is
 *   not a user interface.
 *
 ************************************************************************************/

#ifdef CONFIG_NRF52_GPIOIRQ
int nrf52_gpio_interrupt(nrf52_pinset_t pinset);
#endif

/************************************************************************************
 * Name: nrf52_gpio_irqno
 *
 * Description:
 *   Returns the IRQ number that was associated with an interrupt pin after it was
 *   configured.
 *
 ************************************************************************************/

#ifdef CONFIG_NRF52_GPIOIRQ
int nrf52_gpio_irqno(nrf52_pinset_t pinset);
#endif

/************************************************************************************
 * Name: nrf52_gpio_ackedge
 *
 * Description:
 *   Acknowledge edge interrupts by clearing the associated bits in the rising and
 *   falling registers.  This acknowledgemment is, of course, not needed for level
 *   interupts.
 *
 ************************************************************************************/

#ifdef CONFIG_NRF52_GPIOIRQ
int nrf52_gpio_ackedge(int irq);
#endif

/************************************************************************************
 * Name: rnf52_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ************************************************************************************/

void nrf52_gpio_write(nrf52_pinset_t pinset, bool value);

/************************************************************************************
 * Name: nrf52_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ************************************************************************************/

bool nrf52_gpio_read(nrf52_pinset_t pinset);

/************************************************************************************
 * Function:  nf52_gpio_dump
 *
 * Description:
 *   Dump all GPIO registers associated with the base address of the provided pinset.
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
int nrf52_gpio_dump(nrf52_pinset_t pinset, const char *msg);
#else
#  define nrf52_gpio_dump(p,m)
#endif

#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_NRF52_NRF52_GPIO_H */
