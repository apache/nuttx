/************************************************************************************
 * arch/arm/src/imx6/imx_gpio.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_IMX6_IMX_GPIO_H
#define __ARCH_ARM_SRC_IMX6_IMX_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <chip/imx_gpio.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Encoding:
 *
 *   ENCODING    IRRR ODDD SSLT GGGP PPPP
 *   GPIO INPUT  0... .... ...T GGGP PPPP
 *   GPIO OUTPUT 1RRR ODDD SSL. GGGP PPPP
 */

/* Input/Output Selection:
 *
 *   ENCODING    I... .... .... .... ....
 */

#define GPIO_INPUT             (0)        /* Bit 19: 0=input */
#define GPIO_INPUT             (1 << 19)  /* Bit 19: 1=input */

/* Output Pull Up/Down:
 *
 *   GPIO OUTPUT .RRR .... .... .... ....
 */

#define GPIO_PULL_SHIFT        (16)       /* Bits 16-18: Pull up/down selection */
#define GPIO_PULL_MASK         (7 << GPIO_PULL_SHIFT)
#  define GPIO_PULL_NONE       (0 << GPIO_PULL_SHIFT) /* Pull/keeper disabled */
#  define GPIO_PULL_KEEP       (1 << GPIO_PULL_SHIFT) /* Output determined by keeper */
#  define GPIO_PULL_UP_22K     (2 << GPIO_PULL_SHIFT) /* Pull up with 22 KOhm resister */
#  define GPIO_PULL_UP_47K     (3 << GPIO_PULL_SHIFT) /* Pull up with 47 KOhm resister */
#  define GPIO_PULL_UP_100K    (4 << GPIO_PULL_SHIFT) /* Pull up with 100 KOhm resister */
#  define GPIO_PULL_DOWN_100K  (5 << GPIO_PULL_SHIFT) /* Pull down with 100 KOhm resister */

/* Open Drain Output:
 *
 *   GPIO OUTPUT .... O... .... .... ....
 */

#define GPIO_CMOS_OUTPUT       (1 << 15)  /* Bit 15: 0=CMOS output */
#define GPIO_OPENDRAIN         (1 << 15)  /* Bit 15: 1=Enable open-drain output */

/* Output Drive Strength:
 *
 *   GPIO OUTPUT .... .DDD .... .... ....
 */

#define GPIO_DRIVE_SHIFT       (12)       /* Bits 12-14: Output Drive Strength */
#define GPIO_DRIVE_MASK        (7 << GPIO_DRIVE_SHIFT)
#  define GPIO_DRIVE_HIZ      (0 << GPIO_DRIVE_SHIFT) /* HI-Z */
#  define GPIO_DRIVE_260OHM   (1 << GPIO_DRIVE_SHIFT) /* 150 Ohm @3.3V, 260 Ohm @1.8V */
#  define GPIO_DRIVE_130OHM   (2 << GPIO_DRIVE_SHIFT) /* 75 Ohm @3.3V, 130 Ohm @1.8V */
#  define GPIO_DRIVE_90OHM    (3 << GPIO_DRIVE_SHIFT) /* 50 Ohm @3.3V, 90 Ohm @1.8V */
#  define GPIO_DRIVE_60OHM    (4 << GPIO_DRIVE_SHIFT) /* 37 Ohm @3.3V, 60 Ohm @1.8V */
#  define GPIO_DRIVE_50OHM    (5 << GPIO_DRIVE_SHIFT) /* 30 Ohm @3.3V, 50 Ohm @1.8V */
#  define GPIO_DRIVE_40OHM    (6 << GPIO_DRIVE_SHIFT) /* 25 Ohm @3.3V, 40 Ohm @1.8V */
#  define GPIO_DRIVE_33OHM    (7 << GPIO_DRIVE_SHIFT) /* 20 Ohm @3.3V, 33 Ohm @1.8V */

/* Output Speed:
 *
 *   GPIO OUTPUT .... .... SS.. .... ....
 */

#define GPIO_SPEED_SHIFT       (10)       /* Bits 10-11: Speed */
#define GPIO_SPEED_MASK        (3 << GPIO_SPEED_SHIFT)
#  define GPIO_SPEED_LOW       (0 << GPIO_SPEED_SHIFT) /* Low frequency (50 MHz) */
#  define GPIO_SPEED_MEDIUM    (1 << GPIO_SPEED_SHIFT) /* Medium frequency (100, 150 MHz) */
#  define GPIO_SPEED_MAX       (3 << GPIO_SPEED_SHIFT) /* Maximum frequency (100, 150, 200 MHz) */

/* Output Slew Rate:
 *
 *   GPIO OUTPUT .... .... ..L. .... ....
 */

#define GPIO_SLEW_SLOW         (0)        /* Bit 9: 0=Slow Slew Rate */
#define GPIO_SLEW_FAST         (1 << 9)   /* Bit 9: 1=Fast Slew Rate */
 
/* Input Schmitt Trigger:
 *
 *   GPIO INPUT  0... .... ...T .... ....
 */

#define GPIO_CMOS_INPUT        (0)        /* Bit 8: 0=CMOS input */
#define GPIO_SCHMITT_TRIGGER   (1 << 8)   /* Bit 8: 1=Enable Schmitt trigger if input */

/* GPIO Port Number
 *
 *   ENCODING    .... .... .... GGG. ....
 */

#define GPIO_PORT_SHIFT        (5)       /* Bits 5-7: Speed */
#define GPIO_PORT_MASK         (7 << GPIO_PORT_SHIFT)
# define GPIO_PORT1            (0 << GPIO_PORT_SHIFT) /* GPIO1 */
# define GPIO_PORT2            (1 << GPIO_PORT_SHIFT) /* GPIO2 */
# define GPIO_PORT3            (2 << GPIO_PORT_SHIFT) /* GPIO3 */
# define GPIO_PORT4            (3 << GPIO_PORT_SHIFT) /* GPIO4 */
# define GPIO_PORT5            (4 << GPIO_PORT_SHIFT) /* GPIO5 */
# define GPIO_PORT6            (5 << GPIO_PORT_SHIFT) /* GPIO6 */
# define GPIO_PORT7            (6 << GPIO_PORT_SHIFT) /* GPIO7 */

/* GPIO Pin Number:
 *
 *   ENCODING    .... .... .... ...P PPPP
 */

#define GPIO_PIN_SHIFT         (5)       /* Bits 0-4: Speed */
#define GPIO_PIN_MASK          (15 << GPIO_PIN_SHIFT)
# define GPIO_PIN0             (0 << GPIO_PIN_SHIFT)  /* Pin  0 */
# define GPIO_PIN1             (1 << GPIO_PIN_SHIFT)  /* Pin  1 */
# define GPIO_PIN2             (2 << GPIO_PIN_SHIFT)  /* Pin  2 */
# define GPIO_PIN3             (3 << GPIO_PIN_SHIFT)  /* Pin  3 */
# define GPIO_PIN4             (4 << GPIO_PIN_SHIFT)  /* Pin  4 */
# define GPIO_PIN5             (5 << GPIO_PIN_SHIFT)  /* Pin  5 */
# define GPIO_PIN6             (6 << GPIO_PIN_SHIFT)  /* Pin  6 */
# define GPIO_PIN7             (7 << GPIO_PIN_SHIFT)  /* Pin  7 */
# define GPIO_PIN8             (8 << GPIO_PIN_SHIFT)  /* Pin  8 */
# define GPIO_PIN9             (9 << GPIO_PIN_SHIFT)  /* Pin  9 */
# define GPIO_PIN10            (10 << GPIO_PIN_SHIFT) /* Pin 10 */
# define GPIO_PIN11            (11 << GPIO_PIN_SHIFT) /* Pin 11 */
# define GPIO_PIN12            (12 << GPIO_PIN_SHIFT) /* Pin 12 */
# define GPIO_PIN13            (13 << GPIO_PIN_SHIFT) /* Pin 13 */
# define GPIO_PIN14            (14 << GPIO_PIN_SHIFT) /* Pin 14 */
# define GPIO_PIN15            (15 << GPIO_PIN_SHIFT) /* Pin 15 */
# define GPIO_PIN16            (16 << GPIO_PIN_SHIFT) /* Pin 16 */
# define GPIO_PIN17            (17 << GPIO_PIN_SHIFT) /* Pin 17 */
# define GPIO_PIN18            (18 << GPIO_PIN_SHIFT) /* Pin 18 */
# define GPIO_PIN19            (19 << GPIO_PIN_SHIFT) /* Pin 19 */
# define GPIO_PIN20            (20 << GPIO_PIN_SHIFT) /* Pin 20 */
# define GPIO_PIN21            (21 << GPIO_PIN_SHIFT) /* Pin 21 */
# define GPIO_PIN22            (22 << GPIO_PIN_SHIFT) /* Pin 22 */
# define GPIO_PIN23            (23 << GPIO_PIN_SHIFT) /* Pin 23 */
# define GPIO_PIN24            (24 << GPIO_PIN_SHIFT) /* Pin 24 */
# define GPIO_PIN25            (25 << GPIO_PIN_SHIFT) /* Pin 25 */
# define GPIO_PIN26            (26 << GPIO_PIN_SHIFT) /* Pin 26 */
# define GPIO_PIN27            (27 << GPIO_PIN_SHIFT) /* Pin 27 */
# define GPIO_PIN28            (28 << GPIO_PIN_SHIFT) /* Pin 28 */
# define GPIO_PIN29            (29 << GPIO_PIN_SHIFT) /* Pin 29 */
# define GPIO_PIN30            (30 << GPIO_PIN_SHIFT) /* Pin 30 */
# define GPIO_PIN31            (31 << GPIO_PIN_SHIFT) /* Pin 31 */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/* The smallest integer type that can hold the GPIO encoding */

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

/************************************************************************************
 * Name: imx_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for PIO pins.
 *
 ************************************************************************************/

#ifdef CONFIG_IMX6_GPIO_IRQ
void imx_gpioirqinitialize(void);
#else
#  define imx_gpioirqinitialize()
#endif

/************************************************************************************
 * Name: imx_config_gpio
 *
 * Description:
 *   Configure a PIO pin based on bit-encoded description of the pin.
 *
 ************************************************************************************/

int imx_config_gpio(gpio_pinset_t cfgset);

/************************************************************************************
 * Name: imx_gpio_write
 *
 * Description:
 *   Write one or zero to the selected PIO pin
 *
 ************************************************************************************/

void imx_gpio_write(gpio_pinset_t pinset, bool value);

/************************************************************************************
 * Name: imx_gpio_read
 *
 * Description:
 *   Read one or zero from the selected PIO pin
 *
 ************************************************************************************/

bool imx_gpio_read(gpio_pinset_t pinset);

/************************************************************************************
 * Name: imx_gpioirq
 *
 * Description:
 *   Configure an interrupt for the specified PIO pin.
 *
 ************************************************************************************/

#ifdef CONFIG_IMX6_GPIO_IRQ
void imx_gpioirq(gpio_pinset_t pinset);
#else
#  define imx_gpioirq(pinset)
#endif

/************************************************************************************
 * Name: imx_gpioirq_enable
 *
 * Description:
 *   Enable the interrupt for specified PIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_IMX6_GPIO_IRQ
void imx_gpioirq_enable(int irq);
#else
#  define imx_gpioirq_enable(irq)
#endif

/************************************************************************************
 * Name: imx_gpioirq_disable
 *
 * Description:
 *   Disable the interrupt for specified PIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_IMX6_GPIO_IRQ
void imx_gpioirq_disable(int irq);
#else
#  define imx_gpioirq_disable(irq)
#endif

/************************************************************************************
 * Function:  imx_dump_gpio
 *
 * Description:
 *   Dump all PIO registers associated with the base address of the provided pinset.
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_GPIO
int imx_dump_gpio(uint32_t pinset, const char *msg);
#else
#  define imx_dumpgpio(p,m)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif  /* __ARCH_ARM_SRC_IMX6_IMX_GPIO_H */
