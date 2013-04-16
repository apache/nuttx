/********************************************************************************************
 * arch/arm/src/nuc1xx/chip/nuc_gpio.h
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_KL_CHIP_KL_GPIO_H
#define __ARCH_ARM_SRC_KL_CHIP_KL_GPIO_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* Register offsets *************************************************************************/

#define KL_GPIO_PORTA             0
#define KL_GPIO_PORTB             1
#define KL_GPIO_PORTC             2
#define KL_GPIO_PORTD             3
#define KL_GPIO_PORTE             4

#define KL_GPIO_NPORTS            5

/* GPIO control registers */

#define KL_GPIO_CTRL_OFFSET(n)    (0x0000 + ((n) << 6))
#define KL_GPIOA_CTRL_OFFSET      (0x0000 + ((KL_GPIO_PORTA) << 6))
#define KL_GPIOB_CTRL_OFFSET      (0x0000 + ((KL_GPIO_PORTB) << 6))
#define KL_GPIOC_CTRL_OFFSET      (0x0000 + ((KL_GPIO_PORTC) << 6))
#define KL_GPIOD_CTRL_OFFSET      (0x0000 + ((KL_GPIO_PORTD) << 6))
#define KL_GPIOE_CTRL_OFFSET      (0x0000 + ((KL_GPIO_PORTE) << 6))

#define KL_GPIO_PMD_OFFSET        0x0000 /* GPIO port pin I/O mode control */
#define KL_GPIO_OFFD_OFFSET       0x0004 /* GPIO port pin digital input path disable control */
#define KL_GPIO_DOUT_OFFSET       0x0008 /* GPIO port data output value */
#define KL_GPIO_DMASK_OFFSET      0x000c /* GPIO port data output write mask */
#define KL_GPIO_PIN_OFFSET        0x0010 /* GPIO port pin value */
#define KL_GPIO_DBEN_OFFSET       0x0014 /* GPIO port de-bounce enable */
#define KL_GPIO_IMD_OFFSET        0x0018 /* GPIO port interrupt mode control */
#define KL_GPIO_IEN_OFFSET        0x001c /* GPIO port interrupt enable */
#define KL_GPIO_ISRC_OFFSET       0x0020 /* GPIO port interrupt source flag */

/* GPIO port A control registers */

#define KL_GPIOA_PMD_OFFSET       (KL_GPIOA_CTRL_OFFSET+KL_GPIO_PMD_OFFSET)
#define KL_GPIOA_OFFD_OFFSET      (KL_GPIOA_CTRL_OFFSET+KL_GPIO_OFFD_OFFSET)
#define KL_GPIOA_DOUT_OFFSET      (KL_GPIOA_CTRL_OFFSET+KL_GPIO_DOUT_OFFSET)
#define KL_GPIOA_DMASK_OFFSET     (KL_GPIOA_CTRL_OFFSET+KL_GPIO_DMASK_OFFSET)
#define KL_GPIOA_PIN_OFFSET       (KL_GPIOA_CTRL_OFFSET+KL_GPIO_PIN_OFFSET)
#define KL_GPIOA_DBEN_OFFSET      (KL_GPIOA_CTRL_OFFSET+KL_GPIO_DBEN_OFFSET)
#define KL_GPIOA_IMD_OFFSET       (KL_GPIOA_CTRL_OFFSET+KL_GPIO_IMD_OFFSET)
#define KL_GPIOA_IEN_OFFSET       (KL_GPIOA_CTRL_OFFSET+KL_GPIO_IEN_OFFSET)
#define KL_GPIOA_ISRC_OFFSET      (KL_GPIOA_CTRL_OFFSET+KL_GPIO_ISRC_OFFSET)

/* GPIO port B control registers */

#define KL_GPIOB_PMD_OFFSET       (KL_GPIOB_CTRL_OFFSET+KL_GPIO_PMD_OFFSET)
#define KL_GPIOB_OFFD_OFFSET      (KL_GPIOB_CTRL_OFFSET+KL_GPIO_OFFD_OFFSET)
#define KL_GPIOB_DOUT_OFFSET      (KL_GPIOB_CTRL_OFFSET+KL_GPIO_DOUT_OFFSET)
#define KL_GPIOB_DMASK_OFFSET     (KL_GPIOB_CTRL_OFFSET+KL_GPIO_DMASK_OFFSET)
#define KL_GPIOB_PIN_OFFSET       (KL_GPIOB_CTRL_OFFSET+KL_GPIO_PIN_OFFSET)
#define KL_GPIOB_DBEN_OFFSET      (KL_GPIOB_CTRL_OFFSET+KL_GPIO_DBEN_OFFSET)
#define KL_GPIOB_IMD_OFFSET       (KL_GPIOB_CTRL_OFFSET+KL_GPIO_IMD_OFFSET)
#define KL_GPIOB_IEN_OFFSET       (KL_GPIOB_CTRL_OFFSET+KL_GPIO_IEN_OFFSET)
#define KL_GPIOB_ISRC_OFFSET      (KL_GPIOB_CTRL_OFFSET+KL_GPIO_ISRC_OFFSET)

/* GPIO port C control registers */

#define KL_GPIOC_PMD_OFFSET       (KL_GPIOC_CTRL_OFFSET+KL_GPIO_PMD_OFFSET)
#define KL_GPIOC_OFFD_OFFSET      (KL_GPIOC_CTRL_OFFSET+KL_GPIO_OFFD_OFFSET)
#define KL_GPIOC_DOUT_OFFSET      (KL_GPIOC_CTRL_OFFSET+KL_GPIO_DOUT_OFFSET)
#define KL_GPIOC_DMASK_OFFSET     (KL_GPIOC_CTRL_OFFSET+KL_GPIO_DMASK_OFFSET)
#define KL_GPIOC_PIN_OFFSET       (KL_GPIOC_CTRL_OFFSET+KL_GPIO_PIN_OFFSET)
#define KL_GPIOC_DBEN_OFFSET      (KL_GPIOC_CTRL_OFFSET+KL_GPIO_DBEN_OFFSET)
#define KL_GPIOC_IMD_OFFSET       (KL_GPIOC_CTRL_OFFSET+KL_GPIO_IMD_OFFSET)
#define KL_GPIOC_IEN_OFFSET       (KL_GPIOC_CTRL_OFFSET+KL_GPIO_IEN_OFFSET)
#define KL_GPIOC_ISRC_OFFSET      (KL_GPIOC_CTRL_OFFSET+KL_GPIO_ISRC_OFFSET)

/* GPIO port D control registers */

#define KL_GPIOD_PMD_OFFSET       (KL_GPIOD_CTRL_OFFSET+KL_GPIO_PMD_OFFSET)
#define KL_GPIOD_OFFD_OFFSET      (KL_GPIOD_CTRL_OFFSET+KL_GPIO_OFFD_OFFSET)
#define KL_GPIOD_DOUT_OFFSET      (KL_GPIOD_CTRL_OFFSET+KL_GPIO_DOUT_OFFSET)
#define KL_GPIOD_DMASK_OFFSET     (KL_GPIOD_CTRL_OFFSET+KL_GPIO_DMASK_OFFSET)
#define KL_GPIOD_PIN_OFFSET       (KL_GPIOD_CTRL_OFFSET+KL_GPIO_PIN_OFFSET)
#define KL_GPIOD_DBEN_OFFSET      (KL_GPIOD_CTRL_OFFSET+KL_GPIO_DBEN_OFFSET)
#define KL_GPIOD_IMD_OFFSET       (KL_GPIOD_CTRL_OFFSET+KL_GPIO_IMD_OFFSET)
#define KL_GPIOD_IEN_OFFSET       (KL_GPIOD_CTRL_OFFSET+KL_GPIO_IEN_OFFSET)
#define KL_GPIOD_ISRC_OFFSET      (KL_GPIOD_CTRL_OFFSET+KL_GPIO_ISRC_OFFSET)

/* GPIO port E control registers */

#define KL_GPIOE_PMD_OFFSET       (KL_GPIOE_CTRL_OFFSET+KL_GPIO_PMD_OFFSET)
#define KL_GPIOE_OFFD_OFFSET      (KL_GPIOE_CTRL_OFFSET+KL_GPIO_OFFD_OFFSET)
#define KL_GPIOE_DOUT_OFFSET      (KL_GPIOE_CTRL_OFFSET+KL_GPIO_DOUT_OFFSET)
#define KL_GPIOE_DMASK_OFFSET     (KL_GPIOE_CTRL_OFFSET+KL_GPIO_DMASK_OFFSET)
#define KL_GPIOE_PIN_OFFSET       (KL_GPIOE_CTRL_OFFSET+KL_GPIO_PIN_OFFSET)
#define KL_GPIOE_DBEN_OFFSET      (KL_GPIOE_CTRL_OFFSET+KL_GPIO_DBEN_OFFSET)
#define KL_GPIOE_IMD_OFFSET       (KL_GPIOE_CTRL_OFFSET+KL_GPIO_IMD_OFFSET)
#define KL_GPIOE_IEN_OFFSET       (KL_GPIOE_CTRL_OFFSET+KL_GPIO_IEN_OFFSET)
#define KL_GPIOE_ISRC_OFFSET      (KL_GPIOE_CTRL_OFFSET+KL_GPIO_ISRC_OFFSET)

/* Debounce control registers */

#define KL_GPIO_DBNCECON_OFFSEt   0x0180 /* De-bounce cycle control register */

/* GPIO port data I/O register offsets */

#define KL_PORT_DATAIO_OFFSET(p)  (0x0200 + ((p)<< 6))
#  define KL_PORTA_DATAIO_OFFSET  (0x0200 + ((KL_GPIO_PORTA) << 6))
#  define KL_PORTB_DATAIO_OFFSET  (0x0200 + ((KL_GPIO_PORTB) << 6))
#  define KL_PORTC_DATAIO_OFFSET  (0x0200 + ((KL_GPIO_PORTC) << 6))
#  define KL_PORTD_DATAIO_OFFSET  (0x0200 + ((KL_GPIO_PORTD) << 6))
#  define KL_PORTE_DATAIO_OFFSET  (0x0200 + ((KL_GPIO_PORTE) << 6))

/* GPIO port pin data I/O register offsets */

#define KL_PORT_PIN_OFFSET(n)     ((n) << 2)
#  define KL_PORT_PIN0_OFFSET     (0 << 2)
#  define KL_PORT_PIN1_OFFSET     (1 << 2)
#  define KL_PORT_PIN2_OFFSET     (2 << 2)
#  define KL_PORT_PIN3_OFFSET     (3 << 2)
#  define KL_PORT_PIN4_OFFSET     (4 << 2)
#  define KL_PORT_PIN5_OFFSET     (5 << 2)
#  define KL_PORT_PIN6_OFFSET     (16 << 2)
#  define KL_PORT_PIN7_OFFSET     (17 << 2)
#  define KL_PORT_PIN8_OFFSET     (18 << 2)
#  define KL_PORT_PIN9_OFFSET     (19 << 2)
#  define KL_PORT_PIN10_OFFSET    (10 << 2)
#  define KL_PORT_PIN11_OFFSET    (11 << 2)
#  define KL_PORT_PIN12_OFFSET    (12 << 2)
#  define KL_PORT_PIN13_OFFSET    (13 << 2)
#  define KL_PORT_PIN14_OFFSET    (14 << 2)
#  define KL_PORT_PIN15_OFFSET    (15 << 2)

#define KL_PORT_PDIO_OFFSET(p,n)  (KL_PORT_DATAIO_OFFSET(p)+KL_PORT_PIN_OFFSET(n))

/* GPIO PA Pin Data Input/Output */

#define KL_PORTA_PDIO_OFFSET(n)   (KL_PORTA_DATAIO_OFFSET+KL_PORT_PIN_OFFSET(n))
#  define KL_PA1_PDIO_OFFSET      (KL_PORTA_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PA2_PDIO_OFFSET      (KL_PORTA_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PA3_PDIO_OFFSET      (KL_PORTA_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PA4_PDIO_OFFSET      (KL_PORTA_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PA5_PDIO_OFFSET      (KL_PORTA_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PA6_PDIO_OFFSET      (KL_PORTA_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PA7_PDIO_OFFSET      (KL_PORTA_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PA8_PDIO_OFFSET      (KL_PORTA_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PA9_PDIO_OFFSET      (KL_PORTA_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PA10_PDIO_OFFSET     (KL_PORTA_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PA11_PDIO_OFFSET     (KL_PORTA_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PA12_PDIO_OFFSET     (KL_PORTA_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PA13_PDIO_OFFSET     (KL_PORTA_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PA14_PDIO_OFFSET     (KL_PORTA_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PA15_PDIO_OFFSET     (KL_PORTA_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)

/* GPIO PB Pin Data Input/Output */

#define KL_PORTB_PDIO_OFFSET(n)   (KL_PORTB_DATAIO_OFFSET+KL_PORT_PIN_OFFSET(n))
#  define KL_PB1_PDIO_OFFSET      (KL_PORTB_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PB2_PDIO_OFFSET      (KL_PORTB_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PB3_PDIO_OFFSET      (KL_PORTB_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PB4_PDIO_OFFSET      (KL_PORTB_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PB5_PDIO_OFFSET      (KL_PORTB_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PB6_PDIO_OFFSET      (KL_PORTB_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PB7_PDIO_OFFSET      (KL_PORTB_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PB8_PDIO_OFFSET      (KL_PORTB_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PB9_PDIO_OFFSET      (KL_PORTB_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PB10_PDIO_OFFSET     (KL_PORTB_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PB11_PDIO_OFFSET     (KL_PORTB_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PB12_PDIO_OFFSET     (KL_PORTB_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PB13_PDIO_OFFSET     (KL_PORTB_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PB14_PDIO_OFFSET     (KL_PORTB_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PB15_PDIO_OFFSET     (KL_PORTB_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)

/* GPIO PC Pin Data Input/Output */

#define KL_PORTC_PDIO_OFFSET(n)   (KL_PORTC_DATAIO_OFFSET+KL_PORT_PIN_OFFSET(n))
#  define KL_PC1_PDIO_OFFSET      (KL_PORTC_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PC2_PDIO_OFFSET      (KL_PORTC_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PC3_PDIO_OFFSET      (KL_PORTC_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PC4_PDIO_OFFSET      (KL_PORTC_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PC5_PDIO_OFFSET      (KL_PORTC_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PC6_PDIO_OFFSET      (KL_PORTC_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PC7_PDIO_OFFSET      (KL_PORTC_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PC8_PDIO_OFFSET      (KL_PORTC_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PC9_PDIO_OFFSET      (KL_PORTC_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PC10_PDIO_OFFSET     (KL_PORTC_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PC11_PDIO_OFFSET     (KL_PORTC_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PC12_PDIO_OFFSET     (KL_PORTC_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PC13_PDIO_OFFSET     (KL_PORTC_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PC14_PDIO_OFFSET     (KL_PORTC_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PC15_PDIO_OFFSET     (KL_PORTC_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)

/* GPIO PD Pin Data Input/Output */

#define KL_PORTD_PDIO_OFFSET(n)   (KL_PORTD_DATAIO_OFFSET+KL_PORT_PIN_OFFSET(n))
#  define KL_PD1_PDIO_OFFSET      (KL_PORTD_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PD2_PDIO_OFFSET      (KL_PORTD_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PD3_PDIO_OFFSET      (KL_PORTD_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PD4_PDIO_OFFSET      (KL_PORTD_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PD5_PDIO_OFFSET      (KL_PORTD_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PD6_PDIO_OFFSET      (KL_PORTD_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PD7_PDIO_OFFSET      (KL_PORTD_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PD8_PDIO_OFFSET      (KL_PORTD_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PD9_PDIO_OFFSET      (KL_PORTD_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PD10_PDIO_OFFSET     (KL_PORTD_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PD11_PDIO_OFFSET     (KL_PORTD_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PD12_PDIO_OFFSET     (KL_PORTD_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PD13_PDIO_OFFSET     (KL_PORTD_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PD14_PDIO_OFFSET     (KL_PORTD_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)
#  define KL_PD15_PDIO_OFFSET     (KL_PORTD_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)

/* GPIO PE Pin Data Input/Output */

#define KL_PORTE_PDIO_OFFSET(n)   (KL_PORTE_DATAIO_OFFSET+KL_PORT_PIN_OFFSET(n))
#  define KL_PE5_PDIO_OFFSET      (KL_PORTE_DATAIO_OFFSET+KL_PORT_PIN0_OFFSET)

/* Register addresses ***********************************************************************/

/* GPIO control registers */

#define KL_GPIO_CTRL_BASE(n)      (KL_GPIO_BASEOLD+KL_GPIO_CTRL_OFFSET(n))
#define KL_GPIOA_CTRL_BASE        (KL_GPIO_BASEOLD+KL_GPIOA_CTRL_OFFSET)
#define KL_GPIOB_CTRL_BASE        (KL_GPIO_BASEOLD+KL_GPIOB_CTRL_OFFSET)
#define KL_GPIOC_CTRL_BASE        (KL_GPIO_BASEOLD+KL_GPIOC_CTRL_OFFSET)
#define KL_GPIOD_CTRL_BASE        (KL_GPIO_BASEOLD+KL_GPIOD_CTRL_OFFSET)
#define KL_GPIOE_CTRL_BASE        (KL_GPIO_BASEOLD+KL_GPIOE_CTRL_OFFSET)

/* GPIO port A control registers */

#define KL_GPIOA_PMD              (KL_GPIO_BASEOLD+KL_GPIOA_PMD_OFFSET)
#define KL_GPIOA_OFFD             (KL_GPIO_BASEOLD+KL_GPIOA_OFFD_OFFSET)
#define KL_GPIOA_DOUT             (KL_GPIO_BASEOLD+KL_GPIOA_DOUT_OFFSET)
#define KL_GPIOA_DMASK            (KL_GPIO_BASEOLD+KL_GPIOA_DMASK_OFFSET)
#define KL_GPIOA_PIN              (KL_GPIO_BASEOLD+KL_GPIOA_PIN_OFFSET)
#define KL_GPIOA_DBEN             (KL_GPIO_BASEOLD+KL_GPIOA_DBEN_OFFSET)
#define KL_GPIOA_IMD              (KL_GPIO_BASEOLD+KL_GPIOA_IMD_OFFSET)
#define KL_GPIOA_IEN              (KL_GPIO_BASEOLD+KL_GPIOA_IEN_OFFSET)
#define KL_GPIOA_ISRC             (KL_GPIO_BASEOLD+KL_GPIOA_ISRC_OFFSET)

/* GPIO port B control registers */

#define KL_GPIOB_PMD              (KL_GPIO_BASEOLD+KL_GPIOB_PMD_OFFSET)
#define KL_GPIOB_OFFD             (KL_GPIO_BASEOLD+KL_GPIOB_OFFD_OFFSET)
#define KL_GPIOB_DOUT             (KL_GPIO_BASEOLD+KL_GPIOB_DOUT_OFFSET)
#define KL_GPIOB_DMASK            (KL_GPIO_BASEOLD+KL_GPIOB_DMASK_OFFSET)
#define KL_GPIOB_PIN              (KL_GPIO_BASEOLD+KL_GPIOB_PIN_OFFSET)
#define KL_GPIOB_DBEN             (KL_GPIO_BASEOLD+KL_GPIOB_DBEN_OFFSET)
#define KL_GPIOB_IMD              (KL_GPIO_BASEOLD+KL_GPIOB_IMD_OFFSET)
#define KL_GPIOB_IEN              (KL_GPIO_BASEOLD+KL_GPIOB_IEN_OFFSET)
#define KL_GPIOB_ISRC             (KL_GPIO_BASEOLD+KL_GPIOB_ISRC_OFFSET)

/* GPIO port C control registers */

#define KL_GPIOC_PMD              (KL_GPIO_BASEOLD+KL_GPIOC_PMD_OFFSET)
#define KL_GPIOC_OFFD             (KL_GPIO_BASEOLD+KL_GPIOC_OFFD_OFFSET)
#define KL_GPIOC_DOUT             (KL_GPIO_BASEOLD+KL_GPIOC_DOUT_OFFSET)
#define KL_GPIOC_DMASK            (KL_GPIO_BASEOLD+KL_GPIOC_DMASK_OFFSET)
#define KL_GPIOC_PIN              (KL_GPIO_BASEOLD+KL_GPIOC_PIN_OFFSET)
#define KL_GPIOC_DBEN             (KL_GPIO_BASEOLD+KL_GPIOC_DBEN_OFFSET)
#define KL_GPIOC_IMD              (KL_GPIO_BASEOLD+KL_GPIOC_IMD_OFFSET)
#define KL_GPIOC_IEN              (KL_GPIO_BASEOLD+KL_GPIOC_IEN_OFFSET)
#define KL_GPIOC_ISRC             (KL_GPIO_BASEOLD+KL_GPIOC_ISRC_OFFSET)

/* GPIO port D control registers */

#define KL_GPIOD_PMD              (KL_GPIO_BASEOLD+KL_GPIOD_PMD_OFFSET)
#define KL_GPIOD_OFFD             (KL_GPIO_BASEOLD+KL_GPIOD_OFFD_OFFSET)
#define KL_GPIOD_DOUT             (KL_GPIO_BASEOLD+KL_GPIOD_DOUT_OFFSET)
#define KL_GPIOD_DMASK            (KL_GPIO_BASEOLD+KL_GPIOD_DMASK_OFFSET)
#define KL_GPIOD_PIN              (KL_GPIO_BASEOLD+KL_GPIOD_PIN_OFFSET)
#define KL_GPIOD_DBEN             (KL_GPIO_BASEOLD+KL_GPIOD_DBEN_OFFSET)
#define KL_GPIOD_IMD              (KL_GPIO_BASEOLD+KL_GPIOD_IMD_OFFSET)
#define KL_GPIOD_IEN              (KL_GPIO_BASEOLD+KL_GPIOD_IEN_OFFSET)
#define KL_GPIOD_ISRC             (KL_GPIO_BASEOLD+KL_GPIOD_ISRC_OFFSET)

/* GPIO port E control registers */

#define KL_GPIOE_PMD              (KL_GPIO_BASEOLD+KL_GPIOE_PMD_OFFSET)
#define KL_GPIOE_OFFD             (KL_GPIO_BASEOLD+KL_GPIOE_OFFD_OFFSET)
#define KL_GPIOE_DOUT             (KL_GPIO_BASEOLD+KL_GPIOE_DOUT_OFFSET)
#define KL_GPIOE_DMASK            (KL_GPIO_BASEOLD+KL_GPIOE_DMASK_OFFSET)
#define KL_GPIOE_PIN              (KL_GPIO_BASEOLD+KL_GPIOE_PIN_OFFSET)
#define KL_GPIOE_DBEN             (KL_GPIO_BASEOLD+KL_GPIOE_DBEN_OFFSET)
#define KL_GPIOE_IMD              (KL_GPIO_BASEOLD+KL_GPIOE_IMD_OFFSET)
#define KL_GPIOE_IEN              (KL_GPIO_BASEOLD+KL_GPIOE_IEN_OFFSET)
#define KL_GPIOE_ISRC             (KL_GPIO_BASEOLD+KL_GPIOE_ISRC_OFFSET)

/* Debounce control registers */

#define KL_GPIO_DBNCECON          (KL_GPIO_BASEOLD+KL_GPIO_DBNCECON_OFFSET)

/* GPIO port data I/O register offsets */

#define KL_PORT_DATAIO_BASE(p)    (KL_GPIO_BASEOLD+KL_PORT_DATAIO_OFFSET(p))
#  define KL_PORTA_DATAIO_BASE    (KL_GPIO_BASEOLD+KL_PORTA_DATAIO_OFFSET)
#  define KL_PORTB_DATAIO_BASE    (KL_GPIO_BASEOLD+KL_PORTB_DATAIO_OFFSET)
#  define KL_PORTC_DATAIO_BASE    (KL_GPIO_BASEOLD+KL_PORTC_DATAIO_OFFSET)
#  define KL_PORTD_DATAIO_BASE    (KL_GPIO_BASEOLD+KL_PORTD_DATAIO_OFFSET)
#  define KL_PORTE_DATAIO_BASE    (KL_GPIO_BASEOLD+KL_PORTE_DATAIO_OFFSET)

#define KL_PORT_PDIO(p,n)         (KL_GPIO_BASEOLD+KL_PORT_PDIO_OFFSET(p,n))

/* GPIO PA Pin Data Input/Output */

#define KL_PORTA_PDIO(n)          (KL_GPIO_BASEOLD+KL_PORTA_PDIO_OFFSET(n))
#  define KL_PA1_PDIO             (KL_GPIO_BASEOLD+KL_PA1_PDIO_OFFSET)
#  define KL_PA2_PDIO             (KL_GPIO_BASEOLD+KL_PA2_PDIO_OFFSET)
#  define KL_PA3_PDIO             (KL_GPIO_BASEOLD+KL_PA3_PDIO_OFFSET)
#  define KL_PA4_PDIO             (KL_GPIO_BASEOLD+KL_PA4_PDIO_OFFSET)
#  define KL_PA5_PDIO             (KL_GPIO_BASEOLD+KL_PA5_PDIO_OFFSET)
#  define KL_PA6_PDIO             (KL_GPIO_BASEOLD+KL_PA6_PDIO_OFFSET)
#  define KL_PA7_PDIO             (KL_GPIO_BASEOLD+KL_PA7_PDIO_OFFSET)
#  define KL_PA8_PDIO             (KL_GPIO_BASEOLD+KL_PA8_PDIO_OFFSET)
#  define KL_PA9_PDIO             (KL_GPIO_BASEOLD+KL_PA9_PDIO_OFFSET)
#  define KL_PA10_PDIO            (KL_GPIO_BASEOLD+KL_PA10_PDIO_OFFSET)
#  define KL_PA11_PDIO            (KL_GPIO_BASEOLD+KL_PA11_PDIO_OFFSET)
#  define KL_PA12_PDIO            (KL_GPIO_BASEOLD+KL_PA12_PDIO_OFFSET)
#  define KL_PA13_PDIO            (KL_GPIO_BASEOLD+KL_PA13_PDIO_OFFSET)
#  define KL_PA14_PDIO            (KL_GPIO_BASEOLD+KL_PA14_PDIO_OFFSET)
#  define KL_PA15_PDIO            (KL_GPIO_BASEOLD+KL_PA15_PDIO_OFFSET)

/* GPIO PB Pin Data Input/Output */

#define KL_PORTB_PDIO(n)          (KL_GPIO_BASEOLD+KL_PORTB_PDIO_OFFSET(n))
#  define KL_PB1_PDIO             (KL_GPIO_BASEOLD+KL_PB1_PDIO_OFFSET)
#  define KL_PB2_PDIO             (KL_GPIO_BASEOLD+KL_PB2_PDIO_OFFSET)
#  define KL_PB3_PDIO             (KL_GPIO_BASEOLD+KL_PB3_PDIO_OFFSET)
#  define KL_PB4_PDIO             (KL_GPIO_BASEOLD+KL_PB4_PDIO_OFFSET)
#  define KL_PB5_PDIO             (KL_GPIO_BASEOLD+KL_PB5_PDIO_OFFSET)
#  define KL_PB6_PDIO             (KL_GPIO_BASEOLD+KL_PB6_PDIO_OFFSET)
#  define KL_PB7_PDIO             (KL_GPIO_BASEOLD+KL_PB7_PDIO_OFFSET)
#  define KL_PB8_PDIO             (KL_GPIO_BASEOLD+KL_PB8_PDIO_OFFSET)
#  define KL_PB9_PDIO             (KL_GPIO_BASEOLD+KL_PB9_PDIO_OFFSET)
#  define KL_PB10_PDIO            (KL_GPIO_BASEOLD+KL_PB10_PDIO_OFFSET)
#  define KL_PB11_PDIO            (KL_GPIO_BASEOLD+KL_PB11_PDIO_OFFSET)
#  define KL_PB12_PDIO            (KL_GPIO_BASEOLD+KL_PB12_PDIO_OFFSET)
#  define KL_PB13_PDIO            (KL_GPIO_BASEOLD+KL_PB13_PDIO_OFFSET)
#  define KL_PB14_PDIO            (KL_GPIO_BASEOLD+KL_PB14_PDIO_OFFSET)
#  define KL_PB15_PDIO            (KL_GPIO_BASEOLD+KL_PB15_PDIO_OFFSET)

/* GPIO PC Pin Data Input/Output */

#define KL_PORTC_PDIO(n)          (KL_GPIO_BASEOLD+KL_PORTC_PDIO_OFFSET(n))
#  define KL_PC1_PDIO             (KL_GPIO_BASEOLD+KL_PC1_PDIO_OFFSET)
#  define KL_PC2_PDIO             (KL_GPIO_BASEOLD+KL_PC2_PDIO_OFFSET)
#  define KL_PC3_PDIO             (KL_GPIO_BASEOLD+KL_PC3_PDIO_OFFSET)
#  define KL_PC4_PDIO             (KL_GPIO_BASEOLD+KL_PC4_PDIO_OFFSET)
#  define KL_PC5_PDIO             (KL_GPIO_BASEOLD+KL_PC5_PDIO_OFFSET)
#  define KL_PC6_PDIO             (KL_GPIO_BASEOLD+KL_PC6_PDIO_OFFSET)
#  define KL_PC7_PDIO             (KL_GPIO_BASEOLD+KL_PC7_PDIO_OFFSET)
#  define KL_PC8_PDIO             (KL_GPIO_BASEOLD+KL_PC8_PDIO_OFFSET)
#  define KL_PC9_PDIO             (KL_GPIO_BASEOLD+KL_PC9_PDIO_OFFSET)
#  define KL_PC10_PDIO            (KL_GPIO_BASEOLD+KL_PC10_PDIO_OFFSET)
#  define KL_PC11_PDIO            (KL_GPIO_BASEOLD+KL_PC11_PDIO_OFFSET)
#  define KL_PC12_PDIO            (KL_GPIO_BASEOLD+KL_PC12_PDIO_OFFSET)
#  define KL_PC13_PDIO            (KL_GPIO_BASEOLD+KL_PC13_PDIO_OFFSET)
#  define KL_PC14_PDIO            (KL_GPIO_BASEOLD+KL_PC14_PDIO_OFFSET)
#  define KL_PC15_PDIO            (KL_GPIO_BASEOLD+KL_PC15_PDIO_OFFSET)

/* GPIO PD Pin Data Input/Output */

#define KL_PORTD_PDIO(n)          (KL_GPIO_BASEOLD+KL_PORTD_PDIO_OFFSET(n))
#  define KL_PD1_PDIO             (KL_GPIO_BASEOLD+KL_PD1_PDIO_OFFSET)
#  define KL_PD2_PDIO             (KL_GPIO_BASEOLD+KL_PD2_PDIO_OFFSET)
#  define KL_PD3_PDIO             (KL_GPIO_BASEOLD+KL_PD3_PDIO_OFFSET)
#  define KL_PD4_PDIO             (KL_GPIO_BASEOLD+KL_PD4_PDIO_OFFSET)
#  define KL_PD5_PDIO             (KL_GPIO_BASEOLD+KL_PD5_PDIO_OFFSET)
#  define KL_PD6_PDIO             (KL_GPIO_BASEOLD+KL_PD6_PDIO_OFFSET)
#  define KL_PD7_PDIO             (KL_GPIO_BASEOLD+KL_PD7_PDIO_OFFSET)
#  define KL_PD8_PDIO             (KL_GPIO_BASEOLD+KL_PD8_PDIO_OFFSET)
#  define KL_PD9_PDIO             (KL_GPIO_BASEOLD+KL_PD9_PDIO_OFFSET)
#  define KL_PD10_PDIO            (KL_GPIO_BASEOLD+KL_PD10_PDIO_OFFSET)
#  define KL_PD11_PDIO            (KL_GPIO_BASEOLD+KL_PD11_PDIO_OFFSET)
#  define KL_PD12_PDIO            (KL_GPIO_BASEOLD+KL_PD12_PDIO_OFFSET)
#  define KL_PD13_PDIO            (KL_GPIO_BASEOLD+KL_PD13_PDIO_OFFSET)
#  define KL_PD14_PDIO            (KL_GPIO_BASEOLD+KL_PD14_PDIO_OFFSET)
#  define KL_PD15_PDIO            (KL_GPIO_BASEOLD+KL_PD15_PDIO_OFFSET)

/* GPIO PE Pin Data Input/Output */

#define KL_PORTE_PDIO(n)          (KL_GPIO_BASEOLD+KL_PORTE_PDIO_OFFSET(n))
#  define KL_PE5_PDIO             (KL_GPIO_BASEOLD+KL_PE5_PDIO_OFFSET)

/* Register bit-field definitions ***********************************************************/

/* GPIO port pin I/O mode control */

#define GPIO_PMD_INPUT            0 /* Input */
#define GPIO_PMD_OUTPUT           1 /* Push-pull output */
#define GPIO_PMD_OPENDRAIN        2 /* Open drain output */
#define GPIO_PMD_BIDI             3 /* Quasi bi-directional */

#define GPIO_PMD_SHIFT(n)         ((n) << 1) /* Bits 2n-2n+1: GPIOx Pin[n] mode control */ 
#define GPIO_PMD_MASK(n)          (3 << GPIO_PMD_SHIFT(n))
#  define GPIO_PMD(n,v)           ((v) << GPIO_PMD_SHIFT(n))

#define GPIO_PMD0_SHIFT           (0)      /* Bits 0-1: GPIOx Pin0 mode control */ 
#define GPIO_PMD0_MASK            (3 << GPIO_PMD0_SHIFT)
#  define GPIO_PMD0(v)            ((v) << GPIO_PMD0_SHIFT)
#define GPIO_PMD1_SHIFT           (2)      /* Bits 2-3: GPIOx Pin1 mode control */ 
#define GPIO_PMD1_MASK            (3 << GPIO_PMD1_SHIFT)
#  define GPIO_PMD1(v)            ((v) << GPIO_PMD1_SHIFT)
#define GPIO_PMD2_SHIFT           (4)      /* Bits 4-5: GPIOx Pin2 mode control */ 
#define GPIO_PMD2_MASK            (3 << GPIO_PMD2_SHIFT)
#  define GPIO_PMD2(v)            ((v) << GPIO_PMD2_SHIFT)
#define GPIO_PMD3_SHIFT           (6)      /* Bits 6-7: GPIOx Pin3 mode control */ 
#define GPIO_PMD3_MASK            (3 << GPIO_PMD3_SHIFT)
#  define GPIO_PMD3(v)            ((v) << GPIO_PMD3_SHIFT)
#define GPIO_PMD4_SHIFT           (8)      /* Bits 8-9: GPIOx Pin4 mode control */ 
#define GPIO_PMD4_MASK            (3 << GPIO_PMD4_SHIFT)
#  define GPIO_PMD4(v)            ((v) << GPIO_PMD4_SHIFT)
#define GPIO_PMD5_SHIFT           (10)     /* Bits 10-11: GPIOx Pin5 mode control */ 
#define GPIO_PMD5_MASK            (3 << GPIO_PMD5_SHIFT)
#  define GPIO_PMD5(v)            ((v) << GPIO_PMD5_SHIFT)
#define GPIO_PMD6_SHIFT           (12)     /* Bits 12-13: GPIOx Pin6 mode control */ 
#define GPIO_PMD6_MASK            (3 << GPIO_PMD6_SHIFT)
#  define GPIO_PMD6(v)            ((v) << GPIO_PMD6_SHIFT)
#define GPIO_PMD7_SHIFT           (14)     /* Bits 14-15: GPIOx Pin7 mode control */ 
#define GPIO_PMD7_MASK            (3 << GPIO_PMD7_SHIFT)
#  define GPIO_PMD7(v)            ((v) << GPIO_PMD7_SHIFT)
#define GPIO_PMD8_SHIFT           (16)     /* Bits 16-17: GPIOx Pin8 mode control */ 
#define GPIO_PMD8_MASK            (3 << GPIO_PMD8_SHIFT)
#  define GPIO_PMD8(v)            ((v) << GPIO_PMD8_SHIFT)
#define GPIO_PMD9_SHIFT           (18)     /* Bits 18-19: GPIOx Pin9 mode control */ 
#define GPIO_PMD9_MASK            (3 << GPIO_PMD9_SHIFT)
#  define GPIO_PMD9(v)            ((v) << GPIO_PMD9_SHIFT)
#define GPIO_PMD10_SHIFT          (20)     /* Bits 20-21: GPIOx Pin0 mode control */ 
#define GPIO_PMD10_MASK           (3 << GPIO_PMD10_SHIFT)
#  define GPIO_PMD10(v)           ((v) << GPIO_PMD10_SHIFT)
#define GPIO_PMD11_SHIFT          (22)     /* Bits 22-23: GPIOx Pin1 mode control */ 
#define GPIO_PMD11_MASK           (3 << GPIO_PMD11_SHIFT)
#  define GPIO_PMD11(v)           ((v) << GPIO_PMD11_SHIFT)
#define GPIO_PMD12_SHIFT          (24)     /* Bits 24-25: GPIOx Pin2 mode control */ 
#define GPIO_PMD12_MASK           (3 << GPIO_PMD12_SHIFT)
#  define GPIO_PMD12(v)           ((v) << GPIO_PMD12_SHIFT)
#define GPIO_PMD13_SHIFT          (26)     /* Bits 26-27: GPIOx Pin3 mode control */ 
#define GPIO_PMD13_MASK           (3 << GPIO_PMD13_SHIFT)
#  define GPIO_PMD13(v)           ((v) << GPIO_PMD13_SHIFT)
#define GPIO_PMD14_SHIFT          (28)     /* Bits 28-29: GPIOx Pin4 mode control */ 
#define GPIO_PMD14_MASK           (3 << GPIO_PMD14_SHIFT)
#  define GPIO_PMD14(v)           ((v) << GPIO_PMD14_SHIFT)
#define GPIO_PMD15_SHIFT          (30)     /* Bits 30-31: GPIOx Pin5 mode control */ 
#define GPIO_PMD15_MASK           (3 << GPIO_PMD15_SHIFT)
#  define GPIO_PMD15(v)           ((v) << GPIO_PMD15_SHIFT)

/* GPIO port pin digital input path disable control */

#define GPIO_OFFD(n)              (1 << (n)) /* Bit n: GPIOx Pin[n] digital input path disable control */

/* GPIO port data output value */

#define GPIO_DOUT(n)              (1 << (n)) /* Bit n: GPIOx Pin[n] output value */

/* GPIO port data output write mask */

#define GPIO_DMASK(n)             (1 << (n)) /* Bit n: GPIOx Pin[n] data output write mask */

/* GPIO port pin value */

#define GPIO_PIN(n)               (1 << (n)) /* Bit n: GPIOx Pin[n] pin value */

/* GPIO port de-bounce enable */

#define GPIO_DBEN(n)              (1 << (n)) /* Bit n: GPIOx Pin[n] input signal de-bounce enable */

/* GPIO port interrupt mode control */

#define GPIO_IMD(n)               (1 << (n)) /* Bit n: GPIOx Pin[n] edge/level detection interrupt control */

/* GPIO port interrupt enable */

#define GPIO_IF_EN(n)             (1 << (n)) /* Bit n: GPIOx Pin[n] interrupt enable low/falling */
#define GPIO_IR_EN(n)             (1 << ((n)+16)) /* Bit n: GPIOx Pin[n] interrupt enable high/rising */

/* GPIO port interrupt source flag */

#define GPIO_ISRC(n)              (1 << (n)) /* Bit n: GPIOx Pin[n] interrupt source flag */

/* De-bounce cycle control register */

#define GPIO_DBNCECON_DBCLKSEL_SHIFT   (0)     /* Bits 0-3: De-bounce cycling count selection */
#define GPIO_DBNCECON_DBCLKSEL_MASK    (15 << GPIO_DBNCECON_DBCLKSEL_SHIFT)
#  define GPIO_DBNCECON_DBCLKSEL_1     (0 << GPIO_DBNCECON_DBCLKSEL_SHIFT)
#  define GPIO_DBNCECON_DBCLKSEL_2     (1 << GPIO_DBNCECON_DBCLKSEL_SHIFT)
#  define GPIO_DBNCECON_DBCLKSEL_4     (2 << GPIO_DBNCECON_DBCLKSEL_SHIFT)
#  define GPIO_DBNCECON_DBCLKSEL_8     (3 << GPIO_DBNCECON_DBCLKSEL_SHIFT)
#  define GPIO_DBNCECON_DBCLKSEL_16    (4 << GPIO_DBNCECON_DBCLKSEL_SHIFT)
#  define GPIO_DBNCECON_DBCLKSEL_32    (5 << GPIO_DBNCECON_DBCLKSEL_SHIFT)
#  define GPIO_DBNCECON_DBCLKSEL_64    (6 << GPIO_DBNCECON_DBCLKSEL_SHIFT)
#  define GPIO_DBNCECON_DBCLKSEL_128   (7 << GPIO_DBNCECON_DBCLKSEL_SHIFT)
#  define GPIO_DBNCECON_DBCLKSEL_256   (8 << GPIO_DBNCECON_DBCLKSEL_SHIFT)
#  define GPIO_DBNCECON_DBCLKSEL_512   (9 << GPIO_DBNCECON_DBCLKSEL_SHIFT)
#  define GPIO_DBNCECON_DBCLKSEL_1024  (10 << GPIO_DBNCECON_DBCLKSEL_SHIFT)
#  define GPIO_DBNCECON_DBCLKSEL_2048  (11 << GPIO_DBNCECON_DBCLKSEL_SHIFT)
#  define GPIO_DBNCECON_DBCLKSEL_4096  (12 << GPIO_DBNCECON_DBCLKSEL_SHIFT)
#  define GPIO_DBNCECON_DBCLKSEL_8102  (13 << GPIO_DBNCECON_DBCLKSEL_SHIFT)
#  define GPIO_DBNCECON_DBCLKSEL_16384 (14 << GPIO_DBNCECON_DBCLKSEL_SHIFT)
#  define GPIO_DBNCECON_DBCLKSEL_32768 (15 << GPIO_DBNCECON_DBCLKSEL_SHIFT)
#define GPIO_DBNCECON_DBCLKSRC         (1 << 4)  /* Bit 4:  De-bounce counter clock source selection */
#define GPIO_DBNCECON_ICLK_ON          (1 << 5)  /* Bit 5:  Interrupt clock on mode */

/* GPIO port data I/O registers */

#define PORT_MASK                 (1) /* Bit 0: GPIOx Pin[n] data I/O */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_KL_CHIP_KL_GPIO_H */
