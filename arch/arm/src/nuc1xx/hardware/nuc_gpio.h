/****************************************************************************
 * arch/arm/src/nuc1xx/hardware/nuc_gpio.h
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

#ifndef __ARCH_ARM_SRC_NUC1XX_HARDWARE_NUC_GPIO_H
#define __ARCH_ARM_SRC_NUC1XX_HARDWARE_NUC_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NUC_GPIO_PORTA             0
#define NUC_GPIO_PORTB             1
#define NUC_GPIO_PORTC             2
#define NUC_GPIO_PORTD             3
#define NUC_GPIO_PORTE             4

#define NUC_GPIO_NPORTS            5

/* GPIO control registers */

#define NUC_GPIO_CTRL_OFFSET(n)   (0x0000 + ((n) << 6))
#define NUC_GPIOA_CTRL_OFFSET     (0x0000 + ((NUC_GPIO_PORTA) << 6))
#define NUC_GPIOB_CTRL_OFFSET     (0x0000 + ((NUC_GPIO_PORTB) << 6))
#define NUC_GPIOC_CTRL_OFFSET     (0x0000 + ((NUC_GPIO_PORTC) << 6))
#define NUC_GPIOD_CTRL_OFFSET     (0x0000 + ((NUC_GPIO_PORTD) << 6))
#define NUC_GPIOE_CTRL_OFFSET     (0x0000 + ((NUC_GPIO_PORTE) << 6))

#define NUC_GPIO_PMD_OFFSET        0x0000 /* GPIO port pin I/O mode control */
#define NUC_GPIO_OFFD_OFFSET       0x0004 /* GPIO port pin digital input path disable control */
#define NUC_GPIO_DOUT_OFFSET       0x0008 /* GPIO port data output value */
#define NUC_GPIO_DMASK_OFFSET      0x000c /* GPIO port data output write mask */
#define NUC_GPIO_PIN_OFFSET        0x0010 /* GPIO port pin value */
#define NUC_GPIO_DBEN_OFFSET       0x0014 /* GPIO port de-bounce enable */
#define NUC_GPIO_IMD_OFFSET        0x0018 /* GPIO port interrupt mode control */
#define NUC_GPIO_IEN_OFFSET        0x001c /* GPIO port interrupt enable */
#define NUC_GPIO_ISRC_OFFSET       0x0020 /* GPIO port interrupt source flag */

/* GPIO port A control registers */

#define NUC_GPIOA_PMD_OFFSET       (NUC_GPIOA_CTRL_OFFSET+NUC_GPIO_PMD_OFFSET)
#define NUC_GPIOA_OFFD_OFFSET      (NUC_GPIOA_CTRL_OFFSET+NUC_GPIO_OFFD_OFFSET)
#define NUC_GPIOA_DOUT_OFFSET      (NUC_GPIOA_CTRL_OFFSET+NUC_GPIO_DOUT_OFFSET)
#define NUC_GPIOA_DMASK_OFFSET     (NUC_GPIOA_CTRL_OFFSET+NUC_GPIO_DMASK_OFFSET)
#define NUC_GPIOA_PIN_OFFSET       (NUC_GPIOA_CTRL_OFFSET+NUC_GPIO_PIN_OFFSET)
#define NUC_GPIOA_DBEN_OFFSET      (NUC_GPIOA_CTRL_OFFSET+NUC_GPIO_DBEN_OFFSET)
#define NUC_GPIOA_IMD_OFFSET       (NUC_GPIOA_CTRL_OFFSET+NUC_GPIO_IMD_OFFSET)
#define NUC_GPIOA_IEN_OFFSET       (NUC_GPIOA_CTRL_OFFSET+NUC_GPIO_IEN_OFFSET)
#define NUC_GPIOA_ISRC_OFFSET      (NUC_GPIOA_CTRL_OFFSET+NUC_GPIO_ISRC_OFFSET)

/* GPIO port B control registers */

#define NUC_GPIOB_PMD_OFFSET       (NUC_GPIOB_CTRL_OFFSET+NUC_GPIO_PMD_OFFSET)
#define NUC_GPIOB_OFFD_OFFSET      (NUC_GPIOB_CTRL_OFFSET+NUC_GPIO_OFFD_OFFSET)
#define NUC_GPIOB_DOUT_OFFSET      (NUC_GPIOB_CTRL_OFFSET+NUC_GPIO_DOUT_OFFSET)
#define NUC_GPIOB_DMASK_OFFSET     (NUC_GPIOB_CTRL_OFFSET+NUC_GPIO_DMASK_OFFSET)
#define NUC_GPIOB_PIN_OFFSET       (NUC_GPIOB_CTRL_OFFSET+NUC_GPIO_PIN_OFFSET)
#define NUC_GPIOB_DBEN_OFFSET      (NUC_GPIOB_CTRL_OFFSET+NUC_GPIO_DBEN_OFFSET)
#define NUC_GPIOB_IMD_OFFSET       (NUC_GPIOB_CTRL_OFFSET+NUC_GPIO_IMD_OFFSET)
#define NUC_GPIOB_IEN_OFFSET       (NUC_GPIOB_CTRL_OFFSET+NUC_GPIO_IEN_OFFSET)
#define NUC_GPIOB_ISRC_OFFSET      (NUC_GPIOB_CTRL_OFFSET+NUC_GPIO_ISRC_OFFSET)

/* GPIO port C control registers */

#define NUC_GPIOC_PMD_OFFSET       (NUC_GPIOC_CTRL_OFFSET+NUC_GPIO_PMD_OFFSET)
#define NUC_GPIOC_OFFD_OFFSET      (NUC_GPIOC_CTRL_OFFSET+NUC_GPIO_OFFD_OFFSET)
#define NUC_GPIOC_DOUT_OFFSET      (NUC_GPIOC_CTRL_OFFSET+NUC_GPIO_DOUT_OFFSET)
#define NUC_GPIOC_DMASK_OFFSET     (NUC_GPIOC_CTRL_OFFSET+NUC_GPIO_DMASK_OFFSET)
#define NUC_GPIOC_PIN_OFFSET       (NUC_GPIOC_CTRL_OFFSET+NUC_GPIO_PIN_OFFSET)
#define NUC_GPIOC_DBEN_OFFSET      (NUC_GPIOC_CTRL_OFFSET+NUC_GPIO_DBEN_OFFSET)
#define NUC_GPIOC_IMD_OFFSET       (NUC_GPIOC_CTRL_OFFSET+NUC_GPIO_IMD_OFFSET)
#define NUC_GPIOC_IEN_OFFSET       (NUC_GPIOC_CTRL_OFFSET+NUC_GPIO_IEN_OFFSET)
#define NUC_GPIOC_ISRC_OFFSET      (NUC_GPIOC_CTRL_OFFSET+NUC_GPIO_ISRC_OFFSET)

/* GPIO port D control registers */

#define NUC_GPIOD_PMD_OFFSET       (NUC_GPIOD_CTRL_OFFSET+NUC_GPIO_PMD_OFFSET)
#define NUC_GPIOD_OFFD_OFFSET      (NUC_GPIOD_CTRL_OFFSET+NUC_GPIO_OFFD_OFFSET)
#define NUC_GPIOD_DOUT_OFFSET      (NUC_GPIOD_CTRL_OFFSET+NUC_GPIO_DOUT_OFFSET)
#define NUC_GPIOD_DMASK_OFFSET     (NUC_GPIOD_CTRL_OFFSET+NUC_GPIO_DMASK_OFFSET)
#define NUC_GPIOD_PIN_OFFSET       (NUC_GPIOD_CTRL_OFFSET+NUC_GPIO_PIN_OFFSET)
#define NUC_GPIOD_DBEN_OFFSET      (NUC_GPIOD_CTRL_OFFSET+NUC_GPIO_DBEN_OFFSET)
#define NUC_GPIOD_IMD_OFFSET       (NUC_GPIOD_CTRL_OFFSET+NUC_GPIO_IMD_OFFSET)
#define NUC_GPIOD_IEN_OFFSET       (NUC_GPIOD_CTRL_OFFSET+NUC_GPIO_IEN_OFFSET)
#define NUC_GPIOD_ISRC_OFFSET      (NUC_GPIOD_CTRL_OFFSET+NUC_GPIO_ISRC_OFFSET)

/* GPIO port E control registers */

#define NUC_GPIOE_PMD_OFFSET       (NUC_GPIOE_CTRL_OFFSET+NUC_GPIO_PMD_OFFSET)
#define NUC_GPIOE_OFFD_OFFSET      (NUC_GPIOE_CTRL_OFFSET+NUC_GPIO_OFFD_OFFSET)
#define NUC_GPIOE_DOUT_OFFSET      (NUC_GPIOE_CTRL_OFFSET+NUC_GPIO_DOUT_OFFSET)
#define NUC_GPIOE_DMASK_OFFSET     (NUC_GPIOE_CTRL_OFFSET+NUC_GPIO_DMASK_OFFSET)
#define NUC_GPIOE_PIN_OFFSET       (NUC_GPIOE_CTRL_OFFSET+NUC_GPIO_PIN_OFFSET)
#define NUC_GPIOE_DBEN_OFFSET      (NUC_GPIOE_CTRL_OFFSET+NUC_GPIO_DBEN_OFFSET)
#define NUC_GPIOE_IMD_OFFSET       (NUC_GPIOE_CTRL_OFFSET+NUC_GPIO_IMD_OFFSET)
#define NUC_GPIOE_IEN_OFFSET       (NUC_GPIOE_CTRL_OFFSET+NUC_GPIO_IEN_OFFSET)
#define NUC_GPIOE_ISRC_OFFSET      (NUC_GPIOE_CTRL_OFFSET+NUC_GPIO_ISRC_OFFSET)

/* Debounce control registers */

#define NUC_GPIO_DBNCECON_OFFSEt   0x0180 /* De-bounce cycle control register */

/* GPIO port data I/O register offsets */

#define NUC_PORT_DATAIO_OFFSET(p)  (0x0200 + ((p)<< 6))
#  define NUC_PORTA_DATAIO_OFFSET  (0x0200 + ((NUC_GPIO_PORTA) << 6))
#  define NUC_PORTB_DATAIO_OFFSET  (0x0200 + ((NUC_GPIO_PORTB) << 6))
#  define NUC_PORTC_DATAIO_OFFSET  (0x0200 + ((NUC_GPIO_PORTC) << 6))
#  define NUC_PORTD_DATAIO_OFFSET  (0x0200 + ((NUC_GPIO_PORTD) << 6))
#  define NUC_PORTE_DATAIO_OFFSET  (0x0200 + ((NUC_GPIO_PORTE) << 6))

/* GPIO port pin data I/O register offsets */

#define NUC_PORT_PIN_OFFSET(n)     ((n) << 2)
#  define NUC_PORT_PIN0_OFFSET     (0 << 2)
#  define NUC_PORT_PIN1_OFFSET     (1 << 2)
#  define NUC_PORT_PIN2_OFFSET     (2 << 2)
#  define NUC_PORT_PIN3_OFFSET     (3 << 2)
#  define NUC_PORT_PIN4_OFFSET     (4 << 2)
#  define NUC_PORT_PIN5_OFFSET     (5 << 2)
#  define NUC_PORT_PIN6_OFFSET     (16 << 2)
#  define NUC_PORT_PIN7_OFFSET     (17 << 2)
#  define NUC_PORT_PIN8_OFFSET     (18 << 2)
#  define NUC_PORT_PIN9_OFFSET     (19 << 2)
#  define NUC_PORT_PIN10_OFFSET    (10 << 2)
#  define NUC_PORT_PIN11_OFFSET    (11 << 2)
#  define NUC_PORT_PIN12_OFFSET    (12 << 2)
#  define NUC_PORT_PIN13_OFFSET    (13 << 2)
#  define NUC_PORT_PIN14_OFFSET    (14 << 2)
#  define NUC_PORT_PIN15_OFFSET    (15 << 2)

#define NUC_PORT_PDIO_OFFSET(p,n)  (NUC_PORT_DATAIO_OFFSET(p)+NUC_PORT_PIN_OFFSET(n))

/* GPIO PA Pin Data Input/Output */

#define NUC_PORTA_PDIO_OFFSET(n)   (NUC_PORTA_DATAIO_OFFSET+NUC_PORT_PIN_OFFSET(n))
#  define NUC_PA1_PDIO_OFFSET      (NUC_PORTA_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PA2_PDIO_OFFSET      (NUC_PORTA_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PA3_PDIO_OFFSET      (NUC_PORTA_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PA4_PDIO_OFFSET      (NUC_PORTA_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PA5_PDIO_OFFSET      (NUC_PORTA_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PA6_PDIO_OFFSET      (NUC_PORTA_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PA7_PDIO_OFFSET      (NUC_PORTA_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PA8_PDIO_OFFSET      (NUC_PORTA_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PA9_PDIO_OFFSET      (NUC_PORTA_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PA10_PDIO_OFFSET     (NUC_PORTA_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PA11_PDIO_OFFSET     (NUC_PORTA_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PA12_PDIO_OFFSET     (NUC_PORTA_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PA13_PDIO_OFFSET     (NUC_PORTA_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PA14_PDIO_OFFSET     (NUC_PORTA_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PA15_PDIO_OFFSET     (NUC_PORTA_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)

/* GPIO PB Pin Data Input/Output */

#define NUC_PORTB_PDIO_OFFSET(n)   (NUC_PORTB_DATAIO_OFFSET+NUC_PORT_PIN_OFFSET(n))
#  define NUC_PB1_PDIO_OFFSET      (NUC_PORTB_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PB2_PDIO_OFFSET      (NUC_PORTB_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PB3_PDIO_OFFSET      (NUC_PORTB_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PB4_PDIO_OFFSET      (NUC_PORTB_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PB5_PDIO_OFFSET      (NUC_PORTB_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PB6_PDIO_OFFSET      (NUC_PORTB_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PB7_PDIO_OFFSET      (NUC_PORTB_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PB8_PDIO_OFFSET      (NUC_PORTB_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PB9_PDIO_OFFSET      (NUC_PORTB_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PB10_PDIO_OFFSET     (NUC_PORTB_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PB11_PDIO_OFFSET     (NUC_PORTB_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PB12_PDIO_OFFSET     (NUC_PORTB_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PB13_PDIO_OFFSET     (NUC_PORTB_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PB14_PDIO_OFFSET     (NUC_PORTB_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PB15_PDIO_OFFSET     (NUC_PORTB_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)

/* GPIO PC Pin Data Input/Output */

#define NUC_PORTC_PDIO_OFFSET(n)   (NUC_PORTC_DATAIO_OFFSET+NUC_PORT_PIN_OFFSET(n))
#  define NUC_PC1_PDIO_OFFSET      (NUC_PORTC_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PC2_PDIO_OFFSET      (NUC_PORTC_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PC3_PDIO_OFFSET      (NUC_PORTC_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PC4_PDIO_OFFSET      (NUC_PORTC_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PC5_PDIO_OFFSET      (NUC_PORTC_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PC6_PDIO_OFFSET      (NUC_PORTC_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PC7_PDIO_OFFSET      (NUC_PORTC_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PC8_PDIO_OFFSET      (NUC_PORTC_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PC9_PDIO_OFFSET      (NUC_PORTC_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PC10_PDIO_OFFSET     (NUC_PORTC_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PC11_PDIO_OFFSET     (NUC_PORTC_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PC12_PDIO_OFFSET     (NUC_PORTC_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PC13_PDIO_OFFSET     (NUC_PORTC_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PC14_PDIO_OFFSET     (NUC_PORTC_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PC15_PDIO_OFFSET     (NUC_PORTC_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)

/* GPIO PD Pin Data Input/Output */

#define NUC_PORTD_PDIO_OFFSET(n)   (NUC_PORTD_DATAIO_OFFSET+NUC_PORT_PIN_OFFSET(n))
#  define NUC_PD1_PDIO_OFFSET      (NUC_PORTD_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PD2_PDIO_OFFSET      (NUC_PORTD_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PD3_PDIO_OFFSET      (NUC_PORTD_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PD4_PDIO_OFFSET      (NUC_PORTD_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PD5_PDIO_OFFSET      (NUC_PORTD_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PD6_PDIO_OFFSET      (NUC_PORTD_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PD7_PDIO_OFFSET      (NUC_PORTD_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PD8_PDIO_OFFSET      (NUC_PORTD_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PD9_PDIO_OFFSET      (NUC_PORTD_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PD10_PDIO_OFFSET     (NUC_PORTD_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PD11_PDIO_OFFSET     (NUC_PORTD_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PD12_PDIO_OFFSET     (NUC_PORTD_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PD13_PDIO_OFFSET     (NUC_PORTD_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PD14_PDIO_OFFSET     (NUC_PORTD_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)
#  define NUC_PD15_PDIO_OFFSET     (NUC_PORTD_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)

/* GPIO PE Pin Data Input/Output */

#define NUC_PORTE_PDIO_OFFSET(n)   (NUC_PORTE_DATAIO_OFFSET+NUC_PORT_PIN_OFFSET(n))
#  define NUC_PE5_PDIO_OFFSET      (NUC_PORTE_DATAIO_OFFSET+NUC_PORT_PIN0_OFFSET)

/* Register addresses *******************************************************/

/* GPIO control registers */

#define NUC_GPIO_CTRL_BASE(n)      (NUC_GPIO_BASE+NUC_GPIO_CTRL_OFFSET(n))
#define NUC_GPIOA_CTRL_BASE        (NUC_GPIO_BASE+NUC_GPIOA_CTRL_OFFSET)
#define NUC_GPIOB_CTRL_BASE        (NUC_GPIO_BASE+NUC_GPIOB_CTRL_OFFSET)
#define NUC_GPIOC_CTRL_BASE        (NUC_GPIO_BASE+NUC_GPIOC_CTRL_OFFSET)
#define NUC_GPIOD_CTRL_BASE        (NUC_GPIO_BASE+NUC_GPIOD_CTRL_OFFSET)
#define NUC_GPIOE_CTRL_BASE        (NUC_GPIO_BASE+NUC_GPIOE_CTRL_OFFSET)

/* GPIO port A control registers */

#define NUC_GPIOA_PMD             (NUC_GPIO_BASE+NUC_GPIOA_PMD_OFFSET)
#define NUC_GPIOA_OFFD            (NUC_GPIO_BASE+NUC_GPIOA_OFFD_OFFSET)
#define NUC_GPIOA_DOUT            (NUC_GPIO_BASE+NUC_GPIOA_DOUT_OFFSET)
#define NUC_GPIOA_DMASK           (NUC_GPIO_BASE+NUC_GPIOA_DMASK_OFFSET)
#define NUC_GPIOA_PIN             (NUC_GPIO_BASE+NUC_GPIOA_PIN_OFFSET)
#define NUC_GPIOA_DBEN            (NUC_GPIO_BASE+NUC_GPIOA_DBEN_OFFSET)
#define NUC_GPIOA_IMD             (NUC_GPIO_BASE+NUC_GPIOA_IMD_OFFSET)
#define NUC_GPIOA_IEN             (NUC_GPIO_BASE+NUC_GPIOA_IEN_OFFSET)
#define NUC_GPIOA_ISRC            (NUC_GPIO_BASE+NUC_GPIOA_ISRC_OFFSET)

/* GPIO port B control registers */

#define NUC_GPIOB_PMD              (NUC_GPIO_BASE+NUC_GPIOB_PMD_OFFSET)
#define NUC_GPIOB_OFFD             (NUC_GPIO_BASE+NUC_GPIOB_OFFD_OFFSET)
#define NUC_GPIOB_DOUT             (NUC_GPIO_BASE+NUC_GPIOB_DOUT_OFFSET)
#define NUC_GPIOB_DMASK            (NUC_GPIO_BASE+NUC_GPIOB_DMASK_OFFSET)
#define NUC_GPIOB_PIN              (NUC_GPIO_BASE+NUC_GPIOB_PIN_OFFSET)
#define NUC_GPIOB_DBEN             (NUC_GPIO_BASE+NUC_GPIOB_DBEN_OFFSET)
#define NUC_GPIOB_IMD              (NUC_GPIO_BASE+NUC_GPIOB_IMD_OFFSET)
#define NUC_GPIOB_IEN              (NUC_GPIO_BASE+NUC_GPIOB_IEN_OFFSET)
#define NUC_GPIOB_ISRC             (NUC_GPIO_BASE+NUC_GPIOB_ISRC_OFFSET)

/* GPIO port C control registers */

#define NUC_GPIOC_PMD              (NUC_GPIO_BASE+NUC_GPIOC_PMD_OFFSET)
#define NUC_GPIOC_OFFD             (NUC_GPIO_BASE+NUC_GPIOC_OFFD_OFFSET)
#define NUC_GPIOC_DOUT             (NUC_GPIO_BASE+NUC_GPIOC_DOUT_OFFSET)
#define NUC_GPIOC_DMASK            (NUC_GPIO_BASE+NUC_GPIOC_DMASK_OFFSET)
#define NUC_GPIOC_PIN              (NUC_GPIO_BASE+NUC_GPIOC_PIN_OFFSET)
#define NUC_GPIOC_DBEN             (NUC_GPIO_BASE+NUC_GPIOC_DBEN_OFFSET)
#define NUC_GPIOC_IMD              (NUC_GPIO_BASE+NUC_GPIOC_IMD_OFFSET)
#define NUC_GPIOC_IEN              (NUC_GPIO_BASE+NUC_GPIOC_IEN_OFFSET)
#define NUC_GPIOC_ISRC             (NUC_GPIO_BASE+NUC_GPIOC_ISRC_OFFSET)

/* GPIO port D control registers */

#define NUC_GPIOD_PMD              (NUC_GPIO_BASE+NUC_GPIOD_PMD_OFFSET)
#define NUC_GPIOD_OFFD             (NUC_GPIO_BASE+NUC_GPIOD_OFFD_OFFSET)
#define NUC_GPIOD_DOUT             (NUC_GPIO_BASE+NUC_GPIOD_DOUT_OFFSET)
#define NUC_GPIOD_DMASK            (NUC_GPIO_BASE+NUC_GPIOD_DMASK_OFFSET)
#define NUC_GPIOD_PIN              (NUC_GPIO_BASE+NUC_GPIOD_PIN_OFFSET)
#define NUC_GPIOD_DBEN             (NUC_GPIO_BASE+NUC_GPIOD_DBEN_OFFSET)
#define NUC_GPIOD_IMD              (NUC_GPIO_BASE+NUC_GPIOD_IMD_OFFSET)
#define NUC_GPIOD_IEN              (NUC_GPIO_BASE+NUC_GPIOD_IEN_OFFSET)
#define NUC_GPIOD_ISRC             (NUC_GPIO_BASE+NUC_GPIOD_ISRC_OFFSET)

/* GPIO port E control registers */

#define NUC_GPIOE_PMD              (NUC_GPIO_BASE+NUC_GPIOE_PMD_OFFSET)
#define NUC_GPIOE_OFFD             (NUC_GPIO_BASE+NUC_GPIOE_OFFD_OFFSET)
#define NUC_GPIOE_DOUT             (NUC_GPIO_BASE+NUC_GPIOE_DOUT_OFFSET)
#define NUC_GPIOE_DMASK            (NUC_GPIO_BASE+NUC_GPIOE_DMASK_OFFSET)
#define NUC_GPIOE_PIN              (NUC_GPIO_BASE+NUC_GPIOE_PIN_OFFSET)
#define NUC_GPIOE_DBEN             (NUC_GPIO_BASE+NUC_GPIOE_DBEN_OFFSET)
#define NUC_GPIOE_IMD              (NUC_GPIO_BASE+NUC_GPIOE_IMD_OFFSET)
#define NUC_GPIOE_IEN              (NUC_GPIO_BASE+NUC_GPIOE_IEN_OFFSET)
#define NUC_GPIOE_ISRC             (NUC_GPIO_BASE+NUC_GPIOE_ISRC_OFFSET)

/* Debounce control registers */

#define NUC_GPIO_DBNCECON          (NUC_GPIO_BASE+NUC_GPIO_DBNCECON_OFFSET)

/* GPIO port data I/O register offsets */

#define NUC_PORT_DATAIO_BASE(p)    (NUC_GPIO_BASE+NUC_PORT_DATAIO_OFFSET(p))
#  define NUC_PORTA_DATAIO_BASE    (NUC_GPIO_BASE+NUC_PORTA_DATAIO_OFFSET)
#  define NUC_PORTB_DATAIO_BASE    (NUC_GPIO_BASE+NUC_PORTB_DATAIO_OFFSET)
#  define NUC_PORTC_DATAIO_BASE    (NUC_GPIO_BASE+NUC_PORTC_DATAIO_OFFSET)
#  define NUC_PORTD_DATAIO_BASE    (NUC_GPIO_BASE+NUC_PORTD_DATAIO_OFFSET)
#  define NUC_PORTE_DATAIO_BASE    (NUC_GPIO_BASE+NUC_PORTE_DATAIO_OFFSET)

#define NUC_PORT_PDIO(p,n)         (NUC_GPIO_BASE+NUC_PORT_PDIO_OFFSET(p,n))

/* GPIO PA Pin Data Input/Output */

#define NUC_PORTA_PDIO(n)          (NUC_GPIO_BASE+NUC_PORTA_PDIO_OFFSET(n))
#  define NUC_PA1_PDIO             (NUC_GPIO_BASE+NUC_PA1_PDIO_OFFSET)
#  define NUC_PA2_PDIO             (NUC_GPIO_BASE+NUC_PA2_PDIO_OFFSET)
#  define NUC_PA3_PDIO             (NUC_GPIO_BASE+NUC_PA3_PDIO_OFFSET)
#  define NUC_PA4_PDIO             (NUC_GPIO_BASE+NUC_PA4_PDIO_OFFSET)
#  define NUC_PA5_PDIO             (NUC_GPIO_BASE+NUC_PA5_PDIO_OFFSET)
#  define NUC_PA6_PDIO             (NUC_GPIO_BASE+NUC_PA6_PDIO_OFFSET)
#  define NUC_PA7_PDIO             (NUC_GPIO_BASE+NUC_PA7_PDIO_OFFSET)
#  define NUC_PA8_PDIO             (NUC_GPIO_BASE+NUC_PA8_PDIO_OFFSET)
#  define NUC_PA9_PDIO             (NUC_GPIO_BASE+NUC_PA9_PDIO_OFFSET)
#  define NUC_PA10_PDIO            (NUC_GPIO_BASE+NUC_PA10_PDIO_OFFSET)
#  define NUC_PA11_PDIO            (NUC_GPIO_BASE+NUC_PA11_PDIO_OFFSET)
#  define NUC_PA12_PDIO            (NUC_GPIO_BASE+NUC_PA12_PDIO_OFFSET)
#  define NUC_PA13_PDIO            (NUC_GPIO_BASE+NUC_PA13_PDIO_OFFSET)
#  define NUC_PA14_PDIO            (NUC_GPIO_BASE+NUC_PA14_PDIO_OFFSET)
#  define NUC_PA15_PDIO            (NUC_GPIO_BASE+NUC_PA15_PDIO_OFFSET)

/* GPIO PB Pin Data Input/Output */

#define NUC_PORTB_PDIO(n)          (NUC_GPIO_BASE+NUC_PORTB_PDIO_OFFSET(n))
#  define NUC_PB1_PDIO             (NUC_GPIO_BASE+NUC_PB1_PDIO_OFFSET)
#  define NUC_PB2_PDIO             (NUC_GPIO_BASE+NUC_PB2_PDIO_OFFSET)
#  define NUC_PB3_PDIO             (NUC_GPIO_BASE+NUC_PB3_PDIO_OFFSET)
#  define NUC_PB4_PDIO             (NUC_GPIO_BASE+NUC_PB4_PDIO_OFFSET)
#  define NUC_PB5_PDIO             (NUC_GPIO_BASE+NUC_PB5_PDIO_OFFSET)
#  define NUC_PB6_PDIO             (NUC_GPIO_BASE+NUC_PB6_PDIO_OFFSET)
#  define NUC_PB7_PDIO             (NUC_GPIO_BASE+NUC_PB7_PDIO_OFFSET)
#  define NUC_PB8_PDIO             (NUC_GPIO_BASE+NUC_PB8_PDIO_OFFSET)
#  define NUC_PB9_PDIO             (NUC_GPIO_BASE+NUC_PB9_PDIO_OFFSET)
#  define NUC_PB10_PDIO            (NUC_GPIO_BASE+NUC_PB10_PDIO_OFFSET)
#  define NUC_PB11_PDIO            (NUC_GPIO_BASE+NUC_PB11_PDIO_OFFSET)
#  define NUC_PB12_PDIO            (NUC_GPIO_BASE+NUC_PB12_PDIO_OFFSET)
#  define NUC_PB13_PDIO            (NUC_GPIO_BASE+NUC_PB13_PDIO_OFFSET)
#  define NUC_PB14_PDIO            (NUC_GPIO_BASE+NUC_PB14_PDIO_OFFSET)
#  define NUC_PB15_PDIO            (NUC_GPIO_BASE+NUC_PB15_PDIO_OFFSET)

/* GPIO PC Pin Data Input/Output */

#define NUC_PORTC_PDIO(n)          (NUC_GPIO_BASE+NUC_PORTC_PDIO_OFFSET(n))
#  define NUC_PC1_PDIO             (NUC_GPIO_BASE+NUC_PC1_PDIO_OFFSET)
#  define NUC_PC2_PDIO             (NUC_GPIO_BASE+NUC_PC2_PDIO_OFFSET)
#  define NUC_PC3_PDIO             (NUC_GPIO_BASE+NUC_PC3_PDIO_OFFSET)
#  define NUC_PC4_PDIO             (NUC_GPIO_BASE+NUC_PC4_PDIO_OFFSET)
#  define NUC_PC5_PDIO             (NUC_GPIO_BASE+NUC_PC5_PDIO_OFFSET)
#  define NUC_PC6_PDIO             (NUC_GPIO_BASE+NUC_PC6_PDIO_OFFSET)
#  define NUC_PC7_PDIO             (NUC_GPIO_BASE+NUC_PC7_PDIO_OFFSET)
#  define NUC_PC8_PDIO             (NUC_GPIO_BASE+NUC_PC8_PDIO_OFFSET)
#  define NUC_PC9_PDIO             (NUC_GPIO_BASE+NUC_PC9_PDIO_OFFSET)
#  define NUC_PC10_PDIO            (NUC_GPIO_BASE+NUC_PC10_PDIO_OFFSET)
#  define NUC_PC11_PDIO            (NUC_GPIO_BASE+NUC_PC11_PDIO_OFFSET)
#  define NUC_PC12_PDIO            (NUC_GPIO_BASE+NUC_PC12_PDIO_OFFSET)
#  define NUC_PC13_PDIO            (NUC_GPIO_BASE+NUC_PC13_PDIO_OFFSET)
#  define NUC_PC14_PDIO            (NUC_GPIO_BASE+NUC_PC14_PDIO_OFFSET)
#  define NUC_PC15_PDIO            (NUC_GPIO_BASE+NUC_PC15_PDIO_OFFSET)

/* GPIO PD Pin Data Input/Output */

#define NUC_PORTD_PDIO(n)          (NUC_GPIO_BASE+NUC_PORTD_PDIO_OFFSET(n))
#  define NUC_PD1_PDIO             (NUC_GPIO_BASE+NUC_PD1_PDIO_OFFSET)
#  define NUC_PD2_PDIO             (NUC_GPIO_BASE+NUC_PD2_PDIO_OFFSET)
#  define NUC_PD3_PDIO             (NUC_GPIO_BASE+NUC_PD3_PDIO_OFFSET)
#  define NUC_PD4_PDIO             (NUC_GPIO_BASE+NUC_PD4_PDIO_OFFSET)
#  define NUC_PD5_PDIO             (NUC_GPIO_BASE+NUC_PD5_PDIO_OFFSET)
#  define NUC_PD6_PDIO             (NUC_GPIO_BASE+NUC_PD6_PDIO_OFFSET)
#  define NUC_PD7_PDIO             (NUC_GPIO_BASE+NUC_PD7_PDIO_OFFSET)
#  define NUC_PD8_PDIO             (NUC_GPIO_BASE+NUC_PD8_PDIO_OFFSET)
#  define NUC_PD9_PDIO             (NUC_GPIO_BASE+NUC_PD9_PDIO_OFFSET)
#  define NUC_PD10_PDIO            (NUC_GPIO_BASE+NUC_PD10_PDIO_OFFSET)
#  define NUC_PD11_PDIO            (NUC_GPIO_BASE+NUC_PD11_PDIO_OFFSET)
#  define NUC_PD12_PDIO            (NUC_GPIO_BASE+NUC_PD12_PDIO_OFFSET)
#  define NUC_PD13_PDIO            (NUC_GPIO_BASE+NUC_PD13_PDIO_OFFSET)
#  define NUC_PD14_PDIO            (NUC_GPIO_BASE+NUC_PD14_PDIO_OFFSET)
#  define NUC_PD15_PDIO            (NUC_GPIO_BASE+NUC_PD15_PDIO_OFFSET)

/* GPIO PE Pin Data Input/Output */

#define NUC_PORTE_PDIO(n)          (NUC_GPIO_BASE+NUC_PORTE_PDIO_OFFSET(n))
#  define NUC_PE5_PDIO             (NUC_GPIO_BASE+NUC_PE5_PDIO_OFFSET)

/* Register bit-field definitions *******************************************/

/* GPIO port pin I/O mode control */

#define GPIO_PMD_INPUT             0 /* Input */
#define GPIO_PMD_OUTPUT            1 /* Push-pull output */
#define GPIO_PMD_OPENDRAIN         2 /* Open drain output */
#define GPIO_PMD_BIDI              3 /* Quasi bi-directional */

#define GPIO_PMD_SHIFT(n)          ((n) << 1) /* Bits 2n-2n+1: GPIOx Pin[n] mode control */
#define GPIO_PMD_MASK(n)           (3 << GPIO_PMD_SHIFT(n))
#  define GPIO_PMD(n,v)            ((v) << GPIO_PMD_SHIFT(n))

#define GPIO_PMD0_SHIFT            (0)      /* Bits 0-1: GPIOx Pin0 mode control */
#define GPIO_PMD0_MASK             (3 << GPIO_PMD0_SHIFT)
#  define GPIO_PMD0(v)             ((v) << GPIO_PMD0_SHIFT)
#define GPIO_PMD1_SHIFT            (2)      /* Bits 2-3: GPIOx Pin1 mode control */
#define GPIO_PMD1_MASK             (3 << GPIO_PMD1_SHIFT)
#  define GPIO_PMD1(v)             ((v) << GPIO_PMD1_SHIFT)
#define GPIO_PMD2_SHIFT            (4)      /* Bits 4-5: GPIOx Pin2 mode control */
#define GPIO_PMD2_MASK             (3 << GPIO_PMD2_SHIFT)
#  define GPIO_PMD2(v)             ((v) << GPIO_PMD2_SHIFT)
#define GPIO_PMD3_SHIFT            (6)      /* Bits 6-7: GPIOx Pin3 mode control */
#define GPIO_PMD3_MASK             (3 << GPIO_PMD3_SHIFT)
#  define GPIO_PMD3(v)             ((v) << GPIO_PMD3_SHIFT)
#define GPIO_PMD4_SHIFT            (8)      /* Bits 8-9: GPIOx Pin4 mode control */
#define GPIO_PMD4_MASK             (3 << GPIO_PMD4_SHIFT)
#  define GPIO_PMD4(v)             ((v) << GPIO_PMD4_SHIFT)
#define GPIO_PMD5_SHIFT            (10)     /* Bits 10-11: GPIOx Pin5 mode control */
#define GPIO_PMD5_MASK             (3 << GPIO_PMD5_SHIFT)
#  define GPIO_PMD5(v)             ((v) << GPIO_PMD5_SHIFT)
#define GPIO_PMD6_SHIFT            (12)     /* Bits 12-13: GPIOx Pin6 mode control */
#define GPIO_PMD6_MASK             (3 << GPIO_PMD6_SHIFT)
#  define GPIO_PMD6(v)             ((v) << GPIO_PMD6_SHIFT)
#define GPIO_PMD7_SHIFT            (14)     /* Bits 14-15: GPIOx Pin7 mode control */
#define GPIO_PMD7_MASK             (3 << GPIO_PMD7_SHIFT)
#  define GPIO_PMD7(v)             ((v) << GPIO_PMD7_SHIFT)
#define GPIO_PMD8_SHIFT            (16)     /* Bits 16-17: GPIOx Pin8 mode control */
#define GPIO_PMD8_MASK             (3 << GPIO_PMD8_SHIFT)
#  define GPIO_PMD8(v)             ((v) << GPIO_PMD8_SHIFT)
#define GPIO_PMD9_SHIFT            (18)     /* Bits 18-19: GPIOx Pin9 mode control */
#define GPIO_PMD9_MASK             (3 << GPIO_PMD9_SHIFT)
#  define GPIO_PMD9(v)             ((v) << GPIO_PMD9_SHIFT)
#define GPIO_PMD10_SHIFT           (20)     /* Bits 20-21: GPIOx Pin0 mode control */
#define GPIO_PMD10_MASK            (3 << GPIO_PMD10_SHIFT)
#  define GPIO_PMD10(v)            ((v) << GPIO_PMD10_SHIFT)
#define GPIO_PMD11_SHIFT           (22)     /* Bits 22-23: GPIOx Pin1 mode control */
#define GPIO_PMD11_MASK            (3 << GPIO_PMD11_SHIFT)
#  define GPIO_PMD11(v)            ((v) << GPIO_PMD11_SHIFT)
#define GPIO_PMD12_SHIFT           (24)     /* Bits 24-25: GPIOx Pin2 mode control */
#define GPIO_PMD12_MASK            (3 << GPIO_PMD12_SHIFT)
#  define GPIO_PMD12(v)            ((v) << GPIO_PMD12_SHIFT)
#define GPIO_PMD13_SHIFT           (26)     /* Bits 26-27: GPIOx Pin3 mode control */
#define GPIO_PMD13_MASK            (3 << GPIO_PMD13_SHIFT)
#  define GPIO_PMD13(v)            ((v) << GPIO_PMD13_SHIFT)
#define GPIO_PMD14_SHIFT           (28)     /* Bits 28-29: GPIOx Pin4 mode control */
#define GPIO_PMD14_MASK            (3 << GPIO_PMD14_SHIFT)
#  define GPIO_PMD14(v)            ((v) << GPIO_PMD14_SHIFT)
#define GPIO_PMD15_SHIFT           (30)     /* Bits 30-31: GPIOx Pin5 mode control */
#define GPIO_PMD15_MASK            (3 << GPIO_PMD15_SHIFT)
#  define GPIO_PMD15(v)            ((v) << GPIO_PMD15_SHIFT)

/* GPIO port pin digital input path disable control */

#define GPIO_OFFD(n)               (1 << (n)) /* Bit n: GPIOx Pin[n] digital input path disable control */

/* GPIO port data output value */

#define GPIO_DOUT(n)               (1 << (n)) /* Bit n: GPIOx Pin[n] output value */

/* GPIO port data output write mask */

#define GPIO_DMASK(n)              (1 << (n)) /* Bit n: GPIOx Pin[n] data output write mask */

/* GPIO port pin value */

#define GPIO_PIN(n)                (1 << (n)) /* Bit n: GPIOx Pin[n] pin value */

/* GPIO port de-bounce enable */

#define GPIO_DBEN(n)               (1 << (n)) /* Bit n: GPIOx Pin[n] input signal de-bounce enable */

/* GPIO port interrupt mode control */

#define GPIO_IMD(n)                (1 << (n)) /* Bit n: GPIOx Pin[n] edge/level detection interrupt control */

/* GPIO port interrupt enable */

#define GPIO_IF_EN(n)              (1 << (n)) /* Bit n: GPIOx Pin[n] interrupt enable low/falling */

#define GPIO_IR_EN(n)              (1 << ((n)+16)) /* Bit n: GPIOx Pin[n] interrupt enable high/rising */

/* GPIO port interrupt source flag */

#define GPIO_ISRC(n)               (1 << (n)) /* Bit n: GPIOx Pin[n] interrupt source flag */

/* De-bounce cycle control register */

#define GPIO_DBNCECON_DBCLKSEL_SHIFT (0)     /* Bits 0-3: De-bounce cycling count selection */
#define GPIO_DBNCECON_DBCLKSEL_MASK  (15 << GPIO_DBNCECON_DBCLKSEL_SHIFT)
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
#define GPIO_DBNCECON_DBCLKSRC     (1 << 4)  /* Bit 4:  De-bounce counter clock source selection */
#define GPIO_DBNCECON_ICLK_ON      (1 << 5)  /* Bit 5:  Interrupt clock on mode */

/* GPIO port data I/O registers */

#define PORT_MASK                  (1) /* Bit 0: GPIOx Pin[n] data I/O */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_NUC1XX_HARDWARE_NUC_GPIO_H */
