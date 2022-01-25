/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/hardware/lpc17_40_gpio.h
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

#ifndef __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_GPIO_H
#define __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/lpc17_40_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

/* GPIO block register offsets **********************************************/

#define LPC17_40_FIO0_OFFSET                0x0000
#define LPC17_40_FIO1_OFFSET                0x0020
#define LPC17_40_FIO2_OFFSET                0x0040
#define LPC17_40_FIO3_OFFSET                0x0060
#define LPC17_40_FIO4_OFFSET                0x0080
#ifdef LPC178x_40xx
#  define LPC17_40_FIO5_OFFSET              0x00a0
#endif

#define LPC17_40_FIO_DIR_OFFSET             0x0000  /* Fast GPIO Port Direction control */
#define LPC17_40_FIO_MASK_OFFSET            0x0010  /* Fast Mask register for ports */
#define LPC17_40_FIO_PIN_OFFSET             0x0014  /* Fast Port Pin value registers */
#define LPC17_40_FIO_SET_OFFSET             0x0018  /* Fast Port Output Set registers */
#define LPC17_40_FIO_CLR_OFFSET             0x001c  /* Fast Port Output Clear register */

/* GPIO interrupt block register offsets ************************************/

#define LPC17_40_GPIOINT_OFFSET(n)          (0x10*(n) + 0x80)
#define LPC17_40_GPIOINT0_OFFSET            0x0080
#define LPC17_40_GPIOINT2_OFFSET            0x00a0

#define LPC17_40_GPIOINT_IOINTSTATUS_OFFSET 0x0000  /* GPIO overall Interrupt Status */
#define LPC17_40_GPIOINT_INTSTATR_OFFSET    0x0004  /* GPIO Interrupt Status Rising edge */
#define LPC17_40_GPIOINT_INTSTATF_OFFSET    0x0008  /* GPIO Interrupt Status Falling edge */
#define LPC17_40_GPIOINT_INTCLR_OFFSET      0x000c  /* GPIO Interrupt Clear */
#define LPC17_40_GPIOINT_INTENR_OFFSET      0x0010  /* GPIO Interrupt Enable Rising edge */
#define LPC17_40_GPIOINT_INTENF_OFFSET      0x0014  /* GPIO Interrupt Enable Falling edge */

/* Register addresses *******************************************************/

/* GPIO block register addresses ********************************************/

#define LPC17_40_FIO_BASE(n)                (LPC17_40_GPIO_BASE+LPC17_40_GPIOINT_OFFSET(n))
#define LPC17_40_FIO0_BASE                  (LPC17_40_GPIO_BASE+LPC17_40_FIO0_OFFSET)
#define LPC17_40_FIO1_BASE                  (LPC17_40_GPIO_BASE+LPC17_40_FIO1_OFFSET)
#define LPC17_40_FIO2_BASE                  (LPC17_40_GPIO_BASE+LPC17_40_FIO2_OFFSET)
#define LPC17_40_FIO3_BASE                  (LPC17_40_GPIO_BASE+LPC17_40_FIO3_OFFSET)
#define LPC17_40_FIO4_BASE                  (LPC17_40_GPIO_BASE+LPC17_40_FIO4_OFFSET)
#ifdef LPC178x_40xx
#  define LPC17_40_FIO5_BASE                (LPC17_40_GPIO_BASE+LPC17_40_FIO5_OFFSET)
#endif

#define LPC17_40_FIO_DIR(n)                 (LPC17_40_FIO_BASE(n)+LPC17_40_FIO_DIR_OFFSET)
#define LPC17_40_FIO_MASK(n)                (LPC17_40_FIO_BASE(n)+LPC17_40_FIO_MASK_OFFSET)
#define LPC17_40_FIO_PIN(n)                 (LPC17_40_FIO_BASE(n)+LPC17_40_FIO_PIN_OFFSET)
#define LPC17_40_FIO_SET(n)                 (LPC17_40_FIO_BASE(n)+LPC17_40_FIO_SET_OFFSET)
#define LPC17_40_FIO_CLR(n)                 (LPC17_40_FIO_BASE(n)+LPC17_40_FIO_CLR_OFFSET)

#define LPC17_40_FIO0_DIR                   (LPC17_40_FIO0_BASE+LPC17_40_FIO_DIR_OFFSET)
#define LPC17_40_FIO0_MASK                  (LPC17_40_FIO0_BASE+LPC17_40_FIO_MASK_OFFSET)
#define LPC17_40_FIO0_PIN                   (LPC17_40_FIO0_BASE+LPC17_40_FIO_PIN_OFFSET)
#define LPC17_40_FIO0_SET                   (LPC17_40_FIO0_BASE+LPC17_40_FIO_SET_OFFSET)
#define LPC17_40_FIO0_CLR                   (LPC17_40_FIO0_BASE+LPC17_40_FIO_CLR_OFFSET)

#define LPC17_40_FIO1_DIR                   (LPC17_40_FIO1_BASE+LPC17_40_FIO_DIR_OFFSET)
#define LPC17_40_FIO1_MASK                  (LPC17_40_FIO1_BASE+LPC17_40_FIO_MASK_OFFSET)
#define LPC17_40_FIO1_PIN                   (LPC17_40_FIO1_BASE+LPC17_40_FIO_PIN_OFFSET)
#define LPC17_40_FIO1_SET                   (LPC17_40_FIO1_BASE+LPC17_40_FIO_SET_OFFSET)
#define LPC17_40_FIO1_CLR                   (LPC17_40_FIO1_BASE+LPC17_40_FIO_CLR_OFFSET)

#define LPC17_40_FIO2_DIR                   (LPC17_40_FIO2_BASE+LPC17_40_FIO_DIR_OFFSET)
#define LPC17_40_FIO2_MASK                  (LPC17_40_FIO2_BASE+LPC17_40_FIO_MASK_OFFSET)
#define LPC17_40_FIO2_PIN                   (LPC17_40_FIO2_BASE+LPC17_40_FIO_PIN_OFFSET)
#define LPC17_40_FIO2_SET                   (LPC17_40_FIO2_BASE+LPC17_40_FIO_SET_OFFSET)
#define LPC17_40_FIO2_CLR                   (LPC17_40_FIO2_BASE+LPC17_40_FIO_CLR_OFFSET)

#define LPC17_40_FIO3_DIR                   (LPC17_40_FIO3_BASE+LPC17_40_FIO_DIR_OFFSET)
#define LPC17_40_FIO3_MASK                  (LPC17_40_FIO3_BASE+LPC17_40_FIO_MASK_OFFSET)
#define LPC17_40_FIO3_PIN                   (LPC17_40_FIO3_BASE+LPC17_40_FIO_PIN_OFFSET)
#define LPC17_40_FIO3_SET                   (LPC17_40_FIO3_BASE+LPC17_40_FIO_SET_OFFSET)
#define LPC17_40_FIO3_CLR                   (LPC17_40_FIO3_BASE+LPC17_40_FIO_CLR_OFFSET)

#define LPC17_40_FIO4_DIR                   (LPC17_40_FIO4_BASE+LPC17_40_FIO_DIR_OFFSET)
#define LPC17_40_FIO4_MASK                  (LPC17_40_FIO4_BASE+LPC17_40_FIO_MASK_OFFSET)
#define LPC17_40_FIO4_PIN                   (LPC17_40_FIO4_BASE+LPC17_40_FIO_PIN_OFFSET)
#define LPC17_40_FIO4_SET                   (LPC17_40_FIO4_BASE+LPC17_40_FIO_SET_OFFSET)
#define LPC17_40_FIO4_CLR                   (LPC17_40_FIO4_BASE+LPC17_40_FIO_CLR_OFFSET)

#ifdef LPC178x_40xx
#  define LPC17_40_FIO5_DIR                 (LPC17_40_FIO5_BASE+LPC17_40_FIO_DIR_OFFSET)
#  define LPC17_40_FIO5_MASK                (LPC17_40_FIO5_BASE+LPC17_40_FIO_MASK_OFFSET)
#  define LPC17_40_FIO5_PIN                 (LPC17_40_FIO5_BASE+LPC17_40_FIO_PIN_OFFSET)
#  define LPC17_40_FIO5_SET                 (LPC17_40_FIO5_BASE+LPC17_40_FIO_SET_OFFSET)
#  define LPC17_40_FIO5_CLR                 (LPC17_40_FIO5_BASE+LPC17_40_FIO_CLR_OFFSET)
#endif

/* GPIO interrupt block register addresses **********************************/

#define LPC17_40_GPIOINTn_BASE(n)           (LPC17_40_GPIOINT_BASE+LPC17_40_GPIOINT_OFFSET(n))
#define LPC17_40_GPIOINT0_BASE              (LPC17_40_GPIOINT_BASE+LPC17_40_GPIOINT0_OFFSET)
#define LPC17_40_GPIOINT2_BASE              (LPC17_40_GPIOINT_BASE+LPC17_40_GPIOINT2_OFFSET)

#define LPC17_40_GPIOINT_IOINTSTATUS        (LPC17_40_GPIOINT0_BASE+LPC17_40_GPIOINT_IOINTSTATUS_OFFSET)

#define LPC17_40_GPIOINT_INTSTATR(n)        (LPC17_40_GPIOINTn_BASE(n)+LPC17_40_GPIOINT_INTSTATR_OFFSET)
#define LPC17_40_GPIOINT_INTSTATF(n)        (LPC17_40_GPIOINTn_BASE(n)+LPC17_40_GPIOINT_INTSTATF_OFFSET)
#define LPC17_40_GPIOINT_INTCLR(n)          (LPC17_40_GPIOINTn_BASE(n)+LPC17_40_GPIOINT_INTCLR_OFFSET)
#define LPC17_40_GPIOINT_INTENR(n)          (LPC17_40_GPIOINTn_BASE(n)+LPC17_40_GPIOINT_INTENR_OFFSET)
#define LPC17_40_GPIOINT_INTENF(n)          (LPC17_40_GPIOINTn_BASE(n)+LPC17_40_GPIOINT_INTENF_OFFSET)

/* Pins P0.0-31 */

#define LPC17_40_GPIOINT0_INTSTATR          (LPC17_40_GPIOINT0_BASE+LPC17_40_GPIOINT_INTSTATR_OFFSET)
#define LPC17_40_GPIOINT0_INTSTATF          (LPC17_40_GPIOINT0_BASE+LPC17_40_GPIOINT_INTSTATF_OFFSET)
#define LPC17_40_GPIOINT0_INTCLR            (LPC17_40_GPIOINT0_BASE+LPC17_40_GPIOINT_INTCLR_OFFSET)
#define LPC17_40_GPIOINT0_INTENR            (LPC17_40_GPIOINT0_BASE+LPC17_40_GPIOINT_INTENR_OFFSET)
#define LPC17_40_GPIOINT0_INTENF            (LPC17_40_GPIOINT0_BASE+LPC17_40_GPIOINT_INTENF_OFFSET)

/* Pins P2.0-31 */

#define LPC17_40_GPIOINT2_INTSTATR          (LPC17_40_GPIOINT2_BASE+LPC17_40_GPIOINT_INTSTATR_OFFSET)
#define LPC17_40_GPIOINT2_INTSTATF          (LPC17_40_GPIOINT2_BASE+LPC17_40_GPIOINT_INTSTATF_OFFSET)
#define LPC17_40_GPIOINT2_INTCLR            (LPC17_40_GPIOINT2_BASE+LPC17_40_GPIOINT_INTCLR_OFFSET)
#define LPC17_40_GPIOINT2_INTENR            (LPC17_40_GPIOINT2_BASE+LPC17_40_GPIOINT_INTENR_OFFSET)
#define LPC17_40_GPIOINT2_INTENF            (LPC17_40_GPIOINT2_BASE+LPC17_40_GPIOINT_INTENF_OFFSET)

/* Register bit definitions *************************************************/

/* GPIO block register bit definitions **************************************/

/* Fast GPIO Port Direction control registers (FIODIR) */

/* Fast Mask register for ports (FIOMASK) */

/* Fast Port Pin value registers using FIOMASK (FIOPIN) */

/* Fast Port Output Set registers using FIOMASK (FIOSET) */

/* Fast Port Output Clear register using FIOMASK (FIOCLR) */

#define FIO(n)                              (1 << (n)) /* n=0,1,..31 */

/* GPIO interrupt block register bit definitions ****************************/

/* GPIO overall Interrupt Status (IOINTSTATUS) */
#define GPIOINT_IOINTSTATUS_P0INT           (1 << 0)  /* Bit 0: Port 0 GPIO interrupt pending */
                                                      /* Bit 1: Reserved */
#define GPIOINT_IOINTSTATUS_P2INT           (1 << 2)  /* Bit 2: Port 2 GPIO interrupt pending */
                                                      /* Bits 3-31: Reserved */

/* GPIO Interrupt Status for Rising edge (INTSTATR)
 * GPIO Interrupt Status for Falling edge (INTSTATF)
 * GPIO Interrupt Clear (INTCLR)
 * GPIO Interrupt Enable for Rising edge (INTENR)
 * GPIO Interrupt Enable for Falling edge (INTENF)
 */

#define GPIOINT(n)                          (1 << (n)) /* n=0,1,..31 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_GPIO_H */
