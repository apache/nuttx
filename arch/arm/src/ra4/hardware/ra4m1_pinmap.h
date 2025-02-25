/****************************************************************************
 * arch/arm/src/ra4/hardware/ra4m1_pinmap.h
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

#ifndef __ARCH_ARM_SRC_RA_HARDWARE_RA4M1_PINMAP_H
#define __ARCH_ARM_SRC_RA_HARDWARE_RA4M1_PINMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/ra_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/
#define R_PFS_PSEL_PORT_OFFSET          0x40
#define R_PFS_PSEL_PIN_OFFSET           0x04
#define R_PMISC_PWPR_OFFSET             0x0003

/* Register Addresses *******************************************************/

#define R_PFS(port,pin)                 (R_PFS_BASE + (port)*R_PFS_PSEL_PORT_OFFSET + (pin)*R_PFS_PSEL_PIN_OFFSET)
#define R_PMISC_PWPR                    (R_PMISC_BASE + R_PMISC_PWPR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* PFS - Pmn Pin Function Control Register */

#define R_PFS_PSEL_SHIFT          (24) /* 1000000: Port Function Select These bits select the peripheral function. For individual pin functions, see the MPC table */
#define R_PFS_PSEL_MASK           (0x1f)
#define R_PFS_PMR                 (1 << 16) /* Bit 16: Port Mode Control */
#define R_PFS_ASEL                (1 << 15) /* Bit 15: Analog Input enable */
#define R_PFS_ISEL                (1 << 14) /* Bit 14: IRQ input enable */
#define R_PFS_EOR                 (1 << 13) /* Bit 13: Event on Rising */
#define R_PFS_EOF                 (1 << 12) /* Bit 12: Event on Falling */
#define R_PFS_DSCR1               (1 << 11) /* Bit 11: Port Drive Capability 1 */
#define R_PFS_DSCR                (1 << 10) /* Bit 10: Port Drive Capability */
#define R_PFS_NCODR               (1 <<  6) /* Bit 6: N-Channel Open Drain Control */
#define R_PFS_PCR                 (1 <<  4) /* Bit 4: Pull-up Control */
#define R_PFS_PDR                 (1 <<  2) /* Bit 2: Port Direction */
#define R_PFS_PIDR                (1 <<  1) /* Bit 1: Port Input Data */
#define R_PFS_PODR                (1 <<  0) /* Bit 0: Port Output Data */

/* PMISC - Miscellaneous Port Control Register */

#define R_PMISC_PWPR_B0WI                 (1 <<  7) /* 80: PFSWE Bit Write Disable */
#define R_PMISC_PWPR_PFSWE                (1 <<  6) /* 40: PFS Register Write Enable */

#define PFS_PSEL_HIZ                 (0x00 << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_AGT                 (0x01 << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_GPT                 (0x02 << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_AGT1                (0x03 << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_SCI                 (0x04 << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_SCI1                (0x05 << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_SPI                 (0x06 << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_IIC                 (0x07 << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_KINT                (0x08 << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_CLKOUT_ACMPLP_RTC   (0x09 << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_CAC_ADC14           (0x0a << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_CTSU                (0x0c << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_SLCDC               (0x0d << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_CAN                 (0x10 << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_SSIE                (0x12 << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_USBFS               (0x13 << R_PFS_PSEL_SHIFT)

/* SCI Alternative */

#define GPIO_RXD0_MISO0_SCL0_1              (gpio_pinset_t){ PORT1,PIN0, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_TXD0_MOSI0_SDA0_1              (gpio_pinset_t){ PORT1,PIN1, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_RXD0_MISO0_SCL0_2              (gpio_pinset_t){ PORT2,PIN6, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_TXD0_MOSI0_SDA0_2              (gpio_pinset_t){ PORT2,PIN5, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_RXD0_MISO0_SCL0_3              (gpio_pinset_t){ PORT4,PIN10, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_TXD0_MOSI0_SDA0_3              (gpio_pinset_t){ PORT14,PIN11, (PFS_PSEL_SCI | R_PFS_PMR)}

#define GPIO_RXD1_MISO1_SCL1_1              (gpio_pinset_t){ PORT2,PIN12, (PFS_PSEL_SCI1 | R_PFS_PMR)}
#define GPIO_TXD1_MOSI1_SDA1_1              (gpio_pinset_t){ PORT2,PIN13, (PFS_PSEL_SCI1 | R_PFS_PMR)}
#define GPIO_RXD1_MISO1_SCL1_2              (gpio_pinset_t){ PORT4,PIN2, (PFS_PSEL_SCI1 | R_PFS_PMR)}
#define GPIO_TXD1_MOSI1_SDA1_2              (gpio_pinset_t){ PORT4,PIN1, (PFS_PSEL_SCI1 | R_PFS_PMR)}
#define GPIO_RXD1_MISO1_SCL1_3              (gpio_pinset_t){ PORT5,PIN2, (PFS_PSEL_SCI1 | R_PFS_PMR)}
#define GPIO_TXD1_MOSI1_SDA1_3              (gpio_pinset_t){ PORT5,PIN1, (PFS_PSEL_SCI1 | R_PFS_PMR)}

#define GPIO_RXD2_MISO2_SCL2_1              (gpio_pinset_t){ PORT3,PIN1, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_TXD2_MOSI2_SDA2_1              (gpio_pinset_t){ PORT3,PIN2, (PFS_PSEL_SCI | R_PFS_PMR)}

#define GPIO_RXD9_MISO9_SCL9_1              (gpio_pinset_t){ PORT1,PIN10, (PFS_PSEL_SCI1 | R_PFS_PMR)}
#define GPIO_TXD9_MOSI9_SDA9_1              (gpio_pinset_t){ PORT1,PIN9, (PFS_PSEL_SCI1 | R_PFS_PMR)}
#define GPIO_RXD9_MISO9_SCL9_2              (gpio_pinset_t){ PORT2,PIN2, (PFS_PSEL_SCI1 | R_PFS_PMR)}
#define GPIO_TXD9_MOSI9_SDA9_2              (gpio_pinset_t){ PORT2,PIN3, (PFS_PSEL_SCI1 | R_PFS_PMR)}
#define GPIO_RXD9_MISO9_SCL9_3              (gpio_pinset_t){ PORT4,PIN8, (PFS_PSEL_SCI1 | R_PFS_PMR)}
#define GPIO_TXD9_MOSI9_SDA9_3              (gpio_pinset_t){ PORT4,PIN9, (PFS_PSEL_SCI1 | R_PFS_PMR)}
#define GPIO_RXD9_MISO9_SCL9_4              (gpio_pinset_t){ PORT6,PIN1, (PFS_PSEL_SCI1 | R_PFS_PMR)}
#define GPIO_TXD9_MOSI9_SDA9_4              (gpio_pinset_t){ PORT6,PIN2, (PFS_PSEL_SCI1 | R_PFS_PMR)}

/* GPIO Configuration */

#define GPIO_OUPUT               R_PFS_PDR
#define GPIO_INPUT               ~(R_PFS_PDR | 0xFFFFFFFF)

#define GPIO_LOW_DRIVE          ~(R_PFS_DSCR | 0xFFFFFFFF)
#define GPIO_MIDDLE_DRIVE       R_PFS_DSCR

#define GPIO_OUTPUT_HIGH         R_PFS_PODR
#define GPIO_OUTPUT_LOW         ~(R_PFS_PODR | 0xFFFFFFFF)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_RA_HARDWARE_RA4M1_PINMAP_H */
