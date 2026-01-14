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
#define PFS_PSEL_GPT1                (0x03 << R_PFS_PSEL_SHIFT)
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

/* GTP Alternative */

/* GPT Channel 0 (GTIOC0A / GTIOC0B) */

#define GPIO_GPT_GTIOC0A_P107               (gpio_pinset_t){ PORT1,PIN7, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P107 [1]
#define GPIO_GPT_GTIOC0A_P213               (gpio_pinset_t){ PORT2,PIN13, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P213 [4]
#define GPIO_GPT_GTIOC0A_P415               (gpio_pinset_t){ PORT4,PIN15, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P415 [7]
#define GPIO_GPT_GTIOC0B_P106               (gpio_pinset_t){ PORT1,PIN6, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P106 [1]
#define GPIO_GPT_GTIOC0B_P212               (gpio_pinset_t){ PORT2,PIN12, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P212 [4]
#define GPIO_GPT_GTIOC0B_P414               (gpio_pinset_t){ PORT4,PIN14, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P414 [7]

/* GPT Channel 1 (GTIOC1A / GTIOC1B) */

#define GPIO_GPT_GTIOC1A_P105               (gpio_pinset_t){ PORT1,PIN5, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P105 [1]
#define GPIO_GPT_GTIOC1A_P109               (gpio_pinset_t){ PORT1,PIN9, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P109 [6]
#define GPIO_GPT_GTIOC1A_P405               (gpio_pinset_t){ PORT4,PIN5, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P405 [6]
#define GPIO_GPT_GTIOC1B_P104               (gpio_pinset_t){ PORT1,PIN4, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P104 [1]
#define GPIO_GPT_GTIOC1B_P406               (gpio_pinset_t){ PORT4,PIN6, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P406 [6]

/* GPT Channel 2 (GTIOC2A / GTIOC2B) */

#define GPIO_GPT_GTIOC2A_P103               (gpio_pinset_t){ PORT1,PIN3, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P103 [1]
#define GPIO_GPT_GTIOC2A_P113               (gpio_pinset_t){ PORT1,PIN13, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P113 [2]
#define GPIO_GPT_GTIOC2B_P102               (gpio_pinset_t){ PORT1,PIN2, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P102 [1]
#define GPIO_GPT_GTIOC2B_P114               (gpio_pinset_t){ PORT1,PIN14, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P114 [2]

/* GPT Channel 3 (GTIOC3A / GTIOC3B) */

#define GPIO_GPT_GTIOC3A_P403               (gpio_pinset_t){ PORT4,PIN3, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P403 [6]
#define GPIO_GPT_GTIOC3A_P111               (gpio_pinset_t){ PORT1,PIN11, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P111 [2]
#define GPIO_GPT_GTIOC3B_P404               (gpio_pinset_t){ PORT4,PIN4, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P404 [6]
#define GPIO_GPT_GTIOC3B_P112               (gpio_pinset_t){ PORT1,PIN12, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P112 [2]

/* GPT Channel 4 (GTIOC4A / GTIOC4B) */

#define GPIO_GPT_GTIOC4A_P205               (gpio_pinset_t){ PORT2,PIN5, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P205 [3]
#define GPIO_GPT_GTIOC4A_P302               (gpio_pinset_t){ PORT3,PIN2, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P302 [5]
#define GPIO_GPT_GTIOC4A_P115               (gpio_pinset_t){ PORT1,PIN15, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P115 [2]
#define GPIO_GPT_GTIOC4B_P204               (gpio_pinset_t){ PORT2,PIN4, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P204 [3]
#define GPIO_GPT_GTIOC4B_P301               (gpio_pinset_t){ PORT3,PIN1, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P301 [5]
#define GPIO_GPT_GTIOC4B_P608               (gpio_pinset_t){ PORT6,PIN8, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P608 [8]

/* GPT Channel 5 (GTIOC5A / GTIOC5B) */

#define GPIO_GPT_GTIOC5A_P101               (gpio_pinset_t){ PORT1,PIN1, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P101 [1]
#define GPIO_GPT_GTIOC5A_P203               (gpio_pinset_t){ PORT2,PIN3, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P203 [3]
#define GPIO_GPT_GTIOC5A_P409               (gpio_pinset_t){ PORT4,PIN9, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P409 [7]
#define GPIO_GPT_GTIOC5A_P609               (gpio_pinset_t){ PORT6,PIN9, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P609 [8]
#define GPIO_GPT_GTIOC5B_P100               (gpio_pinset_t){ PORT1,PIN0, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P100 [1]
#define GPIO_GPT_GTIOC5B_P202               (gpio_pinset_t){ PORT2,PIN2, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P202 [3]
#define GPIO_GPT_GTIOC5B_P408               (gpio_pinset_t){ PORT4,PIN8, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P408 [7]
#define GPIO_GPT_GTIOC5B_P610               (gpio_pinset_t){ PORT6,PIN10, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P610 [8]

/* GPT Channel 6 (GTIOC6A / GTIOC6B) */

#define GPIO_GPT_GTIOC6A_P400               (gpio_pinset_t){ PORT4,PIN0, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P400 [6]
#define GPIO_GPT_GTIOC6A_P411               (gpio_pinset_t){ PORT4,PIN11, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P411 [7]
#define GPIO_GPT_GTIOC6A_P601               (gpio_pinset_t){ PORT6,PIN1, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P601 [8]
#define GPIO_GPT_GTIOC6B_P401               (gpio_pinset_t){ PORT4,PIN1, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P401 [6]
#define GPIO_GPT_GTIOC6B_P410               (gpio_pinset_t){ PORT4,PIN10, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P410 [7]
#define GPIO_GPT_GTIOC6B_P600               (gpio_pinset_t){ PORT6,PIN0, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P600 [8]

/* GPT Channel 7 (GTIOC7A / GTIOC7B) */

#define GPIO_GPT_GTIOC7A_P304               (gpio_pinset_t){ PORT3,PIN4, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P304 [5]
#define GPIO_GPT_GTIOC7A_P603               (gpio_pinset_t){ PORT6,PIN3, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P603 [8]
#define GPIO_GPT_GTIOC7B_P303               (gpio_pinset_t){ PORT3,PIN3, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P303 [5]
#define GPIO_GPT_GTIOC7B_P602               (gpio_pinset_t){ PORT6,PIN2, (PFS_PSEL_GPT1 | R_PFS_PMR)} // P602 [8]

/* GPIO Configuration */

#define GPIO_OUPUT               R_PFS_PDR
#define GPIO_INPUT               ~(R_PFS_PDR | 0xFFFFFFFF)

#define GPIO_LOW_DRIVE          ~(R_PFS_DSCR | 0xFFFFFFFF)
#define GPIO_MIDDLE_DRIVE       R_PFS_DSCR

#define GPIO_OUTPUT_HIGH         R_PFS_PODR
#define GPIO_OUTPUT_LOW         ~(R_PFS_PODR | 0xFFFFFFFF)

#define GPIO_PIN_INVALID         (gpio_pinset_t){ PORT_INVALID, PIN_INVALID, 0xFFFFFFFF }

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
