/****************************************************************************
 * arch/arm/src/ra4/hardware/ra_gpio.h
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

#ifndef __ARCH_ARM_SRC_RA_HARDWARE_RA_GPIO_H
#define __ARCH_ARM_SRC_RA_HARDWARE_RA_GPIO_H

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

#define R_PORT_PCNTR1_OFFSET            0x0000  /* Port Control Register 1 (32-bits) */
#define R_PORT_PODR_OFFSET              0x0000  /* Pmn Output Data (16-bits) */
#define R_PORT_PDR_OFFSET               0x0002  /* Pmn Direction (16-bits) */
#define R_PORT_PCNTR2_OFFSET            0x0004  /* Port Control Register 2 (32-bits) */
#define R_PORT_EIDR_OFFSET              0x0004  /* Port Event Input Data (16-bits) */
#define R_PORT_PIDR_OFFSET              0x0006  /* Pmn State (16-bits) */
#define R_PORT_PCNTR3_OFFSET            0x0008  /* Port Control Register 3 (32-bits) */
#define R_PORT_PORR_OFFSET              0x0008  /* Pmn Output Reset (16-bits) */
#define R_PORT_POSR_OFFSET              0x000a  /* Pmn Output Set (16-bits) */
#define R_PORT_PCNTR4_OFFSET            0x000c  /* Port Control Register 3 (32-bits) */
#define R_PORT_EORR_OFFSET              0x000c  /* Pmn Event Output Set (16-bits) */
#define R_PORT_EOSR_OFFSET              0x000e  /* Pmn Output Reset (16-bits) */

#define R_PORT_OFFSET                   0x0020  /* Relative Port Offset */

/* Register Addresses *******************************************************/

#define PORT0 (0)
#define PORT1 (1)
#define PORT2 (2)
#define PORT3 (3)
#define PORT4 (4)
#define PORT5 (5)
#define PORT6 (6)
#define PORT7 (7)
#define PORT8 (8)
#define PORT9 (9)

#define PIN0 (0)
#define PIN1 (1)
#define PIN2 (2)
#define PIN3 (3)
#define PIN4 (4)
#define PIN5 (5)
#define PIN6 (6)
#define PIN7 (7)
#define PIN8 (8)
#define PIN9 (9)
#define PIN10 (10)
#define PIN11 (11)
#define PIN12 (12)
#define PIN13 (13)
#define PIN14 (14)
#define PIN15 (15)

/* Relative PORT Registers */

#  define R_PORT_PCNTR1(port)          (R_PORT0_BASE + (port)*R_PORT_OFFSET + R_PORT_PCNTR1_OFFSET)
#  define R_PORT_PODR(port)            (R_PORT0_BASE + (port)*R_PORT_OFFSET + R_PORT_PODR_OFFSET)
#  define R_PORT_PDR(port)             (R_PORT0_BASE + (port)*R_PORT_OFFSET + R_PORT_PDR_OFFSET)
#  define R_PORT_PCNTR2(port)          (R_PORT0_BASE + (port)*R_PORT_OFFSET + R_PORT_PCNTR2_OFFSET)
#  define R_PORT_EIDR(port)            (R_PORT0_BASE + (port)*R_PORT_OFFSET + R_PORT_EIDR_OFFSET)
#  define R_PORT_PIDR(port)            (R_PORT0_BASE + (port)*R_PORT_OFFSET + R_PORT_PIDR_OFFSET)
#  define R_PORT_PCNTR3(port)          (R_PORT0_BASE + (port)*R_PORT_OFFSET + R_PORT_PCNTR3_OFFSET)
#  define R_PORT_PORR(port)            (R_PORT0_BASE + (port)*R_PORT_OFFSET + R_PORT_PORR_OFFSET)
#  define R_PORT_POSR(port)            (R_PORT0_BASE + (port)*R_PORT_OFFSET + R_PORT_POSR_OFFSET)
#  define R_PORT_PCNTR4(port)          (R_PORT0_BASE + (port)*R_PORT_OFFSET + R_PORT_PCNTR4_OFFSET)
#  define R_PORT_EORR(port)            (R_PORT0_BASE + (port)*R_PORT_OFFSET + R_PORT_EORR_OFFSET)
#  define R_PORT_EOSR(port)            (R_PORT0_BASE + (port)*R_PORT_OFFSET + R_PORT_EOSR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Port Control Register 1 (32-bits) */

#define R_PORT_PCNTR1_PODR_SHIFT              (16) /* 10000: Pmn Output Data */
#define R_PORT_PCNTR1_PODR_MASK               (0xffff)
#define R_PORT_PCNTR1_PDR_SHIFT               (0) /* 01: Pmn Direction */
#define R_PORT_PCNTR1_PDR_MASK                (0xffff)

/* Pmn Output Data (16-bits) */

#define R_PORT_PODR_PODR_SHIFT                (0) /* 01: Pmn Output Data */
#define R_PORT_PODR_PODR_MASK                 (0xffff)

/* Pmn Direction (16-bits) */

#define R_PORT_PDR_PDR_SHIFT                  (0) /* 01: Pmn Direction */
#define R_PORT_PDR_PDR_MASK                   (0xffff)

/* Port Control Register 2 (32-bits) */

#define R_PORT_PCNTR2_EIDR_SHIFT              (16) /* 10000: Pmn Event Input Data */
#define R_PORT_PCNTR2_EIDR_MASK               (0xffff)
#define R_PORT_PCNTR2_PIDR_SHIFT              (0) /* 01: Pmn Input Data */
#define R_PORT_PCNTR2_PIDR_MASK               (0xffff)

/* Port Event Input Data (16-bits) */

#define R_PORT_EIDR_EIDR_SHIFT                (0) /* 01: Pmn Event Input Data */
#define R_PORT_EIDR_EIDR_MASK                 (0xffff)

/* Pmn State (16-bits) */

#define R_PORT_PIDR_PIDR_SHIFT                (0) /* 01: Pmn Input Data */
#define R_PORT_PIDR_PIDR_MASK                 (0xffff)

/* Port Control Register 3 (32-bits) */

#define R_PORT_PCNTR3_PORR_SHIFT              (16) /* 10000: Pmn Output Reset */
#define R_PORT_PCNTR3_PORR_MASK               (0xffff)
#define R_PORT_PCNTR3_POSR_SHIFT              (0) /* 01: Pmn Output Set */
#define R_PORT_PCNTR3_POSR_MASK               (0xffff)

/* Pmn Output Reset (16-bits) */

#define R_PORT_PORR_PORR_SHIFT                (0) /* 01: Pmn Output Reset */
#define R_PORT_PORR_PORR_MASK                 (0xffff)

/* Pmn Output Set (16-bits) */

#define R_PORT_POSR_POSR_SHIFT                (0) /* 01: Pmn Output Set */
#define R_PORT_POSR_POSR_MASK                 (0xffff)

/* Port Control Register 3 (32-bits) */

#define R_PORT_PCNTR4_EORR_SHIFT              (16) /* 10000: Pmn Event Output Reset */
#define R_PORT_PCNTR4_EORR_MASK               (0xffff)
#define R_PORT_PCNTR4_EOSR_SHIFT              (0) /* 01: Pmn Event Output Set */
#define R_PORT_PCNTR4_EOSR_MASK               (0xffff)

/* Pmn Event Output Set (16-bits) */

#define R_PORT_EORR_EORR_SHIFT                (0) /* 01: Pmn Event Output Reset */
#define R_PORT_EORR_EORR_MASK                 (0xffff)

/* Pmn Output Reset (16-bits) */

#define R_PORT_EOSR_EOSR_SHIFT                (0) /* 01: Pmn Event Output Set */
#define R_PORT_EOSR_EOSR_MASK                 (0xffff)

/* PMISC - Miscellaneous Port Control Register */

#define R_PMISC_PWPR_OFFSET               0x0003
#define R_PMISC_PWPR                      (R_PMISC_BASE + R_PMISC_PWPR_OFFSET)
#define R_PMISC_PWPR_B0WI                 (1 <<  7) /* 80: PFSWE Bit Write Disable */
#define R_PMISC_PWPR_PFSWE                (1 <<  6) /* 40: PFS Register Write Enable */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_RA_HARDWARE_RA_GPIO_H */
