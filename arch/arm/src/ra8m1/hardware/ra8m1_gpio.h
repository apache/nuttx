/****************************************************************************
 * arch/arm/src/ra8m1/hardware/ra8m1_gpio.h
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

#ifndef __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_GPIO_H
#define __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/ra8m1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define R_GPIO_PCNTR1_OFFSET                0x0000  /* Port Control Register 1 (32-bits) */
#define R_GPIO_PDR_OFFSET                   0x0000  /* Data direction register (16-bits) */
#define R_GPIO_PODR_OFFSET                  0x0002  /* Output data register (16-bits) */
#define R_GPIO_PCNTR2_OFFSET                0x0004  /* Port Control Register 2 (32-bits) */
#define R_GPIO_PIDR_OFFSET                  0x0004  /* Input data register (16-bits) */
#define R_GPIO_EIDR_OFFSET                  0x0006  /* Port Event Input Data register (16-bits) */
#define R_GPIO_PCNTR3_OFFSET                0x0008  /* Port Control Register 3 (32-bits) */
#define R_GPIO_POSR_OFFSET                  0x0008  /* Output set register (16-bits) */
#define R_GPIO_PORR_OFFSET                  0x000a  /* Output reset register (16-bits) */
#define R_GPIO_PCNTR4_OFFSET                0x000c  /* Port Control Register 4 (32-bits) */
#define R_GPIO_EOSR_OFFSET                  0x000c  /* Event output reset register (16-bits) */
#define R_GPIO_EORR_OFFSET                  0x000e  /* Event output set register (16-bits) */

/* Register Addresses *******************************************************/

/* PORT0 Registers (subset: 9 of 12 registers) */

#define R_PORT0_PCNTR1                     (R_PORT0_BASE + R_GPIO_PCNTR1_OFFSET)
#define R_PORT0_PDR                        (R_PORT0_BASE + R_GPIO_PDR_OFFSET)
#define R_PORT0_PODR                       (R_PORT0_BASE + R_GPIO_PODR_OFFSET)
#define R_PORT0_PCNTR2                     (R_PORT0_BASE + R_GPIO_PCNTR2_OFFSET)
#define R_PORT0_PIDR                       (R_PORT0_BASE + R_GPIO_PIDR_OFFSET)
#define R_PORT0_EIDR                       (R_PORT0_BASE + R_GPIO_EIDR_OFFSET)
#define R_PORT0_PCNTR3                     (R_PORT0_BASE + R_GPIO_PCNTR3_OFFSET)
#define R_PORT0_POSR                       (R_PORT0_BASE + R_GPIO_POSR_OFFSET)
#define R_PORT0_PORR                       (R_PORT0_BASE + R_GPIO_PORR_OFFSET)

/* PORT1 Registers */

#define R_PORT1_PCNTR1                     (R_PORT1_BASE + R_GPIO_PCNTR1_OFFSET)
#define R_PORT1_PDR                        (R_PORT1_BASE + R_GPIO_PDR_OFFSET)
#define R_PORT1_PODR                       (R_PORT1_BASE + R_GPIO_PODR_OFFSET)
#define R_PORT1_PCNTR2                     (R_PORT1_BASE + R_GPIO_PCNTR2_OFFSET)
#define R_PORT1_PIDR                       (R_PORT1_BASE + R_GPIO_PIDR_OFFSET)
#define R_PORT1_EIDR                       (R_PORT1_BASE + R_GPIO_EIDR_OFFSET)
#define R_PORT1_PCNTR3                     (R_PORT1_BASE + R_GPIO_PCNTR3_OFFSET)
#define R_PORT1_POSR                       (R_PORT1_BASE + R_GPIO_POSR_OFFSET)
#define R_PORT1_PORR                       (R_PORT1_BASE + R_GPIO_PORR_OFFSET)
#define R_PORT1_PCNTR4                     (R_PORT1_BASE + R_GPIO_PCNTR4_OFFSET)
#define R_PORT1_EOSR                       (R_PORT1_BASE + R_GPIO_EOSR_OFFSET)
#define R_PORT1_EORR                       (R_PORT1_BASE + R_GPIO_EORR_OFFSET)

/* PORT2 Registers */

#define R_PORT2_PCNTR1                     (R_PORT2_BASE + R_GPIO_PCNTR1_OFFSET)
#define R_PORT2_PDR                        (R_PORT2_BASE + R_GPIO_PDR_OFFSET)
#define R_PORT2_PODR                       (R_PORT2_BASE + R_GPIO_PODR_OFFSET)
#define R_PORT2_PCNTR2                     (R_PORT2_BASE + R_GPIO_PCNTR2_OFFSET)
#define R_PORT2_PIDR                       (R_PORT2_BASE + R_GPIO_PIDR_OFFSET)
#define R_PORT2_EIDR                       (R_PORT2_BASE + R_GPIO_EIDR_OFFSET)
#define R_PORT2_PCNTR3                     (R_PORT2_BASE + R_GPIO_PCNTR3_OFFSET)
#define R_PORT2_POSR                       (R_PORT2_BASE + R_GPIO_POSR_OFFSET)
#define R_PORT2_PORR                       (R_PORT2_BASE + R_GPIO_PORR_OFFSET)
#define R_PORT2_PCNTR4                     (R_PORT2_BASE + R_GPIO_PCNTR4_OFFSET)
#define R_PORT2_EOSR                       (R_PORT2_BASE + R_GPIO_EOSR_OFFSET)
#define R_PORT2_EORR                       (R_PORT2_BASE + R_GPIO_EORR_OFFSET)

/* PORT3 Registers */

#define R_PORT3_PCNTR1                     (R_PORT3_BASE + R_GPIO_PCNTR1_OFFSET)
#define R_PORT3_PDR                        (R_PORT3_BASE + R_GPIO_PDR_OFFSET)
#define R_PORT3_PODR                       (R_PORT3_BASE + R_GPIO_PODR_OFFSET)
#define R_PORT3_PCNTR2                     (R_PORT3_BASE + R_GPIO_PCNTR2_OFFSET)
#define R_PORT3_PIDR                       (R_PORT3_BASE + R_GPIO_PIDR_OFFSET)
#define R_PORT3_EIDR                       (R_PORT3_BASE + R_GPIO_EIDR_OFFSET)
#define R_PORT3_PCNTR3                     (R_PORT3_BASE + R_GPIO_PCNTR3_OFFSET)
#define R_PORT3_POSR                       (R_PORT3_BASE + R_GPIO_POSR_OFFSET)
#define R_PORT3_PORR                       (R_PORT3_BASE + R_GPIO_PORR_OFFSET)
#define R_PORT3_PCNTR4                     (R_PORT3_BASE + R_GPIO_PCNTR4_OFFSET)
#define R_PORT3_EOSR                       (R_PORT3_BASE + R_GPIO_EOSR_OFFSET)
#define R_PORT3_EORR                       (R_PORT3_BASE + R_GPIO_EORR_OFFSET)

/* PORT4 Registers */

#define R_PORT4_PCNTR1                     (R_PORT4_BASE + R_GPIO_PCNTR1_OFFSET)
#define R_PORT4_PDR                        (R_PORT4_BASE + R_GPIO_PDR_OFFSET)
#define R_PORT4_PODR                       (R_PORT4_BASE + R_GPIO_PODR_OFFSET)
#define R_PORT4_PCNTR2                     (R_PORT4_BASE + R_GPIO_PCNTR2_OFFSET)
#define R_PORT4_PIDR                       (R_PORT4_BASE + R_GPIO_PIDR_OFFSET)
#define R_PORT4_EIDR                       (R_PORT4_BASE + R_GPIO_EIDR_OFFSET)
#define R_PORT4_PCNTR3                     (R_PORT4_BASE + R_GPIO_PCNTR3_OFFSET)
#define R_PORT4_POSR                       (R_PORT4_BASE + R_GPIO_POSR_OFFSET)
#define R_PORT4_PORR                       (R_PORT4_BASE + R_GPIO_PORR_OFFSET)
#define R_PORT4_PCNTR4                     (R_PORT4_BASE + R_GPIO_PCNTR4_OFFSET)
#define R_PORT4_EOSR                       (R_PORT4_BASE + R_GPIO_EOSR_OFFSET)
#define R_PORT4_EORR                       (R_PORT4_BASE + R_GPIO_EORR_OFFSET)

/* PORT5 Registers (subset: 9 of 12 registers) */

#define R_PORT5_PCNTR1                     (R_PORT5_BASE + R_GPIO_PCNTR1_OFFSET)
#define R_PORT5_PDR                        (R_PORT5_BASE + R_GPIO_PDR_OFFSET)
#define R_PORT5_PODR                       (R_PORT5_BASE + R_GPIO_PODR_OFFSET)
#define R_PORT5_PCNTR2                     (R_PORT5_BASE + R_GPIO_PCNTR2_OFFSET)
#define R_PORT5_PIDR                       (R_PORT5_BASE + R_GPIO_PIDR_OFFSET)
#define R_PORT5_EIDR                       (R_PORT5_BASE + R_GPIO_EIDR_OFFSET)
#define R_PORT5_PCNTR3                     (R_PORT5_BASE + R_GPIO_PCNTR3_OFFSET)
#define R_PORT5_POSR                       (R_PORT5_BASE + R_GPIO_POSR_OFFSET)
#define R_PORT5_PORR                       (R_PORT5_BASE + R_GPIO_PORR_OFFSET)

/* PORT6 Registers (subset: 9 of 12 registers) */

#define R_PORT6_PCNTR1                     (R_PORT6_BASE + R_GPIO_PCNTR1_OFFSET)
#define R_PORT6_PDR                        (R_PORT6_BASE + R_GPIO_PDR_OFFSET)
#define R_PORT6_PODR                       (R_PORT6_BASE + R_GPIO_PODR_OFFSET)
#define R_PORT6_PCNTR2                     (R_PORT6_BASE + R_GPIO_PCNTR2_OFFSET)
#define R_PORT6_PIDR                       (R_PORT6_BASE + R_GPIO_PIDR_OFFSET)
#define R_PORT6_EIDR                       (R_PORT6_BASE + R_GPIO_EIDR_OFFSET)
#define R_PORT6_PCNTR3                     (R_PORT6_BASE + R_GPIO_PCNTR3_OFFSET)
#define R_PORT6_POSR                       (R_PORT6_BASE + R_GPIO_POSR_OFFSET)
#define R_PORT6_PORR                       (R_PORT6_BASE + R_GPIO_PORR_OFFSET)

/* PORT7 Registers (subset: 9 of 12 registers) */

#define R_PORT7_PCNTR1                     (R_PORT7_BASE + R_GPIO_PCNTR1_OFFSET)
#define R_PORT7_PDR                        (R_PORT7_BASE + R_GPIO_PDR_OFFSET)
#define R_PORT7_PODR                       (R_PORT7_BASE + R_GPIO_PODR_OFFSET)
#define R_PORT7_PCNTR2                     (R_PORT7_BASE + R_GPIO_PCNTR2_OFFSET)
#define R_PORT7_PIDR                       (R_PORT7_BASE + R_GPIO_PIDR_OFFSET)
#define R_PORT7_EIDR                       (R_PORT7_BASE + R_GPIO_EIDR_OFFSET)
#define R_PORT7_PCNTR3                     (R_PORT7_BASE + R_GPIO_PCNTR3_OFFSET)
#define R_PORT7_POSR                       (R_PORT7_BASE + R_GPIO_POSR_OFFSET)
#define R_PORT7_PORR                       (R_PORT7_BASE + R_GPIO_PORR_OFFSET)

/* PORT8 Registers (subset: 9 of 12 registers) */

#define R_PORT8_PCNTR1                     (R_PORT8_BASE + R_GPIO_PCNTR1_OFFSET)
#define R_PORT8_PDR                        (R_PORT8_BASE + R_GPIO_PDR_OFFSET)
#define R_PORT8_PODR                       (R_PORT8_BASE + R_GPIO_PODR_OFFSET)
#define R_PORT8_PCNTR2                     (R_PORT8_BASE + R_GPIO_PCNTR2_OFFSET)
#define R_PORT8_PIDR                       (R_PORT8_BASE + R_GPIO_PIDR_OFFSET)
#define R_PORT8_EIDR                       (R_PORT8_BASE + R_GPIO_EIDR_OFFSET)
#define R_PORT8_PCNTR3                     (R_PORT8_BASE + R_GPIO_PCNTR3_OFFSET)
#define R_PORT8_POSR                       (R_PORT8_BASE + R_GPIO_POSR_OFFSET)
#define R_PORT8_PORR                       (R_PORT8_BASE + R_GPIO_PORR_OFFSET)

/* PORT9 Registers (subset: 9 of 12 registers) */

#define R_PORT9_PCNTR1                     (R_PORT9_BASE + R_GPIO_PCNTR1_OFFSET)
#define R_PORT9_PDR                        (R_PORT9_BASE + R_GPIO_PDR_OFFSET)
#define R_PORT9_PODR                       (R_PORT9_BASE + R_GPIO_PODR_OFFSET)
#define R_PORT9_PCNTR2                     (R_PORT9_BASE + R_GPIO_PCNTR2_OFFSET)
#define R_PORT9_PIDR                       (R_PORT9_BASE + R_GPIO_PIDR_OFFSET)
#define R_PORT9_EIDR                       (R_PORT9_BASE + R_GPIO_EIDR_OFFSET)
#define R_PORT9_PCNTR3                     (R_PORT9_BASE + R_GPIO_PCNTR3_OFFSET)
#define R_PORT9_POSR                       (R_PORT9_BASE + R_GPIO_POSR_OFFSET)
#define R_PORT9_PORR                       (R_PORT9_BASE + R_GPIO_PORR_OFFSET)

/* PORTA Registers (subset: 9 of 12 registers) */

#define R_PORTA_PCNTR1                     (R_PORTA_BASE + R_GPIO_PCNTR1_OFFSET)
#define R_PORTA_PDR                        (R_PORTA_BASE + R_GPIO_PDR_OFFSET)
#define R_PORTA_PODR                       (R_PORTA_BASE + R_GPIO_PODR_OFFSET)
#define R_PORTA_PCNTR2                     (R_PORTA_BASE + R_GPIO_PCNTR2_OFFSET)
#define R_PORTA_PIDR                       (R_PORTA_BASE + R_GPIO_PIDR_OFFSET)
#define R_PORTA_EIDR                       (R_PORTA_BASE + R_GPIO_EIDR_OFFSET)
#define R_PORTA_PCNTR3                     (R_PORTA_BASE + R_GPIO_PCNTR3_OFFSET)
#define R_PORTA_POSR                       (R_PORTA_BASE + R_GPIO_POSR_OFFSET)
#define R_PORTA_PORR                       (R_PORTA_BASE + R_GPIO_PORR_OFFSET)

/* PORTB Registers (subset: 9 of 12 registers) */

#define R_PORTB_PCNTR1                     (R_PORTB_BASE + R_GPIO_PCNTR1_OFFSET)
#define R_PORTB_PDR                        (R_PORTB_BASE + R_GPIO_PDR_OFFSET)
#define R_PORTB_PODR                       (R_PORTB_BASE + R_GPIO_PODR_OFFSET)
#define R_PORTB_PCNTR2                     (R_PORTB_BASE + R_GPIO_PCNTR2_OFFSET)
#define R_PORTB_PIDR                       (R_PORTB_BASE + R_GPIO_PIDR_OFFSET)
#define R_PORTB_EIDR                       (R_PORTB_BASE + R_GPIO_EIDR_OFFSET)
#define R_PORTB_PCNTR3                     (R_PORTB_BASE + R_GPIO_PCNTR3_OFFSET)
#define R_PORTB_POSR                       (R_PORTB_BASE + R_GPIO_POSR_OFFSET)
#define R_PORTB_PORR                       (R_PORTB_BASE + R_GPIO_PORR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Port Control Register 1 (32-bits) ****************************************/

#define R_GPIO_PCNTR1_PDR_SHIFT (0)
#define R_GPIO_PCNTR1_PDR_MASK (0xffff)
#  define R_GPIO_PCNTR1_PDR_V0 (0 << R_GPIO_PCNTR1_PDR_SHIFT)             /* Input (functions as an input pin) */
#  define R_GPIO_PCNTR1_PDR_V1 (1 << R_GPIO_PCNTR1_PDR_SHIFT)             /* Output (functions as an output pin). */
#define R_GPIO_PCNTR1_PODR_SHIFT (16)
#define R_GPIO_PCNTR1_PODR_MASK (0xffff)
#  define R_GPIO_PCNTR1_PODR_LOW_OUTPUT (0 << R_GPIO_PCNTR1_PODR_SHIFT)   /* Low output */
#  define R_GPIO_PCNTR1_PODR_HIGH_OUTPUT (1 << R_GPIO_PCNTR1_PODR_SHIFT)  /* High output. */

/* Data direction register (16-bits) ****************************************/

#define R_GPIO_PDR_PDR_SHIFT (0)
#define R_GPIO_PDR_PDR_MASK (0xffff)
#  define R_GPIO_PDR_PDR_V0 (0 << R_GPIO_PDR_PDR_SHIFT)  /* Input (functions as an input pin) */
#  define R_GPIO_PDR_PDR_V1 (1 << R_GPIO_PDR_PDR_SHIFT)  /* Output (functions as an output pin). */

/* Output data register (16-bits) *******************************************/

#define R_GPIO_PODR_PODR_SHIFT (0)
#define R_GPIO_PODR_PODR_MASK (0xffff)
#  define R_GPIO_PODR_PODR_LOW_OUTPUT (0 << R_GPIO_PODR_PODR_SHIFT)   /* Low output */
#  define R_GPIO_PODR_PODR_HIGH_OUTPUT (1 << R_GPIO_PODR_PODR_SHIFT)  /* High output. */

/* Port Control Register 2 (32-bits) ****************************************/

#define R_GPIO_PCNTR2_PIDR_SHIFT (0)
#define R_GPIO_PCNTR2_PIDR_MASK (0xffff)
#  define R_GPIO_PCNTR2_PIDR_LOW_INPUT (0 << R_GPIO_PCNTR2_PIDR_SHIFT)   /* Low input */
#  define R_GPIO_PCNTR2_PIDR_HIGH_INPUT (1 << R_GPIO_PCNTR2_PIDR_SHIFT)  /* High input. */
#define R_GPIO_PCNTR2_EIDR_SHIFT (16)
#define R_GPIO_PCNTR2_EIDR_MASK (0xffff)
#  define R_GPIO_PCNTR2_EIDR_LOW_INPUT (0 << R_GPIO_PCNTR2_EIDR_SHIFT)   /* Low input */
#  define R_GPIO_PCNTR2_EIDR_HIGH_INPUT (1 << R_GPIO_PCNTR2_EIDR_SHIFT)  /* High input. */

/* Input data register (16-bits) ********************************************/

#define R_GPIO_PIDR_PIDR_SHIFT (0)
#define R_GPIO_PIDR_PIDR_MASK (0xffff)
#  define R_GPIO_PIDR_PIDR_LOW_LEVEL (0 << R_GPIO_PIDR_PIDR_SHIFT)   /* Low level */
#  define R_GPIO_PIDR_PIDR_HIGH_LEVEL (1 << R_GPIO_PIDR_PIDR_SHIFT)  /* high level. */

/* Port Event Input Data register (16-bits) *********************************/

#define R_GPIO_EIDR_EIDR_SHIFT (0)
#define R_GPIO_EIDR_EIDR_MASK (0xffff)
#  define R_GPIO_EIDR_EIDR_LOW_INPUT (0 << R_GPIO_EIDR_EIDR_SHIFT)   /* Low input */
#  define R_GPIO_EIDR_EIDR_HIGH_INPUT (1 << R_GPIO_EIDR_EIDR_SHIFT)  /* High input. */

/* Port Control Register 3 (32-bits) ****************************************/

#define R_GPIO_PCNTR3_POSR_SHIFT (0)
#define R_GPIO_PCNTR3_POSR_MASK (0xffff)
#  define R_GPIO_PCNTR3_POSR_NO_AFFECT_TO_OUTPUT (0 << R_GPIO_PCNTR3_POSR_SHIFT)  /* No affect to output */
#  define R_GPIO_PCNTR3_POSR_HIGH_OUTPUT (1 << R_GPIO_PCNTR3_POSR_SHIFT)          /* High output. */
#define R_GPIO_PCNTR3_PORR_SHIFT (16)
#define R_GPIO_PCNTR3_PORR_MASK (0xffff)
#  define R_GPIO_PCNTR3_PORR_NO_AFFECT_TO_OUTPUT (0 << R_GPIO_PCNTR3_PORR_SHIFT)  /* No affect to output */
#  define R_GPIO_PCNTR3_PORR_LOW_OUTPUT (1 << R_GPIO_PCNTR3_PORR_SHIFT)           /* Low output. */

/* Output set register (16-bits) ********************************************/

#define R_GPIO_POSR_POSR_SHIFT (0)
#define R_GPIO_POSR_POSR_MASK (0xffff)
#  define R_GPIO_POSR_POSR_NO_AFFECT_TO_OUTPUT (0 << R_GPIO_POSR_POSR_SHIFT)  /* No affect to output */
#  define R_GPIO_POSR_POSR_HIGH_OUTPUT (1 << R_GPIO_POSR_POSR_SHIFT)          /* High output. */

/* Output reset register (16-bits) ******************************************/

#define R_GPIO_PORR_PORR_SHIFT (0)
#define R_GPIO_PORR_PORR_MASK (0xffff)
#  define R_GPIO_PORR_PORR_NO_AFFECT_TO_OUTPUT (0 << R_GPIO_PORR_PORR_SHIFT)  /* No affect to output */
#  define R_GPIO_PORR_PORR_LOW_OUTPUT (1 << R_GPIO_PORR_PORR_SHIFT)           /* Low output. */

/* Port Control Register 4 (32-bits) ****************************************/

#define R_GPIO_PCNTR4_EOSR_SHIFT (0)
#define R_GPIO_PCNTR4_EOSR_MASK (0xffff)
#  define R_GPIO_PCNTR4_EOSR_NO_AFFECT_TO_OUTPUT (0 << R_GPIO_PCNTR4_EOSR_SHIFT)  /* No affect to output */
#  define R_GPIO_PCNTR4_EOSR_HIGH_OUTPUT (1 << R_GPIO_PCNTR4_EOSR_SHIFT)          /* High output. */
#define R_GPIO_PCNTR4_EORR_SHIFT (16)
#define R_GPIO_PCNTR4_EORR_MASK (0xffff)
#  define R_GPIO_PCNTR4_EORR_NO_AFFECT_TO_OUTPUT (0 << R_GPIO_PCNTR4_EORR_SHIFT)  /* No affect to output */
#  define R_GPIO_PCNTR4_EORR_LOW_OUTPUT (1 << R_GPIO_PCNTR4_EORR_SHIFT)           /* Low output */

/* Event output reset register (16-bits) ************************************/

#define R_GPIO_EOSR_EOSR_SHIFT (0)
#define R_GPIO_EOSR_EOSR_MASK (0xffff)
#  define R_GPIO_EOSR_EOSR_NO_AFFECT_TO_OUTPUT (0 << R_GPIO_EOSR_EOSR_SHIFT)  /* No affect to output */
#  define R_GPIO_EOSR_EOSR_HIGH_OUTPUT (1 << R_GPIO_EOSR_EOSR_SHIFT)          /* High output. */

/* Event output set register (16-bits) **************************************/

#define R_GPIO_EORR_EORR_SHIFT (0)
#define R_GPIO_EORR_EORR_MASK (0xffff)
#  define R_GPIO_EORR_EORR_NO_AFFECT_TO_OUTPUT (0 << R_GPIO_EORR_EORR_SHIFT)  /* No affect to output */
#  define R_GPIO_EORR_EORR_LOW_OUTPUT (1 << R_GPIO_EORR_EORR_SHIFT)           /* Low output */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_GPIO_H */
