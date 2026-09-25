/****************************************************************************
 * arch/arm/src/ra8m1/hardware/ra8m1_mstp.h
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

#ifndef __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_MSTP_H
#define __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_MSTP_H

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

#define R_MSTP_MSTPCRA_OFFSET               0x0000  /* Module Stop Control Register A (32-bits) */
#define R_MSTP_MSTPCRB_OFFSET               0x0004  /* Module Stop Control Register B (32-bits) */
#define R_MSTP_MSTPCRC_OFFSET               0x0008  /* Module Stop Control Register C (32-bits) */
#define R_MSTP_MSTPCRD_OFFSET               0x000c  /* Module Stop Control Register D (32-bits) */
#define R_MSTP_MSTPCRE_OFFSET               0x0010  /* Module Stop Control Register E (32-bits) */

/* Register Addresses *******************************************************/

/* MSTP Registers */

#define R_MSTP_MSTPCRA                     (R_MSTP_BASE + R_MSTP_MSTPCRA_OFFSET)
#define R_MSTP_MSTPCRB                     (R_MSTP_BASE + R_MSTP_MSTPCRB_OFFSET)
#define R_MSTP_MSTPCRC                     (R_MSTP_BASE + R_MSTP_MSTPCRC_OFFSET)
#define R_MSTP_MSTPCRD                     (R_MSTP_BASE + R_MSTP_MSTPCRD_OFFSET)
#define R_MSTP_MSTPCRE                     (R_MSTP_BASE + R_MSTP_MSTPCRE_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Module Stop Control Register A (32-bits) *********************************/

#define R_MSTP_MSTPCRA_MSTPA0 (1 <<  0)   /* 01: SRAM0 Module Stop */
#define R_MSTP_MSTPCRA_MSTPA1 (1 <<  1)   /* 02: SRAM1 Module Stop */
#define R_MSTP_MSTPCRA_MSTPA15 (1 << 15)  /* 8000: Standby SRAM Module Module Stop */
#define R_MSTP_MSTPCRA_MSTPA22 (1 << 22)  /* 400000: DMA Controller/Data Transfer Controller unit0 Module Stop */

/* Module Stop Control Register B (32-bits) *********************************/

#define R_MSTP_MSTPCRB_MSTPB4 (1 <<  4)   /* 10: I3C Bus Interface Module Stop */
#define R_MSTP_MSTPCRB_MSTPB8 (1 <<  8)   /* 100: I2C Bus Interface 1 Module Stop */
#define R_MSTP_MSTPCRB_MSTPB9 (1 <<  9)   /* 200: I2C Bus Interface 0 Module Stop */
#define R_MSTP_MSTPCRB_MSTPB11 (1 << 11)  /* 800: Universal Serial Bus 2.0 FS0 Interface Module Stop */
#define R_MSTP_MSTPCRB_MSTPB12 (1 << 12)  /* 1000: Universal Serial Bus 2.0 HS Interface Module Stop */
#define R_MSTP_MSTPCRB_MSTPB15 (1 << 15)  /* 8000: ETHERC0 and EDMAC0 Module Stop */
#define R_MSTP_MSTPCRB_MSTPB16 (1 << 16)  /* 10000: Octa Memory Controller Module Stop */
#define R_MSTP_MSTPCRB_MSTPB18 (1 << 18)  /* 40000: Serial Peripheral Interface 1 Module Stop */
#define R_MSTP_MSTPCRB_MSTPB19 (1 << 19)  /* 80000: Serial Peripheral Interface 0 Module Stop */
#define R_MSTP_MSTPCRB_MSTPB22 (1 << 22)  /* 400000: Serial Communication Interface 9 Module Stop */
#define R_MSTP_MSTPCRB_MSTPB27 (1 << 27)  /* 8000000: Serial Communication Interface 4 Module Stop */
#define R_MSTP_MSTPCRB_MSTPB28 (1 << 28)  /* 10000000: Serial Communication Interface 3 Module Stop */
#define R_MSTP_MSTPCRB_MSTPB29 (1 << 29)  /* 20000000: Serial Communication Interface 2 Module Stop */
#define R_MSTP_MSTPCRB_MSTPB30 (1 << 30)  /* 40000000: Serial Communication Interface 1 Module Stop */
#define R_MSTP_MSTPCRB_MSTPB31 (1 << 31)  /* 80000000: Serial Communication Interface 0 Module Stop */

/* Module Stop Control Register C (32-bits) *********************************/

#define R_MSTP_MSTPCRC_MSTPC0 (1 <<  0)   /* 01: Clock Frequency Accuracy Measurement Circuit Module Stop */
#define R_MSTP_MSTPCRC_MSTPC1 (1 <<  1)   /* 02: Cyclic Redundancy Check Calculator Module Stop */
#define R_MSTP_MSTPCRC_MSTPC4 (1 <<  4)   /* 10: Graphics LCD Controller Module Stop */
#define R_MSTP_MSTPCRC_MSTPC6 (1 <<  6)   /* 40: 2D Drawing Engine Module Stop */
#define R_MSTP_MSTPCRC_MSTPC7 (1 <<  7)   /* 80: Serial Sound Interface1 Enhanced Module Stop */
#define R_MSTP_MSTPCRC_MSTPC8 (1 <<  8)   /* 100: Serial Sound Interface0 Enhanced Module Stop */
#define R_MSTP_MSTPCRC_MSTPC10 (1 << 10)  /* 400: MIPI Display Serial Interface Module Stop */
#define R_MSTP_MSTPCRC_MSTPC11 (1 << 11)  /* 800: Secure Digital Host IF 1 Module Stop */
#define R_MSTP_MSTPCRC_MSTPC12 (1 << 12)  /* 1000: Secure Digital Host IF 0 Module Stop */
#define R_MSTP_MSTPCRC_MSTPC13 (1 << 13)  /* 2000: Data Operation Circuit Module Stop */
#define R_MSTP_MSTPCRC_MSTPC14 (1 << 14)  /* 4000: Event Link Controller Module Stop */
#define R_MSTP_MSTPCRC_MSTPC16 (1 << 16)  /* 10000: CEU Module Stop */
#define R_MSTP_MSTPCRC_MSTPC26 (1 << 26)  /* 4000000: Controller Area Network with Flexible Data-Rate 1 Module Stop */
#define R_MSTP_MSTPCRC_MSTPC27 (1 << 27)  /* 8000000: Controller Area Network with Flexible Data-Rate 0 Module Stop */
#define R_MSTP_MSTPCRC_MSTPC31 (1 << 31)  /* 80000000: SHIP Module Stop */

/* Module Stop Control Register D (32-bits) *********************************/

#define R_MSTP_MSTPCRD_MSTPD4 (1 <<  4)   /* 10: Asynchronous General Purpose Timer 1 Module Stop */
#define R_MSTP_MSTPCRD_MSTPD5 (1 <<  5)   /* 20: Asynchronous General Purpose Timer 0 Module Stop */
#define R_MSTP_MSTPCRD_MSTPD11 (1 << 11)  /* 800: Port Output Enable for GPT3 Module Stop */
#define R_MSTP_MSTPCRD_MSTPD12 (1 << 12)  /* 1000: Port Output Enable for GPT2 Module Stop */
#define R_MSTP_MSTPCRD_MSTPD13 (1 << 13)  /* 2000: Port Output Enable for GPT1 Module Stop */
#define R_MSTP_MSTPCRD_MSTPD14 (1 << 14)  /* 4000: Port Output Enable for GPT0 Module Stop */
#define R_MSTP_MSTPCRD_MSTPD15 (1 << 15)  /* 8000: 12-bit A/D Converter 1 Module Stop */
#define R_MSTP_MSTPCRD_MSTPD16 (1 << 16)  /* 10000: 12-bit A/D Converter 0 Module Stop */
#define R_MSTP_MSTPCRD_MSTPD20 (1 << 20)  /* 100000: 12-Bit D/A Converter Module Stop */
#define R_MSTP_MSTPCRD_MSTPD22 (1 << 22)  /* 400000: Temperature Sensor Module Stop */
#define R_MSTP_MSTPCRD_MSTPD27 (1 << 27)  /* 8000000: High-Speed Analog Com-parator 1 Module Stop */
#define R_MSTP_MSTPCRD_MSTPD28 (1 << 28)  /* 10000000: High-Speed Analog Com-parator 0 Module Stop */

/* Module Stop Control Register E (32-bits) *********************************/

#define R_MSTP_MSTPCRE_MSTPE8 (1 <<  8)   /* 100: Ultra-Low Power Timer 1 Module Stop */
#define R_MSTP_MSTPCRE_MSTPE9 (1 <<  9)   /* 200: Ultra-Low Power Timer 0 Module Stop */
#define R_MSTP_MSTPCRE_MSTPE18 (1 << 18)  /* 40000: General PWM Timer 13 Module Stop */
#define R_MSTP_MSTPCRE_MSTPE19 (1 << 19)  /* 80000: General PWM Timer 12 Module Stop */
#define R_MSTP_MSTPCRE_MSTPE20 (1 << 20)  /* 100000: General PWM Timer 11 Module Stop */
#define R_MSTP_MSTPCRE_MSTPE21 (1 << 21)  /* 200000: General PWM Timer 10 Module Stop */
#define R_MSTP_MSTPCRE_MSTPE22 (1 << 22)  /* 400000: General PWM Timer 9 Module Stop */
#define R_MSTP_MSTPCRE_MSTPE23 (1 << 23)  /* 800000: General PWM Timer 8 Module Stop */
#define R_MSTP_MSTPCRE_MSTPE24 (1 << 24)  /* 1000000: General PWM Timer 7 Module Stop */
#define R_MSTP_MSTPCRE_MSTPE25 (1 << 25)  /* 2000000: General PWM Timer 6 Module Stop */
#define R_MSTP_MSTPCRE_MSTPE26 (1 << 26)  /* 4000000: General PWM Timer 5 Module Stop */
#define R_MSTP_MSTPCRE_MSTPE27 (1 << 27)  /* 8000000: General PWM Timer 4 Module Stop */
#define R_MSTP_MSTPCRE_MSTPE28 (1 << 28)  /* 10000000: General PWM Timer 3 Module Stop */
#define R_MSTP_MSTPCRE_MSTPE29 (1 << 29)  /* 20000000: General PWM Timer 2 Module Stop */
#define R_MSTP_MSTPCRE_MSTPE30 (1 << 30)  /* 40000000: General PWM Timer 1 Module Stop */
#define R_MSTP_MSTPCRE_MSTPE31 (1 << 31)  /* 80000000: General PWM Timer 0 Module Stop */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_MSTP_H */
