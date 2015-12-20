/****************************************************************************************************
 * arch/arm/src/tms570/chip/tms570_pbist.h
 * Secondary System Control Register Definitions
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *
 *   TMS570LS04x/03x 16/32-Bit RISC Flash Microcontroller, Technical Reference Manual, Texas
 *   Instruments, Literature Number: SPNU517A, September 2013
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_TMS570_CHIP_TMS570_PBIST_H
#define __ARCH_ARM_SRC_TMS570_CHIP_TMS570_PBIST_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include "chip/tms570_memorymap.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/
/* PBIST RAM Groups */

#define PBIST_PBIST_ROM_GROUP        1   /* ROM */
#define PBIST_STC_ROM_GROUP          2   /* ROM */
#define PBIST_DCAN1_RAM_GROUP        3   /* Dual-port */
#define PBIST_DCAN2_RAM_GROUP        4   /* Dual-port */
#define PBIST_ESRAM1_RAM_GROUP       6   /* Single-port */
#define PBIST_MIBSPI1_RAM_GROUP      7   /* Dual-port */
#define PBIST_VIM_RAM_GROUP          10  /* Dual-port */
#define PBIST_MIBADC_RAM_GROUP       11  /* Dual-port */
#define PBIST_N2HET_RAM_GROUP        13  /* Dual-port */
#define PBIST_HET_TU_RAM_GROUP       14  /* Dual-port */

/* RAM Group Select */

#define PBIST_PBIST_ROM_RGS          1   /* ROM */
#define PBIST_STC_ROM_RGS            2   /* ROM */
#define PBIST_DCAN1_RAM_RGS          3   /* Dual-port */
#define PBIST_DCAN2_RAM_RGS          4   /* Dual-port */
#define PBIST_ESRAM1_RAM_RGS         6   /* Single-port */
#define PBIST_MIBSPI1_RAM_RGS        7   /* Dual-port */
#define PBIST_VIM_RAM_RGS            8   /* Dual-port */
#define PBIST_MIBADC_RAM_RGS         9   /* Dual-port */
#define PBIST_N2HET_RAM_RGS          11  /* Dual-port */
#define PBIST_HET_TU_RAM_RGS         12  /* Dual-port */

/* Register Offsets *********************************************************************************/

#define TMS570_PBIST_RAMT_OFFSET     0x0160 /* RAM Configuration Register */
#define TMS570_PBIST_DLR_OFFSET      0x0164 /* Datalogger Register */
#define TMS570_PBIST_PCR_OFFSET      0x016c /* Program Control Register */
#define TMS570_PBIST_PACT_OFFSET     0x0180 /* PBIST Activate/ROM Clock Enable Register */
#define TMS570_PBIST_PBISTID_OFFSET  0x0184 /* PBIST ID Register */
#define TMS570_PBIST_OVER_OFFSET     0x0188 /* Override Register */
#define TMS570_PBIST_FSRF0_OFFSET    0x0190 /* Fail Status Fail Register 0 */
#define TMS570_PBIST_FSRC0_OFFSET    0x0198 /* Fail Status Count Register 0 */
#define TMS570_PBIST_FSRC1_OFFSET    0x019c /* Fail Status Count Register 1 */
#define TMS570_PBIST_FSRA0_OFFSET    0x01a0 /* Fail Status Address 0 Register */
#define TMS570_PBIST_FSRA1_OFFSET    0x01a4 /* Fail Status Address 1 Register */
#define TMS570_PBIST_FSRDL0_OFFSET   0x01a8 /* Fail Status Data Register 0 */
#define TMS570_PBIST_FSRDL1_OFFSET   0x01b0 /* Fail Status Data Register 1 */
#define TMS570_PBIST_ROM_OFFSET      0x01c0 /* ROM Mask Register */
#define TMS570_PBIST_ALGO_OFFSET     0x01c4 /* ROM Algorithm Mask Register */
#define TMS570_PBIST_RINFOL_OFFSET   0x01c8 /* RAM Info Mask Lower Register */
#define TMS570_PBIST_RINFOU_OFFSET   0x01cc /* RAM Info Mask Upper Register */

/* Register Addresses *******************************************************************************/

#define TMS570_PBIST_RAMT            (TMS570_PBIST_BASE+TMS570_PBIST_RAMT_OFFSET)
#define TMS570_PBIST_DLR             (TMS570_PBIST_BASE+TMS570_PBIST_DLR_OFFSET)
#define TMS570_PBIST_PCR             (TMS570_PBIST_BASE+TMS570_PBIST_PCR_OFFSET)
#define TMS570_PBIST_PACT            (TMS570_PBIST_BASE+TMS570_PBIST_PACT_OFFSET)
#define TMS570_PBIST_PBISTID         (TMS570_PBIST_BASE+TMS570_PBIST_PBISTID_OFFSET)
#define TMS570_PBIST_OVER            (TMS570_PBIST_BASE+TMS570_PBIST_OVER_OFFSET)
#define TMS570_PBIST_FSRF0           (TMS570_PBIST_BASE+TMS570_PBIST_FSRF0_OFFSET)
#define TMS570_PBIST_FSRC0           (TMS570_PBIST_BASE+TMS570_PBIST_FSRC0_OFFSET)
#define TMS570_PBIST_FSRC1           (TMS570_PBIST_BASE+TMS570_PBIST_FSRC1_OFFSET)
#define TMS570_PBIST_FSRA0           (TMS570_PBIST_BASE+TMS570_PBIST_FSRA0_OFFSET)
#define TMS570_PBIST_FSRA1           (TMS570_PBIST_BASE+TMS570_PBIST_FSRA1_OFFSET)
#define TMS570_PBIST_FSRDL0          (TMS570_PBIST_BASE+TMS570_PBIST_FSRDL0_OFFSET)
#define TMS570_PBIST_FSRDL1          (TMS570_PBIST_BASE+TMS570_PBIST_FSRDL1_OFFSET)
#define TMS570_PBIST_ROM             (TMS570_PBIST_BASE+TMS570_PBIST_ROM_OFFSET)
#define TMS570_PBIST_ALGO            (TMS570_PBIST_BASE+TMS570_PBIST_ALGO_OFFSET)
#define TMS570_PBIST_RINFOL          (TMS570_PBIST_BASE+TMS570_PBIST_RINFOL_OFFSET)
#define TMS570_PBIST_RINFOU          (TMS570_PBIST_BASE+TMS570_PBIST_RINFOU_OFFSET)

/* Register Bit-Field Definitions *******************************************************************/

/* RAM Configuration Register */
#define PBIST_RAMT_
/* Datalogger Register */
#define PBIST_DLR_
/* Program Control Register */
#define PBIST_PCR_
/* PBIST Activate/ROM Clock Enable Register */
#define PBIST_PACT_
/* PBIST ID Register */
#define PBIST_PBISTID_
/* Override Register */
#define PBIST_OVER_
/* Fail Status Fail Register 0 */
#define PBIST_FSRF0_
/* Fail Status Count Register 0 */
#define PBIST_FSRC0_
/* Fail Status Count Register 1 */
#define PBIST_FSRC1_
/* Fail Status Address 0 Register */
#define PBIST_FSRA0_
/* Fail Status Address 1 Register */
#define PBIST_FSRA1_
/* Fail Status Data Register 0 */
#define PBIST_FSRDL0_
/* Fail Status Data Register 1 */
#define PBIST_FSRDL1_
/* ROM Mask Register */
#define PBIST_ROM_
/* ROM Algorithm Mask Register */
#define PBIST_ALGO_

/* RAM Info Mask Lower Register */

#define PBIST_RINFOL(n)                 (1 << ((n)-1)) /* Bit n: Select RAM group n+1 */
#  define PBIST_RINFOL_PBIST_ROM        PBIST_RINFOL(PBIST_PBIST_ROM_GROUP)
#  define PBIST_RINFOL_STC_ROM          PBIST_RINFOL(PBIST_STC_ROM_GROUP)
#  define PBIST_RINFOL_DCAN1_RAM        PBIST_RINFOL(PBIST_DCAN1_RAM_GROUP)
#  define PBIST_RINFOL_DCAN2_RAM        PBIST_RINFOL(PBIST_DCAN2_RAM_GROUP)
#  define PBIST_RINFOL_ESRAM1_RAM       PBIST_RINFOL(PBIST_ESRAM1_RAM_GROUP)
#  define PBIST_RINFOL_MIBSPI1_RAM      PBIST_RINFOL(PBIST_MIBSPI1_RAM_GROUP)
#  define PBIST_RINFOL_VIM_RAM          PBIST_RINFOL(PBIST_VIM_RAM_GROUP)
#  define PBIST_RINFOL_MIBADC_RAM       PBIST_RINFOL(PBIST_MIBADC_RAM_GROUP)
#  define PBIST_RINFOL_N2HET_RAM        PBIST_RINFOL(PBIST_N2HET_RAM_GROUP)
#  define PBIST_RINFOL_HET_TU_RAM       PBIST_RINFOL(PBIST_HET_TU_RAM_GROUP)

/* RAM Info Mask Upper Register */
#define PBIST_RINFOU_

#endif /* __ARCH_ARM_SRC_TMS570_CHIP_TMS570_PBIST_H */
