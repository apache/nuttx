/****************************************************************************************************
 * arch/arm/src/tms570/chip/tms570_sys2.h
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

#ifndef __ARCH_ARM_SRC_TMS570_CHIP_TMS570_SYS2_H
#define __ARCH_ARM_SRC_TMS570_CHIP_TMS570_SYS2_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include "chip/tms570_memorymap.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Register Offsets *********************************************************************************/

#define TMS570_SYS2_STCCLKDIV_OFFSET    0x0008  /* CPU Logic BIST Clock Divider */
#define TMS570_SYS2_CLKSLIP_OFFSET      0x0070  /* Clock Slip Register */
#define TMS570_SYS2_EFC_CTLREG_OFFSET   0x00ec  /* EFUSE Controller Control Register */
#define TMS570_SYS2_DIEDL_REG0_OFFSET   0x00f0  /* Die Identification Register Lower Word */
#define TMS570_SYS2_DIEDH_REG1_OFFSET   0x00f4  /* Die Identification Register Upper Word */
#define TMS570_SYS2_DIEDL_REG2_OFFSET   0x00f8  /* Die Identification Register Lower Word */
#define TMS570_SYS2_DIEDH_REG3_OFFSET   0x00fc  /* Die Identification Register Upper Word */

/* Register Addresses *******************************************************************************/

#define TMS570_SYS2_STCCLKDIV           (TMS570_SYS2_BASE+TMS570_SYS2_STCCLKDIV_OFFSET)
#define TMS570_SYS2_CLKSLIP             (TMS570_SYS2_BASE+TMS570_SYS2_CLKSLIP_OFFSET)
#define TMS570_SYS2_EFC_CTLREG          (TMS570_SYS2_BASE+TMS570_SYS2_EFC_CTLREG_OFFSET)
#define TMS570_SYS2_DIEDL_REG0          (TMS570_SYS2_BASE+TMS570_SYS2_DIEDL_REG0_OFFSET)
#define TMS570_SYS2_DIEDH_REG1          (TMS570_SYS2_BASE+TMS570_SYS2_DIEDH_REG1_OFFSET)
#define TMS570_SYS2_DIEDL_REG2          (TMS570_SYS2_BASE+TMS570_SYS2_DIEDL_REG2_OFFSET)
#define TMS570_SYS2_DIEDH_REG3          (TMS570_SYS2_BASE+TMS570_SYS2_DIEDH_REG3_OFFSET)

/* Register Bit-Field Definitions *******************************************************************/

/* CPU Logic BIST Clock Divider */
#define SYS2_STCCLKDIV_
/* Clock Slip Register */
#define SYS2_CLKSLIP_
/* EFUSE Controller Control Register */
#define SYS2_EFC_CTLREG_
/* Die Identification Register Lower Word */
#define SYS2_DIEDL_REG0_
/* Die Identification Register Upper Word */
#define SYS2_DIEDH_REG1_
/* Die Identification Register Lower Word */
#define SYS2_DIEDL_REG2_
/* Die Identification Register Upper Word */
#define SYS2_DIEDH_REG3_

#endif /* __ARCH_ARM_SRC_TMS570_CHIP_TMS570_SYS2_H */
