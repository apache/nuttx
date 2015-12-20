/****************************************************************************************************
 * arch/arm/src/tms570/chip/tms570_sci.h
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

#ifndef __ARCH_ARM_SRC_TMS570_CHIP_TMS570_SCI_H
#define __ARCH_ARM_SRC_TMS570_CHIP_TMS570_SCI_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include "chip/tms570_memorymap.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Register Offsets *********************************************************************************/

#define TMS570_SCI_GCR0_OFFSET        0x0000 /* SCI Global Control Register 0 */
#define TMS570_SCI_GCR1_OFFSET        0x0004 /* SCI Global Control Register 1 */
#define TMS570_SCI_GCR2_OFFSET        0x0008 /* SCI Global Control Register 2 */
#define TMS570_SCI_SETINT_OFFSET      0x000c /* SCI Set Interrupt Register */
#define TMS570_SCI_CLEARINT_OFFSET    0x0010 /* SCI Clear Interrupt Register */
#define TMS570_SCI_SETINTLVL_OFFSET   0x0014 /* SCI Set Interrupt Level Register */
#define TMS570_SCI_CLEARINTLVL_OFFSET 0x0018 /*  SCI Clear Interrupt Level Register */
#define TMS570_SCI_FLR_OFFSET         0x001c /* SCI Flags Register */
#define TMS570_SCI_INTVECT0_OFFSET    0x0020 /* SCI Interrupt Vector Offset 0 */
#define TMS570_SCI_INTVECT1_OFFSET    0x0024 /* SCI Interrupt Vector Offset 1 */
#define TMS570_SCI_FORMAT_OFFSET      0x0028 /* SCI Format Control Register */
#define TMS570_SCI_BRS_OFFSET         0x002c /* Baud Rate Selection Register */
#define TMS570_SCI_ED_OFFSET          0x0030 /* Receiver Emulation Data Buffer */
#define TMS570_SCI_RD_OFFSET          0x0034 /* Receiver Data Buffer */
#define TMS570_SCI_TD_OFFSET          0x0038 /* Transmit Data Buffer */
#define TMS570_SCI_PIO0_OFFSET        0x003c /* SCI Pin I/O Control Register 0 */
#define TMS570_SCI_PIO1_OFFSET        0x0040 /* SCI Pin I/O Control Register 1 */
#define TMS570_SCI_PIO2_OFFSET        0x0044 /* SCI Pin I/O Control Register 2 */
#define TMS570_SCI_PIO3_OFFSET        0x0048 /* SCI Pin I/O Control Register 3 */
#define TMS570_SCI_PIO4_OFFSET        0x004c /* SCI Pin I/O Control Register 4 */
#define TMS570_SCI_PIO5_OFFSET        0x0050 /* SCI Pin I/O Control Register 5 */
#define TMS570_SCI_PIO6_OFFSET        0x0054 /* SCI Pin I/O Control Register 6 */
#define TMS570_SCI_PIO7_OFFSET        0x0058 /* SCI Pin I/O Control Register 7 */
#define TMS570_SCI_PIO8_OFFSET        0x005c /* SCI Pin I/O Control Register 8 */
#define TMS570_LIN_COMPARE_OFFSET     0x0060 /* LIN Compare Register */
#define TMS570_LIN_RD0_OFFSET         0x0064 /* LIN Receive Buffer 0 Register */
#define TMS570_LIN_RD1_OFFSET         0x0068 /* LIN Receive Buffer 1 Register */
#define TMS570_LIN_MASK_OFFSET        0x006c /* LIN Mask Register */
#define TMS570_LIN_ID_OFFSET          0x0070 /* LIN Identification Register */
#define TMS570_LIN_TD0_OFFSET         0x0074 /* LIN Transmit Buffer 0 */
#define TMS570_LIN_TD1_OFFSET         0x0078 /* LIN Transmit Buffer 1 */
#define TMS570_SCI_MBRS_OFFSET        0x007c /* Maximum Baud Rate Selection Register */
#define TMS570_SCI_IODFTCTRL_OFFSET   0x0090 /* Input/Output Error Enable Register */

/* Register Addresses *******************************************************************************/

#define TMS570_SCI1_GCR0              (TMS570_SCI1_BASE+TMS570_SCI_GCR0_OFFSET)
#define TMS570_SCI1_GCR1              (TMS570_SCI1_BASE+TMS570_SCI_GCR1_OFFSET)
#define TMS570_SCI1_GCR2              (TMS570_SCI1_BASE+TMS570_SCI_GCR2_OFFSET)
#define TMS570_SCI1_SETINT            (TMS570_SCI1_BASE+TMS570_SCI_SETINT_OFFSET)
#define TMS570_SCI1_CLEARINT          (TMS570_SCI1_BASE+TMS570_SCI_CLEARINT_OFFSET)
#define TMS570_SCI1_SETINTLVL         (TMS570_SCI1_BASE+TMS570_SCI_SETINTLVL_OFFSET)
#define TMS570_SCI1_CLEARINTLVL       (TMS570_SCI1_BASE+TMS570_SCI_CLEARINTLVL_OFFSET)
#define TMS570_SCI1_FLR               (TMS570_SCI1_BASE+TMS570_SCI_FLR_OFFSET)
#define TMS570_SCI1_INTVECT0          (TMS570_SCI1_BASE+TMS570_SCI_INTVECT0_OFFSET)
#define TMS570_SCI1_INTVECT1          (TMS570_SCI1_BASE+TMS570_SCI_INTVECT1_OFFSET)
#define TMS570_SCI1_FORMAT            (TMS570_SCI1_BASE+TMS570_SCI_FORMAT_OFFSET)
#define TMS570_SCI1_BRS               (TMS570_SCI1_BASE+TMS570_SCI_BRS_OFFSET)
#define TMS570_SCI1_ED                (TMS570_SCI1_BASE+TMS570_SCI_ED_OFFSET)
#define TMS570_SCI1_RD                (TMS570_SCI1_BASE+TMS570_SCI_RD_OFFSET)
#define TMS570_SCI1_TD                (TMS570_SCI1_BASE+TMS570_SCI_TD_OFFSET)
#define TMS570_SCI1_PIO0              (TMS570_SCI1_BASE+TMS570_SCI_PIO0_OFFSET)
#define TMS570_SCI1_PIO1              (TMS570_SCI1_BASE+TMS570_SCI_PIO1_OFFSET)
#define TMS570_SCI1_PIO2              (TMS570_SCI1_BASE+TMS570_SCI_PIO2_OFFSET)
#define TMS570_SCI1_PIO3              (TMS570_SCI1_BASE+TMS570_SCI_PIO3_OFFSET)
#define TMS570_SCI1_PIO4              (TMS570_SCI1_BASE+TMS570_SCI_PIO4_OFFSET)
#define TMS570_SCI1_PIO5              (TMS570_SCI1_BASE+TMS570_SCI_PIO5_OFFSET)
#define TMS570_SCI1_PIO6              (TMS570_SCI1_BASE+TMS570_SCI_PIO6_OFFSET)
#define TMS570_SCI1_PIO7              (TMS570_SCI1_BASE+TMS570_SCI_PIO7_OFFSET)
#define TMS570_SCI1_PIO8              (TMS570_SCI1_BASE+TMS570_SCI_PIO8_OFFSET)
#define TMS570_LIN1_COMPARE           (TMS570_SCI1_BASE+TMS570_LIN_COMPARE_OFFSET)
#define TMS570_LIN1_RD0               (TMS570_SCI1_BASE+TMS570_LIN_RD0_OFFSET)
#define TMS570_LIN1_RD1               (TMS570_SCI1_BASE+TMS570_LIN_RD1_OFFSET)
#define TMS570_LIN1_MASK              (TMS570_SCI1_BASE+TMS570_LIN_MASK_OFFSET)
#define TMS570_LIN1_ID                (TMS570_SCI1_BASE+TMS570_LIN_ID_OFFSET)
#define TMS570_LIN1_TD0               (TMS570_SCI1_BASE+TMS570_LIN_TD0_OFFSET)
#define TMS570_LIN1_TD1               (TMS570_SCI1_BASE+TMS570_LIN_TD1_OFFSET)
#define TMS570_SCI1_MBRS              (TMS570_SCI1_BASE+TMS570_SCI_MBRS_OFFSET)
#define TMS570_SCI1_IODFTCTRL         (TMS570_SCI1_BASE+TMS570_SCI_IODFTCTRL_OFFSET)

#if TMS570_NSCI > 1
#  define TMS570_SCI2_GCR0            (TMS570_SCI2_BASE+TMS570_SCI_GCR0_OFFSET)
#  define TMS570_SCI2_GCR1            (TMS570_SCI2_BASE+TMS570_SCI_GCR1_OFFSET)
#  define TMS570_SCI2_GCR2            (TMS570_SCI2_BASE+TMS570_SCI_GCR2_OFFSET)
#  define TMS570_SCI2_SETINT          (TMS570_SCI2_BASE+TMS570_SCI_SETINT_OFFSET)
#  define TMS570_SCI2_CLEARINT        (TMS570_SCI2_BASE+TMS570_SCI_CLEARINT_OFFSET)
#  define TMS570_SCI2_SETINTLVL       (TMS570_SCI2_BASE+TMS570_SCI_SETINTLVL_OFFSET)
#  define TMS570_SCI2_CLEARINTLVL     (TMS570_SCI2_BASE+TMS570_SCI_CLEARINTLVL_OFFSET)
#  define TMS570_SCI2_FLR             (TMS570_SCI2_BASE+TMS570_SCI_FLR_OFFSET)
#  define TMS570_SCI2_INTVECT0        (TMS570_SCI2_BASE+TMS570_SCI_INTVECT0_OFFSET)
#  define TMS570_SCI2_INTVECT1        (TMS570_SCI2_BASE+TMS570_SCI_INTVECT1_OFFSET)
#  define TMS570_SCI2_FORMAT          (TMS570_SCI2_BASE+TMS570_SCI_FORMAT_OFFSET)
#  define TMS570_SCI2_BRS             (TMS570_SCI2_BASE+TMS570_SCI_BRS_OFFSET)
#  define TMS570_SCI2_ED              (TMS570_SCI2_BASE+TMS570_SCI_ED_OFFSET)
#  define TMS570_SCI2_RD              (TMS570_SCI2_BASE+TMS570_SCI_RD_OFFSET)
#  define TMS570_SCI2_TD              (TMS570_SCI2_BASE+TMS570_SCI_TD_OFFSET)
#  define TMS570_SCI2_PIO0            (TMS570_SCI2_BASE+TMS570_SCI_PIO0_OFFSET)
#  define TMS570_SCI2_PIO1            (TMS570_SCI2_BASE+TMS570_SCI_PIO1_OFFSET)
#  define TMS570_SCI2_PIO2            (TMS570_SCI2_BASE+TMS570_SCI_PIO2_OFFSET)
#  define TMS570_SCI2_PIO3            (TMS570_SCI2_BASE+TMS570_SCI_PIO3_OFFSET)
#  define TMS570_SCI2_PIO4            (TMS570_SCI2_BASE+TMS570_SCI_PIO4_OFFSET)
#  define TMS570_SCI2_PIO5            (TMS570_SCI2_BASE+TMS570_SCI_PIO5_OFFSET)
#  define TMS570_SCI2_PIO6            (TMS570_SCI2_BASE+TMS570_SCI_PIO6_OFFSET)
#  define TMS570_SCI2_PIO7            (TMS570_SCI2_BASE+TMS570_SCI_PIO7_OFFSET)
#  define TMS570_SCI2_PIO8            (TMS570_SCI2_BASE+TMS570_SCI_PIO8_OFFSET)
#  define TMS570_LIN2_COMPARE         (TMS570_SCI2_BASE+TMS570_LIN_COMPARE_OFFSET)
#  define TMS570_LIN2_RD0             (TMS570_SCI2_BASE+TMS570_LIN_RD0_OFFSET)
#  define TMS570_LIN2_RD1             (TMS570_SCI2_BASE+TMS570_LIN_RD1_OFFSET)
#  define TMS570_LIN2_MASK            (TMS570_SCI2_BASE+TMS570_LIN_MASK_OFFSET)
#  define TMS570_LIN2_ID              (TMS570_SCI2_BASE+TMS570_LIN_ID_OFFSET)
#  define TMS570_LIN2_TD0             (TMS570_SCI2_BASE+TMS570_LIN_TD0_OFFSET)
#  define TMS570_LIN2_TD1             (TMS570_SCI2_BASE+TMS570_LIN_TD1_OFFSET)
#  define TMS570_SCI2_MBRS            (TMS570_SCI2_BASE+TMS570_SCI_MBRS_OFFSET)
#  define TMS570_SCI2_IODFTCTRL       (TMS570_SCI2_BASE+TMS570_SCI_IODFTCTRL_OFFSET)
#endif

/* Register Bit-Field Definitions *******************************************************************/

/* SCI Global Control Register 0 */
#define SCI_GCR0_
/* SCI Global Control Register 1 */
#define SCI_GCR1_
/* SCI Global Control Register 2 */
#define SCI_GCR2_
/* SCI Set Interrupt Register */
#define SCI_SETINT_
/* SCI Clear Interrupt Register */
#define SCI_CLEARINT_
/* SCI Set Interrupt Level Register */
#define SCI_SETINTLVL_
/*  SCI Clear Interrupt Level Register */
#define SCI_CLEARINTLVL_
/* SCI Flags Register */
#define SCI_FLR_
/* SCI Interrupt Vector Offset 0 */
#define SCI_INTVECT0_
/* SCI Interrupt Vector Offset 1 */
#define SCI_INTVECT1_
/* SCI Format Control Register */
#define SCI_FORMAT_
/* Baud Rate Selection Register */
#define SCI_BRS_
/* Receiver Emulation Data Buffer */
#define SCI_ED_
/* Receiver Data Buffer */
#define SCI_RD_
/* Transmit Data Buffer */
#define SCI_TD_
/* SCI Pin I/O Control Register 0 */
#define SCI_PIO0_
/* SCI Pin I/O Control Register 1 */
#define SCI_PIO1_
/* SCI Pin I/O Control Register 2 */
#define SCI_PIO2_
/* SCI Pin I/O Control Register 3 */
#define SCI_PIO3_
/* SCI Pin I/O Control Register 4 */
#define SCI_PIO4_
/* SCI Pin I/O Control Register 5 */
#define SCI_PIO5_
/* SCI Pin I/O Control Register 6 */
#define SCI_PIO6_
/* SCI Pin I/O Control Register 7 */
#define SCI_PIO7_
/* SCI Pin I/O Control Register 8 */
#define SCI_PIO8_
/* LIN Compare Register */
#define LIN_COMPARE_
/* LIN Receive Buffer 0 Register */
#define LIN_RD0_
/* LIN Receive Buffer 1 Register */
#define LIN_RD1_
/* LIN Mask Register */
#define LIN_MASK_
/* LIN Identification Register */
#define LIN_ID_
/* LIN Transmit Buffer 0 */
#define LIN_TD0_
/* LIN Transmit Buffer 1 */
#define LIN_TD1_
/* Maximum Baud Rate Selection Register */
#define SCI_MBRS_
/* Input/Output Error Enable Register */
#define SCI_IODFTCTRL_

#endif /* __ARCH_ARM_SRC_TMS570_CHIP_TMS570_SCI_H */
