/************************************************************************************
 * arch/arm/src/xmc4/chip/xmc4_usic.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Reference: XMC4500 Reference Manual V1.5 2014-07 Microcontrollers.
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
 * May include some logic from sample code provided by Infineon:
 *
 *   Copyright (C) 2011-2015 Infineon Technologies AG. All rights reserved.
 *
 * Infineon Technologies AG (Infineon) is supplying this software for use with
 * Infineon's microcontrollers.  This file can be freely distributed within
 * development tools that are supporting such microcontrollers.
 *
 * THIS SOFTWARE IS PROVIDED AS IS. NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 * OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_XMC4_CHIP_XMC4_USIC_H
#define __ARCH_ARM_SRC_XMC4_CHIP_XMC4_USIC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip/xmc4_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

/* PMU Registers -- See ID register */
/* Prefetch Registers -- See PCON register */

/* Kernal Registers */

#define XMC4_USIC_ID_OFFSET          0x0008    /* Kernel State Configuration Register */

/* USIC Channel Registers */

#define XMC4_USIC_CCFG_OFFSET        0x0004    /* Channel Configuration Register */
#define XMC4_USIC_KSCFG_OFFSET       0x000c    /* Kernel State Configuration Register */
#define XMC4_USIC_FDR_OFFSET         0x0010    /* Fractional Divider Register */
#define XMC4_USIC_BRG_OFFSET         0x0014    /* Baud Rate Generator Register */
#define XMC4_USIC_INPR_OFFSET        0x0018    /* Interrupt Node Pointer Register */
#define XMC4_USIC_DX0CR_OFFSET       0x001c    /* Input Control Register 0 */
#define XMC4_USIC_DX1CR_OFFSET       0x0020    /* Input Control Register 1 */
#define XMC4_USIC_DX2CR_OFFSET       0x0024    /* Input Control Register 2 */
#define XMC4_USIC_DX3CR_OFFSET       0x0028    /* Input Control Register 3 */
#define XMC4_USIC_DX4CR_OFFSET       0x002c    /* Input Control Register 4 */
#define XMC4_USIC_DX5CR_OFFSET       0x0030    /* Input Control Register 5 */
#define XMC4_USIC_SCTR_OFFSET        0x0034    /* Shift Control Register */
#define XMC4_USIC_TCSR_OFFSET        0x0038    /* Transmit Control/Status Register */
#define XMC4_USIC_PCR_OFFSET         0x003c    /* Protocol Control Register */
#define XMC4_USIC_CCR_OFFSET         0x0040    /* Channel Control Register */
#define XMC4_USIC_CMTR_OFFSET        0x0044    /* Capture Mode Timer Register */
#define XMC4_USIC_PSR_OFFSET         0x0048    /* Protocol Status Register */
#define XMC4_USIC_PSCR_OFFSET        0x004c    /* Protocol Status Clear Register */
#define XMC4_USIC_RBUFSR_OFFSET      0x0050    /* Receiver Buffer Status Register */
#define XMC4_USIC_RBUF_OFFSET        0x0054    /* Receiver Buffer Register */
#define XMC4_USIC_RBUFD_OFFSET       0x0058    /* Receiver Buffer Register for Debugger */
#define XMC4_USIC_RBUF0_OFFSET       0x005c    /* Receiver Buffer Register 0 */
#define XMC4_USIC_RBUF1_OFFSET       0x0060    /* Receiver Buffer Register 1 */
#define XMC4_USIC_RBUF01SR_OFFSET    0x0064    /*  Receiver Buffer 01 Status Register */
#define XMC4_USIC_FMR_OFFSET         0x0068    /* Flag Modification Register */
#define XMC4_USIC_TBUF_OFFSET        0x0080    /* Transmit Buffer (32 x 4-bytes) */

/* USIC FIFO Registers */

#define XMC4_USIC_BYP_OFFSET         0x0100    /* Bypass Data Register */
#define XMC4_USIC_BYPCR_OFFSET       0x0104    /* Bypass Control Register */
#define XMC4_USIC_TBCTR_OFFSET       0x0108    /* Transmitter Buffer Control Register */
#define XMC4_USIC_RBCTR_OFFSET       0x010c    /* Receiver Buffer Control Register */
#define XMC4_USIC_TRBPTR_OFFSET      0x0110    /* Transmit/Receive Buffer Pointer Register */
#define XMC4_USIC_TRBSR_OFFSET       0x0114    /* Transmit/Receive Buffer Status Register */
#define XMC4_USIC_TRBSCR_OFFSET      0x0118    /* Transmit/Receive Buffer Status Clear Register */
#define XMC4_USIC_OUTR_OFFSET        0x011c    /* Receiver Buffer Output Register */
#define XMC4_USIC_OUTDR_OFFSET       0x0120    /* Receiver Buffer Output Register L for Debugger */
#define XMC4_USIC_IN_OFFSET          0x0180    /* Transmit FIFO Buffer (32 x 4-bytes) */

/* Register Addresses ****************************************************************/

/* USIC0 Registers */
/* Kernal Registers */

#define XMC4_USIC0_ID                (XMC4_USIC0_BASE+XMC4_USIC_ID_OFFSET)

/* USIC0 Channel 0 Registers */

#define XMC4_USIC00_CCFG             (XMC4_USIC0_CH0_BASE+XMC4_USIC_CCFG_OFFSET)
#define XMC4_USIC00_KSCFG            (XMC4_USIC0_CH0_BASE+XMC4_USIC_KSCFG_OFFSET)
#define XMC4_USIC00_FDR              (XMC4_USIC0_CH0_BASE+XMC4_USIC_FDR_OFFSET)
#define XMC4_USIC00_BRG              (XMC4_USIC0_CH0_BASE+XMC4_USIC_BRG_OFFSET)
#define XMC4_USIC00_INPR             (XMC4_USIC0_CH0_BASE+XMC4_USIC_INPR_OFFSET)
#define XMC4_USIC00_DX0CR            (XMC4_USIC0_CH0_BASE+XMC4_USIC_DX0CR_OFFSET)
#define XMC4_USIC00_DX1CR            (XMC4_USIC0_CH0_BASE+XMC4_USIC_DX1CR_OFFSET)
#define XMC4_USIC00_DX2CR            (XMC4_USIC0_CH0_BASE+XMC4_USIC_DX2CR_OFFSET)
#define XMC4_USIC00_DX3CR            (XMC4_USIC0_CH0_BASE+XMC4_USIC_DX3CR_OFFSET)
#define XMC4_USIC00_DX4CR            (XMC4_USIC0_CH0_BASE+XMC4_USIC_DX4CR_OFFSET)
#define XMC4_USIC00_DX5CR            (XMC4_USIC0_CH0_BASE+XMC4_USIC_DX5CR_OFFSET)
#define XMC4_USIC00_SCTR             (XMC4_USIC0_CH0_BASE+XMC4_USIC_SCTR_OFFSET)
#define XMC4_USIC00_TCSR             (XMC4_USIC0_CH0_BASE+XMC4_USIC_TCSR_OFFSET)
#define XMC4_USIC00_PCR              (XMC4_USIC0_CH0_BASE+XMC4_USIC_PCR_OFFSET)
#define XMC4_USIC00_CCR              (XMC4_USIC0_CH0_BASE+XMC4_USIC_CCR_OFFSET)
#define XMC4_USIC00_CMTR             (XMC4_USIC0_CH0_BASE+XMC4_USIC_CMTR_OFFSET)
#define XMC4_USIC00_PSR              (XMC4_USIC0_CH0_BASE+XMC4_USIC_PSR_OFFSET)
#define XMC4_USIC00_PSCR             (XMC4_USIC0_CH0_BASE+XMC4_USIC_PSCR_OFFSET)
#define XMC4_USIC00_RBUFSR           (XMC4_USIC0_CH0_BASE+XMC4_USIC_RBUFSR_OFFSET)
#define XMC4_USIC00_RBUF             (XMC4_USIC0_CH0_BASE+XMC4_USIC_RBUF_OFFSET)
#define XMC4_USIC00_RBUFD            (XMC4_USIC0_CH0_BASE+XMC4_USIC_RBUFD_OFFSET)
#define XMC4_USIC00_RBUF0            (XMC4_USIC0_CH0_BASE+XMC4_USIC_RBUF0_OFFSET)
#define XMC4_USIC00_RBUF1            (XMC4_USIC0_CH0_BASE+XMC4_USIC_RBUF1_OFFSET)
#define XMC4_USIC00_RBUF01SR         (XMC4_USIC0_CH0_BASE+XMC4_USIC_RBUF01SR_OFFSET)
#define XMC4_USIC00_FMR              (XMC4_USIC0_CH0_BASE+XMC4_USIC_FMR_OFFSET)
#define XMC4_USIC00_TBUF             (XMC4_USIC0_CH0_BASE+XMC4_USIC_TBUF_OFFSET)

/* USIC0 Channel 0 FIFO Registers */

#define XMC4_USIC00_BYP              (XMC4_USIC0_CH0_BASE+XMC4_USIC_BYP_OFFSET)
#define XMC4_USIC00_BYPCR            (XMC4_USIC0_CH0_BASE+XMC4_USIC_BYPCR_OFFSET)
#define XMC4_USIC00_TBCTR            (XMC4_USIC0_CH0_BASE+XMC4_USIC_TBCTR_OFFSET)
#define XMC4_USIC00_RBCTR            (XMC4_USIC0_CH0_BASE+XMC4_USIC_RBCTR_OFFSET)
#define XMC4_USIC00_TRBPTR           (XMC4_USIC0_CH0_BASE+XMC4_USIC_TRBPTR_OFFSET)
#define XMC4_USIC00_TRBSR            (XMC4_USIC0_CH0_BASE+XMC4_USIC_TRBSR_OFFSET)
#define XMC4_USIC00_TRBSCR           (XMC4_USIC0_CH0_BASE+XMC4_USIC_TRBSCR_OFFSET)
#define XMC4_USIC00_OUTR             (XMC4_USIC0_CH0_BASE+XMC4_USIC_OUTR_OFFSET)
#define XMC4_USIC00_OUTDR            (XMC4_USIC0_CH0_BASE+XMC4_USIC_OUTDR_OFFSET)
#define XMC4_USIC00_IN               (XMC4_USIC0_CH0_BASE+XMC4_USIC_IN_OFFSET)

/* USIC0 Channel 1 Registers */

#define XMC4_USIC01_CCFG             (XMC4_USIC0_CH1_BASE+XMC4_USIC_CCFG_OFFSET)
#define XMC4_USIC01_KSCFG            (XMC4_USIC0_CH1_BASE+XMC4_USIC_KSCFG_OFFSET)
#define XMC4_USIC01_FDR              (XMC4_USIC0_CH1_BASE+XMC4_USIC_FDR_OFFSET)
#define XMC4_USIC01_BRG              (XMC4_USIC0_CH1_BASE+XMC4_USIC_BRG_OFFSET)
#define XMC4_USIC01_INPR             (XMC4_USIC0_CH1_BASE+XMC4_USIC_INPR_OFFSET)
#define XMC4_USIC01_DX0CR            (XMC4_USIC0_CH1_BASE+XMC4_USIC_DX0CR_OFFSET)
#define XMC4_USIC01_DX1CR            (XMC4_USIC0_CH1_BASE+XMC4_USIC_DX1CR_OFFSET)
#define XMC4_USIC01_DX2CR            (XMC4_USIC0_CH1_BASE+XMC4_USIC_DX2CR_OFFSET)
#define XMC4_USIC01_DX3CR            (XMC4_USIC0_CH1_BASE+XMC4_USIC_DX3CR_OFFSET)
#define XMC4_USIC01_DX4CR            (XMC4_USIC0_CH1_BASE+XMC4_USIC_DX4CR_OFFSET)
#define XMC4_USIC01_DX5CR            (XMC4_USIC0_CH1_BASE+XMC4_USIC_DX5CR_OFFSET)
#define XMC4_USIC01_SCTR             (XMC4_USIC0_CH1_BASE+XMC4_USIC_SCTR_OFFSET)
#define XMC4_USIC01_TCSR             (XMC4_USIC0_CH1_BASE+XMC4_USIC_TCSR_OFFSET)
#define XMC4_USIC01_PCR              (XMC4_USIC0_CH1_BASE+XMC4_USIC_PCR_OFFSET)
#define XMC4_USIC01_CCR              (XMC4_USIC0_CH1_BASE+XMC4_USIC_CCR_OFFSET)
#define XMC4_USIC01_CMTR             (XMC4_USIC0_CH1_BASE+XMC4_USIC_CMTR_OFFSET)
#define XMC4_USIC01_PSR              (XMC4_USIC0_CH1_BASE+XMC4_USIC_PSR_OFFSET)
#define XMC4_USIC01_PSCR             (XMC4_USIC0_CH1_BASE+XMC4_USIC_PSCR_OFFSET)
#define XMC4_USIC01_RBUFSR           (XMC4_USIC0_CH1_BASE+XMC4_USIC_RBUFSR_OFFSET)
#define XMC4_USIC01_RBUF             (XMC4_USIC0_CH1_BASE+XMC4_USIC_RBUF_OFFSET)
#define XMC4_USIC01_RBUFD            (XMC4_USIC0_CH1_BASE+XMC4_USIC_RBUFD_OFFSET)
#define XMC4_USIC01_RBUF0            (XMC4_USIC0_CH1_BASE+XMC4_USIC_RBUF0_OFFSET)
#define XMC4_USIC01_RBUF1            (XMC4_USIC0_CH1_BASE+XMC4_USIC_RBUF1_OFFSET)
#define XMC4_USIC01_RBUF01SR         (XMC4_USIC0_CH1_BASE+XMC4_USIC_RBUF01SR_OFFSET)
#define XMC4_USIC01_FMR              (XMC4_USIC0_CH1_BASE+XMC4_USIC_FMR_OFFSET)
#define XMC4_USIC01_TBUF             (XMC4_USIC0_CH1_BASE+XMC4_USIC_TBUF_OFFSET)

/* USIC0 Channel 1 FIFO Registers */

#define XMC4_USIC01_BYP              (XMC4_USIC0_CH1_BASE+XMC4_USIC_BYP_OFFSET)
#define XMC4_USIC01_BYPCR            (XMC4_USIC0_CH1_BASE+XMC4_USIC_BYPCR_OFFSET)
#define XMC4_USIC01_TBCTR            (XMC4_USIC0_CH1_BASE+XMC4_USIC_TBCTR_OFFSET)
#define XMC4_USIC01_RBCTR            (XMC4_USIC0_CH1_BASE+XMC4_USIC_RBCTR_OFFSET)
#define XMC4_USIC01_TRBPTR           (XMC4_USIC0_CH1_BASE+XMC4_USIC_TRBPTR_OFFSET)
#define XMC4_USIC01_TRBSR            (XMC4_USIC0_CH1_BASE+XMC4_USIC_TRBSR_OFFSET)
#define XMC4_USIC01_TRBSCR           (XMC4_USIC0_CH1_BASE+XMC4_USIC_TRBSCR_OFFSET)
#define XMC4_USIC01_OUTR             (XMC4_USIC0_CH1_BASE+XMC4_USIC_OUTR_OFFSET)
#define XMC4_USIC01_OUTDR            (XMC4_USIC0_CH1_BASE+XMC4_USIC_OUTDR_OFFSET)
#define XMC4_USIC01_IN               (XMC4_USIC0_CH1_BASE+XMC4_USIC_IN_OFFSET)

/* USIC1 Registers */
/* Kernal Registers */

#define XMC4_USIC1_ID                (XMC4_USIC1_BASE+XMC4_USIC_ID_OFFSET)

/* USIC1 Channel 0 Registers */

#define XMC4_USIC10_CCFG             (XMC4_USIC1_CH0_BASE+XMC4_USIC_CCFG_OFFSET)
#define XMC4_USIC10_KSCFG            (XMC4_USIC1_CH0_BASE+XMC4_USIC_KSCFG_OFFSET)
#define XMC4_USIC10_FDR              (XMC4_USIC1_CH0_BASE+XMC4_USIC_FDR_OFFSET)
#define XMC4_USIC10_BRG              (XMC4_USIC1_CH0_BASE+XMC4_USIC_BRG_OFFSET)
#define XMC4_USIC10_INPR             (XMC4_USIC1_CH0_BASE+XMC4_USIC_INPR_OFFSET)
#define XMC4_USIC10_DX0CR            (XMC4_USIC1_CH0_BASE+XMC4_USIC_DX0CR_OFFSET)
#define XMC4_USIC10_DX1CR            (XMC4_USIC1_CH0_BASE+XMC4_USIC_DX1CR_OFFSET)
#define XMC4_USIC10_DX2CR            (XMC4_USIC1_CH0_BASE+XMC4_USIC_DX2CR_OFFSET)
#define XMC4_USIC10_DX3CR            (XMC4_USIC1_CH0_BASE+XMC4_USIC_DX3CR_OFFSET)
#define XMC4_USIC10_DX4CR            (XMC4_USIC1_CH0_BASE+XMC4_USIC_DX4CR_OFFSET)
#define XMC4_USIC10_DX5CR            (XMC4_USIC1_CH0_BASE+XMC4_USIC_DX5CR_OFFSET)
#define XMC4_USIC10_SCTR             (XMC4_USIC1_CH0_BASE+XMC4_USIC_SCTR_OFFSET)
#define XMC4_USIC10_TCSR             (XMC4_USIC1_CH0_BASE+XMC4_USIC_TCSR_OFFSET)
#define XMC4_USIC10_PCR              (XMC4_USIC1_CH0_BASE+XMC4_USIC_PCR_OFFSET)
#define XMC4_USIC10_CCR              (XMC4_USIC1_CH0_BASE+XMC4_USIC_CCR_OFFSET)
#define XMC4_USIC10_CMTR             (XMC4_USIC1_CH0_BASE+XMC4_USIC_CMTR_OFFSET)
#define XMC4_USIC10_PSR              (XMC4_USIC1_CH0_BASE+XMC4_USIC_PSR_OFFSET)
#define XMC4_USIC10_PSCR             (XMC4_USIC1_CH0_BASE+XMC4_USIC_PSCR_OFFSET)
#define XMC4_USIC10_RBUFSR           (XMC4_USIC1_CH0_BASE+XMC4_USIC_RBUFSR_OFFSET)
#define XMC4_USIC10_RBUF             (XMC4_USIC1_CH0_BASE+XMC4_USIC_RBUF_OFFSET)
#define XMC4_USIC10_RBUFD            (XMC4_USIC1_CH0_BASE+XMC4_USIC_RBUFD_OFFSET)
#define XMC4_USIC10_RBUF0            (XMC4_USIC1_CH0_BASE+XMC4_USIC_RBUF0_OFFSET)
#define XMC4_USIC10_RBUF1            (XMC4_USIC1_CH0_BASE+XMC4_USIC_RBUF1_OFFSET)
#define XMC4_USIC10_RBUF01SR         (XMC4_USIC1_CH0_BASE+XMC4_USIC_RBUF01SR_OFFSET)
#define XMC4_USIC10_FMR              (XMC4_USIC1_CH0_BASE+XMC4_USIC_FMR_OFFSET)
#define XMC4_USIC10_TBUF             (XMC4_USIC1_CH0_BASE+XMC4_USIC_TBUF_OFFSET)

/* USIC1 Channel 0 FIFO Registers */

#define XMC4_USIC10_BYP              (XMC4_USIC1_CH0_BASE+XMC4_USIC_BYP_OFFSET)
#define XMC4_USIC10_BYPCR            (XMC4_USIC1_CH0_BASE+XMC4_USIC_BYPCR_OFFSET)
#define XMC4_USIC10_TBCTR            (XMC4_USIC1_CH0_BASE+XMC4_USIC_TBCTR_OFFSET)
#define XMC4_USIC10_RBCTR            (XMC4_USIC1_CH0_BASE+XMC4_USIC_RBCTR_OFFSET)
#define XMC4_USIC10_TRBPTR           (XMC4_USIC1_CH0_BASE+XMC4_USIC_TRBPTR_OFFSET)
#define XMC4_USIC10_TRBSR            (XMC4_USIC1_CH0_BASE+XMC4_USIC_TRBSR_OFFSET)
#define XMC4_USIC10_TRBSCR           (XMC4_USIC1_CH0_BASE+XMC4_USIC_TRBSCR_OFFSET)
#define XMC4_USIC10_OUTR             (XMC4_USIC1_CH0_BASE+XMC4_USIC_OUTR_OFFSET)
#define XMC4_USIC10_OUTDR            (XMC4_USIC1_CH0_BASE+XMC4_USIC_OUTDR_OFFSET)
#define XMC4_USIC10_IN               (XMC4_USIC1_CH0_BASE+XMC4_USIC_IN_OFFSET)

/* USIC1 Channel 1 Registers */

#define XMC4_USCI11_CCFG             (XMC4_USIC1_CH1_BASE+XMC4_USIC_CCFG_OFFSET)
#define XMC4_USCI11_KSCFG            (XMC4_USIC1_CH1_BASE+XMC4_USIC_KSCFG_OFFSET)
#define XMC4_USCI11_FDR              (XMC4_USIC1_CH1_BASE+XMC4_USIC_FDR_OFFSET)
#define XMC4_USCI11_BRG              (XMC4_USIC1_CH1_BASE+XMC4_USIC_BRG_OFFSET)
#define XMC4_USCI11_INPR             (XMC4_USIC1_CH1_BASE+XMC4_USIC_INPR_OFFSET)
#define XMC4_USCI11_DX0CR            (XMC4_USIC1_CH1_BASE+XMC4_USIC_DX0CR_OFFSET)
#define XMC4_USCI11_DX1CR            (XMC4_USIC1_CH1_BASE+XMC4_USIC_DX1CR_OFFSET)
#define XMC4_USCI11_DX2CR            (XMC4_USIC1_CH1_BASE+XMC4_USIC_DX2CR_OFFSET)
#define XMC4_USCI11_DX3CR            (XMC4_USIC1_CH1_BASE+XMC4_USIC_DX3CR_OFFSET)
#define XMC4_USCI11_DX4CR            (XMC4_USIC1_CH1_BASE+XMC4_USIC_DX4CR_OFFSET)
#define XMC4_USCI11_DX5CR            (XMC4_USIC1_CH1_BASE+XMC4_USIC_DX5CR_OFFSET)
#define XMC4_USCI11_SCTR             (XMC4_USIC1_CH1_BASE+XMC4_USIC_SCTR_OFFSET)
#define XMC4_USCI11_TCSR             (XMC4_USIC1_CH1_BASE+XMC4_USIC_TCSR_OFFSET)
#define XMC4_USCI11_PCR              (XMC4_USIC1_CH1_BASE+XMC4_USIC_PCR_OFFSET)
#define XMC4_USCI11_CCR              (XMC4_USIC1_CH1_BASE+XMC4_USIC_CCR_OFFSET)
#define XMC4_USCI11_CMTR             (XMC4_USIC1_CH1_BASE+XMC4_USIC_CMTR_OFFSET)
#define XMC4_USCI11_PSR              (XMC4_USIC1_CH1_BASE+XMC4_USIC_PSR_OFFSET)
#define XMC4_USCI11_PSCR             (XMC4_USIC1_CH1_BASE+XMC4_USIC_PSCR_OFFSET)
#define XMC4_USCI11_RBUFSR           (XMC4_USIC1_CH1_BASE+XMC4_USIC_RBUFSR_OFFSET)
#define XMC4_USCI11_RBUF             (XMC4_USIC1_CH1_BASE+XMC4_USIC_RBUF_OFFSET)
#define XMC4_USCI11_RBUFD            (XMC4_USIC1_CH1_BASE+XMC4_USIC_RBUFD_OFFSET)
#define XMC4_USCI11_RBUF0            (XMC4_USIC1_CH1_BASE+XMC4_USIC_RBUF0_OFFSET)
#define XMC4_USCI11_RBUF1            (XMC4_USIC1_CH1_BASE+XMC4_USIC_RBUF1_OFFSET)
#define XMC4_USCI11_RBUF01SR         (XMC4_USIC1_CH1_BASE+XMC4_USIC_RBUF01SR_OFFSET)
#define XMC4_USCI11_FMR              (XMC4_USIC1_CH1_BASE+XMC4_USIC_FMR_OFFSET)
#define XMC4_USCI11_TBUF             (XMC4_USIC1_CH1_BASE+XMC4_USIC_TBUF_OFFSET)

/* USIC1 Channel 1 FIFO Registers */

#define XMC4_USCI11_BYP              (XMC4_USIC1_CH1_BASE+XMC4_USIC_BYP_OFFSET)
#define XMC4_USCI11_BYPCR            (XMC4_USIC1_CH1_BASE+XMC4_USIC_BYPCR_OFFSET)
#define XMC4_USCI11_TBCTR            (XMC4_USIC1_CH1_BASE+XMC4_USIC_TBCTR_OFFSET)
#define XMC4_USCI11_RBCTR            (XMC4_USIC1_CH1_BASE+XMC4_USIC_RBCTR_OFFSET)
#define XMC4_USCI11_TRBPTR           (XMC4_USIC1_CH1_BASE+XMC4_USIC_TRBPTR_OFFSET)
#define XMC4_USCI11_TRBSR            (XMC4_USIC1_CH1_BASE+XMC4_USIC_TRBSR_OFFSET)
#define XMC4_USCI11_TRBSCR           (XMC4_USIC1_CH1_BASE+XMC4_USIC_TRBSCR_OFFSET)
#define XMC4_USCI11_OUTR             (XMC4_USIC1_CH1_BASE+XMC4_USIC_OUTR_OFFSET)
#define XMC4_USCI11_OUTDR            (XMC4_USIC1_CH1_BASE+XMC4_USIC_OUTDR_OFFSET)
#define XMC4_USCI11_IN               (XMC4_USIC1_CH1_BASE+XMC4_USIC_IN_OFFSET)

/* USCI2 Registers */
/* Kernal Registers */

#define XMC4_USCI2_ID                (XMC4_USCI2_BASE+XMC4_USIC_ID_OFFSET)

/* USCI2 Channel 0 Registers */

#define XMC4_USCI20_CCFG             (XMC4_USCI2_CH0_BASE+XMC4_USIC_CCFG_OFFSET)
#define XMC4_USCI20_KSCFG            (XMC4_USCI2_CH0_BASE+XMC4_USIC_KSCFG_OFFSET)
#define XMC4_USCI20_FDR              (XMC4_USCI2_CH0_BASE+XMC4_USIC_FDR_OFFSET)
#define XMC4_USCI20_BRG              (XMC4_USCI2_CH0_BASE+XMC4_USIC_BRG_OFFSET)
#define XMC4_USCI20_INPR             (XMC4_USCI2_CH0_BASE+XMC4_USIC_INPR_OFFSET)
#define XMC4_USCI20_DX0CR            (XMC4_USCI2_CH0_BASE+XMC4_USIC_DX0CR_OFFSET)
#define XMC4_USCI20_DX1CR            (XMC4_USCI2_CH0_BASE+XMC4_USIC_DX1CR_OFFSET)
#define XMC4_USCI20_DX2CR            (XMC4_USCI2_CH0_BASE+XMC4_USIC_DX2CR_OFFSET)
#define XMC4_USCI20_DX3CR            (XMC4_USCI2_CH0_BASE+XMC4_USIC_DX3CR_OFFSET)
#define XMC4_USCI20_DX4CR            (XMC4_USCI2_CH0_BASE+XMC4_USIC_DX4CR_OFFSET)
#define XMC4_USCI20_DX5CR            (XMC4_USCI2_CH0_BASE+XMC4_USIC_DX5CR_OFFSET)
#define XMC4_USCI20_SCTR             (XMC4_USCI2_CH0_BASE+XMC4_USIC_SCTR_OFFSET)
#define XMC4_USCI20_TCSR             (XMC4_USCI2_CH0_BASE+XMC4_USIC_TCSR_OFFSET)
#define XMC4_USCI20_PCR              (XMC4_USCI2_CH0_BASE+XMC4_USIC_PCR_OFFSET)
#define XMC4_USCI20_CCR              (XMC4_USCI2_CH0_BASE+XMC4_USIC_CCR_OFFSET)
#define XMC4_USCI20_CMTR             (XMC4_USCI2_CH0_BASE+XMC4_USIC_CMTR_OFFSET)
#define XMC4_USCI20_PSR              (XMC4_USCI2_CH0_BASE+XMC4_USIC_PSR_OFFSET)
#define XMC4_USCI20_PSCR             (XMC4_USCI2_CH0_BASE+XMC4_USIC_PSCR_OFFSET)
#define XMC4_USCI20_RBUFSR           (XMC4_USCI2_CH0_BASE+XMC4_USIC_RBUFSR_OFFSET)
#define XMC4_USCI20_RBUF             (XMC4_USCI2_CH0_BASE+XMC4_USIC_RBUF_OFFSET)
#define XMC4_USCI20_RBUFD            (XMC4_USCI2_CH0_BASE+XMC4_USIC_RBUFD_OFFSET)
#define XMC4_USCI20_RBUF0            (XMC4_USCI2_CH0_BASE+XMC4_USIC_RBUF0_OFFSET)
#define XMC4_USCI20_RBUF1            (XMC4_USCI2_CH0_BASE+XMC4_USIC_RBUF1_OFFSET)
#define XMC4_USCI20_RBUF01SR         (XMC4_USCI2_CH0_BASE+XMC4_USIC_RBUF01SR_OFFSET)
#define XMC4_USCI20_FMR              (XMC4_USCI2_CH0_BASE+XMC4_USIC_FMR_OFFSET)
#define XMC4_USCI20_TBUF             (XMC4_USCI2_CH0_BASE+XMC4_USIC_TBUF_OFFSET)

/* USCI2 Channel 0 FIFO Registers */

#define XMC4_USCI20_BYP              (XMC4_USCI2_CH0_BASE+XMC4_USIC_BYP_OFFSET)
#define XMC4_USCI20_BYPCR            (XMC4_USCI2_CH0_BASE+XMC4_USIC_BYPCR_OFFSET)
#define XMC4_USCI20_TBCTR            (XMC4_USCI2_CH0_BASE+XMC4_USIC_TBCTR_OFFSET)
#define XMC4_USCI20_RBCTR            (XMC4_USCI2_CH0_BASE+XMC4_USIC_RBCTR_OFFSET)
#define XMC4_USCI20_TRBPTR           (XMC4_USCI2_CH0_BASE+XMC4_USIC_TRBPTR_OFFSET)
#define XMC4_USCI20_TRBSR            (XMC4_USCI2_CH0_BASE+XMC4_USIC_TRBSR_OFFSET)
#define XMC4_USCI20_TRBSCR           (XMC4_USCI2_CH0_BASE+XMC4_USIC_TRBSCR_OFFSET)
#define XMC4_USCI20_OUTR             (XMC4_USCI2_CH0_BASE+XMC4_USIC_OUTR_OFFSET)
#define XMC4_USCI20_OUTDR            (XMC4_USCI2_CH0_BASE+XMC4_USIC_OUTDR_OFFSET)
#define XMC4_USCI20_IN               (XMC4_USCI2_CH0_BASE+XMC4_USIC_IN_OFFSET)

/* USCI2 Channel 1 Registers */

#define XMC4_USCI21_CCFG             (XMC4_USCI2_CH1_BASE+XMC4_USIC_CCFG_OFFSET)
#define XMC4_USCI21_KSCFG            (XMC4_USCI2_CH1_BASE+XMC4_USIC_KSCFG_OFFSET)
#define XMC4_USCI21_FDR              (XMC4_USCI2_CH1_BASE+XMC4_USIC_FDR_OFFSET)
#define XMC4_USCI21_BRG              (XMC4_USCI2_CH1_BASE+XMC4_USIC_BRG_OFFSET)
#define XMC4_USCI21_INPR             (XMC4_USCI2_CH1_BASE+XMC4_USIC_INPR_OFFSET)
#define XMC4_USCI21_DX0CR            (XMC4_USCI2_CH1_BASE+XMC4_USIC_DX0CR_OFFSET)
#define XMC4_USCI21_DX1CR            (XMC4_USCI2_CH1_BASE+XMC4_USIC_DX1CR_OFFSET)
#define XMC4_USCI21_DX2CR            (XMC4_USCI2_CH1_BASE+XMC4_USIC_DX2CR_OFFSET)
#define XMC4_USCI21_DX3CR            (XMC4_USCI2_CH1_BASE+XMC4_USIC_DX3CR_OFFSET)
#define XMC4_USCI21_DX4CR            (XMC4_USCI2_CH1_BASE+XMC4_USIC_DX4CR_OFFSET)
#define XMC4_USCI21_DX5CR            (XMC4_USCI2_CH1_BASE+XMC4_USIC_DX5CR_OFFSET)
#define XMC4_USCI21_SCTR             (XMC4_USCI2_CH1_BASE+XMC4_USIC_SCTR_OFFSET)
#define XMC4_USCI21_TCSR             (XMC4_USCI2_CH1_BASE+XMC4_USIC_TCSR_OFFSET)
#define XMC4_USCI21_PCR              (XMC4_USCI2_CH1_BASE+XMC4_USIC_PCR_OFFSET)
#define XMC4_USCI21_CCR              (XMC4_USCI2_CH1_BASE+XMC4_USIC_CCR_OFFSET)
#define XMC4_USCI21_CMTR             (XMC4_USCI2_CH1_BASE+XMC4_USIC_CMTR_OFFSET)
#define XMC4_USCI21_PSR              (XMC4_USCI2_CH1_BASE+XMC4_USIC_PSR_OFFSET)
#define XMC4_USCI21_PSCR             (XMC4_USCI2_CH1_BASE+XMC4_USIC_PSCR_OFFSET)
#define XMC4_USCI21_RBUFSR           (XMC4_USCI2_CH1_BASE+XMC4_USIC_RBUFSR_OFFSET)
#define XMC4_USCI21_RBUF             (XMC4_USCI2_CH1_BASE+XMC4_USIC_RBUF_OFFSET)
#define XMC4_USCI21_RBUFD            (XMC4_USCI2_CH1_BASE+XMC4_USIC_RBUFD_OFFSET)
#define XMC4_USCI21_RBUF0            (XMC4_USCI2_CH1_BASE+XMC4_USIC_RBUF0_OFFSET)
#define XMC4_USCI21_RBUF1            (XMC4_USCI2_CH1_BASE+XMC4_USIC_RBUF1_OFFSET)
#define XMC4_USCI21_RBUF01SR         (XMC4_USCI2_CH1_BASE+XMC4_USIC_RBUF01SR_OFFSET)
#define XMC4_USCI21_FMR              (XMC4_USCI2_CH1_BASE+XMC4_USIC_FMR_OFFSET)
#define XMC4_USCI21_TBUF             (XMC4_USCI2_CH1_BASE+XMC4_USIC_TBUF_OFFSET)

/* USCI2 Channel 1 FIFO Registers */

#define XMC4_USCI21_BYP              (XMC4_USCI2_CH1_BASE+XMC4_USIC_BYP_OFFSET)
#define XMC4_USCI21_BYPCR            (XMC4_USCI2_CH1_BASE+XMC4_USIC_BYPCR_OFFSET)
#define XMC4_USCI21_TBCTR            (XMC4_USCI2_CH1_BASE+XMC4_USIC_TBCTR_OFFSET)
#define XMC4_USCI21_RBCTR            (XMC4_USCI2_CH1_BASE+XMC4_USIC_RBCTR_OFFSET)
#define XMC4_USCI21_TRBPTR           (XMC4_USCI2_CH1_BASE+XMC4_USIC_TRBPTR_OFFSET)
#define XMC4_USCI21_TRBSR            (XMC4_USCI2_CH1_BASE+XMC4_USIC_TRBSR_OFFSET)
#define XMC4_USCI21_TRBSCR           (XMC4_USCI2_CH1_BASE+XMC4_USIC_TRBSCR_OFFSET)
#define XMC4_USCI21_OUTR             (XMC4_USCI2_CH1_BASE+XMC4_USIC_OUTR_OFFSET)
#define XMC4_USCI21_OUTDR            (XMC4_USCI2_CH1_BASE+XMC4_USIC_OUTDR_OFFSET)
#define XMC4_USCI21_IN               (XMC4_USCI2_CH1_BASE+XMC4_USIC_IN_OFFSET)

/* Register Bit-Field Definitions **************************************************/

/* Kernal Registers */
/* Kernel State Configuration Register */

#define USIC_ID_MOD_REV_SHIFT       (0)       /* Bits 0-7: Module Revision Number */
#define USIC_ID_MOD_REV_MASK        (0xff << USIC_ID_MOD_REV_SHIFT)
#define USIC_ID_MOD_TYPE_SHIFT      (8)       /* Bits 8-15: Module Type */
#define USIC_ID_MOD_TYPE_MASK       (0xff << USIC_ID_MOD_REV_SHIFT)
#define USIC_ID_MOD_NUMBER_SHIFT    (16)      /* Bits 16-31: Module Number Value */
#define USIC_ID_MOD_NUMBER_MASK     (0xffff << USIC_ID_MOD_NUMBER_SHIFT)

/* USIC Channel Registers */

/* Channel Configuration Register */
#define USIC_CCFG_
/* Kernel State Configuration Register */
#define USIC_KSCFG_
/* Fractional Divider Register */
#define USIC_FDR_
/* Baud Rate Generator Register */
#define USIC_BRG_
/* Interrupt Node Pointer Register */
#define USIC_INPR_
/* Input Control Register 0 */
#define USIC_DX0CR_
/* Input Control Register 1 */
#define USIC_DX1CR_
/* Input Control Register 2 */
#define USIC_DX2CR_
/* Input Control Register 3 */
#define USIC_DX3CR_
/* Input Control Register 4 */
#define USIC_DX4CR_
/* Input Control Register 5 */
#define USIC_DX5CR_
/* Shift Control Register */
#define USIC_SCTR_
/* Transmit Control/Status Register */
#define USIC_TCSR_
/* Protocol Control Register */
#define USIC_PCR_
/* Channel Control Register */
#define USIC_CCR_
/* Capture Mode Timer Register */
#define USIC_CMTR_
/* Protocol Status Register */
#define USIC_PSR_
/* Protocol Status Clear Register */
#define USIC_PSCR_
/* Receiver Buffer Status Register */
#define USIC_RBUFSR_
/* Receiver Buffer Register */
#define USIC_RBUF_
/* Receiver Buffer Register for Debugger */
#define USIC_RBUFD_
/* Receiver Buffer Register 0 */
#define USIC_RBUF0_
/* Receiver Buffer Register 1 */
#define USIC_RBUF1_
/*  Receiver Buffer 01 Status Register */
#define USIC_RBUF01SR_
/* Flag Modification Register */
#define USIC_FMR_
/* Transmit Buffer (32 x 4-bytes) */
#define USIC_TBUF_

/* USIC FIFO Registers */

/* Bypass Data Register */
#define USIC_BYP_
/* Bypass Control Register */
#define USIC_BYPCR_
/* Transmitter Buffer Control Register */
#define USIC_TBCTR_
/* Receiver Buffer Control Register */
#define USIC_RBCTR_
/* Transmit/Receive Buffer Pointer Register */
#define USIC_TRBPTR_
/* Transmit/Receive Buffer Status Register */
#define USIC_TRBSR_
/* Transmit/Receive Buffer Status Clear Register */
#define USIC_TRBSCR_
/* Receiver Buffer Output Register */
#define USIC_OUTR_
/* Receiver Buffer Output Register L for Debugger */
#define USIC_OUTDR_
/* Transmit FIFO Buffer (32 x 4-bytes) */
#define USIC_IN_

#endif /* __ARCH_ARM_SRC_XMC4_CHIP_XMC4_USIC_H */
