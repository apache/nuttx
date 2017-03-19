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

/* Kernel Registers */

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

#define USIC_CCFG_SSC               (1 << 0)  /* Bit 0:   */
#define USIC_CCFG_ASC               (1 << 1)  /* Bit 1:   */
#define USIC_CCFG_IIC               (1 << 2)  /* Bit 2:   */
#define USIC_CCFG_IIS               (1 << 3)  /* Bit 3:   */
#define USIC_CCFG_RB                (1 << 6)  /* Bit 6:   */
#define USIC_CCFG_TB                (1 << 7)  /* Bit 7:   */

/* Kernel State Configuration Register */

#define USIC_KSCFG_MODEN            (1 << 0)  /* Bit 0:   */
#define USIC_KSCFG_BPMODEN          (1 << 1)  /* Bit 1:   */
#define USIC_KSCFG_NOMCFG_SHIFT     (4)       /* Bits 4-5:  */
#define USIC_KSCFG_NOMCFG_MASK      (3 << USIC_KSCFG_NOMCFG_SHIFT)
#define USIC_KSCFG_BPNOM            (1 << 7)  /* Bit 7:   */
#define USIC_KSCFG_SUMCFG_SHIFT     (8)       /* Bits 8-9:  */
#define USIC_KSCFG_SUMCFG_MASK      (3 << USIC_KSCFG_SUMCFG_SHIFT)
#define USIC_KSCFG_BPSUM            (1 << 11) /* Bit 11:   */

/* Fractional Divider Register */

#define USIC_FDR_STEP_SHIFT         (0)       /* Bits 0-9:  */
#define USIC_FDR_STEP_MASK          (0x3ff << USIC_FDR_STEP_SHIFT)
#define USIC_FDR_DM_SHIFT           (14)      /* Bits 14-15:  */
#define USIC_FDR_DM_MASK            (3 << USIC_FDR_DM_SHIFT)
#define USIC_FDR_RESULT_SHIFT       (16)      /* Bits 16-25:  */
#define USIC_FDR_RESULT_MASK        (0x3ff << USIC_FDR_RESULT_SHIFT)

/* Baud Rate Generator Register */

#define USIC_BRG_CLKSEL_SHIFT       (0)       /* Bits 0-1:  */
#define USIC_BRG_CLKSEL_MASK        (3 << USIC_BRG_CLKSEL_SHIFT)
#define USIC_BRG_TMEN               (1 << 3)  /* Bit 3:   */
#define USIC_BRG_PPPEN              (1 << 4)  /* Bit 4:   */
#define USIC_BRG_CTQSEL_SHIFT       (6)       /* Bits 6-7:  */
#define USIC_BRG_CTQSEL_MASK        (3 << USIC_BRG_CTQSEL_SHIFT)
#define USIC_BRG_PCTQ_SHIFT         (8)       /* Bits 8-9:  */
#define USIC_BRG_PCTQ_MASK          (3 << USIC_BRG_PCTQ_SHIFT)
#define USIC_BRG_DCTQ_SHIFT         (10)      /* Bits 10-15:  */
#define USIC_BRG_DCTQ_MASK          (0x3f << USIC_BRG_DCTQ_SHIFT)
#define USIC_BRG_PDIV_SHIFT         (16)      /* Bits 16-25:  */
#define USIC_BRG_PDIV_MASK          (0x3ff << USIC_BRG_PDIV_SHIFT)
#define USIC_BRG_SCLKOSEL           (1 << 28) /* Bit 28:   */
#define USIC_BRG_MCLKCFG            (1 << 29) /* Bit 29:   */
#define USIC_BRG_SCLKCFG            (1 << 30) /* Bit 30:   */

/* Interrupt Node Pointer Register */

#define USIC_INPR_TSINP_SHIFT       (0)       /* Bits 0-2:  */
#define USIC_INPR_TSINP_MASK        (7 << USIC_INPR_TSINP_SHIFT)
#define USIC_INPR_TBINP_SHIFT       (4)       /* Bits 4-6:  */
#define USIC_INPR_TBINP_MASK        (7 << USIC_INPR_TBINP_SHIFT)
#define USIC_INPR_RINP_SHIFT        (8)       /* Bits 8-10:  */
#define USIC_INPR_RINP_MASK         (7 << USIC_INPR_RINP_SHIFT)
#define USIC_INPR_AINP_SHIFT        (12)      /* Bits 12-14:  */
#define USIC_INPR_AINP_MASK         (7 << USIC_INPR_AINP_SHIFT)
#define USIC_INPR_PINP_SHIFT        (16)      /* Bits 16-18:  */
#define USIC_INPR_PINP_MASK         (7 << USIC_INPR_PINP_SHIFT)

/* Input Control Register 0, Input Control Register 1, Input Control Register 2,
 * Input Control Register 3, Input Control Register 4, Input Control Register 5
 */

#define USIC_DXCR_DSEL_SHIFT        (0)      /* Bits 0-2:  */
#define USIC_DXCR_DSEL_MASK         (7 << USIC_DX0CR_DSEL_SHIFT)
#define USIC_DX1CR_DCEN             (1 << 3)  /* Bit 3:   (DX1CR only) */
#define USIC_DXCR_INSW              (1 << 4)  /* Bit 4:   */
#define USIC_DXCR_DFEN              (1 << 5)  /* Bit 5:   */
#define USIC_DXCR_DSEN              (1 << 6)  /* Bit 6:   */
#define USIC_DXCR_DPOL              (1 << 8)  /* Bit 8:   */
#define USIC_DXCR_SFSEL             (1 << 9)  /* Bit 9:   */
#define USIC_DXCR_CM_SHIFT          (10)      /* Bits 10-12:  */
#define USIC_DXCR_CM_MASK           (3 << USIC_DX0CR_CM_SHIFT)
#define USIC_DXCR_DXS               (1 << 15) /* Bit 15:   */

/* Shift Control Register */

#define USIC_SCTR_SDIR              (1 << 0)  /* Bit 0:   */
#define USIC_SCTR_PDL               (1 << 1)  /* Bit `:   */
#define USIC_SCTR_DSM_SHIFT         (2)       /* Bits 2-3:  */
#define USIC_SCTR_DSM_MASK          (3 << USIC_SCTR_DSM_SHIFT)
#define USIC_SCTR_HPCDIR            (1 << 4)  /* Bit 4:   */
#define USIC_SCTR_DOCFG_SHIFT       (6)       /* Bits 6-7:  */
#define USIC_SCTR_DOCFG_MASK        (3 << USIC_SCTR_DOCFG_SHIFT)
#define USIC_SCTR_TRM_SHIFT         (8)       /* Bits 8-9:  */
#define USIC_SCTR_TRM_MASK          (3 << USIC_SCTR_TRM_SHIFT)
#define USIC_SCTR_FLE_SHIFT         (16)      /* Bits 16-21:  */
#define USIC_SCTR_FLE_MASK          (0x3f << USIC_SCTR_FLE_SHIFT)
#define USIC_SCTR_WLE_SHIFT         (24)      /* Bits 24-27:  */
#define USIC_SCTR_WLE_MASK          (15 << USIC_SCTR_WLE_SHIFT)

/* Transmit Control/Status Register */

#define USIC_TCSR_WLEMD             (1 << 0)  /* Bit 0:   */
#define USIC_TCSR_SELMD             (1 << 1)  /* Bit 1:   */
#define USIC_TCSR_FLEMD             (1 << 2)  /* Bit 2:   */
#define USIC_TCSR_WAMD              (1 << 3)  /* Bit 3:   */
#define USIC_TCSR_HPCMD             (1 << 4)  /* Bit 4:   */
#define USIC_TCSR_SOF               (1 << 5)  /* Bit 5:   */
#define USIC_TCSR_EOF               (1 << 6)  /* Bit 6:   */
#define USIC_TCSR_TDV               (1 << 7)  /* Bit 7:   */
#define USIC_TCSR_TDSSM             (1 << 8)  /* Bit 8:   */
#define USIC_TCSR_TDEN_SHIFT        (10)      /* Bits 10-11:  */
#define USIC_TCSR_TDEN_MASK         (3 << USIC_TCSR_TDEN_SHIFT)
#define USIC_TCSR_TDVTR             (1 << 12) /* Bit 12:  */
#define USIC_TCSR_WA                (1 << 13) /* Bit 13:  */
#define USIC_TCSR_TSOF              (1 << 24) /* Bit 24:  */
#define USIC_TCSR_TV                (1 << 26) /* Bit 26:  */
#define USIC_TCSR_TVC               (1 << 27) /* Bit 27:  */
#define USIC_TCSR_TE                (1 << 28) /* Bit 28:  */

/* Protocol Control Register */

#define USIC_PCR_CTR0               (1 << 0)  /* Bit 0:   */
#define USIC_PCR_CTR1               (1 << 1)  /* Bit 1:   */
#define USIC_PCR_CTR2               (1 << 2)  /* Bit 2:   */
#define USIC_PCR_CTR3               (1 << 3)  /* Bit 3:   */
#define USIC_PCR_CTR4               (1 << 4)  /* Bit 4:   */
#define USIC_PCR_CTR5               (1 << 5)  /* Bit 5:   */
#define USIC_PCR_CTR6               (1 << 6)  /* Bit 6:   */
#define USIC_PCR_CTR7               (1 << 7)  /* Bit 7:   */
#define USIC_PCR_CTR8               (1 << 8)  /* Bit 8:   */
#define USIC_PCR_CTR9               (1 << 9)  /* Bit 9:   */
#define USIC_PCR_CTR10              (1 << 10) /* Bit 10:  */
#define USIC_PCR_CTR11              (1 << 11) /* Bit 11:  */
#define USIC_PCR_CTR12              (1 << 12) /* Bit 12:  */
#define USIC_PCR_CTR13              (1 << 13) /* Bit 13:  */
#define USIC_PCR_CTR14              (1 << 14) /* Bit 14:  */
#define USIC_PCR_CTR15              (1 << 15) /* Bit 15:  */
#define USIC_PCR_CTR16              (1 << 16) /* Bit 16:  */
#define USIC_PCR_CTR17              (1 << 17) /* Bit 17:  */
#define USIC_PCR_CTR18              (1 << 18) /* Bit 18:  */
#define USIC_PCR_CTR19              (1 << 19) /* Bit 19:  */
#define USIC_PCR_CTR20              (1 << 20) /* Bit 20:  */
#define USIC_PCR_CTR21              (1 << 21) /* Bit 21:  */
#define USIC_PCR_CTR22              (1 << 22) /* Bit 22:  */
#define USIC_PCR_CTR23              (1 << 23) /* Bit 23:  */
#define USIC_PCR_CTR24              (1 << 24) /* Bit 24:  */
#define USIC_PCR_CTR25              (1 << 25) /* Bit 25:  */
#define USIC_PCR_CTR26              (1 << 26) /* Bit 26:  */
#define USIC_PCR_CTR27              (1 << 27) /* Bit 27:  */
#define USIC_PCR_CTR28              (1 << 28) /* Bit 28:  */
#define USIC_PCR_CTR29              (1 << 29) /* Bit 29:  */
#define USIC_PCR_CTR30              (1 << 30) /* Bit 30:  */
#define USIC_PCR_CTR31              (1 << 31) /* Bit 31:  */

#define USIC_PCR_ASCMODE_SMD        (1 << 0)  /* Bit 0:   */
#define USIC_PCR_ASCMODE_STPB       (1 << 1)  /* Bit 1:   */
#define USIC_PCR_ASCMODE_IDM        (1 << 2)  /* Bit 2:   */
#define USIC_PCR_ASCMODE_SBIEN      (1 << 3)  /* Bit 3:   */
#define USIC_PCR_ASCMODE_CDEN       (1 << 4)  /* Bit 4:   */
#define USIC_PCR_ASCMODE_RNIEN      (1 << 5)  /* Bit 5:   */
#define USIC_PCR_ASCMODE_FEIEN      (1 << 6)  /* Bit 6:   */
#define USIC_PCR_ASCMODE_FFIEN      (1 << 7)  /* Bit 7:   */
#define USIC_PCR_ASCMODE_SP_SHIFT   (8)       /* Bits 8-12:  */
#define USIC_PCR_ASCMODE_SP_MASK    (31 << USIC_PCR_ASCMODE_SP_SHIFT)
#define USIC_PCR_ASCMODE_PL_SHIFT   (13)      /* Bits 13-15:  */
#define USIC_PCR_ASCMODE_PL_MASK    (7 << USIC_PCR_ASCMODE_PL_SHIFT)
#define USIC_PCR_ASCMODE_RSTEN      (1 << 16) /* Bit 16:  */
#define USIC_PCR_ASCMODE_TSTEN      (1 << 17) /* Bit 16:  */
#define USIC_PCR_ASCMODE_MCLK       (1 << 31) /* Bit 31:  */

#define USIC_PCR_SSCMODE_MSLSEN     (1 << 0)  /* Bit 0:   */
#define USIC_PCR_SSCMODE_SELCTR     (1 << 1)  /* Bit 1:   */
#define USIC_PCR_SSCMODE_SELINV     (1 << 2)  /* Bit 2:   */
#define USIC_PCR_SSCMODE_FEM        (1 << 3)  /* Bit 3:   */
#define USIC_PCR_SSCMODE_CTQSEL1_SHIFT (4)    /* Bits 4-5:  */
#define USIC_PCR_SSCMODE_CTQSEL1_MASK  (3 << USIC_PCR_SSCMODE_CTQSEL1_SHIFT)
#define USIC_PCR_SSCMODE_PCTQ1_SHIFT   (6)    /* Bits 6-7:  */
#define USIC_PCR_SSCMODE_PCTQ1_MASK    (3 << USIC_PCR_SSCMODE_PCTQ1_SHIFT)
#define USIC_PCR_SSCMODE_DCTQ1_SHIFT   (8)    /* Bits 8-12:  */
#define USIC_PCR_SSCMODE_DCTQ1_MASK    (0x1f << USIC_PCR_SSCMODE_DCTQ1_SHIFT)
#define USIC_PCR_SSCMODE_PARIEN     (1 << 13) /* Bit 13:  */
#define USIC_PCR_SSCMODE_MSLSIEN    (1 << 14) /* Bit 14:  */
#define USIC_PCR_SSCMODE_DX2TIEN    (1 << 15) /* Bit 15:  */
#define USIC_PCR_SSCMODE_SELO_SHIFT (16)      /* Bits 16-23:  */
#define USIC_PCR_SSCMODE_SELO_MASK  (0xff << USIC_PCR_SSCMODE_SELO_SHIFT)
#define USIC_PCR_SSCMODE_TIWEN      (1 << 24) /* Bit 24:  */
#define USIC_PCR_SSCMODE_SLPHSEL    (1 << 25) /* Bit 25:  */
#define USIC_PCR_SSCMODE_MCLK       (1 << 31) /* Bit 31:  */

#define USIC_PCR_IICMODE_SLAD_SHIFT (0)       /* Bits 0-15:  */
#define USIC_PCR_IICMODE_SLAD_MASK  (0xffff << USIC_PCR_IICMODE_SLAD_SHIFT)
#define USIC_PCR_IICMODE_ACK00      (1 << 16) /* Bit 16:  */
#define USIC_PCR_IICMODE_STIM       (1 << 17) /* Bit 17:  */
#define USIC_PCR_IICMODE_SCRIEN     (1 << 18) /* Bit 18:  */
#define USIC_PCR_IICMODE_RSCRIEN    (1 << 19) /* Bit 19:  */
#define USIC_PCR_IICMODE_PCRIEN     (1 << 20) /* Bit 20:  */
#define USIC_PCR_IICMODE_NACKIEN    (1 << 21) /* Bit 21:  */
#define USIC_PCR_IICMODE_ARLIEN     (1 << 22) /* Bit 22:  */
#define USIC_PCR_IICMODE_SRRIEN     (1 << 23) /* Bit 23:  */
#define USIC_PCR_IICMODE_ERRIEN     (1 << 24) /* Bit 24:  */
#define USIC_PCR_IICMODE_SACKDIS    (1 << 25) /* Bit 25:  */
#define USIC_PCR_IICMODE_HDEL_SHIFT (26)      /* Bits 26-29:  */
#define USIC_PCR_IICMODE_HDEL_MASK  (15 << USIC_PCR_IICMODE_HDEL_SHIFT)
#define USIC_PCR_IICMODE_ACKIEN     (1 << 30) /* Bit 30:  */
#define USIC_PCR_IICMODE_MCLK       (1 << 31) /* Bit 31:  */

#define USIC_PCR_IISMODE_WAGEN      (1 << 0)  /* Bit 0:   */
#define USIC_PCR_IISMODE_DTEN       (1 << 1)  /* Bit 1:   */
#define USIC_PCR_IISMODE_SELINV     (1 << 2)  /* Bit 2:   */
#define USIC_PCR_IISMODE_WAFEIEN    (1 << 4)  /* Bit 4:   */
#define USIC_PCR_IISMODE_WAREIEN    (1 << 5)  /* Bit 5:   */
#define USIC_PCR_IISMODE_ENDIEN     (1 << 6)  /* Bit 6:   */
#define USIC_PCR_IISMODE_TDEL_SHIFT (16)      /* Bits 15-21:  */
#define USIC_PCR_IISMODE_TDEL_MASK  (0x3f << USIC_PCR_IISMODE_TDEL_SHIFT)
#define USIC_PCR_IISMODE_MCLK       (1 << 31) /* Bit 31:  */

/* Channel Control Register */

#define USIC_CCR_MODE_SHIFT         (0)       /* Bits 0-3:  */
#define USIC_CCR_MODE_MASK          (15 << USIC_CCR_MODE_SHIFT)
#define USIC_CCR_HPCEN_SHIFT        (6)       /* Bits 6-7:  */
#define USIC_CCR_HPCEN_MASK         (3 << USIC_CCR_HPCEN_SHIFT)
#define USIC_CCR_PM_SHIFT           (8)       /* Bits 8-9:  */
#define USIC_CCR_PM_MASK            (3 << USIC_CCR_PM_SHIFT)
#define USIC_CCR_RSIEN              (1 << 10) /* Bit 10:  */
#define USIC_CCR_DLIEN              (1 << 11) /* Bit 11:  */
#define USIC_CCR_TSIEN              (1 << 12) /* Bit 12:  */
#define USIC_CCR_TBIEN              (1 << 13) /* Bit 13:  */
#define USIC_CCR_RIEN               (1 << 14) /* Bit 14:  */
#define USIC_CCR_AIEN               (1 << 15) /* Bit 15:  */
#define USIC_CCR_BRGIEN             (1 << 16) /* Bit 16:  */

/* Capture Mode Timer Register */

#define USIC_CMTR_CTV_SHIFT         (0)      /* Bits 0-9:  */
#define USIC_CMTR_CTV_MASK          (0x3ff << USIC_CMTR_CTV_SHIFT)

/* Protocol Status Register */

#define USIC_PSR_ST0                (1 << 0)  /* Bit 0:   */
#define USIC_PSR_ST1                (1 << 1)  /* Bit 1:   */
#define USIC_PSR_ST2                (1 << 2)  /* Bit 2:   */
#define USIC_PSR_ST3                (1 << 3)  /* Bit 3:   */
#define USIC_PSR_ST4                (1 << 4)  /* Bit 4:   */
#define USIC_PSR_ST5                (1 << 5)  /* Bit 5:   */
#define USIC_PSR_ST6                (1 << 6)  /* Bit 6:   */
#define USIC_PSR_ST7                (1 << 7)  /* Bit 7:   */
#define USIC_PSR_ST8                (1 << 8)  /* Bit 8:   */
#define USIC_PSR_ST9                (1 << 9)  /* Bit 9:   */
#define USIC_PSR_RSIF               (1 << 10) /* Bit 10:  */
#define USIC_PSR_DLIF               (1 << 11) /* Bit 11:  */
#define USIC_PSR_TSIF               (1 << 12) /* Bit 12:  */
#define USIC_PSR_TBIF               (1 << 13) /* Bit 13:  */
#define USIC_PSR_RIF                (1 << 14) /* Bit 14:  */
#define USIC_PSR_AIF                (1 << 15) /* Bit 15:  */
#define USIC_PSR_BRGIF              (1 << 16) /* Bit 16:  */

#define USIC_PSR_ASCMODE_TXIDLE     (1 << 0)  /* Bit 0:   */
#define USIC_PSR_ASCMODE_RXIDLE     (1 << 1)  /* Bit 1:   */
#define USIC_PSR_ASCMODE_SBD        (1 << 2)  /* Bit 2:   */
#define USIC_PSR_ASCMODE_COL        (1 << 3)  /* Bit 3:   */
#define USIC_PSR_ASCMODE_RNS        (1 << 4)  /* Bit 4:   */
#define USIC_PSR_ASCMODE_FER0       (1 << 5)  /* Bit 5:   */
#define USIC_PSR_ASCMODE_FER1       (1 << 6)  /* Bit 6:   */
#define USIC_PSR_ASCMODE_RFF        (1 << 7)  /* Bit 7:   */
#define USIC_PSR_ASCMODE_TFF        (1 << 8)  /* Bit 8:   */
#define USIC_PSR_ASCMODE_BUSY       (1 << 9)  /* Bit 9:   */
#define USIC_PSR_ASCMODE_RSIF       (1 << 10) /* Bit 10:  */
#define USIC_PSR_ASCMODE_DLIF       (1 << 11) /* Bit 11:  */
#define USIC_PSR_ASCMODE_TSIF       (1 << 12) /* Bit 12:  */
#define USIC_PSR_ASCMODE_TBIF       (1 << 13) /* Bit 13:  */
#define USIC_PSR_ASCMODE_RIF        (1 << 14) /* Bit 14:  */
#define USIC_PSR_ASCMODE_AIF        (1 << 15) /* Bit 15:  */
#define USIC_PSR_ASCMODE_BRGIF      (1 << 16) /* Bit 16:  */

#define USIC_PSR_SSCMODE_MSLS       (1 << 0)  /* Bit 0:   */
#define USIC_PSR_SSCMODE_DX2S       (1 << 1)  /* Bit 1:   */
#define USIC_PSR_SSCMODE_MSLSEV     (1 << 2)  /* Bit 2:   */
#define USIC_PSR_SSCMODE_DX2TEV     (1 << 3)  /* Bit 3:   */
#define USIC_PSR_SSCMODE_PARERR     (1 << 4)  /* Bit 4:   */
#define USIC_PSR_SSCMODE_RSIF       (1 << 10) /* Bit 10:  */
#define USIC_PSR_SSCMODE_DLIF       (1 << 11) /* Bit 11:  */
#define USIC_PSR_SSCMODE_TSIF       (1 << 12) /* Bit 12:  */
#define USIC_PSR_SSCMODE_TBIF       (1 << 13) /* Bit 13:  */
#define USIC_PSR_SSCMODE_RIF        (1 << 14) /* Bit 14:  */
#define USIC_PSR_SSCMODE_AIF        (1 << 15) /* Bit 15:  */
#define USIC_PSR_SSCMODE_BRGIF      (1 << 16) /* Bit 16:  */

#define USIC_PSR_IICMODE_SLSEL      (1 << 0)  /* Bit 0:   */
#define USIC_PSR_IICMODE_WTDF       (1 << 1)  /* Bit 1:   */
#define USIC_PSR_IICMODE_SCR        (1 << 2)  /* Bit 2:   */
#define USIC_PSR_IICMODE_RSCR       (1 << 3)  /* Bit 3:   */
#define USIC_PSR_IICMODE_PCR        (1 << 4)  /* Bit 4:   */
#define USIC_PSR_IICMODE_NACK       (1 << 5)  /* Bit 5:   */
#define USIC_PSR_IICMODE_ARL        (1 << 6)  /* Bit 6:   */
#define USIC_PSR_IICMODE_SRR        (1 << 7)  /* Bit 7:   */
#define USIC_PSR_IICMODE_ERR        (1 << 8)  /* Bit 8:   */
#define USIC_PSR_IICMODE_ACK        (1 << 9)  /* Bit 9:   */
#define USIC_PSR_IICMODE_RSIF       (1 << 10) /* Bit 10:  */
#define USIC_PSR_IICMODE_DLIF       (1 << 11) /* Bit 11:  */
#define USIC_PSR_IICMODE_TSIF       (1 << 12) /* Bit 12:  */
#define USIC_PSR_IICMODE_TBIF       (1 << 13) /* Bit 13:  */
#define USIC_PSR_IICMODE_RIF        (1 << 14) /* Bit 14:  */
#define USIC_PSR_IICMODE_AIF        (1 << 15) /* Bit 15:  */
#define USIC_PSR_IICMODE_BRGIF      (1 << 16) /* Bit 16:  */

#define USIC_PSR_IISMODE_WA         (1 << 0)  /* Bit 0:   */
#define USIC_PSR_IISMODE_DX2S       (1 << 1)  /* Bit 1:   */
#define USIC_PSR_IISMODE_DX2TEV     (1 << 3)  /* Bit 3:   */
#define USIC_PSR_IISMODE_WAFE       (1 << 4)  /* Bit 4:   */
#define USIC_PSR_IISMODE_WARE       (1 << 5)  /* Bit 5:   */
#define USIC_PSR_IISMODE_END        (1 << 6)  /* Bit 6:   */
#define USIC_PSR_IISMODE_RSIF       (1 << 10) /* Bit 10:  */
#define USIC_PSR_IISMODE_DLIF       (1 << 11) /* Bit 11:  */
#define USIC_PSR_IISMODE_TSIF       (1 << 12) /* Bit 12:  */
#define USIC_PSR_IISMODE_TBIF       (1 << 13) /* Bit 13:  */
#define USIC_PSR_IISMODE_RIF        (1 << 14) /* Bit 14:  */
#define USIC_PSR_IISMODE_AIF        (1 << 15) /* Bit 15:  */
#define USIC_PSR_IISMODE_BRGIF      (1 << 16) /* Bit 16:  */

/* Protocol Status Clear Register */

#define USIC_PSCR_CST0              (1 << 0)  /* Bit 0:   */
#define USIC_PSCR_CST1              (1 << 1)  /* Bit 1:   */
#define USIC_PSCR_CST2              (1 << 2)  /* Bit 2:   */
#define USIC_PSCR_CST3              (1 << 3)  /* Bit 3:   */
#define USIC_PSCR_CST4              (1 << 4)  /* Bit 4:   */
#define USIC_PSCR_CST5              (1 << 5)  /* Bit 5:   */
#define USIC_PSCR_CST6              (1 << 6)  /* Bit 6:   */
#define USIC_PSCR_CST7              (1 << 7)  /* Bit 7:   */
#define USIC_PSCR_CST8              (1 << 8)  /* Bit 8:   */
#define USIC_PSCR_CST9              (1 << 9)  /* Bit 9:   */
#define USIC_PSCR_CRSIF             (1 << 10) /* Bit 10:  */
#define USIC_PSCR_CDLIF             (1 << 11) /* Bit 11:  */
#define USIC_PSCR_CTSIF             (1 << 12) /* Bit 12:  */
#define USIC_PSCR_CTBIF             (1 << 13) /* Bit 13:  */
#define USIC_PSCR_CRIF              (1 << 14) /* Bit 14:  */
#define USIC_PSCR_CAIF              (1 << 15) /* Bit 15:  */
#define USIC_PSCR_CBRGIF            (1 << 16) /* Bit 16:  */

/* Receiver Buffer Status Register */

#define USIC_RBUFSR_WLEN_SHIFT      (0)       /* Bits 0-3:  */
#define USIC_RBUFSR_WLEN_MASK       (15 << USIC_RBUFSR_WLEN_SHIFT)
#define USIC_RBUFSR_SOF             (1 << 6)  /* Bit 6:   */
#define USIC_RBUFSR_PAR             (1 << 8)  /* Bit 8:   */
#define USIC_RBUFSR_PERR            (1 << 9)  /* Bit 9:   */
#define USIC_RBUFSR_RDV0            (1 << 13) /* Bit 13:  */
#define USIC_RBUFSR_RDV1            (1 << 14) /* Bit 14:  */
#define USIC_RBUFSR_DS              (1 << 15) /* Bit 15:  */

/* Receiver Buffer Register */

#define USIC_RBUF_DSR_SHIFT         (0)       /* Bits 0-15:  */
#define USIC_RBUF_DSR_MASK          (0xffff << USIC_RBUF_DSR_SHIFT)

/* Receiver Buffer Register for Debugger */

#define USIC_RBUFD_DSR_SHIFT        (0)       /* Bits 0-15:  */
#define USIC_RBUFD_DSR_MASK         (0xffff << USIC_RBUFD_DSR_SHIFT)

/* Receiver Buffer Register 0 */

#define USIC_RBUF0_DSR0_SHIFT       (0)       /* Bits 0-15:  */
#define USIC_RBUF0_DSR0_MASK        (0xffff << USIC_RBUF0_DSR0_SHIFT)

/* Receiver Buffer Register 1 */

#define USIC_RBUF1_DSR1_SHIFT       (0)       /* Bits 0-15:  */
#define USIC_RBUF1_DSR1_MASK        (0xffff << USIC_RBUF1_DSR1_SHIFT)

/*  Receiver Buffer 01 Status Register */

#define USIC_RBUF01SR_WLEN0_SHIFT   (0)       /* Bits 0-3:  */
#define USIC_RBUF01SR_WLEN0_MASK    (15 << USIC_RBUF01SR_WLEN0_SHIFT)
#define USIC_RBUF01SR_SOF0          (1 << 6)  /* Bit 6:   */
#define USIC_RBUF01SR_PAR0          (1 << 8)  /* Bit 8:   */
#define USIC_RBUF01SR_PERR0         (1 << 9)  /* Bit 9:   */
#define USIC_RBUF01SR_RDV00         (1 << 13) /* Bit 13:  */
#define USIC_RBUF01SR_RDV01         (1 << 14) /* Bit 14:  */
#define USIC_RBUF01SR_DS0           (1 << 15) /* Bit 15:  */
#define USIC_RBUF01SR_WLEN1_SHIFT   (16)      /* Bits 16-19:  */
#define USIC_RBUF01SR_WLEN1_MASK    (15 << USIC_RBUF01SR_WLEN1_SHIFT)
#define USIC_RBUF01SR_SOF1          (1 << 22) /* Bit 22:  */
#define USIC_RBUF01SR_PAR1          (1 << 24) /* Bit 24:  */
#define USIC_RBUF01SR_PERR1         (1 << 25) /* Bit 25:  */
#define USIC_RBUF01SR_RDV10         (1 << 29) /* Bit 29:  */
#define USIC_RBUF01SR_RDV11         (1 << 30) /* Bit 30:  */
#define USIC_RBUF01SR_DS1           (1 << 31) /* Bit 31:  */

/* Flag Modification Register */

#define USIC_FMR_MTDV_SHIFT         (0)       /* Bits 0-1:  */
#define USIC_FMR_MTDV_MASK          (3 << USIC_FMR_MTDV_SHIFT)
#define USIC_FMR_ATVC               (1 << 4)  /* Bit 4:   */
#define USIC_FMR_CRDV0              (1 << 14) /* Bit 14:  */
#define USIC_FMR_CRDV1              (1 << 15) /* Bit 15:  */
#define USIC_FMR_SIO0               (1 << 16) /* Bit 16:  */
#define USIC_FMR_SIO1               (1 << 17) /* Bit 17:  */
#define USIC_FMR_SIO2               (1 << 18) /* Bit 18:  */
#define USIC_FMR_SIO3               (1 << 19) /* Bit 19:  */
#define USIC_FMR_SIO4               (1 << 20) /* Bit 20:  */
#define USIC_FMR_SIO5               (1 << 21) /* Bit 21:  */

/* Transmit Buffer (32 x 4-bytes) */

#define USIC_TBUF_TDATA_SHIFT       (0)       /* Bits 0-15:  */
#define USIC_TBUF_TDATA_MASK        (0xffff << USIC_TBUF_TDATA_SHIFT)

/* USIC FIFO Registers */

/* Bypass Data Register */

#define USIC_BYP_BDATA_SHIFT        (0)       /* Bits 0-15:  */
#define USIC_BYP_BDATA_MASK         (0xffff << USIC_BYP_BDATA_SHIFT)

/* Bypass Control Register */

#define USIC_BYPCR_BWLE_SHIFT       (0)       /* Bits 0-3:  */
#define USIC_BYPCR_BWLE_MASK        (15 << USIC_BYPCR_BWLE_SHIFT)
#define USIC_BYPCR_BDSSM            (1 << 8)  /* Bit 8:   */
#define USIC_BYPCR_BDEN_SHIFT       (10)      /* Bits 10-11:  */
#define USIC_BYPCR_BDEN_MASK        (3 << USIC_BYPCR_BDEN_SHIFT)
#define USIC_BYPCR_BDVTR            (1 << 12) /* Bit 12:  */
#define USIC_BYPCR_BPRIO            (1 << 13) /* Bit 13:  */
#define USIC_BYPCR_BDV              (1 << 15) /* Bit 15:  */
#define USIC_BYPCR_BSELO_SHIFT      (16)      /* Bits 16-20:  */
#define USIC_BYPCR_BSELO_MASK       (31 << USIC_BYPCR_BSELO_SHIFT)
#define USIC_BYPCR_BHPC_SHIFT       (21)      /* Bits 21-23:  */
#define USIC_BYPCR_BHPC_MASK        (7 << USIC_BYPCR_BHPC_SHIFT)

/* Transmitter Buffer Control Register */

#define USIC_TBCTR_DPTR_SHIFT       (0)       /* Bits 0-1:  */
#define USIC_TBCTR_DPTR_MASK        (3 << USIC_TBCTR_DPTR_SHIFT)
#define USIC_TBCTR_LIMIT_SHIFT      (8)       /* Bits 8-13:  */
#define USIC_TBCTR_LIMIT_MASK       (0x3f << USIC_TBCTR_LIMIT_SHIFT)
#define USIC_TBCTR_STBTM            (1 << 14) /* Bit 14:  */
#define USIC_TBCTR_STBTEN           (1 << 15) /* Bit 15:  */
#define USIC_TBCTR_STBINP_SHIFT     (16)      /* Bits 16-18:  */
#define USIC_TBCTR_STBINP_MASK      (7 << USIC_TBCTR_STBINP_SHIFT)
#define USIC_TBCTR_ATBINP_SHIFT     (19)      /* Bits 19-21:  */
#define USIC_TBCTR_ATBINP_MASK      (7 << USIC_TBCTR_ATBINP_SHIFT)
#define USIC_TBCTR_SIZE_SHIFT       (24)      /* Bits 24-26:  */
#define USIC_TBCTR_SIZE_MASK        (7 << USIC_TBCTR_SIZE_SHIFT)
#define USIC_TBCTR_LOF              (1 << 28) /* Bit 28:  */
#define USIC_TBCTR_STBIEN           (1 << 30) /* Bit 30:  */
#define USIC_TBCTR_TBERIEN          (1 << 31) /* Bit 31:  */

/* Receiver Buffer Control Register */

#define USIC_RBCTR_DPTR_SHIFT       (0)       /* Bits 0-5:  */
#define USIC_RBCTR_DPTR_MASK        (0x3f << USIC_RBCTR_DPTR_SHIFT)
#define USIC_RBCTR_LIMIT_SHIFT      (8)       /* Bits 8-13:  */
#define USIC_RBCTR_LIMIT_MASK       (0x3f << USIC_RBCTR_LIMIT_SHIFT)
#define USIC_RBCTR_SRBTM            (1 << 14) /* Bit 14:  */
#define USIC_RBCTR_SRBTEN           (1 << 15) /* Bit 15:  */
#define USIC_RBCTR_SRBINP_SHIFT     (16)      /* Bits 16-18:  */
#define USIC_RBCTR_SRBINP_MASK      (7 << USIC_RBCTR_SRBINP_SHIFT)
#define USIC_RBCTR_ARBINP_SHIFT     (19)      /* Bits 19-21:  */
#define USIC_RBCTR_ARBINP_MASK      (7 << USIC_RBCTR_ARBINP_SHIFT)
#define USIC_RBCTR_RCIM_SHIFT       (22)      /* Bits 22-23:  */
#define USIC_RBCTR_RCIM_MASK        (3 << USIC_RBCTR_RCIM_SHIFT)
#define USIC_RBCTR_SIZE_SHIFT       (24)      /* Bits 24-26:  */
#define USIC_RBCTR_SIZE_MASK        (7 << USIC_RBCTR_SIZE_SHIFT)
#define USIC_RBCTR_RNM              (1 << 27) /* Bit 27:  */
#define USIC_RBCTR_LOF              (1 << 28) /* Bit 28:  */
#define USIC_RBCTR_ARBIEN           (1 << 29) /* Bit 29:  */
#define USIC_RBCTR_SRBIEN           (1 << 30) /* Bit 30:  */
#define USIC_RBCTR_RBERIEN          (1 << 31) /* Bit 31:  */

/* Transmit/Receive Buffer Pointer Register */

#define USIC_TRBPTR_TDIPTR_SHIFT    (0)       /* Bits 0-5:  */
#define USIC_TRBPTR_TDIPTR_MASK     (0x3f << USIC_TRBPTR_TDIPTR_SHIFT)
#define USIC_TRBPTR_TDOPTR_SHIFT    (8)       /* Bits 813xx:  */
#define USIC_TRBPTR_TDOPTR_MASK     (0x3f << USIC_TRBPTR_TDOPTR_SHIFT)
#define USIC_TRBPTR_RDIPTR_SHIFT    (16)      /* Bits 16-21:  */
#define USIC_TRBPTR_RDIPTR_MASK     (0x3f << USIC_TRBPTR_RDIPTR_SHIFT)
#define USIC_TRBPTR_RDOPTR_SHIFT    (24)      /* Bits 24-29:  */
#define USIC_TRBPTR_RDOPTR_MASK     (0x3f << USIC_TRBPTR_RDOPTR_SHIFT)

/* Transmit/Receive Buffer Status Register */

#define USIC_TRBSR_SRBI             (1 << 0)  /* Bit 0:   */
#define USIC_TRBSR_RBERI            (1 << 1)  /* Bit 1:   */
#define USIC_TRBSR_ARBI             (1 << 2)  /* Bit 2:   */
#define USIC_TRBSR_REMPTY           (1 << 3)  /* Bit 3:   */
#define USIC_TRBSR_RFULL            (1 << 4)  /* Bit 4:   */
#define USIC_TRBSR_RBUS             (1 << 5)  /* Bit 5:   */
#define USIC_TRBSR_SRBT             (1 << 6)  /* Bit 6:   */
#define USIC_TRBSR_STBI             (1 << 8)  /* Bit 8:   */
#define USIC_TRBSR_TBERI            (1 << 9)  /* Bit 9:   */
#define USIC_TRBSR_TEMPTY           (1 << 11) /* Bit 11:  */
#define USIC_TRBSR_TFULL            (1 << 12) /* Bit 12:  */
#define USIC_TRBSR_TBUS             (1 << 13) /* Bit 13:  */
#define USIC_TRBSR_STBT             (1 << 14) /* Bit 14:  */
#define USIC_TRBSR_RBFLVL_SHIFT     (16)      /* Bits 16-22:  */
#define USIC_TRBSR_RBFLVL_MASK      (0x7f << USIC_TRBSR_RBFLVL_SHIFT)
#define USIC_TRBSR_TBFLVL_SHIFT     (24)      /* Bits 22-28:  */
#define USIC_TRBSR_TBFLVL_MASK      (0x7f << USIC_TRBSR_TBFLVL_SHIFT)

/* Transmit/Receive Buffer Status Clear Register */

#define USIC_TRBSCR_CSRBI           (1 << 0)  /* Bit 0:   */
#define USIC_TRBSCR_CRBERI          (1 << 1)  /* Bit 1:   */
#define USIC_TRBSCR_CARBI           (1 << 2)  /* Bit 2:   */
#define USIC_TRBSCR_CSTBI           (1 << 8)  /* Bit 8:   */
#define USIC_TRBSCR_CTBERI          (1 << 9)  /* Bit 9:   */
#define USIC_TRBSCR_CBDV            (1 << 10) /* Bit 10:  */
#define USIC_TRBSCR_FLUSHRB         (1 << 14) /* Bit 14:  */
#define USIC_TRBSCR_FLUSHTB         (1 << 15) /* Bit 15:  */

/* Receiver Buffer Output Register */

#define USIC_OUTR_DSR_SHIFT         (0)       /* Bits 0-15:  */
#define USIC_OUTR_DSR_MASK          (0xffff << USIC_OUTR_DSR_SHIFT)
#define USIC_OUTR_RCI_SHIFT         (16)      /* Bits 16-20:  */
#define USIC_OUTR_RCI_MASK          (31 << USIC_OUTR_RCI_SHIFT)

/* Receiver Buffer Output Register L for Debugger */

#define USIC_OUTDR_DSR_SHIFT        (0)       /* Bits 0-15:  */
#define USIC_OUTDR_DSR_MASK         (0xffff << USIC_OUTDR_DSR_SHIFT)
#define USIC_OUTDR_RCI_SHIFT        (16)      /* Bits 16-30:  */
#define USIC_OUTDR_RCI_MASK         (31 << USIC_OUTDR_RCI_SHIFT)

/* Transmit FIFO Buffer (32 x 4-bytes) */

#define USIC_IN_TDATA_SHIFT         (0)      /* Bits 0-15:  */
#define USIC_IN_TDATA_MASK          (0xffff << USIC_IN_TDATA_SHIFT)

#endif /* __ARCH_ARM_SRC_XMC4_CHIP_XMC4_USIC_H */
