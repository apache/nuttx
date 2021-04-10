/****************************************************************************
 * arch/arm/src/xmc4/hardware/xmc4_usic.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * Infineon Technologies AG (Infineon) is supplying this software for use
 * with Infineon's microcontrollers.  This file can be freely distributed
 * within development tools that are supporting such microcontrollers.
 *
 * THIS SOFTWARE IS PROVIDED AS IS. NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS
 * SOFTWARE. INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL,
 * INCIDENTAL, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ****************************************************************************/

/* Reference: XMC4500 Reference Manual V1.5 2014-07 Microcontrollers. */

#ifndef __ARCH_ARM_SRC_XMC4_HARDWARE_XMC4_USIC_H
#define __ARCH_ARM_SRC_XMC4_HARDWARE_XMC4_USIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/xmc4_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

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

/* Register Addresses *******************************************************/

/* USIC0 Registers */

/* Kernel Registers */

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

/* Kernel Registers */

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

/* Kernel Registers */

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

/* Register Bit-Field Definitions *******************************************/

/* Kernel Registers */

/* Kernel State Configuration Register */

#define USIC_ID_MOD_REV_SHIFT       (0)       /* Bits 0-7: Module Revision Number */
#define USIC_ID_MOD_REV_MASK        (0xff << USIC_ID_MOD_REV_SHIFT)
#define USIC_ID_MOD_TYPE_SHIFT      (8)       /* Bits 8-15: Module Type */
#define USIC_ID_MOD_TYPE_MASK       (0xff << USIC_ID_MOD_REV_SHIFT)
#define USIC_ID_MOD_NUMBER_SHIFT    (16)      /* Bits 16-31: Module Number Value */
#define USIC_ID_MOD_NUMBER_MASK     (0xffff << USIC_ID_MOD_NUMBER_SHIFT)

/* USIC Channel Registers */

/* Channel Configuration Register */

#define USIC_CCFG_SSC               (1 << 0)  /* Bit 0:  SSC Protocol Available */
#define USIC_CCFG_ASC               (1 << 1)  /* Bit 1:  ASC Protocol Available */
#define USIC_CCFG_I2C               (1 << 2)  /* Bit 2:  IIC Protocol Available */
#define USIC_CCFG_I2S               (1 << 3)  /* Bit 3:  IIS Protocol Available */
#define USIC_CCFG_RB                (1 << 6)  /* Bit 6:  Receive FIFO Buffer Available */
#define USIC_CCFG_TB                (1 << 7)  /* Bit 7:  Transmit FIFO Buffer Available */

/* Kernel State Configuration Register */

#define USIC_KSCFG_MODEN            (1 << 0)  /* Bit 0:  Module Enable */
#define USIC_KSCFG_BPMODEN          (1 << 1)  /* Bit 1:  Bit Protection for MODEN */
#define USIC_KSCFG_NOMCFG_SHIFT     (4)       /* Bits 4-5: Normal Operation Mode Configuration */
#define USIC_KSCFG_NOMCFG_MASK      (3 << USIC_KSCFG_NOMCFG_SHIFT)
#  define USIC_KSCFG_NOMCFG_RUN0    (0 << USIC_KSCFG_NOMCFG_SHIFT) /* Run mode 0 selected */
#  define USIC_KSCFG_NOMCFG_RUN1    (1 << USIC_KSCFG_NOMCFG_SHIFT) /* Run mode 1 selected */
#  define USIC_KSCFG_NOMCFG_STOP0   (2 << USIC_KSCFG_NOMCFG_SHIFT) /* Stop mode 0 selected */
#  define USIC_KSCFG_NOMCFG_STOP1   (3 << USIC_KSCFG_NOMCFG_SHIFT) /* Stop mode 1 selected */

#define USIC_KSCFG_BPNOM            (1 << 7)  /* Bit 7:  Bit Protection for NOMCFG */
#define USIC_KSCFG_SUMCFG_SHIFT     (8)       /* Bits 8-9: Suspend Mode Configuration */
#define USIC_KSCFG_SUMCFG_MASK      (3 << USIC_KSCFG_SUMCFG_SHIFT)
#  define USIC_KSCFG_SUMCFG_RUN0    (0 << USIC_KSCFG_SUMCFG_SHIFT) /* Run mode 0 selected */
#  define USIC_KSCFG_SUMCFG_RUN1    (1 << USIC_KSCFG_SUMCFG_SHIFT) /* Run mode 1 selected */
#  define USIC_KSCFG_SUMCFG_STOP0   (2 << USIC_KSCFG_SUMCFG_SHIFT) /* Stop mode 0 selected */
#  define USIC_KSCFG_SUMCFG_STOP1   (3 << USIC_KSCFG_SUMCFG_SHIFT) /* Stop mode 1 selected */

#define USIC_KSCFG_BPSUM            (1 << 11) /* Bit 11:  Bit Protection for SUMCFG */

/* Fractional Divider Register */

#define USIC_FDR_STEP_SHIFT         (0)       /* Bits 0-9: Step Value */
#define USIC_FDR_STEP_MASK          (0x3ff << USIC_FDR_STEP_SHIFT)
#  define USIC_FDR_STEP(n)          ((uint32_t)(n) << USIC_FDR_STEP_SHIFT)
#define USIC_FDR_DM_SHIFT           (14)      /* Bits 14-15: Divider Mode */
#define USIC_FDR_DM_MASK            (3 << USIC_FDR_DM_SHIFT)
#  define USIC_FDR_DM_OFF           (0 << USIC_FDR_DM_SHIFT) /* Divider switched off */
#  define USIC_FDR_DM_NORMAL        (1 << USIC_FDR_DM_SHIFT) /* Normal divider mode selected */
#  define USIC_FDR_DM_FRACTIONAL    (2 << USIC_FDR_DM_SHIFT) /* Fractional divider mode selected */

#define USIC_FDR_RESULT_SHIFT       (16)      /* Bits 16-25: Result Value */
#define USIC_FDR_RESULT_MASK        (0x3ff << USIC_FDR_RESULT_SHIFT)

/* Baud Rate Generator Register */

#define USIC_BRG_CLKSEL_SHIFT       (0)       /* Bits 0-1: Clock Selection */
#define USIC_BRG_CLKSEL_MASK        (3 << USIC_BRG_CLKSEL_SHIFT)
#  define USIC_BRG_CLKSEL_FRAC      (0 << USIC_BRG_CLKSEL_SHIFT) /* Fractional divider frequency fFD */
#  define USIC_BRG_CLKSEL_DX1T      (2 << USIC_BRG_CLKSEL_SHIFT) /* Trigger signal DX1T defines fPIN */
#  define USIC_BRG_CLKSEL_DX1S      (3 << USIC_BRG_CLKSEL_SHIFT) /* Frequency fPIN is derived from DX1S */

#define USIC_BRG_TMEN               (1 << 3)  /* Bit 3:  Timing Measurement Enable */
#define USIC_BRG_PPPEN              (1 << 4)  /* Bit 4:  Enable 2:1 Divider for fPPP */
#define USIC_BRG_CTQSEL_SHIFT       (6)       /* Bits 6-7: Input Selection for CTQ */
#define USIC_BRG_CTQSEL_MASK        (3 << USIC_BRG_CTQSEL_SHIFT)
#  define USIC_BRG_CTQSEL_FPDIV     (0 << USIC_BRG_CTQSEL_SHIFT) /* fCTQIN = fPDIV */
#  define USIC_BRG_CTQSEL_FPPP      (1 << USIC_BRG_CTQSEL_SHIFT) /* fCTQIN = fPPP */
#  define USIC_BRG_CTQSEL_FSCLK     (2 << USIC_BRG_CTQSEL_SHIFT) /* fCTQIN = fSCLK */
#  define USIC_BRG_CTQSEL_FMCLK     (3 << USIC_BRG_CTQSEL_SHIFT) /* fCTQIN = fMCLK */

#define USIC_BRG_PCTQ_SHIFT         (8)       /* Bits 8-9: Pre-Divider for Time Quanta Counter */
#define USIC_BRG_PCTQ_MASK          (3 << USIC_BRG_PCTQ_SHIFT)
#  define USIC_BRG_PCTQ(n)          ((uint32_t)((n)-1) << USIC_BRG_PCTQ_SHIFT)
#define USIC_BRG_DCTQ_SHIFT         (10)      /* Bits 10-15: Denominator for Time Quanta Counter */
#define USIC_BRG_DCTQ_MASK          (0x3f << USIC_BRG_DCTQ_SHIFT)
#  define USIC_BRG_DCTQ(n)          ((uint32_t)(n) << USIC_BRG_DCTQ_SHIFT)
#define USIC_BRG_PDIV_SHIFT         (16)      /* Bits 16-25: Divider Mode: Divider Factor to Generate fPDIV */
#define USIC_BRG_PDIV_MASK          (0x3ff << USIC_BRG_PDIV_SHIFT)
#  define USIC_BRG_PDIV(n)          ((uint32_t)(n) << USIC_BRG_PDIV_SHIFT)
#define USIC_BRG_SCLKOSEL           (1 << 28) /* Bit 28:  Shift Clock Output Select */
#define USIC_BRG_MCLKCFG            (1 << 29) /* Bit 29:  Master Clock Configuration */
#define USIC_BRG_SCLKCFG_SHIFT      30        /* Bits 30-31: Shift Clock Output Configuration */
#define USIC_BRG_SCLKCFG_MASK       (3 << USIC_BRG_SCLKCFG_SHIFT)
#  define USIC_BRG_SCLKCFG_NOINVNODLY (0 << USIC_BRG_SCLKCFG_SHIFT) /* No inverted signal and no delay */
#  define USIC_BRG_SCLKCFG_INVNODLY   (1 << USIC_BRG_SCLKCFG_SHIFT) /* Inverted signal and no delay */
#  define USIC_BRG_SCLKCFG_NOINVDLY   (2 << USIC_BRG_SCLKCFG_SHIFT) /* No inverted signal and 1/2 delay */
#  define USIC_BRG_SCLKCFG_INVDLY     (3 << USIC_BRG_SCLKCFG_SHIFT) /* Inverted signal and 1/2 delay */

/* Interrupt Node Pointer Register */

#define USIC_INPR_TSINP_SHIFT       (0)       /* Bits 0-2: Transmit Shift Interrupt Node Pointer */
#define USIC_INPR_TSINP_MASK        (7 << USIC_INPR_TSINP_SHIFT)
#  define USIC_INPR_TSINP_SR0       (0 << USIC_INPR_TSINP_SHIFT) /* Output SR0 activated */
#  define USIC_INPR_TSINP_SR1       (1 << USIC_INPR_TSINP_SHIFT) /* Output SR1 activated */
#  define USIC_INPR_TSINP_SR2       (2 << USIC_INPR_TSINP_SHIFT) /* Output SR2 activated */
#  define USIC_INPR_TSINP_SR3       (3 << USIC_INPR_TSINP_SHIFT) /* Output SR3 activated */
#  define USIC_INPR_TSINP_SR4       (4 << USIC_INPR_TSINP_SHIFT) /* Output SR4 activated */
#  define USIC_INPR_TSINP_SR5       (5 << USIC_INPR_TSINP_SHIFT) /* Output SR5 activated */

#define USIC_INPR_TBINP_SHIFT       (4)       /* Bits 4-6: Transmit Buffer Interrupt Node Pointer */
#define USIC_INPR_TBINP_MASK        (7 << USIC_INPR_TBINP_SHIFT)
#  define USIC_INPR_TBINP_SR0       (0 << USIC_INPR_TBINP_SHIFT) /* Output SR0 activated */
#  define USIC_INPR_TBINP_SR1       (1 << USIC_INPR_TBINP_SHIFT) /* Output SR1 activated */
#  define USIC_INPR_TBINP_SR2       (2 << USIC_INPR_TBINP_SHIFT) /* Output SR2 activated */
#  define USIC_INPR_TBINP_SR3       (3 << USIC_INPR_TBINP_SHIFT) /* Output SR3 activated */
#  define USIC_INPR_TBINP_SR4       (4 << USIC_INPR_TBINP_SHIFT) /* Output SR4 activated */
#  define USIC_INPR_TBINP_SR5       (5 << USIC_INPR_TBINP_SHIFT) /* Output SR5 activated */

#define USIC_INPR_RINP_SHIFT        (8)       /* Bits 8-10: Receive Interrupt Node Pointer */
#define USIC_INPR_RINP_MASK         (7 << USIC_INPR_RINP_SHIFT)
#  define USIC_INPR_RINP_SR0        (0 << USIC_INPR_RINP_SHIFT) /* Output SR0 activated */
#  define USIC_INPR_RINP_SR1        (1 << USIC_INPR_RINP_SHIFT) /* Output SR1 activated */
#  define USIC_INPR_RINP_SR2        (2 << USIC_INPR_RINP_SHIFT) /* Output SR2 activated */
#  define USIC_INPR_RINP_SR3        (3 << USIC_INPR_RINP_SHIFT) /* Output SR3 activated */
#  define USIC_INPR_RINP_SR4        (4 << USIC_INPR_RINP_SHIFT) /* Output SR4 activated */
#  define USIC_INPR_RINP_SR5        (5 << USIC_INPR_RINP_SHIFT) /* Output SR5 activated */

#define USIC_INPR_AINP_SHIFT        (12)      /* Bits 12-14: Alternative Receive Interrupt Node Pointer */
#define USIC_INPR_AINP_MASK         (7 << USIC_INPR_AINP_SHIFT)
#  define USIC_INPR_AINP_SR0        (0 << USIC_INPR_AINP_SHIFT) /* Output SR0 activated */
#  define USIC_INPR_AINP_SR1        (1 << USIC_INPR_AINP_SHIFT) /* Output SR1 activated */
#  define USIC_INPR_AINP_SR2        (2 << USIC_INPR_AINP_SHIFT) /* Output SR2 activated */
#  define USIC_INPR_AINP_SR3        (3 << USIC_INPR_AINP_SHIFT) /* Output SR3 activated */
#  define USIC_INPR_AINP_SR4        (4 << USIC_INPR_AINP_SHIFT) /* Output SR4 activated */
#  define USIC_INPR_AINP_SR5        (5 << USIC_INPR_AINP_SHIFT) /* Output SR5 activated */

#define USIC_INPR_PINP_SHIFT        (16)      /* Bits 16-18: Protocol Interrupt Node Pointer */
#define USIC_INPR_PINP_MASK         (7 << USIC_INPR_PINP_SHIFT)
#  define USIC_INPR_PINP_SR0        (0 << USIC_INPR_PINP_SHIFT) /* Output SR0 activated */
#  define USIC_INPR_PINP_SR1        (1 << USIC_INPR_PINP_SHIFT) /* Output SR1 activated */
#  define USIC_INPR_PINP_SR2        (2 << USIC_INPR_PINP_SHIFT) /* Output SR2 activated */
#  define USIC_INPR_PINP_SR3        (3 << USIC_INPR_PINP_SHIFT) /* Output SR3 activated */
#  define USIC_INPR_PINP_SR4        (4 << USIC_INPR_PINP_SHIFT) /* Output SR4 activated */
#  define USIC_INPR_PINP_SR5        (5 << USIC_INPR_PINP_SHIFT) /* Output SR5 activated */

/* Input Control Register 0, Input Control Register 1,
 * Input Control Register 2, Input Control Register 3,
 * Input Control Register 4, Input Control Register 5
 */

#define USIC_DXCR_DSEL_SHIFT        (0)      /* Bits 0-2: Data Selection for Input Signal */
#define USIC_DXCR_DSEL_MASK         (7 << USIC_DXCR_DSEL_SHIFT)
#  define USIC_DXCR_DSEL_DX(m)      ((uint32_t)(m) << USIC_DXCR_DSEL_SHIFT) /* Data input DXnm selected */

#  define USIC_DXCR_DSEL_DXA        (0 << USIC_DXCR_DSEL_SHIFT) /* Data input DXnA selected */
#  define USIC_DXCR_DSEL_DXB        (1 << USIC_DXCR_DSEL_SHIFT) /* Data input DXnB selected */
#  define USIC_DXCR_DSEL_DXC        (2 << USIC_DXCR_DSEL_SHIFT) /* Data input DXnC selected */
#  define USIC_DXCR_DSEL_DXD        (3 << USIC_DXCR_DSEL_SHIFT) /* Data input DXnD selected */
#  define USIC_DXCR_DSEL_DXE        (4 << USIC_DXCR_DSEL_SHIFT) /* Data input DXnE selected */
#  define USIC_DXCR_DSEL_DXF        (5 << USIC_DXCR_DSEL_SHIFT) /* Data input DXnF selected */
#  define USIC_DXCR_DSEL_DXG        (6 << USIC_DXCR_DSEL_SHIFT) /* Data input DXnG selected */
#  define USIC_DXCR_DSEL_ONE        (7 << USIC_DXCR_DSEL_SHIFT) /* Data input is always 1 */

#define USIC_DX1CR_DCEN             (1 << 3)  /* Bit 3:  Delay Compensation Enable (DX1CR only) */
#define USIC_DXCR_INSW              (1 << 4)  /* Bit 4:  Input Switch */
#define USIC_DXCR_DFEN              (1 << 5)  /* Bit 5:  Digital Filter Enable */
#define USIC_DXCR_DSEN              (1 << 6)  /* Bit 6:  Data Synchronization Enable */
#define USIC_DXCR_DPOL              (1 << 8)  /* Bit 8:  Data Polarity for DXn */
#define USIC_DXCR_SFSEL             (1 << 9)  /* Bit 9:  Sampling Frequency Selection */
#define USIC_DXCR_CM_SHIFT          (10)      /* Bits 10-11: Combination Mode */
#define USIC_DXCR_CM_MASK           (3 << USIC_DXCR_CM_SHIFT)
#  define USIC_DXCR_CM_DISABLE      (0 << USIC_DXCR_CM_SHIFT) /* Trigger activation disabled */
#  define USIC_DXCR_CM_RISING       (1 << USIC_DXCR_CM_SHIFT) /* Rising edge activates DXnT */
#  define USIC_DXCR_CM_FALLING      (2 << USIC_DXCR_CM_SHIFT) /* Falling edge activates DXnT */
#  define USIC_DXCR_CM_BOTH         (3 << USIC_DXCR_CM_SHIFT) /* Both edges activate DXnT */

#define USIC_DXCR_DXS               (1 << 15) /* Bit 15:  Synchronized Data Value */

/* Shift Control Register */

#define USIC_SCTR_SDIR              (1 << 0)  /* Bit 0:  Shift Direction */
#define USIC_SCTR_PDL               (1 << 1)  /* Bit 1:  Passive Data Level */
#  define USIC_SCTR_PDL0            (0)       /*         0=Passive data level is 0 */
#  define USIC_SCTR_PDL1            (1 << 1)  /*         1=Passive data level is 1 */
#define USIC_SCTR_DSM_SHIFT         (2)       /* Bits 2-3: Data Shift Mode */
#define USIC_SCTR_DSM_MASK          (3 << USIC_SCTR_DSM_SHIFT)
#  define USIC_SCTR_DSM_1BIT        (0 << USIC_SCTR_DSM_SHIFT) /* Data is shifted one bit at a time */
#  define USIC_SCTR_DSM_2BITS       (2 << USIC_SCTR_DSM_SHIFT) /* Data is shifted two bits at a time */
#  define USIC_SCTR_DSM_4BITS       (3 << USIC_SCTR_DSM_SHIFT) /* Data is shifted four bits at a time */

#define USIC_SCTR_HPCDIR            (1 << 4)  /* Bit 4:  Port Control Direction */
#define USIC_SCTR_DOCFG_SHIFT       (6)       /* Bits 6-7: Data Output Configuration */
#define USIC_SCTR_DOCFG_MASK        (3 << USIC_SCTR_DOCFG_SHIFT)
#  define USIC_SCTR_DOCFG_NORMAL    (0 << USIC_SCTR_DOCFG_SHIFT) /* DOUTx = shift data value */
#  define USIC_SCTR_DOCFG_INVERT    (1 << USIC_SCTR_DOCFG_SHIFT) /* DOUTx = inverted shift data value */

#define USIC_SCTR_TRM_SHIFT         (8)       /* Bits 8-9: Transmission Mode */
#define USIC_SCTR_TRM_MASK          (3 << USIC_SCTR_TRM_SHIFT)
#  define USIC_SCTR_TRM_INACTIVE    (0 << USIC_SCTR_TRM_SHIFT) /* Inactive */
#  define USIC_SCTR_TRM_1LEVEL      (1 << USIC_SCTR_TRM_SHIFT) /* Active at 1-level */
#  define USIC_SCTR_TRM_0LEVEL      (2 << USIC_SCTR_TRM_SHIFT) /* Active at 0-level */
#  define USIC_SCTR_TRM_ACTIVE      (3 << USIC_SCTR_TRM_SHIFT) /* Active without regard to signal level */

#define USIC_SCTR_FLE_SHIFT         (16)      /* Bits 16-21: Frame Length */
#define USIC_SCTR_FLE_MASK          (0x3f << USIC_SCTR_FLE_SHIFT)
#  define USIC_SCTR_FLE(n)          ((uint32_t)((n)-1) << USIC_SCTR_FLE_SHIFT)
#define USIC_SCTR_WLE_SHIFT         (24)      /* Bits 24-27: Word Length */
#define USIC_SCTR_WLE_MASK          (15 << USIC_SCTR_WLE_SHIFT)
#  define USIC_SCTR_WLE(n)          ((uint32_t)((n)-1) << USIC_SCTR_WLE_SHIFT)

/* Transmit Control/Status Register */

#define USIC_TCSR_WLEMD             (1 << 0)  /* Bit 0:  WLE Mode */
#define USIC_TCSR_SELMD             (1 << 1)  /* Bit 1:  Select Mode */
#define USIC_TCSR_FLEMD             (1 << 2)  /* Bit 2:  FLE Mode */
#define USIC_TCSR_WAMD              (1 << 3)  /* Bit 3:  WA Mode */
#define USIC_TCSR_HPCMD             (1 << 4)  /* Bit 4:  Hardware Port Control Mode */
#define USIC_TCSR_SOF               (1 << 5)  /* Bit 5:  Start Of Frame */
#define USIC_TCSR_EOF               (1 << 6)  /* Bit 6:  End Of Frame */
#define USIC_TCSR_TDV               (1 << 7)  /* Bit 7:  Transmit Data Valid */
#define USIC_TCSR_TDSSM             (1 << 8)  /* Bit 8:  TBUF Data Single Shot Mode */
#define USIC_TCSR_TDEN_SHIFT        (10)      /* Bits 10-11: TBUF Data Enable */
#define USIC_TCSR_TDEN_MASK         (3 << USIC_TCSR_TDEN_SHIFT)
#  define USIC_TCSR_TDEN_DISABLE    (0 << USIC_TCSR_TDEN_SHIFT) /* Transmission of data word disabled */
#  define USIC_TCSR_TDEN_TDIV       (1 << USIC_TCSR_TDEN_SHIFT) /* Transmission of data word if TDV = 1 */
#  define USIC_TCSR_TDEN_TDIVDX2S0  (2 << USIC_TCSR_TDEN_SHIFT) /* Transmission of data word if TDV = 1 while DX2S = 0 */
#  define USIC_TCSR_TDEN_TDIVDX2S1  (3 << USIC_TCSR_TDEN_SHIFT) /* Transmission of data word if TDV = 1 while DX2S = 1 */

#define USIC_TCSR_TDVTR             (1 << 12) /* Bit 12: TBUF Data Valid Trigger */
#define USIC_TCSR_WA                (1 << 13) /* Bit 13: Word Addre */
#define USIC_TCSR_TSOF              (1 << 24) /* Bit 24: Transmitted Start Of Frame */
#define USIC_TCSR_TV                (1 << 26) /* Bit 26: Transmission Valid */
#define USIC_TCSR_TVC               (1 << 27) /* Bit 27: Transmission Valid Cumulated */
#define USIC_TCSR_TE                (1 << 28) /* Bit 28: Trigger Event */

/* Protocol Control Register */

#define USIC_PCR_CTR(n)             (1 << (n))/* Bit n:  Protocol Control Bit n */
#define USIC_PCR_CTR0               (1 << 0)  /* Bit 0:  Protocol Control Bit 0 */
#define USIC_PCR_CTR1               (1 << 1)  /* Bit 1:  Protocol Control Bit 1 */
#define USIC_PCR_CTR2               (1 << 2)  /* Bit 2:  Protocol Control Bit 2 */
#define USIC_PCR_CTR3               (1 << 3)  /* Bit 3:  Protocol Control Bit 3 */
#define USIC_PCR_CTR4               (1 << 4)  /* Bit 4:  Protocol Control Bit 4 */
#define USIC_PCR_CTR5               (1 << 5)  /* Bit 5:  Protocol Control Bit 5 */
#define USIC_PCR_CTR6               (1 << 6)  /* Bit 6:  Protocol Control Bit 6 */
#define USIC_PCR_CTR7               (1 << 7)  /* Bit 7:  Protocol Control Bit 7 */
#define USIC_PCR_CTR8               (1 << 8)  /* Bit 8:  Protocol Control Bit 8 */
#define USIC_PCR_CTR9               (1 << 9)  /* Bit 9:  Protocol Control Bit 9 */
#define USIC_PCR_CTR10              (1 << 10) /* Bit 10: Protocol Control Bit 10 */
#define USIC_PCR_CTR11              (1 << 11) /* Bit 11: Protocol Control Bit 11 */
#define USIC_PCR_CTR12              (1 << 12) /* Bit 12: Protocol Control Bit 12 */
#define USIC_PCR_CTR13              (1 << 13) /* Bit 13: Protocol Control Bit 13 */
#define USIC_PCR_CTR14              (1 << 14) /* Bit 14: Protocol Control Bit 14 */
#define USIC_PCR_CTR15              (1 << 15) /* Bit 15: Protocol Control Bit 15 */
#define USIC_PCR_CTR16              (1 << 16) /* Bit 16: Protocol Control Bit 16 */
#define USIC_PCR_CTR17              (1 << 17) /* Bit 17: Protocol Control Bit 17 */
#define USIC_PCR_CTR18              (1 << 18) /* Bit 18: Protocol Control Bit 18 */
#define USIC_PCR_CTR19              (1 << 19) /* Bit 19: Protocol Control Bit 19 */
#define USIC_PCR_CTR20              (1 << 20) /* Bit 20: Protocol Control Bit 20 */
#define USIC_PCR_CTR21              (1 << 21) /* Bit 21: Protocol Control Bit 21 */
#define USIC_PCR_CTR22              (1 << 22) /* Bit 22: Protocol Control Bit 22 */
#define USIC_PCR_CTR23              (1 << 23) /* Bit 23: Protocol Control Bit 23 */
#define USIC_PCR_CTR24              (1 << 24) /* Bit 24: Protocol Control Bit 24 */
#define USIC_PCR_CTR25              (1 << 25) /* Bit 25: Protocol Control Bit 25 */
#define USIC_PCR_CTR26              (1 << 26) /* Bit 26: Protocol Control Bit 26 */
#define USIC_PCR_CTR27              (1 << 27) /* Bit 27: Protocol Control Bit 27 */
#define USIC_PCR_CTR28              (1 << 28) /* Bit 28: Protocol Control Bit 28 */
#define USIC_PCR_CTR29              (1 << 29) /* Bit 29: Protocol Control Bit 29 */
#define USIC_PCR_CTR30              (1 << 30) /* Bit 30: Protocol Control Bit 30 */
#define USIC_PCR_CTR31              (1 << 31) /* Bit 31: Protocol Control Bit 31 */

#define USIC_PCR_ASCMODE_SMD        (1 << 0)  /* Bit 0:  Sample Mode */
#define USIC_PCR_ASCMODE_STPB       (1 << 1)  /* Bit 1:  Stop Bits */
#define USIC_PCR_ASCMODE_IDM        (1 << 2)  /* Bit 2:  Idle Detection Mode */
#define USIC_PCR_ASCMODE_SBIEN      (1 << 3)  /* Bit 3:  Synchronization Break Interrupt Enable */
#define USIC_PCR_ASCMODE_CDEN       (1 << 4)  /* Bit 4:  Collision Detection Enable */
#define USIC_PCR_ASCMODE_RNIEN      (1 << 5)  /* Bit 5:  Receiver Noise Detection Interrupt Enable */
#define USIC_PCR_ASCMODE_FEIEN      (1 << 6)  /* Bit 6:  Format Error Interrupt Enable */
#define USIC_PCR_ASCMODE_FFIEN      (1 << 7)  /* Bit 7:  Frame Finished Interrupt Enable */
#define USIC_PCR_ASCMODE_SP_SHIFT   (8)       /* Bits 8-12: Sample Point */
#define USIC_PCR_ASCMODE_SP_MASK    (31 << USIC_PCR_ASCMODE_SP_SHIFT)
#  define USIC_PCR_ASCMODE_SP(n)    ((uint32_t)(n) << USIC_PCR_ASCMODE_SP_SHIFT)
#define USIC_PCR_ASCMODE_PL_SHIFT   (13)      /* Bits 13-15: Pulse Length */
#define USIC_PCR_ASCMODE_PL_MASK    (7 << USIC_PCR_ASCMODE_PL_SHIFT)
#  define USIC_PCR_ASCMODE_PLBIT    (0 << USIC_PCR_ASCMODE_PL_SHIFT)                 /* Pulse length = bit length */
#  define USIC_PCR_ASCMODE_PL(n)    ((uint32_t)((n)-1) << USIC_PCR_ASCMODE_PL_SHIFT) /* Pulse length = n quanta */

#define USIC_PCR_ASCMODE_RSTEN      (1 << 16) /* Bit 16: Receiver Status Enable */
#define USIC_PCR_ASCMODE_TSTEN      (1 << 17) /* Bit 17: Transmitter Status Enable */
#define USIC_PCR_ASCMODE_MCLK       (1 << 31) /* Bit 31: Master Clock Enable */

#define USIC_PCR_SSCMODE_MSLSEN     (1 << 0)  /* Bit 0:  MSLS Enable */
#define USIC_PCR_SSCMODE_SELCTR     (1 << 1)  /* Bit 1:  Select Control */
#define USIC_PCR_SSCMODE_SELINV     (1 << 2)  /* Bit 2:  Select Inversion */
#define USIC_PCR_SSCMODE_FEM        (1 << 3)  /* Bit 3:  Frame End Mode */
#define USIC_PCR_SSCMODE_CTQSEL1_SHIFT   (4)  /* Bits 4-5: Input Frequency Selection */
#define USIC_PCR_SSCMODE_CTQSEL1_MASK    (3 << USIC_PCR_SSCMODE_CTQSEL1_SHIFT)
#  define USIC_PCR_SSCMODE_CTQSEL1_FPDIV (0 << USIC_PCR_SSCMODE_CTQSEL1_SHIFT) /* fCTQIN = fPDIV */
#  define USIC_PCR_SSCMODE_CTQSEL1_FPPP  (1 << USIC_PCR_SSCMODE_CTQSEL1_SHIFT) /* fCTQIN = fPPP */
#  define USIC_PCR_SSCMODE_CTQSEL1_FSCLK (2 << USIC_PCR_SSCMODE_CTQSEL1_SHIFT) /* fCTQIN = fSCLK */
#  define USIC_PCR_SSCMODE_CTQSEL1_FMCLK (3 << USIC_PCR_SSCMODE_CTQSEL1_SHIFT) /* fCTQIN = fMCLK */

#define USIC_PCR_SSCMODE_PCTQ1_SHIFT   (6)    /* Bits 6-7: Divider Factor PCTQ1 for Tiw and Tnf */
#define USIC_PCR_SSCMODE_PCTQ1_MASK    (3 << USIC_PCR_SSCMODE_PCTQ1_SHIFT)
#  define USIC_PCR_SSCMODE_PCTQ1(n)    ((uint32_t)((n)-1) << USIC_PCR_SSCMODE_PCTQ1_SHIFT)
#define USIC_PCR_SSCMODE_DCTQ1_SHIFT   (8)    /* Bits 8-12: Divider Factor DCTQ1 for Tiw and Tnf */
#  define USIC_PCR_SSCMODE_DCTQ1(n)    (0x1f << USIC_PCR_SSCMODE_DCTQ1_SHIFT)
#define USIC_PCR_SSCMODE_DCTQ1_MASK    ((uint32_t)((n)-1) << USIC_PCR_SSCMODE_DCTQ1_SHIFT)
#define USIC_PCR_SSCMODE_PARIEN     (1 << 13) /* Bit 13: Parity Error Interrupt Enable */
#define USIC_PCR_SSCMODE_MSLSIEN    (1 << 14) /* Bit 14: MSLS Interrupt Enable */
#define USIC_PCR_SSCMODE_DX2TIEN    (1 << 15) /* Bit 15: DX2T Interrupt Enable */
#define USIC_PCR_SSCMODE_SELO_SHIFT (16)      /* Bits 16-23: Select Output */
#define USIC_PCR_SSCMODE_SELO_MASK  (0xff << USIC_PCR_SSCMODE_SELO_SHIFT)
#  define USIC_PCR_SSCMODE_SELO(n)  (1 << ((n) + USIC_PCR_SSCMODE_SELO_SHIFT))
#define USIC_PCR_SSCMODE_TIWEN      (1 << 24) /* Bit 24: Enable Inter-Word Delay Tiw */
#define USIC_PCR_SSCMODE_SLPHSEL    (1 << 25) /* Bit 25: Slave Mode Clock Phase Select */
#define USIC_PCR_SSCMODE_MCLK       (1 << 31) /* Bit 31: Master Clock Enable */

#define USIC_PCR_IICMODE_SLAD_SHIFT (0)       /* Bits 0-15: Slave Address */
#define USIC_PCR_IICMODE_SLAD_MASK  (0xffff << USIC_PCR_IICMODE_SLAD_SHIFT)
#  define USIC_PCR_IICMODE_SLAD(n)  ((uint32_t)(n) << USIC_PCR_IICMODE_SLAD_SHIFT)
#define USIC_PCR_IICMODE_ACK00      (1 << 16) /* Bit 16: Acknowledge 00H */
#define USIC_PCR_IICMODE_STIM       (1 << 17) /* Bit 17: Symbol Timing */
#define USIC_PCR_IICMODE_SCRIEN     (1 << 18) /* Bit 18: Start Condition Received Interrupt Enable */
#define USIC_PCR_IICMODE_RSCRIEN    (1 << 19) /* Bit 19: Repeated Start Condition Received Interrupt */
#define USIC_PCR_IICMODE_PCRIEN     (1 << 20) /* Bit 20: Stop Condition Received Interrupt Enable */
#define USIC_PCR_IICMODE_NACKIEN    (1 << 21) /* Bit 21: Non-Acknowledge Interrupt Enable */
#define USIC_PCR_IICMODE_ARLIEN     (1 << 22) /* Bit 22: Arbitration Lost Interrupt Enable */
#define USIC_PCR_IICMODE_SRRIEN     (1 << 23) /* Bit 23: Slave Read Request Interrupt Enable */
#define USIC_PCR_IICMODE_ERRIEN     (1 << 24) /* Bit 24: Error Interrupt Enable */
#define USIC_PCR_IICMODE_SACKDIS    (1 << 25) /* Bit 25: Slave Acknowledge Disable */
#define USIC_PCR_IICMODE_HDEL_SHIFT (26)      /* Bits 26-29: Hardware Delay */
#define USIC_PCR_IICMODE_HDEL_MASK  (15 << USIC_PCR_IICMODE_HDEL_SHIFT)
#  define USIC_PCR_IICMODE_HDEL(n)  ((uint32_t)(n) << USIC_PCR_IICMODE_HDEL_SHIFT)
#define USIC_PCR_IICMODE_ACKIEN     (1 << 30) /* Bit 30: Acknowledge Interrupt Enable */
#define USIC_PCR_IICMODE_MCLK       (1 << 31) /* Bit 31: Master Clock Enable */

#define USIC_PCR_IISMODE_WAGEN      (1 << 0)  /* Bit 0:  WA Generation Enable */
#define USIC_PCR_IISMODE_DTEN       (1 << 1)  /* Bit 1:  Data Transfers Enable */
#define USIC_PCR_IISMODE_SELINV     (1 << 2)  /* Bit 2:  Select Inversion */
#define USIC_PCR_IISMODE_WAFEIEN    (1 << 4)  /* Bit 4:  WA Falling Edge Interrupt Enable */
#define USIC_PCR_IISMODE_WAREIEN    (1 << 5)  /* Bit 5:  WA Rising Edge Interrupt Enable */
#define USIC_PCR_IISMODE_ENDIEN     (1 << 6)  /* Bit 6:  END Interrupt Enable */
#define USIC_PCR_IISMODE_DX2TIEN    (1 << 15) /* Bit 15: DX2T Interrupt Enable */
#define USIC_PCR_IISMODE_TDEL_SHIFT (16)      /* Bits 16-21: Transfer Delay */
#define USIC_PCR_IISMODE_TDEL_MASK  (0x3f << USIC_PCR_IISMODE_TDEL_SHIFT)
#  define USIC_PCR_IISMODE_TDEL(n)  ((uint32_t)(n) << USIC_PCR_IISMODE_TDEL_SHIFT)
#define USIC_PCR_IISMODE_MCLK       (1 << 31) /* Bit 31: Master Clock Enable */

/* Channel Control Register */

#define USIC_CCR_MODE_SHIFT         (0)       /* Bits 0-3:  Operating Mode */
#define USIC_CCR_MODE_MASK          (15 << USIC_CCR_MODE_SHIFT)
#  define USIC_CCR_MODE_DISABLE     (0 << USIC_CCR_MODE_SHIFT) /* USIC channel is disabled */
#  define USIC_CCR_MODE_SPI         (1 << USIC_CCR_MODE_SHIFT) /* SSC (SPI) protocol is selected */
#  define USIC_CCR_MODE_ASC         (2 << USIC_CCR_MODE_SHIFT) /* ASC (SCI, UART) protocol is selected */
#  define USIC_CCR_MODE_I2S         (3 << USIC_CCR_MODE_SHIFT) /* IIS protocol is selected */
#  define USIC_CCR_MODE_I2C         (4 << USIC_CCR_MODE_SHIFT) /* IIC protocol is selected */

#define USIC_CCR_HPCEN_SHIFT        (6)       /* Bits 6-7: Hardware Port Control Enable */
#define USIC_CCR_HPCEN_MASK         (3 << USIC_CCR_HPCEN_SHIFT)
#  define USIC_CCR_HPCEN_DISABLE    (0 << USIC_CCR_HPCEN_SHIFT) /* Port control disabled */
#  define USIC_CCR_HPCEN_DX0_1      (1 << USIC_CCR_HPCEN_SHIFT) /* Port control enabled for DX0 and DOUT0 */
#  define USIC_CCR_HPCEN_DX3        (2 << USIC_CCR_HPCEN_SHIFT) /* Port control enabled for DX3, DX0 and DOUT[1:0] */
#  define USIC_CCR_HPCEN_DX0_2      (3 << USIC_CCR_HPCEN_SHIFT) /* Port control enabled for DX0, DX[5:3] and DOUT[3:0] */

#define USIC_CCR_PM_SHIFT           (8)       /* Bits 8-9: Parity Mode */
#define USIC_CCR_PM_MASK            (3 << USIC_CCR_PM_SHIFT)
#  define USIC_CCR_PM_NONE          (0 << USIC_CCR_PM_SHIFT) /* Parity generation is disabled */
#  define USIC_CCR_PM_EVEN          (2 << USIC_CCR_PM_SHIFT) /* Even parity is selected */
#  define USIC_CCR_PM_ODD           (3 << USIC_CCR_PM_SHIFT) /* Odd parity is selected */

#define USIC_CCR_RSIEN              (1 << 10) /* Bit 10: Receiver Start Interrupt Enable */
#define USIC_CCR_DLIEN              (1 << 11) /* Bit 11: Data Lost Interrupt Enable */
#define USIC_CCR_TSIEN              (1 << 12) /* Bit 12: Transmit Shift Interrupt Enable */
#define USIC_CCR_TBIEN              (1 << 13) /* Bit 13: Transmit Buffer Interrupt Enable */
#define USIC_CCR_RIEN               (1 << 14) /* Bit 14: Receive Interrupt Enable */
#define USIC_CCR_AIEN               (1 << 15) /* Bit 15: Alternative Receive Interrupt Enable */
#define USIC_CCR_BRGIEN             (1 << 16) /* Bit 16: Baud Rate Generator Interrupt Enable */

/* Capture Mode Timer Register */

#define USIC_CMTR_CTV_SHIFT         (0)      /* Bits 0-9: Captured Timer Value */
#define USIC_CMTR_CTV_MASK          (0x3ff << USIC_CMTR_CTV_SHIFT)

/* Protocol Status Register */

#define USIC_PSR_ST(n)              (1 << (n))/* Bit n:  Protocol Status Flag n */
#define USIC_PSR_ST0                (1 << 0)  /* Bit 0:  Protocol Status Flag 0 */
#define USIC_PSR_ST1                (1 << 1)  /* Bit 1:  Protocol Status Flag 1 */
#define USIC_PSR_ST2                (1 << 2)  /* Bit 2:  Protocol Status Flag 2 */
#define USIC_PSR_ST3                (1 << 3)  /* Bit 3:  Protocol Status Flag 3 */
#define USIC_PSR_ST4                (1 << 4)  /* Bit 4:  Protocol Status Flag 4 */
#define USIC_PSR_ST5                (1 << 5)  /* Bit 5:  Protocol Status Flag 5 */
#define USIC_PSR_ST6                (1 << 6)  /* Bit 6:  Protocol Status Flag 6 */
#define USIC_PSR_ST7                (1 << 7)  /* Bit 7:  Protocol Status Flag 7 */
#define USIC_PSR_ST8                (1 << 8)  /* Bit 8:  Protocol Status Flag 8 */
#define USIC_PSR_ST9                (1 << 9)  /* Bit 9:  Protocol Status Flag 9 */
#define USIC_PSR_RSIF               (1 << 10) /* Bit 10: Receiver Start Indication Flag */
#define USIC_PSR_DLIF               (1 << 11) /* Bit 11: Data Lost Indication Flag */
#define USIC_PSR_TSIF               (1 << 12) /* Bit 12: Transmit Shift Indication Flag */
#define USIC_PSR_TBIF               (1 << 13) /* Bit 13: Transmit Buffer Indication Flag */
#define USIC_PSR_RIF                (1 << 14) /* Bit 14: Receive Indication Fla */
#define USIC_PSR_AIF                (1 << 15) /* Bit 15: Alternative Receive Indication Flag */
#define USIC_PSR_BRGIF              (1 << 16) /* Bit 16: Baud Rate Generator Indication Fl */

#define USIC_PSR_ASCMODE_TXIDLE     (1 << 0)  /* Bit 0:  Transmission Idle */
#define USIC_PSR_ASCMODE_RXIDLE     (1 << 1)  /* Bit 1:  Reception Idle */
#define USIC_PSR_ASCMODE_SBD        (1 << 2)  /* Bit 2:  Synchronization Break Detected */
#define USIC_PSR_ASCMODE_COL        (1 << 3)  /* Bit 3:  Collision Detected */
#define USIC_PSR_ASCMODE_RNS        (1 << 4)  /* Bit 4:  Receiver Noise Detected */
#define USIC_PSR_ASCMODE_FER0       (1 << 5)  /* Bit 5:  Format Error in Stop Bit 0 */
#define USIC_PSR_ASCMODE_FER1       (1 << 6)  /* Bit 6:  Format Error in Stop Bit 1 */
#define USIC_PSR_ASCMODE_RFF        (1 << 7)  /* Bit 7:  Receive Frame Finished */
#define USIC_PSR_ASCMODE_TFF        (1 << 8)  /* Bit 8:  Transmitter Frame Finished */
#define USIC_PSR_ASCMODE_BUSY       (1 << 9)  /* Bit 9:  Transfer Status BUSY */
#define USIC_PSR_ASCMODE_RSIF       (1 << 10) /* Bit 10: Receiver Start Indication Flag */
#define USIC_PSR_ASCMODE_DLIF       (1 << 11) /* Bit 11: Data Lost Indication Flag */
#define USIC_PSR_ASCMODE_TSIF       (1 << 12) /* Bit 12: Transmit Shift Indication Flag */
#define USIC_PSR_ASCMODE_TBIF       (1 << 13) /* Bit 13: Transmit Buffer Indication Flag */
#define USIC_PSR_ASCMODE_RIF        (1 << 14) /* Bit 14: Receive Indication Flag */
#define USIC_PSR_ASCMODE_AIF        (1 << 15) /* Bit 15: Alternative Receive Indication Flag */
#define USIC_PSR_ASCMODE_BRGIF      (1 << 16) /* Bit 16: Baud Rate Generator Indication Flag */

#define USIC_PSR_SSCMODE_MSLS       (1 << 0)  /* Bit 0:  MSLS Status */
#define USIC_PSR_SSCMODE_DX2S       (1 << 1)  /* Bit 1:  DX2S Status */
#define USIC_PSR_SSCMODE_MSLSEV     (1 << 2)  /* Bit 2:  MSLS Event Detected */
#define USIC_PSR_SSCMODE_DX2TEV     (1 << 3)  /* Bit 3:  DX2T Event Detected */
#define USIC_PSR_SSCMODE_PARERR     (1 << 4)  /* Bit 4:  Parity Error Event Detected */
#define USIC_PSR_SSCMODE_RSIF       (1 << 10) /* Bit 10: Receiver Start Indication Flag */
#define USIC_PSR_SSCMODE_DLIF       (1 << 11) /* Bit 11: Data Lost Indication Flag */
#define USIC_PSR_SSCMODE_TSIF       (1 << 12) /* Bit 12: Transmit Shift Indication Flag */
#define USIC_PSR_SSCMODE_TBIF       (1 << 13) /* Bit 13: Transmit Buffer Indication Flag */
#define USIC_PSR_SSCMODE_RIF        (1 << 14) /* Bit 14: Receive Indication Flag */
#define USIC_PSR_SSCMODE_AIF        (1 << 15) /* Bit 15: Alternative Receive Indication Flag */
#define USIC_PSR_SSCMODE_BRGIF      (1 << 16) /* Bit 16: Baud Rate Generator Indication Flag */

#define USIC_PSR_IICMODE_SLSEL      (1 << 0)  /* Bit 0:  Slave Select */
#define USIC_PSR_IICMODE_WTDF       (1 << 1)  /* Bit 1:  Wrong TDF Code Found */
#define USIC_PSR_IICMODE_SCR        (1 << 2)  /* Bit 2:  Start Condition Received */
#define USIC_PSR_IICMODE_RSCR       (1 << 3)  /* Bit 3:  Repeated Start Condition Received */
#define USIC_PSR_IICMODE_PCR        (1 << 4)  /* Bit 4:  Stop Condition Received */
#define USIC_PSR_IICMODE_NACK       (1 << 5)  /* Bit 5:  Non-Acknowledge Received */
#define USIC_PSR_IICMODE_ARL        (1 << 6)  /* Bit 6:  Arbitration Lost */
#define USIC_PSR_IICMODE_SRR        (1 << 7)  /* Bit 7:  Slave Read Request */
#define USIC_PSR_IICMODE_ERR        (1 << 8)  /* Bit 8:  Error */
#define USIC_PSR_IICMODE_ACK        (1 << 9)  /* Bit 9:  Acknowledge Received */
#define USIC_PSR_IICMODE_RSIF       (1 << 10) /* Bit 10: Receiver Start Indication Flag */
#define USIC_PSR_IICMODE_DLIF       (1 << 11) /* Bit 11: Data Lost Indication Flag */
#define USIC_PSR_IICMODE_TSIF       (1 << 12) /* Bit 12: Transmit Shift Indication Flag */
#define USIC_PSR_IICMODE_TBIF       (1 << 13) /* Bit 13: Transmit Buffer Indication Flag */
#define USIC_PSR_IICMODE_RIF        (1 << 14) /* Bit 14: Receive Indication Flag */
#define USIC_PSR_IICMODE_AIF        (1 << 15) /* Bit 15: Alternative Receive Indication Flag */
#define USIC_PSR_IICMODE_BRGIF      (1 << 16) /* Bit 16: Baud Rate Generator Indication Flag */

#define USIC_PSR_IISMODE_WA         (1 << 0)  /* Bit 0:  Word Address */
#define USIC_PSR_IISMODE_DX2S       (1 << 1)  /* Bit 1:  DX2S Sta */
#define USIC_PSR_IISMODE_DX2TEV     (1 << 3)  /* Bit 3:  DX2T Event Detected */
#define USIC_PSR_IISMODE_WAFE       (1 << 4)  /* Bit 4:  WA Falling Edge Event */
#define USIC_PSR_IISMODE_WARE       (1 << 5)  /* Bit 5:  WA Rising Edge Event */
#define USIC_PSR_IISMODE_END        (1 << 6)  /* Bit 6:  WA Generation End */
#define USIC_PSR_IISMODE_RSIF       (1 << 10) /* Bit 10: Receiver Start Indication Flag */
#define USIC_PSR_IISMODE_DLIF       (1 << 11) /* Bit 11: Data Lost Indication Flag */
#define USIC_PSR_IISMODE_TSIF       (1 << 12) /* Bit 12: Transmit Shift Indication Flag */
#define USIC_PSR_IISMODE_TBIF       (1 << 13) /* Bit 13: Transmit Buffer Indication Flag */
#define USIC_PSR_IISMODE_RIF        (1 << 14) /* Bit 14: Receive Indication Flag */
#define USIC_PSR_IISMODE_AIF        (1 << 15) /* Bit 15: Alternative Receive Indication Flag */
#define USIC_PSR_IISMODE_BRGIF      (1 << 16) /* Bit 16: Baud Rate Generator Indication Flag */

/* Protocol Status Clear Register */

#define USIC_PSCR_CST(n)            (1 << (n))/* Bit n:  Clear Status Flag n in PSR */
#define USIC_PSCR_CST0              (1 << 0)  /* Bit 0:  Clear Status Flag 0 in PSR */
#define USIC_PSCR_CST1              (1 << 1)  /* Bit 1:  Clear Status Flag 1 in PSR */
#define USIC_PSCR_CST2              (1 << 2)  /* Bit 2:  Clear Status Flag 2 in PSR */
#define USIC_PSCR_CST3              (1 << 3)  /* Bit 3:  Clear Status Flag 3 in PSR */
#define USIC_PSCR_CST4              (1 << 4)  /* Bit 4:  Clear Status Flag 4 in PSR */
#define USIC_PSCR_CST5              (1 << 5)  /* Bit 5:  Clear Status Flag 5 in PSR */
#define USIC_PSCR_CST6              (1 << 6)  /* Bit 6:  Clear Status Flag 6 in PSR */
#define USIC_PSCR_CST7              (1 << 7)  /* Bit 7:  Clear Status Flag 7 in PSR */
#define USIC_PSCR_CST8              (1 << 8)  /* Bit 8:  Clear Status Flag 8 in PSR */
#define USIC_PSCR_CST9              (1 << 9)  /* Bit 9:  Clear Status Flag 9 in PSR */
#define USIC_PSCR_CRSIF             (1 << 10) /* Bit 10: Clear Receiver Start Indication Flag */
#define USIC_PSCR_CDLIF             (1 << 11) /* Bit 11: Clear Data Lost Indication Flag */
#define USIC_PSCR_CTSIF             (1 << 12) /* Bit 12: Clear Transmit Shift Indication Flag */
#define USIC_PSCR_CTBIF             (1 << 13) /* Bit 13: Clear Transmit Buffer Indication Flag */
#define USIC_PSCR_CRIF              (1 << 14) /* Bit 14: Clear Receive Indication Flag */
#define USIC_PSCR_CAIF              (1 << 15) /* Bit 15: Clear Alternative Receive Indication Flag */
#define USIC_PSCR_CBRGIF            (1 << 16) /* Bit 16: Clear Baud Rate Generator Indication Flag */

/* Receiver Buffer Status Register */

#define USIC_RBUFSR_WLEN_SHIFT      (0)       /* Bits 0-3: Received Data Word Length in RBUF or RBUFD */
#define USIC_RBUFSR_WLEN_MASK       (15 << USIC_RBUFSR_WLEN_SHIFT)
#define USIC_RBUFSR_SOF             (1 << 6)  /* Bit 6:  Start of Frame in RBUF or RBUFD */
#define USIC_RBUFSR_PAR             (1 << 8)  /* Bit 8:  Protocol-Related Argument in RBUF or RBUFD */
#define USIC_RBUFSR_PERR            (1 << 9)  /* Bit 9:  Protocol-related Error in RBUF or RBUFD */
#define USIC_RBUFSR_RDV0            (1 << 13) /* Bit 13: Receive Data Valid in RBUF or RBUFD */
#define USIC_RBUFSR_RDV1            (1 << 14) /* Bit 14: Receive Data Valid in RBUF or RBUFD */
#define USIC_RBUFSR_DS              (1 << 15) /* Bit 15: Data Source of RBUF or RBUFD */

/* Receiver Buffer Register */

#define USIC_RBUF_DSR_SHIFT         (0)       /* Bits 0-15: Received Data */
#define USIC_RBUF_DSR_MASK          (0xffff << USIC_RBUF_DSR_SHIFT)

/* Receiver Buffer Register for Debugger */

#define USIC_RBUFD_DSR_SHIFT        (0)       /* Bits 0-15: Data from Shift Register */
#define USIC_RBUFD_DSR_MASK         (0xffff << USIC_RBUFD_DSR_SHIFT)

/* Receiver Buffer Register 0 */

#define USIC_RBUF0_DSR0_SHIFT       (0)       /* Bits 0-15: Data of Shift Registers 0[3:0] */
#define USIC_RBUF0_DSR0_MASK        (0xffff << USIC_RBUF0_DSR0_SHIFT)

/* Receiver Buffer Register 1 */

#define USIC_RBUF1_DSR1_SHIFT       (0)       /* Bits 0-15: Data of Shift Registers 1[3:0] */
#define USIC_RBUF1_DSR1_MASK        (0xffff << USIC_RBUF1_DSR1_SHIFT)

/*  Receiver Buffer 01 Status Register */

#define USIC_RBUF01SR_WLEN0_SHIFT   (0)       /* Bits 0-3: Received Data Word Length in RBUF0 */
#define USIC_RBUF01SR_WLEN0_MASK    (15 << USIC_RBUF01SR_WLEN0_SHIFT)
#define USIC_RBUF01SR_SOF0          (1 << 6)  /* Bit 6:  Start of Frame in RBUF0 */
#define USIC_RBUF01SR_PAR0          (1 << 8)  /* Bit 8:  Protocol-Related Argument in RBUF0 */
#define USIC_RBUF01SR_PERR0         (1 << 9)  /* Bit 9:  Protocol-related Error in RBUF0 */
#define USIC_RBUF01SR_RDV00         (1 << 13) /* Bit 13: Receive Data Valid in RBUF0 */
#define USIC_RBUF01SR_RDV01         (1 << 14) /* Bit 14: Receive Data Valid in RBUF1 */
#define USIC_RBUF01SR_DS0           (1 << 15) /* Bit 15: Data Source */
#define USIC_RBUF01SR_WLEN1_SHIFT   (16)      /* Bits 16-19: Received Data Word Length in RBUF1 */
#define USIC_RBUF01SR_WLEN1_MASK    (15 << USIC_RBUF01SR_WLEN1_SHIFT)
#define USIC_RBUF01SR_SOF1          (1 << 22) /* Bit 22: Start of Frame in RBUF1 */
#define USIC_RBUF01SR_PAR1          (1 << 24) /* Bit 24: Protocol-Related Argument in RBUF1 */
#define USIC_RBUF01SR_PERR1         (1 << 25) /* Bit 25: Protocol-related Error in RBU */
#define USIC_RBUF01SR_RDV10         (1 << 29) /* Bit 29: Receive Data Valid in RBUF0 */
#define USIC_RBUF01SR_RDV11         (1 << 30) /* Bit 30: Receive Data Valid in RBUF1 */
#define USIC_RBUF01SR_DS1           (1 << 31) /* Bit 31: Data Source */

/* Flag Modification Register */

#define USIC_FMR_MTDV_SHIFT         (0)       /* Bits 0-1: Modify Transmit Data Valid */
#define USIC_FMR_MTDV_MASK          (3 << USIC_FMR_MTDV_SHIFT)
#  define USIC_FMR_MTDV_NOACTION    (0 << USIC_FMR_MTDV_SHIFT) /* No action */
#  define USIC_FMR_MTDV_TDV         (1 << USIC_FMR_MTDV_SHIFT) /* Bit TDV is set, TE is unchanged */
#  define USIC_FMR_MTDV_TDVTE       (2 << USIC_FMR_MTDV_SHIFT) /* Bits TDV and TE are cleared */

#define USIC_FMR_ATVC               (1 << 4)  /* Bit 4:  Activate Bit TVC */
#define USIC_FMR_CRDV0              (1 << 14) /* Bit 14: Clear Bits RDV for RBUF0 */
#define USIC_FMR_CRDV1              (1 << 15) /* Bit 15: Clear Bit RDV for RBUF1 */
#define USIC_FMR_SIO0               (1 << 16) /* Bit 16: Set Interrupt Output SR0 */
#define USIC_FMR_SIO1               (1 << 17) /* Bit 17: Set Interrupt Output SR1 */
#define USIC_FMR_SIO2               (1 << 18) /* Bit 18: Set Interrupt Output SR2 */
#define USIC_FMR_SIO3               (1 << 19) /* Bit 19: Set Interrupt Output SR3 */
#define USIC_FMR_SIO4               (1 << 20) /* Bit 20: Set Interrupt Output SR4 */
#define USIC_FMR_SIO5               (1 << 21) /* Bit 21: Set Interrupt Output SR5 */

/* Transmit Buffer (32 x 4-bytes) */

#define USIC_TBUF_TDATA_SHIFT       (0)       /* Bits 0-15: Transmit Data */
#define USIC_TBUF_TDATA_MASK        (0xffff << USIC_TBUF_TDATA_SHIFT)

/* USIC FIFO Registers */

/* Bypass Data Register */

#define USIC_BYP_BDATA_SHIFT        (0)       /* Bits 0-15: Bypass Data */
#define USIC_BYP_BDATA_MASK         (0xffff << USIC_BYP_BDATA_SHIFT)

/* Bypass Control Register */

#define USIC_BYPCR_BWLE_SHIFT       (0)       /* Bits 0-3: Bypass Word Length */
#define USIC_BYPCR_BWLE_MASK        (15 << USIC_BYPCR_BWLE_SHIFT)
#  define USIC_BYPCR_BWLE(n)        ((uint32_t)((n)-1) << USIC_BYPCR_BWLE_SHIFT)
#define USIC_BYPCR_BDSSM            (1 << 8)  /* Bit 8:  Bypass Data Single Shot Mode */
#define USIC_BYPCR_BDEN_SHIFT       (10)      /* Bits 10-11: Bypass Data Enable */
#define USIC_BYPCR_BDEN_MASK        (3 << USIC_BYPCR_BDEN_SHIFT)
#  define USIC_BYPCR_BDEN_DISABLE   (0 << USIC_BYPCR_BDEN_SHIFT) /* Transfer of bypass data is disabled */
#  define USIC_BYPCR_BDEN_ENABLED   (1 << USIC_BYPCR_BDEN_SHIFT) /* Transfer of bypass data to TBUF if BDV = 1 */
#  define USIC_BYPCR_BDEN_GATED0    (2 << USIC_BYPCR_BDEN_SHIFT) /* Bypass data transferred if BDV = 1 and DX2S = 0 */
#  define USIC_BYPCR_BDEN_GATED1    (3 << USIC_BYPCR_BDEN_SHIFT) /* Bypass data transferred if BDV = 1 and DX2S = 1 */

#define USIC_BYPCR_BDVTR            (1 << 12) /* Bit 12: Bypass Data Valid Trigger */
#define USIC_BYPCR_BPRIO            (1 << 13) /* Bit 13: Bypass Priority */
#define USIC_BYPCR_BDV              (1 << 15) /* Bit 15: Bypass Data Valid */
#define USIC_BYPCR_BSELO_SHIFT      (16)      /* Bits 16-20: Bypass Select Outputs */
#define USIC_BYPCR_BSELO_MASK       (31 << USIC_BYPCR_BSELO_SHIFT)
#  define USIC_BYPCR_BSELO(n)       ((uint32_t)(n) << USIC_BYPCR_BSELO_SHIFT)
#define USIC_BYPCR_BHPC_SHIFT       (21)      /* Bits 21-23: Bypass Hardware Port Control */
#define USIC_BYPCR_BHPC_MASK        (7 << USIC_BYPCR_BHPC_SHIFT)
#  define USIC_BYPCR_BHPC(n)        ((uint32_t)(n) << USIC_BYPCR_BHPC_SHIFT)

/* Transmitter Buffer Control Register */

#define USIC_TBCTR_DPTR_SHIFT       (0)       /* Bits 0-1: Data Pointer */
#define USIC_TBCTR_DPTR_MASK        (3 << USIC_TBCTR_DPTR_SHIFT)
#  define USIC_TBCTR_DPTR(n)        ((uint32_t)(n) << USIC_TBCTR_DPTR_SHIFT)
#define USIC_TBCTR_LIMIT_SHIFT      (8)       /* Bits 8-13: Limit For Interrupt Generation */
#define USIC_TBCTR_LIMIT_MASK       (0x3f << USIC_TBCTR_LIMIT_SHIFT)
#  define USIC_TBCTR_LIMIT(n)       ((uint32_t)(n) << USIC_TBCTR_LIMIT_SHIFT)
#define USIC_TBCTR_STBTM            (1 << 14) /* Bit 14: Standard Transmit Buffer Trigger Mode */
#define USIC_TBCTR_STBTEN           (1 << 15) /* Bit 15: Standard Transmit Buffer Trigger Enable */
#define USIC_TBCTR_STBINP_SHIFT     (16)      /* Bits 16-18: Standard Transmit Buffer Interrupt Node Pointer */
#define USIC_TBCTR_STBINP_MASK      (7 << USIC_TBCTR_STBINP_SHIFT)
#  define USIC_TBCTR_STBINP_SR0     (0 << USIC_TBCTR_STBINP_SHIFT) /* Output SR0 becomes activated */
#  define USIC_TBCTR_STBINP_SR1     (1 << USIC_TBCTR_STBINP_SHIFT) /* Output SR1 becomes activated */
#  define USIC_TBCTR_STBINP_SR2     (2 << USIC_TBCTR_STBINP_SHIFT) /* Output SR2 becomes activated */
#  define USIC_TBCTR_STBINP_SR3     (3 << USIC_TBCTR_STBINP_SHIFT) /* Output SR3 becomes activated */
#  define USIC_TBCTR_STBINP_SR4     (4 << USIC_TBCTR_STBINP_SHIFT) /* Output SR4 becomes activated */
#  define USIC_TBCTR_STBINP_SR5     (5 << USIC_TBCTR_STBINP_SHIFT) /* Output SR5 becomes activated */

#define USIC_TBCTR_ATBINP_SHIFT     (19)      /* Bits 19-21: Alternative Transmit Buffer Interrupt Node Pointer */
#define USIC_TBCTR_ATBINP_MASK      (7 << USIC_TBCTR_ATBINP_SHIFT)
#  define USIC_TBCTR_ATBINP_SR0     (0 << USIC_TBCTR_ATBINP_SHIFT) /* Output SR0 becomes activated */
#  define USIC_TBCTR_ATBINP_SR1     (1 << USIC_TBCTR_ATBINP_SHIFT) /* Output SR1 becomes activated */
#  define USIC_TBCTR_ATBINP_SR2     (2 << USIC_TBCTR_ATBINP_SHIFT) /* Output SR2 becomes activated */
#  define USIC_TBCTR_ATBINP_SR3     (3 << USIC_TBCTR_ATBINP_SHIFT) /* Output SR3 becomes activated */
#  define USIC_TBCTR_ATBINP_SR4     (4 << USIC_TBCTR_ATBINP_SHIFT) /* Output SR4 becomes activated */
#  define USIC_TBCTR_ATBINP_SR5     (5 << USIC_TBCTR_ATBINP_SHIFT) /* Output SR5 becomes activated */

#define USIC_TBCTR_SIZE_SHIFT       (24)      /* Bits 24-26: Buffer Size */
#define USIC_TBCTR_SIZE_MASK        (7 << USIC_TBCTR_SIZE_SHIFT)
#  define USIC_TBCTR_SIZE_DISABLE   (0 << USIC_TBCTR_SIZE_SHIFT) /* FIFO mechanism is disabled */
#  define USIC_TBCTR_SIZE_2         (1 << USIC_TBCTR_SIZE_SHIFT) /* FIFO buffer contains 2 entries */
#  define USIC_TBCTR_SIZE_4         (2 << USIC_TBCTR_SIZE_SHIFT) /* FIFO buffer contains 4 entries */
#  define USIC_TBCTR_SIZE_8         (3 << USIC_TBCTR_SIZE_SHIFT) /* FIFO buffer contains 8 entries */
#  define USIC_TBCTR_SIZE_16        (4 << USIC_TBCTR_SIZE_SHIFT) /* FIFO buffer contains 16 entries */
#  define USIC_TBCTR_SIZE_32        (5 << USIC_TBCTR_SIZE_SHIFT) /* FIFO buffer contains 32 entries */
#  define USIC_TBCTR_SIZE_64        (6 << USIC_TBCTR_SIZE_SHIFT) /* FIFO buffer contains 64 entries */

#define USIC_TBCTR_LOF              (1 << 28) /* Bit 28: Buffer Event on Limit Overflow */
#define USIC_TBCTR_STBIEN           (1 << 30) /* Bit 30: Standard Transmit Buffer Interrupt Enable */
#define USIC_TBCTR_TBERIEN          (1 << 31) /* Bit 31: Transmit Buffer Error Interrupt Enable */

/* Receiver Buffer Control Register */

#define USIC_RBCTR_DPTR_SHIFT       (0)       /* Bits 0-5: Data Pointer */
#define USIC_RBCTR_DPTR_MASK        (0x3f << USIC_RBCTR_DPTR_SHIFT)
#  define USIC_RBCTR_DPTR(n)        ((uint32_t)(n) << USIC_RBCTR_DPTR_SHIFT)
#define USIC_RBCTR_LIMIT_SHIFT      (8)       /* Bits 8-13: Limit For Interrupt Generation */
#define USIC_RBCTR_LIMIT_MASK       (0x3f << USIC_RBCTR_LIMIT_SHIFT)
#  define USIC_RBCTR_LIMIT(n)       ((uint32_t)(n) << USIC_RBCTR_LIMIT_SHIFT)
#define USIC_RBCTR_SRBTM            (1 << 14) /* Bit 14: Standard Receive Buffer Trigger Mode */
#define USIC_RBCTR_SRBTEN           (1 << 15) /* Bit 15: Standard Receive Buffer Trigger Enable */
#define USIC_RBCTR_SRBINP_SHIFT     (16)      /* Bits 16-18: Standard Receive Buffer Interrupt Node Pointer */
#define USIC_RBCTR_SRBINP_MASK      (7 << USIC_RBCTR_SRBINP_SHIFT)
#  define USIC_RBCTR_SRBINP_SR0     (0 << USIC_RBCTR_SRBINP_SHIFT) /* Output SR0 becomes activated */
#  define USIC_RBCTR_SRBINP_SR1     (1 << USIC_RBCTR_SRBINP_SHIFT) /* Output SR1 becomes activated */
#  define USIC_RBCTR_SRBINP_SR2     (2 << USIC_RBCTR_SRBINP_SHIFT) /* Output SR2 becomes activated */
#  define USIC_RBCTR_SRBINP_SR3     (3 << USIC_RBCTR_SRBINP_SHIFT) /* Output SR3 becomes activated */
#  define USIC_RBCTR_SRBINP_SR4     (4 << USIC_RBCTR_SRBINP_SHIFT) /* Output SR4 becomes activated */
#  define USIC_RBCTR_SRBINP_SR5     (5 << USIC_RBCTR_SRBINP_SHIFT) /* Output SR5 becomes activated */

#define USIC_RBCTR_ARBINP_SHIFT     (19)      /* Bits 19-21: Alternative Receive Buffer Interrupt Node Pointer */
#define USIC_RBCTR_ARBINP_MASK      (7 << USIC_RBCTR_ARBINP_SHIFT)
#  define USIC_RBCTR_ARBINP_SR0     (0 << USIC_RBCTR_ARBINP_SHIFT) /* Output SR0 becomes activated */
#  define USIC_RBCTR_ARBINP_SR1     (1 << USIC_RBCTR_ARBINP_SHIFT) /* Output SR1 becomes activated */
#  define USIC_RBCTR_ARBINP_SR2     (2 << USIC_RBCTR_ARBINP_SHIFT) /* Output SR2 becomes activated */
#  define USIC_RBCTR_ARBINP_SR3     (3 << USIC_RBCTR_ARBINP_SHIFT) /* Output SR3 becomes activated */
#  define USIC_RBCTR_ARBINP_SR4     (4 << USIC_RBCTR_ARBINP_SHIFT) /* Output SR4 becomes activated */
#  define USIC_RBCTR_ARBINP_SR5     (5 << USIC_RBCTR_ARBINP_SHIFT) /* Output SR5 becomes activated */

#define USIC_RBCTR_RCIM_SHIFT       (22)      /* Bits 22-23: Receiver Control Information Mode */
#define USIC_RBCTR_RCIM_MASK        (3 << USIC_RBCTR_RCIM_SHIFT)
#  define USIC_RBCTR_RCIM_MODE0     (0 << USIC_RBCTR_RCIM_SHIFT) /* RCI[4] = PERR, RCI[3:0] = WLEN */
#  define USIC_RBCTR_RCIM_MODE1     (1 << USIC_RBCTR_RCIM_SHIFT) /* RCI[4] = SOF, RCI[3:0] = WLEN */
#  define USIC_RBCTR_RCIM_MODE2     (2 << USIC_RBCTR_RCIM_SHIFT) /* RCI[4] = 0, RCI[3:0] = WLEN */
#  define USIC_RBCTR_RCIM_MODE3     (3 << USIC_RBCTR_RCIM_SHIFT) /* RCI[4] = PERR, RCI[3] = PAR,
                                                                  * RCI[2:1] = 0, RCI[0] = SOF */

#define USIC_RBCTR_SIZE_SHIFT       (24)      /* Bits 24-26: Buffer Size */
#define USIC_RBCTR_SIZE_MASK        (7 << USIC_RBCTR_SIZE_SHIFT)
#  define USIC_RBCTR_SIZE_DISABLE   (0 << USIC_RBCTR_SIZE_SHIFT) /* FIFO mechanism is disabled */
#  define USIC_RBCTR_SIZE_2         (1 << USIC_RBCTR_SIZE_SHIFT) /* FIFO buffer contains 2 entries */
#  define USIC_RBCTR_SIZE_4         (2 << USIC_RBCTR_SIZE_SHIFT) /* FIFO buffer contains 4 entries */
#  define USIC_RBCTR_SIZE_8         (3 << USIC_RBCTR_SIZE_SHIFT) /* FIFO buffer contains 8 entries */
#  define USIC_RBCTR_SIZE_16        (4 << USIC_RBCTR_SIZE_SHIFT) /* FIFO buffer contains 16 entries */
#  define USIC_RBCTR_SIZE_32        (5 << USIC_RBCTR_SIZE_SHIFT) /* FIFO buffer contains 32 entries */
#  define USIC_RBCTR_SIZE_64        (6 << USIC_RBCTR_SIZE_SHIFT) /* FIFO buffer contains 64 entries */

#define USIC_RBCTR_RNM              (1 << 27) /* Bit 27: Receiver Notification Mode */
#define USIC_RBCTR_LOF              (1 << 28) /* Bit 28: Buffer Event on Limit Overflow */
#define USIC_RBCTR_ARBIEN           (1 << 29) /* Bit 29: Alternative Receive Buffer Interrupt Enable */
#define USIC_RBCTR_SRBIEN           (1 << 30) /* Bit 30: Standard Receive Buffer Interrupt Enable */
#define USIC_RBCTR_RBERIEN          (1 << 31) /* Bit 31: Receive Buffer Error Interrupt Enable */

/* Transmit/Receive Buffer Pointer Register */

#define USIC_TRBPTR_TDIPTR_SHIFT    (0)       /* Bits 0-5: Transmitter Data Input Pointer */
#define USIC_TRBPTR_TDIPTR_MASK     (0x3f << USIC_TRBPTR_TDIPTR_SHIFT)
#define USIC_TRBPTR_TDOPTR_SHIFT    (8)       /* Bits 8-13: Transmitter Data Output Pointer */
#define USIC_TRBPTR_TDOPTR_MASK     (0x3f << USIC_TRBPTR_TDOPTR_SHIFT)
#define USIC_TRBPTR_RDIPTR_SHIFT    (16)      /* Bits 16-21: Receiver Data Input Pointer */
#define USIC_TRBPTR_RDIPTR_MASK     (0x3f << USIC_TRBPTR_RDIPTR_SHIFT)
#define USIC_TRBPTR_RDOPTR_SHIFT    (24)      /* Bits 24-29: Receiver Data Output Pointer */
#define USIC_TRBPTR_RDOPTR_MASK     (0x3f << USIC_TRBPTR_RDOPTR_SHIFT)

/* Transmit/Receive Buffer Status Register */

#define USIC_TRBSR_SRBI             (1 << 0)  /* Bit 0:  Standard Receive Buffer Event */
#define USIC_TRBSR_RBERI            (1 << 1)  /* Bit 1:  Receive Buffer Error Event */
#define USIC_TRBSR_ARBI             (1 << 2)  /* Bit 2:  Alternative Receive Buffer Event */
#define USIC_TRBSR_REMPTY           (1 << 3)  /* Bit 3:  Receive Buffer Empty */
#define USIC_TRBSR_RFULL            (1 << 4)  /* Bit 4:  Receive Buffer Full */
#define USIC_TRBSR_RBUS             (1 << 5)  /* Bit 5:  Receive Buffer Busy */
#define USIC_TRBSR_SRBT             (1 << 6)  /* Bit 6:  Standard Receive Buffer Event Trigger */
#define USIC_TRBSR_STBI             (1 << 8)  /* Bit 8:  Standard Transmit Buffer Event */
#define USIC_TRBSR_TBERI            (1 << 9)  /* Bit 9:  Transmit Buffer Error Event */
#define USIC_TRBSR_TEMPTY           (1 << 11) /* Bit 11: Transmit Buffer Empty */
#define USIC_TRBSR_TFULL            (1 << 12) /* Bit 12: Transmit Buffer Full */
#define USIC_TRBSR_TBUS             (1 << 13) /* Bit 13: Transmit Buffer Busy */
#define USIC_TRBSR_STBT             (1 << 14) /* Bit 14: Standard Transmit Buffer Event Trigger */
#define USIC_TRBSR_RBFLVL_SHIFT     (16)      /* Bits 16-22: Receive Buffer Filling Level */
#define USIC_TRBSR_RBFLVL_MASK      (0x7f << USIC_TRBSR_RBFLVL_SHIFT)
#  define USIC_TRBSR_RBFLVL(n)      ((uint32_t)(n) << USIC_TRBSR_RBFLVL_SHIFT)
#define USIC_TRBSR_TBFLVL_SHIFT     (24)      /* Bits 22-28: Transmit Buffer Filling Level */
#define USIC_TRBSR_TBFLVL_MASK      (0x7f << USIC_TRBSR_TBFLVL_SHIFT)
#  define USIC_TRBSR_TBFLVL(n)      ((uint32_t)(n) << USIC_TRBSR_TBFLVL_SHIFT)

/* Transmit/Receive Buffer Status Clear Register */

#define USIC_TRBSCR_CSRBI           (1 << 0)  /* Bit 0:  Clear Standard Receive Buffer Event */
#define USIC_TRBSCR_CRBERI          (1 << 1)  /* Bit 1:  Clear Receive Buffer Error Event */
#define USIC_TRBSCR_CARBI           (1 << 2)  /* Bit 2:  Clear Alternative Receive Buffer Event */
#define USIC_TRBSCR_CSTBI           (1 << 8)  /* Bit 8:  Clear Standard Transmit Buffer Event */
#define USIC_TRBSCR_CTBERI          (1 << 9)  /* Bit 9:  Clear Transmit Buffer Error Event */
#define USIC_TRBSCR_CBDV            (1 << 10) /* Bit 10: Clear Bypass Data Valid */
#define USIC_TRBSCR_FLUSHRB         (1 << 14) /* Bit 14: Flush Receive Buffer */
#define USIC_TRBSCR_FLUSHTB         (1 << 15) /* Bit 15: Flush Transmit Buffer */

/* Receiver Buffer Output Register */

#define USIC_OUTR_DSR_SHIFT         (0)       /* Bits 0-15: Received Data */
#define USIC_OUTR_DSR_MASK          (0xffff << USIC_OUTR_DSR_SHIFT)
#define USIC_OUTR_RCI_SHIFT         (16)      /* Bits 16-20: Receiver Control Information */
#define USIC_OUTR_RCI_MASK          (31 << USIC_OUTR_RCI_SHIFT)

/* Receiver Buffer Output Register L for Debugger */

#define USIC_OUTDR_DSR_SHIFT        (0)       /* Bits 0-15: Data from Shift Register */
#define USIC_OUTDR_DSR_MASK         (0xffff << USIC_OUTDR_DSR_SHIFT)
#define USIC_OUTDR_RCI_SHIFT        (16)      /* Bits 16-30: Receive Control Information from Shift Register */
#define USIC_OUTDR_RCI_MASK         (31 << USIC_OUTDR_RCI_SHIFT)

/* Transmit FIFO Buffer (32 x 4-bytes) */

#define USIC_IN_TDATA_SHIFT         (0)      /* Bits 0-15: Transmit Data */
#define USIC_IN_TDATA_MASK          (0xffff << USIC_IN_TDATA_SHIFT)

#endif /* __ARCH_ARM_SRC_XMC4_HARDWARE_XMC4_USIC_H */
