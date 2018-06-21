/********************************************************************************************
 * arch/arm/src/imxrt/imxrt_lpi2c.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Ivan Ucherdzhiev <ivanucherdjiev@gmail.com>
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_CHIP_IMXRT_LPI2C_H
#define __ARCH_ARM_SRC_IMXRT_CHIP_IMXRT_LPI2C_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* Register offsets *************************************************************************/

#define IMXRT_LPI2C_VERID_OFFSET             0x0000  /* Version ID Register offset */
#define IMXRT_LPI2C_PARAM_OFFSET             0x0004  /* Parameter Register offset */
#define IMXRT_LPI2C_MCR_OFFSET               0x0010  /* Master Control Register offset */
#define IMXRT_LPI2C_MSR_OFFSET               0x0014  /* Master Status Register offset */
#define IMXRT_LPI2C_MIER_OFFSET              0x0018  /* Master Interrupt Enable Register offset */
#define IMXRT_LPI2C_MDER_OFFSET              0x001c  /* Master DMA Enable Register offset */
#define IMXRT_LPI2C_MCFGR0_OFFSET            0x0020  /* Master Config Register 0 offset */
#define IMXRT_LPI2C_MCFGR1_OFFSET            0x0024  /* Master Config Register 1 offset */
#define IMXRT_LPI2C_MCFGR2_OFFSET            0x0028  /* Master Config Register 2 offset */
#define IMXRT_LPI2C_MCFGR3_OFFSET            0x002c  /* Master Config Register 3 offset */
#define IMXRT_LPI2C_MDMR_OFFSET              0x0040  /* Master Data Match Register offset */
#define IMXRT_LPI2C_MCCR0_OFFSET             0x0048  /* Master Clock Configuration Register 0 offset */
#define IMXRT_LPI2C_MCCR1_OFFSET             0x0050  /* Master Clock Configuration Register 1 offset */
#define IMXRT_LPI2C_MFCR_OFFSET              0x0058  /* Master FIFO Control Register offset */
#define IMXRT_LPI2C_MFSR_OFFSET              0x005C  /* Master FIFO Status Register offset */
#define IMXRT_LPI2C_MTDR_OFFSET              0x0060  /* Master Transmit Data Register offset */
#define IMXRT_LPI2C_MRDR_OFFSET              0x0070  /* Master Receive Data Register offset */
#define IMXRT_LPI2C_SCR_OFFSET               0x0110  /* Slave Control Register offset */
#define IMXRT_LPI2C_SSR_OFFSET               0x0114  /* Slave Status Register offset */
#define IMXRT_LPI2C_SIER_OFFSET              0x0118  /* Slave Interrupt Enable Register offset */
#define IMXRT_LPI2C_SDER_OFFSET              0x011c  /* Slave DMA Enable Register offset */
#define IMXRT_LPI2C_SCFGR1_OFFSET            0x0124  /* Slave Config Register 1 offset */
#define IMXRT_LPI2C_SCFGR2_OFFSET            0x0128  /* Slave Config Register 2 offset */
#define IMXRT_LPI2C_SAMR_OFFSET              0x0140  /* Slave Address Match Register offset */
#define IMXRT_LPI2C_SASR_OFFSET              0x0150  /* Slave Address Status Register offset */
#define IMXRT_LPI2C_STAR_OFFSET              0x0154  /* Slave Transmit ACK Register offset */
#define IMXRT_LPI2C_STDR_OFFSET              0x0160  /* Slave Transmit Data Register offset */
#define IMXRT_LPI2C_SRDR_OFFSET              0x0170  /* Slave Receive Data Register offset */

/* Register addresses ***********************************************************************/

/* LPI2C1 Registers */

#define IMXRT_LPI2C1_VERID                  (IMXRT_LPI2C1_BASE + IMXRT_LPI2C_VERID_OFFSET)   /* Version ID Register */
#define IMXRT_LPI2C1_PARAM                  (IMXRT_LPI2C1_BASE + IMXRT_LPI2C_PARAM_OFFSET)   /* Parameter Register  */
#define IMXRT_LPI2C1_MCR                    (IMXRT_LPI2C1_BASE + IMXRT_LPI2C_MCR_OFFSET)     /* Master Control Register  */
#define IMXRT_LPI2C1_MSR                    (IMXRT_LPI2C1_BASE + IMXRT_LPI2C_MSR_OFFSET)     /* Master Status Register  */
#define IMXRT_LPI2C1_MIER                   (IMXRT_LPI2C1_BASE + IMXRT_LPI2C_MIER_OFFSET)    /* Master Interrupt Enable Register  */
#define IMXRT_LPI2C1_MDER                   (IMXRT_LPI2C1_BASE + IMXRT_LPI2C_MDER_OFFSET)    /* Master DMA Enable Register  */
#define IMXRT_LPI2C1_MCFGR0                 (IMXRT_LPI2C1_BASE + IMXRT_LPI2C_MCFGR0_OFFSET)  /* Master Config Register 0  */
#define IMXRT_LPI2C1_MCFGR1                 (IMXRT_LPI2C1_BASE + IMXRT_LPI2C_MCFGR1_OFFSET)  /* Master Config Register 1  */
#define IMXRT_LPI2C1_MCFGR2                 (IMXRT_LPI2C1_BASE + IMXRT_LPI2C_MCFGR2_OFFSET)  /* Master Config Register 2  */
#define IMXRT_LPI2C1_MCFGR3                 (IMXRT_LPI2C1_BASE + IMXRT_LPI2C_MCFGR3_OFFSET)  /* Master Config Register 3  */
#define IMXRT_LPI2C1_MDMR                   (IMXRT_LPI2C1_BASE + IMXRT_LPI2C_MDMR_OFFSET)    /* Master Data Match Register  */
#define IMXRT_LPI2C1_MCCR0                  (IMXRT_LPI2C1_BASE + IMXRT_LPI2C_MCCR0_OFFSET)   /* Master Clock Configuration Register 0  */
#define IMXRT_LPI2C1_MCCR1                  (IMXRT_LPI2C1_BASE + IMXRT_LPI2C_MCCR1_OFFSET)   /* Master Clock Configuration Register 1  */
#define IMXRT_LPI2C1_MFCR                   (IMXRT_LPI2C1_BASE + IMXRT_LPI2C_MFCR_OFFSET)    /* Master FIFO Control Register  */
#define IMXRT_LPI2C1_MFSR                   (IMXRT_LPI2C1_BASE + IMXRT_LPI2C_MFSR_OFFSET)    /* Master FIFO Status Register  */
#define IMXRT_LPI2C1_MTDR                   (IMXRT_LPI2C1_BASE + IMXRT_LPI2C_MTDR_OFFSET)    /* Master Transmit Data Register  */
#define IMXRT_LPI2C1_MRDR                   (IMXRT_LPI2C1_BASE + IMXRT_LPI2C_MRDR_OFFSET)    /* Master Receive Data Register  */
#define IMXRT_LPI2C1_SCR                    (IMXRT_LPI2C1_BASE + IMXRT_LPI2C_SCR_OFFSET)     /* Slave Control Register  */
#define IMXRT_LPI2C1_SSR                    (IMXRT_LPI2C1_BASE + IMXRT_LPI2C_SSR_OFFSET)     /* Slave Status Register  */
#define IMXRT_LPI2C1_SIER                   (IMXRT_LPI2C1_BASE + IMXRT_LPI2C_SIER_OFFSET)    /* Slave Interrupt Enable Register  */
#define IMXRT_LPI2C1_SDER                   (IMXRT_LPI2C1_BASE + IMXRT_LPI2C_SDER_OFFSET)    /* Slave DMA Enable Register  */
#define IMXRT_LPI2C1_SCFGR1                 (IMXRT_LPI2C1_BASE + IMXRT_LPI2C_SCFGR1_OFFSET)  /* Slave Config Register 1  */
#define IMXRT_LPI2C1_SCFGR2                 (IMXRT_LPI2C1_BASE + IMXRT_LPI2C_SCFGR2_OFFSET)  /* Slave Config Register 2  */
#define IMXRT_LPI2C1_SAMR                   (IMXRT_LPI2C1_BASE + IMXRT_LPI2C_SAMR_OFFSET)    /* Slave Address Match Register  */
#define IMXRT_LPI2C1_SASR                   (IMXRT_LPI2C1_BASE + IMXRT_LPI2C_SASR_OFFSET)    /* Slave Address Status Register  */
#define IMXRT_LPI2C1_STAR                   (IMXRT_LPI2C1_BASE + IMXRT_LPI2C_STAR_OFFSET)    /* Slave Transmit ACK Register  */
#define IMXRT_LPI2C1_STDR                   (IMXRT_LPI2C1_BASE + IMXRT_LPI2C_STDR_OFFSET)    /* Slave Transmit Data Register  */
#define IMXRT_LPI2C1_SRDR                   (IMXRT_LPI2C1_BASE + IMXRT_LPI2C_SRDR_OFFSET)    /* Slave Receive Data Register  */

/* LPI2C2 Registers */

#define IMXRT_LPI2C2_VERID                  (IMXRT_LPI2C2_BASE + IMXRT_LPI2C_VERID_OFFSET)   /* Version ID Register */
#define IMXRT_LPI2C2_PARAM                  (IMXRT_LPI2C2_BASE + IMXRT_LPI2C_PARAM_OFFSET)   /* Parameter Register  */
#define IMXRT_LPI2C2_MCR                    (IMXRT_LPI2C2_BASE + IMXRT_LPI2C_MCR_OFFSET)     /* Master Control Register  */
#define IMXRT_LPI2C2_MSR                    (IMXRT_LPI2C2_BASE + IMXRT_LPI2C_MSR_OFFSET)     /* Master Status Register  */
#define IMXRT_LPI2C2_MIER                   (IMXRT_LPI2C2_BASE + IMXRT_LPI2C_MIER_OFFSET)    /* Master Interrupt Enable Register  */
#define IMXRT_LPI2C2_MDER                   (IMXRT_LPI2C2_BASE + IMXRT_LPI2C_MDER_OFFSET)    /* Master DMA Enable Register  */
#define IMXRT_LPI2C2_MCFGR0                 (IMXRT_LPI2C2_BASE + IMXRT_LPI2C_MCFGR0_OFFSET)  /* Master Config Register 0  */
#define IMXRT_LPI2C2_MCFGR1                 (IMXRT_LPI2C2_BASE + IMXRT_LPI2C_MCFGR1_OFFSET)  /* Master Config Register 1  */
#define IMXRT_LPI2C2_MCFGR2                 (IMXRT_LPI2C2_BASE + IMXRT_LPI2C_MCFGR2_OFFSET)  /* Master Config Register 2  */
#define IMXRT_LPI2C2_MCFGR3                 (IMXRT_LPI2C2_BASE + IMXRT_LPI2C_MCFGR3_OFFSET)  /* Master Config Register 3  */
#define IMXRT_LPI2C2_MDMR                   (IMXRT_LPI2C2_BASE + IMXRT_LPI2C_MDMR_OFFSET)    /* Master Data Match Register  */
#define IMXRT_LPI2C2_MCCR0                  (IMXRT_LPI2C2_BASE + IMXRT_LPI2C_MCCR0_OFFSET)   /* Master Clock Configuration Register 0  */
#define IMXRT_LPI2C2_MCCR1                  (IMXRT_LPI2C2_BASE + IMXRT_LPI2C_MCCR1_OFFSET)   /* Master Clock Configuration Register 1  */
#define IMXRT_LPI2C2_MFCR                   (IMXRT_LPI2C2_BASE + IMXRT_LPI2C_MFCR_OFFSET)    /* Master FIFO Control Register  */
#define IMXRT_LPI2C2_MFSR                   (IMXRT_LPI2C2_BASE + IMXRT_LPI2C_MFSR_OFFSET)    /* Master FIFO Status Register  */
#define IMXRT_LPI2C2_MTDR                   (IMXRT_LPI2C2_BASE + IMXRT_LPI2C_MTDR_OFFSET)    /* Master Transmit Data Register  */
#define IMXRT_LPI2C2_MRDR                   (IMXRT_LPI2C2_BASE + IMXRT_LPI2C_MRDR_OFFSET)    /* Master Receive Data Register  */
#define IMXRT_LPI2C2_SCR                    (IMXRT_LPI2C2_BASE + IMXRT_LPI2C_SCR_OFFSET)     /* Slave Control Register  */
#define IMXRT_LPI2C2_SSR                    (IMXRT_LPI2C2_BASE + IMXRT_LPI2C_SSR_OFFSET)     /* Slave Status Register  */
#define IMXRT_LPI2C2_SIER                   (IMXRT_LPI2C2_BASE + IMXRT_LPI2C_SIER_OFFSET)    /* Slave Interrupt Enable Register  */
#define IMXRT_LPI2C2_SDER                   (IMXRT_LPI2C2_BASE + IMXRT_LPI2C_SDER_OFFSET)    /* Slave DMA Enable Register  */
#define IMXRT_LPI2C2_SCFGR1                 (IMXRT_LPI2C2_BASE + IMXRT_LPI2C_SCFGR1_OFFSET)  /* Slave Config Register 1  */
#define IMXRT_LPI2C2_SCFGR2                 (IMXRT_LPI2C2_BASE + IMXRT_LPI2C_SCFGR2_OFFSET)  /* Slave Config Register 2  */
#define IMXRT_LPI2C2_SAMR                   (IMXRT_LPI2C2_BASE + IMXRT_LPI2C_SAMR_OFFSET)    /* Slave Address Match Register  */
#define IMXRT_LPI2C2_SASR                   (IMXRT_LPI2C2_BASE + IMXRT_LPI2C_SASR_OFFSET)    /* Slave Address Status Register  */
#define IMXRT_LPI2C2_STAR                   (IMXRT_LPI2C2_BASE + IMXRT_LPI2C_STAR_OFFSET)    /* Slave Transmit ACK Register  */
#define IMXRT_LPI2C2_STDR                   (IMXRT_LPI2C2_BASE + IMXRT_LPI2C_STDR_OFFSET)    /* Slave Transmit Data Register  */
#define IMXRT_LPI2C2_SRDR                   (IMXRT_LPI2C2_BASE + IMXRT_LPI2C_SRDR_OFFSET)    /* Slave Receive Data Register  */

/* LPI2C3 Registers */

#define IMXRT_LPI2C3_VERID                  (IMXRT_LPI2C3_BASE + IMXRT_LPI2C_VERID_OFFSET)   /* Version ID Register */
#define IMXRT_LPI2C3_PARAM                  (IMXRT_LPI2C3_BASE + IMXRT_LPI2C_PARAM_OFFSET)   /* Parameter Register  */
#define IMXRT_LPI2C3_MCR                    (IMXRT_LPI2C3_BASE + IMXRT_LPI2C_MCR_OFFSET)     /* Master Control Register  */
#define IMXRT_LPI2C3_MSR                    (IMXRT_LPI2C3_BASE + IMXRT_LPI2C_MSR_OFFSET)     /* Master Status Register  */
#define IMXRT_LPI2C3_MIER                   (IMXRT_LPI2C3_BASE + IMXRT_LPI2C_MIER_OFFSET)    /* Master Interrupt Enable Register  */
#define IMXRT_LPI2C3_MDER                   (IMXRT_LPI2C3_BASE + IMXRT_LPI2C_MDER_OFFSET)    /* Master DMA Enable Register  */
#define IMXRT_LPI2C3_MCFGR0                 (IMXRT_LPI2C3_BASE + IMXRT_LPI2C_MCFGR0_OFFSET)  /* Master Config Register 0  */
#define IMXRT_LPI2C3_MCFGR1                 (IMXRT_LPI2C3_BASE + IMXRT_LPI2C_MCFGR1_OFFSET)  /* Master Config Register 1  */
#define IMXRT_LPI2C3_MCFGR2                 (IMXRT_LPI2C3_BASE + IMXRT_LPI2C_MCFGR2_OFFSET)  /* Master Config Register 2  */
#define IMXRT_LPI2C3_MCFGR3                 (IMXRT_LPI2C3_BASE + IMXRT_LPI2C_MCFGR3_OFFSET)  /* Master Config Register 3  */
#define IMXRT_LPI2C3_MDMR                   (IMXRT_LPI2C3_BASE + IMXRT_LPI2C_MDMR_OFFSET)    /* Master Data Match Register  */
#define IMXRT_LPI2C3_MCCR0                  (IMXRT_LPI2C3_BASE + IMXRT_LPI2C_MCCR0_OFFSET)   /* Master Clock Configuration Register 0  */
#define IMXRT_LPI2C3_MCCR1                  (IMXRT_LPI2C3_BASE + IMXRT_LPI2C_MCCR1_OFFSET)   /* Master Clock Configuration Register 1  */
#define IMXRT_LPI2C3_MFCR                   (IMXRT_LPI2C3_BASE + IMXRT_LPI2C_MFCR_OFFSET)    /* Master FIFO Control Register  */
#define IMXRT_LPI2C3_MFSR                   (IMXRT_LPI2C3_BASE + IMXRT_LPI2C_MFSR_OFFSET)    /* Master FIFO Status Register  */
#define IMXRT_LPI2C3_MTDR                   (IMXRT_LPI2C3_BASE + IMXRT_LPI2C_MTDR_OFFSET)    /* Master Transmit Data Register  */
#define IMXRT_LPI2C3_MRDR                   (IMXRT_LPI2C3_BASE + IMXRT_LPI2C_MRDR_OFFSET)    /* Master Receive Data Register  */
#define IMXRT_LPI2C3_SCR                    (IMXRT_LPI2C3_BASE + IMXRT_LPI2C_SCR_OFFSET)     /* Slave Control Register  */
#define IMXRT_LPI2C3_SSR                    (IMXRT_LPI2C3_BASE + IMXRT_LPI2C_SSR_OFFSET)     /* Slave Status Register  */
#define IMXRT_LPI2C3_SIER                   (IMXRT_LPI2C3_BASE + IMXRT_LPI2C_SIER_OFFSET)    /* Slave Interrupt Enable Register  */
#define IMXRT_LPI2C3_SDER                   (IMXRT_LPI2C3_BASE + IMXRT_LPI2C_SDER_OFFSET)    /* Slave DMA Enable Register  */
#define IMXRT_LPI2C3_SCFGR1                 (IMXRT_LPI2C3_BASE + IMXRT_LPI2C_SCFGR1_OFFSET)  /* Slave Config Register 1  */
#define IMXRT_LPI2C3_SCFGR2                 (IMXRT_LPI2C3_BASE + IMXRT_LPI2C_SCFGR2_OFFSET)  /* Slave Config Register 2  */
#define IMXRT_LPI2C3_SAMR                   (IMXRT_LPI2C3_BASE + IMXRT_LPI2C_SAMR_OFFSET)    /* Slave Address Match Register  */
#define IMXRT_LPI2C3_SASR                   (IMXRT_LPI2C3_BASE + IMXRT_LPI2C_SASR_OFFSET)    /* Slave Address Status Register  */
#define IMXRT_LPI2C3_STAR                   (IMXRT_LPI2C3_BASE + IMXRT_LPI2C_STAR_OFFSET)    /* Slave Transmit ACK Register  */
#define IMXRT_LPI2C3_STDR                   (IMXRT_LPI2C3_BASE + IMXRT_LPI2C_STDR_OFFSET)    /* Slave Transmit Data Register  */
#define IMXRT_LPI2C3_SRDR                   (IMXRT_LPI2C3_BASE + IMXRT_LPI2C_SRDR_OFFSET)    /* Slave Receive Data Register  */

/* LPI2C4 Registers */

#define IMXRT_LPI2C4_VERID                  (IMXRT_LPI2C4_BASE + IMXRT_LPI2C_VERID_OFFSET)   /* Version ID Register */
#define IMXRT_LPI2C4_PARAM                  (IMXRT_LPI2C4_BASE + IMXRT_LPI2C_PARAM_OFFSET)   /* Parameter Register  */
#define IMXRT_LPI2C4_MCR                    (IMXRT_LPI2C4_BASE + IMXRT_LPI2C_MCR_OFFSET)     /* Master Control Register  */
#define IMXRT_LPI2C4_MSR                    (IMXRT_LPI2C4_BASE + IMXRT_LPI2C_MSR_OFFSET)     /* Master Status Register  */
#define IMXRT_LPI2C4_MIER                   (IMXRT_LPI2C4_BASE + IMXRT_LPI2C_MIER_OFFSET)    /* Master Interrupt Enable Register  */
#define IMXRT_LPI2C4_MDER                   (IMXRT_LPI2C4_BASE + IMXRT_LPI2C_MDER_OFFSET)    /* Master DMA Enable Register  */
#define IMXRT_LPI2C4_MCFGR0                 (IMXRT_LPI2C4_BASE + IMXRT_LPI2C_MCFGR0_OFFSET)  /* Master Config Register 0  */
#define IMXRT_LPI2C4_MCFGR1                 (IMXRT_LPI2C4_BASE + IMXRT_LPI2C_MCFGR1_OFFSET)  /* Master Config Register 1  */
#define IMXRT_LPI2C4_MCFGR2                 (IMXRT_LPI2C4_BASE + IMXRT_LPI2C_MCFGR2_OFFSET)  /* Master Config Register 2  */
#define IMXRT_LPI2C4_MCFGR3                 (IMXRT_LPI2C4_BASE + IMXRT_LPI2C_MCFGR3_OFFSET)  /* Master Config Register 3  */
#define IMXRT_LPI2C4_MDMR                   (IMXRT_LPI2C4_BASE + IMXRT_LPI2C_MDMR_OFFSET)    /* Master Data Match Register  */
#define IMXRT_LPI2C4_MCCR0                  (IMXRT_LPI2C4_BASE + IMXRT_LPI2C_MCCR0_OFFSET)   /* Master Clock Configuration Register 0  */
#define IMXRT_LPI2C4_MCCR1                  (IMXRT_LPI2C4_BASE + IMXRT_LPI2C_MCCR1_OFFSET)   /* Master Clock Configuration Register 1  */
#define IMXRT_LPI2C4_MFCR                   (IMXRT_LPI2C4_BASE + IMXRT_LPI2C_MFCR_OFFSET)    /* Master FIFO Control Register  */
#define IMXRT_LPI2C4_MFSR                   (IMXRT_LPI2C4_BASE + IMXRT_LPI2C_MFSR_OFFSET)    /* Master FIFO Status Register  */
#define IMXRT_LPI2C4_MTDR                   (IMXRT_LPI2C4_BASE + IMXRT_LPI2C_MTDR_OFFSET)    /* Master Transmit Data Register  */
#define IMXRT_LPI2C4_MRDR                   (IMXRT_LPI2C4_BASE + IMXRT_LPI2C_MRDR_OFFSET)    /* Master Receive Data Register  */
#define IMXRT_LPI2C4_SCR                    (IMXRT_LPI2C4_BASE + IMXRT_LPI2C_SCR_OFFSET)     /* Slave Control Register  */
#define IMXRT_LPI2C4_SSR                    (IMXRT_LPI2C4_BASE + IMXRT_LPI2C_SSR_OFFSET)     /* Slave Status Register  */
#define IMXRT_LPI2C4_SIER                   (IMXRT_LPI2C4_BASE + IMXRT_LPI2C_SIER_OFFSET)    /* Slave Interrupt Enable Register  */
#define IMXRT_LPI2C4_SDER                   (IMXRT_LPI2C4_BASE + IMXRT_LPI2C_SDER_OFFSET)    /* Slave DMA Enable Register  */
#define IMXRT_LPI2C4_SCFGR1                 (IMXRT_LPI2C4_BASE + IMXRT_LPI2C_SCFGR1_OFFSET)  /* Slave Config Register 1  */
#define IMXRT_LPI2C4_SCFGR2                 (IMXRT_LPI2C4_BASE + IMXRT_LPI2C_SCFGR2_OFFSET)  /* Slave Config Register 2  */
#define IMXRT_LPI2C4_SAMR                   (IMXRT_LPI2C4_BASE + IMXRT_LPI2C_SAMR_OFFSET)    /* Slave Address Match Register  */
#define IMXRT_LPI2C4_SASR                   (IMXRT_LPI2C4_BASE + IMXRT_LPI2C_SASR_OFFSET)    /* Slave Address Status Register  */
#define IMXRT_LPI2C4_STAR                   (IMXRT_LPI2C4_BASE + IMXRT_LPI2C_STAR_OFFSET)    /* Slave Transmit ACK Register  */
#define IMXRT_LPI2C4_STDR                   (IMXRT_LPI2C4_BASE + IMXRT_LPI2C_STDR_OFFSET)    /* Slave Transmit Data Register  */
#define IMXRT_LPI2C4_SRDR                   (IMXRT_LPI2C4_BASE + IMXRT_LPI2C_SRDR_OFFSET)    /* Slave Receive Data Register  */

/* Register bit definitions *****************************************************************/

/* LPI2C Version ID Register */

#define LPI2C_VERID_FEATURE_SHIFT           (0)
#define LPI2C_VERID_FEATURE_MASK            (0xffff << LPI2C_VERID_FEATURE_SHIFT)
#define LPI2C_VERID_MINOR_SHIFT             (16)
#define LPI2C_VERID_MINOR_MASK              (0xff << LPI2C_VERID_MINOR_SHIFT)
#define LPI2C_VERID_MAJOR_SHIFT             (24)
#define LPI2C_VERID_MAJOR_MASK              (0xff << LPI2C_VERID_MAJOR_SHIFT)

/* LPI2C Parameter Register  */

#define LPI2C_PARAM_MTXFIFO_MASK            (0x0f) /* Config number of words in master transmit fifo to 2^MTXFIFO (pow(2,MTXFIFO )) */
#  define LPI2C_PARAM_MTXFIFO_1_WORDS       (0)
#  define LPI2C_PARAM_MTXFIFO_2_WORDS       (1)
#  define LPI2C_PARAM_MTXFIFO_4_WORDS       (2)
#  define LPI2C_PARAM_MTXFIFO_8_WORDS       (3)
#  define LPI2C_PARAM_MTXFIFO_16_WORDS      (4)
#  define LPI2C_PARAM_MTXFIFO_32_WORDS      (5)
#  define LPI2C_PARAM_MTXFIFO_64_WORDS      (6)
#  define LPI2C_PARAM_MTXFIFO_128_WORDS     (7)
#  define LPI2C_PARAM_MTXFIFO_256_WORDS     (8)
#  define LPI2C_PARAM_MTXFIFO_512_WORDS     (9)
#  define LPI2C_PARAM_MTXFIFO_1024_WORDS    (10)
#  define LPI2C_PARAM_MTXFIFO_2048_WORDS    (11)
#  define LPI2C_PARAM_MTXFIFO_4096_WORDS    (12)
#  define LPI2C_PARAM_MTXFIFO_8192_WORDS    (13)
#  define LPI2C_PARAM_MTXFIFO_16384_WORDS   (14)
#  define LPI2C_PARAM_MTXFIFO_32768_WORDS   (15)

#define LPI2C_PARAM_MRXFIFO_SHIFT           (8)
#define LPI2C_PARAM_MRXFIFO_MASK            (0x0f << LPI2C_PARAM_MRXFIFO_SHIFT) /* Config number of words in master receive fifo 2^MRXFIFO (pow(2,MTRFIFO )) */
#  define LPI2C_PARAM_MRXFIFO_1_WORDS       (0 << LPI2C_PARAM_MRXFIFO_SHIFT)
#  define LPI2C_PARAM_MRXFIFO_2_WORDS       (1 << LPI2C_PARAM_MRXFIFO_SHIFT)
#  define LPI2C_PARAM_MRXFIFO_4_WORDS       (2 << LPI2C_PARAM_MRXFIFO_SHIFT)
#  define LPI2C_PARAM_MRXFIFO_8_WORDS       (3 << LPI2C_PARAM_MRXFIFO_SHIFT)
#  define LPI2C_PARAM_MRXFIFO_16_WORDS      (4 << LPI2C_PARAM_MRXFIFO_SHIFT)
#  define LPI2C_PARAM_MRXFIFO_32_WORDS      (5 << LPI2C_PARAM_MRXFIFO_SHIFT)
#  define LPI2C_PARAM_MRXFIFO_64_WORDS      (6 << LPI2C_PARAM_MRXFIFO_SHIFT)
#  define LPI2C_PARAM_MRXFIFO_128_WORDS     (7 << LPI2C_PARAM_MRXFIFO_SHIFT)
#  define LPI2C_PARAM_MRXFIFO_256_WORDS     (8 << LPI2C_PARAM_MRXFIFO_SHIFT)
#  define LPI2C_PARAM_MRXFIFO_512_WORDS     (9 << LPI2C_PARAM_MRXFIFO_SHIFT)
#  define LPI2C_PARAM_MRXFIFO_1024_WORDS    (10 << LPI2C_PARAM_MRXFIFO_SHIFT)
#  define LPI2C_PARAM_MRXFIFO_2048_WORDS    (11 << LPI2C_PARAM_MRXFIFO_SHIFT)
#  define LPI2C_PARAM_MRXFIFO_4096_WORDS    (12 << LPI2C_PARAM_MRXFIFO_SHIFT)
#  define LPI2C_PARAM_MRXFIFO_8192_WORDS    (13 << LPI2C_PARAM_MRXFIFO_SHIFT)
#  define LPI2C_PARAM_MRXFIFO_16384_WORDS   (14 << LPI2C_PARAM_MRXFIFO_SHIFT)
#  define LPI2C_PARAM_MRXFIFO_32768_WORDS   (15 << LPI2C_PARAM_MRXFIFO_SHIFT)

/* LPI2C Master Control Register  */

#define LPI2C_MCR_MEN                       (1 << 0)  /* Master Enable Bit */
#define LPI2C_MCR_RST                       (1 << 1)  /* Software Reset Bit */
#define LPI2C_MCR_DOZEN                     (1 << 2)  /* Doze Mode Enable Bit */
#define LPI2C_MCR_DBGEN                     (1 << 3)  /* Debug Enable Bit */
                                                      /* Bits 7-4 Reserved */
#define LPI2C_MCR_RTF                       (1 << 8)  /* Reset Transmit FIFO Bit */
#define LPI2C_MCR_RRF                       (1 << 9)  /* Reset Receive FIFO Bit */
                                                      /* Bits 31-10 Reserved */

/* LPI2C Master Status Register  */

#define LPI2C_MSR_TDF                       (1 << 0)  /* Transmit Data Flag Bit */
#define LPI2C_MSR_RDF                       (1 << 1)  /* Receive Data Flag Bit */
                                                      /* Bits 7-2 Reserved */
#define LPI2C_MSR_EPF                       (1 << 8)  /* End Packet Flag Bit */
#define LPI2C_MSR_SDF                       (1 << 9)  /* STOP Detect Flag Bit */
#define LPI2C_MSR_NDF                       (1 << 10) /* NACK Detect Flag Bit */
#define LPI2C_MSR_ALF                       (1 << 11) /* Arbitration Lost Flag Bit */
#define LPI2C_MSR_FEF                       (1 << 12) /* FIFO Error Flag Bit */
#define LPI2C_MSR_PLTF                      (1 << 13) /* Pin Low Timeout Flag Bit */
#define LPI2C_MSR_DMF                       (1 << 14) /* Data Match Flag Bit */
                                                      /* Bits 23-15 Reserved */
#define LPI2C_MSR_MBF                       (1 << 24) /* Master Busy Flag Bit */
#define LPI2C_MSR_BBF                       (1 << 25) /* Bus Busy Flag Bit */
                                                      /* Bits 31-26 Reserved */

/* LPI2C Master Interrupt Enable Register  */

#define LPI2C_MIER_TDIE                     (1 << 0)  /* Transmit Data Interrupt Enable Bit */
#define LPI2C_MIER_RDIE                     (1 << 1)  /* Receive Data Interrupt Enable Bit */
                                                      /* Bits 7-2 Reserved */
#define LPI2C_MIER_EPIE                     (1 << 8)  /* End Packet Interrupt Enable Bit */
#define LPI2C_MIER_SDIE                     (1 << 9)  /* STOP Detect Interrupt Enable Bit */
#define LPI2C_MIER_NDIE                     (1 << 10) /* NACK Detect Interrupt Enable Bit */
#define LPI2C_MIER_ALIE                     (1 << 11) /* Arbitration Lost Interrupt Enable Bit */
#define LPI2C_MIER_FEIE                     (1 << 12) /* FIFO Error Interrupt Enable Bit */
#define LPI2C_MIER_PLTIE                    (1 << 13) /* Pin Low Timeout Interrupt Enable Bit */
#define LPI2C_MIER_DMIE                     (1 << 14) /* Data Match Interrupt Enable Bit */
                                                      /* Bits 31-15 Reserved */

/* LPI2C Master DMA Enable Register  */

#define LPI2C_MDER_TDDE                     (1 << 0)  /* Transmit Data DMA Enable Bit */
#define LPI2C_MDER_RDDE                     (1 << 1)  /* Transmit Data DMA Enable Bit */
                                                      /* Bits 31-2 Reserved */

/* LPI2C Master Config Register 0  */

#define LPI2C_MCFG0_HREN                    (1 << 0)  /* Host Request Enable Bit */
#define LPI2C_MCFG0_HRPOL                   (1 << 1)  /* Host Request Polarity Bit */
#define LPI2C_MCFG0_HRSEL                   (1 << 2)  /* Host Request Select Bit */
                                                      /* Bits 7-3 Reserved */
#define LPI2C_MCFG0_CIRFIFO                 (1 << 8)  /* Circular FIFO Enable Bit */
#define LPI2C_MCFG0_RDMO                    (1 << 9)  /* Receive Data Match Only Bit */
                                                      /* Bits 31-10 Reserved */

/* LPI2C Master Config Register 1  */

#define LPI2C_MCFG1_PRESCALE_MASK           (7 << 0)  /* Clock Prescaler Bit Mask */
#  define LPI2C_MCFG1_PRESCALE_1            (0)
#  define LPI2C_MCFG1_PRESCALE_2            (1)
#  define LPI2C_MCFG1_PRESCALE_4            (2)
#  define LPI2C_MCFG1_PRESCALE_8            (3)
#  define LPI2C_MCFG1_PRESCALE_16           (4)
#  define LPI2C_MCFG1_PRESCALE_32           (5)
#  define LPI2C_MCFG1_PRESCALE_64           (6)
#  define LPI2C_MCFG1_PRESCALE_128          (7)
#define LPI2C_MCFG1_AUTOSTOP                (1 << 8)  /* Automatic STOP Generation Bit */
#define LPI2C_MCFG1_IGNACK                  (1 << 9)  /* Ignore NACK Bit */
#define LPI2C_MCFG1_TIMECFG                 (1 << 10) /* Timeout Configuration Bit */
                                                      /* Bits 15-11 Reserved */
#define LPI2C_MCFG1_MATCFG_SHIFT            (16)
#define LPI2C_MCFG1_MATCFG_MASK             (7 << LPI2C_MCFG1_MATCFG_SHIFT)  /* Match Configuration Bit Mask */
#define LPI2C_MCFG1_MATCFG(n)               ((n & LPI2C_MCFG1_MATCFG_MASK) << LPI2C_MCFG1_MATCFG_SHIFT)
#  define LPI2C_MCFG1_MATCFG_DISABLE        (0 << LPI2C_MCFG1_MATCFG_SHIFT)
                                                      /* LPI2C_MCFG1_MATCFG = 001b Reserved */
#  define LPI2C_MCFG1_MATCFG2               (2 << LPI2C_MCFG1_MATCFG_SHIFT)
#  define LPI2C_MCFG1_MATCFG3               (3 << LPI2C_MCFG1_MATCFG_SHIFT)
#  define LPI2C_MCFG1_MATCFG4               (4 << LPI2C_MCFG1_MATCFG_SHIFT)
#  define LPI2C_MCFG1_MATCFG5               (5 << LPI2C_MCFG1_MATCFG_SHIFT)
#  define LPI2C_MCFG1_MATCFG6               (6 << LPI2C_MCFG1_MATCFG_SHIFT)
#  define LPI2C_MCFG1_MATCFG7               (7 << LPI2C_MCFG1_MATCFG_SHIFT)
                                                     /* Bits 23-19 Reserved */
#define LPI2C_MCFG1_PINCFG_SHIFT            (24)
#define LPI2C_MCFG1_PINCFG_MASK             (7 << LPI2C_MCFG1_PINCFG_SHIFT)  /* Pin Configuration Bit Mask */
#define LPI2C_MCFG1_PINCFG(n)               ((n & LPI2C_MCFG1_PINCFG_MASK) << LPI2C_MCFG1_PINCFG_SHIFT)
#  define LPI2C_MCFG1_PINCFG0               (0 << LPI2C_MCFG1_PINCFG_SHIFT)
#  define LPI2C_MCFG1_PINCFG1               (1 << LPI2C_MCFG1_PINCFG_SHIFT)
#  define LPI2C_MCFG1_PINCFG2               (2 << LPI2C_MCFG1_PINCFG_SHIFT)
#  define LPI2C_MCFG1_PINCFG3               (3 << LPI2C_MCFG1_PINCFG_SHIFT)
#  define LPI2C_MCFG1_PINCFG4               (4 << LPI2C_MCFG1_PINCFG_SHIFT)
#  define LPI2C_MCFG1_PINCFG5               (5 << LPI2C_MCFG1_PINCFG_SHIFT)
#  define LPI2C_MCFG1_PINCFG6               (6 << LPI2C_MCFG1_PINCFG_SHIFT)
#  define LPI2C_MCFG1_PINCFG7               (7 << LPI2C_MCFG1_PINCFG_SHIFT)
                                                     /* Bits 31-27 Reserved */

/* LPI2C Master Config Register 2  */

#define LPI2C_MCFG2_BUSIDLE_MASK            (0xfff << 0)  /* Bus Idle Timeout Period in Clock Cycles */
#define LPI2C_MCFG2_BUSIDLE_DISABLE         (0)
#define LPI2C_MCFG2_BUSIDLE(n)              (n & LPI2C_MCFG2_BUSIDLE_MASK)
                                                     /* Bits 15-12 Reserved */
#define LPI2C_MCFG2_FILTSCL_SHIFT           (16)
#define LPI2C_MCFG2_FILTSCL_MASK            (15 << LPI2C_MCFG2_FILTSCL_SHIFT)  /* Glitch Filter SCL */
#define LPI2C_MCFG2_FILTSCL_DISABLE         (0 << LPI2C_MCFG2_FILTSCL_SHIFT)
#define LPI2C_MCFG2_FILTSCL_CYCLES(n)       ((n & LPI2C_MCFG2_FILTSCL_MASK) << LPI2C_MCFG2_FILTSCL_SHIFT)
                                                     /* Bits 23-20 Reserved */
#define LPI2C_MCFG2_FILTSDA_SHIFT           (24)
#define LPI2C_MCFG2_FILTSDA_MASK            (15 << LPI2C_MCFG2_FILTSDA_SHIFT)  /* Glitch Filter SDA */
#define LPI2C_MCFG2_FILTSDA_DISABLE         (0 << LPI2C_MCFG2_FILTSDA_SHIFT)
#define LPI2C_MCFG2_FILTSDA_CYCLES(n)       ((n & LPI2C_MCFG2_FILTSDA_MASK) << LPI2C_MCFG2_FILTSDA_SHIFT)
                                                     /* Bits 31-28 Reserved */

#endif /* __ARCH_ARM_SRC_IMXRT_CHIP_IMXRT_LPI2C_H */
