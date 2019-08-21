/************************************************************************************************************
 * arch/arm/src/s32k1xx/hardware/s32k1xx_lpi2c.h
 *
 *    Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *    Author: Ivan Ucherdzhiev <ivanucherdjiev@gmail.com>
 *            Gregory Nutt <gnutt@nuttx.org>
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
 ************************************************************************************************************/

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_LPI2C_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_LPI2C_H

/************************************************************************************************************
 * Included Files
 ************************************************************************************************************/

#include <nuttx/config.h>
#include "hardware/s32k1xx_memorymap.h"

/************************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************************/

/* Register offsets *****************************************************************************************/

#define S32K1XX_LPI2C_VERID_OFFSET          0x0000  /* Version ID Register offset */
#define S32K1XX_LPI2C_PARAM_OFFSET          0x0004  /* Parameter Register offset */
#define S32K1XX_LPI2C_MCR_OFFSET            0x0010  /* Master Control Register offset */
#define S32K1XX_LPI2C_MSR_OFFSET            0x0014  /* Master Status Register offset */
#define S32K1XX_LPI2C_MIER_OFFSET           0x0018  /* Master Interrupt Enable Register offset */
#define S32K1XX_LPI2C_MDER_OFFSET           0x001c  /* Master DMA Enable Register offset */
#define S32K1XX_LPI2C_MCFGR0_OFFSET         0x0020  /* Master Config Register 0 offset */
#define S32K1XX_LPI2C_MCFGR1_OFFSET         0x0024  /* Master Config Register 1 offset */
#define S32K1XX_LPI2C_MCFGR2_OFFSET         0x0028  /* Master Config Register 2 offset */
#define S32K1XX_LPI2C_MCFGR3_OFFSET         0x002c  /* Master Config Register 3 offset */
#define S32K1XX_LPI2C_MDMR_OFFSET           0x0040  /* Master Data Match Register offset */
#define S32K1XX_LPI2C_MCCR0_OFFSET          0x0048  /* Master Clock Configuration Register 0 offset */
#define S32K1XX_LPI2C_MCCR1_OFFSET          0x0050  /* Master Clock Configuration Register 1 offset */
#define S32K1XX_LPI2C_MFCR_OFFSET           0x0058  /* Master FIFO Control Register offset */
#define S32K1XX_LPI2C_MFSR_OFFSET           0x005c  /* Master FIFO Status Register offset */
#define S32K1XX_LPI2C_MTDR_OFFSET           0x0060  /* Master Transmit Data Register offset */
#define S32K1XX_LPI2C_MRDR_OFFSET           0x0070  /* Master Receive Data Register offset */
#define S32K1XX_LPI2C_SCR_OFFSET            0x0110  /* Slave Control Register offset */
#define S32K1XX_LPI2C_SSR_OFFSET            0x0114  /* Slave Status Register offset */
#define S32K1XX_LPI2C_SIER_OFFSET           0x0118  /* Slave Interrupt Enable Register offset */
#define S32K1XX_LPI2C_SDER_OFFSET           0x011c  /* Slave DMA Enable Register offset */
#define S32K1XX_LPI2C_SCFGR1_OFFSET         0x0124  /* Slave Config Register 1 offset */
#define S32K1XX_LPI2C_SCFGR2_OFFSET         0x0128  /* Slave Config Register 2 offset */
#define S32K1XX_LPI2C_SAMR_OFFSET           0x0140  /* Slave Address Match Register offset */
#define S32K1XX_LPI2C_SASR_OFFSET           0x0150  /* Slave Address Status Register offset */
#define S32K1XX_LPI2C_STAR_OFFSET           0x0154  /* Slave Transmit ACK Register offset */
#define S32K1XX_LPI2C_STDR_OFFSET           0x0160  /* Slave Transmit Data Register offset */
#define S32K1XX_LPI2C_SRDR_OFFSET           0x0170  /* Slave Receive Data Register offset */

/* Register addresses ***************************************************************************************/

/* LPI2C0 Registers */

#define S32K1XX_LPI2C0_VERID                (S32K1XX_LPI2C0_BASE + S32K1XX_LPI2C_VERID_OFFSET)   /* Version ID Register */
#define S32K1XX_LPI2C0_PARAM                (S32K1XX_LPI2C0_BASE + S32K1XX_LPI2C_PARAM_OFFSET)   /* Parameter Register  */
#define S32K1XX_LPI2C0_MCR                  (S32K1XX_LPI2C0_BASE + S32K1XX_LPI2C_MCR_OFFSET)     /* Master Control Register  */
#define S32K1XX_LPI2C0_MSR                  (S32K1XX_LPI2C0_BASE + S32K1XX_LPI2C_MSR_OFFSET)     /* Master Status Register  */
#define S32K1XX_LPI2C0_MIER                 (S32K1XX_LPI2C0_BASE + S32K1XX_LPI2C_MIER_OFFSET)    /* Master Interrupt Enable Register  */
#define S32K1XX_LPI2C0_MDER                 (S32K1XX_LPI2C0_BASE + S32K1XX_LPI2C_MDER_OFFSET)    /* Master DMA Enable Register  */
#define S32K1XX_LPI2C0_MCFGR0               (S32K1XX_LPI2C0_BASE + S32K1XX_LPI2C_MCFGR0_OFFSET)  /* Master Config Register 0  */
#define S32K1XX_LPI2C0_MCFGR1               (S32K1XX_LPI2C0_BASE + S32K1XX_LPI2C_MCFGR1_OFFSET)  /* Master Config Register 1  */
#define S32K1XX_LPI2C0_MCFGR2               (S32K1XX_LPI2C0_BASE + S32K1XX_LPI2C_MCFGR2_OFFSET)  /* Master Config Register 2  */
#define S32K1XX_LPI2C0_MCFGR3               (S32K1XX_LPI2C0_BASE + S32K1XX_LPI2C_MCFGR3_OFFSET)  /* Master Config Register 3  */
#define S32K1XX_LPI2C0_MDMR                 (S32K1XX_LPI2C0_BASE + S32K1XX_LPI2C_MDMR_OFFSET)    /* Master Data Match Register  */
#define S32K1XX_LPI2C0_MCCR0                (S32K1XX_LPI2C0_BASE + S32K1XX_LPI2C_MCCR0_OFFSET)   /* Master Clock Configuration Register 0  */
#define S32K1XX_LPI2C0_MCCR1                (S32K1XX_LPI2C0_BASE + S32K1XX_LPI2C_MCCR1_OFFSET)   /* Master Clock Configuration Register 1  */
#define S32K1XX_LPI2C0_MFCR                 (S32K1XX_LPI2C0_BASE + S32K1XX_LPI2C_MFCR_OFFSET)    /* Master FIFO Control Register  */
#define S32K1XX_LPI2C0_MFSR                 (S32K1XX_LPI2C0_BASE + S32K1XX_LPI2C_MFSR_OFFSET)    /* Master FIFO Status Register  */
#define S32K1XX_LPI2C0_MTDR                 (S32K1XX_LPI2C0_BASE + S32K1XX_LPI2C_MTDR_OFFSET)    /* Master Transmit Data Register  */
#define S32K1XX_LPI2C0_MRDR                 (S32K1XX_LPI2C0_BASE + S32K1XX_LPI2C_MRDR_OFFSET)    /* Master Receive Data Register  */
#define S32K1XX_LPI2C0_SCR                  (S32K1XX_LPI2C0_BASE + S32K1XX_LPI2C_SCR_OFFSET)     /* Slave Control Register  */
#define S32K1XX_LPI2C0_SSR                  (S32K1XX_LPI2C0_BASE + S32K1XX_LPI2C_SSR_OFFSET)     /* Slave Status Register  */
#define S32K1XX_LPI2C0_SIER                 (S32K1XX_LPI2C0_BASE + S32K1XX_LPI2C_SIER_OFFSET)    /* Slave Interrupt Enable Register  */
#define S32K1XX_LPI2C0_SDER                 (S32K1XX_LPI2C0_BASE + S32K1XX_LPI2C_SDER_OFFSET)    /* Slave DMA Enable Register  */
#define S32K1XX_LPI2C0_SCFGR1               (S32K1XX_LPI2C0_BASE + S32K1XX_LPI2C_SCFGR1_OFFSET)  /* Slave Config Register 1  */
#define S32K1XX_LPI2C0_SCFGR2               (S32K1XX_LPI2C0_BASE + S32K1XX_LPI2C_SCFGR2_OFFSET)  /* Slave Config Register 2  */
#define S32K1XX_LPI2C0_SAMR                 (S32K1XX_LPI2C0_BASE + S32K1XX_LPI2C_SAMR_OFFSET)    /* Slave Address Match Register  */
#define S32K1XX_LPI2C0_SASR                 (S32K1XX_LPI2C0_BASE + S32K1XX_LPI2C_SASR_OFFSET)    /* Slave Address Status Register  */
#define S32K1XX_LPI2C0_STAR                 (S32K1XX_LPI2C0_BASE + S32K1XX_LPI2C_STAR_OFFSET)    /* Slave Transmit ACK Register  */
#define S32K1XX_LPI2C0_STDR                 (S32K1XX_LPI2C0_BASE + S32K1XX_LPI2C_STDR_OFFSET)    /* Slave Transmit Data Register  */
#define S32K1XX_LPI2C0_SRDR                 (S32K1XX_LPI2C0_BASE + S32K1XX_LPI2C_SRDR_OFFSET)    /* Slave Receive Data Register  */

/* LPI2C1 Registers */

#define S32K1XX_LPI2C1_VERID                (S32K1XX_LPI2C1_BASE + S32K1XX_LPI2C_VERID_OFFSET)   /* Version ID Register */
#define S32K1XX_LPI2C1_PARAM                (S32K1XX_LPI2C1_BASE + S32K1XX_LPI2C_PARAM_OFFSET)   /* Parameter Register  */
#define S32K1XX_LPI2C1_MCR                  (S32K1XX_LPI2C1_BASE + S32K1XX_LPI2C_MCR_OFFSET)     /* Master Control Register  */
#define S32K1XX_LPI2C1_MSR                  (S32K1XX_LPI2C1_BASE + S32K1XX_LPI2C_MSR_OFFSET)     /* Master Status Register  */
#define S32K1XX_LPI2C1_MIER                 (S32K1XX_LPI2C1_BASE + S32K1XX_LPI2C_MIER_OFFSET)    /* Master Interrupt Enable Register  */
#define S32K1XX_LPI2C1_MDER                 (S32K1XX_LPI2C1_BASE + S32K1XX_LPI2C_MDER_OFFSET)    /* Master DMA Enable Register  */
#define S32K1XX_LPI2C1_MCFGR0               (S32K1XX_LPI2C1_BASE + S32K1XX_LPI2C_MCFGR0_OFFSET)  /* Master Config Register 0  */
#define S32K1XX_LPI2C1_MCFGR1               (S32K1XX_LPI2C1_BASE + S32K1XX_LPI2C_MCFGR1_OFFSET)  /* Master Config Register 1  */
#define S32K1XX_LPI2C1_MCFGR2               (S32K1XX_LPI2C1_BASE + S32K1XX_LPI2C_MCFGR2_OFFSET)  /* Master Config Register 2  */
#define S32K1XX_LPI2C1_MCFGR3               (S32K1XX_LPI2C1_BASE + S32K1XX_LPI2C_MCFGR3_OFFSET)  /* Master Config Register 3  */
#define S32K1XX_LPI2C1_MDMR                 (S32K1XX_LPI2C1_BASE + S32K1XX_LPI2C_MDMR_OFFSET)    /* Master Data Match Register  */
#define S32K1XX_LPI2C1_MCCR0                (S32K1XX_LPI2C1_BASE + S32K1XX_LPI2C_MCCR0_OFFSET)   /* Master Clock Configuration Register 0  */
#define S32K1XX_LPI2C1_MCCR1                (S32K1XX_LPI2C1_BASE + S32K1XX_LPI2C_MCCR1_OFFSET)   /* Master Clock Configuration Register 1  */
#define S32K1XX_LPI2C1_MFCR                 (S32K1XX_LPI2C1_BASE + S32K1XX_LPI2C_MFCR_OFFSET)    /* Master FIFO Control Register  */
#define S32K1XX_LPI2C1_MFSR                 (S32K1XX_LPI2C1_BASE + S32K1XX_LPI2C_MFSR_OFFSET)    /* Master FIFO Status Register  */
#define S32K1XX_LPI2C1_MTDR                 (S32K1XX_LPI2C1_BASE + S32K1XX_LPI2C_MTDR_OFFSET)    /* Master Transmit Data Register  */
#define S32K1XX_LPI2C1_MRDR                 (S32K1XX_LPI2C1_BASE + S32K1XX_LPI2C_MRDR_OFFSET)    /* Master Receive Data Register  */
#define S32K1XX_LPI2C1_SCR                  (S32K1XX_LPI2C1_BASE + S32K1XX_LPI2C_SCR_OFFSET)     /* Slave Control Register  */
#define S32K1XX_LPI2C1_SSR                  (S32K1XX_LPI2C1_BASE + S32K1XX_LPI2C_SSR_OFFSET)     /* Slave Status Register  */
#define S32K1XX_LPI2C1_SIER                 (S32K1XX_LPI2C1_BASE + S32K1XX_LPI2C_SIER_OFFSET)    /* Slave Interrupt Enable Register  */
#define S32K1XX_LPI2C1_SDER                 (S32K1XX_LPI2C1_BASE + S32K1XX_LPI2C_SDER_OFFSET)    /* Slave DMA Enable Register  */
#define S32K1XX_LPI2C1_SCFGR1               (S32K1XX_LPI2C1_BASE + S32K1XX_LPI2C_SCFGR1_OFFSET)  /* Slave Config Register 1  */
#define S32K1XX_LPI2C1_SCFGR2               (S32K1XX_LPI2C1_BASE + S32K1XX_LPI2C_SCFGR2_OFFSET)  /* Slave Config Register 2  */
#define S32K1XX_LPI2C1_SAMR                 (S32K1XX_LPI2C1_BASE + S32K1XX_LPI2C_SAMR_OFFSET)    /* Slave Address Match Register  */
#define S32K1XX_LPI2C1_SASR                 (S32K1XX_LPI2C1_BASE + S32K1XX_LPI2C_SASR_OFFSET)    /* Slave Address Status Register  */
#define S32K1XX_LPI2C1_STAR                 (S32K1XX_LPI2C1_BASE + S32K1XX_LPI2C_STAR_OFFSET)    /* Slave Transmit ACK Register  */
#define S32K1XX_LPI2C1_STDR                 (S32K1XX_LPI2C1_BASE + S32K1XX_LPI2C_STDR_OFFSET)    /* Slave Transmit Data Register  */
#define S32K1XX_LPI2C1_SRDR                 (S32K1XX_LPI2C1_BASE + S32K1XX_LPI2C_SRDR_OFFSET)    /* Slave Receive Data Register  */

/* Register bit definitions *********************************************************************************/

/* LPI2C Version ID Register */

#define LPI2C_VERID_FEATURE_SHIFT           (0)
#define LPI2C_VERID_FEATURE_MASK            (0xffff << LPI2C_VERID_FEATURE_SHIFT)
#define LPI2C_VERID_MINOR_SHIFT             (16)
#define LPI2C_VERID_MINOR_MASK              (0xff << LPI2C_VERID_MINOR_SHIFT)
#define LPI2C_VERID_MAJOR_SHIFT             (24)
#define LPI2C_VERID_MAJOR_MASK              (0xff << LPI2C_VERID_MAJOR_SHIFT)

/* LPI2C Parameter Register  */

#define LPI2C_PARAM_MTXFIFO_MASK            (0x0f)    /* Bits 0-3:  Master Transmit FIFO Size */
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
#define LPI2C_PARAM_MRXFIFO_SHIFT           (8)       /* Bits 8-11:  Master Receive FIFO Size */
#define LPI2C_PARAM_MRXFIFO_MASK            (0x0f << LPI2C_PARAM_MRXFIFO_SHIFT)
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
#define LPI2C_MSR_ERROR_MASK                (LPI2C_MSR_NDF | LPI2C_MSR_ALF | \
                                             LPI2C_MSR_FEF)

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

#define LPI2C_MCFGR1_PRESCALE_MASK          (7 << 0)  /* Clock Prescaler Bit Mask */
#  define LPI2C_MCFGR1_PRESCALE(n)          (n & LPI2C_MCFGR1_PRESCALE_MASK)
#  define LPI2C_MCFGR1_PRESCALE_1           (0)
#  define LPI2C_MCFGR1_PRESCALE_2           (1)
#  define LPI2C_MCFGR1_PRESCALE_4           (2)
#  define LPI2C_MCFGR1_PRESCALE_8           (3)
#  define LPI2C_MCFGR1_PRESCALE_16          (4)
#  define LPI2C_MCFGR1_PRESCALE_32          (5)
#  define LPI2C_MCFGR1_PRESCALE_64          (6)
#  define LPI2C_MCFGR1_PRESCALE_128         (7)
#define LPI2C_MCFGR1_AUTOSTOP               (1 << 8)  /* Automatic STOP Generation Bit */
#define LPI2C_MCFGR1_IGNACK                 (1 << 9)  /* Ignore NACK Bit */
#define LPI2C_MCFGR1_TIMECFG                (1 << 10) /* Timeout Configuration Bit */
                                                     /* Bits 15-11 Reserved */
#define LPI2C_MCFGR1_MATCFG_SHIFT           (16)
#define LPI2C_MCFGR1_MATCFG_MASK            (7 << LPI2C_MCFGR1_MATCFG_SHIFT)  /* Match Configuration Bit Mask */
#  define LPI2C_MCFGR1_MATCFG(n)            ((n << LPI2C_MCFGR1_MATCFG_SHIFT) & LPI2C_MCFGR1_MATCFG_MASK)
#  define LPI2C_MCFGR1_MATCFG_DISABLE       (0 << LPI2C_MCFGR1_MATCFG_SHIFT)
                                                     /* LPI2C_MCFG1_MATCFG = 001b Reserved */
#  define LPI2C_MCFGR1_MATCFG2              (2 << LPI2C_MCFGR1_MATCFG_SHIFT)
#  define LPI2C_MCFGR1_MATCFG3              (3 << LPI2C_MCFGR1_MATCFG_SHIFT)
#  define LPI2C_MCFGR1_MATCFG4              (4 << LPI2C_MCFGR1_MATCFG_SHIFT)
#  define LPI2C_MCFGR1_MATCFG5              (5 << LPI2C_MCFGR1_MATCFG_SHIFT)
#  define LPI2C_MCFGR1_MATCFG6              (6 << LPI2C_MCFGR1_MATCFG_SHIFT)
#  define LPI2C_MCFGR1_MATCFG7              (7 << LPI2C_MCFGR1_MATCFG_SHIFT)
                                                    /* Bits 23-19 Reserved */
#define LPI2C_MCFGR1_PINCFG_SHIFT           (24)
#define LPI2C_MCFGR1_PINCFG_MASK            (7 << LPI2C_MCFGR1_PINCFG_SHIFT)  /* Pin Configuration Bit Mask */
#  define LPI2C_MCFGR1_PINCFG(n)            ((n << LPI2C_MCFGR1_PINCFG_SHIFT) & LPI2C_MCFGR1_PINCFG_MASK)
#  define LPI2C_MCFGR1_PINCFG0              (0 << LPI2C_MCFGR1_PINCFG_SHIFT)
#  define LPI2C_MCFGR1_PINCFG1              (1 << LPI2C_MCFGR1_PINCFG_SHIFT)
#  define LPI2C_MCFGR1_PINCFG2              (2 << LPI2C_MCFGR1_PINCFG_SHIFT)
#  define LPI2C_MCFGR1_PINCFG3              (3 << LPI2C_MCFGR1_PINCFG_SHIFT)
#  define LPI2C_MCFGR1_PINCFG4              (4 << LPI2C_MCFGR1_PINCFG_SHIFT)
#  define LPI2C_MCFGR1_PINCFG5              (5 << LPI2C_MCFGR1_PINCFG_SHIFT)
#  define LPI2C_MCFGR1_PINCFG6              (6 << LPI2C_MCFGR1_PINCFG_SHIFT)
#  define LPI2C_MCFGR1_PINCFG7              (7 << LPI2C_MCFGR1_PINCFG_SHIFT)
                                                    /* Bits 31-27 Reserved */

/* LPI2C Master Config Register 2  */

#define LPI2C_MCFG2_BUSIDLE_MASK            (0xfff << 0)  /* Bus Idle Timeout Period in Clock Cycles */
#define LPI2C_MCFG2_BUSIDLE_DISABLE         (0)
#  define LPI2C_MCFG2_BUSIDLE(n)            (n & LPI2C_MCFG2_BUSIDLE_MASK)
                                                     /* Bits 15-12 Reserved */
#define LPI2C_MCFG2_FILTSCL_SHIFT           (16)
#define LPI2C_MCFG2_FILTSCL_MASK            (15 << LPI2C_MCFG2_FILTSCL_SHIFT)  /* Glitch Filter SCL */
#define LPI2C_MCFG2_FILTSCL_DISABLE         (0 << LPI2C_MCFG2_FILTSCL_SHIFT)
#  define LPI2C_MCFG2_FILTSCL_CYCLES(n)     ((n << LPI2C_MCFG2_FILTSCL_SHIFT) & LPI2C_MCFG2_FILTSCL_MASK)
                                                     /* Bits 23-20 Reserved */
#define LPI2C_MCFG2_FILTSDA_SHIFT           (24)
#define LPI2C_MCFG2_FILTSDA_MASK            (15 << LPI2C_MCFG2_FILTSDA_SHIFT)  /* Glitch Filter SDA */
#define LPI2C_MCFG2_FILTSDA_DISABLE         (0 << LPI2C_MCFG2_FILTSDA_SHIFT)
#  define LPI2C_MCFG2_FILTSDA_CYCLES(n)     ((n << LPI2C_MCFG2_FILTSDA_SHIFT) & LPI2C_MCFG2_FILTSDA_MASK)
                                                     /* Bits 31-28 Reserved */
/* LPI2C Master Config Register 3  */

                                                     /* Bits 7-0 Reserved */
#define LPI2C_MCFG3_PINLOW_SHIFT            (8)
#define LPI2C_MCFG3_PINLOW_MASK             (0xfff << LPI2C_MCFG3_PINLOW_SHIFT)  /* Configure The Pin Low Timeout in Clock Cycles */
#  define LPI2C_MCFG3_PINLOW_CYCLES(n)      ((n << LPI2C_MCFG3_PINLOW_SHIFT) & LPI2C_MCFG3_PINLOW_MASK)
                                                     /* Bits 31-20 Reserved */

/* LPI2C Master Data Match Register  */

#define LPI2C_MDMR_MATCH0_SHIFT             (0)
#define LPI2C_MDMR_MATCH0_MASK              (0xff << LPI2C_MDMR_MATCH0_SHIFT)  /* Match 0 Value */
#  define LPI2C_MDMR_MATCH0(n)              ((n << LPI2C_MDMR_MATCH0_SHIFT) & LPI2C_MDMR_MATCH0_MASK)
                                                     /* Bits 15-8 Reserved */
#define LPI2C_MDMR_MATCH1_SHIFT             (16)
#define LPI2C_MDMR_MATCH1_MASK              (0xff << LPI2C_MDMR_MATCH1_SHIFT)  /* Match 1 Value */
#  define LPI2C_MDMR_MATCH1(n)              ((n << LPI2C_MDMR_MATCH1_SHIFT) & LPI2C_MDMR_MATCH1_MASK)
                                                     /* Bits 31-24 Reserved */

/* LPI2C Master Clock Configuration Register 0 */

#define LPI2C_MCCR0_CLKLO_SHIFT             (0)
#define LPI2C_MCCR0_CLKLO_MASK              (0x3f << LPI2C_MCCR0_CLKLO_SHIFT)  /* Clock Low Period */
#  define LPI2C_MCCR0_CLKLO(n)              ((n << LPI2C_MCCR0_CLKLO_SHIFT) & LPI2C_MCCR0_CLKLO_MASK)
                                                     /* Bits 7-6 Reserved */
#define LPI2C_MCCR0_CLKHI_SHIFT             (8)
#define LPI2C_MCCR0_CLKHI_MASK              (0x3f << LPI2C_MCCR0_CLKHI_SHIFT)  /* Clock High Period */
#  define LPI2C_MCCR0_CLKHI(n)              ((n << LPI2C_MCCR0_CLKHI_SHIFT) & LPI2C_MCCR0_CLKHI_MASK)
                                                     /* Bits 15-14 Reserved */
#define LPI2C_MCCR0_SETHOLD_SHIFT           (16)
#define LPI2C_MCCR0_SETHOLD_MASK            (0x3f << LPI2C_MCCR0_SETHOLD_SHIFT)  /* Setup Hold Delay */
#  define LPI2C_MCCR0_SETHOLD(n)            ((n << LPI2C_MCCR0_SETHOLD_SHIFT) & LPI2C_MCCR0_SETHOLD_MASK)
                                                     /* Bits 23-22 Reserved */
#define LPI2C_MCCR0_DATAVD_SHIFT            (24)
#define LPI2C_MCCR0_DATAVD_MASK             (0x3f << LPI2C_MCCR0_DATAVD_SHIFT)  /* Setup Hold Delay */
#  define LPI2C_MCCR0_DATAVD(n)             ((n << LPI2C_MCCR0_DATAVD_SHIFT) & LPI2C_MCCR0_DATAVD_MASK)
                                                     /* Bits 31-30 Reserved */

/* LPI2C Master Clock Configuration Register 1 */

#define LPI2C_MCCR1_CLKLO_SHIFT             (0)
#define LPI2C_MCCR1_CLKLO_MASK              (0x3f << LPI2C_MCCR1_CLKLO_SHIFT)  /* Clock Low Period */
#  define LPI2C_MCCR1_CLKLO(n)              ((n << LPI2C_MCCR1_CLKLO_SHIFT) & LPI2C_MCCR1_CLKLO_MASK)
                                                     /* Bits 7-6 Reserved */
#define LPI2C_MCCR1_CLKHI_SHIFT             (8)
#define LPI2C_MCCR1_CLKHI_MASK              (0x3f << LPI2C_MCCR1_CLKHI_SHIFT)  /* Clock High Period */
#  define LPI2C_MCCR1_CLKHI(n)              ((n << LPI2C_MCCR1_CLKHI_SHIFT) & LPI2C_MCCR1_CLKHI_MASK)
                                                     /* Bits 15-14 Reserved */
#define LPI2C_MCCR1_SETHOLD_SHIFT           (16)
#define LPI2C_MCCR1_SETHOLD_MASK            (0x3f << LPI2C_MCCR1_SETHOLD_SHIFT)  /* Setup Hold Delay */
#  define LPI2C_MCCR1_SETHOLD(n)            ((n << LPI2C_MCCR1_SETHOLD_SHIFT) & LPI2C_MCCR1_SETHOLD_MASK)
                                                     /* Bits 23-22 Reserved */
#define LPI2C_MCCR1_DATAVD_SHIFT            (24)
#define LPI2C_MCCR1_DATAVD_MASK             (0x3f << LPI2C_MCCR1_DATAVD_SHIFT)  /* Setup Hold Delay */
#  define LPI2C_MCCR1_DATAVD(n)             ((n << LPI2C_MCCR1_DATAVD_SHIFT) & LPI2C_MCCR1_DATAVD_MASK)
                                                     /* Bits 31-30 Reserved */

/* LPI2C Master FIFO Control Register */

#define LPI2C_MFCR_TXWATER_SHIFT            (0)
#define LPI2C_MFCR_TXWATER_MASK             (3 << LPI2C_MFCR_TXWATER_SHIFT)  /* Transmit FIFO Watermark*/
#  define LPI2C_MFCR_TXWATER(n)             ((n << LPI2C_MFCR_TXWATER_SHIFT) &  LPI2C_MFCR_TXWATER_MASK)  /* Transmit FIFO Watermark*/
                                                     /* Bits 15-2 Reserved */
#define LPI2C_MFCR_RXWATER_SHIFT            (16)
#define LPI2C_MFCR_RXWATER_MASK             (3 << LPI2C_MFCR_RXWATER_SHIFT)  /* Receive FIFO Watermark */
#  define LPI2C_MFCR_RXWATER(n)             ((n << LPI2C_MFCR_RXWATER_SHIFT) &  LPI2C_MFCR_RXWATER_MASK)  /* Transmit FIFO Watermark*/
                                                     /* Bits 31-18 Reserved */

/* LPI2C Master FIFO Status Register */

#define LPI2C_MFSR_TXCOUNT_SHIFT            (0)
#define LPI2C_MFSR_TXCOUNT_MASK             (3 << LPI2C_MFSR_TXCOUNT_SHIFT)  /* Transmit FIFO Count */
                                                     /* Bits 15-2 Reserved */
#define LPI2C_MFSR_RXCOUNT_SHIFT            (16)
#define LPI2C_MFSR_RXCOUNT_MASK             (3 << LPI2C_MFSR_RXCOUNT_SHIFT)  /* Receive FIFO Count */
                                                     /* Bits 31-18 Reserved */

/* LPI2C Master Transmit Data Register */

#define LPI2C_MTDR_DATA_SHIFT               (0)
#define LPI2C_MTDR_DATA_MASK                (0xff << LPI2C_MTDR_DATA_SHIFT)  /* Transmit Data */
#  define LPI2C_MTDR_DATA(n)                (n & LPI2C_MTDR_DATA_MASK)
#define LPI2C_MTDR_CMD_SHIFT                (8)
#define LPI2C_MTDR_CMD_MASK                 (7 << LPI2C_MTDR_CMD_SHIFT)  /* Command Data */
#  define LPI2C_MTDR_CMD(n)                 ((n << LPI2C_MTDR_CMD_SHIFT) & LPI2C_MTDR_CMD_MASK)
#  define LPI2C_MTDR_CMD_TXD                (0 << LPI2C_MTDR_CMD_SHIFT)
#  define LPI2C_MTDR_CMD_RXD                (1 << LPI2C_MTDR_CMD_SHIFT)
#  define LPI2C_MTDR_CMD_STOP               (2 << LPI2C_MTDR_CMD_SHIFT)
#  define LPI2C_MTDR_CMD_RXD_DISC           (3 << LPI2C_MTDR_CMD_SHIFT)
#  define LPI2C_MTDR_CMD_START              (4 << LPI2C_MTDR_CMD_SHIFT)
#  define LPI2C_MTDR_CMD_START_NACK         (5 << LPI2C_MTDR_CMD_SHIFT)
#  define LPI2C_MTDR_CMD_START_HI           (6 << LPI2C_MTDR_CMD_SHIFT)
#  define LPI2C_MTDR_CMD_START_HI_NACK      (7 << LPI2C_MTDR_CMD_SHIFT)
                                                     /* Bits 31-11 Reserved */

/* LPI2C Master Receive Data Register */

#define LPI2C_MRDR_DATA_SHIFT               (0)
#define LPI2C_MRDR_DATA_MASK                (0xff << LPI2C_MRDR_DATA_SHIFT)  /* Receive Data */
                                                     /* Bits 13-8 Reserved */
#define LPI2C_MRDR_RXEMPTY_SHIFT            (14)
#define LPI2C_MRDR_RXEMPTY_MASK             (1 << LPI2C_MRDR_RXEMPTY_SHIFT)  /* Rx Empty */
                                                     /* Bits 31-15 Reserved */

/* LPI2C Slave Control Register */

#define LPI2C_SCR_SEN                       (1 << 0)  /* Slave Enable Bit */
#define LPI2C_SCR_RST                       (1 << 1)  /* Software Reset Bit */
                                                      /* Bits 3-2 Reserved */
#define LPI2C_SCR_FILTEN                    (1 << 4)  /* Filter Enable Bit */
#define LPI2C_SCR_FILTDZ                    (1 << 5)  /* Filter Doze Enable Bit */
                                                      /* Bits 7-4 Reserved */
#define LPI2C_SCR_RTF                       (1 << 8)  /* Reset Transmit FIFO Bit */
#define LPI2C_SCR_RRF                       (1 << 9)  /* Reset Receive FIFO Bit */
                                                      /* Bits 31-10 Reserved */

/* LPI2C Slave Status Register  */

#define LPI2C_SSR_TDF                       (1 << 0)  /* Transmit Data Flag Bit */
#define LPI2C_SSR_RDF                       (1 << 1)  /* Receive Data Flag Bit */
#define LPI2C_SSR_AVF                       (1 << 2)  /* Address Valid Flag Bit */
#define LPI2C_SSR_TAF                       (1 << 3)  /* Transmit ACK Flag Bit */
                                                      /* Bits 7-4 Reserved */
#define LPI2C_SSR_RSF                       (1 << 8)  /* Repeated Start Flag Bit */
#define LPI2C_SSR_SDF                       (1 << 9)  /* STOP Detect Flag Bit */
#define LPI2C_SSR_BEF                       (1 << 10) /* Bit Error Flag Bit */
#define LPI2C_SSR_FEF                       (1 << 11) /* FIFO Error Flag Bit */
#define LPI2C_SSR_AM0F                      (1 << 12) /* Address Match 0 Flag Bit */
#define LPI2C_SSR_AM1F                      (1 << 13) /* Address Match 1 Flag Bit */
#define LPI2C_SSR_GCF                       (1 << 14) /* General Call Flag Bit */
#define LPI2C_SSR_SARF                      (1 << 15) /* SMBus Alert Response Flag Bit */
                                                      /* Bits 23-16 Reserved */
#define LPI2C_MSR_SBF                       (1 << 24) /* Slave Busy Flag Bit */
#define LPI2C_MSR_BBF                       (1 << 25) /* Bus Busy Flag Bit */
                                                      /* Bits 31-26 Reserved */

/* LPI2C Slave Interrupt Enable Register  */

#define LPI2C_SIER_TDIE                     (1 << 0)  /* Transmit Data Interrupt Enable Bit */
#define LPI2C_SIER_RDIE                     (1 << 1)  /* Receive Data Interrupt Enable Bit */
#define LPI2C_SIER_AVIE                     (1 << 2)  /* Address Valid Interrupt Enable Bit */
#define LPI2C_SIER_TAIE                     (1 << 3)  /* Transmit ACK Interrupt Enable Bit */
                                                      /* Bits 7-4 Reserved */
#define LPI2C_SIER_RSIE                     (1 << 8)  /* Repeated Start Interrupt Enable Bit */
#define LPI2C_SIER_SDIE                     (1 << 9)  /* STOP Detect Interrupt Enable Bit */
#define LPI2C_SIER_BEIE                     (1 << 10) /* Bit Error Interrupt Enable Bit */
#define LPI2C_SIER_FEIE                     (1 << 11) /* FIFO Error Interrupt Enable Bit */
#define LPI2C_SIER_AM0IE                    (1 << 12) /* Address Match 0 Interrupt Enable Bit */
#define LPI2C_SIER_AM1IE                    (1 << 13) /* Address Match 1 Interrupt Enable Bit */
#define LPI2C_SIER_GCIE                     (1 << 14) /* General Call Interrupt Enable Bit */
#define LPI2C_SIER_SARIE                    (1 << 15) /* SMBus Alert Response Interrupt Enable Bit */
                                                      /* Bits 31-16 Reserved */

/* LPI2C Slave DMA Enable Register  */

#define LPI2C_SDER_TDDE                     (1 << 0)  /* Transmit Data DMA Enable Bit */
#define LPI2C_SDER_RDDE                     (1 << 1)  /* Transmit Data DMA Enable Bit */
#define LPI2C_SDER_AVDE                     (1 << 2)  /* Address Valid DMA Enable Bit */
                                                      /* Bits 31-3 Reserved */

/* LPI2C Slave Configuration Register 1  */

#define LPI2C_SCFGR1_ADRSTALL               (1 << 0)  /* Address SCL Stall */
#define LPI2C_SCFGR1_RXSTALL                (1 << 1)  /* RX SCL Stall */
#define LPI2C_SCFGR1_TXSTALL                (1 << 2)  /* TX Data SCL Stall */
#define LPI2C_SCFGR1_ACKSTALL               (1 << 3)  /* ACK SCL Stall */
                                                      /* Bits 7-4 Reserved */
#define LPI2C_SCFGR1_GCEN                   (1 << 8)  /* General Call Enable */
#define LPI2C_SCFGR1_SAEN                   (1 << 9)  /* SMBus Alert Enable */
#define LPI2C_SCFGR1_TXCFG                  (1 << 10) /* Transmit Flag Configuration */
#define LPI2C_SCFGR1_RXCFG                  (1 << 11) /* Receive Data Configuration */
#define LPI2C_SCFGR1_IFNACK                 (1 << 12) /* Ignore NACK */
#define LPI2C_SCFGR1_HSMEN                  (1 << 13) /* High Speed Mode Enable */
                                                      /* Bits 15-14 Reserved */
#define LPI2C_SCFG1_ADDRCFG_SHIFT           (16)
#define LPI2C_SCFG1_ADDRCFG_MASK            (7 << LPI2C_SCFG1_ADDRCFG_SHIFT)  /* Address Configuration Bit Mask */
#  define LPI2C_SCFG1_ADDRCFG(n)            ((n << LPI2C_SCFG1_ADDRCFG_SHIFT) & LPI2C_SCFG1_ADDRCFG_MASK)
#  define LPI2C_SCFG1_ADDRCFG0              (0 << LPI2C_SCFG1_ADDRCFG_SHIFT)
#  define LPI2C_SCFG1_ADDRCFG1              (2 << LPI2C_SCFG1_ADDRCFG_SHIFT)
#  define LPI2C_SCFG1_ADDRCFG2              (2 << LPI2C_SCFG1_ADDRCFG_SHIFT)
#  define LPI2C_SCFG1_ADDRCFG3              (3 << LPI2C_SCFG1_ADDRCFG_SHIFT)
#  define LPI2C_SCFG1_ADDRCFG4              (4 << LPI2C_SCFG1_ADDRCFG_SHIFT)
#  define LPI2C_SCFG1_ADDRCFG5              (5 << LPI2C_SCFG1_ADDRCFG_SHIFT)
#  define LPI2C_SCFG1_ADDRCFG6              (6 << LPI2C_SCFG1_ADDRCFG_SHIFT)
#  define LPI2C_SCFG1_ADDRCFG7              (7 << LPI2C_SCFG1_ADDRCFG_SHIFT)
                                                      /* Bits 31-19 Reserved */

/* LPI2C Slave Configuration Register 2  */

#define LPI2C_SCFG2_CLKHOLD_MASK            (15 << 0) /* Clock Hold Time */
#  define LPI2C_SCFG2_CLKHOLD(n)            (n & LPI2C_SCFG2_CLKHOLD_MASK)
                                                      /* Bits 7-4 Reserved */
#define LPI2C_SCFG2_DATAVD_SHIFT            (8)
#define LPI2C_SCFG2_DATAVD_MASK             (0x3f << LPI2C_SCFG2_DATAVD_SHIFT)  /* Data Valid Delay */
#  define LPI2C_SCFG2_DATAVD(n)             ((n << LPI2C_SCFG2_DATAVD_SHIFT) & LPI2C_SCFG2_DATAVD_MASK)
                                                      /* Bits 15-14 Reserved */
#define LPI2C_SCFG2_FILTSCL_SHIFT           (16)
#define LPI2C_SCFG2_FILTSCL_MASK            (15 << LPI2C_SCFG2_FILTSCL_SHIFT)  /* Glitch Filter SCL */
#define LPI2C_SCFG2_FILTSCL_DISABLE         (0 << LPI2C_SCFG2_FILTSCL_SHIFT)
#  define LPI2C_SCFG2_FILTSCL_CYCLES(n)     ((n << LPI2C_SCFG2_FILTSCL_SHIFT) & LPI2C_SCFG2_FILTSCL_MASK)
                                                      /* Bits 23-20 Reserved */
#define LPI2C_SCFG2_FILTSDA_SHIFT           (24)
#define LPI2C_SCFG2_FILTSDA_MASK            (15 << LPI2C_SCFG2_FILTSDA_SHIFT)  /* Glitch Filter SDA */
#define LPI2C_SCFG2_FILTSDA_DISABLE         (0 << LPI2C_SCFG2_FILTSDA_SHIFT)
#  define LPI2C_SCFG2_FILTSDA_CYCLES(n)     ((n << LPI2C_SCFG2_FILTSDA_SHIFT) & LPI2C_SCFG2_FILTSDA_MASK)
                                                      /* Bits 31-28 Reserved */

/* LPI2C Slave Address Match Register  */

                                                      /* Bit 0 Reserved */
#define LPI2C_SAMR_ADDR0_SHIFT              (1)
#define LPI2C_SAMR_ADDR0_MASK               (0x3ff << LPI2C_SAMR_ADDR0_SHIFT)  /* Address 0 Value */
#  define LPI2C_SAMR_ADDR0(n)               ((n << LPI2C_SAMR_ADDR0_SHIFT) & LPI2C_SAMR_ADDR0_MASK)
                                                      /* Bits 16-11 Reserved */
#define LPI2C_SAMR_ADDR1_SHIFT              (17)
#define LPI2C_SAMR_ADDR1_MASK               (0x3ff << LPI2C_SAMR_ADDR1_SHIFT)  /* Address 1 Value */
#  define LPI2C_SAMR_ADDR1(n)               ((n << LPI2C_SAMR_ADDR1_SHIFT) & LPI2C_SAMR_ADDR1_MASK)
                                                      /* Bits 31-27 Reserved */

/* LPI2C Slave Address Status Register  */

#define LPI2C_SASR_RADDR_MASK               (0x7ff << 0) /* Received Address */
                                                      /* Bits 16-11 Reserved */
#define LPI2C_SASR_ANV                      (1 << 14) /* Address Not Valid */
                                                      /* Bits 31-15 Reserved */

/* LPI2C Slave Transmit ACK Register  */

#define LPI2C_STAR_TXNACK                   (1 << 0)  /* Transmit NACK */
                                                      /* Bits 31-1 Reserved */

/* LPI2C Slave Transmit Data Register  */

#define LPI2C_STDR_DATA_SHIFT               (0)
#define LPI2C_STDR_DATA_MASK                (0xff << LPI2C_STDR_DATA_SHIFT)  /* Transmit Data */
#  define LPI2C_STDR_DATA(n)                ((n << LPI2C_STDR_DATA_SHIFT) & LPI2C_STDR_DATA_MASK)
                                                      /* Bits 31-8 Reserved */

/* LPI2C Slave Receive Data Register  */

#define LPI2C_SRDR_DATA_SHIFT               (0)
#define LPI2C_SRDR_DATA_MASK                (0xff << LPI2C_SRDR_DATA_SHIFT)  /* Receive Data */
#  define LPI2C_SRDR_DATA(n)                ((n << LPI2C_SRDR_DATA_SHIFT) & LPI2C_SRDR_DATA_MASK)
                                                      /* Bits 8-31 Reserved */

#endif /* __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_LPI2C_H */
