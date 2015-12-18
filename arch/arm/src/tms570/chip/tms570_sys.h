/****************************************************************************************************
 * arch/arm/src/tms570/chip/tms570_sys.h
 * Primary System Control Register Definitions
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

#ifndef __ARCH_ARM_SRC_TMS570_CHIP_TMS570_SYS_H
#define __ARCH_ARM_SRC_TMS570_CHIP_TMS570_SYS_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include "chip/tms570_memorymap.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Register Offsets *********************************************************************************/

#define TMS570_SYS_PC1_OFFSET           0x0000  /* SYS Pin Control Register 1 */
#define TMS570_SYS_PC2_OFFSET           0x0004  /* SYS Pin Control Register 2 */
#define TMS570_SYS_PC3_OFFSET           0x0008  /* SYS Pin Control Register 3 */
#define TMS570_SYS_PC4_OFFSET           0x000c  /* SYS Pin Control Register 4 */
#define TMS570_SYS_PC5_OFFSET           0x0010  /* SYS Pin Control Register 5 */
#define TMS570_SYS_PC6_OFFSET           0x0014  /* SYS Pin Control Register 6 */
#define TMS570_SYS_PC7_OFFSET           0x0018  /* SYS Pin Control Register 7 */
#define TMS570_SYS_PC8_OFFSET           0x001c  /* SYS Pin Control Register 8 */
#define TMS570_SYS_PC9_OFFSET           0x0020  /* SYS Pin Control Register 9 */
#define TMS570_SYS_CSDIS_OFFSET         0x0030  /* Clock Source Disable Register */
#define TMS570_SYS_CSDISSET_OFFSET      0x0034  /* Clock Source Disable Set Register */
#define TMS570_SYS_CSDISCLR_OFFSET      0x0038  /* Clock Source Disable Clear Register */
#define TMS570_SYS_CDDIS_OFFSET         0x003c  /* Clock Domain Disable Register */
#define TMS570_SYS_CDDISSET_OFFSET      0x0040  /* Clock Domain Disable Set Register */
#define TMS570_SYS_CDDISCLR_OFFSET      0x0044  /* Clock Domain Disable Clear Register */
#define TMS570_SYS_GHVSRC_OFFSET        0x0048  /* GCLK, HCLK, VCLK, and VCLK2 Source Register */
#define TMS570_SYS_VCLKASRC_OFFSET      0x004c  /* Peripheral Asynchronous Clock Source Register */
#define TMS570_SYS_RCLKSRC_OFFSET       0x0050  /* RTI Clock Source Register */
#define TMS570_SYS_CSVSTAT_OFFSET       0x0054  /* Clock Source Valid Status Register */
#define TMS570_SYS_MSTGCR_OFFSET        0x0058  /* Memory Self-Test Global Control Register */
#define TMS570_SYS_MINITGCR_OFFSET      0x005c  /* Memory Hardware Initialization Global Control Register */
#define TMS570_SYS_MSIENA_OFFSET        0x0060  /* Memory Self-Test/Initialization Enable Register */
#define TMS570_SYS_MSTFAIL_OFFSET       0x0064  /* Memory Self-Test Fail Status Register */
#define TMS570_SYS_MSTCGSTAT_OFFSET     0x0068  /* MSTC Global Status Register */
#define TMS570_SYS_MINISTAT_OFFSET      0x006c  /* Memory Hardware Initialization Status Register */
#define TMS570_SYS_PLLCTL1_OFFSET       0x0070  /* PLL Control Register 1 */
#define TMS570_SYS_PLLCTL2_OFFSET       0x0074  /* PLL Control Register 2 */
#define TMS570_SYS_PC10_OFFSET          0x0078  /* SYS Pin Control Register 10 */
#define TMS570_SYS_DIEIDL_OFFSET        0x007c  /* Die Identification Register, Lower Word */
#define TMS570_SYS_DIEIDH_OFFSET        0x0080  /* Die Identification Register, Upper Word */
#define TMS570_SYS_LPOMONCTL_OFFSET     0x0088  /* LPO/Clock Monitor Control Register */
#define TMS570_SYS_CLKTEST_OFFSET       0x008c  /* Clock Test Register */
#define TMS570_SYS_DFTCTRLREG_OFFSET    0x0090  /* DFT Control Register */
#define TMS570_SYS_DFTCTRLREG2_OFFSET   0x0094  /* DFT Control Register 2 */
#define TMS570_SYS_GPREG1_OFFSET        0x00a0  /* General Purpose Register */
#define TMS570_SYS_IMPFASTS_OFFSET      0x00a8  /* Imprecise Fault Status Register */
#define TMS570_SYS_IMPFTADD_OFFSET      0x00ac  /* Imprecise Fault Write Address Register */
#define TMS570_SYS_SSIR1_OFFSET         0x00b0  /* System Software Interrupt Request 1 Register */
#define TMS570_SYS_SSIR2_OFFSET         0x00b4  /* System Software Interrupt Request 2 Register */
#define TMS570_SYS_SSIR3_OFFSET         0x00b8  /* System Software Interrupt Request 3 Register */
#define TMS570_SYS_SSIR4_OFFSET         0x00bc  /* System Software Interrupt Request 4 Register */
#define TMS570_SYS_RAMGCR_OFFSET        0x00c0  /* RAM Control Register */
#define TMS570_SYS_BMMCR1_OFFSET        0x00c4  /* Bus Matrix Module Control Register 1 */
#define TMS570_SYS_CPURSTCR_OFFSET      0x00cc  /* CPU Reset Control Register */
#define TMS570_SYS_CLKCNTL_OFFSET       0x00d0  /* Clock Control Register */
#define TMS570_SYS_ECPCNTL_OFFSET       0x00d4  /* ECP Control Register */
#define TMS570_SYS_DEVCR1_OFFSET        0x00dc  /* DEV Parity Control Register 1 */
#define TMS570_SYS_ECR_OFFSET           0x00e0  /* System Exception Control Register */
#define TMS570_SYS_ESR_OFFSET           0x00e4  /* System Exception Status Register */
#define TMS570_SYS_TASR_OFFSET          0x00e8  /* System Test Abort Status Register */
#define TMS570_SYS_GLBSTAT_OFFSET       0x00ec  /* Global Status Register */
#define TMS570_SYS_DEVID_OFFSET         0x00f0  /* Device Identification Register */
#define TMS570_SYS_SSIVEC_OFFSET        0x00f4  /* Software Interrupt Vector Register */
#define TMS570_SYS_SSIF_OFFSET          0x00f8  /* System Software Interrupt Flag Register */

/* Register Addresses *******************************************************************************/

#define TMS570_SYS_PC1                  (TMS570_SYS_BASE+TMS570_SYS_PC1_OFFSET)
#define TMS570_SYS_PC2                  (TMS570_SYS_BASE+TMS570_SYS_PC2_OFFSET)
#define TMS570_SYS_PC3                  (TMS570_SYS_BASE+TMS570_SYS_PC3_OFFSET)
#define TMS570_SYS_PC4                  (TMS570_SYS_BASE+TMS570_SYS_PC4_OFFSET)
#define TMS570_SYS_PC5                  (TMS570_SYS_BASE+TMS570_SYS_PC5_OFFSET)
#define TMS570_SYS_PC6                  (TMS570_SYS_BASE+TMS570_SYS_PC6_OFFSET)
#define TMS570_SYS_PC7                  (TMS570_SYS_BASE+TMS570_SYS_PC7_OFFSET)
#define TMS570_SYS_PC8                  (TMS570_SYS_BASE+TMS570_SYS_PC8_OFFSET)
#define TMS570_SYS_PC9                  (TMS570_SYS_BASE+TMS570_SYS_PC9_OFFSET)
#define TMS570_SYS_CSDIS                (TMS570_SYS_BASE+TMS570_SYS_CSDIS_OFFSET)
#define TMS570_SYS_CSDISSET             (TMS570_SYS_BASE+TMS570_SYS_CSDISSET_OFFSET)
#define TMS570_SYS_CSDISCLR             (TMS570_SYS_BASE+TMS570_SYS_CSDISCLR_OFFSET)
#define TMS570_SYS_CDDIS                (TMS570_SYS_BASE+TMS570_SYS_CDDIS_OFFSET)
#define TMS570_SYS_CDDISSET             (TMS570_SYS_BASE+TMS570_SYS_CDDISSET_OFFSET)
#define TMS570_SYS_CDDISCLR             (TMS570_SYS_BASE+TMS570_SYS_CDDISCLR_OFFSET)
#define TMS570_SYS_GHVSRC               (TMS570_SYS_BASE+TMS570_SYS_GHVSRC_OFFSET)
#define TMS570_SYS_VCLKASRC             (TMS570_SYS_BASE+TMS570_SYS_VCLKASRC_OFFSET)
#define TMS570_SYS_RCLKSRC              (TMS570_SYS_BASE+TMS570_SYS_RCLKSRC_OFFSET)
#define TMS570_SYS_CSVSTAT              (TMS570_SYS_BASE+TMS570_SYS_CSVSTAT_OFFSET)
#define TMS570_SYS_MSTGCR               (TMS570_SYS_BASE+TMS570_SYS_MSTGCR_OFFSET)
#define TMS570_SYS_MINITGCR             (TMS570_SYS_BASE+TMS570_SYS_MINITGCR_OFFSET)
#define TMS570_SYS_MSIENA               (TMS570_SYS_BASE+TMS570_SYS_MSIENA_OFFSET)
#define TMS570_SYS_MSTFAIL              (TMS570_SYS_BASE+TMS570_SYS_MSTFAIL_OFFSET)
#define TMS570_SYS_MSTCGSTAT            (TMS570_SYS_BASE+TMS570_SYS_MSTCGSTAT_OFFSET)
#define TMS570_SYS_MINISTAT             (TMS570_SYS_BASE+TMS570_SYS_MINISTAT_OFFSET)
#define TMS570_SYS_PLLCTL1              (TMS570_SYS_BASE+TMS570_SYS_PLLCTL1_OFFSET)
#define TMS570_SYS_PLLCTL2              (TMS570_SYS_BASE+TMS570_SYS_PLLCTL2_OFFSET)
#define TMS570_SYS_PC10                 (TMS570_SYS_BASE+TMS570_SYS_PC10_OFFSET)
#define TMS570_SYS_DIEIDL               (TMS570_SYS_BASE+TMS570_SYS_DIEIDL_OFFSET)
#define TMS570_SYS_DIEIDH               (TMS570_SYS_BASE+TMS570_SYS_DIEIDH_OFFSET)
#define TMS570_SYS_LPOMONCTL            (TMS570_SYS_BASE+TMS570_SYS_LPOMONCTL_OFFSET)
#define TMS570_SYS_CLKTEST              (TMS570_SYS_BASE+TMS570_SYS_CLKTEST_OFFSET)
#define TMS570_SYS_DFTCTRLREG           (TMS570_SYS_BASE+TMS570_SYS_DFTCTRLREG_OFFSET)
#define TMS570_SYS_DFTCTRLREG2          (TMS570_SYS_BASE+TMS570_SYS_DFTCTRLREG2_OFFSET)
#define TMS570_SYS_GPREG1               (TMS570_SYS_BASE+TMS570_SYS_GPREG1_OFFSET)
#define TMS570_SYS_IMPFASTS             (TMS570_SYS_BASE+TMS570_SYS_IMPFASTS_OFFSET)
#define TMS570_SYS_IMPFTADD             (TMS570_SYS_BASE+TMS570_SYS_IMPFTADD_OFFSET)
#define TMS570_SYS_SSIR1                (TMS570_SYS_BASE+TMS570_SYS_SSIR1_OFFSET)
#define TMS570_SYS_SSIR2                (TMS570_SYS_BASE+TMS570_SYS_SSIR2_OFFSET)
#define TMS570_SYS_SSIR3                (TMS570_SYS_BASE+TMS570_SYS_SSIR3_OFFSET)
#define TMS570_SYS_SSIR4                (TMS570_SYS_BASE+TMS570_SYS_SSIR4_OFFSET)
#define TMS570_SYS_RAMGCR               (TMS570_SYS_BASE+TMS570_SYS_RAMGCR_OFFSET)
#define TMS570_SYS_BMMCR1               (TMS570_SYS_BASE+TMS570_SYS_BMMCR1_OFFSET)
#define TMS570_SYS_CPURSTCR             (TMS570_SYS_BASE+TMS570_SYS_CPURSTCR_OFFSET)
#define TMS570_SYS_CLKCNTL              (TMS570_SYS_BASE+TMS570_SYS_CLKCNTL_OFFSET)
#define TMS570_SYS_ECPCNTL              (TMS570_SYS_BASE+TMS570_SYS_ECPCNTL_OFFSET)
#define TMS570_SYS_DEVCR1               (TMS570_SYS_BASE+TMS570_SYS_DEVCR1_OFFSET)
#define TMS570_SYS_ECR                  (TMS570_SYS_BASE+TMS570_SYS_ECR_OFFSET)
#define TMS570_SYS_ESR                  (TMS570_SYS_BASE+TMS570_SYS_ESR_OFFSET)
#define TMS570_SYS_TASR                 (TMS570_SYS_BASE+TMS570_SYS_TASR_OFFSET)
#define TMS570_SYS_GLBSTAT              (TMS570_SYS_BASE+TMS570_SYS_GLBSTAT_OFFSET)
#define TMS570_SYS_DEVID                (TMS570_SYS_BASE+TMS570_SYS_DEVID_OFFSET)
#define TMS570_SYS_SSIVEC               (TMS570_SYS_BASE+TMS570_SYS_SSIVEC_OFFSET)
#define TMS570_SYS_SSIF                 (TMS570_SYS_BASE+TMS570_SYS_SSIF_OFFSET)

/* Register Bit-Field Definitions *******************************************************************/

/* SYS Pin Control Register 1 */
#define SYS_PC1_
/* SYS Pin Control Register 2 */
#define SYS_PC2_
/* SYS Pin Control Register 3 */
#define SYS_PC3_
/* SYS Pin Control Register 4 */
#define SYS_PC4_
/* SYS Pin Control Register 5 */
#define SYS_PC5_
/* SYS Pin Control Register 6 */
#define SYS_PC6_
/* SYS Pin Control Register 7 */
#define SYS_PC7_
/* SYS Pin Control Register 8 */
#define SYS_PC8_
/* SYS Pin Control Register 9 */
#define SYS_PC9_
/* Clock Source Disable Register */
#define SYS_CSDIS_
/* Clock Source Disable Set Register */
#define SYS_CSDISSET_
/* Clock Source Disable Clear Register */
#define SYS_CSDISCLR_
/* Clock Domain Disable Register */
#define SYS_CDDIS_
/* Clock Domain Disable Set Register */
#define SYS_CDDISSET_
/* Clock Domain Disable Clear Register */
#define SYS_CDDISCLR_
/* GCLK, HCLK, VCLK, and VCLK2 Source Register */
#define SYS_GHVSRC_
/* Peripheral Asynchronous Clock Source Register */
#define SYS_VCLKASRC_
/* RTI Clock Source Register */
#define SYS_RCLKSRC_
/* Clock Source Valid Status Register */
#define SYS_CSVSTAT_
/* Memory Self-Test Global Control Register */
#define SYS_MSTGCR_

/* Memory Hardware Initialization Global Control Register */

#define SYS_MINITGCR_MASK               (0xff)    /* Bits 0-7: Memory hardware initialization key */
#  define SYS_MINITGCR_ENABLE           (0x0a)    /*   Enable */
#  define SYS_MINITGCR_DISABLE          (0x05)    /*   Any other value disables */

/* Memory Self-Test/Initialization Enable Register */

#define SYS_MSIENA_

#if defined(CONFIG_ARCH_CHIP_TMS570LS0332PZ) || defined(CONFIG_ARCH_CHIP_TMS570LS0432PZ)
  /* From TMS570LS0x32 Data Sheet */

#  define SYS_MSIENA_RAM                (1 << 0)
#  define SYS_MSIENA_VIM_RAM            (1 << 2)
#  define SYS_MSIENA_N2HET_RAM          (1 << 3)
#  define SYS_MSIENA_HTU_RAM            (1 << 4)
#  define SYS_MSIENA_DCAN1_RAM          (1 << 5)
#  define SYS_MSIENA_DCAN2_RAM          (1 << 6)
#  define SYS_MSIENA_MIBSPI1_RAM        (1 << 7)
#  define SYS_MSIENA_MIBADC_RAM         (1 << 8)
#endif

/* Memory Self-Test Fail Status Register */
#define SYS_MSTFAIL_

/* MSTC Global Status Register */

#define SYS_MSTCGSTAT_MSTDONE           (1 << 0)  /* Bit 0: Memory self-test done */
#define SYS_MSTCGSTAT_MINIDONE          (1 << 8)  /* Bit 8: Hardware initialization of all memory done */

/* Memory Hardware Initialization Status Register */
#define SYS_MINISTAT_
/* PLL Control Register 1 */
#define SYS_PLLCTL1_
/* PLL Control Register 2 */
#define SYS_PLLCTL2_
/* SYS Pin Control Register 10 */
#define SYS_PC10_
/* Die Identification Register, Lower Word */
#define SYS_DIEIDL_
/* Die Identification Register, Upper Word */
#define SYS_DIEIDH_
/* LPO/Clock Monitor Control Register */
#define SYS_LPOMONCTL_
/* Clock Test Register */
#define SYS_CLKTEST_
/* DFT Control Register */
#define SYS_DFTCTRLREG_
/* DFT Control Register 2 */
#define SYS_DFTCTRLREG2_
/* General Purpose Register */
#define SYS_GPREG1_
/* Imprecise Fault Status Register */
#define SYS_IMPFASTS_
/* Imprecise Fault Write Address Register */
#define SYS_IMPFTADD_
/* System Software Interrupt Request 1 Register */
#define SYS_SSIR1_
/* System Software Interrupt Request 2 Register */
#define SYS_SSIR2_
/* System Software Interrupt Request 3 Register */
#define SYS_SSIR3_
/* System Software Interrupt Request 4 Register */
#define SYS_SSIR4_
/* RAM Control Register */
#define SYS_RAMGCR_
/* Bus Matrix Module Control Register 1 */
#define SYS_BMMCR1_
/* CPU Reset Control Register */
#define SYS_CPURSTCR_
/* Clock Control Register */
#define SYS_CLKCNTL_
/* ECP Control Register */
#define SYS_ECPCNTL_
/* DEV Parity Control Register 1 */
#define SYS_DEVCR1_
/* System Exception Control Register */
#define SYS_ECR_
/* System Exception Status Register */

#define SYS_ESR_MPMODE  (1 << 0)  /* Bit 0:  Current memory protection unit (MPU) mode */
#define SYS_ESR_EXTRST  (1 << 3)  /* Bit 3:  External reset flag */
#define SYS_ESR_SWRST   (1 << 4)  /* Bit 4:  Software reset flag */
#define SYS_ESR_CPURST  (1 << 5)  /* Bit 5:  CPU reset flag */
#define SYS_ESR_WDRST   (1 << 13) /* Bit 13: Watchdog reset flag */
#define SYS_ESR_OSCRST  (1 << 14) /* Bit 14: Reset caused by an oscillator failure or PLL cycle slip */
#define SYS_ESR_PORST   (1 << 15) /* Bit 15: Power-up reset */

#define SYS_ESR_RSTALL  (0x0000e038)

/* System Test Abort Status Register */
#define SYS_TASR_
/* Global Status Register */
#define SYS_GLBSTAT_
/* Device Identification Register */
#define SYS_DEVID_
/* Software Interrupt Vector Register */
#define SYS_SSIVEC_
/* System Software Interrupt Flag Register */
#define SYS_SSIF_

#endif /* __ARCH_ARM_SRC_TMS570_CHIP_TMS570_SYS_H */
