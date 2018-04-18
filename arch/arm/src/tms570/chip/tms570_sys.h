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

/* The LPO trim value may be programmed into the TI OTP: */

#define TMS570_TITCM_LPOTRIM_OFFSET     0x01b4
#define TMS570_TITCM_LPOTRIM            (TMS570_TITCM_BASE+TMS570_TITCM_LPOTRIM_OFFSET)
#  define TMS570_TITCM_LPOTRIM_SHIFT    (16)    /* Bits 16-31: LPO trim value */
#  define TMS570_TITCM_LPOTRIM_MASK     (0xffff << TMS570_TITCM_LPOTRIM_SHIFT)

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

#define SYS_GLBSTAT_OSC_ERR_MASK        (0x01)
#define SYS_GLBSTAT_OSC_ERR_CLR         (0x0301)

/* SYS Pin Control Register 1 */

#define SYS_PC1_ECPCLKFUN               (1 << 0)  /* Bit 0: ECLK function */

/* SYS Pin Control Register 2 */

#define SYS_PC2_ECPCLKDIR               (1 << 0)  /* Bit 0: ECLK data direction */

/* SYS Pin Control Register 3 */

#define SYS_PC3_ECPCLKDIN               (1 << 0)  /* Bit 0: ECLK data in */

/* SYS Pin Control Register 4 */

#define SYS_PC4_ECPCLKDOUT              (1 << 0)  /* Bit 0: ECLK data out write */

/* SYS Pin Control Register 5 */

#define SYS_PC5_ECPCLKSET               (1 << 0)  /* Bit 0: ECLK data out set */

/* SYS Pin Control Register 6 */

#define SYS_PC6_ECPCLKCLR               (1 << 0)  /* Bit 0: ECLK data out clear */

/* SYS Pin Control Register 7 */

#define SYS_PC7_ECPCLKODE               (1 << 0)  /* Bit 0: ECLK open drain enable */

/* SYS Pin Control Register 8 */

#define SYS_PC8_ECPCLKPUE               (1 << 0)  /* Bit 0: ECLK pull enable */

/* SYS Pin Control Register 9 */

#define SYS_PC9_ECPCLKPS                (1 << 0)  /* Bit 0: ECLK pull up/pull down select */

/* Clock Source Disable Register, Clock Source Disable Set Register, and Clock Source
 * Disable Clear Register
 */

#define SYS_CSDIS_CLKSR0OFF             (1 << 0)  /* Bit 0: Clock source 0 */
#define SYS_CSDIS_CLKSR1OFF             (1 << 1)  /* Bit 1: Clock source 1 */
#define SYS_CSDIS_CLKSR3OFF             (1 << 3)  /* Bit 3: Clock source 3 */
#define SYS_CSDIS_CLKSR4OFF             (1 << 4)  /* Bit 4: Clock source 4 */
#define SYS_CSDIS_CLKSR5OFF             (1 << 5)  /* Bit 5: Clock source 5 */
#define SYS_CSDIS_CLKSR6OFF             (1 << 6)  /* Bit 4: Clock source 4 */
#define SYS_CSDIS_CLKSR7OFF             (1 << 7)  /* Bit 5: Clock source 5 */
#define SYS_CSDIS_CLKSROFFALL           (0xff)

#define SYS_CSDIS_CLKSRC_OSC            SYS_CSDIS_CLKSR0OFF /* Oscillator */
#define SYS_CSDIS_CLKSRC_PLL1           SYS_CSDIS_CLKSR1OFF /* PLL1 */
#define SYS_CSDIS_CLKSRC_EXTCLKIN1      SYS_CSDIS_CLKSR3OFF /* EXTCLKIN */
#define SYS_CSDIS_CLKSRC_LFLPO          SYS_CSDIS_CLKSR4OFF /* Low Frequency LPO (Low Power Oscillator) clock */
#define SYS_CSDIS_CLKSRC_HFLPO          SYS_CSDIS_CLKSR5OFF /* High Frequency LPO (Low Power Oscillator) clock */
#define SYS_CSDIS_CLKSRC_PLL2           SYS_CSDIS_CLKSR6OFF /* PLL2 */
#define SYS_CSDIS_CLKSRC_EXTCLKIN2      SYS_CSDIS_CLKSR7OFF /* EXTCLKIN2 */

/* Clock Domain Disable Register, Clock Domain Disable Set Register, and Clock Domain
 * Disable Clear Register.
 */

#define SYS_CDDIS_GCLKOFF               (1 << 0)  /* Bit 0:  GCLK domain off */
#define SYS_CDDIS_HCLKOFF               (1 << 1)  /* Bit 1:  HCLK and VCLK_sys domains off */
#define SYS_CDDIS_VCLKPOFF              (1 << 2)  /* Bit 2:  VCLK_periph domain off */
#define SYS_CDDIS_VCLK2OFF              (1 << 3)  /* Bit 3:  VCLK2 domain off */
#define SYS_CDDIS_VCLKA1OFF             (1 << 4)  /* Bit 4:  VCLKA1 domain off */
#define SYS_CDDIS_RTICLK1OFF            (1 << 6)  /* Bit 6:  RTICLK1 domain off */
#define SYS_CDDIS_VCLKEQEPOFF           (1 << 9)  /* Bit 9:  VCLK_EQEP_OFF domain off */

/* GCLK, HCLK, VCLK, and VCLK2 Source Register */

#define SYS_CLKSRC_OSC                  0         /* Alias for oscillator clock Source                */
#define SYS_CLKSRC_PLL1                 1         /* Alias for Pll1 clock Source                      */
#define SYS_CLKSRC_EXTERNAL1            3         /* Alias for external clock Source                  */
#define SYS_CLKSRC_LPOLOW               4         /* Alias for low power oscillator low clock Source  */
#define SYS_CLKSRC_LPOHIGH              5         /* Alias for low power oscillator high clock Source */
#define SYS_CLKSRC_PLL2                 6         /* Alias for Pll2 clock Source                      */
#define SYS_CLKSRC_EXTERNAL2            7         /* Alias for external 2 clock Source                */
#define SYS_CLKSRC_VCLK                 9         /* Alias for synchronous VCLK1 clock Source         */

#define SYS_GHVSRC_GHVSRC_SHIFT         (0)       /* Bits 0-3: GCLK, HCLK, VCLK, VCLK2 current source */
#define SYS_GHVSRC_GHVSRC_MASK          (15 << SYS_GHVSRC_GHVSRC_SHIFT)
#  define SYS_GHVSRC_GHVSRC_SRC(n)      ((uint32_t)(n) << SYS_GHVSRC_GHVSRC_SHIFT)
#  define SYS_GHVSRC_GHVSRC_SRC0        (0 << SYS_GHVSRC_GHVSRC_SHIFT) /* Clock source0 for GCLK, HCLK, VCLK, VCLK2 */
#  define SYS_GHVSRC_GHVSRC_SRC1        (1 << SYS_GHVSRC_GHVSRC_SHIFT) /* Clock source1 for GCLK, HCLK, VCLK, VCLK2 */
#  define SYS_GHVSRC_GHVSRC_SRC2        (2 << SYS_GHVSRC_GHVSRC_SHIFT) /* Clock source2 for GCLK, HCLK, VCLK, VCLK2 */
#  define SYS_GHVSRC_GHVSRC_SRC3        (3 << SYS_GHVSRC_GHVSRC_SHIFT) /* Clock source3 for GCLK, HCLK, VCLK, VCLK2 */
#  define SYS_GHVSRC_GHVSRC_SRC4        (4 << SYS_GHVSRC_GHVSRC_SHIFT) /* Clock source4 for GCLK, HCLK, VCLK, VCLK2 */
#  define SYS_GHVSRC_GHVSRC_SRC5        (5 << SYS_GHVSRC_GHVSRC_SHIFT) /* Clock source5 for GCLK, HCLK, VCLK, VCLK2 */
#  define SYS_GHVSRC_GHVSRC_SRC6        (6 << SYS_GHVSRC_GHVSRC_SHIFT) /* Clock source6 for GCLK, HCLK, VCLK, VCLK2 */
#  define SYS_GHVSRC_GHVSRC_SRC7        (7 << SYS_GHVSRC_GHVSRC_SHIFT) /* Clock source7 for GCLK, HCLK, VCLK, VCLK2 */

#  define SYS_GHVSRC_GHVSRC_SRC(n)      ((uint32_t)(n) << SYS_GHVSRC_GHVSRC_SHIFT)
#  define SYS_GHVSRC_GHVSRC_OSC         SYS_GHVSRC_GHVSRC_SRC(SYS_CLKSRC_OSC)
#  define SYS_GHVSRC_GHVSRC_PLL1        SYS_GHVSRC_GHVSRC_SRC(SYS_CLKSRC_PLL1)
#  define SYS_GHVSRC_GHVSRC_EXTERNAL1   SYS_GHVSRC_GHVSRC_SRC(SYS_CLKSRC_EXTERNAL1)
#  define SYS_GHVSRC_GHVSRC_LPOLOW      SYS_GHVSRC_GHVSRC_SRC(SYS_CLKSRC_LPOLOW)
#  define SYS_GHVSRC_GHVSRC_LPOHIGH     SYS_GHVSRC_GHVSRC_SRC(SYS_CLKSRC_LPOHIGH)
#  define SYS_GHVSRC_GHVSRC_PLL2        SYS_GHVSRC_GHVSRC_SRC(SYS_CLKSRC_PLL2)
#  define SYS_GHVSRC_GHVSRC_EXTERNAL2   SYS_GHVSRC_GHVSRC_SRC(SYS_CLKSRC_EXTERNAL2)
#  define SYS_GHVSRC_GHVSRC_VCLK        SYS_GHVSRC_GHVSRC_SRC(SYS_CLKSRC_VCLK)

#define SYS_GHVSRC_HVLPM_SHIFT          (16)      /* Bits 16-19: HCLK, VCLK, VCLK2 source on wakeup when GCLK is turned off */
#define SYS_GHVSRC_HVLPM_MASK           (15 << SYS_GHVSRC_HVLPM_SHIFT)
#  define SYS_GHVSRC_HVLPM_SRC0         (0 << SYS_GHVSRC_HVLPM_SHIFT) /* Clock source0 for HCLK, VCLK, VCLK2 on wakeup */
#  define SYS_GHVSRC_HVLPM_SRC1         (1 << SYS_GHVSRC_HVLPM_SHIFT) /* Clock source1 for HCLK, VCLK, VCLK2 on wakeup */
#  define SYS_GHVSRC_HVLPM_SRC2         (2 << SYS_GHVSRC_HVLPM_SHIFT) /* Clock source2 for HCLK, VCLK, VCLK2 on wakeup */
#  define SYS_GHVSRC_HVLPM_SRC3         (3 << SYS_GHVSRC_HVLPM_SHIFT) /* Clock source3 for HCLK, VCLK, VCLK2 on wakeup */
#  define SYS_GHVSRC_HVLPM_SRC4         (4 << SYS_GHVSRC_HVLPM_SHIFT) /* Clock source4 for HCLK, VCLK, VCLK2 on wakeup */
#  define SYS_GHVSRC_HVLPM_SRC5         (5 << SYS_GHVSRC_HVLPM_SHIFT) /* Clock source5 for HCLK, VCLK, VCLK2 on wakeup */
#  define SYS_GHVSRC_HVLPM_SRC6         (6 << SYS_GHVSRC_HVLPM_SHIFT) /* Clock source6 for HCLK, VCLK, VCLK2 on wakeup */
#  define SYS_GHVSRC_HVLPM_SRC7         (7 << SYS_GHVSRC_HVLPM_SHIFT) /* Clock source7 for HCLK, VCLK, VCLK2 on wakeup */

#  define SYS_GHVSRC_HVLPM(n)           ((uint32_t)(n) << SYS_GHVSRC_HVLPM_SHIFT)
#  define SYS_GHVSRC_HVLPM_OSC          SYS_GHVSRC_HVLPM(SYS_CLKSRC_OSC)
#  define SYS_GHVSRC_HVLPM_PLL1         SYS_GHVSRC_HVLPM(SYS_CLKSRC_PLL1)
#  define SYS_GHVSRC_HVLPM_EXTERNAL1    SYS_GHVSRC_HVLPM(SYS_CLKSRC_EXTERNAL1)
#  define SYS_GHVSRC_HVLPM_LPOLOW       SYS_GHVSRC_HVLPM(SYS_CLKSRC_LPOLOW)
#  define SYS_GHVSRC_HVLPM_LPOHIGH      SYS_GHVSRC_HVLPM(SYS_CLKSRC_LPOHIGH)
#  define SYS_GHVSRC_HVLPM_PLL2         SYS_GHVSRC_HVLPM(SYS_CLKSRC_PLL2)
#  define SYS_GHVSRC_HVLPM_EXTERNAL2    SYS_GHVSRC_HVLPM(SYS_CLKSRC_EXTERNAL2)
#  define SYS_GHVSRC_HVLPM_VCLK         SYS_GHVSRC_HVLPM(SYS_CLKSRC_VCLK)

#define SYS_GHVSRC_GHVWAKE_SHIFT        (24)      /* Bits 24-17: GCLK, HCLK, VCLK, VCLK2 source on wakeup */
#define SYS_GHVSRC_GHVWAKE_MASK         (15 << SYS_GHVSRC_GHVWAKE_SHIFT)
#  define SYS_GHVSRC_GHVWAKE_SRC0       (0 << SYS_GHVSRC_GHVWAKE_SHIFT) /* Clock source0 for GCLK, HCLK, VCLK, VCLK2 on wakeup */
#  define SYS_GHVSRC_GHVWAKE_SRC1       (1 << SYS_GHVSRC_GHVWAKE_SHIFT) /* Clock source1 for GCLK, HCLK, VCLK, VCLK2 on wakeup */
#  define SYS_GHVSRC_GHVWAKE_SRC2       (2 << SYS_GHVSRC_GHVWAKE_SHIFT) /* Clock source2 for GCLK, HCLK, VCLK, VCLK2 on wakeup */
#  define SYS_GHVSRC_GHVWAKE_SRC3       (3 << SYS_GHVSRC_GHVWAKE_SHIFT) /* Clock source3 for GCLK, HCLK, VCLK, VCLK2 on wakeup */
#  define SYS_GHVSRC_GHVWAKE_SRC4       (4 << SYS_GHVSRC_GHVWAKE_SHIFT) /* Clock source4 for GCLK, HCLK, VCLK, VCLK2 on wakeup */
#  define SYS_GHVSRC_GHVWAKE_SRC5       (5 << SYS_GHVSRC_GHVWAKE_SHIFT) /* Clock source5 for GCLK, HCLK, VCLK, VCLK2 on wakeup */
#  define SYS_GHVSRC_GHVWAKE_SRC6       (6 << SYS_GHVSRC_GHVWAKE_SHIFT) /* Clock source6 for GCLK, HCLK, VCLK, VCLK2 on wakeup */
#  define SYS_GHVSRC_GHVWAKE_SRC7       (7 << SYS_GHVSRC_GHVWAKE_SHIFT) /* Clock source7 for GCLK, HCLK, VCLK, VCLK2 on wakeup */

#  define SYS_GHVSRC_GHVWAKE(n)         ((uint32_t)(n) << SYS_GHVSRC_GHVWAKE_SHIFT)
#  define SYS_GHVSRC_GHVWAKE_OSC        SYS_GHVSRC_GHVWAKE(SYS_CLKSRC_OSC)
#  define SYS_GHVSRC_GHVWAKE_PLL1       SYS_GHVSRC_GHVWAKE(SYS_CLKSRC_PLL1)
#  define SYS_GHVSRC_GHVWAKE_EXTERNAL1  SYS_GHVSRC_GHVWAKE(SYS_CLKSRC_EXTERNAL1)
#  define SYS_GHVSRC_GHVWAKE_LPOLOW     SYS_GHVSRC_GHVWAKE(SYS_CLKSRC_LPOLOW)
#  define SYS_GHVSRC_GHVWAKE_LPOHIGH    SYS_GHVSRC_GHVWAKE(SYS_CLKSRC_LPOHIGH)
#  define SYS_GHVSRC_GHVWAKE_PLL2       SYS_GHVSRC_GHVWAKE(SYS_CLKSRC_PLL2)
#  define SYS_GHVSRC_GHVWAKE_EXTERNAL2  SYS_GHVSRC_GHVWAKE(SYS_CLKSRC_EXTERNAL2)
#  define SYS_GHVSRC_GHVWAKE_VCLK       SYS_GHVSRC_GHVWAKE(SYS_CLKSRC_VCLK)

/* Peripheral Asynchronous Clock Source Register */

#define SYS_VCLKASRC_VCLKA1S_SHIFT      (0)      /* Bits 0-3: Peripheral asynchronous clock1 source */
#define SYS_VCLKASRC_VCLKA1S_MASK       (15 << SYS_VCLKASRC_VCLKA1S_SHIFT)
#  define SYS_VCLKASRC_VCLKA1S_SRC0     (0 << SYS_VCLKASRC_VCLKA1S_SHIFT) /* Clock source0 for RTICLK1 */
#  define SYS_VCLKASRC_VCLKA1S_SRC1     (1 << SYS_VCLKASRC_VCLKA1S_SHIFT) /* Clock source1 for RTICLK1 */
#  define SYS_VCLKASRC_VCLKA1S_SRC2     (2 << SYS_VCLKASRC_VCLKA1S_SHIFT) /* Clock source2 for RTICLK1 */
#  define SYS_VCLKASRC_VCLKA1S_SRC3     (3 << SYS_VCLKASRC_VCLKA1S_SHIFT) /* Clock source3 for RTICLK1 */
#  define SYS_VCLKASRC_VCLKA1S_SRC4     (4 << SYS_VCLKASRC_VCLKA1S_SHIFT) /* Clock source4 for RTICLK1 */
#  define SYS_VCLKASRC_VCLKA1S_SRC5     (5 << SYS_VCLKASRC_VCLKA1S_SHIFT) /* Clock source5 for RTICLK1 */
#  define SYS_VCLKASRC_VCLKA1S_SRC6     (6 << SYS_VCLKASRC_VCLKA1S_SHIFT) /* Clock source6 for RTICLK1 */
#  define SYS_VCLKASRC_VCLKA1S_SRC7     (7 << SYS_VCLKASRC_VCLKA1S_SHIFT) /* Clock source7 for RTICLK1 */

#  define SYS_VCLKASRC_VCLKA1S(n)         ((uint32_t)(n) << SYS_VCLKASRC_VCLKA1S_SHIFT)
#  define SYS_VCLKASRC_VCLKA1S_OSC        SYS_VCLKASRC_VCLKA1S(SYS_CLKSRC_OSC)
#  define SYS_VCLKASRC_VCLKA1S_PLL1       SYS_VCLKASRC_VCLKA1S(SYS_CLKSRC_PLL1)
#  define SYS_VCLKASRC_VCLKA1S_EXTERNAL1  SYS_VCLKASRC_VCLKA1S(SYS_CLKSRC_EXTERNAL1)
#  define SYS_VCLKASRC_VCLKA1S_LPOLOW     SYS_VCLKASRC_VCLKA1S(SYS_CLKSRC_LPOLOW)
#  define SYS_VCLKASRC_VCLKA1S_LPOHIGH    SYS_VCLKASRC_VCLKA1S(SYS_CLKSRC_LPOHIGH)
#  define SYS_VCLKASRC_VCLKA1S_PLL2       SYS_VCLKASRC_VCLKA1S(SYS_CLKSRC_PLL2)
#  define SYS_VCLKASRC_VCLKA1S_EXTERNAL2  SYS_VCLKASRC_VCLKA1S(SYS_CLKSRC_EXTERNAL2)
#  define SYS_VCLKASRC_VCLKA1S_VCLK       SYS_VCLKASRC_VCLKA1S(SYS_CLKSRC_VCLK)

#if defined(CONFIG_ARCH_CHIP_TMS570LS3137ZWT)
#define SYS_VCLKASRC_VCLKA2S_SHIFT      (8)      /* Bits 0-3: Peripheral asynchronous clock2 source */
#define SYS_VCLKASRC_VCLKA2S_MASK       (15 << SYS_VCLKASRC_VCLKA2S_SHIFT)
#  define SYS_VCLKASRC_VCLKA2S_SRC0     (0 << SYS_VCLKASRC_VCLKA2S_SHIFT) /* Clock source0 for VCLKA2S */
#  define SYS_VCLKASRC_VCLKA2S_SRC1     (1 << SYS_VCLKASRC_VCLKA2S_SHIFT) /* Clock source1 for VCLKA2S */
#  define SYS_VCLKASRC_VCLKA2S_SRC2     (2 << SYS_VCLKASRC_VCLKA2S_SHIFT) /* Clock source2 for VCLKA2S */
#  define SYS_VCLKASRC_VCLKA2S_SRC3     (3 << SYS_VCLKASRC_VCLKA2S_SHIFT) /* Clock source3 for VCLKA2S */
#  define SYS_VCLKASRC_VCLKA2S_SRC4     (4 << SYS_VCLKASRC_VCLKA2S_SHIFT) /* Clock source4 for VCLKA2S */
#  define SYS_VCLKASRC_VCLKA2S_SRC5     (5 << SYS_VCLKASRC_VCLKA2S_SHIFT) /* Clock source5 for VCLKA2S */
#  define SYS_VCLKASRC_VCLKA2S_SRC6     (6 << SYS_VCLKASRC_VCLKA2S_SHIFT) /* Clock source6 for VCLKA2S */
#  define SYS_VCLKASRC_VCLKA2S_SRC7     (7 << SYS_VCLKASRC_VCLKA2S_SHIFT) /* Clock source7 for VCLKA2S */

#  define SYS_VCLKASRC_VCLKA2S(n)         ((uint32_t)(n) << SYS_VCLKASRC_VCLKA2S_SHIFT)
#  define SYS_VCLKASRC_VCLKA2S_OSC        SYS_VCLKASRC_VCLKA2S(SYS_CLKSRC_OSC)
#  define SYS_VCLKASRC_VCLKA2S_PLL1       SYS_VCLKASRC_VCLKA2S(SYS_CLKSRC_PLL1)
#  define SYS_VCLKASRC_VCLKA2S_EXTERNAL1  SYS_VCLKASRC_VCLKA2S(SYS_CLKSRC_EXTERNAL1)
#  define SYS_VCLKASRC_VCLKA2S_LPOLOW     SYS_VCLKASRC_VCLKA2S(SYS_CLKSRC_LPOLOW)
#  define SYS_VCLKASRC_VCLKA2S_LPOHIGH    SYS_VCLKASRC_VCLKA2S(SYS_CLKSRC_LPOHIGH)
#  define SYS_VCLKASRC_VCLKA2S_PLL2       SYS_VCLKASRC_VCLKA2S(SYS_CLKSRC_PLL2)
#  define SYS_VCLKASRC_VCLKA2S_EXTERNAL2  SYS_VCLKASRC_VCLKA2S(SYS_CLKSRC_EXTERNAL2)
#  define SYS_VCLKASRC_VCLKA2S_VCLK       SYS_VCLKASRC_VCLKA2S(SYS_CLKSRC_VCLK)
#endif

/* RTI Clock Source Register */

#define SYS_RCLKSRC_RTI1SRC_SHIFT       (0)      /* Bits 0-3: RTI clock1 source */
#define SYS_RCLKSRC_RTI1SRC_MASK        (15 << SYS_RCLKSRC_RTI1SRC_SHIFT)
#  define SYS_RCLKSRC_RTI1SRC_SRC0      (0 << SYS_RCLKSRC_RTI1SRC_SHIFT) /* Clock source0 for RTICLK1 */
#  define SYS_RCLKSRC_RTI1SRC_SRC1      (1 << SYS_RCLKSRC_RTI1SRC_SHIFT) /* Clock source1 for RTICLK1 */
#  define SYS_RCLKSRC_RTI1SRC_SRC2      (2 << SYS_RCLKSRC_RTI1SRC_SHIFT) /* Clock source2 for RTICLK1 */
#  define SYS_RCLKSRC_RTI1SRC_SRC3      (3 << SYS_RCLKSRC_RTI1SRC_SHIFT) /* Clock source3 for RTICLK1 */
#  define SYS_RCLKSRC_RTI1SRC_SRC4      (4 << SYS_RCLKSRC_RTI1SRC_SHIFT) /* Clock source4 for RTICLK1 */
#  define SYS_RCLKSRC_RTI1SRC_SRC5      (5 << SYS_RCLKSRC_RTI1SRC_SHIFT) /* Clock source5 for RTICLK1 */
#  define SYS_RCLKSRC_RTI1SRC_SRC6      (6 << SYS_RCLKSRC_RTI1SRC_SHIFT) /* Clock source6 for RTICLK1 */
#  define SYS_RCLKSRC_RTI1SRC_SRC7      (7 << SYS_RCLKSRC_RTI1SRC_SHIFT) /* Clock source7 for RTICLK1 */

#  define SYS_RCLKSRC_RTI1SRC(n)        ((uint32_t)(n) << SYS_RCLKSRC_RTI1SRC_SHIFT)
#  define SYS_RCLKSRC_RTI1SRC_OSC       SYS_RCLKSRC_RTI1SRC(SYS_CLKSRC_OSC)
#  define SYS_RCLKSRC_RTI1SRC_PLL1      SYS_RCLKSRC_RTI1SRC(SYS_CLKSRC_PLL1)
#  define SYS_RCLKSRC_RTI1SRC_EXTERNAL1 SYS_RCLKSRC_RTI1SRC(SYS_CLKSRC_EXTERNAL1)
#  define SYS_RCLKSRC_RTI1SRC_LPOLOW    SYS_RCLKSRC_RTI1SRC(SYS_CLKSRC_LPOLOW)
#  define SYS_RCLKSRC_RTI1SRC_LPOHIGH   SYS_RCLKSRC_RTI1SRC(SYS_CLKSRC_LPOHIGH)
#  define SYS_RCLKSRC_RTI1SRC_PLL2      SYS_RCLKSRC_RTI1SRC(SYS_CLKSRC_PLL2)
#  define SYS_RCLKSRC_RTI1SRC_EXTERNAL2 SYS_RCLKSRC_RTI1SRC(SYS_CLKSRC_EXTERNAL2)
#  define SYS_RCLKSRC_RTI1SRC_VCLK      SYS_RCLKSRC_RTI1SRC(SYS_CLKSRC_VCLK)

#define SYS_RCLKSRC_RTI1DIV_SHIFT       (8)       /* Bits 8-9: RTI clock 1 divider */
#define SYS_RCLKSRC_RTI1DIV_MASK        (3 << SYS_RCLKSRC_RTI1DIV_SHIFT)
#  define SYS_RCLKSRC_RTI1DIV_DIV1      (0 << SYS_RCLKSRC_RTI1DIV_SHIFT) /* RTICLK1 divider value is 1 */
#  define SYS_RCLKSRC_RTI1DIV_DIV2      (1 << SYS_RCLKSRC_RTI1DIV_SHIFT) /* RTICLK1 divider value is 2 */
#  define SYS_RCLKSRC_RTI1DIV_DIV4      (2 << SYS_RCLKSRC_RTI1DIV_SHIFT) /* RTICLK1 divider value is 4 */
#  define SYS_RCLKSRC_RTI1DIV_DIV8      (3 << SYS_RCLKSRC_RTI1DIV_SHIFT) /* RTICLK1 divider value is 8 */

/* Clock Source Valid Status Register */

#define SYS_CSVSTAT_CLKSR0V             (1 << 0)  /* Bit 0:  Clock source xx valid */
#define SYS_CSVSTAT_CLKSR1V             (1 << 1)  /* Bit 1:  Clock source xx valid */
#define SYS_CSVSTAT_CLKSR3V             (1 << 3)  /* Bit 3:  Clock source xx valid */
#define SYS_CSVSTAT_CLKSR4V             (1 << 4)  /* Bit 4:  Clock source xx valid */
#define SYS_CSVSTAT_CLKSR5V             (1 << 5)  /* Bit 5:  Clock source xx valid */
#define SYS_CSVSTAT_CLKSRVALL           (0x3b)

/* Memory Self-Test Global Control Register */

#define SYS_MSTGCR_MSTGENA_SHIFT        (0)      /* Bits 0-3:  Memory self-test controller global enable key */
#define SYS_MSTGCR_MSTGENA_MASK         (15 << SYS_MSTGCR_MSTGENA_SHIFT)
#  define SYS_MSTGCR_MSTGENA_ENABLE     (10 << SYS_MSTGCR_MSTGENA_SHIFT)
#  define SYS_MSTGCR_MSTGENA_DISABLE    (5 << SYS_MSTGCR_MSTGENA_SHIFT)
#define SYS_MSTGCR_ROMDIV_SHIFT         (8)      /* Bits 8-9:  Prescaler divider bits for ROM clock source */
#define SYS_MSTGCR_ROMDIV_MASK          (3 << SYS_MSTGCR_ROMDIV_SHIFT)
#  define SYS_MSTGCR_ROMDIV_DIV1        (0 << SYS_MSTGCR_ROMDIV_SHIFT) /* ROM clock=HCL/1; PBIST reset=16 VBUS cycles */
#  define SYS_MSTGCR_ROMDIV_DIV2        (1 << SYS_MSTGCR_ROMDIV_SHIFT) /* ROM clock=HCLK/2; PBIST reset=32 VBUS cycles */
#  define SYS_MSTGCR_ROMDIV_DIV4        (2 << SYS_MSTGCR_ROMDIV_SHIFT) /* ROM clock=HCLK/4. PBIST reset=64 VBUS cycles */
#  define SYS_MSTGCR_ROMDIV_DIV8        (3 << SYS_MSTGCR_ROMDIV_SHIFT) /* ROM clock=HCLK/8. PBIST reset=96 VBUS cycles */

/* Memory Hardware Initialization Global Control Register */

#define SYS_MINITGCR_MASK               (0xff)    /* Bits 0-7: Memory hardware initialization key */
#  define SYS_MINITGCR_ENABLE           (0x0a)    /*   Enable */
#  define SYS_MINITGCR_DISABLE          (0x05)    /*   Any other value disables */

/* Memory Self-Test/Initialization Enable Register */

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

#elif defined(CONFIG_ARCH_CHIP_TMS570LS3137ZWT)
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

#define SYS_PLLCTL1_PLLMUL_SHIFT        (0)       /* Bits 0-15: PLL Multiplication Factor */
#define SYS_PLLCTL1_PLLMUL_MASK         (0xffff << SYS_PLLCTL1_PLLMUL_SHIFT)
#  define SYS_PLLCTL1_PLLMUL(n)         ((uint32_t)(n) << SYS_PLLCTL1_PLLMUL_SHIFT)
#define SYS_PLLCTL1_REFCLKDIV_SHIFT     (16)      /* Bits 16-21: Reference Clock Divider */
#define SYS_PLLCTL1_REFCLKDIV_MASK      (0x3f << SYS_PLLCTL1_REFCLKDIV_SHIFT)
#  define SYS_PLLCTL1_REFCLKDIV(n)      ((uint32_t)(n) << SYS_PLLCTL1_REFCLKDIV_SHIFT)
#define SYS_PLLCTL1_ROF                 (1 << 23) /* Bit 23:  Reset on Oscillator Fail */
#define SYS_PLLCTL1_PLLDIV_SHIFT        (24)      /* Bits 24-28: PLL Output Clock Divider */
#define SYS_PLLCTL1_PLLDIV_MASK         (0x1f << SYS_PLLCTL1_PLLDIV_SHIFT)
#  define SYS_PLLCTL1_PLLDIV(n)         ((uint32_t)(n) << SYS_PLLCTL1_PLLDIV_SHIFT)
#  define SYS_PLLCTL1_PLLDIV_MAX        SYS_PLLCTL1_PLLDIV_MASK
#define SYS_PLLCTL1_MASKSLIP_SHIFT      (29)      /* Bits 29-30: Mask detection of PLL slip */
#define SYS_PLLCTL1_MASKSLIP_MASK       (3 << SYS_PLLCTL1_MASKSLIP_SHIFT)
#  define SYS_PLLCTL1_MASKSLIP_DISABLE  (0 << SYS_PLLCTL1_MASKSLIP_SHIFT) /* All values but 2 disable */
#  define SYS_PLLCTL1_MASKSLIP_ENABLE   (1 << SYS_PLLCTL1_MASKSLIP_SHIFT)
#define SYS_PLLCTL1_ROS                 (1 << 31)  /* Bit 31:  Reset on PLL Slip */

/* PLL Control Register 2 */

#define SYS_PLLCTL2_SPRAMOUNT_SHIFT     (0)        /* Bits 0-8:  Spreading Amount */
#define SYS_PLLCTL2_SPRAMOUNT_MASK      (0xff << SYS_PLLCTL2_SPRAMOUNT_SHIFT)
#  define SYS_PLLCTL2_SPRAMOUNT(n)      ((uint32_t)(n) << SYS_PLLCTL2_SPRAMOUNT_SHIFT)
#define SYS_PLLCTL2_ODPLL_SHIFT         (9)        /* Bits 9-11:  Internal PLL Output Divider */
#define SYS_PLLCTL2_ODPLL_MASK          (7 << SYS_PLLCTL2_ODPLL_SHIFT)
#  define SYS_PLLCTL2_ODPLL(n)          ((uint32_t)(n) << SYS_PLLCTL2_ODPLL_SHIFT)
#define SYS_PLLCTL2_MULMOD_SHIFT        (12)       /* Bits 12-20:  Multiplier Correction when Frequency Modulation is enabled */
#define SYS_PLLCTL2_MULMOD_MASK         (0x1ff << SYS_PLLCTL2_MULMOD_SHIFT)
#  define SYS_PLLCTL2_MULMOD(n)         ((uint32_t)(n) << SYS_PLLCTL2_MULMOD_SHIFT)
#define SYS_PLLCTL2_SPRRATE_SHIFT       (22)       /* Bits 22-30:  NS = SPREADINGRATE + 1 */
#define SYS_PLLCTL2_SPRRATE_MASK        (0x1ff << SYS_PLLCTL2_SPRRATE_SHIFT)
#  define SYS_PLLCTL2_SPRRATE(n)        ((uint32_t)(n) << SYS_PLLCTL2_SPRRATE_SHIFT)
#define SYS_PLLCTL2_FMENA               (1 << 31) /* Bit 31:  Frequency Modulation Enable */

/* SYS Pin Control Register 10 */

#define SYS_PC10_ECLCSLEW               (1 << 0)  /* Bit 0: ECLK slew control */

/* Die Identification Register, Lower Word */
#define SYS_DIEIDL_
/* Die Identification Register, Upper Word */
#define SYS_DIEIDH_
/* LPO/Clock Monitor Control Register */

#define SYS_LPOMONCTL_LFTRIM_SHIFT      (0)       /* Bits 0-4: Low frequency oscillator trim value */
#define SYS_LPOMONCTL_LFTRIM_MASK       (31 << SYS_LPOMONCTL_LFTRIM_SHIFT)
#  define SYS_LPOMONCTL_20p67           (0 << SYS_LPOMONCTL_LFTRIM_SHIFT) /* 20.67% */
#  define SYS_LPOMONCTL_25p76           (1 << SYS_LPOMONCTL_LFTRIM_SHIFT) /* 25.76% */
#  define SYS_LPOMONCTL_30p84           (2 << SYS_LPOMONCTL_LFTRIM_SHIFT) /* 30.84% */
#  define SYS_LPOMONCTL_35p90           (3 << SYS_LPOMONCTL_LFTRIM_SHIFT) /* 35.90% */
#  define SYS_LPOMONCTL_40p93           (4 << SYS_LPOMONCTL_LFTRIM_SHIFT) /* 40.93% */
#  define SYS_LPOMONCTL_45p95           (5 << SYS_LPOMONCTL_LFTRIM_SHIFT) /* 45.95% */
#  define SYS_LPOMONCTL_50p97           (6 << SYS_LPOMONCTL_LFTRIM_SHIFT) /* 50.97% */
#  define SYS_LPOMONCTL_55p91           (7 << SYS_LPOMONCTL_LFTRIM_SHIFT) /* 55.91% */
#  define SYS_LPOMONCTL_60p86           (8 << SYS_LPOMONCTL_LFTRIM_SHIFT) /* 60.86% */
#  define SYS_LPOMONCTL_65p78           (9 << SYS_LPOMONCTL_LFTRIM_SHIFT) /* 65.78% */
#  define SYS_LPOMONCTL_70p75           (10 << SYS_LPOMONCTL_LFTRIM_SHIFT) /* 70.75% */
#  define SYS_LPOMONCTL_75p63           (11 << SYS_LPOMONCTL_LFTRIM_SHIFT) /* 75.63% */
#  define SYS_LPOMONCTL_80p61           (12 << SYS_LPOMONCTL_LFTRIM_SHIFT) /* 80.61% */
#  define SYS_LPOMONCTL_85p39           (13 << SYS_LPOMONCTL_LFTRIM_SHIFT) /* 85.39% */
#  define SYS_LPOMONCTL_90p23           (14 << SYS_LPOMONCTL_LFTRIM_SHIFT) /* 90.23% */
#  define SYS_LPOMONCTL_95p11           (15 << SYS_LPOMONCTL_LFTRIM_SHIFT) /* 95.11% */
#  define SYS_LPOMONCTL_100p00          (16 << SYS_LPOMONCTL_LFTRIM_SHIFT) /* 100.00% */
#  define SYS_LPOMONCTL_104p84          (17 << SYS_LPOMONCTL_LFTRIM_SHIFT) /* 104.84% */
#  define SYS_LPOMONCTL_109p51          (18 << SYS_LPOMONCTL_LFTRIM_SHIFT) /* 109.51% */
#  define SYS_LPOMONCTL_114p31          (19 << SYS_LPOMONCTL_LFTRIM_SHIFT) /* 114.31% */
#  define SYS_LPOMONCTL_119p01          (20 << SYS_LPOMONCTL_LFTRIM_SHIFT) /* 119.01% */
#  define SYS_LPOMONCTL_123p75          (21 << SYS_LPOMONCTL_LFTRIM_SHIFT) /* 123.75% */
#  define SYS_LPOMONCTL_128p62          (22 << SYS_LPOMONCTL_LFTRIM_SHIFT) /* 128.62% */
#  define SYS_LPOMONCTL_133p31          (23 << SYS_LPOMONCTL_LFTRIM_SHIFT) /* 133.31% */
#  define SYS_LPOMONCTL_138p03          (24 << SYS_LPOMONCTL_LFTRIM_SHIFT) /* 138.03% */
#  define SYS_LPOMONCTL_142p75          (25 << SYS_LPOMONCTL_LFTRIM_SHIFT) /* 142.75% */
#  define SYS_LPOMONCTL_147p32          (26 << SYS_LPOMONCTL_LFTRIM_SHIFT) /* 147.32% */
#  define SYS_LPOMONCTL_152p02          (27 << SYS_LPOMONCTL_LFTRIM_SHIFT) /* 152.02% */
#  define SYS_LPOMONCTL_156p63          (28 << SYS_LPOMONCTL_LFTRIM_SHIFT) /* 156.63% */
#  define SYS_LPOMONCTL_161p38          (29 << SYS_LPOMONCTL_LFTRIM_SHIFT) /* 161.38% */
#  define SYS_LPOMONCTL_165p90          (30 << SYS_LPOMONCTL_LFTRIM_SHIFT) /* 165.90% */
#  define SYS_LPOMONCTL_170p42          (31 << SYS_LPOMONCTL_LFTRIM_SHIFT) /* 170.42% */
#define SYS_LPOMONCTL_HFTRIM_SHIFT      (8)       /* Bits 8-12: High frequency oscillator trim value */
#define SYS_LPOMONCTL_HFTRIM_MASK       (31 << SYS_LPOMONCTL_HFTRIM_SHIFT)
#  define SYS_LPOMONCTL_HFTRIM_29p52    (0 << SYS_LPOMONCTL_HFTRIM_SHIFT) /* 29.52% */
#  define SYS_LPOMONCTL_HFTRIM_34p24    (1 << SYS_LPOMONCTL_HFTRIM_SHIFT) /* 34.24% */
#  define SYS_LPOMONCTL_HFTRIM_38p85    (2 << SYS_LPOMONCTL_HFTRIM_SHIFT) /* 38.85% */
#  define SYS_LPOMONCTL_HFTRIM_43p45    (3 << SYS_LPOMONCTL_HFTRIM_SHIFT) /* 43.45% */
#  define SYS_LPOMONCTL_HFTRIM_47p99    (4 << SYS_LPOMONCTL_HFTRIM_SHIFT) /* 47.99% */
#  define SYS_LPOMONCTL_HFTRIM_52p55    (5 << SYS_LPOMONCTL_HFTRIM_SHIFT) /* 52.55% */
#  define SYS_LPOMONCTL_HFTRIM_57p02    (6 << SYS_LPOMONCTL_HFTRIM_SHIFT) /* 57.02% */
#  define SYS_LPOMONCTL_HFTRIM_61p46    (7 << SYS_LPOMONCTL_HFTRIM_SHIFT) /* 61.46% */
#  define SYS_LPOMONCTL_HFTRIM_65p92    (8 << SYS_LPOMONCTL_HFTRIM_SHIFT) /* 65.92% */
#  define SYS_LPOMONCTL_HFTRIM_70p17    (9 << SYS_LPOMONCTL_HFTRIM_SHIFT) /* 70.17% */
#  define SYS_LPOMONCTL_HFTRIM_74p55    (10 << SYS_LPOMONCTL_HFTRIM_SHIFT) /* 74.55% */
#  define SYS_LPOMONCTL_HFTRIM_78p92    (11 << SYS_LPOMONCTL_HFTRIM_SHIFT) /* 78.92% */
#  define SYS_LPOMONCTL_HFTRIM_83p17    (12 << SYS_LPOMONCTL_HFTRIM_SHIFT) /* 83.17% */
#  define SYS_LPOMONCTL_HFTRIM_87p43    (13 << SYS_LPOMONCTL_HFTRIM_SHIFT) /* 87.43% */
#  define SYS_LPOMONCTL_HFTRIM_91p75    (14 << SYS_LPOMONCTL_HFTRIM_SHIFT) /* 91.75% */
#  define SYS_LPOMONCTL_HFTRIM_95p89    (15 << SYS_LPOMONCTL_HFTRIM_SHIFT) /* 95.89% */
#  define SYS_LPOMONCTL_HFTRIM_100p00   (16 << SYS_LPOMONCTL_HFTRIM_SHIFT) /* 100.00% */
#  define SYS_LPOMONCTL_HFTRIM_104p09   (17 << SYS_LPOMONCTL_HFTRIM_SHIFT) /* 104.09% */
#  define SYS_LPOMONCTL_HFTRIM_108p17   (18 << SYS_LPOMONCTL_HFTRIM_SHIFT) /* 108.17% */
#  define SYS_LPOMONCTL_HFTRIM_112p32   (19 << SYS_LPOMONCTL_HFTRIM_SHIFT) /* 112.32% */
#  define SYS_LPOMONCTL_HFTRIM_116p41   (20 << SYS_LPOMONCTL_HFTRIM_SHIFT) /* 116.41% */
#  define SYS_LPOMONCTL_HFTRIM_120p67   (21 << SYS_LPOMONCTL_HFTRIM_SHIFT) /* 120.67% */
#  define SYS_LPOMONCTL_HFTRIM_124p42   (22 << SYS_LPOMONCTL_HFTRIM_SHIFT) /* 124.42% */
#  define SYS_LPOMONCTL_HFTRIM_128p38   (23 << SYS_LPOMONCTL_HFTRIM_SHIFT) /* 128.38% */
#  define SYS_LPOMONCTL_HFTRIM_132p24   (24 << SYS_LPOMONCTL_HFTRIM_SHIFT) /* 132.24% */
#  define SYS_LPOMONCTL_HFTRIM_136p15   (25 << SYS_LPOMONCTL_HFTRIM_SHIFT) /* 136.15% */
#  define SYS_LPOMONCTL_HFTRIM_140p15   (26 << SYS_LPOMONCTL_HFTRIM_SHIFT) /* 140.15% */
#  define SYS_LPOMONCTL_HFTRIM_143p94   (27 << SYS_LPOMONCTL_HFTRIM_SHIFT) /* 143.94% */
#  define SYS_LPOMONCTL_HFTRIM_148p02   (28 << SYS_LPOMONCTL_HFTRIM_SHIFT) /* 148.02% */
#  define SYS_LPOMONCTL_HFTRIM_151p80   (29 << SYS_LPOMONCTL_HFTRIM_SHIFT) /* 151.80% */
#  define SYS_LPOMONCTL_HFTRIM_155p50   (30 << SYS_LPOMONCTL_HFTRIM_SHIFT) /* 155.50% */
#  define SYS_LPOMONCTL_HFTRIM_159p35   (31 << SYS_LPOMONCTL_HFTRIM_SHIFT) /* 159.35% */
#define SYS_LPOMONCTL_OSCFRQCONFIGCNT   (1 << 16) /* Bit 16:  Configures the counter based on OSC frequency. */
#define SYS_LPOMONCTL_BIASENABLE        (1 << 24) /* Bit 24:  Bias enable. */

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

#define SYS_CLKCNTL_PENA                (1 << 8)  /* Bit 8:  Peripheral enable bit */
#define SYS_CLKCNTL_VCLKR_SHIFT         (16)      /* Bits 16-19: VBUS clock ratio */
#define SYS_CLKCNTL_VCLKR_MASK          (15 << SYS_CLKCNTL_VCLKR_SHIFT)
#  define SYS_CLKCNTL_VCLKR_DIV1        (0 << SYS_CLKCNTL_VCLKR_SHIFT)
#  define SYS_CLKCNTL_VCLKR_DIV2        (1 << SYS_CLKCNTL_VCLKR_SHIFT)
#define SYS_CLKCNTL_VCLKR2_SHIFT        (24)      /* Bits 24-27: VBUS clock2 ratio */
#define SYS_CLKCNTL_VCLKR2_MASK         (15 << SYS_CLKCNTL_VCLKR2_SHIFT)
#  define SYS_CLKCNTL_VCLKR2_DIV1       (0 << SYS_CLKCNTL_VCLKR2_SHIFT)
#  define SYS_CLKCNTL_VCLKR2_DIV2       (1 << SYS_CLKCNTL_VCLKR2_SHIFT)

/* ECP Control Register */

#define SYS_ECPCNTL_ECPDIV_SHIFT        (0)       /* Bits 0-15: ECP divider value */
#define SYS_ECPCNTL_ECPDIV_MASK         (0xffff << SYS_ECPCNTL_ECPDIV_SHIFT)
#  define SYS_ECPCNTL_ECPDIV(n)         ((uint32_t)(n) << SYS_ECPCNTL_ECPDIV_SHIFT)
#define SYS_ECPCNTL_ECPINSEL_SHIFT      (16)      /* Bits 16-17: Select ECP input clock source */
#define SYS_ECPCNTL_ECPINSEL_MASK       (3 << SYS_ECPCNTL_ECPINSEL_SHIFT)
#  define SYS_ECPCNTL_ECPINSEL_LOW      (0 << SYS_ECPCNTL_ECPINSEL_SHIFT) /* Tied Low */
#  define SYS_ECPCNTL_ECPINSEL_HCLK     (1 << SYS_ECPCNTL_ECPINSEL_SHIFT) /* HCLK */
#  define SYS_ECPCNTL_ECPINSEL_EXTCLK   (2 << SYS_ECPCNTL_ECPINSEL_SHIFT) /* External clock */
#define SYS_ECPCNTL_ECPCOS              (1 << 23) /* Bit 23: ECP continue on suspend */
#define SYS_ECPCNTL_ECPSSEL             (1 << 24) /* Bit 24: Select VCLK os OSCIN as for ECLK */

/* DEV Parity Control Register 1 */
#define SYS_DEVCR1_
/* System Exception Control Register */
#define SYS_ECR_
/* System Exception Status Register */

#define SYS_ESR_MPMODE                  (1 << 0)  /* Bit 0:  Current memory protection unit (MPU) mode */
#define SYS_ESR_EXTRST                  (1 << 3)  /* Bit 3:  External reset flag */
#define SYS_ESR_SWRST                   (1 << 4)  /* Bit 4:  Software reset flag */
#define SYS_ESR_CPURST                  (1 << 5)  /* Bit 5:  CPU reset flag */
#define SYS_ESR_WDRST                   (1 << 13) /* Bit 13: Watchdog reset flag */
#define SYS_ESR_OSCRST                  (1 << 14) /* Bit 14: Reset caused by an oscillator failure or PLL cycle slip */
#define SYS_ESR_PORST                   (1 << 15) /* Bit 15: Power-up reset */

#define SYS_ESR_RSTALL                  (0x0000e038)
#define SYS_ESR_FAILALL                 (0x00006000)

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
