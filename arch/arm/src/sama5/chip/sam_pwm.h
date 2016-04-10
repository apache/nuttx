/************************************************************************************
 * arch/arm/src/sama5/chip/sam_pwm.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMA5_CHIP_SAM_PWM_H
#define __ARCH_ARM_SRC_SAMA5_CHIP_SAM_PWM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip/sam_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define SAM_PWM_NCHANNELS          4      /* Four channels numbered 0..3 */

/* PWM Register Offsets *************************************************************/

#define SAM_PWM_CLK_OFFSET         0x0000 /* PWM Clock Register */
#define SAM_PWM_ENA_OFFSET         0x0004 /* PWM Enable Register */
#define SAM_PWM_DIS_OFFSET         0x0008 /* PWM Disable Register */
#define SAM_PWM_SR_OFFSET          0x000c /* PWM Status Register */
#define SAM_PWM_IER1_OFFSET        0x0010 /* PWM Interrupt Enable Register 1 */
#define SAM_PWM_IDR1_OFFSET        0x0014 /* PWM Interrupt Disable Register 1 */
#define SAM_PWM_IMR1_OFFSET        0x0018 /* PWM Interrupt Mask Register 1 */
#define SAM_PWM_ISR1_OFFSET        0x001c /* PWM Interrupt Status Register 1 */
#define SAM_PWM_SCM_OFFSET         0x0020 /* PWM Sync Channels Mode Register */

#ifdef ATSAMA5D4
#  define SAM_PWM_DMAR_OFFSET      0x0024 /* PWM DMA Register */
#endif

#define SAM_PWM_SCUC_OFFSET        0x0028 /* PWM Sync Channels Update Control Register */
#define SAM_PWM_SCUP_OFFSET        0x002c /* PWM Sync Channels Update Period Register */
#define SAM_PWM_SCUPUPD_OFFSET     0x0030 /* PWM Sync Channels Update Period Update Register */
#define SAM_PWM_IER2_OFFSET        0x0034 /* PWM Interrupt Enable Register 2 */
#define SAM_PWM_IDR2_OFFSET        0x0038 /* PWM Interrupt Disable Register 2 */
#define SAM_PWM_IMR2_OFFSET        0x003c /* PWM Interrupt Mask Register 2 */
#define SAM_PWM_ISR2_OFFSET        0x0040 /* PWM Interrupt Status Register 2 */
#define SAM_PWM_OOV_OFFSET         0x0044 /* PWM Output Override Value Register */
#define SAM_PWM_OS_OFFSET          0x0048 /* PWM Output Selection Register */
#define SAM_PWM_OSS_OFFSET         0x004c /* PWM Output Selection Set Register */
#define SAM_PWM_OSC_OFFSET         0x0050 /* PWM Output Selection Clear Register */
#define SAM_PWM_OSSUPD_OFFSET      0x0054 /* PWM Output Selection Set Update Register */
#define SAM_PWM_OSCUPD_OFFSET      0x0058 /* PWM Output Selection Clear Update Register */
#define SAM_PWM_FMR_OFFSET         0x005c /* PWM Fault Mode Register */
#define SAM_PWM_FSR_OFFSET         0x0060 /* PWM Fault Status Register */
#define SAM_PWM_FCR_OFFSET         0x0064 /* PWM Fault Clear Register */
#define SAM_PWM_FPV_OFFSET         0x0068 /* PWM Fault Protection Value Register 1 */
#define SAM_PWM_FPE_OFFSET         0x006c /* PWM Fault Protection Enable Register */
                                          /* 0x0070-0x0078 Reserved */
#define SAM_PWM_ELMR0_OFFSET       0x007c /* PWM Event Line 0 Mode Register */
#define SAM_PWM_ELMR1_OFFSET       0x0080 /* PWM Event Line 1 Mode Register */

#ifdef ATSAMA5D4
#  define SAM_PWM_SSPR_OFFSET      0x00a0 /* PWM Spread Spectrum Register  */
#  define SAM_PWM_SSPUP_OFFSET     0x00a4 /* PWM Spread Spectrum Update Register */
#endif

#define SAM_PWM_SMMR_OFFSET        0x00b0 /* PWM Stepper Motor Mode Register */

#ifdef ATSAMA5D4
#  define SAM_PWM_FPV2_OFFSET      0x00c0 /* PWM Fault Protection Value Register 2 */
#endif

#define SAM_PWM_WPCR_OFFSET        0x00e4 /* PWM Write Protect Control Register */
#define SAM_PWM_WPSR_OFFSET        0x00e8 /* PWM Write Protect Status Register */
                                          /* 0x00ec - 0x00fc Reserved */

#define SAM_PWM_CMPV_OFFSET(n)     (0x0130 + ((int)(n) << 4)) /* n=0..7 */
#define SAM_PWM_CMPVUPD_OFFSET(n)  (0x0134 + ((int)(n) << 4)) /* n=0-7 */
#define SAM_PWM_CMPM_OFFSET(n)     (0x0138 + ((int)(n) << 4)) /* n=0-7 */
#define SAM_PWM_CMPMUPD_OFFSET(n)  (0x013c + ((int)(n) << 4)) /* n=0-7 */

#define SAM_PWM_CMPV0_OFFSET       0x0130 /* PWM Comparison 0 Value Register */
#define SAM_PWM_CMPVUPD0_OFFSET    0x0134 /* PWM Comparison 0 Value Update Register */
#define SAM_PWM_CMPM0_OFFSET       0x0138 /* PWM Comparison 0 Mode Register */
#define SAM_PWM_CMPMUPD0_OFFSET    0x013c /* PWM Comparison 0 Mode Update Register */

#define SAM_PWM_CMPV1_OFFSET       0x0140 /* PWM Comparison 1 Value Register */
#define SAM_PWM_CMPVUPD1_OFFSET    0x0144 /* PWM Comparison 1 Value Update Register */
#define SAM_PWM_CMPM1_OFFSET       0x0148 /* PWM Comparison 1 Mode Register */
#define SAM_PWM_CMPMUPD1_OFFSET    0x014c /* PWM Comparison 1 Mode Update Register */

#define SAM_PWM_CMPV2_OFFSET       0x0150 /* PWM Comparison 2 Value Register */
#define SAM_PWM_CMPVUPD2_OFFSET    0x0154 /* PWM Comparison 2 Value Update Register */
#define SAM_PWM_CMPM2_OFFSET       0x0158 /* PWM Comparison 2 Mode Register */
#define SAM_PWM_CMPMUPD2_OFFSET    0x015c /* PWM Comparison 2 Mode Update Register */

#define SAM_PWM_CMPV3_OFFSET       0x0160 /* PWM Comparison 3 Value Register */
#define SAM_PWM_CMPVUPD3_OFFSET    0x0164 /* PWM Comparison 3 Value Update Register */
#define SAM_PWM_CMPM3_OFFSET       0x0168 /* PWM Comparison 3 Mode Register */
#define SAM_PWM_CMPMUPD3_OFFSET    0x016c /* PWM Comparison 3 Mode Update Register */

#define SAM_PWM_CMPV4_OFFSET       0x0170 /* PWM Comparison 4 Value Register */
#define SAM_PWM_CMPVUPD4_OFFSET    0x0174 /* PWM Comparison 4 Value Update Register */
#define SAM_PWM_CMPM4_OFFSET       0x0178 /* PWM Comparison 4 Mode Register */
#define SAM_PWM_CMPMUPD4_OFFSET    0x017c /* PWM Comparison 4 Mode Update Register */

#define SAM_PWM_CMPV5_OFFSET       0x0180 /* PWM Comparison 5 Value Register */
#define SAM_PWM_CMPVUPD5_OFFSET    0x0184 /* PWM Comparison 5 Value Update Register */
#define SAM_PWM_CMPM5_OFFSET       0x0188 /* PWM Comparison 5 Mode Register */
#define SAM_PWM_CMPMUPD5_OFFSET    0x018c /* PWM Comparison 5 Mode Update Register */

#define SAM_PWM_CMPV6_OFFSET       0x0190 /* PWM Comparison 6 Value Register */
#define SAM_PWM_CMPVUPD6_OFFSET    0x0194 /* PWM Comparison 6 Value Update Register */
#define SAM_PWM_CMPM6_OFFSET       0x0198 /* PWM Comparison 6 Mode Register */
#define SAM_PWM_CMPMUPD6_OFFSET    0x019c /* PWM Comparison 6 Mode Update Register */

#define SAM_PWM_CMPV7_OFFSET       0x01a0 /* PWM Comparison 7 Value Register */
#define SAM_PWM_CMPVUPD7_OFFSET    0x01a4 /* PWM Comparison 7 Value Update Register */
#define SAM_PWM_CMPM7_OFFSET       0x01a8 /* PWM Comparison 7 Mode Register */
#define SAM_PWM_CMPMUPD7_OFFSET    0x01ac /* PWM Comparison 7 Mode Update Register */
                                          /* 0x01b0 - 0x01fc Reserved */
#define SAM_PWM_CHANA_OFFSET(n)    (0x0200 + ((int)(n) << 5)) /* n=0..3 */
#define SAM_PWM_CMR_OFFSET         0x0000 /* PWM Channel Mode Register */
#define SAM_PWM_CDTY_OFFSET        0x0004 /* PWM Channel Duty Cycle Register */
#define SAM_PWM_CDTYUPD_OFFSET     0x0008 /* PWM Channel Duty Cycle Update Register */
#define SAM_PWM_CPRD_OFFSET        0x000c /* PWM Channel Period Register */
#define SAM_PWM_CPRDUPD_OFFSET     0x0010 /* PWM Channel Period Update Register */
#define SAM_PWM_CCNT_OFFSET        0x0014 /* PWM Channel Counter Register */
#define SAM_PWM_DT_OFFSET          0x0018 /* PWM Channel Dead Time Register */
#define SAM_PWM_DTUPD_OFFSET       0x001c /* PWM Channel Dead Time Update Register */

#define SAM_PWM_CHANAB_DELTA       0x0200 /* Offset of second group from the first */

#ifdef ATSAMA5D4
#  define SAM_PWM_CHANB_OFFSET(n)  (0x0400 + ((int)(n) << 5)) /* n=0..3 */
#  define SAM_PWM_CMUPD_OFFSET     0x0000 /* PWM Channel Mode Update Register */
#endif

/* PWM Register Addresses ***********************************************************/

#define SAM_PWM_CLK                (SAM_PWMC_VBASE+SAM_PWM_CLK_OFFSET)
#define SAM_PWM_ENA                (SAM_PWMC_VBASE+SAM_PWM_ENA_OFFSET)
#define SAM_PWM_DIS                (SAM_PWMC_VBASE+SAM_PWM_DIS_OFFSET)
#define SAM_PWM_SR                 (SAM_PWMC_VBASE+SAM_PWM_SR_OFFSET)
#define SAM_PWM_IER1               (SAM_PWMC_VBASE+SAM_PWM_IER1_OFFSET)
#define SAM_PWM_IDR1               (SAM_PWMC_VBASE+SAM_PWM_IDR1_OFFSET)
#define SAM_PWM_IMR1               (SAM_PWMC_VBASE+SAM_PWM_IMR1_OFFSET)
#define SAM_PWM_ISR1               (SAM_PWMC_VBASE+SAM_PWM_ISR1_OFFSET)
#define SAM_PWM_SCM                (SAM_PWMC_VBASE+SAM_PWM_SCM_OFFSET)

#ifdef ATSAMA5D4
#  define SAM_PWM_DMAR             (SAM_PWMC_VBASE+SAM_PWM_DMAR_OFFSET)
#endif

#define SAM_PWM_SCUC               (SAM_PWMC_VBASE+SAM_PWM_SCUC_OFFSET)
#define SAM_PWM_SCUP               (SAM_PWMC_VBASE+SAM_PWM_SCUP_OFFSET)
#define SAM_PWM_SCUPUPD            (SAM_PWMC_VBASE+SAM_PWM_SCUPUPD_OFFSET)
#define SAM_PWM_IER2               (SAM_PWMC_VBASE+SAM_PWM_IER2_OFFSET)
#define SAM_PWM_IDR2               (SAM_PWMC_VBASE+SAM_PWM_IDR2_OFFSET)
#define SAM_PWM_IMR2               (SAM_PWMC_VBASE+SAM_PWM_IMR2_OFFSET)
#define SAM_PWM_ISR2               (SAM_PWMC_VBASE+SAM_PWM_ISR2_OFFSET)
#define SAM_PWM_OOV                (SAM_PWMC_VBASE+SAM_PWM_OOV_OFFSET)
#define SAM_PWM_OS                 (SAM_PWMC_VBASE+SAM_PWM_OS_OFFSET)
#define SAM_PWM_OSS                (SAM_PWMC_VBASE+SAM_PWM_OSS_OFFSET)
#define SAM_PWM_OSC                (SAM_PWMC_VBASE+SAM_PWM_OSC_OFFSET)
#define SAM_PWM_OSSUPD             (SAM_PWMC_VBASE+SAM_PWM_OSSUPD_OFFSET)
#define SAM_PWM_OSCUPD             (SAM_PWMC_VBASE+SAM_PWM_OSCUPD_OFFSET)
#define SAM_PWM_FMR                (SAM_PWMC_VBASE+SAM_PWM_FMR_OFFSET)
#define SAM_PWM_FSR                (SAM_PWMC_VBASE+SAM_PWM_FSR_OFFSET)
#define SAM_PWM_FCR                (SAM_PWMC_VBASE+SAM_PWM_FCR_OFFSET)
#define SAM_PWM_FPV                (SAM_PWMC_VBASE+SAM_PWM_FPV_OFFSET)
#define SAM_PWM_FPE                (SAM_PWMC_VBASE+SAM_PWM_FPE_OFFSET)
#define SAM_PWM_ELMR0              (SAM_PWMC_VBASE+SAM_PWM_ELMR0_OFFSET)
#define SAM_PWM_ELMR1              (SAM_PWMC_VBASE+SAM_PWM_ELMR1_OFFSET)

#ifdef ATSAMA5D4
#  define SAM_PWM_SSPR             (SAM_PWMC_VBASE+SAM_PWM_SSPR_OFFSET)
#  define SAM_PWM_SSPUP            (SAM_PWMC_VBASE+SAM_PWM_SSPUP_OFFSET)
#endif

#define SAM_PWM_SMMR               (SAM_PWMC_VBASE+SAM_PWM_SMMR_OFFSET)

#ifdef ATSAMA5D4
#  define SAM_PWM_FPV2             (SAM_PWMC_VBASE+SAM_PWM_FPV2_OFFSET)
#endif

#define SAM_PWM_WPCR               (SAM_PWMC_VBASE+SAM_PWM_WPCR_OFFSET)
#define SAM_PWM_WPSR               (SAM_PWMC_VBASE+SAM_PWM_WPSR_OFFSET)

#define SAM_PWM_CMPV(n)            (SAM_PWMC_VBASE+SAM_PWM_CMPV_OFFSET(n))
#define SAM_PWM_CMPVUPD(n)         (SAM_PWMC_VBASE+SAM_PWM_CMPVUPD_OFFSET(n))
#define SAM_PWM_CMPM(n)            (SAM_PWMC_VBASE+SAM_PWM_CMPM_OFFSET(n))
#define SAM_PWM_CMPMUPD(n)         (SAM_PWMC_VBASE+SAM_PWM_CMPMUPD_OFFSET(n))

#define SAM_PWM_CMPV0              (SAM_PWMC_VBASE+SAM_PWM_CMPV0_OFFSET)
#define SAM_PWM_CMPVUPD0           (SAM_PWMC_VBASE+SAM_PWM_CMPVUPD0_OFFSET)
#define SAM_PWM_CMPM0              (SAM_PWMC_VBASE+SAM_PWM_CMPM0_OFFSET)
#define SAM_PWM_CMPMUPD0           (SAM_PWMC_VBASE+SAM_PWM_CMPMUPD0_OFFSET)

#define SAM_PWM_CMPV1              (SAM_PWMC_VBASE+SAM_PWM_CMPV1_OFFSET)
#define SAM_PWM_CMPVUPD1           (SAM_PWMC_VBASE+SAM_PWM_CMPVUPD1_OFFSET)
#define SAM_PWM_CMPM1              (SAM_PWMC_VBASE+SAM_PWM_CMPM1_OFFSET)
#define SAM_PWM_CMPMUPD1           (SAM_PWMC_VBASE+SAM_PWM_CMPMUPD1_OFFSET)

#define SAM_PWM_CMPV2              (SAM_PWMC_VBASE+SAM_PWM_CMPV2_OFFSET)
#define SAM_PWM_CMPVUPD2           (SAM_PWMC_VBASE+SAM_PWM_CMPVUPD2_OFFSET)
#define SAM_PWM_CMPM2              (SAM_PWMC_VBASE+SAM_PWM_CMPM2_OFFSET)
#define SAM_PWM_CMPMUPD2           (SAM_PWMC_VBASE+SAM_PWM_CMPMUPD2_OFFSET)

#define SAM_PWM_CMPV3              (SAM_PWMC_VBASE+SAM_PWM_CMPV3_OFFSET)
#define SAM_PWM_CMPVUPD3           (SAM_PWMC_VBASE+SAM_PWM_CMPVUPD3_OFFSET)
#define SAM_PWM_CMPM3              (SAM_PWMC_VBASE+SAM_PWM_CMPM3_OFFSET)
#define SAM_PWM_CMPMUPD3           (SAM_PWMC_VBASE+SAM_PWM_CMPMUPD3_OFFSET)

#define SAM_PWM_CMPV4              (SAM_PWMC_VBASE+SAM_PWM_CMPV4_OFFSET)
#define SAM_PWM_CMPVUPD4           (SAM_PWMC_VBASE+SAM_PWM_CMPVUPD4_OFFSET)
#define SAM_PWM_CMPM4              (SAM_PWMC_VBASE+SAM_PWM_CMPM4_OFFSET)
#define SAM_PWM_CMPMUPD4           (SAM_PWMC_VBASE+SAM_PWM_CMPMUPD4_OFFSET)

#define SAM_PWM_CMPV5              (SAM_PWMC_VBASE+SAM_PWM_CMPV5_OFFSET)
#define SAM_PWM_CMPVUPD5           (SAM_PWMC_VBASE+SAM_PWM_CMPVUPD5_OFFSET)
#define SAM_PWM_CMPM5              (SAM_PWMC_VBASE+SAM_PWM_CMPM5_OFFSET)
#define SAM_PWM_CMPMUPD5           (SAM_PWMC_VBASE+SAM_PWM_CMPMUPD5_OFFSET)

#define SAM_PWM_CMPV6              (SAM_PWMC_VBASE+SAM_PWM_CMPV6_OFFSET)
#define SAM_PWM_CMPVUPD6           (SAM_PWMC_VBASE+SAM_PWM_CMPVUPD6_OFFSET)
#define SAM_PWM_CMPM6              (SAM_PWMC_VBASE+SAM_PWM_CMPM6_OFFSET)
#define SAM_PWM_CMPMUPD6           (SAM_PWMC_VBASE+SAM_PWM_CMPMUPD6_OFFSET)

#define SAM_PWM_CMPV7              (SAM_PWMC_VBASE+SAM_PWM_CMPV7_OFFSET)
#define SAM_PWM_CMPVUPD7           (SAM_PWMC_VBASE+SAM_PWM_CMPVUPD7_OFFSET)
#define SAM_PWM_CMPM7              (SAM_PWMC_VBASE+SAM_PWM_CMPM7_OFFSET)
#define SAM_PWM_CMPMUPD7           (SAM_PWMC_VBASE+SAM_PWM_CMPMUPD7_OFFSET)

#define SAM_PWM_CHANA_BASE(n)      (SAM_PWMC_VBASE+SAM_PWM_CHANA_OFFSET(n))
#define SAM_PWM_CMR(n)             (SAM_PWM_CHANA_BASE(n)+SAM_PWM_CMR_OFFSET)
#define SAM_PWM_CDTY(n)            (SAM_PWM_CHANA_BASE(n)+SAM_PWM_CDTY_OFFSET)
#define SAM_PWM_CDTYUPD(n)         (SAM_PWM_CHANA_BASE(n)+SAM_PWM_CDTYUPD_OFFSET)
#define SAM_PWM_CPRD(n)            (SAM_PWM_CHANA_BASE(n)+SAM_PWM_CPRD_OFFSET)
#define SAM_PWM_CPRDUPD(n)         (SAM_PWM_CHANA_BASE(n)+SAM_PWM_CPRDUPD_OFFSET)
#define SAM_PWM_CCNT(n)            (SAM_PWM_CHANA_BASE(n)+SAM_PWM_CCNT_OFFSET)
#define SAM_PWM_DT(n)              (SAM_PWM_CHANA_BASE(n)+SAM_PWM_DT_OFFSET)
#define SAM_PWM_DTUPD(n)           (SAM_PWM_CHANA_BASE(n)+SAM_PWM_DTUPD_OFFSET)

#ifdef ATSAMA5D4
#  define SAM_PWM_CHANB_BASE(n)    (SAM_PWMC_VBASE+define SAM_PWM_CHANB_OFFSET(n))
#  define SAM_PWM_CMUPD(n)         (SAM_PWM_CHANB_BASE(n)+SAM_PWM_CMUPD_OFFSET)
#endif

/* PWM Register Bit Definitions *****************************************************/

/* PWM Clock Register */

#define PWM_CLK_DIVA_SHIFT         (0)       /* Bits 0-7: CLKA Divide Factor */
#define PWM_CLK_DIVA_MASK          (0xff << PWM_CLK_DIVA_SHIFT)
#  define PWM_CLK_DIVA_OFF         (0 << PWM_CLK_DIVA_SHIFT)             /* CLKA clock = off */
#  define PWM_CLK_DIVA_PREA        (1 << PWM_CLK_DIVA_SHIFT)             /* CLKA clock = clock selected by PREA  */
#  define PWM_CLK_DIVA(n)          ((uint32_t)(n) << PWM_CLK_DIVA_SHIFT) /* CLKA clock = clock selected by PREA / DIVA */
#define PWM_CLK_PREA_SHIFT         (8)       /* Bits 8-11: CLKA Source Clock Selection */
#define PWM_CLK_PREA_MASK          (15 << PWM_CLK_PREA_SHIFT)
#  define PWM_CLK_PREA_DIV1        (0 << PWM_CLK_PREA_SHIFT)  /* MCK */
#  define PWM_CLK_PREA_DIV2        (1 << PWM_CLK_PREA_SHIFT)  /* MCK/2 */
#  define PWM_CLK_PREA_DIV4        (2 << PWM_CLK_PREA_SHIFT)  /* MCK/4 */
#  define PWM_CLK_PREA_DIV8        (3 << PWM_CLK_PREA_SHIFT)  /* MCK/8 */
#  define PWM_CLK_PREA_DIV16       (4 << PWM_CLK_PREA_SHIFT)  /* MCK/16 */
#  define PWM_CLK_PREA_DIV32       (5 << PWM_CLK_PREA_SHIFT)  /* MCK/32 */
#  define PWM_CLK_PREA_DIV64       (6 << PWM_CLK_PREA_SHIFT)  /* MCK/64 */
#  define PWM_CLK_PREA_DIV128      (7 << PWM_CLK_PREA_SHIFT)  /* MCK/128 */
#  define PWM_CLK_PREA_DIV256      (8 << PWM_CLK_PREA_SHIFT)  /* MCK/256 */
#  define PWM_CLK_PREA_DIV512      (9 << PWM_CLK_PREA_SHIFT)  /* MCK/512 */
#  define PWM_CLK_PREA_DIV1024     (10 << PWM_CLK_PREA_SHIFT) /* MCK/1024 */
#define PWM_CLK_DIVB_SHIFT         (16)      /* Bits 16-23: CLKB Divide Factor */
#  define PWM_CLK_DIVB_OFF         (0 << PWM_CLK_DIVB_SHIFT)             /* CLKB clock = off */
#  define PWM_CLK_DIVB_PREB        (1 << PWM_CLK_DIVB_SHIFT)             /* CLKB clock = clock selected by PREB  */
#  define PWM_CLK_DIVB(n)          ((uint32_t)(n) << PWM_CLK_DIVB_SHIFT) /* CLKB clock = clock selected by PREB / DIVB */
#define PWM_CLK_PREB_SHIFT         (24)      /* Bits 24-27: CLKB Source Clock Selection */
#define PWM_CLK_PREB_MASK          (15 << PWM_CLK_PREB_SHIFT)
#  define PWM_CLK_PREB_DIV1        (0 << PWM_CLK_PREB_SHIFT)  /* MCK */
#  define PWM_CLK_PREB_DIV2        (1 << PWM_CLK_PREB_SHIFT)  /* MCK/2 */
#  define PWM_CLK_PREB_DIV4        (2 << PWM_CLK_PREB_SHIFT)  /* MCK/4 */
#  define PWM_CLK_PREB_DIV8        (3 << PWM_CLK_PREB_SHIFT)  /* MCK/8 */
#  define PWM_CLK_PREB_DIV16       (4 << PWM_CLK_PREB_SHIFT)  /* MCK/16 */
#  define PWM_CLK_PREB_DIV32       (5 << PWM_CLK_PREB_SHIFT)  /* MCK/32 */
#  define PWM_CLK_PREB_DIV64       (6 << PWM_CLK_PREB_SHIFT)  /* MCK/64 */
#  define PWM_CLK_PREB_DIV128      (7 << PWM_CLK_PREB_SHIFT)  /* MCK/128 */
#  define PWM_CLK_PREB_DIV256      (8 << PWM_CLK_PREB_SHIFT)  /* MCK/256 */
#  define PWM_CLK_PREB_DIV512      (9 << PWM_CLK_PREB_SHIFT)  /* MCK/512 */
#  define PWM_CLK_PREB_DIV1024     (10 << PWM_CLK_PREB_SHIFT) /* MCK/1024 */

/* PWM Enable Register, PWM Disable Register, and PWM Status Register */

#define PWM_CHID(n)                (1 << (n))      /* Bits 0-3: Channel ID n, n=0..3 */

/* PWM Interrupt Enable Register 1, PWM Interrupt Disable Register 1, PWM Interrupt
 * Mask Register 1, and PWM Interrupt Status Register 1
 */

#define PWM_INT1_CHID(n)           (1 << (n))      /* Bits 0-3: Counter Event on Channel n Interrupt, n=0..3 */
#define PWM_INT1_FCHID(n)          (1 << ((n)+16)) /* Bits 16-19: Fault Protection Trigger on Channel n Interrupt, n=0..3 */
#define PWM_INT1_ALL               (0x000f000f)

/* PWM Sync Channels Mode Register */

#define PWM_SCM_SYNC(n)            (1 << (n)) /* Bits 0-3: Synchronous Channel n, n=0..3 */
#  define PWM_SCM_SYNC0            (1 << 0)   /* Bits 0:  Synchronous Channel 0 */
#  define PWM_SCM_SYNC1            (1 << 1)   /* Bits 1:  Synchronous Channel 1 */
#  define PWM_SCM_SYNC2            (1 << 2)   /* Bits 2:  Synchronous Channel 2 */
#  define PWM_SCM_SYNC3            (1 << 3)   /* Bits 3:  Synchronous Channel 3 */
#define PWM_SCM_UPDM_SHIFT         (16)       /* Bits 16-17: Synchronous Channels Update Mode */
#define PWM_SCM_UPDM_MASK          (3 << PWM_SCM_UPDM_SHIFT)
#  define PWM_SCM_UPDM_MODE0       (0 << PWM_SCM_UPDM_SHIFT) /* Manual double buffer / manual channel update */
#  define PWM_SCM_UPDM_MODE1       (1 << PWM_SCM_UPDM_SHIFT) /* Manual double buffer / automatic channel update */

#ifdef ATSAMA5D4
#  define PWM_SCM_PTRM             (1 << 20)  /* Bit 20: DMA Transfer Request Mode */
#  define PWM_SCM_PTRCS_SHIFT      (21)       /* Bits 21-23: DMA Transfer Request Comparison Selection */
#  define PWM_SCM_PTRCS_MASK       (7 << PWM_SCM_PTRCS_SHIFT)
#    define PWM_SCM_PTRCS(n)       ((uint32_t)(n) << PWM_SCM_PTRCS_SHIFT)
#endif

#ifdef ATSAMA5D4
/* PWM DMA Register */

#  define PWM_DMAR_DMADUTY_SHIFT   (0)       /* Bits 0-23:  Duty-Cycle Holding Register for DMA Access */
#  define PWM_DMAR_DMADUTY_MASK    (0x00ffffff << PWM_DMAR_DMADUTY_SHIFT)
#    define PWM_DMAR_DMADUTY(n)    ((uint32_t)(n) << PWM_DMAR_DMADUTY_SHIFT)
#endif

/* PWM Sync Channels Update Control Register */

#define PWM_SCUC_UPDULOCK          (1 << 0)  /* Bit 0:  Synchronous Channels Update Unlock */

/* PWM Sync Channels Update Period Register */

#define PWM_SCUP_UPR_SHIFT         (0)       /* Bits 0-3: Update Period */
#define PWM_SCUP_UPR_MASK          (15 << PWM_SCUP_UPR_SHIFT)
#  define PWM_SCUP_UPR(n)          ((uint32_t)(n) << PWM_SCUP_UPR_SHIFT)
#define PWM_SCUP_UPRCNT_SHIFT      (4)       /* Bits 4-7: Update Period Counter */
#define PWM_SCUP_UPRCNT_MASK       (15 << PWM_SCUP_UPRCNT_SHIFT)
#  define PWM_SCUP_UPRCNT(n)       ((uint32_t)(n) << PWM_SCUP_UPRCNT_SHIFT)

/* PWM Sync Channels Update Period Update Register */

#define PWM_SCUPUPD_UPRUPD_SHIFT   (0)       /* Bits 0-3: Update Period Update */
#define PWM_SCUPUPD_UPRUPD_MASK    (15 << PWM_SCUPUPD_UPRUPD_SHIFT)
#  define PWM_SCUPUPD_UPRUPD(n)    ((uint32_t)(n) << PWM_SCUPUPD_UPRUPD_SHIFT)

/* PWM Interrupt Enable Register 2, PWM Interrupt Disable Register 2, PWM Interrupt
 * Mask Register 2, and PWM Interrupt Status Register 2.
 */

#define PWM_INT2_WRDY              (1 << 0)  /* Bit 0:  Write Ready for Synchronous Channels Update Interrupt Enable */
#define PWM_INT2_UNRE              (1 << 3)  /* Bit 3:  Synchronous Channels Update Underrun Error Interrupt Enable */
#define PWM_INT2_CMPM(n)           (1 << ((n)+8))  /* Bits 8-15:  Comparison n Match Interrupt Enable, n=0..7 */
#  define PWM_INT2_CMPM0           (1 << 8)  /* Bit 8:  Comparison 0 Match Interrupt Enable */
#  define PWM_INT2_CMPM1           (1 << 9)  /* Bit 9:  Comparison 1 Match Interrupt Enable */
#  define PWM_INT2_CMPM2           (1 << 10) /* Bit 10: Comparison 2 Match Interrupt Enable */
#  define PWM_INT2_CMPM3           (1 << 11) /* Bit 11: Comparison 3 Match Interrupt Enable */
#  define PWM_INT2_CMPM4           (1 << 12) /* Bit 12: Comparison 4 Match Interrupt Enable */
#  define PWM_INT2_CMPM5           (1 << 13) /* Bit 13: Comparison 5 Match Interrupt Enable */
#  define PWM_INT2_CMPM6           (1 << 14) /* Bit 14: Comparison 6 Match Interrupt Enable */
#  define PWM_INT2_CMPM7           (1 << 15) /* Bit 15: Comparison 7 Match Interrupt Enable */
#define PWM_INT2_CMPU(n)           (1 << ((n)+16)) /* Bits 16-23:  Comparison n Update Interrupt Enable, n=0..7 */
#  define PWM_INT2_CMPU0           (1 << 16) /* Bit 16:  Comparison 0 Update Interrupt Enable */
#  define PWM_INT2_CMPU1           (1 << 17) /* Bit 17:  Comparison 1 Update Interrupt Enable */
#  define PWM_INT2_CMPU2           (1 << 18) /* Bit 18:  Comparison 2 Update Interrupt Enable */
#  define PWM_INT2_CMPU3           (1 << 19) /* Bit 19:  Comparison 3 Update Interrupt Enable */
#  define PWM_INT2_CMPU4           (1 << 20) /* Bit 20:  Comparison 4 Update Interrupt Enable */
#  define PWM_INT2_CMPU5           (1 << 21) /* Bit 21:  Comparison 5 Update Interrupt Enable */
#  define PWM_INT2_CMPU6           (1 << 22) /* Bit 22:  Comparison 6 Update Interrupt Enable */
#  define PWM_INT2_CMPU7           (1 << 23) /* Bit 23:  Comparison 7 Update Interrupt Enable */
#define PWM_INT2_ALL               (0x00ffff09)

/* PWM Output Override Value Register */

#define PWM_OOV_H(n)               (1 << (n))      /* Bits 0-3: Output Override PWMH channel n, n=0..3 */
#  define PWM_OOV_H0               (1 << 0)  /* Bit 0:  Output Override PWMH channel 0 */
#  define PWM_OOV_H1               (1 << 1)  /* Bit 1:  Output Override PWMH channel 1 */
#  define PWM_OOV_H2               (1 << 2)  /* Bit 2:  Output Override PWMH channel 2 */
#  define PWM_OOV_H3               (1 << 3)  /* Bit 3:  Output Override PWMH channel 3 */
#define PWM_OOV_L(n)               (1 << ((n)+16)) /* Bits 16-19: Output Override PWML channel n, n=0..3 */
#  define PWM_OOV_L0               (1 << 16) /* Bit 16: Output Override PWML channel 0 */
#  define PWM_OOV_L1               (1 << 17) /* Bit 17: Output Override PWML channel 1 */
#  define PWM_OOV_L2               (1 << 18) /* Bit 18: Output Override PWML channel 2 */
#  define PWM_OOV_L3               (1 << 19) /* Bit 19: Output Override PWML channel 3 */

/* PWM Output Selection Register */

#define PWM_OS_H(n)                (1 << (n))      /* Bits 0-3: Output Selection for PWMH channel n, n=0..3 */
#  define PWM_OS_H0                (1 << 0)  /* Bit 0:  Output Selection for PWMH channel 0 */
#  define PWM_OS_H1                (1 << 1)  /* Bit 0:  Output Selection for PWMH channel 0 */
#  define PWM_OS_H2                (1 << 2)  /* Bit 0:  Output Selection for PWMH channel 0 */
#  define PWM_OS_H3                (1 << 3)  /* Bit 0:  Output Selection for PWMH channel 0 */
#define PWM_OS_L(n)                (1 << ((n)+16)) /* Bits 16-19: Output Selection for PWML channel n, n=0..3 */
#  define PWM_OS_L0                (1 << 16) /* Bit 16: Output Selection for PWML channel 0 */
#  define PWM_OS_L1                (1 << 17) /* Bit 17: Output Selection for PWML channel 1 */
#  define PWM_OS_L2                (1 << 18) /* Bit 18: Output Selection for PWML channel 2 */
#  define PWM_OS_L3                (1 << 19) /* Bit 19: Output Selection for PWML channel 3 */

/* PWM Output Selection Set Register */

#define PWM_OSS_H(n)               (1 << (n))      /* Bits 0-3: Output Selection Set for PWMH channel n, n=0..3 */
#  define PWM_OSS_H0               (1 << 0)  /* Bit 0:  Output Selection Set for PWMH channel 0 */
#  define PWM_OSS_H1               (1 << 1)  /* Bit 1:  Output Selection Set for PWMH channel 1 */
#  define PWM_OSS_H2               (1 << 2)  /* Bit 2:  Output Selection Set for PWMH channel 2 */
#  define PWM_OSS_H3               (1 << 3)  /* Bit 3:  Output Selection Set for PWMH channel 3 */
#define PWM_OSS_L(n)               (1 << ((n)+16)) /* Bits 16-19: Output Selection Set for PWML channel n, n=0..3 */
#  define PWM_OSS_L0               (1 << 16) /* Bit 16: Output Selection Set for PWML channel 0 */
#  define PWM_OSS_L1               (1 << 17) /* Bit 17: Output Selection Set for PWML channel 1 */
#  define PWM_OSS_L2               (1 << 18) /* Bit 18: Output Selection Set for PWML channel 2 */
#  define PWM_OSS_L3               (1 << 19) /* Bit 19: Output Selection Set for PWML channel 3 */

/* PWM Output Selection Clear Register */

#define PWM_OSC_H(n)               (1 << (n))      /* Bits 0-3: Output Selection Clear for PWMH channel n, n=0..3 */
#  define PWM_OSC_H0               (1 << 0)  /* Bit 0:  Output Selection Clear for PWMH channel 0 */
#  define PWM_OSC_H1               (1 << 1)  /* Bit 1:  Output Selection Clear for PWMH channel 1 */
#  define PWM_OSC_H2               (1 << 2)  /* Bit 2:  Output Selection Clear for PWMH channel 2 */
#  define PWM_OSC_H3               (1 << 3)  /* Bit 3:  Output Selection Clear for PWMH channel 3 */
#define PWM_OSC_L(n)               (1 << ((n)+16)) /* Bits 16-19: Output Selection Clear for PWML channel n, n=0..3 */
#  define PWM_OSC_L0               (1 << 16) /* Bit 16: Output Selection Clear for PWML channel 0 */
#  define PWM_OSC_L1               (1 << 17) /* Bit 17: Output Selection Clear for PWML channel 1 */
#  define PWM_OSC_L2               (1 << 18) /* Bit 18: Output Selection Clear for PWML channel 2 */
#  define PWM_OSC_L3               (1 << 19) /* Bit 19: Output Selection Clear for PWML channel 3 */

/* PWM Output Selection Set Update Register */

#define PWM_OSSUPD_H(n)            (1 << (n))      /* Bits 0-3: Output Selection Set for PWMH channel n, n=0..3 */
#  define PWM_OSSUPD_H0            (1 << 0)  /* Bit 0:  Output Selection Set for PWMH channel 0 */
#  define PWM_OSSUPD_H1            (1 << 1)  /* Bit 1:  Output Selection Set for PWMH channel 1 */
#  define PWM_OSSUPD_H2            (1 << 2)  /* Bit 2:  Output Selection Set for PWMH channel 2 */
#  define PWM_OSSUPD_H3            (1 << 3)  /* Bit 3:  Output Selection Set for PWMH channel 3 */
#define PWM_OSSUPD_L(n)            (1 << ((n)+16)) /* Bits 16-19: Output Selection Set for PWML channel n, n=0..3 */
#  define PWM_OSSUPD_L0            (1 << 16) /* Bit 16: Output Selection Set for PWML channel 0 */
#  define PWM_OSSUPD_L1            (1 << 17) /* Bit 17: Output Selection Set for PWML channel 1 */
#  define PWM_OSSUPD_L2            (1 << 18) /* Bit 18: Output Selection Set for PWML channel 2 */
#  define PWM_OSSUPD_L3            (1 << 19) /* Bit 19: Output Selection Set for PWML channel 3 */

/* PWM Output Selection Clear Update Register */

#define PWM_OSCUPD_H(n)            (1 << (n))      /* Bits 0-3: Output Selection Clear for PWMH channel n, n=0..3 */
#  define PWM_OSCUPD_H0            (1 << 0)  /* Bit 0:  Output Selection Clear for PWMH channel 0 */
#  define PWM_OSCUPD_H1            (1 << 1)  /* Bit 1:  Output Selection Clear for PWMH channel 1 */
#  define PWM_OSCUPD_H2            (1 << 2)  /* Bit 2:  Output Selection Clear for PWMH channel 2 */
#  define PWM_OSCUPD_H3            (1 << 3)  /* Bit 3:  Output Selection Clear for PWMH channel 3 */
#define PWM_OSCUPD_L(n)            (1 << ((n)+16)) /* Bits 16-19: Output Selection Clear for PWML channel n, n=0..3 */
#  define PWM_OSCUPD_L0            (1 << 16) /* Bit 16: Output Selection Clear for PWML channel 0 */
#  define PWM_OSCUPD_L1            (1 << 17) /* Bit 17: Output Selection Clear for PWML channel 1 */
#  define PWM_OSCUPD_L2            (1 << 18) /* Bit 18: Output Selection Clear for PWML channel 2 */
#  define PWM_OSCUPD_L3            (1 << 19) /* Bit 19: Output Selection Clear for PWML channel 3 */

/* PWM Fault Mode Register */

#define PWM_FMR_FPOL_SHIFT         (0)       /* Bits 0-7: Fault Polarity */
#define PWM_FMR_FPOL_MASK          (0xff << PWM_FMR_FPOL_SHIFT)
#  define PWM_FMR_FPOL(n)          ((uint32_t)(n) << PWM_FMR_FPOL_SHIFT)
#define PWM_FMR_FMOD_SHIFT         (8)       /* Bits 8-15: Fault Activation Mode */
#define PWM_FMR_FMOD_MASK          (0xff << PWM_FMR_FMOD_SHIFT)
#  define PWM_FMR_FMOD(n)          ((uint32_t)(n) << PWM_FMR_FMOD_SHIFT)
#define PWM_FMR_FFIL_SHIFT         (16)      /* Bits 16-23: Fault Filtering */
#define PWM_FMR_FFIL_MASK          (0xff << PWM_FMR_FFIL_SHIFT)
#  define PWM_FMR_FFIL(n)          ((uint32_t)(n) << PWM_FMR_FFIL_SHIFT)

/* PWM Fault Status Register */

#define PWM_FSR_FIV_SHIFT          (0)     /* Bits 0-7: Fault Input Value */
#define PWM_FSR_FIV_MASK           (0xff << PWM_FSR_FIV_SHIFT)
#  define PWM_FSR_FIV(n)           ((uint32_t)(n) << PWM_FSR_FIV_SHIFT)
#define PWM_FSR_FS_SHIFT           (8)     /* Bits 8-15: Fault Status */
#define PWM_FSR_FS_MASK            (0xff << PWM_FSR_FS_SHIFT)
#  define PWM_FSR_FS(n)            ((uint32_t)(n) << PWM_FSR_FS_SHIFT)

/* PWM Fault Clear Register */

#define PWM_FCR_FCLR_SHIFT         (0)     /* Bits 0-7: Fault Clear */
#define PWM_FCR_FCLR_MASK          (0xff << PWM_FCR_FCLR_SHIFT)
#  define PWM_FCR_FCLR(n)          (1 << (n)) /* Fault Clear n, n=0..7 */

/* PWM Fault Protection Value Register 1 */

#define PWM_FPV_H(n)               (1 << (n))      /* Bits 0-3: Fault Protection PWMH output on channel n, n=0..3 */
#  define PWM_FPV_H0               (1 << 0)  /* Bit 0:  Fault Protection PWMH output on channel 0 */
#  define PWM_FPV_H1               (1 << 1)  /* Bit 1:  Fault Protection PWMH output on channel 1 */
#  define PWM_FPV_H2               (1 << 2)  /* Bit 2:  Fault Protection PWMH output on channel 2 */
#  define PWM_FPV_H3               (1 << 3)  /* Bit 3:  Fault Protection PWMH output on channel 3 */
#define PWM_FPV_L(n)               (1 << ((n)+16)) /* Bits 16-19: Fault Protection PWML output on channel n, n=0..3 */
#  define PWM_FPV_L0               (1 << 16) /* Bit 16: Fault Protection PWML output on channel 0 */
#  define PWM_FPV_L1               (1 << 17) /* Bit 17: Fault Protection PWML output on channel 1 */
#  define PWM_FPV_L2               (1 << 18) /* Bit 18: Fault Protection PWML output on channel 2 */
#  define PWM_FPV_L3               (1 << 19) /* Bit 19: Fault Protection PWML output on channel 3 */

/* PWM Fault Protection Enable Register */

#define PWM_FPE_SHIFT(n)           (1 << ((n) << 3)) /* Fault Protection Enable for channel n, n=0..3 */
#define PWM_FPE_MASK(n)            (0xff << PWM_FPE_SHIFT(n))
#  define PWM_FPE(n,v)             ((uint32_t)(v) << PWM_FPE_SHIFT(n))
#define PWM_FPE0_SHIFT             (0)      /* Bits 0-7: Fault Protection Enable for channel 0 */
#define PWM_FPE0_MASK              (0xff << PWM_FPE0_SHIFT)
#  define PWM_FPE0(n)              ((uint32_t)(n) << PWM_FPE0_SHIFT)
#define PWM_FPE1_SHIFT             (8)      /* Bits 8-15: Fault Protection Enable for channel 1 */
#define PWM_FPE1_MASK              (0xff << PWM_FPE1_SHIFT)
#  define PWM_FPE1(n)              ((uint32_t)(n) << PWM_FPE1_SHIFT)
#define PWM_FPE2_SHIFT             (16)     /* Bits 16-23: Fault Protection Enable for channel 2 */
#define PWM_FPE2_MASK              (0xff << PWM_FPE2_SHIFT)
#  define PWM_FPE2(n)              ((uint32_t)(n) << PWM_FPE2_SHIFT)
#define PWM_FPE3_SHIFT             (24)     /* Bits 24-31: Fault Protection Enable for channel 3 */
#define PWM_FPE3_MASK              (0xff << PWM_FPE3_SHIFT)
#  define PWM_FPE3(n)              ((uint32_t)(n) << PWM_FPE3_SHIFT)

/* PWM Event Line 0/1 Mode Register */

#define PWM_ELMR_CSEL(n)           (1 << (n))      /* Bits 0-7: Comparison n Selection, n=0..7 */
#  define PWM_ELMR_CSEL0           (1 << 0)  /* Bit 0:  Comparison 0 Selection */
#  define PWM_ELMR_CSEL1           (1 << 1)  /* Bit 1:  Comparison 1 Selection */
#  define PWM_ELMR_CSEL2           (1 << 2)  /* Bit 2:  Comparison 2 Selection */
#  define PWM_ELMR_CSEL3           (1 << 3)  /* Bit 3:  Comparison 3 Selection */
#  define PWM_ELMR_CSEL4           (1 << 4)  /* Bit 4:  Comparison 4 Selection */
#  define PWM_ELMR_CSEL5           (1 << 5)  /* Bit 5:  Comparison 5 Selection */
#  define PWM_ELMR_CSEL6           (1 << 6)  /* Bit 6:  Comparison 6 Selection */
#  define PWM_ELMR_CSEL7           (1 << 7)  /* Bit 7:  Comparison 7 Selection */

#ifdef ATSAMA5D4
/* PWM Spread Spectrum Register  */

#  define PWM_SSPR_SPRD_SHIFT      (0)       /* Bits 0-23: Spread Spectrum Limit Value */
#  define PWM_SSPR_SPRD_MASK       (0x00ffffff << PWM_SSPR_SPRD_SHIFT)
#    define PWM_SSPR_SPRD(n)       ((uint32_t)(n) << PWM_SSPR_SPRD_SHIFT)
#  define PWM_SSPR_SPRDM           (1 << 24) /* Bit 24: Spread Spectrum Counter Mode */
#endif

#ifdef ATSAMA5D4
/* PWM Spread Spectrum Update Register */

#  define PWM_SSPUP_SPRDUP_SHIFT   (0)       /* Bits 0-23: Spread Spectrum Limit Value Update */
#  define PWM_SSPUP_SPRDUP_MASK    (0x00ffffff << PWM_SSPUP_SPRDUP_SHIFT)
#    define PWM_SSPUP_SPRDUP(n)    ((uint32_t)(n) << PWM_SSPUP_SPRDUP_SHIFT)
#endif

/* PWM Stepper Motor Mode Register */

#define PWM_SMMR_GCEN(n)           (1 << (n))      /* Bits 0-1: Gray Count ENable, n=0..1 */
#  define PWM_SMMR_GCEN0           (1 << 0)        /* Bit 0: Enables gray count generation on PWMH/L[0] and PWMH/L[1] */
#  define PWM_SMMR_GCEN1           (1 << 1)        /* Bits 1: Enables gray count generation on PWMH/L[2] and PWMH/L[3] */
#define PWM_SMMR_DOWN(n)           (1 << ((n)+16)) /* Bits 16-17: DOWN Count, n=0..1 */
#  define PWM_SMMR_DOWN0           (1 << 16)
#  define PWM_SMMR_DOWN1           (1 << 17)

#ifdef ATSAMA5D4
/* PWM Fault Protection Value Register 2 */

#define PWM_FPV2_FPZH(n)           (1 << (n)) /* Bits 0-3: Fault Protection Hi-Z for PWMH channel n, n=0..3 */
#  define PWM_FPV2_FPZH0           (1 << 0)   /* Bit 0:  Fault Protection Hi-Z for PWMH channel 0 */
#  define PWM_FPV2_FPZH1           (1 << 1)   /* Bit 1:  Fault Protection Hi-Z for PWMH channel 1 */
#  define PWM_FPV2_FPZH2           (1 << 2)   /* Bit 2:  Fault Protection Hi-Z for PWMH channel 2 */
#  define PWM_FPV2_FPZH3           (1 << 3)   /* Bit 3:  Fault Protection Hi-Z for PWMH channel 3 */
#define PWM_FPV2_L(n)              (1 << ((n)+16)) /* Bits 16-19: Fault Protection Hi-Z for PWML channel n, n=0..3 */
#  define PWM_FPV2_FPZL0           (1 << 16)  /* Bit 16: Fault Protection Hi-Z for PWML channel 0 */
#  define PWM_FPV2_FPZL1           (1 << 17)  /* Bit 17: Fault Protection Hi-Z for PWML channel 1 */
#  define PWM_FPV2_FPZL2           (1 << 18)  /* Bit 18: Fault Protection Hi-Z for PWML channel 2 */
#  define PWM_FPV2_FPZL3           (1 << 19)  /* Bit 19: FFault Protection Hi-Z for PWML channel 3 */
#endif

/* PWM Write Protect Control Register */

#define PWM_WPCR_WPCMD_SHIFT       (0)       /* Bits 0-1: Write Protect Command */
#define PWM_WPCR_WPCMD_MASK        (3 << PWM_WPCR_WPCMD_SHIFT)
#  define PWM_WPCR_WPCMD_DSWPROT   (0 << PWM_WPCR_WPCMD_SHIFT) /* Disable software write protection */
#  define PWM_WPCR_WPCMD_ESWPROT   (1 << PWM_WPCR_WPCMD_SHIFT) /* Enable software write protection */
#  define PWM_WPCR_WPCMD_EHWPROT   (2 << PWM_WPCR_WPCMD_SHIFT) /* Enable hardware write protection */
#define PWM_WPCR_WPRG(n)           (1 << ((n)+2)) /* Bits 2-7: Write Protect Register Group n, n=0..5 */
#  define PWM_WPCR_WPRG0           (1 << 2)  /* Bit 2:  Write Protect Register Group 0 */
#  define PWM_WPCR_WPRG1           (1 << 3)  /* Bit 3:  Write Protect Register Group 1 */
#  define PWM_WPCR_WPRG2           (1 << 4)  /* Bit 4:  Write Protect Register Group 2 */
#  define PWM_WPCR_WPRG3           (1 << 5)  /* Bit 5:  Write Protect Register Group 3 */
#  define PWM_WPCR_WPRG4           (1 << 6)  /* Bit 6:  Write Protect Register Group 4 */
#  define PWM_WPCR_WPRG5           (1 << 7)  /* Bit 7:  Write Protect Register Group 5 */
#define PWM_WPCR_WPKEY_SHIFT       (8)       /* Bits 8-31: Write Protect Key */
#define PWM_WPCR_WPKEY_MASK        (0x00ffffff << PWM_WPCR_WPKEY_SHIFT)
#  define PWM_WPCR_WPKEY           (0x0050574d << PWM_WPCR_WPKEY_SHIFT) /* "PWM" in ASCII) */

/* PWM Write Protect Status Register */

#define PWM_WPSR_WPSWS(n)          (1 << (n))      /* Bits 0-5: Write Protect SW Status, n=0..5 */
#  define PWM_WPSR_WPSWS0          (1 << 0)  /* Bit 0:  Write Protect SW Status 0 */
#  define PWM_WPSR_WPSWS1          (1 << 1)  /* Bit 1:  Write Protect SW Status 1 */
#  define PWM_WPSR_WPSWS2          (1 << 2)  /* Bit 2:  Write Protect SW Status 2 */
#  define PWM_WPSR_WPSWS3          (1 << 3)  /* Bit 3:  Write Protect SW Status 3 */
#  define PWM_WPSR_WPSWS4          (1 << 4)  /* Bit 4:  Write Protect SW Status 4 */
#  define PWM_WPSR_WPSWS5          (1 << 5)  /* Bit 5:  Write Protect SW Status 5 */
#define PWM_WPSR_WPVS              (1 << 7)  /* Bit 7:  Write Protect Violation Status */
#define PWM_WPSR_WPHWS(n)          (1 << ((n)+8)) /* Bits 8-13: Write Protect HW Status, n=0..5 */
#  define PWM_WPSR_WPHWS0          (1 << 8)  /* Bit 8:  Write Protect HW Status 0 */
#  define PWM_WPSR_WPHWS1          (1 << 9)  /* Bit 9:  Write Protect HW Status 1 */
#  define PWM_WPSR_WPHWS2          (1 << 10) /* Bit 10: Write Protect HW Status 2 */
#  define PWM_WPSR_WPHWS3          (1 << 11) /* Bit 11: Write Protect HW Status 3 */
#  define PWM_WPSR_WPHWS4          (1 << 12) /* Bit 12: Write Protect HW Status 4 */
#  define PWM_WPSR_WPHWS5          (1 << 13) /* Bit 13: Write Protect HW Status 5 */
#define PWM_WPSR_WPVSRC_SHIFT      (16)      /* Bits 16-31: Write Protect Violation Source */
#define PWM_WPSR_WPVSRC_MASK       (0xffff << PWM_WPSR_WPVSRC_SHIFT)

/* PWM Comparison n Value Register, n=0..7 */

#define PWM_CMPV_CV_SHIFT          (0)       /* Bits 0-23: Comparison Value */
#define PWM_CMPV_CV_MASK           (0xffffff << PWM_CMPV_CV_SHIFT)
#  define PWM_CMPV_CV(n)           ((uint32_t)(n) << PWM_CMPV_CV_SHIFT)
#define PWM_CMPV_CVM               (1 << 24) /* Bits 24: Comparison Value Mode */

/* PWM Comparison n Value Update Register, n=0..7 */

#define PWM_CMPVUPD_CVUPD_SHIFT    (0)       /* Bits 0-24: Comparison Value Update */
#define PWM_CMPVUPD_CVUPD_MASK     (0xffffff << PWM_CMPVUPD_CVUPD_SHIFT)
#  define PWM_CMPVUPD_CVUPD(n)     ((uint32_t)(n) << PWM_CMPVUPD_CVUPD_SHIFT)
#define PWM_CMPVUPD_CVMUPD         (1 << 24) /* Bits 24: Comparison Value Mode Update */

/* PWM Comparison n Mode Register, n=0..7 */

#define PWM_CMPM_CEN               (1 << 0)) /* Bit 0: Comparison Enable */
#define PWM_CMPM_CTR_SHIFT         (4)       /* Bits 4-7: Comparison Trigger */
#define PWM_CMPM_CTR_MASK          (15 << PWM_CMPM_CTR_SHIFT)
#  define PWM_CMPM_CTR(n)          ((uint32_t)(n) << PWM_CMPM_CTR_SHIFT)
#define PWM_CMPM_CPR_SHIFT         (8)       /* Bits 8-11: Comparison Period */
#define PWM_CMPM_CPR_MASK          (15 << PWM_CMPM_CPR_SHIFT)
#  define PWM_CMPM_CPR(n)          ((uint32_t)(n) << PWM_CMPM_CPR_SHIFT)
#define PWM_CMPM_CPRCNT_SHIFT      (12)      /* Bits 12-15: Comparison Period Counter */
#define PWM_CMPM_CPRCNT_MASK       (15 << PWM_CMPM_CPRCNT_SHIFT)
#  define PWM_CMPM_CPRCNT(n)       ((uint32_t)(n) << PWM_CMPM_CPRCNT_SHIFT)
#define PWM_CMPM_CUPR_SHIFT        (16)      /* Bits 16-19: Comparison Update Period */
#define PWM_CMPM_CUPR_MASK         (15 << PWM_CMPM_CUPR_SHIFT)
#  define PWM_CMPM_CUPR(n)         ((uint32_t)(n) << PWM_CMPM_CUPR_SHIFT)
#define PWM_CMPM_CUPRCNT_SHIFT     (20)      /* Bits 20-23: Comparison Update Period Counter */
#define PWM_CMPM_CUPRCNT_MASK      (15 << PWM_CMPM_CUPRCNT_SHIFT)
#  define PWM_CMPM_CUPRCNT(n)      ((uint32_t)(n) << PWM_CMPM_CUPRCNT_SHIFT)

/* PWM Comparison n Mode Update Register, n=0..7 */

#define PWM_CMPMUPD_CENUPD         (1 << 0)  /* Bit 0:  Comparison Enable Update */
#define PWM_CMPMUPD_CTRUPD_SHIFT   (4)       /* Bits 4-7: Comparison Trigger Update */
#define PWM_CMPMUPD_CTRUPD_MASK    (15 << PWM_CMPMUPD_CTRUPD_SHIFT)
#  define PWM_CMPMUPD_CTRUPD(n)    ((uint32_t)(n) << PWM_CMPMUPD_CTRUPD_SHIFT)
#define PWM_CMPMUPD_CPRUPD_SHIFT   (8)       /* Bits 8-11: Comparison Period Up */
#define PWM_CMPMUPD_CPRUPD_MASK    (15 << PWM_CMPMUPD_CPRUPD_SHIFT)
#  define PWM_CMPMUPD_CPRUPD(n)    ((uint32_t)(n) << PWM_CMPMUPD_CPRUPD_SHIFT)
#define PWM_CMPMUPD_CUPRUPD_SHIFT  (16)      /* Bits 16-19: Comparison xpdate Period Update */
#define PWM_CMPMUPD_CUPRUPD_MASK   (15 << PWM_CMPMUPD_CUPRUPD_SHIFT)
#  define PWM_CMPMUPD_CUPRUPD(n)   ((uint32_t)(n) << PWM_CMPMUPD_CUPRUPD_SHIFT)

/* PWM Channel Mode Register */

#define PWM_CMR_CPRE_SHIFT         (0)       /* Bits 0-3: Channel Pre-scaler */
#define PWM_CMR_CPRE_MASK          (15 << PWM_CMR_CPRE_SHIFT)
#  define PWM_CMR_CPRE_MCKDIV(n)   ((uint32_t)(n) << PWM_CMR_CPRE_SHIFT)  /* Master clock */
#  define PWM_CMR_CPRE_MCKDIV1     (0 << PWM_CMR_CPRE_SHIFT)  /* Master clock/2 */
#  define PWM_CMR_CPRE_MCKDIV2     (1 << PWM_CMR_CPRE_SHIFT)  /* Master clock/2 */
#  define PWM_CMR_CPRE_MCKDIV4     (2 << PWM_CMR_CPRE_SHIFT)  /* Master clock/4 */
#  define PWM_CMR_CPRE_MCKDIV8     (3 << PWM_CMR_CPRE_SHIFT)  /* Master clock/8 */
#  define PWM_CMR_CPRE_MCKDIV16    (4 << PWM_CMR_CPRE_SHIFT)  /* Master clock/16 */
#  define PWM_CMR_CPRE_MCKDIV32    (5 << PWM_CMR_CPRE_SHIFT)  /* Master clock/32 */
#  define PWM_CMR_CPRE_MCKDIV64    (6 << PWM_CMR_CPRE_SHIFT)  /* Master clock/64 */
#  define PWM_CMR_CPRE_MCKDIV128   (7 << PWM_CMR_CPRE_SHIFT)  /* Master clock/128 */
#  define PWM_CMR_CPRE_MCKDIV256   (8 << PWM_CMR_CPRE_SHIFT)  /* Master clock/256 */
#  define PWM_CMR_CPRE_MCKDIV512   (9 << PWM_CMR_CPRE_SHIFT)  /* Master clock/512 */
#  define PWM_CMR_CPRE_MCKDIV1024  (10 << PWM_CMR_CPRE_SHIFT) /* Master clock/1024 */
#  define PWM_CMR_CPRE_CLKA        (11 << PWM_CMR_CPRE_SHIFT) /* Clock A */
#  define PWM_CMR_CPRE_CLKB        (12 << PWM_CMR_CPRE_SHIFT) /* Clock B */
#define PWM_CMR_CALG               (1 << 8)  /* Bit 8:  Channel Alignment */
#define PWM_CMR_CPOL               (1 << 9)  /* Bit 9:  Channel Polarity */
#define PWM_CMR_CES                (1 << 10) /* Bit 10: Counter Event Selection */

#ifdef ATSAMA5D4
#  define PWM_CMR_UPDS             (1 << 11) /* Bit 10: Update Selection */
#endif

#define PWM_CMR_DTE                (1 << 16) /* Bit 16: Dead-Time Generator Enable */
#define PWM_CMR_DTHI               (1 << 17) /* Bit 17: Dead-Time PWMH Output Inverted */
#define PWM_CMR_DTLI               (1 << 18) /* Bit 18: Dead-Time PWML Output Inverted */

/* PWM Channel Duty Cycle Register */

#define PWM_CDTY_MASK              (0x00ffffff) /* Bits 0-23: Channel Duty-Cycle */

/* PWM Channel Duty Cycle Update Register */

#define PWM_CDTYUPD_MASK           (0x00ffffff) /* Bits 0-23: Channel Duty-Cycle Update */

/* PWM Channel Period Register */

#define PWM_CPRD_MASK              (0x00ffffff) /* Bits 0-23: Channel Period */

/* PWM Channel Period Update Register */

#define PWM_CPRDUPD_MASK           (0x00ffffff) /* Bits 0-23: Channel Period Update */

/* PWM Channel Counter Register */

#define PWM_CCNT_MASK              (0x00ffffff) /* Bits 0-23:Channel Counter */

/* PWM Channel Dead Time Register */

#define PWM_DT_H_SHIFT             (0)       /* Bits 0-15: Dead-Time PWMH Output */
#define PWM_DT_H_MASK              (0xffff << PWM_DT_H_SHIFT)
#  define PWM_DT_H(n)              ((uint32_t)(n) << PWM_DT_H_SHIFT)
#define PWM_DT_L_SHIFT             (16)      /* Bits 16-31: Dead-Time PWML Output */
#define PWM_DT_L_MASK              (0xffff << PWM_DT_L_SHIFT)
#  define PWM_DT_L(n)              ((uint32_t)(n) << PWM_DT_L_SHIFT

/* PWM Channel Dead Time Update Register */

#define PWM_DTUPD_H_SHIFT          (0)     /* Bits 0-15: Dead-Time Value Update for PWMH Output */
#define PWM_DTUPD_H_MASK           (0xffff << PWM_DTUPD_H_SHIFT)
#  define PWM_DTUPD_H(n)           ((uint32_t)(n) << PWM_DTUPD_H_SHIFT)
#define PWM_DTUPD_L_SHIFT          (16)     /* Bits 16-31: Dead-Time Value Update for PWML Output */
#define PWM_DTUPD_L_MASK           (0xffff << PWM_DTUPD_L_SHIFT)
#  define PWM_DTUPD_L(n)           ((uint32_t)(n) << PWM_DTUPD_L_SHIFT)

#ifdef ATSAMA5D4
/* PWM Channel Mode Update Register */

#  define PWM_CMUPD_CPOLUP         (1 << 9)  /* Bit 9:  Channel Polarity Update */
#  define PWM_CMUPD_CPOLINVUP      (1 << 13) /* Bit 13: Channel Polarity Inversion Update */
#endif

#endif /* __ARCH_ARM_SRC_SAMA5_CHIP_SAM_PWM_H */
