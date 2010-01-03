/****************************************************************************************
 * arch/arm/src/sam3u/sam3u_pwm.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 ****************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAM3U_SAM3U_PWM_H
#define __ARCH_ARM_SRC_SAM3U_SAM3U_PWM_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "sam3u_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* PWM register offsets *****************************************************************/

#define SAM3U_PWM_CLK_OFFSET           0x000 /* PWM Clock Register */
#define SAM3U_PWM_ENA_OFFSET           0x004 /* PWM Enable Register */
#define SAM3U_PWM_DIS_OFFSET           0x008 /* PWM Disable Register */
#define SAM3U_PWM_SRCSTOMUL_OFFSET     0x00c /* PWM Status Register */
#define SAM3U_PWM_IER1_OFFSET          0x010 /* PWM Interrupt Enable Register 1 */
#define SAM3U_PWM_IDR1_OFFSET          0x014 /* PWM Interrupt Disable Register 1 */
#define SAM3U_PWM_IMR1CSTOMUL_OFFSET   0x018 /* PWM Interrupt Mask Register 1 */
#define SAM3U_PWM_ISR1CSTOMUL_OFFSET   0x01c /* PWM Interrupt Status Register 1 */
#define SAM3U_PWM_SCM_OFFSET           0x020 /* PWM Sync Channels Mode Register */
                                             /* 0x024: Reserved */
#define SAM3U_PWM_SCUC_OFFSET          0x028 /* PWM Sync Channels Update Control Register */
#define SAM3U_PWM_SCUP_OFFSET          0x02c /* PWM Sync Channels Update Period Register */
#define SAM3U_PWM_SCUPUPD_OFFSET       0x030 /* PWM Sync Channels Update Period Update Register */
#define SAM3U_PWM_IER2_OFFSET          0x034 /* PWM Interrupt Enable Register 2 */
#define SAM3U_PWM_IDR2_OFFSET          0x038 /* PWM Interrupt Disable Register 2 */
#define SAM3U_PWM_IMR2CSTOMUL_OFFSET   0x03c /* PWM Interrupt Mask Register 2 */
#define SAM3U_PWM_ISR2CSTOMUL_OFFSET   0x040 /* PWM Interrupt Status Register 2 */
#define SAM3U_PWM_OOV_OFFSET           0x044 /* PWM Output Override Value Register */
#define SAM3U_PWM_OS_OFFSET            0x048 /* PWM Output Selection Register */
#define SAM3U_PWM_OSS_OFFSET           0x04c /* PWM Output Selection Set Register */
#define SAM3U_PWM_OSC_OFFSET           0x050 /* PWM Output Selection Clear Register */
#define SAM3U_PWM_OSSUPD_OFFSET        0x054 /* PWM Output Selection Set Update Register */
#define SAM3U_PWM_OSCUPD_OFFSET        0x058 /* PWM Output Selection Clear Update Register */
#define SAM3U_PWM_FMR_OFFSET           0x05c /* PWM Fault Mode Register */
#define SAM3U_PWM_FSRCSTOMUL_OFFSET    0x060 /* PWM Fault Status Register */
#define SAM3U_PWM_FCR_OFFSET           0x064 /* PWM Fault Clear Register */
#define SAM3U_PWM_FPV_OFFSET           0x068 /* PWM Fault Protection Value Register */
#define SAM3U_PWM_FPE_OFFSET           0x06c /* PWM Fault Protection Enable Register */
                                             /* 0x070-0x078: Reserved */
#define SAM3U_PWM_EL0MR_OFFSET         0x07c /* PWM Event Line 0 Mode Register */
#define SAM3U_PWM_EL1MR_OFFSET         0x080 /* PWM Event Line 1 Mode Register */
                                             /* 0x084-0x0ac: Reserved */
                                             /* 0x0b4-0x0e0: Reserved */
#define SAM3U_PWM_WPCR_OFFSET          0x0e4 /* PWM Write Protect Control Register */
#define SAM3U_PWM_WPSRCSTOMUL_OFFSET   0x0e8 /* PWM Write Protect Status Register */
                                             /* 0x100-0x128: Reserved for PDC registers */
                                             /* 0x12c: Reserved */
/* PWM Comparison Registers */

#define SAM3U_PWMCMP_OFFSET(n)        (0x130+((n)<<4))
#define SAM3U_PWMCMP_V_OFFSET          0x00  /* PWM Comparison Value Register */
#define SAM3U_PWMCMP_VUPD_OFFSET       0x04  /* PWM Comparison Value Update Register */
#define SAM3U_PWMCMP_M_OFFSET          0x08  /* PWM Comparison Mode Register */
#define SAM3U_PWMCMP_MUPD_OFFSET       0x0c  /* PWM Comparison Mode Update Register */

#define SAM3U_PWMCMP0_V_OFFSET         0x130 /* PWM Comparison 0 Value Register */
#define SAM3U_PWMCMP0_VUPD_OFFSET      0x134 /* PWM Comparison 0 Value Update Register */
#define SAM3U_PWMCMP0_M_OFFSET         0x138 /* PWM Comparison 0 Mode Register */
#define SAM3U_PWMCMP0_MUPD_OFFSET      0x13c /* PWM Comparison 0 Mode Update Register */

#define SAM3U_PWMCMP1_V_OFFSET         0x140 /* PWM Comparison 1 Value Register */
#define SAM3U_PWMCMP1_VUPD_OFFSET      0x144 /* PWM Comparison 1 Value Update Register */
#define SAM3U_PWMCMP1_M_OFFSET         0x148 /* PWM Comparison 1 Mode Register */
#define SAM3U_PWMCMP1_MUPD_OFFSET      0x14c /* PWM Comparison 1 Mode Update Register */

#define SAM3U_PWMCMP2_V_OFFSET         0x150 /* PWM Comparison 2 Value Register */
#define SAM3U_PWMCMP2_VUPD_OFFSET      0x154 /* PWM Comparison 2 Value Update Register */
#define SAM3U_PWMCMP2_M_OFFSET         0x158 /* PWM Comparison 2 Mode Register */
#define SAM3U_PWMCMP2_MUPD_OFFSET      0x15c /* PWM Comparison 2 Mode Update Register */

#define SAM3U_PWMCMP3_V_OFFSET         0x160 /* PWM Comparison 3 Value Register */
#define SAM3U_PWMCMP3_VUPD_OFFSET      0x164 /* PWM Comparison 3 Value Update Register */
#define SAM3U_PWMCMP3_M_OFFSET         0x168 /* PWM Comparison 3 Mode Register */
#define SAM3U_PWMCMP3_MUPD_OFFSET      0x16c /* PWM Comparison 3 Mode Update Register */

#define SAM3U_PWMCMP4_V_OFFSET         0x170 /* PWM Comparison 4 Value Register */
#define SAM3U_PWMCMP4_VUPD_OFFSET      0x174 /* PWM Comparison 4 Value Update Register */
#define SAM3U_PWMCMP4_M_OFFSET         0x178 /* PWM Comparison 4 Mode Register */
#define SAM3U_PWMCMP4_MUPD_OFFSET      0x17c /* PWM Comparison 4 Mode Update Register */

#define SAM3U_PWMCMP5_V_OFFSET         0x180 /* PWM Comparison 5 Value Register */
#define SAM3U_PWMCMP5_VUPD_OFFSET      0x184 /* PWM Comparison 5 Value Update Register */
#define SAM3U_PWMCMP5_M_OFFSET         0x188 /* PWM Comparison 5 Mode Register */
#define SAM3U_PWMCMP5_MUPD_OFFSET      0x18c /* PWM Comparison 5 Mode Update Register */

#define SAM3U_PWMCMP6_V_OFFSET         0x190 /* PWM Comparison 6 Value Register */
#define SAM3U_PWMCMP6_VUPD_OFFSET      0x194 /* PWM Comparison 6 Value Update Register */
#define SAM3U_PWMCMP6_M_OFFSET         0x198 /* PWM Comparison 6 Mode Register */
#define SAM3U_PWMCMP6_MUPD_OFFSET      0x19c /* PWM Comparison 6 Mode Update Register */

#define SAM3U_PWMCMP7_V_OFFSET         0x1a0 /* PWM Comparison 7 Value Register */
#define SAM3U_PWMCMP7_VUPD_OFFSET      0x1a4 /* PWM Comparison 7 Value Update Register */
#define SAM3U_PWMCMP7_M_OFFSET         0x1a8 /* PWM Comparison 7 Mode Register */
#define SAM3U_PWMCMP7_MUPD_OFFSET      0x1ac /* PWM Comparison 7 Mode Update Register */
                                             /* 0x1b0-0x1fc: Reserved */

/* PWM Channel Registers */

#define SAM3U_PWMCH_OFFSET(n)          (0x200+((n)<< 5))
#define SAM3U_PWMCH_MR_OFFSET          0x00  /* PWM Channel Mode Register */
#define SAM3U_PWMCH_DTY_OFFSET         0x04  /* PWM Channel Duty Cycle Register */
#define SAM3U_PWMCH_DTYUPD_OFFSET      0x08  /* PWM Channel Duty Cycle Update Register */
#define SAM3U_PWMCH_PRD_OFFSET         0x0c  /* PWM Channel Period Register */
#define SAM3U_PWMCH_PRDUPD_OFFSET      0x10  /* PWM Channel Period Update Register */
#define SAM3U_PWMCH_CNTCSTOMUL_OFFSET  0x14  /* PWM Channel Counter Register */
#define SAM3U_PWMCH_DT_OFFSET          0x18  /* PWM Channel Dead Time Register */
#define SAM3U_PWMCH_DTUPD_OFFSET       0x1c  /* PWM Channel Dead Time Update Register */

#define SAM3U_PWMCH0_MR_OFFSET         0x200 /* PWM Channel 0 Mode Register */
#define SAM3U_PWMCH0_DTY_OFFSET        0x204 /* PWM Channel 0 Duty Cycle Register */
#define SAM3U_PWMCH0_DTYUPD_OFFSET     0x208 /* PWM Channel 0 Duty Cycle Update Register */
#define SAM3U_PWMCH0_PRD_OFFSET        0x20c /* PWM Channel 0 Period Register */
#define SAM3U_PWMCH0_PRDUPD_OFFSET     0x210 /* PWM Channel 0 Period Update Register */
#define SAM3U_PWMCH0_CNTCSTOMUL_OFFSET 0x214 /* PWM Channel 0 Counter Register */
#define SAM3U_PWMCH0_DT_OFFSET         0x218 /* PWM Channel 0 Dead Time Register */
#define SAM3U_PWMCH0_DTUPD_OFFSET      0x21c /* PWM Channel 0 Dead Time Update Register */

#define SAM3U_PWMCH1_MR_OFFSET         0x220 /* PWM Channel 1 Mode Register */
#define SAM3U_PWMCH1_DTY_OFFSET        0x224 /* PWM Channel 1 Duty Cycle Register */
#define SAM3U_PWMCH1_DTYUPD_OFFSET     0x228 /* PWM Channel 1 Duty Cycle Update Register */
#define SAM3U_PWMCH1_PRD_OFFSET        0x22c /* PWM Channel 1 Period Register */
#define SAM3U_PWMCH1_PRDUPD_OFFSET     0x230 /* PWM Channel 1 Period Update Register */
#define SAM3U_PWMCH1_CNTCSTOMUL_OFFSET 0x234 /* PWM Channel 1 Counter Register */
#define SAM3U_PWMCH1_DT_OFFSET         0x238 /* PWM Channel 1 Dead Time Register */
#define SAM3U_PWMCH1_DTUPD_OFFSET      0x23c /* PWM Channel 1 Dead Time Update Register */

#define SAM3U_PWMCH2_MR_OFFSET         0x240 /* PWM Channel 2 Mode Register */
#define SAM3U_PWMCH2_DTY_OFFSET        0x244 /* PWM Channel 2 Duty Cycle Register */
#define SAM3U_PWMCH2_DTYUPD_OFFSET     0x248 /* PWM Channel 2 Duty Cycle Update Register */
#define SAM3U_PWMCH2_PRD_OFFSET        0x24c /* PWM Channel 2 Period Register */
#define SAM3U_PWMCH2_PRDUPD_OFFSET     0x250 /* PWM Channel 2 Period Update Register */
#define SAM3U_PWMCH2_CNTCSTOMUL_OFFSET 0x254 /* PWM Channel 2 Counter Register */
#define SAM3U_PWMCH2_DT_OFFSET         0x258 /* PWM Channel 2 Dead Time Register */
#define SAM3U_PWMCH2_DTUPD_OFFSET      0x25c /* PWM Channel 2 Dead Time Update Register */

#define SAM3U_PWMCH3_MR_OFFSET         0x260 /* PWM Channel 3 Mode Register */
#define SAM3U_PWMCH3_DTY_OFFSET        0x264 /* PWM Channel 3 Duty Cycle Register */
#define SAM3U_PWMCH3_DTYUPD_OFFSET     0x268 /* PWM Channel 3 Duty Cycle Update Register */
#define SAM3U_PWMCH3_PRD_OFFSET        0x26c /* PWM Channel 3 Period Register */
#define SAM3U_PWMCH3_PRDUPD_OFFSET     0x270 /* PWM Channel 3 Period Update Register */
#define SAM3U_PWMCH3_CNTCSTOMUL_OFFSET 0x274 /* PWM Channel 3 Counter Register */
#define SAM3U_PWMCH3_DT_OFFSET         0x278 /* PWM Channel 3 Dead Time Register */
#define SAM3U_PWMCH3_DTUPD_OFFSET      0x27c /* PWM Channel 3 Dead Time Update Register */

/* PWM register adresses ****************************************************************/

#define SAM3U_PWM_CLK                  (SAM3U_PWM_BASE+SAM3U_PWM_CLK_OFFSET)
#define SAM3U_PWM_ENA                  (SAM3U_PWM_BASE+SAM3U_PWM_ENA_OFFSET)
#define SAM3U_PWM_DIS                  (SAM3U_PWM_BASE+SAM3U_PWM_DIS_OFFSET)
#define SAM3U_PWM_SRCSTOMUL            (SAM3U_PWM_BASE+SAM3U_PWM_SRCSTOMUL_OFFSET)
#define SAM3U_PWM_IER1                 (SAM3U_PWM_BASE+SAM3U_PWM_IER1_OFFSET)
#define SAM3U_PWM_IDR1                 (SAM3U_PWM_BASE+SAM3U_PWM_IDR1_OFFSET)
#define SAM3U_PWM_IMR1CSTOMUL          (SAM3U_PWM_BASE+SAM3U_PWM_IMR1CSTOMUL_OFFSET)
#define SAM3U_PWM_ISR1CSTOMUL          (SAM3U_PWM_BASE+SAM3U_PWM_ISR1CSTOMUL_OFFSET)
#define SAM3U_PWM_SCM                  (SAM3U_PWM_BASE+SAM3U_PWM_SCM_OFFSET)
#define SAM3U_PWM_SCUC                 (SAM3U_PWM_BASE+SAM3U_PWM_SCUC_OFFSET)
#define SAM3U_PWM_SCUP                 (SAM3U_PWM_BASE+SAM3U_PWM_SCUP_OFFSET)
#define SAM3U_PWM_SCUPUPD              (SAM3U_PWM_BASE+SAM3U_PWM_SCUPUPD_OFFSET)
#define SAM3U_PWM_IER2                 (SAM3U_PWM_BASE+SAM3U_PWM_IER2_OFFSET)
#define SAM3U_PWM_IDR2                 (SAM3U_PWM_BASE+SAM3U_PWM_IDR2_OFFSET)
#define SAM3U_PWM_IMR2CSTOMUL          (SAM3U_PWM_BASE+SAM3U_PWM_IMR2CSTOMUL_OFFSET)
#define SAM3U_PWM_ISR2CSTOMUL          (SAM3U_PWM_BASE+SAM3U_PWM_ISR2CSTOMUL_OFFSET)
#define SAM3U_PWM_OOV                  (SAM3U_PWM_BASE+SAM3U_PWM_OOV_OFFSET)
#define SAM3U_PWM_OS                   (SAM3U_PWM_BASE+SAM3U_PWM_OS_OFFSET)
#define SAM3U_PWM_OSS                  (SAM3U_PWM_BASE+SAM3U_PWM_OSS_OFFSET)
#define SAM3U_PWM_OSC                  (SAM3U_PWM_BASE+SAM3U_PWM_OSC_OFFSET)
#define SAM3U_PWM_OSSUPD               (SAM3U_PWM_BASE+SAM3U_PWM_OSSUPD_OFFSET)
#define SAM3U_PWM_OSCUPD               (SAM3U_PWM_BASE+SAM3U_PWM_OSCUPD_OFFSET)
#define SAM3U_PWM_FMR                  (SAM3U_PWM_BASE+SAM3U_PWM_FMR_OFFSET)
#define SAM3U_PWM_FSRCSTOMUL           (SAM3U_PWM_BASE+SAM3U_PWM_FSRCSTOMUL_OFFSET)
#define SAM3U_PWM_FCR                  (SAM3U_PWM_BASE+SAM3U_PWM_FCR_OFFSET)
#define SAM3U_PWM_FPV                  (SAM3U_PWM_BASE+SAM3U_PWM_FPV_OFFSET)
#define SAM3U_PWM_FPE                  (SAM3U_PWM_BASE+SAM3U_PWM_FPE_OFFSET)
#define SAM3U_PWM_EL0MR                (SAM3U_PWM_BASE+SAM3U_PWM_EL0MR_OFFSET)
#define SAM3U_PWM_EL1MR                (SAM3U_PWM_BASE+SAM3U_PWM_EL1MR_OFFSET)
#define SAM3U_PWM_WPCR                 (SAM3U_PWM_BASE+SAM3U_PWM_WPCR_OFFSET)
#define SAM3U_PWM_WPSRCSTOMUL          (SAM3U_PWM_BASE+SAM3U_PWM_WPSRCSTOMUL_OFFSET)

/* PWM Comparison Registers */

#define SAM3U_PWCMP_BASE(n)            (SAM3U_PWM_BASE+SAM3U_PWCMP_OFFSET(n))
#define SAM3U_PWMCMP0_BASE             (SAM3U_PWM_BASE+0x0130)
#define SAM3U_PWMCMP1_BASE             (SAM3U_PWM_BASE+0x0140)
#define SAM3U_PWMCMP2_BASE             (SAM3U_PWM_BASE+0x0150)
#define SAM3U_PWMCMP3_BASE             (SAM3U_PWM_BASE+0x0160)
#define SAM3U_PWMCMP4_BASE             (SAM3U_PWM_BASE+0x0170)
#define SAM3U_PWMCMP5_BASE             (SAM3U_PWM_BASE+0x0180)
#define SAM3U_PWMCMP6_BASE             (SAM3U_PWM_BASE+0x0190)
#define SAM3U_PWMCMP7_BASE             (SAM3U_PWM_BASE+0x01a0)

#define SAM3U_PWMCMP0_V                (SAM3U_PWMCMP0_BASE+SAM3U_PWMCMP_V_OFFSET)
#define SAM3U_PWMCMP0_VUPD             (SAM3U_PWMCMP0_BASE+SAM3U_PWMCMP_VUPD_OFFSET)
#define SAM3U_PWMCMP0_M                (SAM3U_PWMCMP0_BASE+SAM3U_PWMCMP_M_OFFSET)
#define SAM3U_PWMCMP0_MUPD             (SAM3U_PWMCMP0_BASE+SAM3U_PWMCMP_MUPD_OFFSET)

#define SAM3U_PWMCMP1_V                (SAM3U_PWMCMP1_BASE+SAM3U_PWMCMP_V_OFFSET)
#define SAM3U_PWMCMP1_VUPD             (SAM3U_PWMCMP1_BASE+SAM3U_PWMCMP_VUPD_OFFSET)
#define SAM3U_PWMCMP1_M                (SAM3U_PWMCMP1_BASE+SAM3U_PWMCMP_M_OFFSET)
#define SAM3U_PWMCMP1_MUPD             (SAM3U_PWMCMP1_BASE+SAM3U_PWMCMP_MUPD_OFFSET)

#define SAM3U_PWMCMP2_V                (SAM3U_PWMCMP2_BASE+SAM3U_PWMCMP_V_OFFSET)
#define SAM3U_PWMCMP2_VUPD             (SAM3U_PWMCMP2_BASE+SAM3U_PWMCMP_VUPD_OFFSET)
#define SAM3U_PWMCMP2_M                (SAM3U_PWMCMP2_BASE+SAM3U_PWMCMP_M_OFFSET)
#define SAM3U_PWMCMP2_MUPD             (SAM3U_PWMCMP2_BASE+SAM3U_PWMCMP_MUPD_OFFSET)

#define SAM3U_PWMCMP3_V                (SAM3U_PWMCMP3_BASE+SAM3U_PWMCMP_V_OFFSET)
#define SAM3U_PWMCMP3_VUPD             (SAM3U_PWMCMP3_BASE+SAM3U_PWMCMP_VUPD_OFFSET)
#define SAM3U_PWMCMP3_M                (SAM3U_PWMCMP3_BASE+SAM3U_PWMCMP_M_OFFSET)
#define SAM3U_PWMCMP3_MUPD             (SAM3U_PWMCMP3_BASE+SAM3U_PWMCMP_MUPD_OFFSET)

#define SAM3U_PWMCMP4_V                (SAM3U_PWMCMP4_BASE+SAM3U_PWMCMP_V_OFFSET)
#define SAM3U_PWMCMP4_VUPD             (SAM3U_PWMCMP4_BASE+SAM3U_PWMCMP_VUPD_OFFSET)
#define SAM3U_PWMCMP4_M                (SAM3U_PWMCMP4_BASE+SAM3U_PWMCMP_M_OFFSET)
#define SAM3U_PWMCMP4_MUPD             (SAM3U_PWMCMP4_BASE+SAM3U_PWMCMP_MUPD_OFFSET)

#define SAM3U_PWMCMP5_V                (SAM3U_PWMCMP5_BASE+SAM3U_PWMCMP_V_OFFSET)
#define SAM3U_PWMCMP5_VUPD             (SAM3U_PWMCMP5_BASE+SAM3U_PWMCMP_VUPD_OFFSET)
#define SAM3U_PWMCMP5_M                (SAM3U_PWMCMP5_BASE+SAM3U_PWMCMP_M_OFFSET)
#define SAM3U_PWMCMP5_MUPD             (SAM3U_PWMCMP5_BASE+SAM3U_PWMCMP_MUPD_OFFSET)

#define SAM3U_PWMCMP6_V                (SAM3U_PWMCMP6_BASE+SAM3U_PWMCMP_V_OFFSET)
#define SAM3U_PWMCMP6_VUPD             (SAM3U_PWMCMP6_BASE+SAM3U_PWMCMP_VUPD_OFFSET)
#define SAM3U_PWMCMP6_M                (SAM3U_PWMCMP6_BASE+SAM3U_PWMCMP_M_OFFSET)
#define SAM3U_PWMCMP6_MUPD             (SAM3U_PWMCMP6_BASE+SAM3U_PWMCMP_MUPD_OFFSET)

#define SAM3U_PWMCMP7_V                (SAM3U_PWMCMP7_BASE+SAM3U_PWMCMP_V_OFFSET)
#define SAM3U_PWMCMP7_VUPD             (SAM3U_PWMCMP7_BASE+SAM3U_PWMCMP_VUPD_OFFSET)
#define SAM3U_PWMCMP7_M                (SAM3U_PWMCMP7_BASE+SAM3U_PWMCMP_M_OFFSET)
#define SAM3U_PWMCMP7_MUPD             (SAM3U_PWMCMP7_BASE+SAM3U_PWMCMP_MUPD_OFFSET)

/* PWM Channel Registers */

#define SAM3U_PWCH_BASE(n)             (SAM3U_PWM_BASE+SAM3U_PWCH_OFFSET(n))
#define SAM3U_PWMCH0_BASE              (SAM3U_PWM_BASE+0x0200)
#define SAM3U_PWMCH1_BASE              (SAM3U_PWM_BASE+0x0220)
#define SAM3U_PWMCH2_BASE              (SAM3U_PWM_BASE+0x0240)
#define SAM3U_PWMCH3_BASE              (SAM3U_PWM_BASE+0x0260)

#define SAM3U_PWMCH0_MR                (SAM3U_PWMCH0_BASE+SAM3U_PWMCH_MR_OFFSET)
#define SAM3U_PWMCH0_DTY               (SAM3U_PWMCH0_BASE+SAM3U_PWMCH_DTY_OFFSET)
#define SAM3U_PWMCH0_DTYUPD            (SAM3U_PWMCH0_BASE+SAM3U_PWMCH_DTYUPD_OFFSET)
#define SAM3U_PWMCH0_PRD               (SAM3U_PWMCH0_BASE+SAM3U_PWMCH_PRD_OFFSET)
#define SAM3U_PWMCH0_PRDUPD            (SAM3U_PWMCH0_BASE+SAM3U_PWMCH_PRDUPD_OFFSET)
#define SAM3U_PWMCH0_CNTCSTOMUL        (SAM3U_PWMCH0_BASE+SAM3U_PWMCH_CNTCSTOMUL_OFFSET)
#define SAM3U_PWMCH0_DT                (SAM3U_PWMCH0_BASE+SAM3U_PWMCH_DT_OFFSET)
#define SAM3U_PWMCH0_DTUPD             (SAM3U_PWMCH0_BASE+SAM3U_PWMCH_DTUPD_OFFSET)

#define SAM3U_PWMCH1_MR                (SAM3U_PWMCH1_BASE+SAM3U_PWMCH_MR_OFFSET)
#define SAM3U_PWMCH1_DTY               (SAM3U_PWMCH1_BASE+SAM3U_PWMCH_DTY_OFFSET)
#define SAM3U_PWMCH1_DTYUPD            (SAM3U_PWMCH1_BASE+SAM3U_PWMCH_DTYUPD_OFFSET)
#define SAM3U_PWMCH1_PRD               (SAM3U_PWMCH1_BASE+SAM3U_PWMCH_PRD_OFFSET)
#define SAM3U_PWMCH1_PRDUPD            (SAM3U_PWMCH1_BASE+SAM3U_PWMCH_PRDUPD_OFFSET)
#define SAM3U_PWMCH1_CNTCSTOMUL        (SAM3U_PWMCH1_BASE+SAM3U_PWMCH_CNTCSTOMUL_OFFSET)
#define SAM3U_PWMCH1_DT                (SAM3U_PWMCH1_BASE+SAM3U_PWMCH_DT_OFFSET)
#define SAM3U_PWMCH1_DTUPD             (SAM3U_PWMCH1_BASE+SAM3U_PWMCH_DTUPD_OFFSET)

#define SAM3U_PWMCH2_MR                (SAM3U_PWMCH2_BASE+SAM3U_PWMCH_MR_OFFSET)
#define SAM3U_PWMCH2_DTY               (SAM3U_PWMCH2_BASE+SAM3U_PWMCH_DTY_OFFSET)
#define SAM3U_PWMCH2_DTYUPD            (SAM3U_PWMCH2_BASE+SAM3U_PWMCH_DTYUPD_OFFSET)
#define SAM3U_PWMCH2_PRD               (SAM3U_PWMCH2_BASE+SAM3U_PWMCH_PRD_OFFSET)
#define SAM3U_PWMCH2_PRDUPD            (SAM3U_PWMCH2_BASE+SAM3U_PWMCH_PRDUPD_OFFSET)
#define SAM3U_PWMCH2_CNTCSTOMUL        (SAM3U_PWMCH2_BASE+SAM3U_PWMCH_CNTCSTOMUL_OFFSET)
#define SAM3U_PWMCH2_DT                (SAM3U_PWMCH2_BASE+SAM3U_PWMCH_DT_OFFSET)
#define SAM3U_PWMCH2_DTUPD             (SAM3U_PWMCH2_BASE+SAM3U_PWMCH_DTUPD_OFFSET)

#define SAM3U_PWMCH3_MR                (SAM3U_PWMCH3_BASE+SAM3U_PWMCH_MR_OFFSET)
#define SAM3U_PWMCH3_DTY               (SAM3U_PWMCH3_BASE+SAM3U_PWMCH_DTY_OFFSET)
#define SAM3U_PWMCH3_DTYUPD            (SAM3U_PWMCH3_BASE+SAM3U_PWMCH_DTYUPD_OFFSET)
#define SAM3U_PWMCH3_PRD               (SAM3U_PWMCH3_BASE+SAM3U_PWMCH_PRD_OFFSET)
#define SAM3U_PWMCH3_PRDUPD            (SAM3U_PWMCH3_BASE+SAM3U_PWMCH_PRDUPD_OFFSET)
#define SAM3U_PWMCH3_CNTCSTOMUL        (SAM3U_PWMCH3_BASE+SAM3U_PWMCH_CNTCSTOMUL_OFFSET)
#define SAM3U_PWMCH3_DT                (SAM3U_PWMCH3_BASE+SAM3U_PWMCH_DT_OFFSET)
#define SAM3U_PWMCH3_DTUPD             (SAM3U_PWMCH3_BASE+SAM3U_PWMCH_DTUPD_OFFSET)

/* PWM register bit definitions *********************************************************/

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM3U_SAM3U_PWM_H */
