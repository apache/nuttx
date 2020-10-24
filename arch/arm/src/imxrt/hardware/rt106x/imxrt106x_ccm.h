/************************************************************************************************************
 * arch/arm/src/imxrt/hardware/rt106x/imxrt_ccm.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors:  Janne Rosberg <janne@offcode.fi>
 *             David Sidrane <david_s5@nscdg.com>
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT106X_CCM_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT106X_CCM_H

/************************************************************************************************************
 * Included Files
 ************************************************************************************************************/

#include <nuttx/config.h>
#include "hardware/imxrt_memorymap.h"

/************************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************************/

/* Register offsets *****************************************************************************************/

#define IMXRT_CCM_CCR_OFFSET                     0x0000  /* CCM Control Register */
                                                         /* 0x0004  Reserved */
#define IMXRT_CCM_CSR_OFFSET                     0x0008  /* CCM Status Register */
#define IMXRT_CCM_CCSR_OFFSET                    0x000c  /* CCM Clock Switcher Register */
#define IMXRT_CCM_CACRR_OFFSET                   0x0010  /* CCM Arm Clock Root Register */
#define IMXRT_CCM_CBCDR_OFFSET                   0x0014  /* CCM Bus Clock Divider Register */
#define IMXRT_CCM_CBCMR_OFFSET                   0x0018  /* CCM Bus Clock Multiplexer Register */
#define IMXRT_CCM_CSCMR1_OFFSET                  0x001c  /* CCM Serial Clock Multiplexer Register 1 */
#define IMXRT_CCM_CSCMR2_OFFSET                  0x0020  /* CCM Serial Clock Multiplexer Register 2 */
#define IMXRT_CCM_CSCDR1_OFFSET                  0x0024  /* CCM Serial Clock Divider Register 1 */
#define IMXRT_CCM_CS1CDR_OFFSET                  0x0028  /* CCM Clock Divider Register */
#define IMXRT_CCM_CS2CDR_OFFSET                  0x002c  /* CCM Clock Divider Register */
#define IMXRT_CCM_CDCDR_OFFSET                   0x0030  /* CCM D1 Clock Divider Register */
                                                         /* 0x0034  Reserved */
#define IMXRT_CCM_CSCDR2_OFFSET                  0x0038  /* CCM Serial Clock Divider Register 2 */
#define IMXRT_CCM_CSCDR3_OFFSET                  0x003c  /* CCM Serial Clock Divider Register 3 */
                                                         /* 0x0040  Reserved */
                                                         /* 0x0044  Reserved */
#define IMXRT_CCM_CDHIPR_OFFSET                  0x0048  /* CCM Divider Handshake In-Process Register */
                                                         /* 0x004c  Reserved */
                                                         /* 0x0050  Reserved */
#define IMXRT_CCM_CLPCR_OFFSET                   0x0054  /* CCM Low Power Control Register */

#define IMXRT_CCM_CISR_OFFSET                    0x0058  /* CCM Interrupt Status Register */
#define IMXRT_CCM_CIMR_OFFSET                    0x005c  /* CCM Interrupt Mask Register */
#define IMXRT_CCM_CCOSR_OFFSET                   0x0060  /* CCM Clock Output Source Register */
#define IMXRT_CCM_CGPR_OFFSET                    0x0064  /* CCM General Purpose Register */
#define IMXRT_CCM_CCGR0_OFFSET                   0x0068  /* CCM Clock Gating Register 0 */
#define IMXRT_CCM_CCGR1_OFFSET                   0x006c  /* CCM Clock Gating Register 1 */
#define IMXRT_CCM_CCGR2_OFFSET                   0x0070  /* CCM Clock Gating Register 2 */
#define IMXRT_CCM_CCGR3_OFFSET                   0x0074  /* CCM Clock Gating Register 3 */
#define IMXRT_CCM_CCGR4_OFFSET                   0x0078  /* CCM Clock Gating Register 4 */
#define IMXRT_CCM_CCGR5_OFFSET                   0x007c  /* CCM Clock Gating Register 5 */
#define IMXRT_CCM_CCGR6_OFFSET                   0x0080  /* CCM Clock Gating Register 6 */
#define IMXRT_CCM_CCGR7_OFFSET                   0x0084  /* CCM Clock Gating Register 7 */
                                                         /* 0x0084  Reserved */
#define IMXRT_CCM_CMEOR_OFFSET                   0x0088  /* CCM Module Enable Override Register */

/* Analog */

#define IMXRT_CCM_ANALOG_PLL_ARM_OFFSET          0x0000  /* Analog ARM PLL control Register */
#define IMXRT_CCM_ANALOG_PLL_USB1_OFFSET         0x0010  /* Analog USB1 480MHz PLL Control Register */
#define IMXRT_CCM_ANALOG_PLL_USB2_OFFSET         0x0020  /* Analog USB2 480MHz PLL Control Register */
#define IMXRT_CCM_ANALOG_PLL_SYS_OFFSET          0x0030  /* Analog System PLL Control Register */
#define IMXRT_CCM_ANALOG_PLL_SYS_SS_OFFSET       0x0040  /* 528MHz System PLL Spread Spectrum Register */
#define IMXRT_CCM_ANALOG_PLL_SYS_NUM_OFFSET      0x0050  /* Numerator of 528MHz System PLL Fractional Loop Divider */
#define IMXRT_CCM_ANALOG_PLL_SYS_DENOM_OFFSET    0x0060  /* Denominator of 528MHz System PLL Fractional Loop Divider */
#define IMXRT_CCM_ANALOG_PLL_AUDIO_OFFSET        0x0070  /* Analog Audio PLL control Register */
#define IMXRT_CCM_ANALOG_PLL_AUDIO_NUM_OFFSET    0x0080  /* Numerator of Audio PLL Fractional Loop Divider */
#define IMXRT_CCM_ANALOG_PLL_AUDIO_DENOM_OFFSET  0x0090  /* Denominator of Audio PLL Fractional Loop Divider */
#define IMXRT_CCM_ANALOG_PLL_VIDEO_OFFSET        0x00a0  /* Analog Video PLL control Register */
#define IMXRT_CCM_ANALOG_PLL_VIDEO_NUM_OFFSET    0x00b0  /* Numerator of Video PLL Fractional Loop Divider */
#define IMXRT_CCM_ANALOG_PLL_VIDEO_DENOM_OFFSET  0x00c0  /* Denominator of Video PLL Fractional Loop Divider */
#define IMXRT_CCM_ANALOG_PLL_ENET_OFFSET         0x00e0  /* Analog ENET PLL Control Register */
#define IMXRT_CCM_ANALOG_PFD_480_OFFSET          0x00f0  /* 480MHz Clock (PLL3) Phase Fractional Divider Control */
#define IMXRT_CCM_ANALOG_PFD_528_OFFSET          0x0100  /* 528MHz Clock (PLL2) Phase Fractional Divider Control */
#define IMXRT_CCM_ANALOG_MISC0_OFFSET            0x0150  /* Miscellaneous Register 0 */
#define IMXRT_CCM_ANALOG_MISC1_OFFSET            0x0160  /* Miscellaneous Register 1 */
#define IMXRT_CCM_ANALOG_MISC2_OFFSET            0x0170  /* Miscellaneous Register 2 */

/* Register addresses ***************************************************************************************/

#define IMXRT_CCM_CCR                            (IMXRT_CCM_BASE + IMXRT_CCM_CCR_OFFSET)
#define IMXRT_CCM_CSR                            (IMXRT_CCM_BASE + IMXRT_CCM_CSR_OFFSET)
#define IMXRT_CCM_CCSR                           (IMXRT_CCM_BASE + IMXRT_CCM_CCSR_OFFSET)
#define IMXRT_CCM_CACRR                          (IMXRT_CCM_BASE + IMXRT_CCM_CACRR_OFFSET)
#define IMXRT_CCM_CBCDR                          (IMXRT_CCM_BASE + IMXRT_CCM_CBCDR_OFFSET)
#define IMXRT_CCM_CBCMR                          (IMXRT_CCM_BASE + IMXRT_CCM_CBCMR_OFFSET)
#define IMXRT_CCM_CSCMR1                         (IMXRT_CCM_BASE + IMXRT_CCM_CSCMR1_OFFSET)
#define IMXRT_CCM_CSCMR2                         (IMXRT_CCM_BASE + IMXRT_CCM_CSCMR2_OFFSET)
#define IMXRT_CCM_CSCDR1                         (IMXRT_CCM_BASE + IMXRT_CCM_CSCDR1_OFFSET)
#define IMXRT_CCM_CS1CDR                         (IMXRT_CCM_BASE + IMXRT_CCM_CS1CDR_OFFSET)
#define IMXRT_CCM_CS2CDR                         (IMXRT_CCM_BASE + IMXRT_CCM_CS2CDR_OFFSET)
#define IMXRT_CCM_CDCDR                          (IMXRT_CCM_BASE + IMXRT_CCM_CDCDR_OFFSET)
#define IMXRT_CCM_CSCDR2                         (IMXRT_CCM_BASE + IMXRT_CCM_CSCDR2_OFFSET)
#define IMXRT_CCM_CSCDR3                         (IMXRT_CCM_BASE + IMXRT_CCM_CSCDR3_OFFSET)
#define IMXRT_CCM_CDHIPR                         (IMXRT_CCM_BASE + IMXRT_CCM_CDHIPR_OFFSET)
#define IMXRT_CCM_CLPCR                          (IMXRT_CCM_BASE + IMXRT_CCM_CLPCR_OFFSET)
#define IMXRT_CCM_CISR                           (IMXRT_CCM_BASE + IMXRT_CCM_CISR_OFFSET)
#define IMXRT_CCM_CIMR                           (IMXRT_CCM_BASE + IMXRT_CCM_CIMR_OFFSET)
#define IMXRT_CCM_CCOSR                          (IMXRT_CCM_BASE + IMXRT_CCM_CCOSR_OFFSET)
#define IMXRT_CCM_CGPR                           (IMXRT_CCM_BASE + IMXRT_CCM_CGPR_OFFSET)
#define IMXRT_CCM_CCGR0                          (IMXRT_CCM_BASE + IMXRT_CCM_CCGR0_OFFSET)
#define IMXRT_CCM_CCGR1                          (IMXRT_CCM_BASE + IMXRT_CCM_CCGR1_OFFSET)
#define IMXRT_CCM_CCGR2                          (IMXRT_CCM_BASE + IMXRT_CCM_CCGR2_OFFSET)
#define IMXRT_CCM_CCGR3                          (IMXRT_CCM_BASE + IMXRT_CCM_CCGR3_OFFSET)
#define IMXRT_CCM_CCGR4                          (IMXRT_CCM_BASE + IMXRT_CCM_CCGR4_OFFSET)
#define IMXRT_CCM_CCGR5                          (IMXRT_CCM_BASE + IMXRT_CCM_CCGR5_OFFSET)
#define IMXRT_CCM_CCGR6                          (IMXRT_CCM_BASE + IMXRT_CCM_CCGR6_OFFSET)
#define IMXRT_CCM_CCGR7                          (IMXRT_CCM_BASE + IMXRT_CCM_CCGR7_OFFSET)
#define IMXRT_CCM_CMEOR                          (IMXRT_CCM_BASE + IMXRT_CCM_CMEOR_OFFSET)

#define IMXRT_CCM_ANALOG_PLL_ARM                 (IMXRT_ANATOP_BASE + IMXRT_CCM_ANALOG_PLL_ARM_OFFSET)
#define IMXRT_CCM_ANALOG_PLL_USB1                (IMXRT_ANATOP_BASE + IMXRT_CCM_ANALOG_PLL_USB1_OFFSET)
#define IMXRT_CCM_ANALOG_PLL_USB2                (IMXRT_ANATOP_BASE + IMXRT_CCM_ANALOG_PLL_USB2_OFFSET)
#define IMXRT_CCM_ANALOG_PLL_SYS                 (IMXRT_ANATOP_BASE + IMXRT_CCM_ANALOG_PLL_SYS_OFFSET)
#define IMXRT_CCM_ANALOG_PLL_SYS_SS              (IMXRT_ANATOP_BASE + IMXRT_CCM_ANALOG_PLL_SYS_SS_OFFSET)
#define IMXRT_CCM_ANALOG_PLL_SYS_NUM             (IMXRT_ANATOP_BASE + IMXRT_CCM_ANALOG_PLL_SYS_NUM_OFFSET)
#define IMXRT_CCM_ANALOG_PLL_SYS_DENOM           (IMXRT_ANATOP_BASE + IMXRT_CCM_ANALOG_PLL_SYS_DENOM_OFFSET)
#define IMXRT_CCM_ANALOG_PLL_AUDIO               (IMXRT_ANATOP_BASE + IMXRT_CCM_ANALOG_PLL_AUDIO_OFFSET)
#define IMXRT_CCM_ANALOG_PLL_AUDIO_NUM           (IMXRT_ANATOP_BASE + IMXRT_CCM_ANALOG_PLL_AUDIO_NUM_OFFSET)
#define IMXRT_CCM_ANALOG_PLL_AUDIO_DENOM         (IMXRT_ANATOP_BASE + IMXRT_CCM_ANALOG_PLL_AUDIO_DENOM_OFFSET)
#define IMXRT_CCM_ANALOG_PLL_VIDEO               (IMXRT_ANATOP_BASE + IMXRT_CCM_ANALOG_PLL_VIDEO_OFFSET)
#define IMXRT_CCM_ANALOG_PLL_VIDEO_NUM           (IMXRT_ANATOP_BASE + IMXRT_CCM_ANALOG_PLL_VIDEO_NUM_OFFSET)
#define IMXRT_CCM_ANALOG_PLL_VIDEO_DENOM         (IMXRT_ANATOP_BASE + IMXRT_CCM_ANALOG_PLL_VIDEO_DENOM_OFFSET)
#define IMXRT_CCM_ANALOG_PLL_ENET                (IMXRT_ANATOP_BASE + IMXRT_CCM_ANALOG_PLL_ENET_OFFSET)
#define IMXRT_CCM_ANALOG_PFD_480                 (IMXRT_ANATOP_BASE + IMXRT_CCM_ANALOG_PFD_480_OFFSET)
#define IMXRT_CCM_ANALOG_PFD_528                 (IMXRT_ANATOP_BASE + IMXRT_CCM_ANALOG_PFD_528_OFFSET)
#define IMXRT_CCM_ANALOG_MISC0                   (IMXRT_ANATOP_BASE + IMXRT_CCM_ANALOG_MISC0_OFFSET)
#define IMXRT_CCM_ANALOG_MISC1                   (IMXRT_ANATOP_BASE + IMXRT_CCM_ANALOG_MISC1_OFFSET)
#define IMXRT_CCM_ANALOG_MISC2                   (IMXRT_ANATOP_BASE + IMXRT_CCM_ANALOG_MISC2_OFFSET)

/* Helper Macros ********************************************************************************************/

#define CCM_PODF_FROM_DIVISOR(n)                   ((n)-1)  /* PODF Values are divisor-1 */
#define CCM_PRED_FROM_DIVISOR(n)                   ((n)-1)  /* PRED Values are divisor-1 */

/* Register bit definitions *********************************************************************************/

/* Control Register */

#define CCM_CCR_OSCNT_SHIFT                      (0)       /* Bits 0-7:   Oscillator ready counter value */
#define CCM_CCR_OSCNT_MASK                       (0xff << CCM_CCR_OSCNT_SHIFT)
#  define CCM_CCR_OSCNT(n)                       ((uint32_t)(n) << CCM_CCR_OSCNT_SHIFT)
                                                           /* Bits 8-11:  Reserved */
#define CCM_CCR_COSC_EN                          (1 << 12) /* Bit  12:    On chip oscillator enable */
                                                           /* Bits 13-20: Reserved */
#define CCM_CCR_REG_BYPASS_COUNT_SHIFT           (21)      /* Bits 21-26: Counter for analog_reg_bypass */
#define CCM_CCR_REG_BYPASS_COUNT_MASK            (0x3f << CCM_CCR_REG_BYPASS_COUNT_SHIFT)
#  define CCM_CCR_REG_BYPASS_COUNT(n)            ((uint32_t)(n) << CCM_CCR_REG_BYPASS_COUNT_SHIFT)
#define CCM_CCR_RBC_EN                           (1 << 27) /* Bit  27:    Enable for REG_BYPASS_COUNTER */
                                                           /* Bits 28-31: Reserved */

/*  Status Register  */

#define CCM_CSR_REF_EN_B                         (1 << 0)  /* Bit 0:      Status of the value of CCM_REF_EN_B */
                                                           /* Bits 1-2:   Reserved */
#define CCM_CSR_CAMP2_READY                      (3 << 0)  /* Bit 3:      Status indication of CAMP2 */
                                                           /* Bit 4:      Reserved */
#define CCM_CSR_COSC_READY                       (5 << 0)  /* Bit 5:      Status indication of on board oscillator */
                                                           /* Bits 6-31:  Reserved */

/* Clock Switcher Register */

#define CCM_CCSR_PLL3_SW_CLK_SEL                 (1 << 0)  /* Bit 0: Selects source to generate pll3_sw_clk */

/* Arm Clock Root Register */

#define CCM_CACRR_ARM_PODF_SHIFT                 (0)       /* Bits 0-2:   Divider for ARM clock root */
#define CCM_CACRR_ARM_PODF_MASK                  (0x3 << CCM_CACRR_ARM_PODF_SHIFT)
#  define CCM_CACRR_ARM_PODF(n)                  ((uint32_t)(n) << CCM_CACRR_ARM_PODF_SHIFT)

/* Bus Clock Divider Register */

                                                           /* Bits 0-5:   Reserved */
#define CCM_CBCDR_SEMC_CLK_SEL                   (1 << 6)  /* Bit 6:      SEMC clock source select */
#define CCM_CBCDR_SEMC_ALT_CLK_SEL               (1 << 7)  /* Bit 7:      SEMC alternative clock select */
#define CCM_CBCDR_SEMC_ALT_CLK_SEL_PLL2          (0 << 7)  /* Bit 7:      PLL2 PFD2 will be selected */
#define CCM_CBCDR_SEMC_ALT_CLK_SEL_PLL3          (1 << 7)  /* Bit 7:      PLL3 PFD1 will be selected  */
#define CCM_CBCDR_IPG_PODF_SHIFT                 (8)       /* Bits 8-9:   Divider for ipg podf */
#define CCM_CBCDR_IPG_PODF_MASK                  (0x3 << CCM_CBCDR_IPG_PODF_SHIFT)
#  define CCM_CBCDR_IPG_PODF(n)                  ((uint32_t)(n) << CCM_CBCDR_IPG_PODF_SHIFT)
#define CCM_CBCDR_AHB_PODF_SHIFT                 (10)      /* Bits 10-12: Divider for AHB PODF */
#define CCM_CBCDR_AHB_PODF_MASK                  (0x3 << CCM_CBCDR_AHB_PODF_SHIFT)
#  define CCM_CBCDR_AHB_PODF(n)                  ((uint32_t)(n) << CCM_CBCDR_AHB_PODF_SHIFT)
                                                           /* Bits 13-15: Reserved */
#define CCM_CBCDR_SEMC_PODF_SHIFT                (16)      /* Bits 16-18: Post divider for SEMC clock */
#define CCM_CBCDR_SEMC_PODF_MASK                 (0x3 << CCM_CBCDR_SEMC_PODF_SHIFT)
# define CCM_CBCDR_SEMC_PODF(n)                  ((uint32_t)(n) << CCM_CBCDR_SEMC_PODF_SHIFT)
                                                           /* Bits 19-24: Reserved */
#define CCM_CBCDR_PERIPH_CLK_SEL_SHIFT           (25)      /* Bit 25:     Selector for peripheral main clock */
#define CCM_CBCDR_PERIPH_CLK_SEL_MASK            (1 << CCM_CBCDR_PERIPH_CLK_SEL_SHIFT)
#  define CCM_CBCDR_PERIPH_CLK_SEL(n)            ((uint32_t)(n) << CCM_CBCDR_PERIPH_CLK_SEL_SHIFT)
#  define CCM_CBCDR_PERIPH_CLK_SEL_PRE_PERIPH    (0)
#  define CCM_CBCDR_PERIPH_CLK_SEL_PERIPH_CLK2   (1)

                                                           /* Bit 26:     Reserved */
#define CCM_CBCDR_PERIPH_CLK2_PODF_SHIFT         (27)      /* Bits 27-29: Divider for periph_clk2_podf */
#define CCM_CBCDR_PERIPH_CLK2_PODF_MASK          (0x7 << CCM_CBCDR_PERIPH_CLK2_PODF_SHIFT)
#  define CCM_CBCDR_PERIPH_CLK2_PODF(n)          ((uint32_t)(n) << CCM_CBCDR_PERIPH_CLK2_PODF_SHIFT)
                                                          /* Bits 30-31: Reserved */

/* Bus Clock Multiplexer Register */

                                                           /* Bits 0-3:   Reserved */
#define CCM_CBCMR_LPSPI_CLK_SEL_SHIFT            (4)       /* Bits 4-5:   Selector for lpspi clock multiplexer */
#define CCM_CBCMR_LPSPI_CLK_SEL_MASK             (0x3 << CCM_CBCMR_LPSPI_CLK_SEL_SHIFT)
#  define CCM_CBCMR_LPSPI_CLK_SEL(n)             ((uint32_t)(n) << CCM_CBCMR_LPSPI_CLK_SEL_SHIFT)
#  define CCM_CBCMR_LPSPI_CLK_SEL_PLL3_PFD1      ((uint32_t)(0) << CCM_CBCMR_LPSPI_CLK_SEL_SHIFT)
#  define CCM_CBCMR_LPSPI_CLK_SEL_PLL3_PFD0      ((uint32_t)(1) << CCM_CBCMR_LPSPI_CLK_SEL_SHIFT)
#  define CCM_CBCMR_LPSPI_CLK_SEL_PLL2           ((uint32_t)(2) << CCM_CBCMR_LPSPI_CLK_SEL_SHIFT)
#  define CCM_CBCMR_LPSPI_CLK_SEL_PLL2_PFD2      ((uint32_t)(3) << CCM_CBCMR_LPSPI_CLK_SEL_SHIFT)
                                                           /* Bits 6-11:  Reserved */
#define CCM_CBCMR_PERIPH_CLK2_SEL_SHIFT          (12)      /* Bits 12-13: Selector for peripheral clk2 clock multiplexer */
#define CCM_CBCMR_PERIPH_CLK2_SEL_MASK           (0x3 << CCM_CBCMR_PERIPH_CLK2_SEL_SHIFT)
#  define CCM_CBCMR_PERIPH_CLK2_SEL(n)           ((uint32_t)(n) << CCM_CBCMR_PERIPH_CLK2_SEL_SHIFT)
#  define CCM_CBCMR_PERIPH_CLK2_SEL_PLL3_SW      ((uint32_t)(0) << CCM_CBCMR_PERIPH_CLK2_SEL_SHIFT)
#  define CCM_CBCMR_PERIPH_CLK2_SEL_OSC_CLK      ((uint32_t)(1) << CCM_CBCMR_PERIPH_CLK2_SEL_SHIFT)
#  define CCM_CBCMR_PERIPH_CLK2_SEL_PLL2_BP      ((uint32_t)(2) << CCM_CBCMR_PERIPH_CLK2_SEL_SHIFT)
#define CCM_CBCMR_TRACE_CLK_SEL_SHIFT            (14)      /* Bits 14-15: Selector for Trace clock multiplexer */
#define CCM_CBCMR_TRACE_CLK_SEL_MASK             (0x3 << CCM_CBCMR_TRACE_CLK_SEL_SHIFT)
#  define CCM_CBCMR_TRACE_CLK_SEL(n)             ((uint32_t)(n) << CCM_CBCMR_TRACE_CLK_SEL_SHIFT)
#  define CCM_CBCMR_TRACE_CLK_SEL_PLL2           ((uint32_t)(0) << CCM_CBCMR_TRACE_CLK_SEL_SHIFT)
#  define CCM_CBCMR_TRACE_CLK_SEL_PLL2_PFD2      ((uint32_t)(1) << CCM_CBCMR_TRACE_CLK_SEL_SHIFT)
#  define CCM_CBCMR_TRACE_CLK_SEL_PLL2_PFD0      ((uint32_t)(2) << CCM_CBCMR_TRACE_CLK_SEL_SHIFT)
#  define CCM_CBCMR_TRACE_CLK_SEL_PLL2_PFD1      ((uint32_t)(3) << CCM_CBCMR_TRACE_CLK_SEL_SHIFT)
                                                           /* Bits 16-17: Reserved */
#define CCM_CBCMR_PRE_PERIPH_CLK_SEL_SHIFT       (18)      /* Bits 18-19: Selector for pre_periph clock multiplexer */
#define CCM_CBCMR_PRE_PERIPH_CLK_SEL_MASK        (0x3 << CCM_CBCMR_PRE_PERIPH_CLK_SEL_SHIFT)
#  define CCM_CBCMR_PRE_PERIPH_CLK_SEL(n)        ((uint32_t)(n) << CCM_CBCMR_PRE_PERIPH_CLK_SEL_SHIFT)
#  define CCM_CBCMR_PRE_PERIPH_CLK_SEL_PLL2      (0)
#  define CCM_CBCMR_PRE_PERIPH_CLK_SEL_PLL2_PFD2 (1)
#  define CCM_CBCMR_PRE_PERIPH_CLK_SEL_PLL2_PFD0 (2)
#  define CCM_CBCMR_PRE_PERIPH_CLK_SEL_PLL1      (3)
                                                           /* Bits 20-22: Reserved */
#define CCM_CBCMR_LCDIF_PODF_SHIFT               (23)      /* Bits 23-25: Post-divider for LCDIF clock */
#define CCM_CBCMR_LCDIF_PODF_MASK                (0x7 << CCM_CBCMR_LCDIF_PODF_SHIFT)
# define CCM_CBCMR_LCDIF_PODF(n)                 ((uint32_t)(n) << CCM_CBCMR_LCDIF_PODF_SHIFT)
#define CCM_CBCMR_LPSPI_PODF_SHIFT               (26)      /* Bits 26-28: Divider for LPSPI */
#define CCM_CBCMR_LPSPI_PODF_MASK                (0x7 << CCM_CBCMR_LPSPI_PODF_SHIFT)
#  define CCM_CBCMR_LPSPI_PODF(n)                ((uint32_t)(n) << CCM_CBCMR_LPSPI_PODF_SHIFT)

/* Serial Clock Multiplexer Register 1 */

#define CCM_CSCMR1_PERCLK_PODF_SHIFT             (0)       /* Bits 0-5:   Divider for perclk podf  */
#define CCM_CSCMR1_PERCLK_PODF_MASK              (0x3f << CCM_CSCMR1_PERCLK_PODF_SHIFT)
#  define CCM_CSCMR1_PERCLK_PODF(n)              ((uint32_t)(n) << CCM_CSCMR1_PERCLK_PODF_SHIFT)
#define CCM_CSCMR1_PERCLK_CLK_SEL_SHIFT          (6)       /* Bit 6:      Selector for the perclk clock multiplexer */
#define CCM_CSCMR1_PERCLK_CLK_SEL_MASK           (1 << CCM_CSCMR1_PERCLK_CLK_SEL_SHIFT)
#  define CCM_CSCMR1_PERCLK_CLK_SEL(n)           ((uint32_t)(n) << CCM_CSCMR1_PERCLK_CLK_SEL_SHIFT)
#  define CCM_CSCMR1_PERCLK_CLK_SEL_IPG_CLK_ROOT (0)
#  define CCM_CSCMR1_PERCLK_CLK_SEL_OSC_CLK      (1)
                                                           /* Bits 7-9:   Reserved */
#define CCM_CSCMR1_SAI1_CLK_SEL_SHIFT            (10)      /* Bits 10-11: Selector for sai1 clock multiplexer */
#define CCM_CSCMR1_SAI1_CLK_SEL_MASK             (0x3 << CCM_CSCMR1_SAI1_CLK_SEL_SHIFT)
#  define CCM_CSCMR1_SAI1_CLK_SEL(n)             ((uint32_t)(n) << CCM_CSCMR1_SAI1_CLK_SEL_SHIFT)
#  define CCM_CSCMR1_SAI1_CLK_SEL_PLL3_PFD2      ((uint32_t)(0) << CCM_CSCMR1_SAI1_CLK_SEL_SHIFT)
#  define CCM_CSCMR1_SAI1_CLK_SEL_PLL5           ((uint32_t)(1) << CCM_CSCMR1_SAI1_CLK_SEL_SHIFT)
#  define CCM_CSCMR1_SAI1_CLK_SEL_PLL4           ((uint32_t)(2) << CCM_CSCMR1_SAI1_CLK_SEL_SHIFT)
#define CCM_CSCMR1_SAI2_CLK_SEL_SHIFT            (12)      /* Bits 12-13: Selector for sai2 clock multiplexer */
#define CCM_CSCMR1_SAI2_CLK_SEL_MASK             (0x3 << CCM_CSCMR1_SAI2_CLK_SEL_SHIFT)
#  define CCM_CSCMR1_SAI2_CLK_SEL(n)             ((uint32_t)(n) << CCM_CSCMR1_SAI2_CLK_SEL_SHIFT)
#  define CCM_CSCMR1_SAI2_CLK_SEL_PLL3_PFD2      ((uint32_t)(0) << CCM_CSCMR1_SAI2_CLK_SEL_SHIFT)
#  define CCM_CSCMR1_SAI2_CLK_SEL_PLL5           ((uint32_t)(1) << CCM_CSCMR1_SAI2_CLK_SEL_SHIFT)
#  define CCM_CSCMR1_SAI2_CLK_SEL_PLL4           ((uint32_t)(2) << CCM_CSCMR1_SAI2_CLK_SEL_SHIFT)
#define CCM_CSCMR1_SAI3_CLK_SEL_SHIFT            (14)      /* Bits 14-15: Selector for sai3 clock multiplexer */
#define CCM_CSCMR1_SAI3_CLK_SEL_MASK             (0x3 << CCM_CSCMR1_SAI3_CLK_SEL_SHIFT)
#  define CCM_CSCMR1_SAI3_CLK_SEL(n)             ((uint32_t)(n) << CCM_CSCMR1_SAI3_CLK_SEL_SHIFT)
#  define CCM_CSCMR1_SAI3_CLK_SEL_PLL3_PFD2      ((uint32_t)(0) << CCM_CSCMR1_SAI3_CLK_SEL_SHIFT)
#  define CCM_CSCMR1_SAI3_CLK_SEL_PLL5           ((uint32_t)(1) << CCM_CSCMR1_SAI3_CLK_SEL_SHIFT)
#  define CCM_CSCMR1_SAI3_CLK_SEL_PLL4           ((uint32_t)(2) << CCM_CSCMR1_SAI3_CLK_SEL_SHIFT)
#define CCM_CSCMR1_USDHC1_CLK_SEL                (1 << 16) /* Bit 16:     Selector for usdhc1 clock multiplexer */
#  define CCM_CSCMR1_USDHC1_CLK_SEL_PLL2_PFD2    (0 << 16) /* derive clock from PLL2 PFD2 */
#  define CCM_CSCMR1_USDHC1_CLK_SEL_PLL2_PFD0    (1 << 16) /* derive clock from PLL2 PFD0 */
#define CCM_CSCMR1_USDHC2_CLK_SEL                (1 << 17) /* Bit 17:     Selector for usdhc2 clock multiplexer */
#  define CCM_CSCMR1_USDHC2_CLK_SEL_PLL2_PFD2    (0 << 17) /* derive clock from PLL2 PFD2 */
#  define CCM_CSCMR1_USDHC2_CLK_SEL_PLL2_PFD0    (1 << 17) /* derive clock from PLL2 PFD0 */
                                                           /* Bits 18-22: Reserved */
#define CCM_CSCMR1_FLEXSPI_PODF_SHIFT            (23)      /* Bits 23-25: Divider for flexspi clock root */
#define CCM_CSCMR1_FLEXSPI_PODF_MASK             (0x7 << CCM_CSCMR1_FLEXSPI_PODF_SHIFT)
#  define CCM_CSCMR1_FLEXSPI_PODF(n)             ((uint32_t)(n) << CCM_CSCMR1_FLEXSPI_PODF_SHIFT)
                                                           /* Bits 26-28: Reserved */
#define CCM_CSCMR1_FLEXSPI_CLK_SEL_SHIFT         (29)      /* Bits 29-30: Selector for flexspi clock multiplexer */
#define CCM_CSCMR1_FLEXSPI_CLK_SEL_MASK          (0x3 << CCM_CSCMR1_FLEXSPI_CLK_SEL_SHIFT)
#  define CCM_CSCMR1_FLEXSPI_CLK_SEL(n)          ((uint32_t)(n) << CCM_CSCMR1_FLEXSPI_CLK_SEL_SHIFT)
#  define CCM_CSCMR1_FLEXSPI_CLK_SEL_SEMC_CLK    ((uint32_t)(0) << CCM_CSCMR1_FLEXSPI_CLK_SEL_SHIFT)
#  define CCM_CSCMR1_FLEXSPI_CLK_SEL_PLL3_SW     ((uint32_t)(1) << CCM_CSCMR1_FLEXSPI_CLK_SEL_SHIFT)
#  define CCM_CSCMR1_FLEXSPI_CLK_SEL_PLL2_PFD2   ((uint32_t)(2) << CCM_CSCMR1_FLEXSPI_CLK_SEL_SHIFT)
#  define CCM_CSCMR1_FLEXSPI_CLK_SEL_PLL3_PFD0   ((uint32_t)(3) << CCM_CSCMR1_FLEXSPI_CLK_SEL_SHIFT)
                                                           /* Bit 31:     Reserved */

/* Serial Clock Multiplexer Register 2 */

#define CCM_CSCMR2_CAN_CLK_PODF_SHIFT            (2)       /* Bits 2-7:   Divider for can clock podf */
#define CCM_CSCMR2_CAN_CLK_PODF_MASK             (0x3f << CCM_CSCMR2_CAN_CLK_PODF_SHIFT)
#  define CCM_CSCMR2_CAN_CLK_PODF(n)             ((uint32_t)(n) << CCM_CSCMR2_CAN_CLK_PODF_SHIFT)
#define CCM_CSCMR2_CAN_CLK_SEL_SHIFT             (8)       /* Bits 8-9:   Selector for FlexCAN clock multiplexer */
#define CCM_CSCMR2_CAN_CLK_SEL_MASK              (0x3 << CCM_CSCMR2_CAN_CLK_SEL_SHIFT)
#  define CCM_CSCMR2_CAN_CLK_SEL(n)              ((uint32_t)(n) << CCM_CSCMR2_CAN_CLK_SEL_SHIFT)
#  define CCM_CSCMR2_CAN_CLK_SEL_PLL3_SW_60      ((uint32_t)(0) << CCM_CSCMR2_CAN_CLK_SEL_SHIFT)
#  define CCM_CSCMR2_CAN_CLK_SEL_OSC_CLK         ((uint32_t)(1) << CCM_CSCMR2_CAN_CLK_SEL_SHIFT)
#  define CCM_CSCMR2_CAN_CLK_SEL_PLL3_SW_80      ((uint32_t)(2) << CCM_CSCMR2_CAN_CLK_SEL_SHIFT)
                                                           /* Bits 10-18: Reserved */
#define CCM_CSCMR2_FLEXIO2_CLK_SEL_SHIFT         (19)      /* Bits 19-20: Selector for flexio2 clock multiplexer */
#define CCM_CSCMR2_FLEXIO2_CLK_SEL_MASK          (0x3 << CCM_CSCMR2_FLEXIO2_CLK_SEL_SHIFT)
#  define CCM_CSCMR2_FLEXIO2_CLK_SEL(n)          ((uint32_t)(n) << CCM_CSCMR2_FLEXIO2_CLK_SEL_SHIFT)
#  define CCM_CSCMR2_FLEXIO2_CLK_SEL_PLL4        ((uint32_t)(0) << CCM_CSCMR2_FLEXIO2_CLK_SEL_SHIFT)
#  define CCM_CSCMR2_FLEXIO2_CLK_SEL_PLL3_PFD2   ((uint32_t)(1) << CCM_CSCMR2_FLEXIO2_CLK_SEL_SHIFT)
#  define CCM_CSCMR2_FLEXIO2_CLK_SEL_PLL5        ((uint32_t)(2) << CCM_CSCMR2_FLEXIO2_CLK_SEL_SHIFT)
#  define CCM_CSCMR2_FLEXIO2_CLK_SEL_PLL3_SW     ((uint32_t)(3) << CCM_CSCMR2_FLEXIO2_CLK_SEL_SHIFT)
                                                           /* Bits 21-31: Reserved */

/* Serial Clock Divider Register 1 */

#define CCM_CSCDR1_UART_CLK_PODF_SHIFT           (0)       /* Bits 0-5:   Divider for uart clock podf */
#define CCM_CSCDR1_UART_CLK_PODF_MASK            (0x3f << CCM_CSCDR1_UART_CLK_PODF_SHIFT)
#  define CCM_CSCDR1_UART_CLK_PODF(n)            ((uint32_t)(n) << CCM_CSCDR1_UART_CLK_PODF_SHIFT)
#define CCM_CSCDR1_UART_CLK_SEL                  (1 << 6)  /* Bit 6:      Selector for the UART clock multiplexer */
#  define CCM_CSCDR1_UART_CLK_SEL_PLL3_80        (0 << 6)  /* derive clock from pll3_80m */
#  define CCM_CSCDR1_UART_CLK_SEL_OSC_CLK        (1 << 6)  /* derive clock from osc_clk */
                                                           /* Bits 7-10:  Reserved */
#define CCM_CSCDR1_USDHC1_PODF_SHIFT             (11)      /* Bits 11-13: Divider for usdhc1 clock podf */
#define CCM_CSCDR1_USDHC1_PODF_MASK              (0x7 << CCM_CSCDR1_USDHC1_PODF_SHIFT)
#  define CCM_CSCDR1_USDHC1_PODF(n)              ((uint32_t)(n) << CCM_CSCDR1_USDHC1_PODF_SHIFT)
                                                           /* Bits 14-15: Reserved */
#define CCM_CSCDR1_USDHC2_PODF_SHIFT             (16)      /* Bits 16-18: Divider for usdhc2 clock */
#define CCM_CSCDR1_USDHC2_PODF_MASK              (0x7 << CCM_CSCDR1_USDHC2_PODF_SHIFT)
#  define CCM_CSCDR1_USDHC2_PODF(n)              ((uint32_t)(n) << CCM_CSCDR1_USDHC2_PODF_SHIFT)
                                                           /* Bits 19-24: Reserved */
#define CCM_CSCDR1_TRACE_PODF_SHIFT              (25)      /* Bits 25-27: Divider for trace clock */
#define CCM_CSCDR1_TRACE_PODF_MASK               (0x7 << CCM_CSCDR1_TRACE_PODF_SHIFT)
#  define CCM_CSCDR1_TRACE_PODF(n)               ((uint32_t)(n) << CCM_CSCDR1_TRACE_PODF_SHIFT)
                                                           /* Bits 28-31: Reserved */

/* Clock Divider Register 1 */

#define CCM_CS1CDR_SAI1_CLK_PODF_SHIFT           (0)       /* Bits 0-5:   Divider for sai1 clock podf */
#define CCM_CS1CDR_SAI1_CLK_PODF_MASK            (0x3f << CCM_CS1CDR_SAI1_CLK_PODF_SHIFT)
#  define CCM_CS1CDR_SAI1_CLK_PODF(n)            ((uint32_t)(n) << CCM_CS1CDR_SAI1_CLK_PODF_SHIFT)
#define CCM_CS1CDR_SAI1_CLK_PRED_SHIFT           (6)       /* Bits 6-8:   Divider for sai1 clock pred */
#define CCM_CS1CDR_SAI1_CLK_PRED_MASK            (0x7 << CCM_CS1CDR_SAI1_CLK_PRED_SHIFT)
#  define CCM_CS1CDR_SAI1_CLK_PRED(n)            ((uint32_t)(n) << CCM_CS1CDR_SAI1_CLK_PRED_SHIFT)
#define CCM_CS1CDR_FLEXIO2_CLK_PRED_SHIFT        (9)       /* Bits 9-11:  Divider for flexio2 clock */
#define CCM_CS1CDR_FLEXIO2_CLK_PRED_MASK         (0x7 << CCM_CS1CDR_FLEXIO2_CLK_PRED_SHIFT)
#  define CCM_CS1CDR_FLEXIO2_CLK_PRED(n)         ((uint32_t)(n) << CCM_CS1CDR_FLEXIO2_CLK_PRED_SHIFT)
                                                           /* Bits 12-15: Reserved */
#define CCM_CS1CDR_SAI3_CLK_PODF_SHIFT           (16)      /* Bits 16-21: Divider for sai3 clock podf */
#define CCM_CS1CDR_SAI3_CLK_PODF_MASK            (0x3f << CCM_CS1CDR_SAI3_CLK_PODF_SHIFT)
#  define CCM_CS1CDR_SAI3_CLK_PODF(n)            ((uint32_t)(n) << CCM_CS1CDR_SAI3_CLK_PODF_SHIFT)
#define CCM_CS1CDR_SAI3_CLK_PRED_SHIFT           (22)      /* Bits 22-24:   Divider for sai3 clock pred */
#define CCM_CS1CDR_SAI3_CLK_PRED_MASK            (0x7 << CCM_CS1CDR_SAI3_CLK_PRED_SHIFT)
#  define CCM_CS1CDR_SAI3_CLK_PRED(n)            ((uint32_t)(n) << CCM_CS1CDR_SAI3_CLK_PRED_SHIFT)
#define CCM_CS1CDR_FLEXIO2_CLK_PODF_SHIFT        (25)      /* Bits 25-27:  Divider for flexio2 clock */
#define CCM_CS1CDR_FLEXIO2_CLK_PODF_MASK         (0x7 << CCM_CS1CDR_FLEXIO2_CLK_PODF_SHIFT)
#  define CCM_CS1CDR_FLEXIO2_CLK_PODF(n)         ((uint32_t)(n) << CCM_CS1CDR_FLEXIO2_CLK_PODF_SHIFT)
                                                           /* Bits 28-31: Reserved */

/* Clock Divider Register 2 */

#define CCM_CS2CDR_SAI2_CLK_PODF_SHIFT           (0)       /* Bits 0-5:   Divider for sai2 clock podf */
#define CCM_CS2CDR_SAI2_CLK_PODF_MASK            (0x3f << CCM_CS2CDR_SAI2_CLK_PODF_SHIFT)
#  define CCM_CS2CDR_SAI2_CLK_PODF(n)            ((uint32_t)(n) << CCM_CS2CDR_SAI2_CLK_PODF_SHIFT)
#define CCM_CS2CDR_SAI2_CLK_PRED_SHIFT           (6)       /* Bits 6-8:   Divider for sai2 clock pred */
#define CCM_CS2CDR_SAI2_CLK_PRED_MASK            (0x7 << CCM_CS2CDR_SAI2_CLK_PRED_SHIFT)
#  define CCM_CS2CDR_SAI2_CLK_PRED(n)            ((uint32_t)(n) << CCM_CS2CDR_SAI2_CLK_PRED_SHIFT)

/* D1 Clock Divider Register */

#define CCM_CDCDR_FLEXIO1_CLK_SEL_SHIFT          (7)       /* Bits 7-8:   Selector for flexio1 clock multiplexer */
#define CCM_CDCDR_FLEXIO1_CLK_SEL_MASK           (0x3 << CCM_CDCDR_FLEXIO1_CLK_SEL_SHIFT)
#  define CCM_CDCDR_FLEXIO1_CLK_SEL(n)           ((uint32_t)(n) << CCM_CDCDR_FLEXIO1_CLK_SEL_SHIFT)
#  define CCM_CDCDR_FLEXIO1_CLK_SEL_PLL4         ((uint32_t)(0) << CCM_CDCDR_FLEXIO1_CLK_SEL_SHIFT)
#  define CCM_CDCDR_FLEXIO1_CLK_SEL_PLL3_PFD2    ((uint32_t)(1) << CCM_CDCDR_FLEXIO1_CLK_SEL_SHIFT)
#  define CCM_CDCDR_FLEXIO1_CLK_SEL_PLL3_PLL5    ((uint32_t)(2) << CCM_CDCDR_FLEXIO1_CLK_SEL_SHIFT)
#  define CCM_CDCDR_FLEXIO1_CLK_SEL_PLL3_PLL3_SW ((uint32_t)(3) << CCM_CDCDR_FLEXIO1_CLK_SEL_SHIFT)
#define CCM_CDCDR_FLEXIO1_CLK_PODF_SHIFT         (9)       /* Bits 9-11:  Divider for flexio1 clock podf */
#define CCM_CDCDR_FLEXIO1_CLK_PODF_MASK          (0x7 << CCM_CDCDR_FLEXIO1_CLK_PODF_SHIFT)
#  define CCM_CDCDR_FLEXIO1_CLK_PODF(n)          ((uint32_t)(n) << CCM_CDCDR_FLEXIO1_CLK_PODF_SHIFT)
#define CCM_CDCDR_FLEXIO1_CLK_PRED_SHIFT         (12)      /* Bits 12-14:  Divider for flexio1 clock pred */
#define CCM_CDCDR_FLEXIO1_CLK_PRED_MASK          (0x7 << CCM_CDCDR_FLEXIO1_CLK_PRED_SHIFT)
#  define CCM_CDCDR_FLEXIO1_CLK_PRED(n)          ((uint32_t)(n) << CCM_CDCDR_FLEXIO1_CLK_PRED_SHIFT)
                                                           /* Bits 15-19: Reserved */
#define CCM_CDCDR_SPDIF0_CLK_SEL_SHIFT           (20)      /* Bits 20-21: Selector for spdif0 clock multiplexer */
#define CCM_CDCDR_SPDIF0_CLK_SEL_MASK            (0x3 << CCM_CDCDR_SPDIF0_CLK_SEL_SHIFT)
#  define CCM_CDCDR_SPDIF0_CLK_SEL(n)            ((uint32_t)(n) << CCM_CDCDR_SPDIF0_CLK_SEL_SHIFT)
#  define CCM_CDCDR_SPDIF0_CLK_SEL_PLL4          ((uint32_t)(0) << CCM_CDCDR_SPDIF0_CLK_SEL_SHIFT)
#  define CCM_CDCDR_SPDIF0_CLK_SEL_PLL3_PFD2     ((uint32_t)(1) << CCM_CDCDR_SPDIF0_CLK_SEL_SHIFT)
#  define CCM_CDCDR_SPDIF0_CLK_SEL_PLL3_PLL5     ((uint32_t)(2) << CCM_CDCDR_SPDIF0_CLK_SEL_SHIFT)
#  define CCM_CDCDR_SPDIF0_CLK_SEL_PLL3_PLL3_SW  ((uint32_t)(3) << CCM_CDCDR_SPDIF0_CLK_SEL_SHIFT)
#define CCM_CDCDR_SPDIF0_CLK_PODF_SHIFT          (22)      /* Bits 22-24: Divider for spdif0 clock podf */
#define CCM_CDCDR_SPDIF0_CLK_PODF_MASK           (0x7 << CCM_CDCDR_SPDIF0_CLK_PODF_SHIFT)
#  define CCM_CDCDR_SPDIF0_CLK_PODF(n)           ((uint32_t)(n) << CCM_CDCDR_SPDIF0_CLK_PODF_SHIFT)
#define CCM_CDCDR_SPDIF0_CLK_PRED_SHIFT          (25)      /* Bits 25-27: Divider for spdif0 clock pred */
#define CCM_CDCDR_SPDIF0_CLK_PRED_MASK           (0x7 << CCM_CDCDR_SPDIF0_CLK_PRED_SHIFT)
#  define CCM_CDCDR_SPDIF0_CLK_PRED(n)           ((uint32_t)(n) << CCM_CDCDR_SPDIF0_CLK_PRED_SHIFT)

/* Serial Clock Divider Register 2 */

                                                           /* Bits 0-8:   Reserved */
#define CCM_CSCDR2_LCDIF_CLK_SEL_SHIFT           (9)       /* Bits 9-11:  Selector for LCDIF root clock multiplexer */
#define CCM_CSCDR2_LCDIF_CLK_SEL_MASK            (0x7 << CCM_CSCDR2_LCDIF_CLK_SEL_SHIFT)
#  define CCM_CSCDR2_LCDIF_CLK_SEL(n)            ((uint32_t)(n) << CCM_CSCDR2_LCDIF_CLK_SEL_SHIFT)
#  define CCM_CSCDR2_LCDIF_CLK_SEL_LCDIF         ((uint32_t)(0) << CCM_CSCDR2_LCDIF_CLK_SEL_SHIFT)
#  define CCM_CSCDR2_LCDIF_CLK_SEL_IPP_DI0_CLK   ((uint32_t)(1) << CCM_CSCDR2_LCDIF_CLK_SEL_SHIFT)
#  define CCM_CSCDR2_LCDIF_CLK_SEL_IPP_DI1_CLK   ((uint32_t)(2) << CCM_CSCDR2_LCDIF_CLK_SEL_SHIFT)
#  define CCM_CSCDR2_LCDIF_CLK_SEL_IDB_DI0_CLK   ((uint32_t)(3) << CCM_CSCDR2_LCDIF_CLK_SEL_SHIFT)
#  define CCM_CSCDR2_LCDIF_CLK_SEL_IDB_DI1_CLK   ((uint32_t)(4) << CCM_CSCDR2_LCDIF_CLK_SEL_SHIFT)
#define CCM_CSCDR2_LCDIF_PRED_SHIFT              (12)      /* Bits 12-14: Pre-divider for lcdif clock */
#define CCM_CSCDR2_LCDIF_PRED_MASK               (0x7 << CCM_CSCDR2_LCDIF_PRED_SHIFT)
#  define CCM_CSCDR2_LCDIF_PRED(n)               ((uint32_t)(n) << CCM_CSCDR2_LCDIF_PRED_SHIFT)
#define CCM_CSCDR2_LCDIF_PRE_CLK_SEL_SHIFT       (15)      /* Bits 15-17: Selector for lcdif root clock pre-multiplexer */
#define CCM_CSCDR2_LCDIF_PRE_CLK_SEL_MASK        (0x7 << CCM_CSCDR2_LCDIF_PRE_CLK_SEL_SHIFT)
#  define CCM_CSCDR2_LCDIF_PRE_CLK_SEL_(n)       ((uint32_t)(n) << CCM_CSCDR2_LCDIF_PRE_CLK_SEL_SHIFT)
#  define CCM_CSCDR2_LCDIF_PRE_CLK_SEL_PLL2      ((uint32_t)(0) << CCM_CSCDR2_LCDIF_PRE_CLK_SEL_SHIFT)
#  define CCM_CSCDR2_LCDIF_PRE_CLK_SEL_PLL3_PFD3 ((uint32_t)(1) << CCM_CSCDR2_LCDIF_PRE_CLK_SEL_SHIFT)
#  define CCM_CSCDR2_LCDIF_PRE_CLK_SEL_PLL5      ((uint32_t)(2) << CCM_CSCDR2_LCDIF_PRE_CLK_SEL_SHIFT)
#  define CCM_CSCDR2_LCDIF_PRE_CLK_SEL_PLL2_PFD0 ((uint32_t)(3) << CCM_CSCDR2_LCDIF_PRE_CLK_SEL_SHIFT)
#  define CCM_CSCDR2_LCDIF_PRE_CLK_SEL_PLL2_PFD1 ((uint32_t)(4) << CCM_CSCDR2_LCDIF_PRE_CLK_SEL_SHIFT)
#  define CCM_CSCDR2_LCDIF_PRE_CLK_SEL_PLL3_PFD1 ((uint32_t)(5) << CCM_CSCDR2_LCDIF_PRE_CLK_SEL_SHIFT)
#define CCM_CSCDR2_LPI2C_CLK_SEL                 (1 << 18) /* Bit 18:     Selector for the LPI2C clock multiplexer */
#  define CCM_CSCDR2_LPI2C_CLK_SEL_PLL3_60M      (0 << 18) /*  derive clock from pll3_60m */
#  define CCM_CSCDR2_LPI2C_CLK_SEL_OSC_CLK       (1 << 18) /*  derive clock from ock_clk */
#define CCM_CSCDR2_LPI2C_CLK_PODF_SHIFT          (19)      /* Bits 19-24: Divider for lpi2c clock podf */
#define CCM_CSCDR2_LPI2C_CLK_PODF_MASK           (0x3f << CCM_CSCDR2_LPI2C_CLK_PODF_SHIFT)
#  define CCM_CSCDR2_LPI2C_CLK_PODF(n)           ((uint32_t)(n) << CCM_CSCDR2_LPI2C_CLK_PODF_SHIFT)
                                                           /* Bits 25-31: Reserved */

/* Serial Clock Divider Register 3 */

                                                           /* Bits 0-8:   Reserved */
#define CCM_CSCDR3_CSI_CLK_SEL_SHIFT             (9)       /* Bits 9-10:  Selector for csi_mclk multiplexer */
#define CCM_CSCDR3_CSI_CLK_SEL_MASK              (0x3 << CCM_CSCDR3_CSI_CLK_SEL_SHIFT)
#  define CCM_CSCDR3_CSI_CLK_SEL(n)              ((uint32_t)(n) << CCM_CSCDR3_CSI_CLK_SEL_SHIFT)
#  define CCM_CSCDR3_CSI_CLK_SEL_OSC_CLK         ((uint32_t)(0) << CCM_CSCDR3_CSI_CLK_SEL_SHIFT)
#  define CCM_CSCDR3_CSI_CLK_SEL_OSC_PLL2_PFD2   ((uint32_t)(1) << CCM_CSCDR3_CSI_CLK_SEL_SHIFT)
#  define CCM_CSCDR3_CSI_CLK_SEL_OSC_PLL3_120M   ((uint32_t)(2) << CCM_CSCDR3_CSI_CLK_SEL_SHIFT)
#  define CCM_CSCDR3_CSI_CLK_SEL_OSC_PLL3_PFD1   ((uint32_t)(3) << CCM_CSCDR3_CSI_CLK_SEL_SHIFT)
#define CCM_CSCDR3_CSI_PODF_SHIFT                (11)      /* Bits 11-13: Post divider for csi_mclk */
#define CCM_CSCDR3_CSI_PODF_MASK                 (0x3 << CCM_CSCDR3_CSI_PODF_SHIFT)
#  define CCM_CSCDR3_CSI_PODF(n)                 ((uint32_t)(n) << CCM_CSCDR3_CSI_PODF_SHIFT)

/* Divider Handshake In-Process Register */

#define CCM_CDHIPR_SEMC_PODF_BUSY                (1 << 0)  /* Bit 0:      Busy indicator for semc_podf */
#define CCM_CDHIPR_AHB_PODF_BUSY                 (1 << 1)  /* Bit 1:      Busy indicator for ahb_podf */
                                                           /* Bit 2:      Reserved */
#define CCM_CDHIPR_PERIPH2_CLK_SEL_BUSY          (1 << 3)  /* Bit 3:      Busy indicator for periph2_clk_sel mux control */
                                                           /* Bit 4:      Reserved */
#define CCM_CDHIPR_PERIPH_CLK_SEL_BUSY           (1 << 5)  /* Bit 5:      Busy indicator for periph_clk_sel mux control */
                                                           /* Bits 6-15:  Reserved */
#define CCM_CDHIPR_ARM_PODF_BUSY                 (1 << 16) /* Bit 16:     Busy indicator for arm_podf */
                                                           /* Bits 17-31: Reserved */

/* Low Power Control Register */

#define CCM_CLPCR_LPM_SHIFT                      (0)       /* Bits 0-1:   Setting the low power mode */
#define CCM_CLPCR_LPM_MASK                       (0x3 << CCM_CLPCR_LPM_SHIFT)
#  define CCM_CLPCR_LPM(n)                       ((uint32_t)(n) << CCM_CLPCR_LPM_SHIFT)
#  define CCM_CLPCR_LPM_RUN                      ((uint32_t)(0) << CCM_CLPCR_LPM_SHIFT) /* Remain in run mode */
#  define CCM_CLPCR_LPM_WAIT                     ((uint32_t)(1) << CCM_CLPCR_LPM_SHIFT) /* Transfer to wait mode */
#  define CCM_CLPCR_LPM_STOP                     ((uint32_t)(2) << CCM_CLPCR_LPM_SHIFT) /* Transfer to stop mode */

                                                           /* Bits 2-4:   Reserved */
#define CCM_CLPCR_ARM_CLK_DIS_ON_LPM             (1 << 5)  /* Bit 5:      ARM clocks disabled on wait mode */
#define CCM_CLPCR_SBYOS                          (1 << 6)  /* Bit 6:      Standby clock oscillator bit */
#define CCM_CLPCR_DIS_REF_OSC                    (1 << 7)  /* Bit 7:      external high frequency oscillator disable */
#define CCM_CLPCR_VSTBY                          (1 << 8)  /* Bit 8:      Voltage standby request bit */
#define CCM_CLPCR_STBY_COUNT_SHIFT               (9)       /* Bits 9-10:  Standby counter definition */
#define CCM_CLPCR_STBY_COUNT_MASK                (0x3 << CCM_CLPCR_STBY_COUNT_SHIFT)
#  define CCM_CLPCR_STBY_COUNT(n)                ((uint32_t)(n) << CCM_CLPCR_STBY_COUNT_SHIFT)
#define CCM_CLPCR_COSC_PWRDOWN                   (1 << 11) /* Bit 11:     On chip oscillator power down */
                                                           /* Bits 12-18: Reserved */
#define CCM_CLPCR_BYPASS_LPM_HS1                 (1 << 19) /* Bit 19:     Bypass low power mode handshake */
                                                           /* Bit 20:     Reserved */
#define CCM_CLPCR_BYPASS_LPM_HS0                 (1 << 21) /* Bit 21:     Bypass low power mode handshake */
#define CCM_CLPCR_MASK_CORE0_WFI                 (1 << 22) /* Bit 22:     Mask WFI of core0 for entering low power mode */
                                                           /* Bits 23-25: Reserved */
#define CCM_CLPCR_MASK_SCU_IDLE                  (1 << 26) /* Bit 26:     Mask SCU IDLE for entering low power mode */
#define CCM_CLPCR_MASK_L2CC_IDLE                 (1 << 27) /* Bit 27:     Mask L2CC IDLE for entering low power mode */
                                                           /* Bits 28-31: Reserved */

/* Interrupt Status Register */

#define CCM_CISR_LRF_PLL                         (1 << 0)  /* Bit 0:      CCM irq2, lock of all enabled and not bypassed PLLs */
                                                           /* Bits 1-5:   Reserved */
#define CCM_CISR_COSC_READY                      (1 << 6)  /* Bit 6:      CCM irq2, on board oscillator ready */
                                                           /* Bits 7-16:  Reserved */
#define CCM_CISR_SEMC_PODF_LOADED                (1 << 17) /* Bit 17:     CCM irq1, frequency change of semc_podf */
                                                           /* Bit 18:     Reserved */
#define CCM_CISR_PERIPH2_CLK_SEL_LOADED          (1 << 19) /* Bit 19:     CCM irq1, frequency change of periph2_clk_sel */
#define CCM_CISR_AHB_PODF_LOADED                 (1 << 20) /* Bit 20:     CCM irq1, frequency change of ahb_podf */
                                                           /* Bit 21:     Reserved */
#define CCM_CISR_PERIPH_CLK_SEL_LOADED           (1 << 22) /* Bit 22:     CCM irq1, update of periph_clk_sel */
                                                           /* Bits 23-25: Reserved */
#define CCM_CISR_ARM_PODF_LOADED                 (1 << 26) /* Bit 26:     CCM irq1, frequency change of arm_podf */
                                                           /* Bits 27-31: Reserved */

/* Interrupt Mask Register */

#define CCM_CIMR_MASK_LRF_PLL                    (1 << 0)  /* Bit 0:      mask interrupt generation due to lrf of PLLs */
                                                           /* Bits 1-5:   Reserved */
#define CCM_CIMR_MASK_COSC_READY                 (1 << 6)  /* Bit 6:      mask interrupt generation due to on board oscillator ready */
                                                           /* Bits 7-16:  Reserved */
#define CCM_CIMR_MASK_SEMC_PODF_LOADED           (1 << 17) /* Bit 17:     mask interrupt generation due to frequency change of semc_podf */
                                                           /* Bit 18:     Reserved */
#define CCM_CIMR_MASK_PERIPH2_CLK_SEL_LOADED     (1 << 19) /* Bit 19:     mask interrupt generation due to update of periph2_clk_sel */
#define CCM_CIMR_MASK_AHB_PODF_LOADED            (1 << 20) /* Bit 20:     mask interrupt generation due to frequency change of ahb_podf */
                                                           /* Bit 21:     Reserved */
#define CCM_CIMR_MASK_PERIPH_CLK_SEL_LOADED      (1 << 22) /* Bit 22:     mask interrupt generation due to update of periph_clk_sel */
                                                           /* Bits 23-25: Reserved */
#define CCM_CIMR_MASK_ARM_PODF_LOADED            (1 << 26) /* Bit 26:     mask interrupt generation due to frequency change of arm_podf */
                                                           /* Bits 27-31: Reserved */

/* Clock Output Source Register */

#define CCM_CCOSR_CLKO1_SEL_SHIFT                (0)       /* Bits 0-3:   Selection of the clock to be generated on CCM_CLKO1 */
#define CCM_CCOSR_CLKO1_SEL_MASK                 (0xF << CCM_CCOSR_CLKO1_SEL_SHIFT)
#  define CCM_CCOSR_CLKO1_SEL_SEMC_CLK           ((uint32_t)(5) << CCM_CCOSR_CLKO1_SEL_SHIFT)
#  define CCM_CCOSR_CLKO1_SEL_ENC_CLK            ((uint32_t)(6) << CCM_CCOSR_CLKO1_SEL_SHIFT)
#  define CCM_CCOSR_CLKO1_SEL_LCDIF_PIX_CLK      ((uint32_t)(10) << CCM_CCOSR_CLKO1_SEL_SHIFT)
#  define CCM_CCOSR_CLKO1_SEL_AHB_CLK            ((uint32_t)(11) << CCM_CCOSR_CLKO1_SEL_SHIFT)
#  define CCM_CCOSR_CLKO1_SEL_IPG_CLK            ((uint32_t)(12) << CCM_CCOSR_CLKO1_SEL_SHIFT)
#  define CCM_CCOSR_CLKO1_SEL_PER_CLK            ((uint32_t)(13) << CCM_CCOSR_CLKO1_SEL_SHIFT)
#  define CCM_CCOSR_CLKO1_SEL_CKIL_SYNC_CLK      ((uint32_t)(14) << CCM_CCOSR_CLKO1_SEL_SHIFT)
#  define CCM_CCOSR_CLKO1_SEL_PLL4_MAIN_CLK      ((uint32_t)(15) << CCM_CCOSR_CLKO1_SEL_SHIFT)
#define CCM_CCOSR_CLKO1_DIV_SHIFT                (4)       /* Bits 4-6:   Setting the divider of CCM_CLKO1 */
#define CCM_CCOSR_CLKO1_DIV_MASK                 (0x7 << CCM_CCOSR_CLKO1_DIV_SHIFT)
#  define CCM_CCOSR_CLKO1_DIV(n)                 ((uint32_t)(n) << CCM_CCOSR_CLKO1_DIV_SHIFT)
#define CCM_CCOSR_CLKO1_EN                       (1 << 7)  /* Bit 7:      Enable of CCM_CLKO1 clock */
#define CCM_CCOSR_CLK_OUT_SEL                    (1 << 8)  /* Bit 8:      CCM_CLKO1 output to reflect CCM_CLKO1 or CCM_CLKO2 clocks */
                                                           /* Bits 9-15:  Reserved */
#define CCM_CCOSR_CLKO2_SEL_SHIFT                (16)      /* Bits 16-20: Selection of the clock to be generated on CCM_CLKO2 */
#define CCM_CCOSR_CLKO2_SEL_MASK                 (0x1F << CCM_CCOSR_CLKO2_SEL_SHIFT)
#  define CCM_CCOSR_CLKO2_SEL_USDHC1_CLK         ((uint32_t)(3) << CCM_CCOSR_CLKO2_SEL_SHIFT)
#  define CCM_CCOSR_CLKO2_SEL_WRCK_CLK           ((uint32_t)(5) << CCM_CCOSR_CLKO2_SEL_SHIFT)
#  define CCM_CCOSR_CLKO2_SEL_LPI2C_CLK          ((uint32_t)(6) << CCM_CCOSR_CLKO2_SEL_SHIFT)
#  define CCM_CCOSR_CLKO2_SEL_CSI_CORE           ((uint32_t)(11) << CCM_CCOSR_CLKO2_SEL_SHIFT)
#  define CCM_CCOSR_CLKO2_SEL_OSC_CLK            ((uint32_t)(14) << CCM_CCOSR_CLKO2_SEL_SHIFT)
#  define CCM_CCOSR_CLKO2_SEL_USDHC2_CLK         ((uint32_t)(17) << CCM_CCOSR_CLKO2_SEL_SHIFT)
#  define CCM_CCOSR_CLKO2_SEL_SAI1_CLK           ((uint32_t)(18) << CCM_CCOSR_CLKO2_SEL_SHIFT)
#  define CCM_CCOSR_CLKO2_SEL_SAI2_CLK           ((uint32_t)(19) << CCM_CCOSR_CLKO2_SEL_SHIFT)
#  define CCM_CCOSR_CLKO2_SEL_SAI3_CLK           ((uint32_t)(20) << CCM_CCOSR_CLKO2_SEL_SHIFT)
#  define CCM_CCOSR_CLKO2_SEL_CAN_CLK            ((uint32_t)(23) << CCM_CCOSR_CLKO2_SEL_SHIFT)
#  define CCM_CCOSR_CLKO2_SEL_FLEXSPI_CLK        ((uint32_t)(27) << CCM_CCOSR_CLKO2_SEL_SHIFT)
#  define CCM_CCOSR_CLKO2_SEL_UART_CLK           ((uint32_t)(28) << CCM_CCOSR_CLKO2_SEL_SHIFT)
#  define CCM_CCOSR_CLKO2_SEL_SPDIF0_CLK         ((uint32_t)(29) << CCM_CCOSR_CLKO2_SEL_SHIFT)
#define CCM_CCOSR_CLKO2_DIV_SHIFT                (21)      /* Bits 21-23: Setting the divider of CCM_CLKO2 */
#define CCM_CCOSR_CLKO2_DIV_MASK                 (0x7 << CCM_CCOSR_CLKO2_DIV_SHIFT)
#  define CCM_CCOSR_CLKO2_DIV(n)                 ((uint32_t)(n) << CCM_CCOSR_CLKO2_DIV_SHIFT)
#define CCM_CCOSR_CLKO2_EN                       (1 << 24) /* Bit 24:     Enable of CCM_CLKO2 clock */
                                                           /* Bits 25-31: Reserved */

/* General Purpose Register */

#define CCM_CGPR_PMIC_DELAY_SCALER               (1 << 0)  /* Bit 0:      Defines clock division of clock for stby_count */
                                                           /* Bits 1-3:   Reserved */
#define CCM_CGPR_EFUSE_PROG_SUPPLY_GATE          (1 << 4)  /* Bit 4:      allow fuse programming */
                                                           /* Bits 5-13:  Reserved */
#define CCM_CGPR_SYS_MEM_DS_CTRL_SHIFT           (14)      /* Bits 14-15: System memory DS control */
#define CCM_CGPR_SYS_MEM_DS_CTRL_MASK            (0x7 << CCM_CGPR_SYS_MEM_DS_CTRL_SHIFT)
#  define CCM_CGPR_SYS_MEM_DS_CTRL(n)            ((uint32_t)(n) << CCM_CGPR_SYS_MEM_DS_CTRL_SHIFT)
#define CCM_CGPR_FPL                             (1 << 16) /* Bit 16:     Fast PLL enable */
#define CCM_CGPR_INT_MEM_CLK_LPM                 (1 << 17) /* Bit 17:     Control for the Deep Sleep signal to the ARM Platform memories */
                                                           /* Bits 18-31: Reserved */

/* Clock Gating Register 0-6 */

#define CCM_CG_OFF                               (0)  /* Clock is off during all modes */
#define CCM_CG_RUN                               (1)  /* Clock is on in run mode, but off in WAIT and STOP modes */
#define CCM_CG_ALL                               (3)  /* Clock is on during all modes, except STOP mode. */

#define CCM_CCGRX_CG_SHIFT(r)                    ((r) << 1)
#define CCM_CCGRX_CG_MASK(r)                     (0x3 << CCM_CCGRX_CG_SHIFT(r))
#  define CCM_CCGRX_CG(r,v)                      ((uint32_t)(v) << CCM_CCGRX_CG_SHIFT(r))

#define CCM_CCGRX_CG0_SHIFT                      (0)
#define CCM_CCGRX_CG0_MASK                       (0x3 << CCM_CCGRX_CG0_SHIFT)
#  define CCM_CCGRX_CG0(n)                       ((uint32_t)(n) << CCM_CCGRX_CG0_SHIFT)
#define CCM_CCGRX_CG1_SHIFT                      (2)
#define CCM_CCGRX_CG1_MASK                       (0x3 << CCM_CCGRX_CG1_SHIFT)
#  define CCM_CCGRX_CG1(n)                       ((uint32_t)(n) << CCM_CCGRX_CG1_SHIFT)
#define CCM_CCGRX_CG2_SHIFT                      (4)
#define CCM_CCGRX_CG2_MASK                       (0x3 << CCM_CCGRX_CG2_SHIFT)
#  define CCM_CCGRX_CG2(n)                       ((uint32_t)(n) << CCM_CCGRX_CG2_SHIFT)
#define CCM_CCGRX_CG3_SHIFT                      (6)
#define CCM_CCGRX_CG3_MASK                       (0x3 << CCM_CCGRX_CG3_SHIFT)
#  define CCM_CCGRX_CG3(n)                       ((uint32_t)(n) << CCM_CCGRX_CG3_SHIFT)
#define CCM_CCGRX_CG4_SHIFT                      (8)
#define CCM_CCGRX_CG4_MASK                       (0x3 << CCM_CCGRX_CG4_SHIFT)
#  define CCM_CCGRX_CG4(n)                       ((uint32_t)(n) << CCM_CCGRX_CG4_SHIFT)
#define CCM_CCGRX_CG5_SHIFT                      (10)
#define CCM_CCGRX_CG5_MASK                       (0x3 << CCM_CCGRX_CG5_SHIFT)
#  define CCM_CCGRX_CG5(n)                       ((uint32_t)(n) << CCM_CCGRX_CG5_SHIFT)
#define CCM_CCGRX_CG6_SHIFT                      (12)
#define CCM_CCGRX_CG6_MASK                       (0x3 << CCM_CCGRX_CG6_SHIFT)
#  define CCM_CCGRX_CG6(n)                       ((uint32_t)(n) << CCM_CCGRX_CG6_SHIFT)
#define CCM_CCGRX_CG7_SHIFT                      (14)
#define CCM_CCGRX_CG7_MASK                       (0x3 << CCM_CCGRX_CG7_SHIFT)
#  define CCM_CCGRX_CG7(n)                       ((uint32_t)(n) << CCM_CCGRX_CG7_SHIFT)
#define CCM_CCGRX_CG8_SHIFT                      (16)
#define CCM_CCGRX_CG8_MASK                       (0x3 << CCM_CCGRX_CG8_SHIFT)
#  define CCM_CCGRX_CG8(n)                       ((uint32_t)(n) << CCM_CCGRX_CG8_SHIFT)
#define CCM_CCGRX_CG9_SHIFT                      (18)
#define CCM_CCGRX_CG9_MASK                       (0x3 << CCM_CCGRX_CG9_SHIFT)
#  define CCM_CCGRX_CG9(n)                       ((uint32_t)(n) << CCM_CCGRX_CG9_SHIFT)
#define CCM_CCGRX_CG10_SHIFT                     (20)
#define CCM_CCGRX_CG10_MASK                      (0x3 << CCM_CCGRX_CG10_SHIFT)
#  define CCM_CCGRX_CG10(n)                      ((uint32_t)(n) << CCM_CCGRX_CG10_SHIFT)
#define CCM_CCGRX_CG11_SHIFT                     (22)
#define CCM_CCGRX_CG11_MASK                      (0x3 << CCM_CCGRX_CG11_SHIFT)
#  define CCM_CCGRX_CG11(n)                      ((uint32_t)(n) << CCM_CCGRX_CG11_SHIFT)
#define CCM_CCGRX_CG12_SHIFT                     (24)
#define CCM_CCGRX_CG12_MASK                      (0x3 << CCM_CCGRX_CG12_SHIFT)
#  define CCM_CCGRX_CG12(n)                      ((uint32_t)(n) << CCM_CCGRX_CG12_SHIFT)
#define CCM_CCGRX_CG13_SHIFT                     (26)
#define CCM_CCGRX_CG13_MASK                      (0x3 << CCM_CCGRX_CG13_SHIFT)
#  define CCM_CCGRX_CG13(n)                      ((uint32_t)(n) << CCM_CCGRX_CG13_SHIFT)
#define CCM_CCGRX_CG14_SHIFT                     (28)
#define CCM_CCGRX_CG14_MASK                      (0x3 << CCM_CCGRX_CG14_SHIFT)
#  define CCM_CCGRX_CG14(n)                      ((uint32_t)(n) << CCM_CCGRX_CG14_SHIFT)
#define CCM_CCGRX_CG15_SHIFT                     (30)
#define CCM_CCGRX_CG15_MASK                      (0x3 << CCM_CCGRX_CG15_SHIFT)
#  define CCM_CCGRX_CG15(n)                      ((uint32_t)(n) << CCM_CCGRX_CG15_SHIFT)

/* Macros used by imxrt_periphclks.h */

#define CCM_CCGR_GPIO2                           IMXRT_CCM_CCGR0, 15
#define CCM_CCGR_LPUART2                         IMXRT_CCM_CCGR0, 14
#define CCM_CCGR_GPT2_SERIAL                     IMXRT_CCM_CCGR0, 13
#define CCM_CCGR_GPT2_BUS                        IMXRT_CCM_CCGR0, 12
#define CCM_CCGR_TRACE                           IMXRT_CCM_CCGR0, 11
#define CCM_CCGR_CAN2_SERIAL                     IMXRT_CCM_CCGR0, 10
#define CCM_CCGR_CAN2                            IMXRT_CCM_CCGR0, 9
#define CCM_CCGR_CAN1_SERIAL                     IMXRT_CCM_CCGR0, 8
#define CCM_CCGR_CAN1                            IMXRT_CCM_CCGR0, 7
#define CCM_CCGR_LPUART3                         IMXRT_CCM_CCGR0, 6
#define CCM_CCGR_DCP                             IMXRT_CCM_CCGR0, 5
#define CCM_CCGR_MQS                             IMXRT_CCM_CCGR0, 2
#define CCM_CCGR_AIPS_TZ2                        IMXRT_CCM_CCGR0, 1
#define CCM_CCGR_AIPS_TZ1                        IMXRT_CCM_CCGR0, 0

#define CCM_CCGR_CSU                             IMXRT_CCM_CCGR1, 14
#define CCM_CCGR_GPIO1                           IMXRT_CCM_CCGR1, 13
#define CCM_CCGR_LPUART4                         IMXRT_CCM_CCGR1, 12
#define CCM_CCGR_GPT_SERIAL                      IMXRT_CCM_CCGR1, 11
#define CCM_CCGR_GPT_BUS                         IMXRT_CCM_CCGR1, 10
#define CCM_CCGR_ADC1                            IMXRT_CCM_CCGR1, 8
#define CCM_CCGR_AOI2                            IMXRT_CCM_CCGR1, 7
#define CCM_CCGR_PIT                             IMXRT_CCM_CCGR1, 6
#define CCM_CCGR_ENET                            IMXRT_CCM_CCGR1, 5
#define CCM_CCGR_ADC2                            IMXRT_CCM_CCGR1, 4
#define CCM_CCGR_LPSPI4                          IMXRT_CCM_CCGR1, 3
#define CCM_CCGR_LPSPI3                          IMXRT_CCM_CCGR1, 2
#define CCM_CCGR_LPSPI2                          IMXRT_CCM_CCGR1, 1
#define CCM_CCGR_LPSPI1                          IMXRT_CCM_CCGR1, 0

#define CCM_CCGR_PXP                             IMXRT_CCM_CCGR2, 15
#define CCM_CCGR_LCD                             IMXRT_CCM_CCGR2, 14
#define CCM_CCGR_GPIO3                           IMXRT_CCM_CCGR2, 13
#define CCM_CCGR_XBAR2                           IMXRT_CCM_CCGR2, 12
#define CCM_CCGR_XBAR1                           IMXRT_CCM_CCGR2, 11
#define CCM_CCGR_IPMUX3                          IMXRT_CCM_CCGR2, 10
#define CCM_CCGR_IPMUX2                          IMXRT_CCM_CCGR2, 9
#define CCM_CCGR_IPMUX1                          IMXRT_CCM_CCGR2, 8
#define CCM_CCGR_XBAR3                           IMXRT_CCM_CCGR2, 7
#define CCM_CCGR_OCOTP_CTRL                      IMXRT_CCM_CCGR2, 6
#define CCM_CCGR_LPI2C3                          IMXRT_CCM_CCGR2, 5
#define CCM_CCGR_LPI2C2                          IMXRT_CCM_CCGR2, 4
#define CCM_CCGR_LPI2C1                          IMXRT_CCM_CCGR2, 3
#define CCM_CCGR_IOMUXC_SNVS                     IMXRT_CCM_CCGR2, 2
#define CCM_CCGR_CSI                             IMXRT_CCM_CCGR2, 1

#define CCM_CCGR_IOMUXC_SNVS_GPR                 IMXRT_CCM_CCGR3, 15
#define CCM_CCGR_OCRAM                           IMXRT_CCM_CCGR3, 14
#define CCM_CCGR_ACMP4                           IMXRT_CCM_CCGR3, 13
#define CCM_CCGR_ACMP3                           IMXRT_CCM_CCGR3, 12
#define CCM_CCGR_ACMP2                           IMXRT_CCM_CCGR3, 11
#define CCM_CCGR_ACMP1                           IMXRT_CCM_CCGR3, 10
#define CCM_CCGR_FLEXRAM                         IMXRT_CCM_CCGR3, 9
#define CCM_CCGR_WDOG1                           IMXRT_CCM_CCGR3, 8
#define CCM_CCGR_EWM                             IMXRT_CCM_CCGR3, 7
#define CCM_CCGR_GPIO4                           IMXRT_CCM_CCGR3, 6
#define CCM_CCGR_LCDIF_PIX                       IMXRT_CCM_CCGR3, 5
#define CCM_CCGR_AOI1                            IMXRT_CCM_CCGR3, 4
#define CCM_CCGR_LPUART6                         IMXRT_CCM_CCGR3, 3
#define CCM_CCGR_SEMC                            IMXRT_CCM_CCGR3, 2
#define CCM_CCGR_LPUART5                         IMXRT_CCM_CCGR3, 1
#define CCM_CCGR_FLEXIO2                         IMXRT_CCM_CCGR3, 0

#define CCM_CCGR_ENC4                            IMXRT_CCM_CCGR4, 15
#define CCM_CCGR_ENC3                            IMXRT_CCM_CCGR4, 14
#define CCM_CCGR_ENC2                            IMXRT_CCM_CCGR4, 13
#define CCM_CCGR_ENC1                            IMXRT_CCM_CCGR4, 12
#define CCM_CCGR_PWM4                            IMXRT_CCM_CCGR4, 11
#define CCM_CCGR_PWM3                            IMXRT_CCM_CCGR4, 10
#define CCM_CCGR_PWM2                            IMXRT_CCM_CCGR4, 9
#define CCM_CCGR_PWM1                            IMXRT_CCM_CCGR4, 8
#define CCM_CCGR_SIM_EMS                         IMXRT_CCM_CCGR4, 7
#define CCM_CCGR_SIM_M                           IMXRT_CCM_CCGR4, 6
#define CCM_CCGR_TSC_DIG                         IMXRT_CCM_CCGR4, 5
#define CCM_CCGR_SIM_M7                          IMXRT_CCM_CCGR4, 4
#define CCM_CCGR_BEE                             IMXRT_CCM_CCGR4, 3
#define CCM_CCGR_IOMUXC_GPR                      IMXRT_CCM_CCGR4, 2
#define CCM_CCGR_IOMUXC                          IMXRT_CCM_CCGR4, 1

#define CCM_CCGR_SNVS_LP                         IMXRT_CCM_CCGR5, 15
#define CCM_CCGR_SNVS_HP                         IMXRT_CCM_CCGR5, 14
#define CCM_CCGR_LPUART7                         IMXRT_CCM_CCGR5, 13
#define CCM_CCGR_LPUART1                         IMXRT_CCM_CCGR5, 12
#define CCM_CCGR_SAI3                            IMXRT_CCM_CCGR5, 11
#define CCM_CCGR_SAI2                            IMXRT_CCM_CCGR5, 10
#define CCM_CCGR_SAI1                            IMXRT_CCM_CCGR5, 9
#define CCM_CCGR_SIM_MAIN                        IMXRT_CCM_CCGR5, 8
#define CCM_CCGR_SPDIF                           IMXRT_CCM_CCGR5, 7
#define CCM_CCGR_AIPSTZ4                         IMXRT_CCM_CCGR5, 6
#define CCM_CCGR_WDOG2                           IMXRT_CCM_CCGR5, 5
#define CCM_CCGR_KPP                             IMXRT_CCM_CCGR5, 4
#define CCM_CCGR_DMA                             IMXRT_CCM_CCGR5, 3
#define CCM_CCGR_WDOG3                           IMXRT_CCM_CCGR5, 2
#define CCM_CCGR_FLEXIO1                         IMXRT_CCM_CCGR5, 1
#define CCM_CCGR_ROM                             IMXRT_CCM_CCGR5, 0

#define CCM_CCGR_TIMER3                          IMXRT_CCM_CCGR6, 15
#define CCM_CCGR_TIMER2                          IMXRT_CCM_CCGR6, 14
#define CCM_CCGR_TIMER1                          IMXRT_CCM_CCGR6, 13
#define CCM_CCGR_LPI2C4_SERIAL                   IMXRT_CCM_CCGR6, 12
#define CCM_CCGR_ANADIG                          IMXRT_CCM_CCGR6, 11
#define CCM_CCGR_SIM_PER                         IMXRT_CCM_CCGR6, 10
#define CCM_CCGR_AIPS_TZ3                        IMXRT_CCM_CCGR6, 9
#define CCM_CCGR_TIMER4                          IMXRT_CCM_CCGR6, 8
#define CCM_CCGR_LPUART8                         IMXRT_CCM_CCGR6, 7
#define CCM_CCGR_TRNG                            IMXRT_CCM_CCGR6, 6
#define CCM_CCGR_FLEXSPI                         IMXRT_CCM_CCGR6, 5
#define CCM_CCGR_IPMUX4                          IMXRT_CCM_CCGR6, 4
#define CCM_CCGR_DCDC                            IMXRT_CCM_CCGR6, 3
#define CCM_CCGR_USDHC2                          IMXRT_CCM_CCGR6, 2
#define CCM_CCGR_USDHC1                          IMXRT_CCM_CCGR6, 1
#define CCM_CCGR_USBOH3                          IMXRT_CCM_CCGR6, 0

#define CCM_CCGR_FLEXIO                          IMXRT_CCM_CCGR7, 6
#define CCM_CCGR_AIPS                            IMXRT_CCM_CCGR7, 5
#define CCM_CCGR_CAN3_SERIAL                     IMXRT_CCM_CCGR7, 4
#define CCM_CCGR_CAN3                            IMXRT_CCM_CCGR7, 3
#define CCM_CCGR_AXBS                            IMXRT_CCM_CCGR7, 2
#define CCM_CCGR_FLEXSPI2                        IMXRT_CCM_CCGR7, 1
#define CCM_CCGR_ENET2                           IMXRT_CCM_CCGR7, 0

/* Module Enable Override Register */

                                                           /* Bits 0-4: Reserved */
#define CCM_CMEOR_MOD_EN_OV_GPT                  (1 << 5)  /* Bit 5:      Override clock enable signal from GPT */
#define CCM_CMEOR_MOD_EN_OV_PIT                  (1 << 6)  /* Bit 6:      Override clock enable signal from PIT */
#define CCM_CMEOR_MOD_EN_OV_USDHC                (1 << 7)  /* Bit 7:      Override clock enable signal from USDHC */
#define CCM_CMEOR_MOD_EN_OV_TRNG                 (1 << 9)  /* Bit 9:      Override clock enable signal from TRNG */
#define CCM_CMEOR_MOD_EN_OV_CANFD_CPI            (1 << 10) /* Bit 10:     Override clock enable signal from CAN3 */
                                                           /* Bits 11-27: Reserved */
#define CCM_CMEOR_MOD_EN_OV_CAN2_CPI             (1 << 28) /* Bit 28:     Override clock enable signal from CAN2 */
#define CCM_CMEOR_MOD_EN_OV_CAN1_CPI             (1 << 30) /* Bit 30:     Override clock enable signal from CAN1 */
                                                           /* Bit 31:     Reserved */

/* Analog ARM PLL control Register */

#define CCM_ANALOG_PLL_ARM_DIV_SELECT_SHIFT             (0)       /* Bits 0-6:   This field controls the PLL loop divider 54-108 */
#define CCM_ANALOG_PLL_ARM_DIV_SELECT_MASK              (0x7f << CCM_ANALOG_PLL_ARM_DIV_SELECT_SHIFT)
#  define CCM_ANALOG_PLL_ARM_DIV_SELECT(n)              ((uint32_t)(n) << CCM_ANALOG_PLL_ARM_DIV_SELECT_SHIFT)
                                                                  /* Bits 7-11   Reserved */
#define CCM_ANALOG_PLL_ARM_POWERDOWN                    (1 << 12) /* Bit 12:     Powers down the PLL */
#define CCM_ANALOG_PLL_ARM_ENABLE                       (1 << 13) /* Bit 13:     Enable the clock output */
#define CCM_ANALOG_PLL_ARM_BYPASS_CLK_SRC_SHIFT         (14)      /* Bits 14-15: Determines the bypass source */
#define CCM_ANALOG_PLL_ARM_BYPASS_CLK_SRC_MASK          (0x3 << CCM_ANALOG_PLL_ARM_BYPASS_CLK_SRC_SHIFT)
#  define CCM_ANALOG_PLL_ARM_BYPASS_CLK_SRC_REF_24M     ((uint32_t)(0) << CCM_ANALOG_PLL_ARM_BYPASS_CLK_SRC_SHIFT) /* Select 24Mhz Osc as source */
#  define CCM_ANALOG_PLL_ARM_BYPASS_CLK_SRC_CLK1        ((uint32_t)(1) << CCM_ANALOG_PLL_ARM_BYPASS_CLK_SRC_SHIFT) /* Select the CLK1_N / CLK1_P as source */
#define CCM_ANALOG_PLL_ARM_BYPASS                       (1 << 16)                                                  /* Bit 16:     Bypass the PLL */

                                                                  /* Bits 17-18  Reserved */
#define CCM_ANALOG_PLL_ARM_PLL_SEL                      (1 << 39) /* Bit 19:     ? */
#define CCM_ANALOG_PLL_ARM_LOCK                         (1 << 31) /* Bit 31:     PLL is currently locked */

/* Analog USB1 480MHz PLL Control Register */

#define CCM_ANALOG_PLL_USB1_DIV_SELECT_SHIFT            (1)       /* Bits 0-1:   This field controls the PLL loop divider 20 or 22 */
#define CCM_ANALOG_PLL_USB1_DIV_SELECT_MASK             (0x1 << CCM_ANALOG_PLL_USB1_DIV_SELECT_SHIFT)
#  define CCM_ANALOG_PLL_USB1_DIV_SELECT_20             ((uint32_t)(0) << CCM_ANALOG_PLL_USB1_DIV_SELECT_SHIFT)
#  define CCM_ANALOG_PLL_USB1_DIV_SELECT_22             ((uint32_t)(1) << CCM_ANALOG_PLL_USB1_DIV_SELECT_SHIFT)
                                                                  /* Bits 2-5    Reserved */
#define CCM_ANALOG_PLL_USB1_EN_USB_CLKS                 (1 << 6)  /* Bit 6:      Enable PLL outputs for USBPHYn */
#define CCM_ANALOG_PLL_USB1_POWER                       (1 << 12) /* Bit 12:     Powers up the PLL */
#define CCM_ANALOG_PLL_USB1_ENABLE                      (1 << 13) /* Bit 13:     Enable the PLL clock output */
#define CCM_ANALOG_PLL_USB1_BYPASS_CLK_SRC_SHIFT        (14)      /* Bits 14-15: Determines the bypass source */
#define CCM_ANALOG_PLL_USB1_BYPASS_CLK_SRC_MASK         (0x3 << CCM_ANALOG_PLL_USB1_BYPASS_CLK_SRC_SHIFT)
#  define CCM_ANALOG_PLL_USB1_BYPASS_CLK_SRC_REF_24M    ((uint32_t)(0) << CCM_ANALOG_PLL_USB1_BYPASS_CLK_SRC_SHIFT) /* Select 24Mhz Osc as source */
#  define CCM_ANALOG_PLL_USB1_BYPASS_CLK_SRC_CLK1       ((uint32_t)(1) << CCM_ANALOG_PLL_USB1_BYPASS_CLK_SRC_SHIFT) /* Select the CLK1_N / CLK1_P as source */
#  define CCM_ANALOG_PLL_USB1_BYPASS_CLK_SRC_GPANAIO    ((uint32_t)(2) << CCM_ANALOG_PLL_USB1_BYPASS_CLK_SRC_SHIFT) /*  */
#  define CCM_ANALOG_PLL_USB1_BYPASS_CLK_SRC_CHRG_DET_B ((uint32_t)(3) << CCM_ANALOG_PLL_USB1_BYPASS_CLK_SRC_SHIFT) /*  */
#define CCM_ANALOG_PLL_USB1_BYPASS                      (1 << 16)                                                   /* Bit 16:     Bypass the PLL */

                                                                  /* Bits 17-30  Reserved */
#define CCM_ANALOG_PLL_USB1_LOCK                        (1 << 31) /* Bit 31:     PLL is currently locked */

/* Analog USB2 480MHz PLL Control Register */

#define CCM_ANALOG_PLL_USB2_DIV_SELECT_SHIFT            (0)       /* Bits 0-1:   This field controls the PLL loop divider 20 or 22 */
#define CCM_ANALOG_PLL_USB2_DIV_SELECT_MASK             (0x3 << CCM_ANALOG_PLL_USB2_DIV_SELECT_SHIFT)
#  define CCM_ANALOG_PLL_USB2_DIV_SELECT_20             ((uint32_t)(0) << CCM_ANALOG_PLL_USB2_DIV_SELECT_SHIFT)
#  define CCM_ANALOG_PLL_USB2_DIV_SELECT_22             ((uint32_t)(1) << CCM_ANALOG_PLL_USB2_DIV_SELECT_SHIFT)
                                                                  /* Bits 2-5    Reserved */
#define CCM_ANALOG_PLL_USB2_EN_USB_CLKS                 (1 << 6)  /* Bit 6:      Enable PLL outputs for USBPHYn */
#define CCM_ANALOG_PLL_USB2_POWER                       (1 << 12) /* Bit 12:     Powers up the PLL */
#define CCM_ANALOG_PLL_USB2_ENABLE                      (1 << 13) /* Bit 13:     Enable the PLL clock output */
#define CCM_ANALOG_PLL_USB2_BYPASS_CLK_SRC_SHIFT        (14)      /* Bits 14-15: Determines the bypass source */
#define CCM_ANALOG_PLL_USB2_BYPASS_CLK_SRC_MASK         (0x3 << CCM_ANALOG_PLL_USB2_BYPASS_CLK_SRC_SHIFT)
#  define CCM_ANALOG_PLL_USB2_BYPASS_CLK_SRC_REF_24M    ((uint32_t)(0) << CCM_ANALOG_PLL_USB2_BYPASS_CLK_SRC_SHIFT) /* Select 24Mhz Osc as source */
#  define CCM_ANALOG_PLL_USB2_BYPASS_CLK_SRC_CLK1       ((uint32_t)(1) << CCM_ANALOG_PLL_USB2_BYPASS_CLK_SRC_SHIFT) /* Select the CLK1_N / CLK1_P as source */
#define CCM_ANALOG_PLL_USB2_BYPASS                      (1 << 16)                                                   /* Bit 16:     Bypass the PLL */

                                                                  /* Bits 17-30  Reserved */
#define CCM_ANALOG_PLL_USB2_LOCK                        (1 << 31) /* Bit 31:     PLL is currently locked */

/*  Analog System PLL Control Register */

#define CCM_ANALOG_PLL_SYS_DIV_SELECT_SHIFT             (0)       /* Bits 0-1:   This field controls the PLL loop divider 20 or 22 */
#define CCM_ANALOG_PLL_SYS_DIV_SELECT_MASK              (0x3 << CCM_ANALOG_PLL_SYS_DIV_SELECT_SHIFT)
#  define CCM_ANALOG_PLL_SYS_DIV_SELECT(n)              ((uint32_t)(n) << CCM_ANALOG_PLL_SYS_DIV_SELECT_SHIFT)
#  define CCM_ANALOG_PLL_SYS_DIV_SELECT_20              (0)
#  define CCM_ANALOG_PLL_SYS_DIV_SELECT_22              (1)
#define CCM_ANALOG_PLL_SYS_POWERDOWN                    (1 << 12) /* Bit 12:     Powers down the PLL */
#define CCM_ANALOG_PLL_SYS_ENABLE                       (1 << 13) /* Bit 13:     Enable the PLL clock output */
#define CCM_ANALOG_PLL_SYS_BYPASS_CLK_SRC_SHIFT         (14)      /* Bits 14-15: Determines the bypass source */
#define CCM_ANALOG_PLL_SYS_BYPASS_CLK_SRC_MASK          (0x3 << CCM_ANALOG_PLL_SYS_BYPASS_CLK_SRC_SHIFT)
#  define CCM_ANALOG_PLL_SYS_BYPASS_CLK_SRC_REF_24M     ((uint32_t)(0) << CCM_ANALOG_PLL_SYS_BYPASS_CLK_SRC_SHIFT) /* Select 24Mhz Osc as source */
#  define CCM_ANALOG_PLL_SYS_BYPASS_CLK_SRC_CLK1        ((uint32_t)(1) << CCM_ANALOG_PLL_SYS_BYPASS_CLK_SRC_SHIFT) /* Select the CLK1_N / CLK1_P as source */
#  define CCM_ANALOG_PLL_SYS_BYPASS_CLK_SRC_GPANAIO     ((uint32_t)(2) << CCM_ANALOG_PLL_SYS_BYPASS_CLK_SRC_SHIFT) /*  */
#  define CCM_ANALOG_PLL_SYS_BYPASS_CLK_SRC_CHRG_DET_B  ((uint32_t)(3) << CCM_ANALOG_PLL_SYS_BYPASS_CLK_SRC_SHIFT) /*  */
#define CCM_ANALOG_PLL_SYS_BYPASS                       (1 << 16)                                                  /* Bit 16:     Bypass the PLL */

                                                                  /* Bit 17:     Reserved */
#define CCM_ANALOG_PLL_SYS_PFD_OFFSET_EN                (1 << 18) /* Bit 18:     Enables an offset in the phase frequency detector */

                                                                  /* Bits 19-30  Reserved */
#define CCM_ANALOG_PLL_SYS_LOCK                         (1 << 31) /* Bit 31:     PLL is currently locked */

/* 528MHz System PLL Spread Spectrum Register */

#define CCM_ANALOG_PLL_SYS_SS_STEP_SHIFT                (0)       /* Bits 0-14:  Frequency change step */
#define CCM_ANALOG_PLL_SYS_SS_STEP_MASK                 (0x3FFF << CCM_ANALOG_PLL_SYS_SS_STEP_SHIFT)
#  define CCM_ANALOG_PLL_SYS_SS_STEP(n)                 ((uint32_t)(n) << CCM_ANALOG_PLL_SYS_SS_STEP_SHIFT)
#define CCM_ANALOG_PLL_SYS_SS_ENABLE                    (1 << 15) /* Bit 15:  Enable bit */
#define CCM_ANALOG_PLL_SYS_SS_STOP_SHIFT                (0)       /* Bits 16-31:  Frequency change */
#define CCM_ANALOG_PLL_SYS_SS_STOP_MASK                 (0xFFFF << CCM_ANALOG_PLL_SYS_SS_STOP_SHIFT)
#  define CCM_ANALOG_PLL_SYS_SS_STOP(n)                 ((uint32_t)(n) << CCM_ANALOG_PLL_SYS_SS_STOP_SHIFT)

/* Numerator of 528MHz System PLL Fractional Loop Divider Register */

#define CCM_ANALOG_PLL_SYS_NUM_A_SHIFT                  (0)       /* Bits 0-29:   30 bit numerator (A) */
#define CCM_ANALOG_PLL_SYS_NUM_A_MASK                   (0x3FFFFFFF << CCM_ANALOG_PLL_SYS_NUM_A_SHIFT)
#define CCM_ANALOG_PLL_SYS_NUM_A(n)                     ((uint32_t)(n) << CCM_ANALOG_PLL_SYS_NUM_A_SHIFT)
                                                                  /* Bits 30-31:  Reserved */

/* Denominator of 528MHz System PLL Fractional Loop Divider Register */

#define CCM_ANALOG_PLL_SYS_DENOM_B_SHIFT                (0)       /* Bits 0-29:   30 bit numerator (A) */
#define CCM_ANALOG_PLL_SYS_DENOM_B_MASK                 (0x3FFFFFFF << CCM_ANALOG_PLL_SYS_DENOM_B_SHIFT)
#define CCM_ANALOG_PLL_SYS_DENOM_B(n)                   ((uint32_t)(n) << CCM_ANALOG_PLL_SYS_DENOM_B_SHIFT)
                                                                  /* Bits 30-31:  Reserved */

/*  Analog Audio PLL control Register */

#define CCM_ANALOG_PLL_AUDIO_DIV_SELECT_SHIFT           (0)       /* Bits 0-6:    This field controls the PLL loop divider: 27-54 */
#define CCM_ANALOG_PLL_AUDIO_DIV_SELECT_MASK            (0x7f << CCM_ANALOG_PLL_AUDIO_DIV_SELECT_SHIFT)
#  define CCM_ANALOG_PLL_AUDIO_DIV_SELECT(n)            ((uint32_t)(n) << CCM_ANALOG_PLL_AUDIO_DIV_SELECT_SHIFT)
                                                                  /* Bits 7-11:  Reserved */
#define CCM_ANALOG_PLL_AUDIO_POWERDOWN                  (1 << 12) /* Bit 12:     Powers down the PLL */
#define CCM_ANALOG_PLL_AUDIO_ENABLE                     (1 << 13) /* Bit 13:     Enable PLL output */
#define CCM_ANALOG_PLL_AUDIO_BYPASS_CLK_SRC_SHIFT       (14)      /* Bits 14-15: Determines the bypass source */
#define CCM_ANALOG_PLL_AUDIO_BYPASS_CLK_SRC_MASK        (0x3 << CCM_ANALOG_PLL_AUDIO_BYPASS_CLK_SRC_SHIFT)
#  define CCM_ANALOG_PLL_AUDIO_BYPASS_CLK_SRC_REF_24M   ((uint32_t)(0) << CCM_ANALOG_PLL_AUDIO_BYPASS_CLK_SRC_SHIFT) /* Select 24Mhz Osc as source */
#  define CCM_ANALOG_PLL_AUDIO_BYPASS_CLK_SRC_CLK1      ((uint32_t)(1) << CCM_ANALOG_PLL_AUDIO_BYPASS_CLK_SRC_SHIFT) /* Select the CLK1_N / CLK1_P as source */
#define CCM_ANALOG_PLL_AUDIO_BYPASS                     (1 << 16)                                                    /* Bit 16:     Bypass the PLL */

                                                                  /* Bit 17:     Reserved */
#define CCM_ANALOG_PLL_AUDIO_PFD_OFFSET_EN              (1 << 18) /* Bit 18:     Enables an offset in the phase frequency detector */
#define CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT_SHIFT      (19)      /* Bits 19-20: These bits implement a divider after the PLL */
#define CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT_MASK       (0x7f << CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT_SHIFT)
#  define CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT_DIV4     ((uint32_t)(0) << CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT_SHIFT)
#  define CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT_DIV2     ((uint32_t)(1) << CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT_SHIFT)
#  define CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT_DIV1     ((uint32_t)(2) << CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT_SHIFT)
                                                                  /* Bits 21-30:  Reserved */
#define CCM_ANALOG_PLL_AUDIO_LOCK                       (1 << 31) /* Bit 31:     PLL is currently locked */

/* Numerator of Audio PLL Fractional Loop Divider Register */

#define CCM_ANALOG_PLL_AUDIO_NUM_A_SHIFT                (0)       /* Bits 0-29:   30 bit numerator (A) */
#define CCM_ANALOG_PLL_AUDIO_NUM_A_MASK                 (0x3FFFFFFF << CCM_ANALOG_PLL_AUDIO_NUM_A_SHIFT)
#define CCM_ANALOG_PLL_AUDIO_NUM_A(n)                   ((uint32_t)(n) << CCM_ANALOG_PLL_AUDIO_NUM_A_SHIFT)
/* Bits 30-31:  Reserved */

/* Denominator of Audio PLL Fractional Loop Divider Register */

#define CCM_ANALOG_PLL_AUDIO_DENOM_B_SHIFT              (0)       /* Bits 0-29:   30 bit numerator (A) */
#define CCM_ANALOG_PLL_AUDIO_DENOM_B_MASK               (0x3FFFFFFF << CCM_ANALOG_PLL_AUDIO_DENOM_B_SHIFT)
#define CCM_ANALOG_PLL_AUDIO_DENOM_B(n)                 ((uint32_t)(n) << CCM_ANALOG_PLL_AUDIO_DENOM_B_SHIFT)
                                                                  /* Bits 30-31:  Reserved */

/*  Analog Video PLL control Register */

#define CCM_ANALOG_PLL_VIDEO_DIV_SELECT_SHIFT           (0)       /* Bits 0-6:    This field controls the PLL loop divider: 27-54 */
#define CCM_ANALOG_PLL_VIDEO_DIV_SELECT_MASK            (0x7f << CCM_ANALOG_PLL_VIDEO_DIV_SELECT_SHIFT)
#  define CCM_ANALOG_PLL_VIDEO_DIV_SELECT(n)            ((uint32_t)(n) << CCM_ANALOG_PLL_VIDEO_DIV_SELECT_SHIFT)
                                                                  /* Bits 7-11:  Reserved */
#define CCM_ANALOG_PLL_VIDEO_POWERDOWN                  (1 << 12) /* Bit 12:     Powers down the PLL */
#define CCM_ANALOG_PLL_VIDEO_ENABLE                     (1 << 13) /* Bit 13:     Enable PLL output */
#define CCM_ANALOG_PLL_VIDEO_BYPASS_CLK_SRC_SHIFT       (14)      /* Bits 14-15: Determines the bypass source */
#define CCM_ANALOG_PLL_VIDEO_BYPASS_CLK_SRC_MASK        (0x3 << CCM_ANALOG_PLL_VIDEO_BYPASS_CLK_SRC_SHIFT)
#  define CCM_ANALOG_PLL_VIDEO_BYPASS_CLK_SRC_REF_24M   ((uint32_t)(0) << CCM_ANALOG_PLL_VIDEO_BYPASS_CLK_SRC_SHIFT) /* Select 24Mhz Osc as source */
#  define CCM_ANALOG_PLL_VIDEO_BYPASS_CLK_SRC_CLK1      ((uint32_t)(1) << CCM_ANALOG_PLL_VIDEO_BYPASS_CLK_SRC_SHIFT) /* Select the CLK1_N / CLK1_P as source */
#define CCM_ANALOG_PLL_VIDEO_BYPASS                     (1 << 16)                                                    /* Bit 16:     Bypass the PLL */

                                                                  /* Bit 17:     Reserved */
#define CCM_ANALOG_PLL_VIDEO_PFD_OFFSET_EN              (1 << 18) /* Bit 18:     Enables an offset in the phase frequency detector */
#define CCM_ANALOG_PLL_VIDEO_POST_DIV_SELECT_SHIFT      (19)      /* Bits 19-20: These bits implement a divider after the PLL */
#define CCM_ANALOG_PLL_VIDEO_POST_DIV_SELECT_MASK       (0x7f << CCM_ANALOG_PLL_VIDEO_POST_DIV_SELECT_SHIFT)
#  define CCM_ANALOG_PLL_VIDEO_POST_DIV_SELECT_DIV4     ((uint32_t)(0) << CCM_ANALOG_PLL_VIDEO_POST_DIV_SELECT_SHIFT)
#  define CCM_ANALOG_PLL_VIDEO_POST_DIV_SELECT_DIV2     ((uint32_t)(1) << CCM_ANALOG_PLL_VIDEO_POST_DIV_SELECT_SHIFT)
#  define CCM_ANALOG_PLL_VIDEO_POST_DIV_SELECT_DIV1     ((uint32_t)(2) << CCM_ANALOG_PLL_VIDEO_POST_DIV_SELECT_SHIFT)
                                                                  /* Bits 21-30:  Reserved */
#define CCM_ANALOG_PLL_VIDEO_LOCK                       (1 << 31) /* Bit 31:     PLL is currently locked */

/* Numerator of Video PLL Fractional Loop Divider Register */

#define CCM_ANALOG_PLL_VIDEO_NUM_A_SHIFT                (0)       /* Bits 0-29:   30 bit numerator (A) */
#define CCM_ANALOG_PLL_VIDEO_NUM_A_MASK                 (0x3FFFFFFF << CCM_ANALOG_PLL_VIDEO_NUM_A_SHIFT)
#define CCM_ANALOG_PLL_VIDEO_NUM_A(n)                   ((uint32_t)(n) << CCM_ANALOG_PLL_VIDEO_NUM_A_SHIFT)
                                                                  /* Bits 30-31:  Reserved */

/* Denominator of Video PLL Fractional Loop Divider Register */

#define CCM_ANALOG_PLL_VIDEO_DENOM_B_SHIFT              (0)       /* Bits 0-29:   30 bit numerator (A) */
#define CCM_ANALOG_PLL_VIDEO_DENOM_B_MASK               (0x3FFFFFFF << CCM_ANALOG_PLL_VIDEO_DENOM_B_SHIFT)
#define CCM_ANALOG_PLL_VIDEO_DENOM_B(n)                 ((uint32_t)(n) << CCM_ANALOG_PLL_VIDEO_DENOM_B_SHIFT)
                                                                  /* Bits 30-31:  Reserved */

/* Analog ENET PLL Control Register */

#define CCM_ANALOG_PLL_ENET_ENET0_DIV_SELECT_SHIFT      (0)       /* Bits 0-1:    Controls the frequency of the ethernet0 reference clock */
#define CCM_ANALOG_PLL_ENET_ENET0_DIV_SELECT_MASK       (0x3 << CCM_ANALOG_PLL_ENET_ENET0_DIV_SELECT_SHIFT)
#  define CCM_ANALOG_PLL_ENET_ENET0_DIV_SELECT_25MHZ    ((uint32_t)(0) << CCM_ANALOG_PLL_ENET_ENET0_DIV_SELECT_SHIFT)
#  define CCM_ANALOG_PLL_ENET_ENET0_DIV_SELECT_50MHZ    ((uint32_t)(1) << CCM_ANALOG_PLL_ENET_ENET0_DIV_SELECT_SHIFT)
#  define CCM_ANALOG_PLL_ENET_ENET0_DIV_SELECT_100MHZ   ((uint32_t)(2) << CCM_ANALOG_PLL_ENET_ENET0_DIV_SELECT_SHIFT)
#  define CCM_ANALOG_PLL_ENET_ENET0_DIV_SELECT_125MHZ   ((uint32_t)(3) << CCM_ANALOG_PLL_ENET_ENET0_DIV_SELECT_SHIFT)
#define CCM_ANALOG_PLL_ENET_ENET1_DIV_SELECT_SHIFT      (2)       /* Bits 0-1:    Controls the frequency of the ethernet1 reference clock */
#define CCM_ANALOG_PLL_ENET_ENET1_DIV_SELECT_MASK       (0x3 << CCM_ANALOG_PLL_ENET_ENET1_DIV_SELECT_SHIFT)
#  define CCM_ANALOG_PLL_ENET_ENET1_DIV_SELECT_25MHZ    ((uint32_t)(0) << CCM_ANALOG_PLL_ENET_ENET1_DIV_SELECT_SHIFT)
#  define CCM_ANALOG_PLL_ENET_ENET1_DIV_SELECT_50MHZ    ((uint32_t)(1) << CCM_ANALOG_PLL_ENET_ENET1_DIV_SELECT_SHIFT)
#  define CCM_ANALOG_PLL_ENET_ENET1_DIV_SELECT_100MHZ   ((uint32_t)(2) << CCM_ANALOG_PLL_ENET_ENET1_DIV_SELECT_SHIFT)
#  define CCM_ANALOG_PLL_ENET_ENET1_DIV_SELECT_125MHZ   ((uint32_t)(3) << CCM_ANALOG_PLL_ENET_ENET1_DIV_SELECT_SHIFT)
                                                                  /* Bits 4-11:  Reserved */
#define CCM_ANALOG_PLL_ENET_POWERDOWN                   (1 << 12) /* Bit 12:     Powers down the PLL */
#define CCM_ANALOG_PLL_ENET_ENET1_125M_EN               (1 << 13) /* Bit 13:     Enable the PLL providing the ENET1 125 MHz reference clock */
#define CCM_ANALOG_PLL_ENET_BYPASS_CLK_SRC_SHIFT       (14)       /* Bits 14-15: Determines the bypass source */
#define CCM_ANALOG_PLL_ENET_BYPASS_CLK_SRC_MASK        (0x3 << CCM_ANALOG_PLL_ENET_BYPASS_CLK_SRC_SHIFT)
#  define CCM_ANALOG_PLL_ENET_BYPASS_CLK_SRC_REF_24M   ((uint32_t)(0) << CCM_ANALOG_PLL_ENET_BYPASS_CLK_SRC_SHIFT) /* Select 24Mhz Osc as source */
#  define CCM_ANALOG_PLL_ENET_BYPASS_CLK_SRC_CLK1      ((uint32_t)(1) << CCM_ANALOG_PLL_ENET_BYPASS_CLK_SRC_SHIFT) /* Select the CLK1_N / CLK1_P as source */
#define CCM_ANALOG_PLL_ENET_BYPASS                     (1 << 16)                                                   /* Bit 16:     Bypass the PLL */

                                                                 /* Bit 17:     Reserved */
#define CCM_ANALOG_PLL_ENET_PFD_OFFSET_EN              (1 << 18) /* Bit 18:     Enables an offset in the phase frequency detector */
#define CCM_ANALOG_PLL_ENET_ENABLE_125M                (1 << 19) /* Bit 19:     */
#define CCM_ANALOG_PLL_ENET_ENET2_125M_EN              (1 << 20) /* Bit 20:     Enable the PLL providing the ENET2 125 MHz reference clock */
#define CCM_ANALOG_PLL_ENET_ENET_25M_REF_EN            (1 << 21) /* Bit 21:     Enable the PLL providing ENET 25 MHz reference clock */
#define CCM_ANALOG_PLL_ENET_ENET_500M_REF_EN           (1 << 22) /* Bit 22:     Enable the PLL providing NET 500 MHz reference clock */
#define CCM_ANALOG_PLL_ENET_LOCK                       (1 << 31) /* Bit 31:     PLL is currently locked */

/* 480MHz Clock (PLL3) Phase Fractional Divider Control Register */

#define CCM_ANALOG_PFD_480_PFD0_FRAC_SHIFT             (0)       /* Bits 0-5:   This field controls the fractional divide value */
#define CCM_ANALOG_PFD_480_PFD0_FRAC_MASK              (0x3f << CCM_ANALOG_PFD_480_PFD0_FRAC_SHIFT)
#  define CCM_ANALOG_PFD_480_PFD0_FRAC(n)              ((uint32_t)(n) << CCM_ANALOG_PFD_480_PFD0_FRAC_SHIFT)
#define CCM_ANALOG_PFD_480_PFD0_STABLE                 (6)       /* Bit 6:      */
#define CCM_ANALOG_PFD_480_PFD0_CLKGATE                (7)       /* Bit 7:      If set to 1, the IO fractional divider clock is off */
#define CCM_ANALOG_PFD_480_PFD1_FRAC_SHIFT             (8)       /* Bits 8-13:  This field controls the fractional divide value */
#define CCM_ANALOG_PFD_480_PFD1_FRAC_MASK              (0x3f << CCM_ANALOG_PFD_480_PFD1_FRAC_SHIFT)
#  define CCM_ANALOG_PFD_480_PFD1_FRAC(n)              ((uint32_t)(n) << CCM_ANALOG_PFD_481_PFD1_FRAC_SHIFT)
#define CCM_ANALOG_PFD_480_PFD1_STABLE                 (14)      /* Bit 14:     */
#define CCM_ANALOG_PFD_480_PFD1_CLKGATE                (15)      /* Bit 15:     If set to 1, the IO fractional divider clock is off */
#define CCM_ANALOG_PFD_480_PFD2_FRAC_SHIFT             (16)      /* Bits 8-21:  This field controls the fractional divide value */
#define CCM_ANALOG_PFD_480_PFD2_FRAC_MASK              (0x3f << CCM_ANALOG_PFD_480_PFD2_FRAC_SHIFT)
#  define CCM_ANALOG_PFD_480_PFD2_FRAC(n)              ((uint32_t)(n) << CCM_ANALOG_PFD_482_PFD2_FRAC_SHIFT)
#define CCM_ANALOG_PFD_480_PFD2_STABLE                 (22)      /* Bit 22:     */
#define CCM_ANALOG_PFD_480_PFD2_CLKGATE                (23)      /* Bit 23:     If set to 1, the IO fractional divider clock is off */
#define CCM_ANALOG_PFD_480_PFD3_FRAC_SHIFT             (24)      /* Bits 24-29:  This field controls the fractional divide value */
#define CCM_ANALOG_PFD_480_PFD3_FRAC_MASK              (0x3f << CCM_ANALOG_PFD_480_PFD3_FRAC_SHIFT)
#  define CCM_ANALOG_PFD_480_PFD3_FRAC(n)              ((uint32_t)(n) << CCM_ANALOG_PFD_482_PFD3_FRAC_SHIFT)
#define CCM_ANALOG_PFD_480_PFD3_STABLE                 (30)      /* Bit 30:     */
#define CCM_ANALOG_PFD_480_PFD3_CLKGATE                (31)      /* Bit 31:     If set to 1, the IO fractional divider clock is off */

/* 528MHz Clock (PLL2) Phase Fractional Divider Control */

#define CCM_ANALOG_PFD_528_PFD0_FRAC_SHIFT             (0)       /* Bits 0-5:   This field controls the fractional divide value */
#define CCM_ANALOG_PFD_528_PFD0_FRAC_MASK              (0x3f << CCM_ANALOG_PFD_528_PFD0_FRAC_SHIFT)
#  define CCM_ANALOG_PFD_528_PFD0_FRAC(n)              ((uint32_t)(n) << CCM_ANALOG_PFD_528_PFD0_FRAC_SHIFT)
#define CCM_ANALOG_PFD_528_PFD0_STABLE                 (6)       /* Bit 6:      */
#define CCM_ANALOG_PFD_528_PFD0_CLKGATE                (7)       /* Bit 7:      If set to 1, the IO fractional divider clock is off */
#define CCM_ANALOG_PFD_528_PFD1_FRAC_SHIFT             (8)       /* Bits 8-13:  This field controls the fractional divide value */
#define CCM_ANALOG_PFD_528_PFD1_FRAC_MASK              (0x3f << CCM_ANALOG_PFD_528_PFD1_FRAC_SHIFT)
#  define CCM_ANALOG_PFD_528_PFD1_FRAC(n)              ((uint32_t)(n) << CCM_ANALOG_PFD_481_PFD1_FRAC_SHIFT)
#define CCM_ANALOG_PFD_528_PFD1_STABLE                 (14)      /* Bit 14:     */
#define CCM_ANALOG_PFD_528_PFD1_CLKGATE                (15)      /* Bit 15:     If set to 1, the IO fractional divider clock is off */
#define CCM_ANALOG_PFD_528_PFD2_FRAC_SHIFT             (16)      /* Bits 8-21:  This field controls the fractional divide value */
#define CCM_ANALOG_PFD_528_PFD2_FRAC_MASK              (0x3f << CCM_ANALOG_PFD_528_PFD2_FRAC_SHIFT)
#  define CCM_ANALOG_PFD_528_PFD2_FRAC(n)              ((uint32_t)(n) << CCM_ANALOG_PFD_482_PFD2_FRAC_SHIFT)
#define CCM_ANALOG_PFD_528_PFD2_STABLE                 (22)      /* Bit 22:     */
#define CCM_ANALOG_PFD_528_PFD2_CLKGATE                (23)      /* Bit 23:     If set to 1, the IO fractional divider clock is off */
#define CCM_ANALOG_PFD_528_PFD3_FRAC_SHIFT             (24)      /* Bits 24-29:  This field controls the fractional divide value */
#define CCM_ANALOG_PFD_528_PFD3_FRAC_MASK              (0x3f << CCM_ANALOG_PFD_528_PFD3_FRAC_SHIFT)
#  define CCM_ANALOG_PFD_528_PFD3_FRAC(n)              ((uint32_t)(n) << CCM_ANALOG_PFD_482_PFD3_FRAC_SHIFT)
#define CCM_ANALOG_PFD_528_PFD3_STABLE                 (30)      /* Bit 30:     */
#define CCM_ANALOG_PFD_528_PFD3_CLKGATE                (31)      /* Bit 31:     If set to 1, the IO fractional divider clock is off */

/* Miscellaneous Register 0 */

#define CCM_ANALOG_MISC0_REFTOP_PWD                    (1 << 0)  /* Bit 0:      Control bit to power-down the analog bandgap reference circuitry */
                                                                 /* Bits 1-2:   Reserved */
#define CCM_ANALOG_MISC0_REFTOP_SELFBIASOFF            (1 << 2)  /* Bit 3:      Control bit to disable the self-bias circuit in the analog bandgap */
#define CCM_ANALOG_MISC0_REFTOP_VBGADJ_SHIFT           (4)       /* Bits 4-6:   VBG (PMU) */
#define CCM_ANALOG_MISC0_REFTOP_VBGADJ_MASK            (0x3 << CCM_ANALOG_MISC0_REFTOP_VBGADJ_SHIFT)
#  define CCM_ANALOG_MISC0_REFTOP_VBGADJ(n)            ((uint32_t)(n) << CCM_ANALOG_MISC0_REFTOP_VBGADJ_SHIFT)
#define CCM_ANALOG_MISC0_REFTOP_VBGUP                  (1 << 7)  /* Bit 7:      Status bit that signals the analog bandgap voltage is up and stable */
                                                                 /* Bits 8-9:   Reserved */
#define CCM_ANALOG_MISC0_STOP_MODE_CONFIG_SHIFT        (10)      /* Bits 10-11: Configure the analog behavior in stop mode */
#define CCM_ANALOG_MISC0_STOP_MODE_CONFIG_MASK         (0x3 << CCM_ANALOG_MISC0_STOP_MODE_CONFIG_SHIFT)
#  define CCM_ANALOG_MISC0_STOP_MODE_CONFIG(n)         ((uint32_t)(n) << CCM_ANALOG_MISC0_STOP_MODE_CONFIG_SHIFT)
#define CCM_ANALOG_MISC0_DISCON_HIGH_SNVS              (1 << 12) /* Bit 12:     This bit controls a switch from VDD_HIGH_IN to VDD_SNVS_IN */
#define CCM_ANALOG_MISC0_OSC_I_SHIFT                   (13)      /* Bits 13-14: This field determines the bias current in the 24MHz oscillator */
#define CCM_ANALOG_MISC0_OSC_I_MASK                    (0x3 << CCM_ANALOG_MISC0_OSC_I_SHIFT)
#  define CCM_ANALOG_MISC0_OSC_I(n)                    ((uint32_t)(n) << CCM_ANALOG_MISC0_OSC_I_SHIFT)
#define CCM_ANALOG_MISC0_OSC_XTALOK                    (1 << 15) /* Bit 15:     bit that signals 24-MHz crystal oscillator is stable */
#define CCM_ANALOG_MISC0_OSC_XTALOK_EN                 (1 << 16) /* Bit 16:     enables the detector that signals 24MHz crystal oscillator is stable */
                                                                 /* Bits 17-24: Reserved */
#define CCM_ANALOG_MISC0_CLKGATE_CTRL                  (1 << 25) /* Bit 25:     This bit allows disabling the clock gate */
#define CCM_ANALOG_MISC0_CLKGATE_DELAY_SHIFT           (26)      /* Bits 26-28: delay powering up the XTAL 24MHz */
#define CCM_ANALOG_MISC0_CLKGATE_DELAY_MASK            (0x7 << CCM_ANALOG_MISC0_CLKGATE_DELAY_SHIFT)
#  define CCM_ANALOG_MISC0_CLKGATE_DELAY(n)            ((uint32_t)(n) << CCM_ANALOG_MISC0_CLKGATE_DELAY_SHIFT)
#define CCM_ANALOG_MISC0_RTC_XTAL_SOURCE               (1 << 29) /* Bit 29:     which chip source is being used for the rtc clock */
#define CCM_ANALOG_MISC0_XTAL_24M_PWD                  (1 << 30) /* Bit 30:     power down the 24M crystal oscillator if set true */
#define CCM_ANALOG_MISC0_VID_PLL_PREDIV                (1 << 31) /* Bit 31:     Predivider for the source clock of the PLL's */
#define CCM_ANALOG_MISC0_VID_PLL_PREDIV_1              (0 << 31) /* Bit 31:     Predivider for the source clock of the PLL's */
#define CCM_ANALOG_MISC0_VID_PLL_PREDIV_2              (1 << 31) /* Bit 31:     Predivider for the source clock of the PLL's */

/* Miscellaneous Register 1 */

#define CCM_ANALOG_MISC1_LVDS1_CLK_SEL_SHIFT           (0)       /* Bits 0-4:   This field selects the clk to be routed to anaclk1/1b */
#define CCM_ANALOG_MISC1_LVDS1_CLK_SEL_MASK            (0x1f << CCM_ANALOG_MISC1_LVDS1_CLK_SEL_SHIFT)
#  define CCM_ANALOG_MISC1_LVDS1_CLK_SEL_ARM_PLL       ((uint32_t)(0) << CCM_ANALOG_MISC1_LVDS1_CLK_SEL_SHIFT)
#  define CCM_ANALOG_MISC1_LVDS1_CLK_SEL_SYS_PLL       ((uint32_t)(1) << CCM_ANALOG_MISC1_LVDS1_CLK_SEL_SHIFT)
#  define CCM_ANALOG_MISC1_LVDS1_CLK_SEL_PFD4          ((uint32_t)(2) << CCM_ANALOG_MISC1_LVDS1_CLK_SEL_SHIFT)
#  define CCM_ANALOG_MISC1_LVDS1_CLK_SEL_PFD5          ((uint32_t)(3) << CCM_ANALOG_MISC1_LVDS1_CLK_SEL_SHIFT)
#  define CCM_ANALOG_MISC1_LVDS1_CLK_SEL_PFD6          ((uint32_t)(4) << CCM_ANALOG_MISC1_LVDS1_CLK_SEL_SHIFT)
#  define CCM_ANALOG_MISC1_LVDS1_CLK_SEL_PFD7          ((uint32_t)(5) << CCM_ANALOG_MISC1_LVDS1_CLK_SEL_SHIFT)
#  define CCM_ANALOG_MISC1_LVDS1_CLK_SEL_AUDIO_PLL     ((uint32_t)(6) << CCM_ANALOG_MISC1_LVDS1_CLK_SEL_SHIFT)
#  define CCM_ANALOG_MISC1_LVDS1_CLK_SEL_VIDEO_PLL     ((uint32_t)(7) << CCM_ANALOG_MISC1_LVDS1_CLK_SEL_SHIFT)
#  define CCM_ANALOG_MISC1_LVDS1_CLK_SEL_ETHERNET_REF  ((uint32_t)(9) << CCM_ANALOG_MISC1_LVDS1_CLK_SEL_SHIFT)
#  define CCM_ANALOG_MISC1_LVDS1_CLK_SEL_USB1_PLL      ((uint32_t)(12) << CCM_ANALOG_MISC1_LVDS1_CLK_SEL_SHIFT)
#  define CCM_ANALOG_MISC1_LVDS1_CLK_SEL_USB2_PLL      ((uint32_t)(13) << CCM_ANALOG_MISC1_LVDS1_CLK_SEL_SHIFT)
#  define CCM_ANALOG_MISC1_LVDS1_CLK_SEL_PFD0          ((uint32_t)(14) << CCM_ANALOG_MISC1_LVDS1_CLK_SEL_SHIFT)
#  define CCM_ANALOG_MISC1_LVDS1_CLK_SEL_PFD1          ((uint32_t)(15) << CCM_ANALOG_MISC1_LVDS1_CLK_SEL_SHIFT)
#  define CCM_ANALOG_MISC1_LVDS1_CLK_SEL_PFD2          ((uint32_t)(16) << CCM_ANALOG_MISC1_LVDS1_CLK_SEL_SHIFT)
#  define CCM_ANALOG_MISC1_LVDS1_CLK_SEL_PFD3          ((uint32_t)(17) << CCM_ANALOG_MISC1_LVDS1_CLK_SEL_SHIFT)
#  define CCM_ANALOG_MISC1_LVDS1_CLK_SEL_XTAL          ((uint32_t)(18) << CCM_ANALOG_MISC1_LVDS1_CLK_SEL_SHIFT)
                                                                 /* Bits 5-9:   Reserved */
#define CCM_ANALOG_MISC1_LVDSCLK1_OBEN                 (1 << 10) /* Bit 10:     This enables the LVDS output buffer for anaclk1/1b */
                                                                 /* Bit 11:     Reserved */
#define CCM_ANALOG_MISC1_LVDSCLK1_IBEN                 (1 << 12) /* Bit 12:     This enables the LVDS input buffer for anaclk1/1b */
                                                                 /* Bits 13-15: Reserved */
#define CCM_ANALOG_MISC1_PFD_480_AUTOGATE_EN           (1 << 16) /* Bit 16:      */
#define CCM_ANALOG_MISC1_PFD_528_AUTOGATE_EN           (1 << 17) /* Bit 17:      */
                                                                 /* Bits 18-26: Reserved */
#define CCM_ANALOG_MISC1_IRQ_TEMPPANIC                 (1 << 27) /* Bit 27:     temperature sensor panic interrupt */
#define CCM_ANALOG_MISC1_IRQ_TEMPLOW                   (1 << 28) /* Bit 28:     temperature sensor low interrupt */
#define CCM_ANALOG_MISC1_IRQ_TEMPHIGH                  (1 << 29) /* Bit 29:     temperature sensor high interrupt */
#define CCM_ANALOG_MISC1_IRQ_ANA_BO                    (1 << 30) /* Bit 30:     analog regulator brownout interrupt */
#define CCM_ANALOG_MISC1_IRQ_DIG_BO                    (1 << 31) /* Bit 31:     digital regulator brownout interrupt */

/* Miscellaneous Register 2 */

#define CCM_ANALOG_MISC2_REG0_BO_OFFSET_SHIFT          (0)       /* Bits 0-2:   This field defines the brown out voltage offset */
#define CCM_ANALOG_MISC2_REG0_BO_OFFSET_MASK           (0x7 << CCM_ANALOG_MISC2_REG0_BO_OFFSET_SHIFT)
#  define CCM_ANALOG_MISC2_REG0_BO_OFFSET_0_100        ((uint32_t)(0) << CCM_ANALOG_MISC2_REG0_BO_OFFSET_SHIFT)
#  define CCM_ANALOG_MISC2_REG0_BO_OFFSET_0_175        ((uint32_t)(1) << CCM_ANALOG_MISC2_REG0_BO_OFFSET_SHIFT)
#define CCM_ANALOG_MISC2_REG0_BO_STATUS                (1 << 3)  /* Bit 3:      Reg0 brownout status bit */
                                                                 /* Bit 4:      Reserved */
#define CCM_ANALOG_MISC2_REG0_ENABLE_BO                (1 << 5)  /* Bit 5:      Enables the brownout detection */
#define CCM_ANALOG_MISC2_REG0_OK                       (1 << 6)  /* Bit 6:      ARM supply */
#define CCM_ANALOG_MISC2_PLL3_DISABLE                  (1 << 7)  /* Bit 7:      PLL3 can be disabled when the SoC is not in any low power mode */
#define CCM_ANALOG_MISC2_REG1_BO_OFFSET_SHIFT          (8)       /* Bits 8-10:  This field defines the brown out voltage offset */
#define CCM_ANALOG_MISC2_REG1_BO_OFFSET_MASK           (0x7 << CCM_ANALOG_MISC2_REG1_BO_OFFSET_SHIFT)
#  define CCM_ANALOG_MISC2_REG1_BO_OFFSET_0_100        ((uint32_t)(0) << CCM_ANALOG_MISC2_REG1_BO_OFFSET_SHIFT)
#  define CCM_ANALOG_MISC2_REG1_BO_OFFSET_0_175        ((uint32_t)(1) << CCM_ANALOG_MISC2_REG1_BO_OFFSET_SHIFT)
#define CCM_ANALOG_MISC2_REG1_BO_STATUS                (1 << 11) /* Bit 11:     Reg1 brownout status bit */
                                                                 /* Bit 12:     Reserved */
#define CCM_ANALOG_MISC2_REG1_ENABLE_BO                (1 << 13) /* Bit 13:     Enables the brownout detection */
#define CCM_ANALOG_MISC2_REG1_OK                       (1 << 14) /* Bit 14:     GPU/VPU supply */
#define CCM_ANALOG_MISC2_AUDIO_DIV_LSB                 (1 << 15) /* Bit 15:     LSB of Post-divider for Audio PLL */

#define CCM_ANALOG_MISC2_REG2_BO_OFFSET_SHIFT          (16)      /* Bits 16-18: This field defines the brown out voltage offset */
#define CCM_ANALOG_MISC2_REG2_BO_OFFSET_MASK           (0x7 << CCM_ANALOG_MISC2_REG2_BO_OFFSET_SHIFT)
#  define CCM_ANALOG_MISC2_REG2_BO_OFFSET_0_100        ((uint32_t)(0) << CCM_ANALOG_MISC2_REG2_BO_OFFSET_SHIFT)
#  define CCM_ANALOG_MISC2_REG2_BO_OFFSET_0_175        ((uint32_t)(1) << CCM_ANALOG_MISC2_REG2_BO_OFFSET_SHIFT)
#define CCM_ANALOG_MISC2_REG2_BO_STATUS                (1 << 19) /* Bit 19:     Reg2 brownout status bit */
                                                                 /* Bit 20:     Reserved */
#define CCM_ANALOG_MISC2_REG2_ENABLE_BO                (1 << 21) /* Bit 21:     Enables the brownout detection */
#define CCM_ANALOG_MISC2_REG2_OK                       (1 << 22) /* Bit 22:     voltage is above the brownout level for the SOC supply */
#define CCM_ANALOG_MISC2_AUDIO_DIV_MSB                 (1 << 23) /* Bit 23:     MSB of Post-divider for Audio PLL */
#define CCM_ANALOG_MISC2_REG0_STEP_TIME_SHIFT          (24)      /* Bits 24-25: Number of clock periods (24MHz clock) */
#define CCM_ANALOG_MISC2_REG0_STEP_TIME_MASK           (0x3 << CCM_ANALOG_MISC2_REG0_STEP_TIME_SHIFT)
#  define CCM_ANALOG_MISC2_REG0_STEP_TIME_64           ((uint32_t)(0) << CCM_ANALOG_MISC2_REG0_STEP_TIME_SHIFT)
#  define CCM_ANALOG_MISC2_REG0_STEP_TIME_128          ((uint32_t)(1) << CCM_ANALOG_MISC2_REG0_STEP_TIME_SHIFT)
#  define CCM_ANALOG_MISC2_REG0_STEP_TIME_256          ((uint32_t)(2) << CCM_ANALOG_MISC2_REG0_STEP_TIME_SHIFT)
#  define CCM_ANALOG_MISC2_REG0_STEP_TIME_512          ((uint32_t)(3) << CCM_ANALOG_MISC2_REG0_STEP_TIME_SHIFT)
#define CCM_ANALOG_MISC2_REG1_STEP_TIME_SHIFT          (26)      /* Bits 26-27: Number of clock periods (24MHz clock) */
#define CCM_ANALOG_MISC2_REG1_STEP_TIME_MASK           (0x3 << CCM_ANALOG_MISC2_REG1_STEP_TIME_SHIFT)
#  define CCM_ANALOG_MISC2_REG1_STEP_TIME_64           ((uint32_t)(0) << CCM_ANALOG_MISC2_REG1_STEP_TIME_SHIFT)
#  define CCM_ANALOG_MISC2_REG1_STEP_TIME_128          ((uint32_t)(1) << CCM_ANALOG_MISC2_REG1_STEP_TIME_SHIFT)
#  define CCM_ANALOG_MISC2_REG1_STEP_TIME_256          ((uint32_t)(2) << CCM_ANALOG_MISC2_REG1_STEP_TIME_SHIFT)
#  define CCM_ANALOG_MISC2_REG1_STEP_TIME_512          ((uint32_t)(3) << CCM_ANALOG_MISC2_REG1_STEP_TIME_SHIFT)
#define CCM_ANALOG_MISC2_REG2_STEP_TIME_SHIFT          (28)      /* Bits 28-29: Number of clock periods (24MHz clock) */
#define CCM_ANALOG_MISC2_REG2_STEP_TIME_MASK           (0x3 << CCM_ANALOG_MISC2_REG2_STEP_TIME_SHIFT)
#  define CCM_ANALOG_MISC2_REG2_STEP_TIME_64           ((uint32_t)(0) << CCM_ANALOG_MISC2_REG2_STEP_TIME_SHIFT)
#  define CCM_ANALOG_MISC2_REG2_STEP_TIME_128          ((uint32_t)(1) << CCM_ANALOG_MISC2_REG2_STEP_TIME_SHIFT)
#  define CCM_ANALOG_MISC2_REG2_STEP_TIME_256          ((uint32_t)(2) << CCM_ANALOG_MISC2_REG2_STEP_TIME_SHIFT)
#  define CCM_ANALOG_MISC2_REG2_STEP_TIME_512          ((uint32_t)(3) << CCM_ANALOG_MISC2_REG2_STEP_TIME_SHIFT)

#define CCM_ANALOG_MISC2_VIDEO_DIV_SHIFT               (30)      /* Bits 30-31: Post-divider for video */
#define CCM_ANALOG_MISC2_VIDEO_DIV_MASK                (0x3 << CCM_ANALOG_MISC2_VIDEO_DIV_SHIFT)
#  define CCM_ANALOG_MISC2_VIDEO_DIV(n)                ((uint32_t)(n) << CCM_ANALOG_MISC2_VIDEO_DIV_SHIFT)

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT106X_CCM_H */
