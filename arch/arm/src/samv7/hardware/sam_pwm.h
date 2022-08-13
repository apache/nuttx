/****************************************************************************
 * arch/arm/src/samv7/hardware/sam_pwm.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_FLEXPWM_H
#define __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_FLEXPWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define SAMV7_PWM_CLK       0x0000  /* Clock Register */
#define SAMV7_PWM_ENA       0x0004  /* Enable Register */
#define SAMV7_PWM_DIS       0x0008  /* Disable register */
#define SAMV7_PWM_SR        0x000c  /* Status Register */
#define SAMV7_PWM_IER1      0x0010  /* Interrupt Enable Register 1 */
#define SAMV7_PWM_IDR1      0x0014  /* Interrupt Disable Register 1 */
#define SAMV7_PWM_IMR1      0x0018  /* Interrupt Mask Register 1 */
#define SAMV7_PWM_ISR1      0x001c  /* Interrupt Status Register 1 */
#define SAMV7_PWM_SCM       0x0020  /* Sync Channels Mode Register */
#define SAMV7_PWM_DMAR      0x0024  /* DMA register */
#define SAMV7_PWM_SCUC      0x0028  /* Sync Channels Update Control Register */
#define SAMV7_PWM_SCUP      0x002c  /* Sync Channels Update Period Register */
#define SAMV7_PWM_SCUPUPDF  0x0030  /* Sync Channels Update Period Update Register */
#define SAMV7_PWM_IER2      0x0034  /* Interrupt Enable Register 2 */
#define SAMV7_PWM_IDR2      0x0038  /* Interrupt Disable Register 2 */
#define SAMV7_PWM_IMR2      0x003c  /* Interrupt Mask Register 2 */
#define SAMV7_PWM_ISR2      0x0040  /* Interrupt Status Register 2 */
#define SAMV7_PWM_OOV       0x0044  /* Output Override Value Register */
#define SAMV7_PWM_OS        0x0048  /* Output Selection Register */
#define SAMV7_PWM_OSS       0x004c  /* Output Selection Set Register */
#define SAMV7_PWM_OSC       0x0050  /* Output Selection Clear Register */
#define SAMV7_PWM_OSSUPD    0x0054  /* Output Selection Set Update Register */
#define SAMV7_PWM_OSCUPD    0x0058  /* Output Selection Clear Update Register */
#define SAMV7_PWM_FMR       0x005c  /* Fault Mode Register */
#define SAMV7_PWM_FSR       0x0060  /* Fault Status Register */
#define SAMV7_PWM_FCR       0x0064  /* Fault Clear Register */
#define SAMV7_PWM_FPV1      0x0068  /* Fault Protection Value Register 1 */
#define SAMV7_PWM_FPE       0x006c  /* Fault Protection Enable Register */
#define SAMV7_PWM_ELMR1     0x007c  /* Event Line 1 Mode Register */
#define SAMV7_PWM_ELMR2     0x0080  /* Event Line 2 Mode Register */
#define SAMV7_PWM_SSPR      0x00a0  /* Spread Spectrum Register */
#define SAMV7_PWM_SSPRUP    0x00a4  /* Spread Spectrum Update Register */
#define SAMV7_PWM_SMMR      0x00b0  /* Stepper Motor Mode Register */
#define SAMV7_PWM_FPV2      0x00c0  /* Fault Protection Value Register 2 */
#define SAMV7_PWM_WPCR      0x00e4  /* Write Protection Control Register */
#define SAMV7_PWM_WPSR      0x00e8  /* Write Protection Status Register */
#define SAMV7_PWM_CMPVX     0x0130  /* Comparison x Value Register */
#define SAMV7_PWM_CMPVUPDX  0x0134  /* Comparison x Value Update Register */
#define SAMV7_PWM_CMPMX     0x0138  /* Comparison x Mode Register */
#define SAMV7_PWM_CMPMUPDX  0x013c  /* Comparison x Mode Update Register */
#define SAMV7_PWM_CMRX      0x0200  /* Channel x Mode Register */
#define SAMV7_PWM_CDTYX     0x0204  /* Channel x Duty Cycle Register */
#define SAMV7_PWM_CDTYUPDX  0x0208  /* Channel x Duty Cycle Update Register */
#define SAMV7_PWM_CPRDX     0x020c  /* Channel x Period Register */
#define SAMV7_PWM_CPRDUPDX  0x0210  /* Channel x Period Update Register */
#define SAMV7_PWM_CCNTX     0x0214  /* Channel x Counter Register */
#define SAMV7_PWM_DTX       0x0218  /* Channel x Dead Time Register */
#define SAMV7_PWM_DTUPDX    0x021c  /* Channel x Dead Time Update Register */
#define SAMV7_PWM_CMUPDX    0x0400  /* Channel x Mode Update Register */
#define SAMV7_PWM_ETRG1     0x042c  /* External Trigger 1 Register */
#define SAMV7_PWM_LEBR1     0x0430  /* Leading-Edge Blanking 1 Register */
#define SAMV7_PWM_ETRG2     0x044c  /* External Trigger 2 Register */
#define SAMV7_PWM_LEBR2     0x0450  /* Leading-Edge Blanking 2 Register */

/* Register Bit Definitions *************************************************/

/* Clock Register */

#define CLK_DIVA_SHIFT             (0)    /* Bits: 0-7  CLKA Divide Factor */
#define CLK_DIVA_MASK              (0xff << CLK_DIVA_SHIFT)
#  define CLK_DIVA_SEL(n)          ((uint32_t)(n) << CLK_DIVA_SHIFT)
#  define CLK_DIVA_CLKA_POFF       (0 << CLK_DIVA_SHIFT)  /* CLKA clock is turned off */
#  define CLK_DIVA_PREA            (1 << CLK_DIVA_SHIFT)  /* CLKA clock is selected by PREA */

#define CLK_PREA_SHIFT             (8)    /* Bits: 8-11  CLKA Source Clock Selection */
#define CLK_PREA_MASK              (0x7 << CLK_PREA_SHIFT)
#  define CLK_PREA_SEL(n)          ((uint32_t)(n) << CLK_PREA_SHIFT)
#  define CLK_PREA_CLK             (0 << CLK_PREA_SHIFT)  /* Peripheral Clock */
#  define CLK_PREA_CLK_DIV2        (1 << CLK_PREA_SHIFT)  /* Peripheral Clock/2 */
#  define CLK_PREA_CLK_DIV4        (2 << CLK_PREA_SHIFT)  /* Peripheral Clock/4 */
#  define CLK_PREA_CLK_DIV8        (3 << CLK_PREA_SHIFT)  /* Peripheral Clock/8 */
#  define CLK_PREA_CLK_DIV16       (4 << CLK_PREA_SHIFT)  /* Peripheral Clock/16 */
#  define CLK_PREA_CLK_DIV32       (5 << CLK_PREA_SHIFT)  /* Peripheral Clock/32 */
#  define CLK_PREA_CLK_DIV64       (6 << CLK_PREA_SHIFT)  /* Peripheral Clock/64 */
#  define CLK_PREA_CLK_DIV128      (7 << CLK_PREA_SHIFT)  /* Peripheral Clock/128 */
#  define CLK_PREA_CLK_DIV256      (8 << CLK_PREA_SHIFT)  /* Peripheral Clock/256 */
#  define CLK_PREA_CLK_DIV512      (9 << CLK_PREA_SHIFT)  /* Peripheral Clock/512 */
#  define CLK_PREA_CLK_DIV1024     (10 << CLK_PREA_SHIFT) /* Peripheral Clock/1024 */

#define CLK_DIVB_SHIFT             (16)   /* Bits: 16-23  CLKB Divide Factor */
#define CLK_DIVB_MASK              (0xff << CLK_DIVB_SHIFT)
#  define CLK_DIVB_SEL(n)          ((uint32_t)(n) << CLK_DIVB_SHIFT)
#  define CLK_DIVB_CLKB_POFF       (0 << CLK_DIVB_SHIFT)  /* CLKB clock is turned off */
#  define CLK_DIVB_PREB            (1 << CLK_DIVB_SHIFT)  /* CLKB clock is selected by PREB */

#define CLK_PREB_SHIFT             (24)    /* Bits: 24-27  CLKB Source Clock Selection */
#define CLK_PREB_MASK              (0x7 << CLK_PREB_SHIFT)
#  define CLK_PREB_CLK             (0 << CLK_PREB_SHIFT)  /* Peripheral Clock */
#  define CLK_PREB_CLK_DIV2        (1 << CLK_PREB_SHIFT)  /* Peripheral Clock/2 */
#  define CLK_PREB_CLK_DIV4        (2 << CLK_PREB_SHIFT)  /* Peripheral Clock/4 */
#  define CLK_PREB_CLK_DIV8        (3 << CLK_PREB_SHIFT)  /* Peripheral Clock/8 */
#  define CLK_PREB_CLK_DIV16       (4 << CLK_PREB_SHIFT)  /* Peripheral Clock/16 */
#  define CLK_PREB_CLK_DIV32       (5 << CLK_PREB_SHIFT)  /* Peripheral Clock/32 */
#  define CLK_PREB_CLK_DIV64       (6 << CLK_PREB_SHIFT)  /* Peripheral Clock/64 */
#  define CLK_PREB_CLK_DIV128      (7 << CLK_PREB_SHIFT)  /* Peripheral Clock/128 */
#  define CLK_PREB_CLK_DIV256      (8 << CLK_PREB_SHIFT)  /* Peripheral Clock/256 */
#  define CLK_PREB_CLK_DIV512      (9 << CLK_PREB_SHIFT)  /* Peripheral Clock/512 */
#  define CLK_PREB_CLK_DIV1024     (10 << CLK_PREB_SHIFT) /* Peripheral Clock/1024 */
                                                          /* Bits: 27-31  Reserver */

/* Enable/Disable/Status Register */

#define CHID_SHIFT               (0)    /* Bits: 0-3  Channel ID Enable */
#define CHID_MASK                (0xf << CHID_SHIFT)
#  define CHID_SEL(n)            ((uint32_t)(n) << CHID_SHIFT)
                                        /* Bits 4-31  Reserved */

/* Interrupt Enable/Disable/Mask/Stats Register 1 */

#define IR1_CHID_SHIFT           (0)    /* Bits: 0-3  Counter Event on Channel x */
#define IR1_CHID_MASK            (0xf << IR1_CHID_SHIFT)
#  define IR1_CHID_SEL(n)        ((uint32_t)(n) << IR1_CHID_SHIFT)
                                        /* Bits 4-15  Reserved */
#define IR1_FCHID_SHIFT          (16)   /* Bits: 16-19  Fault Protection Trigger */
#define IR1_FCHID_MASK           (0xf << IR1_FCHID_SHIFT)
#  define IR1_FCHID_SEL(n)       ((uint32_t)(n) << IR1_FCHID_SHIFT)
                                         /* Bits 20-31  Reserved */

/* Sync Channels Mode Register */

#define SCM_SYNC_SHIFT            (0)    /* Bits: 0-3  Synchronous Channel x */
#define SCM_SYNC_MASK             (0xf << SCM_SYNC_SHIFT)
#  define SCM_SYNC_SEL(n)         ((uint32_t)(n) << SCM_SYNC_SHIFT)
                                         /* Bits 4-15  Reserved */
#define SCM_UPDM_SHIFT            (16)   /* Synchronous Channels Update Mode */
#define SCM_UPDM_MASK             (0x3 << SCM_UPDM_SHIFT)
#  define SCM_UPDM_SEL(n)         ((uint32_t)(n) << SCM_UPDM_SHIFT)
#  define SCM_UPDM_MODE0          (0 << SCM_UPDM_SHIFT)
#  define SCM_UPDM_MODE1          (1 << SCM_UPDM_SHIFT)
#  define SCM_UPDM_MODE2          (2 << SCM_UPDM_SHIFT)
#define SCM_PTRM                  (1 << 20) /* Bit: 20  DMA Controller Transfer Request Mode */
#define SCM_PTRCS_SHIFT           (21)      /* Bits: 21-23  DMA Controller Transfer Request Comp Sel x */
#define SCM_PTRCS_MASK            (0x7 << SCM_PTRCS_SHIFT)
#  define SCM_PTRCS_SEL(n)        ((uint32_t)(n) << SCM_PTRCS_SHIFT)

/* DMA Register */

#define DMAR_DMADUTY_SHIFT        (0)    /* Bits: 0-23  Duty Cycle Holding Register */
#define DMAR_DMADUTY_MASK         (0xffff << DMAR_DMADUTY_SHIFT)
#  define DMAR_DMADUTY_SEL(n)     ((uint32_t)(n) << DMAR_DMADUTY_SHIFT)

/* Sync Channels Update Control Register */

#define SCUC_UPDULOCK             (1 << 0)  /* Bit 0: Synchronous Channels Update Unlock */

/* Sync Channels Update Period Register */

#define SCUP_UPR_SHIFT            (0)    /* Bits: 0-3  Update Period */
#define SCUP_UPR_MASK             (0xf << SCUP_UPR_SHIFT)
#  define SCUP_UPR_SEL(n)         ((uint32_t)(n) << SCUP_UPR_SHIFT)
#define SCUP_UPRCNT_SHIFT         (4)    /* Bits: 4-7  Update Period */
#define SCUP_UPRCNT_MASK          (0xf << SCUP_UPRCNT_SHIFT)
#  define SCUP_UPRCNT_SEL(n)      ((uint32_t)(n) << SCUP_UPRCNT_SHIFT)

/* Sync Channels Update Period Update Register */

#define SCUPUPD_UPRUPD_SHIFT      (0)    /* Bits: 0-3  Update Period Update */
#define SCUPUPD_UPRUPD_MASK       (0xf << SCUPUPD_UPRUPD_SHIFT)
#  define SCUPUPD_UPRUPD_SEL(n)   ((uint32_t)(n) << SCUPUPD_UPRUPD_SHIFT)

/* Interrupt Enable/Disable/Mask/Status Register 2 */

#define IR2_WRDY                 (1 << 0)  /* Bit 0: Write Ready */
#define IR2_UNRE                 (1 << 3)  /* Bit 3: Underrun Error */
#define IR2_CMPM_SHIFT           (8)       /* Bits: 8-15: Comparison x Match */
#define IR2_CMPM_MASK            (0xff <<  IR2_CMPM_SHIFT)
#  define IR2_CMPM_SEL(n)        ((uint32_t)(n) << IR2_CMPM_SHIFT)
#define IR2_CMPU_SHIFT           (16)      /* Bits: 16-23: Comparison x Update */
#define IR2_CMPU_MASK            (0xff <<  IR2_CMPU_SHIFT)
#  define IR2_CMPU_SEL(n)        ((uint32_t)(n) << IR2_CMPU_SHIFT)

/* Output Register */

#define OR_OH_SHIFT            (0)    /* Bits: 0-4  Value for PWMH Output */
#define OR_OH_MASK             (0xf << OR_OH_SHIFT)
#  define OR_OH_SEL(n)         ((uint32_t)(n) << OR_OH_SHIFT)
#define OR_OL_SHIFT            (16)    /* Bits: 16-19  Value for PWML Output */
#define OR_OL_MASK             (0xf << OR_OL_SHIFT)
#  define OR_OL_SEL(n)         ((uint32_t)(n) << OR_OL_SHIFT)

/* Fault Mode Register */

#define FMR_FPOL_SHIFT         (0)    /* Bits: 0-7  Fault Polarity */
#define FMR_FPOL_MASK          (0xff << FMR_FPOL_SHIFT)
#  define FMR_FPOL_SEL(n)      ((uint32_t)(n) << FMR_FPOL_SHIFT)
#define FMR_FMOD_SHIFT         (8)    /* Bits: 8-15  Fault Activation Mode */
#define FMR_FMOD_MASK          (0xff << FMR_FMOD_SHIFT)
#  define FMR_FMOD_SEL(n)      ((uint32_t)(n) << FMR_FMOD_SHIFT)
#define FMR_FFIL_SHIFT         (16)    /* Bits: 16-23  Fault Filtering */
#define FMR_FFIL_MASK          (0xff << FMR_FFIL_SHIFT)
#  define FMR_FFIL_SEL(n)      ((uint32_t)(n) << FMR_FFIL_SHIFT)

/* Fault Status Register */

#define FSR_FIV_SHIFT          (0)    /* Bits: 0-7  Fault Input Value */
#define FSR_FIV_MASK           (0xff << FSR_FIV_SHIFT)
#  define FSR_FIV_SEL(n)       ((uint32_t)(n) << FSR_FIV_SHIFT)
#define FSR_FS_SHIFT           (8)    /* Bits: 8-15  Fault Status */
#define FSR_FS_MASK            (0xff << FSR_FS_SHIFT)
#  define FSR_FS_SEL(n)        ((uint32_t)(n) << FSR_FS_SHIFT)

/* Fault Clear Register */

#define FCR_FCLR_SHIFT        (0)    /* Bits: 0-7  Fault Clear */
#define FCR_FCLR_MASK         (0xff << FCR_FCLR_SHIFT)
#  define FCR_FCLR_SEL(n)     ((uint32_t)(n) << FCR_FCLR_SHIFT)

/* Fault Protection Value Register 1 */

#define FPV1_FPVH_SHIFT        (0)    /* Bits: 0-3  Fault Protection for PWMH */
#define FPV1_FPVH_MASK         (0xf << FPV1_FPVH_SHIFT)
#  define FPV1_FPVH_SEL(n)     ((uint32_t)(n) << FPV1_FPVH_SHIFT)
#define FPV1_FPVL_SHIFT        (16)    /* Bits: 16-20  Fault Protection for PWML */
#define FPV1_FPVL_MASK         (0xf << FPV1_FPVL_SHIFT)
#  define FPV1_FPVL_SEL(n)     ((uint32_t)(n) << FPV1_FPVL_SHIFT)

/* Fault Protection Enable Register */

#define FPE_FPE0_SHIFT         (0)    /* Bits: 0-7  Fault Protection for channel 0 */
#define FPE_FPE0_MASK          (0xff << FPE_FPE0_SHIFT)
#  define FPE_FPE0_SEL(n)      ((uint32_t)(n) << FPE_FPE0_SHIFT)
#define FPE_FPE1_SHIFT         (8)    /* Bits: 8-15  Fault Protection for channel 1 */
#define FPE_FPE1_MASK          (0xff << FPE_FPE1_SHIFT)
#  define FPE_FPE1_SEL(n)      ((uint32_t)(n) << FPE_FPE1_SHIFT)
#define FPE_FPE2_SHIFT         (16)   /* Bits: 16-23  Fault Protection for channel 2 */
#define FPE_FPE2_MASK          (0xff << FPE_FPE2_SHIFT)
#  define FPE_FPE2_SEL(n)      ((uint32_t)(n) << FPE_FPE2_SHIFT)
#define FPE_FPE3_SHIFT         (24)   /* Bits: 24-31  Fault Protection for channel 3 */
#define FPE_FPE3_MASK          (0xff << FPE_FPE3_SHIFT)
#  define FPE_FPE3_SEL(n)      ((uint32_t)(n) << FPE_FPE3_SHIFT)

/* Event Line Mode Register */

#define ELMR_CSEL_SHIFT        (0)    /* Bits: 0-7  Comparison Selection */
#define ELMR_CSEL_MASK         (0xff << ELMR_CSEL_SHIFT)
#  define ELMR_CSEL_SEL(n)     ((uint32_t)(n) << ELMR_CSEL_SHIFT)

/* Spread Spectrum Register */

#define SSPR_SPRD_SHIFT        (0)    /* Bits: 0-23  Spread Spectrum Limit Value */
#define SSPR_SPRD_MASK         (0xffff << SSPR_SPRD_SHIFT)
#  define SSPR_SPRD_SEL(n)     ((uint32_t)(n) << SSPR_SPRD_SHIFT)
#define SSPR_SPRDM             (1 << 24)  /* Bit 24: Spread Spectrum Counter Mode */

/* Spread Spectrum Update Register */

#define SSPUP_SPRDUP_SHIFT     (0)    /* Bits: 0-23  Spread Spectrum Limit Value Update */
#define SSPUP_SPRDUP_MASK      (0xffff << SSPUP_SPRDUP_SHIFT)
#  define SSPUP_SPRDUP_SEL(n)  ((uint32_t)(n) << SSPUP_SPRDUP_SHIFT)

/* Stepper Motor Mode Register */

#define SMMR_GCEN0             (1 << 0)   /* Bit 0: Enable gray count generation */
#define SMMR_GCEN1             (1 << 1)   /* Bit 1: Enable gray count generation */
#define SMMR_DOWN0             (1 << 16)  /* Bit 16: Down counter */
#define SMMR_DOWN1             (1 << 17)  /* Bit 17: Down counter */

/* Fault Protection Value Register 2 */

#define FPV2_FPZH_SHIFT        (0)    /* Bits: 0-3  Fault Protection for PWMH */
#define FPV2_FPZH_MASK         (0xf << FPV2_FPZH_SHIFT)
#  define FPV2_FPZH_SEL(n)     ((uint32_t)(n) << FPV2_FPZH_SHIFT)
#define FPV2_FPZL_SHIFT        (16)    /* Bits: 16-20  Fault Protection for PWML */
#define FPV2_FPZL_MASK         (0xf << FPV2_FPZL_SHIFT)
#  define FPV2_FPZL_SEL(n)     ((uint32_t)(n) << FPV2_FPZL_SHIFT)

/* Write Protection Control Register */

#define WPCR_WPRCMD_SHIFT      (0)    /* Bits 0-1: Write Protection Command */
#define WPCR_WPRCMD_MASK       (0x3 << WPCR_WPRCMD_SHIFT)
#  define WPCR_WPRCMD_SEL(n)   ((uint32_t)(n) << WPCR_WPRCMD_SHIFT)
#define WPCR_WPRG_SHIFT        (2)    /* Bits 2-7: Write Protection Register Group */
#define WPCR_WPRG_MASK         (0x3f << WPCR_WPRG_SHIFT)
#  define WPCR_WPRG_SEL(n)     ((uint32_t)(n) << WPCR_WPRG_SHIFT)
#define WPCR_WPKEY_SHIFT       (8)    /* Bits 8-31: Write Protection Key */
#define WPCR_WPKEY_MASK        (0xfff << WPCR_WPKEY_SHIFT)
#  define WPCR_WPKEY_SEL(n)    ((uint32_t)(n) << WPCR_WPKEY_SHIFT)

/* Write Protection Status Register */

#define WPSR_WPSWS_SHIFT       (0)    /* Bits 0-5: Write Protection SW status */
#define WPSR_WPSWS_MASK        (0x3f << WPSR_WPSWS_SHIFT)
#  define WPSR_WPSWS_SEL(n)    ((uint32_t)(n) << WPSR_WPSWS_SHIFT)
#define WPSR_WPVS              (1 << 7) /* Bit 7: Write Protection Violation Status */
#define WPSR_WPHWS_SHIFT       (8)      /* Bits 8-13: Write Protection HW status */
#define WPSR_WPHWS_MASK        (0x3f << WPSR_WPHWS_SHIFT)
#  define WPSR_WPHWS_SEL(n)    ((uint32_t)(n) << WPSR_WPHWS_SHIFT)
#define WPSR_WPVSRC_SHIFT      (16)      /* Bits 16-31: Write Violation Source */
#define WPSR_WPVSRC_MASK       (0x3f << WPSR_WPVSRC_SHIFT)
#  define WPSR_WPVSRC_SEL(n)   ((uint32_t)(n) << WPSR_WPVSRC_SHIFT)

/* Comparison Value Register */

#define CMPV_CV_SHIFT          (0)    /* Bits 0-23: Comparison Value */
#define CMPV_CV_MASK           (0xfff << CMPV_CV_SHIFT)
#  define CMPV_CV_SEL(n)       ((uint32_t)(n) << CMPV_CV_SHIFT)
#define CMPV_CVM               (1 << 24)  /* Bit 24: Comparison Value Mode */

/* Comparison Value Update Register */

#define CMPVUPD_CVUPD_SHIFT    (0)    /* Bits 0-23: Comparison Value Update */
#define CMPVUPD_CVUPD_MASK     (0xfff << CMPVUPD_CVUPD_SHIFT)
#  define CMPVUPD_CV_SEL(n)    ((uint32_t)(n) << CMPVUPD_CVUPD_SHIFT)
#define CMPVUPD_CVMUPD         (1 << 24)  /* Bit 24: Comparison Value Mode Update */

/* Comparison Mode Register */

#define CMPM_CEN               (1 << 0)   /* Bit 0: Comparison Enable */
#define CMPM_CTR_SHIFT         (4)        /* Bits 4-7: Comparison Trigger */
#define CMPM_CTR_MASK          (0xf << CMPM_CTR_SHIFT)
#  define CMPM_CTR_SEL(n)      ((uint32_t)(n) << CMPM_CTR_SHIFT)
#define CMPM_CPR_SHIFT         (8)        /* Bits 8-11: Comparison Period */
#define CMPM_CPR_MASK          (0xf << CMPM_CPR_SHIFT)
#  define CMPM_CPR_SEL(n)      ((uint32_t)(n) << CMPM_CPR_SHIFT)
#define CMPM_CPRCNT_SHIFT      (12)        /* Bits 12-15: Comparison Period Counter*/
#define CMPM_CPRCNT_MASK       (0xf << CMPM_CPRCNT_SHIFT)
#  define CMPM_CPRCNT_SEL(n)   ((uint32_t)(n) << CMPM_CPRCNT_SHIFT)
#define CMPM_CUPR_SHIFT        (16)        /* Bits 16-19: Comparison Update Period */
#define CMPM_CUPR_MASK         (0xf << CMPM_CUPR_SHIFT)
#  define CMPM_CUPR_SEL(n)     ((uint32_t)(n) << CMPM_CUPR_SHIFT)
#define CMPM_CUPRCNT_SHIFT     (20)        /* Bits 20-23: Comparison Update Period Counter */
#define CMPM_CUPRCNT_MASK      (0xf << CMPM_CUPRCNT_SHIFT)
#  define CMPM_CUPRCNT_SEL(n)  ((uint32_t)(n) << CMPM_CUPRCNT_SHIFT)

/* Comparison Mode Update Register */

#define CMPM_CENUPD            (1 << 0)   /* Bit 0: Comparison Enable Unable */
#define CMPM_CTRUPD_SHIFT      (4)        /* Bits 4-7: Comparison Trigger Update */
#define CMPM_CTRUPD_MASK       (0xf << CMPM_CTRUPD_SHIFT)
#  define CMPM_CTRUPD_SEL(n)   ((uint32_t)(n) << CMPM_CTRUPD_SHIFT)
#define CMPM_CPRUPD_SHIFT      (8)        /* Bits 8-11: Comparison Period Update */
#define CMPM_CPRUPD_MASK       (0xf << CMPM_CPRUPD_SHIFT)
#  define CMPM_CPRUPD_SEL(n)   ((uint32_t)(n) << CMPM_CPRUPD_SHIFT)
#define CMPM_CUPRUPD_SHIFT     (16)       /* Bits 16-19: Comparison Update Period Update */
#define CMPM_CUPRUPD_MASK      (0xf << CMPM_CUPRUPD_SHIFT)
#  define CMPM_CUPRUPD_SEL(n)  ((uint32_t)(n) << CMPM_CUPRUPD_SHIFT)

/* Channel Mode Register */

#define CMR_CPRE_SHIFT         (0)        /* Bits 0-3: Channel Prescaler */
#define CMR_CPRE_MASK          (0xf << CMR_CPRE_SHIFT)
#  define CMR_CPRE_SEL(n)      ((uint32_t)(n) << CMR_CPRE_SHIFT)
#define CMR_CALG               (1 << 8)   /* Bit  8: Channel Alignment */
#define CMR_CPOL               (1 << 9)   /* Bit  9: Channel Polarity */
#define CMR_CES                (1 << 10)  /* Bit 10: Counter Event Selection */
#define CMR_UPDS               (1 << 11)  /* Bit 11: Update Selection */
#define CMR_DPOLI              (1 << 12)  /* Bit 12: Disable Polarity Inverted */
#define CMR_TCTS               (1 << 13)  /* Bit 13: Timer Counter Trigger Selection */
#define CMR_DTE                (1 << 16)  /* Bit 16: Dead Time Generation Enable */
#define CMR_DTHI               (1 << 17)  /* Bit 17: Dead Time PWMH output Inverted */
#define CMR_DTLI               (1 << 18)  /* Bit 18: Dead Time PWML output Inverted */
#define CMR_PPM                (1 << 19)  /* Bit 19: Push Pull Mode */

/* Channel Duty Cycle Register */

#define CDTY_CDTY_SHIFT        (0)    /* Bits: 0-23  Duty Cycle */
#define CDTY_CDTY_MASK         (0xffff << CDTY_CDTY_SHIFT)
#  define CDTY_CDTY_SEL(n)     ((uint32_t)(n) << CDTY_CDTY_SHIFT)

/* Channel Duty Cycle Update Register */

#define CDTYUPD_CDTYUPD_SHIFT     (0)    /* Bits: 0-23  Duty Cycle Update */
#define CDTYUPD_CDTYUPD_MASK      (0xffff << CDTYUPD_CDTYUPD_SHIFT)
#  define CDTYUPD_CDTYUPD_SEL(n)  ((uint32_t)(n) << CDTYUPD_CDTYUPD_SHIFT)

/* Channel Period Register */

#define CPRD_CPRD_SHIFT        (0)    /* Bits: 0-23  Channel Period */
#define CPRD_CPRD_MASK         (0xffff << CPRD_CPRD_SHIFT)
#  define CPRD_CPRD_SEL(n)     ((uint32_t)(n) << CPRD_CPRD_SHIFT)

/* Channel Period Update Register */

#define CPRDUPD_CPRDUPD_SHIFT     (0)    /* Bits: 0-23  Channel Period Update */
#define CPRDUPD_CPRDUPD_MASK      (0xffff << CPRDUPD_CPRDUPD_SHIFT)
#  define CPRDUPD_CPRDUPD_SEL(n)  ((uint32_t)(n) << CPRDUPD_CPRDUPD_SHIFT)

/* Channel Counter Register */

#define CCNT_CNT_SHIFT        (0)    /* Bits: 0-23  Channel Counter */
#define CCNT_CNT_MASK         (0xffff << CCNT_CNT_SHIFT)
#  define CCNT_CNT_SEL(n)     ((uint32_t)(n) << CCNT_CNT_SHIFT)

/* Channel Dead Time Register */

#define DT_DTH_SHIFT          (0)    /* Bits: 0-15  Dead Time for PWMH */
#define DT_DTH_MASK           (0xff << DT_DTH_SHIFT)
#  define DT_DTH_SEL(n)       ((uint32_t)(n) << DT_DTH_SHIFT)
#define DT_DTL_SHIFT          (16)   /* Bits: 16-31  Dead Time for PWML */
#define DT_DTL_MASK           (0xff << DT_DTL_SHIFT)
#  define DT_DTL_SEL(n)       ((uint32_t)(n) << DT_DTL_SHIFT)

/* Channel Dead Time Update Register */

#define DTUPD_DTHUPD_SHIFT        (0)    /* Bits: 0-15  Dead Time Update for PWMH */
#define DTUPD_DTHUPD_MASK         (0xff << DTUPD_DTHUPD_SHIFT)
#  define DTUPD_DTHUPD_SEL(n)     ((uint32_t)(n) << DTUPD_DTHUPD_SHIFT)
#define DTUPD_DTLUPD_SHIFT        (16)   /* Bits: 16-31  Dead Time Update for PWML */
#define DTUPD_DTLUPD_MASK         (0xff << DTUPD_DTLUPD_SHIFT)
#  define DTUPD_DTLUPD_SEL(n)     ((uint32_t)(n) << DTUPD_DTLUPD_SHIFT)

/* Channel Mode Update Register */

#define CMUPD_CPOLUP          (1 << 9)  /* Bit 9: Channel Polarity Update */
#define CMUPD_CPOLINVUP       (1 << 13) /* Bit 9: Channel Polarity Inversion Update */

/* External Trigger Register */

#define ETRG_MAXCNT_SHIFT     (0)    /* Bits: 0-23  Maximum Counter Value */
#define ETRG_MAXCNT_MASK      (0xffff << ETRG_MAXCNT_SHIFT)
#  define ETRG_MAXCNT_SEL(n)  ((uint32_t)(n) << ETRG_MAXCNT_SHIFT)
#define ETRG_TRGMODE_SHIFT    (24)   /* Bits: 24-25  External Trigger Mode */
#define ETRG_TRGMODE_MASK     (0xffff << ETRG_TRGMODE_SHIFT)
#  define ETRG_TRGMODE_SEL(n) ((uint32_t)(n) << ETRG_TRGMODE_SHIFT)
#define ETRG_TRGEDGE          (1 << 28) /* Bit 28: Edge Selection */
#define ETRG_TRGFILT          (1 << 29) /* Bit 29: Filtered Input */
#define ETRG_TRGSRC           (1 << 30) /* Bit 30: Trigger Source */
#define ETRG_RFEN             (1 << 31) /* Bit 31: Recoverable Fault Update */

/* Leading Edge Blanking Register */

#define LEBR_LEBDELAY_SHIFT     (0)    /* Bits 0-6:  Leading Edge Blanking Delay */
#define LEBR_LEBDELAY_MASK      (0x3f << LEBR_LEBDELAY_SHIFT)
#  define LEBR_LEBDELAY_SEL(n)  ((uint32_t)(n) << LEBR_LEBDELAY_SHIFT)
#define LEBR_PWMLFEN            (1 << 16) /* Bit 16: PWML Falling Edge Enable */
#define LEBR_PWMLREN            (1 << 17) /* Bit 18: PWML Rising Edge Enable */
#define LEBR_PWMHFEN            (1 << 18) /* Bit 17: PWMH Falling Edge Enable */
#define LEBR_PWMHREN            (1 << 19) /* Bit 18: PWMH Rising Edge Enable */

#endif  /* CONFIG_SAMV7_PWM */
