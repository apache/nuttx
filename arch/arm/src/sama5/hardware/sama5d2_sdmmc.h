/************************************************************************************
 * arch/arm/src/sama5/hardware/sama5d2_sdmmc.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org> & Contributors
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

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE_SAMA5D2_SDMMC_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE_SAMA5D2_SDMMC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define SAMA5D2_SDMMC_SSAR_OFFSET          0x0000 /* SDMA System Address / Argument 2 Register */
#define SAMA5D2_SDMMC_BSR_OFFSET           0x0004 /* Block Size Register */
#define SAMA5D2_SDMMC_BCR_OFFSET           0x0006 /* Block Count Register */
#define SAMA5D2_SDMMC_ARG1R_OFFSET         0x0008 /* Argument 1 Register */
#define SAMA5D2_SDMMC_TMR_OFFSET           0x000c /* Transfer Mode Register */
#define SAMA5D2_SDMMC_CR_OFFSET            0x000e /* Command Register */
#define SAMA5D2_SDMMC_RR0_OFFSET           0x0010 /* Response Register 0 */
#define SAMA5D2_SDMMC_RR1_OFFSET           0x0014 /* Response Register 1 */
#define SAMA5D2_SDMMC_RR2_OFFSET           0x0018 /* Response Register 2 */
#define SAMA5D2_SDMMC_RR3_OFFSET           0x001c /* Response Register 3 */
#define SAMA5D2_SDMMC_BDPR_OFFSET          0x0020 /* Buffer Data Port Register */
#define SAMA5D2_SDMMC_PSR_OFFSET           0x0024 /* Present State Register */
#define SAMA5D2_SDMMC_HCR1_OFFSET          0x0028 /* Host Control 1 Register */
#define SAMA5D2_SDMMC_PR_OFFSET            0x0029 /* Power Control Register */
#define SAMA5D2_SDMMC_BGCR_OFFSET          0x002a /* Block Gap Control Register */
#define SAMA5D2_SDMMC_WCR_OFFSET           0x002b /* Wakeup Control Register */
#define SAMA5D2_SDMMC_CCR_OFFSET           0x002c /* Clock Control Register */
#define SAMA5D2_SDMMC_TCR_OFFSET           0x002e /* Timeout Control Register */
#define SAMA5D2_SDMMC_SRR_OFFSET           0x002f /* Software Reset Register */
#define SAMA5D2_SDMMC_NISTR_OFFSET         0x0030 /* Normal Interrupt Status Register */
#define SAMA5D2_SDMMC_EISTR_OFFSET         0x0032 /* Error Interrupt Status Register */
#define SAMA5D2_SDMMC_NISTER_OFFSET        0x0034 /* Normal Interrupt Status Enable Register */
#define SAMA5D2_SDMMC_EISTER_OFFSET        0x0036 /* Error Interrupt Status Enable Register */
#define SAMA5D2_SDMMC_NISIER_OFFSET        0x0038 /* Normal Interrupt Signal Enable Register */
#define SAMA5D2_SDMMC_EISIER_OFFSET        0x003a /* Error Interrupt Signal Enable Register */
#define SAMA5D2_SDMMC_ACESR_OFFSET         0x003c /* Auto CMD Error Status Register */
#define SAMA5D2_SDMMC_HC2R_OFFSET          0x003e /* Host Control 2 Register */
#define SAMA5D2_SDMMC_CA0R_OFFSET          0x0040 /* Capabilities 0 Register */
#define SAMA5D2_SDMMC_CA1R_OFFSET          0x0044 /* Capabilities 1 Register */
#define SAMA5D2_SDMMC_MCCAR_OFFSET         0x0048 /* Maximum Current Capabilities Register */
#define SAMA5D2_SDMMC_FERACES_OFFSET       0x0050 /* Force Event Register for Auto CMD Error Status */
#define SAMA5D2_SDMMC_FEREIS_OFFSET        0x0052 /* Force Event Register for Error Interrupt Status */
#define SAMA5D2_SDMMC_AESR_OFFSET          0x0054 /* ADMA Error Status Register */
#define SAMA5D2_SDMMC_ASAR0_OFFSET         0x0054 /* ADMA System Address Register 0 */
#define SAMA5D2_SDMMC_PVRX_OFFSET(x)       (0x60 + x * 0x02) /* Preset Value Register */
#define SAMA5D2_SDMMC_PVR0_OFFSET          SAMA5D2_SDMMC_PVRX_OFFSET(0) /* Initialization */
#define SAMA5D2_SDMMC_PVR1_OFFSET          SAMA5D2_SDMMC_PVRX_OFFSET(1) /* Default Speed */
#define SAMA5D2_SDMMC_PVR2_OFFSET          SAMA5D2_SDMMC_PVRX_OFFSET(2) /* High Speed */
#define SAMA5D2_SDMMC_PVR3_OFFSET          SAMA5D2_SDMMC_PVRX_OFFSET(3) /* SDR12 */
#define SAMA5D2_SDMMC_PVR4_OFFSET          SAMA5D2_SDMMC_PVRX_OFFSET(4) /* SDR25 */
#define SAMA5D2_SDMMC_PVR5_OFFSET          SAMA5D2_SDMMC_PVRX_OFFSET(5) /* SDR50 */
#define SAMA5D2_SDMMC_PVR6_OFFSET          SAMA5D2_SDMMC_PVRX_OFFSET(6) /* SDR104/HS200 */
#define SAMA5D2_SDMMC_PVR7_OFFSET          SAMA5D2_SDMMC_PVRX_OFFSET(7) /* DDR50 */
#define SAMA5D2_SDMMC_SISR_OFFSET          0x00fc /* Slot Interrupt Status Register */
#define SAMA5D2_SDMMC_HCVR_OFFSET          0x00fe /* Host Controller Version Register */
#define SAMA5D2_SDMMC_APSR_OFFSET          0x0200 /* Additional Present State Register */
#define SAMA5D2_SDMMC_MC1R_OFFSET          0x0204 /* e.MMC Control 1 Register */
#define SAMA5D2_SDMMC_MC2R_OFFSET          0x0204 /* e.MMC Control 2 Register */
#define SAMA5D2_SDMMC_ACR_OFFSET           0x0208 /* AHB Control Register */
#define SAMA5D2_SDMMC_CC2R_OFFSET          0x020c /* Clock Control 2 Register */
#define SAMA5D2_SDMMC_RTC1R_OFFSET         0x0210 /* Retuning Control 1 Register */
#define SAMA5D2_SDMMC_RTC2R_OFFSET         0x0211 /* Retuning Control 2 Register */
#define SAMA5D2_SDMMC_RTCVR_OFFSET         0x0214 /* Retuning Counter Value Register */
#define SAMA5D2_SDMMC_RTISTER_OFFSET       0x0218 /* Retuning Interrupt Status Enable Register */
#define SAMA5D2_SDMMC_RTISIER_OFFSET       0x0219 /* Retuning Interrupt Signal Enable Register */
#define SAMA5D2_SDMMC_RTISTR_OFFSET        0x021c /* Retuning Interrupt Status Register */
#define SAMA5D2_SDMMC_RTSSR_OFFSET         0x021d /* Retuning Status Slots Register */
#define SAMA5D2_SDMMC_TUNCR_OFFSET         0x0220 /* Tuning Control Register */
#define SAMA5D2_SDMMC_CACR_OFFSET          0x0230 /* Capabilities Control Register */
#define SAMA5D2_SDMMC_CALCR_OFFSET         0x0240 /* Calibration Control Register */

/* Register Addresses ***************************************************************/

/* For SDMMC0 ... */

#define SAMA5D2_SDMMC0_SSAR          (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_SSAR_OFFSET)
#define SAMA5D2_SDMMC0_BSR           (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_BSR_OFFSET)
#define SAMA5D2_SDMMC0_BCR           (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_BCR_OFFSET)
#define SAMA5D2_SDMMC0_ARG1R         (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_ARG1R_OFFSET)
#define SAMA5D2_SDMMC0_TMR           (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_TMR_OFFSET)
#define SAMA5D2_SDMMC0_CR            (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_CR_OFFSET)
#define SAMA5D2_SDMMC0_RR0           (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_RR0_OFFSET)
#define SAMA5D2_SDMMC0_RR1           (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_RR1_OFFSET)
#define SAMA5D2_SDMMC0_RR2           (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_RR2_OFFSET)
#define SAMA5D2_SDMMC0_RR3           (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_RR3_OFFSET)
#define SAMA5D2_SDMMC0_BDPR          (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_BDPR_OFFSET)
#define SAMA5D2_SDMMC0_PSR           (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_PSR_OFFSET)
#define SAMA5D2_SDMMC0_HCR1          (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_HCR1_OFFSET)
#define SAMA5D2_SDMMC0_PR            (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_PR_OFFSET)
#define SAMA5D2_SDMMC0_BGCR          (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_BGCR_OFFSET)
#define SAMA5D2_SDMMC0_WCR           (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_WCR_OFFSET)
#define SAMA5D2_SDMMC0_CCR           (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_CCR_OFFSET)
#define SAMA5D2_SDMMC0_TCR           (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_TCR_OFFSET)
#define SAMA5D2_SDMMC0_SRR           (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_SRR_OFFSET)
#define SAMA5D2_SDMMC0_NISTR         (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_NISTR_OFFSET)
#define SAMA5D2_SDMMC0_EISTR         (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_EISTR_OFFSET)
#define SAMA5D2_SDMMC0_NISTER        (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_NISTER_OFFSET)
#define SAMA5D2_SDMMC0_EISTER        (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_EISTER_OFFSET)
#define SAMA5D2_SDMMC0_NISIER        (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_NISIER_OFFSET)
#define SAMA5D2_SDMMC0_EISIER        (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_EISIER_OFFSET)
#define SAMA5D2_SDMMC0_ACESR         (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_ACESR_OFFSET)
#define SAMA5D2_SDMMC0_HC2R          (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_HC2R_OFFSET)
#define SAMA5D2_SDMMC0_CA0R          (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_CA0R_OFFSET)
#define SAMA5D2_SDMMC0_CA1R          (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_CA1R_OFFSET)
#define SAMA5D2_SDMMC0_MCCAR         (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_MCCAR_OFFSET)
#define SAMA5D2_SDMMC0_FERACES       (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_FERACES_OFFSET)
#define SAMA5D2_SDMMC0_FEREIS        (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_FEREIS_OFFSET)
#define SAMA5D2_SDMMC0_AESR          (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_AESR_OFFSET)
#define SAMA5D2_SDMMC0_ASAR0         (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_ASAR0_OFFSET)
#define SAMA5D2_SDMMC0_PVRX(x)       (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_PVRX_OFFSET(x))
#define SAMA5D2_SDMMC0_PVR0          (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_PVR0_OFFSET)
#define SAMA5D2_SDMMC0_PVR1          (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_PVR1_OFFSET)
#define SAMA5D2_SDMMC0_PVR2          (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_PVR2_OFFSET)
#define SAMA5D2_SDMMC0_PVR3          (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_PVR3_OFFSET)
#define SAMA5D2_SDMMC0_PVR4          (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_PVR4_OFFSET)
#define SAMA5D2_SDMMC0_PVR5          (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_PVR5_OFFSET)
#define SAMA5D2_SDMMC0_PVR6          (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_PVR6_OFFSET)
#define SAMA5D2_SDMMC0_PVR7          (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_PVR7_OFFSET)
#define SAMA5D2_SDMMC0_SISR          (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_SISR_OFFSET)
#define SAMA5D2_SDMMC0_HCVR          (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_HCVR_OFFSET)
#define SAMA5D2_SDMMC0_APSR          (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_APSR_OFFSET)
#define SAMA5D2_SDMMC0_MC1R          (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_MC1R_OFFSET)
#define SAMA5D2_SDMMC0_MC2R          (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_MC2R_OFFSET)
#define SAMA5D2_SDMMC0_ACR           (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_ACR_OFFSET)
#define SAMA5D2_SDMMC0_CC2R          (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_CC2R_OFFSET)
#define SAMA5D2_SDMMC0_RTC1R         (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_RTC1R_OFFSET)
#define SAMA5D2_SDMMC0_RTC2R         (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_RTC2R_OFFSET)
#define SAMA5D2_SDMMC0_RTCVR         (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_RTCVR_OFFSET)
#define SAMA5D2_SDMMC0_RTISTER       (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_RTISTER_OFFSET)
#define SAMA5D2_SDMMC0_RTISIER       (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_RTISIER_OFFSET)
#define SAMA5D2_SDMMC0_RTISTR        (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_RTISTR_OFFSET)
#define SAMA5D2_SDMMC0_RTSSR         (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_RTSSR_OFFSET)
#define SAMA5D2_SDMMC0_TUNCR         (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_TUNCR_OFFSET)
#define SAMA5D2_SDMMC0_CACR          (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_CACR_OFFSET)
#define SAMA5D2_SDMMC0_CALCR         (SAM_SDMMC0_VBASE + SAMA5D2_SDMMC_CALCR_OFFSET)

/* For SDMMC1 ... */

#define SAMA5D2_SDMMC1_SSAR          (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_SSAR_OFFSET)
#define SAMA5D2_SDMMC1_BSR           (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_BSR_OFFSET)
#define SAMA5D2_SDMMC1_BCR           (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_BCR_OFFSET)
#define SAMA5D2_SDMMC1_ARG1R         (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_ARG1R_OFFSET)
#define SAMA5D2_SDMMC1_TMR           (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_TMR_OFFSET)
#define SAMA5D2_SDMMC1_CR            (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_CR_OFFSET)
#define SAMA5D2_SDMMC1_RR0           (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_RR0_OFFSET)
#define SAMA5D2_SDMMC1_RR1           (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_RR1_OFFSET)
#define SAMA5D2_SDMMC1_RR2           (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_RR2_OFFSET)
#define SAMA5D2_SDMMC1_RR3           (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_RR3_OFFSET)
#define SAMA5D2_SDMMC1_BDPR          (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_BDPR_OFFSET)
#define SAMA5D2_SDMMC1_PSR           (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_PSR_OFFSET)
#define SAMA5D2_SDMMC1_HCR1          (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_HCR1_OFFSET)
#define SAMA5D2_SDMMC1_PR            (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_PR_OFFSET)
#define SAMA5D2_SDMMC1_BGCR          (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_BGCR_OFFSET)
#define SAMA5D2_SDMMC1_WCR           (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_WCR_OFFSET)
#define SAMA5D2_SDMMC1_CCR           (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_CCR_OFFSET)
#define SAMA5D2_SDMMC1_TCR           (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_TCR_OFFSET)
#define SAMA5D2_SDMMC1_SRR           (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_SRR_OFFSET)
#define SAMA5D2_SDMMC1_NISTR         (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_NISTR_OFFSET)
#define SAMA5D2_SDMMC1_EISTR         (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_EISTR_OFFSET)
#define SAMA5D2_SDMMC1_NISTER        (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_NISTER_OFFSET)
#define SAMA5D2_SDMMC1_EISTER        (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_EISTER_OFFSET)
#define SAMA5D2_SDMMC1_NISIER        (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_NISIER_OFFSET)
#define SAMA5D2_SDMMC1_EISIER        (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_EISIER_OFFSET)
#define SAMA5D2_SDMMC1_ACESR         (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_ACESR_OFFSET)
#define SAMA5D2_SDMMC1_HC2R          (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_HC2R_OFFSET)
#define SAMA5D2_SDMMC1_CA0R          (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_CA0R_OFFSET)
#define SAMA5D2_SDMMC1_CA1R          (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_CA1R_OFFSET)
#define SAMA5D2_SDMMC1_MCCAR         (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_MCCAR_OFFSET)
#define SAMA5D2_SDMMC1_FERACES       (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_FERACES_OFFSET)
#define SAMA5D2_SDMMC1_FEREIS        (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_FEREIS_OFFSET)
#define SAMA5D2_SDMMC1_AESR          (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_AESR_OFFSET)
#define SAMA5D2_SDMMC1_ASAR0         (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_ASAR0_OFFSET)
#define SAMA5D2_SDMMC1_PVRX(x)       (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_PVRX_OFFSET(x))
#define SAMA5D2_SDMMC1_PVR0          (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_PVR0_OFFSET)
#define SAMA5D2_SDMMC1_PVR1          (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_PVR1_OFFSET)
#define SAMA5D2_SDMMC1_PVR2          (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_PVR2_OFFSET)
#define SAMA5D2_SDMMC1_PVR3          (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_PVR3_OFFSET)
#define SAMA5D2_SDMMC1_PVR4          (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_PVR4_OFFSET)
#define SAMA5D2_SDMMC1_PVR5          (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_PVR5_OFFSET)
#define SAMA5D2_SDMMC1_PVR6          (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_PVR6_OFFSET)
#define SAMA5D2_SDMMC1_PVR7          (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_PVR7_OFFSET)
#define SAMA5D2_SDMMC1_SISR          (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_SISR_OFFSET)
#define SAMA5D2_SDMMC1_HCVR          (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_HCVR_OFFSET)
#define SAMA5D2_SDMMC1_APSR          (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_APSR_OFFSET)
#define SAMA5D2_SDMMC1_MC1R          (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_MC1R_OFFSET)
#define SAMA5D2_SDMMC1_MC2R          (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_MC2R_OFFSET)
#define SAMA5D2_SDMMC1_ACR           (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_ACR_OFFSET)
#define SAMA5D2_SDMMC1_CC2R          (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_CC2R_OFFSET)
#define SAMA5D2_SDMMC1_RTC1R         (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_RTC1R_OFFSET)
#define SAMA5D2_SDMMC1_RTC2R         (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_RTC2R_OFFSET)
#define SAMA5D2_SDMMC1_RTCVR         (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_RTCVR_OFFSET)
#define SAMA5D2_SDMMC1_RTISTER       (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_RTISTER_OFFSET)
#define SAMA5D2_SDMMC1_RTISIER       (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_RTISIER_OFFSET)
#define SAMA5D2_SDMMC1_RTISTR        (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_RTISTR_OFFSET)
#define SAMA5D2_SDMMC1_RTSSR         (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_RTSSR_OFFSET)
#define SAMA5D2_SDMMC1_TUNCR         (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_TUNCR_OFFSET)
#define SAMA5D2_SDMMC1_CACR          (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_CACR_OFFSET)
#define SAMA5D2_SDMMC1_CALCR         (SAM_SDMMC1_VBASE + SAMA5D2_SDMMC_CALCR_OFFSET)

/* Register Bit Definitions *********************************************************/

/* Block Size Register (0x04) */

#define SDMMC_BSR_BLKSIZE_SHIFT          (0)        /* Bits 0-9: Transfer Block Size */
#define SDMMC_BSR_BLKSIZE_MASK           (0x3ff << SDMMC_BSR_SIZE_SHIFT)
#  define SDMMC_BSR_BLKSIZE(n)           ((n & SDMMC_BSR_BLKSIZE_MASK) << SDMMC_BSR_BLKSIZE_SHIFT)

#define SDMMC_BSR_BOUNDARY_SHIFT         (12)       /* Bits 12-14: Buffer Boundary */
#  define SDMMC_BSR_BOUNDARY_4K          (0 << SDMMC_BSR_BOUNDARY_SHIFT)
#  define SDMMC_BSR_BOUNDARY_8K          (1 << SDMMC_BSR_BOUNDARY_SHIFT)
#  define SDMMC_BSR_BOUNDARY_16K         (2 << SDMMC_BSR_BOUNDARY_SHIFT)
#  define SDMMC_BSR_BOUNDARY_32K         (3 << SDMMC_BSR_BOUNDARY_SHIFT)
#  define SDMMC_BSR_BOUNDARY_64K         (4 << SDMMC_BSR_BOUNDARY_SHIFT)
#  define SDMMC_BSR_BOUNDARY_128K        (5 << SDMMC_BSR_BOUNDARY_SHIFT)
#  define SDMMC_BSR_BOUNDARY_256K        (6 << SDMMC_BSR_BOUNDARY_SHIFT)
#  define SDMMC_BSR_BOUNDARY_512K        (7 << SDMMC_BSR_BOUNDARY_SHIFT)

/* Block Count Register (0x06) */

#define SDMMC_BCR_BLKCNT_SHIFT           (0)        /* Bits 0-15: Blocks Count For Current Transfer */
#define SDMMC_BCR_BLKCNT_MASK            (0xffff << SDMMC_BCR_BLKCNT_SHIFT)
#  define SDMMC_BCR_BLKCNT(n)            ((n) << SDMMC_BCR_BLKCNT_SHIFT)

/* Transfer Mode Register (0x0c) */

#define SDMMC_TMR_MSBSEL                 (1 << 5)   /* Bit 5: Multi/Single Block Selection */
#define SDMMC_TMR_DTDSEL                 (1 << 4)   /* Bit 4: Data Transfer Direction Selection */
#define SDMMC_TMR_ACMDEN_SHIFT           (2)
#  define SDMMC_TMR_ACMDEN_DISABLED      (0 << SDMMC_TMR_ACMDEN_SHIFT)
#  define SDMMC_TMR_ACMDEN_CMD12         (1 << SDMMC_TMR_ACMDEN_SHIFT)
#  define SDMMC_TMR_ACMDEN_CMD23         (2 << SDMMC_TMR_ACMDEN_SHIFT)
#define SDMMC_TMR_BCEN                   (1 << 1)   /* Bit 1: Block Count Enable */
#define SDMMC_TMR_DMAEN                  (1 << 0)   /* Bit 0: DMA Enable */
#define SDMMC_TMR_SINGLE                 (0)
#define SDMMC_TMR_INFINITE               (SDMMC_TMR_MSBSEL)
#define SDMMC_TMR_MULTIPLE               (SDMMC_TMR_MSBSEL | SDMMC_TMR_BCEN)

/* Command Register (0x0e) */

#define SDMMC_CR_RSPTYP_SHIFT            (0)         /* Bits 0-1: Response Type Select */
#define SDMMC_CR_RSPTYP_MASK             (3 << SDMMC_CR_RSPTYP_SHIFT)
#  define SDMMC_CR_RSPTYP_NORESP         (0 << SDMMC_CR_RSPTYP_SHIFT) /* No response */
#  define SDMMC_CR_RSPTYP_RL136          (1 << SDMMC_CR_RSPTYP_SHIFT) /* Response length 136 */
#  define SDMMC_CR_RSPTYP_RL48           (2 << SDMMC_CR_RSPTYP_SHIFT) /* Response length 48 */
#  define SDMMC_CR_RSPTYP_RL48BUSY       (3 << SDMMC_CR_RSPTYP_SHIFT) /* Response length 48 with Busy */

#define SDMMC_CR_CMDCCCEN                (1 << 3)    /* Bit 3: Command CRC Check Enable */
#define SDMMC_CR_CMDCICEN                (1 << 4)    /* Bit 4: Command Index Check Enable */
#define SDMMC_CR_DPSEL                   (1 << 5)    /* Bit 5: Data Present Select */
#define SDMMC_CR_CMDTYP_SHIFT            (6)         /* Bits 6-7: Command Type */
#define SDMMC_CR_CMDTYP_MASK             (3 << SDMMC_CR_CMDTYP_SHIFT)
#  define SDMMC_CR_CMDTYP_NORMAL         (0 << SDMMC_CR_CMDTYP_SHIFT) /* Other commands */
#  define SDMMC_CR_CMDTYP_SUSPEND        (1 << SDMMC_CR_CMDTYP_SHIFT) /* CMD52 to write "Bus Suspend" in CCCR */
#  define SDMMC_CR_CMDTYP_RESUME         (2 << SDMMC_CR_CMDTYP_SHIFT) /* CMD52 to write "Function Select" in CCCR */
#  define SDMMC_CR_CMDTYP_ABORT          (3 << SDMMC_CR_CMDTYP_SHIFT) /* CMD12, CMD52 to write "I/O Abort" in CCCR */
#define SDMMC_CR_CMDINX_SHIFT            (8)         /* Bits 8-13: Command Index */
#define SDMMC_CR_CMDINX_MASK             (0x3f << SDMMC_CR_CMDINX_SHIFT)

/* Present State Register (0x24) */

#define SDMMC_PSR_CMDINHC                (1 << 0)     /* Bit 0:  Command Inhibit (CMD) */
#define SDMMC_PSR_CMDINHD                (1 << 1)     /* Bit 1:  Command Inhibit (DAT) */
#define SDMMC_PSR_DLACT                  (1 << 2)     /* Bit 2:  Data Line Active */
                                                      /* Bits 3-7: Reserved */
#define SDMMC_PSR_WTACT                  (1 << 8)     /* Bit 8:  Write Transfer Active */
#define SDMMC_PSR_RTACT                  (1 << 9)     /* Bit 9:  Read Transfer Active */
#define SDMMC_PSR_BUFWREN                (1 << 10)    /* Bit 10: Buffer Write Enable */
#define SDMMC_PSR_BUFRDEN                (1 << 11)    /* Bit 11: Buffer Read Enable */
                                                      /* Bits 12-15: Reserved */
#define SDMMC_PSR_CARDINS                (1 << 16)    /* Bit 16: Card Inserted */
#define SDMMC_PSR_CARDSS                 (1 << 17)    /* Bit 17: Card State Stable */
#define SDMMC_PSR_CARDDPL                (1 << 18)    /* Bit 18: Card Detect Pin Level */
#define SDMMC_PSR_WRPPL                  (1 << 19)    /* Bit 19: Write Protect Pin Level */
#define SDMMC_PSR_DATLL_SHIFT            (20)
#define SDMMC_PSR_DATLL_MASK             (0x0f << SDMMC_PSR_DATLL_SHIFT)
#  define SDMMC_PSR_DATLL_DAT0           (0x01 << SDMMC_PSR_DATLL_SHIFT)
#  define SDMMC_PSR_DATLL_DAT1           (0x02 << SDMMC_PSR_DATLL_SHIFT)
#  define SDMMC_PSR_DATLL_DAT2           (0x04 << SDMMC_PSR_DATLL_SHIFT)
#  define SDMMC_PSR_DATLL_DAT3           (0x08 << SDMMC_PSR_DATLL_SHIFT)
#define SDMMC_PSR_CMDLL                  (1 << 24)    /* Bit 24: CMD Line Level */

/* Host Control 1 Register (0x28) */

#define SDMMC_HC1R_LEDCTRL               (1 << 0)     /* Bit 0: LED Control */
#define SDMMC_HC1R_DW_SHIFT              (1)          /* Bit 1: Data Width */
#define SDMMC_HC1R_DW_MASK               (1 << SDMMC_HC1R_DW_SHIFT)
#  define SDMMC_HC1R_DW_1BIT             (0 << SDMMC_HC1R_DW_SHIFT) /* 1-bit mode */
#  define SDMMC_HC1R_DW_4BIT             (1 << SDMMC_HC1R_DW_SHIFT) /* 4-bit mode */
#define SDMMC_H1CR_HSEN                  (1 << 2)     /* Bit 2: High Speed Enable */
#define SDMMC_H1CR_DMASEL_SHIFT          (3)          /* Bits 3-4: DMA Select */
#define SDMMC_H1CR_DMASEL_MASK           (3 << SDMMC_DMASEL_SHIFT)
#  define SDMMC_H1CR_DMASEL_SDMA         (0 << SDMMC_DMASEL_SHIFT) /* SDMA is selected */
#  define SDMMC_H1CR_DMASEL_ADMA32       (2 << SDMMC_DMASEL_SHIFT) /* 32-bit Address ADMA2 is selected */
#define SDMMC_H1CR_EXTDW                 (1 << 5)     /* Bit 5:  Extended Data Width (e.MMC) */
#define SDMMC_H1CR_CARDDTL               (1 << 6)     /* Bit 6:  Card Detect Test Level */
#define SDMMC_H1CR_CARDDSEL              (1 << 7)     /* Bit 7:  Card Detect Signal Selection */

/* Power Control Register (0x29) */

#define SDMMC_PCR_SDBPWR                 (1 << 0)     /* Bit 0: SD Bus Power */

/* Block Gap Control Register (0x2a) */

#define SDMMC_BGCR_STPBGR                (1 << 0)     /* Bit 0: Stop At Block Gap Request */
#define SDMMC_BGCR_CONTR                 (1 << 1)     /* Bit 1: Continue Request */
#define SDMMC_BGCR_RWCTRL                (1 << 2)     /* Bit 2: Read Wait Control */
#define SDMMC_BGCR_INTBG                 (1 << 3)     /* Bit 3: Interrupt at Block Gap */

/* Wakeup Control Register (0x2b) */

#define SDMMC_WCR_WKENCINT               (1 << 0)     /* Bit 0: Wakeup Event Enable on Card Interrupt */
#define SDMMC_WCR_WKENCINS               (1 << 1)     /* Bit 1: Wakeup Event Enable on Card Insertion */
#define SDMMC_WCR_WKENCREM               (1 << 2)     /* Bit 2: Wakeup Event Enable on Card Removal */

/* Clock Control Register (0x2c) */

#define SDMMC_CCR_INTCLKEN               (1 << 0)     /* Bit 0: Internal Clock Enable */
#define SDMMC_CCR_INTCLKS                (1 << 1)     /* Bit 1: Internal Clock Stable */
#define SDMMC_CCR_SDCLKEN                (1 << 2)     /* Bit 2: SD Clock Enable */
#define SDMMC_CCR_CLKGSEL                (1 << 5)     /* Bit 5: Clock Generator Select */
#define SDMMC_CCR_USDCLKFSEL_SHIFT       (6)          /* Bits 6-7: Upper Bits of SDCLK Frequency Select */
#define SDMMC_CCR_USDCLKFSEL_MASK        (3 << SDMMC_CCR_USDCLKFSEL_SHIFT)
#define SDMMC_CCR_SDCLKFSEL_SHIFT        (8)          /* Bits 8-15: SDCLK Frequency Select */
#define SDMMC_CCR_SDCLKFSEL_MASK         (0xff << SDMMC_CCR_SDCLKFSEL_SHIFT)
#  define SDMMC_CCR_SDCLKFSEL(n)         (((n & 0xff) << SDMMC_CCR_SDCLKFSEL_SHIFT) | \
                                          ((n & 0x300) >> 2))
#  define SDMMC_CCR_SDCLKFSEL_VAL(reg)   (((reg & 0xff00) >> SDMMC_CCR_SDCLKFSEL_SHIFT) | \
                                          ((reg & 0x0030) << 2))

/* Timeout Control Register (0x2e) */

#define SDMMC_TCR_DTCVAL_SHIFT           (0)          /* Bits 0-3: Data Timeout Counter Value */
#define SDMMC_TCR_DTCVAL_MASK            (0x0f << SDMMC_TCR_DTCVAL_SHIFT)
#  define SDMMC_TCR_DTCVAL(n)            ((n & SDMMC_TCR_DTCVAL_MASK) << SDMMC_TCR_DTCVAL_SHIFT)

/* Software Reset Register (0x2f) */

#define SDMMC_SRR_SWRSTALL               (1 << 0)     /* Bit 0: Software Reset for All */
#define SDMMC_SRR_SWRSTCMD               (1 << 1)     /* Bit 1: Software Reset for CMD Line */
#define SDMMC_SRR_SWRSTDAT               (1 << 2)     /* Bit 2: Software Reset for DAT Line */

/* Normal Interrupt Status Register (0x30) */

#define SDMMC_NISTR_CMDC                 (1 << 0)     /* Bit 0: Command Complete */
#define SDMMC_NISTR_TRFC                 (1 << 1)     /* Bit 1: Transfer Complete */
#define SDMMC_NISTR_BLKGE                (1 << 2)     /* Bit 2: Block Gap Event */
#define SDMMC_NISTR_DMAINT               (1 << 3)     /* Bit 3: DMA Interrupt */
#define SDMMC_NISTR_BWRRDY               (1 << 4)     /* Bit 4: Buffer Write Ready */
#define SDMMC_NISTR_BRDRDY               (1 << 5)     /* Bit 5: Buffer Read Ready */
#define SDMMC_NISTR_CINS                 (1 << 6)     /* Bit 6: Card Insertion */
#define SDMMC_NISTR_CREM                 (1 << 7)     /* Bit 7: Card Removal */
#define SDMMC_NISTR_CINT                 (1 << 8)     /* Bit 8: Card Interrupt */
#define SDMMC_NISTR_BOOTAR               (1 << 14)    /* Bit 14: Boot Acknowledge Received (e.MMC) */
#define SDMMC_NISTR_ERRINT               (1 << 15)    /* Bit 15: Error Interrupt */

/* Error Interrupt Status Register (0x32) */

#define SDMMC_EISTR_CMDTEO               (1 << 0)     /* Bit 0: Command TImeout Error */
#define SDMMC_EISTR_CMDCRC               (1 << 1)     /* Bit 1: Command CRC Error */
#define SDMMC_EISTR_CMDEND               (1 << 2)     /* Bit 2: Command End Bit Error */
#define SDMMC_EISTR_CMDIDX               (1 << 3)     /* Bit 3: Command Index Error */
#define SDMMC_EISTR_DATTEO               (1 << 4)     /* Bit 4: Data Timeout Error */
#define SDMMC_EISTR_DATCRC               (1 << 5)     /* Bit 5: Data CRC Error */
#define SDMMC_EISTR_DATEND               (1 << 6)     /* Bit 6: Data End Bit Error */
#define SDMMC_EISTR_CURLIM               (1 << 7)     /* Bit 7: Current Limit Error */
#define SDMMC_EISTR_ACMD                 (1 << 8)     /* Bit 8: Auto CMD Error */
#define SDMMC_EISTR_ADMA                 (1 << 9)     /* Bit 9: ADMA Error */
#define SDMMC_EISTR_BOOTAE               (1 << 12)    /* Bit 12: Boot Acknowledge Error (e.MMC) */

/* Normal Interrupt Status Enable Register (0x34) */

#define SDMMC_NISTER_CMDC                (1 << 0)     /* Bit 0: Command Complete Status Enable */
#define SDMMC_NISTER_TRFC                (1 << 1)     /* Bit 1: Transfer Complete Status Enable */
#define SDMMC_NISTER_BLKGE               (1 << 2)     /* Bit 2: Block Gap Event Status Enable */
#define SDMMC_NISTER_DMAINT              (1 << 3)     /* Bit 3: DMA Interrupt Status Enable */
#define SDMMC_NISTER_BWRRDY              (1 << 4)     /* Bit 4: Buffer Write Ready Status Enable */
#define SDMMC_NISTER_BRDRDY              (1 << 5)     /* Bit 5: Buffer Read Ready Status Enable */
#define SDMMC_NISTER_CINS                (1 << 6)     /* Bit 6: Card Insertion Status Enable */
#define SDMMC_NISTER_CREM                (1 << 7)     /* Bit 7: Card Removal Status Enable */
#define SDMMC_NISTER_CINT                (1 << 8)     /* Bit 8: Card Interrupt Status Enable */
#define SDMMC_NISTER_BOOTAR              (1 << 14)    /* Bit 14: Boot Acknowledge Received Status Enable (e.MMC) */

/* Error Interrupt Status Enable Register (0x36) */

#define SDMMC_EISTER_CMDTEO              (1 << 0)     /* Bit 0: Command TImeout Error Status Enable */
#define SDMMC_EISTER_CMDCRC              (1 << 1)     /* Bit 1: Command CRC Error Status Enable */
#define SDMMC_EISTER_CMDEND              (1 << 2)     /* Bit 2: Command End Bit Error Status Enable */
#define SDMMC_EISTER_CMDIDX              (1 << 3)     /* Bit 3: Command Index Error Status Enable */
#define SDMMC_EISTER_DATTEO              (1 << 4)     /* Bit 4: Data Timeout Error Status Enable */
#define SDMMC_EISTER_DATCRC              (1 << 5)     /* Bit 5: Data CRC Error Status Enable */
#define SDMMC_EISTER_DATEND              (1 << 6)     /* Bit 6: Data End Bit Error Status Enable */
#define SDMMC_EISTER_CURLIM              (1 << 7)     /* Bit 7: Current Limit Error Status Enable */
#define SDMMC_EISTER_ACMD                (1 << 8)     /* Bit 8: Auto CMD Error Status Enable */
#define SDMMC_EISTER_ADMA                (1 << 9)     /* Bit 9: ADMA Error Status Enable */
#define SDMMC_EISTER_BOOTAE              (1 << 12)    /* Bit 12: Boot Acknowledge Error Status Enable (e.MMC) */

/* Normal Interrupt Signal Enable Register (0x38) */

#define SDMMC_NISIER_CMDC                (1 << 0)     /* Bit 0: Command Complete Signal Enable */
#define SDMMC_NISIER_TRFC                (1 << 1)     /* Bit 1: Transfer Complete Signal Enable */
#define SDMMC_NISIER_BLKGE               (1 << 2)     /* Bit 2: Block Gap Event Signal Enable */
#define SDMMC_NISIER_DMAINT              (1 << 3)     /* Bit 3: DMA Interrupt Signal Enable */
#define SDMMC_NISIER_BWRRDY              (1 << 4)     /* Bit 4: Buffer Write Ready Signal Enable */
#define SDMMC_NISIER_BRDRDY              (1 << 5)     /* Bit 5: Buffer Read Ready Signal Enable */
#define SDMMC_NISIER_CINS                (1 << 6)     /* Bit 6: Card Insertion Signal Enable */
#define SDMMC_NISIER_CREM                (1 << 7)     /* Bit 7: Card Removal Signal Enable */
#define SDMMC_NISIER_CINT                (1 << 8)     /* Bit 8: Card Interrupt Signal Enable */
#define SDMMC_NISIER_BOOTAR              (1 << 14)    /* Bit 14: Boot Acknowledge Received Signal Enable (e.MMC) */

/* Error Interrupt Signal Enable Register (0x3a) */

#define SDMMC_EISIER_CMDTEO              (1 << 0)     /* Bit 0: Command TImeout Error Signal Enable */
#define SDMMC_EISIER_CMDCRC              (1 << 1)     /* Bit 1: Command CRC Error Signal Enable */
#define SDMMC_EISIER_CMDEND              (1 << 2)     /* Bit 2: Command End Bit Error Signal Enable */
#define SDMMC_EISIER_CMDIDX              (1 << 3)     /* Bit 3: Command Index Error Signal Enable */
#define SDMMC_EISIER_DATTEO              (1 << 4)     /* Bit 4: Data Timeout Error Signal Enable */
#define SDMMC_EISIER_DATCRC              (1 << 5)     /* Bit 5: Data CRC Error Signal Enable */
#define SDMMC_EISIER_DATEND              (1 << 6)     /* Bit 6: Data End Bit Error Signal Enable */
#define SDMMC_EISIER_CURLIM              (1 << 7)     /* Bit 7: Current Limit Error Signal Enable */
#define SDMMC_EISIER_ACMD                (1 << 8)     /* Bit 8: Auto CMD Error Signal Enable */
#define SDMMC_EISIER_ADMA                (1 << 9)     /* Bit 9: ADMA Error Signal Enable */
#define SDMMC_EISIER_BOOTAE              (1 << 12)    /* Bit 12: Boot Acknowledge Error Signal Enable (e.MMC) */

/* Auto CMD Error Status Register (0x3c) */
#define SDMMC_ACESR_ACMD12NE             (1 << 0)     /* Bit 0: Auto CMD12 Not Executed */
#define SDMMC_ACESR_ACMDTER              (1 << 1)     /* Bit 1: Auto Command Timeout Error */
#define SDMMC_ACESR_ACMDCRC              (1 << 2)     /* Bit 2: Auto CMD CRC Error */
#define SDMMC_ACESR_ACMDEND              (1 << 3)     /* Bit 3: Auto CMD End Bit Error */
#define SDMMC_ACESR_ACMDIDX              (1 << 4)     /* Bit 4: Auto CMD Index Error */
#define SDMMC_ACESR_CMDNI                (1 << 7)     /* Bit 7: Command Not Issued by Auto CMD12 Error */

/* Host Control 2 Register (0x3e) */
#define SDMMC_HC2R_UHSMS_SHIFT           (0)          /* Bits 0-2: UHS Mode Select */
#define SDMMC_HC2R_UHSMS_MASK            (0x07 << SDMMC_HC2R_UHSMS_SHIFT)
#  define SDMMC_HC2R_UHSMS_SDR12         (0 << SDMMC_HC2R_UHSMS_SHIFT) /* UHS SDR12 Mode */
#  define SDMMC_HC2R_UHSMS_SDR25         (1 << SDMMC_HC2R_UHSMS_SHIFT) /* UHS SDR25 Mode */
#  define SDMMC_HC2R_UHSMS_SDR50         (2 << SDMMC_HC2R_UHSMS_SHIFT) /* UHS SDR50 Mode */
#  define SDMMC_HC2R_UHSMS_SDR104        (3 << SDMMC_HC2R_UHSMS_SHIFT) /* UHS SDR104 Mode */
#  define SDMMC_HC2R_UHSMS_DDR50         (4 << SDMMC_HC2R_UHSMS_SHIFT) /* UHS DDR50 Mode */
#define SDMMC_HC2R_HS200EN_SHIFT         (0)          /* Bits 0-3: HS200 Mode Enable (e.MMC) */
#define SDMMC_HC2R_HS200EN_MASK          (0x0f << SDMMC_HC2R_HS200EN_SHIFT)
#  define SDMMC_HC2R_HS200EN_ENABLE      (0x0b << SDMMC_HC2R_HS200EN_SHIFT) /* HS200 mode is enabled */
#define SDMMC_HC2R_VS18EN                (1 << 3)     /* Bit 3: 1.8V Signaling Enable */
#define SDMMC_HC2R_DRVSEL_SHIFT          (4)          /* Bits 4-5: Driver Strength Select */
#define SDMMC_HC2R_DRVSEL_MASK           (3 << SDMMC_HC2R_DRVSEL_SHIFT)
#  define SDMMC_HC2R_DRVSEL_TYPEB        (0 << SDMMC_HC2R_DRVSEL_SHIFT) /* Driver Type B is selected (Default) */
#  define SDMMC_HC2R_DRVSEL_TYPEA        (1 << SDMMC_HC2R_DRVSEL_SHIFT) /* Driver Type A is selected */
#  define SDMMC_HC2R_DRVSEL_TYPEC        (2 << SDMMC_HC2R_DRVSEL_SHIFT) /* Driver Type C is selected */
#  define SDMMC_HC2R_DRVSEL_TYPED        (3 << SDMMC_HC2R_DRVSEL_SHIFT) /* Driver Type D is selected */
#define SDMMC_HC2R_EXTUN                 (1 << 6)     /* Bit 6: Execute Tuning */
#define SDMMC_HC2R_SCLKSEL               (1 << 7)     /* Bit 7: Sampling Clock Select */
#define SDMMC_HC2R_ASINTEN               (1 << 14)    /* Bit 14: Asynchronous Interrupt Enable */
#define SDMMC_HC2R_PVALEN                (1 << 15)    /* Bit 15: Preset Value Enable */

/* Capabilities 0 Register (0x40) */

#define SDMMC_CA0R_TEOCLKF_SHIFT         (0)          /* Bits 0-5: Timeout Clock Frequency */
#define SDMMC_CA0R_TEOCLKF_MASK          (0x3f << SDMMC_CA0R_TEOCLKF_SHIFT)
#define   SDMMC_CA0R_TEOCLKF(n)          ((n &0x3f) << SDMMC_CA0R_TEOCLKF_SHIFT)
#define SDMMC_CA0R_TEOCLKU               (1 << 7)     /* Bit 7: Timeout Clock Unit */
#define   SDMMC_CA0R_TEOCLKU_KHz         (0 << 7)     /*        KHz */
#define   SDMMC_CA0R_TEOCLKU_MHz         (1 << 7)     /*        MHz */
#define SDMMC_CA0R_BASECLKF_SHIFT        (8)          /* Bits 8-15: Base Clock Frequency */
#define SDMMC_CA0R_BASECLKF_MASK         (0xf << SDMMC_BASECLKF_SHIFT)
#define   SDMMC_CA0R_BASECLKF(n)         ((n & 0xf) << SDMMC_BASECLKF_SHIFT)
#define SDMMC_CA0R_MAXBLKL_SHIFT         (16)         /* Bits 16-17: Max Block Length */
#define SDMMC_CA0R_MAXBLKL_MASK          (3 << SDMMC_CA0R_MAXBLKL_SHIFT)
#define   SDMMC_CA0R_MAXBLKL_512         (0 << SDMMC_CA0R_MAXBLKL_SHIFT) /* 512 bytes */
#define   SDMMC_CA0R_MAXBLKL_1024        (1 << SDMMC_CA0R_MAXBLKL_SHIFT) /* 1024 bytes */
#define   SDMMC_CA0R_MAXBLKL_2048        (2 << SDMMC_CA0R_MAXBLKL_SHIFT) /* 2048 bytes */
#define   SDMMC_CA0R_MAXBLKL_NONE        (3 << SDMMC_CA0R_MAXBLKL_SHIFT) /* Reserved */
#define SDMMC_CA0R_ED8SUP                (1 << 18)    /* Bit 18: 8-Bit Support for Embedded Device */
#define SDMMC_CA0R_ADMA2SUP              (1 << 19)    /* Bit 19: ADMA2 Support */
#define SDMMC_CA0R_HSSUP                 (1 << 21)    /* Bit 21: High Speed Support */
#define SDMMC_CA0R_SDMASUP               (1 << 22)    /* Bit 22: SDMA Support */
#define SDMMC_CA0R_SRSUP                 (1 << 23)    /* Bit 23: Suspend/Resume Support */
#define SDMMC_CA0R_V33VSUP               (1 << 24)    /* Bit 24: Voltable Support 3.3V */
#define SDMMC_CA0R_V30VSUP               (1 << 25)    /* Bit 25: Voltable Support 3.0V */
#define SDMMC_CA0R_V18VSUP               (1 << 26)    /* Bit 26: Voltable Support 1.8V */
#define SDMMC_CA0R_SB64SUP               (1 << 28)    /* Bit 28: 64-Bit System Bus Support */
#define SDMMC_CA0R_ASINTSUP              (1 << 29)    /* Bit 29: Asynchronous Interrupt Support */
#define SDMMC_CA0R_SLTYPE_SHIFT          (30)         /* Bits 30-31: Slot Type */
#define SDMMC_CA0R_SLTYPE_MASK           (3 << SDMMC_CA0R_SLTYPE_SHIFT)
#define   SDMMC_CA0R_SLTYPE_REMOVABLE    (0 << SDMMC_CA0R_SLTYPE_SHIFT) /* Removable Card Slot */
#define   SDMMC_CA0R_SLTYPE_EMBEDDED     (1 << SDMMC_CA0R_SLTYPE_SHIFT) /* Embedded Slot for One Device */

/* Capabilities 1 Register (0x44) */

#define SDMMC_CA1R_SDR50SUP              (1 << 0)     /* Bit 0: SDR50 Support */
#define SDMMC_CA1R_SDR104SUP             (1 << 1)     /* Bit 1: SDR104 Support */
#define SDMMC_CA1R_DDR50SUP              (1 << 2)     /* Bit 2: DDR50 Support */
#define SDMMC_CA1R_DRVASUP               (1 << 4)     /* Bit 4: Driver Type A Support */
#define SDMMC_CA1R_DRVCSUP               (1 << 5)     /* Bit 5: Driver Type C Support */
#define SDMMC_CA1R_DRVDSUP               (1 << 6)     /* Bit 6: Driver Type D Support */
#define SDMMC_CA1R_TCNTRT_SHIFT          (8)          /* Bits 8-11: Timer Count For Retuning */
#define SDMMC_CA1R_TCNTRT_MASK           (0x0f << SDMMC_CA1R_TCNTRT_SHIFT)
#define   SDMMC_CA1R_TCNTRT(n)           ((n & 0x0f) << SDMMC_CA1R_TCNTRT_SHIFT)
#define SDMMC_CA1R_TSDR50                (1 << 13)    /* Bit 13: Use Tuning for SDR50 */
#define SDMMC_CA1R_RTMOD_SHIFT           (14)         /* Bits 14-15: Retuning Modes */
#define SDMMC_CA1R_RTMOD_MASK            (3 << SDMMC_CA1R_RTMOD_SHIFT)
#define   SDMMC_CA1R_RTMOD_MODE1         (0 << SDMMC_CA1R_RTMOD_SHIFT) /* MODE1: Timer */
#define   SDMMC_CA1R_RTMOD_MODE2         (1 << SDMMC_CA1R_RTMOD_SHIFT) /* MODE2: Timer and Retuning Request */
#define   SDMMC_CA1R_RTMOD_MODE3         (2 << SDMMC_CA1R_RTMOD_SHIFT) /* MODE3: Auto Retuning Timer and Retuning Request */
#define SDMMC_CA1R_CLKMULT_SHIFT         (16)         /* Bits 16-23: Clock Multiplier */
#define SDMMC_CA1R_CLKMULT_MASK          (0xf << SDMMC_CA1R_CLKMULT_SHIFT)
#define   SDMMC_CA1R_CLKMULT(n)          ((n & 0xf) << SDMMC_CA1R_CLKMULT_SHIFT)

/* Preset Value Register (0x60 + x * 0x02 [x=0..7]) */

#define SDMMC_PVRx_SDCLKFSEL_SHIFT       (0)          /* Bits 0-9: SDCLK Frequency Select */
#define SDMMC_PVRx_SDCLKFSEL_MASK        (0x3f << SDMMC_PVRx_SDCLKFSEL_SHIFT)
#define   SDMMC_PVRx_SDCLKFSEL(n)        ((n & 0x3f) << SDMMC_PVRx_SDCLKFSEL_SHIFT)
#define SDMMC_PVRx_CLKGSEL               (1 << 10)    /* Bit 10: Clock Generator Select */
#define SDMMC_PVRx_DRVSEL_SHIFT          (14)         /* Bits 14-15: Driver Strength Select */
#define SDMMC_PVRx_DRVSEL_MASK           (3 << SDMMC_PVRx_DRVSEL_SHIFT)
#define   SDMMC_PVRx_DRVSEL(n)           ((n & 3) << SDMMC_PVRx_DRVSEL_SHIFT)

/* Slot Interrupt Status Register (0xfc) */

#define SDMMC_SISR_INTSSL0               (1 << 0)     /* Bit 0: Interrupt Signal for Slot 0 */
#define SDMMC_SISR_INTSSL1               (1 << 1)     /* Bit 1: Interrupt Signal for Slot 1 */

/* Host Controller Version Register (0xfe) */

#define SDMMC_HCVR_SVER_SHIFT            (0)          /* Bits 0-7: Specification Version Number */
#define SDMMC_HCVR_SVER_MASK             (0xf << SDMMC_HCVR_SVER_SHIFT)
#define   SDMMC_HCVR_SVER(reg)           ((reg & SDMMC_HCVR_SVER_MASK) >> SDMMC_HCVR_SVER_SHIFT)
#define SDMMC_HCVR_VVER_SHIFT            (8)          /* Bits 8-15: Vendor Version Number */
#define SDMMC_HCVR_VVER_MASK             (0xf << SDMMC_HCVR_VVER_SHIFT)
#define   SDMMC_HCVR_VVER(reg)           ((reg & SDMMC_HCVR_VVER_MASK) >> SDMMC_HCVR_VVER_SHIFT)

/* Additional Present State Register (0x200) */

#define SDMMC_APSR_HDATLL_SHIFT          (0)          /* Bits 0-3: DAT[7:4] High Line Level */
#define SDMMC_APSR_HDATLL_MASK           (0xf << SDMMC_APSR_HDATLL_SHIFT)
#define   SDMMC_APSR_HDATLL(reg)         ((reg & SDMMC_APSR_HDATLL_MASK) >> SDMMC_APSR_HDATLL_SHIFT)

/* e.MMC Control 1 Register (0x204) */

#define SDMMC_MC1R_CMDTYPE_SHIFT         (0)          /* Bits 0-1: e.MMC Command Type */
#define SDMMC_MC1R_CMDTYPE_MASK          (3 << SDMMC_MC1R_CMDTYPE_SHIFT)
#define   SDMMC_MC1R_CMDTYPE_NORMAL      (0 << SDMMC_MC1R_CMDTYPE_SHIFT)
#define   SDMMC_MC1R_CMDTYPE_WAITIRQ     (1 << SDMMC_MC1R_CMDTYPE_SHIFT)
#define   SDMMC_MC1R_CMDTYPE_STREAM      (2 << SDMMC_MC1R_CMDTYPE_SHIFT)
#define   SDMMC_MC1R_CMDTYPE_BOOT        (3 << SDMMC_MC1R_CMDTYPE_SHIFT)
#define SDMMC_MC1R_DDR                   (1 << 3)     /* Bit 3: e.MMC HSDDR Mode */
#define SDMMC_MC1R_OPD                   (1 << 4)     /* Bit 4: e.MMC Open Drain Mode */
#define SDMMC_MC1R_BOOTA                 (1 << 5)     /* Bit 5: e.MMC Boot Acknowledge Mode */
#define SDMMC_MC1R_RSTN                  (1 << 6)     /* Bit 6: e.MMC Reset Signal */
#define SDMMC_MC1R_FCD                   (1 << 7)     /* Bit 7: e.MMC Force Card Detect */

/* e.MMC Control 2 Register (0x205) */

#define SDMMC_MC2R_SRESP                 (1 << 0)     /* Bit 0: e.MMC Abort Wait IRQ */
#define SDMMC_MC2R_ABOOT                 (1 << 1)     /* Bit 1: e.MMC Abort Boot */

/* Capabilities Control Register (0x230) */

#define SDMMC_CACR_CAPWREN               (1 << 0)     /* Bit 0: Capabilities Write Enable */
#define SDMMC_CACR_KEY_SHIFT             (8)          /* Bits 8-15: Key */
#define SDMMC_CACR_KEY_MASK              (0xff << SDMMC_CACR_KEY_SHIFT)
#define   SDMMC_CACR_KEY_ENABLE          (0x46 << SDMMC_CACR_KEY_SHIFT)
#define   SDMMC_CACR_KEY_DISABLE         (0x00 << SDMMC_CACR_KEY_SHIFT)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE_SAMA5D2_SDMMC_H */
