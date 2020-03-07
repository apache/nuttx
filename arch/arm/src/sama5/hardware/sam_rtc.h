/****************************************************************************************
 * arch/arm/src/sama5/hardware/sam_rtc.h
 * Real-time Clock (RTC) definitions for the SAMA5D3
 *
 *   Copyright (C) 2013-2014 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_RTC_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_RTC_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* RTC register offsets *****************************************************************/

#define SAM_RTC_CR_OFFSET            0x0000 /* Control Register */
#define SAM_RTC_MR_OFFSET            0x0004 /* Mode Register */
#define SAM_RTC_TIMR_OFFSET          0x0008 /* Time Register */
#define SAM_RTC_CALR_OFFSET          0x000c /* Calendar Register */
#define SAM_RTC_TIMALR_OFFSET        0x0010 /* Time Alarm Register */
#define SAM_RTC_CALALR_OFFSET        0x0014 /* Calendar Alarm Register */
#define SAM_RTC_SR_OFFSET            0x0018 /* Status Register */
#define SAM_RTC_SCCR_OFFSET          0x001c /* Status Clear Command Register */
#define SAM_RTC_IER_OFFSET           0x0020 /* Interrupt Enable Register */
#define SAM_RTC_IDR_OFFSET           0x0024 /* Interrupt Disable Register */
#define SAM_RTC_IMR_OFFSET           0x0028 /* Interrupt Mask Register */
#define SAM_RTC_VER_OFFSET           0x002c /* Valid Entry Register */

#ifdef ATSAMA5D4
#  define SAM_RTC_TSTR0_OFFSET       0x00b0 /* TimeStamp Time Register 0 */
#  define SAM_RTC_TSDR0_OFFSET       0x00b4 /* TimeStamp Date Register 0 */
#  define SAM_RTC_TSSR0_OFFSET       0x00b8 /* TimeStamp Source Register 0 */
#  define SAM_RTC_TSTR1_OFFSET       0x00bc /* TimeStamp Time Register 1 */
#  define SAM_RTC_TSDR1_OFFSET       0x00c0 /* TimeStamp Date Register 1 */
#  define SAM_RTC_TSSR1_OFFSET       0x00c4 /* TimeStamp Source Register 1 */
#endif

/* RTC register addresses ***************************************************************/

#define SAM_RTC_CR                   (SAM_RTCC_VBASE+SAM_RTC_CR_OFFSET)
#define SAM_RTC_MR                   (SAM_RTCC_VBASE+SAM_RTC_MR_OFFSET)
#define SAM_RTC_TIMR                 (SAM_RTCC_VBASE+SAM_RTC_TIMR_OFFSET)
#define SAM_RTC_CALR                 (SAM_RTCC_VBASE+SAM_RTC_CALR_OFFSET)
#define SAM_RTC_TIMALR               (SAM_RTCC_VBASE+SAM_RTC_TIMALR_OFFSET)
#define SAM_RTC_CALALR               (SAM_RTCC_VBASE+SAM_RTC_CALALR_OFFSET)
#define SAM_RTC_SR                   (SAM_RTCC_VBASE+SAM_RTC_SR_OFFSET)
#define SAM_RTC_SCCR                 (SAM_RTCC_VBASE+SAM_RTC_SCCR_OFFSET)
#define SAM_RTC_IER                  (SAM_RTCC_VBASE+SAM_RTC_IER_OFFSET)
#define SAM_RTC_IDR                  (SAM_RTCC_VBASE+SAM_RTC_IDR_OFFSET)
#define SAM_RTC_IMR                  (SAM_RTCC_VBASE+SAM_RTC_IMR_OFFSET)
#define SAM_RTC_VER                  (SAM_RTCC_VBASE+SAM_RTC_VER_OFFSET)

#ifdef ATSAMA5D4
#  define SAM_RTC_TSTR0              (SAM_RTCC_VBASE+SAM_RTC_TSTR0_OFFSET)
#  define SAM_RTC_TSDR0              (SAM_RTCC_VBASE+SAM_RTC_TSDR0_OFFSET)
#  define SAM_RTC_TSSR0              (SAM_RTCC_VBASE+SAM_RTC_TSSR0_OFFSET)
#  define SAM_RTC_TSTR1              (SAM_RTCC_VBASE+SAM_RTC_TSTR1_OFFSET)
#  define SAM_RTC_TSDR1              (SAM_RTCC_VBASE+SAM_RTC_TSDR1_OFFSET)
#  define SAM_RTC_TSSR1              (SAM_RTCC_VBASE+SAM_RTC_TSSR1_OFFSET)
#endif

/* RTC register bit definitions *********************************************************/

/* RTC Control Register */

#define RTC_CR_UPDTIM                (1 << 0)  /* Bit 0:  Update Request Time Register */
#define RTC_CR_UPDCAL                (1 << 1)  /* Bit 1:  Update Request Calendar Register */
#define RTC_CR_TIMEVSEL_SHIFT        (8)       /* Bits 8-9: Time Event Selection */
#define RTC_CR_TIMEVSEL_MASK         (3 << RTC_CR_TIMEVSEL_SHIFT)
#  define RTC_CR_TIMEVSEL_MIN        (0 << RTC_CR_TIMEVSEL_SHIFT)
#  define RTC_CR_TIMEVSEL_HOUR       (1 << RTC_CR_TIMEVSEL_SHIFT)
#  define RTC_CR_TIMEVSEL_MIDNIGHT   (2 << RTC_CR_TIMEVSEL_SHIFT)
#  define RTC_CR_TIMEVSEL_NOON       (3 << RTC_CR_TIMEVSEL_SHIFT)
#define RTC_CR_CALEVSEL_SHIFT        (16)      /* Bits 16-17:  Calendar Event Selection */
#define RTC_CR_CALEVSEL_MASK         (3 << RTC_CR_CALEVSEL_SHIFT)
#  define RTC_CR_CALEVSEL_WEEK       (0 << RTC_CR_CALEVSEL_SHIFT)
#  define RTC_CR_CALEVSEL_MONTH      (1 << RTC_CR_CALEVSEL_SHIFT)
#  define RTC_CR_CALEVSEL_YEAR       (2 << RTC_CR_CALEVSEL_SHIFT)

/* RTC Mode Register */

#define RTC_MR_HRMOD                 (1 << 0)  /* Bit 0:  12-/24-hour Mode */

#ifdef ATSAMA5D4
#  define RTC_MR_PERSIAN             (1 << 1)  /* Bit 1:  PERSIAN Calendar */
#  define RTC_MR_NEGPPM              (1 << 4)  /* Bit 4:  NEGative PPM Correction */
#  define RTC_MR_CORRECTION_SHIFT    (8)       /* Bits 8-14: Slow Clock Correction */
#  define RTC_MR_CORRECTION_MASK     (0x7f << RTC_MR_CORRECTION_SHIFT)
#    define RTC_MR_CORRECTION_NONE   (0 << RTC_MR_CORRECTION_SHIFT)
#    define RTC_MR_CORRECTION(n)     ((uint8_t)(n) << RTC_MR_CORRECTION_SHIFT)
#  define RTC_MR_HIGHPPM             (1 << 15) /* Bit 15: HIGH PPM Correction */
#endif

/* RTC Time Register */

#define RTC_TIMR_SEC_SHIFT           (0)       /* Bits 0-6:  Current Second */
#define RTC_TIMR_SEC_MASK            (0x7f << RTC_TIMR_SEC_SHIFT)
#  define RTC_TIMR_SEC(n)            ((uint32_t)(n) << RTC_TIMR_SEC_SHIFT)
#define RTC_TIMR_MIN_SHIFT           (8)       /* Bits 8-14:  Current Minute */
#define RTC_TIMR_MIN_MASK            (0x7f <<  RTC_TIMR_MIN_SHIFT)
#  define RTC_TIMR_MIN(n)            ((uint32_t)(n) <<  RTC_TIMR_MIN_SHIFT)
#define RTC_TIMR_HOUR_SHIFT          (16)      /* Bits 16-21: Current Hour */
#define RTC_TIMR_HOUR_MASK           (0x3f << RTC_TIMR_HOUR_SHIFT)
#  define RTC_TIMR_HOUR(n)           ((uint32_t)(n) << RTC_TIMR_HOUR_SHIFT)
#define RTC_TIMR_AMPM                (1 << 22) /* Bit 22: Ante Meridiem Post Meridiem Indicator */

/* RTC Calendar Register */

#define RTC_CALR_CENT_SHIFT          (0)       /* Bits 0-6:  Current Century */
#define RTC_CALR_CENT_MASK           (0x7f << RTC_CALR_CENT_SHIFT)
#  define RTC_CALR_CENT(n)           ((uint32_t)(n) << RTC_TIMR_HOUR_SHIFT)
#define RTC_CALR_YEAR_SHIFT          (8)       /* Bits 8-15:  Current Year */
#define RTC_CALR_YEAR_MASK           (0xff << RTC_CALR_YEAR_SHIFT)
#  define RTC_CALR_YEAR(n)           ((uint32_t)(n) << RTC_CALR_YEAR_SHIFT)
#define RTC_CALR_MONTH_SHIFT         (16)      /* Bits 16-20: Current Month */
#define RTC_CALR_MONTH_MASK          (0x1f << RTC_CALR_MONTH_SHIFT)
#  define RTC_CALR_MONTH(n)          ((uint32_t)(n) << RTC_CALR_MONTH_SHIFT)
#define RTC_CALR_DAY_SHIFT           (21)      /* Bits 21-23: Current Day in Current Week */
#define RTC_CALR_DAY_MASK            (7 << RTC_CALR_DAY_SHIFT)
#  define RTC_CALR_DAY(n)            ((uint32_t)(n) << RTC_CALR_DAY_SHIFT)
#define RTC_CALR_DATE_SHIFT          (24)      /* Bits 24-29: Current Day in Current Month */
#define RTC_CALR_DATE_MASK           (0x3f << RTC_CALR_DATE_SHIFT)
#  define RTC_CALR_DATE(n)           ((uint32_t)(n) << RTC_CALR_DATE_SHIFT)

/* RTC Time Alarm Register */

#define RTC_TIMALR_SEC_SHIFT         (0)       /* Bits 0-6:  Second Alarm */
#define RTC_TIMALR_SEC_MASK          (0x7f << RTC_TIMALR_SEC_SHIFT)
#  define RTC_TIMALR_SEC(n)          ((uint32_t)(n) << RTC_TIMALR_SEC_SHIFT)
#define RTC_TIMALR_SECEN             (1 << 7)  /* Bit 7:  Second Alarm Enable */
#define RTC_TIMALR_MIN_SHIFT         (8)       /* Bits 8-14:  Minute Alarm */
#define RTC_TIMALR_MIN_MASK          (0x7f << RTC_TIMALR_MIN_SHIFT)
#  define RTC_TIMALR_MIN(n)          ((uint32_t)(n) << RTC_TIMALR_MIN_SHIFT)
#define RTC_TIMALR_MINEN             (1 << 15) /* Bit 15: Minute Alarm Enable */
#define RTC_TIMALR_HOUR_SHIFT        (16)      /* Bits 16-21:  Hour Alarm */
#define RTC_TIMALR_HOUR_MASK         (0x3f << RTC_TIMALR_HOUR_SHIFT)
#  define RTC_TIMALR_HOUR(n)         ((uint32_t)(n) << RTC_TIMALR_HOUR_SHIFT)
#define RTC_TIMALR_AMPM              (1 << 22) /* Bit 22: AM/PM Indicator */
#define RTC_TIMALR_HOUREN            (1 << 23) /* Bit 23: Hour Alarm Enable */

/* RTC Calendar Alarm Register */

#define RTC_CALALR_MONTH_SHIFT       (16)      /* Bits 16-20:  Month Alarm */
#define RTC_CALALR_MONTH_MASK        (0x1f << RTC_CALALR_MONTH_SHIFT)
#  define RTC_CALALR_MONTH(n)        ((uint32_t)(n) << RTC_CALALR_MONTH_SHIFT)
#define RTC_CALALR_MTHEN             (1 << 23) /* Bit 23: Month Alarm Enable */
#define RTC_CALALR_DATE_SHIFT        (24)      /* Bits 24-29:  Date Alarm */
#define RTC_CALALR_DATE_MASK         (0x3f << RTC_CALALR_DATE_SHIFT)
#  define RTC_CALALR_DATE(n)         ((uint32_t)(n) << RTC_CALALR_DATE_SHIFT)
#define RTC_CALALR_DATEEN            (1 << 31) /* Bit 31: Date Alarm Enable */

/* RTC Status Register */

#define RTC_SR_ACKUPD                (1 << 0)  /* Bit 0:  Acknowledge for Update */
#define RTC_SR_ALARM                 (1 << 1)  /* Bit 1:  Alarm Flag */
#define RTC_SR_SEC                   (1 << 2)  /* Bit 2:  Second Event */
#define RTC_SR_TIMEV                 (1 << 3)  /* Bit 3:  Time Event */
#define RTC_SR_CALEV                 (1 << 4)  /* Bit 4:  Calendar Event */

#ifdef ATSAMA5D4
#  define RTC_SR_TDERR               (1 << 5)  /* Bit 5:  Time and/or Date Free Running Error */
#endif

/* RTC Status Clear Command Register */

#define RTC_SCCR_ACKCLR              (1 << 0)  /* Bit 0:  Acknowledge Clear */
#define RTC_SCCR_ALRCLR              (1 << 1)  /* Bit 1:  Alarm Clear */
#define RTC_SCCR_SECCLR              (1 << 2)  /* Bit 2:  Second Clear */
#define RTC_SCCR_TIMCLR              (1 << 3)  /* Bit 3:  Time Clear */
#define RTC_SCCR_CALCLR              (1 << 4)  /* Bit 4:  Calendar Clear */

#ifdef ATSAMA5D4
#  define RTC_SCCR_TDERRCLR          (1 << 5)  /* Bit 5:  Time and/or Date Free Running Error Clear */
#endif

/* RTC Interrupt Enable Register */

#define RTC_IER_ACKEN                (1 << 0)  /* Bit 0:  Acknowledge Update Interrupt Enable */
#define RTC_IER_ALREN                (1 << 1)  /* Bit 1:  Alarm Interrupt Enable */
#define RTC_IER_SECEN                (1 << 2)  /* Bit 2:  Second Event Interrupt Enable */
#define RTC_IER_TIMEN                (1 << 3)  /* Bit 3:  Time Event Interrupt Enable */
#define RTC_IER_CALEN                (1 << 4)  /* Bit 4:  Calendar Event Interrupt Enable */

#ifdef ATSAMA5D4
#  define RTC_IER_TDERREN            (1 << 5)  /* Bit 5:  Time and/or Date Free Running Error Enable */
#endif

/* RTC Interrupt Disable Register */

#define RTC_IDR_ACKDIS               (1 << 0)  /* Bit 0:  Acknowledge Update Interrupt Disable */
#define RTC_IDR_ALRDIS               (1 << 1)  /* Bit 1:  Alarm Interrupt Disable */
#define RTC_IDR_SECDIS               (1 << 2)  /* Bit 2:  Second Event Interrupt Disable */
#define RTC_IDR_TIMDIS               (1 << 3)  /* Bit 3:  Time Event Interrupt Disable */
#define RTC_IDR_CALDIS               (1 << 4)  /* Bit 4:  Calendar Event Interrupt Disable */

#ifdef ATSAMA5D4
#  define RTC_IDR_TDERRDIS           (1 << 5)  /* Bit 5:  Time and/or Date Free Running Error Disable */
#endif

/* RTC Interrupt Mask Register */

#define RTC_IMR_ACK                  (1 << 0)  /* Bit 0:  Acknowledge Update Interrupt Mask */
#define RTC_IMR_ALR                  (1 << 1)  /* Bit 1:  Alarm Interrupt Mask */
#define RTC_IMR_SEC                  (1 << 2)  /* Bit 2:  Second Event Interrupt Mask */
#define RTC_IMR_TIM                  (1 << 3)  /* Bit 3:  Time Event Interrupt Mask */
#define RTC_IMR_CAL                  (1 << 4)  /* Bit 4:  Calendar Event Interrupt Mask */

#ifdef ATSAMA5D4
#  define RTC_IMR_TDERR              (1 << 5)  /* Bit 5:  Time and/or Date Free Running Error Mask */
#endif

/* RTC Valid Entry Register */

#define RTC_VER_NVTIM                (1 << 0)  /* Bit 0:  Non-valid Time */
#define RTC_VER_NVCAL                (1 << 1)  /* Bit 1:  Non-valid Calendar */
#define RTC_VER_NVTIMALR             (1 << 2)  /* Bit 2:  Non-valid Time Alarm */
#define RTC_VER_NVCALALR             (1 << 3)  /* Bit 3:  Non-valid Calendar Alarm */

#ifdef ATSAMA5D4
/* TimeStamp Time Register 0/1 */

#  define RTC_TSTR_SEC_SHIFT        (0)       /* Bits 0-6:  Seconds of the Tamper */
#  define RTC_TSTR_SEC_MASK         (0x7f << RTC_TSTR_SEC_SHIFT)
#    define RTC_TSTR_SEC(n)         ((uint32_t)(n) << RTC_TSTR_SEC_SHIFT)
#  define RTC_TSTR_MIN_SHIFT        (8)       /* Bits 8-14:  Minutes of the Tamper */
#  define RTC_TSTR_MIN_MASK         (0x7f << RTC_TSTR_MIN_SHIFT)
#    define RTC_TSTR_MIN(n)         ((uint32_t)(n) << RTC_TSTR_MIN_SHIFT)
#  define RTC_TSTR_HOUR_SHIFT       (16)      /* Bits 16-21:  Hours of the Tamper */
#  define RTC_TSTR_HOUR_MASK        (0x3f << RTC_TSTR_HOUR_SHIFT)
#    define RTC_TSTR_HOUR(n)        ((uint32_t)(n) << RTC_TSTR_HOUR_SHIFT)
#  define RTC_TSTR_AMPM             (1 << 22) /* Bit 22: AM/PM Indicator of the Tamper */

/* Only TSTR0 has the event counter */

#  define RTC_TSTR0_TEVCNT_SHIFT    (24)      /* Bits 24-27: Tamper Events Counter */
#  define RTC_TSTR0_TEVCNT_MASK     (15 << RTC_TSTR0_TEVCNT_SHIFT)

#  define RTC_TSTR_BACKUP           (1 << 31) /* Bit 31: System Mode of the Tamper */
#endif

#ifdef ATSAMA5D4
/* TimeStamp Date Register 0 and 1 */

#define RTC_TSDR_CENT_SHIFT          (0)       /* Bits 0-6:  Century of the Tamper */
#define RTC_TSDR_CENT_MASK           (0x7f << RTC_TSDR_CENT_SHIFT)
#  define RTC_TSDR_CENT(n)           ((uint32_t)(n) << RTC_TIMR_HOUR_SHIFT)
#define RTC_TSDR_YEAR_SHIFT          (8)       /* Bits 8-15:  Year of the Tamper */
#define RTC_TSDR_YEAR_MASK           (0xff << RTC_TSDR_YEAR_SHIFT)
#  define RTC_TSDR_YEAR(n)           ((uint32_t)(n) << RTC_TSDR_YEAR_SHIFT)
#define RTC_TSDR_MONTH_SHIFT         (16)      /* Bits 16-20: Month of the Tamper */
#define RTC_TSDR_MONTH_MASK          (0x1f << RTC_TSDR_MONTH_SHIFT)
#  define RTC_TSDR_MONTH(n)          ((uint32_t)(n) << RTC_TSDR_MONTH_SHIFT)
#define RTC_TSDR_DAY_SHIFT           (21)      /* Bits 21-23: Day of the Tamper */
#define RTC_TSDR_DAY_MASK            (7 << RTC_TSDR_DAY_SHIFT)
#  define RTC_TSDR_DAY(n)            ((uint32_t)(n) << RTC_TSDR_DAY_SHIFT)
#define RTC_TSDR_DATE_SHIFT          (24)      /* Bits 24-29: Date of the Tamper */
#define RTC_TSDR_DATE_MASK           (0x3f << RTC_TSDR_DATE_SHIFT)
#  define RTC_TSDR_DATE(n)           ((uint32_t)(n) << RTC_TSDR_DATE_SHIFT)
#endif

#ifdef ATSAMA5D4
/* TimeStamp Source Register 0 and 1*/

#  define RTC_TSSR_SHLDM             (1 << 0)  /* Bit 0: Shield Monitor */
#  define RTC_TSSR_DBLFM             (1 << 1)  /* Bit 1: Double Frequency Monitor */
#  define RTC_TSSR_TST               (1 << 2)  /* Bit 2: Test Pin Monitor */
#  define RTC_TSSR_JTAG              (1 << 3)  /* Bit 3: JTAG Pins Monitor */
#  define RTC_TSSR_REGUL             (1 << 4)  /* Bit 4: Core Regulator Disconnection Monitor */
#  define RTC_TSSR_MCKM              (1 << 5)  /* Bit 5: Master Clock Monitor */
#  define RTC_TSSR_TPML              (1 << 6)  /* Bit 6: Low Temperature Monitor */
#  define RTC_TSSR_TPMH              (1 << 7)  /* Bit 7: High Temperature Monitor */
#  define RTC_TSSR_VDDBUL            (1 << 8)  /* Bit 8: Low VDDBU Voltage Monitor */
#  define RTC_TSSR_VDDBUH            (1 << 9)  /* Bit 9: High VDDBU Voltage Monitor */
#  define RTC_TSSR_VDDCOREL          (1 << 10) /* Bit 10: Low VDDCORE Voltage Monitor */
#  define RTC_TSSR_VDDCOREH          (1 << 11) /* Bit 11: High VDDCORE Voltage Monitor */
#  define RTC_TSSR_VDDIOL            (1 << 12) /* Bit 12: Low VDDIO Voltage Monitor */
#  define RTC_TSSR_VDDIOH            (1 << 13) /* Bit 13: High VDDIO Voltage Monitor */
#  define RTC_TSSR_VDDRL             (1 << 14) /* Bit 14: Low VDDDDR Voltage Monitor */
#  define RTC_TSSR_VDDRH             (1 << 15) /* Bit 15: High VDDDDR Voltage Monitor */

#  define RTC_TSSR_DET(n)            (1 << ((n) + 16))
#  define RTC_TSSR_DET0              (1 << 16) /* Bit 16: PIOBU Intrusion Detector */
#  define RTC_TSSR_DET1              (1 << 17) /* Bit 17: PIOBU Intrusion Detector */
#  define RTC_TSSR_DET2              (1 << 18) /* Bit 18: PIOBU Intrusion Detector */
#  define RTC_TSSR_DET3              (1 << 19) /* Bit 19: PIOBU Intrusion Detector */
#  define RTC_TSSR_DET4              (1 << 20) /* Bit 20: PIOBU Intrusion Detector */
#  define RTC_TSSR_DET5              (1 << 21) /* Bit 21: PIOBU Intrusion Detector */
#  define RTC_TSSR_DET6              (1 << 22) /* Bit 22: PIOBU Intrusion Detector */
#  define RTC_TSSR_DET7              (1 << 23) /* Bit 23: PIOBU Intrusion Detector */
#  define RTC_TSSR_DET8              (1 << 24) /* Bit 24: PIOBU Intrusion Detector */
#  define RTC_TSSR_DET9              (1 << 25) /* Bit 25: PIOBU Intrusion Detector */
#  define RTC_TSSR_DET10             (1 << 26) /* Bit 26: PIOBU Intrusion Detector */
#  define RTC_TSSR_DET11             (1 << 27) /* Bit 27: PIOBU Intrusion Detector */
#  define RTC_TSSR_DET12             (1 << 28) /* Bit 28: PIOBU Intrusion Detector */
#  define RTC_TSSR_DET13             (1 << 29) /* Bit 29: PIOBU Intrusion Detector */
#  define RTC_TSSR_DET14             (1 << 30) /* Bit 30: PIOBU Intrusion Detector */
#  define RTC_TSSR_DET15             (1 << 31) /* Bit 31: PIOBU Intrusion Detector */
#endif

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_RTC_H */
