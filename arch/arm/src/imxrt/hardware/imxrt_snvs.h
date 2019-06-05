/********************************************************************************************
 * arch/arm/src/imxrt/hardware/imxrt_snvs.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_SNVS_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_SNVS_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>
#include "hardware/imxrt_memorymap.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

#define IMXRT_SNVS_LP_MAXTAMPER       10

/* Register offsets *************************************************************************/

#define IMXRT_SNVS_HPLR_OFFSET        0x0000  /* SNVS_HP Lock Register */
#define IMXRT_SNVS_HPCOMR_OFFSET      0x0004  /* SNVS_HP Command Register */
#define IMXRT_SNVS_HPCR_OFFSET        0x0008  /* SNVS_HP Control Register */
#define IMXRT_SNVS_HPSR_OFFSET        0x0014  /* SNVS_HP Status Register */
#define IMXRT_SNVS_HPRTCMR_OFFSET     0x0024  /* SNVS_HP Real Time Counter MSB Register */
#define IMXRT_SNVS_HPRTCLR_OFFSET     0x0028  /* SNVS_HP Real Time Counter LSB Register */
#define IMXRT_SNVS_HPTAMR_OFFSET      0x002c  /* SNVS_HP Time Alarm MSB Register */
#define IMXRT_SNVS_HPTALR_OFFSET      0x0030  /* SNVS_HP Time Alarm LSB Register */
#define IMXRT_SNVS_LPLR_OFFSET        0x0034  /* SNVS_LP Lock Register */
#define IMXRT_SNVS_LPCR_OFFSET        0x0038  /* SNVS_LP Control Register */
#define IMXRT_SNVS_LPSR_OFFSET        0x004c  /* SNVS_LP Status Register */
#define IMXRT_SNVS_LPSMCMR_OFFSET     0x005c  /* SNVS_LP Secure Monotonic Counter MSB Register */
#define IMXRT_SNVS_LPSMCLR_OFFSET     0x0060  /* SNVS_LP Secure Monotonic Counter LSB Register */

#define IMXRT_SNVS_LPGPR0L_OFFSET     0x0068  /* SNVS_LP General Purpose Register 0 (legacy alias) */

#define IMXRT_SNVS_LPGPRA_OFFSET(n)  (0x0090 + ((n) << 2))
#define IMXRT_SNVS_LPGPR0A_OFFSET     0x0090  /* NVS_LP General Purpose Registers 0 LPGPR0_alias */
#define IMXRT_SNVS_LPGPR1A_OFFSET     0x0094  /* NVS_LP General Purpose Registers 1 LPGPR1_alias */
#define IMXRT_SNVS_LPGPR2A_OFFSET     0x0098  /* NVS_LP General Purpose Registers 2 LPGPR2_alias */
#define IMXRT_SNVS_LPGPR3A_OFFSET     0x009c  /* NVS_LP General Purpose Registers 3 LPGPR3_alias */

#define IMXRT_SNVS_LPGPR_OFFSET(n)    (0x0100 + ((n) << 2))
#define IMXRT_SNVS_LPGPR0_OFFSET      0x0100  /* SNVS_LP General Purpose Registers 0 */
#define IMXRT_SNVS_LPGPR1_OFFSET      0x0104  /* SNVS_LP General Purpose Registers 1 */
#define IMXRT_SNVS_LPGPR2_OFFSET      0x0108  /* SNVS_LP General Purpose Registers 2 */
#define IMXRT_SNVS_LPGPR3_OFFSET      0x010c  /* SNVS_LP General Purpose Registers 3 */

#define IMXRT_SNVS_HPVIDR1_OFFSET     0x0bf8  /* SNVS_HP Version ID Register 1 */
#define IMXRT_SNVS_HPVIDR2_OFFSET     0x0bfc  /* SNVS_HP Version ID Register 2 */

/* Register addresses ***********************************************************************/

#define IMXRT_SNVS_HPLR               (IMXRT_SNVSHP_BASE + IMXRT_SNVS_HPLR_OFFSET)
#define IMXRT_SNVS_HPCOMR             (IMXRT_SNVSHP_BASE + IMXRT_SNVS_HPCOMR_OFFSET)
#define IMXRT_SNVS_HPCR               (IMXRT_SNVSHP_BASE + IMXRT_SNVS_HPCR_OFFSET)
#define IMXRT_SNVS_HPSR               (IMXRT_SNVSHP_BASE + IMXRT_SNVS_HPSR_OFFSET)
#define IMXRT_SNVS_HPRTCMR            (IMXRT_SNVSHP_BASE + IMXRT_SNVS_HPRTCMR_OFFSET)
#define IMXRT_SNVS_HPRTCLR            (IMXRT_SNVSHP_BASE + IMXRT_SNVS_HPRTCLR_OFFSET)
#define IMXRT_SNVS_HPTAMR             (IMXRT_SNVSHP_BASE + IMXRT_SNVS_HPTAMR_OFFSET)
#define IMXRT_SNVS_HPTALR             (IMXRT_SNVSHP_BASE + IMXRT_SNVS_HPTALR_OFFSET)
#define IMXRT_SNVS_LPLR               (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPLR_OFFSET)
#define IMXRT_SNVS_LPCR               (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPCR_OFFSET)
#define IMXRT_SNVS_LPSR               (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPSR_OFFSET)
#define IMXRT_SNVS_LPSMCMR            (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPSMCMR_OFFSET)
#define IMXRT_SNVS_LPSMCLR            (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPSMCLR_OFFSET)

#define IMXRT_SNVS_LPGPR0L            (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPGPR0L_OFFSET)

#define IMXRT_SNVS_LPGPRA(n)          (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPGPRA_OFFSET(n))
#define IMXRT_SNVS_LPGPR0A            (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPGPR0A_OFFSET)
#define IMXRT_SNVS_LPGPR1A            (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPGPR1A_OFFSET)
#define IMXRT_SNVS_LPGPR2A            (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPGPR2A_OFFSET)
#define IMXRT_SNVS_LPGPR3A            (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPGPR3A_OFFSET)

#define IMXRT_SNVS_LPGPR(n)           (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPGPR_OFFSET(n))
#define IMXRT_SNVS_LPGPR0             (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPGPR0_OFFSET)
#define IMXRT_SNVS_LPGPR1             (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPGPR1_OFFSET)
#define IMXRT_SNVS_LPGPR2             (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPGPR2_OFFSET)
#define IMXRT_SNVS_LPGPR3             (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPGPR3_OFFSET)

#define IMXRT_SNVS_HPVIDR1            (IMXRT_SNVSHP_BASE + IMXRT_SNVS_HPVIDR1_OFFSET)
#define IMXRT_SNVS_HPVIDR2            (IMXRT_SNVSHP_BASE + IMXRT_SNVS_HPVIDR2_OFFSET)

/* Register bit definitions *****************************************************************/

/* SNVS_HP Lock Register */

                                                /* Bits 0-3: Reserved */
#define SNVS_HPLR_MCSL                (1 << 4)  /* Bit 4:  Monotonic Counter Soft Lock */
#define SNVS_HPLR_GPRSL               (1 << 5)  /* Bit 5:  General Purpose Register Soft Lock */
                                                /* Bits 6-31: Reserved */

/* SNVS_HP Command Register */

                                                /* Bits 0-3: Reserved */
#define SNVS_HPCOMR_LPSWR             (1 << 4)  /* Bit 4:  LP Software Reset */
#define SNVS_HPCOMR_LPSWRDIS          (1 << 5)  /* Bit 5:  LP Software Reset Disable */
                                                /* Bits 6-7: Reserved */
#define SNVS_HPCOMR_SWSV              (1 << 8)  /* Bit 8:  */
                                                /* Bits 9-30: Reserved */
#define SNVS_HPCOMR_NPSWAEN           (1 << 31) /* Bit 31: Non-Privileged Software Access Enable */

/* SNVS_HP Control Register */

#define SNVS_HPCR_RTCEN               (1 << 0)  /* Bit 0:  HP Real Time Counter Enable */
#define SNVS_HPCR_HPTAEN              (1 << 1)  /* Bit 1:  HP Time Alarm Enable */
#define SNVS_HPCR_DISPI               (1 << 2)  /* Bit 2:  Disable periodic interrupt in the functional interrupt */
                                                /* Bit 2:  Reserved */
#define SNVS_HPCR_PIEN                (1 << 3)  /* Bit 3:  HP Periodic Interrupt Enable */
#define SNVS_HPCR_PIFREQ_SHIFT        (4)       /* Bits 4-7:  Periodic Interrupt Frequency */
#define SNVS_HPCR_PIFREQ_MASK         (15 << SNVS_HPCR_PIFREQ_SHIFT)
#  define SNVS_HPCR_PIFREQ(n)         ((uint32_t)(n) << SNVS_HPCR_PIFREQ_SHIFT)
#define SNVS_HPCR_HPCALBEN            (1 << 8)  /* Bit 8:  HP Real Time Counter Calibration Enabled */
                                                /* Bit 9:  Reserved */
#define SNVS_HPCR_HPCALBVAL_SHIFT     (10)      /* Bits 10-14: HP Calibration Value */
#define SNVS_HPCR_HPCALBVAL_MASK      (31 << SNVS_HPCR_HPCALBVAL_SHIFT)
#  define SNVS_HPCR_HPCALBVAL(n)      ((uint32_t)(n) << SNVS_HPCR_HPCALBVAL_SHIFT)
#  define SNVS_HPCR_HPCALBVAL_ZERO    (0  << SNVS_HPCR_HPCALBVAL_SHIFT) /* +0  counts per 32768 ticks */
#  define SNVS_HPCR_HPCALBVAL_P1      (1  << SNVS_HPCR_HPCALBVAL_SHIFT) /* +1  counts per 32768 ticks */
#  define SNVS_HPCR_HPCALBVAL_P2      (2  << SNVS_HPCR_HPCALBVAL_SHIFT) /* +2  counts per 32768 ticks */
#  define SNVS_HPCR_HPCALBVAL_P15     (15 << SNVS_HPCR_HPCALBVAL_SHIFT) /* +15 counts per 32768 ticks */
#  define SNVS_HPCR_HPCALBVAL_M16     (16 << SNVS_HPCR_HPCALBVAL_SHIFT) /* -16 counts per 32768 ticks */
#  define SNVS_HPCR_HPCALBVAL_M15     (17 << SNVS_HPCR_HPCALBVAL_SHIFT) /* -15 counts per 32768 ticks */
#  define SNVS_HPCR_HPCALBVAL_M2      (30 << SNVS_HPCR_HPCALBVAL_SHIFT) /* -2  counts per 32768 ticks */
#  define SNVS_HPCR_HPCALBVAL_M1      (31 << SNVS_HPCR_HPCALBVAL_SHIFT) /* -1  counts per 32768 ticks */
                                                /* Bits 15: Reserved */
#define SNVS_HPCR_HPTS                (1 << 16) /* Bit 16: LPSRTC time sychronization */
                                                /* Bits 17-23: Reserved */
#define SNVS_HPCR_BTNCONFIG_SHIFT     (24)      /* Bits 24-26: Button Configuration */
#define SNVS_HPCR_BTNCONFIG_MASK      (7 << SNVS_HPCR_BTNCONFIG_SHIFT)
#  define SNVS_HPCR_BTNCONFIG_ LOW    (0 << SNVS_HPCR_BTNCONFIG_SHIFT) /* Button signal active low */
#  define SNVS_HPCR_BTNCONFIG_HIGH    (1 << SNVS_HPCR_BTNCONFIG_SHIFT) /* Button signal active high */
#  define SNVS_HPCR_BTNCONFIG_RISING  (2 << SNVS_HPCR_BTNCONFIG_SHIFT) /* Button signal active on rising edge */
#  define SNVS_HPCR_BTNCONFIG_FALLING (3 << SNVS_HPCR_BTNCONFIG_SHIFT) /* Button signal active on falling edge */
#  define SNVS_HPCR_BTNCONFIG_BOTH    (4 << SNVS_HPCR_BTNCONFIG_SHIFT) /* Button signal active on any edge */
#define SNVS_HPCR_BTNMASK             (1 << 27) /* Bit 27: Button interrupt mask */
                                                /* Bits 28-31: Reserved */

/* SNVS_HP Status Register */

#define SNVS_HPSR_HPTA                (1 << 0)  /* Bit 0:  HP Time Alarm */
#define SNVS_HPSR_PI                  (1 << 1)  /* Bit 1:  Periodic Interrupt */
                                                /* Bits 2-3: Reserved */
#define SNVS_HPSR_LPDIS               (1 << 4)  /* Bit 4:  Low Power Disable */
                                                /* Bit 95  Reserved */
#define SNVS_HPSR_BTN                 (1 << 6)  /* Bit 6:  Button */
#define SNVS_HPSR_BI                  (1 << 7)  /* Bit 7:  Button Interrupt */
                                                /* Bits 8-31: Reserved */

/* SNVS_HP Real Time Counter MSB Register (15-bit MSB of counter) */
/* SNVS_HP Real Time Counter LSB Register (32-bit LSB of counter) */

#define SNVS_HPRTCMR_MASK             0x00007fff /* Bits 0-14: HP Real Time Counter */

/* SNVS_HP Time Alarm MSB Register (15-bit MSB of counter) */
/* SNVS_HP Time Alarm LSB Register (32-bit LSB of counter) */

#define SNVS_HPTAMR_MASK              0x00007fff /* Bits 0-14: HP Time Alarm, most-significant 15 bits */

/* SNVS_LP Lock Register */

                                                /* Bits 0-3: Reserved */
#define SNVS_LPLR_MCHL                (1 << 4)  /* Bit 4:  Monotonic Counter Hard Lock */
#define SNVS_LPLR_GPRHL               (1 << 5)  /* Bit 5:  General Purpose Register Hard Lock */
                                                /* Bits 6-31: Reserved */

/* SNVS_LP Control Register */

#define SVNS_LPCR_SRTCENV             (1 << 0)  /* Bit 0:  Start SVNS RTC time counter */
#define SVNS_LPCR_LPTAEN              (1 << 1)  /* Bit 1:  Enable SVNS RTC time alarm */
#define SNVS_LPCR_MCENV               (1 << 2)  /* Bit 2:  Monotonic Counter Enabled and Valid */
#define SNVS_LPCR_LPWUIEN             (1 << 3)  /* Bit 3:  LP Wake-Up Interrupt Enable */
                                                /* Bit 4:  Reserved */
#define SNVS_LPCR_DPEN                (1 << 5)  /* Bit 5:  Dumb PMIC Enabled */
#define SNVS_LPCR_TOP                 (1 << 6)  /* Bit 6:  Turn off System Power */
#define SNVS_LPCR_PWRGLITCHEN         (1 << 7)  /* Bit 7:  Power Glitch Enable */
                                                /* Bits 8-15:  Reserved for i.MX1050 family */
#define SNVS_LPCR_LPCALBEN            (1 << 8)  /* Bit 8:  LP Real Time Counter Calibration Enabled */
                                                /* Bit 9:  Reserved */
#define SNVS_LPCR_LPCALBVAL_SHIFT     (10)      /* Bits 10-14: LP Calibration Value */
#define SNVS_LPCR_LPCALBVAL_MASK      (31 << SNVS_LPCR_LPCALBVAL_SHIFT)
#  define SNVS_LPCR_LPCALBVAL(n)      ((uint32_t)(n) << SNVS_LPCR_LPCALBVAL_SHIFT)
#  define SNVS_LPCR_LPCALBVAL_ZERO    (0  << SNVS_LPCR_LPCALBVAL_SHIFT) /* +0  counts per 32768 ticks */
#  define SNVS_LPCR_LPCALBVAL_P1      (1  << SNVS_LPCR_LPCALBVAL_SHIFT) /* +1  counts per 32768 ticks */
#  define SNVS_LPCR_LPCALBVAL_P2      (2  << SNVS_LPCR_LPCALBVAL_SHIFT) /* +2  counts per 32768 ticks */
#  define SNVS_LPCR_LPCALBVAL_P15     (15 << SNVS_LPCR_LPCALBVAL_SHIFT) /* +15 counts per 32768 ticks */
#  define SNVS_LPCR_LPCALBVAL_M16     (16 << SNVS_LPCR_LPCALBVAL_SHIFT) /* -16 counts per 32768 ticks */
#  define SNVS_LPCR_LPCALBVAL_M15     (17 << SNVS_LPCR_LPCALBVAL_SHIFT) /* -15 counts per 32768 ticks */
#  define SNVS_LPCR_LPCALBVAL_M2      (30 << SNVS_LPCR_LPCALBVAL_SHIFT) /* -2  counts per 32768 ticks */
#  define SNVS_LPCR_LPCALBVAL_M1      (31 << SNVS_LPCR_LPCALBVAL_SHIFT) /* -1  counts per 32768 ticks */
                                                /* Bit 15:  Reserved */
#define SNVS_LPCR_BTNPRESSTIME_SHIFT  (16)      /* Bits 16-17: PMIC button press time out values */
#define SNVS_LPCR_BTNPRESSTIME_MASK   (3 << SNVS_LPCR_BTNPRESSTIME_SHIFT)
#  define SNVS_LPCR_BTNPRESSTIME_5SEC   (0 << SNVS_LPCR_BTNPRESSTIME_SHIFT) /* 5 secs */
#  define SNVS_LPCR_BTNPRESSTIME_10SEC  (1 << SNVS_LPCR_BTNPRESSTIME_SHIFT) /* 10 secs */
#  define SNVS_LPCR_BTNPRESSTIME_15SEC  (2 << SNVS_LPCR_BTNPRESSTIME_SHIFT) /* 15 secs */
#  define SNVS_LPCR_BTNPRESSTIME_DESAB  (3 << SNVS_LPCR_BTNPRESSTIME_SHIFT) /* Long press disabled */
#define SNVS_LPCR_DEBOUNCE_SHIFT      (18)      /* Bits 18-19: Debounce time for BTN input signal */
#define SNVS_LPCR_DEBOUNCE_MASK       (3 << SNVS_LPCR_DEBOUNCE_SHIFT)
#  define SNVS_LPCR_DEBOUNCE_50MS     (0 << SNVS_LPCR_DEBOUNCE_SHIFT) /* 50msec debounce */
#  define SNVS_LPCR_DEBOUNCE_100MS    (1 << SNVS_LPCR_DEBOUNCE_SHIFT) /* 100msec debounce */
#  define SNVS_LPCR_DEBOUNCE_500MS    (2 << SNVS_LPCR_DEBOUNCE_SHIFT) /* 500msec debounce */
#  define SNVS_LPCR_DEBOUNCE_NONE     (3 << SNVS_LPCR_DEBOUNCE_SHIFT) /* 0msec debounce */
#define SNVS_LPCR_ONTIME_SHIFT        (20)      /* Bits 20-21: ON time configuration */
#define SNVS_LPCR_ONTIME_MASK         (3 << SNVS_LPCR_ONTIME_SHIFT)
#  define SNVS_LPCR_ONTIME_500MS      (0 << SNVS_LPCR_ONTIME_SHIFT) /* 500msec off->on transition time */
#  define SNVS_LPCR_ONTIME_50MS       (1 << SNVS_LPCR_ONTIME_SHIFT) /* 50msec off->on transition time */
#  define SNVS_LPCR_ONTIME_100MS      (2 << SNVS_LPCR_ONTIME_SHIFT) /* 100msec off->on transition time */
#  define SNVS_LPCR_ONTIME_NONE       (3 << SNVS_LPCR_ONTIME_SHIFT) /* 0msec off->on transition time */
#define SNVS_LPCR_PKEN                (1 << 22) /* Bit 22: PMIC On Request Enable */
#define SNVS_LPCR_PKOVERRIDE          (1 << 23) /* Bit 23: PMIC On Request Override */
                                                /* Bits 24-31: Reserved */

/* SNVS_LP Status Register */

                                                /* Bits 0-1: Reserved */
#define SNVS_LPSR_MCR                 (1 << 2)  /* Bit 2:  Monotonic Counter Rollover */
                                                /* Bits 3-16: Reserved */
#define SNVS_LPSR_EO                  (1 << 17) /* Bit 17: Emergency Off */
#define SNVS_LPSR_SPO                 (1 << 18) /* Bit 18: Set Power Off */
                                                /* Bits 19-31: Reserved */

/* SNVS_LP Secure Monotonic Counter MSB Register */
/* SNVS_LP Secure Monotonic Counter LSB Register (32-bit LSB counter value) */

#define SNVS_LPSMCMR_MONCOUNTER_SHIFT (0)       /* Bits 0-15: Monotonic Counter most-significant 16 Bits */
#define SNVS_LPSMCMR_MONCOUNTER_MASK  (0xffff << SNVS_LPSMCMR_MONCOUNTER_SHIFT)
#  define SNVS_LPSMCMR_MONCOUNTER(n)  ((uint32_t)(n) << SNVS_LPSMCMR_MONCOUNTER_SHIFT)
#define SNVS_LPSMCMR_MCERABITS_SHIFT  (16)      /* Bits 16-31: Monotonic Counter Era Bits */
#define SNVS_LPSMCMR_MCERABITS_MASK   (0xffff << SNVS_LPSMCMR_MCERABITS_SHIFT)
#  define SNVS_LPSMCMR_MCERABITS(n)   ((uint32_t)(n) << SNVS_LPSMCMR_MCERABITS_SHIFT)

/* SNVS_LP General Purpose Register 0 (legacy alias) (32-bit value) */
/* NVS_LP General Purpose Registers 0 LPGPR0_alias (32-bit value) */
/* NVS_LP General Purpose Registers 1 LPGPR1_alias (32-bit value) */
/* NVS_LP General Purpose Registers 2 LPGPR2_alias (32-bit value) */
/* NVS_LP General Purpose Registers 3 LPGPR3_alias (32-bit value) */
/* SNVS_LP General Purpose Registers 0 (32-bit value) */
/* SNVS_LP General Purpose Registers 1 (32-bit value) */
/* SNVS_LP General Purpose Registers 2 (32-bit value) */
/* SNVS_LP General Purpose Registers 3 (32-bit value) */

/* SNVS_HP Version ID Register 1 */

#define SNVS_HPVIDR1_MINORREV_SHIFT   (0)       /* Bits 0-7: SNVS block minor version number */
#define SNVS_HPVIDR1_MINORREV_MASK    (0xff << SNVS_HPVIDR1_MINORREV_SHIFT)
#define SNVS_HPVIDR1_MAJORREV_SHIFT   (8)       /* Bits 8-15: SNVS block major version number */
#define SNVS_HPVIDR1_MAJORREV_MASK    (0xff << SNVS_HPVIDR1_MAJORREV_SHIFT)
#define SNVS_HPVIDR1_IPID_SHIFT       (16)      /* Bits 16-31: SNVS block ID */
#define SNVS_HPVIDR1_IPID_MASK        (0xffff << SNVS_HPVIDR1_IPID_SHIFT)

/* SNVS_HP Version ID Register 2 */

#define SNVS_HPVIDR2_CONFIGOPT_SHIFT  (0)       /* Bits 0-7: SNVS Configuration Options */
#define SNVS_HPVIDR2_CONFIGOPT_MASK   (0xff << SNVS_HPVIDR2_CONFIGOPT_SHIFT)
#define SNVS_HPVIDR2_ECOREV_SHIFT     (8)       /* Bits 8-15: SNVS ECO Revision */
#define SNVS_HPVIDR2_ECOREV_MASK      (0xff << SNVS_HPVIDR2_ECOREV_SHIFT)
#define SNVS_HPVIDR2_INTGOPT_SHIFT    (16)      /* Bits 16-23: SNVS Integration Options */
#define SNVS_HPVIDR2_INTGOPT_MASK     (0xff << SNVS_HPVIDR2_INTGOPT_SHIFT)
#define SNVS_HPVIDR2_IPERA_SHIFT      (24)      /* Bits 24-31: IP Era */
#define SNVS_HPVIDR2_IPERA_MASK       (0xff << SNVS_HPVIDR2_IPERA_SHIFT)

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_SNVS_H */
