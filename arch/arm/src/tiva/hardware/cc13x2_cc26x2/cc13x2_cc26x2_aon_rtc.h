/********************************************************************************************************************
 * arch/arm/src/tiva/hardware/cc13x2_cc26x2/cc13x2_cc26x2_aon_rtc.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *
 * Technical content derives from a TI header file that has a compatible BSD license:
 *
 *   Copyright (c) 2015-2017, Texas Instruments Incorporated
 *   All rights reserved.
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
 ********************************************************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_CC13X2_CC26X2_AON_RTC_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_CC13X2_CC26X2_AON_RTC_H

/********************************************************************************************************************
 * Included Files
 ********************************************************************************************************************/

#include <nuttx/config.h>
#include "hardware/tiva_memorymap.h"

/********************************************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************************************/

/* AON RTC Register Offsets *****************************************************************************************/

#define TIVA_AON_RTC_CTL_OFFSET         0x0000  /* Control */
#define TIVA_AON_RTC_EVFLAGS_OFFSET     0x0004  /* Event Flags, RTC Status */
#define TIVA_AON_RTC_SEC_OFFSET         0x0008  /* Second Counter Value, Integer Part */
#define TIVA_AON_RTC_SUBSEC_OFFSET      0x000c  /* Second Counter Value, Fractional Part */
#define TIVA_AON_RTC_SUBSECINC_OFFSET   0x0010  /* Subseconds Increment */
#define TIVA_AON_RTC_CHCTL_OFFSET       0x0014  /* Channel Configuration */
#define TIVA_AON_RTC_CH0CMP_OFFSET      0x0018  /* Channel 0 Compare Value */
#define TIVA_AON_RTC_CH1CMP_OFFSET      0x001c  /* Channel 1 Compare Value */
#define TIVA_AON_RTC_CH2CMP_OFFSET      0x0020  /* Channel 2 Compare Value */
#define TIVA_AON_RTC_CH2CMPINC_OFFSET   0x0024  /* Channel 2 Compare Value Auto-increment */
#define TIVA_AON_RTC_CH1CAPT_OFFSET     0x0028  /* Channel 1 Capture Value */
#define TIVA_AON_RTC_SYNC_OFFSET        0x002c  /* AON Synchronization */
#define TIVA_AON_RTC_TIME_OFFSET        0x0030  /* Current  Counter Value */
#define TIVA_AON_RTC_SYNCLF_OFFSET      0x0034  /* Synchronization to SCLK_LF */

/* AON RTC Register Addresses ***************************************************************************************/

#define TIVA_AON_RTC_CTL                (TIVA_AON_RTC_BASE + TIVA_AON_RTC_CTL_OFFSET)
#define TIVA_AON_RTC_EVFLAGS            (TIVA_AON_RTC_BASE + TIVA_AON_RTC_EVFLAGS_OFFSET)
#define TIVA_AON_RTC_SEC                (TIVA_AON_RTC_BASE + TIVA_AON_RTC_SEC_OFFSET)
#define TIVA_AON_RTC_SUBSEC             (TIVA_AON_RTC_BASE + TIVA_AON_RTC_SUBSEC_OFFSET)
#define TIVA_AON_RTC_SUBSECINC          (TIVA_AON_RTC_BASE + TIVA_AON_RTC_SUBSECINC_OFFSET)
#define TIVA_AON_RTC_CHCTL              (TIVA_AON_RTC_BASE + TIVA_AON_RTC_CHCTL_OFFSET)
#define TIVA_AON_RTC_CH0CMP             (TIVA_AON_RTC_BASE + TIVA_AON_RTC_CH0CMP_OFFSET)
#define TIVA_AON_RTC_CH1CMP             (TIVA_AON_RTC_BASE + TIVA_AON_RTC_CH1CMP_OFFSET)
#define TIVA_AON_RTC_CH2CMP             (TIVA_AON_RTC_BASE + TIVA_AON_RTC_CH2CMP_OFFSET)
#define TIVA_AON_RTC_CH2CMPINC          (TIVA_AON_RTC_BASE + TIVA_AON_RTC_CH2CMPINC_OFFSET)
#define TIVA_AON_RTC_CH1CAPT            (TIVA_AON_RTC_BASE + TIVA_AON_RTC_CH1CAPT_OFFSET)
#define TIVA_AON_RTC_SYNC               (TIVA_AON_RTC_BASE + TIVA_AON_RTC_SYNC_OFFSET)
#define TIVA_AON_RTC_TIME               (TIVA_AON_RTC_BASE + TIVA_AON_RTC_TIME_OFFSET)
#define TIVA_AON_RTC_SYNCLF             (TIVA_AON_RTC_BASE + TIVA_AON_RTC_SYNCLF_OFFSET)

/* AON RTC Bitfield Definitions *************************************************************************************/

/* TIVA_AON_RTC_CTL */

#define AON_RTC_CTL_EN                  (1 << 0)  /* Bit 0:  Enable RTC counter */
#define AON_RTC_CTL_RTC_UPD_EN          (1 << 1)  /* Bit 1:  Enable 16-KHz RTC_UPD output */
#define AON_RTC_CTL_RTC_4KHZ_EN         (1 << 2)  /* Bit 2:  Enabvle 4KHz reference output */
#define AON_RTC_CTL_RESET               (1 << 7)  /* Bit 7:  RTC counter reset */
#define AON_RTC_CTL_EV_DELAY_SHIFT      (8)       /* Bits 8-11:  Number SCLK_LF delay for events */
#define AON_RTC_CTL_EV_DELAY_MASK       (15 << AON_RTC_CTL_EV_DELAY_SHIFT)
#  define AON_RTC_CTL_EV_DELAY_D0       (0 << AON_RTC_CTL_EV_DELAY_SHIFT)  /* No delay on event */
#  define AON_RTC_CTL_EV_DELAY_D1       (1 << AON_RTC_CTL_EV_DELAY_SHIFT)  /* Delay by 1 clock cycle */
#  define AON_RTC_CTL_EV_DELAY_D2       (2 << AON_RTC_CTL_EV_DELAY_SHIFT)  /* Delay by 2 clock cycles */
#  define AON_RTC_CTL_EV_DELAY_D4       (3 << AON_RTC_CTL_EV_DELAY_SHIFT)  /* Delay by 4 clock cycles */
#  define AON_RTC_CTL_EV_DELAY_D8       (4 << AON_RTC_CTL_EV_DELAY_SHIFT)  /* Delay by 8 clock cycles */
#  define AON_RTC_CTL_EV_DELAY_D16      (5 << AON_RTC_CTL_EV_DELAY_SHIFT)  /* Delay by 16 clock cycles */
#  define AON_RTC_CTL_EV_DELAY_D32      (6 << AON_RTC_CTL_EV_DELAY_SHIFT)  /* Delay by 32 clock cycles */
#  define AON_RTC_CTL_EV_DELAY_D48      (7 << AON_RTC_CTL_EV_DELAY_SHIFT)  /* Delay by 48 clock cycles */
#  define AON_RTC_CTL_EV_DELAY_D64      (8 << AON_RTC_CTL_EV_DELAY_SHIFT)  /* Delay by 64 clock cycles */
#  define AON_RTC_CTL_EV_DELAY_D80      (9 << AON_RTC_CTL_EV_DELAY_SHIFT)  /* Delay by 80 clock cycles */
#  define AON_RTC_CTL_EV_DELAY_D96      (10 << AON_RTC_CTL_EV_DELAY_SHIFT) /* Delay by 96 clock cycles */
#  define AON_RTC_CTL_EV_DELAY_D112     (11 << AON_RTC_CTL_EV_DELAY_SHIFT) /* Delay by 112 clock cycles */
#  define AON_RTC_CTL_EV_DELAY_D128     (12 << AON_RTC_CTL_EV_DELAY_SHIFT) /* Delay by 128 clock cycles */
#  define AON_RTC_CTL_EV_DELAY_D144     (13 << AON_RTC_CTL_EV_DELAY_SHIFT) /* Delay by 144 clock cycles */
#define AON_RTC_CTL_COMB_EV_MASK_SHIFT  (16)      /* Bits 16-18:  Select how delayed event form combined events */
#define AON_RTC_CTL_COMB_EV_MASK_MASK   (7 << AON_RTC_CTL_COMB_EV_MASK_SHIFT)
#  define AON_RTC_CTL_COMB_EV_MASK_NONE (0 << AON_RTC_CTL_COMB_EV_MASK_SHIFT) /* No event for combined event */
#  define AON_RTC_CTL_COMB_EV_MASK_CH0  (1 << AON_RTC_CTL_COMB_EV_MASK_SHIFT) /* Use Chan 0 delayed event to combine */
#  define AON_RTC_CTL_COMB_EV_MASK_CH1  (2 << AON_RTC_CTL_COMB_EV_MASK_SHIFT) /* Use Chan 1 delayed event to combine */
#  define AON_RTC_CTL_COMB_EV_MASK_CH2  (4 << AON_RTC_CTL_COMB_EV_MASK_SHIFT) /* Use Chan 2 delayed event to combine */

/* TIVA_AON_RTC_EVFLAGS */

#define AON_RTC_EVFLAGS_CH0             (1 << 0)  /* Bit 0:  Channel 0 event flag */
#define AON_RTC_EVFLAGS_CH1             (1 << 8)  /* Bit 8:  Channel 1 event flag */
#define AON_RTC_EVFLAGS_CH2             (1 << 16) /* Bit 16: Channel 2 event flag */

/* TIVA_AON_RTC_SEC (32-bit value, units of seconds) */
/* TIVA_AON_RTC_SUBSEC (32-bit value, b32 fractional seconds) */
/* TIVA_AON_RTC_SUBSECINC (32-bit value) */

/* TIVA_AON_RTC_CHCTL */

#define AON_RTC_CHCTL_CH0_EN            (1 << 0)  /* Bit 0:  RTC Channel 0 enable */
#define AON_RTC_CHCTL_CH1_EN            (1 << 8)  /* Bit 8:  RTC Channel 1 enable */
#define AON_RTC_CHCTL_CH1_CAPT_EN       (1 << 9)  /* Bit 9:  Channel 1 mode */
#  define AON_RTC_CHCTL_CH1_CAPT_CMP    (0)                        /* Compare mode */
#  define AON_RTC_CHCTL_CH1_CAPT_CAPT   AON_RTC_CHCTL_CH1_CAPT_EN  /* Capture mode */
#define AON_RTC_CHCTL_CH2_EN            (1 << 16) /* Bit 16: RTC Channel 2 Enable */
#define AON_RTC_CHCTL_CH2_CONT_EN       (1 << 18) /* Bit 18: Enable Channel 2 Continuous Operation */

/* TIVA_AON_RTC_CH0CMP (32-bit value) */
/* TIVA_AON_RTC_CH1CMP (32-bit value) */
/* TIVA_AON_RTC_CH2CMP (32-bit value) */
/* TIVA_AON_RTC_CH2CMPINC (32-bit value) */

/* TIVA_AON_RTC_CH1CAPT */

#define AON_RTC_CH1CAPT_SUBSEC_SHIFT    (0)       /* Value of SUBSEC.VALUE bits 31:16 at capture time */
#define AON_RTC_CH1CAPT_SUBSEC_MASK     (0xffff << AON_RTC_CH1CAPT_SUBSEC_SHIFT)
#define AON_RTC_CH1CAPT_SEC_SHIFT       (16)      /* Bits 16-31: Value of SEC.VALUE bits 15:0 at capture time */
#define AON_RTC_CH1CAPT_SEC_MASK        (0xffff << AON_RTC_CH1CAPT_SEC_SHIFT)

/* TIVA_AON_RTC_SYNC */

#define AON_RTC_SYNC_WBUSY              (1 << 0)  /* Bit 0:
                                                   * Read: Outstanding MCU/AON write request
                                                   * Write: Force wait for SCLK_MF edge */

/* TIVA_AON_RTC_TIME */

#define AON_RTC_TIME_SUBSEC_H_SHIFT     (0)       /* Bits 0-15: Upper halfword of SUBSEC register */
#define AON_RTC_TIME_SUBSEC_H_MASK      (0xffff << AON_RTC_TIME_SUBSEC_H_SHIFT)
#define AON_RTC_TIME_SEC_L_SHIFT        (16)      /* Bits 16-31:  Lower halfward of SEC register */
#define AON_RTC_TIME_SEC_L_MASK         (0xffff << AON_RTC_TIME_SEC_L_SHIFT)

/* TIVA_AON_RTC_SYNCLF */

#define AON_RTC_SYNCLF_PHASE            (1 << 0)  /* Bit 0: SCLK_LF PHASE */
#  define AON_RTC_SYNCLF_FALLING        (0)
#  define AON_RTC_SYNCLF_RISING         AON_RTC_SYNCLF_PHASE

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_CC13X2_CC26X2_AON_RTC_H */
