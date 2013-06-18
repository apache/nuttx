/************************************************************************************
 * arch/arm/src/kl/chip/kl_tsi.h
 *
 *   Copyright (C) 2013 Alan Carvalho de Assis
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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

#ifndef __ARCH_ARM_SRC_KL_CHIP_KL_TSI_H
#define __ARCH_ARM_SRC_KL_CHIP_KL_TSI_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define KL_TSI_GENCS_OFFSET            0x0000 /* General Control and Status Register */
#define KL_TSI_DATA_OFFSET             0x0004 /* SCAN control register */
#define KL_TSI_TSHD_OFFSET             0x0008 /* Pin enable register */

/* Register Addresses ***************************************************************/

#define KL_TSI_GENCS                   (KL_TSI_BASE+KL_TSI_GENCS_OFFSET)
#define KL_TSI_DATA                    (KL_TSI_BASE+KL_TSI_DATA_OFFSET)
#define KL_TSI_TSHD                    (KL_TSI_BASE+KL_TSI_TSHD_OFFSET)

/* Register Bit Definitions *********************************************************/

/* General Control and Status Register */
                                                 /* Bit 0: Reserved */
#define TSI_GENCS_CURSW                (1 << 1)  /* Bit 1: Current sources for oscillators swapped */
#define TSI_GENCS_EOSF                 (1 << 2)  /* Bit 2: End of scan flag */
#define TSI_GENCS_SCNIP                (1 << 3)  /* Bit 3: Scan in progress status */
#define TSI_GENCS_STM                  (1 << 4)  /* Bit 4: Scan trigger mode */
#define TSI_GENCS_STPE                 (1 << 5)  /* Bit 5: TSI STOP enable */
#define TSI_GENCS_TSIIEN               (1 << 6)  /* Bit 6: TSI module interrupt enable */
#define TSI_GENCS_TSIEN                (1 << 7)  /* Bit 7: TSI module enable */
#define TSI_GENCS_NSCN_SHIFT           (8)       /* Bits 8-12: Electrode oscillator count used in a scan */
#define TSI_GENCS_NSCN_MASK            (31 << TSI_GENCS_NSCN_SHIFT)
#  define TSI_GENCS_NSCN_TIMES(n)      (((n)-1) << TSI_GENCS_NSCN_SHIFT) /* n times per electrode,n=1..32 */
#define TSI_GENCS_PS_SHIFT             (13)      /* Bits 13-15: Prescaler value */
#define TSI_GENCS_PS_MASK              (7 << TSI_GENCS_PS_SHIFT)
#  define TSI_GENCS_PS_DIV1            (0 << TSI_GENCS_PS_SHIFT) /* Electrode oscillator / 1 */
#  define TSI_GENCS_PS_DIV2            (1 << TSI_GENCS_PS_SHIFT) /* Electrode oscillator / 2 */
#  define TSI_GENCS_PS_DIV4            (2 << TSI_GENCS_PS_SHIFT) /* Electrode oscillator / 4 */
#  define TSI_GENCS_PS_DIV8            (3 << TSI_GENCS_PS_SHIFT) /* Electrode oscillator / 8 */
#  define TSI_GENCS_PS_DIV16           (4 << TSI_GENCS_PS_SHIFT) /* Electrode oscillator / 16 */
#  define TSI_GENCS_PS_DIV32           (5 << TSI_GENCS_PS_SHIFT) /* Electrode oscillator / 32 */
#  define TSI_GENCS_PS_DIV64           (6 << TSI_GENCS_PS_SHIFT) /* Electrode oscillator / 64 */
#  define TSI_GENCS_PS_DIV128          (7 << TSI_GENCS_PS_SHIFT) /* Electrode oscillator / 128 */
#define TSI_GENCS_EXTCHRG_SHIFT        (16)      /* Bits 16-18: Electrode Osc charge/discharge value */
#define TSI_GENCS_EXTCHRG_MASK         (7 << TSI_GENCS_EXTCHRG_SHIFT)
#  define TSI_GENCS_EXTCHRG_500NA      (0 << TSI_GENCS_EXTCHRG_SHIFT)
#  define TSI_GENCS_EXTCHRG_1UA        (1 << TSI_GENCS_EXTCHRG_SHIFT)
#  define TSI_GENCS_EXTCHRG_2UA        (2 << TSI_GENCS_EXTCHRG_SHIFT)
#  define TSI_GENCS_EXTCHRG_4UA        (3 << TSI_GENCS_EXTCHRG_SHIFT)
#  define TSI_GENCS_EXTCHRG_8UA        (4 << TSI_GENCS_EXTCHRG_SHIFT)
#  define TSI_GENCS_EXTCHRG_16UA       (5 << TSI_GENCS_EXTCHRG_SHIFT)
#  define TSI_GENCS_EXTCHRG_32UA       (6 << TSI_GENCS_EXTCHRG_SHIFT)
#  define TSI_GENCS_EXTCHRG_64A        (7 << TSI_GENCS_EXTCHRG_SHIFT)
#define TSI_GENCS_DVOLT_SHIFT          (19)      /* Bits 19-20: Oscilattor voltage rails */
#define TSI_GENCS_DVOLT_MASK           (3 << TSI_GENCS_DVOLT_SHIFT)
#  define TSI_GENCS_DVOLT_1p03V        (0 << TSI_GENCS_DVOLT_SHIFT)
#  define TSI_GENCS_DVOLT_0p73V        (1 << TSI_GENCS_DVOLT_SHIFT)
#  define TSI_GENCS_DVOLT_0p43V        (2 << TSI_GENCS_DVOLT_SHIFT)
#  define TSI_GENCS_DVOLT_0p29V        (3 << TSI_GENCS_DVOLT_SHIFT)
#define TSI_GENCS_REFCHRG_SHIFT        (21)      /* Bits 21-23: Reference Osc charge/discharge value */
#define TSI_GENCS_REFCHRG_MASK         (7 << TSI_GENCS_REFCHRG_SHIFT)
#  define TSI_GENCS_REFCHRG_500NA      (0 << TSI_GENCS_REFCHRG_SHIFT)
#  define TSI_GENCS_REFCHRG_1UA        (1 << TSI_GENCS_REFCHRG_SHIFT)
#  define TSI_GENCS_REFCHRG_2UA        (2 << TSI_GENCS_REFCHRG_SHIFT)
#  define TSI_GENCS_REFCHRG_4UA        (3 << TSI_GENCS_REFCHRG_SHIFT)
#  define TSI_GENCS_REFCHRG_8UA        (4 << TSI_GENCS_REFCHRG_SHIFT)
#  define TSI_GENCS_REFCHRG_16UA       (5 << TSI_GENCS_REFCHRG_SHIFT)
#  define TSI_GENCS_REFCHRG_32UA       (6 << TSI_GENCS_REFCHRG_SHIFT)
#  define TSI_GENCS_REFCHRG_64UA       (7 << TSI_GENCS_REFCHRG_SHIFT)
#define TSI_GENCS_MODE_SHIFT           (24)      /* Bits 24-27: Analog mode setup and status bits */
#define TSI_GENCS_MODE_MASK            (15 << TSI_GENCS_MODE_SHIFT)
#  define TSI_GENCS_MODE_CAPSENSING    (0 << TSI_GENCS_MODE_SHIFT)
#  define TSI_GENCS_MODE_SGTSHD_NOFREQ (4 << TSI_GENCS_MODE_SHIFT)
#  define TSI_GENCS_MODE_SGTSHD_FRQLIM (8 << TSI_GENCS_MODE_SHIFT)
#  define TSI_GENCS_MODE_AUTODETECT    (12 << TSI_GENCS_MODE_SHIFT)
#define TSI_GENCS_ESOR                 (1 << 28) /* Bit 28: End/Out-of-range interrupt selection */
                                                 /* Bits 29-30: Reserved */
#define TSI_GENCS_OUTRFG               (1 << 31) /* Bit 31: Out of range flag */

/* SCAN control register */

#define TSI_DATA_TSICNT_SHIFT          (0)       /* Bits 0-15: TSI Conversion counter value */
#define TSI_DATA_TSICNT_MASK           (0xffff << TSI_DATA_TSICNT_SHIFT)
                                                 /* Bits 16-21: Reserved */
#define TSI_DATA_SWTS                  (1 << 22) /* Bit 22: Software trigger start */
#define TSI_DATA_DMAEN                 (1 << 23) /* Bit 23: DMA Transfer enabled */
                                                 /* Bits 24-27: Reserved */
#define TSI_DATA_TSICH_SHIFT           (28)      /* Bits 28-31: Current channel to be measured */
#define TSI_DATA_TSICH_MASK            (15 << TSI_DATA_TSICH_SHIFT)
#  define TSI_DATA_TSICH(n)            (n << TSI_DATA_TSICH_SHIFT)  /* Channel to measure, n=0..15 */

/* Channel n threshold register */

#define TSI_THRESHLD_HTHH_SHIFT        (0)       /* Bits 0-15: High threshold value */
#define TSI_THRESHLD_HTHH_MASK         (0xffff << TSI_THRESHLD_HTHH_SHIFT)
#define TSI_THRESHLD_LTHH_SHIFT        (16)      /* Bits 16-31: Low threshold value */
#define TSI_THRESHLD_LTHH_MASK         (0xffff << TSI_THRESHLD_LTHH_SHIFT)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_KL_CHIP_KL_TSI_H */
