/****************************************************************************
 * arch/arm/src/tiva/hardware/cc13x2_cc26x2/cc13x2_cc26x2_smph.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *
 * Technical content derives from a TI header file that has a compatible
 * BSD license:
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_CC13X2_CC26X2_SMPH_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_CC13X2_CC26X2_SMPH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/tiva_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SMPH Register Offsets ****************************************************/

#define TIVA_SMPH_SMPH_OFFSET(n)        (0x0000 + ((n) << 2))
#  define TIVA_SMPH_SMPH0_OFFSET        0x0000  /* MCU SEMAPHORE 0 */
#  define TIVA_SMPH_SMPH1_OFFSET        0x0004  /* MCU SEMAPHORE 1 */
#  define TIVA_SMPH_SMPH2_OFFSET        0x0008  /* MCU SEMAPHORE 2 */
#  define TIVA_SMPH_SMPH3_OFFSET        0x000c  /* MCU SEMAPHORE 3 */
#  define TIVA_SMPH_SMPH4_OFFSET        0x0010  /* MCU SEMAPHORE 4 */
#  define TIVA_SMPH_SMPH5_OFFSET        0x0014  /* MCU SEMAPHORE 5 */
#  define TIVA_SMPH_SMPH6_OFFSET        0x0018  /* MCU SEMAPHORE 6 */
#  define TIVA_SMPH_SMPH7_OFFSET        0x001c  /* MCU SEMAPHORE 7 */
#  define TIVA_SMPH_SMPH8_OFFSET        0x0020  /* MCU SEMAPHORE 8 */
#  define TIVA_SMPH_SMPH9_OFFSET        0x0024  /* MCU SEMAPHORE 9 */
#  define TIVA_SMPH_SMPH10_OFFSET       0x0028  /* MCU SEMAPHORE 10 */
#  define TIVA_SMPH_SMPH11_OFFSET       0x002c  /* MCU SEMAPHORE 11 */
#  define TIVA_SMPH_SMPH12_OFFSET       0x0030  /* MCU SEMAPHORE 12 */
#  define TIVA_SMPH_SMPH13_OFFSET       0x0034  /* MCU SEMAPHORE 13 */
#  define TIVA_SMPH_SMPH14_OFFSET       0x0038  /* MCU SEMAPHORE 14 */
#  define TIVA_SMPH_SMPH15_OFFSET       0x003c  /* MCU SEMAPHORE 15 */
#  define TIVA_SMPH_SMPH16_OFFSET       0x0040  /* MCU SEMAPHORE 16 */
#  define TIVA_SMPH_SMPH17_OFFSET       0x0044  /* MCU SEMAPHORE 17 */
#  define TIVA_SMPH_SMPH18_OFFSET       0x0048  /* MCU SEMAPHORE 18 */
#  define TIVA_SMPH_SMPH19_OFFSET       0x004c  /* MCU SEMAPHORE 19 */
#  define TIVA_SMPH_SMPH20_OFFSET       0x0050  /* MCU SEMAPHORE 20 */
#  define TIVA_SMPH_SMPH21_OFFSET       0x0054  /* MCU SEMAPHORE 21 */
#  define TIVA_SMPH_SMPH22_OFFSET       0x0058  /* MCU SEMAPHORE 22 */
#  define TIVA_SMPH_SMPH23_OFFSET       0x005c  /* MCU SEMAPHORE 23 */
#  define TIVA_SMPH_SMPH24_OFFSET       0x0060  /* MCU SEMAPHORE 24 */
#  define TIVA_SMPH_SMPH25_OFFSET       0x0064  /* MCU SEMAPHORE 25 */
#  define TIVA_SMPH_SMPH26_OFFSET       0x0068  /* MCU SEMAPHORE 26 */
#  define TIVA_SMPH_SMPH27_OFFSET       0x006c  /* MCU SEMAPHORE 27 */
#  define TIVA_SMPH_SMPH28_OFFSET       0x0070  /* MCU SEMAPHORE 28 */
#  define TIVA_SMPH_SMPH29_OFFSET       0x0074  /* MCU SEMAPHORE 29 */
#  define TIVA_SMPH_SMPH30_OFFSET       0x0078  /* MCU SEMAPHORE 30 */
#  define TIVA_SMPH_SMPH31_OFFSET       0x007c  /* MCU SEMAPHORE 31 */

#define TIVA_SMPH_PEEK_OFFSET(n)        (0x0800 + ((n) << 2))
#  define TIVA_SMPH_PEEK0_OFFSET        0x0800  /* MCU SEMAPHORE 0 ALIAS */
#  define TIVA_SMPH_PEEK1_OFFSET        0x0804  /* MCU SEMAPHORE 1 ALIAS */
#  define TIVA_SMPH_PEEK2_OFFSET        0x0808  /* MCU SEMAPHORE 2 ALIAS */
#  define TIVA_SMPH_PEEK3_OFFSET        0x080c  /* MCU SEMAPHORE 3 ALIAS */
#  define TIVA_SMPH_PEEK4_OFFSET        0x0810  /* MCU SEMAPHORE 4 ALIAS */
#  define TIVA_SMPH_PEEK5_OFFSET        0x0814  /* MCU SEMAPHORE 5 ALIAS */
#  define TIVA_SMPH_PEEK6_OFFSET        0x0818  /* MCU SEMAPHORE 6 ALIAS */
#  define TIVA_SMPH_PEEK7_OFFSET        0x081c  /* MCU SEMAPHORE 7 ALIAS */
#  define TIVA_SMPH_PEEK8_OFFSET        0x0820  /* MCU SEMAPHORE 8 ALIAS */
#  define TIVA_SMPH_PEEK9_OFFSET        0x0824  /* MCU SEMAPHORE 9 ALIAS */
#  define TIVA_SMPH_PEEK10_OFFSET       0x0828  /* MCU SEMAPHORE 10 ALIAS */
#  define TIVA_SMPH_PEEK11_OFFSET       0x082c  /* MCU SEMAPHORE 11 ALIAS */
#  define TIVA_SMPH_PEEK12_OFFSET       0x0830  /* MCU SEMAPHORE 12 ALIAS */
#  define TIVA_SMPH_PEEK13_OFFSET       0x0834  /* MCU SEMAPHORE 13 ALIAS */
#  define TIVA_SMPH_PEEK14_OFFSET       0x0838  /* MCU SEMAPHORE 14 ALIAS */
#  define TIVA_SMPH_PEEK15_OFFSET       0x083c  /* MCU SEMAPHORE 15 ALIAS */
#  define TIVA_SMPH_PEEK16_OFFSET       0x0840  /* MCU SEMAPHORE 16 ALIAS */
#  define TIVA_SMPH_PEEK17_OFFSET       0x0844  /* MCU SEMAPHORE 17 ALIAS */
#  define TIVA_SMPH_PEEK18_OFFSET       0x0848  /* MCU SEMAPHORE 18 ALIAS */
#  define TIVA_SMPH_PEEK19_OFFSET       0x084c  /* MCU SEMAPHORE 19 ALIAS */
#  define TIVA_SMPH_PEEK20_OFFSET       0x0850  /* MCU SEMAPHORE 20 ALIAS */
#  define TIVA_SMPH_PEEK21_OFFSET       0x0854  /* MCU SEMAPHORE 21 ALIAS */
#  define TIVA_SMPH_PEEK22_OFFSET       0x0858  /* MCU SEMAPHORE 22 ALIAS */
#  define TIVA_SMPH_PEEK23_OFFSET       0x085c  /* MCU SEMAPHORE 23 ALIAS */
#  define TIVA_SMPH_PEEK24_OFFSET       0x0860  /* MCU SEMAPHORE 24 ALIAS */
#  define TIVA_SMPH_PEEK25_OFFSET       0x0864  /* MCU SEMAPHORE 25 ALIAS */
#  define TIVA_SMPH_PEEK26_OFFSET       0x0868  /* MCU SEMAPHORE 26 ALIAS */
#  define TIVA_SMPH_PEEK27_OFFSET       0x086c  /* MCU SEMAPHORE 27 ALIAS */
#  define TIVA_SMPH_PEEK28_OFFSET       0x0870  /* MCU SEMAPHORE 28 ALIAS */
#  define TIVA_SMPH_PEEK29_OFFSET       0x0874  /* MCU SEMAPHORE 29 ALIAS */
#  define TIVA_SMPH_PEEK30_OFFSET       0x0878  /* MCU SEMAPHORE 30 ALIAS */
#  define TIVA_SMPH_PEEK31_OFFSET       0x087c  /* MCU SEMAPHORE 31 ALIAS */

/* SMPH Register Addresses **************************************************/

#define TIVA_SMPH_SMPH(n)               (TIVA_SMPH_BASE + TIVA_SMPH_SMPH_OFFSET(n))
#  define TIVA_SMPH_SMPH0               (TIVA_SMPH_BASE + TIVA_SMPH_SMPH0_OFFSET)
#  define TIVA_SMPH_SMPH1               (TIVA_SMPH_BASE + TIVA_SMPH_SMPH1_OFFSET)
#  define TIVA_SMPH_SMPH2               (TIVA_SMPH_BASE + TIVA_SMPH_SMPH2_OFFSET)
#  define TIVA_SMPH_SMPH3               (TIVA_SMPH_BASE + TIVA_SMPH_SMPH3_OFFSET)
#  define TIVA_SMPH_SMPH4               (TIVA_SMPH_BASE + TIVA_SMPH_SMPH4_OFFSET)
#  define TIVA_SMPH_SMPH5               (TIVA_SMPH_BASE + TIVA_SMPH_SMPH5_OFFSET)
#  define TIVA_SMPH_SMPH6               (TIVA_SMPH_BASE + TIVA_SMPH_SMPH6_OFFSET)
#  define TIVA_SMPH_SMPH7               (TIVA_SMPH_BASE + TIVA_SMPH_SMPH7_OFFSET)
#  define TIVA_SMPH_SMPH8               (TIVA_SMPH_BASE + TIVA_SMPH_SMPH8_OFFSET)
#  define TIVA_SMPH_SMPH9               (TIVA_SMPH_BASE + TIVA_SMPH_SMPH9_OFFSET)
#  define TIVA_SMPH_SMPH10              (TIVA_SMPH_BASE + TIVA_SMPH_SMPH10_OFFSET)
#  define TIVA_SMPH_SMPH11              (TIVA_SMPH_BASE + TIVA_SMPH_SMPH11_OFFSET)
#  define TIVA_SMPH_SMPH12              (TIVA_SMPH_BASE + TIVA_SMPH_SMPH12_OFFSET)
#  define TIVA_SMPH_SMPH13              (TIVA_SMPH_BASE + TIVA_SMPH_SMPH13_OFFSET)
#  define TIVA_SMPH_SMPH14              (TIVA_SMPH_BASE + TIVA_SMPH_SMPH14_OFFSET)
#  define TIVA_SMPH_SMPH15              (TIVA_SMPH_BASE + TIVA_SMPH_SMPH15_OFFSET)
#  define TIVA_SMPH_SMPH16              (TIVA_SMPH_BASE + TIVA_SMPH_SMPH16_OFFSET)
#  define TIVA_SMPH_SMPH17              (TIVA_SMPH_BASE + TIVA_SMPH_SMPH17_OFFSET)
#  define TIVA_SMPH_SMPH18              (TIVA_SMPH_BASE + TIVA_SMPH_SMPH18_OFFSET)
#  define TIVA_SMPH_SMPH19              (TIVA_SMPH_BASE + TIVA_SMPH_SMPH19_OFFSET)
#  define TIVA_SMPH_SMPH20              (TIVA_SMPH_BASE + TIVA_SMPH_SMPH20_OFFSET)
#  define TIVA_SMPH_SMPH21              (TIVA_SMPH_BASE + TIVA_SMPH_SMPH21_OFFSET)
#  define TIVA_SMPH_SMPH22              (TIVA_SMPH_BASE + TIVA_SMPH_SMPH22_OFFSET)
#  define TIVA_SMPH_SMPH23              (TIVA_SMPH_BASE + TIVA_SMPH_SMPH23_OFFSET)
#  define TIVA_SMPH_SMPH24              (TIVA_SMPH_BASE + TIVA_SMPH_SMPH24_OFFSET)
#  define TIVA_SMPH_SMPH25              (TIVA_SMPH_BASE + TIVA_SMPH_SMPH25_OFFSET)
#  define TIVA_SMPH_SMPH26              (TIVA_SMPH_BASE + TIVA_SMPH_SMPH26_OFFSET)
#  define TIVA_SMPH_SMPH27              (TIVA_SMPH_BASE + TIVA_SMPH_SMPH27_OFFSET)
#  define TIVA_SMPH_SMPH28              (TIVA_SMPH_BASE + TIVA_SMPH_SMPH28_OFFSET)
#  define TIVA_SMPH_SMPH29              (TIVA_SMPH_BASE + TIVA_SMPH_SMPH29_OFFSET)
#  define TIVA_SMPH_SMPH30              (TIVA_SMPH_BASE + TIVA_SMPH_SMPH30_OFFSET)
#  define TIVA_SMPH_SMPH31              (TIVA_SMPH_BASE + TIVA_SMPH_SMPH31_OFFSET)

#define TIVA_SMPH_PEEK(n)               (TIVA_SMPH_BASE + TIVA_SMPH_PEEK_OFFSET(n))
#  define TIVA_SMPH_PEEK0               (TIVA_SMPH_BASE + TIVA_SMPH_PEEK0_OFFSET)
#  define TIVA_SMPH_PEEK1               (TIVA_SMPH_BASE + TIVA_SMPH_PEEK1_OFFSET)
#  define TIVA_SMPH_PEEK2               (TIVA_SMPH_BASE + TIVA_SMPH_PEEK2_OFFSET)
#  define TIVA_SMPH_PEEK3               (TIVA_SMPH_BASE + TIVA_SMPH_PEEK3_OFFSET)
#  define TIVA_SMPH_PEEK4               (TIVA_SMPH_BASE + TIVA_SMPH_PEEK4_OFFSET)
#  define TIVA_SMPH_PEEK5               (TIVA_SMPH_BASE + TIVA_SMPH_PEEK5_OFFSET)
#  define TIVA_SMPH_PEEK6               (TIVA_SMPH_BASE + TIVA_SMPH_PEEK6_OFFSET)
#  define TIVA_SMPH_PEEK7               (TIVA_SMPH_BASE + TIVA_SMPH_PEEK7_OFFSET)
#  define TIVA_SMPH_PEEK8               (TIVA_SMPH_BASE + TIVA_SMPH_PEEK8_OFFSET)
#  define TIVA_SMPH_PEEK9               (TIVA_SMPH_BASE + TIVA_SMPH_PEEK9_OFFSET)
#  define TIVA_SMPH_PEEK10              (TIVA_SMPH_BASE + TIVA_SMPH_PEEK10_OFFSET)
#  define TIVA_SMPH_PEEK11              (TIVA_SMPH_BASE + TIVA_SMPH_PEEK11_OFFSET)
#  define TIVA_SMPH_PEEK12              (TIVA_SMPH_BASE + TIVA_SMPH_PEEK12_OFFSET)
#  define TIVA_SMPH_PEEK13              (TIVA_SMPH_BASE + TIVA_SMPH_PEEK13_OFFSET)
#  define TIVA_SMPH_PEEK14              (TIVA_SMPH_BASE + TIVA_SMPH_PEEK14_OFFSET)
#  define TIVA_SMPH_PEEK15              (TIVA_SMPH_BASE + TIVA_SMPH_PEEK15_OFFSET)
#  define TIVA_SMPH_PEEK16              (TIVA_SMPH_BASE + TIVA_SMPH_PEEK16_OFFSET)
#  define TIVA_SMPH_PEEK17              (TIVA_SMPH_BASE + TIVA_SMPH_PEEK17_OFFSET)
#  define TIVA_SMPH_PEEK18              (TIVA_SMPH_BASE + TIVA_SMPH_PEEK18_OFFSET)
#  define TIVA_SMPH_PEEK19              (TIVA_SMPH_BASE + TIVA_SMPH_PEEK19_OFFSET)
#  define TIVA_SMPH_PEEK20              (TIVA_SMPH_BASE + TIVA_SMPH_PEEK20_OFFSET)
#  define TIVA_SMPH_PEEK21              (TIVA_SMPH_BASE + TIVA_SMPH_PEEK21_OFFSET)
#  define TIVA_SMPH_PEEK22              (TIVA_SMPH_BASE + TIVA_SMPH_PEEK22_OFFSET)
#  define TIVA_SMPH_PEEK23              (TIVA_SMPH_BASE + TIVA_SMPH_PEEK23_OFFSET)
#  define TIVA_SMPH_PEEK24              (TIVA_SMPH_BASE + TIVA_SMPH_PEEK24_OFFSET)
#  define TIVA_SMPH_PEEK25              (TIVA_SMPH_BASE + TIVA_SMPH_PEEK25_OFFSET)
#  define TIVA_SMPH_PEEK26              (TIVA_SMPH_BASE + TIVA_SMPH_PEEK26_OFFSET)
#  define TIVA_SMPH_PEEK27              (TIVA_SMPH_BASE + TIVA_SMPH_PEEK27_OFFSET)
#  define TIVA_SMPH_PEEK28              (TIVA_SMPH_BASE + TIVA_SMPH_PEEK28_OFFSET)
#  define TIVA_SMPH_PEEK29              (TIVA_SMPH_BASE + TIVA_SMPH_PEEK29_OFFSET)
#  define TIVA_SMPH_PEEK30              (TIVA_SMPH_BASE + TIVA_SMPH_PEEK30_OFFSET)
#  define TIVA_SMPH_PEEK31              (TIVA_SMPH_BASE + TIVA_SMPH_PEEK31_OFFSET)

/* SMPH Register Bitfield Definitions ***************************************/

/* TIVA_SMPH_SMPH0-TIVA_SMPH_SMPH31 */

#define SMPH_SMPH_STAT                  (1 << 0)  /* Bit 0:  Semaphore is available */
#  define SMPH_SMPH0_STAT               SMPH_SMPH_STAT
#  define SMPH_SMPH1_STAT               SMPH_SMPH_STAT
#  define SMPH_SMPH2_STAT               SMPH_SMPH_STAT
#  define SMPH_SMPH3_STAT               SMPH_SMPH_STAT
#  define SMPH_SMPH4_STAT               SMPH_SMPH_STAT
#  define SMPH_SMPH5_STAT               SMPH_SMPH_STAT
#  define SMPH_SMPH6_STAT               SMPH_SMPH_STAT
#  define SMPH_SMPH7_STAT               SMPH_SMPH_STAT
#  define SMPH_SMPH8_STAT               SMPH_SMPH_STAT
#  define SMPH_SMPH9_STAT               SMPH_SMPH_STAT
#  define SMPH_SMPH10_STAT              SMPH_SMPH_STAT
#  define SMPH_SMPH11_STAT              SMPH_SMPH_STAT
#  define SMPH_SMPH12_STAT              SMPH_SMPH_STAT
#  define SMPH_SMPH13_STAT              SMPH_SMPH_STAT
#  define SMPH_SMPH14_STAT              SMPH_SMPH_STAT
#  define SMPH_SMPH15_STAT              SMPH_SMPH_STAT
#  define SMPH_SMPH16_STAT              SMPH_SMPH_STAT
#  define SMPH_SMPH17_STAT              SMPH_SMPH_STAT
#  define SMPH_SMPH18_STAT              SMPH_SMPH_STAT
#  define SMPH_SMPH19_STAT              SMPH_SMPH_STAT
#  define SMPH_SMPH20_STAT              SMPH_SMPH_STAT
#  define SMPH_SMPH21_STAT              SMPH_SMPH_STAT
#  define SMPH_SMPH22_STAT              SMPH_SMPH_STAT
#  define SMPH_SMPH23_STAT              SMPH_SMPH_STAT
#  define SMPH_SMPH24_STAT              SMPH_SMPH_STAT
#  define SMPH_SMPH25_STAT              SMPH_SMPH_STAT
#  define SMPH_SMPH26_STAT              SMPH_SMPH_STAT
#  define SMPH_SMPH27_STAT              SMPH_SMPH_STAT
#  define SMPH_SMPH28_STAT              SMPH_SMPH_STAT
#  define SMPH_SMPH29_STAT              SMPH_SMPH_STAT
#  define SMPH_SMPH30_STAT              SMPH_SMPH_STAT
#  define SMPH_SMPH31_STAT              SMPH_SMPH_STAT

/* TIVA_SMPH_PEEK0-TIVA_SMPH_PEEK31 */

#define SMPH_PEEK_STAT                  (1 << 0)  /* Bit 0:  Semaphore is available */
#  define SMPH_PEEK0_STAT               SMPH_PEEK_STAT
#  define SMPH_PEEK1_STAT               SMPH_PEEK_STAT
#  define SMPH_PEEK2_STAT               SMPH_PEEK_STAT
#  define SMPH_PEEK3_STAT               SMPH_PEEK_STAT
#  define SMPH_PEEK4_STAT               SMPH_PEEK_STAT
#  define SMPH_PEEK5_STAT               SMPH_PEEK_STAT
#  define SMPH_PEEK6_STAT               SMPH_PEEK_STAT
#  define SMPH_PEEK7_STAT               SMPH_PEEK_STAT
#  define SMPH_PEEK8_STAT               SMPH_PEEK_STAT
#  define SMPH_PEEK9_STAT               SMPH_PEEK_STAT
#  define SMPH_PEEK10_STAT              SMPH_PEEK_STAT
#  define SMPH_PEEK11_STAT              SMPH_PEEK_STAT
#  define SMPH_PEEK12_STAT              SMPH_PEEK_STAT
#  define SMPH_PEEK13_STAT              SMPH_PEEK_STAT
#  define SMPH_PEEK14_STAT              SMPH_PEEK_STAT
#  define SMPH_PEEK15_STAT              SMPH_PEEK_STAT
#  define SMPH_PEEK16_STAT              SMPH_PEEK_STAT
#  define SMPH_PEEK17_STAT              SMPH_PEEK_STAT
#  define SMPH_PEEK18_STAT              SMPH_PEEK_STAT
#  define SMPH_PEEK19_STAT              SMPH_PEEK_STAT
#  define SMPH_PEEK20_STAT              SMPH_PEEK_STAT
#  define SMPH_PEEK21_STAT              SMPH_PEEK_STAT
#  define SMPH_PEEK22_STAT              SMPH_PEEK_STAT
#  define SMPH_PEEK23_STAT              SMPH_PEEK_STAT
#  define SMPH_PEEK24_STAT              SMPH_PEEK_STAT
#  define SMPH_PEEK25_STAT              SMPH_PEEK_STAT
#  define SMPH_PEEK26_STAT              SMPH_PEEK_STAT
#  define SMPH_PEEK27_STAT              SMPH_PEEK_STAT
#  define SMPH_PEEK28_STAT              SMPH_PEEK_STAT
#  define SMPH_PEEK29_STAT              SMPH_PEEK_STAT
#  define SMPH_PEEK30_STAT              SMPH_PEEK_STAT
#  define SMPH_PEEK31_STAT              SMPH_PEEK_STAT

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_CC13X2_CC26X2_SMPH_H */
