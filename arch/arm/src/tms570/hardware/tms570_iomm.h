/****************************************************************************************************
 * arch/arm/src/tms570/hardware/tms570_iomm.h
 * I/O Muliplexing and Control Module (IOMM) Definitions
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *
 *   TMS570LS04x/03x 16/32-Bit RISC Flash Microcontroller, Technical Reference Manual, Texas
 *   Instruments, Literature Number: SPNU517A, September 2013
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_TMS570_HARDWARE_TMS570_IOMM_H
#define __ARCH_ARM_SRC_TMS570_HARDWARE_TMS570_IOMM_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include "hardware/tms570_memorymap.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Register Offsets *********************************************************************************/

#define TMS570_IOMM_REVISION_OFFSET     0x0000 /* Revision Register */
#define TMS570_IOMM_BOOT_OFFSET         0x0020 /* Boot Mode Register */
#define TMS570_IOMM_KICK0_OFFSET        0x0038 /* Kicker Register 0 */
#define TMS570_IOMM_KICK1_OFFSET        0x003c /* Kicker Register 1 */
#define TMS570_IOMM_ERRRAWSTATUS_OFFSET 0x00e0 /* Error Raw Status / Set Register */
#define TMS570_IOMM_ERRSTATUS_OFFSET    0x00e4 /* Error Enabled Status / Clear Register */
#define TMS570_IOMM_ERRENABLE_OFFSET    0x00e8 /* Error Signaling Enable Register */
#define TMS570_IOMM_ERRENABLECLR_OFFSET 0x00ec /* Error Signaling Enable Clear Register */
#define TMS570_IOMM_FAULTADDRESS_OFFSET 0x00f4 /* Fault Address Register */
#define TMS570_IOMM_FAULTSTATUS_OFFSET  0x00f8 /* Fault Status Register */
#define TMS570_IOMM_FAULTCLR_OFFSET     0x00fc /* Fault Clear Register */

#define TMS570_IOMM_PINMMR_OFFSET(n)    (0x0110 + ((unsigned int)(n) << 2))
#  define TMS570_IOMM_PINMMR0_OFFSET    0x0110 /* Pin Multiplexing Control Register 0 */
#  define TMS570_IOMM_PINMMR1_OFFSET    0x0114 /* Pin Multiplexing Control Register 1 */
#  define TMS570_IOMM_PINMMR2_OFFSET    0x0118 /* Pin Multiplexing Control Register 2 */
#  define TMS570_IOMM_PINMMR3_OFFSET    0x011c /* Pin Multiplexing Control Register 3 */
#  define TMS570_IOMM_PINMMR4_OFFSET    0x0120 /* Pin Multiplexing Control Register 4 */
#  define TMS570_IOMM_PINMMR5_OFFSET    0x0124 /* Pin Multiplexing Control Register 5 */
#  define TMS570_IOMM_PINMMR6_OFFSET    0x0128 /* Pin Multiplexing Control Register 6 */
#  define TMS570_IOMM_PINMMR7_OFFSET    0x012c /* Pin Multiplexing Control Register 7 */
#  define TMS570_IOMM_PINMMR8_OFFSET    0x0130 /* Pin Multiplexing Control Register 8 */
#  define TMS570_IOMM_PINMMR9_OFFSET    0x0134 /* Pin Multiplexing Control Register 9 */
#  define TMS570_IOMM_PINMMR10_OFFSET   0x0138 /* Pin Multiplexing Control Register 10 */
#  define TMS570_IOMM_PINMMR11_OFFSET   0x013c /* Pin Multiplexing Control Register 11 */
#  define TMS570_IOMM_PINMMR12_OFFSET   0x0140 /* Pin Multiplexing Control Register 12 */
#  define TMS570_IOMM_PINMMR13_OFFSET   0x0144 /* Pin Multiplexing Control Register 13 */
#  define TMS570_IOMM_PINMMR14_OFFSET   0x0148 /* Pin Multiplexing Control Register 14 */
#  define TMS570_IOMM_PINMMR15_OFFSET   0x014c /* Pin Multiplexing Control Register 15 */
#  define TMS570_IOMM_PINMMR16_OFFSET   0x0150 /* Pin Multiplexing Control Register 16 */
#  define TMS570_IOMM_PINMMR17_OFFSET   0x0154 /* Pin Multiplexing Control Register 17 */
#  define TMS570_IOMM_PINMMR18_OFFSET   0x0158 /* Pin Multiplexing Control Register 18 */
#  define TMS570_IOMM_PINMMR19_OFFSET   0x015c /* Pin Multiplexing Control Register 19 */
#  define TMS570_IOMM_PINMMR20_OFFSET   0x0160 /* Pin Multiplexing Control Register 20 */
#  define TMS570_IOMM_PINMMR21_OFFSET   0x0164 /* Pin Multiplexing Control Register 21 */
#  define TMS570_IOMM_PINMMR22_OFFSET   0x0168 /* Pin Multiplexing Control Register 22 */
#  define TMS570_IOMM_PINMMR23_OFFSET   0x016c /* Pin Multiplexing Control Register 23 */
#  define TMS570_IOMM_PINMMR24_OFFSET   0x0170 /* Pin Multiplexing Control Register 24 */
#  define TMS570_IOMM_PINMMR25_OFFSET   0x0174 /* Pin Multiplexing Control Register 25 */
#  define TMS570_IOMM_PINMMR26_OFFSET   0x0178 /* Pin Multiplexing Control Register 26 */
#  define TMS570_IOMM_PINMMR27_OFFSET   0x017c /* Pin Multiplexing Control Register 27 */
#  define TMS570_IOMM_PINMMR28_OFFSET   0x0180 /* Pin Multiplexing Control Register 28 */
#  define TMS570_IOMM_PINMMR29_OFFSET   0x0184 /* Pin Multiplexing Control Register 29 */
#  define TMS570_IOMM_PINMMR30_OFFSET   0x0188 /* Pin Multiplexing Control Register 30 */

/* Register Addresses *******************************************************************************/

#define TMS570_IOMM_REVISION            (TMS570_IOMM_BASE+TMS570_IOMM_REVISION_OFFSET)
#define TMS570_IOMM_BOOT                (TMS570_IOMM_BASE+TMS570_IOMM_BOOT_OFFSET)
#define TMS570_IOMM_KICK0               (TMS570_IOMM_BASE+TMS570_IOMM_KICK0_OFFSET)
#define TMS570_IOMM_KICK1               (TMS570_IOMM_BASE+TMS570_IOMM_KICK1_OFFSET)
#define TMS570_IOMM_ERRRAWSTATUS        (TMS570_IOMM_BASE+TMS570_IOMM_ERRRAWSTATUS_OFFSET)
#define TMS570_IOMM_ERRSTATUS           (TMS570_IOMM_BASE+TMS570_IOMM_ERRSTATUS_OFFSET)
#define TMS570_IOMM_ERRENABLE           (TMS570_IOMM_BASE+TMS570_IOMM_ERRENABLE_OFFSET)
#define TMS570_IOMM_ERRENABLECLR        (TMS570_IOMM_BASE+TMS570_IOMM_ERRENABLECLR_OFFSET)
#define TMS570_IOMM_FAULTADDRESS        (TMS570_IOMM_BASE+TMS570_IOMM_FAULTADDRESS_OFFSET)
#define TMS570_IOMM_FAULTSTATUS         (TMS570_IOMM_BASE+TMS570_IOMM_FAULTSTATUS_OFFSET)
#define TMS570_IOMM_FAULTCLR            (TMS570_IOMM_BASE+TMS570_IOMM_FAULTCLR_OFFSET)

#define TMS570_IOMM_PINMMR(n)           (TMS570_IOMM_BASE+TMS570_IOMM_PINMMR_OFFSET(n))
#  define TMS570_IOMM_PINMMR0           (TMS570_IOMM_BASE+TMS570_IOMM_PINMMR0_OFFSET)
#  define TMS570_IOMM_PINMMR1           (TMS570_IOMM_BASE+TMS570_IOMM_PINMMR1_OFFSET)
#  define TMS570_IOMM_PINMMR2           (TMS570_IOMM_BASE+TMS570_IOMM_PINMMR2_OFFSET)
#  define TMS570_IOMM_PINMMR3           (TMS570_IOMM_BASE+TMS570_IOMM_PINMMR3_OFFSET)
#  define TMS570_IOMM_PINMMR4           (TMS570_IOMM_BASE+TMS570_IOMM_PINMMR4_OFFSET)
#  define TMS570_IOMM_PINMMR5           (TMS570_IOMM_BASE+TMS570_IOMM_PINMMR5_OFFSET)
#  define TMS570_IOMM_PINMMR6           (TMS570_IOMM_BASE+TMS570_IOMM_PINMMR6_OFFSET)
#  define TMS570_IOMM_PINMMR7           (TMS570_IOMM_BASE+TMS570_IOMM_PINMMR7_OFFSET)
#  define TMS570_IOMM_PINMMR8           (TMS570_IOMM_BASE+TMS570_IOMM_PINMMR8_OFFSET)
#  define TMS570_IOMM_PINMMR9           (TMS570_IOMM_BASE+TMS570_IOMM_PINMMR9_OFFSET)
#  define TMS570_IOMM_PINMMR10          (TMS570_IOMM_BASE+TMS570_IOMM_PINMMR10_OFFSET)
#  define TMS570_IOMM_PINMMR11          (TMS570_IOMM_BASE+TMS570_IOMM_PINMMR11_OFFSET)
#  define TMS570_IOMM_PINMMR12          (TMS570_IOMM_BASE+TMS570_IOMM_PINMMR12_OFFSET)
#  define TMS570_IOMM_PINMMR13          (TMS570_IOMM_BASE+TMS570_IOMM_PINMMR13_OFFSET)
#  define TMS570_IOMM_PINMMR14          (TMS570_IOMM_BASE+TMS570_IOMM_PINMMR14_OFFSET)
#  define TMS570_IOMM_PINMMR15          (TMS570_IOMM_BASE+TMS570_IOMM_PINMMR15_OFFSET)
#  define TMS570_IOMM_PINMMR16          (TMS570_IOMM_BASE+TMS570_IOMM_PINMMR16_OFFSET)
#  define TMS570_IOMM_PINMMR17          (TMS570_IOMM_BASE+TMS570_IOMM_PINMMR17_OFFSET)
#  define TMS570_IOMM_PINMMR18          (TMS570_IOMM_BASE+TMS570_IOMM_PINMMR18_OFFSET)
#  define TMS570_IOMM_PINMMR19          (TMS570_IOMM_BASE+TMS570_IOMM_PINMMR19_OFFSET)
#  define TMS570_IOMM_PINMMR20          (TMS570_IOMM_BASE+TMS570_IOMM_PINMMR20_OFFSET)
#  define TMS570_IOMM_PINMMR21          (TMS570_IOMM_BASE+TMS570_IOMM_PINMMR21_OFFSET)
#  define TMS570_IOMM_PINMMR22          (TMS570_IOMM_BASE+TMS570_IOMM_PINMMR22_OFFSET)
#  define TMS570_IOMM_PINMMR23          (TMS570_IOMM_BASE+TMS570_IOMM_PINMMR23_OFFSET)
#  define TMS570_IOMM_PINMMR24          (TMS570_IOMM_BASE+TMS570_IOMM_PINMMR24_OFFSET)
#  define TMS570_IOMM_PINMMR25          (TMS570_IOMM_BASE+TMS570_IOMM_PINMMR25_OFFSET)
#  define TMS570_IOMM_PINMMR26          (TMS570_IOMM_BASE+TMS570_IOMM_PINMMR26_OFFSET)
#  define TMS570_IOMM_PINMMR27          (TMS570_IOMM_BASE+TMS570_IOMM_PINMMR27_OFFSET)
#  define TMS570_IOMM_PINMMR28          (TMS570_IOMM_BASE+TMS570_IOMM_PINMMR28_OFFSET)
#  define TMS570_IOMM_PINMMR29          (TMS570_IOMM_BASE+TMS570_IOMM_PINMMR29_OFFSET)
#  define TMS570_IOMM_PINMMR30          (TMS570_IOMM_BASE+TMS570_IOMM_PINMMR30_OFFSET)

/* Register Bit-Field Definitions *******************************************************************/

/* Revision Register */
#define IOMM_REVISION_
/* Boot Mode Register */
#define IOMM_BOOT_

/* Kicker Register 0 */

#define IOMM_KICK0_UNLOCK               0x83e70b13 /* Unlock value */
#define IOMM_KICK0_LOCK                 0x00000000 /* Any other value locks */

/* Kicker Register 1 */

#define IOMM_KICK1_UNLOCK               0x95a4f1e0 /* Unlock value */
#define IOMM_KICK1_LOCK                 0x00000000 /* Any other value locks */

/* Error Raw Status / Set Register */
#define IOMM_ERRRAWSTATUS_
/* Error Enabled Status / Clear Register */
#define IOMM_ERRSTATUS_
/* Error Signaling Enable Register */
#define IOMM_ERRENABLE_
/* Error Signaling Enable Clear Register */
#define IOMM_ERRENABLECLR_
/* Fault Address Register */
#define IOMM_FAULTADDRESS_
/* Fault Status Register */
#define IOMM_FAULTSTATUS_
/* Fault Clear Register */
#define IOMM_FAULTCLR_

/* Pin Multiplexing Control Register n, n=0..30.  Each 8-bit field controls the functionality of
 * one pin/ball.  There are then a maximum of 31*4 = 124 pin/ball configurations supported.
 */

#define IOMM_PINMMR_REGNDX(n)           ((n) >> 2)
#define IOMM_PINMMR_PINSHIFT(n)         (((n) & 3) << 3)
#define IOMM_PINMMR_PINMASK(n)          (0xff << IOMM_PINMMR_PINSHIFT(n))
#  define IOMM_PINMMR_PINVALUE(n,v)     ((uint32_t)(v) << IOMM_PINMMR_PINSHIFT(n))

#endif /* __ARCH_ARM_SRC_TMS570_HARDWARE_TMS570_IOMM_H */
