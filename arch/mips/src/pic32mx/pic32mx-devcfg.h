/****************************************************************************
 * arch/mips/src/pic32mx/pic32mx-devcfg.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

#ifndef __ARCH_MIPS_SRC_PIC32MX_PIC32MX_DEVCFG_H
#define __ARCH_MIPS_SRC_PIC32MX_PIC32MX_DEVCFG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "pic32mx-memorymap.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Register Offsets *********************************************************/

#define PIC32MX_DEVCFG3_OFFSET    0x0000 /* Device configuration word 3 */
#define PIC32MX_DEVCFG2_OFFSET    0x0004 /* Device configuration word 2 */
#define PIC32MX_DEVCFG1_OFFSET    0x0008 /* Device configuration word 1 */
#define PIC32MX_DEVCFG0_OFFSET    0x000c /* Device configuration word 0 */

/* Register Addresses *******************************************************/

#define PIC32MX_DEVCFG3           (PIC32MX_DEVCFG_K1BASE+PIC32MX_DEVCFG3_OFFSET)
#define PIC32MX_DEVCFG2           (PIC32MX_DEVCFG_K1BASE+PIC32MX_DEVCFG2_OFFSET)
#define PIC32MX_DEVCFG1           (PIC32MX_DEVCFG_K1BASE+PIC32MX_DEVCFG1_OFFSET)
#define PIC32MX_DEVCFG0           (PIC32MX_DEVCFG_K1BASE+PIC32MX_DEVCFG0_OFFSET)

/* Register Bit-Field Definitions *******************************************/

/* Device configuration word 3 */

#define DEVCFG3_USERID(n)         (1 << (n))
#define DEVCFG3_USERID0           (1 << 0)  /* Bit 0:  xx */
#define DEVCFG3_USERID1           (1 << 1)  /* Bit 1:  xx */
#define DEVCFG3_USERID2           (1 << 2)  /* Bit 2:  xx */
#define DEVCFG3_USERID3           (1 << 3)  /* Bit 3:  xx */
#define DEVCFG3_USERID4           (1 << 4)  /* Bit 4:  xx */
#define DEVCFG3_USERID5           (1 << 5)  /* Bit 5:  xx */
#define DEVCFG3_USERID6           (1 << 6)  /* Bit 6:  xx */
#define DEVCFG3_USERID7           (1 << 7)  /* Bit 7:  xx */
#define DEVCFG3_USERID8           (1 << 8)  /* Bit 8:  xx */
#define DEVCFG3_USERID9           (1 << 9)  /* Bit 9:  xx */
#define DEVCFG3_USERID10          (1 << 10) /* Bit 10: xx */
#define DEVCFG3_USERID11          (1 << 11) /* Bit 11: xx */
#define DEVCFG3_USERID12          (1 << 12) /* Bit 12: xx */
#define DEVCFG3_USERID13          (1 << 13) /* Bit 13: xx */
#define DEVCFG3_USERID14          (1 << 14) /* Bit 14: xx */
#define DEVCFG3_USERID15          (1 << 15) /* Bit 15: xx */

/* Device configuration word 2 */

#define DEVCFG2_FPLLIDIV_SHIFT    (0)       /* Bits 0-2: PLL input divider value */
#define DEVCFG2_FPLLIDIV_MASK     (7 << DEVCFG2_FPLLIDIV_SHIFT)
#  define DEVCFG2_FPLLIDIV_DIV1   (0 << DEVCFG2_FPLLIDIV_SHIFT)
#  define DEVCFG2_FPLLIDIV_DIV2   (1 << DEVCFG2_FPLLIDIV_SHIFT)
#  define DEVCFG2_FPLLIDIV_DIV3   (2 << DEVCFG2_FPLLIDIV_SHIFT)
#  define DEVCFG2_FPLLIDIV_DIV4   (3 << DEVCFG2_FPLLIDIV_SHIFT)
#  define DEVCFG2_FPLLIDIV_DIV5   (4 << DEVCFG2_FPLLIDIV_SHIFT)
#  define DEVCFG2_FPLLIDIV_DIV6   (5 << DEVCFG2_FPLLIDIV_SHIFT)
#  define DEVCFG2_FPLLIDIV_DIV10  (6 << DEVCFG2_FPLLIDIV_SHIFT)
#  define DEVCFG2_FPLLIDIV_DIV12  (7 << DEVCFG2_FPLLIDIV_SHIFT)
#define DEVCFG2_FPLLMULT_SHIFT    (4)       /* Bits 4-6: Initial PLL multiplier value */
#define DEVCFG2_FPLLMULT_MASK     (7 << DEVCFG2_FPLLMULT_SHIFT)
#  define DEVCFG2_FPLLMULT_MUL15  (0 << DEVCFG2_FPLLMULT_SHIFT)
#  define DEVCFG2_FPLLMULT_MUL16  (1 << DEVCFG2_FPLLMULT_SHIFT)
#  define DEVCFG2_FPLLMULT_MUL17  (2 << DEVCFG2_FPLLMULT_SHIFT)
#  define DEVCFG2_FPLLMULT_MUL18  (3 << DEVCFG2_FPLLMULT_SHIFT)
#  define DEVCFG2_FPLLMULT_MUL19  (4 << DEVCFG2_FPLLMULT_SHIFT)
#  define DEVCFG2_FPLLMULT_MUL20  (5 << DEVCFG2_FPLLMULT_SHIFT)
#  define DEVCFG2_FPLLMULT_MUL21  (6 << DEVCFG2_FPLLMULT_SHIFT)
#  define DEVCFG2_FPLLMULT_MUL24  (7 << DEVCFG2_FPLLMULT_SHIFT)
#define DEVCFG2_FUPLLIDIV_SHIFT   (8)       /* Bits 8-10: PLL input divider */
#define DEVCFG2_FUPLLIDIV_MASK    (7 << DEVCFG2_FUPLLIDIV_SHIFT)
#  define DEVCFG2_FUPLLIDIV_DIV1  (0 << DEVCFG2_FUPLLIDIV_SHIFT)
#  define DEVCFG2_FUPLLIDIV_DIV2  (1 << DEVCFG2_FUPLLIDIV_SHIFT)
#  define DEVCFG2_FUPLLIDIV_DIV3  (2 << DEVCFG2_FUPLLIDIV_SHIFT)
#  define DEVCFG2_FUPLLIDIV_DIV4  (3 << DEVCFG2_FUPLLIDIV_SHIFT)
#  define DEVCFG2_FUPLLIDIV_DIV5  (4 << DEVCFG2_FUPLLIDIV_SHIFT)
#  define DEVCFG2_FUPLLIDIV_DIV6  (5 << DEVCFG2_FUPLLIDIV_SHIFT)
#  define DEVCFG2_FUPLLIDIV_DIV10 (6 << DEVCFG2_FUPLLIDIV_SHIFT)
#  define DEVCFG2_FUPLLIDIV_DIV12 (7 << DEVCFG2_FUPLLIDIV_SHIFT)
#define DEVCFG2_FUPLLEN           (1 << 15) /* Bit 15: USB PLL enable */
#define DEVCFG2_FPLLODIV_SHIFT    (16)      /* Bits 16-18: Default postscaler for PLL bits */
#define DEVCFG2_FPLLODIV_MASK     (7 << DEVCFG2_FPLLODIV_SHIFT)
#  define DEVCFG2_FPLLODIV_DIV1   (0 << DEVCFG2_FPLLODIV_SHIFT)
#  define DEVCFG2_FPLLODIV_DIV2   (1 << DEVCFG2_FPLLODIV_SHIFT)
#  define DEVCFG2_FPLLODIV_DIV4   (2 << DEVCFG2_FPLLODIV_SHIFT)
#  define DEVCFG2_FPLLODIV_DIV8   (3 << DEVCFG2_FPLLODIV_SHIFT)
#  define DEVCFG2_FPLLODIV_DIV16  (4 << DEVCFG2_FPLLODIV_SHIFT)
#  define DEVCFG2_FPLLODIV_DIV32  (5 << DEVCFG2_FPLLODIV_SHIFT)
#  define DEVCFG2_FPLLODIV_DIV64  (6 << DEVCFG2_FPLLODIV_SHIFT)
#  define DEVCFG2_FPLLODIV_DIV256 (7 << DEVCFG2_FPLLODIV_SHIFT)

/* Device configuration word 1 */

#define DEVCFG1_FNOSC_SHIFT       (0)       /* Bits 0-2: Oscillator xelection */
#define DEVCFG1_FNOSC_MASK        (7 << DEVCFG1_FNOSC_SHIFT)
#  define DEVCFG1_FNOSC_ FRC      (0 << DEVCFG1_FNOSC_SHIFT) /* FRC oscillator */
#  define DEVCFG1_FNOSC_ FRCPLL   (1 << DEVCFG1_FNOSC_SHIFT) /* FRC w/PLL module */
#  define DEVCFG1_FNOSC_ POSC     (2 << DEVCFG1_FNOSC_SHIFT) /* Primary oscillator */
#  define DEVCFG1_FNOSC_ POSCPLL  (3 << DEVCFG1_FNOSC_SHIFT) /* Primary oscillator w/PLL */
#  define DEVCFG1_FNOSC_ SOSC     (4 << DEVCFG1_FNOSC_SHIFT) /* Secondary oscillator */
#  define DEVCFG1_FNOSC_ LPRC     (5 << DEVCFG1_FNOSC_SHIFT) /* Low power RC oscillator */
#  define DEVCFG1_FNOSC_ FRCDIV   (7 << DEVCFG1_FNOSC_SHIFT) /* FRC oscillator with FRCDIV */
#define DEVCFG1_FSOSCEN           (1 << 5)  /* Bit 5: Secondary oscillator (sosc) enable bit */
#define DEVCFG1_IESO              (1 << 7)  /* Bit 7: Internal external switch over */
#define DEVCFG1_POSCMOD_SHIFT     (8)       /* Bits 8-9: Primary oscillator (posc) configuration */
#define DEVCFG1_POSCMOD_MASK      (3 << DEVCFG1_POSCMOD_SHIFT)
#  define DEVCFG1_POSCMOD_ EC     (0 << DEVCFG1_POSCMOD_SHIFT) /* EC mode */
#  define DEVCFG1_POSCMOD_ XT     (1 << DEVCFG1_POSCMOD_SHIFT) /* XT mode */
#  define DEVCFG1_POSCMOD_ HS     (2 << DEVCFG1_POSCMOD_SHIFT) /* HS mode */
#  define DEVCFG1_POSCMOD_DIS     (3 << DEVCFG1_POSCMOD_SHIFT) /* Primary Oscillator disabled */
#define DEVCFG1_OSCIOFNC          (1 << 10) /* Bit 10: CLKO (clock-out) enable configuration */
#define DEVCFG1_FPBDIV_SHIFT      (12)      /* Bits 12-13: Peripheral bus clock divisor default value */
#define DEVCFG1_FPBDIV_MASK       (3 << DEVCFG1_FPBDIV_SHIFT)
#  define DEVCFG1_FPBDIV_DIV1     (0 << DEVCFG1_FPBDIV_SHIFT) /* PBCLK is SYSCLK/1 */
#  define DEVCFG1_FPBDIV_DIV2     (1 << DEVCFG1_FPBDIV_SHIFT) /* PBCLK is SYSCLK/2 */
#  define DEVCFG1_FPBDIV_DIV4     (2 << DEVCFG1_FPBDIV_SHIFT) /* PBCLK is SYSCLK/4 */
#  define DEVCFG1_FPBDIV_DIV8     (3 << DEVCFG1_FPBDIV_SHIFT) /* PBCLK is SYSCLK /8 */
#define DEVCFG1_FCKSM_SHIFT       (14)      /* Bits 14-15: Clock switching and monitor selection configuration */
#define DEVCFG1_FCKSM_MASK        (3 << DEVCFG1_FCKSM_SHIFT)
#  define DEVCFG1_FCKSM_BOTH      (0 << DEVCFG1_FCKSM_SHIFT) /* Clock switching and FSCM are enabled */
#  define DEVCFG1_FCKSM_CSONLY    (1 << DEVCFG1_FCKSM_SHIFT) /* Clock switching is enabled, FSCM is disabled */
#  define DEVCFG1_FCKSM_NONE      (2 << DEVCFG1_FCKSM_SHIFT) /* Clock switching and FSCM are disabled */
#define DEVCFG1_WDTPS_SHIFT       (16)      /* Bits 16-20: WDT postscaler select */
#define DEVCFG1_WDTPS_MASK        (31 << DEVCFG1_WDTPS_SHIFT)
#  define DEVCFG1_WDTPS_1         (0 << DEVCFG1_WDTPS_SHIFT) /* 1:1 */
#  define DEVCFG1_WDTPS_2         (1 << DEVCFG1_WDTPS_SHIFT) /* 1:2 */
#  define DEVCFG1_WDTPS_4         (2 << DEVCFG1_WDTPS_SHIFT) /* 1:4 */
#  define DEVCFG1_WDTPS_8         (3 << DEVCFG1_WDTPS_SHIFT) /* 1:8 */
#  define DEVCFG1_WDTPS_16        (4 << DEVCFG1_WDTPS_SHIFT) /* 1:16 */
#  define DEVCFG1_WDTPS_32        (5 << DEVCFG1_WDTPS_SHIFT) /* 1:32 */
#  define DEVCFG1_WDTPS_64        (6 << DEVCFG1_WDTPS_SHIFT) /* 1:64 */
#  define DEVCFG1_WDTPS_128       (7 << DEVCFG1_WDTPS_SHIFT) /* 1:128 */
#  define DEVCFG1_WDTPS_256       (8 << DEVCFG1_WDTPS_SHIFT) /* 1:256 */
#  define DEVCFG1_WDTPS_512       (9 << DEVCFG1_WDTPS_SHIFT) /* 1:512 */
#  define DEVCFG1_WDTPS_1024      (10 << DEVCFG1_WDTPS_SHIFT) /* 1:1024 */
#  define DEVCFG1_WDTPS_2048      (11 << DEVCFG1_WDTPS_SHIFT) /* 1:2048 */
#  define DEVCFG1_WDTPS_4096      (12 << DEVCFG1_WDTPS_SHIFT) /* 1:4096 */
#  define DEVCFG1_WDTPS_8192      (13 << DEVCFG1_WDTPS_SHIFT) /* 1:8192 */
#  define DEVCFG1_WDTPS_16384     (14 << DEVCFG1_WDTPS_SHIFT) /* 1:16384 */
#  define DEVCFG1_WDTPS_32768     (15 << DEVCFG1_WDTPS_SHIFT) /* 1:32768 */
#  define DEVCFG1_WDTPS_65536     (16 << DEVCFG1_WDTPS_SHIFT) /* 1:65536 */
#  define DEVCFG1_WDTPS_131072    (17 << DEVCFG1_WDTPS_SHIFT) /* 1:131072 */
#  define DEVCFG1_WDTPS_262144    (18 << DEVCFG1_WDTPS_SHIFT) /* 1:262144 */
#  define DEVCFG1_WDTPS_524288    (19 << DEVCFG1_WDTPS_SHIFT) /* 1:524288 */
#  define DEVCFG1_WDTPS_1048576   (20 << DEVCFG1_WDTPS_SHIFT) /* 1:1048576 */
#define DEVCFG1_FWDTEN            (1 << 23) /* Bit 23: WDT enable */

/* Device configuration word 0 */


#define DEVCFG0_DEBUG_SHIFT       (0)      /* Bits 0-1: xx */
#define DEVCFG0_DEBUG_MASK        (3 << DEVCFG0_DEBUG_SHIFT)
#define DEVCFG0_ICESEL            (1 << 3)  /* Bit 3:  xx */
#define DEVCFG0_PWP12             (1 << 12) /* Bit 12:  xx */
#define DEVCFG0_PWP13             (1 << 13) /* Bit 13:  xx */
#define DEVCFG0_PWP14             (1 << 14) /* Bit 14:  xx */
#define DEVCFG0_PWP15             (1 << 15) /* Bit 15:  xx */
#define DEVCFG0_PWP16             (1 << 16) /* Bit 16:  xx */
#define DEVCFG0_PWP17             (1 << 17) /* Bit 17:  xx */
#define DEVCFG0_PWP18             (1 << 18) /* Bit 18:  xx */
#define DEVCFG0_PWP19             (1 << 19) /* Bit 19:  xx */
#define DEVCFG0_BWP               (1 << 24) /* Bit 24:  xx */
#define DEVCFG0_CP                (1 << 28) /* Bit 28:  xx */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_PIC32MX_PIC32MX_DEVCFG_H */
