/****************************************************************************************************
 * arch/arm/src/tms570/chip/tms570_pcr.h
 * Peripheral Control Register (PCR) Definitions
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

#ifndef __ARCH_ARM_SRC_TMS570_CHIP_TMS570_PCR_H
#define __ARCH_ARM_SRC_TMS570_CHIP_TMS570_PCR_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include "chip/tms570_memorymap.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Register Offsets *********************************************************************************/

#define TMS570_PCR_PMPROTSET0_OFFSET    0x0000  /* Peripheral Memory Protection Set Register 0 */
#define TMS570_PCR_PMPROTSET1_OFFSET    0x0004  /* Peripheral Memory Protection Set Register 1 */
#define TMS570_PCR_PMPROTCLR0_OFFSET    0x0010  /* Peripheral Memory Protection Clear Register 0 */
#define TMS570_PCR_PMPROTCLR1_OFFSET    0x0014  /* Peripheral Memory Protection Clear Register 1 */
#define TMS570_PCR_PPROTSET0_OFFSET     0x0020  /* Peripheral Protection Set Register 0 */
#define TMS570_PCR_PPROTSET1_OFFSET     0x0024  /* Peripheral Protection Set Register 1 */
#define TMS570_PCR_PPROTSET2_OFFSET     0x0028  /* Peripheral Protection Set Register 2 */
#define TMS570_PCR_PPROTSET3_OFFSET     0x002c  /* Peripheral Protection Set Register 3 */
#define TMS570_PCR_PPROTCLR0_OFFSET     0x0040  /* Peripheral Protection Clear Register 0 */
#define TMS570_PCR_PPROTCLR1_OFFSET     0x0044  /* Peripheral Protection Clear Register 1 */
#define TMS570_PCR_PPROTCLR2_OFFSET     0x0048  /* Peripheral Protection Clear Register 2 */
#define TMS570_PCR_PPROTCLR3_OFFSET     0x004c  /* Peripheral Protection Clear Register 3 */
#define TMS570_PCR_PCSPWRDWNSET0_OFFSET 0x0060  /* Peripheral Memory Power-Down Set Register 0 */
#define TMS570_PCR_PCSPWRDWNSET1_OFFSET 0x0064  /* Peripheral Memory Power-Down Set Register 1 */
#define TMS570_PCR_PCSPWRDWNCLR0_OFFSET 0x0070  /* Peripheral Memory Power-Down Clear Register 0 */
#define TMS570_PCR_PCSPWRDWNCLR1_OFFSET 0x0074  /* Peripheral Memory Power-Down Clear Register 1 */
#define TMS570_PCR_PSPWRDWNSET0_OFFSET  0x0080  /* Peripheral Power-Down Set Register 0 */
#define TMS570_PCR_PSPWRDWNSET1_OFFSET  0x0084  /* Peripheral Power-Down Set Register 1 */
#define TMS570_PCR_PSPWRDWNSET2_OFFSET  0x0088  /* Peripheral Power-Down Set Register 2 */
#define TMS570_PCR_PSPWRDWNSET3_OFFSET  0x008c  /* Peripheral Power-Down Set Register 3 */
#define TMS570_PCR_PSPWRDWNCLR0_OFFSET  0x00a0  /* Peripheral Power-Down Clear Register 0 */
#define TMS570_PCR_PSPWRDWNCLR1_OFFSET  0x00a4  /* Peripheral Power-Down Clear Register 1 */
#define TMS570_PCR_PSPWRDWNCLR2_OFFSET  0x00a8  /* Peripheral Power-Down Clear Register 2 */
#define TMS570_PCR_PSPWRDWNCLR3_OFFSET  0x00ac  /* Peripheral Power-Down Clear Register 3 */

/* Register Addresses *******************************************************************************/

#define TMS570_PCR_PMPROTSET0           (TMS570_PCR_BASE+TMS570_PCR_PMPROTSET0_OFFSET)
#define TMS570_PCR_PMPROTSET1           (TMS570_PCR_BASE+TMS570_PCR_PMPROTSET1_OFFSET)
#define TMS570_PCR_PMPROTCLR0           (TMS570_PCR_BASE+TMS570_PCR_PMPROTCLR0_OFFSET)
#define TMS570_PCR_PMPROTCLR1           (TMS570_PCR_BASE+TMS570_PCR_PMPROTCLR1_OFFSET)
#define TMS570_PCR_PPROTSET0            (TMS570_PCR_BASE+TMS570_PCR_PPROTSET0_OFFSET)
#define TMS570_PCR_PPROTSET1            (TMS570_PCR_BASE+TMS570_PCR_PPROTSET1_OFFSET)
#define TMS570_PCR_PPROTSET2            (TMS570_PCR_BASE+TMS570_PCR_PPROTSET2_OFFSET)
#define TMS570_PCR_PPROTSET3            (TMS570_PCR_BASE+TMS570_PCR_PPROTSET3_OFFSET)
#define TMS570_PCR_PPROTCLR0            (TMS570_PCR_BASE+TMS570_PCR_PPROTCLR0_OFFSET)
#define TMS570_PCR_PPROTCLR1            (TMS570_PCR_BASE+TMS570_PCR_PPROTCLR1_OFFSET)
#define TMS570_PCR_PPROTCLR2            (TMS570_PCR_BASE+TMS570_PCR_PPROTCLR2_OFFSET)
#define TMS570_PCR_PPROTCLR3            (TMS570_PCR_BASE+TMS570_PCR_PPROTCLR3_OFFSET)
#define TMS570_PCR_PCSPWRDWNSET0        (TMS570_PCR_BASE+TMS570_PCR_PCSPWRDWNSET0_OFFSET)
#define TMS570_PCR_PCSPWRDWNSET1        (TMS570_PCR_BASE+TMS570_PCR_PCSPWRDWNSET1_OFFSET)
#define TMS570_PCR_PCSPWRDWNCLR0        (TMS570_PCR_BASE+TMS570_PCR_PCSPWRDWNCLR0_OFFSET)
#define TMS570_PCR_PCSPWRDWNCLR1        (TMS570_PCR_BASE+TMS570_PCR_PCSPWRDWNCLR1_OFFSET)
#define TMS570_PCR_PSPWRDWNSET0         (TMS570_PCR_BASE+TMS570_PCR_PSPWRDWNSET0_OFFSET)
#define TMS570_PCR_PSPWRDWNSET1         (TMS570_PCR_BASE+TMS570_PCR_PSPWRDWNSET1_OFFSET)
#define TMS570_PCR_PSPWRDWNSET2         (TMS570_PCR_BASE+TMS570_PCR_PSPWRDWNSET2_OFFSET)
#define TMS570_PCR_PSPWRDWNSET3         (TMS570_PCR_BASE+TMS570_PCR_PSPWRDWNSET3_OFFSET)
#define TMS570_PCR_PSPWRDWNCLR0         (TMS570_PCR_BASE+TMS570_PCR_PSPWRDWNCLR0_OFFSET)
#define TMS570_PCR_PSPWRDWNCLR1         (TMS570_PCR_BASE+TMS570_PCR_PSPWRDWNCLR1_OFFSET)
#define TMS570_PCR_PSPWRDWNCLR2         (TMS570_PCR_BASE+TMS570_PCR_PSPWRDWNCLR2_OFFSET)
#define TMS570_PCR_PSPWRDWNCLR3         (TMS570_PCR_BASE+TMS570_PCR_PSPWRDWNCLR3_OFFSET)

/* Register Bit-Field Definitions *******************************************************************/

/* Peripheral Memory Protection Set Register 0 */
#define PCR_PMPROTSET0_

/* Peripheral Memory Protection Set Register 1 */
#define PCR_PMPROTSET1_

/* Peripheral Memory Protection Clear Register 0 */
#define PCR_PMPROTCLR0_

/* Peripheral Memory Protection Clear Register 1 */
#define PCR_PMPROTCLR1_

/* Peripheral Protection Set Register 0 */
#define PCR_PPROTSET0_

/* Peripheral Protection Set Register 1 */
#define PCR_PPROTSET1_

/* Peripheral Protection Set Register 2 */
#define PCR_PPROTSET2_

/* Peripheral Protection Set Register 3 */
#define PCR_PPROTSET3_

/* Peripheral Protection Clear Register 0 */
#define PCR_PPROTCLR0_

/* Peripheral Protection Clear Register 1 */
#define PCR_PPROTCLR1_

/* Peripheral Protection Clear Register 2 */
#define PCR_PPROTCLR2_

/* Peripheral Protection Clear Register 3 */
#define PCR_PPROTCLR3_

/* Peripheral Memory Power-Down Set Register 0 */
#define PCR_PCSPWRDWNSET0_

/* Peripheral Memory Power-Down Set Register 1 */
#define PCR_PCSPWRDWNSET1_

/* Peripheral Memory Power-Down Clear Register 0 */
#define PCR_PCSPWRDWNCLR0_

/* Peripheral Memory Power-Down Clear Register 1 */
#define PCR_PCSPWRDWNCLR1_

/* Peripheral Power-Down Set Register 0 and Peripheral Power-Down Clear Register 0 */

#define PCR_PSPWERDWN0_PS0_SHIFT        (0)       /* Bits 0-3: Quadrants for PS0 */
#define PCR_PSPWERDWN0_PS0_MASK         (15 << PCR_PSPWERDWN0_PS0_SHIFT)
#  define PCR_PSPWERDWN0_PS0_Q1         (1 << PCR_PSPWERDWN0_PS0_SHIFT)
#  define PCR_PSPWERDWN0_PS0_Q2         (2 << PCR_PSPWERDWN0_PS0_SHIFT)
#  define PCR_PSPWERDWN0_PS0_Q3         (4 << PCR_PSPWERDWN0_PS0_SHIFT)
#  define PCR_PSPWERDWN0_PS0_Q4         (8 << PCR_PSPWERDWN0_PS0_SHIFT)
#  define PCR_PSPWERDWN0_PS0_QALL       (15 << PCR_PSPWERDWN0_PS0_SHIFT)
#define PCR_PSPWERDWN0_PS1_SHIFT        (4)       /* Bits 4-7: Quadrants for PS1 */
#define PCR_PSPWERDWN0_PS1_MASK         (15 << PCR_PSPWERDWN0_PS1_SHIFT)
#  define PCR_PSPWERDWN0_PS1_Q1         (1 << PCR_PSPWERDWN0_PS1_SHIFT)
#  define PCR_PSPWERDWN0_PS1_Q2         (2 << PCR_PSPWERDWN0_PS1_SHIFT)
#  define PCR_PSPWERDWN0_PS1_Q3         (4 << PCR_PSPWERDWN0_PS1_SHIFT)
#  define PCR_PSPWERDWN0_PS1_Q4         (8 << PCR_PSPWERDWN0_PS1_SHIFT)
#  define PCR_PSPWERDWN0_PS1_QALL       (15 << PCR_PSPWERDWN0_PS1_SHIFT)
#define PCR_PSPWERDWN0_PS2_SHIFT        (8)       /* Bits 8-11: Quadrants for PS2 */
#define PCR_PSPWERDWN0_PS2_MASK         (15 << PCR_PSPWERDWN0_PS2_SHIFT)
#  define PCR_PSPWERDWN0_PS2_Q1         (1 << PCR_PSPWERDWN0_PS2_SHIFT)
#  define PCR_PSPWERDWN0_PS2_Q2         (2 << PCR_PSPWERDWN0_PS2_SHIFT)
#  define PCR_PSPWERDWN0_PS2_Q3         (4 << PCR_PSPWERDWN0_PS2_SHIFT)
#  define PCR_PSPWERDWN0_PS2_Q4         (8 << PCR_PSPWERDWN0_PS2_SHIFT)
#  define PCR_PSPWERDWN0_PS2_QALL       (15 << PCR_PSPWERDWN0_PS2_SHIFT)
#define PCR_PSPWERDWN0_PS3_SHIFT        (12)      /* Bits 12-15: Quadrants for PS3 */
#define PCR_PSPWERDWN0_PS3_MASK         (15 << PCR_PSPWERDWN0_PS3_SHIFT)
#  define PCR_PSPWERDWN0_PS3_Q1         (1 << PCR_PSPWERDWN0_PS3_SHIFT)
#  define PCR_PSPWERDWN0_PS3_Q2         (2 << PCR_PSPWERDWN0_PS3_SHIFT)
#  define PCR_PSPWERDWN0_PS3_Q3         (4 << PCR_PSPWERDWN0_PS3_SHIFT)
#  define PCR_PSPWERDWN0_PS3_Q4         (8 << PCR_PSPWERDWN0_PS3_SHIFT)
#  define PCR_PSPWERDWN0_PS3_QALL       (15 << PCR_PSPWERDWN0_PS3_SHIFT)
#define PCR_PSPWERDWN0_PS4_SHIFT        (16)      /* Bits 16-19: Quadrants for PS4 */
#define PCR_PSPWERDWN0_PS4_MASK         (15 << PCR_PSPWERDWN0_PS4_SHIFT)
#  define PCR_PSPWERDWN0_PS4_Q1         (1 << PCR_PSPWERDWN0_PS4_SHIFT)
#  define PCR_PSPWERDWN0_PS4_Q2         (2 << PCR_PSPWERDWN0_PS4_SHIFT)
#  define PCR_PSPWERDWN0_PS4_Q3         (4 << PCR_PSPWERDWN0_PS4_SHIFT)
#  define PCR_PSPWERDWN0_PS4_Q4         (8 << PCR_PSPWERDWN0_PS4_SHIFT)
#  define PCR_PSPWERDWN0_PS4_QALL       (15 << PCR_PSPWERDWN0_PS4_SHIFT)
#define PCR_PSPWERDWN0_PS5_SHIFT        (20)      /* Bits 20-23: Quadrants for PS5 */
#define PCR_PSPWERDWN0_PS5_MASK         (15 << PCR_PSPWERDWN0_PS5_SHIFT)
#  define PCR_PSPWERDWN0_PS5_Q1         (1 << PCR_PSPWERDWN0_PS5_SHIFT)
#  define PCR_PSPWERDWN0_PS5_Q2         (2 << PCR_PSPWERDWN0_PS5_SHIFT)
#  define PCR_PSPWERDWN0_PS5_Q3         (4 << PCR_PSPWERDWN0_PS5_SHIFT)
#  define PCR_PSPWERDWN0_PS5_Q4         (8 << PCR_PSPWERDWN0_PS5_SHIFT)
#  define PCR_PSPWERDWN0_PS5_QALL       (15 << PCR_PSPWERDWN0_PS5_SHIFT)
#define PCR_PSPWERDWN0_PS6_SHIFT        (24)      /* Bits 24-27: Quadrants for PS6 */
#define PCR_PSPWERDWN0_PS6_MASK         (15 << PCR_PSPWERDWN0_PS6_SHIFT)
#  define PCR_PSPWERDWN0_PS6_Q1         (1 << PCR_PSPWERDWN0_PS6_SHIFT)
#  define PCR_PSPWERDWN0_PS6_Q2         (2 << PCR_PSPWERDWN0_PS6_SHIFT)
#  define PCR_PSPWERDWN0_PS6_Q3         (4 << PCR_PSPWERDWN0_PS6_SHIFT)
#  define PCR_PSPWERDWN0_PS6_Q4         (8 << PCR_PSPWERDWN0_PS6_SHIFT)
#  define PCR_PSPWERDWN0_PS6_QALL       (15 << PCR_PSPWERDWN0_PS6_SHIFT)
#define PCR_PSPWERDWN0_PS7_SHIFT        (28)      /* Bits 28-31: Quadrants for PS7 */
#define PCR_PSPWERDWN0_PS7_MASK         (15 << PCR_PSPWERDWN0_PS7_SHIFT)
#  define PCR_PSPWERDWN0_PS7_Q1         (1 << PCR_PSPWERDWN0_PS7_SHIFT)
#  define PCR_PSPWERDWN0_PS7_Q2         (2 << PCR_PSPWERDWN0_PS7_SHIFT)
#  define PCR_PSPWERDWN0_PS7_Q3         (4 << PCR_PSPWERDWN0_PS7_SHIFT)
#  define PCR_PSPWERDWN0_PS7_Q4         (8 << PCR_PSPWERDWN0_PS7_SHIFT)
#  define PCR_PSPWERDWN0_PS7_QALL       (15 << PCR_PSPWERDWN0_PS7_SHIFT)

/* Peripheral Power-Down Set Register 1 and Peripheral Power-Down Clear Register 1 */

#define PCR_PSPWERDWN1_PS8_SHIFT        (0)       /* Bits 0-3: Quadrants for PS8 */
#define PCR_PSPWERDWN1_PS8_MASK         (15 << PCR_PSPWERDWN1_PS8_SHIFT)
#  define PCR_PSPWERDWN1_PS8_Q1         (1 << PCR_PSPWERDWN1_PS8_SHIFT)
#  define PCR_PSPWERDWN1_PS8_Q2         (2 << PCR_PSPWERDWN1_PS8_SHIFT)
#  define PCR_PSPWERDWN1_PS8_Q3         (4 << PCR_PSPWERDWN1_PS8_SHIFT)
#  define PCR_PSPWERDWN1_PS8_Q4         (8 << PCR_PSPWERDWN1_PS8_SHIFT)
#  define PCR_PSPWERDWN1_PS8_QALL       (15 << PCR_PSPWERDWN1_PS8_SHIFT)
#define PCR_PSPWERDWN1_PS9_SHIFT        (4)       /* Bits 4-7: Quadrants for PS9 */
#define PCR_PSPWERDWN1_PS9_MASK         (15 << PCR_PSPWERDWN1_PS9_SHIFT)
#  define PCR_PSPWERDWN1_PS9_Q1         (1 << PCR_PSPWERDWN1_PS9_SHIFT)
#  define PCR_PSPWERDWN1_PS9_Q2         (2 << PCR_PSPWERDWN1_PS9_SHIFT)
#  define PCR_PSPWERDWN1_PS9_Q3         (4 << PCR_PSPWERDWN1_PS9_SHIFT)
#  define PCR_PSPWERDWN1_PS9_Q4         (8 << PCR_PSPWERDWN1_PS9_SHIFT)
#  define PCR_PSPWERDWN1_PS9_QALL       (15 << PCR_PSPWERDWN1_PS9_SHIFT)
#define PCR_PSPWERDWN1_PS10_SHIFT       (8)       /* Bits 8-11: Quadrants for PS10 */
#define PCR_PSPWERDWN1_PS10_MASK        (15 << PCR_PSPWERDWN1_PS10_SHIFT)
#  define PCR_PSPWERDWN1_PS10_Q1        (1 << PCR_PSPWERDWN1_PS10_SHIFT)
#  define PCR_PSPWERDWN1_PS10_Q2        (2 << PCR_PSPWERDWN1_PS10_SHIFT)
#  define PCR_PSPWERDWN1_PS10_Q3        (4 << PCR_PSPWERDWN1_PS10_SHIFT)
#  define PCR_PSPWERDWN1_PS10_Q4        (8 << PCR_PSPWERDWN1_PS10_SHIFT)
#  define PCR_PSPWERDWN1_PS10_QALL      (15 << PCR_PSPWERDWN1_PS10_SHIFT)
#define PCR_PSPWERDWN1_PS11_SHIFT       (12)      /* Bits 12-15: Quadrants for PS11 */
#define PCR_PSPWERDWN1_PS11_MASK        (15 << PCR_PSPWERDWN1_PS11_SHIFT)
#  define PCR_PSPWERDWN1_PS11_Q1        (1 << PCR_PSPWERDWN1_PS11_SHIFT)
#  define PCR_PSPWERDWN1_PS11_Q2        (2 << PCR_PSPWERDWN1_PS11_SHIFT)
#  define PCR_PSPWERDWN1_PS11_Q3        (4 << PCR_PSPWERDWN1_PS11_SHIFT)
#  define PCR_PSPWERDWN1_PS11_Q4        (8 << PCR_PSPWERDWN1_PS11_SHIFT)
#  define PCR_PSPWERDWN1_PS11_QALL      (15 << PCR_PSPWERDWN1_PS11_SHIFT)
#define PCR_PSPWERDWN1_PS12_SHIFT       (16)      /* Bits 16-19: Quadrants for PS12 */
#define PCR_PSPWERDWN1_PS12_MASK        (15 << PCR_PSPWERDWN1_PS12_SHIFT)
#  define PCR_PSPWERDWN1_PS12_Q1        (1 << PCR_PSPWERDWN1_PS12_SHIFT)
#  define PCR_PSPWERDWN1_PS12_Q2        (2 << PCR_PSPWERDWN1_PS12_SHIFT)
#  define PCR_PSPWERDWN1_PS12_Q3        (4 << PCR_PSPWERDWN1_PS12_SHIFT)
#  define PCR_PSPWERDWN1_PS12_Q4        (8 << PCR_PSPWERDWN1_PS12_SHIFT)
#  define PCR_PSPWERDWN1_PS12_QALL      (15 << PCR_PSPWERDWN1_PS12_SHIFT)
#define PCR_PSPWERDWN1_PS13_SHIFT       (20)      /* Bits 20-23: Quadrants for PS13 */
#define PCR_PSPWERDWN1_PS13_MASK        (15 << PCR_PSPWERDWN1_PS13_SHIFT)
#  define PCR_PSPWERDWN1_PS13_Q1        (1 << PCR_PSPWERDWN1_PS13_SHIFT)
#  define PCR_PSPWERDWN1_PS13_Q2        (2 << PCR_PSPWERDWN1_PS13_SHIFT)
#  define PCR_PSPWERDWN1_PS13_Q3        (4 << PCR_PSPWERDWN1_PS13_SHIFT)
#  define PCR_PSPWERDWN1_PS13_Q4        (8 << PCR_PSPWERDWN1_PS13_SHIFT)
#  define PCR_PSPWERDWN1_PS13_QALL      (15 << PCR_PSPWERDWN1_PS13_SHIFT)
#define PCR_PSPWERDWN1_PS14_SHIFT       (24)      /* Bits 24-27: Quadrants for PS14 */
#define PCR_PSPWERDWN1_PS14_MASK        (15 << PCR_PSPWERDWN1_PS14_SHIFT)
#  define PCR_PSPWERDWN1_PS14_Q1        (1 << PCR_PSPWERDWN1_PS14_SHIFT)
#  define PCR_PSPWERDWN1_PS14_Q2        (2 << PCR_PSPWERDWN1_PS14_SHIFT)
#  define PCR_PSPWERDWN1_PS14_Q3        (4 << PCR_PSPWERDWN1_PS14_SHIFT)
#  define PCR_PSPWERDWN1_PS14_Q4        (8 << PCR_PSPWERDWN1_PS14_SHIFT)
#  define PCR_PSPWERDWN1_PS14_QALL      (15 << PCR_PSPWERDWN1_PS14_SHIFT)
#define PCR_PSPWERDWN1_PS15_SHIFT       (28)      /* Bits 28-31: Quadrants for PS15 */
#define PCR_PSPWERDWN1_PS15_MASK        (15 << PCR_PSPWERDWN1_PS15_SHIFT)
#  define PCR_PSPWERDWN1_PS15_Q1        (1 << PCR_PSPWERDWN1_PS15_SHIFT)
#  define PCR_PSPWERDWN1_PS15_Q2        (2 << PCR_PSPWERDWN1_PS15_SHIFT)
#  define PCR_PSPWERDWN1_PS15_Q3        (4 << PCR_PSPWERDWN1_PS15_SHIFT)
#  define PCR_PSPWERDWN1_PS15_Q4        (8 << PCR_PSPWERDWN1_PS15_SHIFT)
#  define PCR_PSPWERDWN1_PS15_QALL      (15 << PCR_PSPWERDWN1_PS15_SHIFT)

/* Peripheral Power-Down Set Register 2 and Peripheral Power-Down Clear Register 2*/

#define PCR_PSPWERDWN2_PS16_SHIFT       (0)       /* Bits 0-3: Quadrants for PS16 */
#define PCR_PSPWERDWN2_PS16_MASK        (15 << PCR_PSPWERDWN2_PS16_SHIFT)
#  define PCR_PSPWERDWN2_PS16_Q1        (1 << PCR_PSPWERDWN2_PS16_SHIFT)
#  define PCR_PSPWERDWN2_PS16_Q2        (2 << PCR_PSPWERDWN2_PS16_SHIFT)
#  define PCR_PSPWERDWN2_PS16_Q3        (4 << PCR_PSPWERDWN2_PS16_SHIFT)
#  define PCR_PSPWERDWN2_PS16_Q4        (8 << PCR_PSPWERDWN2_PS16_SHIFT)
#  define PCR_PSPWERDWN2_PS16_QALL      (15 << PCR_PSPWERDWN2_PS16_SHIFT)
#define PCR_PSPWERDWN2_PS17_SHIFT       (4)       /* Bits 4-7: Quadrants for PS17 */
#define PCR_PSPWERDWN2_PS17_MASK        (15 << PCR_PSPWERDWN2_PS17_SHIFT)
#  define PCR_PSPWERDWN2_PS17_Q1        (1 << PCR_PSPWERDWN2_PS17_SHIFT)
#  define PCR_PSPWERDWN2_PS17_Q2        (2 << PCR_PSPWERDWN2_PS17_SHIFT)
#  define PCR_PSPWERDWN2_PS17_Q3        (4 << PCR_PSPWERDWN2_PS17_SHIFT)
#  define PCR_PSPWERDWN2_PS17_Q4        (8 << PCR_PSPWERDWN2_PS17_SHIFT)
#  define PCR_PSPWERDWN2_PS17_QALL      (15 << PCR_PSPWERDWN2_PS17_SHIFT)
#define PCR_PSPWERDWN2_PS18_SHIFT       (8)       /* Bits 8-11: Quadrants for PS18 */
#define PCR_PSPWERDWN2_PS18_MASK        (15 << PCR_PSPWERDWN2_PS18_SHIFT)
#  define PCR_PSPWERDWN2_PS18_Q1        (1 << PCR_PSPWERDWN2_PS18_SHIFT)
#  define PCR_PSPWERDWN2_PS18_Q2        (2 << PCR_PSPWERDWN2_PS18_SHIFT)
#  define PCR_PSPWERDWN2_PS18_Q3        (4 << PCR_PSPWERDWN2_PS18_SHIFT)
#  define PCR_PSPWERDWN2_PS18_Q4        (8 << PCR_PSPWERDWN2_PS18_SHIFT)
#  define PCR_PSPWERDWN2_PS18_QALL      (15 << PCR_PSPWERDWN2_PS18_SHIFT)
#define PCR_PSPWERDWN2_PS19_SHIFT       (12)      /* Bits 12-15: Quadrants for PS19 */
#define PCR_PSPWERDWN2_PS19_MASK        (15 << PCR_PSPWERDWN2_PS19_SHIFT)
#  define PCR_PSPWERDWN2_PS19_Q1        (1 << PCR_PSPWERDWN2_PS19_SHIFT)
#  define PCR_PSPWERDWN2_PS19_Q2        (2 << PCR_PSPWERDWN2_PS19_SHIFT)
#  define PCR_PSPWERDWN2_PS19_Q3        (4 << PCR_PSPWERDWN2_PS19_SHIFT)
#  define PCR_PSPWERDWN2_PS19_Q4        (8 << PCR_PSPWERDWN2_PS19_SHIFT)
#  define PCR_PSPWERDWN2_PS19_QALL      (15 << PCR_PSPWERDWN2_PS19_SHIFT)
#define PCR_PSPWERDWN2_PS20_SHIFT       (16)      /* Bits 16-19: Quadrants for PS20 */
#define PCR_PSPWERDWN2_PS20_MASK        (15 << PCR_PSPWERDWN2_PS20_SHIFT)
#  define PCR_PSPWERDWN2_PS20_Q1        (1 << PCR_PSPWERDWN2_PS20_SHIFT)
#  define PCR_PSPWERDWN2_PS20_Q2        (2 << PCR_PSPWERDWN2_PS20_SHIFT)
#  define PCR_PSPWERDWN2_PS20_Q3        (4 << PCR_PSPWERDWN2_PS20_SHIFT)
#  define PCR_PSPWERDWN2_PS20_Q4        (8 << PCR_PSPWERDWN2_PS20_SHIFT)
#  define PCR_PSPWERDWN2_PS20_QALL      (15 << PCR_PSPWERDWN2_PS20_SHIFT)
#define PCR_PSPWERDWN2_PS21_SHIFT       (20)      /* Bits 20-23: Quadrants for PS21 */
#define PCR_PSPWERDWN2_PS21_MASK        (15 << PCR_PSPWERDWN2_PS21_SHIFT)
#  define PCR_PSPWERDWN2_PS21_Q1        (1 << PCR_PSPWERDWN2_PS21_SHIFT)
#  define PCR_PSPWERDWN2_PS21_Q2        (2 << PCR_PSPWERDWN2_PS21_SHIFT)
#  define PCR_PSPWERDWN2_PS21_Q3        (4 << PCR_PSPWERDWN2_PS21_SHIFT)
#  define PCR_PSPWERDWN2_PS21_Q4        (8 << PCR_PSPWERDWN2_PS21_SHIFT)
#  define PCR_PSPWERDWN2_PS21_QALL      (15 << PCR_PSPWERDWN2_PS21_SHIFT)
#define PCR_PSPWERDWN2_PS22_SHIFT       (24)      /* Bits 24-27: Quadrants for PS22 */
#define PCR_PSPWERDWN2_PS22_MASK        (15 << PCR_PSPWERDWN2_PS22_SHIFT)
#  define PCR_PSPWERDWN2_PS22_Q1        (1 << PCR_PSPWERDWN2_PS22_SHIFT)
#  define PCR_PSPWERDWN2_PS22_Q2        (2 << PCR_PSPWERDWN2_PS22_SHIFT)
#  define PCR_PSPWERDWN2_PS22_Q3        (4 << PCR_PSPWERDWN2_PS22_SHIFT)
#  define PCR_PSPWERDWN2_PS22_Q4        (8 << PCR_PSPWERDWN2_PS22_SHIFT)
#  define PCR_PSPWERDWN2_PS22_QALL      (15 << PCR_PSPWERDWN2_PS22_SHIFT)
#define PCR_PSPWERDWN2_PS23_SHIFT       (28)      /* Bits 28-31: Quadrants for PS23 */
#define PCR_PSPWERDWN2_PS23_MASK        (15 << PCR_PSPWERDWN2_PS23_SHIFT)
#  define PCR_PSPWERDWN2_PS23_Q1        (1 << PCR_PSPWERDWN2_PS23_SHIFT)
#  define PCR_PSPWERDWN2_PS23_Q2        (2 << PCR_PSPWERDWN2_PS23_SHIFT)
#  define PCR_PSPWERDWN2_PS23_Q3        (4 << PCR_PSPWERDWN2_PS23_SHIFT)
#  define PCR_PSPWERDWN2_PS23_Q4        (8 << PCR_PSPWERDWN2_PS23_SHIFT)
#  define PCR_PSPWERDWN2_PS23_QALL      (15 << PCR_PSPWERDWN2_PS23_SHIFT)

/* Peripheral Power-Down Set Register 3 and Peripheral Power-Down Clear Register 3 */

#define PCR_PSPWERDWN3_PS24_SHIFT       (0)       /* Bits 0-3: Quadrants for PS24 */
#define PCR_PSPWERDWN3_PS24_MASK        (15 << PCR_PSPWERDWN3_PS24_SHIFT)
#  define PCR_PSPWERDWN3_PS24_Q1        (1 << PCR_PSPWERDWN3_PS24_SHIFT)
#  define PCR_PSPWERDWN3_PS24_Q2        (2 << PCR_PSPWERDWN3_PS24_SHIFT)
#  define PCR_PSPWERDWN3_PS24_Q3        (4 << PCR_PSPWERDWN3_PS24_SHIFT)
#  define PCR_PSPWERDWN3_PS24_Q4        (8 << PCR_PSPWERDWN3_PS24_SHIFT)
#  define PCR_PSPWERDWN3_PS24_QALL      (15 << PCR_PSPWERDWN3_PS24_SHIFT)
#define PCR_PSPWERDWN3_PS25_SHIFT       (4)       /* Bits 4-7: Quadrants for PS25 */
#define PCR_PSPWERDWN3_PS25_MASK        (15 << PCR_PSPWERDWN3_PS25_SHIFT)
#  define PCR_PSPWERDWN3_PS25_Q1        (1 << PCR_PSPWERDWN3_PS25_SHIFT)
#  define PCR_PSPWERDWN3_PS25_Q2        (2 << PCR_PSPWERDWN3_PS25_SHIFT)
#  define PCR_PSPWERDWN3_PS25_Q3        (4 << PCR_PSPWERDWN3_PS25_SHIFT)
#  define PCR_PSPWERDWN3_PS25_Q4        (8 << PCR_PSPWERDWN3_PS25_SHIFT)
#  define PCR_PSPWERDWN3_PS25_QALL      (15 << PCR_PSPWERDWN3_PS25_SHIFT)
#define PCR_PSPWERDWN3_PS26_SHIFT       (8)       /* Bits 8-11: Quadrants for PS26 */
#define PCR_PSPWERDWN3_PS26_MASK        (15 << PCR_PSPWERDWN3_PS26_SHIFT)
#  define PCR_PSPWERDWN3_PS26_Q1        (1 << PCR_PSPWERDWN3_PS26_SHIFT)
#  define PCR_PSPWERDWN3_PS26_Q2        (2 << PCR_PSPWERDWN3_PS26_SHIFT)
#  define PCR_PSPWERDWN3_PS26_Q3        (4 << PCR_PSPWERDWN3_PS26_SHIFT)
#  define PCR_PSPWERDWN3_PS26_Q4        (8 << PCR_PSPWERDWN3_PS26_SHIFT)
#  define PCR_PSPWERDWN3_PS26_QALL      (15 << PCR_PSPWERDWN3_PS26_SHIFT)
#define PCR_PSPWERDWN3_PS27_SHIFT       (12)      /* Bits 12-15: Quadrants for PS27 */
#define PCR_PSPWERDWN3_PS27_MASK        (15 << PCR_PSPWERDWN3_PS27_SHIFT)
#  define PCR_PSPWERDWN3_PS27_Q1        (1 << PCR_PSPWERDWN3_PS27_SHIFT)
#  define PCR_PSPWERDWN3_PS27_Q2        (2 << PCR_PSPWERDWN3_PS27_SHIFT)
#  define PCR_PSPWERDWN3_PS27_Q3        (4 << PCR_PSPWERDWN3_PS27_SHIFT)
#  define PCR_PSPWERDWN3_PS27_Q4        (8 << PCR_PSPWERDWN3_PS27_SHIFT)
#  define PCR_PSPWERDWN3_PS27_QALL      (15 << PCR_PSPWERDWN3_PS27_SHIFT)
#define PCR_PSPWERDWN3_PS28_SHIFT       (16)      /* Bits 16-19: Quadrants for PS28 */
#define PCR_PSPWERDWN3_PS28_MASK        (15 << PCR_PSPWERDWN3_PS28_SHIFT)
#  define PCR_PSPWERDWN3_PS28_Q1        (1 << PCR_PSPWERDWN3_PS28_SHIFT)
#  define PCR_PSPWERDWN3_PS28_Q2        (2 << PCR_PSPWERDWN3_PS28_SHIFT)
#  define PCR_PSPWERDWN3_PS28_Q3        (4 << PCR_PSPWERDWN3_PS28_SHIFT)
#  define PCR_PSPWERDWN3_PS28_Q4        (8 << PCR_PSPWERDWN3_PS28_SHIFT)
#  define PCR_PSPWERDWN3_PS28_QALL      (15 << PCR_PSPWERDWN3_PS28_SHIFT)
#define PCR_PSPWERDWN3_PS29_SHIFT       (20)      /* Bits 20-23: Quadrants for PS29 */
#define PCR_PSPWERDWN3_PS29_MASK        (15 << PCR_PSPWERDWN3_PS29_SHIFT)
#  define PCR_PSPWERDWN3_PS29_Q1        (1 << PCR_PSPWERDWN3_PS29_SHIFT)
#  define PCR_PSPWERDWN3_PS29_Q2        (2 << PCR_PSPWERDWN3_PS29_SHIFT)
#  define PCR_PSPWERDWN3_PS29_Q3        (4 << PCR_PSPWERDWN3_PS29_SHIFT)
#  define PCR_PSPWERDWN3_PS29_Q4        (8 << PCR_PSPWERDWN3_PS29_SHIFT)
#  define PCR_PSPWERDWN3_PS29_QALL      (15 << PCR_PSPWERDWN3_PS29_SHIFT)
#define PCR_PSPWERDWN3_PS30_SHIFT       (24)      /* Bits 24-27: Quadrants for PS30 */
#define PCR_PSPWERDWN3_PS30_MASK        (15 << PCR_PSPWERDWN3_PS30_SHIFT)
#  define PCR_PSPWERDWN3_PS30_Q1        (1 << PCR_PSPWERDWN3_PS30_SHIFT)
#  define PCR_PSPWERDWN3_PS30_Q2        (2 << PCR_PSPWERDWN3_PS30_SHIFT)
#  define PCR_PSPWERDWN3_PS30_Q3        (4 << PCR_PSPWERDWN3_PS30_SHIFT)
#  define PCR_PSPWERDWN3_PS30_Q4        (8 << PCR_PSPWERDWN3_PS30_SHIFT)
#  define PCR_PSPWERDWN3_PS30_QALL      (15 << PCR_PSPWERDWN3_PS30_SHIFT)
#define PCR_PSPWERDWN3_PS31_SHIFT       (28)      /* Bits 28-31: Quadrants for PS31 */
#define PCR_PSPWERDWN3_PS31_MASK        (15 << PCR_PSPWERDWN3_PS31_SHIFT)
#  define PCR_PSPWERDWN3_PS31_Q1        (1 << PCR_PSPWERDWN3_PS31_SHIFT)
#  define PCR_PSPWERDWN3_PS31_Q2        (2 << PCR_PSPWERDWN3_PS31_SHIFT)
#  define PCR_PSPWERDWN3_PS31_Q3        (4 << PCR_PSPWERDWN3_PS31_SHIFT)
#  define PCR_PSPWERDWN3_PS31_Q4        (8 << PCR_PSPWERDWN3_PS31_SHIFT)
#  define PCR_PSPWERDWN3_PS31_QALL      (15 << PCR_PSPWERDWN3_PS31_SHIFT)

#endif /* __ARCH_ARM_SRC_TMS570_CHIP_TMS570_PCR_H */
