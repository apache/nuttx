/****************************************************************************************************
 * arch/arm/src/stm32/chip/stm32f33xxx_opamp.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Mateusz Szafoni <raiden00@railab.me>
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

#ifndef __ARCH_ARM_SRC_STM32_CHIP_STM32_OPAMP_H
#define __ARCH_ARM_SRC_STM32_CHIP_STM32_OPAMP_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Register Offsets *********************************************************************************/

#define STM32_OPAMP2_CSR_OFFSET     0x003C    /* OPAMP2 Control register */

/* Register Addresses *******************************************************************************/

#define STM32_OPAMP2_CSR            (STM32_OPAMP_BASE+STM32_OPAMP2_CSR_OFFSET)

/* Register Bitfield Definitions ****************************************************/

/* OPAMP control and status register */

#define OPAMP_CSR_OPAMPEN           (1 << 0)    /* Bit 0: OPAMP enable */
#define OPAMP_CSR_FORCE_VP          (1 << 1)    /* Bit 1: Force a calibration reference voltage on non-nverting */
                                                /* input and disables external connections */
#define OPAMP_CSR_VPSEL_SHIFT       (3)         /* Bits 2-3: OPAMP non inverting input selection */
#define OPAMP_CSR_VPSEL_MASK        (3 << OPAMP_CSR_VPSEL_SHIFT)
                                                                    /* 00: Reserved  */
#  define OPAMP_CSR_VPSEL_PB14      (1 << OPAMP_CSR_VPSEL_SHIFT)    /* 01: PB14 */
#  define OPAMP_CSR_VPSEL_PB0       (2 << OPAMP_CSR_VPSEL_SHIFT)    /* 10: PB0 */
#  define OPAMP_CSR_VPSEL_PA7       (3 << OPAMP_CSR_VPSEL_SHIFT)    /* 11: PA7 */
                                                /* Bit 4: Reserved  */
#define OPAMP_CSR_VMSEL_SHIFT       (5)         /* Bits 5-6: OPAMP inverting input selection */
#define OPAMP_CSR_VMSEL_MASK        (3 << OPAMP_CSR_VMSEL_SHIFT)
#  define OPAMP_CSR_VMSEL_PC5       (0 << OPAMP_CSR_VMSEL_SHIFT)    /* 00: PC5 */
#  define OPAMP_CSR_VMSEL_PA5       (1 << OPAMP_CSR_VMSEL_SHIFT)    /* 01: PA5 */
#  define OPAMP_CSR_VMSEL_PGA       (2 << OPAMP_CSR_VMSEL_SHIFT)    /* 10: Resistor feedback output (PGA mode)*/
#  define OPAMP_CSR_VMSEL_FOLLOWER  (3 << OPAMP_CSR_VMSEL_SHIFT)    /* 11: Follower mode */
#define OPAMP_CSR_TCMEN             (1 << 7)    /* Bit 7: Timer controlled Mux mode enable */
#define OPAMP_CSR_VMSSEL            (1 << 8)    /* Bit 8: OPAMP inverting input secondary selection */
#define OPAMP_CSR_VPSSEL_SHIFT      (1 << 9)    /* Bits 9-10: OPAMP Non inverting input secondary selection */
#define OPAMP_CSR_VPSSEL_MASK       (3 << OPAMP_CSR_VPSSEL_SHIFT)
                                                                    /* 00: Reserved */
#  define OPAMP_CSR_VPSSEL_PB14     (1 << OPAMP_CSR_VPSSEL_SHIFT)   /* 01: PB14 */
#  define OPAMP_CSR_VPSSEL_PB0      (2 << OPAMP_CSR_VPSSEL_SHIFT)   /* 10: PB0 */
#  define OPAMP_CSR_VPSSEL_PA7      (3 << OPAMP_CSR_VPSSEL_SHIFT)   /* 11: PA7 */
#define OPAMP_CSR_CALON             (1 << 11)   /* Bit 11: Calibration mode enable */
#define OPAMP_CSR_CALSEL_SHIFT      (12)        /* Bits 12-13: Calibration selection */
#define OPAMP_CSR_CALSEL_MASK       (3 << OPAMP_CSR_CALSEL_SHIFT)
#  define OPAMP_CSR_CALSEL_3P3      (0 << OPAMP_CSR_CALSEL_SHIFT)   /* 00 V_REFOPAMP = 3.3% V_DDA */
#  define OPAMP_CSR_CALSEL_10       (1 << OPAMP_CSR_CALSEL_SHIFT)   /* 01 V_REFOPAMP = 10% V_DDA */
#  define OPAMP_CSR_CALSEL_50       (2 << OPAMP_CSR_CALSEL_SHIFT)   /* 10 V_REFOPAMP = 50% V_DDA */
#  define OPAMP_CSR_CALSEL_90       (3 << OPAMP_CSR_CALSEL_SHIFT)   /* 11 V_REFOPAMP = 90% V_DDA */
#define OPAMP_CSR_PGAGAIN_SHIFT     (14)        /* Bits 14-17: Gain in PGA mode  */
#define OPAMP_CSR_PGAGAIN_MASK      (15 << OPAMP_CSR_PGAGAIN_SHIFT)
#  define OPAMP_CSR_PGAGAIN_2       (0 << OPAMP_CSR_PGAGAIN_SHIFT)  /* 0X00: Non-inverting gain = 2 */
#  define OPAMP_CSR_PGAGAIN_4       (1 << OPAMP_CSR_PGAGAIN_SHIFT)  /* 0X01: Non-inverting gain = 4 */
#  define OPAMP_CSR_PGAGAIN_8       (2 << OPAMP_CSR_PGAGAIN_SHIFT)  /* 0X1-: Non-inverting gain = 8 */
#  define OPAMP_CSR_PGAGAIN_16      (3 << OPAMP_CSR_PGAGAIN_SHIFT)  /* 0X11: Non-inverting gain = 16 */
#  define OPAMP_CSR_PGAGAIN_2VM0    (8 << OPAMP_CSR_PGAGAIN_SHIFT)  /* 1000: Non-inverting gain = 2 - VM0*/
#  define OPAMP_CSR_PGAGAIN_4VM0    (9 << OPAMP_CSR_PGAGAIN_SHIFT)  /* 1001: Non-inverting gain = 4 - VM0*/
#  define OPAMP_CSR_PGAGAIN_8VM0    (10 << OPAMP_CSR_PGAGAIN_SHIFT) /* 1010: Non-inverting gain = 8 - VM0*/
#  define OPAMP_CSR_PGAGAIN_16VM0   (11 << OPAMP_CSR_PGAGAIN_SHIFT) /* 1011: Non-inverting gain = 16 - VM0*/
#  define OPAMP_CSR_PGAGAIN_2VM1    (12 << OPAMP_CSR_PGAGAIN_SHIFT) /* 1100: Non-inverting gain = 2 - VM1*/
#  define OPAMP_CSR_PGAGAIN_4VM1    (13 << OPAMP_CSR_PGAGAIN_SHIFT) /* 1101: Non-inverting gain = 4 - VM1*/
#  define OPAMP_CSR_PGAGAIN_8VM1    (14 << OPAMP_CSR_PGAGAIN_SHIFT) /* 1110: Non-inverting gain = 8 - VM1*/
#  define OPAMP_CSR_PGAGAIN_16VM1   (15 << OPAMP_CSR_PGAGAIN_SHIFT) /* 1111: Non-inverting gain = 16 - VM1*/
#define OPAMP_CSR_USERTRIM          (1 << 18)   /* Bit 18: User trimming enable */
#define OPAMP_CSR_TRIMOFFSETP_SHIFT (19)        /* Bits 19-23: Offset trimming value (PMOS)*/
#define OPAMP_CSR_TRIMOFFSETP_MASK  (31 << OPAMP_CSR_TRIMOFFSETP_SHIFT)
#define OPAMP_CSR_TRIMOFFSETN_SHIFT (24)        /* Bits 24-28: Offset trimming value (NMOS) */
#define OPAMP_CSR_TRIMOFFSETN_MASK  (31 << OPAMP_CSR_TRIMOFFSETN_SHIFT)
#define OPAMP_CSR_TSTREF            (1 << 29)   /* Bit 29:  Output the internal reference voltage */
#define OPAMP_CSR_OUTCAL            (1 << 30)   /* Bit 30: OPAMP output status flag */
#define OPAMP_CSR_LOCK              (1 << 31)   /* Bit 31: OPAMP 2 lock */

#endif /* __ARCH_ARM_SRC_STM32_CHIP_STM32_OPAMP_H */
