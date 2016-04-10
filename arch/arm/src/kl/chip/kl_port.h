/************************************************************************************
 * arch/arm/src/kl/kl_port.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_KL_KL_PORT_H
#define __ARCH_ARM_SRC_KL_KL_PORT_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* General Definitions **************************************************************/

#define KL_PORTA                   (0)
#define KL_PORTB                   (1)
#define KL_PORTC                   (2)
#define KL_PORTD                   (3)
#define KL_PORTE                   (4)
#define KL_NPORTS                  (5)

/* Register Offsets *****************************************************************/

#define KL_PORT_PCR_OFFSET(n)      ((n) << 2) /* Pin Control Register n, n=0..31 */
#define KL_PORT_PCR0_OFFSET        0x0000 /* Pin Control Register 0 */
#define KL_PORT_PCR1_OFFSET        0x0004 /* Pin Control Register 1 */
#define KL_PORT_PCR2_OFFSET        0x0008 /* Pin Control Register 2 */
#define KL_PORT_PCR3_OFFSET        0x000C /* Pin Control Register 3 */
#define KL_PORT_PCR4_OFFSET        0x0010 /* Pin Control Register 4 */
#define KL_PORT_PCR5_OFFSET        0x0014 /* Pin Control Register 5 */
#define KL_PORT_PCR6_OFFSET        0x0018 /* Pin Control Register 6 */
#define KL_PORT_PCR7_OFFSET        0x001c /* Pin Control Register 7 */
#define KL_PORT_PCR8_OFFSET        0x0020 /* Pin Control Register 8 */
#define KL_PORT_PCR9_OFFSET        0x0024 /* Pin Control Register 9 */
#define KL_PORT_PCR10_OFFSET       0x0028 /* Pin Control Register 10 */
#define KL_PORT_PCR11_OFFSET       0x002c /* Pin Control Register 11 */
#define KL_PORT_PCR12_OFFSET       0x0030 /* Pin Control Register 12 */
#define KL_PORT_PCR13_OFFSET       0x0034 /* Pin Control Register 13 */
#define KL_PORT_PCR14_OFFSET       0x0038 /* Pin Control Register 14 */
#define KL_PORT_PCR15_OFFSET       0x003c /* Pin Control Register 15 */
#define KL_PORT_PCR16_OFFSET       0x0040 /* Pin Control Register 16 */
#define KL_PORT_PCR17_OFFSET       0x0044 /* Pin Control Register 17 */
#define KL_PORT_PCR18_OFFSET       0x0048 /* Pin Control Register 18 */
#define KL_PORT_PCR19_OFFSET       0x004c /* Pin Control Register 19 */
#define KL_PORT_PCR20_OFFSET       0x0050 /* Pin Control Register 20 */
#define KL_PORT_PCR21_OFFSET       0x0054 /* Pin Control Register 21 */
#define KL_PORT_PCR22_OFFSET       0x0058 /* Pin Control Register 22 */
#define KL_PORT_PCR23_OFFSET       0x005c /* Pin Control Register 23 */
#define KL_PORT_PCR24_OFFSET       0x0060 /* Pin Control Register 24 */
#define KL_PORT_PCR25_OFFSET       0x0064 /* Pin Control Register 25 */
#define KL_PORT_PCR26_OFFSET       0x0068 /* Pin Control Register 26 */
#define KL_PORT_PCR27_OFFSET       0x006c /* Pin Control Register 27 */
#define KL_PORT_PCR28_OFFSET       0x0070 /* Pin Control Register 28 */
#define KL_PORT_PCR29_OFFSET       0x0074 /* Pin Control Register 29 */
#define KL_PORT_PCR30_OFFSET       0x0078 /* Pin Control Register 30 */
#define KL_PORT_PCR31_OFFSET       0x007c /* Pin Control Register 31 */
#define KL_PORT_GPCLR_OFFSET       0x0080 /* Global Pin Control Low Register */
#define KL_PORT_GPCHR_OFFSET       0x0084 /* Global Pin Control High Register */
#define KL_PORT_ISFR_OFFSET        0x00a0 /* Interrupt Status Flag Register */
#define KL_PORT_DFER_OFFSET        0x00c0 /* Digital Filter Enable Register */
#define KL_PORT_DFCR_OFFSET        0x00c4 /* Digital Filter Clock Register */
#define KL_PORT_DFWR_OFFSET        0x00c8 /* Digital Filter Width Register */

/* Register Addresses ***************************************************************/

#define KL_PORT_PCR(p,n)           (KL_PORT_BASE(p)+KL_PORT_PCR_OFFSET(n)
#define KL_PORT_PCR0(p)            (KL_PORT_BASE(p)+KL_PORT_PCR0_OFFSET)
#define KL_PORT_PCR1(p)            (KL_PORT_BASE(p)+KL_PORT_PCR1_OFFSET)
#define KL_PORT_PCR2(p)            (KL_PORT_BASE(p)+KL_PORT_PCR2_OFFSET)
#define KL_PORT_PCR3(p)            (KL_PORT_BASE(p)+KL_PORT_PCR3_OFFSET)
#define KL_PORT_PCR4(p)            (KL_PORT_BASE(p)+KL_PORT_PCR4_OFFSET)
#define KL_PORT_PCR5(p)            (KL_PORT_BASE(p)+KL_PORT_PCR5_OFFSET)
#define KL_PORT_PCR6(p)            (KL_PORT_BASE(p)+KL_PORT_PCR6_OFFSET)
#define KL_PORT_PCR7(p)            (KL_PORT_BASE(p)+KL_PORT_PCR7_OFFSET)
#define KL_PORT_PCR8(p)            (KL_PORT_BASE(p)+KL_PORT_PCR8_OFFSET)
#define KL_PORT_PCR9(p)            (KL_PORT_BASE(p)+KL_PORT_PCR9_OFFSET)
#define KL_PORT_PCR10(p)           (KL_PORT_BASE(p)+KL_PORT_PCR10_OFFSET)
#define KL_PORT_PCR11(p)           (KL_PORT_BASE(p)+KL_PORT_PCR11_OFFSET)
#define KL_PORT_PCR12(p)           (KL_PORT_BASE(p)+KL_PORT_PCR12_OFFSET)
#define KL_PORT_PCR13(p)           (KL_PORT_BASE(p)+KL_PORT_PCR13_OFFSET)
#define KL_PORT_PCR14(p)           (KL_PORT_BASE(p)+KL_PORT_PCR14_OFFSET)
#define KL_PORT_PCR15(p)           (KL_PORT_BASE(p)+KL_PORT_PCR15_OFFSET)
#define KL_PORT_PCR16(p)           (KL_PORT_BASE(p)+KL_PORT_PCR16_OFFSET)
#define KL_PORT_PCR17(p)           (KL_PORT_BASE(p)+KL_PORT_PCR17_OFFSET)
#define KL_PORT_PCR18(p)           (KL_PORT_BASE(p)+KL_PORT_PCR18_OFFSET)
#define KL_PORT_PCR19(p)           (KL_PORT_BASE(p)+KL_PORT_PCR19_OFFSET)
#define KL_PORT_PCR20(p)           (KL_PORT_BASE(p)+KL_PORT_PCR20_OFFSET)
#define KL_PORT_PCR21(p)           (KL_PORT_BASE(p)+KL_PORT_PCR21_OFFSET)
#define KL_PORT_PCR22(p)           (KL_PORT_BASE(p)+KL_PORT_PCR22_OFFSET)
#define KL_PORT_PCR23(p)           (KL_PORT_BASE(p)+KL_PORT_PCR23_OFFSET)
#define KL_PORT_PCR24(p)           (KL_PORT_BASE(p)+KL_PORT_PCR24_OFFSET)
#define KL_PORT_PCR25(p)           (KL_PORT_BASE(p)+KL_PORT_PCR25_OFFSET)
#define KL_PORT_PCR26(p)           (KL_PORT_BASE(p)+KL_PORT_PCR26_OFFSET)
#define KL_PORT_PCR27(p)           (KL_PORT_BASE(p)+KL_PORT_PCR27_OFFSET)
#define KL_PORT_PCR28(p)           (KL_PORT_BASE(p)+KL_PORT_PCR28_OFFSET)
#define KL_PORT_PCR29(p)           (KL_PORT_BASE(p)+KL_PORT_PCR29_OFFSET)
#define KL_PORT_PCR30(p)           (KL_PORT_BASE(p)+KL_PORT_PCR30_OFFSET)
#define KL_PORT_PCR31(p)           (KL_PORT_BASE(p)+KL_PORT_PCR31_OFFSET)
#define KL_PORT_GPCLR(p)           (KL_PORT_BASE(p)+KL_PORT_GPCLR_OFFSET)
#define KL_PORT_GPCHR(p)           (KL_PORT_BASE(p)+KL_PORT_GPCHR_OFFSET)
#define KL_PORT_ISFR(p)            (KL_PORT_BASE(p)+KL_PORT_ISFR_OFFSET)
#define KL_PORT_DFER(p)            (KL_PORT_BASE(p)+KL_PORT_DFER_OFFSET)
#define KL_PORT_DFCR(p)            (KL_PORT_BASE(p)+KL_PORT_DFCR_OFFSET)
#define KL_PORT_DFWR(p)            (KL_PORT_BASE(p)+KL_PORT_DFWR_OFFSET)

#define KL_PORTA_PCR(n)            (KL_PORTA_BASE+KL_PORT_PCR_OFFSET(n)
#define KL_PORTA_PCR0              (KL_PORTA_BASE+KL_PORT_PCR0_OFFSET)
#define KL_PORTA_PCR1              (KL_PORTA_BASE+KL_PORT_PCR1_OFFSET)
#define KL_PORTA_PCR2              (KL_PORTA_BASE+KL_PORT_PCR2_OFFSET)
#define KL_PORTA_PCR3              (KL_PORTA_BASE+KL_PORT_PCR3_OFFSET)
#define KL_PORTA_PCR4              (KL_PORTA_BASE+KL_PORT_PCR4_OFFSET)
#define KL_PORTA_PCR5              (KL_PORTA_BASE+KL_PORT_PCR5_OFFSET)
#define KL_PORTA_PCR6              (KL_PORTA_BASE+KL_PORT_PCR6_OFFSET)
#define KL_PORTA_PCR7              (KL_PORTA_BASE+KL_PORT_PCR7_OFFSET)
#define KL_PORTA_PCR8              (KL_PORTA_BASE+KL_PORT_PCR8_OFFSET)
#define KL_PORTA_PCR9              (KL_PORTA_BASE+KL_PORT_PCR9_OFFSET)
#define KL_PORTA_PCR10             (KL_PORTA_BASE+KL_PORT_PCR10_OFFSET)
#define KL_PORTA_PCR11             (KL_PORTA_BASE+KL_PORT_PCR11_OFFSET)
#define KL_PORTA_PCR12             (KL_PORTA_BASE+KL_PORT_PCR12_OFFSET)
#define KL_PORTA_PCR13             (KL_PORTA_BASE+KL_PORT_PCR13_OFFSET)
#define KL_PORTA_PCR14             (KL_PORTA_BASE+KL_PORT_PCR14_OFFSET)
#define KL_PORTA_PCR15             (KL_PORTA_BASE+KL_PORT_PCR15_OFFSET)
#define KL_PORTA_PCR16             (KL_PORTA_BASE+KL_PORT_PCR16_OFFSET)
#define KL_PORTA_PCR17             (KL_PORTA_BASE+KL_PORT_PCR17_OFFSET)
#define KL_PORTA_PCR18             (KL_PORTA_BASE+KL_PORT_PCR18_OFFSET)
#define KL_PORTA_PCR19             (KL_PORTA_BASE+KL_PORT_PCR19_OFFSET)
#define KL_PORTA_PCR20             (KL_PORTA_BASE+KL_PORT_PCR20_OFFSET)
#define KL_PORTA_PCR21             (KL_PORTA_BASE+KL_PORT_PCR21_OFFSET)
#define KL_PORTA_PCR22             (KL_PORTA_BASE+KL_PORT_PCR22_OFFSET)
#define KL_PORTA_PCR23             (KL_PORTA_BASE+KL_PORT_PCR23_OFFSET)
#define KL_PORTA_PCR24             (KL_PORTA_BASE+KL_PORT_PCR24_OFFSET)
#define KL_PORTA_PCR25             (KL_PORTA_BASE+KL_PORT_PCR25_OFFSET)
#define KL_PORTA_PCR26             (KL_PORTA_BASE+KL_PORT_PCR26_OFFSET)
#define KL_PORTA_PCR27             (KL_PORTA_BASE+KL_PORT_PCR27_OFFSET)
#define KL_PORTA_PCR28             (KL_PORTA_BASE+KL_PORT_PCR28_OFFSET)
#define KL_PORTA_PCR29             (KL_PORTA_BASE+KL_PORT_PCR29_OFFSET)
#define KL_PORTA_PCR30             (KL_PORTA_BASE+KL_PORT_PCR30_OFFSET)
#define KL_PORTA_PCR31             (KL_PORTA_BASE+KL_PORT_PCR31_OFFSET)
#define KL_PORTA_GPCLR             (KL_PORTA_BASE+KL_PORT_GPCLR_OFFSET)
#define KL_PORTA_GPCHR             (KL_PORTA_BASE+KL_PORT_GPCHR_OFFSET)
#define KL_PORTA_ISFR              (KL_PORTA_BASE+KL_PORT_ISFR_OFFSET)
#define KL_PORTA_DFER              (KL_PORTA_BASE+KL_PORT_DFER_OFFSET)
#define KL_PORTA_DFCR              (KL_PORTA_BASE+KL_PORT_DFCR_OFFSET)
#define KL_PORTA_DFWR              (KL_PORTA_BASE+KL_PORT_DFWR_OFFSET)

#define KL_PORTB_PCR(n)            (KL_PORTB_BASE+KL_PORT_PCR_OFFSET(n)
#define KL_PORTB_PCR0              (KL_PORTB_BASE+KL_PORT_PCR0_OFFSET)
#define KL_PORTB_PCR1              (KL_PORTB_BASE+KL_PORT_PCR1_OFFSET)
#define KL_PORTB_PCR2              (KL_PORTB_BASE+KL_PORT_PCR2_OFFSET)
#define KL_PORTB_PCR3              (KL_PORTB_BASE+KL_PORT_PCR3_OFFSET)
#define KL_PORTB_PCR4              (KL_PORTB_BASE+KL_PORT_PCR4_OFFSET)
#define KL_PORTB_PCR5              (KL_PORTB_BASE+KL_PORT_PCR5_OFFSET)
#define KL_PORTB_PCR6              (KL_PORTB_BASE+KL_PORT_PCR6_OFFSET)
#define KL_PORTB_PCR7              (KL_PORTB_BASE+KL_PORT_PCR7_OFFSET)
#define KL_PORTB_PCR8              (KL_PORTB_BASE+KL_PORT_PCR8_OFFSET)
#define KL_PORTB_PCR9              (KL_PORTB_BASE+KL_PORT_PCR9_OFFSET)
#define KL_PORTB_PCR10             (KL_PORTB_BASE+KL_PORT_PCR10_OFFSET)
#define KL_PORTB_PCR11             (KL_PORTB_BASE+KL_PORT_PCR11_OFFSET)
#define KL_PORTB_PCR12             (KL_PORTB_BASE+KL_PORT_PCR12_OFFSET)
#define KL_PORTB_PCR13             (KL_PORTB_BASE+KL_PORT_PCR13_OFFSET)
#define KL_PORTB_PCR14             (KL_PORTB_BASE+KL_PORT_PCR14_OFFSET)
#define KL_PORTB_PCR15             (KL_PORTB_BASE+KL_PORT_PCR15_OFFSET)
#define KL_PORTB_PCR16             (KL_PORTB_BASE+KL_PORT_PCR16_OFFSET)
#define KL_PORTB_PCR17             (KL_PORTB_BASE+KL_PORT_PCR17_OFFSET)
#define KL_PORTB_PCR18             (KL_PORTB_BASE+KL_PORT_PCR18_OFFSET)
#define KL_PORTB_PCR19             (KL_PORTB_BASE+KL_PORT_PCR19_OFFSET)
#define KL_PORTB_PCR20             (KL_PORTB_BASE+KL_PORT_PCR20_OFFSET)
#define KL_PORTB_PCR21             (KL_PORTB_BASE+KL_PORT_PCR21_OFFSET)
#define KL_PORTB_PCR22             (KL_PORTB_BASE+KL_PORT_PCR22_OFFSET)
#define KL_PORTB_PCR23             (KL_PORTB_BASE+KL_PORT_PCR23_OFFSET)
#define KL_PORTB_PCR24             (KL_PORTB_BASE+KL_PORT_PCR24_OFFSET)
#define KL_PORTB_PCR25             (KL_PORTB_BASE+KL_PORT_PCR25_OFFSET)
#define KL_PORTB_PCR26             (KL_PORTB_BASE+KL_PORT_PCR26_OFFSET)
#define KL_PORTB_PCR27             (KL_PORTB_BASE+KL_PORT_PCR27_OFFSET)
#define KL_PORTB_PCR28             (KL_PORTB_BASE+KL_PORT_PCR28_OFFSET)
#define KL_PORTB_PCR29             (KL_PORTB_BASE+KL_PORT_PCR29_OFFSET)
#define KL_PORTB_PCR30             (KL_PORTB_BASE+KL_PORT_PCR30_OFFSET)
#define KL_PORTB_PCR31             (KL_PORTB_BASE+KL_PORT_PCR31_OFFSET)
#define KL_PORTB_GPCLR             (KL_PORTB_BASE+KL_PORT_GPCLR_OFFSET)
#define KL_PORTB_GPCHR             (KL_PORTB_BASE+KL_PORT_GPCHR_OFFSET)
#define KL_PORTB_ISFR              (KL_PORTB_BASE+KL_PORT_ISFR_OFFSET)
#define KL_PORTB_DFER              (KL_PORTB_BASE+KL_PORT_DFER_OFFSET)
#define KL_PORTB_DFCR              (KL_PORTB_BASE+KL_PORT_DFCR_OFFSET)
#define KL_PORTB_DFWR              (KL_PORTB_BASE+KL_PORT_DFWR_OFFSET)

#define KL_PORTC_PCR(n)            (KL_PORTC_BASE+KL_PORT_PCR_OFFSET(n)
#define KL_PORTC_PCR0              (KL_PORTC_BASE+KL_PORT_PCR0_OFFSET)
#define KL_PORTC_PCR1              (KL_PORTC_BASE+KL_PORT_PCR1_OFFSET)
#define KL_PORTC_PCR2              (KL_PORTC_BASE+KL_PORT_PCR2_OFFSET)
#define KL_PORTC_PCR3              (KL_PORTC_BASE+KL_PORT_PCR3_OFFSET)
#define KL_PORTC_PCR4              (KL_PORTC_BASE+KL_PORT_PCR4_OFFSET)
#define KL_PORTC_PCR5              (KL_PORTC_BASE+KL_PORT_PCR5_OFFSET)
#define KL_PORTC_PCR6              (KL_PORTC_BASE+KL_PORT_PCR6_OFFSET)
#define KL_PORTC_PCR7              (KL_PORTC_BASE+KL_PORT_PCR7_OFFSET)
#define KL_PORTC_PCR8              (KL_PORTC_BASE+KL_PORT_PCR8_OFFSET)
#define KL_PORTC_PCR9              (KL_PORTC_BASE+KL_PORT_PCR9_OFFSET)
#define KL_PORTC_PCR10             (KL_PORTC_BASE+KL_PORT_PCR10_OFFSET)
#define KL_PORTC_PCR11             (KL_PORTC_BASE+KL_PORT_PCR11_OFFSET)
#define KL_PORTC_PCR12             (KL_PORTC_BASE+KL_PORT_PCR12_OFFSET)
#define KL_PORTC_PCR13             (KL_PORTC_BASE+KL_PORT_PCR13_OFFSET)
#define KL_PORTC_PCR14             (KL_PORTC_BASE+KL_PORT_PCR14_OFFSET)
#define KL_PORTC_PCR15             (KL_PORTC_BASE+KL_PORT_PCR15_OFFSET)
#define KL_PORTC_PCR16             (KL_PORTC_BASE+KL_PORT_PCR16_OFFSET)
#define KL_PORTC_PCR17             (KL_PORTC_BASE+KL_PORT_PCR17_OFFSET)
#define KL_PORTC_PCR18             (KL_PORTC_BASE+KL_PORT_PCR18_OFFSET)
#define KL_PORTC_PCR19             (KL_PORTC_BASE+KL_PORT_PCR19_OFFSET)
#define KL_PORTC_PCR20             (KL_PORTC_BASE+KL_PORT_PCR20_OFFSET)
#define KL_PORTC_PCR21             (KL_PORTC_BASE+KL_PORT_PCR21_OFFSET)
#define KL_PORTC_PCR22             (KL_PORTC_BASE+KL_PORT_PCR22_OFFSET)
#define KL_PORTC_PCR23             (KL_PORTC_BASE+KL_PORT_PCR23_OFFSET)
#define KL_PORTC_PCR24             (KL_PORTC_BASE+KL_PORT_PCR24_OFFSET)
#define KL_PORTC_PCR25             (KL_PORTC_BASE+KL_PORT_PCR25_OFFSET)
#define KL_PORTC_PCR26             (KL_PORTC_BASE+KL_PORT_PCR26_OFFSET)
#define KL_PORTC_PCR27             (KL_PORTC_BASE+KL_PORT_PCR27_OFFSET)
#define KL_PORTC_PCR28             (KL_PORTC_BASE+KL_PORT_PCR28_OFFSET)
#define KL_PORTC_PCR29             (KL_PORTC_BASE+KL_PORT_PCR29_OFFSET)
#define KL_PORTC_PCR30             (KL_PORTC_BASE+KL_PORT_PCR30_OFFSET)
#define KL_PORTC_PCR31             (KL_PORTC_BASE+KL_PORT_PCR31_OFFSET)
#define KL_PORTC_GPCLR             (KL_PORTC_BASE+KL_PORT_GPCLR_OFFSET)
#define KL_PORTC_GPCHR             (KL_PORTC_BASE+KL_PORT_GPCHR_OFFSET)
#define KL_PORTC_ISFR              (KL_PORTC_BASE+KL_PORT_ISFR_OFFSET)
#define KL_PORTC_DFER              (KL_PORTC_BASE+KL_PORT_DFER_OFFSET)
#define KL_PORTC_DFCR              (KL_PORTC_BASE+KL_PORT_DFCR_OFFSET)
#define KL_PORTC_DFWR              (KL_PORTC_BASE+KL_PORT_DFWR_OFFSET)

#define KL_PORTD_PCR(n)            (KL_PORTD_BASE+KL_PORT_PCR_OFFSET(n)
#define KL_PORTD_PCR0              (KL_PORTD_BASE+KL_PORT_PCR0_OFFSET)
#define KL_PORTD_PCR1              (KL_PORTD_BASE+KL_PORT_PCR1_OFFSET)
#define KL_PORTD_PCR2              (KL_PORTD_BASE+KL_PORT_PCR2_OFFSET)
#define KL_PORTD_PCR3              (KL_PORTD_BASE+KL_PORT_PCR3_OFFSET)
#define KL_PORTD_PCR4              (KL_PORTD_BASE+KL_PORT_PCR4_OFFSET)
#define KL_PORTD_PCR5              (KL_PORTD_BASE+KL_PORT_PCR5_OFFSET)
#define KL_PORTD_PCR6              (KL_PORTD_BASE+KL_PORT_PCR6_OFFSET)
#define KL_PORTD_PCR7              (KL_PORTD_BASE+KL_PORT_PCR7_OFFSET)
#define KL_PORTD_PCR8              (KL_PORTD_BASE+KL_PORT_PCR8_OFFSET)
#define KL_PORTD_PCR9              (KL_PORTD_BASE+KL_PORT_PCR9_OFFSET)
#define KL_PORTD_PCR10             (KL_PORTD_BASE+KL_PORT_PCR10_OFFSET)
#define KL_PORTD_PCR11             (KL_PORTD_BASE+KL_PORT_PCR11_OFFSET)
#define KL_PORTD_PCR12             (KL_PORTD_BASE+KL_PORT_PCR12_OFFSET)
#define KL_PORTD_PCR13             (KL_PORTD_BASE+KL_PORT_PCR13_OFFSET)
#define KL_PORTD_PCR14             (KL_PORTD_BASE+KL_PORT_PCR14_OFFSET)
#define KL_PORTD_PCR15             (KL_PORTD_BASE+KL_PORT_PCR15_OFFSET)
#define KL_PORTD_PCR16             (KL_PORTD_BASE+KL_PORT_PCR16_OFFSET)
#define KL_PORTD_PCR17             (KL_PORTD_BASE+KL_PORT_PCR17_OFFSET)
#define KL_PORTD_PCR18             (KL_PORTD_BASE+KL_PORT_PCR18_OFFSET)
#define KL_PORTD_PCR19             (KL_PORTD_BASE+KL_PORT_PCR19_OFFSET)
#define KL_PORTD_PCR20             (KL_PORTD_BASE+KL_PORT_PCR20_OFFSET)
#define KL_PORTD_PCR21             (KL_PORTD_BASE+KL_PORT_PCR21_OFFSET)
#define KL_PORTD_PCR22             (KL_PORTD_BASE+KL_PORT_PCR22_OFFSET)
#define KL_PORTD_PCR23             (KL_PORTD_BASE+KL_PORT_PCR23_OFFSET)
#define KL_PORTD_PCR24             (KL_PORTD_BASE+KL_PORT_PCR24_OFFSET)
#define KL_PORTD_PCR25             (KL_PORTD_BASE+KL_PORT_PCR25_OFFSET)
#define KL_PORTD_PCR26             (KL_PORTD_BASE+KL_PORT_PCR26_OFFSET)
#define KL_PORTD_PCR27             (KL_PORTD_BASE+KL_PORT_PCR27_OFFSET)
#define KL_PORTD_PCR28             (KL_PORTD_BASE+KL_PORT_PCR28_OFFSET)
#define KL_PORTD_PCR29             (KL_PORTD_BASE+KL_PORT_PCR29_OFFSET)
#define KL_PORTD_PCR30             (KL_PORTD_BASE+KL_PORT_PCR30_OFFSET)
#define KL_PORTD_PCR31             (KL_PORTD_BASE+KL_PORT_PCR31_OFFSET)
#define KL_PORTD_GPCLR             (KL_PORTD_BASE+KL_PORT_GPCLR_OFFSET)
#define KL_PORTD_GPCHR             (KL_PORTD_BASE+KL_PORT_GPCHR_OFFSET)
#define KL_PORTD_ISFR              (KL_PORTD_BASE+KL_PORT_ISFR_OFFSET)
#define KL_PORTD_DFER              (KL_PORTD_BASE+KL_PORT_DFER_OFFSET)
#define KL_PORTD_DFCR              (KL_PORTD_BASE+KL_PORT_DFCR_OFFSET)
#define KL_PORTD_DFWR              (KL_PORTD_BASE+KL_PORT_DFWR_OFFSET)

#define KL_PORTE_PCR(n)            (KL_PORTE_BASE+KL_PORT_PCR_OFFSET(n)
#define KL_PORTE_PCR0              (KL_PORTE_BASE+KL_PORT_PCR0_OFFSET)
#define KL_PORTE_PCR1              (KL_PORTE_BASE+KL_PORT_PCR1_OFFSET)
#define KL_PORTE_PCR2              (KL_PORTE_BASE+KL_PORT_PCR2_OFFSET)
#define KL_PORTE_PCR3              (KL_PORTE_BASE+KL_PORT_PCR3_OFFSET)
#define KL_PORTE_PCR4              (KL_PORTE_BASE+KL_PORT_PCR4_OFFSET)
#define KL_PORTE_PCR5              (KL_PORTE_BASE+KL_PORT_PCR5_OFFSET)
#define KL_PORTE_PCR6              (KL_PORTE_BASE+KL_PORT_PCR6_OFFSET)
#define KL_PORTE_PCR7              (KL_PORTE_BASE+KL_PORT_PCR7_OFFSET)
#define KL_PORTE_PCR8              (KL_PORTE_BASE+KL_PORT_PCR8_OFFSET)
#define KL_PORTE_PCR9              (KL_PORTE_BASE+KL_PORT_PCR9_OFFSET)
#define KL_PORTE_PCR10             (KL_PORTE_BASE+KL_PORT_PCR10_OFFSET)
#define KL_PORTE_PCR11             (KL_PORTE_BASE+KL_PORT_PCR11_OFFSET)
#define KL_PORTE_PCR12             (KL_PORTE_BASE+KL_PORT_PCR12_OFFSET)
#define KL_PORTE_PCR13             (KL_PORTE_BASE+KL_PORT_PCR13_OFFSET)
#define KL_PORTE_PCR14             (KL_PORTE_BASE+KL_PORT_PCR14_OFFSET)
#define KL_PORTE_PCR15             (KL_PORTE_BASE+KL_PORT_PCR15_OFFSET)
#define KL_PORTE_PCR16             (KL_PORTE_BASE+KL_PORT_PCR16_OFFSET)
#define KL_PORTE_PCR17             (KL_PORTE_BASE+KL_PORT_PCR17_OFFSET)
#define KL_PORTE_PCR18             (KL_PORTE_BASE+KL_PORT_PCR18_OFFSET)
#define KL_PORTE_PCR19             (KL_PORTE_BASE+KL_PORT_PCR19_OFFSET)
#define KL_PORTE_PCR20             (KL_PORTE_BASE+KL_PORT_PCR20_OFFSET)
#define KL_PORTE_PCR21             (KL_PORTE_BASE+KL_PORT_PCR21_OFFSET)
#define KL_PORTE_PCR22             (KL_PORTE_BASE+KL_PORT_PCR22_OFFSET)
#define KL_PORTE_PCR23             (KL_PORTE_BASE+KL_PORT_PCR23_OFFSET)
#define KL_PORTE_PCR24             (KL_PORTE_BASE+KL_PORT_PCR24_OFFSET)
#define KL_PORTE_PCR25             (KL_PORTE_BASE+KL_PORT_PCR25_OFFSET)
#define KL_PORTE_PCR26             (KL_PORTE_BASE+KL_PORT_PCR26_OFFSET)
#define KL_PORTE_PCR27             (KL_PORTE_BASE+KL_PORT_PCR27_OFFSET)
#define KL_PORTE_PCR28             (KL_PORTE_BASE+KL_PORT_PCR28_OFFSET)
#define KL_PORTE_PCR29             (KL_PORTE_BASE+KL_PORT_PCR29_OFFSET)
#define KL_PORTE_PCR30             (KL_PORTE_BASE+KL_PORT_PCR30_OFFSET)
#define KL_PORTE_PCR31             (KL_PORTE_BASE+KL_PORT_PCR31_OFFSET)
#define KL_PORTE_GPCLR             (KL_PORTE_BASE+KL_PORT_GPCLR_OFFSET)
#define KL_PORTE_GPCHR             (KL_PORTE_BASE+KL_PORT_GPCHR_OFFSET)
#define KL_PORTE_ISFR              (KL_PORTE_BASE+KL_PORT_ISFR_OFFSET)
#define KL_PORTE_DFER              (KL_PORTE_BASE+KL_PORT_DFER_OFFSET)
#define KL_PORTE_DFCR              (KL_PORTE_BASE+KL_PORT_DFCR_OFFSET)
#define KL_PORTE_DFWR              (KL_PORTE_BASE+KL_PORT_DFWR_OFFSET)

/* Register Bit Definitions *********************************************************/
/* Pin Control Register n, n=0..31 */

#define PORT_PCR_PS                (1 << 0)  /* Bit 0: Pull Select */
#define PORT_PCR_PE                (1 << 1)  /* Bit 1: Pull Enable */
#define PORT_PCR_SRE               (1 << 2)  /* Bit 2: Slew Rate Enable */
                                             /* Bit 3: Reserved */
#define PORT_PCR_PFE               (1 << 4)  /* Bit 4: Passive Filter Enable */
#define PORT_PCR_ODE               (1 << 5)  /* Bit 5: Open Drain Enable */
#define PORT_PCR_DSE               (1 << 6)  /* Bit 6: Drive Strength Enable */
                                             /* Bit 7: Reserved */
#define PORT_PCR_MUX_SHIFT         (8)       /* Bits 8-10: Pin Mux Control */
#define PORT_PCR_MUX_MASK          (7 << PORT_PCR_MUX_SHIFT)
#  define PORT_PCR_MUX_ANALOG      (0 << PORT_PCR_MUX_SHIFT)  /* Pin Disabled (Analog) */
#  define PORT_PCR_MUX_GPIO        (1 << PORT_PCR_MUX_SHIFT)  /* Alternative 1 (GPIO) */
#  define PORT_PCR_MUX_ALT1        (1 << PORT_PCR_MUX_SHIFT)  /* Alternative 1 (GPIO) */
#  define PORT_PCR_MUX_ALT2        (2 << PORT_PCR_MUX_SHIFT)  /* Alternative 2 (chip specific) */
#  define PORT_PCR_MUX_ALT3        (3 << PORT_PCR_MUX_SHIFT)  /* Alternative 3 (chip specific) */
#  define PORT_PCR_MUX_ALT4        (4 << PORT_PCR_MUX_SHIFT)  /* Alternative 4 (chip specific) */
#  define PORT_PCR_MUX_ALT5        (5 << PORT_PCR_MUX_SHIFT)  /* Alternative 5 (chip specific) */
#  define PORT_PCR_MUX_ALT6        (6 << PORT_PCR_MUX_SHIFT)  /* Alternative 6 (chip specific) */
#  define PORT_PCR_MUX_ALT7        (7 << PORT_PCR_MUX_SHIFT)  /* Alternative 7 (chip specific / JTAG / NMI) */
                                             /* Bits 11-14: Reserved */
#define PORT_PCR_LK                (1 << 15) /* Bit 15: Lock Register */
#define PORT_PCR_IRQC_SHIFT        (16)      /* Bits 16-19: Interrupt Configuration */
#define PORT_PCR_IRQC_MASK         (15 << PORT_PCR_IRQC_SHIFT)
#  define PORT_PCR_IRQC_DISABLED   (0 << PORT_PCR_IRQC_SHIFT)  /* Interrupt/DMA Request disabled */
#  define PORT_PCR_IRQC_DMARISING  (1 << PORT_PCR_IRQC_SHIFT)  /* DMA Request on rising edge */
#  define PORT_PCR_IRQC_DMAFALLING (2 << PORT_PCR_IRQC_SHIFT)  /* DMA Request on falling edge */
#  define PORT_PCR_IRQC_DMABOTH    (3 << PORT_PCR_IRQC_SHIFT)  /* DMA Request on either edge */
#  define PORT_PCR_IRQC_ZERO       (8 << PORT_PCR_IRQC_SHIFT)  /* Interrupt when logic zero */
#  define PORT_PCR_IRQC_RISING     (9 << PORT_PCR_IRQC_SHIFT)  /* Interrupt on rising edge */
#  define PORT_PCR_IRQC_FALLING    (10 << PORT_PCR_IRQC_SHIFT) /* Interrupt on falling edge */
#  define PORT_PCR_IRQC_BOTH       (11 << PORT_PCR_IRQC_SHIFT) /* Interrupt on either edge */
#  define PORT_PCR_IRQC_ONE        (12 << PORT_PCR_IRQC_SHIFT) /* Interrupt when logic one */
                                             /* Bits 20-23: Reserved */
#define PORT_PCR_ISF               (1 << 24) /* Bit 24: Interrupt Status Flag */
                                             /* Bits 25-31: Reserved */

/* Global Pin Control Low Register */

#define PORT_GPCLR_GPWD_SHIFT      (0)       /* Bits 0-15: Global Pin Write Data */
#define PORT_GPCLR_GPWD_MASK       (0xffff << PORT_GPCLR_GPWD_SHIFT)
#  define PORT_GPCLR_GPWD(n)       ((1 << (n)) << PORT_GPCLR_GPWD_SHIFT)
#define PORT_GPCLR_GPWE_SHIFT      (16)      /* Bits 16-31: Global Pin Write Enable */
#define PORT_GPCLR_GPWE_MASK       (0xffff << PORT_GPCLR_GPWE_SHIFT)
#  define PORT_GPCLR_GPWE(n)       ((1 << (n)) << PORT_GPCLR_GPWE_SHIFT)

/* Global Pin Control High Register */

#define PORT_GPCHR_

#define PORT_GPCHR_GPWD_SHIFT      (0)       /* Bits 0-15: Global Pin Write Data */
#define PORT_GPCHR_GPWD_MASK       (0xffff << PORT_GPCHR_GPWD_SHIFT)
#  define PORT_GPCHR_GPWD(n)       ((1 << (n)) << PORT_GPCHR_GPWD_SHIFT)
#define PORT_GPCHR_GPWE_SHIFT      (16)      /* Bits 16-31: Global Pin Write Enable */
#define PORT_GPCHR_GPWE_MASK       (0xffff << PORT_GPCHR_GPWE_SHIFT)
#  define PORT_GPCHR_GPWE(n)       ((1 << (n)) << PORT_GPCHR_GPWE_SHIFT)

/* Interrupt Status Flag Register */

#define PORT_ISFR(n)               (1 << (n))

/* Digital Filter Enable Register */

#define PORT_DFER(n)               (1 << (n))

/* Digital Filter Clock Register */

#define PORT_DFCR_CS               (1 << 0)  /* Bit 0: Clock Source */

/* Digital Filter Width Register */

#define PORT_DFWR_FILT_SHIFT       (0)       /* Bits 0-4: Filter Length */
#define PORT_DFWR_FILT_MASK        (31 << PORT_DFWR_FILT_SHIFT)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_KL_KL_PORT_H */
