/****************************************************************************
 * arch/mips/src/pic32mx/pic32mx-int.h
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

#ifndef __ARCH_MIPS_SRC_PIC32MX_PIC32MX_INT_H
#define __ARCH_MIPS_SRC_PIC32MX_PIC32MX_INT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "pic32mx-memorymap.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Register Offsets *********************************************************/

#define PIC32MX_INT_INTCON_OFFSET     0x0000 /* Interrupt control register */
#define PIC32MX_INT_INTCONCLR_OFFSET  0x0004 /* Interrupt control clear register */
#define PIC32MX_INT_INTCONSET_OFFSET  0x0008 /* Interrupt control set register */
#define PIC32MX_INT_INTCONINV_OFFSET  0x000c /* Interrupt control invert register */
#define PIC32MX_INT_INTSTAT_OFFSET    0x0010 /* Interrupt status register */
#define PIC32MX_INT_INTSTATCLR_OFFSET 0x0014 /* Interrupt status clear register */
#define PIC32MX_INT_INTSTATSET_OFFSET 0x0018 /* Interrupt status set register */
#define PIC32MX_INT_INTSTATINV_OFFSET 0x001c /* Interrupt status invert register */
#define PIC32MX_INT_TPTMR_OFFSET      0x0020 /* Temporal proximity timer register */
#define PIC32MX_INT_TPTMRCLR_OFFSET   0x0024 /* Temporal proximity timer clear register */
#define PIC32MX_INT_TPTMRSET_OFFSET   0x0028 /* Temporal proximity timer set register */
#define PIC32MX_INT_TPTMRINV_OFFSET   0x002c /* Temporal proximity timer invert register */
#define PIC32MX_INT_IFS_OFFSET(n)     (0x0030 + ((n) << 4))
#define PIC32MX_INT_IFSCLR_OFFSET(n)  (0x0034 + ((n) << 4))
#define PIC32MX_INT_IFSSET_OFFSET(n)  (0x0038 + ((n) << 4))
#define PIC32MX_INT_IFSINV_OFFSET(n)  (0x003c + ((n) << 4))
#define PIC32MX_INT_IFS0_OFFSET       0x0030 /* Interrupt flag status register 0 */
#define PIC32MX_INT_IFS0CLR_OFFSET    0x0034 /* Interrupt flag status clear register 0 */
#define PIC32MX_INT_IFS0SET_OFFSET    0x0038 /* Interrupt flag status set register 0 */
#define PIC32MX_INT_IFS0INV_OFFSET    0x003c /* Interrupt flag status invert register 0 */
#define PIC32MX_INT_IFS1_OFFSET       0x0040 /* Interrupt flag status register 1 */
#define PIC32MX_INT_IFS1CLR_OFFSET    0x0044 /* Interrupt flag status clear register 1 */
#define PIC32MX_INT_IFS1SET_OFFSET    0x0048 /* Interrupt flag status set register 1 */
#define PIC32MX_INT_IFS1INV_OFFSET    0x004c /* Interrupt flag status invert register 1 */
#define PIC32MX_INT_IEC_OFFSET(n)     (0x0060 + ((n) << 4))
#define PIC32MX_INT_IECCLR_OFFSET(n)  (0x0064 + ((n) << 4))
#define PIC32MX_INT_IECSET_OFFSET(n)  (0x0068 + ((n) << 4))
#define PIC32MX_INT_IECINV_OFFSET(n)  (0x006c + ((n) << 4))
#define PIC32MX_INT_IEC0_OFFSET       0x0060 /* Interrupt enable control register 0 */
#define PIC32MX_INT_IEC0CLR_OFFSET    0x0064 /* Interrupt enable control clear register 0 */
#define PIC32MX_INT_IEC0SET_OFFSET    0x0068 /* Interrupt enable control set register 0 */
#define PIC32MX_INT_IEC0INV_OFFSET    0x006c /* Interrupt enable control invert register 0 */
#define PIC32MX_INT_IEC1_OFFSET       0x0070 /* Interrupt enable control register 1 */
#define PIC32MX_INT_IEC1CLR_OFFSET    0x0074 /* Interrupt enable control clear register 1 */
#define PIC32MX_INT_IEC1SET_OFFSET    0x0078 /* Interrupt enable control set register 1 */
#define PIC32MX_INT_IEC1INV_OFFSET    0x007c /* Interrupt enable control invert register 1 */
#define PIC32MX_INT_IPC_OFFSET(n)     (0x0090 + ((n) << 4))
#define PIC32MX_INT_IPCCLR_OFFSET(n)  (0x0094 + ((n) << 4))
#define PIC32MX_INT_IPCSET_OFFSET(n)  (0x0098 + ((n) << 4))
#define PIC32MX_INT_IPCINV_OFFSET(n)  (0x009c + ((n) << 4))
#define PIC32MX_INT_IPC0_OFFSET       0x0090 /* Interrupt priority control register 0 */
#define PIC32MX_INT_IPC0CLR_OFFSET    0x0094 /* Interrupt priority control clear register 0 */
#define PIC32MX_INT_IPC0SET_OFFSET    0x0098 /* Interrupt priority control set register 0 */
#define PIC32MX_INT_IPC0INV_OFFSET    0x009c /* Interrupt priority control invert register 0 */
#define PIC32MX_INT_IPC1_OFFSET       0x00a0 /* Interrupt priority control register 1 */
#define PIC32MX_INT_IPC1CLR_OFFSET    0x00a4 /* Interrupt priority control clear register 1 */
#define PIC32MX_INT_IPC1SET_OFFSET    0x00a8 /* Interrupt priority control set register 1 */
#define PIC32MX_INT_IPC1INV_OFFSET    0x00ac /* Interrupt priority control invert register 1 */
#define PIC32MX_INT_IPC2_OFFSET       0x00b0 /* Interrupt priority control register 2 */
#define PIC32MX_INT_IPC2CLR_OFFSET    0x00b4 /* Interrupt priority control clear register 2 */
#define PIC32MX_INT_IPC2SET_OFFSET    0x00b8 /* Interrupt priority control set register 2 */
#define PIC32MX_INT_IPC2INV_OFFSET    0x00bc /* Interrupt priority control invert register 2 */
#define PIC32MX_INT_IPC3_OFFSET       0x00c0 /* Interrupt priority control register 3 */
#define PIC32MX_INT_IPC3CLR_OFFSET    0x00c4 /* Interrupt priority control clear register 3 */
#define PIC32MX_INT_IPC3SET_OFFSET    0x00c8 /* Interrupt priority control set register 3 */
#define PIC32MX_INT_IPC3INV_OFFSET    0x00cc /* Interrupt priority control invert register 3 */
#define PIC32MX_INT_IPC4_OFFSET       0x00d0 /* Interrupt priority control register 4 */
#define PIC32MX_INT_IPC4CLR_OFFSET    0x00d4 /* Interrupt priority control clear register 4 */
#define PIC32MX_INT_IPC4SET_OFFSET    0x00d8 /* Interrupt priority control set register 4 */
#define PIC32MX_INT_IPC4INV_OFFSET    0x00dc /* Interrupt priority control invert register 4 */
#define PIC32MX_INT_IPC5_OFFSET       0x00e0 /* Interrupt priority control register 5 */
#define PIC32MX_INT_IPC5CLR_OFFSET    0x00e4 /* Interrupt priority control clear register 5 */
#define PIC32MX_INT_IPC5SET_OFFSET    0x00e8 /* Interrupt priority control set register 5 */
#define PIC32MX_INT_IPC5INV_OFFSET    0x00ec /* Interrupt priority control invert register 5 */
#define PIC32MX_INT_IPC6_OFFSET       0x00f0 /* Interrupt priority control register 6 */
#define PIC32MX_INT_IPC6CLR_OFFSET    0x00f4 /* Interrupt priority control clear register 6 */
#define PIC32MX_INT_IPC6SET_OFFSET    0x00f8 /* Interrupt priority control set register 6 */
#define PIC32MX_INT_IPC6INV_OFFSET    0x00fc /* Interrupt priority control invert register 6 */
#define PIC32MX_INT_IPC7_OFFSET       0x0100 /* Interrupt priority control register 7 */
#define PIC32MX_INT_IPC7CLR_OFFSET    0x0104 /* Interrupt priority control clear register 7 */
#define PIC32MX_INT_IPC7SET_OFFSET    0x0108 /* Interrupt priority control set register 7 */
#define PIC32MX_INT_IPC7INV_OFFSET    0x010c /* Interrupt priority control invert register 7 */
#define PIC32MX_INT_IPC8_OFFSET       0x0110 /* Interrupt priority control register 8 */
#define PIC32MX_INT_IPC8CLR_OFFSET    0x0114 /* Interrupt priority control clear register 8 */
#define PIC32MX_INT_IPC8SET_OFFSET    0x0118 /* Interrupt priority control set register 8 */
#define PIC32MX_INT_IPC8INV_OFFSET    0x011c /* Interrupt priority control invert register 8 */
#define PIC32MX_INT_IPC9_OFFSET       0x0120 /* Interrupt priority control register 9 */
#define PIC32MX_INT_IPC9CLR_OFFSET    0x0124 /* Interrupt priority control clear register 9 */
#define PIC32MX_INT_IPC9SET_OFFSET    0x0128 /* Interrupt priority control set register 9 */
#define PIC32MX_INT_IPC9INV_OFFSET    0x012c /* Interrupt priority control invert register 9 */
#define PIC32MX_INT_IPC11_OFFSET      0x0140 /* Interrupt priority control register 11 */
#define PIC32MX_INT_IPC11CLR_OFFSET   0x0144 /* Interrupt priority control clear register 11 */
#define PIC32MX_INT_IPC11SET_OFFSET   0x0148 /* Interrupt priority control set register 11 */
#define PIC32MX_INT_IPC11INV_OFFSET   0x014c /* Interrupt priority control invert register 11 */

/* Register Addresses *******************************************************/

#define PIC32MX_INT_INTCON           (PIC32MX_INT_K1BASE+PIC32MX_INT_INTCON_OFFSET)
#define PIC32MX_INT_INTCONCLR        (PIC32MX_INT_K1BASE+PIC32MX_INT_INTCONCLR_OFFSET)
#define PIC32MX_INT_INTCONSET        (PIC32MX_INT_K1BASE+PIC32MX_INT_INTCONSET_OFFSET)
#define PIC32MX_INT_INTCONINV        (PIC32MX_INT_K1BASE+PIC32MX_INT_INTCONINV_OFFSET)
#define PIC32MX_INT_INTSTAT          (PIC32MX_INT_K1BASE+PIC32MX_INT_INTSTAT_OFFSET)
#define PIC32MX_INT_INTSTATCLR       (PIC32MX_INT_K1BASE+PIC32MX_INT_INTSTATCLR_OFFSET)
#define PIC32MX_INT_INTSTATSET       (PIC32MX_INT_K1BASE+PIC32MX_INT_INTSTATSET_OFFSET)
#define PIC32MX_INT_INTSTATINV       (PIC32MX_INT_K1BASE+PIC32MX_INT_INTSTATINV_OFFSET)
#define PIC32MX_INT_TPTMR            (PIC32MX_INT_K1BASE+PIC32MX_INT_TPTMR_OFFSET)
#define PIC32MX_INT_TPTMRCLR         (PIC32MX_INT_K1BASE+PIC32MX_INT_TPTMRCLR_OFFSET)
#define PIC32MX_INT_TPTMRSET         (PIC32MX_INT_K1BASE+PIC32MX_INT_TPTMRSET_OFFSET)
#define PIC32MX_INT_TPTMRINV         (PIC32MX_INT_K1BASE+PIC32MX_INT_TPTMRINV_OFFSET)
#define PIC32MX_INT_IFS(n)           (PIC32MX_INT_K1BASE+PIC32MX_INT_IFS_OFFSET(n))
#define PIC32MX_INT_IFSCLR(n)        (PIC32MX_INT_K1BASE+PIC32MX_INT_IFSCLR_OFFSET(n))
#define PIC32MX_INT_IFSSET(n)        (PIC32MX_INT_K1BASE+PIC32MX_INT_IFSSET_OFFSET(n))
#define PIC32MX_INT_IFSINV(n)        (PIC32MX_INT_K1BASE+PIC32MX_INT_IFSINV_OFFSET(n))
#define PIC32MX_INT_IFS0             (PIC32MX_INT_K1BASE+PIC32MX_INT_IFS0_OFFSET)
#define PIC32MX_INT_IFS0CLR          (PIC32MX_INT_K1BASE+PIC32MX_INT_IFS0CLR_OFFSET)
#define PIC32MX_INT_IFS0SET          (PIC32MX_INT_K1BASE+PIC32MX_INT_IFS0SET_OFFSET)
#define PIC32MX_INT_IFS0INV          (PIC32MX_INT_K1BASE+PIC32MX_INT_IFS0INV_OFFSET)
#define PIC32MX_INT_IFS1             (PIC32MX_INT_K1BASE+PIC32MX_INT_IFS1_OFFSET)
#define PIC32MX_INT_IFS1CLR          (PIC32MX_INT_K1BASE+PIC32MX_INT_IFS1CLR_OFFSET)
#define PIC32MX_INT_IFS1SET          (PIC32MX_INT_K1BASE+PIC32MX_INT_IFS1SET_OFFSET)
#define PIC32MX_INT_IFS1INV          (PIC32MX_INT_K1BASE+PIC32MX_INT_IFS1INV_OFFSET)
#define PIC32MX_INT_IEC(n)           (PIC32MX_INT_K1BASE+PIC32MX_INT_IEC_OFFSET(n))
#define PIC32MX_INT_IECCLR(n)        (PIC32MX_INT_K1BASE+PIC32MX_INT_IECCLR_OFFSET(n))
#define PIC32MX_INT_IECSET(n)        (PIC32MX_INT_K1BASE+PIC32MX_INT_IECSET_OFFSET(n))
#define PIC32MX_INT_IECINV(n)        (PIC32MX_INT_K1BASE+PIC32MX_INT_IECINV_OFFSET(n))
#define PIC32MX_INT_IEC0             (PIC32MX_INT_K1BASE+PIC32MX_INT_IEC0_OFFSET)
#define PIC32MX_INT_IEC0CLR          (PIC32MX_INT_K1BASE+PIC32MX_INT_IEC0CLR_OFFSET)
#define PIC32MX_INT_IEC0SET          (PIC32MX_INT_K1BASE+PIC32MX_INT_IEC0SET_OFFSET)
#define PIC32MX_INT_IEC0INV          (PIC32MX_INT_K1BASE+PIC32MX_INT_IEC0_OFFSET)
#define PIC32MX_INT_IEC1             (PIC32MX_INT_K1BASE+PIC32MX_INT_IEC1_OFFSET)
#define PIC32MX_INT_IEC1CLR          (PIC32MX_INT_K1BASE+PIC32MX_INT_IEC1CLR_OFFSET)
#define PIC32MX_INT_IEC1SET          (PIC32MX_INT_K1BASE+PIC32MX_INT_IEC1SET_OFFSET)
#define PIC32MX_INT_IEC1INV          (PIC32MX_INT_K1BASE+PIC32MX_INT_IEC1INV_OFFSET)
#define PIC32MX_INT_IPC(n)           (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC_OFFSET(n))
#define PIC32MX_INT_IPCCLR(n)        (PIC32MX_INT_K1BASE+PIC32MX_INT_IPCCLR_OFFSET(n))
#define PIC32MX_INT_IPCSET(n)        (PIC32MX_INT_K1BASE+PIC32MX_INT_IPCSET_OFFSET(n))
#define PIC32MX_INT_IPCINV(n)        (PIC32MX_INT_K1BASE+PIC32MX_INT_IPCINV_OFFSET(n))
#define PIC32MX_INT_IPC0             (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC0_OFFSET)
#define PIC32MX_INT_IPC0CLR          (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC0CLR_OFFSET)
#define PIC32MX_INT_IPC0SET          (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC0SET_OFFSET)
#define PIC32MX_INT_IPC0INV          (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC0INV_OFFSET)
#define PIC32MX_INT_IPC1             (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC1_OFFSET)
#define PIC32MX_INT_IPC1CLR          (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC1CLR_OFFSET)
#define PIC32MX_INT_IPC1SET          (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC1SET_OFFSET)
#define PIC32MX_INT_IPC1INV          (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC1INV_OFFSET)
#define PIC32MX_INT_IPC2             (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC2_OFFSET)
#define PIC32MX_INT_IPC2CLR          (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC2CLR_OFFSET)
#define PIC32MX_INT_IPC2SET          (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC2SET_OFFSET)
#define PIC32MX_INT_IPC2INV          (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC2INV_OFFSET)
#define PIC32MX_INT_IPC3             (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC3_OFFSET)
#define PIC32MX_INT_IPC3CLR          (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC3CLR_OFFSET)
#define PIC32MX_INT_IPC3SET          (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC3SET_OFFSET)
#define PIC32MX_INT_IPC3INV          (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC3INV_OFFSET)
#define PIC32MX_INT_IPC4             (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC4_OFFSET)
#define PIC32MX_INT_IPC4CLR          (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC4CLR_OFFSET)
#define PIC32MX_INT_IPC4SET          (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC4SET_OFFSET)
#define PIC32MX_INT_IPC4INV          (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC4INV_OFFSET)
#define PIC32MX_INT_IPC5             (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC5_OFFSET)
#define PIC32MX_INT_IPC5CLR          (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC5CLR_OFFSET)
#define PIC32MX_INT_IPC5SET          (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC5SET_OFFSET)
#define PIC32MX_INT_IPC5INV          (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC5INV_OFFSET)
#define PIC32MX_INT_IPC6             (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC6_OFFSET)
#define PIC32MX_INT_IPC6CLR          (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC6CLR_OFFSET)
#define PIC32MX_INT_IPC6SET          (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC6SET_OFFSET)
#define PIC32MX_INT_IPC6INV          (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC6INV_OFFSET)
#define PIC32MX_INT_IPC7             (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC7_OFFSET)
#define PIC32MX_INT_IPC7CLR          (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC7CLR_OFFSET)
#define PIC32MX_INT_IPC7SET          (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC7SET_OFFSET)
#define PIC32MX_INT_IPC7INV          (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC7INV_OFFSET)
#define PIC32MX_INT_IPC8             (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC8_OFFSET)
#define PIC32MX_INT_IPC8CLR          (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC8CLR_OFFSET)
#define PIC32MX_INT_IPC8SET          (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC8SET_OFFSET)
#define PIC32MX_INT_IPC8INV          (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC8INV_OFFSET)
#define PIC32MX_INT_IPC9             (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC9_OFFSET)
#define PIC32MX_INT_IPC9CLR          (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC9CLR_OFFSET)
#define PIC32MX_INT_IPC9SET          (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC9SET_OFFSET)
#define PIC32MX_INT_IPC9INV          (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC9INV_OFFSET)
#define PIC32MX_INT_IPC11            (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC11_OFFSET)
#define PIC32MX_INT_IPC11CLR         (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC11CLR_OFFSET)
#define PIC32MX_INT_IPC11SET         (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC11SET_OFFSET)
#define PIC32MX_INT_IPC11INV         (PIC32MX_INT_K1BASE+PIC32MX_INT_IPC11INV_OFFSET)

/* Register Bit-Field Definitions *******************************************/

/* Interrupt control register */

#define INT_INTCON_INT0EP            (1 << 0)  /* Bit 0:  External interrupt 0 edge polarity control */
#define INT_INTCON_INT1EP            (1 << 1)  /* Bit 1:  External interrupt 1 edge polarity control */
#define INT_INTCON_INT2EP            (1 << 2)  /* Bit 2:  External interrupt 2 edge polarity control */
#define INT_INTCON_INT3EP            (1 << 3)  /* Bit 3:  External interrupt 3 edge polarity control */
#define INT_INTCON_INT4EP            (1 << 4)  /* Bit 4:  External interrupt 4 edge polarity control */
#define INT_INTCON_TPC_SHIFT         (8)       /* Bits 8-10: Temporal proximity control */
#define INT_INTCON_TPC_MASK          (7 << INT_INTCON_TPC_SHIFT)
#  define INT_INTCON_TPC_DIS         (0 << INT_INTCON_TPC_SHIFT) /* Disables proximity timer */
#  define INT_INTCON_TPC_PRIO1       (1 << INT_INTCON_TPC_SHIFT) /* Int group priority 1 start IP timer */
#  define INT_INTCON_TPC_PRIO2       (2 << INT_INTCON_TPC_SHIFT) /* Int group priority <=2 start TP timer */
#  define INT_INTCON_TPC_PRIO3       (3 << INT_INTCON_TPC_SHIFT) /* Int group priority <=3 start TP timer */
#  define INT_INTCON_TPC_PRIO4       (4 << INT_INTCON_TPC_SHIFT) /* Int group priority <=4 start TP timer */
#  define INT_INTCON_TPC_PRIO5       (5 << INT_INTCON_TPC_SHIFT) /* Int group priority <=5 start TP timer */
#  define INT_INTCON_TPC_PRIO6       (6 << INT_INTCON_TPC_SHIFT) /* Int group priority <=6 start TP timer */
#  define INT_INTCON_TPC_PRIO7       (7 << INT_INTCON_TPC_SHIFT) /* Int group priority <=7 start TP timer */
#define INT_INTCON_MVEC              (1 << 12) /* Bit 12: Multi vector configuration */
#define INT_INTCON_FRZ               (1 << 14) /* Bit 14: Freeze in debug exception mode */
#define INT_INTCON_SS0               (1 << 16) /* Bit 16: Single vector shadow register set */

/* Interrupt status register */

#define INT_INTSTAT_VEC_SHIFT        (0)       /* Bits 0-5: Interrupt vector */
#define INT_INTSTAT_VEC_MASK         (0x3f << INT_INTSTAT_VEC_SHIFT)
#define INT_INTSTAT_RIPL_SHIFT       (8)       /* Bits 8-10: Requested priority level */
#define INT_INTSTAT_RIPL_MASK        (7 << INT_INTSTAT_RIPL_SHIFT)

/* Temporal proximity timer register -- This register contains a 32-bit value
 * with no field definitions.
 */

/* Interrupt flag status register 0 and Interrupt enable control register 0 */

#define INT_CT                       (1 << 0)  /* Vector: 0, Core Timer Interrupt */
#define INT_CS0                      (1 << 1)  /* Vector: 1, Core Software Interrupt 0 */
#define INT_CS1                      (1 << 2)  /* Vector: 2, Core Software Interrupt 1 */
#define INT_INT0                     (1 << 3)  /* Vector: 3, External Interrupt 0 */
#define INT_T1                       (1 << 4)  /* Vector: 4, Timer 1 */
#define INT_IC1                      (1 << 5)  /* Vector: 5, Input Capture 1 */
#define INT_OC1                      (1 << 6)  /* Vector: 6, Output Compare 1 */
#define INT_INT1                     (1 << 7)  /* Vector: 7, External Interrupt 1 */
#define INT_T2                       (1 << 8)  /* Vector: 8, Timer 2 */
#define INT_IC2                      (1 << 9)  /* Vector: 9, Input Capture 2 */
#define INT_OC2                      (1 << 10) /* Vector: 10, Output Compare 2 */
#define INT_INT2                     (1 << 11) /* Vector: 11, External Interrupt 2 */
#define INT_T3                       (1 << 12) /* Vector: 12, Timer 3 */
#define INT_IC3                      (1 << 13) /* Vector: 13, Input Capture 3 */
#define INT_OC3                      (1 << 14) /* Vector: 14, Output Compare 3 */
#define INT_INT3                     (1 << 15) /* Vector: 15, External Interrupt 3 */
#define INT_T4                       (1 << 16) /* Vector: 16, Timer 4 */
#define INT_IC4                      (1 << 17) /* Vector: 17, Input Capture 4 */
#define INT_OC4                      (1 << 18) /* Vector: 18, Output Compare 4 */
#define INT_INT4                     (1 << 19) /* Vector: 19, External Interrupt 4 */
#define INT_T5                       (1 << 20) /* Vector: 20, Timer 5 */
#define INT_IC5                      (1 << 21) /* Vector: 21, Input Capture 5 */
#define INT_OC5                      (1 << 22) /* Vector: 22, Output Compare 5 */
#define INT_SPI1E                    (1 << 23) /* Vector: 23, SPI1 */
#define INT_SPI1TX                   (1 << 24) /* Vector: 23, "  " */
#define INT_SPI1RX                   (1 << 25) /* Vector: 23, "  " */
#define INT_U1E                      (1 << 26) /* Vector: 24, UART1 */
#define INT_U1RX                     (1 << 27) /* Vector: 24, "   " */
#define INT_U1TX                     (1 << 28) /* Vector: 24, "   " */
#define INT_I2C1B                    (1 << 29) /* Vector: 25, I2C1 */
#define INT_I2C1S                    (1 << 30) /* Vector: 25, "  " */
#define INT_I2C1M                    (1 << 31) /* Vector: 25, "  " */

/* Interrupt flag status register 1 and Interrupt enable control register 1 */

#define INT_CN                       (1 << 0)  /* Vector: 26, Input Change Interrupt */
#define INT_AD1                      (1 << 1)  /* Vector: 27, ADC1 Convert Done */
#define INT_PMP                      (1 << 2)  /* Vector: 28, Parallel Master Port */
#define INT_CMP1                     (1 << 3)  /* Vector: 29, Comparator Interrupt */
#define INT_CMP2                     (1 << 4)  /* Vector: 30, Comparator Interrupt */
#define INT_SPI2E                    (1 << 5)  /* Vector: 31, SPI2 */
#define INT_SPI2TX                   (1 << 6)  /* Vector: 31, "  " */
#define INT_SPI2RX                   (1 << 7)  /* Vector: 31, "  " */
#define INT_U2E                      (1 << 8)  /* Vector: 32, UART2 */
#define INT_U2RX                     (1 << 9)  /* Vector: 32, "   " */
#define INT_U2TX                     (1 << 10) /* Vector: 32, "   " */
#define INT_I2C2B                    (1 << 11) /* Vector: 33, I2C2 */
#define INT_I2C2S                    (1 << 12) /* Vector: 33, "  " */
#define INT_I2C2M                    (1 << 13) /* Vector: 33, "  " */
#define INT_FSCM                     (1 << 14) /* Vector: 34, Fail-Safe Clock Monitor */
#define INT_RTCC                     (1 << 15) /* Vector: 35, Real-Time Clock and Calendar */
#define INT_DMA0                     (1 << 16) /* Vector: 36, DMA Channel 0 */
#define INT_DMA1                     (1 << 17) /* Vector: 37, DMA Channel 1 */
#define INT_DMA2                     (1 << 18) /* Vector: 38, DMA Channel 2 */
#define INT_DMA3                     (1 << 19) /* Vector: 39, DMA Channel 3 */
#define INT_FCE                      (1 << 24) /* Vector: 44, Flash Control Event */
#define INT_USB                      (1 << 25) /* Vector: 45, USB */

/* Interrupt priority control register 0-11 */

#define INT_ICP_DISABLED             0     /* Disabled! */
#define INT_ICP_MIN_PRIORITY         1     /* Minimum (enabled) priority */
#define INT_ICP_MID_PRIORITY         4     /* Can be used as the default */
#define INT_ICP_MAX_PRIORITY         7     /* Maximum priority */
#define INT_ICP_MIN_SUBPRIORITY      0     /* Minimum sub-priority */
#define INT_ICP_MAX_SUBPRIORITY      0     /* Maximum sub-priority */

#define INT_IPC0_CTIS_SHIFT          (0)   /* Bits 0-1, Vector: 0, Core Timer Interrupt */
#define INT_IPC0_CTIS_MASK           (3 << INT_IPC0_CTIS_SHIFT)
#define INT_IPC0_CTIP_SHIFT          (2)   /* Bits 2-4, Vector: 0, Core Timer Interrupt */
#define INT_IPC0_CTIP_MASK           (7 << INT_IPC0_CTIP_SHIFT)
#define INT_IPC0_CS0IS_SHIFT         (8)   /* Bits 8-9, Vector: 1, Core Software Interrupt 0 */
#define INT_IPC0_CS0IS_MASK          (3 << INT_IPC0_CS0IS_SHIFT)
#define INT_IPC0_CS0IP_SHIFT         (10)  /* Bits 10-12, Vector: 1, Core Software Interrupt 0 */
#define INT_IPC0_CS0IP_MASK          (7 << INT_IPC0_CS0IP_SHIFT)
#define INT_IPC0_CS1IS_SHIFT         (16)  /* Bits 16-17, Vector: 2, Core Software Interrupt 1 */
#define INT_IPC0_CS1IS_MASK          (3 << INT_IPC0_CS1IS_SHIFT)
#define INT_IPC0_CS1IP_SHIFT         (18)  /* Bits 18-20, Vector: 2, Core Software Interrupt 1 */
#define INT_IPC0_CS1IP_MASK          (7 << INT_IPC0_CS1IP_SHIFT)
#define INT_IPC0_INT0IS_SHIFT        (24)  /* Bits 24-25, Vector: 3, External Interrupt 0 */
#define INT_IPC0_INT0IS_MASK         (3 << INT_IPC0_INT0IS_SHIFT)
#define INT_IPC0_INT0IP_SHIFT        (26)  /* Bits 26-28, Vector: 3, External Interrupt 0 */
#define INT_IPC0_INT0IP_MASK         (7 << INT_IPC0_INT0IP_SHIFT)

#define INT_IPC1_T1IS_SHIFT          (0)   /* Bits 0-1, Vector: 4, Timer 1 */
#define INT_IPC1_T1IS_MASK           (3 << INT_IPC1_T1IS_SHIFT)
#define INT_IPC1_T1IP_SHIFT          (2)   /* Bits 2-4, Vector: 4, Timer 1 */
#define INT_IPC1_T1IP_MASK           (7 << INT_IPC1_T1IP_SHIFT)
#define INT_IPC1_IC1IS_SHIFT         (8)   /* Bits 8-9, Vector: 5, Input Capture 1 */
#define INT_IPC1_IC1IS_MASK          (3 << INT_IPC1_IC1IS_SHIFT)
#define INT_IPC1_IC1IP_SHIFT         (10)  /* Bits 10-12, Vector: 5, Input Capture 1 */
#define INT_IPC1_IC1IP_MASK          (7 << INT_IPC1_IC1IP_SHIFT)
#define INT_IPC1_OC1IS_SHIFT         (16)  /* Bits 16-17, Vector: 6, Output Compare 1 */
#define INT_IPC1_OC1IS_MASK          (3 << INT_IPC1_OC1IS_SHIFT)
#define INT_IPC1_OC1IP_SHIFT         (18)  /* Bits 18-20, Vector: 6, Output Compare 1 */
#define INT_IPC1_OC1IP_MASK          (7 << INT_IPC1_OC1IP_SHIFT)
#define INT_IPC1_INT1IS_SHIFT        (24)  /* Bits 24-25, Vector: 7, External Interrupt 1 */
#define INT_IPC1_INT1IS_MASK         (3 << INT_IPC1_INT1IS_SHIFT)
#define INT_IPC1_INT1IP_SHIFT        (26)  /* Bits 26-28, Vector: 7, External Interrupt 1 */
#define INT_IPC1_INT1IP_MASK         (7 << INT_IPC1_INT1IP_SHIFT)

#define INT_IPC2_T2IS_SHIFT          (0)   /* Bits 0-1, Vector: 8, Timer 2 */
#define INT_IPC2_T2IS_MASK           (3 << INT_IPC2_T2IS_SHIFT)
#define INT_IPC2_T2IP_SHIFT          (2)   /* Bits 2-4, Vector: 8, Timer 2 */
#define INT_IPC2_T2IP_MASK           (7 << INT_IPC2_T2IP_SHIFT)
#define INT_IPC2_IC2IS_SHIFT         (8)   /* Bits 8-9, Vector: 9, Input Capture 2 */
#define INT_IPC2_IC2IS_MASK          (3 << INT_IPC2_IC2IS_SHIFT)
#define INT_IPC2_IC2IP_SHIFT         (10)  /* Bits 10-12, Vector: 9, Input Capture 2 */
#define INT_IPC2_IC2IP_MASK          (7 << INT_IPC2_IC2IP_SHIFT)
#define INT_IPC2_OC2IS_SHIFT         (16)  /* Bits 16-17, Vector: 10, Output Compare 2 */
#define INT_IPC2_OC2IS_MASK          (3 << INT_IPC2_OC2IS_SHIFT)
#define INT_IPC2_OC2IP_SHIFT         (18)  /* Bits 18-20, Vector: 10, Output Compare 2 */
#define INT_IPC2_OC2IP_MASK          (7 << INT_IPC2_OC2IP_SHIFT)
#define INT_IPC2_INT2IS_SHIFT        (24)  /* Bits 24-25, Vector: 11, External Interrupt 2 */
#define INT_IPC2_INT2IS_MASK         (3 << INT_IPC2_INT2IS_SHIFT)
#define INT_IPC2_INT2IP_SHIFT        (26)  /* Bits 26-28, Vector: 11, External Interrupt 2 */
#define INT_IPC2_INT2IP_MASK         (7 << INT_IPC2_INT2IP_SHIFT)

#define INT_IPC3_T3IS_SHIFT          (0)   /* Bits 0-1, Vector: 12, Timer 3 */
#define INT_IPC3_T3IS_MASK           (3 << INT_IPC3_T3IS_SHIFT)
#define INT_IPC3_T3IP_SHIFT          (2)   /* Bits 2-4, Vector: 12, Timer 3 */
#define INT_IPC3_T3IP_MASK           (7 << INT_IPC3_T3IP_SHIFT)
#define INT_IPC3_IC3IS_SHIFT         (8)   /* Bits 8-9, Vector: 13, Input Capture 3 */
#define INT_IPC3_IC3IS_MASK          (3 << INT_IPC3_IC3IS_SHIFT)
#define INT_IPC3_IC3IP_SHIFT         (10)  /* Bits 10-12, Vector: 13, Input Capture 3 */
#define INT_IPC3_IC3IP_MASK          (7 << INT_IPC3_IC3IP_SHIFT)
#define INT_IPC3_OC3IS_SHIFT         (16)  /* Bits 16-17, Vector: 14, Output Compare 3 */
#define INT_IPC3_OC3IS_MASK          (3 << INT_IPC3_OC3IS_SHIFT)
#define INT_IPC3_OC3IP_SHIFT         (18)  /* Bits 18-20, Vector: 14, Output Compare 3 */
#define INT_IPC3_OC3IP_MASK          (7 << INT_IPC3_OC3IP_SHIFT)
#define INT_IPC3_INT3IS_SHIFT        (24)  /* Bits 24-25, Vector: 15, External Interrupt 3 */
#define INT_IPC3_INT3IS_MASK         (3 << INT_IPC3_INT3IS_SHIFT)
#define INT_IPC3_INT3IP_SHIFT        (26)  /* Bits 26-28, Vector: 15, External Interrupt 3 */
#define INT_IPC3_INT3IP_MASK         (7 << INT_IPC3_INT3IP_SHIFT)

#define INT_IPC4_T4IS_SHIFT          (0)   /* Bits 0-1, Vector: 16, Timer 4 */
#define INT_IPC4_T4IS_MASK           (3 << INT_IPC4_T4IS_SHIFT)
#define INT_IPC4_T4IP_SHIFT          (2)   /* Bits 2-4, Vector: 16, Timer 4 */
#define INT_IPC4_T4IP_MASK           (7 << INT_IPC4_T4IP_SHIFT)
#define INT_IPC4_IC4IS_SHIFT         (8)   /* Bits 8-9, Vector: 17, Input Capture 4 */
#define INT_IPC4_IC4IS_MASK          (3 << INT_IPC4_IC4IS_SHIFT)
#define INT_IPC4_IC4IP_SHIFT         (10)  /* Bits 10-12, Vector: 17, Input Capture 4 */
#define INT_IPC4_IC4IP_MASK          (7 << INT_IPC4_IC4IP_SHIFT)
#define INT_IPC4_OC4IS_SHIFT         (16)  /* Bits 16-17, Vector: 18, Output Compare 4 */
#define INT_IPC4_OC4IS_MASK          (3 << INT_IPC4_OC4IS_SHIFT)
#define INT_IPC4_OC4IP_SHIFT         (18)  /* Bits 18-20, Vector: 18, Output Compare 4 */
#define INT_IPC4_OC4IP_MASK          (7 << INT_IPC4_OC4IP_SHIFT)
#define INT_IPC4_INT4IS_SHIFT        (24)  /* Bits 24-25, Vector: 19, External Interrupt 4 */
#define INT_IPC4_INT4IS_MASK         (3 << INT_IPC4_INT4IS_SHIFT)
#define INT_IPC4_INT4IP_SHIFT        (26)  /* Bits 26-28, Vector: 19, External Interrupt 4 */
#define INT_IPC4_INT4IP_MASK         (7 << INT_IPC4_INT4IP_SHIFT)

#define INT_IPC5_T5IS_SHIFT          (0)   /* Bits 0-1, Vector: 20, Timer 5 */
#define INT_IPC5_T5IS_MASK           (3 << INT_IPC5_T5IS_SHIFT)
#define INT_IPC5_T5IP_SHIFT          (2)   /* Bits 2-4, Vector: 20, Timer 5 */
#define INT_IPC5_T5IP_MASK           (7 << INT_IPC5_T5IP_SHIFT)
#define INT_IPC5_IC5IS_SHIFT         (8)   /* Bits 8-9, Vector: 21, Input Capture 5 */
#define INT_IPC5_IC5IS_MASK          (3 << INT_IPC5_IC5IS_SHIFT)
#define INT_IPC5_IC5IP_SHIFT         (10)  /* Bits 10-12, Vector: 21, Input Capture 5 */
#define INT_IPC5_IC5IP_MASK          (7 << INT_IPC5_IC5IP_SHIFT)
#define INT_IPC5_OC5IS_SHIFT         (16)  /* Bits 16-17, Vector: 22, Output Compare 5 */
#define INT_IPC5_OC5IS_MASK          (3 << INT_IPC5_OC5IS_SHIFT)
#define INT_IPC5_OC5IP_SHIFT         (18)  /* Bits 18-20, Vector: 22, Output Compare 5 */
#define INT_IPC5_OC5IP_MASK          (7 << INT_IPC5_OC5IP_SHIFT)
#define INT_IPC5_SPI1IS_SHIFT        (24)  /* Bits 24-25, Vector: 23, SPI1 */
#define INT_IPC5_SPI1IS_MASK         (3 << INT_IPC5_SPI1IS_SHIFT)
#define INT_IPC5_SPI1IP_SHIFT        (26)  /* Bits 26-28, Vector: 23, SPI1 */
#define INT_IPC5_SPI1IP_MASK         (7 << INT_IPC5_SPI1IP_SHIFT)

#define INT_IPC6_U1IS_SHIFT          (0)   /* Bits 0-1, Vector: 24, UART1 */
#define INT_IPC6_U1IS_MASK           (3 << INT_IPC6_U1IS_SHIFT)
#define INT_IPC6_U1IP_SHIFT          (2)   /* Bits 2-4, Vector: 24, UART1 */
#define INT_IPC6_U1IP_MASK           (7 << INT_IPC6_U1IP_SHIFT)
#define INT_IPC6_I2C1IS_SHIFT        (8)   /* Bits 8-9, Vector: 25, I2C1 */
#define INT_IPC6_I2C1IS_MASK         (3 << INT_IPC6_I2C1IS_SHIFT)
#define INT_IPC6_I2C1IP_SHIFT        (10)  /* Bits 10-12, Vector: 25, I2C1 */
#define INT_IPC6_I2C1IP_MASK         (7 << INT_IPC6_I2C1IP_SHIFT)
#define INT_IPC6_CNIS_SHIFT          (16)  /* Bits 16-17, Vector: 26, Input Change Interrupt */
#define INT_IPC6_CNIS_MASK           (3 << INT_IPC6_CNIS_SHIFT)
#define INT_IPC6_CNIP_SHIFT          (18)  /* Bits 18-20, Vector: 26, Input Change Interrupt */
#define INT_IPC6_CNIP_MASK           (7 << INT_IPC6_CNIP_SHIFT)
#define INT_IPC6_AD1IS_SHIFT         (24)  /* Bits 24-25, Vector: 27, ADC1 Convert Done */
#define INT_IPC6_AD1IS_MASK          (3 << INT_IPC6_AD1IS_SHIFT)
#define INT_IPC6_AD1IP_SHIFT         (26)  /* Bits 26-28, Vector: 27, ADC1 Convert Done */
#define INT_IPC6_AD1IP_MASK          (7 << INT_IPC6_AD1IP_SHIFT)

#define INT_IPC7_PMPIS_SHIFT         (0)   /* Bits 0-1, Vector: 28, Parallel Master Port */
#define INT_IPC7_PMPIS_MASK          (3 << INT_IPC7_PMPIS_SHIFT)
#define INT_IPC7_PMPIP_SHIFT         (2)   /* Bits 2-4, Vector: 28, Parallel Master Port */
#define INT_IPC7_PMPIP_MASK          (7 << INT_IPC7_PMPIP_SHIFT)
#define INT_IPC7_CMP1IS_SHIFT        (8)   /* Bits 8-9, Vector: 29, Comparator Interrupt */
#define INT_IPC7_CMP1IS_MASK         (3 << INT_IPC7_CMP1IS_SHIFT)
#define INT_IPC7_CMP1IP_SHIFT        (10)  /* Bits 10-12, Vector: 29, Comparator Interrupt */
#define INT_IPC7_CMP1IP_MASK         (7 << INT_IPC7_CMP1IP_SHIFT)
#define INT_IPC7_CMP2IS_SHIFT        (16)  /* Bits 16-17, Vector: 30, Comparator Interrupt */
#define INT_IPC7_CMP2IS_MASK         (3 << INT_IPC7_CMP2IS_SHIFT)
#define INT_IPC7_CMP2IP_SHIFT        (18)  /* Bits 18-20, Vector: 30, Comparator Interrupt */
#define INT_IPC7_CMP2IP_MASK         (7 << INT_IPC7_CMP2IP_SHIFT)
#define INT_IPC7_SPI2IS_SHIFT        (24)  /* Bits 24-25, Vector: 31, SPI2 */
#define INT_IPC7_SPI2IS_MASK         (3 << INT_IPC7_SPI2IS_SHIFT)
#define INT_IPC7_SPI2IP_SHIFT        (26)  /* Bits 26-28, Vector: 31, SPI2 */
#define INT_IPC7_SPI2IP_MASK         (7 << INT_IPC7_SPI2IP_SHIFT)

#define INT_IPC8_U2IS_SHIFT          (0)   /* Bits 0-1, Vector: 32, UART2 */
#define INT_IPC8_U2IS_MASK           (3 << INT_IPC8_U2IS_SHIFT)
#define INT_IPC8_U2IP_SHIFT          (2)   /* Bits 2-4, Vector: 32, UART2 */
#define INT_IPC8_U2IP_MASK           (7 << INT_IPC8_U2IP_SHIFT)
#define INT_IPC8_I2C2IS_SHIFT        (8)   /* Bits 8-9, Vector: 33, I2C2 */
#define INT_IPC8_I2C2IS_MASK         (3 << INT_IPC8_I2C2IS_SHIFT)
#define INT_IPC8_I2C2IP_SHIFT        (10)  /* Bits 10-12, Vector: 33, I2C2 */
#define INT_IPC8_I2C2IP_MASK         (7 << INT_IPC8_I2C2IP_SHIFT)
#define INT_IPC8_FSCMIS_SHIFT        (16)  /* Bits 16-17, Vector: 34, Fail-Safe Clock Monitor */
#define INT_IPC8_FSCMIS_MASK         (3 << INT_IPC8_FSCMIS_SHIFT)
#define INT_IPC8_FSCMIP_SHIFT        (18)  /* Bits 18-20, Vector: 34, Fail-Safe Clock Monitor */
#define INT_IPC8_FSCMIP_MASK         (7 << INT_IPC8_FSCMIP_SHIFT)
#define INT_IPC8_RTCCIS_SHIFT        (24)  /* Bits 24-25, Vector: 35, Real-Time Clock and Calendar */
#define INT_IPC8_RTCCIS_MASK         (3 << INT_IPC8_RTCCIS_SHIFT)
#define INT_IPC8_RTCCIP_SHIFT        (26)  /* Bits 26-28, Vector: 35, Real-Time Clock and Calendar */
#define INT_IPC8_RTCCIP_MASK         (7 << INT_IPC8_RTCCIP_SHIFT)

#define INT_IPC9_DMA0IS_SHIFT        (0)   /* Bits 0-1, Vector: 36, DMA Channel 0 */
#define INT_IPC9_DMA0IS_MASK         (3 << INT_IPC9_DMA0IS_SHIFT)
#define INT_IPC9_DMA0IP_SHIFT        (2)   /* Bits 2-4, Vector: 36, DMA Channel 0 */
#define INT_IPC9_DMA0IP_MASK         (7 << INT_IPC9_DMA0IP_SHIFT)
#define INT_IPC9_DMA1IS_SHIFT        (8)   /* Bits 8-9, Vector: 37, DMA Channel 1 */
#define INT_IPC9_DMA1IS_MASK         (3 << INT_IPC9_DMA1IS_SHIFT)
#define INT_IPC9_DMA1IP_SHIFT        (10)  /* Bits 10-12, Vector: 37, DMA Channel 1 */
#define INT_IPC9_DMA1IP_MASK         (7 << INT_IPC9_DMA1IP_SHIFT)
#define INT_IPC9_DMA2IS_SHIFT        (16)  /* Bits 16-17, Vector: 38, DMA Channel 2 */
#define INT_IPC9_DMA2IS_MASK         (3 << INT_IPC9_DMA2IS_SHIFT)
#define INT_IPC9_DMA2IP_SHIFT        (18)  /* Bits 18-20, Vector: 38, DMA Channel 2 */
#define INT_IPC9_DMA2IP_MASK         (7 << INT_IPC9_DMA2IP_SHIFT)
#define INT_IPC9_DMA3IS_SHIFT        (24)  /* Bits 24-25, Vector: 39, DMA Channel 3 */
#define INT_IPC9_DMA3IS_MASK         (3 << INT_IPC9_DMA3IS_SHIFT)
#define INT_IPC9_DMA3IP_SHIFT        (26)  /* Bits 26-28, Vector: 39, DMA Channel 3 */
#define INT_IPC9_DMA3IP_MASK         (7 << INT_IPC9_DMA3IP_SHIFT)

#define INT_IPC11_FCEIS_SHIFT        (0)   /* Bits 0-1, Vector: 44, Flash Control Event */
#define INT_IPC11_FCEIS_MASK         (3 << INT_IPC11_FCEIS_SHIFT)
#define INT_IPC11_FCEIP_SHIFT        (2)   /* Bits 2-4, Vector: 44, Flash Control Event */
#define INT_IPC11_FCEIP_MASK         (7 << INT_IPC11_FCEIP_SHIFT)
#define INT_IPC11_USBIS_SHIFT        (8)   /* Bits 8-9, Vector: 45, USB */
#define INT_IPC11_USBIS_MASK         (3 << INT_IPC11_USBIS_SHIFT)
#define INT_IPC11_USBIP_SHIFT        (10)  /* Bits 10-12, Vector: 45, USB */
#define INT_IPC11_USBIP_MASK         (7 << INT_IPC11_USBIP_SHIFT)

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
#endif /* __ARCH_MIPS_SRC_PIC32MX_PIC32MX_INT_H */
