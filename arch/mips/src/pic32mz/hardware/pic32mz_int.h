/****************************************************************************
 * arch/mips/src/pic32mz/hardware/pic32mz_int.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZ_INT_H
#define __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZ_INT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "pic32mz_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define PIC32MZ_INT_INTCON_OFFSET     0x0000 /* Interrupt control register */
#define PIC32MZ_INT_INTCONCLR_OFFSET  0x0004 /* Interrupt control clear register */
#define PIC32MZ_INT_INTCONSET_OFFSET  0x0008 /* Interrupt control set register */
#define PIC32MZ_INT_INTCONINV_OFFSET  0x000c /* Interrupt control invert register */

#define PIC32MZ_INT_PRISS_OFFSET       0x0010 /* Priority shadow select register */
#define PIC32MZ_INT_PRISSCLR_OFFSET    0x0010 /* Priority shadow clear register */
#define PIC32MZ_INT_PRISSSET_OFFSET    0x0010 /* Priority shadow set register */
#define PIC32MZ_INT_PRISSINV_OFFSET    0x0010 /* Priority shadow invert register */

#define PIC32MZ_INT_INTSTAT_OFFSET    0x0020 /* Interrupt status register */
#define PIC32MZ_INT_INTSTATCLR_OFFSET 0x0024 /* Interrupt status clear register */
#define PIC32MZ_INT_INTSTATSET_OFFSET 0x0028 /* Interrupt status set register */
#define PIC32MZ_INT_INTSTATINV_OFFSET 0x002c /* Interrupt status invert register */

#define PIC32MZ_INT_IPTMR_OFFSET      0x0030 /* Interrupt proximity timer register */
#define PIC32MZ_INT_IPTMRCLR_OFFSET   0x0034 /* Interrupt proximity timer clear register */
#define PIC32MZ_INT_IPTMRSET_OFFSET   0x0038 /* Interrupt proximity timer set register */
#define PIC32MZ_INT_IPTMRINV_OFFSET   0x003c /* Interrupt proximity timer invert register */

#define PIC32MZ_INT_IFS_OFFSET(n)     (0x0040 + ((n) << 4))
#define PIC32MZ_INT_IFSCLR_OFFSET(n)  (0x0044 + ((n) << 4))
#define PIC32MZ_INT_IFSSET_OFFSET(n)  (0x0048 + ((n) << 4))
#define PIC32MZ_INT_IFSINV_OFFSET(n)  (0x004c + ((n) << 4))

#define PIC32MZ_INT_IFS0_OFFSET       0x0040 /* Interrupt flag status register 0 */
#define PIC32MZ_INT_IFS0CLR_OFFSET    0x0044 /* Interrupt flag status clear register 0 */
#define PIC32MZ_INT_IFS0SET_OFFSET    0x0048 /* Interrupt flag status set register 0 */
#define PIC32MZ_INT_IFS0INV_OFFSET    0x004c /* Interrupt flag status invert register 0 */

#define PIC32MZ_INT_IFS1_OFFSET       0x0050 /* Interrupt flag status register 1 */
#define PIC32MZ_INT_IFS1CLR_OFFSET    0x0054 /* Interrupt flag status clear register 1 */
#define PIC32MZ_INT_IFS1SET_OFFSET    0x0058 /* Interrupt flag status set register 1 */
#define PIC32MZ_INT_IFS1INV_OFFSET    0x005c /* Interrupt flag status invert register 1 */

#define PIC32MZ_INT_IFS2_OFFSET       0x0060 /* Interrupt flag status register 2 */
#define PIC32MZ_INT_IFS2CLR_OFFSET    0x0064 /* Interrupt flag status clear register 2 */
#define PIC32MZ_INT_IFS2SET_OFFSET    0x0068 /* Interrupt flag status set register 2 */
#define PIC32MZ_INT_IFS2INV_OFFSET    0x006c /* Interrupt flag status invert register 2 */

#define PIC32MZ_INT_IFS3_OFFSET       0x0070 /* Interrupt flag status register 3 */
#define PIC32MZ_INT_IFS3CLR_OFFSET    0x0074 /* Interrupt flag status clear register 3 */
#define PIC32MZ_INT_IFS3SET_OFFSET    0x0078 /* Interrupt flag status set register 3 */
#define PIC32MZ_INT_IFS3INV_OFFSET    0x007c /* Interrupt flag status invert register 3 */

#define PIC32MZ_INT_IFS4_OFFSET       0x0080 /* Interrupt flag status register 4 */
#define PIC32MZ_INT_IFS4CLR_OFFSET    0x0084 /* Interrupt flag status clear register 4 */
#define PIC32MZ_INT_IFS4SET_OFFSET    0x0088 /* Interrupt flag status set register 4 */
#define PIC32MZ_INT_IFS4INV_OFFSET    0x008c /* Interrupt flag status invert register 4 */

#define PIC32MZ_INT_IFS5_OFFSET       0x0090 /* Interrupt flag status register 5 */
#define PIC32MZ_INT_IFS5CLR_OFFSET    0x0094 /* Interrupt flag status clear register 5 */
#define PIC32MZ_INT_IFS5SET_OFFSET    0x0098 /* Interrupt flag status set register 5 */
#define PIC32MZ_INT_IFS5INV_OFFSET    0x009c /* Interrupt flag status invert register 5 */

#define PIC32MZ_INT_IEC_OFFSET(n)     (0x00c0 + ((n) << 4))
#define PIC32MZ_INT_IECCLR_OFFSET(n)  (0x00c4 + ((n) << 4))
#define PIC32MZ_INT_IECSET_OFFSET(n)  (0x00c8 + ((n) << 4))
#define PIC32MZ_INT_IECINV_OFFSET(n)  (0x00cc + ((n) << 4))

#define PIC32MZ_INT_IEC0_OFFSET       0x00c0 /* Interrupt enable control register 0 */
#define PIC32MZ_INT_IEC0CLR_OFFSET    0x00c4 /* Interrupt enable control clear register 0 */
#define PIC32MZ_INT_IEC0SET_OFFSET    0x00c8 /* Interrupt enable control set register 0 */
#define PIC32MZ_INT_IEC0INV_OFFSET    0x00cc /* Interrupt enable control invert register 0 */

#define PIC32MZ_INT_IEC1_OFFSET       0x00d0 /* Interrupt enable control register 1 */
#define PIC32MZ_INT_IEC1CLR_OFFSET    0x00d4 /* Interrupt enable control clear register 1 */
#define PIC32MZ_INT_IEC1SET_OFFSET    0x00d8 /* Interrupt enable control set register 1 */
#define PIC32MZ_INT_IEC1INV_OFFSET    0x00dc /* Interrupt enable control invert register 1 */

#define PIC32MZ_INT_IEC2_OFFSET       0x00e0 /* Interrupt enable control register 2 */
#define PIC32MZ_INT_IEC2CLR_OFFSET    0x00e4 /* Interrupt enable control clear register 2 */
#define PIC32MZ_INT_IEC2SET_OFFSET    0x00e8 /* Interrupt enable control set register 2 */
#define PIC32MZ_INT_IEC2INV_OFFSET    0x00ec /* Interrupt enable control invert register 2 */

#define PIC32MZ_INT_IEC3_OFFSET       0x00f0 /* Interrupt enable control register 3 */
#define PIC32MZ_INT_IEC3CLR_OFFSET    0x00f4 /* Interrupt enable control clear register 3 */
#define PIC32MZ_INT_IEC3SET_OFFSET    0x00f8 /* Interrupt enable control set register 3 */
#define PIC32MZ_INT_IEC3INV_OFFSET    0x00fc /* Interrupt enable control invert register 3 */

#define PIC32MZ_INT_IEC4_OFFSET       0x0100 /* Interrupt enable control register 4 */
#define PIC32MZ_INT_IEC4CLR_OFFSET    0x0104 /* Interrupt enable control clear register 4 */
#define PIC32MZ_INT_IEC4SET_OFFSET    0x0108 /* Interrupt enable control set register 4 */
#define PIC32MZ_INT_IEC4INV_OFFSET    0x010c /* Interrupt enable control invert register 4 */

#define PIC32MZ_INT_IEC5_OFFSET       0x0110 /* Interrupt enable control register 5 */
#define PIC32MZ_INT_IEC5CLR_OFFSET    0x0114 /* Interrupt enable control clear register 5 */
#define PIC32MZ_INT_IEC5SET_OFFSET    0x0118 /* Interrupt enable control set register 5 */
#define PIC32MZ_INT_IEC5INV_OFFSET    0x011c /* Interrupt enable control invert register 5 */

#define PIC32MZ_INT_IPC_OFFSET(n)     (0x0140 + ((n) << 4))
#define PIC32MZ_INT_IPCCLR_OFFSET(n)  (0x0144 + ((n) << 4))
#define PIC32MZ_INT_IPCSET_OFFSET(n)  (0x0148 + ((n) << 4))
#define PIC32MZ_INT_IPCINV_OFFSET(n)  (0x014c + ((n) << 4))

#define PIC32MZ_INT_IPC0_OFFSET       0x0140 /* Interrupt priority control register 0 */
#define PIC32MZ_INT_IPC0CLR_OFFSET    0x0144 /* Interrupt priority control clear register 0 */
#define PIC32MZ_INT_IPC0SET_OFFSET    0x0148 /* Interrupt priority control set register 0 */
#define PIC32MZ_INT_IPC0INV_OFFSET    0x014c /* Interrupt priority control invert register 0 */

#define PIC32MZ_INT_IPC1_OFFSET       0x0150 /* Interrupt priority control register 1 */
#define PIC32MZ_INT_IPC1CLR_OFFSET    0x0154 /* Interrupt priority control clear register 1 */
#define PIC32MZ_INT_IPC1SET_OFFSET    0x0158 /* Interrupt priority control set register 1 */
#define PIC32MZ_INT_IPC1INV_OFFSET    0x015c /* Interrupt priority control invert register 1 */

#define PIC32MZ_INT_IPC2_OFFSET       0x0160 /* Interrupt priority control register 2 */
#define PIC32MZ_INT_IPC2CLR_OFFSET    0x0164 /* Interrupt priority control clear register 2 */
#define PIC32MZ_INT_IPC2SET_OFFSET    0x0168 /* Interrupt priority control set register 2 */
#define PIC32MZ_INT_IPC2INV_OFFSET    0x016c /* Interrupt priority control invert register 2 */

#define PIC32MZ_INT_IPC3_OFFSET       0x0170 /* Interrupt priority control register 3 */
#define PIC32MZ_INT_IPC3CLR_OFFSET    0x0174 /* Interrupt priority control clear register 3 */
#define PIC32MZ_INT_IPC3SET_OFFSET    0x0178 /* Interrupt priority control set register 3 */
#define PIC32MZ_INT_IPC3INV_OFFSET    0x017c /* Interrupt priority control invert register 3 */

#define PIC32MZ_INT_IPC4_OFFSET       0x0180 /* Interrupt priority control register 4 */
#define PIC32MZ_INT_IPC4CLR_OFFSET    0x0184 /* Interrupt priority control clear register 4 */
#define PIC32MZ_INT_IPC4SET_OFFSET    0x0188 /* Interrupt priority control set register 4 */
#define PIC32MZ_INT_IPC4INV_OFFSET    0x018c /* Interrupt priority control invert register 4 */

#define PIC32MZ_INT_IPC5_OFFSET       0x0190 /* Interrupt priority control register 5 */
#define PIC32MZ_INT_IPC5CLR_OFFSET    0x0194 /* Interrupt priority control clear register 5 */
#define PIC32MZ_INT_IPC5SET_OFFSET    0x0198 /* Interrupt priority control set register 5 */
#define PIC32MZ_INT_IPC5INV_OFFSET    0x019c /* Interrupt priority control invert register 5 */

#define PIC32MZ_INT_IPC6_OFFSET       0x01a0 /* Interrupt priority control register 6 */
#define PIC32MZ_INT_IPC6CLR_OFFSET    0x01a4 /* Interrupt priority control clear register 6 */
#define PIC32MZ_INT_IPC6SET_OFFSET    0x01a8 /* Interrupt priority control set register 6 */
#define PIC32MZ_INT_IPC6INV_OFFSET    0x01ac /* Interrupt priority control invert register 6 */

#define PIC32MZ_INT_IPC7_OFFSET       0x01b0 /* Interrupt priority control register 7 */
#define PIC32MZ_INT_IPC7CLR_OFFSET    0x01b4 /* Interrupt priority control clear register 7 */
#define PIC32MZ_INT_IPC7SET_OFFSET    0x01b8 /* Interrupt priority control set register 7 */
#define PIC32MZ_INT_IPC7INV_OFFSET    0x01bc /* Interrupt priority control invert register 7 */

#define PIC32MZ_INT_IPC8_OFFSET       0x01c0 /* Interrupt priority control register 8 */
#define PIC32MZ_INT_IPC8CLR_OFFSET    0x01c4 /* Interrupt priority control clear register 8 */
#define PIC32MZ_INT_IPC8SET_OFFSET    0x01c8 /* Interrupt priority control set register 8 */
#define PIC32MZ_INT_IPC8INV_OFFSET    0x01cc /* Interrupt priority control invert register 8 */

#define PIC32MZ_INT_IPC9_OFFSET       0x01d0 /* Interrupt priority control register 9 */
#define PIC32MZ_INT_IPC9CLR_OFFSET    0x01d4 /* Interrupt priority control clear register 9 */
#define PIC32MZ_INT_IPC9SET_OFFSET    0x01d8 /* Interrupt priority control set register 9 */
#define PIC32MZ_INT_IPC9INV_OFFSET    0x01dc /* Interrupt priority control invert register 9 */

#define PIC32MZ_INT_IPC10_OFFSET      0x01e0 /* Interrupt priority control register 10 */
#define PIC32MZ_INT_IPC10CLR_OFFSET   0x01e4 /* Interrupt priority control clear register 10 */
#define PIC32MZ_INT_IPC10SET_OFFSET   0x01e8 /* Interrupt priority control set register 10 */
#define PIC32MZ_INT_IPC10INV_OFFSET   0x01ec /* Interrupt priority control invert register 10 */

#define PIC32MZ_INT_IPC11_OFFSET      0x01f0 /* Interrupt priority control register 11 */
#define PIC32MZ_INT_IPC11CLR_OFFSET   0x01f4 /* Interrupt priority control clear register 11 */
#define PIC32MZ_INT_IPC11SET_OFFSET   0x01f8 /* Interrupt priority control set register 11 */
#define PIC32MZ_INT_IPC11INV_OFFSET   0x01fc /* Interrupt priority control invert register 11 */

#define PIC32MZ_INT_IPC12_OFFSET      0x0200 /* Interrupt priority control register 12 */
#define PIC32MZ_INT_IPC12CLR_OFFSET   0x0204 /* Interrupt priority control clear register 12 */
#define PIC32MZ_INT_IPC12SET_OFFSET   0x0208 /* Interrupt priority control set register 12 */
#define PIC32MZ_INT_IPC12INV_OFFSET   0x020c /* Interrupt priority control invert register 12 */

#define PIC32MZ_INT_IPC13_OFFSET      0x0210 /* Interrupt priority control register 13 */
#define PIC32MZ_INT_IPC13CLR_OFFSET   0x0214 /* Interrupt priority control clear register 13 */
#define PIC32MZ_INT_IPC13SET_OFFSET   0x0218 /* Interrupt priority control set register 13 */
#define PIC32MZ_INT_IPC13INV_OFFSET   0x021c /* Interrupt priority control invert register 13 */

#define PIC32MZ_INT_IPC14_OFFSET      0x0220 /* Interrupt priority control register 14 */
#define PIC32MZ_INT_IPC14CLR_OFFSET   0x0224 /* Interrupt priority control clear register 14 */
#define PIC32MZ_INT_IPC14SET_OFFSET   0x0228 /* Interrupt priority control set register 14 */
#define PIC32MZ_INT_IPC14INV_OFFSET   0x022c /* Interrupt priority control invert register 14 */

#define PIC32MZ_INT_IPC15_OFFSET      0x0230 /* Interrupt priority control register 15 */
#define PIC32MZ_INT_IPC15CLR_OFFSET   0x0234 /* Interrupt priority control clear register 15 */
#define PIC32MZ_INT_IPC15SET_OFFSET   0x0238 /* Interrupt priority control set register 15 */
#define PIC32MZ_INT_IPC15INV_OFFSET   0x023c /* Interrupt priority control invert register 15 */

#define PIC32MZ_INT_IPC16_OFFSET      0x0240 /* Interrupt priority control register 16 */
#define PIC32MZ_INT_IPC16CLR_OFFSET   0x0244 /* Interrupt priority control clear register 16 */
#define PIC32MZ_INT_IPC16SET_OFFSET   0x0248 /* Interrupt priority control set register 16 */
#define PIC32MZ_INT_IPC16INV_OFFSET   0x024c /* Interrupt priority control invert register 16 */

#define PIC32MZ_INT_IPC17_OFFSET      0x0250 /* Interrupt priority control register 17 */
#define PIC32MZ_INT_IPC17CLR_OFFSET   0x0254 /* Interrupt priority control clear register 17 */
#define PIC32MZ_INT_IPC17SET_OFFSET   0x0258 /* Interrupt priority control set register 17 */
#define PIC32MZ_INT_IPC17INV_OFFSET   0x025c /* Interrupt priority control invert register 17 */

#define PIC32MZ_INT_IPC18_OFFSET      0x0260 /* Interrupt priority control register 18 */
#define PIC32MZ_INT_IPC18CLR_OFFSET   0x0264 /* Interrupt priority control clear register 18 */
#define PIC32MZ_INT_IPC18SET_OFFSET   0x0268 /* Interrupt priority control set register 18 */
#define PIC32MZ_INT_IPC18INV_OFFSET   0x026c /* Interrupt priority control invert register 18 */

#define PIC32MZ_INT_IPC19_OFFSET      0x0270 /* Interrupt priority control register 19 */
#define PIC32MZ_INT_IPC19CLR_OFFSET   0x0274 /* Interrupt priority control clear register 19 */
#define PIC32MZ_INT_IPC19SET_OFFSET   0x0278 /* Interrupt priority control set register 19 */
#define PIC32MZ_INT_IPC19INV_OFFSET   0x027c /* Interrupt priority control invert register 19 */

#define PIC32MZ_INT_IPC20_OFFSET      0x0280 /* Interrupt priority control register 20 */
#define PIC32MZ_INT_IPC20CLR_OFFSET   0x0284 /* Interrupt priority control clear register 20 */
#define PIC32MZ_INT_IPC20SET_OFFSET   0x0288 /* Interrupt priority control set register 20 */
#define PIC32MZ_INT_IPC20INV_OFFSET   0x028c /* Interrupt priority control invert register 20 */

#define PIC32MZ_INT_IPC21_OFFSET      0x0290 /* Interrupt priority control register 21 */
#define PIC32MZ_INT_IPC21CLR_OFFSET   0x0294 /* Interrupt priority control clear register 21 */
#define PIC32MZ_INT_IPC21SET_OFFSET   0x0298 /* Interrupt priority control set register 21 */
#define PIC32MZ_INT_IPC21INV_OFFSET   0x029c /* Interrupt priority control invert register 21 */

#define PIC32MZ_INT_IPC22_OFFSET      0x02a0 /* Interrupt priority control register 22 */
#define PIC32MZ_INT_IPC22CLR_OFFSET   0x02a4 /* Interrupt priority control clear register 22 */
#define PIC32MZ_INT_IPC22SET_OFFSET   0x02a8 /* Interrupt priority control set register 22 */
#define PIC32MZ_INT_IPC22INV_OFFSET   0x02ac /* Interrupt priority control invert register 22 */

#define PIC32MZ_INT_IPC23_OFFSET      0x02b0 /* Interrupt priority control register 23 */
#define PIC32MZ_INT_IPC23CLR_OFFSET   0x02b4 /* Interrupt priority control clear register 23 */
#define PIC32MZ_INT_IPC23SET_OFFSET   0x02b8 /* Interrupt priority control set register 23 */
#define PIC32MZ_INT_IPC23INV_OFFSET   0x02bc /* Interrupt priority control invert register 23 */

#define PIC32MZ_INT_IPC24_OFFSET      0x02c0 /* Interrupt priority control register 24 */
#define PIC32MZ_INT_IPC24CLR_OFFSET   0x02c4 /* Interrupt priority control clear register 24 */
#define PIC32MZ_INT_IPC24SET_OFFSET   0x02c8 /* Interrupt priority control set register 24 */
#define PIC32MZ_INT_IPC24INV_OFFSET   0x02cc /* Interrupt priority control invert register 24 */

#define PIC32MZ_INT_IPC25_OFFSET      0x02d0 /* Interrupt priority control register 25 */
#define PIC32MZ_INT_IPC25CLR_OFFSET   0x02d4 /* Interrupt priority control clear register 25 */
#define PIC32MZ_INT_IPC25SET_OFFSET   0x02d8 /* Interrupt priority control set register 25 */
#define PIC32MZ_INT_IPC25INV_OFFSET   0x02dc /* Interrupt priority control invert register 25 */

#define PIC32MZ_INT_IPC26_OFFSET      0x02e0 /* Interrupt priority control register 26 */
#define PIC32MZ_INT_IPC26CLR_OFFSET   0x02e4 /* Interrupt priority control clear register 26 */
#define PIC32MZ_INT_IPC26SET_OFFSET   0x02e8 /* Interrupt priority control set register 26 */
#define PIC32MZ_INT_IPC26INV_OFFSET   0x02ec /* Interrupt priority control invert register 26 */

#define PIC32MZ_INT_IPC27_OFFSET      0x02f0 /* Interrupt priority control register 27 */
#define PIC32MZ_INT_IPC27CLR_OFFSET   0x02f4 /* Interrupt priority control clear register 27 */
#define PIC32MZ_INT_IPC27SET_OFFSET   0x02f8 /* Interrupt priority control set register 27 */
#define PIC32MZ_INT_IPC27INV_OFFSET   0x02fc /* Interrupt priority control invert register 27 */

#define PIC32MZ_INT_IPC28_OFFSET      0x0300 /* Interrupt priority control register 28 */
#define PIC32MZ_INT_IPC28CLR_OFFSET   0x0304 /* Interrupt priority control clear register 28 */
#define PIC32MZ_INT_IPC28SET_OFFSET   0x0308 /* Interrupt priority control set register 28 */
#define PIC32MZ_INT_IPC28INV_OFFSET   0x030c /* Interrupt priority control invert register 28 */

#define PIC32MZ_INT_IPC29_OFFSET      0x0310 /* Interrupt priority control register 29 */
#define PIC32MZ_INT_IPC29CLR_OFFSET   0x0314 /* Interrupt priority control clear register 29 */
#define PIC32MZ_INT_IPC29SET_OFFSET   0x0318 /* Interrupt priority control set register 29 */
#define PIC32MZ_INT_IPC29INV_OFFSET   0x031c /* Interrupt priority control invert register 29 */

#define PIC32MZ_INT_IPC30_OFFSET      0x0320 /* Interrupt priority control register 30 */
#define PIC32MZ_INT_IPC30CLR_OFFSET   0x0324 /* Interrupt priority control clear register 30 */
#define PIC32MZ_INT_IPC30SET_OFFSET   0x0328 /* Interrupt priority control set register 30 */
#define PIC32MZ_INT_IPC30INV_OFFSET   0x032c /* Interrupt priority control invert register 30 */

#define PIC32MZ_INT_IPC31_OFFSET      0x0330 /* Interrupt priority control register 31 */
#define PIC32MZ_INT_IPC31CLR_OFFSET   0x0334 /* Interrupt priority control clear register 31 */
#define PIC32MZ_INT_IPC31SET_OFFSET   0x0338 /* Interrupt priority control set register 31 */
#define PIC32MZ_INT_IPC31INV_OFFSET   0x033c /* Interrupt priority control invert register 31 */

#define PIC32MZ_INT_IPC32_OFFSET      0x0340 /* Interrupt priority control register 32 */
#define PIC32MZ_INT_IPC32CLR_OFFSET   0x0344 /* Interrupt priority control clear register 32 */
#define PIC32MZ_INT_IPC32SET_OFFSET   0x0348 /* Interrupt priority control set register 32 */
#define PIC32MZ_INT_IPC32INV_OFFSET   0x034c /* Interrupt priority control invert register 32 */

#define PIC32MZ_INT_IPC33_OFFSET      0x0350 /* Interrupt priority control register 33 */
#define PIC32MZ_INT_IPC33CLR_OFFSET   0x0354 /* Interrupt priority control clear register 33 */
#define PIC32MZ_INT_IPC33SET_OFFSET   0x0358 /* Interrupt priority control set register 33 */
#define PIC32MZ_INT_IPC33INV_OFFSET   0x035c /* Interrupt priority control invert register 33 */

#define PIC32MZ_INT_IPC34_OFFSET      0x0360 /* Interrupt priority control register 34 */
#define PIC32MZ_INT_IPC34CLR_OFFSET   0x0364 /* Interrupt priority control clear register 34 */
#define PIC32MZ_INT_IPC34SET_OFFSET   0x0368 /* Interrupt priority control set register 34 */
#define PIC32MZ_INT_IPC34INV_OFFSET   0x036c /* Interrupt priority control invert register 34 */

#define PIC32MZ_INT_IPC35_OFFSET      0x0370 /* Interrupt priority control register 35 */
#define PIC32MZ_INT_IPC35CLR_OFFSET   0x0374 /* Interrupt priority control clear register 35 */
#define PIC32MZ_INT_IPC35SET_OFFSET   0x0378 /* Interrupt priority control set register 35 */
#define PIC32MZ_INT_IPC35INV_OFFSET   0x037c /* Interrupt priority control invert register 35 */

#define PIC32MZ_INT_IPC36_OFFSET      0x0380 /* Interrupt priority control register 36 */
#define PIC32MZ_INT_IPC36CLR_OFFSET   0x0384 /* Interrupt priority control clear register 36 */
#define PIC32MZ_INT_IPC36SET_OFFSET   0x0388 /* Interrupt priority control set register 36 */
#define PIC32MZ_INT_IPC36INV_OFFSET   0x038c /* Interrupt priority control invert register 36 */

#define PIC32MZ_INT_IPC37_OFFSET      0x0390 /* Interrupt priority control register 37 */
#define PIC32MZ_INT_IPC37CLR_OFFSET   0x0394 /* Interrupt priority control clear register 37 */
#define PIC32MZ_INT_IPC37SET_OFFSET   0x0398 /* Interrupt priority control set register 37 */
#define PIC32MZ_INT_IPC37INV_OFFSET   0x039c /* Interrupt priority control invert register 37 */

#define PIC32MZ_INT_IPC38_OFFSET      0x03a0 /* Interrupt priority control register 38 */
#define PIC32MZ_INT_IPC38CLR_OFFSET   0x03a4 /* Interrupt priority control clear register 38 */
#define PIC32MZ_INT_IPC38SET_OFFSET   0x03a8 /* Interrupt priority control set register 38 */
#define PIC32MZ_INT_IPC38INV_OFFSET   0x03ac /* Interrupt priority control invert register 38 */

#define PIC32MZ_INT_IPC39_OFFSET      0x03b0 /* Interrupt priority control register 39 */
#define PIC32MZ_INT_IPC39CLR_OFFSET   0x03b4 /* Interrupt priority control clear register 39 */
#define PIC32MZ_INT_IPC39SET_OFFSET   0x03b8 /* Interrupt priority control set register 39 */
#define PIC32MZ_INT_IPC39INV_OFFSET   0x03bc /* Interrupt priority control invert register 39 */

#define PIC32MZ_INT_IPC40_OFFSET      0x03c0 /* Interrupt priority control register 40 */
#define PIC32MZ_INT_IPC40CLR_OFFSET   0x03c4 /* Interrupt priority control clear register 40 */
#define PIC32MZ_INT_IPC40SET_OFFSET   0x03c8 /* Interrupt priority control set register 40 */
#define PIC32MZ_INT_IPC40INV_OFFSET   0x03cc /* Interrupt priority control invert register 40 */

#define PIC32MZ_INT_IPC41_OFFSET      0x03d0 /* Interrupt priority control register 41 */
#define PIC32MZ_INT_IPC41CLR_OFFSET   0x03d4 /* Interrupt priority control clear register 41 */
#define PIC32MZ_INT_IPC41SET_OFFSET   0x03d8 /* Interrupt priority control set register 41 */
#define PIC32MZ_INT_IPC41INV_OFFSET   0x03dc /* Interrupt priority control invert register 41 */

#define PIC32MZ_INT_IPC42_OFFSET      0x03e0 /* Interrupt priority control register 42 */
#define PIC32MZ_INT_IPC42CLR_OFFSET   0x03e4 /* Interrupt priority control clear register 42 */
#define PIC32MZ_INT_IPC42SET_OFFSET   0x03e8 /* Interrupt priority control set register 42 */
#define PIC32MZ_INT_IPC42INV_OFFSET   0x03ec /* Interrupt priority control invert register 42 */

#define PIC32MZ_INT_IPC43_OFFSET      0x03f0 /* Interrupt priority control register 43 */
#define PIC32MZ_INT_IPC43CLR_OFFSET   0x03f4 /* Interrupt priority control clear register 43 */
#define PIC32MZ_INT_IPC43SET_OFFSET   0x03f8 /* Interrupt priority control set register 43 */
#define PIC32MZ_INT_IPC43INV_OFFSET   0x03fc /* Interrupt priority control invert register 43 */

#define PIC32MZ_INT_IPC44_OFFSET      0x0400 /* Interrupt priority control register 44 */
#define PIC32MZ_INT_IPC44CLR_OFFSET   0x0404 /* Interrupt priority control clear register 44 */
#define PIC32MZ_INT_IPC44SET_OFFSET   0x0408 /* Interrupt priority control set register 44 */
#define PIC32MZ_INT_IPC44INV_OFFSET   0x040c /* Interrupt priority control invert register 44 */

#define PIC32MZ_INT_IPC45_OFFSET      0x0410 /* Interrupt priority control register 45 */
#define PIC32MZ_INT_IPC45CLR_OFFSET   0x0414 /* Interrupt priority control clear register 45 */
#define PIC32MZ_INT_IPC45SET_OFFSET   0x0418 /* Interrupt priority control set register 45 */
#define PIC32MZ_INT_IPC45INV_OFFSET   0x041c /* Interrupt priority control invert register 45 */

#define PIC32MZ_INT_IPC46_OFFSET      0x0420 /* Interrupt priority control register 46 */
#define PIC32MZ_INT_IPC46CLR_OFFSET   0x0424 /* Interrupt priority control clear register 46 */
#define PIC32MZ_INT_IPC46SET_OFFSET   0x0428 /* Interrupt priority control set register 46 */
#define PIC32MZ_INT_IPC46INV_OFFSET   0x042c /* Interrupt priority control invert register 46 */

#define PIC32MZ_INT_IPC47_OFFSET      0x0430 /* Interrupt priority control register 47 */
#define PIC32MZ_INT_IPC47CLR_OFFSET   0x0434 /* Interrupt priority control clear register 47 */
#define PIC32MZ_INT_IPC47SET_OFFSET   0x0438 /* Interrupt priority control set register 47 */
#define PIC32MZ_INT_IPC47INV_OFFSET   0x043c /* Interrupt priority control invert register 47 */

#define PIC32MZ_INT_OFF_OFFSET(n)   (0x0540 + ((n) << 2)
#define PIC32MZ_INT_OFF000_OFFSET    0x0540  /* Interrupt vector 0 address offset */
#define PIC32MZ_INT_OFF001_OFFSET    0x0544  /* Interrupt vector 1 address offset */
#define PIC32MZ_INT_OFF002_OFFSET    0x0548  /* Interrupt vector 2 address offset */
#define PIC32MZ_INT_OFF003_OFFSET    0x054c  /* Interrupt vector 3 address offset */
#define PIC32MZ_INT_OFF004_OFFSET    0x0550  /* Interrupt vector 4 address offset */
#define PIC32MZ_INT_OFF005_OFFSET    0x0554  /* Interrupt vector 5 address offset */
#define PIC32MZ_INT_OFF006_OFFSET    0x0558  /* Interrupt vector 6 address offset */
#define PIC32MZ_INT_OFF007_OFFSET    0x055c  /* Interrupt vector 7 address offset */
#define PIC32MZ_INT_OFF008_OFFSET    0x0560  /* Interrupt vector 8 address offset */
#define PIC32MZ_INT_OFF009_OFFSET    0x0564  /* Interrupt vector 9 address offset */
#define PIC32MZ_INT_OFF010_OFFSET    0x0568  /* Interrupt vector 10 address offset */
#define PIC32MZ_INT_OFF011_OFFSET    0x056c  /* Interrupt vector 11 address offset */
#define PIC32MZ_INT_OFF012_OFFSET    0x0570  /* Interrupt vector 12 address offset */
#define PIC32MZ_INT_OFF013_OFFSET    0x0574  /* Interrupt vector 13 address offset */
#define PIC32MZ_INT_OFF014_OFFSET    0x0578  /* Interrupt vector 14 address offset */
#define PIC32MZ_INT_OFF015_OFFSET    0x057c  /* Interrupt vector 15 address offset */

#define PIC32MZ_INT_OFF016_OFFSET    0x0580  /* Interrupt vector 16 address offset */
#define PIC32MZ_INT_OFF017_OFFSET    0x0584  /* Interrupt vector 17 address offset */
#define PIC32MZ_INT_OFF018_OFFSET    0x0588  /* Interrupt vector 18 address offset */
#define PIC32MZ_INT_OFF019_OFFSET    0x058c  /* Interrupt vector 19 address offset */
#define PIC32MZ_INT_OFF020_OFFSET    0x0590  /* Interrupt vector 20 address offset */
#define PIC32MZ_INT_OFF021_OFFSET    0x0594  /* Interrupt vector 21 address offset */
#define PIC32MZ_INT_OFF022_OFFSET    0x0598  /* Interrupt vector 22 address offset */
#define PIC32MZ_INT_OFF023_OFFSET    0x059c  /* Interrupt vector 23 address offset */
#define PIC32MZ_INT_OFF024_OFFSET    0x05a0  /* Interrupt vector 24 address offset */
#define PIC32MZ_INT_OFF025_OFFSET    0x05a4  /* Interrupt vector 25 address offset */
#define PIC32MZ_INT_OFF026_OFFSET    0x05a8  /* Interrupt vector 26 address offset */
#define PIC32MZ_INT_OFF027_OFFSET    0x05ac  /* Interrupt vector 27 address offset */
#define PIC32MZ_INT_OFF028_OFFSET    0x05b0  /* Interrupt vector 28 address offset */
#define PIC32MZ_INT_OFF029_OFFSET    0x05b4  /* Interrupt vector 29 address offset */
#define PIC32MZ_INT_OFF030_OFFSET    0x05b8  /* Interrupt vector 30 address offset */
#define PIC32MZ_INT_OFF031_OFFSET    0x05bc  /* Interrupt vector 31 address offset */

#define PIC32MZ_INT_OFF032_OFFSET    0x05c0  /* Interrupt vector 32 address offset */
#define PIC32MZ_INT_OFF033_OFFSET    0x05c4  /* Interrupt vector 33 address offset */
#define PIC32MZ_INT_OFF034_OFFSET    0x05c8  /* Interrupt vector 34 address offset */
#define PIC32MZ_INT_OFF035_OFFSET    0x05cc  /* Interrupt vector 35 address offset */
#define PIC32MZ_INT_OFF036_OFFSET    0x05d0  /* Interrupt vector 36 address offset */
#define PIC32MZ_INT_OFF037_OFFSET    0x05d4  /* Interrupt vector 37 address offset */
#define PIC32MZ_INT_OFF038_OFFSET    0x05d8  /* Interrupt vector 38 address offset */
#define PIC32MZ_INT_OFF039_OFFSET    0x05dc  /* Interrupt vector 39 address offset */
#define PIC32MZ_INT_OFF040_OFFSET    0x05e0  /* Interrupt vector 40 address offset */
#define PIC32MZ_INT_OFF041_OFFSET    0x05e4  /* Interrupt vector 41 address offset */
#define PIC32MZ_INT_OFF042_OFFSET    0x05e8  /* Interrupt vector 42 address offset */
#define PIC32MZ_INT_OFF043_OFFSET    0x05ec  /* Interrupt vector 43 address offset */
#define PIC32MZ_INT_OFF044_OFFSET    0x05f0  /* Interrupt vector 44 address offset */
#define PIC32MZ_INT_OFF045_OFFSET    0x05f4  /* Interrupt vector 45 address offset */
#define PIC32MZ_INT_OFF046_OFFSET    0x05f8  /* Interrupt vector 46 address offset */
#define PIC32MZ_INT_OFF047_OFFSET    0x05fc  /* Interrupt vector 47 address offset */

#define PIC32MZ_INT_OFF048_OFFSET    0x0600  /* Interrupt vector 48 address offset */
#define PIC32MZ_INT_OFF049_OFFSET    0x0604  /* Interrupt vector 49 address offset */
#define PIC32MZ_INT_OFF050_OFFSET    0x0608  /* Interrupt vector 50 address offset */
#define PIC32MZ_INT_OFF051_OFFSET    0x060c  /* Interrupt vector 51 address offset */
#define PIC32MZ_INT_OFF052_OFFSET    0x0610  /* Interrupt vector 52 address offset */
#define PIC32MZ_INT_OFF053_OFFSET    0x0614  /* Interrupt vector 53 address offset */
#define PIC32MZ_INT_OFF054_OFFSET    0x0618  /* Interrupt vector 54 address offset */
#define PIC32MZ_INT_OFF055_OFFSET    0x061c  /* Interrupt vector 55 address offset */
#define PIC32MZ_INT_OFF056_OFFSET    0x0620  /* Interrupt vector 56 address offset */
#define PIC32MZ_INT_OFF057_OFFSET    0x0624  /* Interrupt vector 57 address offset */
#define PIC32MZ_INT_OFF058_OFFSET    0x0628  /* Interrupt vector 58 address offset */
#define PIC32MZ_INT_OFF059_OFFSET    0x062c  /* Interrupt vector 59 address offset */
#define PIC32MZ_INT_OFF060_OFFSET    0x0630  /* Interrupt vector 60 address offset */
#define PIC32MZ_INT_OFF061_OFFSET    0x0634  /* Interrupt vector 61 address offset */
#define PIC32MZ_INT_OFF062_OFFSET    0x0638  /* Interrupt vector 62 address offset */
#define PIC32MZ_INT_OFF063_OFFSET    0x063c  /* Interrupt vector 63 address offset */

#define PIC32MZ_INT_OFF064_OFFSET    0x0640  /* Interrupt vector 64 address offset */
#define PIC32MZ_INT_OFF065_OFFSET    0x0644  /* Interrupt vector 65 address offset */
#define PIC32MZ_INT_OFF066_OFFSET    0x0648  /* Interrupt vector 66 address offset */
#define PIC32MZ_INT_OFF067_OFFSET    0x064c  /* Interrupt vector 67 address offset */
#define PIC32MZ_INT_OFF068_OFFSET    0x0650  /* Interrupt vector 68 address offset */
#define PIC32MZ_INT_OFF069_OFFSET    0x0654  /* Interrupt vector 69 address offset */
#define PIC32MZ_INT_OFF070_OFFSET    0x0658  /* Interrupt vector 70 address offset */
#define PIC32MZ_INT_OFF071_OFFSET    0x065c  /* Interrupt vector 71 address offset */
#define PIC32MZ_INT_OFF072_OFFSET    0x0660  /* Interrupt vector 72 address offset */
#define PIC32MZ_INT_OFF073_OFFSET    0x0664  /* Interrupt vector 73 address offset */
#define PIC32MZ_INT_OFF074_OFFSET    0x0668  /* Interrupt vector 74 address offset */
#define PIC32MZ_INT_OFF075_OFFSET    0x066c  /* Interrupt vector 75 address offset */
#define PIC32MZ_INT_OFF076_OFFSET    0x0670  /* Interrupt vector 76 address offset */
#define PIC32MZ_INT_OFF077_OFFSET    0x0674  /* Interrupt vector 77 address offset */
#define PIC32MZ_INT_OFF078_OFFSET    0x0678  /* Interrupt vector 78 address offset */
#define PIC32MZ_INT_OFF079_OFFSET    0x067c  /* Interrupt vector 79 address offset */

#define PIC32MZ_INT_OFF080_OFFSET    0x0680  /* Interrupt vector 80 address offset */
#define PIC32MZ_INT_OFF081_OFFSET    0x0684  /* Interrupt vector 81 address offset */
#define PIC32MZ_INT_OFF082_OFFSET    0x0688  /* Interrupt vector 82 address offset */
#define PIC32MZ_INT_OFF083_OFFSET    0x068c  /* Interrupt vector 83 address offset */
#define PIC32MZ_INT_OFF084_OFFSET    0x0690  /* Interrupt vector 84 address offset */
#define PIC32MZ_INT_OFF085_OFFSET    0x0694  /* Interrupt vector 85 address offset */
#define PIC32MZ_INT_OFF086_OFFSET    0x0698  /* Interrupt vector 86 address offset */
#define PIC32MZ_INT_OFF087_OFFSET    0x069c  /* Interrupt vector 87 address offset */
#define PIC32MZ_INT_OFF088_OFFSET    0x06a0  /* Interrupt vector 88 address offset */
#define PIC32MZ_INT_OFF089_OFFSET    0x06a4  /* Interrupt vector 89 address offset */
#define PIC32MZ_INT_OFF090_OFFSET    0x06a8  /* Interrupt vector 90 address offset */
#define PIC32MZ_INT_OFF091_OFFSET    0x06ac  /* Interrupt vector 91 address offset */
#define PIC32MZ_INT_OFF092_OFFSET    0x06b0  /* Interrupt vector 92 address offset */
#define PIC32MZ_INT_OFF093_OFFSET    0x06b4  /* Interrupt vector 93 address offset */
#define PIC32MZ_INT_OFF094_OFFSET    0x06b8  /* Interrupt vector 94 address offset */
#define PIC32MZ_INT_OFF095_OFFSET    0x06bc  /* Interrupt vector 95 address offset */

#define PIC32MZ_INT_OFF096_OFFSET    0x06c0  /* Interrupt vector 96 address offset */
#define PIC32MZ_INT_OFF097_OFFSET    0x06c4  /* Interrupt vector 97 address offset */
#define PIC32MZ_INT_OFF098_OFFSET    0x06c8  /* Interrupt vector 98 address offset */
#define PIC32MZ_INT_OFF099_OFFSET    0x06cc  /* Interrupt vector 99 address offset */
#define PIC32MZ_INT_OFF100_OFFSET    0x06d0  /* Interrupt vector 100 address offset */
#define PIC32MZ_INT_OFF101_OFFSET    0x06d4  /* Interrupt vector 101 address offset */
#define PIC32MZ_INT_OFF102_OFFSET    0x06d8  /* Interrupt vector 102 address offset */
#define PIC32MZ_INT_OFF103_OFFSET    0x06dc  /* Interrupt vector 103 address offset */
#define PIC32MZ_INT_OFF104_OFFSET    0x06e0  /* Interrupt vector 104 address offset */
#define PIC32MZ_INT_OFF105_OFFSET    0x06e4  /* Interrupt vector 105 address offset */
#define PIC32MZ_INT_OFF106_OFFSET    0x06e8  /* Interrupt vector 106 address offset */
#define PIC32MZ_INT_OFF107_OFFSET    0x06ec  /* Interrupt vector 107 address offset */
#define PIC32MZ_INT_OFF108_OFFSET    0x06f0  /* Interrupt vector 108 address offset */
#define PIC32MZ_INT_OFF109_OFFSET    0x06f4  /* Interrupt vector 109 address offset */
#define PIC32MZ_INT_OFF110_OFFSET    0x06f8  /* Interrupt vector 110 address offset */
#define PIC32MZ_INT_OFF111_OFFSET    0x06fc  /* Interrupt vector 111 address offset */

#define PIC32MZ_INT_OFF112_OFFSET    0x0700  /* Interrupt vector 112 address offset */
#define PIC32MZ_INT_OFF113_OFFSET    0x0704  /* Interrupt vector 113 address offset */
#define PIC32MZ_INT_OFF114_OFFSET    0x0708  /* Interrupt vector 114 address offset */
#define PIC32MZ_INT_OFF115_OFFSET    0x070c  /* Interrupt vector 115 address offset */
#define PIC32MZ_INT_OFF116_OFFSET    0x0710  /* Interrupt vector 116 address offset */
#define PIC32MZ_INT_OFF117_OFFSET    0x0714  /* Interrupt vector 117 address offset */
#define PIC32MZ_INT_OFF118_OFFSET    0x0718  /* Interrupt vector 118 address offset */
#define PIC32MZ_INT_OFF119_OFFSET    0x071c  /* Interrupt vector 119 address offset */
#define PIC32MZ_INT_OFF120_OFFSET    0x0720  /* Interrupt vector 120 address offset */
#define PIC32MZ_INT_OFF121_OFFSET    0x0724  /* Interrupt vector 121 address offset */
#define PIC32MZ_INT_OFF122_OFFSET    0x0728  /* Interrupt vector 122 address offset */
#define PIC32MZ_INT_OFF123_OFFSET    0x072c  /* Interrupt vector 123 address offset */
#define PIC32MZ_INT_OFF124_OFFSET    0x0730  /* Interrupt vector 124 address offset */
#define PIC32MZ_INT_OFF125_OFFSET    0x0734  /* Interrupt vector 125 address offset */
#define PIC32MZ_INT_OFF126_OFFSET    0x0738  /* Interrupt vector 126 address offset */
#define PIC32MZ_INT_OFF127_OFFSET    0x073c  /* Interrupt vector 127 address offset */

#define PIC32MZ_INT_OFF128_OFFSET    0x0740  /* Interrupt vector 128 address offset */
#define PIC32MZ_INT_OFF129_OFFSET    0x0744  /* Interrupt vector 129 address offset */
#define PIC32MZ_INT_OFF130_OFFSET    0x0748  /* Interrupt vector 130 address offset */
#define PIC32MZ_INT_OFF131_OFFSET    0x074c  /* Interrupt vector 131 address offset */
#define PIC32MZ_INT_OFF132_OFFSET    0x0750  /* Interrupt vector 132 address offset */
#define PIC32MZ_INT_OFF133_OFFSET    0x0754  /* Interrupt vector 133 address offset */
#define PIC32MZ_INT_OFF134_OFFSET    0x0758  /* Interrupt vector 134 address offset */
#define PIC32MZ_INT_OFF135_OFFSET    0x075c  /* Interrupt vector 135 address offset */
#define PIC32MZ_INT_OFF136_OFFSET    0x0760  /* Interrupt vector 136 address offset */
#define PIC32MZ_INT_OFF137_OFFSET    0x0764  /* Interrupt vector 137 address offset */
#define PIC32MZ_INT_OFF138_OFFSET    0x0768  /* Interrupt vector 138 address offset */
#define PIC32MZ_INT_OFF139_OFFSET    0x076c  /* Interrupt vector 139 address offset */
#define PIC32MZ_INT_OFF140_OFFSET    0x0770  /* Interrupt vector 140 address offset */
#define PIC32MZ_INT_OFF141_OFFSET    0x0774  /* Interrupt vector 141 address offset */
#define PIC32MZ_INT_OFF142_OFFSET    0x0778  /* Interrupt vector 142 address offset */
#define PIC32MZ_INT_OFF143_OFFSET    0x077c  /* Interrupt vector 143 address offset */

#define PIC32MZ_INT_OFF144_OFFSET    0x0780  /* Interrupt vector 144 address offset */
#define PIC32MZ_INT_OFF145_OFFSET    0x0784  /* Interrupt vector 145 address offset */
#define PIC32MZ_INT_OFF146_OFFSET    0x0788  /* Interrupt vector 146 address offset */
#define PIC32MZ_INT_OFF147_OFFSET    0x078c  /* Interrupt vector 147 address offset */
#define PIC32MZ_INT_OFF148_OFFSET    0x0790  /* Interrupt vector 148 address offset */
#define PIC32MZ_INT_OFF149_OFFSET    0x0794  /* Interrupt vector 149 address offset */
#define PIC32MZ_INT_OFF150_OFFSET    0x0798  /* Interrupt vector 150 address offset */
#define PIC32MZ_INT_OFF151_OFFSET    0x079c  /* Interrupt vector 151 address offset */
#define PIC32MZ_INT_OFF152_OFFSET    0x07a0  /* Interrupt vector 152 address offset */
#define PIC32MZ_INT_OFF153_OFFSET    0x07a4  /* Interrupt vector 153 address offset */
#define PIC32MZ_INT_OFF154_OFFSET    0x07a8  /* Interrupt vector 154 address offset */
#define PIC32MZ_INT_OFF155_OFFSET    0x07ac  /* Interrupt vector 155 address offset */
#define PIC32MZ_INT_OFF156_OFFSET    0x07b0  /* Interrupt vector 156 address offset */
#define PIC32MZ_INT_OFF157_OFFSET    0x07b4  /* Interrupt vector 157 address offset */
#define PIC32MZ_INT_OFF158_OFFSET    0x07b8  /* Interrupt vector 158 address offset */
#define PIC32MZ_INT_OFF159_OFFSET    0x07bc  /* Interrupt vector 159 address offset */

#define PIC32MZ_INT_OFF160_OFFSET    0x07c0  /* Interrupt vector 160 address offset */
#define PIC32MZ_INT_OFF161_OFFSET    0x07c4  /* Interrupt vector 161 address offset */
#define PIC32MZ_INT_OFF162_OFFSET    0x07c8  /* Interrupt vector 162 address offset */
#define PIC32MZ_INT_OFF163_OFFSET    0x07cc  /* Interrupt vector 163 address offset */
#define PIC32MZ_INT_OFF164_OFFSET    0x07d0  /* Interrupt vector 164 address offset */
#define PIC32MZ_INT_OFF165_OFFSET    0x07d4  /* Interrupt vector 165 address offset */
#define PIC32MZ_INT_OFF166_OFFSET    0x07d8  /* Interrupt vector 166 address offset */
#define PIC32MZ_INT_OFF167_OFFSET    0x07dc  /* Interrupt vector 167 address offset */
#define PIC32MZ_INT_OFF168_OFFSET    0x07e0  /* Interrupt vector 168 address offset */
#define PIC32MZ_INT_OFF169_OFFSET    0x07e4  /* Interrupt vector 169 address offset */
#define PIC32MZ_INT_OFF170_OFFSET    0x07e8  /* Interrupt vector 170 address offset */
#define PIC32MZ_INT_OFF171_OFFSET    0x07ec  /* Interrupt vector 171 address offset */
#define PIC32MZ_INT_OFF172_OFFSET    0x07f0  /* Interrupt vector 172 address offset */
#define PIC32MZ_INT_OFF173_OFFSET    0x07f4  /* Interrupt vector 173 address offset */
#define PIC32MZ_INT_OFF174_OFFSET    0x07f8  /* Interrupt vector 174 address offset */
#define PIC32MZ_INT_OFF175_OFFSET    0x07fc  /* Interrupt vector 175 address offset */

#define PIC32MZ_INT_OFF176_OFFSET    0x0800  /* Interrupt vector 176 address offset */
#define PIC32MZ_INT_OFF177_OFFSET    0x0804  /* Interrupt vector 177 address offset */
#define PIC32MZ_INT_OFF178_OFFSET    0x0808  /* Interrupt vector 178 address offset */
#define PIC32MZ_INT_OFF179_OFFSET    0x080c  /* Interrupt vector 179 address offset */
#define PIC32MZ_INT_OFF180_OFFSET    0x0810  /* Interrupt vector 180 address offset */
#define PIC32MZ_INT_OFF181_OFFSET    0x0814  /* Interrupt vector 181 address offset */
#define PIC32MZ_INT_OFF182_OFFSET    0x0818  /* Interrupt vector 182 address offset */
#define PIC32MZ_INT_OFF183_OFFSET    0x081c  /* Interrupt vector 183 address offset */
#define PIC32MZ_INT_OFF184_OFFSET    0x0820  /* Interrupt vector 184 address offset */
#define PIC32MZ_INT_OFF185_OFFSET    0x0824  /* Interrupt vector 185 address offset */
#define PIC32MZ_INT_OFF186_OFFSET    0x0828  /* Interrupt vector 186 address offset */
#define PIC32MZ_INT_OFF187_OFFSET    0x082c  /* Interrupt vector 187 address offset */
#define PIC32MZ_INT_OFF188_OFFSET    0x0830  /* Interrupt vector 188 address offset */
#define PIC32MZ_INT_OFF189_OFFSET    0x0834  /* Interrupt vector 189 address offset */
#define PIC32MZ_INT_OFF190_OFFSET    0x0838  /* Interrupt vector 190 address offset */

/* Register Addresses *******************************************************/

#define PIC32MZ_INT_INTCON           (PIC32MZ_INT_K1BASE+PIC32MZ_INT_INTCON_OFFSET)
#define PIC32MZ_INT_INTCONCLR        (PIC32MZ_INT_K1BASE+PIC32MZ_INT_INTCONCLR_OFFSET)
#define PIC32MZ_INT_INTCONSET        (PIC32MZ_INT_K1BASE+PIC32MZ_INT_INTCONSET_OFFSET)
#define PIC32MZ_INT_INTCONINV        (PIC32MZ_INT_K1BASE+PIC32MZ_INT_INTCONINV_OFFSET)

#define PIC32MZ_INT_PRISS            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_PRISS_OFFSET)
#define PIC32MZ_INT_PRISSCLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_PRISSCLR_OFFSET)
#define PIC32MZ_INT_PRISSSET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_PRISSSET_OFFSET)
#define PIC32MZ_INT_PRISSINV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_PRISSINV_OFFSET)

#define PIC32MZ_INT_INTSTAT          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_INTSTAT_OFFSET)
#define PIC32MZ_INT_INTSTATCLR       (PIC32MZ_INT_K1BASE+PIC32MZ_INT_INTSTATCLR_OFFSET)
#define PIC32MZ_INT_INTSTATSET       (PIC32MZ_INT_K1BASE+PIC32MZ_INT_INTSTATSET_OFFSET)
#define PIC32MZ_INT_INTSTATINV       (PIC32MZ_INT_K1BASE+PIC32MZ_INT_INTSTATINV_OFFSET)

#define PIC32MZ_INT_IPTMR            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPTMR_OFFSET)
#define PIC32MZ_INT_IPTMRCLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPTMRCLR_OFFSET)
#define PIC32MZ_INT_IPTMRSET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPTMRSET_OFFSET)
#define PIC32MZ_INT_IPTMRINV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPTMRINV_OFFSET)

#define PIC32MZ_INT_IFS(n)           (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IFS_OFFSET(n))
#define PIC32MZ_INT_IFSCLR(n)        (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IFSCLR_OFFSET(n))
#define PIC32MZ_INT_IFSSET(n)        (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IFSSET_OFFSET(n))
#define PIC32MZ_INT_IFSINV(n)        (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IFSINV_OFFSET(n))

#define PIC32MZ_INT_IFS0             (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IFS0_OFFSET)
#define PIC32MZ_INT_IFS0CLR          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IFS0CLR_OFFSET)
#define PIC32MZ_INT_IFS0SET          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IFS0SET_OFFSET)
#define PIC32MZ_INT_IFS0INV          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IFS0INV_OFFSET)

#define PIC32MZ_INT_IFS1             (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IFS1_OFFSET)
#define PIC32MZ_INT_IFS1CLR          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IFS1CLR_OFFSET)
#define PIC32MZ_INT_IFS1SET          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IFS1SET_OFFSET)
#define PIC32MZ_INT_IFS1INV          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IFS1INV_OFFSET)

#define PIC32MZ_INT_IFS2             (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IFS2_OFFSET)
#define PIC32MZ_INT_IFS2CLR          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IFS2CLR_OFFSET)
#define PIC32MZ_INT_IFS2SET          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IFS2SET_OFFSET)
#define PIC32MZ_INT_IFS2INV          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IFS2INV_OFFSET)

#define PIC32MZ_INT_IFS3             (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IFS3_OFFSET)
#define PIC32MZ_INT_IFS3CLR          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IFS3CLR_OFFSET)
#define PIC32MZ_INT_IFS3SET          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IFS3SET_OFFSET)
#define PIC32MZ_INT_IFS3INV          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IFS3INV_OFFSET)

#define PIC32MZ_INT_IFS4             (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IFS4_OFFSET)
#define PIC32MZ_INT_IFS4CLR          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IFS4CLR_OFFSET)
#define PIC32MZ_INT_IFS4SET          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IFS4SET_OFFSET)
#define PIC32MZ_INT_IFS4INV          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IFS4INV_OFFSET)

#define PIC32MZ_INT_IFS5             (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IFS5_OFFSET)
#define PIC32MZ_INT_IFS5CLR          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IFS5CLR_OFFSET)
#define PIC32MZ_INT_IFS5SET          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IFS5SET_OFFSET)
#define PIC32MZ_INT_IFS5INV          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IFS5INV_OFFSET)

#define PIC32MZ_INT_IEC(n)           (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IEC_OFFSET(n))
#define PIC32MZ_INT_IECCLR(n)        (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IECCLR_OFFSET(n))
#define PIC32MZ_INT_IECSET(n)        (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IECSET_OFFSET(n))
#define PIC32MZ_INT_IECINV(n)        (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IECINV_OFFSET(n))

#define PIC32MZ_INT_IEC0             (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IEC0_OFFSET)
#define PIC32MZ_INT_IEC0CLR          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IEC0CLR_OFFSET)
#define PIC32MZ_INT_IEC0SET          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IEC0SET_OFFSET)
#define PIC32MZ_INT_IEC0INV          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IEC0_OFFSET)

#define PIC32MZ_INT_IEC1             (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IEC1_OFFSET)
#define PIC32MZ_INT_IEC1CLR          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IEC1CLR_OFFSET)
#define PIC32MZ_INT_IEC1SET          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IEC1SET_OFFSET)
#define PIC32MZ_INT_IEC1INV          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IEC1INV_OFFSET)

#define PIC32MZ_INT_IEC2             (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IEC2_OFFSET)
#define PIC32MZ_INT_IEC2CLR          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IEC2CLR_OFFSET)
#define PIC32MZ_INT_IEC2SET          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IEC2SET_OFFSET)
#define PIC32MZ_INT_IEC2INV          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IEC2INV_OFFSET)

#define PIC32MZ_INT_IEC3             (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IEC3_OFFSET)
#define PIC32MZ_INT_IEC3CLR          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IEC3CLR_OFFSET)
#define PIC32MZ_INT_IEC3SET          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IEC3SET_OFFSET)
#define PIC32MZ_INT_IEC3INV          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IEC3INV_OFFSET)

#define PIC32MZ_INT_IEC4             (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IEC4_OFFSET)
#define PIC32MZ_INT_IEC4CLR          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IEC4CLR_OFFSET)
#define PIC32MZ_INT_IEC4SET          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IEC4SET_OFFSET)
#define PIC32MZ_INT_IEC4INV          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IEC4INV_OFFSET)

#define PIC32MZ_INT_IEC5             (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IEC5_OFFSET)
#define PIC32MZ_INT_IEC5CLR          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IEC5CLR_OFFSET)
#define PIC32MZ_INT_IEC5SET          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IEC5SET_OFFSET)
#define PIC32MZ_INT_IEC5INV          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IEC5INV_OFFSET)

#define PIC32MZ_INT_IPC(n)           (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC_OFFSET(n))
#define PIC32MZ_INT_IPCCLR(n)        (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPCCLR_OFFSET(n))
#define PIC32MZ_INT_IPCSET(n)        (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPCSET_OFFSET(n))
#define PIC32MZ_INT_IPCINV(n)        (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPCINV_OFFSET(n))

#define PIC32MZ_INT_IPC0             (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC0_OFFSET)
#define PIC32MZ_INT_IPC0CLR          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC0CLR_OFFSET)
#define PIC32MZ_INT_IPC0SET          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC0SET_OFFSET)
#define PIC32MZ_INT_IPC0INV          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC0INV_OFFSET)

#define PIC32MZ_INT_IPC1             (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC1_OFFSET)
#define PIC32MZ_INT_IPC1CLR          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC1CLR_OFFSET)
#define PIC32MZ_INT_IPC1SET          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC1SET_OFFSET)
#define PIC32MZ_INT_IPC1INV          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC1INV_OFFSET)

#define PIC32MZ_INT_IPC2             (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC2_OFFSET)
#define PIC32MZ_INT_IPC2CLR          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC2CLR_OFFSET)
#define PIC32MZ_INT_IPC2SET          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC2SET_OFFSET)
#define PIC32MZ_INT_IPC2INV          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC2INV_OFFSET)

#define PIC32MZ_INT_IPC3             (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC3_OFFSET)
#define PIC32MZ_INT_IPC3CLR          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC3CLR_OFFSET)
#define PIC32MZ_INT_IPC3SET          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC3SET_OFFSET)
#define PIC32MZ_INT_IPC3INV          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC3INV_OFFSET)

#define PIC32MZ_INT_IPC4             (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC4_OFFSET)
#define PIC32MZ_INT_IPC4CLR          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC4CLR_OFFSET)
#define PIC32MZ_INT_IPC4SET          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC4SET_OFFSET)
#define PIC32MZ_INT_IPC4INV          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC4INV_OFFSET)

#define PIC32MZ_INT_IPC5             (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC5_OFFSET)
#define PIC32MZ_INT_IPC5CLR          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC5CLR_OFFSET)
#define PIC32MZ_INT_IPC5SET          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC5SET_OFFSET)
#define PIC32MZ_INT_IPC5INV          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC5INV_OFFSET)

#define PIC32MZ_INT_IPC6             (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC6_OFFSET)
#define PIC32MZ_INT_IPC6CLR          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC6CLR_OFFSET)
#define PIC32MZ_INT_IPC6SET          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC6SET_OFFSET)
#define PIC32MZ_INT_IPC6INV          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC6INV_OFFSET)

#define PIC32MZ_INT_IPC7             (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC7_OFFSET)
#define PIC32MZ_INT_IPC7CLR          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC7CLR_OFFSET)
#define PIC32MZ_INT_IPC7SET          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC7SET_OFFSET)
#define PIC32MZ_INT_IPC7INV          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC7INV_OFFSET)

#define PIC32MZ_INT_IPC8             (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC8_OFFSET)
#define PIC32MZ_INT_IPC8CLR          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC8CLR_OFFSET)
#define PIC32MZ_INT_IPC8SET          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC8SET_OFFSET)
#define PIC32MZ_INT_IPC8INV          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC8INV_OFFSET)

#define PIC32MZ_INT_IPC9             (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC9_OFFSET)
#define PIC32MZ_INT_IPC9CLR          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC9CLR_OFFSET)
#define PIC32MZ_INT_IPC9SET          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC9SET_OFFSET)
#define PIC32MZ_INT_IPC9INV          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC9INV_OFFSET)

#define PIC32MZ_INT_IPC10            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC10_OFFSET)
#define PIC32MZ_INT_IPC10CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC10CLR_OFFSET)
#define PIC32MZ_INT_IPC10SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC10SET_OFFSET)
#define PIC32MZ_INT_IPC10INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC10INV_OFFSET)

#define PIC32MZ_INT_IPC11            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC11_OFFSET)
#define PIC32MZ_INT_IPC11CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC11CLR_OFFSET)
#define PIC32MZ_INT_IPC11SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC11SET_OFFSET)
#define PIC32MZ_INT_IPC11INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC11INV_OFFSET)

#define PIC32MZ_INT_IPC12            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC12_OFFSET)
#define PIC32MZ_INT_IPC12CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC12CLR_OFFSET)
#define PIC32MZ_INT_IPC12SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC12SET_OFFSET)
#define PIC32MZ_INT_IPC12INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC12INV_OFFSET)

#define PIC32MZ_INT_IPC13            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC13_OFFSET)
#define PIC32MZ_INT_IPC13CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC13CLR_OFFSET)
#define PIC32MZ_INT_IPC13SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC13SET_OFFSET)
#define PIC32MZ_INT_IPC13INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC13INV_OFFSET)

#define PIC32MZ_INT_IPC14            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC14_OFFSET)
#define PIC32MZ_INT_IPC14CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC14CLR_OFFSET)
#define PIC32MZ_INT_IPC14SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC14SET_OFFSET)
#define PIC32MZ_INT_IPC14INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC14INV_OFFSET)

#define PIC32MZ_INT_IPC15            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC15_OFFSET)
#define PIC32MZ_INT_IPC15CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC15CLR_OFFSET)
#define PIC32MZ_INT_IPC15SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC15SET_OFFSET)
#define PIC32MZ_INT_IPC15INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC15INV_OFFSET)

#define PIC32MZ_INT_IPC16            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC16_OFFSET)
#define PIC32MZ_INT_IPC16CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC16CLR_OFFSET)
#define PIC32MZ_INT_IPC16SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC16SET_OFFSET)
#define PIC32MZ_INT_IPC16INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC16INV_OFFSET)

#define PIC32MZ_INT_IPC17            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC17_OFFSET)
#define PIC32MZ_INT_IPC17CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC17CLR_OFFSET)
#define PIC32MZ_INT_IPC17SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC17SET_OFFSET)
#define PIC32MZ_INT_IPC17INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC17INV_OFFSET)

#define PIC32MZ_INT_IPC18            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC18_OFFSET)
#define PIC32MZ_INT_IPC18CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC18CLR_OFFSET)
#define PIC32MZ_INT_IPC18SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC18SET_OFFSET)
#define PIC32MZ_INT_IPC18INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC18INV_OFFSET)

#define PIC32MZ_INT_IPC19            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC19_OFFSET)
#define PIC32MZ_INT_IPC19CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC19CLR_OFFSET)
#define PIC32MZ_INT_IPC19SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC19SET_OFFSET)
#define PIC32MZ_INT_IPC19INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC19INV_OFFSET)

#define PIC32MZ_INT_IPC20            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC20_OFFSET)
#define PIC32MZ_INT_IPC20CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC20CLR_OFFSET)
#define PIC32MZ_INT_IPC20SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC20SET_OFFSET)
#define PIC32MZ_INT_IPC20INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC20INV_OFFSET)

#define PIC32MZ_INT_IPC21            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC21_OFFSET)
#define PIC32MZ_INT_IPC21CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC21CLR_OFFSET)
#define PIC32MZ_INT_IPC21SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC21SET_OFFSET)
#define PIC32MZ_INT_IPC21INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC21INV_OFFSET)

#define PIC32MZ_INT_IPC22            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC22_OFFSET)
#define PIC32MZ_INT_IPC22CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC22CLR_OFFSET)
#define PIC32MZ_INT_IPC22SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC22SET_OFFSET)
#define PIC32MZ_INT_IPC22INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC22INV_OFFSET)

#define PIC32MZ_INT_IPC23            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC23_OFFSET)
#define PIC32MZ_INT_IPC23CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC23CLR_OFFSET)
#define PIC32MZ_INT_IPC23SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC23SET_OFFSET)
#define PIC32MZ_INT_IPC23INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC23INV_OFFSET)

#define PIC32MZ_INT_IPC24            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC24_OFFSET)
#define PIC32MZ_INT_IPC24CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC24CLR_OFFSET)
#define PIC32MZ_INT_IPC24SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC24SET_OFFSET)
#define PIC32MZ_INT_IPC24INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC24INV_OFFSET)

#define PIC32MZ_INT_IPC25            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC25_OFFSET)
#define PIC32MZ_INT_IPC25CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC25CLR_OFFSET)
#define PIC32MZ_INT_IPC25SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC25SET_OFFSET)
#define PIC32MZ_INT_IPC25INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC25INV_OFFSET)

#define PIC32MZ_INT_IPC26            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC26_OFFSET)
#define PIC32MZ_INT_IPC26CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC26CLR_OFFSET)
#define PIC32MZ_INT_IPC26SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC26SET_OFFSET)
#define PIC32MZ_INT_IPC26INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC26INV_OFFSET)

#define PIC32MZ_INT_IPC27            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC27_OFFSET)
#define PIC32MZ_INT_IPC27CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC27CLR_OFFSET)
#define PIC32MZ_INT_IPC27SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC27SET_OFFSET)
#define PIC32MZ_INT_IPC27INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC27INV_OFFSET)

#define PIC32MZ_INT_IPC28            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC28_OFFSET)
#define PIC32MZ_INT_IPC28CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC28CLR_OFFSET)
#define PIC32MZ_INT_IPC28SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC28SET_OFFSET)
#define PIC32MZ_INT_IPC28INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC28INV_OFFSET)

#define PIC32MZ_INT_IPC29            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC29_OFFSET)
#define PIC32MZ_INT_IPC29CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC29CLR_OFFSET)
#define PIC32MZ_INT_IPC29SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC29SET_OFFSET)
#define PIC32MZ_INT_IPC29INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC29INV_OFFSET)

#define PIC32MZ_INT_IPC30            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC30_OFFSET)
#define PIC32MZ_INT_IPC30CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC30CLR_OFFSET)
#define PIC32MZ_INT_IPC30SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC30SET_OFFSET)
#define PIC32MZ_INT_IPC30INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC30INV_OFFSET)

#define PIC32MZ_INT_IPC31            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC31_OFFSET)
#define PIC32MZ_INT_IPC31CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC31CLR_OFFSET)
#define PIC32MZ_INT_IPC31SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC31SET_OFFSET)
#define PIC32MZ_INT_IPC31INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC31INV_OFFSET)

#define PIC32MZ_INT_IPC32            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC32_OFFSET)
#define PIC32MZ_INT_IPC32CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC32CLR_OFFSET)
#define PIC32MZ_INT_IPC32SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC32SET_OFFSET)
#define PIC32MZ_INT_IPC32INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC32INV_OFFSET)

#define PIC32MZ_INT_IPC33            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC33_OFFSET)
#define PIC32MZ_INT_IPC33CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC33CLR_OFFSET)
#define PIC32MZ_INT_IPC33SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC33SET_OFFSET)
#define PIC32MZ_INT_IPC33INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC33INV_OFFSET)

#define PIC32MZ_INT_IPC34            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC34_OFFSET)
#define PIC32MZ_INT_IPC34CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC34CLR_OFFSET)
#define PIC32MZ_INT_IPC34SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC34SET_OFFSET)
#define PIC32MZ_INT_IPC34INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC34INV_OFFSET)

#define PIC32MZ_INT_IPC35            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC35_OFFSET)
#define PIC32MZ_INT_IPC35CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC35CLR_OFFSET)
#define PIC32MZ_INT_IPC35SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC35SET_OFFSET)
#define PIC32MZ_INT_IPC35INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC35INV_OFFSET)

#define PIC32MZ_INT_IPC36            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC36_OFFSET)
#define PIC32MZ_INT_IPC36CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC36CLR_OFFSET)
#define PIC32MZ_INT_IPC36SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC36SET_OFFSET)
#define PIC32MZ_INT_IPC36INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC36INV_OFFSET)

#define PIC32MZ_INT_IPC37            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC37_OFFSET)
#define PIC32MZ_INT_IPC37CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC37CLR_OFFSET)
#define PIC32MZ_INT_IPC37SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC37SET_OFFSET)
#define PIC32MZ_INT_IPC37INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC37INV_OFFSET)

#define PIC32MZ_INT_IPC38            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC38_OFFSET)
#define PIC32MZ_INT_IPC38CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC38CLR_OFFSET)
#define PIC32MZ_INT_IPC38SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC38SET_OFFSET)
#define PIC32MZ_INT_IPC38INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC38INV_OFFSET)

#define PIC32MZ_INT_IPC39            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC39_OFFSET)
#define PIC32MZ_INT_IPC39CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC39CLR_OFFSET)
#define PIC32MZ_INT_IPC39SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC39SET_OFFSET)
#define PIC32MZ_INT_IPC39INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC39INV_OFFSET)

#define PIC32MZ_INT_IPC40            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC40_OFFSET)
#define PIC32MZ_INT_IPC40CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC40CLR_OFFSET)
#define PIC32MZ_INT_IPC40SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC40SET_OFFSET)
#define PIC32MZ_INT_IPC40INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC40INV_OFFSET)

#define PIC32MZ_INT_IPC41            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC41_OFFSET)
#define PIC32MZ_INT_IPC41CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC41CLR_OFFSET)
#define PIC32MZ_INT_IPC41SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC41SET_OFFSET)
#define PIC32MZ_INT_IPC41INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC41INV_OFFSET)

#define PIC32MZ_INT_IPC42            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC42_OFFSET)
#define PIC32MZ_INT_IPC42CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC42CLR_OFFSET)
#define PIC32MZ_INT_IPC42SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC42SET_OFFSET)
#define PIC32MZ_INT_IPC42INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC42INV_OFFSET)

#define PIC32MZ_INT_IPC43            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC43_OFFSET)
#define PIC32MZ_INT_IPC43CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC43CLR_OFFSET)
#define PIC32MZ_INT_IPC43SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC43SET_OFFSET)
#define PIC32MZ_INT_IPC43INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC43INV_OFFSET)

#define PIC32MZ_INT_IPC44            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC44_OFFSET)
#define PIC32MZ_INT_IPC44CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC44CLR_OFFSET)
#define PIC32MZ_INT_IPC44SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC44SET_OFFSET)
#define PIC32MZ_INT_IPC44INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC44INV_OFFSET)

#define PIC32MZ_INT_IPC45            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC45_OFFSET)
#define PIC32MZ_INT_IPC45CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC45CLR_OFFSET)
#define PIC32MZ_INT_IPC45SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC45SET_OFFSET)
#define PIC32MZ_INT_IPC45INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC45INV_OFFSET)

#define PIC32MZ_INT_IPC46            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC46_OFFSET)
#define PIC32MZ_INT_IPC46CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC46CLR_OFFSET)
#define PIC32MZ_INT_IPC46SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC46SET_OFFSET)
#define PIC32MZ_INT_IPC46INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC46INV_OFFSET)

#define PIC32MZ_INT_IPC47            (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC47_OFFSET)
#define PIC32MZ_INT_IPC47CLR         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC47CLR_OFFSET)
#define PIC32MZ_INT_IPC47SET         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC47SET_OFFSET)
#define PIC32MZ_INT_IPC47INV         (PIC32MZ_INT_K1BASE+PIC32MZ_INT_IPC47INV_OFFSET)

#define PIC32MZ_INT_OFF(n)          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF_OFFSET(n)
#define PIC32MZ_INT_OFF000          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF000_OFFSET)
#define PIC32MZ_INT_OFF001          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF001_OFFSET)
#define PIC32MZ_INT_OFF002          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF002_OFFSET)
#define PIC32MZ_INT_OFF003          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF003_OFFSET)
#define PIC32MZ_INT_OFF004          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF004_OFFSET)
#define PIC32MZ_INT_OFF005          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF005_OFFSET)
#define PIC32MZ_INT_OFF006          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF006_OFFSET)
#define PIC32MZ_INT_OFF007          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF007_OFFSET)
#define PIC32MZ_INT_OFF008          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF008_OFFSET)
#define PIC32MZ_INT_OFF009          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF009_OFFSET)
#define PIC32MZ_INT_OFF010          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF010_OFFSET)
#define PIC32MZ_INT_OFF011          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF011_OFFSET)
#define PIC32MZ_INT_OFF012          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF012_OFFSET)
#define PIC32MZ_INT_OFF013          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF013_OFFSET)
#define PIC32MZ_INT_OFF014          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF014_OFFSET)
#define PIC32MZ_INT_OFF015          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF015_OFFSET)

#define PIC32MZ_INT_OFF016          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF016_OFFSET)
#define PIC32MZ_INT_OFF017          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF017_OFFSET)
#define PIC32MZ_INT_OFF018          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF018_OFFSET)
#define PIC32MZ_INT_OFF019          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF019_OFFSET)
#define PIC32MZ_INT_OFF020          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF020_OFFSET)
#define PIC32MZ_INT_OFF021          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF021_OFFSET)
#define PIC32MZ_INT_OFF022          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF022_OFFSET)
#define PIC32MZ_INT_OFF023          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF023_OFFSET)
#define PIC32MZ_INT_OFF024          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF024_OFFSET)
#define PIC32MZ_INT_OFF025          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF025_OFFSET)
#define PIC32MZ_INT_OFF026          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF026_OFFSET)
#define PIC32MZ_INT_OFF027          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF027_OFFSET)
#define PIC32MZ_INT_OFF028          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF028_OFFSET)
#define PIC32MZ_INT_OFF029          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF029_OFFSET)
#define PIC32MZ_INT_OFF030          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF030_OFFSET)
#define PIC32MZ_INT_OFF031          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF031_OFFSET)

#define PIC32MZ_INT_OFF032          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF032_OFFSET)
#define PIC32MZ_INT_OFF033          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF033_OFFSET)
#define PIC32MZ_INT_OFF034          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF034_OFFSET)
#define PIC32MZ_INT_OFF035          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF035_OFFSET)
#define PIC32MZ_INT_OFF036          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF036_OFFSET)
#define PIC32MZ_INT_OFF037          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF037_OFFSET)
#define PIC32MZ_INT_OFF038          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF038_OFFSET)
#define PIC32MZ_INT_OFF039          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF039_OFFSET)
#define PIC32MZ_INT_OFF040          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF040_OFFSET)
#define PIC32MZ_INT_OFF041          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF041_OFFSET)
#define PIC32MZ_INT_OFF042          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF042_OFFSET)
#define PIC32MZ_INT_OFF043          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF043_OFFSET)
#define PIC32MZ_INT_OFF044          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF044_OFFSET)
#define PIC32MZ_INT_OFF045          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF045_OFFSET)
#define PIC32MZ_INT_OFF046          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF046_OFFSET)
#define PIC32MZ_INT_OFF047          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF047_OFFSET)

#define PIC32MZ_INT_OFF048          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF048_OFFSET)
#define PIC32MZ_INT_OFF049          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF049_OFFSET)
#define PIC32MZ_INT_OFF050          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF050_OFFSET)
#define PIC32MZ_INT_OFF051          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF051_OFFSET)
#define PIC32MZ_INT_OFF052          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF052_OFFSET)
#define PIC32MZ_INT_OFF053          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF053_OFFSET)
#define PIC32MZ_INT_OFF054          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF054_OFFSET)
#define PIC32MZ_INT_OFF055          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF055_OFFSET)
#define PIC32MZ_INT_OFF056          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF056_OFFSET)
#define PIC32MZ_INT_OFF057          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF057_OFFSET)
#define PIC32MZ_INT_OFF058          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF058_OFFSET)
#define PIC32MZ_INT_OFF059          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF059_OFFSET)
#define PIC32MZ_INT_OFF060          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF060_OFFSET)
#define PIC32MZ_INT_OFF061          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF061_OFFSET)
#define PIC32MZ_INT_OFF062          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF062_OFFSET)
#define PIC32MZ_INT_OFF063          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF063_OFFSET)

#define PIC32MZ_INT_OFF064          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF064_OFFSET)
#define PIC32MZ_INT_OFF065          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF065_OFFSET)
#define PIC32MZ_INT_OFF066          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF066_OFFSET)
#define PIC32MZ_INT_OFF067          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF067_OFFSET)
#define PIC32MZ_INT_OFF068          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF068_OFFSET)
#define PIC32MZ_INT_OFF069          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF069_OFFSET)
#define PIC32MZ_INT_OFF070          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF070_OFFSET)
#define PIC32MZ_INT_OFF071          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF071_OFFSET)
#define PIC32MZ_INT_OFF072          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF072_OFFSET)
#define PIC32MZ_INT_OFF073          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF073_OFFSET)
#define PIC32MZ_INT_OFF074          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF074_OFFSET)
#define PIC32MZ_INT_OFF075          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF075_OFFSET)
#define PIC32MZ_INT_OFF076          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF076_OFFSET)
#define PIC32MZ_INT_OFF077          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF077_OFFSET)
#define PIC32MZ_INT_OFF078          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF078_OFFSET)
#define PIC32MZ_INT_OFF079          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF079_OFFSET)

#define PIC32MZ_INT_OFF080          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF080_OFFSET)
#define PIC32MZ_INT_OFF081          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF081_OFFSET)
#define PIC32MZ_INT_OFF082          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF082_OFFSET)
#define PIC32MZ_INT_OFF083          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF083_OFFSET)
#define PIC32MZ_INT_OFF084          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF084_OFFSET)
#define PIC32MZ_INT_OFF085          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF085_OFFSET)
#define PIC32MZ_INT_OFF086          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF086_OFFSET)
#define PIC32MZ_INT_OFF087          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF087_OFFSET)
#define PIC32MZ_INT_OFF088          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF088_OFFSET)
#define PIC32MZ_INT_OFF089          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF089_OFFSET)
#define PIC32MZ_INT_OFF090          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF090_OFFSET)
#define PIC32MZ_INT_OFF091          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF091_OFFSET)
#define PIC32MZ_INT_OFF092          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF092_OFFSET)
#define PIC32MZ_INT_OFF093          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF093_OFFSET)
#define PIC32MZ_INT_OFF094          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF094_OFFSET)
#define PIC32MZ_INT_OFF095          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF095_OFFSET)

#define PIC32MZ_INT_OFF096          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF096_OFFSET)
#define PIC32MZ_INT_OFF097          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF097_OFFSET)
#define PIC32MZ_INT_OFF098          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF098_OFFSET)
#define PIC32MZ_INT_OFF099          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF099_OFFSET)
#define PIC32MZ_INT_OFF100          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF100_OFFSET)
#define PIC32MZ_INT_OFF101          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF101_OFFSET)
#define PIC32MZ_INT_OFF102          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF102_OFFSET)
#define PIC32MZ_INT_OFF103          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF103_OFFSET)
#define PIC32MZ_INT_OFF104          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF104_OFFSET)
#define PIC32MZ_INT_OFF105          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF105_OFFSET)
#define PIC32MZ_INT_OFF106          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF106_OFFSET)
#define PIC32MZ_INT_OFF107          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF107_OFFSET)
#define PIC32MZ_INT_OFF108          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF108_OFFSET)
#define PIC32MZ_INT_OFF109          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF109_OFFSET)
#define PIC32MZ_INT_OFF110          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF110_OFFSET)
#define PIC32MZ_INT_OFF111          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF111_OFFSET)

#define PIC32MZ_INT_OFF112          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF112_OFFSET)
#define PIC32MZ_INT_OFF113          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF113_OFFSET)
#define PIC32MZ_INT_OFF114          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF114_OFFSET)
#define PIC32MZ_INT_OFF115          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF115_OFFSET)
#define PIC32MZ_INT_OFF116          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF116_OFFSET)
#define PIC32MZ_INT_OFF117          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF117_OFFSET)
#define PIC32MZ_INT_OFF118          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF118_OFFSET)
#define PIC32MZ_INT_OFF119          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF119_OFFSET)
#define PIC32MZ_INT_OFF120          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF120_OFFSET)
#define PIC32MZ_INT_OFF121          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF121_OFFSET)
#define PIC32MZ_INT_OFF122          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF122_OFFSET)
#define PIC32MZ_INT_OFF123          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF123_OFFSET)
#define PIC32MZ_INT_OFF124          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF124_OFFSET)
#define PIC32MZ_INT_OFF125          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF125_OFFSET)
#define PIC32MZ_INT_OFF126          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF126_OFFSET)
#define PIC32MZ_INT_OFF127          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF127_OFFSET)

#define PIC32MZ_INT_OFF128          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF128_OFFSET)
#define PIC32MZ_INT_OFF129          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF129_OFFSET)
#define PIC32MZ_INT_OFF130          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF130_OFFSET)
#define PIC32MZ_INT_OFF131          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF131_OFFSET)
#define PIC32MZ_INT_OFF132          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF132_OFFSET)
#define PIC32MZ_INT_OFF133          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF133_OFFSET)
#define PIC32MZ_INT_OFF134          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF134_OFFSET)
#define PIC32MZ_INT_OFF135          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF135_OFFSET)
#define PIC32MZ_INT_OFF136          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF136_OFFSET)
#define PIC32MZ_INT_OFF137          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF137_OFFSET)
#define PIC32MZ_INT_OFF138          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF138_OFFSET)
#define PIC32MZ_INT_OFF139          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF139_OFFSET)
#define PIC32MZ_INT_OFF140          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF140_OFFSET)
#define PIC32MZ_INT_OFF141          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF141_OFFSET)
#define PIC32MZ_INT_OFF142          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF142_OFFSET)
#define PIC32MZ_INT_OFF143          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF143_OFFSET)

#define PIC32MZ_INT_OFF144          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF144_OFFSET)
#define PIC32MZ_INT_OFF145          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF145_OFFSET)
#define PIC32MZ_INT_OFF146          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF146_OFFSET)
#define PIC32MZ_INT_OFF147          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF147_OFFSET)
#define PIC32MZ_INT_OFF148          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF148_OFFSET)
#define PIC32MZ_INT_OFF149          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF149_OFFSET)
#define PIC32MZ_INT_OFF150          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF150_OFFSET)
#define PIC32MZ_INT_OFF151          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF151_OFFSET)
#define PIC32MZ_INT_OFF152          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF152_OFFSET)
#define PIC32MZ_INT_OFF153          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF153_OFFSET)
#define PIC32MZ_INT_OFF154          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF154_OFFSET)
#define PIC32MZ_INT_OFF155          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF155_OFFSET)
#define PIC32MZ_INT_OFF156          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF156_OFFSET)
#define PIC32MZ_INT_OFF157          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF157_OFFSET)
#define PIC32MZ_INT_OFF158          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF158_OFFSET)
#define PIC32MZ_INT_OFF159          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF159_OFFSET)

#define PIC32MZ_INT_OFF160          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF160_OFFSET)
#define PIC32MZ_INT_OFF161          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF161_OFFSET)
#define PIC32MZ_INT_OFF162          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF162_OFFSET)
#define PIC32MZ_INT_OFF163          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF163_OFFSET)
#define PIC32MZ_INT_OFF164          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF164_OFFSET)
#define PIC32MZ_INT_OFF165          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF165_OFFSET)
#define PIC32MZ_INT_OFF166          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF166_OFFSET)
#define PIC32MZ_INT_OFF167          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF167_OFFSET)
#define PIC32MZ_INT_OFF168          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF168_OFFSET)
#define PIC32MZ_INT_OFF169          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF169_OFFSET)
#define PIC32MZ_INT_OFF170          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF170_OFFSET)
#define PIC32MZ_INT_OFF171          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF171_OFFSET)
#define PIC32MZ_INT_OFF172          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF172_OFFSET)
#define PIC32MZ_INT_OFF173          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF173_OFFSET)
#define PIC32MZ_INT_OFF174          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF174_OFFSET)
#define PIC32MZ_INT_OFF175          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF175_OFFSET)

#define PIC32MZ_INT_OFF176          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF176_OFFSET)
#define PIC32MZ_INT_OFF177          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF177_OFFSET)
#define PIC32MZ_INT_OFF178          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF178_OFFSET)
#define PIC32MZ_INT_OFF179          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF179_OFFSET)
#define PIC32MZ_INT_OFF180          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF180_OFFSET)
#define PIC32MZ_INT_OFF181          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF181_OFFSET)
#define PIC32MZ_INT_OFF182          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF182_OFFSET)
#define PIC32MZ_INT_OFF183          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF183_OFFSET)
#define PIC32MZ_INT_OFF184          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF184_OFFSET)
#define PIC32MZ_INT_OFF185          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF185_OFFSET)
#define PIC32MZ_INT_OFF186          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF186_OFFSET)
#define PIC32MZ_INT_OFF187          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF187_OFFSET)
#define PIC32MZ_INT_OFF188          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF188_OFFSET)
#define PIC32MZ_INT_OFF189          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF189_OFFSET)
#define PIC32MZ_INT_OFF190          (PIC32MZ_INT_K1BASE+PIC32MZ_INT_OFF190_OFFSET)

/* Register Bit-Field Definitions *******************************************/

/* Interrupt control register */

#define INT_INTCON_INT0EP            (1 << 0)  /* Bit 0:  External interrupt 0 edge polarity control */
#define INT_INTCON_INT1EP            (1 << 1)  /* Bit 1:  External interrupt 1 edge polarity control */
#define INT_INTCON_INT2EP            (1 << 2)  /* Bit 2:  External interrupt 2 edge polarity control */
#define INT_INTCON_INT3EP            (1 << 3)  /* Bit 3:  External interrupt 3 edge polarity control */
#define INT_INTCON_INT4EP            (1 << 4)  /* Bit 4:  External interrupt 4 edge polarity control */
#define INT_INTCON_TPC_SHIFT         (8)       /* Bits 8-10: Interrupt proximity timer control */
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

/* Priority shadow select register */

#define INT_PRISS_SS0                (1 << 0)  /* Bit 0: Single Vector Shadow Register Set bit */
#define INT_PRISS_PRI1SS_SHIFT       (4)       /* Bits 4-7: Interrupt with Priority Level 1 Shadow Set bits */
#define INT_PRISS_PRI1SS_MASK        (15 << INT_PRISS_PRI1SS_SHIFT)
#  define INT_PRISS_PRI1SS(n)        ((uint32_t)(n) << INT_PRISS_PRI1SS_SHIFT)
#define INT_PRISS_PRI2SS_SHIFT       (8)       /* Bits 8-11: Interrupt with Priority Level 2 Shadow Set bits */
#define INT_PRISS_PRI2SS_MASK        (15 << INT_PRISS_PRI2SS_SHIFT)
#  define INT_PRISS_PRI2SS(n)        ((uint32_t)(n) << INT_PRISS_PRI2SS_SHIFT)
#define INT_PRISS_PRI3SS_SHIFT       (12)      /* Bits 12-15: Interrupt with Priority Level 3 Shadow Set bits */
#define INT_PRISS_PRI3SS_MASK        (15 << INT_PRISS_PRI3SS_SHIFT)
#  define INT_PRISS_PRI3SS(n)        ((uint32_t)(n) << INT_PRISS_PRI3SS_SHIFT)
#define INT_PRISS_PRI4SS_SHIFT       (16)      /* Bits 16-19: Interrupt with Priority Level 4 Shadow Set bits */
#define INT_PRISS_PRI4SS_MASK        (15 << INT_PRISS_PRI4SS_SHIFT)
#  define INT_PRISS_PRI4SS(n)        ((uint32_t)(n) << INT_PRISS_PRI4SS_SHIFT)
#define INT_PRISS_PRI5SS_SHIFT       (20)      /* Bits 20-23: Interrupt with Priority Level 5 Shadow Set bits */
#define INT_PRISS_PRI5SS_MASK        (15 << INT_PRISS_PRI5SS_SHIFT)
#  define INT_PRISS_PRI5SS(n)        ((uint32_t)(n) << INT_PRISS_PRI5SS_SHIFT)
#define INT_PRISS_PRI6SS_SHIFT       (24)      /* Bits 24-27: Interrupt with Priority Level 6 Shadow Set bits */
#define INT_PRISS_PRI6SS_MASK        (15 << INT_PRISS_PRI6SS_SHIFT)
#  define INT_PRISS_PRI6SS(n)        ((uint32_t)(n) << INT_PRISS_PRI6SS_SHIFT)
#define INT_PRISS_PRI7SS_SHIFT       (28)      /* Bits 28-31: Interrupt with Priority Level 7 Shadow Set bits */
#define INT_PRISS_PRI7SS_MASK        (15 << INT_PRISS_PRI7SS_SHIFT)
#  define INT_PRISS_PRI7SS(n)        ((uint32_t)(n) << INT_PRISS_PRI7SS_SHIFT)

/* Interrupt status register */

#define INT_INTSTAT_SIRQ_SHIFT       (0)       /* Bits 0-7: Last Interrupt Request Serviced Status */
#define INT_INTSTAT_SIRQ_MASK        (0xff << INT_INTSTAT_SIRQ_SHIFT)
#define INT_INTSTAT_SRIPL_SHIFT      (8)       /* Bits 8-10: Requested Priority Level (Single Vector Mode) */
#define INT_INTSTAT_SRIPL_MASK       (7 << INT_INTSTAT_SRIPL_SHIFT)

/* Interrupt proximity timer register -- This register contains a 32-bit
 * reload value with no field definitions.
 */

/* Interrupt flag status register 0-5 and Interrupt enable control
 * register 0-5
 * Contains interrupt status/control bits, one for each interrupt:
 *
 *  IFS0/IEC0 - Interrupts 0-31
 *  IFS1/IEC1 - Interrupts 32-63
 *  IFS2/IEC2 - Interrupts 64-95
 *  IFS3/IEC3 - Interrupts 96-127
 *  IFS4/IEC4 - Interrupts 128-159
 *  IFS5/IEC5 - Interrupts 160-190
 */

/* Interrupt priority control register 0-47 */

#define INT_IPC_DISABLED             0     /* Disabled! */
#define INT_IPC_MIN_PRIORITY         1     /* Minimum (enabled) priority */
#define INT_IPC_MID_PRIORITY         4     /* Can be used as the default */
#define INT_IPC_MAX_PRIORITY         7     /* Maximum priority */

/* Each of the 48 IPC register contains controls for 4 interrupts */

#define INT_IPC_REGNDX(n)           ((n) >> 2)
#define INT_IPC_REGOFFSET(n)        ((n) & 3)
#define INT_IOC_REGADDR(n)          PIC32MZ_INT_IPC(INT_IPC_REGNDX(n))

/* Interrupt sub-priority (IS) field is aligned at bits 0, 8, 16, and 24 in
 * each IPC register
 */

#define INT_IPC_MIN_SUBPRIORITY      0     /* Minimum sub-priority */
#define INT_IPC_MAX_SUBPRIORITY      0     /* Maximum sub-priority */

#define INT_IPC_IS_SHIFT(n)         (INT_IPC_REGOFFSET(n) << 3)
#define INT_IPC_IS_MASK(n)          (3 << INT_IPC_IS_SHIFT(n))

/* Interrupt priority (IP) field is aligned at bits 2, 10, 18, and 26 in
 * each IPC register
 */

#define INT_IPC_IP_SHIFT(n)         (2 + (INT_IPC_REGOFFSET(n) << 3))
#define INT_IPC_IP_MASK(n)          (7 << INT_IPC_IP_SHIFT(n))

/* Interrupt vector n address address offset, n=0-190 */

#define INT_OFF_VOFF_SHIFT          (0)      /* Bits 1-17: Interrupt vector address offset */
#define INT_OFF_VOFF_MASK           (0x3fffe << INT_OFF_VOFF_SHIFT)

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
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZ_INT_H */
