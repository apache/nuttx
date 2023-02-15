/****************************************************************************
 * arch/mips/src/pic32mz/hardware/pic32mzec_pps.h
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

#ifndef __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZEC_PPS_H
#define __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZEC_PPS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "pic32mz_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PPS Register Offsets *****************************************************/

/* Peripheral pin select input register map */

#define PIC32MZ_INTnR_OFFSET(n)          (0x1400 + ((n << 2)) /* n=1..4 */
#  define PIC32MZ_INT1R_OFFSET           0x1404
#  define PIC32MZ_INT2R_OFFSET           0x1408
#  define PIC32MZ_INT3R_OFFSET           0x140c
#  define PIC32MZ_INT4R_OFFSET           0x1410
#define PIC32MZ_TnCKR_OFFSET(n)          (0x1410 + ((n << 2)) /* n=2..9 */
#  define PIC32MZ_I2TKR_OFFSET           0x1418
#  define PIC32MZ_I3TKR_OFFSET           0x141c
#  define PIC32MZ_I4TKR_OFFSET           0x1420
#  define PIC32MZ_I5TKR_OFFSET           0x1424
#  define PIC32MZ_I6TKR_OFFSET           0x1428
#  define PIC32MZ_I7TKR_OFFSET           0x142c
#  define PIC32MZ_I8TKR_OFFSET           0x1430
#  define PIC32MZ_I9TKR_OFFSET           0x1434
#define PIC32MZ_ICnR_OFFSET(n)           (0x1434 + ((n << 2)) /* n=1..9 */
#  define PIC32MZ_IC1R_OFFSET            0x1438
#  define PIC32MZ_IC2R_OFFSET            0x143c
#  define PIC32MZ_IC3R_OFFSET            0x1440
#  define PIC32MZ_IC4R_OFFSET            0x1444
#  define PIC32MZ_IC5R_OFFSET            0x1448
#  define PIC32MZ_IC6R_OFFSET            0x144c
#  define PIC32MZ_IC7R_OFFSET            0x1450
#  define PIC32MZ_IC8R_OFFSET            0x1454
#  define PIC32MZ_IC9R_OFFSET            0x1458
#define PIC32MZ_OCFAR_OFFSET             0x1460
#define PIC32MZ_UnRXR_OFFSET(n)          (0x1460 + ((n << 3)) /* n=1..6 */
#  define PIC32MZ_U1RXR_OFFSET           0x1468
#  define PIC32MZ_U2RXR_OFFSET           0x1470
#  define PIC32MZ_U3RXR_OFFSET           0x1478
#  define PIC32MZ_U4RXR_OFFSET           0x1480
#  define PIC32MZ_U5RXR_OFFSET           0x1488
#  define PIC32MZ_U6RXR_OFFSET           0x1490
#define PIC32MZ_UnCTSR_OFFSET(n)         (0x1464 + ((n << 3)) /* n=1..6 */
#  define PIC32MZ_U1CTSR_OFFSET          0x146c
#  define PIC32MZ_U2CTSR_OFFSET          0x1474
#  define PIC32MZ_U3CTSR_OFFSET          0x147c
#  define PIC32MZ_U4CTSR_OFFSET          0x1484
#  define PIC32MZ_U5CTSR_OFFSET          0x148c
#  define PIC32MZ_U6CTSR_OFFSET          0x1494
#define PIC32MZ_SDInR_OFFSET(n)          (0x1490 + 12*(n)) /* n=1..6 */
#  define PIC32MZ_SDI1R_OFFSET           0x149c
#  define PIC32MZ_SDI2R_OFFSET           0x14a8
#  define PIC32MZ_SDI3R_OFFSET           0x14b4
#  define PIC32MZ_SDI4R_OFFSET           0x14c0
#  define PIC32MZ_SDI5R_OFFSET           0x14cc
#  define PIC32MZ_SDI6R_OFFSET           0x14d8
#define PIC32MZ_SSnR_OFFSET(n)           (0x1494 + 12*(n)) /* n=1..6 */
#  define PIC32MZ_SS1R_OFFSET            0x14a0
#  define PIC32MZ_SS2R_OFFSET            0x14ac
#  define PIC32MZ_SS3R_OFFSET            0x14b8
#  define PIC32MZ_SS4R_OFFSET            0x14c4
#  define PIC32MZ_SS5R_OFFSET            0x14d0
#  define PIC32MZ_SS6R_OFFSET            0x14dc
#define PIC32MZ_CnRXR_OFFSET(n)          (0x14dc + ((n) << 2)) /* n=1..2 */
#  define PIC32MZ_C1RXR_OFFSET           0x14e0
#  define PIC32MZ_C2RXR_OFFSET           0x14e4
#define PIC32MZ_REFCLKInR_OFFSET(n)      (0x14e4 + ((n) << 2)) /* n=1,3,4 */
#  define PIC32MZ_REFCLKI1R_OFFSET       0x14e8
#  define PIC32MZ_REFCLKI3R_OFFSET       0x14f0
#  define PIC32MZ_REFCLKI4R_OFFSET       0x14f4

/* Peripheral pin select output register map */

#define PIC32MZ_RPAnR_OFFSET(n)          (0x1500 + ((n) << 2)) /* n=14,15 */
#  define PIC32MZ_RPA14R_OFFSET          0x1538
#  define PIC32MZ_RPA15R_OFFSET          0x153c
#define PIC32MZ_RPBnR_OFFSET(n)          (0x1540 + ((n) << 2)) /* n=0..,15 */
#  define PIC32MZ_RPB0R_OFFSET           0x1540
#  define PIC32MZ_RPB1R_OFFSET           0x1544
#  define PIC32MZ_RPB2R_OFFSET           0x1548
#  define PIC32MZ_RPB3R_OFFSET           0x154c
#  define PIC32MZ_RPB4R_OFFSET           0x1550
#  define PIC32MZ_RPB5R_OFFSET           0x1554
#  define PIC32MZ_RPB6R_OFFSET           0x1558
#  define PIC32MZ_RPB7R_OFFSET           0x155c
#  define PIC32MZ_RPB8R_OFFSET           0x1560
#  define PIC32MZ_RPB9R_OFFSET           0x1564
#  define PIC32MZ_RPB10R_OFFSET          0x1568
#  define PIC32MZ_RPB11R_OFFSET          0x156c
#  define PIC32MZ_RPB12R_OFFSET          0x1570
#  define PIC32MZ_RPB13R_OFFSET          0x1574
#  define PIC32MZ_RPB14R_OFFSET          0x1578
#  define PIC32MZ_RPB15R_OFFSET          0x157c
#define PIC32MZ_RPCnR_OFFSET(n)          (0x1580 + ((n) << 2)) /* n=1..4,13,14 */
#  define PIC32MZ_RPC1R_OFFSET           0x1584
#  define PIC32MZ_RPC2R_OFFSET           0x1588
#  define PIC32MZ_RPC3R_OFFSET           0x158c
#  define PIC32MZ_RPC4R_OFFSET           0x1590
#  define PIC32MZ_RPC13R_OFFSET          0x15b4
#  define PIC32MZ_RPC14R_OFFSET          0x15b8
#define PIC32MZ_RPDnR_OFFSET(n)          (0x15c0 + ((n) << 2)) /* n=0..15 */
#  define PIC32MZ_RPD0R_OFFSET           0x15c0
#  define PIC32MZ_RPD1R_OFFSET           0x15c4
#  define PIC32MZ_RPD2R_OFFSET           0x15c8
#  define PIC32MZ_RPD3R_OFFSET           0x15cc
#  define PIC32MZ_RPD4R_OFFSET           0x15d0
#  define PIC32MZ_RPD5R_OFFSET           0x15d4
#  define PIC32MZ_RPD6R_OFFSET           0x15d8
#  define PIC32MZ_RPD7R_OFFSET           0x15dc
#  define PIC32MZ_RPD8R_OFFSET           0x15e0
#  define PIC32MZ_RPD9R_OFFSET           0x15e4
#  define PIC32MZ_RPD10R_OFFSET          0x15e8
#  define PIC32MZ_RPD11R_OFFSET          0x15ec
#  define PIC32MZ_RPD12R_OFFSET          0x15f0
#  define PIC32MZ_RPD13R_OFFSET          0x15f4
#  define PIC32MZ_RPD14R_OFFSET          0x15f8
#  define PIC32MZ_RPD15R_OFFSET          0x15fc
#define PIC32MZ_RPEnR_OFFSET(n)          (0x1600 + ((n) << 2)) /* n=3,5,8-9 */
#  define PIC32MZ_RPE3R_OFFSET           0x160c
#  define PIC32MZ_RPE5R_OFFSET           0x1614
#  define PIC32MZ_RPE8R_OFFSET           0x1620
#  define PIC32MZ_RPE9R_OFFSET           0x1624
#define PIC32MZ_RPFnR_OFFSET(n)          (0x1640 + ((n) << 2)) /* n=0-5,8,12-13 */
#  define PIC32MZ_RPF0R_OFFSET           0x1640
#  define PIC32MZ_RPF1R_OFFSET           0x1644
#  define PIC32MZ_RPF2R_OFFSET           0x1648
#  define PIC32MZ_RPF3R_OFFSET           0x164c
#  define PIC32MZ_RPF4R_OFFSET           0x1650
#  define PIC32MZ_RPF5R_OFFSET           0x1654
#  define PIC32MZ_RPF8R_OFFSET           0x1660
#  define PIC32MZ_RPF12R_OFFSET          0x1670
#  define PIC32MZ_RPF13R_OFFSET          0x1674
#define PIC32MZ_RPGnR_OFFSET(n)          (0x1680 + ((n) << 2)) /* n=0-1,6-8,9 */
#  define PIC32MZ_RPG0R_OFFSET           0x1680
#  define PIC32MZ_RPG1R_OFFSET           0x1684
#  define PIC32MZ_RPG6R_OFFSET           0x1698
#  define PIC32MZ_RPG7R_OFFSET           0x169c
#  define PIC32MZ_RPG8R_OFFSET           0x16a0
#  define PIC32MZ_RPG9R_OFFSET           0x16a4

/* PPS Register Addresses ***************************************************/

#define PIC32MZ_INTnR(n)                 (PIC32MZ_SFR_K1BASE+PIC32MZ_INTnR_OFFSET(n))
#  define PIC32MZ_INT1R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_INT1R_OFFSET)
#  define PIC32MZ_INT2R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_INT2R_OFFSET)
#  define PIC32MZ_INT3R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_INT3R_OFFSET)
#  define PIC32MZ_INT4R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_INT4R_OFFSET)
#define PIC32MZ_TnCKR(n)                 (PIC32MZ_SFR_K1BASE+PIC32MZ_TnCKR_OFFSET(n))
#  define PIC32MZ_I2TKR                  (PIC32MZ_SFR_K1BASE+PIC32MZ_I2TKR_OFFSET)
#  define PIC32MZ_I3TKR                  (PIC32MZ_SFR_K1BASE+PIC32MZ_I3TKR_OFFSET)
#  define PIC32MZ_I4TKR                  (PIC32MZ_SFR_K1BASE+PIC32MZ_I4TKR_OFFSET)
#  define PIC32MZ_I5TKR                  (PIC32MZ_SFR_K1BASE+PIC32MZ_I5TKR_OFFSET)
#  define PIC32MZ_I6TKR                  (PIC32MZ_SFR_K1BASE+PIC32MZ_I6TKR_OFFSET)
#  define PIC32MZ_I7TKR                  (PIC32MZ_SFR_K1BASE+PIC32MZ_I7TKR_OFFSET)
#  define PIC32MZ_I8TKR                  (PIC32MZ_SFR_K1BASE+PIC32MZ_I8TKR_OFFSET)
#  define PIC32MZ_I9TKR                  (PIC32MZ_SFR_K1BASE+PIC32MZ_I9TKR_OFFSET)
#define PIC32MZ_ICnR(n)                  (PIC32MZ_SFR_K1BASE+PIC32MZ_ICnR_OFFSET(n))
#  define PIC32MZ_IC1R                   (PIC32MZ_SFR_K1BASE+PIC32MZ_IC1R_OFFSET)
#  define PIC32MZ_IC2R                   (PIC32MZ_SFR_K1BASE+PIC32MZ_IC2R_OFFSET)
#  define PIC32MZ_IC3R                   (PIC32MZ_SFR_K1BASE+PIC32MZ_IC3R_OFFSET)
#  define PIC32MZ_IC4R                   (PIC32MZ_SFR_K1BASE+PIC32MZ_IC4R_OFFSET)
#  define PIC32MZ_IC5R                   (PIC32MZ_SFR_K1BASE+PIC32MZ_IC5R_OFFSET)
#  define PIC32MZ_IC6R                   (PIC32MZ_SFR_K1BASE+PIC32MZ_IC6R_OFFSET)
#  define PIC32MZ_IC7R                   (PIC32MZ_SFR_K1BASE+PIC32MZ_IC7R_OFFSET)
#  define PIC32MZ_IC8R                   (PIC32MZ_SFR_K1BASE+PIC32MZ_IC8R_OFFSET)
#  define PIC32MZ_IC9R                   (PIC32MZ_SFR_K1BASE+PIC32MZ_IC9R_OFFSET)
#define PIC32MZ_OCFAR                    (PIC32MZ_SFR_K1BASE+PIC32MZ_OCFAR_OFFSET)
#define PIC32MZ_UnRXR(n)                 (PIC32MZ_SFR_K1BASE+PIC32MZ_UnRXR_OFFSET(n))
#  define PIC32MZ_U1RXR                  (PIC32MZ_SFR_K1BASE+PIC32MZ_U1RXR_OFFSET)
#  define PIC32MZ_U2RXR                  (PIC32MZ_SFR_K1BASE+PIC32MZ_U2RXR_OFFSET)
#  define PIC32MZ_U3RXR                  (PIC32MZ_SFR_K1BASE+PIC32MZ_U3RXR_OFFSET)
#  define PIC32MZ_U4RXR                  (PIC32MZ_SFR_K1BASE+PIC32MZ_U4RXR_OFFSET)
#  define PIC32MZ_U5RXR                  (PIC32MZ_SFR_K1BASE+PIC32MZ_U5RXR_OFFSET)
#  define PIC32MZ_U6RXR                  (PIC32MZ_SFR_K1BASE+PIC32MZ_U6RXR_OFFSET)
#define PIC32MZ_UnCTSR(n)                (PIC32MZ_SFR_K1BASE+PIC32MZ_UnCTSR_OFFSET(n)
#  define PIC32MZ_U1CTSR                 (PIC32MZ_SFR_K1BASE+PIC32MZ_U1CTSR_OFFSET)
#  define PIC32MZ_U2CTSR                 (PIC32MZ_SFR_K1BASE+PIC32MZ_U2CTSR_OFFSET)
#  define PIC32MZ_U3CTSR                 (PIC32MZ_SFR_K1BASE+PIC32MZ_U3CTSR_OFFSET)
#  define PIC32MZ_U4CTSR                 (PIC32MZ_SFR_K1BASE+PIC32MZ_U4CTSR_OFFSET)
#  define PIC32MZ_U5CTSR                 (PIC32MZ_SFR_K1BASE+PIC32MZ_U5CTSR_OFFSET)
#  define PIC32MZ_U6CTSR                 (PIC32MZ_SFR_K1BASE+PIC32MZ_U6CTSR_OFFSET)
#define PIC32MZ_SDInR(n)                 (PIC32MZ_SFR_K1BASE+PIC32MZ_SDInR_OFFSET(n))
#  define PIC32MZ_SDI1R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_SDI1R_OFFSET)
#  define PIC32MZ_SDI2R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_SDI2R_OFFSET)
#  define PIC32MZ_SDI3R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_SDI3R_OFFSET)
#  define PIC32MZ_SDI4R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_SDI4R_OFFSET)
#  define PIC32MZ_SDI5R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_SDI5R_OFFSET)
#  define PIC32MZ_SDI6R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_SDI6R_OFFSET)
#define PIC32MZ_SSnR(n)                  (PIC32MZ_SFR_K1BASE+PIC32MZ_SSnR_OFFSET(n))
#  define PIC32MZ_SS1R                   (PIC32MZ_SFR_K1BASE+PIC32MZ_SS1R_OFFSET)
#  define PIC32MZ_SS2R                   (PIC32MZ_SFR_K1BASE+PIC32MZ_SS2R_OFFSET)
#  define PIC32MZ_SS3R                   (PIC32MZ_SFR_K1BASE+PIC32MZ_SS3R_OFFSET)
#  define PIC32MZ_SS4R                   (PIC32MZ_SFR_K1BASE+PIC32MZ_SS4R_OFFSET)
#  define PIC32MZ_SS5R                   (PIC32MZ_SFR_K1BASE+PIC32MZ_SS5R_OFFSET)
#  define PIC32MZ_SS6R                   (PIC32MZ_SFR_K1BASE+PIC32MZ_SS6R_OFFSET)
#define PIC32MZ_CnRXR(n)                 (PIC32MZ_SFR_K1BASE+PIC32MZ_CnRXR_OFFSET(n))
#  define PIC32MZ_C1RXR                  (PIC32MZ_SFR_K1BASE+PIC32MZ_C1RXR_OFFSET)
#  define PIC32MZ_C2RXR                  (PIC32MZ_SFR_K1BASE+PIC32MZ_C2RXR_OFFSET)
#define PIC32MZ_REFCLKInR(n)             (PIC32MZ_SFR_K1BASE+PIC32MZ_REFCLKInR_OFFSET(n))
#  define PIC32MZ_REFCLKI1R              (PIC32MZ_SFR_K1BASE+PIC32MZ_REFCLKI1R_OFFSET)
#  define PIC32MZ_REFCLKI3R              (PIC32MZ_SFR_K1BASE+PIC32MZ_REFCLKI3R_OFFSET)
#  define PIC32MZ_REFCLKI4R              (PIC32MZ_SFR_K1BASE+PIC32MZ_REFCLKI4R_OFFSET)

/* Peripheral pin select output register map */

#define PIC32MZ_RPAnR(n)                 (PIC32MZ_SFR_K1BASE+PIC32MZ_RPAnR_OFFSET(n))
#  define PIC32MZ_RPA14R                 (PIC32MZ_SFR_K1BASE+PIC32MZ_RPA14R_OFFSET)
#  define PIC32MZ_RPA15R                 (PIC32MZ_SFR_K1BASE+PIC32MZ_RPA15R_OFFSET)
#define PIC32MZ_RPBnR(n)                 (PIC32MZ_SFR_K1BASE+PIC32MZ_RPBnR_OFFSET(n))
#  define PIC32MZ_RPB0R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPB0R_OFFSET)
#  define PIC32MZ_RPB1R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPB1R_OFFSET)
#  define PIC32MZ_RPB2R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPB2R_OFFSET)
#  define PIC32MZ_RPB3R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPB3R_OFFSET)
#  define PIC32MZ_RPB4R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPB4R_OFFSET)
#  define PIC32MZ_RPB5R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPB5R_OFFSET)
#  define PIC32MZ_RPB6R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPB6R_OFFSET)
#  define PIC32MZ_RPB7R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPB7R_OFFSET)
#  define PIC32MZ_RPB8R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPB8R_OFFSET)
#  define PIC32MZ_RPB9R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPB9R_OFFSET)
#  define PIC32MZ_RPB10R                 (PIC32MZ_SFR_K1BASE+PIC32MZ_RPB10R_OFFSET)
#  define PIC32MZ_RPB11R                 (PIC32MZ_SFR_K1BASE+PIC32MZ_RPB11R_OFFSET)
#  define PIC32MZ_RPB12R                 (PIC32MZ_SFR_K1BASE+PIC32MZ_RPB12R_OFFSET)
#  define PIC32MZ_RPB13R                 (PIC32MZ_SFR_K1BASE+PIC32MZ_RPB13R_OFFSET)
#  define PIC32MZ_RPB14R                 (PIC32MZ_SFR_K1BASE+PIC32MZ_RPB14R_OFFSET)
#  define PIC32MZ_RPB15R                 (PIC32MZ_SFR_K1BASE+PIC32MZ_RPB15R_OFFSET)
#define PIC32MZ_RPCnR(n)                 (PIC32MZ_SFR_K1BASE+PIC32MZ_RPCnR_OFFSET(n))
#  define PIC32MZ_RPC1R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPC1R_OFFSET)
#  define PIC32MZ_RPC2R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPC2R_OFFSET)
#  define PIC32MZ_RPC3R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPC3R_OFFSET)
#  define PIC32MZ_RPC4R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPC4R_OFFSET)
#  define PIC32MZ_RPC13R                 (PIC32MZ_SFR_K1BASE+PIC32MZ_RPC13R_OFFSET)
#  define PIC32MZ_RPC14R                 (PIC32MZ_SFR_K1BASE+PIC32MZ_RPC14R_OFFSET)
#define PIC32MZ_RPDnR(n)                 (PIC32MZ_SFR_K1BASE+PIC32MZ_RPDnR_OFFSET(n))
#  define PIC32MZ_RPD0R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPD0R_OFFSET)
#  define PIC32MZ_RPD1R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPD1R_OFFSET)
#  define PIC32MZ_RPD2R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPD2R_OFFSET)
#  define PIC32MZ_RPD3R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPD3R_OFFSET)
#  define PIC32MZ_RPD4R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPD4R_OFFSET)
#  define PIC32MZ_RPD5R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPD5R_OFFSET)
#  define PIC32MZ_RPD6R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPD6R_OFFSET)
#  define PIC32MZ_RPD7R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPD7R_OFFSET)
#  define PIC32MZ_RPD8R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPD8R_OFFSET)
#  define PIC32MZ_RPD9R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPD9R_OFFSET)
#  define PIC32MZ_RPD10R                 (PIC32MZ_SFR_K1BASE+PIC32MZ_RPD10R_OFFSET)
#  define PIC32MZ_RPD11R                 (PIC32MZ_SFR_K1BASE+PIC32MZ_RPD11R_OFFSET)
#  define PIC32MZ_RPD12R                 (PIC32MZ_SFR_K1BASE+PIC32MZ_RPD12R_OFFSET)
#  define PIC32MZ_RPD13R                 (PIC32MZ_SFR_K1BASE+PIC32MZ_RPD13R_OFFSET)
#  define PIC32MZ_RPD14R                 (PIC32MZ_SFR_K1BASE+PIC32MZ_RPD14R_OFFSET)
#  define PIC32MZ_RPD15R                 (PIC32MZ_SFR_K1BASE+PIC32MZ_RPD15R_OFFSET)
#define PIC32MZ_RPEnR(n)                 (PIC32MZ_SFR_K1BASE+PIC32MZ_RPEnR_OFFSET(n))
#  define PIC32MZ_RPE3R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPE3R_OFFSET)
#  define PIC32MZ_RPE5R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPE5R_OFFSET)
#  define PIC32MZ_RPE8R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPE8R_OFFSET)
#  define PIC32MZ_RPE9R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPE9R_OFFSET)
#define PIC32MZ_RPFnR(n)                 (PIC32MZ_SFR_K1BASE+PIC32MZ_RPFnR_OFFSET(n))
#  define PIC32MZ_RPF0R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPF0R_OFFSET)
#  define PIC32MZ_RPF1R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPF1R_OFFSET)
#  define PIC32MZ_RPF2R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPF2R_OFFSET)
#  define PIC32MZ_RPF3R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPF3R_OFFSET)
#  define PIC32MZ_RPF4R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPF4R_OFFSET)
#  define PIC32MZ_RPF5R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPF5R_OFFSET)
#  define PIC32MZ_RPF8R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPF8R_OFFSET)
#  define PIC32MZ_RPF12R                 (PIC32MZ_SFR_K1BASE+PIC32MZ_RPF12R_OFFSET)
#  define PIC32MZ_RPF13R                 (PIC32MZ_SFR_K1BASE+PIC32MZ_RPF13R_OFFSET)
#define PIC32MZ_RPGnR(n)                 (PIC32MZ_SFR_K1BASE+PIC32MZ_RPGnR_OFFSET(n))
#  define PIC32MZ_RPG0R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPG0R_OFFSET)
#  define PIC32MZ_RPG1R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPG1R_OFFSET)
#  define PIC32MZ_RPG6R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPG6R_OFFSET)
#  define PIC32MZ_RPG7R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPG7R_OFFSET)
#  define PIC32MZ_RPG8R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPG8R_OFFSET)
#  define PIC32MZ_RPG9R                  (PIC32MZ_SFR_K1BASE+PIC32MZ_RPG9R_OFFSET)

/* Input Pin Selection ******************************************************/

/* The encoding of the input pin selection is simple.
 * Since we know the devices, we also can infer the register
 * address so we need only the value for the register which is
 * exactly what is provided by the following definitions.
 */

#define C1RXR_RPA15                      13
#define C1RXR_RPB1                       5
#define C1RXR_RPB3                       8
#define C1RXR_RPC4                       10
#define C1RXR_RPC13                      7
#define C1RXR_RPD3                       0
#define C1RXR_RPD7                       14
#define C1RXR_RPD11                      3
#define C1RXR_RPD15                      11
#define C1RXR_RPE5                       6
#define C1RXR_RPF0                       4
#define C1RXR_RPF5                       2
#define C1RXR_RPG0                       12
#define C1RXR_RPG7                       1

#define C2RXR_RPB0                       5
#define C2RXR_RPB7                       7
#define C2RXR_RPB8                       2
#define C2RXR_RPB15                      3
#define C2RXR_RPC3                       12
#define C2RXR_RPD4                       4
#define C2RXR_RPD9                       0
#define C2RXR_RPD12                      10
#define C2RXR_RPE3                       6
#define C2RXR_RPE9                       13
#define C2RXR_RPF8                       11
#define C2RXR_RPF12                      9
#define C2RXR_RPG6                       1

#define IC1R_RPB2                        7
#define IC1R_RPB6                        5
#define IC1R_RPB14                       2
#define IC1R_RPC2                        12
#define IC1R_RPD0                        3
#define IC1R_RPD1                        0
#define IC1R_RPD5                        6
#define IC1R_RPE8                        13
#define IC1R_RPF2                        11
#define IC1R_RPF3                        8
#define IC1R_RPF13                       9
#define IC1R_RPG9                        1

#define IC2R_RPB0                        5
#define IC2R_RPB7                        7
#define IC2R_RPB8                        2
#define IC2R_RPB15                       3
#define IC2R_RPC3                        12
#define IC2R_RPD4                        4
#define IC2R_RPD9                        0
#define IC2R_RPD12                       10
#define IC2R_RPE3                        6
#define IC2R_RPE9                        13
#define IC2R_RPF8                        11
#define IC2R_RPF12                       9
#define IC2R_RPG6                        1

#define IC3R_RPA14                       13
#define IC3R_RPB5                        8
#define IC3R_RPB9                        5
#define IC3R_RPB10                       6
#define IC3R_RPC1                        10
#define IC3R_RPC14                       7
#define IC3R_RPD2                        0
#define IC3R_RPD6                        14
#define IC3R_RPD10                       3
#define IC3R_RPD14                       11
#define IC3R_RPF1                        4
#define IC3R_RPF4                        2
#define IC3R_RPG1                        12
#define IC3R_RPG8                        1

#define IC4R_RPA15                       13
#define IC4R_RPB1                        5
#define IC4R_RPB3                        8
#define IC4R_RPC4                        10
#define IC4R_RPC13                       7
#define IC4R_RPD3                        0
#define IC4R_RPD7                        14
#define IC4R_RPD11                       3
#define IC4R_RPD15                       11
#define IC4R_RPE5                        6
#define IC4R_RPF0                        4
#define IC4R_RPF5                        2
#define IC4R_RPG0                        12
#define IC4R_RPG7                        1

#define IC5R_RPB0                        5
#define IC5R_RPB7                        7
#define IC5R_RPB8                        2
#define IC5R_RPB15                       3
#define IC5R_RPC3                        12
#define IC5R_RPD4                        4
#define IC5R_RPD9                        0
#define IC5R_RPD12                       10
#define IC5R_RPE3                        6
#define IC5R_RPE9                        13
#define IC5R_RPF8                        11
#define IC5R_RPF12                       9
#define IC5R_RPG6                        1

#define IC6R_RPB2                        7
#define IC6R_RPB6                        5
#define IC6R_RPB14                       2
#define IC6R_RPC2                        12
#define IC6R_RPD0                        3
#define IC6R_RPD1                        0
#define IC6R_RPD5                        6
#define IC6R_RPE8                        13
#define IC6R_RPF2                        11
#define IC6R_RPF3                        8
#define IC6R_RPF13                       9
#define IC6R_RPG9                        1

#define IC7R_RPA14                       13
#define IC7R_RPB5                        8
#define IC7R_RPB9                        5
#define IC7R_RPB10                       6
#define IC7R_RPC1                        10
#define IC7R_RPC14                       7
#define IC7R_RPD2                        0
#define IC7R_RPD6                        14
#define IC7R_RPD10                       3
#define IC7R_RPD14                       11
#define IC7R_RPF1                        4
#define IC7R_RPF4                        2
#define IC7R_RPG1                        12
#define IC7R_RPG8                        1

#define IC8R_RPA15                       13
#define IC8R_RPB1                        5
#define IC8R_RPB3                        8
#define IC8R_RPC4                        10
#define IC8R_RPC13                       7
#define IC8R_RPD3                        0
#define IC8R_RPD7                        14
#define IC8R_RPD11                       3
#define IC8R_RPD15                       11
#define IC8R_RPE5                        6
#define IC8R_RPF0                        4
#define IC8R_RPF5                        2
#define IC8R_RPG0                        12
#define IC8R_RPG7                        1

#define IC9R_RPB0                        5
#define IC9R_RPB7                        7
#define IC9R_RPB8                        2
#define IC9R_RPB15                       3
#define IC9R_RPC3                        12
#define IC9R_RPD4                        4
#define IC9R_RPD9                        0
#define IC9R_RPD12                       10
#define IC9R_RPE3                        6
#define IC9R_RPE9                        13
#define IC9R_RPF8                        11
#define IC9R_RPF12                       9
#define IC9R_RPG6                        1

#define INT1R_RPB2                       7
#define INT1R_RPB6                       5
#define INT1R_RPB14                      2
#define INT1R_RPC2                       12
#define INT1R_RPD0                       3
#define INT1R_RPD1                       0
#define INT1R_RPD5                       6
#define INT1R_RPE8                       13
#define INT1R_RPF2                       11
#define INT1R_RPF3                       8
#define INT1R_RPF13                      9
#define INT1R_RPG9                       1

#define INT2R_RPB0                       5
#define INT2R_RPB7                       7
#define INT2R_RPB8                       2
#define INT2R_RPB15                      3
#define INT2R_RPC3                       12
#define INT2R_RPD4                       4
#define INT2R_RPD9                       0
#define INT2R_RPD12                      10
#define INT2R_RPE3                       6
#define INT2R_RPE9                       13
#define INT2R_RPF8                       11
#define INT2R_RPF12                      9
#define INT2R_RPG6                       1

#define INT3R_RPA14                      13
#define INT3R_RPB5                       8
#define INT3R_RPB9                       5
#define INT3R_RPB10                      6
#define INT3R_RPC1                       10
#define INT3R_RPC14                      7
#define INT3R_RPD2                       0
#define INT3R_RPD6                       14
#define INT3R_RPD10                      3
#define INT3R_RPD14                      11
#define INT3R_RPF1                       4
#define INT3R_RPF4                       2
#define INT3R_RPG1                       12
#define INT3R_RPG8                       1

#define INT4R_RPA15                      13
#define INT4R_RPB1                       5
#define INT4R_RPB3                       8
#define INT4R_RPC4                       10
#define INT4R_RPC13                      7
#define INT4R_RPD3                       0
#define INT4R_RPD7                       14
#define INT4R_RPD11                      3
#define INT4R_RPD15                      11
#define INT4R_RPE5                       6
#define INT4R_RPF0                       4
#define INT4R_RPF5                       2
#define INT4R_RPG0                       12
#define INT4R_RPG7                       1

#define OCFAR_RPB2                       7
#define OCFAR_RPB6                       5
#define OCFAR_RPB14                      2
#define OCFAR_RPC2                       12
#define OCFAR_RPD0                       3
#define OCFAR_RPD1                       0
#define OCFAR_RPD5                       6
#define OCFAR_RPE8                       13
#define OCFAR_RPF2                       11
#define OCFAR_RPF3                       8
#define OCFAR_RPF13                      9
#define OCFAR_RPG9                       1

#define REFCLKI1R_RPA14                  13
#define REFCLKI1R_RPB5                   8
#define REFCLKI1R_RPB9                   5
#define REFCLKI1R_RPB10                  6
#define REFCLKI1R_RPC1                   10
#define REFCLKI1R_RPC14                  7
#define REFCLKI1R_RPD2                   0
#define REFCLKI1R_RPD6                   14
#define REFCLKI1R_RPD10                  3
#define REFCLKI1R_RPD14                  11
#define REFCLKI1R_RPF1                   4
#define REFCLKI1R_RPF4                   2
#define REFCLKI1R_RPG1                   12
#define REFCLKI1R_RPG8                   1

#define REFCLKI3R_RPB2                   7
#define REFCLKI3R_RPB6                   5
#define REFCLKI3R_RPB14                  2
#define REFCLKI3R_RPC2                   12
#define REFCLKI3R_RPD0                   3
#define REFCLKI3R_RPD1                   0
#define REFCLKI3R_RPD5                   6
#define REFCLKI3R_RPE8                   13
#define REFCLKI3R_RPF2                   11
#define REFCLKI3R_RPF3                   8
#define REFCLKI3R_RPF13                  9
#define REFCLKI3R_RPG9                   1

#define REFCLKI4R_RPA15                  13
#define REFCLKI4R_RPB1                   5
#define REFCLKI4R_RPB3                   8
#define REFCLKI4R_RPC4                   10
#define REFCLKI4R_RPC13                  7
#define REFCLKI4R_RPD3                   0
#define REFCLKI4R_RPD7                   14
#define REFCLKI4R_RPD11                  3
#define REFCLKI4R_RPD15                  11
#define REFCLKI4R_RPE5                   6
#define REFCLKI4R_RPF0                   4
#define REFCLKI4R_RPF5                   2
#define REFCLKI4R_RPG0                   12
#define REFCLKI4R_RPG7                   1

#define SDI1R_RPA14                      13
#define SDI1R_RPB5                       8
#define SDI1R_RPB9                       5
#define SDI1R_RPB10                      6
#define SDI1R_RPC1                       10
#define SDI1R_RPC14                      7
#define SDI1R_RPD2                       0
#define SDI1R_RPD6                       14
#define SDI1R_RPD10                      3
#define SDI1R_RPD14                      11
#define SDI1R_RPF1                       4
#define SDI1R_RPF4                       2
#define SDI1R_RPG1                       12
#define SDI1R_RPG8                       1

#define SDI2R_RPA15                      13
#define SDI2R_RPB1                       5
#define SDI2R_RPB3                       8
#define SDI2R_RPC4                       10
#define SDI2R_RPC13                      7
#define SDI2R_RPD3                       0
#define SDI2R_RPD7                       14
#define SDI2R_RPD11                      3
#define SDI2R_RPD15                      11
#define SDI2R_RPE5                       6
#define SDI2R_RPF0                       4
#define SDI2R_RPF5                       2
#define SDI2R_RPG0                       12
#define SDI2R_RPG7                       1

#define SDI3R_RPA14                      13
#define SDI3R_RPB5                       8
#define SDI3R_RPB9                       5
#define SDI3R_RPB10                      6
#define SDI3R_RPC1                       10
#define SDI3R_RPC14                      7
#define SDI3R_RPD2                       0
#define SDI3R_RPD10                      3
#define SDI3R_RPD14                      11
#define SDI3R_RPD6                       14
#define SDI3R_RPF1                       4
#define SDI3R_RPF4                       2
#define SDI3R_RPG1                       12
#define SDI3R_RPG8                       1

#define SDI4R_RPA15                      13
#define SDI4R_RPB1                       5
#define SDI4R_RPB3                       8
#define SDI4R_RPC4                       10
#define SDI4R_RPC13                      7
#define SDI4R_RPD3                       0
#define SDI4R_RPD7                       14
#define SDI4R_RPD11                      3
#define SDI4R_RPD15                      11
#define SDI4R_RPE5                       6
#define SDI4R_RPF0                       4
#define SDI4R_RPF5                       2
#define SDI4R_RPG0                       12
#define SDI4R_RPG7                       1

#define SDI5R_RPA14                      13
#define SDI5R_RPB5                       8
#define SDI5R_RPB9                       5
#define SDI5R_RPB10                      6
#define SDI5R_RPC1                       10
#define SDI5R_RPC14                      7
#define SDI5R_RPD2                       0
#define SDI5R_RPD6                       14
#define SDI5R_RPD10                      3
#define SDI5R_RPD14                      11
#define SDI5R_RPF1                       4
#define SDI5R_RPF4                       2
#define SDI5R_RPG1                       12
#define SDI5R_RPG8                       1

#define SDI6R_RPB2                       7
#define SDI6R_RPB6                       5
#define SDI6R_RPB14                      2
#define SDI6R_RPC2                       12
#define SDI6R_RPD0                       3
#define SDI6R_RPD1                       0
#define SDI6R_RPD5                       6
#define SDI6R_RPE8                       13
#define SDI6R_RPF2                       11
#define SDI6R_RPF3                       8
#define SDI6R_RPF13                      9
#define SDI6R_RPG9                       1

#define SS1R_RPB0                        5
#define SS1R_RPB15                       3
#define SS1R_RPB7                        7
#define SS1R_RPB8                        2
#define SS1R_RPC3                        12
#define SS1R_RPD4                        4
#define SS1R_RPD9                        0
#define SS1R_RPD12                       10
#define SS1R_RPE3                        6
#define SS1R_RPE9                        13
#define SS1R_RPF8                        11
#define SS1R_RPF12                       9
#define SS1R_RPG6                        1

#define SS2R_RPB2                        7
#define SS2R_RPB6                        5
#define SS2R_RPB14                       2
#define SS2R_RPC2                        12
#define SS2R_RPD0                        3
#define SS2R_RPD1                        0
#define SS2R_RPD5                        6
#define SS2R_RPE8                        13
#define SS2R_RPF2                        11
#define SS2R_RPF3                        8
#define SS2R_RPF13                       9
#define SS2R_RPG9                        1

#define SS3R_RPB0                        5
#define SS3R_RPB7                        7
#define SS3R_RPB8                        2
#define SS3R_RPB15                       3
#define SS3R_RPC3                        12
#define SS3R_RPD4                        4
#define SS3R_RPD9                        0
#define SS3R_RPD12                       10
#define SS3R_RPE3                        6
#define SS3R_RPE9                        13
#define SS3R_RPF8                        11
#define SS3R_RPF12                       9
#define SS3R_RPG6                        1

#define SS4R_RPB0                        5
#define SS4R_RPB7                        7
#define SS4R_RPB8                        2
#define SS4R_RPB15                       3
#define SS4R_RPC3                        12
#define SS4R_RPD4                        4
#define SS4R_RPD9                        0
#define SS4R_RPD12                       10
#define SS4R_RPE3                        6
#define SS4R_RPE9                        13
#define SS4R_RPF8                        11
#define SS4R_RPF12                       9
#define SS4R_RPG6                        1

#define SS5R_RPB0                        5
#define SS5R_RPB7                        7
#define SS5R_RPB8                        2
#define SS5R_RPB15                       3
#define SS5R_RPC3                        12
#define SS5R_RPD4                        4
#define SS5R_RPD9                        0
#define SS5R_RPD12                       10
#define SS5R_RPE3                        6
#define SS5R_RPE9                        13
#define SS5R_RPF12                       9
#define SS5R_RPF8                        11
#define SS5R_RPG6                        1

#define SS6R_RPA14                       13
#define SS6R_RPB5                        8
#define SS6R_RPB9                        5
#define SS6R_RPB10                       6
#define SS6R_RPC1                        10
#define SS6R_RPC14                       7
#define SS6R_RPD2                        0
#define SS6R_RPD6                        14
#define SS6R_RPD10                       3
#define SS6R_RPD14                       11
#define SS6R_RPF1                        4
#define SS6R_RPF4                        2
#define SS6R_RPG1                        12
#define SS6R_RPG8                        1

#define T2CKR_RPA14                      13
#define T2CKR_RPB5                       8
#define T2CKR_RPB9                       5
#define T2CKR_RPB10                      6
#define T2CKR_RPC1                       10
#define T2CKR_RPC14                      7
#define T2CKR_RPD2                       0
#define T2CKR_RPD6                       14
#define T2CKR_RPD10                      3
#define T2CKR_RPD14                      11
#define T2CKR_RPF1                       4
#define T2CKR_RPF4                       2
#define T2CKR_RPG1                       12
#define T2CKR_RPG8                       1

#define T3CKR_RPB0                       5
#define T3CKR_RPB7                       7
#define T3CKR_RPB8                       2
#define T3CKR_RPB15                      3
#define T3CKR_RPC3                       12
#define T3CKR_RPD4                       4
#define T3CKR_RPD9                       0
#define T3CKR_RPD12                      10
#define T3CKR_RPE3                       6
#define T3CKR_RPE9                       13
#define T3CKR_RPF8                       11
#define T3CKR_RPF12                      9
#define T3CKR_RPG6                       1

#define T4CKR_RPB2                       7
#define T4CKR_RPB6                       5
#define T4CKR_RPB14                      2
#define T4CKR_RPC2                       12
#define T4CKR_RPD0                       3
#define T4CKR_RPD1                       0
#define T4CKR_RPD5                       6
#define T4CKR_RPE8                       13
#define T4CKR_RPF2                       11
#define T4CKR_RPF3                       8
#define T4CKR_RPF13                      9
#define T4CKR_RPG9                       1

#define T5CKR_RPA15                      13
#define T5CKR_RPB1                       5
#define T5CKR_RPB3                       8
#define T5CKR_RPC4                       10
#define T5CKR_RPC13                      7
#define T5CKR_RPD3                       0
#define T5CKR_RPD7                       14
#define T5CKR_RPD11                      3
#define T5CKR_RPD15                      11
#define T5CKR_RPE5                       6
#define T5CKR_RPF0                       4
#define T5CKR_RPF5                       2
#define T5CKR_RPG0                       12
#define T5CKR_RPG7                       1

#define T6CKR_RPA14                      13
#define T6CKR_RPB5                       8
#define T6CKR_RPB9                       5
#define T6CKR_RPB10                      6
#define T6CKR_RPC1                       10
#define T6CKR_RPC14                      7
#define T6CKR_RPD2                       0
#define T6CKR_RPD6                       14
#define T6CKR_RPD10                      3
#define T6CKR_RPD14                      11
#define T6CKR_RPF1                       4
#define T6CKR_RPF4                       2
#define T6CKR_RPG1                       12
#define T6CKR_RPG8                       1

#define T7CKR_RPA15                      13
#define T7CKR_RPB1                       5
#define T7CKR_RPB3                       8
#define T7CKR_RPC4                       10
#define T7CKR_RPC13                      7
#define T7CKR_RPD3                       0
#define T7CKR_RPD7                       14
#define T7CKR_RPD11                      3
#define T7CKR_RPD15                      11
#define T7CKR_RPE5                       6
#define T7CKR_RPF0                       4
#define T7CKR_RPF5                       2
#define T7CKR_RPG0                       12
#define T7CKR_RPG7                       1

#define T8CKR_RPB0                       5
#define T8CKR_RPB7                       7
#define T8CKR_RPB8                       2
#define T8CKR_RPB15                      3
#define T8CKR_RPC3                       12
#define T8CKR_RPD4                       4
#define T8CKR_RPD9                       0
#define T8CKR_RPD12                      10
#define T8CKR_RPE3                       6
#define T8CKR_RPE9                       13
#define T8CKR_RPF8                       11
#define T8CKR_RPF12                      9
#define T8CKR_RPG6                       1

#define T9CKR_RPB2                       7
#define T9CKR_RPB6                       5
#define T9CKR_RPB14                      2
#define T9CKR_RPC2                       12
#define T9CKR_RPD0                       3
#define T9CKR_RPD1                       0
#define T9CKR_RPD5                       6
#define T9CKR_RPE8                       13
#define T9CKR_RPF2                       11
#define T9CKR_RPF3                       8
#define T9CKR_RPF13                      9
#define T9CKR_RPG9                       1

#define U1CTSR_RPB0                      5
#define U1CTSR_RPB7                      7
#define U1CTSR_RPB8                      2
#define U1CTSR_RPB15                     3
#define U1CTSR_RPC3                      12
#define U1CTSR_RPD4                      4
#define U1CTSR_RPD9                      0
#define U1CTSR_RPD12                     10
#define U1CTSR_RPE3                      6
#define U1CTSR_RPE9                      13
#define U1CTSR_RPF8                      11
#define U1CTSR_RPF12                     9
#define U1CTSR_RPG6                      1

#define U1RXR_RPA14                      13
#define U1RXR_RPB5                       8
#define U1RXR_RPB9                       5
#define U1RXR_RPB10                      6
#define U1RXR_RPC1                       10
#define U1RXR_RPC14                      7
#define U1RXR_RPD2                       0
#define U1RXR_RPD6                       14
#define U1RXR_RPD10                      3
#define U1RXR_RPD14                      11
#define U1RXR_RPF1                       4
#define U1RXR_RPF4                       2
#define U1RXR_RPG1                       12
#define U1RXR_RPG8                       1

#define U2CTSR_RPA14                     13
#define U2CTSR_RPB5                      8
#define U2CTSR_RPB9                      5
#define U2CTSR_RPB10                     6
#define U2CTSR_RPC1                      10
#define U2CTSR_RPC14                     7
#define U2CTSR_RPD2                      0
#define U2CTSR_RPD6                      14
#define U2CTSR_RPD10                     3
#define U2CTSR_RPD14                     11
#define U2CTSR_RPF1                      4
#define U2CTSR_RPF4                      2
#define U2CTSR_RPG1                      12
#define U2CTSR_RPG8                      1

#define U2RXR_RPB0                       5
#define U2RXR_RPB7                       7
#define U2RXR_RPB8                       2
#define U2RXR_RPB15                      3
#define U2RXR_RPC3                       12
#define U2RXR_RPD4                       4
#define U2RXR_RPD9                       0
#define U2RXR_RPD12                      10
#define U2RXR_RPE3                       6
#define U2RXR_RPE9                       13
#define U2RXR_RPF8                       11
#define U2RXR_RPF12                      9
#define U2RXR_RPG6                       1

#define U3CTSR_RPB2                      7
#define U3CTSR_RPB6                      5
#define U3CTSR_RPB14                     2
#define U3CTSR_RPC2                      12
#define U3CTSR_RPD0                      3
#define U3CTSR_RPD1                      0
#define U3CTSR_RPD5                      6
#define U3CTSR_RPE8                      13
#define U3CTSR_RPF2                      11
#define U3CTSR_RPF3                      8
#define U3CTSR_RPF13                     9
#define U3CTSR_RPG9                      1

#define U3RXR_RPA15                      13
#define U3RXR_RPB1                       5
#define U3RXR_RPB3                       8
#define U3RXR_RPC4                       10
#define U3RXR_RPC13                      7
#define U3RXR_RPD3                       0
#define U3RXR_RPD7                       14
#define U3RXR_RPD11                      3
#define U3RXR_RPD15                      11
#define U3RXR_RPE5                       6
#define U3RXR_RPF0                       4
#define U3RXR_RPF5                       2
#define U3RXR_RPG0                       12
#define U3RXR_RPG7                       1

#define U4CTSR_RPA15                     13
#define U4CTSR_RPB1                      5
#define U4CTSR_RPB3                      8
#define U4CTSR_RPC4                      10
#define U4CTSR_RPC13                     7
#define U4CTSR_RPD3                      0
#define U4CTSR_RPD7                      14
#define U4CTSR_RPD11                     3
#define U4CTSR_RPD15                     11
#define U4CTSR_RPE5                      6
#define U4CTSR_RPF0                      4
#define U4CTSR_RPF5                      2
#define U4CTSR_RPG0                      12
#define U4CTSR_RPG7                      1

#define U4RXR_RPB2                       7
#define U4RXR_RPB6                       5
#define U4RXR_RPB14                      2
#define U4RXR_RPC2                       12
#define U4RXR_RPD0                       3
#define U4RXR_RPD1                       0
#define U4RXR_RPD5                       6
#define U4RXR_RPE8                       13
#define U4RXR_RPF2                       11
#define U4RXR_RPF3                       8
#define U4RXR_RPF13                      9
#define U4RXR_RPG9                       1

#define U5CTSR_RPB0                      5
#define U5CTSR_RPB7                      7
#define U5CTSR_RPB8                      2
#define U5CTSR_RPB15                     3
#define U5CTSR_RPC3                      12
#define U5CTSR_RPD4                      4
#define U5CTSR_RPD9                      0
#define U5CTSR_RPD12                     10
#define U5CTSR_RPE3                      6
#define U5CTSR_RPE9                      13
#define U5CTSR_RPF8                      11
#define U5CTSR_RPF12                     9
#define U5CTSR_RPG6                      1

#define U5RXR_RPA14                      13
#define U5RXR_RPB5                       8
#define U5RXR_RPB9                       5
#define U5RXR_RPB10                      6
#define U5RXR_RPC1                       10
#define U5RXR_RPC14                      7
#define U5RXR_RPD2                       0
#define U5RXR_RPD6                       14
#define U5RXR_RPD10                      3
#define U5RXR_RPD14                      11
#define U5RXR_RPF1                       4
#define U5RXR_RPF4                       2
#define U5RXR_RPG1                       12
#define U5RXR_RPG8                       1

#define U6CTSR_RPA14                     13
#define U6CTSR_RPB5                      8
#define U6CTSR_RPB9                      5
#define U6CTSR_RPB10                     6
#define U6CTSR_RPC1                      10
#define U6CTSR_RPC14                     7
#define U6CTSR_RPD2                      0
#define U6CTSR_RPD6                      14
#define U6CTSR_RPD10                     3
#define U6CTSR_RPD14                     11
#define U6CTSR_RPF1                      4
#define U6CTSR_RPF4                      2
#define U6CTSR_RPG1                      12
#define U6CTSR_RPG8                      1

#define U6RXR_RPB2                       7
#define U6RXR_RPB6                       5
#define U6RXR_RPB14                      2
#define U6RXR_RPC2                       12
#define U6RXR_RPD0                       3
#define U6RXR_RPD1                       0
#define U6RXR_RPD5                       6
#define U6RXR_RPE8                       13
#define U6RXR_RPF2                       11
#define U6RXR_RPF3                       8
#define U6RXR_RPF13                      9
#define U6RXR_RPG9                       1

/* Output Pin Selection *****************************************************/

/* The encoding of the output pin selection is a little more complex.
 * Knowing the device does not provide sufficient information.
 * So the following definitions include both the register value  and
 * the register address.
 */

#define C1OUT_RPB0R                      14, PIC32MZ_RPB0R
#define C1OUT_RPB7R                      14, PIC32MZ_RPB7R
#define C1OUT_RPB8R                      14, PIC32MZ_RPB8R
#define C1OUT_RPB15R                     14, PIC32MZ_RPB15R
#define C1OUT_RPC3R                      14, PIC32MZ_RPC3R
#define C1OUT_RPD4R                      14, PIC32MZ_RPD4R
#define C1OUT_RPD9R                      14, PIC32MZ_RPD9R
#define C1OUT_RPD12R                     14, PIC32MZ_RPD12R
#define C1OUT_RPE3R                      14, PIC32MZ_RPE3R
#define C1OUT_RPE9R                      14, PIC32MZ_RPE9R
#define C1OUT_RPF8R                      14, PIC32MZ_RPF8R
#define C1OUT_RPF12R                     14, PIC32MZ_RPF12R
#define C1OUT_RPG6R                      14, PIC32MZ_RPG6R

#define C1TX_RPA14R                      15, PIC32MZ_RPA14R
#define C1TX_RPB5R                       15, PIC32MZ_RPB5R
#define C1TX_RPB9R                       15, PIC32MZ_RPB9R
#define C1TX_RPB10R                      15, PIC32MZ_RPB10R
#define C1TX_RPC1R                       15, PIC32MZ_RPC1R
#define C1TX_RPC14R                      15, PIC32MZ_RPC14R
#define C1TX_RPD2R                       15, PIC32MZ_RPD2R
#define C1TX_RPD6R                       15, PIC32MZ_RPD6R
#define C1TX_RPD10R                      15, PIC32MZ_RPD10R
#define C1TX_RPD14R                      15, PIC32MZ_RPD14R
#define C1TX_RPF1R                       15, PIC32MZ_RPF1R
#define C1TX_RPF4R                       15, PIC32MZ_RPF4R
#define C1TX_RPG1R                       15, PIC32MZ_RPG1R
#define C1TX_RPG8R                       15, PIC32MZ_RPG8R

#define C2OUT_RPA14R                     14, PIC32MZ_RPA14R
#define C2OUT_RPB5R                      14, PIC32MZ_RPB5R
#define C2OUT_RPB9R                      14, PIC32MZ_RPB9R
#define C2OUT_RPB10R                     14, PIC32MZ_RPB10R
#define C2OUT_RPC1R                      14, PIC32MZ_RPC1R
#define C2OUT_RPC14R                     14, PIC32MZ_RPC14R
#define C2OUT_RPD2R                      14, PIC32MZ_RPD2R
#define C2OUT_RPD6R                      14, PIC32MZ_RPD6R
#define C2OUT_RPD10R                     14, PIC32MZ_RPD10R
#define C2OUT_RPD14R                     14, PIC32MZ_RPD14R
#define C2OUT_RPF1R                      14, PIC32MZ_RPF1R
#define C2OUT_RPF4R                      14, PIC32MZ_RPF4R
#define C2OUT_RPG1R                      14, PIC32MZ_RPG1R
#define C2OUT_RPG8R                      14, PIC32MZ_RPG8R

#define C2TX_RPB2R                       15, PIC32MZ_RPB2R
#define C2TX_RPB6R                       15, PIC32MZ_RPB6R
#define C2TX_RPB14R                      15, PIC32MZ_RPB14R
#define C2TX_RPC2R                       15, PIC32MZ_RPC2R
#define C2TX_RPD0R                       15, PIC32MZ_RPD0R
#define C2TX_RPD1R                       15, PIC32MZ_RPD1R
#define C2TX_RPD5R                       15, PIC32MZ_RPD5R
#define C2TX_RPE8R                       15, PIC32MZ_RPE8R
#define C2TX_RPF2R                       15, PIC32MZ_RPF2R
#define C2TX_RPF3R                       15, PIC32MZ_RPF3R
#define C2TX_RPF13R                      15, PIC32MZ_RPF13R
#define C2TX_RPG9R                       15, PIC32MZ_RPG9R

#define OC1_RPB2R                        12, PIC32MZ_RPB2R
#define OC1_RPB6R                        12, PIC32MZ_RPB6R
#define OC1_RPB14R                       12, PIC32MZ_RPB14R
#define OC1_RPC2R                        12, PIC32MZ_RPC2R
#define OC1_RPD0R                        12, PIC32MZ_RPD0R
#define OC1_RPD1R                        12, PIC32MZ_RPD1R
#define OC1_RPD5R                        12, PIC32MZ_RPD5R
#define OC1_RPE8R                        12, PIC32MZ_RPE8R
#define OC1_RPF2R                        12, PIC32MZ_RPF2R
#define OC1_RPF3R                        12, PIC32MZ_RPF3R
#define OC1_RPF13R                       12, PIC32MZ_RPF13R
#define OC1_RPG9R                        12, PIC32MZ_RPG9R

#define OC2_RPB2R                        11, PIC32MZ_RPB2R
#define OC2_RPB6R                        11, PIC32MZ_RPB6R
#define OC2_RPB14R                       11, PIC32MZ_RPB14R
#define OC2_RPC2R                        11, PIC32MZ_RPC2R
#define OC2_RPD0R                        11, PIC32MZ_RPD0R
#define OC2_RPD1R                        11, PIC32MZ_RPD1R
#define OC2_RPD5R                        11, PIC32MZ_RPD5R
#define OC2_RPE8R                        11, PIC32MZ_RPE8R
#define OC2_RPF2R                        11, PIC32MZ_RPF2R
#define OC2_RPF3R                        11, PIC32MZ_RPF3R
#define OC2_RPF13R                       11, PIC32MZ_RPF13R
#define OC2_RPG9R                        11, PIC32MZ_RPG9R

#define OC3_RPA14R                       11, PIC32MZ_RPA14R
#define OC3_RPB5R                        11, PIC32MZ_RPB5R
#define OC3_RPB9R                        11, PIC32MZ_RPB9R
#define OC3_RPB10R                       11, PIC32MZ_RPB10R
#define OC3_RPC1R                        11, PIC32MZ_RPC1R
#define OC3_RPC14R                       11, PIC32MZ_RPC14R
#define OC3_RPD2R                        11, PIC32MZ_RPD2R
#define OC3_RPD6R                        11, PIC32MZ_RPD6R
#define OC3_RPD10R                       11, PIC32MZ_RPD10R
#define OC3_RPD14R                       11, PIC32MZ_RPD14R
#define OC3_RPF1R                        11, PIC32MZ_RPF1R
#define OC3_RPF4R                        11, PIC32MZ_RPF4R
#define OC3_RPG1R                        11, PIC32MZ_RPG1R
#define OC3_RPG8R                        11, PIC32MZ_RPG8R

#define OC4_RPA15R                       11, PIC32MZ_RPA15R
#define OC4_RPB1R                        11, PIC32MZ_RPB1R
#define OC4_RPB3R                        11, PIC32MZ_RPB3R
#define OC4_RPC4R                        11, PIC32MZ_RPC4R
#define OC4_RPC13R                       11, PIC32MZ_RPC13R
#define OC4_RPD3R                        11, PIC32MZ_RPD3R
#define OC4_RPD7R                        11, PIC32MZ_RPD7R
#define OC4_RPD11R                       11, PIC32MZ_RPD11R
#define OC4_RPD15R                       11, PIC32MZ_RPD15R
#define OC4_RPE5R                        11, PIC32MZ_RPE5R
#define OC4_RPF0R                        11, PIC32MZ_RPF0R
#define OC4_RPF5R                        11, PIC32MZ_RPF5R
#define OC4_RPG0R                        11, PIC32MZ_RPG0R
#define OC4_RPG7R                        11, PIC32MZ_RPG7R

#define OC5_RPB0R                        11, PIC32MZ_RPB0R
#define OC5_RPB7R                        11, PIC32MZ_RPB7R
#define OC5_RPB8R                        11, PIC32MZ_RPB8R
#define OC5_RPB15R                       11, PIC32MZ_RPB15R
#define OC5_RPC3R                        11, PIC32MZ_RPC3R
#define OC5_RPD4R                        11, PIC32MZ_RPD4R
#define OC5_RPD9R                        11, PIC32MZ_RPD9R
#define OC5_RPD12R                       11, PIC32MZ_RPD12R
#define OC5_RPE3R                        11, PIC32MZ_RPE3R
#define OC5_RPE9R                        11, PIC32MZ_RPE9R
#define OC5_RPF8R                        11, PIC32MZ_RPF8R
#define OC5_RPF12R                       11, PIC32MZ_RPF12R
#define OC5_RPG6R                        11, PIC32MZ_RPG6R

#define OC6_RPA14R                       12, PIC32MZ_RPA14R
#define OC6_RPB5R                        12, PIC32MZ_RPB5R
#define OC6_RPB9R                        12, PIC32MZ_RPB9R
#define OC6_RPB10R                       12, PIC32MZ_RPB10R
#define OC6_RPC1R                        12, PIC32MZ_RPC1R
#define OC6_RPC14R                       12, PIC32MZ_RPC14R
#define OC6_RPD2R                        12, PIC32MZ_RPD2R
#define OC6_RPD6R                        12, PIC32MZ_RPD6R
#define OC6_RPD10R                       12, PIC32MZ_RPD10R
#define OC6_RPD14R                       12, PIC32MZ_RPD14R
#define OC6_RPF1R                        12, PIC32MZ_RPF1R
#define OC6_RPF4R                        12, PIC32MZ_RPF4R
#define OC6_RPG1R                        12, PIC32MZ_RPG1R
#define OC6_RPG8R                        12, PIC32MZ_RPG8R

#define OC7_RPA15R                       12, PIC32MZ_RPA15R
#define OC7_RPB1R                        12, PIC32MZ_RPB1R
#define OC7_RPB3R                        12, PIC32MZ_RPB3R
#define OC7_RPC4R                        12, PIC32MZ_RPC4R
#define OC7_RPC13R                       12, PIC32MZ_RPC13R
#define OC7_RPD3R                        12, PIC32MZ_RPD3R
#define OC7_RPD7R                        12, PIC32MZ_RPD7R
#define OC7_RPD11R                       12, PIC32MZ_RPD11R
#define OC7_RPD15R                       12, PIC32MZ_RPD15R
#define OC7_RPE5R                        12, PIC32MZ_RPE5R
#define OC7_RPF0R                        12, PIC32MZ_RPF0R
#define OC7_RPF5R                        12, PIC32MZ_RPF5R
#define OC7_RPG0R                        12, PIC32MZ_RPG0R
#define OC7_RPG7R                        12, PIC32MZ_RPG7R

#define OC8_RPB0R                        12, PIC32MZ_RPB0R
#define OC8_RPB7R                        12, PIC32MZ_RPB7R
#define OC8_RPB8R                        12, PIC32MZ_RPB8R
#define OC8_RPB15R                       12, PIC32MZ_RPB15R
#define OC8_RPC3R                        12, PIC32MZ_RPC3R
#define OC8_RPD4R                        12, PIC32MZ_RPD4R
#define OC8_RPD9R                        12, PIC32MZ_RPD9R
#define OC8_RPD12R                       12, PIC32MZ_RPD12R
#define OC8_RPE3R                        12, PIC32MZ_RPE3R
#define OC8_RPE9R                        12, PIC32MZ_RPE9R
#define OC8_RPF8R                        12, PIC32MZ_RPF8R
#define OC8_RPF12R                       12, PIC32MZ_RPF12R
#define OC8_RPG6R                        12, PIC32MZ_RPG6R

#define OC9_RPB2R                        13, PIC32MZ_RPB2R
#define OC9_RPB6R                        13, PIC32MZ_RPB6R
#define OC9_RPB14R                       13, PIC32MZ_RPB14R
#define OC9_RPC2R                        13, PIC32MZ_RPC2R
#define OC9_RPD0R                        13, PIC32MZ_RPD0R
#define OC9_RPD1R                        13, PIC32MZ_RPD1R
#define OC9_RPD5R                        13, PIC32MZ_RPD5R
#define OC9_RPE8R                        13, PIC32MZ_RPE8R
#define OC9_RPF2R                        13, PIC32MZ_RPF2R
#define OC9_RPF3R                        13, PIC32MZ_RPF3R
#define OC9_RPF13R                       13, PIC32MZ_RPF13R
#define OC9_RPG9R                        13, PIC32MZ_RPG9R

#define REFCLKO1_RPA15R                  15, PIC32MZ_RPA15R
#define REFCLKO1_RPB1R                   15, PIC32MZ_RPB1R
#define REFCLKO1_RPB3R                   15, PIC32MZ_RPB3R
#define REFCLKO1_RPC4R                   15, PIC32MZ_RPC4R
#define REFCLKO1_RPC13R                  15, PIC32MZ_RPC13R
#define REFCLKO1_RPD3R                   15, PIC32MZ_RPD3R
#define REFCLKO1_RPD7R                   15, PIC32MZ_RPD7R
#define REFCLKO1_RPD11R                  15, PIC32MZ_RPD11R
#define REFCLKO1_RPD15R                  15, PIC32MZ_RPD15R
#define REFCLKO1_RPE5R                   15, PIC32MZ_RPE5R
#define REFCLKO1_RPF0R                   15, PIC32MZ_RPF0R
#define REFCLKO1_RPF5R                   15, PIC32MZ_RPF5R
#define REFCLKO1_RPG0R                   15, PIC32MZ_RPG0R
#define REFCLKO1_RPG7R                   15, PIC32MZ_RPG7R

#define REFCLKO3_RPG6R                   15, PIC32MZ_RPG6R
#define REFCLKO3_RPB0R                   15, PIC32MZ_RPB0R
#define REFCLKO3_RPB7R                   15, PIC32MZ_RPB7R
#define REFCLKO3_RPB8R                   15, PIC32MZ_RPB8R
#define REFCLKO3_RPB15R                  15, PIC32MZ_RPB15R
#define REFCLKO3_RPC3R                   15, PIC32MZ_RPC3R
#define REFCLKO3_RPD4R                   15, PIC32MZ_RPD4R
#define REFCLKO3_RPD9R                   15, PIC32MZ_RPD9R
#define REFCLKO3_RPD12R                  15, PIC32MZ_RPD12R
#define REFCLKO3_RPE3R                   15, PIC32MZ_RPE3R
#define REFCLKO3_RPE9R                   15, PIC32MZ_RPE9R
#define REFCLKO3_RPF8R                   15, PIC32MZ_RPF8R
#define REFCLKO3_RPF12R                  15, PIC32MZ_RPF12R

#define REFCLKO4_RPA14R                  13, PIC32MZ_RPA14R
#define REFCLKO4_RPB5R                   13, PIC32MZ_RPB5R
#define REFCLKO4_RPB9R                   13, PIC32MZ_RPB9R
#define REFCLKO4_RPB10R                  13, PIC32MZ_RPB10R
#define REFCLKO4_RPC1R                   13, PIC32MZ_RPC1R
#define REFCLKO4_RPC14R                  13, PIC32MZ_RPC14R
#define REFCLKO4_RPD2R                   13, PIC32MZ_RPD2R
#define REFCLKO4_RPD6R                   13, PIC32MZ_RPD6R
#define REFCLKO4_RPD10R                  13, PIC32MZ_RPD10R
#define REFCLKO4_RPD14R                  13, PIC32MZ_RPD14R
#define REFCLKO4_RPF1R                   13, PIC32MZ_RPF1R
#define REFCLKO4_RPF4R                   13, PIC32MZ_RPF4R
#define REFCLKO4_RPG1R                   13, PIC32MZ_RPG1R
#define REFCLKO4_RPG8R                   13, PIC32MZ_RPG8R

#define SDO1_RPA14R                      5, PIC32MZ_RPA14R
#define SDO1_RPA15R                      5, PIC32MZ_RPA15R
#define SDO1_RPB1R                       5, PIC32MZ_RPB1R
#define SDO1_RPB3R                       5, PIC32MZ_RPB3R
#define SDO1_RPB5R                       5, PIC32MZ_RPB5R
#define SDO1_RPB9R                       5, PIC32MZ_RPB9R
#define SDO1_RPB10R                      5, PIC32MZ_RPB10R
#define SDO1_RPC1R                       5, PIC32MZ_RPC1R
#define SDO1_RPC4R                       5, PIC32MZ_RPC4R
#define SDO1_RPC13R                      5, PIC32MZ_RPC13R
#define SDO1_RPC14R                      5, PIC32MZ_RPC14R
#define SDO1_RPD2R                       5, PIC32MZ_RPD2R
#define SDO1_RPD3R                       5, PIC32MZ_RPD3R
#define SDO1_RPD6R                       5, PIC32MZ_RPD6R
#define SDO1_RPD7R                       5, PIC32MZ_RPD7R
#define SDO1_RPD10R                      5, PIC32MZ_RPD10R
#define SDO1_RPD11R                      5, PIC32MZ_RPD11R
#define SDO1_RPD14R                      5, PIC32MZ_RPD14R
#define SDO1_RPD15R                      5, PIC32MZ_RPD15R
#define SDO1_RPE5R                       5, PIC32MZ_RPE5R
#define SDO1_RPF0R                       5, PIC32MZ_RPF0R
#define SDO1_RPF1R                       5, PIC32MZ_RPF1R
#define SDO1_RPF4R                       5, PIC32MZ_RPF4R
#define SDO1_RPF5R                       5, PIC32MZ_RPF5R
#define SDO1_RPG0R                       5, PIC32MZ_RPG0R
#define SDO1_RPG1R                       5, PIC32MZ_RPG1R
#define SDO1_RPG7R                       5, PIC32MZ_RPG7R
#define SDO1_RPG8R                       5, PIC32MZ_RPG8R

#define SDO2_RPA14R                      6, PIC32MZ_RPA14R
#define SDO2_RPA15R                      6, PIC32MZ_RPA15R
#define SDO2_RPB1R                       6, PIC32MZ_RPB1R
#define SDO2_RPB3R                       6, PIC32MZ_RPB3R
#define SDO2_RPB5R                       6, PIC32MZ_RPB5R
#define SDO2_RPB9R                       6, PIC32MZ_RPB9R
#define SDO2_RPB10R                      6, PIC32MZ_RPB10R
#define SDO2_RPC1R                       6, PIC32MZ_RPC1R
#define SDO2_RPC4R                       6, PIC32MZ_RPC4R
#define SDO2_RPC13R                      6, PIC32MZ_RPC13R
#define SDO2_RPC14R                      6, PIC32MZ_RPC14R
#define SDO2_RPD2R                       6, PIC32MZ_RPD2R
#define SDO2_RPD3R                       6, PIC32MZ_RPD3R
#define SDO2_RPD6R                       6, PIC32MZ_RPD6R
#define SDO2_RPD7R                       6, PIC32MZ_RPD7R
#define SDO2_RPD10R                      6, PIC32MZ_RPD10R
#define SDO2_RPD11R                      6, PIC32MZ_RPD11R
#define SDO2_RPD14R                      6, PIC32MZ_RPD14R
#define SDO2_RPD15R                      6, PIC32MZ_RPD15R
#define SDO2_RPE5R                       6, PIC32MZ_RPE5R
#define SDO2_RPF0R                       6, PIC32MZ_RPF0R
#define SDO2_RPF1R                       6, PIC32MZ_RPF1R
#define SDO2_RPF4R                       6, PIC32MZ_RPF4R
#define SDO2_RPF5R                       6, PIC32MZ_RPF5R
#define SDO2_RPG0R                       6, PIC32MZ_RPG0R
#define SDO2_RPG1R                       6, PIC32MZ_RPG1R
#define SDO2_RPG7R                       6, PIC32MZ_RPG7R
#define SDO2_RPG8R                       6, PIC32MZ_RPG8R

#define SDO3_RPA14R                      7, PIC32MZ_RPA14R
#define SDO3_RPA15R                      7, PIC32MZ_RPA15R
#define SDO3_RPB1R                       7, PIC32MZ_RPB1R
#define SDO3_RPB3R                       7, PIC32MZ_RPB3R
#define SDO3_RPB5R                       7, PIC32MZ_RPB5R
#define SDO3_RPB9R                       7, PIC32MZ_RPB9R
#define SDO3_RPB10R                      7, PIC32MZ_RPB10R
#define SDO3_RPC1R                       7, PIC32MZ_RPC1R
#define SDO3_RPC4R                       7, PIC32MZ_RPC4R
#define SDO3_RPC13R                      7, PIC32MZ_RPC13R
#define SDO3_RPC14R                      7, PIC32MZ_RPC14R
#define SDO3_RPD2R                       7, PIC32MZ_RPD2R
#define SDO3_RPD3R                       7, PIC32MZ_RPD3R
#define SDO3_RPD6R                       7, PIC32MZ_RPD6R
#define SDO3_RPD7R                       7, PIC32MZ_RPD7R
#define SDO3_RPD10R                      7, PIC32MZ_RPD10R
#define SDO3_RPD11R                      7, PIC32MZ_RPD11R
#define SDO3_RPD14R                      7, PIC32MZ_RPD14R
#define SDO3_RPD15R                      7, PIC32MZ_RPD15R
#define SDO3_RPE5R                       7, PIC32MZ_RPE5R
#define SDO3_RPF0R                       7, PIC32MZ_RPF0R
#define SDO3_RPF1R                       7, PIC32MZ_RPF1R
#define SDO3_RPF4R                       7, PIC32MZ_RPF4R
#define SDO3_RPF5R                       7, PIC32MZ_RPF5R
#define SDO3_RPG0R                       7, PIC32MZ_RPG0R
#define SDO3_RPG1R                       7, PIC32MZ_RPG1R
#define SDO3_RPG7R                       7, PIC32MZ_RPG7R
#define SDO3_RPG8R                       7, PIC32MZ_RPG8R

#define SDO4_RPA15R                      8, PIC32MZ_RPA15R
#define SDO4_RPB1R                       8, PIC32MZ_RPB1R
#define SDO4_RPB2R                       8, PIC32MZ_RPB2R
#define SDO4_RPB3R                       8, PIC32MZ_RPB3R
#define SDO4_RPB6R                       8, PIC32MZ_RPB6R
#define SDO4_RPB14R                      8, PIC32MZ_RPB14R
#define SDO4_RPC2R                       8, PIC32MZ_RPC2R
#define SDO4_RPC4R                       8, PIC32MZ_RPC4R
#define SDO4_RPC13R                      8, PIC32MZ_RPC13R
#define SDO4_RPD0R                       8, PIC32MZ_RPD0R
#define SDO4_RPD1R                       8, PIC32MZ_RPD1R
#define SDO4_RPD3R                       8, PIC32MZ_RPD3R
#define SDO4_RPD5R                       8, PIC32MZ_RPD5R
#define SDO4_RPD7R                       8, PIC32MZ_RPD7R
#define SDO4_RPD11R                      8, PIC32MZ_RPD11R
#define SDO4_RPD15R                      8, PIC32MZ_RPD15R
#define SDO4_RPE5R                       8, PIC32MZ_RPE5R
#define SDO4_RPE8R                       8, PIC32MZ_RPE8R
#define SDO4_RPF0R                       8, PIC32MZ_RPF0R
#define SDO4_RPF2R                       8, PIC32MZ_RPF2R
#define SDO4_RPF3R                       8, PIC32MZ_RPF3R
#define SDO4_RPF5R                       8, PIC32MZ_RPF5R
#define SDO4_RPF13R                      8, PIC32MZ_RPF13R
#define SDO4_RPG0R                       8, PIC32MZ_RPG0R
#define SDO4_RPG7R                       8, PIC32MZ_RPG7R
#define SDO4_RPG9R                       8, PIC32MZ_RPG9R

#define SDO5_RPA14R                      9, PIC32MZ_RPA14R
#define SDO5_RPA15R                      9, PIC32MZ_RPA15R
#define SDO5_RPB1R                       9, PIC32MZ_RPB1R
#define SDO5_RPB3R                       9, PIC32MZ_RPB3R
#define SDO5_RPB5R                       9, PIC32MZ_RPB5R
#define SDO5_RPB9R                       9, PIC32MZ_RPB9R
#define SDO5_RPB10R                      9, PIC32MZ_RPB10R
#define SDO5_RPC1R                       9, PIC32MZ_RPC1R
#define SDO5_RPC4R                       9, PIC32MZ_RPC4R
#define SDO5_RPC13R                      9, PIC32MZ_RPC13R
#define SDO5_RPC14R                      9, PIC32MZ_RPC14R
#define SDO5_RPD2R                       9, PIC32MZ_RPD2R
#define SDO5_RPD3R                       9, PIC32MZ_RPD3R
#define SDO5_RPD6R                       9, PIC32MZ_RPD6R
#define SDO5_RPD7R                       9, PIC32MZ_RPD7R
#define SDO5_RPD10R                      9, PIC32MZ_RPD10R
#define SDO5_RPD11R                      9, PIC32MZ_RPD11R
#define SDO5_RPD14R                      9, PIC32MZ_RPD14R
#define SDO5_RPD15R                      9, PIC32MZ_RPD15R
#define SDO5_RPE5R                       9, PIC32MZ_RPE5R
#define SDO5_RPF0R                       9, PIC32MZ_RPF0R
#define SDO5_RPF1R                       9, PIC32MZ_RPF1R
#define SDO5_RPF4R                       9, PIC32MZ_RPF4R
#define SDO5_RPF5R                       9, PIC32MZ_RPF5R
#define SDO5_RPG0R                       9, PIC32MZ_RPG0R
#define SDO5_RPG1R                       9, PIC32MZ_RPG1R
#define SDO5_RPG7R                       9, PIC32MZ_RPG7R
#define SDO5_RPG8R                       9, PIC32MZ_RPG8R

#define SDO6_RPB0R                       10, PIC32MZ_RPB0R
#define SDO6_RPB2R                       10, PIC32MZ_RPB2R
#define SDO6_RPB6R                       10, PIC32MZ_RPB6R
#define SDO6_RPB7R                       10, PIC32MZ_RPB7R
#define SDO6_RPB8R                       10, PIC32MZ_RPB8R
#define SDO6_RPB14R                      10, PIC32MZ_RPB14R
#define SDO6_RPB15R                      10, PIC32MZ_RPB15R
#define SDO6_RPC2R                       10, PIC32MZ_RPC2R
#define SDO6_RPC3R                       10, PIC32MZ_RPC3R
#define SDO6_RPD0R                       10, PIC32MZ_RPD0R
#define SDO6_RPD1R                       10, PIC32MZ_RPD1R
#define SDO6_RPD4R                       10, PIC32MZ_RPD4R
#define SDO6_RPD5R                       10, PIC32MZ_RPD5R
#define SDO6_RPD9R                       10, PIC32MZ_RPD9R
#define SDO6_RPD12R                      10, PIC32MZ_RPD12R
#define SDO6_RPE3R                       10, PIC32MZ_RPE3R
#define SDO6_RPE8R                       10, PIC32MZ_RPE8R
#define SDO6_RPE9R                       10, PIC32MZ_RPE9R
#define SDO6_RPF2R                       10, PIC32MZ_RPF2R
#define SDO6_RPF3R                       10, PIC32MZ_RPF3R
#define SDO6_RPF8R                       10, PIC32MZ_RPF8R
#define SDO6_RPF12R                      10, PIC32MZ_RPF12R
#define SDO6_RPF13R                      10, PIC32MZ_RPF13R
#define SDO6_RPG6R                       10, PIC32MZ_RPG6R
#define SDO6_RPG9R                       10, PIC32MZ_RPG9R

#define SS1_RPB0R                        5, PIC32MZ_RPB0R
#define SS1_RPB7R                        5, PIC32MZ_RPB7R
#define SS1_RPB8R                        5, PIC32MZ_RPB8R
#define SS1_RPB15R                       5, PIC32MZ_RPB15R
#define SS1_RPC3R                        5, PIC32MZ_RPC3R
#define SS1_RPD4R                        5, PIC32MZ_RPD4R
#define SS1_RPD9R                        5, PIC32MZ_RPD9R
#define SS1_RPD12R                       5, PIC32MZ_RPD12R
#define SS1_RPE3R                        5, PIC32MZ_RPE3R
#define SS1_RPE9R                        5, PIC32MZ_RPE9R
#define SS1_RPF8R                        5, PIC32MZ_RPF8R
#define SS1_RPF12R                       5, PIC32MZ_RPF12R
#define SS1_RPG6R                        5, PIC32MZ_RPG6R

#define SS2_RPB2R                        6, PIC32MZ_RPB2R
#define SS2_RPB6R                        6, PIC32MZ_RPB6R
#define SS2_RPB14R                       6, PIC32MZ_RPB14R
#define SS2_RPC2R                        6, PIC32MZ_RPC2R
#define SS2_RPD0R                        6, PIC32MZ_RPD0R
#define SS2_RPD1R                        6, PIC32MZ_RPD1R
#define SS2_RPD5R                        6, PIC32MZ_RPD5R
#define SS2_RPE8R                        6, PIC32MZ_RPE8R
#define SS2_RPF2R                        6, PIC32MZ_RPF2R
#define SS2_RPF3R                        6, PIC32MZ_RPF3R
#define SS2_RPF13R                       6, PIC32MZ_RPF13R
#define SS2_RPG9R                        6, PIC32MZ_RPG9R

#define SS3_RPB0R                        7, PIC32MZ_RPB0R
#define SS3_RPB7R                        7, PIC32MZ_RPB7R
#define SS3_RPB8R                        7, PIC32MZ_RPB8R
#define SS3_RPB15R                       7, PIC32MZ_RPB15R
#define SS3_RPC3R                        7, PIC32MZ_RPC3R
#define SS3_RPD4R                        7, PIC32MZ_RPD4R
#define SS3_RPD9R                        7, PIC32MZ_RPD9R
#define SS3_RPD12R                       7, PIC32MZ_RPD12R
#define SS3_RPE3R                        7, PIC32MZ_RPE3R
#define SS3_RPE9R                        7, PIC32MZ_RPE9R
#define SS3_RPF8R                        7, PIC32MZ_RPF8R
#define SS3_RPF12R                       7, PIC32MZ_RPF12R
#define SS3_RPG6R                        7, PIC32MZ_RPG6R

#define SS4_RPB0R                        8, PIC32MZ_RPB0R
#define SS4_RPB7R                        8, PIC32MZ_RPB7R
#define SS4_RPB8R                        8, PIC32MZ_RPB8R
#define SS4_RPB15R                       8, PIC32MZ_RPB15R
#define SS4_RPC3R                        8, PIC32MZ_RPC3R
#define SS4_RPD4R                        8, PIC32MZ_RPD4R
#define SS4_RPD9R                        8, PIC32MZ_RPD9R
#define SS4_RPD12R                       8, PIC32MZ_RPD12R
#define SS4_RPE3R                        8, PIC32MZ_RPE3R
#define SS4_RPE9R                        8, PIC32MZ_RPE9R
#define SS4_RPF8R                        8, PIC32MZ_RPF8R
#define SS4_RPF12R                       8, PIC32MZ_RPF12R
#define SS4_RPG6R                        8, PIC32MZ_RPG6R

#define SS5_RPB0R                        9, PIC32MZ_RPB0R
#define SS5_RPB7R                        9, PIC32MZ_RPB7R
#define SS5_RPB8R                        9, PIC32MZ_RPB8R
#define SS5_RPB15R                       9, PIC32MZ_RPB15R
#define SS5_RPC3R                        9, PIC32MZ_RPC3R
#define SS5_RPD4R                        9, PIC32MZ_RPD4R
#define SS5_RPD9R                        9, PIC32MZ_RPD9R
#define SS5_RPD12R                       9, PIC32MZ_RPD12R
#define SS5_RPE3R                        9, PIC32MZ_RPE3R
#define SS5_RPE9R                        9, PIC32MZ_RPE9R
#define SS5_RPF8R                        9, PIC32MZ_RPF8R
#define SS5_RPF12R                       9, PIC32MZ_RPF12R
#define SS5_RPG6R                        9, PIC32MZ_RPG6R

#define SS6_RPA14R                       10, PIC32MZ_RPA14R
#define SS6_RPB5R                        10, PIC32MZ_RPB5R
#define SS6_RPB9R                        10, PIC32MZ_RPB9R
#define SS6_RPB10R                       10, PIC32MZ_RPB10R
#define SS6_RPC1R                        10, PIC32MZ_RPC1R
#define SS6_RPC14R                       10, PIC32MZ_RPC14R
#define SS6_RPD2R                        10, PIC32MZ_RPD2R
#define SS6_RPD6R                        10, PIC32MZ_RPD6R
#define SS6_RPD10R                       10, PIC32MZ_RPD10R
#define SS6_RPD14R                       10, PIC32MZ_RPD14R
#define SS6_RPF1R                        10, PIC32MZ_RPF1R
#define SS6_RPF4R                        10, PIC32MZ_RPF4R
#define SS6_RPG1R                        10, PIC32MZ_RPG1R
#define SS6_RPG8R                        10, PIC32MZ_RPG8R

#define U1RTS_RPB2R                      1, PIC32MZ_RPB2R
#define U1RTS_RPB6R                      1, PIC32MZ_RPB6R
#define U1RTS_RPB14R                     1, PIC32MZ_RPB14R
#define U1RTS_RPC2R                      1, PIC32MZ_RPC2R
#define U1RTS_RPD0R                      1, PIC32MZ_RPD0R
#define U1RTS_RPD1R                      1, PIC32MZ_RPD1R
#define U1RTS_RPD5R                      1, PIC32MZ_RPD5R
#define U1RTS_RPE8R                      1, PIC32MZ_RPE8R
#define U1RTS_RPF2R                      1, PIC32MZ_RPF2R
#define U1RTS_RPF3R                      1, PIC32MZ_RPF3R
#define U1RTS_RPF13R                     1, PIC32MZ_RPF13R
#define U1RTS_RPG9R                      1, PIC32MZ_RPG9R

#define U1TX_RPA15R                      1, PIC32MZ_RPA15R
#define U1TX_RPB1R                       1, PIC32MZ_RPB1R
#define U1TX_RPB3R                       1, PIC32MZ_RPB3R
#define U1TX_RPC4R                       1, PIC32MZ_RPC4R
#define U1TX_RPC13R                      1, PIC32MZ_RPC13R
#define U1TX_RPD3R                       1, PIC32MZ_RPD3R
#define U1TX_RPD7R                       1, PIC32MZ_RPD7R
#define U1TX_RPD11R                      1, PIC32MZ_RPD11R
#define U1TX_RPD15R                      1, PIC32MZ_RPD15R
#define U1TX_RPE5R                       1, PIC32MZ_RPE5R
#define U1TX_RPF0R                       1, PIC32MZ_RPF0R
#define U1TX_RPF5R                       1, PIC32MZ_RPF5R
#define U1TX_RPG0R                       1, PIC32MZ_RPG0R
#define U1TX_RPG7R                       1, PIC32MZ_RPG7R

#define U2RTS_RPA15R                     2, PIC32MZ_RPA15R
#define U2RTS_RPB1R                      2, PIC32MZ_RPB1R
#define U2RTS_RPB3R                      2, PIC32MZ_RPB3R
#define U2RTS_RPC4R                      2, PIC32MZ_RPC4R
#define U2RTS_RPC13R                     2, PIC32MZ_RPC13R
#define U2RTS_RPD3R                      2, PIC32MZ_RPD3R
#define U2RTS_RPD7R                      2, PIC32MZ_RPD7R
#define U2RTS_RPD11R                     2, PIC32MZ_RPD11R
#define U2RTS_RPD15R                     2, PIC32MZ_RPD15R
#define U2RTS_RPE5R                      2, PIC32MZ_RPE5R
#define U2RTS_RPF0R                      2, PIC32MZ_RPF0R
#define U2RTS_RPF5R                      2, PIC32MZ_RPF5R
#define U2RTS_RPG0R                      2, PIC32MZ_RPG0R
#define U2RTS_RPG7R                      2, PIC32MZ_RPG7R

#define U2TX_RPB2R                       2, PIC32MZ_RPB2R
#define U2TX_RPB6R                       2, PIC32MZ_RPB6R
#define U2TX_RPB14R                      2, PIC32MZ_RPB14R
#define U2TX_RPC2R                       2, PIC32MZ_RPC2R
#define U2TX_RPD0R                       2, PIC32MZ_RPD0R
#define U2TX_RPD1R                       2, PIC32MZ_RPD1R
#define U2TX_RPD5R                       2, PIC32MZ_RPD5R
#define U2TX_RPE8R                       2, PIC32MZ_RPE8R
#define U2TX_RPF2R                       2, PIC32MZ_RPF2R
#define U2TX_RPF3R                       2, PIC32MZ_RPF3R
#define U2TX_RPF13R                      2, PIC32MZ_RPF13R
#define U2TX_RPG9R                       2, PIC32MZ_RPG9R

#define U3RTS_RPB0R                      1, PIC32MZ_RPB0R
#define U3RTS_RPB7R                      1, PIC32MZ_RPB7R
#define U3RTS_RPB8R                      1, PIC32MZ_RPB8R
#define U3RTS_RPB15R                     1, PIC32MZ_RPB15R
#define U3RTS_RPC3R                      1, PIC32MZ_RPC3R
#define U3RTS_RPD4R                      1, PIC32MZ_RPD4R
#define U3RTS_RPD9R                      1, PIC32MZ_RPD9R
#define U3RTS_RPD12R                     1, PIC32MZ_RPD12R
#define U3RTS_RPE3R                      1, PIC32MZ_RPE3R
#define U3RTS_RPE9R                      1, PIC32MZ_RPE9R
#define U3RTS_RPF8R                      1, PIC32MZ_RPF8R
#define U3RTS_RPF12R                     1, PIC32MZ_RPF12R
#define U3RTS_RPG6R                      1, PIC32MZ_RPG6R

#define U3TX_RPA14R                      1, PIC32MZ_RPA14R
#define U3TX_RPB5R                       1, PIC32MZ_RPB5R
#define U3TX_RPB9R                       1, PIC32MZ_RPB9R
#define U3TX_RPB10R                      1, PIC32MZ_RPB10R
#define U3TX_RPC1R                       1, PIC32MZ_RPC1R
#define U3TX_RPC14R                      1, PIC32MZ_RPC14R
#define U3TX_RPD2R                       1, PIC32MZ_RPD2R
#define U3TX_RPD6R                       1, PIC32MZ_RPD6R
#define U3TX_RPD10R                      1, PIC32MZ_RPD10R
#define U3TX_RPD14R                      1, PIC32MZ_RPD14R
#define U3TX_RPF1R                       1, PIC32MZ_RPF1R
#define U3TX_RPF4R                       1, PIC32MZ_RPF4R
#define U3TX_RPG1R                       1, PIC32MZ_RPG1R
#define U3TX_RPG8R                       1, PIC32MZ_RPG8R

#define U4RTS_RPA14R                     2, PIC32MZ_RPA14R
#define U4RTS_RPB5R                      2, PIC32MZ_RPB5R
#define U4RTS_RPB9R                      2, PIC32MZ_RPB9R
#define U4RTS_RPB10R                     2, PIC32MZ_RPB10R
#define U4RTS_RPC1R                      2, PIC32MZ_RPC1R
#define U4RTS_RPC14R                     2, PIC32MZ_RPC14R
#define U4RTS_RPD2R                      2, PIC32MZ_RPD2R
#define U4RTS_RPD6R                      2, PIC32MZ_RPD6R
#define U4RTS_RPD10R                     2, PIC32MZ_RPD10R
#define U4RTS_RPD14R                     2, PIC32MZ_RPD14R
#define U4RTS_RPF1R                      2, PIC32MZ_RPF1R
#define U4RTS_RPF4R                      2, PIC32MZ_RPF4R
#define U4RTS_RPG1R                      2, PIC32MZ_RPG1R
#define U4RTS_RPG8R                      2, PIC32MZ_RPG8R

#define U4TX_RPB0R                       2, PIC32MZ_RPB0R
#define U4TX_RPB7R                       2, PIC32MZ_RPB7R
#define U4TX_RPB8R                       2, PIC32MZ_RPB8R
#define U4TX_RPB15R                      2, PIC32MZ_RPB15R
#define U4TX_RPC3R                       2, PIC32MZ_RPC3R
#define U4TX_RPD4R                       2, PIC32MZ_RPD4R
#define U4TX_RPD9R                       2, PIC32MZ_RPD9R
#define U4TX_RPD12R                      2, PIC32MZ_RPD12R
#define U4TX_RPE3R                       2, PIC32MZ_RPE3R
#define U4TX_RPE9R                       2, PIC32MZ_RPE9R
#define U4TX_RPF8R                       2, PIC32MZ_RPF8R
#define U4TX_RPF12R                      2, PIC32MZ_RPF12R
#define U4TX_RPG6R                       2, PIC32MZ_RPG6R

#define U5RTS_RPB2R                      3, PIC32MZ_RPB2R
#define U5RTS_RPB6R                      3, PIC32MZ_RPB6R
#define U5RTS_RPB14R                     3, PIC32MZ_RPB14R
#define U5RTS_RPC2R                      3, PIC32MZ_RPC2R
#define U5RTS_RPD0R                      3, PIC32MZ_RPD0R
#define U5RTS_RPD1R                      3, PIC32MZ_RPD1R
#define U5RTS_RPD5R                      3, PIC32MZ_RPD5R
#define U5RTS_RPE8R                      3, PIC32MZ_RPE8R
#define U5RTS_RPF2R                      3, PIC32MZ_RPF2R
#define U5RTS_RPF3R                      3, PIC32MZ_RPF3R
#define U5RTS_RPF13R                     3, PIC32MZ_RPF13R
#define U5RTS_RPG9R                      3, PIC32MZ_RPG9R

#define U5TX_RPA15R                      3, PIC32MZ_RPA15R
#define U5TX_RPB1R                       3, PIC32MZ_RPB1R
#define U5TX_RPB3R                       3, PIC32MZ_RPB3R
#define U5TX_RPC4R                       3, PIC32MZ_RPC4R
#define U5TX_RPC13R                      3, PIC32MZ_RPC13R
#define U5TX_RPD3R                       3, PIC32MZ_RPD3R
#define U5TX_RPD7R                       3, PIC32MZ_RPD7R
#define U5TX_RPD11R                      3, PIC32MZ_RPD11R
#define U5TX_RPD15R                      3, PIC32MZ_RPD15R
#define U5TX_RPE5R                       3, PIC32MZ_RPE5R
#define U5TX_RPF0R                       3, PIC32MZ_RPF0R
#define U5TX_RPF5R                       3, PIC32MZ_RPF5R
#define U5TX_RPG0R                       3, PIC32MZ_RPG0R
#define U5TX_RPG7R                       3, PIC32MZ_RPG7R

#define U6RTS_RPA15R                     4, PIC32MZ_RPA15R
#define U6RTS_RPB1R                      4, PIC32MZ_RPB1R
#define U6RTS_RPB3R                      4, PIC32MZ_RPB3R
#define U6RTS_RPC4R                      4, PIC32MZ_RPC4R
#define U6RTS_RPC13R                     4, PIC32MZ_RPC13R
#define U6RTS_RPD3R                      4, PIC32MZ_RPD3R
#define U6RTS_RPD7R                      4, PIC32MZ_RPD7R
#define U6RTS_RPD11R                     4, PIC32MZ_RPD11R
#define U6RTS_RPD15R                     4, PIC32MZ_RPD15R
#define U6RTS_RPE5R                      4, PIC32MZ_RPE5R
#define U6RTS_RPF0R                      4, PIC32MZ_RPF0R
#define U6RTS_RPF5R                      4, PIC32MZ_RPF5R
#define U6RTS_RPG0R                      4, PIC32MZ_RPG0R
#define U6RTS_RPG7R                      4, PIC32MZ_RPG7R

#define U6TX_RPB0R                       4, PIC32MZ_RPB0R
#define U6TX_RPB2R                       4, PIC32MZ_RPB2R
#define U6TX_RPB6R                       4, PIC32MZ_RPB6R
#define U6TX_RPB7R                       4, PIC32MZ_RPB7R
#define U6TX_RPB8R                       4, PIC32MZ_RPB8R
#define U6TX_RPB14R                      4, PIC32MZ_RPB14R
#define U6TX_RPB15R                      4, PIC32MZ_RPB15R
#define U6TX_RPC2R                       4, PIC32MZ_RPC2R
#define U6TX_RPC3R                       4, PIC32MZ_RPC3R
#define U6TX_RPD0R                       4, PIC32MZ_RPD0R
#define U6TX_RPD1R                       4, PIC32MZ_RPD1R
#define U6TX_RPD4R                       4, PIC32MZ_RPD4R
#define U6TX_RPD5R                       4, PIC32MZ_RPD5R
#define U6TX_RPD9R                       4, PIC32MZ_RPD9R
#define U6TX_RPD12R                      4, PIC32MZ_RPD12R
#define U6TX_RPE3R                       4, PIC32MZ_RPE3R
#define U6TX_RPE8R                       4, PIC32MZ_RPE8R
#define U6TX_RPE9R                       4, PIC32MZ_RPE9R
#define U6TX_RPF2R                       4, PIC32MZ_RPF2R
#define U6TX_RPF3R                       4, PIC32MZ_RPF3R
#define U6TX_RPF8R                       4, PIC32MZ_RPF8R
#define U6TX_RPF12R                      4, PIC32MZ_RPF12R
#define U6TX_RPF13R                      4, PIC32MZ_RPF13R
#define U6TX_RPG6R                       4, PIC32MZ_RPG6R
#define U6TX_RPG9R                       4, PIC32MZ_RPG9R

#endif /* __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZEC_PPS_H */
