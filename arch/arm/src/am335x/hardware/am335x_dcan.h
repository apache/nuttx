/****************************************************************************
 * arch/arm/src/am335x/hardware/am335x_dcan.h
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

#ifndef __ARCH_ARM_SRC_AM335X_HARDWARE_AM335X_DCAN_H
#define __ARCH_ARM_SRC_AM335X_HARDWARE_AM335X_DCAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <hardware/am335x_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define AM335X_DCAN_CTL_OFFSET          0x0000 /* CAN Control Register */
#define AM335X_DCAN_ES_OFFSET           0x0004 /* Error and Status Register */
#define AM335X_DCAN_ERRC_OFFSET         0x0008 /* Error Counter Register */
#define AM335X_DCAN_BTR_OFFSET          0x000c /* Bit Timing Register */
#define AM335X_DCAN_INT_OFFSET          0x0010 /* Interrupt Register */
#define AM335X_DCAN_TEST_OFFSET         0x0014 /* Test Register */
#define AM335X_DCAN_PERR_OFFSET         0x001c /* Parity Error Code Register */
#define AM335X_DCAN_ABOTR_OFFSET        0x0080 /* Auto-Bus-On Time Register */
#define AM335X_DCAN_TXRQ_X_OFFSET       0x0084 /* Transmission Request X Register */
#define AM335X_DCAN_TXRQ12_OFFSET       0x0088 /* Transmission Request Register 12 */
#define AM335X_DCAN_TXRQ34_OFFSET       0x008c /* Transmission Request Register 34 */
#define AM335X_DCAN_TXRQ56_OFFSET       0x0090 /* Transmission Request Register 56 */
#define AM335X_DCAN_TXRQ78_OFFSET       0x0094 /* Transmission Request Register 78 */
#define AM335X_DCAN_NWDAT_X_OFFSET      0x0098 /* New Data X Register */
#define AM335X_DCAN_NWDAT12_OFFSET      0x009c /* New Data Register 12 */
#define AM335X_DCAN_NWDAT34_OFFSET      0x00a0 /* New Data Register 34 */
#define AM335X_DCAN_NWDAT56_OFFSET      0x00a4 /* New Data Register 56 */
#define AM335X_DCAN_NWDAT78_OFFSET      0x00a8 /* New Data Register 78 */
#define AM335X_DCAN_INTPND_X_OFFSET     0x00ac /* Interrupt Pending X Register */
#define AM335X_DCAN_INTPND12_OFFSET     0x00b0 /* Interrupt Pending Register 12 */
#define AM335X_DCAN_INTPND34_OFFSET     0x00b4 /* Interrupt Pending Register 34 */
#define AM335X_DCAN_INTPND56_OFFSET     0x00b8 /* Interrupt Pending Register 56 */
#define AM335X_DCAN_INTPND78_OFFSET     0x00bc /* Interrupt Pending Register 78 */
#define AM335X_DCAN_MSGVAL_X_OFFSET     0x00c0 /* Message Valid X Register */
#define AM335X_DCAN_MSGVAL12_OFFSET     0x00c4 /* Message Valid Register 12 */
#define AM335X_DCAN_MSGVAL34_OFFSET     0x00c8 /* Message Valid Register 34 */
#define AM335X_DCAN_MSGVAL56_OFFSET     0x00cc /* Message Valid Register 56 */
#define AM335X_DCAN_MSGVAL78_OFFSET     0x00d0 /* Message Valid Register 78 */
#define AM335X_DCAN_INTMUX12_OFFSET     0x00d8 /* Interrupt Multiplexer Register 12 */
#define AM335X_DCAN_INTMUX34_OFFSET     0x00dc /* Interrupt Multiplexer Register 34 */
#define AM335X_DCAN_INTMUX56_OFFSET     0x00e0 /* Interrupt Multiplexer Register 56 */
#define AM335X_DCAN_INTMUX78_OFFSET     0x00e4 /* Interrupt Multiplexer Register 78 */
#define AM335X_DCAN_IF1CMD_OFFSET       0x0100 /* IF1 Command Registers */
#define AM335X_DCAN_IF1MSK_OFFSET       0x0104 /* IF1 Mask Register */
#define AM335X_DCAN_IF1ARB_OFFSET       0x0108 /* IF1 Arbitration Register */
#define AM335X_DCAN_IF1MCTL_OFFSET      0x010c /* IF1 Message Control Register */
#define AM335X_DCAN_IF1DATA_OFFSET      0x0110 /* IF1 Data A Register */
#define AM335X_DCAN_IF1DATB_OFFSET      0x0114 /* IF1 Data B Register */
#define AM335X_DCAN_IF2CMD_OFFSET       0x0120 /* IF2 Command Registers */
#define AM335X_DCAN_IF2MSK_OFFSET       0x0124 /* IF2 Mask Register */
#define AM335X_DCAN_IF2ARB_OFFSET       0x0128 /* IF2 Arbitration Register */
#define AM335X_DCAN_IF2MCTL_OFFSET      0x012c /* IF2 Message Control Register */
#define AM335X_DCAN_IF2DATA_OFFSET      0x0130 /* IF2 Data A Register */
#define AM335X_DCAN_IF2DATB_OFFSET      0x0134 /* IF2 Data B Register */
#define AM335X_DCAN_IF3OBS_OFFSET       0x0140 /* IF3 Observation Register */
#define AM335X_DCAN_IF3MSK_OFFSET       0x0144 /* IF3 Mask Register */
#define AM335X_DCAN_IF3ARB_OFFSET       0x0148 /* IF3 Arbitration Register */
#define AM335X_DCAN_IF3MCTL_OFFSET      0x014c /* IF3 Message Control Register */
#define AM335X_DCAN_IF3DATA_OFFSET      0x0150 /* IF3 Data A Register */
#define AM335X_DCAN_IF3DATB_OFFSET      0x0154 /* IF3 Data B Register */
#define AM335X_DCAN_IF3UPD12_OFFSET     0x0160 /* IF3 Update Enable Register 12 */
#define AM335X_DCAN_IF3UPD34_OFFSET     0x0164 /* IF3 Update Enable Register 34 */
#define AM335X_DCAN_IF3UPD56_OFFSET     0x0168 /* IF3 Update Enable Register 56 */
#define AM335X_DCAN_IF3UPD78_OFFSET     0x016c /* IF3 Update Enable Register 78 */
#define AM335X_DCAN_TIOC_OFFSET         0x01e0 /* CAN TX IO Control Register */
#define AM335X_DCAN_RIOC_OFFSET         0x01e4 /* CAN RX IO Control Register */

#define AM335X_DCAN_TXRQ_OFFSET(n)      (0x0088 + ((((unsigned int)(n) - 1) >> 5) << 2))
#define AM335X_DCAN_NWDAT_OFFSET(n)     (0x009c + ((((unsigned int)(n) - 1) >> 5) << 2))
#define AM335X_DCAN_INTPND_OFFSET(n)    (0x00b0 + ((((unsigned int)(n) - 1) >> 5) << 2))
#define AM335X_DCAN_MSGVAL_OFFSET(n)    (0x00c4 + ((((unsigned int)(n) - 1) >> 5) << 2))
#define AM335X_DCAN_INTMUX_OFFSET(n)    (0x00d8 + ((((unsigned int)(n) - 1) >> 5) << 2))
#define AM335X_DCAN_IFCMD_OFFSET(n)     (0x0100 + ((unsigned int)(n) - 1) * 0x20)
#define AM335X_DCAN_IFMSK_OFFSET(n)     (0x0104 + ((unsigned int)(n) - 1) * 0x20)
#define AM335X_DCAN_IFARB_OFFSET(n)     (0x0108 + ((unsigned int)(n) - 1) * 0x20)
#define AM335X_DCAN_IFMCTL_OFFSET(n)    (0x010c + ((unsigned int)(n) - 1) * 0x20)
#define AM335X_DCAN_IFDATA_OFFSET(n)    (0x0110 + ((unsigned int)(n) - 1) * 0x20)
#define AM335X_DCAN_IFDATB_OFFSET(n)    (0x0114 + ((unsigned int)(n) - 1) * 0x20)
#define AM335X_DCAN_IF3UPD_OFFSET(n)    (0x0160 + ((((unsigned int)(n) - 1) >> 5) << 2))

/* Register virtual addresses ***********************************************/

#define AM335X_DCAN0_CTL                (AM335X_DCAN0_VADDR + AM335X_DCAN_CTL_OFFSET)
#define AM335X_DCAN0_ES                 (AM335X_DCAN0_VADDR + AM335X_DCAN_ES_OFFSET)
#define AM335X_DCAN0_ERRC               (AM335X_DCAN0_VADDR + AM335X_DCAN_ERRC_OFFSET)
#define AM335X_DCAN0_BTR                (AM335X_DCAN0_VADDR + AM335X_DCAN_BTR_OFFSET)
#define AM335X_DCAN0_INT                (AM335X_DCAN0_VADDR + AM335X_DCAN_INT_OFFSET)
#define AM335X_DCAN0_TEST               (AM335X_DCAN0_VADDR + AM335X_DCAN_TEST_OFFSET)
#define AM335X_DCAN0_PERR               (AM335X_DCAN0_VADDR + AM335X_DCAN_PERR_OFFSET)
#define AM335X_DCAN0_ABOTR              (AM335X_DCAN0_VADDR + AM335X_DCAN_ABOTR_OFFSET)
#define AM335X_DCAN0_TXRQ_X             (AM335X_DCAN0_VADDR + AM335X_DCAN_TXRQ_X_OFFSET)
#define AM335X_DCAN0_TXRQ12             (AM335X_DCAN0_VADDR + AM335X_DCAN_TXRQ12_OFFSET)
#define AM335X_DCAN0_TXRQ34             (AM335X_DCAN0_VADDR + AM335X_DCAN_TXRQ34_OFFSET)
#define AM335X_DCAN0_TXRQ56             (AM335X_DCAN0_VADDR + AM335X_DCAN_TXRQ56_OFFSET)
#define AM335X_DCAN0_TXRQ78             (AM335X_DCAN0_VADDR + AM335X_DCAN_TXRQ78_OFFSET)
#define AM335X_DCAN0_NWDAT_X            (AM335X_DCAN0_VADDR + AM335X_DCAN_NWDAT_X_OFFSET)
#define AM335X_DCAN0_NWDAT12            (AM335X_DCAN0_VADDR + AM335X_DCAN_NWDAT12_OFFSET)
#define AM335X_DCAN0_NWDAT34            (AM335X_DCAN0_VADDR + AM335X_DCAN_NWDAT34_OFFSET)
#define AM335X_DCAN0_NWDAT56            (AM335X_DCAN0_VADDR + AM335X_DCAN_NWDAT56_OFFSET)
#define AM335X_DCAN0_NWDAT78            (AM335X_DCAN0_VADDR + AM335X_DCAN_NWDAT78_OFFSET)
#define AM335X_DCAN0_INTPND_X           (AM335X_DCAN0_VADDR + AM335X_DCAN_INTPND_X_OFFSET)
#define AM335X_DCAN0_INTPND12           (AM335X_DCAN0_VADDR + AM335X_DCAN_INTPND12_OFFSET)
#define AM335X_DCAN0_INTPND34           (AM335X_DCAN0_VADDR + AM335X_DCAN_INTPND34_OFFSET)
#define AM335X_DCAN0_INTPND56           (AM335X_DCAN0_VADDR + AM335X_DCAN_INTPND56_OFFSET)
#define AM335X_DCAN0_INTPND78           (AM335X_DCAN0_VADDR + AM335X_DCAN_INTPND78_OFFSET)
#define AM335X_DCAN0_MSGVAL_X           (AM335X_DCAN0_VADDR + AM335X_DCAN_MSGVAL_X_OFFSET)
#define AM335X_DCAN0_MSGVAL12           (AM335X_DCAN0_VADDR + AM335X_DCAN_MSGVAL12_OFFSET)
#define AM335X_DCAN0_MSGVAL34           (AM335X_DCAN0_VADDR + AM335X_DCAN_MSGVAL34_OFFSET)
#define AM335X_DCAN0_MSGVAL56           (AM335X_DCAN0_VADDR + AM335X_DCAN_MSGVAL56_OFFSET)
#define AM335X_DCAN0_MSGVAL78           (AM335X_DCAN0_VADDR + AM335X_DCAN_MSGVAL78_OFFSET)
#define AM335X_DCAN0_INTMUX12           (AM335X_DCAN0_VADDR + AM335X_DCAN_INTMUX12_OFFSET)
#define AM335X_DCAN0_INTMUX34           (AM335X_DCAN0_VADDR + AM335X_DCAN_INTMUX34_OFFSET)
#define AM335X_DCAN0_INTMUX56           (AM335X_DCAN0_VADDR + AM335X_DCAN_INTMUX56_OFFSET)
#define AM335X_DCAN0_INTMUX78           (AM335X_DCAN0_VADDR + AM335X_DCAN_INTMUX78_OFFSET)
#define AM335X_DCAN0_IF1CMD             (AM335X_DCAN0_VADDR + AM335X_DCAN_IF1CMD_OFFSET)
#define AM335X_DCAN0_IF1MSK             (AM335X_DCAN0_VADDR + AM335X_DCAN_IF1MSK_OFFSET)
#define AM335X_DCAN0_IF1ARB             (AM335X_DCAN0_VADDR + AM335X_DCAN_IF1ARB_OFFSET)
#define AM335X_DCAN0_IF1MCTL            (AM335X_DCAN0_VADDR + AM335X_DCAN_IF1MCTL_OFFSET)
#define AM335X_DCAN0_IF1DATA            (AM335X_DCAN0_VADDR + AM335X_DCAN_IF1DATA_OFFSET)
#define AM335X_DCAN0_IF1DATB            (AM335X_DCAN0_VADDR + AM335X_DCAN_IF1DATB_OFFSET)
#define AM335X_DCAN0_IF2CMD             (AM335X_DCAN0_VADDR + AM335X_DCAN_IF2CMD_OFFSET)
#define AM335X_DCAN0_IF2MSK             (AM335X_DCAN0_VADDR + AM335X_DCAN_IF2MSK_OFFSET)
#define AM335X_DCAN0_IF2ARB             (AM335X_DCAN0_VADDR + AM335X_DCAN_IF2ARB_OFFSET)
#define AM335X_DCAN0_IF2MCTL            (AM335X_DCAN0_VADDR + AM335X_DCAN_IF2MCTL_OFFSET)
#define AM335X_DCAN0_IF2DATA            (AM335X_DCAN0_VADDR + AM335X_DCAN_IF2DATA_OFFSET)
#define AM335X_DCAN0_IF2DATB            (AM335X_DCAN0_VADDR + AM335X_DCAN_IF2DATB_OFFSET)
#define AM335X_DCAN0_IF3OBS             (AM335X_DCAN0_VADDR + AM335X_DCAN_IF3OBS_OFFSET)
#define AM335X_DCAN0_IF3MSK             (AM335X_DCAN0_VADDR + AM335X_DCAN_IF3MSK_OFFSET)
#define AM335X_DCAN0_IF3ARB             (AM335X_DCAN0_VADDR + AM335X_DCAN_IF3ARB_OFFSET)
#define AM335X_DCAN0_IF3MCTL            (AM335X_DCAN0_VADDR + AM335X_DCAN_IF3MCTL_OFFSET)
#define AM335X_DCAN0_IF3DATA            (AM335X_DCAN0_VADDR + AM335X_DCAN_IF3DATA_OFFSET)
#define AM335X_DCAN0_IF3DATB            (AM335X_DCAN0_VADDR + AM335X_DCAN_IF3DATB_OFFSET)
#define AM335X_DCAN0_IF3UPD12           (AM335X_DCAN0_VADDR + AM335X_DCAN_IF3UPD12_OFFSET)
#define AM335X_DCAN0_IF3UPD34           (AM335X_DCAN0_VADDR + AM335X_DCAN_IF3UPD34_OFFSET)
#define AM335X_DCAN0_IF3UPD56           (AM335X_DCAN0_VADDR + AM335X_DCAN_IF3UPD56_OFFSET)
#define AM335X_DCAN0_IF3UPD78           (AM335X_DCAN0_VADDR + AM335X_DCAN_IF3UPD78_OFFSET)
#define AM335X_DCAN0_TIOC               (AM335X_DCAN0_VADDR + AM335X_DCAN_TIOC_OFFSET)
#define AM335X_DCAN0_RIOC               (AM335X_DCAN0_VADDR + AM335X_DCAN_RIOC_OFFSET)

#define AM335X_DCAN0_TXRQ(n)            (AM335X_DCAN0_VADDR + AM335X_DCAN_TXRQ_OFFSET(n))
#define AM335X_DCAN0_NWDAT(n)           (AM335X_DCAN0_VADDR + AM335X_DCAN_NWDAT_OFFSET(n))
#define AM335X_DCAN0_INTPND(n)          (AM335X_DCAN0_VADDR + AM335X_DCAN_INTPND_OFFSET(n))
#define AM335X_DCAN0_MSGVAL(n)          (AM335X_DCAN0_VADDR + AM335X_DCAN_MSGVAL_OFFSET(n))
#define AM335X_DCAN0_INTMUX(n)          (AM335X_DCAN0_VADDR + AM335X_DCAN_INTMUX_OFFSET(n))
#define AM335X_DCAN0_IFCMD(n)           (AM335X_DCAN0_VADDR + AM335X_DCAN_IFCMD_OFFSET(n))
#define AM335X_DCAN0_IFMSK(n)           (AM335X_DCAN0_VADDR + AM335X_DCAN_IFMSK_OFFSET(n))
#define AM335X_DCAN0_IFARB(n)           (AM335X_DCAN0_VADDR + AM335X_DCAN_IFARB_OFFSET(n))
#define AM335X_DCAN0_IFMCTL(n)          (AM335X_DCAN0_VADDR + AM335X_DCAN_IFMCTL_OFFSET(n))
#define AM335X_DCAN0_IFDATA(n)          (AM335X_DCAN0_VADDR + AM335X_DCAN_IFDATA_OFFSET(n))
#define AM335X_DCAN0_IFDATB(n)          (AM335X_DCAN0_VADDR + AM335X_DCAN_IFDATB_OFFSET(n))
#define AM335X_DCAN0_IF3UPD(n)          (AM335X_DCAN0_VADDR + AM335X_DCAN_IF3UPD_OFFSET(n))

#define AM335X_DCAN1_CTL                (AM335X_DCAN1_VADDR + AM335X_DCAN_CTL_OFFSET)
#define AM335X_DCAN1_ES                 (AM335X_DCAN1_VADDR + AM335X_DCAN_ES_OFFSET)
#define AM335X_DCAN1_ERRC               (AM335X_DCAN1_VADDR + AM335X_DCAN_ERRC_OFFSET)
#define AM335X_DCAN1_BTR                (AM335X_DCAN1_VADDR + AM335X_DCAN_BTR_OFFSET)
#define AM335X_DCAN1_INT                (AM335X_DCAN1_VADDR + AM335X_DCAN_INT_OFFSET)
#define AM335X_DCAN1_TEST               (AM335X_DCAN1_VADDR + AM335X_DCAN_TEST_OFFSET)
#define AM335X_DCAN1_PERR               (AM335X_DCAN1_VADDR + AM335X_DCAN_PERR_OFFSET)
#define AM335X_DCAN1_ABOTR              (AM335X_DCAN1_VADDR + AM335X_DCAN_ABOTR_OFFSET)
#define AM335X_DCAN1_TXRQ_X             (AM335X_DCAN1_VADDR + AM335X_DCAN_TXRQ_X_OFFSET)
#define AM335X_DCAN1_TXRQ12             (AM335X_DCAN1_VADDR + AM335X_DCAN_TXRQ12_OFFSET)
#define AM335X_DCAN1_TXRQ34             (AM335X_DCAN1_VADDR + AM335X_DCAN_TXRQ34_OFFSET)
#define AM335X_DCAN1_TXRQ56             (AM335X_DCAN1_VADDR + AM335X_DCAN_TXRQ56_OFFSET)
#define AM335X_DCAN1_TXRQ78             (AM335X_DCAN1_VADDR + AM335X_DCAN_TXRQ78_OFFSET)
#define AM335X_DCAN1_NWDAT_X            (AM335X_DCAN1_VADDR + AM335X_DCAN_NWDAT_X_OFFSET)
#define AM335X_DCAN1_NWDAT12            (AM335X_DCAN1_VADDR + AM335X_DCAN_NWDAT12_OFFSET)
#define AM335X_DCAN1_NWDAT34            (AM335X_DCAN1_VADDR + AM335X_DCAN_NWDAT34_OFFSET)
#define AM335X_DCAN1_NWDAT56            (AM335X_DCAN1_VADDR + AM335X_DCAN_NWDAT56_OFFSET)
#define AM335X_DCAN1_NWDAT78            (AM335X_DCAN1_VADDR + AM335X_DCAN_NWDAT78_OFFSET)
#define AM335X_DCAN1_INTPND_X           (AM335X_DCAN1_VADDR + AM335X_DCAN_INTPND_X_OFFSET)
#define AM335X_DCAN1_INTPND12           (AM335X_DCAN1_VADDR + AM335X_DCAN_INTPND12_OFFSET)
#define AM335X_DCAN1_INTPND34           (AM335X_DCAN1_VADDR + AM335X_DCAN_INTPND34_OFFSET)
#define AM335X_DCAN1_INTPND56           (AM335X_DCAN1_VADDR + AM335X_DCAN_INTPND56_OFFSET)
#define AM335X_DCAN1_INTPND78           (AM335X_DCAN1_VADDR + AM335X_DCAN_INTPND78_OFFSET)
#define AM335X_DCAN1_MSGVAL_X           (AM335X_DCAN1_VADDR + AM335X_DCAN_MSGVAL_X_OFFSET)
#define AM335X_DCAN1_MSGVAL12           (AM335X_DCAN1_VADDR + AM335X_DCAN_MSGVAL12_OFFSET)
#define AM335X_DCAN1_MSGVAL34           (AM335X_DCAN1_VADDR + AM335X_DCAN_MSGVAL34_OFFSET)
#define AM335X_DCAN1_MSGVAL56           (AM335X_DCAN1_VADDR + AM335X_DCAN_MSGVAL56_OFFSET)
#define AM335X_DCAN1_MSGVAL78           (AM335X_DCAN1_VADDR + AM335X_DCAN_MSGVAL78_OFFSET)
#define AM335X_DCAN1_INTMUX12           (AM335X_DCAN1_VADDR + AM335X_DCAN_INTMUX12_OFFSET)
#define AM335X_DCAN1_INTMUX34           (AM335X_DCAN1_VADDR + AM335X_DCAN_INTMUX34_OFFSET)
#define AM335X_DCAN1_INTMUX56           (AM335X_DCAN1_VADDR + AM335X_DCAN_INTMUX56_OFFSET)
#define AM335X_DCAN1_INTMUX78           (AM335X_DCAN1_VADDR + AM335X_DCAN_INTMUX78_OFFSET)
#define AM335X_DCAN1_IF1CMD             (AM335X_DCAN1_VADDR + AM335X_DCAN_IF1CMD_OFFSET)
#define AM335X_DCAN1_IF1MSK             (AM335X_DCAN1_VADDR + AM335X_DCAN_IF1MSK_OFFSET)
#define AM335X_DCAN1_IF1ARB             (AM335X_DCAN1_VADDR + AM335X_DCAN_IF1ARB_OFFSET)
#define AM335X_DCAN1_IF1MCTL            (AM335X_DCAN1_VADDR + AM335X_DCAN_IF1MCTL_OFFSET)
#define AM335X_DCAN1_IF1DATA            (AM335X_DCAN1_VADDR + AM335X_DCAN_IF1DATA_OFFSET)
#define AM335X_DCAN1_IF1DATB            (AM335X_DCAN1_VADDR + AM335X_DCAN_IF1DATB_OFFSET)
#define AM335X_DCAN1_IF2CMD             (AM335X_DCAN1_VADDR + AM335X_DCAN_IF2CMD_OFFSET)
#define AM335X_DCAN1_IF2MSK             (AM335X_DCAN1_VADDR + AM335X_DCAN_IF2MSK_OFFSET)
#define AM335X_DCAN1_IF2ARB             (AM335X_DCAN1_VADDR + AM335X_DCAN_IF2ARB_OFFSET)
#define AM335X_DCAN1_IF2MCTL            (AM335X_DCAN1_VADDR + AM335X_DCAN_IF2MCTL_OFFSET)
#define AM335X_DCAN1_IF2DATA            (AM335X_DCAN1_VADDR + AM335X_DCAN_IF2DATA_OFFSET)
#define AM335X_DCAN1_IF2DATB            (AM335X_DCAN1_VADDR + AM335X_DCAN_IF2DATB_OFFSET)
#define AM335X_DCAN1_IF3OBS             (AM335X_DCAN1_VADDR + AM335X_DCAN_IF3OBS_OFFSET)
#define AM335X_DCAN1_IF3MSK             (AM335X_DCAN1_VADDR + AM335X_DCAN_IF3MSK_OFFSET)
#define AM335X_DCAN1_IF3ARB             (AM335X_DCAN1_VADDR + AM335X_DCAN_IF3ARB_OFFSET)
#define AM335X_DCAN1_IF3MCTL            (AM335X_DCAN1_VADDR + AM335X_DCAN_IF3MCTL_OFFSET)
#define AM335X_DCAN1_IF3DATA            (AM335X_DCAN1_VADDR + AM335X_DCAN_IF3DATA_OFFSET)
#define AM335X_DCAN1_IF3DATB            (AM335X_DCAN1_VADDR + AM335X_DCAN_IF3DATB_OFFSET)
#define AM335X_DCAN1_IF3UPD12           (AM335X_DCAN1_VADDR + AM335X_DCAN_IF3UPD12_OFFSET)
#define AM335X_DCAN1_IF3UPD34           (AM335X_DCAN1_VADDR + AM335X_DCAN_IF3UPD34_OFFSET)
#define AM335X_DCAN1_IF3UPD56           (AM335X_DCAN1_VADDR + AM335X_DCAN_IF3UPD56_OFFSET)
#define AM335X_DCAN1_IF3UPD78           (AM335X_DCAN1_VADDR + AM335X_DCAN_IF3UPD78_OFFSET)
#define AM335X_DCAN1_TIOC               (AM335X_DCAN1_VADDR + AM335X_DCAN_TIOC_OFFSET)
#define AM335X_DCAN1_RIOC               (AM335X_DCAN1_VADDR + AM335X_DCAN_RIOC_OFFSET)

#define AM335X_DCAN1_TXRQ(n)            (AM335X_DCAN1_VADDR + AM335X_DCAN_TXRQ_OFFSET(n))
#define AM335X_DCAN1_NWDAT(n)           (AM335X_DCAN1_VADDR + AM335X_DCAN_NWDAT_OFFSET(n))
#define AM335X_DCAN1_INTPND(n)          (AM335X_DCAN1_VADDR + AM335X_DCAN_INTPND_OFFSET(n))
#define AM335X_DCAN1_MSGVAL(n)          (AM335X_DCAN1_VADDR + AM335X_DCAN_MSGVAL_OFFSET(n))
#define AM335X_DCAN1_INTMUX(n)          (AM335X_DCAN1_VADDR + AM335X_DCAN_INTMUX_OFFSET(n))
#define AM335X_DCAN1_IFCMD(n)           (AM335X_DCAN1_VADDR + AM335X_DCAN_IFCMD_OFFSET(n))
#define AM335X_DCAN1_IFMSK(n)           (AM335X_DCAN1_VADDR + AM335X_DCAN_IFMSK_OFFSET(n))
#define AM335X_DCAN1_IFARB(n)           (AM335X_DCAN1_VADDR + AM335X_DCAN_IFARB_OFFSET(n))
#define AM335X_DCAN1_IFMCTL(n)          (AM335X_DCAN1_VADDR + AM335X_DCAN_IFMCTL_OFFSET(n))
#define AM335X_DCAN1_IFDATA(n)          (AM335X_DCAN1_VADDR + AM335X_DCAN_IFDATA_OFFSET(n))
#define AM335X_DCAN1_IFDATB(n)          (AM335X_DCAN1_VADDR + AM335X_DCAN_IFDATB_OFFSET(n))
#define AM335X_DCAN1_IF3UPD(n)          (AM335X_DCAN1_VADDR + AM335X_DCAN_IF3UPD_OFFSET(n))

/* Register bit field definitions *******************************************/

#define DCAN_CTL_INIT                   (1 << 0)  /* Bit 0:  Initialization mode */
#define DCAN_CTL_IE0                    (1 << 1)  /* Bit 1:  Interrupt line 0 enable */
#define DCAN_CTL_SIE                    (1 << 2)  /* Bit 2:  Status change interrupt enable */
#define DCAN_CTL_EIE                    (1 << 3)  /* Bit 3:  Error interrupt enable */
#define DCAN_CTL_DAR                    (1 << 5)  /* Bit 5:  Disable automatic retransmission */
#define DCAN_CTL_CCE                    (1 << 6)  /* Bit 6:  Configuration change enable */
#define DCAN_CTL_TEST                   (1 << 7)  /* Bit 7:  Test mode enable */
#define DCAN_CTL_IDS                    (1 << 8)  /* Bit 8:  Interruption debug support enable */
#define DCAN_CTL_ABO                    (1 << 9)  /* Bit 9:  Auto-Bus-On enable */

#define DCAN_CTL_PMD_SHIFT              (10)  /* Bits 10-13:  Parity on/off. */
#define DCAN_CTL_PMD_MASK               (15 << DCAN_CTL_PMD_SHIFT)
#  define DCAN_CTL_PMD_OFF              (5 << DCAN_CTL_PMD_SHIFT)  /* Parity function disabled */
#  define DCAN_CTL_PMD_ON               (10 << DCAN_CTL_PMD_SHIFT) /* Parity function enabled */

#define DCAN_CTL_SWR                    (1 << 15)  /* Bit 15:  Software reset enable */
#define DCAN_CTL_INITDBG                (1 << 16)  /* Bit 16:  Internal init state while debug access */
#define DCAN_CTL_IE1                    (1 << 17)  /* Bit 17:  Interrupt line 1 enable */
#define DCAN_CTL_DE1                    (1 << 18)  /* Bit 18:  Enable DMA request line for IF1 */
#define DCAN_CTL_DE2                    (1 << 19)  /* Bit 19:  Enable DMA request line for IF2 */
#define DCAN_CTL_DE3                    (1 << 20)  /* Bit 20:  Enable DMA request line for IF3 */
#define DCAN_CTL_PDR                    (1 << 24)  /* Bit 24:  Request for local low power-down mode */
#define DCAN_CTL_WUBA                   (1 << 25)  /* Bit 25:  Automatic wake up on bus activity when in local power-down mode */

#define DCAN_ES_LEC_SHIFT               (0)  /* Bits 0-2:  Last error code. */
#define DCAN_ES_LEC_MASK                (7 << DCAN_ES_LEC_SHIFT)
#  define DCAN_ES_LEC_NO_ERROR          (0 << DCAN_ES_LEC_SHIFT)  /* No error */
#  define DCAN_ES_LEC_STUFF_ERROR       (1 << DCAN_ES_LEC_SHIFT)  /* Stuff error */
#  define DCAN_ES_LEC_FORM_ERROR        (2 << DCAN_ES_LEC_SHIFT)  /* Form error */
#  define DCAN_ES_LEC_ACK_ERROR         (3 << DCAN_ES_LEC_SHIFT)  /* Ack error */
#  define DCAN_ES_LEC_BIT1_ERROR        (4 << DCAN_ES_LEC_SHIFT)  /* Bit1 error */
#  define DCAN_ES_LEC_BIT0_ERROR        (5 << DCAN_ES_LEC_SHIFT)  /* Bit0 error */
#  define DCAN_ES_LEC_CRC_ERROR         (6 << DCAN_ES_LEC_SHIFT)  /* CRC error */
#  define DCAN_ES_LEC_NO_EVENT          (7 << DCAN_ES_LEC_SHIFT)  /* No CAN bus event since last read */

#define DCAN_ES_TX_OK                   (1 << 3)  /* Bit 3: Transmitted a message successfully */
#define DCAN_ES_RX_OK                   (1 << 4)  /* Bit 4: Received a message successfully */
#define DCAN_ES_EPASSIVE                (1 << 5)  /* Bit 5: Error passive state */
#define DCAN_ES_EWARN                   (1 << 6)  /* Bit 6: Warning state */
#define DCAN_ES_BUSOFF                  (1 << 7)  /* Bit 7: Bus-Off state */
#define DCAN_ES_PER                     (1 << 8)  /* Bit 8: Parity error detected */
#define DCAN_ES_WKUP_PND                (1 << 9)  /* Bit 9: Wake up pending */
#define DCAN_ES_PDA                     (1 << 10) /* Bit 10: Local power-down mode acknowledge */

#define DCAN_ERRC_TEC_SHIFT             (0)  /* Bits 10-13:  Parity on/off. */
#define DCAN_ERRC_TEC_MASK              (255 << DCAN_ERRC_TEC_SHIFT)
#define DCAN_ERRC_REC_SHIFT             (8)  /* Bits 10-13:  Parity on/off. */
#define DCAN_ERRC_REC_MASK              (255 << DCAN_ERRC_REC_SHIFT)
#define DCAN_ERRC_RP                    (1 << 15)  /* Bit 15:  Receive error passive */

#define DCAN_BTR_BRP_SHIFT              (0)  /* Bits 0-5:  Baud rate prescaler */
#define DCAN_BTR_BRP_MASK               (63 << DCAN_BTR_BRP_SHIFT)
#define DCAN_BTR_SJW_SHIFT              (6)  /* Bits 6-7:  Synchronization Jump Width */
#define DCAN_BTR_SJW_MASK               (3 << DCAN_BTR_SJW_SHIFT)
#define DCAN_BTR_TSEG1_SHIFT            (8)  /* Bits 8-11:  Time segment before the sample point */
#define DCAN_BTR_TSEG1_MASK             (15 << DCAN_BTR_TSEG1_SHIFT)
#define DCAN_BTR_TSEG2_SHIFT            (12)  /* Bits 12-14:  Time segment after the sample point */
#define DCAN_BTR_TSEG2_MASK             (7 << DCAN_BTR_TSEG2_SHIFT)
#define DCAN_BTR_BRPE_SHIFT             (16)  /* Bits 16-19:  Baud rate prescaler extension */
#define DCAN_BTR_BRPE_MASK              (15 << DCAN_BTR_BRPE_SHIFT)

#define DCAN_INT_LINE0_SHIFT            (0)  /* Bits 0-15:  Interrupt Line 0 Identifier */
#define DCAN_INT_LINE0_MASK             (65535 << DCAN_INT_LINE0_SHIFT)
#  define DCAN_INT_LINE0_STATUS         (32768 << DCAN_INT_LINE0_SHIFT)
#define DCAN_INT_LINE1_SHIFT            (16)  /* Bits 16-23:  Interrupt Line 1 Identifier */
#define DCAN_INT_LINE1_MASK             (255 << DCAN_INT_LINE1_SHIFT)

#define DCAN_TEST_SILENT                (1 << 3)  /* Bit 3:  Silent mode */
#define DCAN_TEST_LBACK                 (1 << 4)  /* Bit 4:  Loopback mode */
#define DCAN_TEST_TX_SHIFT              (5)       /* Bits 5-6:  Control of CAN_TX pin */
#define DCAN_TEST_TX_MASK               (3 << DCAN_TEST_TX_SHIFT)
#  define DCAN_TEST_TX_NORMAL           (0 << DCAN_TEST_TX_SHIFT)  /* Normal operation */
#  define DCAN_TEST_TX_SAMLE            (1 << DCAN_TEST_TX_SHIFT)  /* Sample point can be monitored at CAN_TX pin */
#  define DCAN_TEST_TX_DOMINANT         (2 << DCAN_TEST_TX_SHIFT)  /* CAN_TX pin drives a dominant value */
#  define DCAN_TEST_TX_RECESSIVE        (3 << DCAN_TEST_TX_SHIFT)  /* CAN_TX pin drives a recessive value */

#define DCAN_TEST_RX                    (1 << 7)  /* Bit 7:  Receive pin monitoring */
#define DCAN_TEST_EXL                   (1 << 8)  /* Bit 8:  External loopback mode */
#define DCAN_TEST_RDA                   (1 << 9)  /* Bit 9:  RAM direct access enable */

#define DCAN_PERR_MSG_NUM_SHIFT         (0)  /* Bits 0-7:  Message number */
#define DCAN_PERR_MSG_NUM_MASK          (255 << DCAN_PERR_MSG_NUM_SHIFT)
#define DCAN_PERR_WORD_NUM_SHIFT        (8)  /* Bits 8-10:  Word number where parity error has been detected */
#define DCAN_PERR_WORD_NUM_MASK         (7 << DCAN_PERR_WORD_NUM_SHIFT)

#define DCAN_TXRQ(n)                    (1 << (((unsigned int)(n) - 1) & 0x1f))  /* Bit 0-31:  Transmission request bits (for all message objects) */

#define DCAN_NWDAT(n)                   (1 << (((unsigned int)(n) - 1) & 0x1f))  /* Bit 0-31:  New Data Bits (for all message objects) */

#define DCAN_INTPND(n)                  (1 << (((unsigned int)(n) - 1) & 0x1f))  /* Bit 0-31:  Interrupt Pending Bits (for all message objects) */

#define DCAN_MSGVAL(n)                  (1 << (((unsigned int)(n) - 1) & 0x1f))  /* Bit 0-31:  Message valid bits (for all message objects) */

#define DCAN_INTMUX_LAST                (1 << 0)  /* Bit 0:  Last implemented message object */

#define DCAN_INTMUX(n)                  (1 << ((unsigned int)(n) & 0x1f))  /* Bit n:  Message object number n */

#define DCAN_IFCMD_MSG_NUM_SHIFT        (0)  /* Bits 0-7:  Number of message object in message RAM which is used for data transfer */
#define DCAN_IFCMD_MSG_NUM_MASK         (0xff << DCAN_IFCMD_MSG_NUM_SHIFT)
#  define DCAN_IFCMD_MSG_NUM(n)         (((unsigned int)(n) & DCAN_IFCMD_MSG_NUM_MASK) << DCAN_IFCMD_MSG_NUM_SHIFT)
#define DCAN_IFCMD_DMA_ACTIVE           (1 << 14)  /* Bit 14:  Activation of DMA feature for subsequent internal IF update */
#define DCAN_IFCMD_BUSY                 (1 << 15)  /* Bit 15:  Busy flag */
#define DCAN_IFCMD_DATAB                (1 << 16)  /* Bit 16:  Access Data Bytes 4 to 7 */
#define DCAN_IFCMD_DATAA                (1 << 17)  /* Bit 17:  Access Data Bytes 0 to 3 */
#define DCAN_IFCMD_TX_RQST_NEWDAT       (1 << 18)  /* Bit 18:  Access transmission request bit */
#define DCAN_IFCMD_CLR_INTPND           (1 << 19)  /* Bit 19:  Clear interrupt pending bit */
#define DCAN_IFCMD_CTL                  (1 << 20)  /* Bit 20:  Access control bits */
#define DCAN_IFCMD_ARB                  (1 << 21)  /* Bit 21:  Access arbitration bits */
#define DCAN_IFCMD_MASK                 (1 << 22)  /* Bit 22:  Access mask bits */
#define DCAN_IFCMD_WR_RD                (1 << 23)  /* Bit 23:  Write/Read direction */

#define DCAN_IFMSK_MSK_SHIFT            (0)  /* Bits 0-28:  Message identifier mask */
#define DCAN_IFMSK_MSK_MASK             (0x1fffffff << DCAN_IFCMD_MSK_SHIFT)
#define DCAN_IFMSK_MDIR                 (1 << 30)  /* Bit 30:  Mask message direction */
#define DCAN_IFMSK_MXTD                 (1 << 31)  /* Bit 31:  Mask extended identifier */

#define DCAN_IFARB_ID_SHIFT             (0)  /* Bits 0-28:  Message identifier */
#define DCAN_IFARB_ID_MASK              (0x1fffffff << DCAN_IFARB_ID_SHIFT)
#define DCAN_IFARB_DIR                  (1 << 29)  /* Bit 29:  Message direction */
#define DCAN_IFARB_XTD                  (1 << 30)  /* Bit 30:  Extended identifier */
#define DCAN_IFARB_MSG_VAL              (1 << 31)  /* Bit 31:  Message valid */

#define DCAN_IFMCTL_DLC_SHIFT           (0)  /* Bits 0-3:  Data length code */
#define DCAN_IFMCTL_DLC_MASK            (15 << DCAN_IFMCTL_DLC_SHIFT)
#define DCAN_IFMCTL_EOB                 (1 << 7)   /* Bit 7:  Data frame has 0 to 8 data bits. */
#define DCAN_IFMCTL_TX_RQST             (1 << 8)   /* Bit 8:  Transmit request */
#define DCAN_IFMCTL_RMT_EN              (1 << 9)   /* Bit 9:  Remote enable */
#define DCAN_IFMCTL_RX_IE               (1 << 10)  /* Bit 10:  Receive interrupt enable */
#define DCAN_IFMCTL_TX_IE               (1 << 11)  /* Bit 11:  Transmit interrupt enable */
#define DCAN_IFMCTL_UMASK               (1 << 12)  /* Bit 12:  Use acceptance mask */
#define DCAN_IFMCTL_INTPND              (1 << 13)  /* Bit 13:  Interrupt pending */
#define DCAN_IFMCTL_MSGLST              (1 << 14)  /* Bit 14:  Message lost (only valid for message objects with direction Receive) */
#define DCAN_IFMCTL_NEWDAT              (1 << 15)  /* Bit 15:  New data */

#define DCAN_IF3OBS_MASK                (1 << 0)   /* Bit 0:  Mask data read observation */
#define DCAN_IF3OBS_ARB                 (1 << 1)   /* Bit 1:  Arbitration data read observation */
#define DCAN_IF3OBS_CTRL                (1 << 2)   /* Bit 2:  Control read observation */
#define DCAN_IF3OBS_DATAA               (1 << 3)   /* Bit 3:  Data A read observation */
#define DCAN_IF3OBS_DATAB               (1 << 4)   /* Bit 4:  Data B read observation */
#define DCAN_IF3OBS_SM                  (1 << 8)   /* Bit 8:  Status of Mask data read access */
#define DCAN_IF3OBS_SA                  (1 << 9)   /* Bit 9:  Status of Arbitration data read access */
#define DCAN_IF3OBS_SC                  (1 << 10)  /* Bit 10:  Status of control bits read access */
#define DCAN_IF3OBS_SDA                 (1 << 11)  /* Bit 11:  Status of Data A read access */
#define DCAN_IF3OBS_SDB                 (1 << 12)  /* Bit 12:  Status of Data B read access */
#define DCAN_IF3OBS_UPD                 (1 << 15)  /* Bit 15:  Update Data*/

#define DCAN_IF3UPD(n)                  (1 << (((unsigned int)(n) - 1) & 0x1f))  /* Bit 0-31: IF3 Update Enabled (for all message objects) */

#define DCAN_TIOC_IN                    (1 << 0)   /* Bit 0:  CAN_TX data in */
#define DCAN_TIOC_OUT                   (1 << 1)   /* Bit 1:  CAN_TX data out write */
#define DCAN_TIOC_DIR                   (1 << 2)   /* Bit 2:  CAN_TX data direction */
#define DCAN_TIOC_FUNC                  (1 << 3)   /* Bit 3:  CAN_TX function */
#define DCAN_TIOC_OD                    (1 << 16)  /* Bit 16:  CAN_TX open drain enable */
#define DCAN_TIOC_PD                    (1 << 17)  /* Bit 17:  CAN_TX pull disable */
#define DCAN_TIOC_PU                    (1 << 18)  /* Bit 18:  CAN_TX pull up/pull down select */

#define DCAN_RIOC_IN                    (1 << 0)   /* Bit 0:  CAN_RX data in */
#define DCAN_RIOC_OUT                   (1 << 1)   /* Bit 1:  CAN_RX data out write */
#define DCAN_RIOC_DIR                   (1 << 2)   /* Bit 2:  CAN_RX data direction */
#define DCAN_RIOC_FUNC                  (1 << 3)   /* Bit 3:  CAN_RX function */
#define DCAN_RIOC_OD                    (1 << 16)  /* Bit 16:  CAN_RX open drain enable */
#define DCAN_RIOC_PD                    (1 << 17)  /* Bit 17:  CAN_RX pull disable */
#define DCAN_RIOC_PU                    (1 << 18)  /* Bit 18:  CAN_RX pull up/pull down select */

#endif /* __ARCH_ARM_SRC_AM335X_HARDWARE_AM335X_DCAN_H */
