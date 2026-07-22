/****************************************************************************
 * arch/arm/src/rm57/hardware/rm57_rti.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* Register layout taken from TI's HALCoGen HL_reg_rti.h for
 * RM57L843 - matches TMS570's RTI+DWWD address (0xfffffc00). Used for
 * the NuttX system tick.
 */

#ifndef __ARCH_ARM_SRC_RM57_HARDWARE_RM57_RTI_H
#define __ARCH_ARM_SRC_RM57_HARDWARE_RM57_RTI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/rm57l843_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define RM57_RTI_GCTRL_OFFSET       0x0000 /* Global Control Register */
#define RM57_RTI_TBCTRL_OFFSET      0x0004 /* Timebase Control Register */
#define RM57_RTI_CAPCTRL_OFFSET     0x0008 /* Capture Control Register */
#define RM57_RTI_COMPCTRL_OFFSET    0x000c /* Compare Control Register */

/* Counter x block (x = 0, 1), block size 0x20 */

/* Free Running Counter x */
#define RM57_RTI_FRCx_OFFSET(x) (0x0010 + ((x) << 5))
#define RM57_RTI_UCx_OFFSET(x)      (0x0014 + ((x) << 5)) /* Up Counter x */

/* Compare Up Counter x */
#define RM57_RTI_CPUCx_OFFSET(x) (0x0018 + ((x) << 5))

/* Capture Free Running Counter x */
#define RM57_RTI_CAFRCx_OFFSET(x) (0x0020 + ((x) << 5))

/* Capture Up Counter x */
#define RM57_RTI_CAUCx_OFFSET(x) (0x0024 + ((x) << 5))

/* Compare x block (x = 0..3), block size 0x08 */

#define RM57_RTI_COMPx_OFFSET(x)    (0x0050 + ((x) << 3)) /* Compare x */

/* Update Compare x */
#define RM57_RTI_UDCPx_OFFSET(x) (0x0054 + ((x) << 3))

/* External Clock Timebase Low Compare Register */
#define RM57_RTI_TBLCOMP_OFFSET 0x0070

/* External Clock Timebase High Compare Register */
#define RM57_RTI_TBHCOMP_OFFSET 0x0074
#define RM57_RTI_SETINTENA_OFFSET   0x0080 /* Set/Status Interrupt Register */

/* Clear/Status Interrupt Register */
#define RM57_RTI_CLEARINTENA_OFFSET 0x0084
#define RM57_RTI_INTFLAG_OFFSET     0x0088 /* Interrupt Flag Register */

/* Digital Watchdog Control Register */
#define RM57_RTI_DWDCTRL_OFFSET 0x0090

/* Digital Watchdog Preload Register */
#define RM57_RTI_DWDPRLD_OFFSET 0x0094
#define RM57_RTI_WDSTATUS_OFFSET    0x0098 /* Watchdog Status Register */
#define RM57_RTI_WDKEY_OFFSET       0x009c /* Watchdog Key Register */
#define RM57_RTI_DWDCNTR_OFFSET     0x00a0 /* Digital Watchdog Down Counter */

/* Windowed Watchdog Reaction Control */
#define RM57_RTI_WWDRXNCTRL_OFFSET 0x00a4

/* Windowed Watchdog Window Size Control */
#define RM57_RTI_WWDSIZECTRL_OFFSET 0x00a8

/* Compare Interrupt Clear Enable Register */
#define RM57_RTI_INTCLRENABLE_OFFSET 0x00ac
#define RM57_RTI_COMP0CLR_OFFSET    0x00b0 /* Compare 0 Clear Register */
#define RM57_RTI_COMP1CLR_OFFSET    0x00b4 /* Compare 1 Clear Register */
#define RM57_RTI_COMP2CLR_OFFSET    0x00b8 /* Compare 2 Clear Register */
#define RM57_RTI_COMP3CLR_OFFSET    0x00bc /* Compare 3 Clear Register */

/* Register Addresses (RTI1, counter/compare block 0) ***********************/

#define RM57_RTI_GCTRL              (RM57_RTI1_BASE + RM57_RTI_GCTRL_OFFSET)
#define RM57_RTI_TBCTRL             (RM57_RTI1_BASE + RM57_RTI_TBCTRL_OFFSET)
#define RM57_RTI_CAPCTRL            (RM57_RTI1_BASE + RM57_RTI_CAPCTRL_OFFSET)
#define RM57_RTI_COMPCTRL (RM57_RTI1_BASE + RM57_RTI_COMPCTRL_OFFSET)
#define RM57_RTI_FRC0 (RM57_RTI1_BASE + RM57_RTI_FRCx_OFFSET(0))
#define RM57_RTI_UC0                 (RM57_RTI1_BASE + RM57_RTI_UCx_OFFSET(0))
#define RM57_RTI_CPUC0 (RM57_RTI1_BASE + RM57_RTI_CPUCx_OFFSET(0))
#define RM57_RTI_COMP0 (RM57_RTI1_BASE + RM57_RTI_COMPx_OFFSET(0))
#define RM57_RTI_UDCP0 (RM57_RTI1_BASE + RM57_RTI_UDCPx_OFFSET(0))
#define RM57_RTI_SETINTENA (RM57_RTI1_BASE + RM57_RTI_SETINTENA_OFFSET)
#define RM57_RTI_CLEARINTENA (RM57_RTI1_BASE + RM57_RTI_CLEARINTENA_OFFSET)
#define RM57_RTI_INTFLAG            (RM57_RTI1_BASE + RM57_RTI_INTFLAG_OFFSET)

/* Register Bit-Field Definitions *******************************************/

/* Global Control Register */

/* Bit 0: Enable counter block 0 */
#define RTI_GCTRL_CNT0EN (1 << 0)

/* Bit 1: Enable counter block 1 */
#define RTI_GCTRL_CNT1EN (1 << 1)

/* Compare/interrupt bits (SETINTENA/CLEARINTENA/INTFLAG) */

#define RTI_INT_COMPARE0             (1 << 0)
#define RTI_INT_COMPARE1             (1 << 1)
#define RTI_INT_COMPARE2             (1 << 2)
#define RTI_INT_COMPARE3             (1 << 3)
#define RTI_INT_OVERFLOW0            (1 << 16)
#define RTI_INT_OVERFLOW1            (1 << 17)
#define RTI_INT_TIMEBASE             (1 << 18)

#endif /* __ARCH_ARM_SRC_RM57_HARDWARE_RM57_RTI_H */
