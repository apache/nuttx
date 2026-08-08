/****************************************************************************
 * arch/arm/src/rm57/hardware/rm57l843_memorymap.h
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

/* Base addresses below are taken directly from TI's HALCoGen-generated
 * register headers (HL_reg_*.h) for RM57L843, not assumed from TMS570
 * parity - several peripheral base addresses differ between the two
 * Hercules generations (VIM, PCR, and pinmux in particular).
 */

#ifndef __ARCH_ARM_SRC_RM57_HARDWARE_RM57L843_MEMORYMAP_H
#define __ARCH_ARM_SRC_RM57_HARDWARE_RM57L843_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory Map Overview (from HL_sys_link.ld) */

/* 0x00000000-0x003fffff: Program FLASH (4MB) */
#define RM57_FLASH_BASE 0x00000000

/* 0x08000000-0x0807ffff: RAM (512KB) */
#define RM57_RAM_BASE 0x08000000

/* Peripheral Control Registers (from HL_reg_*.h) */

#define RM57_GIO_BASE             0xfff7bc00 /* GIO (gioREG) */
#define RM57_GIO_PORTA_BASE       0xfff7bc34 /* GIO Port A (gioPORTA) */
#define RM57_GIO_PORTB_BASE       0xfff7bc54 /* GIO Port B (gioPORTB) */
#define RM57_SCI1_BASE            0xfff7e400 /* SCI1/LIN1 (sciREG1) */
#define RM57_SCI1_PORT_BASE       0xfff7e440 /* SCI1 GIO port (sciPORT1) */
#define RM57_SCI3_BASE            0xfff7e500 /* SCI3 (sciREG3) */
#define RM57_SCI3_PORT_BASE       0xfff7e540 /* SCI3 GIO port (sciPORT3) */
#define RM57_SCI2_BASE            0xfff7e600 /* SCI2 (sciREG2) */
#define RM57_SCI2_PORT_BASE       0xfff7e640 /* SCI2 GIO port (sciPORT2) */
#define RM57_SCI4_BASE            0xfff7e700 /* SCI4 (sciREG4) */
#define RM57_SCI4_PORT_BASE       0xfff7e740 /* SCI4 GIO port (sciPORT4) */

/* System Modules Control Registers and Memories */

#define RM57_VIMRAM_BASE          0xfff82000 /* VIM RAM (vimRAM) */
#define RM57_FWRAP_BASE           0xfff87000 /* Flash Wrapper (flashWREG) */
#define RM57_PCR1_BASE            0xffff1000 /* PCR frame 1 (pcrREG1) */

/* Pin Multiplexing (pinMuxReg) */
#define RM57_PINMUX_BASE 0xffff1c00
#define RM57_PCR3_BASE            0xfff78000 /* PCR frame 3 (pcrREG3) */

/* PCR frame 2, mirrored bus (pcrREG2) */
#define RM57_PCR2_BASE 0xfcff1000

/* System Module - Frame 2 (systemREG2) */
#define RM57_SYS2_BASE 0xffffe100
#define RM57_ESM_BASE             0xfffff500 /* ESM (esmREG) */
#define RM57_RTI1_BASE            0xfffffc00 /* RTI + DWWD (rtiREG1) */
#define RM57_VIM_BASE             0xfffffd00 /* VIM (vimREG) */

/* System Module - Frame 1 (systemREG1) */
#define RM57_SYS1_BASE 0xffffff00
#define RM57_ECLK_PORT_BASE       0xffffff04 /* ECLK GIO port (systemPORT) */

#endif /* __ARCH_ARM_SRC_RM57_HARDWARE_RM57L843_MEMORYMAP_H */
