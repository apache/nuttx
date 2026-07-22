/****************************************************************************
 * arch/arm/include/rm57/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_RM57_CHIP_H
#define __ARCH_ARM_INCLUDE_RM57_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Values below are taken from TI's HALCoGen project for RM57L843
 * (HL_sys_link.ld for flash/RAM size, RM57L_Contents.dil VIM channel
 * table for peripheral counts - see rm57l843_irq.h).
 */

#if defined(CONFIG_ARCH_CHIP_RM57L843)
#  undef  RM57_CORTEX_R4                  /* Not Cortex-R4 family */
#  undef  RM57_CORTEX_R4F                 /* Not Cortex-R4F family */
#  undef  RM57_CORTEX_R5                  /* Not Cortex-R5 family */
#  define RM57_CORTEX_R5F    1            /* Cortex-R5F family */

/* 4MB Program FLASH (HL_sys_link.ld) */
#  define RM57_PFLASH (4096*1024)
#  define RM57_SRAM          (512*1024)   /* 512KB SRAM (HL_sys_link.ld) */

/* DCAN1-4 (VIM req 16/29, 35/42, 45/55, 113/117) */
#  define RM57_NCAN 4

/* SCI1(LIN1)/SCI2/SCI3/SCI4 (HL_reg_sci.h) */
#  define RM57_NSCI 4

/* MibSPI1-5 (VIM req 12/26, 30, 37/38, 49/54, 53/56) */
#  define RM57_NMIBSPI 5

/* GIO Port A, Port B (HL_reg_gio.h) */
#  define RM57_NGIOPORT 2
#else
#  error Unrecognized RM57 chip
#endif

#endif /* __ARCH_ARM_INCLUDE_RM57_CHIP_H */
