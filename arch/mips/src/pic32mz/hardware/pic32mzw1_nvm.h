/****************************************************************************
 * arch/mips/src/pic32mz/hardware/pic32mzw1_nvm.h
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

#ifndef __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZW1_NVM_H
#define __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZW1_NVM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "pic32mz_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define PIC32MZ_NVM_CON_OFFSET      0x0000 /* Programming control */
#define PIC32MZ_NVM_CONCLR_OFFSET   0x0004 /* Programming control clear */
#define PIC32MZ_NVM_CONSET_OFFSET   0x0008 /* Programming control set */
#define PIC32MZ_NVM_CONINV_OFFSET   0x000c /* Programming control invert */
#define PIC32MZ_NVM_KEY_OFFSET      0x0020 /* Unlock register */
#define PIC32MZ_NVM_ADDR_OFFSET     0x0030 /* Flash address (physical) */
#define PIC32MZ_NVM_SRCADDR_OFFSET  0x00c0 /* Row source address (physical) */

/* Register Addresses *******************************************************/

#define PIC32MZ_NVM_CON             (PIC32MZ_NVM_K1BASE + PIC32MZ_NVM_CON_OFFSET)
#define PIC32MZ_NVM_CONCLR          (PIC32MZ_NVM_K1BASE + PIC32MZ_NVM_CONCLR_OFFSET)
#define PIC32MZ_NVM_CONSET          (PIC32MZ_NVM_K1BASE + PIC32MZ_NVM_CONSET_OFFSET)
#define PIC32MZ_NVM_CONINV          (PIC32MZ_NVM_K1BASE + PIC32MZ_NVM_CONINV_OFFSET)
#define PIC32MZ_NVM_KEY             (PIC32MZ_NVM_K1BASE + PIC32MZ_NVM_KEY_OFFSET)
#define PIC32MZ_NVM_ADDR            (PIC32MZ_NVM_K1BASE + PIC32MZ_NVM_ADDR_OFFSET)
#define PIC32MZ_NVM_SRCADDR         (PIC32MZ_NVM_K1BASE + PIC32MZ_NVM_SRCADDR_OFFSET)

/* Programming Control Register Bit Definitions *****************************/

#define NVM_CON_NVMOP_SHIFT         (0)       /* Bits 0-3: NVM operation */
#define NVM_CON_NVMOP_MASK          (15 << NVM_CON_NVMOP_SHIFT)
#  define NVM_CON_NVMOP_NOP         (0 << NVM_CON_NVMOP_SHIFT) /* No operation */
#  define NVM_CON_NVMOP_WORD        (1 << NVM_CON_NVMOP_SHIFT) /* Word program */
#  define NVM_CON_NVMOP_QUADWORD    (2 << NVM_CON_NVMOP_SHIFT) /* Quad word (256 bit) program */
#  define NVM_CON_NVMOP_ROW         (3 << NVM_CON_NVMOP_SHIFT) /* Row program */
#  define NVM_CON_NVMOP_PAGEERASE   (4 << NVM_CON_NVMOP_SHIFT) /* Page erase */

#define NVM_CON_HTDPGM              (1 << 8)  /* Bit 8:  High temperature detected */
#define NVM_CON_LVDERR              (1 << 12) /* Bit 12: Low voltage detected */
#define NVM_CON_WRERR               (1 << 13) /* Bit 13: Write error */
#define NVM_CON_WREN                (1 << 14) /* Bit 14: Write enable */
#define NVM_CON_WR                  (1 << 15) /* Bit 15: Start (HW clears when done) */

/* Unlock sequence written to NVMKEY, after a zero */

#define NVM_KEY1                    0xaa996655
#define NVM_KEY2                    0x556699aa

#endif /* __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZW1_NVM_H */
