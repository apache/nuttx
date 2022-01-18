/****************************************************************************
 * arch/arm/src/efm32/hardware/efm32_romtable.h
 *
 *  Copyright 2014 Silicon Laboratories, Inc. http://www.silabs.com</b>
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.@n
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.@n
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Laboratories, Inc.
 * has no obligation to support this Software. Silicon Laboratories, Inc. is
 * providing the Software "AS IS", with no express or implied warranties of
 * any kind, including, but not limited to, any implied warranties of
 * merchantability or fitness for any particular purpose or warranties
 * against infringement of any proprietary rights of a third party.
 *
 * Silicon Laboratories, Inc. will not be liable for any consequential,
 * incidental, or special damages, or any other relief, or for any claim by
 * any third party, arising from your use of this Software.
 *
 *   Copyright (C) 2014 Pierre-noel Bouteville . All rights reserved.
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Authors: Pierre-noel Bouteville <pnb990@gmail.com>
 *            Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_ARM_SRC_EFM32_HARDWARE_EFM32_ROMTABLE_H
#define __ARCH_ARM_SRC_EFM32_HARDWARE_EFM32_ROMTABLE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/efm32_memorymap.h"

#if !defined(CONFIG_EFM32_EFM32GG)
#  warning This is the EFM32GG header file; Review/modification needed for this architecture
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ROMTABLE ((const struct efm32_romtable_s *)EFM32_ROMTABLE_BASE)

/* Bit fields for struct efm32_romtable_s */

#define _ROMTABLE_PID0_FAMILYLSB_MASK       0x000000C0UL /* Least Significant Bits [1:0] of CHIP FAMILY, mask */
#define _ROMTABLE_PID0_FAMILYLSB_SHIFT      6            /* Least Significant Bits [1:0] of CHIP FAMILY, shift */
#define _ROMTABLE_PID0_REVMAJOR_MASK        0x0000003FUL /* CHIP MAJOR Revision, mask */
#define _ROMTABLE_PID0_REVMAJOR_SHIFT       0            /* CHIP MAJOR Revision, shift */

#define _ROMTABLE_PID1_FAMILYMSB_MASK       0x0000000FUL /* Most Significant Bits [5:2] of CHIP FAMILY, mask */
#define _ROMTABLE_PID1_FAMILYMSB_SHIFT      0            /* Most Significant Bits [5:2] of CHIP FAMILY, shift */

#define _ROMTABLE_PID2_REVMINORMSB_MASK     0x000000F0UL /* Most Significant Bits [7:4] of CHIP MINOR revision, mask */
#define _ROMTABLE_PID2_REVMINORMSB_SHIFT    4            /* Most Significant Bits [7:4] of CHIP MINOR revision, mask */

#define _ROMTABLE_PID3_REVMINORLSB_MASK     0x000000F0UL /* Least Significant Bits [3:0] of CHIP MINOR revision, mask */
#define _ROMTABLE_PID3_REVMINORLSB_SHIFT    4            /* Least Significant Bits [3:0] of CHIP MINOR revision, shift */

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct efm32_romtable_s
{
  const uint32_t pid4; /* JEP_106_BANK */
  const uint32_t pid5; /* Unused */
  const uint32_t pid6; /* Unused */
  const uint32_t pid7; /* Unused */
  const uint32_t pid0; /* Chip family LSB, chip major revision */
  const uint32_t pid1; /* JEP_106_NO, Chip family MSB */
  const uint32_t pid2; /* Chip minor rev MSB, JEP_106_PRESENT, JEP_106_NO */
  const uint32_t pid3; /* Chip minor rev LSB */
  const uint32_t cid0; /* Unused */
};

#endif /* __ARCH_ARM_SRC_EFM32_HARDWARE_EFM32_ROMTABLE_H */
