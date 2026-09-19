/****************************************************************************
 * arch/arm/src/rm57/hardware/rm57_flash.h
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

/* Register layout taken from TI's HALCoGen HL_reg_flash.h for
 * RM57L843 - matches TMS570's Flash Wrapper address (0xfff87000). Used
 * for flash read-access wait-state configuration during clock/PLL
 * setup (a higher CPU clock requires more flash wait states).
 */

#ifndef __ARCH_ARM_SRC_RM57_HARDWARE_RM57_FLASH_H
#define __ARCH_ARM_SRC_RM57_HARDWARE_RM57_FLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/rm57l843_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

/* Flash Read Control Register */
#define RM57_FLASH_FRDCNTL_OFFSET 0x0000

/* EEPROM/Flash EDAC Control Register 1 */
#define RM57_FLASH_FEDACCTRL1_OFFSET 0x0008

/* Flash Bank Primary Address Tag Register */
#define RM57_FLASH_FPRIM_ADD_TAG_OFFSET 0x0028

/* Flash Bank Access Control Register */
#define RM57_FLASH_FBAC_OFFSET 0x003c

/* Flash Bank Power Mode Register */
#define RM57_FLASH_FBPWRMODE_OFFSET 0x0040

/* Flash Bank Power Mode Status Register */
#define RM57_FLASH_FBPRDY_OFFSET 0x0044

/* Register Addresses *******************************************************/

#define RM57_FLASH_FRDCNTL (RM57_FWRAP_BASE + RM57_FLASH_FRDCNTL_OFFSET)
#define RM57_FLASH_FEDACCTRL1 (RM57_FWRAP_BASE + RM57_FLASH_FEDACCTRL1_OFFSET)
#define RM57_FLASH_FBAC (RM57_FWRAP_BASE + RM57_FLASH_FBAC_OFFSET)
#define RM57_FLASH_FBPWRMODE (RM57_FWRAP_BASE + RM57_FLASH_FBPWRMODE_OFFSET)
#define RM57_FLASH_FBPRDY (RM57_FWRAP_BASE + RM57_FLASH_FBPRDY_OFFSET)

/* Register Bit-Field Definitions *******************************************/

/* Flash Read Control Register */

/* Bits 8-11: Random read wait-states */
#define FLASH_FRDCNTL_RWAIT_SHIFT (8)
#define FLASH_FRDCNTL_RWAIT_MASK       (0xf << FLASH_FRDCNTL_RWAIT_SHIFT)
#  define FLASH_FRDCNTL_RWAIT(n) ((uint32_t)(n) << FLASH_FRDCNTL_RWAIT_SHIFT)

#endif /* __ARCH_ARM_SRC_RM57_HARDWARE_RM57_FLASH_H */
