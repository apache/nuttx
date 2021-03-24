/****************************************************************************
 * arch/arm/src/max326xx/hardware/max32660_flc.h
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

#ifndef __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_FLC_H
#define __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_FLC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/max326_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define MAX326_FLC_ADDR_OFFSET      0x0000  /* Flash Controller Address Pointer Register */
#define MAX326_FLC_CLKDIV_OFFSET    0x0004  /* Flash Controller Clock Divisor Register */
#define MAX326_FLC_CTRL_OFFSET      0x0008  /* Flash Controller Control Register */
#define MAX326_FLC_INTR_OFFSET      0x0024  /* Flash Controller Interrupt Register */
#define MAX326_FLC_DATA0_OFFSET     0x0030  /* Flash Controller Data Register 0 */
#define MAX326_FLC_DATA1_OFFSET     0x0034  /* Flash Controller Data Register 1 */
#define MAX326_FLC_DATA2_OFFSET     0x0038  /* Flash Controller Data Register 2 */
#define MAX326_FLC_DATA3_OFFSET     0x003c  /* Flash Controller Data Register 3 */

/* Register Addresses *******************************************************/

#define MAX326_FLC_ADDR             (MAX326_FLC_BASE + MAX326_FLC_ADDR_OFFSET)
#define MAX326_FLC_CLKDIV           (MAX326_FLC_BASE + MAX326_FLC_CLKDIV_OFFSET)
#define MAX326_FLC_CTRL             (MAX326_FLC_BASE + MAX326_FLC_CTRL_OFFSET)
#define MAX326_FLC_INTR             (MAX326_FLC_BASE + MAX326_FLC_INTR_OFFSET)
#define MAX326_FLC_DATA0            (MAX326_FLC_BASE + MAX326_FLC_DATA0_OFFSET)
#define MAX326_FLC_DATA1            (MAX326_FLC_BASE + MAX326_FLC_DATA1_OFFSET)
#define MAX326_FLC_DATA2            (MAX326_FLC_BASE + MAX326_FLC_DATA2_OFFSET)
#define MAX326_FLC_DATA3            (MAX326_FLC_BASE + MAX326_FLC_DATA3_OFFSET)

/* Register Bit-field Definitions *******************************************/

/* Flash Controller Address Pointer Register (32-bit FLASH address) */

/* Flash Controller Clock Divisor Register */

#define FLC_CLKDIV_MASK            (0xff)  /* Bits 0-7: Flash Controller Clock Divisor */

/* Flash Controller Control Register */

#define FLC_CTRL_WRITE             (1 << 0)  /* Bit 0:  Write */
#define FLC_CTRL_MASSERASE         (1 << 1)  /* Bit 1:  Mass Erase */
#define FLC_CTRL_PAGEERASE         (1 << 2)  /* Bit 2:  Page Erase */
#define FLC_CTRL_WIDTH             (1 << 4)  /* Bit 4:  Data Width Select */
#define FLC_CTRL_ERASECODE_SHIFT   (8)       /* Bits 8-15: Erase Code */
#define FLC_CTRL_ERASECODE_MASK    (0xff << FLC_CTRL_ERASECODE_SHIFT)
#  define FLC_CTRL_ERASECODE_DIS   (0x00 << FLC_CTRL_ERASECODE_SHIFT) /* Erase disabled */
#  define FLC_CTRL_ERASECODE_PAGE  (0x55 << FLC_CTRL_ERASECODE_SHIFT) /* Page erase code */
#  define FLC_CTRL_ERASECODE_MASS  (0xaa << FLC_CTRL_ERASECODE_SHIFT) /* Enable mass erase via
                                                                       * the JTAG debug port */

#define FLC_CTRL_BUSY              (1 << 24) /* Bit 24: Flash Busy Flag */
#define FLC_CTRL_LVE               (1 << 25) /* Bit 25: Low Voltage Enable */
#define FLC_CTRL_UNLOCKCODE_SHIFT  (28)      /* Bits 28-31: Flash Unlock */
#define FLC_CTRL_UNLOCKCODE_MASK   (15 << FLC_CTRL_UNLOCKCODE_SHIFT)
#  define FLC_CTRL_UNLOCKCODE      (2 << FLC_CTRL_UNLOCKCODE_SHIFT)

/* Flash Controller Interrupt Register */

#define FLC_INTR_DONE              (1 << 0)  /* Flash Operation Complete
                                              * Interrupt Flag */
#define FLC_INTR_ACCESS_FAIL       (1 << 1)  /* Flash Access Fail
                                              * Interrupt Flag */
#define FLC_INTR_DONE_IE           (1 << 8)  /* Flash Operation Complete
                                              * Interrupt Enable */
#define FLC_INTR_ACCESS_FAIL_IE    (1 << 9)  /* Flash Access Fail
                                              * Interrupt Enable */

/* Flash Controller Data Register 0 (32-bit FLASH data) */

/* Flash Controller Data Register 1 (32-bit FLASH data) */

/* Flash Controller Data Register 2 (32-bit FLASH data) */

/* Flash Controller Data Register 3 (32-bit FLASH data) */

#endif /* __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_FLC_H */
